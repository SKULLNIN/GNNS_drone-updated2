"""
gNNS Drone — VIO Tracker
=========================
Visual Inertial Odometry position tracking using Intel RealSense.

Supports:
  - RealSense T265 (direct pose output)
  - RealSense D435i / D455 (depth + IMU → VIOAlgorithm pipeline)
  - Simulated mode (for testing without hardware)

Sends position to FC via MAVLink VISION_POSITION_ESTIMATE.

D435i / D455 pipeline (NEW):
  RealSense streams (color @ 30 Hz, depth @ 30 Hz, accel/gyro @ 200 Hz)
    → IMU samples buffered in deque
    → VIOAlgorithm.process_frame(color, depth, imu_samples)
    → VIOState (full rich output)
    → mapped back to VIOPose for backward compatibility
    → get_full_state() returns the complete VIOState
"""

import time
import math
import threading
import logging
from collections import deque
from enum import IntEnum
from dataclasses import dataclass, field
from typing import Optional

logger = logging.getLogger("gnns.vio")


class VIOStatus(IntEnum):
    """VIO tracker status (mirrors GNNS_VIO_STATUS enum in MAVLink dialect)."""
    UNINITIALIZED = 0
    INITIALIZING = 1
    TRACKING = 2
    DEGRADED = 3
    LOST = 4
    RELOCALIZING = 5


@dataclass
class VIOPose:
    """A single VIO pose measurement."""
    timestamp: float = 0.0
    x: float = 0.0      # North (meters)
    y: float = 0.0      # East (meters)
    z: float = 0.0      # Down (meters)
    vx: float = 0.0     # North velocity (m/s)
    vy: float = 0.0     # East velocity (m/s)
    vz: float = 0.0     # Down velocity (m/s)
    roll: float = 0.0   # radians
    pitch: float = 0.0  # radians
    yaw: float = 0.0    # radians
    confidence: int = 0  # 0-100
    features: int = 0    # Feature count
    covariance: float = 0.0  # Position uncertainty (meters)


@dataclass
class VIOStats:
    """VIO tracker statistics for debugging."""
    status: VIOStatus = VIOStatus.UNINITIALIZED
    frames_processed: int = 0
    frames_dropped: int = 0
    current_fps: float = 0.0
    avg_confidence: float = 0.0
    max_velocity_seen: float = 0.0
    total_distance: float = 0.0
    tracking_lost_count: int = 0
    last_pose: VIOPose = field(default_factory=VIOPose)


class VIOTracker:
    """
    Visual Inertial Odometry tracker.
    
    Reads pose from RealSense cameras and provides NED-frame
    position/velocity to be sent to the flight controller.
    
    Usage:
        tracker = VIOTracker(camera_type="t265")
        tracker.start()
        
        # In your control loop:
        pose = tracker.get_pose()
        bridge.send_vision_position(pose.x, pose.y, pose.z,
                                     pose.roll, pose.pitch, pose.yaw)
        
        # Check health:
        if tracker.status == VIOStatus.LOST:
            emergency_land()
            
        tracker.stop()
    """

    def __init__(self, camera_type: str = "t265", config: Optional[dict] = None):
        self.camera_type = camera_type.lower()
        self.config = config or {}
        
        self._stats = VIOStats()
        self._pose = VIOPose()
        self._pose_lock = threading.Lock()
        self._running = False
        self._thread = None
        
        # Callbacks for status changes
        self._on_status_change = []
        self._on_tracking_lost = []
        
        # Frame transform config
        tf = self.config.get("transform", {})
        self._x_axis = tf.get("x_axis", "-z")
        self._y_axis = tf.get("y_axis", "x")
        self._z_axis = tf.get("z_axis", "y")
        
        # Thresholds
        thresh = self.config.get("thresholds", {})
        self._min_confidence = thresh.get("min_confidence", 30)
        self._critical_confidence = thresh.get("critical_confidence", 10)
        self._min_features = thresh.get("min_features", 50)

    @property
    def status(self) -> VIOStatus:
        return self._stats.status

    @property
    def stats(self) -> VIOStats:
        return self._stats

    def get_pose(self) -> VIOPose:
        """Get the latest VIO pose (thread-safe)."""
        with self._pose_lock:
            return VIOPose(
                timestamp=self._pose.timestamp,
                x=self._pose.x, y=self._pose.y, z=self._pose.z,
                vx=self._pose.vx, vy=self._pose.vy, vz=self._pose.vz,
                roll=self._pose.roll, pitch=self._pose.pitch, yaw=self._pose.yaw,
                confidence=self._pose.confidence,
                features=self._pose.features,
                covariance=self._pose.covariance,
            )

    def on_status_change(self, callback):
        """Register callback for VIO status changes: callback(old_status, new_status)."""
        self._on_status_change.append(callback)

    def on_tracking_lost(self, callback):
        """Register callback for tracking lost event: callback()."""
        self._on_tracking_lost.append(callback)

    def start(self):
        """Start the VIO tracking thread."""
        if self._running:
            logger.warning("VIO tracker already running!")
            return

        logger.info(f"Starting VIO tracker (camera={self.camera_type})...")
        self._running = True

        if self.camera_type == "t265":
            self._thread = threading.Thread(target=self._run_t265,
                                             daemon=True, name="vio-t265")
        elif self.camera_type == "d435i":
            self._thread = threading.Thread(target=self._run_d435i,
                                             daemon=True, name="vio-d435i")
        elif self.camera_type == "simulated":
            self._thread = threading.Thread(target=self._run_simulated,
                                             daemon=True, name="vio-sim")
        else:
            raise ValueError(f"Unknown camera type: {self.camera_type}")

        self._thread.start()

    def stop(self):
        """Stop the VIO tracking thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        logger.info("VIO tracker stopped")

    def get_full_state(self):
        """
        Return the latest rich VIOState from the D435i/D455 pipeline.

        Returns None when running in T265 or simulated mode (those modes
        do not produce a VIOState; use get_pose() instead).
        """
        if hasattr(self, "_vio_state"):
            return self._vio_state
        return None

    def _set_status(self, new_status: VIOStatus):
        """Update status and fire callbacks."""
        old = self._stats.status
        if old != new_status:
            self._stats.status = new_status
            logger.info(f"VIO status: {old.name} -> {new_status.name}")
            for cb in self._on_status_change:
                try:
                    cb(old, new_status)
                except Exception as e:
                    logger.error(f"Status callback error: {e}")
            if new_status == VIOStatus.LOST:
                self._stats.tracking_lost_count += 1
                for cb in self._on_tracking_lost:
                    try:
                        cb()
                    except Exception as e:
                        logger.error(f"Tracking lost callback error: {e}")

    # ==============================================================
    # T265 TRACKING CAMERA
    # ==============================================================

    def _run_t265(self):
        """Main loop for Intel RealSense T265."""
        self._set_status(VIOStatus.INITIALIZING)

        try:
            import pyrealsense2 as rs
        except ImportError:
            logger.error(
                "pyrealsense2 not installed! Install with:\n"
                "  pip install pyrealsense2\n"
                "Falling back to simulated mode."
            )
            self._run_simulated()
            return

        try:
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.pose)
            
            profile = pipe.start(cfg)
            device = profile.get_device()
            logger.info(f"T265 connected: {device.get_info(rs.camera_info.name)}")
            logger.info(f"  Serial: {device.get_info(rs.camera_info.serial_number)}")
            logger.info(f"  FW: {device.get_info(rs.camera_info.firmware_version)}")

        except Exception as e:
            logger.error(f"T265 init failed: {e}")
            self._set_status(VIOStatus.LOST)
            return

        self._set_status(VIOStatus.TRACKING)
        fps_counter = 0
        fps_timer = time.time()
        prev_pos = (0, 0, 0)

        while self._running:
            try:
                frames = pipe.wait_for_frames(timeout_ms=500)
                pose_frame = frames.get_pose_frame()
                
                if not pose_frame:
                    self._stats.frames_dropped += 1
                    continue

                data = pose_frame.get_pose_data()
                self._stats.frames_processed += 1
                fps_counter += 1

                # Transform T265 frame to NED
                # T265: +x right, +y down, +z backward (from camera POV)
                # When camera faces FORWARD on drone:
                #   NED North = -T265.z
                #   NED East  = T265.x
                #   NED Down  = T265.y
                north = -data.translation.z
                east = data.translation.x
                down = data.translation.y

                # Velocities
                vn = -data.velocity.z
                ve = data.velocity.x
                vd = data.velocity.y

                # Yaw from quaternion
                q = data.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
                pitch = math.asin(max(-1, min(1,
                    2.0 * (q.w * q.y - q.z * q.x)
                )))
                roll = math.atan2(
                    2.0 * (q.w * q.x + q.y * q.z),
                    1.0 - 2.0 * (q.x * q.x + q.y * q.y)
                )

                # T265 tracking confidence: 0=Failed, 1=Low, 2=Medium, 3=High
                confidence_map = {0: 0, 1: 25, 2: 60, 3: 95}
                confidence = confidence_map.get(data.tracker_confidence, 0)

                # Update pose
                with self._pose_lock:
                    self._pose.timestamp = time.time()
                    self._pose.x = north
                    self._pose.y = east
                    self._pose.z = down
                    self._pose.vx = vn
                    self._pose.vy = ve
                    self._pose.vz = vd
                    self._pose.roll = roll
                    self._pose.pitch = pitch
                    self._pose.yaw = yaw
                    self._pose.confidence = confidence
                    self._pose.covariance = 0.01 * (100 - confidence)

                self._stats.last_pose = self.get_pose()

                # Track distance
                dist = math.sqrt(
                    (north - prev_pos[0])**2 +
                    (east - prev_pos[1])**2 +
                    (down - prev_pos[2])**2
                )
                self._stats.total_distance += dist
                prev_pos = (north, east, down)

                # Track velocity
                speed = math.sqrt(vn**2 + ve**2 + vd**2)
                self._stats.max_velocity_seen = max(
                    self._stats.max_velocity_seen, speed
                )

                # Update status based on confidence
                if confidence >= self._min_confidence:
                    self._set_status(VIOStatus.TRACKING)
                elif confidence >= self._critical_confidence:
                    self._set_status(VIOStatus.DEGRADED)
                else:
                    self._set_status(VIOStatus.LOST)

                # FPS calculation
                if time.time() - fps_timer >= 1.0:
                    self._stats.current_fps = fps_counter
                    fps_counter = 0
                    fps_timer = time.time()

            except Exception as e:
                logger.error(f"T265 frame error: {e}")
                self._stats.frames_dropped += 1
                time.sleep(0.01)

        # Cleanup
        try:
            pipe.stop()
        except Exception:
            pass
        logger.info("T265 pipeline stopped")

    # ==============================================================
    # D435i / D455  →  VIOAlgorithm pipeline
    # ==============================================================

    def _run_d435i(self):
        """
        D435i / D455 VIO loop using the custom VIOAlgorithm pipeline.

        Pipeline:
          1. Configure RealSense color (30 Hz), depth (30 Hz), accel+gyro (200 Hz).
          2. IMU samples are buffered in a deque; drained per camera frame.
          3. VIOAlgorithm.process_frame() returns a full VIOState each frame.
          4. VIOState is mapped to VIOPose for backward compat.
          5. get_full_state() exposes the complete VIOState.
        """
        self._set_status(VIOStatus.INITIALIZING)
        self._vio_state = None  # Will hold the latest VIOState

        # ---- Import dependencies ----------------------------------------
        try:
            import pyrealsense2 as rs
        except ImportError:
            logger.error(
                "pyrealsense2 not installed! pip install pyrealsense2\n"
                "Falling back to simulated mode."
            )
            self._run_simulated()
            return

        try:
            from .vio_algorithm import VIOAlgorithm
        except ImportError as e:
            logger.error(f"VIOAlgorithm import failed: {e}\n"
                         "Falling back to simulated mode.")
            self._run_simulated()
            return

        # ---- Initialise VIOAlgorithm ----------------------------------------
        detector_method = self.config.get("detector_method", "fast9")
        algo_config     = self.config.get("vio_algorithm",   {})
        algo = VIOAlgorithm(detector_method=detector_method, config=algo_config)
        algo.start()

        # IMU sample deque (thread-safe via GIL for simple appends)
        imu_deque: deque = deque(maxlen=500)

        # ---- Configure RealSense pipeline -----------------------------------
        try:
            pipe = rs.pipeline()
            cfg  = rs.config()

            # Color stream (used for feature detection)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            # Depth stream aligned to color
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
            # IMU streams
            cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
            cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, 200)

            profile = pipe.start(cfg)
            device  = profile.get_device()
            logger.info(f"D455/D435i connected: {device.get_info(rs.camera_info.name)}")
            logger.info(f"  Serial: {device.get_info(rs.camera_info.serial_number)}")

            # Read depth scale
            depth_sensor  = device.first_depth_sensor()
            depth_scale   = depth_sensor.get_depth_scale()

            # Read color intrinsics for the algorithm
            color_stream  = profile.get_stream(rs.stream.color)
            color_intr    = color_stream.as_video_stream_profile().get_intrinsics()
            algo.update_camera_intrinsics(
                color_intr.fx, color_intr.fy,
                color_intr.ppx, color_intr.ppy,
                depth_scale,
            )

            # Align depth to color
            align = rs.align(rs.stream.color)

        except Exception as e:
            logger.error(f"D455/D435i init failed: {e}")
            self._set_status(VIOStatus.LOST)
            algo.stop()
            return

        self._set_status(VIOStatus.INITIALIZING)
        fps_counter = 0
        fps_timer   = time.time()
        prev_pos    = (0.0, 0.0, 0.0)

        # ---- Main frame loop ------------------------------------------------
        while self._running:
            try:
                frames = pipe.wait_for_frames(timeout_ms=1000)
            except Exception as e:
                logger.warning(f"D455 wait_for_frames timeout: {e}")
                self._stats.frames_dropped += 1
                continue

            try:
                # ---- Collect IMU frames (motion frames) --------------------
                for frame in frames:
                    if frame.is_motion_frame():
                        mf   = frame.as_motion_frame()
                        data = mf.get_motion_data()
                        t    = mf.get_timestamp() / 1000.0  # ms → s

                        if mf.get_profile().stream_type() == rs.stream.accel:
                            # Store temporarily; merge with next gyro
                            imu_deque.append(
                                (data.x, data.y, data.z, None, None, None, t)
                            )
                        elif mf.get_profile().stream_type() == rs.stream.gyro:
                            # Find the most recent accel entry and merge
                            merged = False
                            for i in range(len(imu_deque) - 1, -1, -1):
                                entry = imu_deque[i]
                                if entry[3] is None:  # accel entry
                                    imu_deque[i] = (
                                        entry[0], entry[1], entry[2],
                                        data.x, data.y, data.z, t
                                    )
                                    merged = True
                                    break
                            if not merged:
                                # No pending accel; push zeros for accel
                                imu_deque.append(
                                    (0.0, 0.0, 0.0, data.x, data.y, data.z, t)
                                )

                # ---- Get aligned color+depth frame -------------------------
                aligned = align.process(frames)
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()

                if not color_frame:
                    self._stats.frames_dropped += 1
                    continue

                # Convert to numpy
                import numpy as np
                color_img = np.asanyarray(color_frame.get_data())
                depth_img = np.asanyarray(depth_frame.get_data()) if depth_frame else None

                # Drain IMU deque: only use complete (accel+gyro) samples
                imu_samples = []
                while imu_deque:
                    entry = imu_deque.popleft()
                    if entry[3] is not None:   # has gyro data
                        imu_samples.append(entry)

                # ---- Run VIO pipeline -------------------------------------
                vio_state = algo.process_frame(color_img, depth_img, imu_samples)
                self._vio_state = vio_state

                # ---- Map VIOState → VIOPose (backward compat) ------------
                with self._pose_lock:
                    self._pose.timestamp  = vio_state.timestamp
                    self._pose.x          = vio_state.north
                    self._pose.y          = vio_state.east
                    self._pose.z          = vio_state.down
                    self._pose.vx         = vio_state.vn
                    self._pose.vy         = vio_state.ve
                    self._pose.vz         = vio_state.vd
                    self._pose.roll       = vio_state.roll
                    self._pose.pitch      = vio_state.pitch
                    self._pose.yaw        = vio_state.yaw
                    self._pose.confidence = int(vio_state.confidence)
                    self._pose.features   = vio_state.num_features_tracked
                    self._pose.covariance = vio_state.pos_cov_scalar

                self._stats.frames_processed += 1
                fps_counter += 1

                # ---- Update stats ----------------------------------------
                north, east, down = vio_state.north, vio_state.east, vio_state.down
                dist = math.sqrt(
                    (north - prev_pos[0]) ** 2 +
                    (east  - prev_pos[1]) ** 2 +
                    (down  - prev_pos[2]) ** 2
                )
                self._stats.total_distance += dist
                prev_pos = (north, east, down)

                speed = vio_state.speed_3d
                self._stats.max_velocity_seen = max(
                    self._stats.max_velocity_seen, speed
                )
                self._stats.last_pose = self.get_pose()

                # ---- Update VIO status -----------------------------------
                self._set_status(vio_state.state)

                # ---- FPS ------------------------------------------------
                if time.time() - fps_timer >= 1.0:
                    self._stats.current_fps = fps_counter
                    fps_counter = 0
                    fps_timer   = time.time()

            except Exception as e:
                logger.error(f"D455 frame processing error: {e}")
                self._stats.frames_dropped += 1
                time.sleep(0.01)

        # ---- Cleanup -------------------------------------------------------
        try:
            pipe.stop()
        except Exception:
            pass
        algo.stop()
        logger.info("D455/D435i VIO pipeline stopped")

    # ==============================================================
    # SIMULATED MODE (for testing)
    # ==============================================================

    def _run_simulated(self):
        """Simulated VIO for testing without hardware."""
        self._set_status(VIOStatus.INITIALIZING)
        logger.info("Running in SIMULATED VIO mode")
        time.sleep(1.0)
        self._set_status(VIOStatus.TRACKING)

        sim_x, sim_y, sim_z = 0.0, 0.0, 0.0
        fps_counter = 0
        fps_timer = time.time()

        while self._running:
            # In simulation, just report zero position
            # When connected to SITL, actual position comes from FC
            with self._pose_lock:
                self._pose.timestamp = time.time()
                self._pose.x = sim_x
                self._pose.y = sim_y
                self._pose.z = sim_z
                self._pose.confidence = 90
                self._pose.features = 200

            self._stats.frames_processed += 1
            self._stats.last_pose = self.get_pose()
            fps_counter += 1

            if time.time() - fps_timer >= 1.0:
                self._stats.current_fps = fps_counter
                fps_counter = 0
                fps_timer = time.time()

            time.sleep(1.0 / 30)  # 30 Hz

        logger.info("Simulated VIO stopped")

    # ==============================================================
    # DEBUG OUTPUT
    # ==============================================================

    def print_status(self):
        """Print current VIO status for debugging."""
        s = self._stats
        p = self.get_pose()
        print(f"\n--- VIO Status ---")
        print(f"  Status:      {s.status.name}")
        print(f"  FPS:         {s.current_fps:.1f}")
        print(f"  Frames:      {s.frames_processed} processed, "
              f"{s.frames_dropped} dropped")
        print(f"  Position:    N={p.x:.3f} E={p.y:.3f} D={p.z:.3f}")
        print(f"  Velocity:    N={p.vx:.2f} E={p.vy:.2f} D={p.vz:.2f}")
        print(f"  Orientation: R={math.degrees(p.roll):.1f}° "
              f"P={math.degrees(p.pitch):.1f}° "
              f"Y={math.degrees(p.yaw):.1f}°")
        print(f"  Confidence:  {p.confidence}%")
        print(f"  Covariance:  {p.covariance:.4f}m")
        print(f"  Distance:    {s.total_distance:.2f}m total")
        print(f"  Max speed:   {s.max_velocity_seen:.2f}m/s")
        print(f"  Lost count:  {s.tracking_lost_count}")
