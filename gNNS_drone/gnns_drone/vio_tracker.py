"""
gNNS Drone — VIO Tracker
=========================
Visual Inertial Odometry position tracking using Intel RealSense.

Supports:
  - RealSense T265 (direct pose output)
  - RealSense D435i (depth + IMU, requires external VIO pipeline)
  - Simulated mode (for testing without hardware)

Sends position to FC via MAVLink VISION_POSITION_ESTIMATE.
"""

import time
import math
import threading
import logging
from enum import IntEnum
from dataclasses import dataclass, field

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

    def __init__(self, camera_type: str = "t265", config: dict = None):
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
    # D435i (placeholder — needs ORB-SLAM3 or RTAB-Map)
    # ==============================================================

    def _run_d435i(self):
        """
        D435i VIO loop.
        
        NOTE: The D435i doesn't provide direct pose output like T265.
        You need an external VIO pipeline like:
          - ORB-SLAM3
          - RTAB-Map
          - VINS-Mono
          - Kimera-VIO
          
        For now, this falls back to simulated mode.
        TODO: Integrate with your chosen VIO pipeline.
        """
        logger.warning(
            "D435i VIO not yet implemented! Needs ORB-SLAM3 or RTAB-Map.\n"
            "Falling back to simulated mode for now.\n"
            "TODO: Integrate ROS2 RTAB-Map node → subscribe to /odom topic"
        )
        self._run_simulated()

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
