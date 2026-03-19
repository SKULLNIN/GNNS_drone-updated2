"""
gNNS Drone — RTAB-Map Odometry Integration
============================================
Uses RTAB-Map SLAM for high-quality visual odometry.

Why RTAB-Map instead of raw T265?
  - T265 drifts over long distances (no loop closure)
  - RTAB-Map does loop closure → corrects drift
  - RTAB-Map fuses depth (D435i) + IMU → more robust
  - Gives us a 3D map for obstacle avoidance too

Pipeline:
  D435i (depth + color + IMU) → RTAB-Map → Odometry (x,y,z,vx,vy,vz)
                                         → 3D Map
                                         → Loop Closures

This module can run RTAB-Map in two ways:
  1. ROS2 mode: subscribes to /odom from rtabmap_ros
  2. Standalone mode: uses rtabmap Python bindings directly
"""

import time
import math
import threading
import logging
import numpy as np
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Callable, List

import yaml

logger = logging.getLogger("gnns.rtabmap")


def _load_vio_config() -> dict:
    """Load vio_config.yaml and return ros2_odom section (or empty dict)."""
    cfg_path = Path(__file__).parent.parent / "config" / "vio_config.yaml"
    if not cfg_path.exists():
        return {}
    try:
        with open(cfg_path, "r") as f:
            data = yaml.safe_load(f)
        return data.get("ros2_odom", {}) if data else {}
    except Exception as e:
        logger.warning(f"Could not load vio_config: {e}")
        return {}


@dataclass
class OdomData:
    """
    Odometry data from RTAB-Map.
    All values in NED frame (North-East-Down), meters and m/s.
    """
    timestamp: float = 0.0

    # Position (meters from start point)
    x: float = 0.0       # North
    y: float = 0.0       # East
    z: float = 0.0       # Down (negative = above start)

    # Velocity (m/s) — derived from position or from IMU integration
    vx: float = 0.0      # North velocity
    vy: float = 0.0      # East velocity
    vz: float = 0.0      # Down velocity

    # Acceleration (m/s²) — derived from velocity
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0

    # Orientation (radians)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Angular rates (rad/s)
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0

    # Quality metrics
    confidence: int = 0        # 0-100
    num_features: int = 0      # Visual features tracked
    num_inliers: int = 0       # RANSAC inliers (motion estimation quality)
    loop_closure_id: int = -1  # >0 if loop closure just happened
    covariance_pos: float = 0.0  # Position uncertainty (meters, 1-sigma)
    covariance_rot: float = 0.0  # Rotation uncertainty (radians, 1-sigma)

    @property
    def speed_horizontal(self) -> float:
        """Horizontal ground speed in m/s."""
        return math.sqrt(self.vx ** 2 + self.vy ** 2)

    @property
    def speed_3d(self) -> float:
        """3D speed in m/s."""
        return math.sqrt(self.vx ** 2 + self.vy ** 2 + self.vz ** 2)

    @property
    def altitude(self) -> float:
        """Altitude above start point (positive up)."""
        return -self.z

    @property 
    def distance_from_home(self) -> float:
        """Horizontal distance from start point."""
        return math.sqrt(self.x ** 2 + self.y ** 2)


class VelocityEstimator:
    """
    Derives velocity and acceleration from position updates.
    Uses a simple low-pass filter for smooth derivatives.
    
    Why not just use raw differentiation?
    - Position updates can be noisy
    - Simple dp/dt amplifies noise
    - Low-pass filtered derivatives are much smoother
    - This gives us clean velocity for the precision landing controller
    """

    def __init__(self, alpha: float = 0.3):
        """
        Args:
            alpha: Low-pass filter coefficient (0-1).
                   Lower = smoother but slower response.
                   Higher = faster but noisier.
                   0.3 is a good starting point.
        """
        self.alpha = alpha
        self._prev_pos = None
        self._prev_vel = np.zeros(3)
        self._prev_acc = np.zeros(3)
        self._prev_time = 0.0

    def update(self, x: float, y: float, z: float,
               timestamp: float) -> tuple:
        """
        Update with new position and compute velocity + acceleration.
        
        Returns:
            (vx, vy, vz, ax, ay, az) — filtered velocity and acceleration
        """
        pos = np.array([x, y, z])

        if self._prev_pos is None:
            self._prev_pos = pos
            self._prev_time = timestamp
            return (0, 0, 0, 0, 0, 0)

        dt = timestamp - self._prev_time
        if dt <= 0 or dt > 1.0:  # Invalid or too long gap
            self._prev_pos = pos
            self._prev_time = timestamp
            return tuple(self._prev_vel) + tuple(self._prev_acc)

        # Raw velocity = dp/dt
        raw_vel = (pos - self._prev_pos) / dt

        # Low-pass filter: filtered = alpha * raw + (1-alpha) * prev
        vel = self.alpha * raw_vel + (1 - self.alpha) * self._prev_vel

        # Acceleration = dv/dt (also filtered)
        raw_acc = (vel - self._prev_vel) / dt
        acc = self.alpha * raw_acc + (1 - self.alpha) * self._prev_acc

        self._prev_pos = pos
        self._prev_vel = vel
        self._prev_acc = acc
        self._prev_time = timestamp

        return (vel[0], vel[1], vel[2], acc[0], acc[1], acc[2])


class RTABMapOdom:
    """
    RTAB-Map odometry provider.
    
    Runs RTAB-Map visual SLAM and provides smooth odometry
    with derived velocity and acceleration.
    
    Usage:
        odom = RTABMapOdom(mode="ros2")  # or "standalone" or "simulated"
        odom.start()
        
        data = odom.get()  # OdomData with pos, vel, acc, orientation
        
        # Send to FC
        bridge.send_vision_position(data.x, data.y, data.z,
                                     data.roll, data.pitch, data.yaw)
        bridge.send_vision_speed(data.vx, data.vy, data.vz)
    """

    def __init__(self, mode: str = "ros2", config: Optional[dict] = None):
        """
        Args:
            mode: "ros2" — subscribe to RTAB-Map ROS2 node's /odom topic
                  "standalone" — use RTAB-Map Python bindings (no ROS)
                  "t265_raw" — use T265 raw tracking (fallback)
                  "simulated" — simulated odometry for testing
        """
        self.mode = mode
        self.config = config or {}
        
        self._odom = OdomData()
        self._odom_lock = threading.Lock()
        self._vel_estimator = VelocityEstimator(alpha=0.3)
        self._running = False
        self._thread = None
        
        # Callbacks
        self._on_loop_closure: List[Callable] = []
        self._on_quality_drop: List[Callable] = []
        
        # Stats
        self.total_distance = 0.0
        self.loop_closures = 0
        self._prev_pos = (0, 0, 0)

    def get(self) -> OdomData:
        """Get latest odometry (thread-safe)."""
        with self._odom_lock:
            return OdomData(
                timestamp=self._odom.timestamp,
                x=self._odom.x, y=self._odom.y, z=self._odom.z,
                vx=self._odom.vx, vy=self._odom.vy, vz=self._odom.vz,
                ax=self._odom.ax, ay=self._odom.ay, az=self._odom.az,
                roll=self._odom.roll, pitch=self._odom.pitch, yaw=self._odom.yaw,
                roll_rate=self._odom.roll_rate, pitch_rate=self._odom.pitch_rate,
                yaw_rate=self._odom.yaw_rate,
                confidence=self._odom.confidence,
                num_features=self._odom.num_features,
                num_inliers=self._odom.num_inliers,
                loop_closure_id=self._odom.loop_closure_id,
                covariance_pos=self._odom.covariance_pos,
                covariance_rot=self._odom.covariance_rot,
            )

    def on_loop_closure(self, callback: Callable):
        """Register callback when RTAB-Map detects a loop closure."""
        self._on_loop_closure.append(callback)

    def on_quality_drop(self, callback: Callable):
        """Register callback when odometry quality drops below threshold."""
        self._on_quality_drop.append(callback)

    def start(self):
        self._running = True
        if self.mode == "ros2":
            self._thread = threading.Thread(target=self._run_ros2, daemon=True)
        elif self.mode == "t265_raw":
            self._thread = threading.Thread(target=self._run_t265, daemon=True)
        elif self.mode == "simulated":
            self._thread = threading.Thread(target=self._run_simulated, daemon=True)
        elif self.mode == "sitl":
            self._thread = threading.Thread(target=self._run_sitl, daemon=True)
        else:
            raise ValueError(f"Unknown mode: {self.mode}")
        self._thread.start()
        logger.info(f"RTAB-Map odometry started (mode={self.mode})")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        logger.info("Odometry stopped")

    def _update_odom(self, x, y, z, roll, pitch, yaw, 
                     timestamp, confidence=90, features=0, inliers=0,
                     loop_closure_id=-1):
        """Internal: update odometry with new pose and compute derivatives."""
        # Compute velocity and acceleration
        vx, vy, vz, ax, ay, az = self._vel_estimator.update(x, y, z, timestamp)

        # Track total distance
        dist = math.sqrt(
            (x - self._prev_pos[0])**2 +
            (y - self._prev_pos[1])**2 +
            (z - self._prev_pos[2])**2
        )
        self.total_distance += dist
        self._prev_pos = (x, y, z)

        # Update odometry
        with self._odom_lock:
            self._odom.timestamp = timestamp
            self._odom.x = x
            self._odom.y = y
            self._odom.z = z
            self._odom.vx = vx
            self._odom.vy = vy
            self._odom.vz = vz
            self._odom.ax = ax
            self._odom.ay = ay
            self._odom.az = az
            self._odom.roll = roll
            self._odom.pitch = pitch
            self._odom.yaw = yaw
            self._odom.confidence = confidence
            self._odom.num_features = features
            self._odom.num_inliers = inliers
            self._odom.loop_closure_id = loop_closure_id
            self._odom.covariance_pos = 0.01 * (100 - confidence)

        # Fire callbacks
        if loop_closure_id > 0:
            self.loop_closures += 1
            logger.info(f"Loop closure detected! ID={loop_closure_id}")
            for cb in self._on_loop_closure:
                cb(loop_closure_id)

        if confidence < 30:
            for cb in self._on_quality_drop:
                cb(confidence)

    # ==============================================================
    # ROS2 MODE — Subscribe to RTAB-Map /odom topic
    # ==============================================================

    def _run_ros2(self):
        """
        Subscribe to RTAB-Map's /odom topic via ROS2.
        
        Requires:
          - ROS2 Humble/Foxy installed
          - rtabmap_ros package running
          - D435i publishing to /camera/depth and /camera/color topics
          
        Launch RTAB-Map with:
          ros2 launch rtabmap_ros rtabmap.launch.py \\
            rgb_topic:=/camera/color/image_raw \\
            depth_topic:=/camera/depth/image_raw \\
            camera_info_topic:=/camera/color/camera_info \\
            frame_id:=camera_link \\
            approx_sync:=true \\
            odom_frame_id:=odom \\
            visual_odometry:=true
        """
        try:
            import rclpy
            from rclpy.node import Node
            from nav_msgs.msg import Odometry
            from rtabmap_msgs.msg import Info
        except ImportError:
            logger.error(
                "ROS2 not installed! Install ROS2 Humble and rtabmap_ros.\n"
                "Falling back to T265 raw mode."
            )
            self._run_t265()
            return

        rclpy.init()
        node = rclpy.create_node('gnns_odom_subscriber')

        def odom_callback(msg: Odometry):
            """Process odometry from RTAB-Map."""
            # Extract position (ROS2 uses ENU, convert to NED)
            # ROS ENU: x=East, y=North, z=Up
            # Our NED: x=North, y=East, z=Down
            pos = msg.pose.pose.position
            north = pos.y   # ROS Y (North) → NED X (North)
            east = pos.x    # ROS X (East) → NED Y (East)
            down = -pos.z   # ROS Z (Up) → NED Z (Down, so negate)

            # Extract orientation quaternion → euler
            q = msg.pose.pose.orientation
            # Convert quaternion to euler (roll, pitch, yaw)
            sinr = 2.0 * (q.w * q.x + q.y * q.z)
            cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            roll = math.atan2(sinr, cosr)

            sinp = 2.0 * (q.w * q.y - q.z * q.x)
            sinp = max(-1.0, min(1.0, sinp))
            pitch = math.asin(sinp)

            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)

            # Covariance → confidence approximation
            pos_cov = msg.pose.covariance[0]  # xx variance
            confidence = max(0, min(100, int(100 - pos_cov * 100)))

            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            self._update_odom(
                north, east, down, roll, pitch, yaw,
                timestamp, confidence=confidence
            )

        def info_callback(msg: Info):
            """Process RTAB-Map info (loop closures, features)."""
            if msg.loop_closure_id > 0:
                with self._odom_lock:
                    self._odom.loop_closure_id = msg.loop_closure_id
                    self._odom.num_features = msg.ref_words
                    self._odom.num_inliers = msg.loop_closure_transform_accepted

        node.create_subscription(Odometry, '/odom', odom_callback, 10)
        node.create_subscription(Info, '/rtabmap/info', info_callback, 10)

        logger.info("Subscribed to /odom and /rtabmap/info")

        while self._running:
            rclpy.spin_once(node, timeout_sec=0.1)

        node.destroy_node()
        rclpy.shutdown()

    # ==============================================================
    # T265 RAW MODE — Direct T265 tracking (fallback)
    # ==============================================================

    def _run_t265(self):
        """Use T265 raw tracking as odometry source."""
        try:
            import pyrealsense2 as rs
        except ImportError:
            logger.error("pyrealsense2 not available, using simulated")
            self._run_simulated()
            return

        try:
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.pose)
            pipe.start(cfg)
            logger.info("T265 raw tracking started")
        except Exception as e:
            logger.error(f"T265 init failed: {e}")
            self._run_simulated()
            return

        while self._running:
            try:
                frames = pipe.wait_for_frames(timeout_ms=500)
                pose_frame = frames.get_pose_frame()
                if not pose_frame:
                    continue

                data = pose_frame.get_pose_data()
                t = data.translation
                r = data.rotation

                # T265 frame → NED
                north = -t.z
                east = t.x
                down = t.y

                yaw = math.atan2(2*(r.w*r.z + r.x*r.y), 1 - 2*(r.y*r.y + r.z*r.z))
                pitch = math.asin(max(-1, min(1, 2*(r.w*r.y - r.z*r.x))))
                roll = math.atan2(2*(r.w*r.x + r.y*r.z), 1 - 2*(r.x*r.x + r.y*r.y))

                conf_map = {0: 0, 1: 25, 2: 60, 3: 95}
                confidence = conf_map.get(data.tracker_confidence, 0)

                self._update_odom(
                    north, east, down, roll, pitch, yaw,
                    time.time(), confidence=confidence
                )

            except Exception as e:
                logger.error(f"T265 error: {e}")
                time.sleep(0.01)

        pipe.stop()

    # ==============================================================
    # SIMULATED MODE
    # ==============================================================

    def _run_simulated(self):
        """Simulated odometry for testing."""
        logger.info("Running simulated odometry")
        while self._running:
            self._update_odom(0, 0, 0, 0, 0, 0, time.time(), confidence=90)
            time.sleep(1.0 / 30)

    # ==============================================================
    # SITL MODE — Read position from MAVLink SITL
    # ==============================================================

    def _run_sitl(self):
        """
        Read position from ArduPilot SITL via MAVLink bridge.
        
        In SITL mode, the MAVLink bridge caches LOCAL_POSITION_NED.
        We read that cached message and treat it as our odometry source.
        This means the SAME navigator/flight_controller code works
        for both real hardware (RTAB-Map) and SITL (ground truth position).
        """
        logger.info("Running SITL odometry (reading from MAVLink bridge)")
        bridge = self.config.get("bridge")
        if bridge is None:
            logger.error("SITL mode requires 'bridge' in config!")
            self._run_simulated()
            return

        while self._running:
            pos = bridge.get_latest("LOCAL_POSITION_NED")
            att = bridge.get_latest("ATTITUDE")
            
            if pos is not None:
                north = pos.x
                east = pos.y
                down = pos.z  # Already NED
                
                roll = att.roll if att else 0
                pitch = att.pitch if att else 0
                yaw = att.yaw if att else 0
                
                self._update_odom(
                    north, east, down, roll, pitch, yaw,
                    time.time(), confidence=95
                )
                
                # Update angular rates from ATTITUDE message
                if att:
                    with self._odom_lock:
                        self._odom.roll_rate = att.rollspeed if hasattr(att, 'rollspeed') else 0
                        self._odom.pitch_rate = att.pitchspeed if hasattr(att, 'pitchspeed') else 0
                        self._odom.yaw_rate = att.yawspeed if hasattr(att, 'yawspeed') else 0
                
                # Use ground-truth velocity from SITL (much better than noisy derivatives)
                with self._odom_lock:
                    self._odom.vx = pos.vx
                    self._odom.vy = pos.vy
                    self._odom.vz = pos.vz
            
            time.sleep(1.0 / 50)  # 50 Hz for tight PID control

    def print_status(self):
        """Print current odometry status."""
        d = self.get()
        print(f"\n--- Odometry Status ---")
        print(f"  Position:  N={d.x:.3f}  E={d.y:.3f}  D={d.z:.3f}  (alt={d.altitude:.2f}m)")
        print(f"  Velocity:  N={d.vx:.3f}  E={d.vy:.3f}  D={d.vz:.3f}  (speed={d.speed_horizontal:.2f}m/s)")
        print(f"  Accel:     N={d.ax:.3f}  E={d.ay:.3f}  D={d.az:.3f}")
        print(f"  Orient:    R={math.degrees(d.roll):.1f}  P={math.degrees(d.pitch):.1f}  Y={math.degrees(d.yaw):.1f}")
        print(f"  Confidence: {d.confidence}%  Features: {d.num_features}  Inliers: {d.num_inliers}")
        print(f"  Distance:  {self.total_distance:.2f}m  Home: {d.distance_from_home:.2f}m")
        print(f"  Loop closures: {self.loop_closures}")


# ==============================================================
# ODOM PROVIDER FACTORY
# ==============================================================


def create_odom_provider(
    source: str,
    config: Optional[dict] = None,
    bridge=None,
) -> "RTABMapOdom":
    """
    Create an odometry provider based on source.
    
    Args:
        source: "sitl" | "ros2" | "orbslam3" | "t265_raw" | "simulated"
        config: Optional dict; for orbslam3/ros2 may include odom_topic, etc.
        bridge: MAVLink bridge (required for sitl mode)
    
    Returns:
        Odometry provider with get(), start(), stop() interface.
    """
    cfg = config or {}
    if source == "sitl":
        return RTABMapOdom(mode="sitl", config={"bridge": bridge, **cfg})
    if source == "orbslam3":
        from .orbslam3_odom import ORBSLAM3Odom
        return ORBSLAM3Odom(config=cfg)
    if source in ("ros2", "rtabmap"):
        return RTABMapOdom(mode="ros2", config=cfg)
    if source == "t265_raw":
        return RTABMapOdom(mode="t265_raw", config=cfg)
    if source == "simulated":
        return RTABMapOdom(mode="simulated", config=cfg)
    raise ValueError(f"Unknown odom source: {source}")
