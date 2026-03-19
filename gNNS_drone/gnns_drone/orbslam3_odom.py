"""
gNNS Drone — ORB-SLAM3 Odometry Integration
============================================
Subscribes to ORB-SLAM3 ROS2 wrapper's nav_msgs/Odometry (IMU + RGB-D VIO)
and provides OdomData in NED frame for MAVLink VISION_POSITION_ESTIMATE.

Pipeline:
  RealSense (color + depth) + IMU → ORB-SLAM3 ROS2 wrapper → /odom
                                                          → this module → FC

Uses same OdomData interface as RTABMapOdom for drop-in replacement.
"""

import time
import math
import threading
import logging
from typing import Optional, Callable, List

from .rtabmap_odom import OdomData, VelocityEstimator

logger = logging.getLogger("gnns.orbslam3_odom")


def _ros_enu_to_ned(pos_x: float, pos_y: float, pos_z: float) -> tuple:
    """
    Convert ROS ENU position to NED.
    ROS ENU: x=East, y=North, z=Up
    NED: x=North, y=East, z=Down
    """
    north = pos_y
    east = pos_x
    down = -pos_z
    return north, east, down


def _ros_enu_vel_to_ned(vx: float, vy: float, vz: float) -> tuple:
    """
    Convert ROS ENU velocity to NED.
    """
    vn = vy   # North
    ve = vx   # East
    vd = -vz  # Down
    return vn, ve, vd


class ORBSLAM3Odom:
    """
    ORB-SLAM3 VIO odometry provider via ROS2.
    Subscribes to nav_msgs/Odometry and converts to NED OdomData.
    """

    def __init__(self, config: Optional[dict] = None):
        """
        Args:
            config: Optional dict with:
                - odom_topic: str, default "/odom"
                - odom_frame_convention: str, default "ros_enu_to_ned"
                - rtabmap_info_topic: str (optional), for loop-closure diagnostics
        """
        self.config = config or {}
        self._odom_topic = self.config.get("odom_topic", "/odom")
        self._frame_convention = self.config.get("odom_frame_convention", "ros_enu_to_ned")
        self._rtabmap_info_topic = self.config.get("rtabmap_info_topic")

        self._odom = OdomData()
        self._odom_lock = threading.Lock()
        self._vel_estimator = VelocityEstimator(alpha=0.3)
        self._running = False
        self._thread = None

        self._on_loop_closure: List[Callable] = []
        self._on_quality_drop: List[Callable] = []

        self.total_distance = 0.0
        self.loop_closures = 0
        self._prev_pos = (0.0, 0.0, 0.0)

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
        """Register callback when loop closure is detected (if rtabmap info used)."""
        self._on_loop_closure.append(callback)

    def on_quality_drop(self, callback: Callable):
        """Register callback when odometry quality drops below threshold."""
        self._on_quality_drop.append(callback)

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run_ros2, daemon=True)
        self._thread.start()
        logger.info(f"ORB-SLAM3 odometry started (topic={self._odom_topic})")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        logger.info("ORB-SLAM3 odometry stopped")

    def _update_odom(
        self,
        north: float,
        east: float,
        down: float,
        roll: float,
        pitch: float,
        yaw: float,
        timestamp: float,
        vx: Optional[float] = None,
        vy: Optional[float] = None,
        vz: Optional[float] = None,
        confidence: int = 90,
        features: int = 0,
        inliers: int = 0,
        loop_closure_id: int = -1,
    ):
        """Internal: update odometry with new pose."""
        if vx is None or vy is None or vz is None:
            vx, vy, vz, ax, ay, az = self._vel_estimator.update(
                north, east, down, timestamp
            )
        else:
            ax, ay, az = 0.0, 0.0, 0.0

        dist = math.sqrt(
            (north - self._prev_pos[0]) ** 2
            + (east - self._prev_pos[1]) ** 2
            + (down - self._prev_pos[2]) ** 2
        )
        self.total_distance += dist
        self._prev_pos = (north, east, down)

        with self._odom_lock:
            self._odom.timestamp = timestamp
            self._odom.x = north
            self._odom.y = east
            self._odom.z = down
            self._odom.vx = vx if vx is not None else self._odom.vx
            self._odom.vy = vy if vy is not None else self._odom.vy
            self._odom.vz = vz if vz is not None else self._odom.vz
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

        if loop_closure_id > 0:
            self.loop_closures += 1
            logger.info(f"Loop closure detected! ID={loop_closure_id}")
            for cb in self._on_loop_closure:
                try:
                    cb(loop_closure_id)
                except Exception as e:
                    logger.error(f"Loop closure callback error: {e}")

        if confidence < 30:
            for cb in self._on_quality_drop:
                try:
                    cb(confidence)
                except Exception as e:
                    logger.error(f"Quality drop callback error: {e}")

    def _run_ros2(self):
        """Subscribe to ORB-SLAM3 /odom and optional rtabmap/info."""
        try:
            import rclpy
            from rclpy.node import Node
            from nav_msgs.msg import Odometry
        except ImportError:
            logger.error(
                "ROS2 not installed! Install ROS2 Humble/Jazzy and nav_msgs.\n"
                "Cannot run ORB-SLAM3 odom subscriber."
            )
            return

        try:
            from rtabmap_msgs.msg import Info as RtabmapInfo
            has_rtabmap = True
        except ImportError:
            has_rtabmap = False
            RtabmapInfo = None

        rclpy.init()
        node = rclpy.create_node("gnns_orbslam3_odom_subscriber")

        def odom_callback(msg: Odometry):
            pos = msg.pose.pose.position
            twist = msg.twist.twist

            use_twist = all(
                math.isfinite(getattr(twist.linear, c, float("nan")))
                for c in ("x", "y", "z")
            ) and all(
                abs(getattr(twist.linear, c, 0)) < 50 for c in ("x", "y", "z")
            )

            if self._frame_convention == "ros_enu_to_ned":
                north, east, down = _ros_enu_to_ned(pos.x, pos.y, pos.z)
                if use_twist:
                    vn, ve, vd = _ros_enu_vel_to_ned(
                        twist.linear.x, twist.linear.y, twist.linear.z
                    )
                else:
                    vn = ve = vd = None
            else:
                north, east, down = pos.x, pos.y, pos.z
                vn = twist.linear.x if use_twist else None
                ve = twist.linear.y if use_twist else None
                vd = twist.linear.z if use_twist else None

            q = msg.pose.pose.orientation
            sinr = 2.0 * (q.w * q.x + q.y * q.z)
            cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            roll = math.atan2(sinr, cosr)

            sinp = 2.0 * (q.w * q.y - q.z * q.x)
            sinp = max(-1.0, min(1.0, sinp))
            pitch = math.asin(sinp)

            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)

            pos_cov = msg.pose.covariance[0] if len(msg.pose.covariance) > 0 else 0
            confidence = max(0, min(100, int(100 - pos_cov * 100)))

            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            self._update_odom(
                north, east, down, roll, pitch, yaw,
                timestamp,
                vx=vn, vy=ve, vz=vd,
                confidence=confidence,
            )

        def info_callback(msg):
            if hasattr(msg, "loop_closure_id") and msg.loop_closure_id > 0:
                with self._odom_lock:
                    self._odom.loop_closure_id = msg.loop_closure_id
                    self._odom.num_features = getattr(msg, "ref_words", 0)
                    self._odom.num_inliers = getattr(
                        msg, "loop_closure_transform_accepted", 0
                    )

        node.create_subscription(Odometry, self._odom_topic, odom_callback, 10)

        if has_rtabmap and self._rtabmap_info_topic and RtabmapInfo:
            node.create_subscription(
                RtabmapInfo, self._rtabmap_info_topic, info_callback, 10
            )
            logger.info(f"Subscribed to {self._odom_topic} and {self._rtabmap_info_topic}")
        else:
            logger.info(f"Subscribed to {self._odom_topic}")

        while self._running:
            rclpy.spin_once(node, timeout_sec=0.1)

        node.destroy_node()
        rclpy.shutdown()

    def print_status(self):
        """Print current odometry status."""
        d = self.get()
        print("\n--- ORB-SLAM3 Odometry Status ---")
        print(f"  Position:  N={d.x:.3f}  E={d.y:.3f}  D={d.z:.3f}  (alt={d.altitude:.2f}m)")
        print(f"  Velocity:  N={d.vx:.3f}  E={d.vy:.3f}  D={d.vz:.3f}  (speed={d.speed_horizontal:.2f}m/s)")
        print(f"  Orient:    R={math.degrees(d.roll):.1f}  P={math.degrees(d.pitch):.1f}  Y={math.degrees(d.yaw):.1f}")
        print(f"  Confidence: {d.confidence}%  Features: {d.num_features}")
        print(f"  Distance:  {self.total_distance:.2f}m  Home: {d.distance_from_home:.2f}m")
        print(f"  Loop closures: {self.loop_closures}")
