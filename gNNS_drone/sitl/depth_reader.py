"""
gNNS Drone — Depth Camera Reader (ROS Noetic + Gazebo Classic)
================================================================
Reads depth images from Gazebo's depth camera sensor via ROS topics.

The Gazebo model uses libgazebo_ros_depth_camera.so which publishes:
  /gnns/depth/image_raw  →  sensor_msgs/Image (32FC1 depth)
  /gnns/depth/image      →  sensor_msgs/Image (RGB)
  /gnns/depth/points     →  sensor_msgs/PointCloud2

This module subscribes to the depth topic and provides:
  - get_center_distance()    → distance at image center (like rangefinder)
  - get_depth_grid()         → NxN grid of distances (obstacle map)
  - get_min_distance()       → closest obstacle distance
  - check_landing_safety()   → is the area below safe to land?

Requires: source /opt/ros/noetic/setup.bash before running.
"""

import threading
import time
import math
import logging
import numpy as np

logger = logging.getLogger("gnns.depth")

# Try to import ROS — graceful fallback if not available
try:
    import rospy
    from sensor_msgs.msg import Image
    HAS_ROS = True
    logger.info("ROS Noetic available — depth camera enabled")
except ImportError:
    HAS_ROS = False
    logger.warning("ROS not available — depth camera disabled (run: source /opt/ros/noetic/setup.bash)")


class DepthReader:
    """
    Reads depth images from Gazebo depth camera via ROS.

    Subscribes to the depth image topic and processes frames
    to extract obstacle information for the safety grid.
    """

    def __init__(self, depth_topic="/gnns/depth/image_raw",
                 image_width=320, image_height=240,
                 hfov_deg=80.0, min_range=0.2, max_range=10.0):
        """
        Args:
            depth_topic: ROS topic for depth image (32FC1 format)
            image_width: Expected image width
            image_height: Expected image height
            hfov_deg: Horizontal field of view in degrees
            min_range: Minimum valid depth (meters)
            max_range: Maximum valid depth (meters)
        """
        self.depth_topic = depth_topic
        self.image_width = image_width
        self.image_height = image_height
        self.hfov = math.radians(hfov_deg)
        self.min_range = min_range
        self.max_range = max_range

        # Latest depth frame
        self._depth_frame = None
        self._frame_lock = threading.Lock()
        self._frame_time = 0
        self._frame_count = 0

        # ROS state
        self._ros_initialized = False
        self._subscriber = None

    def start(self):
        """Initialize ROS node and subscribe to depth topic."""
        if not HAS_ROS:
            logger.error("Cannot start depth reader — ROS not available!")
            return False

        try:
            if not rospy.core.is_initialized():
                rospy.init_node("gnns_depth_reader", anonymous=True,
                               disable_signals=True)
            self._subscriber = rospy.Subscriber(
                self.depth_topic, Image, self._depth_callback,
                queue_size=1, buff_size=2**20
            )
            self._ros_initialized = True
            logger.info(f"Depth reader started — topic: {self.depth_topic}")
            return True
        except Exception as e:
            logger.error(f"Failed to start depth reader: {e}")
            return False

    def stop(self):
        """Unsubscribe from depth topic."""
        if self._subscriber:
            self._subscriber.unregister()
            self._subscriber = None
        self._ros_initialized = False

    def _depth_callback(self, msg: 'Image'):
        """Process incoming depth image from Gazebo."""
        try:
            # Convert ROS Image to numpy array
            if msg.encoding == "32FC1":
                # 32-bit float depth image
                depth = np.frombuffer(msg.data, dtype=np.float32)
                depth = depth.reshape((msg.height, msg.width))
            elif msg.encoding == "16UC1":
                # 16-bit unsigned integer (mm)
                depth = np.frombuffer(msg.data, dtype=np.uint16)
                depth = depth.reshape((msg.height, msg.width)).astype(np.float32) / 1000.0
            else:
                logger.warning(f"Unknown depth encoding: {msg.encoding}")
                return

            # Replace inf/nan with max_range
            depth = np.where(np.isfinite(depth), depth, self.max_range)
            depth = np.clip(depth, self.min_range, self.max_range)

            with self._frame_lock:
                self._depth_frame = depth
                self._frame_time = time.time()
                self._frame_count += 1

        except Exception as e:
            logger.error(f"Depth callback error: {e}")

    @property
    def has_data(self) -> bool:
        """Check if we have recent depth data."""
        return (self._depth_frame is not None and
                time.time() - self._frame_time < 2.0)

    @property
    def fps(self) -> float:
        """Approximate frames per second."""
        return self._frame_count / max(1, time.time() - self._frame_time) if self._frame_count > 0 else 0

    def get_center_distance(self) -> float:
        """
        Get distance at image center (like a rangefinder).

        Uses the median of a small center patch for noise rejection.
        Returns -1 if no data.
        """
        with self._frame_lock:
            if self._depth_frame is None:
                return -1.0

            h, w = self._depth_frame.shape
            # Take center 10x10 patch
            cy, cx = h // 2, w // 2
            patch = self._depth_frame[cy-5:cy+5, cx-5:cx+5]
            return float(np.median(patch))

    def get_min_distance(self) -> float:
        """Get closest obstacle distance in the entire field of view."""
        with self._frame_lock:
            if self._depth_frame is None:
                return -1.0
            valid = self._depth_frame[self._depth_frame > self.min_range]
            if len(valid) == 0:
                return -1.0
            return float(np.min(valid))

    def get_depth_grid(self, grid_size: int = 5) -> np.ndarray:
        """
        Divide depth image into NxN grid and return average depth per cell.

        Args:
            grid_size: Number of grid cells per row/column (default 5x5)

        Returns:
            NxN numpy array of average depths, or None if no data.
            Low values = close obstacle, high values = clear ground.
        """
        with self._frame_lock:
            if self._depth_frame is None:
                return None

            h, w = self._depth_frame.shape
            cell_h, cell_w = h // grid_size, w // grid_size
            grid = np.zeros((grid_size, grid_size), dtype=np.float32)

            for r in range(grid_size):
                for c in range(grid_size):
                    patch = self._depth_frame[
                        r*cell_h:(r+1)*cell_h,
                        c*cell_w:(c+1)*cell_w
                    ]
                    # Use median for robustness against noise
                    grid[r, c] = np.median(patch)

            return grid

    def check_landing_safety(self, altitude: float) -> dict:
        """
        Analyze depth image to determine if area below is safe to land.

        Compares expected depth (altitude) with actual depth readings.
        If readings are much shorter than altitude = obstacle present.

        Args:
            altitude: Current drone altitude (meters)

        Returns:
            dict with:
              - safe: bool — is landing safe?
              - center_dist: float — distance at center
              - min_dist: float — closest obstacle
              - obstacle_fraction: float — fraction of image with obstacles
              - grid: 5x5 depth grid
        """
        result = {
            "safe": True,
            "center_dist": -1,
            "min_dist": -1,
            "obstacle_fraction": 0.0,
            "grid": None,
            "has_data": self.has_data,
        }

        if not self.has_data:
            result["safe"] = False  # No data = unsafe
            return result

        result["center_dist"] = self.get_center_distance()
        result["min_dist"] = self.get_min_distance()
        result["grid"] = self.get_depth_grid(5)

        # Obstacle detection: if depth reading is much less than altitude,
        # something is sticking up below the drone
        with self._frame_lock:
            if self._depth_frame is not None:
                # Threshold: obstacle if depth < 60% of altitude
                threshold = altitude * 0.6
                obstacle_pixels = np.sum(self._depth_frame < threshold)
                total_pixels = self._depth_frame.size
                result["obstacle_fraction"] = float(obstacle_pixels / total_pixels)

                # Unsafe if > 15% of the image shows obstacles
                if result["obstacle_fraction"] > 0.15:
                    result["safe"] = False

                # Unsafe if center has obstacle
                if result["center_dist"] > 0 and result["center_dist"] < threshold:
                    result["safe"] = False

        return result

    def depth_to_ground_map(self, altitude: float, drone_n: float, drone_e: float,
                            drone_yaw: float = 0, fov_radius: float = 3.0) -> list:
        """
        Convert depth image to list of (n, e, height_offset) points.
        Projects and ROTATES pixels according to drone yaw.
        """
        points = []
        grid = self.get_depth_grid(7)  # 7x7 grid

        if grid is None:
            return points

        cos_y = math.cos(drone_yaw)
        sin_y = math.sin(drone_yaw)

        for r in range(grid.shape[0]):
            for c in range(grid.shape[1]):
                depth = grid[r, c]

                # Map grid cell to local frame offsets
                # frac_r increases -> South. frac_c increases -> East.
                frac_r = (r - grid.shape[0] / 2) / (grid.shape[0] / 2)
                frac_c = (c - grid.shape[1] / 2) / (grid.shape[1] / 2)
                
                # Local offsets relative to drone facing North
                local_n = -frac_r * fov_radius
                local_e = frac_c * fov_radius

                # Rotate local offsets by drone yaw to get world offsets
                world_n = local_n * cos_y - local_e * sin_y
                world_e = local_n * sin_y + local_e * cos_y

                n = drone_n + world_n
                e = drone_e + world_e

                # Height offset and state classification
                height_offset = max(0, altitude - depth)
                if depth < altitude * 0.6:
                    state = 3  # OBSTACLE
                elif depth < altitude * 0.85:
                    state = 2  # ROUGH
                else:
                    state = 1  # SAFE

                points.append((n, e, state, height_offset))

        return points
