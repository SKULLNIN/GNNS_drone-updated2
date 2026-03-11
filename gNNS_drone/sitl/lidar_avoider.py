"""
gNNS Drone — Lidar Obstacle Avoidance (ROS LaserScan → MAVLink)
================================================================
Subscribes to /gnns/scan (72 samples, 360°) and sends
OBSTACLE_DISTANCE to ArduPilot for BendyRuler avoidance.

The Gazebo plugin "libgazebo_ros_laser.so" publishes to:
  /gnns/scan  (robotNamespace=/gnns, topicName=scan)

This script:
  1. Subscribes to /gnns/scan
  2. Converts LaserScan → 72-element distance array (cm)
  3. Sends OBSTACLE_DISTANCE via MAVLink bridge
  4. ArduPilot uses BendyRuler to navigate around obstacles
"""

import time
import math
import threading
import logging
import numpy as np

logger = logging.getLogger("gnns.avoider")

# Try ROS Noetic (Python 2/3 compatible)
HAS_ROS = False
try:
    import rospy
    from sensor_msgs.msg import LaserScan
    HAS_ROS = True
    logger.info("ROS Noetic LaserScan imported successfully")
except ImportError:
    logger.warning("rospy not available — Lidar avoidance disabled")


class LidarAvoider:
    """
    Bridges ROS LaserScan → MAVLink OBSTACLE_DISTANCE.
    
    ArduPilot Parameters needed:
      PRX1_TYPE = 2     (MAVLink proximity)
      AVOID_ENABLE = 7  (All avoidance sources)
      OA_TYPE = 1       (BendyRuler path planner)
      AVOID_MARGIN = 1.5 (Safety bubble in meters)
    """

    def __init__(self, bridge):
        self.bridge = bridge
        self._subscriber = None
        self._running = False
        self._min_distance = 99.0
        self._last_log_time = 0
        self._msg_count = 0
        self._scan_topic = "/gnns/scan"

    def start(self):
        """Start the Lidar subscriber."""
        if not HAS_ROS:
            logger.error("ROS not available. Lidar avoidance DISABLED.")
            return False

        try:
            # Initialize ROS node if not already done
            if not rospy.core.is_initialized():
                rospy.init_node("gnns_lidar_avoider", anonymous=True,
                                disable_signals=True)

            self._subscriber = rospy.Subscriber(
                self._scan_topic, LaserScan,
                self._scan_callback, queue_size=1
            )
            self._running = True

            # Start a watchdog thread to periodically log status
            self._watchdog = threading.Thread(
                target=self._watchdog_loop, daemon=True, name="lidar-wd"
            )
            self._watchdog.start()

            logger.info(f"Lidar Avoider started — subscribing to {self._scan_topic}")
            return True

        except Exception as e:
            logger.error(f"Lidar Avoider failed to start: {e}")
            return False

    def _scan_callback(self, msg):
        """Convert ROS LaserScan to MAVLink OBSTACLE_DISTANCE."""
        try:
            raw_ranges = np.array(msg.ranges, dtype=np.float64)

            # Replace inf/nan with max_range
            max_range = msg.range_max if msg.range_max > 0 else 12.0
            min_range = msg.range_min if msg.range_min > 0 else 0.15
            raw_ranges = np.where(np.isfinite(raw_ranges), raw_ranges, max_range)

            # Resample to exactly 72 points (5° each for 360°)
            n_samples = len(raw_ranges)
            if n_samples == 72:
                distances = raw_ranges
            elif n_samples > 0:
                # Linear interpolation to 72 points
                indices = np.linspace(0, n_samples - 1, 72)
                distances = np.interp(indices, np.arange(n_samples), raw_ranges)
            else:
                return  # Empty scan

            # Convert to centimeters (MAVLink format)
            distances_cm = []
            for d in distances:
                d_clamped = max(min_range, min(d, max_range))
                distances_cm.append(int(d_clamped * 100))

            # Track closest obstacle
            current_min = float(np.min(distances))
            self._min_distance = current_min
            self._msg_count += 1

            # Log warnings for close objects
            if current_min < 2.0 and time.time() - self._last_log_time > 2.0:
                logger.warning(f"⚠️ OBSTACLE at {current_min:.1f}m!")
                self._last_log_time = time.time()

            # Send to ArduPilot Flight Controller
            self.bridge.send_obstacle_distances(
                distances_cm,
                increment_deg=5,
                angle_offset=0
            )

        except Exception as e:
            logger.error(f"Lidar callback error: {e}")

    def _watchdog_loop(self):
        """Periodically log lidar status."""
        while self._running:
            if self._msg_count > 0:
                logger.info(
                    f"LIDAR: {self._msg_count} scans received | "
                    f"Min dist: {self._min_distance:.1f}m"
                )
            else:
                logger.warning(
                    f"LIDAR: No scan data received yet on {self._scan_topic}. "
                    f"Check: rostopic list | grep scan"
                )
            time.sleep(10.0)

    def stop(self):
        """Stop the Lidar subscriber."""
        self._running = False
        if self._subscriber:
            self._subscriber.unregister()
            self._subscriber = None
        logger.info("Lidar Avoider stopped")

    @property
    def min_dist(self):
        """Closest obstacle distance (meters)."""
        return self._min_distance

    @property
    def is_receiving(self):
        """True if we've received at least one scan."""
        return self._msg_count > 0
