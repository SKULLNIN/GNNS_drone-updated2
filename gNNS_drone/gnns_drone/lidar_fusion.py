"""
gNNS Drone — LiDAR Fusion
==========================
Integrates RPLidar 2-D scans into the VIO pipeline.

Roles:
  1. Obstacle avoidance  — resample scan to 72 × 5° bins, send as
     MAVLink OBSTACLE_DISTANCE to ArduPilot's OA layer.
  2. Altitude estimation — when lidar is nadir-facing (or a downward
     bin is available), provides range_m for the EKF down-axis update.
  3. Proximity alert     — fires on_obstacle_close callback when any
     sector is closer than warn_m (default 1.5 m).

Extends / refactors the functionality in sitl/lidar_avoider.py.
The bridge reference is only used for obstacle_distance messages; all
range data is also accessible without a bridge (for unit tests / SITL).

Usage:
    lidar = LidarFusion(bridge, mode="altitude_also")
    lidar.on_obstacle_close(lambda d: print(f"CLOSE! {d:.2f}m"))
    lidar.start()

    # In VIO loop:
    algo.feed_lidar(lidar.get_nadir_range())

    # In safety loop:
    if lidar.get_min_distance() < 0.5:
        emergency_land()

    lidar.stop()
"""

import time
import math
import logging
import threading
from typing import List, Optional, Callable

logger = logging.getLogger("gnns.lidar")

# Number of obstacle-distance sectors ArduPilot expects for 360°
_NUM_SECTORS = 72
_SECTOR_DEG  = 360.0 / _NUM_SECTORS   # 5°


class LidarFusion:
    """
    LiDAR scan processor and obstacle/altitude provider.

    Modes:
      "obstacle_only"  — forward obstacle distances to ArduPilot only.
      "altitude_also"  — also derive nadir altitude from the forward
                         scan's centre bin (useful with a nadir LiDAR
                         or when sensor is tilted 90°).

    In ROS2 environments, start() subscribes to the scan topic via rclpy.
    Without ROS2, start() returns False and the caller must inject scans
    manually via _scan_callback_raw(ranges, angle_min, angle_increment).
    """

    def __init__(self, bridge=None, mode: str = "obstacle_only",
                 config: Optional[dict] = None):
        cfg = config or {}
        self.bridge = bridge
        self.mode   = mode

        # ROS2 topic to subscribe to
        self._scan_topic   = cfg.get("scan_topic", "/gnns/scan")

        # Obstacle alert threshold (metres)
        self._warn_m       = cfg.get("warn_m",       1.5)

        # Maximum usable range (cm for MAVLink, m for internal)
        self._max_range_cm = cfg.get("max_range_cm", 800)   # 8 m
        self._min_range_m  = cfg.get("min_range_m",  0.15)

        # Nadir bin index (0–71). 36 = directly forward → nadir when
        # sensor is mounted facing down. Adjust per installation.
        self._nadir_bin    = cfg.get("nadir_bin", 36)

        # Watchdog: if no scan for this many seconds, mark as disconnected
        self._watchdog_s   = cfg.get("watchdog_s", 2.0)

        # Internal state
        self._sectors: List[float] = [-1.0] * _NUM_SECTORS  # metres, -1=invalid
        self._last_scan_t: float   = 0.0
        self._running              = False
        self._thread: Optional[threading.Thread] = None
        self._watchdog_thread: Optional[threading.Thread] = None

        # Callbacks
        self._on_obstacle_close: List[Callable[[float], None]] = []

        self._lock = threading.Lock()

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def start(self) -> bool:
        """
        Start the LiDAR scan listener.

        Returns True if ROS2 subscription was established, False otherwise.
        In the False case, the caller must feed scans via inject_scan().
        """
        self._running = True
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True, name="lidar-watchdog"
        )
        self._watchdog_thread.start()

        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import LaserScan

            if not rclpy.ok():
                rclpy.init()

            node = rclpy.create_node("gnns_lidar_fusion")
            node.create_subscription(
                LaserScan,
                self._scan_topic,
                self._scan_callback,
                10,
            )
            self._thread = threading.Thread(
                target=self._ros_spin_loop,
                args=(node,),
                daemon=True,
                name="lidar-ros2",
            )
            self._thread.start()
            logger.info(f"LidarFusion subscribed to {self._scan_topic} (ROS2)")
            return True

        except ImportError:
            logger.warning(
                "ROS2 not available — LidarFusion in manual-inject mode. "
                "Call inject_scan() from your scan provider."
            )
            return False

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._watchdog_thread:
            self._watchdog_thread.join(timeout=2.0)
        logger.info("LidarFusion stopped")

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def on_obstacle_close(self, callback: Callable[[float], None]):
        """
        Register a callback fired when any sector < warn_m.

        callback(distance_m: float)
        """
        self._on_obstacle_close.append(callback)

    # ------------------------------------------------------------------ #
    # Data accessors
    # ------------------------------------------------------------------ #

    def get_nadir_range(self) -> float:
        """
        Return the nadir-facing LiDAR range in metres.

        Returns -1.0 if:
          - No scan received in the last watchdog_s seconds.
          - The nadir bin is invalid (zero or out of range).
        """
        with self._lock:
            if time.time() - self._last_scan_t > self._watchdog_s:
                return -1.0
            val = self._sectors[self._nadir_bin]
            return val if val > self._min_range_m else -1.0

    def get_min_distance(self) -> float:
        """
        Return the closest range across all valid sectors (metres).

        Returns -1.0 if no valid scan data.
        """
        with self._lock:
            if time.time() - self._last_scan_t > self._watchdog_s:
                return -1.0
            valid = [s for s in self._sectors if s > self._min_range_m]
            return float(min(valid)) if valid else -1.0

    def get_obstacle_sectors(self) -> List[float]:
        """
        Return a copy of the 72-element sector array (metres).

        Values <= 0 mean invalid / out-of-range.
        """
        with self._lock:
            return list(self._sectors)

    def get_obstacle_sectors_cm(self) -> List[int]:
        """
        Return 72-element sector array in centimetres (for MAVLink).

        Invalid sectors are encoded as 0 (ArduPilot ignores them).
        """
        with self._lock:
            return [
                min(int(s * 100), self._max_range_cm) if s > 0 else 0
                for s in self._sectors
            ]

    @property
    def is_receiving(self) -> bool:
        """True if a scan was received within the last watchdog period."""
        return time.time() - self._last_scan_t < self._watchdog_s

    @property
    def min_dist(self) -> float:
        """Alias for get_min_distance()."""
        return self.get_min_distance()

    # ------------------------------------------------------------------ #
    # Scan injection (non-ROS path / testing)
    # ------------------------------------------------------------------ #

    def inject_scan(self, ranges: List[float],
                    angle_min: float, angle_increment: float):
        """
        Manually inject a LaserScan range array.

        Args:
            ranges:          raw range values (metres, 0 = invalid).
            angle_min:       starting angle of first range (radians).
            angle_increment: angular step between consecutive ranges (radians).
        """
        self._process_scan(ranges, angle_min, angle_increment)

    # ------------------------------------------------------------------ #
    # Internal: ROS2 callback
    # ------------------------------------------------------------------ #

    def _scan_callback(self, msg):
        """ROS2 LaserScan message handler."""
        self._process_scan(
            list(msg.ranges),
            msg.angle_min,
            msg.angle_increment,
        )

    def _process_scan(self, ranges: List[float],
                      angle_min: float, angle_increment: float):
        """
        Resample arbitrary-resolution scan into 72 fixed 5° bins.

        Uses minimum range within each bin (conservative for safety).
        """
        if not ranges:
            return

        new_sectors = [float("inf")] * _NUM_SECTORS

        for i, r in enumerate(ranges):
            if r == 0.0 or math.isinf(r) or math.isnan(r):
                continue
            if r < self._min_range_m:
                continue

            angle_rad = angle_min + i * angle_increment
            angle_deg = math.degrees(angle_rad) % 360.0
            bin_idx   = int(angle_deg / _SECTOR_DEG) % _NUM_SECTORS
            if r < new_sectors[bin_idx]:
                new_sectors[bin_idx] = r

        # Replace inf with -1 (no data)
        final = [
            s if s != float("inf") else -1.0
            for s in new_sectors
        ]

        with self._lock:
            self._sectors = final
            self._last_scan_t = time.time()

        # Forward to ArduPilot obstacle avoidance
        if self.bridge is not None:
            try:
                sectors_cm = [
                    min(int(s * 100), self._max_range_cm) if s > 0 else 0
                    for s in final
                ]
                self.bridge.send_obstacle_distances(
                    distances=sectors_cm,
                    increment_deg=int(_SECTOR_DEG),
                    min_distance_cm=int(self._min_range_m * 100),
                    max_distance_cm=self._max_range_cm,
                )
            except Exception as e:
                logger.debug(f"send_obstacle_distances error: {e}")

        # Check proximity and fire callbacks
        valid_sectors = [s for s in final if s > 0]
        if valid_sectors:
            closest = min(valid_sectors)
            if closest < self._warn_m:
                for cb in self._on_obstacle_close:
                    try:
                        cb(closest)
                    except Exception as exc:
                        logger.error(f"obstacle_close callback error: {exc}")

    # ------------------------------------------------------------------ #
    # Internal: ROS2 spin loop
    # ------------------------------------------------------------------ #

    def _ros_spin_loop(self, node):
        try:
            import rclpy
            while self._running:
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception as e:
            logger.error(f"LidarFusion ROS2 spin error: {e}")
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass

    # ------------------------------------------------------------------ #
    # Internal: watchdog
    # ------------------------------------------------------------------ #

    def _watchdog_loop(self):
        """Log a warning every watchdog_s if no scan is received."""
        while self._running:
            time.sleep(self._watchdog_s)
            if not self.is_receiving and self._running:
                logger.warning(
                    f"LidarFusion: no scan for >{self._watchdog_s:.1f}s — "
                    "obstacle avoidance disabled"
                )

    # ------------------------------------------------------------------ #
    # Debug
    # ------------------------------------------------------------------ #

    def print_status(self):
        """Print a compact status summary."""
        md = self.get_min_distance()
        nr = self.get_nadir_range()
        valid = sum(1 for s in self._sectors if s > 0)
        print(f"\n--- LiDAR Status ---")
        print(f"  Receiving:    {self.is_receiving}")
        print(f"  Min distance: {md:.2f}m  Nadir: {nr:.2f}m")
        print(f"  Valid bins:   {valid}/{_NUM_SECTORS}")
        print(f"  Warn threshold: {self._warn_m}m")
        print(f"  Last scan:    {time.time()-self._last_scan_t:.1f}s ago")
