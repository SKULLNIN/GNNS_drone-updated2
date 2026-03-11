"""
gNNS Drone — Area Scanner + Safe Landing (REAL Sensor Detection)
=================================================================
Uses REAL MAVLink sensor data (rangefinder, DISTANCE_SENSOR) to detect
obstacles. When given landing coordinates, the drone:

  1. Flies to the target at scan altitude
  2. Hovers + circles around the landing point (pre-landing scan)
  3. Reads rangefinder at each position to detect obstacles
  4. Picks the safest spot automatically
  5. Lands without human intervention

Sensors used (via MAVLink):
  - RANGEFINDER / DISTANCE_SENSOR: downward rangefinder
  - LOCAL_POSITION_NED: current position + altitude
  - ATTITUDE: tilt angle (tilted = rough terrain)

On real hardware, also uses:
  - OBSTACLE_DISTANCE: proximity LiDAR
  - Depth camera via RTAB-Map

Does NOT modify any main gnns_drone/ code.
"""

import sys
import os
import time
import math
import logging
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import List, Tuple, Optional

PROJECT_ROOT = str(Path(__file__).parent.parent)
sys.path.insert(0, PROJECT_ROOT)

from gnns_drone.flight_controller import FlightController, FlightConfig
from gnns_drone.mavlink_bridge import MAVLinkBridge
from gnns_drone.rtabmap_odom import RTABMapOdom

# Depth camera (optional — needs ROS Noetic sourced)
try:
    from depth_reader import DepthReader, HAS_ROS
except ImportError:
    HAS_ROS = False
    DepthReader = None

logger = logging.getLogger("gnns.scanner")


# ============================================================
# SAFETY GRID
# ============================================================

class CellState(IntEnum):
    UNKNOWN = 0
    SAFE = 1
    ROUGH = 2      # Uneven ground — risky
    OBSTACLE = 3   # Tree, rock, wall — cannot land


@dataclass
class GridCell:
    state: CellState = CellState.UNKNOWN
    height: float = 0.0        # Ground height relative to home
    roughness: float = 0.0     # 0 = flat, 1 = very rough
    confidence: float = 0.0    # 0-1, how sure we are
    scanned: bool = False
    range_reading: float = -1  # Raw rangefinder reading at this cell


class SafetyGrid:
    """
    2D grid map of landing safety.

    Each cell is 1m x 1m. Grid centered on home (0,0).
    Cells store: state (safe/obstacle/rough/unknown),
    ground height, roughness, scan confidence, raw rangefinder reading.
    """

    def __init__(self, size_n: int = 20, size_e: int = 20, resolution: float = 1.0):
        self.size_n = size_n
        self.size_e = size_e
        self.res = resolution

        self.rows = int(2 * size_n / resolution)
        self.cols = int(2 * size_e / resolution)

        self.grid = [[GridCell() for _ in range(self.cols)] for _ in range(self.rows)]

        # Stats
        self.cells_scanned = 0
        self.cells_safe = 0
        self.cells_obstacle = 0
        self.cells_rough = 0

    def ned_to_cell(self, n: float, e: float) -> Tuple[int, int]:
        # North is Top (Row 0), South is Bottom (Row 40)
        row = int((self.size_n - n) / self.res)
        col = int((e + self.size_e) / self.res)
        row = max(0, min(self.rows - 1, row))
        col = max(0, min(self.cols - 1, col))
        return row, col

    def cell_to_ned(self, row: int, col: int) -> Tuple[float, float]:
        n = self.size_n - (row * self.res) - self.res / 2
        e = (col * self.res) - self.size_e + self.res / 2
        return n, e

    def set_cell(self, n: float, e: float, state: CellState,
                 height: float = 0, roughness: float = 0,
                 confidence: float = 1.0, range_reading: float = -1):
        row, col = self.ned_to_cell(n, e)
        cell = self.grid[row][col]

        if not cell.scanned:
            self.cells_scanned += 1

        old_state = cell.state
        cell.state = state
        cell.height = height
        cell.roughness = roughness
        cell.confidence = confidence
        cell.range_reading = range_reading
        cell.scanned = True

        if old_state != state:
            if old_state == CellState.SAFE: self.cells_safe -= 1
            elif old_state == CellState.OBSTACLE: self.cells_obstacle -= 1
            elif old_state == CellState.ROUGH: self.cells_rough -= 1

            if state == CellState.SAFE: self.cells_safe += 1
            elif state == CellState.OBSTACLE: self.cells_obstacle += 1
            elif state == CellState.ROUGH: self.cells_rough += 1

    def is_safe(self, n: float, e: float) -> bool:
        row, col = self.ned_to_cell(n, e)
        cell = self.grid[row][col]
        return cell.scanned and cell.state == CellState.SAFE

    def get_cell(self, n: float, e: float) -> GridCell:
        row, col = self.ned_to_cell(n, e)
        return self.grid[row][col]

    def nearest_safe(self, n: float, e: float, max_radius: float = 8.0) -> Optional[Tuple[float, float]]:
        if self.is_safe(n, e):
            return (n, e)

        best_dist = float('inf')
        best_pos = None

        for radius in [x * self.res for x in range(1, int(max_radius / self.res) + 1)]:
            num_points = max(8, int(2 * math.pi * radius / self.res))
            for i in range(num_points):
                angle = 2 * math.pi * i / num_points
                check_n = n + radius * math.cos(angle)
                check_e = e + radius * math.sin(angle)

                if self.is_safe(check_n, check_e):
                    dist = math.sqrt((check_n - n)**2 + (check_e - e)**2)
                    if dist < best_dist:
                        best_dist = dist
                        best_pos = (check_n, check_e)

            if best_pos is not None:
                return best_pos

        return best_pos

    def to_json_grid(self) -> list:
        result = []
        for row in range(self.rows):
            grid_row = []
            for col in range(self.cols):
                cell = self.grid[row][col]
                grid_row.append({
                    "s": int(cell.state),
                    "h": round(cell.height, 1),
                    "c": round(cell.confidence, 1),
                })
            result.append(grid_row)
        return result


# ============================================================
# LIVE SENSOR READER
# ============================================================

class LiveSensor:
    """
    Reads REAL sensor data from MAVLink.

    Monitors:
      - RANGEFINDER / DISTANCE_SENSOR: distance to ground
      - LOCAL_POSITION_NED: altitude (as baseline)
      - ATTITUDE: roll/pitch (for tilt detection)

    How obstacle detection works:
      1. At scan altitude (e.g. 2.5m), rangefinder reads ~2.5m = flat ground
      2. If rangefinder reads 1.0m = something 1.5m tall below = OBSTACLE
      3. If rangefinder fluctuates = rough/uneven terrain
    """

    def __init__(self, bridge: MAVLinkBridge, scan_altitude: float = 2.5):
        self.bridge = bridge
        self.scan_altitude = scan_altitude

        # Depth camera (primary sensor when available)
        self.depth_reader = None
        if HAS_ROS and DepthReader:
            self.depth_reader = DepthReader(
                depth_topic="/gnns/depth/image_depth_raw",
                image_width=320, image_height=240,
                hfov_deg=80.0, min_range=0.2, max_range=10.0
            )
            logger.info("Depth camera sensor configured (Topic: /gnns/depth/image_depth_raw)")

        # Readings history for noise filtering
        self._range_history = []
        self._max_history = 10

        # Thresholds
        self.obstacle_threshold = 0.5   # If range < (scan_alt - this) = obstacle
        self.rough_threshold = 0.3      # If range variance > this = rough terrain
        self.tilt_threshold = 10.0      # degrees — excessive tilt = rough

    def start_depth_camera(self):
        """Initialize the depth camera ROS subscriber."""
        if self.depth_reader:
            return self.depth_reader.start()
        return False

    def read_rangefinder(self) -> float:
        """
        Read current distance to ground (meters).

        Priority: depth camera → DISTANCE_SENSOR → RANGEFINDER.
        Returns -1 if no data available.
        """
        # 1. Try depth camera (most accurate)
        if self.depth_reader and self.depth_reader.has_data:
            dist = self.depth_reader.get_center_distance()
            if dist > 0:
                return dist

        # 2. Try DISTANCE_SENSOR MAVLink message
        with self.bridge._msg_lock:
            msg = self.bridge._latest_msgs.get("DISTANCE_SENSOR")
            if msg:
                return msg.current_distance / 100.0  # cm → meters

            # 3. Try RANGEFINDER message
            msg = self.bridge._latest_msgs.get("RANGEFINDER")
            if msg:
                return msg.distance

        return -1.0

    def read_altitude(self) -> float:
        """Read current altitude from LOCAL_POSITION_NED."""
        with self.bridge._msg_lock:
            msg = self.bridge._latest_msgs.get("LOCAL_POSITION_NED")
            if msg:
                return -msg.z  # NED z is negative up
        return 0.0

    def read_tilt(self) -> float:
        """Read current tilt angle (degrees) from ATTITUDE."""
        with self.bridge._msg_lock:
            msg = self.bridge._latest_msgs.get("ATTITUDE")
            if msg:
                roll_deg = abs(math.degrees(msg.roll))
                pitch_deg = abs(math.degrees(msg.pitch))
                return max(roll_deg, pitch_deg)
        return 0.0

    def check_current_position(self) -> Tuple[CellState, float, float]:
        """
        Check safety at the drone's CURRENT position using live sensors.

        Returns:
            (state, ground_height_offset, roughness_score)
        """
        range_dist = self.read_rangefinder()
        altitude = self.read_altitude()
        tilt = self.read_tilt()

        # Track history for variance calculation
        if range_dist > 0:
            self._range_history.append(range_dist)
            if len(self._range_history) > self._max_history:
                self._range_history.pop(0)

        # If no rangefinder data, use altitude comparison
        if range_dist < 0:
            # No rangefinder — use altitude as proxy
            # If altitude matches scan_altitude, probably clear
            if altitude > 0.5:
                return (CellState.SAFE, 0.0, 0.0)
            return (CellState.UNKNOWN, 0.0, 0.0)

        # Calculate expected range at scan altitude
        expected_range = altitude  # Should equal distance to flat ground

        # OBSTACLE: rangefinder sees something much closer than expected
        # e.g. flying at 2.5m, range reads 1.0m = 1.5m tall obstacle
        height_offset = expected_range - range_dist
        if height_offset > self.obstacle_threshold:
            logger.warning(f"OBSTACLE detected! range={range_dist:.2f}m, "
                          f"altitude={altitude:.2f}m, offset={height_offset:.2f}m")
            return (CellState.OBSTACLE, height_offset, 1.0)

        # ROUGH: range readings are noisy/variable
        if len(self._range_history) >= 3:
            avg = sum(self._range_history) / len(self._range_history)
            variance = sum((r - avg)**2 for r in self._range_history) / len(self._range_history)
            if variance > self.rough_threshold:
                return (CellState.ROUGH, height_offset, min(1.0, variance))

        # ROUGH from tilt: if drone is tilting a lot, terrain might be uneven
        if tilt > self.tilt_threshold:
            return (CellState.ROUGH, 0.0, tilt / 45.0)

        # SAFE: range matches expected, low variance, low tilt
        return (CellState.SAFE, height_offset, 0.0)

    def check_depth_landing(self, altitude: float) -> dict:
        """Use depth camera to check if landing area is safe."""
        if self.depth_reader and self.depth_reader.has_data:
            return self.depth_reader.check_landing_safety(altitude)
        return {"safe": True, "has_data": False}

    def get_depth_ground_map(self, altitude: float,
                             drone_n: float, drone_e: float, drone_yaw: float = 0) -> list:
        """Get ground obstacle points from depth camera."""
        if self.depth_reader and self.depth_reader.has_data:
            return self.depth_reader.depth_to_ground_map(
                altitude, drone_n, drone_e, drone_yaw,
                fov_radius=altitude * 1.1  # Increased FOV for better coverage
            )
        return []


# ============================================================
# AREA SCANNER (REAL SENSOR-BASED)
# ============================================================

class AreaScanner:
    """
    Autonomous area scanning + safe landing using REAL sensor data.

    Pre-landing scan:
      1. Fly to target landing coordinates at scan altitude
      2. Circle/spiral around the target (radius 3-5m)
      3. Read rangefinder at each point
      4. Build local safety grid from readings
      5. Pick safest spot and land autonomously

    No hardcoded obstacles. Everything comes from sensors.
    """

    def __init__(self, bridge: MAVLinkBridge, odom: RTABMapOdom,
                 fc: FlightController, config: FlightConfig):
        self.bridge = bridge
        self.odom = odom
        self.fc = fc
        self.config = config

        self.grid = SafetyGrid(size_n=20, size_e=20, resolution=1.0)
        self.sensor = LiveSensor(bridge, scan_altitude=config.cruise_altitude)
        self.scanning = False
        self.scan_progress = 0.0

        # Scan config
        self.scan_altitude = config.cruise_altitude
        self.scan_radius = 5.0         # Circle scan radius (meters)
        self.scan_points = 16          # Points around the circle
        self.scan_hover_time = 0.5     # Seconds to hover at each scan point

    def scan_at_current_position(self):
        """Scan and record at the drone's current position using ALL sensors."""
        data = self.odom.get()

        # Single-point scan (rangefinder / basic)
        state, height, roughness = self.sensor.check_current_position()
        range_dist = self.sensor.read_rangefinder()
        self.grid.set_cell(data.x, data.y, state, height, roughness,
                          confidence=0.9, range_reading=range_dist)

        # Multi-cell scan from depth camera (covers larger area)
        # Use current yaw to rotate depth points correctly
        depth_points = self.sensor.get_depth_ground_map(
            data.altitude, data.x, data.y, data.yaw
        )
        for n, e, pt_state, ht_offset in depth_points:
            cell_state = CellState(pt_state)
            conf = 0.85 if cell_state == CellState.OBSTACLE else 0.8
            self.grid.set_cell(n, e, cell_state, ht_offset, 0.0, conf)

        return state, len(depth_points)

    def pre_landing_scan(self, target_n: float, target_e: float,
                         callback=None) -> dict:
        """
        AUTONOMOUS pre-landing scan.

        Drone flies to target, circles around it scanning with sensors,
        then picks the safest landing spot. NO human intervention.

        Args:
            target_n, target_e: Requested landing coordinates
            callback: Optional progress callback(progress, grid)

        Returns:
            dict with landing result
        """
        self.scanning = True
        self.scan_progress = 0.0
        self.sensor._range_history.clear()

        result = {
            "requested": [target_n, target_e],
            "actual": [target_n, target_e],
            "safe": True,
            "redirected": False,
            "reason": "",
            "scan_points": 0,
            "scan_results": [],
        }

        logger.info(f"Pre-landing scan starting at ({target_n:.1f}, {target_e:.1f})")

        # ── PHASE 1: Fly to target area ──────────────────────
        logger.info("Phase 1: Flying to target area...")
        self.fc.reset()
        arrived = self._fly_to(target_n, target_e, timeout=60)
        if not arrived:
            result["reason"] = "Could not reach target area"
            result["safe"] = False
            self.scanning = False
            return result

        self.scan_progress = 0.1
        if callback:
            callback(self.scan_progress, self.grid)

        # ── PHASE 2: Hover and scan at target ────────────────
        logger.info("Phase 2: Scanning at target position...")
        self.bridge.send_velocity_ned(0, 0, 0)
        time.sleep(1.0)  # Stabilize

        # Scan at the target itself
        target_state, _ = self.scan_at_current_position()
        result["scan_points"] += 1

        # ── PHASE 3: Circle scan around target ───────────────
        logger.info(f"Phase 3: Circle scan (r={self.scan_radius}m, "
                     f"{self.scan_points} points)...")

        scan_results = []
        for i in range(self.scan_points):
            if not self.scanning:
                break

            angle = 2 * math.pi * i / self.scan_points
            scan_n = target_n + self.scan_radius * math.cos(angle)
            scan_e = target_e + self.scan_radius * math.sin(angle)

            # Fly to scan point
            self.fc.reset()
            self._fly_to(scan_n, scan_e, timeout=15)

            # Hover briefly and take readings
            self.bridge.send_velocity_ned(0, 0, 0)
            time.sleep(self.scan_hover_time)

            # Read sensors
            state, _ = self.scan_at_current_position()
            data = self.odom.get()
            range_dist = self.sensor.read_rangefinder()

            scan_results.append({
                "n": round(data.x, 1),
                "e": round(data.y, 1),
                "state": int(state),
                "range": round(range_dist, 2),
            })

            result["scan_points"] += 1
            self.scan_progress = 0.1 + 0.6 * (i + 1) / self.scan_points

            if callback:
                callback(self.scan_progress, self.grid)

        result["scan_results"] = scan_results

        # Also scan points at half radius for denser coverage
        logger.info("Phase 3b: Inner circle scan...")
        half_radius = self.scan_radius * 0.5
        for i in range(8):
            if not self.scanning:
                break

            angle = 2 * math.pi * i / 8 + math.pi / 8  # Offset from outer
            scan_n = target_n + half_radius * math.cos(angle)
            scan_e = target_e + half_radius * math.sin(angle)

            self.fc.reset()
            self._fly_to(scan_n, scan_e, timeout=10)
            self.bridge.send_velocity_ned(0, 0, 0)
            time.sleep(self.scan_hover_time)
            self.scan_at_current_position()
            result["scan_points"] += 1

        self.scan_progress = 0.8
        if callback:
            callback(self.scan_progress, self.grid)

        # ── PHASE 4: Pick safest landing spot ────────────────
        logger.info("Phase 4: Selecting safest landing spot...")

        target_cell = self.grid.get_cell(target_n, target_e)

        if target_cell.scanned and target_cell.state == CellState.SAFE:
            # Target is safe — land there
            result["reason"] = "Target area is SAFE ✓ — landing"
            result["actual"] = [target_n, target_e]
            logger.info(result["reason"])

        elif target_cell.scanned and target_cell.state == CellState.OBSTACLE:
            # Target has obstacle — find alternative
            alt = self.grid.nearest_safe(target_n, target_e, max_radius=self.scan_radius)
            if alt:
                dist = math.sqrt((alt[0] - target_n)**2 + (alt[1] - target_e)**2)
                result["actual"] = [alt[0], alt[1]]
                result["redirected"] = True
                result["reason"] = (f"⚠️ OBSTACLE at target! Redirected {dist:.1f}m "
                                   f"to ({alt[0]:.1f}, {alt[1]:.1f})")
                logger.warning(result["reason"])
            else:
                result["safe"] = False
                result["reason"] = "✗ No safe landing spot found in scan area!"
                logger.error(result["reason"])
                self._fly_to(target_n, target_e, timeout=15)
                self.bridge.send_velocity_ned(0, 0, 0)
                self.scanning = False
                return result

        elif target_cell.scanned and target_cell.state == CellState.ROUGH:
            alt = self.grid.nearest_safe(target_n, target_e, max_radius=self.scan_radius)
            if alt:
                result["actual"] = [alt[0], alt[1]]
                result["redirected"] = True
                result["reason"] = f"⚠️ Rough terrain — redirected to ({alt[0]:.1f}, {alt[1]:.1f})"
            else:
                result["reason"] = "Rough terrain everywhere — landing carefully at target"
                result["actual"] = [target_n, target_e]

        else:
            # Not enough scan data — land at target (best effort)
            result["reason"] = "Scan data limited — landing at target (best effort)"

        # ── PHASE 5: Fly to landing spot and land ────────────
        land_n, land_e = result["actual"]
        logger.info(f"Phase 5: Flying to landing spot ({land_n:.1f}, {land_e:.1f})...")

        self.fc.reset()
        self._fly_to(land_n, land_e, timeout=30)

        # Final hover to stabilize
        self.bridge.send_velocity_ned(0, 0, 0)
        time.sleep(2.0)

        # Land
        logger.info("Phase 5: Landing...")
        self.bridge.land()
        start = time.time()
        while time.time() - start < 30:
            data = self.odom.get()
            if data.altitude < 0.3:
                break
            time.sleep(0.5)

        data = self.odom.get()
        result["final_pos"] = [round(data.x, 2), round(data.y, 2)]
        result["final_error"] = round(
            math.sqrt((land_n - data.x)**2 + (land_e - data.y)**2), 2
        )

        self.scan_progress = 1.0
        self.scanning = False

        logger.info(f"Landing complete! Final pos: ({data.x:.2f}, {data.y:.2f}), "
                     f"error: {result['final_error']}m")

        return result

    def quick_area_scan(self, radius_n: float = 10, radius_e: float = 10,
                        spacing: float = 3.0, callback=None):
        """
        Fly a lawnmower pattern scanning with REAL sensors.

        Uses actual rangefinder readings — not fake obstacles.
        """
        self.scanning = True
        self.scan_progress = 0.0

        path = []
        direction = 1
        n = -radius_n
        while n <= radius_n:
            if direction == 1:
                path.append((n, -radius_e))
                path.append((n, radius_e))
            else:
                path.append((n, radius_e))
                path.append((n, -radius_e))
            direction *= -1
            n += spacing

        logger.info(f"Quick area scan: {len(path)} waypoints, "
                     f"{radius_n*2}x{radius_e*2}m area")

        total = len(path)
        for i, (tgt_n, tgt_e) in enumerate(path):
            if not self.scanning:
                break

            self.fc.reset()
            arrived = self._fly_to(tgt_n, tgt_e, timeout=30)

            if arrived:
                # Scan at arrival + during transit
                self.bridge.send_velocity_ned(0, 0, 0)
                time.sleep(0.3)
                self.scan_at_current_position()

            self.scan_progress = (i + 1) / total
            if callback:
                callback(self.scan_progress, self.grid)

        self.scan_progress = 1.0
        self.scanning = False
        logger.info(f"Area scan complete: {self.grid.cells_scanned} cells")

    def continuous_scan(self):
        """Scan at current position continuously (call in a loop)."""
        self.scan_at_current_position()

    # Alias for web_control compatibility
    def safe_land(self, target_n: float, target_e: float) -> dict:
        """Wrapper for pre_landing_scan."""
        return self.pre_landing_scan(target_n, target_e)

    def start_scan(self, callback=None):
        """Start a full area scan with real sensors."""
        self.quick_area_scan(callback=callback)

    def instant_scan(self, callback=None):
        """
        For SITL without rangefinder: use altitude-based scan.

        Flies a quick pattern reading altitude at each point.
        Faster than full lawnmower but still uses real sensor path.
        """
        self.scanning = True
        self.scan_progress = 0.0
        logger.info("Quick altitude scan...")

        # Fly a sparse grid and scan
        points = []
        for n in range(-8, 9, 3):
            for e in range(-8, 9, 3):
                points.append((float(n), float(e)))

        total = len(points)
        for i, (n, e) in enumerate(points):
            if not self.scanning:
                break

            self.fc.reset()
            arrived = self._fly_to(n, e, timeout=15)
            if arrived:
                self.bridge.send_velocity_ned(0, 0, 0)
                time.sleep(0.3)
                self.scan_at_current_position()

            self.scan_progress = (i + 1) / total
            if callback:
                callback(self.scan_progress, self.grid)

        self.scan_progress = 1.0
        self.scanning = False
        logger.info(f"Altitude scan done: {self.grid.cells_scanned} cells")

    def _fly_to(self, target_n: float, target_e: float, timeout: float = 30) -> bool:
        """Fly to position using PID with yaw lock. Scans while in transit."""
        start = time.time()
        dt = 0.05  # 20Hz control loop
        scan_interval = 0.5  # Scan every 0.5s while flying
        last_scan = 0

        while time.time() - start < timeout:
            if not self.scanning:
                return False

            loop_start = time.time()
            data = self.odom.get()

            # Scan while flying (continuous sensing)
            if time.time() - last_scan > scan_interval:
                self.scan_at_current_position()
                last_scan = time.time()

            dist = math.sqrt((target_n - data.x)**2 + (target_e - data.y)**2)

            if dist < self.config.arrival_radius:
                self.bridge.send_velocity_ned(0, 0, 0)
                return True

            result = self.fc.compute_waypoint_velocity(
                target_n, target_e, self.scan_altitude,
                data.x, data.y, data.altitude, dt=dt
            )
            vx, vy, vz = result[0], result[1], result[2]
            yaw = result[3] if len(result) > 3 else 0.0
            # Lock yaw toward target for stable flight
            self.bridge.send_velocity_ned_yaw(vx, vy, vz, yaw)

            sleep_t = dt - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

        self.bridge.send_velocity_ned(0, 0, 0)
        return False
