"""
gNNS Drone — Precision Landing Controller
===========================================
Guides the drone to land precisely on a detected target.

Landing sequence (3 phases):
  Phase 1 — APPROACH (alt > 3m):
     Fly horizontally toward target at cruise altitude
     Uses odometry + target detection

  Phase 2 — ALIGN (1m < alt < 3m):
     Descend slowly while centering over target
     Uses target pixel offsets to send velocity commands
     Drone must be within 50cm horizontally before descending further

  Phase 3 — FINAL DESCENT (alt < 1m):
     Very slow descent, locked horizontal position
     Uses depth camera for ground distance
     Cuts throttle when rangefinder reads < 20cm

Velocity-based control:
  Instead of position commands (go to X,Y), we use velocity commands.
  This gives MUCH smoother landing because:
  - We control speed directly (0.1-0.5 m/s descent)
  - We can adjust horizontally during descent
  - No "overshoot" like with position commands
"""

import time
import math
import logging
from enum import IntEnum
from dataclasses import dataclass
from .mavlink_bridge import MAVLinkBridge
from .rtabmap_odom import RTABMapOdom, OdomData
from .target_detector import TargetDetector, TargetDetection
from .flight_controller import FlightController, FlightConfig

logger = logging.getLogger("gnns.landing")


class LandingPhase(IntEnum):
    IDLE = 0
    APPROACH = 1       # Flying toward waypoint area
    SEARCH = 2         # Searching for landing target
    ALIGN = 3          # Centering over target
    FINAL_DESCENT = 4  # Slow controlled descent
    TOUCHDOWN = 5      # On ground
    ABORTED = 6        # Landing aborted


@dataclass
class LandingConfig:
    """Precision landing tuning parameters."""
    # Approach phase
    approach_altitude: float = 3.0    # Stay at this altitude while approaching
    approach_speed: float = 1.0       # m/s horizontal approach speed

    # Search phase
    search_timeout: float = 30.0      # seconds to search before giving up
    search_pattern_radius: float = 3.0  # meters — spiral search radius

    # Align phase
    align_descent_rate: float = 0.3   # m/s descent while aligning
    align_max_offset: float = 0.5     # Must be within this (m) to descend
    align_p_gain: float = 0.8         # Proportional gain for centering
    align_max_speed: float = 0.5      # Max horizontal correction speed (m/s)

    # Final descent phase
    final_descent_rate: float = 0.15  # m/s — very slow final descent
    final_max_offset: float = 0.3     # Must be within this (m) for final
    final_altitude: float = 1.0       # Switch to final below this altitude

    # Touchdown
    touchdown_altitude: float = 0.15  # Disarm when below this (m)

    # Safety
    max_landing_time: float = 60.0    # Total landing timeout (seconds)
    max_tilt_deg: float = 15.0        # Abort if tilt exceeds this


class PrecisionLander:
    """
    Precision landing controller.
    
    Uses target detection + odometry to guide the drone
    onto the exact landing target with velocity commands.
    
    Usage:
        lander = PrecisionLander(bridge, odom, detector)
        success = lander.execute(target_ned_north=50.0, target_ned_east=30.0)
        if success:
            print("Landed on target!")
    """

    def __init__(self, bridge: MAVLinkBridge, odom: RTABMapOdom,
                 detector: TargetDetector, config: LandingConfig = None,
                 flight_config: FlightConfig = None):
        self.bridge = bridge
        self.odom = odom
        self.detector = detector
        self.config = config or LandingConfig()

        # PID flight controller for smooth landing corrections
        self.fc = FlightController(flight_config or FlightConfig.from_yaml())

        self.phase = LandingPhase.IDLE
        self.landing_start_time = 0.0
        self._abort_requested = False
        self._target_lost_since = 0.0
        self._target_lost_timeout = 10.0

    def abort(self):
        """Request landing abort — drone will hold position."""
        self._abort_requested = True
        logger.warning("Landing abort requested!")

    def execute(self, target_north: float, target_east: float,
                target_altitude: float = None) -> bool:
        """
        Execute precision landing sequence.
        
        Args:
            target_north: Target NED north (meters from home)
            target_east: Target NED east (meters from home)
            target_altitude: Target altitude AGL. If None, uses approach_altitude.
            
        Returns:
            True if successfully landed on target
        """
        self._abort_requested = False
        self.landing_start_time = time.time()
        approach_alt = target_altitude or self.config.approach_altitude

        logger.info(f"Precision landing start: target=({target_north:.1f}, {target_east:.1f})")

        try:
            # Phase 1: Approach the waypoint area at altitude
            if not self._phase_approach(target_north, target_east, approach_alt):
                return False

            # Phase 2: Search for landing target
            if not self._phase_search():
                return False

            # Phase 3: Align (center) over target while descending
            if not self._phase_align():
                return False

            # Phase 4: Final descent
            if not self._phase_final_descent():
                return False

            # Phase 5: Touchdown
            self._phase_touchdown()
            return True

        except Exception as e:
            logger.error(f"Landing error: {e}")
            self.phase = LandingPhase.ABORTED
            self._hold_position()
            return False

    # ==============================================================
    # PHASE 1: APPROACH
    # ==============================================================

    def _phase_approach(self, north: float, east: float, alt: float) -> bool:
        """Fly to waypoint area at cruise altitude using PID altitude hold."""
        self.phase = LandingPhase.APPROACH
        self.fc.reset()  # Clean PID state
        logger.info(f"PHASE 1 — APPROACH to ({north:.1f}, {east:.1f}) at {alt:.1f}m")

        while not self._should_abort():
            odom = self.odom.get()
            dist = math.sqrt((odom.x - north)**2 + (odom.y - east)**2)

            if dist < 2.0:
                logger.info(f"Arrived at waypoint area (dist={dist:.2f}m)")
                return True

            # Horizontal: bearing-based approach
            bearing = math.atan2(east - odom.y, north - odom.x)
            speed = min(self.config.approach_speed, dist * 0.5)
            vx = speed * math.cos(bearing)
            vy = speed * math.sin(bearing)

            # Vertical: PID altitude hold
            vz = self.fc.compute_altitude_hold(alt, odom.altitude)

            self._send_velocity(vx, vy, vz)
            time.sleep(0.1)

        return False

    # ==============================================================
    # PHASE 2: SEARCH FOR TARGET
    # ==============================================================

    def _phase_search(self) -> bool:
        """Search for landing target within the waypoint area."""
        self.phase = LandingPhase.SEARCH
        logger.info("PHASE 2 — SEARCHING for landing target...")

        search_start = time.time()

        while not self._should_abort():
            elapsed = time.time() - search_start
            if elapsed > self.config.search_timeout:
                logger.warning("Search timeout! Landing at current position.")
                return True  # Land anyway (fallback)

            det = self.detector.get_detection()
            if det.detected and det.confidence > 0.5:
                logger.info(f"Target FOUND! ID={det.marker_id} "
                             f"dist={det.distance_m:.2f}m conf={det.confidence:.2f}")
                return True

            # Slow spiral search pattern
            angle = elapsed * 0.5  # rad/s spiral
            radius = min(elapsed * 0.3, self.config.search_pattern_radius)
            vx = radius * 0.3 * math.cos(angle)
            vy = radius * 0.3 * math.sin(angle)
            self._send_velocity(vx, vy, 0)

            if int(elapsed) % 5 == 0 and elapsed > 0:
                logger.debug(f"  Searching... {elapsed:.0f}s elapsed")

            time.sleep(0.1)

        return False

    # ==============================================================
    # PHASE 3: ALIGN (CENTER OVER TARGET + DESCEND)
    # ==============================================================

    def _phase_align(self) -> bool:
        """Center over target while slowly descending — PID controlled."""
        self.phase = LandingPhase.ALIGN
        self.fc.reset()  # Clean PID for landing mode
        logger.info("PHASE 3 — ALIGNING over target (PID)")

        while not self._should_abort():
            det = self.detector.get_detection()
            odom = self.odom.get()

            if not det.detected:
                if self._target_lost_since == 0:
                    self._target_lost_since = time.time()
                lost_duration = time.time() - self._target_lost_since
                logger.debug(f"  Target lost during align ({lost_duration:.1f}s) — holding")
                self._send_velocity(0, 0, 0)
                if lost_duration > self._target_lost_timeout:
                    logger.error(f"Target lost for {self._target_lost_timeout}s! Aborting.")
                    self.phase = LandingPhase.ABORTED
                    return False
                time.sleep(0.1)
                continue
            self._target_lost_since = 0.0

            horiz_offset = det.horizontal_offset
            should_descend = horiz_offset < self.config.align_max_offset

            # PID-controlled centering + descent
            vx, vy, vz = self.fc.compute_landing_velocity(
                det.offset_forward, det.offset_right,
                phase="align", should_descend=should_descend
            )

            self._send_velocity(vx, vy, vz)

            logger.debug(f"  Align: fwd={det.offset_forward:.2f}m "
                          f"right={det.offset_right:.2f}m "
                          f"horiz={horiz_offset:.2f}m descend={should_descend}")

            # Check if low enough for final descent
            if (det.offset_down < self.config.final_altitude and
                    horiz_offset < self.config.final_max_offset):
                logger.info(f"Aligned! offset={horiz_offset:.2f}m, alt={det.offset_down:.2f}m")
                return True

            time.sleep(0.05)  # 20 Hz

        return False

    # ==============================================================
    # PHASE 4: FINAL DESCENT
    # ==============================================================

    def _phase_final_descent(self) -> bool:
        """Very slow, PID-controlled final descent."""
        self.phase = LandingPhase.FINAL_DESCENT
        self.fc.reset()  # Fresh PID state for final phase
        logger.info("PHASE 4 — FINAL DESCENT (PID, gentle)")

        while not self._should_abort():
            det = self.detector.get_detection()
            odom = self.odom.get()

            if not det.detected:
                if self._target_lost_since == 0:
                    self._target_lost_since = time.time()
                lost_duration = time.time() - self._target_lost_since
                logger.warning(f"  Target lost during final ({lost_duration:.1f}s)")
                self._send_velocity(0, 0, 0)
                if lost_duration > 5.0:
                    logger.error("Target lost too long in final descent! Aborting!")
                    self.phase = LandingPhase.ABORTED
                    self._hold_position()
                    return False
                time.sleep(0.1)
                continue
            self._target_lost_since = 0.0

            # PID-controlled centering with 'final' phase limits
            vx, vy, vz = self.fc.compute_landing_velocity(
                det.offset_forward, det.offset_right,
                phase="final", should_descend=True
            )

            self._send_velocity(vx, vy, vz)

            # Check for touchdown
            if det.offset_down < self.config.touchdown_altitude:
                logger.info(f"TOUCHDOWN ALTITUDE! alt={det.offset_down:.3f}m")
                return True

            logger.debug(f"  Final: alt={det.offset_down:.2f}m "
                          f"offset=({det.offset_forward:.2f}, {det.offset_right:.2f})")

            time.sleep(0.05)  # 20 Hz

        return False

    # ==============================================================
    # PHASE 5: TOUCHDOWN
    # ==============================================================

    def _phase_touchdown(self):
        """Final touchdown — switch to LAND mode or disarm."""
        self.phase = LandingPhase.TOUCHDOWN
        logger.info("PHASE 5 — TOUCHDOWN")

        # Use ArduPilot's LAND mode for the final few cm
        # It has better ground detection than our code
        self.bridge.land()

        # Wait for disarm
        start = time.time()
        while time.time() - start < 10:
            if not self.bridge.is_armed:
                logger.info("LANDED SUCCESSFULLY!")
                return
            time.sleep(0.2)

        logger.warning("Touchdown timeout — may need manual disarm")

    # ==============================================================
    # VELOCITY COMMAND
    # ==============================================================

    def _send_velocity(self, vx: float, vy: float, vz: float,
                        yaw_rate: float = 0):
        """Send velocity command via bridge (centralized, with NaN guards)."""
        self.bridge.send_velocity_ned(vx, vy, vz, yaw_rate)

    def _hold_position(self):
        """Send zero velocity to hold current position."""
        self._send_velocity(0, 0, 0)

    def _should_abort(self) -> bool:
        """Check if we should abort the landing."""
        if self._abort_requested:
            self.phase = LandingPhase.ABORTED
            self._hold_position()
            return True

        elapsed = time.time() - self.landing_start_time
        if elapsed > self.config.max_landing_time:
            logger.error("Landing timeout!")
            self.phase = LandingPhase.ABORTED
            self._hold_position()
            return True

        # Check tilt (too much tilt = dangerous at low altitude)
        odom = self.odom.get()
        tilt = math.degrees(math.sqrt(odom.roll**2 + odom.pitch**2))
        if tilt > self.config.max_tilt_deg and self.phase >= LandingPhase.FINAL_DESCENT:
            logger.error(f"Excessive tilt: {tilt:.1f}°! Aborting!")
            self.phase = LandingPhase.ABORTED
            self._hold_position()
            return True

        return False
