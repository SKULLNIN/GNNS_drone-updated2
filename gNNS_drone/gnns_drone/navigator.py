"""
gNNS Drone — Navigator (PID + Smooth Flight)
===============================================
Waypoint navigation controller with per-axis PID control
and acceleration-limited velocity profiles.

Ties together:
  - FlightController (PID + velocity smoothing)
  - RTAB-Map odometry (position + velocity)
  - Target detection (ArUco / color pad)
  - Precision landing (5-phase smooth landing)
  - MAVLink bridge (FC communication)

Improvements over v1:
  - Per-axis PID replaces single-gain P-controller
  - Acceleration limiting prevents jerky movements
  - Proper altitude hold with PID (not vz=0)
  - Velocity ramp-down near waypoints
  - Arm verification before takeoff
  - Smooth takeoff with velocity ramp
"""

import time
import math
import threading
import logging
from typing import Optional
from .mavlink_bridge import MAVLinkBridge
from .rtabmap_odom import RTABMapOdom, OdomData
from .target_detector import TargetDetector, TargetDetection
from .precision_landing import PrecisionLander, LandingConfig
from .flight_controller import FlightController, FlightConfig
from .coordinate_utils import GPSCoord, NEDCoord, WaypointManager

logger = logging.getLogger("gnns.navigator")


class Navigator:
    """
    High-level navigation controller with PID-based flight control.
    
    Orchestrates:
      1. Odometry → FC forwarding (30 Hz)
      2. PID-based waypoint-to-waypoint flight
      3. Target search at each waypoint
      4. Precision landing on target (5-phase)
      5. Smooth takeoff after landing
    """

    def __init__(self, bridge: MAVLinkBridge, odom: RTABMapOdom,
                 detector: TargetDetector, flight_config_path: Optional[str] = None):
        self.bridge = bridge
        self.odom = odom
        self.detector = detector

        # Load flight config from YAML
        self.fc = FlightController(FlightConfig.from_yaml(flight_config_path))
        self.config = self.fc.config

        # Precision lander (uses its own PID from FlightConfig)
        landing_cfg = LandingConfig(
            approach_altitude=self.config.approach_altitude,
            approach_speed=self.config.approach_max_speed,
            search_timeout=self.config.search_timeout,
            align_descent_rate=self.config.align_descent_rate,
            align_max_offset=self.config.align_center_tol,
            align_p_gain=self.config.align_gains.kp,
            align_max_speed=self.config.landing_horiz_speed,
            final_descent_rate=self.config.final_descent_rate,
            final_max_offset=self.config.final_center_tol,
            final_altitude=1.0,
            touchdown_altitude=self.config.touchdown_altitude,
            max_landing_time=self.config.max_landing_time,
            max_tilt_deg=self.config.max_tilt_deg,
        )
        self.lander = PrecisionLander(bridge, odom, detector, landing_cfg)

        self.waypoints: WaypointManager = None
        self.current_wp_index = 0
        self.waypoints_completed = 0

        # Odom → FC forwarding
        self._fwd_running = False
        self._fwd_thread = None
        self._fwd_rate = 30  # Hz

        # Mission state
        self._mission_active = False

        # Odom confidence hysteresis (avoid toggling velocity fusion at EKF)
        self._conf_hold_count = 0
        self._CONF_HOLD_CYCLES = 3

    def set_waypoints(self, wm: WaypointManager):
        self.waypoints = wm

    # ==============================================================
    # ODOMETRY → FC FORWARDING
    # ==============================================================

    def start_odom_forwarding(self):
        """Start sending odometry to FC as VISION_POSITION_ESTIMATE."""
        self._fwd_running = True
        self._fwd_thread = threading.Thread(
            target=self._odom_forward_loop, daemon=True, name="odom-fwd"
        )
        self._fwd_thread.start()
        logger.info(f"Odom→FC forwarding started at {self._fwd_rate} Hz")

    def stop_odom_forwarding(self):
        self._fwd_running = False
        if self._fwd_thread:
            self._fwd_thread.join(timeout=2.0)

    def _vision_pos_variance_m2(self, data: OdomData) -> float:
        """Position variance (m^2) for VISION_POSITION_ESTIMATE covariance."""
        if data.covariance_pos > 0:
            return max(0.05, float(data.covariance_pos) ** 2)
        return max(0.05, (100 - data.confidence) * 0.01)

    def _odom_forward_loop(self):
        """Send RTAB-Map odometry to FC at 30 Hz."""
        interval = 1.0 / self._fwd_rate
        min_confidence = self.config.min_odom_confidence

        while self._fwd_running:
            data = self.odom.get()

            if data.is_stale:
                logger.warning(f"Odom stale (age={data.age_sec:.1f}s), skipping FC send")
            else:
                if data.confidence >= min_confidence:
                    self._conf_hold_count = self._CONF_HOLD_CYCLES
                elif self._conf_hold_count > 0:
                    self._conf_hold_count -= 1

                send_full = (
                    data.confidence >= min_confidence or self._conf_hold_count > 0
                )
                send_pos_only = (not send_full) and data.confidence > 10

                pos_var = self._vision_pos_variance_m2(data)

                if send_full:
                    self.bridge.send_vision_position(
                        data.x, data.y, data.z,
                        data.roll, data.pitch, data.yaw,
                        pos_variance_m2=pos_var,
                    )
                    self.bridge.send_vision_speed(data.vx, data.vy, data.vz)
                elif send_pos_only:
                    self.bridge.send_vision_position(
                        data.x, data.y, data.z,
                        data.roll, data.pitch, data.yaw,
                        pos_variance_m2=pos_var,
                    )

            time.sleep(interval)

    # ==============================================================
    # MISSION EXECUTION
    # ==============================================================

    def execute_mission(self, flight_alt: Optional[float] = None) -> bool:
        """
        Execute the full 5-waypoint mission with precision landing.
        
        Returns True if all waypoints visited.
        """
        if flight_alt:
            self.config.cruise_altitude = flight_alt

        if not self.waypoints or self.waypoints.count == 0:
            logger.error("No waypoints loaded!")
            return False

        self._mission_active = True

        logger.info(f"\n{'='*60}")
        logger.info(f"  STARTING MISSION: {self.waypoints.count} waypoints")
        logger.info(f"  Cruise altitude: {self.config.cruise_altitude}m")
        logger.info(f"  PID gains: H_kp={self.config.horizontal_gains.kp}, "
                     f"V_kp={self.config.vertical_gains.kp}")
        logger.info(f"{'='*60}\n")

        for i in range(self.waypoints.count):
            if not self._mission_active:
                logger.warning("Mission aborted!")
                return False

            self.current_wp_index = i
            ned = self.waypoints.get_waypoint_ned(i)
            gps = self.waypoints.get_waypoint_gps(i)

            logger.info(f"\n{'='*50}")
            logger.info(f"  WAYPOINT {i+1}/{self.waypoints.count}")
            logger.info(f"  GPS: {gps}")
            logger.info(f"  NED: N={ned.north:.1f}m E={ned.east:.1f}m")
            logger.info(f"{'='*50}")

            # --- FLY TO WAYPOINT AREA (PID controlled) ---
            self.fc.reset()  # Reset PID state for new waypoint
            if not self._fly_to_waypoint(ned):
                logger.error(f"Failed to reach WP{i+1} area!")
                return False

            # --- PRECISION LANDING ---
            logger.info(f"Starting precision landing at WP{i+1}...")
            landed = self.lander.execute(ned.north, ned.east)

            if landed:
                self.waypoints_completed += 1
                self._log_landing_result(i, ned)
            else:
                logger.warning(f"WP{i+1}: Precision landing failed, doing normal land")
                self.bridge.land()
                self.bridge.wait_landed(timeout=30)
                self.waypoints_completed += 1

            # --- WAIT ON GROUND ---
            logger.info("Waiting 3s on ground...")
            time.sleep(3.0)

            # --- TAKEOFF FOR NEXT WP ---
            if i < self.waypoints.count - 1:
                logger.info("Taking off for next waypoint...")
                if not self._smooth_takeoff():
                    logger.error("Takeoff failed!")
                    return False

        # --- RETURN TO HOME ---
        logger.info(f"\n{'='*50}")
        logger.info("  RETURNING TO HOME")
        logger.info(f"{'='*50}")

        if not self._smooth_takeoff():
            return False

        self.fc.reset()
        home_ned = NEDCoord(0, 0, 0)
        if not self._fly_to_waypoint(home_ned):
            logger.warning("RTH navigation failed, landing at current position")
            self.bridge.land()

        self.lander.execute(0, 0)

        logger.info(f"\n  MISSION COMPLETE!")
        logger.info(f"  Waypoints: {self.waypoints_completed}/{self.waypoints.count}")
        self._mission_active = False
        return True

    def abort_mission(self):
        """Abort the mission — hold position."""
        self._mission_active = False
        self.bridge.send_velocity_ned(0, 0, 0)
        logger.warning("Mission ABORTED!")

    # ==============================================================
    # FLY TO WAYPOINT (PID-BASED)
    # ==============================================================

    def _fly_to_waypoint(self, target: NEDCoord, timeout: float = 120) -> bool:
        """
        Fly to a waypoint using PID control with acceleration limiting.
        
        Improvements:
          - Per-axis PID (not single Kp*error)
          - Velocity ramp-down near target (smooth deceleration)
          - Acceleration limiting (no sudden velocity jumps)
          - Proper altitude hold with PID
        """
        logger.info(f"Flying to ({target.north:.1f}, {target.east:.1f})...")
        start = time.time()
        last_log = 0

        while time.time() - start < timeout:
            if not self._mission_active:
                return False

            data = self.odom.get()

            # Check arrival
            distance = math.sqrt(
                (target.north - data.x) ** 2 + (target.east - data.y) ** 2
            )

            if distance < self.config.arrival_radius:
                logger.info(f"Reached waypoint area (dist={distance:.2f}m)")
                for _ in range(10):
                    data = self.odom.get()
                    result = self.fc.compute_waypoint_velocity(
                        target.north, target.east, self.config.cruise_altitude,
                        data.x, data.y, data.altitude
                    )
                    vx, vy, vz = result[0], result[1], result[2]
                    self.bridge.send_velocity_ned(vx * 0.5, vy * 0.5, vz)
                    time.sleep(0.05)
                self.bridge.send_velocity_ned(0, 0, 0)
                return True

            # PID + acceleration-limited velocity command
            result = self.fc.compute_waypoint_velocity(
                target.north, target.east, self.config.cruise_altitude,
                data.x, data.y, data.altitude
            )
            vx, vy, vz = result[0], result[1], result[2]
            yaw = result[3] if len(result) > 3 else 0.0

            self.bridge.send_velocity_ned_yaw(vx, vy, vz, yaw)

            # Rate-limited logging (every 3 seconds)
            now = time.time()
            if now - last_log > 3.0:
                speed = math.sqrt(vx ** 2 + vy ** 2)
                logger.debug(
                    f"  dist={distance:.1f}m cmd_speed={speed:.2f}m/s "
                    f"alt={data.altitude:.1f}m odom_conf={data.confidence}%"
                )
                last_log = now

            time.sleep(0.05)  # 20 Hz control loop

        logger.warning("Fly-to-waypoint timeout!")
        return False

    # ==============================================================
    # SMOOTH TAKEOFF
    # ==============================================================

    def _smooth_takeoff(self, altitude: Optional[float] = None) -> bool:
        """
        Arm and takeoff with smooth velocity ramp.
        
        Instead of instant takeoff command, we:
          1. Set GUIDED mode
          2. Arm (with verification)
          3. Use ArduPilot takeoff command
          4. Monitor altitude with logging
          5. Stabilize at altitude before proceeding
          
        Inspired by autonomy_core take_off_tracker:
          thrust_rate: 5.0 (they ramp thrust)
          We ramp via ArduPilot's built-in takeoff + monitoring
        """
        alt = altitude or self.config.takeoff_altitude
        logger.info(f"Smooth takeoff to {alt:.1f}m...")

        # Step 1: GUIDED mode
        if not self.bridge.set_mode("GUIDED"):
            logger.error("Failed to set GUIDED mode!")
            return False
        time.sleep(0.5)

        # Step 2: Arm with verification
        if not self.bridge.arm():
            logger.error("ARM FAILED!")
            return False
        time.sleep(self.config.takeoff_ramp_time)

        # Verify armed
        if not self.bridge.is_armed:
            logger.error("Not armed after arm command!")
            return False

        # Step 3: Takeoff
        result = self.bridge.takeoff(alt, timeout=15.0)

        # Step 4: Stabilize — hold position for a moment
        logger.info(f"Stabilizing at {alt:.1f}m...")
        stable_start = time.time()
        while time.time() - stable_start < self.config.takeoff_stabilize_time:
            # Send zero velocity to hold position at altitude
            data = self.odom.get()
            vz = self.fc.compute_altitude_hold(alt, data.altitude)
            self.bridge.send_velocity_ned(0, 0, vz)
            time.sleep(0.1)

        logger.info(f"Takeoff complete, altitude: {self.odom.get().altitude:.1f}m")
        return True

    # ==============================================================
    # LOGGING
    # ==============================================================

    def _log_landing_result(self, wp_index, target_ned):
        """Log precision landing accuracy."""
        data = self.odom.get()
        err_n = data.x - target_ned.north
        err_e = data.y - target_ned.east
        horiz_error = math.sqrt(err_n ** 2 + err_e ** 2)

        det = self.detector.get_detection()

        logger.info(f"  WP{wp_index+1} LANDED")
        logger.info(f"    Position error: {horiz_error:.3f}m "
                     f"(N={err_n:.3f}m, E={err_e:.3f}m)")
        logger.info(f"    Landing vel: vx={data.vx:.3f} vy={data.vy:.3f} vz={data.vz:.3f}")
        logger.info(f"    Target: detected={det.detected}, conf={det.confidence:.2f}")
        logger.info(f"    Odom confidence: {data.confidence}%")
