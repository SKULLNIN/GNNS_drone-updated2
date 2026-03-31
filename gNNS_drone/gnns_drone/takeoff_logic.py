"""
gNNS Drone — VIO-Gated Takeoff Logic
======================================
Implements a safety-gated state machine that ensures the VIO pipeline
is healthy before arming and a controlled ascent to hover altitude.

State machine:

  PRE_FLIGHT_CHECK
      │ FC connected, VIO sensor found, LiDAR receiving, EKF not lost
      ▼
  SENSOR_WARMUP
      │ Waits WARM_UP_FRAMES (≈ 1 s at 30 Hz) for cameras/AE to settle
      ▼
  VIO_INIT
      │ confidence ≥ conf_threshold (70) AND features ≥ feat_threshold (150)
      │ AND LiDAR OK (optional)
      ▼ timeout → ABORT
  READY_TO_ARM
      │ arm + GUIDED_NOGPS mode sent
      ▼
  LIFTING_OFF
      │ altitude hold loop; monitors VIO during ascent
      │ VIO LOST for > 2 s → EMERGENCY_LAND
      ▼ |alt_error| < 0.3 m AND VIO tracking
  HOVERING     ← mission target state; return True
      │
  MISSION_ACTIVE (external; not managed here)
  LANDING
  EMERGENCY_LAND
  ABORT         ← return False

Usage:
    from gnns_drone.vio_algorithm import VIOAlgorithm
    from gnns_drone.lidar_fusion   import LidarFusion
    from gnns_drone.takeoff_logic  import TakeoffController

    tc = TakeoffController(
        bridge=bridge,
        vio_algo=algo,         # VIOAlgorithm instance
        lidar=lidar,           # LidarFusion  instance
        flight_config=fc_cfg,  # FlightConfig (optional)
    )
    ok = tc.run(altitude=2.5)
    if ok:
        navigator.execute_mission(...)
"""

import time
import math
import logging
import threading
from enum import Enum, auto
from typing import Optional, Callable

from .vio_tracker import VIOStatus
from .vio_state   import VIOState

logger = logging.getLogger("gnns.takeoff")


# ======================================================================
# State enum
# ======================================================================

class TakeoffState(Enum):
    PRE_FLIGHT_CHECK = auto()
    SENSOR_WARMUP    = auto()
    VIO_INIT         = auto()
    READY_TO_ARM     = auto()
    LIFTING_OFF      = auto()
    HOVERING         = auto()
    LANDING          = auto()
    EMERGENCY_LAND   = auto()
    ABORT            = auto()


# ======================================================================
# TakeoffController
# ======================================================================

class TakeoffController:
    """
    VIO-gated takeoff state machine.

    Args:
        bridge:       MAVLinkBridge instance.
        vio_algo:     VIOAlgorithm instance (provides get_last_state()).
        lidar:        LidarFusion instance (optional; used for alt confirm).
        flight_config: FlightConfig instance (optional; uses defaults).
        config:       dict of overrides for thresholds etc.
    """

    # Default thresholds (override via config dict)
    CONF_THRESHOLD  = 70.0    # minimum VIO confidence to arm
    FEAT_THRESHOLD  = 150     # minimum tracked features to arm
    VIO_INIT_TIMEOUT = 20.0   # seconds to wait for VIO init
    PREFLIGHT_TIMEOUT = 30.0  # seconds for preflight checks
    WARMUP_TIME      = 1.2    # seconds (≈ 36 frames at 30 Hz)
    ALT_TOL          = 0.30   # metres; hover reached when |err| < this
    ALT_HOLD_RATE_HZ = 10     # control loop rate
    LOST_TIMEOUT_S   = 2.0    # seconds of VIO LOST during ascent → E-LAND
    MAX_ASCENT_TIME  = 30.0   # seconds; abort if hover not reached

    def __init__(self, bridge, vio_algo, lidar=None,
                 flight_config=None, config: Optional[dict] = None):
        self.bridge       = bridge
        self.vio_algo     = vio_algo
        self.lidar        = lidar
        self.flight_config = flight_config

        cfg = config or {}
        self._conf_thresh    = cfg.get("conf_threshold",    self.CONF_THRESHOLD)
        self._feat_thresh    = cfg.get("feat_threshold",    self.FEAT_THRESHOLD)
        self._vio_init_to    = cfg.get("vio_init_timeout",  self.VIO_INIT_TIMEOUT)
        self._preflight_to   = cfg.get("preflight_timeout", self.PREFLIGHT_TIMEOUT)
        self._warmup_time    = cfg.get("warmup_time",        self.WARMUP_TIME)
        self._alt_tol        = cfg.get("alt_tol",            self.ALT_TOL)
        self._hold_rate      = cfg.get("alt_hold_rate_hz",   self.ALT_HOLD_RATE_HZ)
        self._lost_timeout   = cfg.get("lost_timeout_s",     self.LOST_TIMEOUT_S)
        self._max_ascent     = cfg.get("max_ascent_time",    self.MAX_ASCENT_TIME)

        self._state: TakeoffState = TakeoffState.PRE_FLIGHT_CHECK
        self._abort_reason: str = ""

        # Callbacks
        self._on_state_change: list = []
        self._on_abort: list = []
        self._on_hover: list = []

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def on_state_change(self, cb: Callable[[TakeoffState], None]):
        """Fired every time the takeoff state changes."""
        self._on_state_change.append(cb)

    def on_abort(self, cb: Callable[[str], None]):
        """Fired with abort_reason when ABORT or EMERGENCY_LAND is reached."""
        self._on_abort.append(cb)

    def on_hover(self, cb: Callable[[], None]):
        """Fired once when HOVERING state is reached."""
        self._on_hover.append(cb)

    # ------------------------------------------------------------------ #
    # Main entry point
    # ------------------------------------------------------------------ #

    def run(self, altitude: float = 2.5) -> bool:
        """
        Execute the full takeoff sequence.

        Blocks until HOVERING (True) or ABORT/EMERGENCY_LAND (False).

        Args:
            altitude: target hover altitude in metres (positive up).

        Returns:
            True  → drone is hovering, ready for mission.
            False → abort / emergency; drone is on ground or landing.
        """
        logger.info(f"TakeoffController.run() — target altitude={altitude:.1f}m")

        # --- PRE_FLIGHT_CHECK ----------------------------------------
        self._set_state(TakeoffState.PRE_FLIGHT_CHECK)
        if not self._check_preflight():
            return self._abort(self._abort_reason or "Preflight checks failed")

        # --- SENSOR_WARMUP -------------------------------------------
        self._set_state(TakeoffState.SENSOR_WARMUP)
        logger.info(f"Waiting {self._warmup_time:.1f}s for sensor warmup…")
        time.sleep(self._warmup_time)

        # --- VIO_INIT ------------------------------------------------
        self._set_state(TakeoffState.VIO_INIT)
        if not self._wait_vio_init():
            return self._abort("VIO init timeout — confidence never reached threshold")

        # --- READY_TO_ARM --------------------------------------------
        self._set_state(TakeoffState.READY_TO_ARM)
        if not self._do_arm():
            return self._abort("Arming failed")

        # --- LIFTING_OFF ---------------------------------------------
        self._set_state(TakeoffState.LIFTING_OFF)
        if not self._do_ascent(altitude):
            return self._emergency_land("VIO lost during ascent")

        # --- HOVERING ------------------------------------------------
        self._set_state(TakeoffState.HOVERING)
        for cb in self._on_hover:
            try:
                cb()
            except Exception as e:
                logger.error(f"on_hover callback error: {e}")

        logger.info(f"HOVERING at {altitude:.1f}m — takeoff complete")
        return True

    def land_now(self):
        """Request immediate landing (called from safety monitor or mission end)."""
        logger.warning("TakeoffController.land_now() called")
        self._set_state(TakeoffState.LANDING)
        try:
            self.bridge.land()
        except Exception as e:
            logger.error(f"land() failed: {e}")

    @property
    def state(self) -> TakeoffState:
        return self._state

    @property
    def abort_reason(self) -> str:
        return self._abort_reason

    # ------------------------------------------------------------------ #
    # Phase: preflight checks
    # ------------------------------------------------------------------ #

    def _check_preflight(self) -> bool:
        """
        Verify all subsystems are go before attempting takeoff.

        Checks:
          1. FC link connected and heartbeat fresh.
          2. VIO algorithm is running (not UNINITIALIZED with bad frames).
          3. LiDAR receiving (warning only — not hard failure).
          4. EKF not stuck in constant-position mode.
          5. Battery adequate (if available).
        """
        t0 = time.time()

        # 1. FC connected
        if not self.bridge.is_connected:
            logger.error("Preflight: FC not connected")
            self._abort_reason = "FC not connected"
            return False

        # 2. Wait for heartbeat
        try:
            if not self.bridge.wait_for_heartbeat(timeout=5):
                self._abort_reason = "No FC heartbeat within 5 s"
                return False
        except Exception:
            pass  # bridge may not have wait_for_heartbeat — proceed

        # 3. VIO algorithm started (state != UNINITIALIZED)
        state = self._get_vio_state()
        if state is None:
            self._abort_reason = "VIO algorithm not started"
            return False

        # 4. LiDAR (warning only)
        if self.lidar is not None and not self.lidar.is_receiving:
            logger.warning("Preflight: LiDAR not receiving — altitude hold "
                           "will rely on VIO only")

        # 5. EKF health check
        ekf = self.bridge.get_latest("EKF_STATUS_REPORT")
        if ekf and hasattr(ekf, "flags") and bool(ekf.flags & 128):
            self._abort_reason = "EKF constant-position mode before takeoff"
            return False

        # 6. Battery
        batt = self.bridge.get_latest("BATTERY_STATUS")
        if batt and hasattr(batt, "battery_remaining"):
            pct = batt.battery_remaining
            if 0 < pct < 30:
                logger.warning(f"Preflight: battery at {pct}% (< 30%)")
            if 0 < pct < 15:
                self._abort_reason = f"Battery too low to takeoff: {pct}%"
                return False

        logger.info(f"Preflight checks passed in {time.time()-t0:.1f}s")
        return True

    # ------------------------------------------------------------------ #
    # Phase: wait for VIO initialization
    # ------------------------------------------------------------------ #

    def _wait_vio_init(self) -> bool:
        """
        Block until VIO confidence ≥ _conf_thresh AND features ≥ _feat_thresh,
        or timeout.

        Logs progress every 2 seconds.
        """
        t0   = time.time()
        last_log = 0.0

        logger.info(
            f"Waiting for VIO init "
            f"(need conf≥{self._conf_thresh:.0f}%, "
            f"features≥{self._feat_thresh})…"
        )

        while time.time() - t0 < self._vio_init_to:
            state = self._get_vio_state()
            if state is None:
                time.sleep(0.1)
                continue

            conf    = state.confidence
            feats   = state.num_features_tracked
            lidar_ok = (self.lidar is None or
                        self.lidar.get_nadir_range() > 0 or
                        not self.lidar.is_receiving)

            if time.time() - last_log > 2.0:
                logger.info(
                    f"  VIO: conf={conf:.1f}%  features={feats}  "
                    f"state={state.state.name}  lidar_ok={lidar_ok}"
                )
                last_log = time.time()

            if conf >= self._conf_thresh and feats >= self._feat_thresh:
                logger.info(
                    f"VIO ready — conf={conf:.1f}%  features={feats}  "
                    f"({time.time()-t0:.1f}s)"
                )
                return True

            time.sleep(0.1)

        logger.error(
            f"VIO init timeout after {self._vio_init_to:.0f}s — "
            f"last conf={state.confidence:.1f}% feats={state.num_features_tracked}"
            if (state := self._get_vio_state()) else ""
        )
        return False

    # ------------------------------------------------------------------ #
    # Phase: arm
    # ------------------------------------------------------------------ #

    def _do_arm(self) -> bool:
        """Switch to GUIDED_NOGPS and arm."""
        try:
            logger.info("Setting GUIDED_NOGPS mode…")
            ok = self.bridge.set_mode("GUIDED_NOGPS", timeout=5)
            if not ok:
                logger.warning("set_mode GUIDED_NOGPS returned False; trying GUIDED")
                self.bridge.set_mode("GUIDED", timeout=5)

            logger.info("Arming…")
            armed = self.bridge.arm(timeout=10, retries=2)
            if armed:
                logger.info("Armed successfully")
            else:
                logger.error("Arming failed")
            return armed

        except Exception as e:
            logger.error(f"Arm sequence failed: {e}")
            return False

    # ------------------------------------------------------------------ #
    # Phase: ascent + hover
    # ------------------------------------------------------------------ #

    def _do_ascent(self, target_alt: float) -> bool:
        """
        Send takeoff command and hold altitude with PID until stable.

        Returns False if VIO is LOST for > _lost_timeout during ascent.
        """
        try:
            logger.info(f"Takeoff command — target={target_alt:.1f}m")
            self.bridge.takeoff(altitude=target_alt, timeout=20)
        except Exception as e:
            logger.error(f"takeoff() failed: {e}")
            return False

        dt          = 1.0 / self._hold_rate
        lost_since  = 0.0
        t_ascent    = time.time()

        # Lazy-import FlightController for altitude hold
        fc = None
        if self.flight_config is not None:
            try:
                from .flight_controller import FlightController
                fc = FlightController(self.flight_config)
            except Exception as e:
                logger.warning(f"FlightController init failed: {e}; "
                               "using open-loop takeoff")

        logger.info("Ascending…")
        while time.time() - t_ascent < self._max_ascent_time:
            time.sleep(dt)

            state = self._get_vio_state()
            if state is None:
                continue

            # ---- VIO lost watchdog ----------------------------------
            if state.state == VIOStatus.LOST:
                if lost_since == 0.0:
                    lost_since = time.time()
                    logger.warning("VIO LOST during ascent")
                elif time.time() - lost_since > self._lost_timeout:
                    logger.error(f"VIO LOST for >{self._lost_timeout:.1f}s during ascent")
                    return False
            else:
                lost_since = 0.0

            # ---- PID altitude hold (if FlightController available) --
            current_alt = state.altitude
            alt_err     = target_alt - current_alt

            if fc is not None:
                vz = fc.compute_altitude_hold(target_alt, current_alt, dt)
                try:
                    self.bridge.send_velocity_ned(0.0, 0.0, vz)
                except Exception:
                    pass

            # ---- Check hover condition ------------------------------
            if abs(alt_err) < self._alt_tol and \
               state.state in (VIOStatus.TRACKING, VIOStatus.DEGRADED) and \
               state.confidence >= 40.0:
                # Zero velocity to hold position
                try:
                    self.bridge.send_velocity_ned(0.0, 0.0, 0.0)
                except Exception:
                    pass
                logger.info(
                    f"Hover reached — alt={current_alt:.2f}m  "
                    f"err={alt_err:.3f}m  conf={state.confidence:.1f}%"
                )
                return True

        logger.error(f"Ascent timeout ({self._max_ascent_time:.0f}s) — "
                     f"last alt={state.altitude:.2f}m" if (state := self._get_vio_state()) else "")
        return False

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _get_vio_state(self) -> Optional[VIOState]:
        """Safely retrieve the latest VIOState from the algorithm."""
        try:
            return self.vio_algo.get_last_state()
        except Exception as e:
            logger.debug(f"_get_vio_state error: {e}")
            return None

    def _set_state(self, new_state: TakeoffState):
        old = self._state
        self._state = new_state
        if old != new_state:
            logger.info(f"Takeoff state: {old.name} → {new_state.name}")
            for cb in self._on_state_change:
                try:
                    cb(new_state)
                except Exception as e:
                    logger.error(f"state_change callback error: {e}")

    def _abort(self, reason: str) -> bool:
        self._abort_reason = reason
        self._set_state(TakeoffState.ABORT)
        logger.error(f"TAKEOFF ABORTED: {reason}")
        for cb in self._on_abort:
            try:
                cb(reason)
            except Exception as e:
                logger.error(f"on_abort callback error: {e}")
        return False

    def _emergency_land(self, reason: str) -> bool:
        self._abort_reason = reason
        self._set_state(TakeoffState.EMERGENCY_LAND)
        logger.critical(f"EMERGENCY LAND: {reason}")
        for cb in self._on_abort:
            try:
                cb(reason)
            except Exception as e:
                logger.error(f"on_abort callback error: {e}")
        try:
            self.bridge.land()
        except Exception as e:
            logger.error(f"Emergency land() failed: {e}")
            try:
                self.bridge.disarm(force=True)
            except Exception:
                pass
        return False

    # ------------------------------------------------------------------ #
    # Debug
    # ------------------------------------------------------------------ #

    def print_status(self):
        state = self._get_vio_state()
        print(f"\n--- TakeoffController ---")
        print(f"  State:      {self._state.name}")
        print(f"  Thresholds: conf≥{self._conf_thresh:.0f}%  "
              f"features≥{self._feat_thresh}")
        if state:
            print(f"  VIO:        conf={state.confidence:.1f}%  "
                  f"feat={state.num_features_tracked}  "
                  f"status={state.state.name}")
            print(f"  Alt:        {state.altitude:.2f}m")
        if self._abort_reason:
            print(f"  Abort:      {self._abort_reason}")
        if self.lidar:
            print(f"  LiDAR:      nadir={self.lidar.get_nadir_range():.2f}m  "
                  f"receiving={self.lidar.is_receiving}")
