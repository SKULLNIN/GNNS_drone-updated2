"""
gNNS Drone — Safety & Failsafe Systems
========================================
Monitors drone health and triggers failsafes when issues detected.

Original failsafe triggers (unchanged):
  - VIO/SLAM tracking lost → LAND immediately
  - FC connection lost → LAND
  - Battery low → RTH
  - Geofence breach → RTH or LAND
  - EKF unhealthy → LAND

Extended VIO-state checks (new, via _check_vio_state):
  - Feature starvation      : num_features_tracked < 30 for 3 s → LAND
  - Velocity spike          : speed_3d delta > 10 m/s in one frame → LAND
  - IMU dt spike            : imu_dt > 0.5 s for 3 consecutive frames → LAND
  - Position jump           : position delta > 2 m in one frame → LAND
  - Depth sensor loss       : depth_range_m == -1 for 10 s → DEGRADED warn
  - LiDAR loss              : lidar min_distance == -1 for 5 s → DEGRADED warn
  - Image blur              : mean_flow_magnitude > 60 px/frame → DEGRADED
"""

import time
import math
import threading
import logging
from typing import Optional
from .mavlink_bridge import MAVLinkBridge
from .rtabmap_odom import RTABMapOdom

logger = logging.getLogger("gnns.safety")


class SafetyMonitor:
    """
    Monitors all drone systems and triggers failsafes.
    
    FIXED: Now uses RTABMapOdom (not the old VIOTracker).
    
    Usage:
        safety = SafetyMonitor(bridge, odom)
        safety.set_geofence(radius=50.0)
        safety.start()
        
        # In main loop:
        if not safety.is_safe:
            # Safety system already triggered failsafe
            break
    """

    def __init__(self, bridge: MAVLinkBridge, odom: RTABMapOdom,
                 geofence_radius: float = 100.0,
                 min_battery_pct: float = 20.0,
                 min_odom_confidence: int = 15,
                 max_altitude: float = 10.0,
                 vio_lost_action: str = "LAND",
                 vio_algo=None,
                 lidar=None):
        self.bridge = bridge
        self.odom = odom
        self.geofence_radius = geofence_radius
        self.min_battery_pct = min_battery_pct
        self.min_odom_confidence = min_odom_confidence
        self.max_altitude = max_altitude
        self.vio_lost_action = vio_lost_action

        # Optional references for extended VIO-state checks
        self._vio_algo = vio_algo    # VIOAlgorithm instance (or None)
        self._lidar    = lidar       # LidarFusion instance (or None)

        self._running = False
        self._thread = None
        self._is_safe = True
        self._failsafe_triggered = False
        self._failsafe_reason = ""

        # Tracking quality monitoring
        self._low_confidence_start = 0.0
        self._low_confidence_timeout = 5.0  # seconds of bad odom → failsafe

        # ---- Extended VIO-state failure tracking ---------------------

        # Feature starvation: track < 30 features for > 3 s
        self._feat_starv_start: float = 0.0
        self._FEAT_STARV_THRESH = 30
        self._FEAT_STARV_TIME   = 3.0

        # Velocity spike: |Δspeed| > 10 m/s in one cycle
        self._prev_speed_3d: float = 0.0
        self._VEL_SPIKE_DELTA = 10.0

        # IMU dt spike: imu_dt > 0.5 s for 3 consecutive frames
        self._imu_spike_count: int = 0
        self._IMU_DT_THRESH   = 0.5
        self._IMU_SPIKE_N     = 3

        # Position jump: |Δpos| > 2 m in one cycle
        self._prev_north: float = 0.0
        self._prev_east:  float = 0.0
        self._prev_down:  float = 0.0
        self._POS_JUMP_THRESH = 2.0
        self._pos_init: bool  = False

        # Depth sensor loss: depth_range_m == -1 for > 10 s
        self._depth_loss_start: float = 0.0
        self._DEPTH_LOSS_TIME   = 10.0

        # LiDAR loss: lidar min_distance == -1 for > 5 s
        self._lidar_loss_start: float = 0.0
        self._LIDAR_LOSS_TIME   = 5.0

        # Image blur / excessive flow: mean_flow_magnitude > 60 px/frame
        self._FLOW_BLUR_THRESH  = 60.0

    @property
    def is_safe(self) -> bool:
        return self._is_safe

    @property
    def failsafe_reason(self) -> str:
        return self._failsafe_reason

    def set_geofence(self, radius: float):
        """Set geofence radius in meters from home."""
        self.geofence_radius = radius
        logger.info(f"Geofence set: {radius}m radius")

    def start(self):
        """Start the safety monitoring thread."""
        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop,
                                         daemon=True, name="safety")
        self._thread.start()
        logger.info("Safety monitor started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _trigger_failsafe(self, reason: str, action: str = "LAND"):
        """Trigger a failsafe action."""
        if self._failsafe_triggered:
            return  # Already in failsafe

        self._failsafe_triggered = True
        self._is_safe = False
        self._failsafe_reason = reason
        logger.critical(f"FAILSAFE: {reason} -> {action}")

        try:
            if action == "LAND":
                self.bridge.land()
            elif action == "RTH":
                self.bridge.set_mode("RTL")
            elif action == "DISARM":
                self.bridge.disarm(force=True)
        except Exception as e:
            logger.error(f"Failsafe action '{action}' failed: {e}, emergency disarm")
            try:
                self.bridge.disarm(force=True)
            except Exception:
                pass

    def _monitor_loop(self):
        """Main safety monitoring loop (~10 Hz; battery/EKF/VIO-state at ~2 Hz)."""
        slow_cycle = 0
        while self._running:
            try:
                # Only check safety when armed
                if not self.bridge.is_armed:
                    self._low_confidence_start = 0.0
                    time.sleep(0.1)
                    continue

                slow_cycle += 1
                do_slow = slow_cycle % 5 == 0

                # 1. Check FC connection
                if not self.bridge.is_connected:
                    self._trigger_failsafe("FC connection lost", "LAND")

                # 2. Check odometry staleness
                odom_data = self.odom.get()
                if odom_data.is_stale:
                    self._trigger_failsafe(
                        f"Odometry stale (age={odom_data.age_sec:.1f}s)",
                        self.vio_lost_action
                    )

                # 3. Check odometry/SLAM health
                if odom_data.confidence < self.min_odom_confidence:
                    if self._low_confidence_start == 0:
                        self._low_confidence_start = time.time()
                        logger.warning(f"Odom confidence LOW: {odom_data.confidence}%")
                    elif time.time() - self._low_confidence_start > self._low_confidence_timeout:
                        self._trigger_failsafe(
                            f"SLAM tracking lost (conf={odom_data.confidence}% for "
                            f"{self._low_confidence_timeout}s)",
                            self.vio_lost_action
                        )
                else:
                    # Confidence recovered
                    if self._low_confidence_start > 0:
                        logger.info(f"Odom confidence recovered: {odom_data.confidence}%")
                    self._low_confidence_start = 0.0

                # 4. Check geofence
                dist = math.sqrt(odom_data.x ** 2 + odom_data.y ** 2)
                if dist > self.geofence_radius:
                    self._trigger_failsafe(
                        f"Geofence breach! {dist:.1f}m > {self.geofence_radius}m",
                        "RTH"
                    )

                # 5. Check altitude ceiling
                if odom_data.altitude > self.max_altitude:
                    self._trigger_failsafe(
                        f"Altitude limit! {odom_data.altitude:.1f}m > {self.max_altitude}m",
                        "LAND"
                    )

                # 6–8. Slower checks (battery, EKF, extended VIO state)
                if do_slow:
                    batt = self.bridge.get_latest("BATTERY_STATUS")
                    if batt and hasattr(batt, 'battery_remaining'):
                        if 0 < batt.battery_remaining < self.min_battery_pct:
                            self._trigger_failsafe(
                                f"Low battery: {batt.battery_remaining}%", "RTH"
                            )

                    ekf = self.bridge.get_latest("EKF_STATUS_REPORT")
                    if ekf and hasattr(ekf, 'flags'):
                        const_pos = bool(ekf.flags & 128)
                        if const_pos:
                            self._trigger_failsafe("EKF constant position mode", "LAND")

                    if self._vio_algo is not None:
                        try:
                            vio_state = self._vio_algo.get_last_state()
                            if vio_state is not None:
                                self._check_vio_state(vio_state)
                        except Exception as exc:
                            logger.debug(f"_check_vio_state error: {exc}")

            except Exception as e:
                logger.error(f"Safety monitor error: {e}")

            time.sleep(0.1)  # 10 Hz base rate

    # ==============================================================
    # Extended VIO-state failure checks
    # ==============================================================

    def _check_vio_state(self, state) -> None:
        """
        Run extended failure checks on a VIOState object.

        Called from _monitor_loop when a VIOAlgorithm reference is set.
        All checks are non-fatal unless explicitly noted.

        Args:
            state: VIOState dataclass (from vio_state.py).
        """
        # ----------------------------------------------------------------
        # 1. Feature starvation: < 30 tracked features for 3 seconds
        # ----------------------------------------------------------------
        if state.num_features_tracked < self._FEAT_STARV_THRESH:
            if self._feat_starv_start == 0.0:
                self._feat_starv_start = time.time()
                logger.warning(
                    f"Feature starvation: only {state.num_features_tracked} "
                    f"tracked (< {self._FEAT_STARV_THRESH})"
                )
            elif time.time() - self._feat_starv_start > self._FEAT_STARV_TIME:
                self._trigger_failsafe(
                    f"Feature starvation for >{self._FEAT_STARV_TIME:.0f}s "
                    f"({state.num_features_tracked} features)",
                    "LAND"
                )
        else:
            if self._feat_starv_start > 0.0:
                logger.info("Feature count recovered")
            self._feat_starv_start = 0.0

        # ----------------------------------------------------------------
        # 2. Velocity spike: speed_3d jumped > 10 m/s since last check
        # ----------------------------------------------------------------
        current_speed = state.speed_3d
        delta_speed   = abs(current_speed - self._prev_speed_3d)
        if self._prev_speed_3d > 0 and delta_speed > self._VEL_SPIKE_DELTA:
            self._trigger_failsafe(
                f"Velocity spike: Δspeed={delta_speed:.1f}m/s "
                f"(threshold={self._VEL_SPIKE_DELTA}m/s)",
                "LAND"
            )
        self._prev_speed_3d = current_speed

        # ----------------------------------------------------------------
        # 3. IMU dt spike: imu_dt > 0.5 s for 3 consecutive checks
        # ----------------------------------------------------------------
        if state.imu_dt > self._IMU_DT_THRESH:
            self._imu_spike_count += 1
            logger.warning(
                f"IMU dt spike: {state.imu_dt*1000:.0f}ms "
                f"(count={self._imu_spike_count}/{self._IMU_SPIKE_N})"
            )
            if self._imu_spike_count >= self._IMU_SPIKE_N:
                self._trigger_failsafe(
                    f"IMU dt>{self._IMU_DT_THRESH*1000:.0f}ms for "
                    f"{self._IMU_SPIKE_N} consecutive checks",
                    "LAND"
                )
        else:
            self._imu_spike_count = 0

        # ----------------------------------------------------------------
        # 4. Position jump: > 2 m in one safety cycle
        # ----------------------------------------------------------------
        if self._pos_init:
            dn = state.north - self._prev_north
            de = state.east  - self._prev_east
            dd = state.down  - self._prev_down
            jump = math.sqrt(dn*dn + de*de + dd*dd)
            if jump > self._POS_JUMP_THRESH:
                self._trigger_failsafe(
                    f"Position jump: {jump:.2f}m in one cycle "
                    f"(threshold={self._POS_JUMP_THRESH}m)",
                    "LAND"
                )
        self._prev_north = state.north
        self._prev_east  = state.east
        self._prev_down  = state.down
        self._pos_init   = True

        # ----------------------------------------------------------------
        # 5. Depth sensor loss: depth_range_m == -1 for > 10 s
        # ----------------------------------------------------------------
        if state.depth_range_m < 0:
            if self._depth_loss_start == 0.0:
                self._depth_loss_start = time.time()
            elif time.time() - self._depth_loss_start > self._DEPTH_LOSS_TIME:
                logger.warning(
                    f"Depth sensor offline for >{self._DEPTH_LOSS_TIME:.0f}s — "
                    "VIO scale may drift"
                )
                # Reset timer so warning only appears once every 10 s
                self._depth_loss_start = time.time()
        else:
            self._depth_loss_start = 0.0

        # ----------------------------------------------------------------
        # 6. LiDAR loss: min_distance == -1 for > 5 s
        # ----------------------------------------------------------------
        if self._lidar is not None:
            if self._lidar.get_min_distance() < 0:
                if self._lidar_loss_start == 0.0:
                    self._lidar_loss_start = time.time()
                elif time.time() - self._lidar_loss_start > self._LIDAR_LOSS_TIME:
                    logger.warning(
                        f"LiDAR offline for >{self._LIDAR_LOSS_TIME:.0f}s — "
                        "obstacle avoidance disabled"
                    )
                    self._lidar_loss_start = time.time()
            else:
                self._lidar_loss_start = 0.0

        # ----------------------------------------------------------------
        # 7. Image blur / excessive optical flow
        # ----------------------------------------------------------------
        if state.mean_flow_magnitude > self._FLOW_BLUR_THRESH:
            logger.warning(
                f"Excessive optical flow: {state.mean_flow_magnitude:.1f}px "
                f"(threshold={self._FLOW_BLUR_THRESH}px) — "
                "possible motion blur or vibration"
            )
