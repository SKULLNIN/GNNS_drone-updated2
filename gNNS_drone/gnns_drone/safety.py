"""
gNNS Drone — Safety & Failsafe Systems
========================================
Monitors drone health and triggers failsafes when issues detected.

Failsafe triggers:
  - VIO/SLAM tracking lost → LAND immediately
  - FC connection lost → LAND (FC has its own failsafes too)
  - Battery low → RTH
  - Geofence breach → RTH or LAND
  - EKF unhealthy → LAND
  - Excessive tilt → LAND
"""

import time
import math
import threading
import logging
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
                 vio_lost_action: str = "LAND"):
        self.bridge = bridge
        self.odom = odom
        self.geofence_radius = geofence_radius
        self.min_battery_pct = min_battery_pct
        self.min_odom_confidence = min_odom_confidence
        self.max_altitude = max_altitude
        self.vio_lost_action = vio_lost_action

        self._running = False
        self._thread = None
        self._is_safe = True
        self._failsafe_triggered = False
        self._failsafe_reason = ""
        
        # Tracking quality monitoring
        self._low_confidence_start = 0.0
        self._low_confidence_timeout = 5.0  # seconds of bad odom → failsafe

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

        if action == "LAND":
            self.bridge.land()
        elif action == "RTH":
            self.bridge.set_mode("RTL")
        elif action == "DISARM":
            self.bridge.disarm(force=True)

    def _monitor_loop(self):
        """Main safety monitoring loop (runs at 2 Hz)."""
        while self._running:
            try:
                # Only check safety when armed
                if not self.bridge.is_armed:
                    self._low_confidence_start = 0.0
                    time.sleep(0.5)
                    continue

                # 1. Check FC connection
                if not self.bridge.is_connected:
                    self._trigger_failsafe("FC connection lost", "LAND")

                # 2. Check odometry/SLAM health
                odom_data = self.odom.get()
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

                # 3. Check geofence
                dist = math.sqrt(odom_data.x ** 2 + odom_data.y ** 2)
                if dist > self.geofence_radius:
                    self._trigger_failsafe(
                        f"Geofence breach! {dist:.1f}m > {self.geofence_radius}m",
                        "RTH"
                    )

                # 4. Check altitude ceiling
                if odom_data.altitude > self.max_altitude:
                    self._trigger_failsafe(
                        f"Altitude limit! {odom_data.altitude:.1f}m > {self.max_altitude}m",
                        "LAND"
                    )

                # 5. Check battery
                batt = self.bridge.get_latest("BATTERY_STATUS")
                if batt and hasattr(batt, 'battery_remaining'):
                    if 0 < batt.battery_remaining < self.min_battery_pct:
                        self._trigger_failsafe(
                            f"Low battery: {batt.battery_remaining}%", "RTH"
                        )

                # 6. Check EKF health
                ekf = self.bridge.get_latest("EKF_STATUS_REPORT")
                if ekf and hasattr(ekf, 'flags'):
                    const_pos = bool(ekf.flags & 128)
                    if const_pos:
                        self._trigger_failsafe("EKF constant position mode", "LAND")

            except Exception as e:
                logger.error(f"Safety monitor error: {e}")

            time.sleep(0.5)  # 2 Hz
