"""
gNNS Drone — Diagnostics & Parameter Validator
================================================
Pre-flight diagnostic tool that validates:
  - MAVLink connection to FC
  - ArduPilot parameter configuration
  - EKF3 health and convergence
  - VIO sensor connectivity
  - LiDAR sensor connectivity
  - Link quality and latency

Run this FIRST before every flight!
"""

import time
import sys
import logging
from .mavlink_bridge import MAVLinkBridge

logger = logging.getLogger("gnns.diag")


class DiagResult:
    """Result of a single diagnostic check."""
    def __init__(self, name: str, passed: bool, message: str, 
                 value=None, expected=None):
        self.name = name
        self.passed = passed
        self.message = message
        self.value = value
        self.expected = expected

    def __repr__(self):
        icon = "PASS" if self.passed else "FAIL"
        s = f"  [{icon}] {self.name}: {self.message}"
        if self.value is not None and self.expected is not None:
            s += f" (got={self.value}, expected={self.expected})"
        return s


class Diagnostics:
    """
    Pre-flight diagnostic system.
    
    Usage:
        diag = Diagnostics()
        results = diag.run_all()
        if diag.all_passed:
            print("Ready to fly!")
        else:
            print("Fix issues before flight!")
    """

    def __init__(self, bridge: MAVLinkBridge = None, config_path: str = None):
        self.bridge = bridge
        self.results: list[DiagResult] = []
        self.config_path = config_path

    @property
    def all_passed(self) -> bool:
        return all(r.passed for r in self.results)

    @property
    def fail_count(self) -> int:
        return sum(1 for r in self.results if not r.passed)

    def _add(self, name, passed, message, value=None, expected=None):
        r = DiagResult(name, passed, message, value, expected)
        self.results.append(r)
        return r

    # ==============================================================
    # CHECK: FC CONNECTION
    # ==============================================================

    def check_connection(self) -> bool:
        """Verify MAVLink connection to flight controller."""
        print("\n--- Checking FC Connection ---")

        if self.bridge is None:
            self.bridge = MAVLinkBridge(self.config_path)

        if not self.bridge.conn:
            if not self.bridge.connect():
                self._add("FC Connect", False, "Cannot open serial port")
                return False
            self._add("FC Connect", True, "Serial port opened")

        if not self.bridge.wait_for_heartbeat(timeout=10.0):
            self._add("FC Heartbeat", False,
                       "No heartbeat — check wiring, baud rate, SERIAL2_PROTOCOL=2")
            return False
        self._add("FC Heartbeat", True, "Heartbeat received")

        return True

    # ==============================================================
    # CHECK: ARDUPILOT PARAMETERS
    # ==============================================================

    def check_params(self) -> bool:
        """Validate critical ArduPilot parameters."""
        print("\n--- Checking ArduPilot Parameters ---")
        all_ok = True

        # Critical parameters and their expected values
        checks = [
            # (param_name, expected_value, description, tolerance)
            ("SERIAL2_PROTOCOL", 2, "TELEM2 must be MAVLink2", 0),
            ("GPS1_TYPE", 0, "GPS must be disabled", 0),
            ("EK3_ENABLE", 1, "EKF3 must be enabled", 0),
            ("EK2_ENABLE", 0, "EKF2 must be disabled", 0),
            ("AHRS_EKF_TYPE", 3, "AHRS must use EKF3", 0),
            ("EK3_SRC1_POSXY", 6, "Position source must be ExternalNav", 0),
            ("EK3_SRC1_VELXY", 6, "Velocity source must be ExternalNav", 0),
            ("EK3_SRC1_YAW", 6, "Yaw source must be ExternalNav", 0),
            ("VISO_TYPE", 1, "Vision must be MAVLink type", 0),
        ]

        for param, expected, desc, tol in checks:
            value = self.bridge.get_param(param)
            if value is None:
                self._add(f"Param {param}", False,
                           f"Cannot read — param might not exist")
                all_ok = False
            elif abs(value - expected) <= tol:
                self._add(f"Param {param}", True, desc, value, expected)
            else:
                self._add(f"Param {param}", False,
                           f"{desc} — WRONG VALUE!", value, expected)
                all_ok = False

        # Check arming — should skip GPS check
        arming = self.bridge.get_param("ARMING_CHECK")
        if arming is not None:
            # Bit 4 (value 16) is GPS check. If ARMING_CHECK is -17,
            # it means "all checks except GPS"
            gps_check_disabled = (int(arming) & 16) == 0 or arming < 0
            if not gps_check_disabled and arming != 0:
                self._add("Param ARMING_CHECK", False,
                           "GPS arming check should be disabled",
                           int(arming), "-17 (skip GPS)")
                all_ok = False
            else:
                self._add("Param ARMING_CHECK", True,
                           "GPS arming check disabled", int(arming))

        return all_ok

    # ==============================================================
    # CHECK: EKF3 STATUS
    # ==============================================================

    def check_ekf(self) -> bool:
        """Check EKF3 health and convergence."""
        print("\n--- Checking EKF3 Status ---")
        
        # Request EKF status
        self.bridge.configure_message_rates()
        time.sleep(2)  # Wait for messages to arrive

        ekf = self.bridge.get_latest("EKF_STATUS_REPORT")
        if ekf is None:
            self._add("EKF Status", False,
                       "No EKF_STATUS_REPORT received — send VIO data first!")
            return False

        # Check EKF flags
        # EKF_STATUS_REPORT flags:
        #   bit 0: attitude OK
        #   bit 1: velocity horiz OK
        #   bit 2: velocity vert OK  
        #   bit 3: pos horiz rel OK
        #   bit 4: pos horiz abs OK
        #   bit 5: pos vert abs OK
        #   bit 6: pos vert agl OK
        #   bit 7: const pos mode (bad!)
        #   bit 8: pred pos horiz rel OK
        #   bit 9: pred pos horiz abs OK
        flags = ekf.flags if hasattr(ekf, 'flags') else 0

        attitude_ok = bool(flags & 1)
        self._add("EKF Attitude", attitude_ok,
                   "Attitude estimation" + (" healthy" if attitude_ok else " NOT converged"))

        vel_ok = bool(flags & 2)
        self._add("EKF Velocity", vel_ok,
                   "Velocity estimation" + (" healthy" if vel_ok else " NOT converged"))

        pos_ok = bool(flags & 8)
        self._add("EKF Position", pos_ok,
                   "Position estimation" + (" healthy" if pos_ok else " NOT converged — send VIO data!"))

        const_pos = bool(flags & 128)
        self._add("EKF Const Pos", not const_pos,
                   "Constant position mode" + (" (BAD — EKF has no position source!)" if const_pos else " not active (good)"))

        # Check variances
        vel_var = ekf.velocity_variance if hasattr(ekf, 'velocity_variance') else -1
        pos_var = ekf.pos_horiz_variance if hasattr(ekf, 'pos_horiz_variance') else -1

        if vel_var >= 0:
            ok = vel_var < 1.0
            self._add("EKF Vel Variance", ok,
                       f"Velocity variance={vel_var:.3f}" +
                       (" (good)" if ok else " (HIGH — check VIO quality!)"))

        if pos_var >= 0:
            ok = pos_var < 1.0
            self._add("EKF Pos Variance", ok,
                       f"Position variance={pos_var:.3f}" +
                       (" (good)" if ok else " (HIGH — check VIO quality!)"))

        return attitude_ok and pos_ok and not const_pos

    # ==============================================================
    # CHECK: VIO SENSOR
    # ==============================================================

    def check_vio_sensor(self) -> bool:
        """Check if VIO camera is connected."""
        print("\n--- Checking VIO Sensor ---")

        try:
            import pyrealsense2 as rs
            self._add("pyrealsense2", True, "Library installed")
        except ImportError:
            self._add("pyrealsense2", False,
                       "NOT installed! Run: pip install pyrealsense2")
            return False

        # Check for connected devices
        ctx = rs.context()
        devices = ctx.query_devices()

        if len(devices) == 0:
            self._add("RealSense Device", False,
                       "No RealSense camera detected — check USB connection")
            return False

        for dev in devices:
            name = dev.get_info(rs.camera_info.name)
            serial = dev.get_info(rs.camera_info.serial_number)
            fw = dev.get_info(rs.camera_info.firmware_version)
            self._add(f"RealSense {name}", True,
                       f"Connected (S/N: {serial}, FW: {fw})")

        return True

    # ==============================================================
    # CHECK: LIDAR
    # ==============================================================

    def check_lidar(self) -> bool:
        """Check if LiDAR is connected."""
        print("\n--- Checking LiDAR ---")

        try:
            from rplidar import RPLidar
            self._add("rplidar lib", True, "Library installed")
        except ImportError:
            self._add("rplidar lib", False,
                       "NOT installed! Run: pip install rplidar-roboticia")
            return False

        # Try to connect
        try:
            lidar = RPLidar('/dev/ttyUSB0')
            info = lidar.get_info()
            health = lidar.get_health()
            self._add("LiDAR Connect", True,
                       f"Connected — Model: {info.get('model', '?')}, "
                       f"Health: {health[0]}")
            lidar.disconnect()
            return True
        except Exception as e:
            self._add("LiDAR Connect", False,
                       f"Cannot connect: {e} — check USB port")
            return False

    # ==============================================================
    # CHECK: LINK QUALITY
    # ==============================================================

    def check_link_quality(self) -> bool:
        """Test MAVLink link quality."""
        print("\n--- Checking Link Quality ---")

        # Measure latency via TIMESYNC
        start = time.time()
        self.bridge.conn.mav.timesync_send(0, int(time.time() * 1e9))
        
        msg = self.bridge.conn.recv_match(type='TIMESYNC', blocking=True,
                                           timeout=3.0)
        if msg:
            latency = (time.time() - start) * 1000
            ok = latency < 50
            self._add("Link Latency", ok,
                       f"{latency:.1f}ms" + (" (good)" if ok else " (HIGH!)"))
        else:
            self._add("Link Latency", False, "TIMESYNC timeout")

        # Check message rate
        count_before = self.bridge.stats.messages_received
        time.sleep(2.0)
        count_after = self.bridge.stats.messages_received
        rate = (count_after - count_before) / 2.0

        ok = rate > 5
        self._add("Message Rate", ok,
                   f"{rate:.0f} msg/s from FC" + 
                   ("" if ok else " (LOW — check baud rate!)"))

        return True

    # ==============================================================
    # RUN ALL CHECKS
    # ==============================================================

    def run_all(self) -> list[DiagResult]:
        """Run all diagnostic checks."""
        self.results = []
        print("\n" + "=" * 60)
        print("  gNNS DRONE — PRE-FLIGHT DIAGNOSTICS")
        print("=" * 60)

        # 1. Connection
        if not self.check_connection():
            self.print_summary()
            return self.results

        # 2. Parameters
        self.check_params()

        # 3. EKF
        self.check_ekf()

        # 4. VIO sensor
        self.check_vio_sensor()

        # 5. LiDAR
        self.check_lidar()

        # 6. Link quality
        self.check_link_quality()

        self.print_summary()
        return self.results

    def print_summary(self):
        """Print diagnostic results summary."""
        print("\n" + "=" * 60)
        print("  DIAGNOSTIC RESULTS")
        print("=" * 60)
        for r in self.results:
            print(r)
        print("-" * 60)
        passed = sum(1 for r in self.results if r.passed)
        total = len(self.results)
        fails = self.fail_count
        print(f"  {passed}/{total} checks passed, {fails} failed")

        if self.all_passed:
            print("\n  *** ALL CHECKS PASSED — READY TO FLY! ***")
        else:
            print("\n  *** FIX FAILURES BEFORE FLIGHT! ***")
            for r in self.results:
                if not r.passed:
                    print(f"    -> {r.name}: {r.message}")
        print("=" * 60)


# ==============================================================
# ENTRY POINT
# ==============================================================

def main():
    """Run diagnostics from command line."""
    import argparse
    parser = argparse.ArgumentParser(description="gNNS Drone Pre-Flight Diagnostics")
    parser.add_argument("--config", default=None, help="Path to mavlink_config.yaml")
    parser.add_argument("--port", default=None, help="Override connection port")
    args = parser.parse_args()

    diag = Diagnostics(config_path=args.config)
    diag.run_all()

    sys.exit(0 if diag.all_passed else 1)


if __name__ == "__main__":
    main()
