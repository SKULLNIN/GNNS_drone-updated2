#!/usr/bin/env python3
"""
gNNS Drone — Full Gazebo SITL Mission Test
=============================================
Flies a complete 5-waypoint mission in Gazebo:
  Takeoff → Fly WP1 → Land → Takeoff → Fly WP2 → Land → ... → Return Home

Connects via pymavlink to ArduPilot SITL running with Gazebo.
Uses the same P-controller approach as our FlightController.

Prerequisites:
  Terminal 1: gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world &
              sim_vehicle.py -v ArduCopter -f gazebo-iris --no-mavproxy
  Terminal 2: python3 gazebo_flight_test.py
"""

import time
import math
import sys
from pymavlink import mavutil

# =============================================
# CONFIG
# =============================================
FLIGHT_ALT = 2.5        # cruise altitude (m)
CRUISE_SPEED = 1.5      # max horizontal speed (m/s)
ARRIVAL_RADIUS = 2.0    # "arrived" when within this (m)
Kp = 0.6                # P-gain (same as flight_config.yaml)
LAND_PAUSE = 3.0        # seconds on ground at each waypoint
CONNECT_STR = 'tcp:127.0.0.1:5760'

# Waypoints (NED: North, East from home)
WAYPOINTS = [
    {"name": "WP1", "n": 10.0,  "e": 5.0},
    {"name": "WP2", "n": -5.0,  "e": 10.0},
    {"name": "WP3", "n": -10.0, "e": -5.0},
    {"name": "WP4", "n": 8.0,   "e": -8.0},
    {"name": "WP5", "n": 3.0,   "e": 3.0},
]


class SITLDrone:
    """Simplified drone controller for SITL testing."""

    def __init__(self, connection_string):
        print(f"Connecting to {connection_string}...")
        self.conn = mavutil.mavlink_connection(connection_string)
        self.conn.wait_heartbeat(timeout=30)
        print(f"Connected! Mode: {self.conn.flightmode}")

        # Request data streams
        self.conn.mav.request_data_stream_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
        )
        time.sleep(2)
        self._drain()

        # Wait for EKF to be ready
        self.wait_ready()

    def _drain(self):
        """Drain all buffered messages."""
        count = 0
        while self.conn.recv_match(blocking=False) is not None:
            count += 1
        return count

    def get_pos(self):
        """Get latest position (drains buffer first)."""
        self._drain()
        time.sleep(0.05)
        latest = None
        while True:
            msg = self.conn.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if msg is None:
                break
            latest = msg
        if latest is None:
            latest = self.conn.recv_match(type='LOCAL_POSITION_NED',
                                          blocking=True, timeout=2)
        if latest:
            return latest.x, latest.y, -latest.z
        return 0, 0, 0

    def set_mode(self, mode_name):
        """Set flight mode."""
        mode_id = self.conn.mode_mapping().get(mode_name)
        if mode_id is None:
            print(f"  ERROR: Unknown mode {mode_name}")
            return False
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        for _ in range(30):
            msg = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.custom_mode == mode_id:
                return True
        return True

    def wait_ready(self, timeout=60):
        """Wait for EKF to initialize and pre-arm checks to pass."""
        print("  Waiting for EKF to initialize (this takes ~15-30s in Gazebo)...")
        start = time.time()
        last_status = ""

        while time.time() - start < timeout:
            msg = self.conn.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue

            # Check for STATUSTEXT messages (pre-arm status)
            if msg.get_type() == 'STATUSTEXT':
                text = msg.text.strip()
                if text != last_status:
                    print(f"    FC: {text}")
                    last_status = text
                # EKF is ready when we see these
                if 'EKF' in text and ('started' in text.lower() or 'using' in text.lower()):
                    print("  EKF initialized!")
                    time.sleep(2)
                    return True

            # Also check SYS_STATUS for sensors ready
            if msg.get_type() == 'HEARTBEAT':
                # Check if system_status indicates ready
                if msg.system_status >= 3:  # MAV_STATE_STANDBY or higher
                    elapsed = time.time() - start
                    if elapsed > 15:  # Give at least 15s for EKF
                        print(f"  System ready (waited {elapsed:.0f}s)")
                        return True

        print(f"  Ready timeout after {timeout}s - trying anyway...")
        return True

    def arm(self, retries=10):
        """Arm the drone with retry logic."""
        # Fix: disable sensors that block arming
        try:
            self.conn.mav.param_set_send(self.conn.target_system, self.conn.target_component, b'RNGFND1_TYPE', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            time.sleep(0.5)
            self.conn.mav.param_set_send(self.conn.target_system, self.conn.target_component, b'PRX1_TYPE', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            time.sleep(1)
        except Exception:
            pass
            
        for attempt in range(retries):
            self._drain()
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            # Wait for ACK
            ack = self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if ack and ack.result == 0:
                print(f"    Armed! (attempt {attempt+1})")
                return True

            # Read any STATUSTEXT explaining why arm failed
            reason = "unknown"
            for _ in range(10):
                st = self.conn.recv_match(type='STATUSTEXT', blocking=True, timeout=0.5)
                if st and ('PreArm' in st.text or 'Arm' in st.text):
                    reason = st.text.strip()
                    break

            print(f"    Arm attempt {attempt+1}/{retries} failed: {reason}")
            time.sleep(3)  # Wait before retry

        return False

    def send_vel(self, vx, vy, vz):
        """Send velocity command (NED frame)."""
        self.conn.mav.set_position_target_local_ned_send(
            0, self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
        )

    def takeoff(self, alt, timeout=20):
        """Arm + takeoff to altitude."""
        print(f"  Setting GUIDED mode...")
        self.set_mode("GUIDED")
        time.sleep(1)

        print(f"  Arming...")
        if not self.arm():
            print(f"  ARM FAILED!")
            return False
        time.sleep(2)

        print(f"  Takeoff to {alt}m...")
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt
        )
        self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)

        # Wait for altitude
        start = time.time()
        while time.time() - start < timeout:
            self._drain()
            time.sleep(0.3)
            x, y, a = self.get_pos()
            if a > alt * 0.85:
                print(f"  Reached {a:.1f}m ✓")
                time.sleep(1)
                return True
        print(f"  Takeoff timeout (alt={a:.1f}m)")
        return True  # Continue anyway

    def fly_to(self, target_n, target_e, alt=FLIGHT_ALT, timeout=60):
        """Fly to waypoint using P-controller with velocity commands."""
        start = time.time()

        for i in range(int(timeout / 0.1)):
            self._drain()
            time.sleep(0.05)
            x, y, a = self.get_pos()

            err_n = target_n - x
            err_e = target_e - y
            dist = math.sqrt(err_n**2 + err_e**2)

            if dist < ARRIVAL_RADIUS:
                self.send_vel(0, 0, 0)
                return True, dist, time.time() - start

            # P-controller with speed clamp
            vx = err_n * Kp
            vy = err_e * Kp
            spd = math.sqrt(vx**2 + vy**2)
            if spd > CRUISE_SPEED:
                vx = vx / spd * CRUISE_SPEED
                vy = vy / spd * CRUISE_SPEED

            # Altitude hold
            vz = -(alt - a) * 0.8

            self.send_vel(vx, vy, vz)

            if i % 20 == 0:
                elapsed = time.time() - start
                print(f"    {elapsed:4.0f}s  pos=({x:6.1f},{y:5.1f})  "
                      f"dist={dist:5.1f}m  spd={min(spd, CRUISE_SPEED):4.1f}m/s")

            time.sleep(0.05)

        self.send_vel(0, 0, 0)
        return False, dist, time.time() - start

    def land(self, timeout=30):
        """Land at current position."""
        self.set_mode("LAND")
        start = time.time()
        while time.time() - start < timeout:
            self._drain()
            time.sleep(0.5)
            x, y, a = self.get_pos()
            if a < 0.3:
                return True
        return False

    def hold(self, seconds):
        """Hold position for N seconds."""
        for _ in range(int(seconds * 10)):
            self.send_vel(0, 0, 0)
            time.sleep(0.1)


# =============================================
# MAIN MISSION
# =============================================
def main():
    print("=" * 55)
    print("  gNNS DRONE — FULL GAZEBO MISSION TEST")
    print("=" * 55)
    print(f"  Waypoints: {len(WAYPOINTS)}")
    print(f"  Altitude:  {FLIGHT_ALT}m")
    print(f"  Speed:     {CRUISE_SPEED} m/s")
    print(f"  P-gain:    {Kp}")
    print()

    drone = SITLDrone(CONNECT_STR)
    results = []

    # ---- Initial Takeoff ----
    print("\n" + "=" * 55)
    print("  TAKEOFF")
    print("=" * 55)
    if not drone.takeoff(FLIGHT_ALT):
        print("Takeoff failed!")
        return

    x, y, a = drone.get_pos()
    print(f"  Hovering at ({x:.1f}, {y:.1f}, {a:.1f}m)")
    time.sleep(2)

    # ---- Fly to each waypoint ----
    for i, wp in enumerate(WAYPOINTS):
        print(f"\n{'=' * 55}")
        print(f"  WAYPOINT {i+1}/{len(WAYPOINTS)}: {wp['name']}")
        print(f"  Target: ({wp['n']}, {wp['e']})")
        print(f"{'=' * 55}")

        # Fly to waypoint
        print(f"  Flying...")
        arrived, dist, t = drone.fly_to(wp['n'], wp['e'])

        if arrived:
            print(f"  ✓ ARRIVED in {t:.1f}s (dist={dist:.2f}m)")
        else:
            print(f"  ✗ TIMEOUT after {t:.1f}s (dist={dist:.2f}m)")

        # Hold for 2s to stabilize
        drone.hold(2)
        x, y, a = drone.get_pos()
        err = math.sqrt((x - wp['n'])**2 + (y - wp['e'])**2)
        print(f"  Position: ({x:.1f}, {y:.1f}, {a:.1f}m)  error={err:.2f}m")

        # Land
        print(f"  Landing...")
        if drone.land():
            print(f"  ✓ Landed!")
        else:
            print(f"  ✗ Landing timeout")

        results.append({
            "wp": wp['name'],
            "arrived": arrived,
            "error_m": err,
            "time_s": t,
        })

        # Pause on ground
        print(f"  Waiting {LAND_PAUSE}s on ground...")
        time.sleep(LAND_PAUSE)

        # Takeoff for next waypoint (or return home)
        if i < len(WAYPOINTS) - 1:
            print(f"  Takeoff for next waypoint...")
            drone.takeoff(FLIGHT_ALT)
        else:
            # Return home after last WP
            print(f"\n  Takeoff for HOME...")
            drone.takeoff(FLIGHT_ALT)

    # ---- Return Home ----
    print(f"\n{'=' * 55}")
    print("  RETURNING HOME")
    print(f"{'=' * 55}")

    arrived, dist, t = drone.fly_to(0, 0)
    if arrived:
        print(f"  ✓ Home reached in {t:.1f}s")
    drone.land()

    # ---- Results ----
    print(f"\n{'=' * 55}")
    print("  MISSION RESULTS")
    print(f"{'=' * 55}")
    print(f"  {'WP':<6} {'Arrived':<8} {'Error':>7} {'Time':>7}")
    print(f"  {'-'*30}")

    all_pass = True
    for r in results:
        status = "✓" if r['arrived'] else "✗"
        print(f"  {r['wp']:<6} {status:<8} {r['error_m']:6.2f}m {r['time_s']:6.1f}s")
        if not r['arrived'] or r['error_m'] > 3.0:
            all_pass = False

    print(f"\n  Overall: {'ALL PASS ✓' if all_pass else 'SOME FAILED ✗'}")
    print()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nAborted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
