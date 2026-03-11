#!/usr/bin/env python3
"""
gNNS Drone — SITL Integration Test
=====================================
Uses our ACTUAL FlightController PID to fly in SITL.
Tests the real code that will run on the drone.

Terminal 1: sim_vehicle.py -v ArduCopter --no-mavproxy
Terminal 2: python3 sitl/integration_test.py
"""

import sys
import os
import time
import math

# Import our actual gNNS code
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from gnns_drone.flight_controller import FlightController, FlightConfig
from pymavlink import mavutil

# =============================================
# SITL HELPERS
# =============================================
class SITLConnection:
    """Minimal SITL bridge — same message flow as MAVLinkBridge."""

    def __init__(self, url='tcp:127.0.0.1:5760'):
        print(f"Connecting to {url}...")
        self.conn = mavutil.mavlink_connection(url)
        self.conn.wait_heartbeat(timeout=30)
        print(f"Connected! Mode: {self.conn.flightmode}")
        self.conn.mav.request_data_stream_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
        )
        time.sleep(2)
        self.drain()

    def drain(self):
        while self.conn.recv_match(blocking=False):
            pass

    def get_pos(self):
        """Returns (north, east, altitude) — altitude positive up."""
        self.drain()
        time.sleep(0.02)
        latest = None
        for _ in range(100):
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

    def send_vel(self, vx, vy, vz):
        if any(math.isnan(v) or math.isinf(v) for v in (vx, vy, vz)):
            vx, vy, vz = 0, 0, 0
        self.conn.mav.set_position_target_local_ned_send(
            0, self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
        )

    def set_mode(self, mode):
        mode_id = self.conn.mode_mapping().get(mode)
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        for _ in range(20):
            msg = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.custom_mode == mode_id:
                return True
        return True

    def arm(self):
        for attempt in range(15):
            self.drain()
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            ack = self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if ack and ack.result == 0:
                print(f"  Armed (attempt {attempt+1})")
                return True
            # Show failure reason
            for _ in range(5):
                st = self.conn.recv_match(type='STATUSTEXT', blocking=True, timeout=0.3)
                if st and ('PreArm' in st.text or 'Arm' in st.text):
                    print(f"  Attempt {attempt+1}: {st.text.strip()}")
                    break
            time.sleep(2)
        return False

    def takeoff_cmd(self, alt):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt
        )
        self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)

    def wait_ekf(self, timeout=60):
        print("  Waiting for EKF to initialize...")
        start = time.time()
        while time.time() - start < timeout:
            msg = self.conn.recv_match(blocking=True, timeout=1)
            if msg and msg.get_type() == 'STATUSTEXT':
                t = msg.text.strip()
                if t:
                    print(f"    {t}")
            if msg and msg.get_type() == 'HEARTBEAT':
                if msg.system_status >= 3 and (time.time() - start) > 10:
                    print(f"  EKF ready ({time.time()-start:.0f}s)")
                    return
        print("  EKF timeout, continuing...")


# =============================================
# MAIN TEST
# =============================================
def main():
    print("=" * 55)
    print("  gNNS DRONE — PID INTEGRATION TEST")
    print("  Uses real FlightController from flight_config.yaml")
    print("=" * 55)

    # Load OUR config
    cfg_path = os.path.join(PROJECT_ROOT, "config", "flight_config.yaml")
    cfg = FlightConfig.from_yaml(cfg_path)
    fc = FlightController(cfg)

    print(f"\n  Config loaded:")
    print(f"    Kp={cfg.horizontal_gains.kp}  Ki={cfg.horizontal_gains.ki}  Kd={cfg.horizontal_gains.kd}")
    print(f"    Cruise: {cfg.cruise_max_speed} m/s  Accel: {cfg.horiz_accel_limit} m/s²")
    print(f"    Arrival: {cfg.arrival_radius}m  Slowdown: {cfg.slowdown_radius}m")

    # Connect
    sitl = SITLConnection()
    sitl.wait_ekf()

    # Waypoints
    waypoints = [
        ("WP1", 10.0, 5.0),
        ("WP2", -5.0, 8.0),
        ("WP3", 4.0, -4.0),
    ]

    # === TAKEOFF ===
    print(f"\n{'='*55}")
    print("  TAKEOFF")
    print(f"{'='*55}")
    sitl.set_mode("GUIDED")
    time.sleep(1)
    if not sitl.arm():
        print("  FAILED TO ARM!")
        return
    time.sleep(2)
    sitl.takeoff_cmd(cfg.takeoff_altitude)

    # Wait for alt
    for _ in range(60):
        sitl.drain()
        time.sleep(0.3)
        _, _, alt = sitl.get_pos()
        if alt > cfg.takeoff_altitude * 0.8:
            print(f"  At {alt:.1f}m ✓")
            break
    time.sleep(2)

    # === FLY EACH WAYPOINT ===
    results = []
    for wp_name, tgt_n, tgt_e in waypoints:
        print(f"\n{'='*55}")
        print(f"  {wp_name}: fly to ({tgt_n}, {tgt_e})")
        print(f"{'='*55}")

        fc.reset()  # Clean PID state for each waypoint
        start = time.time()
        dt = 0.1  # 10Hz control loop
        arrived = False

        for step in range(500):  # 50s max
            loop_start = time.time()

            x, y, alt = sitl.get_pos()
            dist = math.sqrt((tgt_n - x)**2 + (tgt_e - y)**2)

            if dist < cfg.arrival_radius:
                sitl.send_vel(0, 0, 0)
                elapsed = time.time() - start
                print(f"  ✓ ARRIVED in {elapsed:.1f}s  dist={dist:.2f}m  pos=({x:.1f},{y:.1f})")
                arrived = True
                results.append((wp_name, True, dist, elapsed))
                break

            # ===== OUR ACTUAL FLIGHT CONTROLLER =====
            vx, vy, vz = fc.compute_waypoint_velocity(
                tgt_n, tgt_e, cfg.cruise_altitude,
                x, y, alt,
                dt=dt
            )
            sitl.send_vel(vx, vy, vz)

            speed = math.sqrt(vx**2 + vy**2)
            if step % 15 == 0:
                elapsed = time.time() - start
                print(f"    {elapsed:4.0f}s  pos=({x:6.1f},{y:5.1f},{alt:4.1f})  "
                      f"dist={dist:5.1f}m  spd={speed:4.2f}m/s  "
                      f"cmd=({vx:5.2f},{vy:5.2f},{vz:5.2f})")

            # Maintain consistent loop rate
            loop_elapsed = time.time() - loop_start
            sleep_time = dt - loop_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        if not arrived:
            elapsed = time.time() - start
            print(f"  ✗ TIMEOUT  dist={dist:.2f}m")
            results.append((wp_name, False, dist, elapsed))
            sitl.send_vel(0, 0, 0)

        time.sleep(2)

    # === RETURN HOME + LAND ===
    print(f"\n{'='*55}")
    print("  RETURN HOME + LAND")
    print(f"{'='*55}")

    fc.reset()
    start = time.time()
    for step in range(300):
        loop_start = time.time()
        x, y, alt = sitl.get_pos()
        dist = math.sqrt(x**2 + y**2)
        if dist < cfg.arrival_radius:
            sitl.send_vel(0, 0, 0)
            print(f"  Home reached in {time.time()-start:.1f}s ✓")
            break
        vx, vy, vz = fc.compute_waypoint_velocity(0, 0, cfg.cruise_altitude, x, y, alt, dt=0.1)
        sitl.send_vel(vx, vy, vz)
        sleep_t = 0.1 - (time.time() - loop_start)
        if sleep_t > 0:
            time.sleep(sleep_t)

    sitl.set_mode("LAND")
    print("  Landing...")
    for _ in range(60):
        sitl.drain()
        time.sleep(0.5)
        _, _, alt = sitl.get_pos()
        if alt < 0.3:
            print(f"  Landed ✓")
            break

    # === RESULTS ===
    print(f"\n{'='*55}")
    print("  RESULTS")
    print(f"{'='*55}")
    print(f"  {'WP':<6} {'Status':<10} {'Error':>7} {'Time':>7}")
    print(f"  {'-'*35}")
    all_pass = True
    for name, ok, err, t in results:
        s = "PASS ✓" if ok else "FAIL ✗"
        print(f"  {name:<6} {s:<10} {err:6.2f}m {t:6.1f}s")
        if not ok:
            all_pass = False

    print(f"\n  Controller: FlightController PID")
    print(f"  Config:     flight_config.yaml")
    print(f"  Overall:    {'ALL PASS ✓' if all_pass else 'SOME FAILED ✗'}")
    print()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nAborted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
