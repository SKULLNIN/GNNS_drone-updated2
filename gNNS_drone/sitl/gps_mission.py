#!/usr/bin/env python3
"""
gNNS Drone — GPS Mission Runner (SITL)
==========================================
The COMPETITION FLOW:
  1. Enter current GPS coordinates (start/home)
  2. Enter 5 destination GPS coordinates
  3. Drone takes off
  4. Flies to each coordinate, lands, waits, takes off again
  5. Returns home

Uses:
  - coordinate_utils.py  → GPS to local NED conversion
  - flight_controller.py → PID navigation
  - Same flow as real drone (just pymavlink instead of MAVLinkBridge)

Run:
  Terminal 1: sim_vehicle.py -v ArduCopter --no-mavproxy
  Terminal 2: python3 sitl/gps_mission.py
"""

import sys
import os
import time
import math

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from gnns_drone.flight_controller import FlightController, FlightConfig
from gnns_drone.coordinate_utils import GPSCoord, WaypointManager, gps_to_ned
from pymavlink import mavutil


# =============================================
# SITL DEFAULT HOME (ArduPilot SITL default)
# =============================================
SITL_HOME = GPSCoord(-35.3632621, 149.1652299, 584.0)


class SITLDrone:
    """SITL connection with proper drone dynamics."""

    def __init__(self, url='tcp:127.0.0.1:5760'):
        print(f"\nConnecting to {url}...")
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
        self.drain()
        time.sleep(0.02)
        latest = None
        for _ in range(200):
            msg = self.conn.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if msg is None:
                break
            latest = msg
        if latest is None:
            latest = self.conn.recv_match(type='LOCAL_POSITION_NED',
                                          blocking=True, timeout=2)
        if latest:
            return latest.x, latest.y, -latest.z
        return 0.0, 0.0, 0.0

    def get_gps(self):
        """Get current GPS from SITL (GLOBAL_POSITION_INT)."""
        self.drain()
        time.sleep(0.02)
        latest = None
        for _ in range(200):
            msg = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg is None:
                break
            latest = msg
        if latest is None:
            latest = self.conn.recv_match(type='GLOBAL_POSITION_INT',
                                          blocking=True, timeout=2)
        if latest:
            return GPSCoord(
                lat=latest.lat / 1e7,
                lon=latest.lon / 1e7,
                alt=latest.relative_alt / 1000.0
            )
        return None

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
            for _ in range(5):
                st = self.conn.recv_match(type='STATUSTEXT', blocking=True, timeout=0.3)
                if st and 'PreArm' in st.text:
                    print(f"  Attempt {attempt+1}: {st.text.strip()}")
                    break
            time.sleep(2)
        return False

    def takeoff_and_wait(self, alt, timeout=25):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt
        )
        self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        for _ in range(int(timeout / 0.3)):
            self.drain()
            time.sleep(0.3)
            _, _, a = self.get_pos()
            if a > alt * 0.8:
                print(f"  Reached {a:.1f}m ✓")
                return True
        return False

    def wait_ekf(self, timeout=60):
        print("  Waiting for EKF...")
        start = time.time()
        while time.time() - start < timeout:
            msg = self.conn.recv_match(blocking=True, timeout=1)
            if msg and msg.get_type() == 'STATUSTEXT':
                t = msg.text.strip()
                if t:
                    print(f"    {t}")
            if msg and msg.get_type() == 'HEARTBEAT':
                if msg.system_status >= 3 and (time.time() - start) > 10:
                    print(f"  Ready ({time.time()-start:.0f}s)")
                    return
        print("  Timeout, continuing...")

    def land_and_wait(self, timeout=30):
        self.set_mode("LAND")
        for _ in range(int(timeout / 0.5)):
            self.drain()
            time.sleep(0.5)
            _, _, a = self.get_pos()
            if a < 0.3:
                print(f"  Touchdown ✓")
                return True
        return False


def get_gps_input():
    """Interactive GPS coordinate entry — competition style."""
    print("\n" + "=" * 60)
    print("  GPS COORDINATE ENTRY")
    print("=" * 60)

    # Home/Start position
    print("\n  Enter START position (home) GPS coordinates.")
    print("  (For SITL, press Enter to use default)")
    home_input = input("  Home LAT,LON (or Enter for SITL default): ").strip()

    if home_input:
        parts = home_input.replace(" ", "").split(",")
        home = GPSCoord(float(parts[0]), float(parts[1]))
    else:
        home = SITL_HOME
        print(f"  Using SITL default: {home}")

    # Destination waypoints
    waypoints = []
    print(f"\n  Enter destination GPS coordinates (up to 5).")
    print("  Format: LAT,LON  (e.g. -35.3622,149.1655)")
    print("  Type 'done' to finish, 'demo' for demo waypoints\n")

    while len(waypoints) < 5:
        prompt = f"  WP{len(waypoints)+1}/5: "
        wp_input = input(prompt).strip().lower()

        if wp_input == 'done' or wp_input == '':
            if len(waypoints) == 0:
                print("  Need at least 1 waypoint!")
                continue
            break

        if wp_input == 'demo':
            # Generate demo waypoints around SITL home
            demo_offsets = [
                (0.0001, 0.0001),   # ~11m NE
                (-0.0001, 0.0001),  # ~11m SE (relative N is lat)
                (-0.0001, -0.0001), # ~11m SW
                (0.0001, -0.0001),  # ~11m NW
                (0.00005, 0.0),     # ~5.5m N
            ]
            for dlat, dlon in demo_offsets:
                wp = GPSCoord(home.lat + dlat, home.lon + dlon)
                waypoints.append(wp)
            print(f"  Added 5 demo waypoints around home")
            break

        try:
            parts = wp_input.replace(" ", "").split(",")
            wp = GPSCoord(float(parts[0]), float(parts[1]))
            waypoints.append(wp)
            # Show distance from home
            ned = gps_to_ned(home, wp)
            print(f"    → NED: N={ned.north:.1f}m E={ned.east:.1f}m"
                  f" ({ned.horizontal_distance:.1f}m away)")
        except (ValueError, IndexError):
            print("    Invalid format! Use: LAT,LON  (e.g. -35.3622,149.1655)")

    return home, waypoints


def fly_to_waypoint(drone, fc, cfg, tgt_n, tgt_e, tgt_alt, name="WP"):
    """Fly to a NED waypoint using our FlightController PID."""
    fc.reset()
    start = time.time()
    dt = 0.1

    for step in range(500):  # 50s max
        loop_start = time.time()
        x, y, alt = drone.get_pos()
        dist = math.sqrt((tgt_n - x)**2 + (tgt_e - y)**2)

        if dist < cfg.arrival_radius:
            drone.send_vel(0, 0, 0)
            elapsed = time.time() - start
            print(f"  ✓ Arrived at {name} in {elapsed:.1f}s  (error={dist:.2f}m)")
            return True, dist, elapsed

        vx, vy, vz = fc.compute_waypoint_velocity(
            tgt_n, tgt_e, tgt_alt, x, y, alt, dt=dt
        )
        drone.send_vel(vx, vy, vz)

        spd = math.sqrt(vx**2 + vy**2)
        if step % 15 == 0:
            elapsed = time.time() - start
            # Also show estimated GPS position
            print(f"    {elapsed:4.0f}s  N={x:7.1f} E={y:6.1f} Alt={alt:4.1f}  "
                  f"dist={dist:5.1f}m  spd={spd:4.1f}m/s")

        sleep_t = dt - (time.time() - loop_start)
        if sleep_t > 0:
            time.sleep(sleep_t)

    drone.send_vel(0, 0, 0)
    return False, dist, time.time() - start


def main():
    print("=" * 60)
    print("  gNNS DRONE — GPS MISSION RUNNER")
    print("  Competition Mode: Enter coordinates → Fly → Land")
    print("=" * 60)

    # Get GPS coordinates from user
    home_gps, wp_gps_list = get_gps_input()

    # Build waypoint plan
    wm = WaypointManager()
    wm.set_home(home_gps)
    for wp in wp_gps_list:
        wm.add_waypoint(wp)
    wm.print_mission_summary()

    # Confirm
    print("\n  Press Enter to START MISSION (Ctrl+C to abort)")
    input("  > ")

    # Load flight controller
    cfg = FlightConfig.from_yaml(os.path.join(PROJECT_ROOT, "config", "flight_config.yaml"))
    fc = FlightController(cfg)
    print(f"\n  FlightController: kp={cfg.horizontal_gains.kp}, "
          f"speed={cfg.cruise_max_speed}m/s, accel={cfg.horiz_accel_limit}m/s²")

    # Connect
    drone = SITLDrone()
    drone.wait_ekf()

    # Get actual SITL GPS to verify home
    actual_gps = drone.get_gps()
    if actual_gps:
        print(f"\n  SITL GPS:   {actual_gps}")
        print(f"  Your home:  {home_gps}")
        offset = gps_to_ned(actual_gps, home_gps)
        print(f"  Offset:     {offset.horizontal_distance:.1f}m")

    # === MISSION START ===
    print(f"\n{'='*60}")
    print("  MISSION START")
    print(f"{'='*60}")

    # Takeoff
    print("\n[TAKEOFF]")
    drone.set_mode("GUIDED")
    time.sleep(1)
    if not drone.arm():
        print("  ARM FAILED!")
        return
    time.sleep(2)
    drone.takeoff_and_wait(cfg.takeoff_altitude)
    time.sleep(2)

    results = []

    # Fly to each waypoint
    for i, (gps, ned) in enumerate(wm.get_all()):
        print(f"\n{'='*60}")
        print(f"  WAYPOINT {i+1}/{wm.count}")
        print(f"  GPS:  {gps}")
        print(f"  NED:  {ned}")
        print(f"  Dist: {ned.horizontal_distance:.1f}m")
        print(f"{'='*60}")

        # Fly there
        arrived, err, t = fly_to_waypoint(
            drone, fc, cfg,
            ned.north, ned.east, cfg.cruise_altitude,
            name=f"WP{i+1}"
        )

        # Stabilize
        drone.send_vel(0, 0, 0)
        time.sleep(2)
        x, y, alt = drone.get_pos()
        actual_err = math.sqrt((ned.north - x)**2 + (ned.east - y)**2)

        # Land
        print(f"  Landing at WP{i+1}...")
        if drone.land_and_wait():
            print(f"  Landed! Waiting 3s...")
        else:
            print(f"  Landing timeout")

        results.append({
            "wp": f"WP{i+1}",
            "gps": gps,
            "arrived": arrived,
            "error_m": actual_err,
            "time_s": t,
        })

        time.sleep(3)  # Pause on ground (competition requirement)

        # Takeoff for next (or return home)
        if i < wm.count - 1 or True:  # Always takeoff (need to go home)
            print(f"  Takeoff...")
            drone.set_mode("GUIDED")
            time.sleep(1)
            drone.arm()
            time.sleep(2)
            drone.takeoff_and_wait(cfg.takeoff_altitude)
            time.sleep(2)

    # Return home
    print(f"\n{'='*60}")
    print("  RETURNING HOME")
    print(f"{'='*60}")
    arrived, err, t = fly_to_waypoint(
        drone, fc, cfg, 0, 0, cfg.cruise_altitude, name="HOME"
    )
    drone.land_and_wait()

    # === RESULTS ===
    print(f"\n{'='*60}")
    print("  MISSION RESULTS")
    print(f"{'='*60}")
    print(f"  Home: {home_gps}")
    print(f"  {'WP':<6} {'GPS Lat':>12} {'GPS Lon':>12} {'Status':<8} {'Error':>7} {'Time':>6}")
    print(f"  {'-'*55}")

    all_pass = True
    for r in results:
        s = "✓" if r['arrived'] else "✗"
        print(f"  {r['wp']:<6} {r['gps'].lat:12.7f} {r['gps'].lon:12.7f}"
              f" {s:<8} {r['error_m']:6.2f}m {r['time_s']:5.1f}s")
        if not r['arrived'] or r['error_m'] > 3.0:
            all_pass = False

    print(f"\n  Result: {'MISSION COMPLETE ✓' if all_pass else 'MISSION INCOMPLETE ✗'}")
    print()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nMission aborted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
