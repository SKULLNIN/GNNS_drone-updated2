"""
gNNS Drone — Mission Runner
=============================
Main entry point for the autonomous GPS-denied navigation mission.

Orchestrates all components:
  1. Connect to FC (serial or SITL TCP)
  2. Initialize odometry (RTAB-Map or SITL position)
  3. Input coordinates or interactive control
  4. PID-based navigation with velocity commands
  5. Land at waypoints, return home

Usage:
  Real drone:      python -m gnns_drone
  SITL:            python -m gnns_drone --sitl
  SITL demo:       python -m gnns_drone --sitl --demo
  SITL interactive: python -m gnns_drone --sitl --interactive
"""

import time
import sys
import math
import signal
import logging
import threading
from pathlib import Path
from typing import Optional
from .mavlink_bridge import MAVLinkBridge
from .rtabmap_odom import RTABMapOdom, OdomData
from .flight_controller import FlightController, FlightConfig
from .coordinate_utils import GPSCoord, NEDCoord, WaypointManager, gps_to_ned

logger = logging.getLogger("gnns.mission")

# ============================================================
# DEFAULTS
# ============================================================
FLIGHT_ALTITUDE = 2.5
VIO_SEND_RATE = 30
GROUND_WAIT_TIME = 3.0

# ArduPilot SITL default home
SITL_HOME = GPSCoord(-35.3632621, 149.1652299, 584.0)

# Demo waypoints (around SITL home, ~11m apart)
DEMO_WAYPOINTS = [
    GPSCoord(-35.3631621, 149.1653299),  # ~11m NE
    GPSCoord(-35.3633621, 149.1653299),  # ~11m SE
    GPSCoord(-35.3633621, 149.1651299),  # ~11m SW
    GPSCoord(-35.3631621, 149.1651299),  # ~11m NW
    GPSCoord(-35.3632121, 149.1652299),  # ~5.5m N
]


class MissionRunner:
    """
    Executes the full autonomous GPS-denied mission.
    
    Works with BOTH:
      - Real hardware: MAVLinkBridge (serial) + RTABMapOdom (camera)
      - SITL simulation: MAVLinkBridge (TCP) + RTABMapOdom (SITL mode)
    
    Same code path. Same PID controller. Same Navigator logic.
    """

    def __init__(self, config_path: Optional[str] = None, sitl_mode: bool = False):
        self.sitl_mode = sitl_mode

        # Load MAVLink config — override port for SITL
        if sitl_mode and config_path is None:
            # Use SITL TCP connection
            self.bridge = MAVLinkBridge()
            self.bridge.config["connection"]["port"] = "tcp:127.0.0.1:5760"
            self.bridge.config["connection"]["sitl_mode"] = True
        else:
            self.bridge = MAVLinkBridge(config_path)

        # Flight controller (PID + velocity smoothing)
        fc_config_path = str(Path(__file__).parent.parent / "config" / "flight_config.yaml")
        self.fc = FlightController(FlightConfig.from_yaml(fc_config_path))
        self.config = self.fc.config

        # Odometry — SITL reads from MAVLink, real uses RTAB-Map
        if sitl_mode:
            self.odom = RTABMapOdom(mode="sitl", config={"bridge": self.bridge})
        else:
            self.odom = RTABMapOdom(mode="ros2")

        # Waypoint manager
        self.waypoints = WaypointManager()
        self.current_waypoint = 0
        self.waypoints_completed = 0
        self.mission_active = False
        self.mission_start_time = 0.0

        # VIO → FC forwarding (not needed in SITL — FC already has position)
        self._vio_send_running = False
        self._vio_send_thread = None

        # Graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, sig, frame):
        logger.critical("CTRL+C — Emergency landing!")
        self.mission_active = False
        try:
            self.bridge.land()
        except Exception:
            pass
        sys.exit(1)

    # ==============================================================
    # COORDINATE INPUT
    # ==============================================================

    def input_coordinates(self):
        """Interactive GPS coordinate input — competition style."""
        print("\n" + "=" * 60)
        print("  gNNS DRONE — GPS MISSION")
        if self.sitl_mode:
            print("  [SITL SIMULATION MODE]")
        print("=" * 60)

        # Home position
        print("\n  Enter START position (home):")
        if self.sitl_mode:
            print("  (Press Enter for SITL default)")
        home_input = input("  Home LAT,LON: ").strip()

        if home_input:
            parts = home_input.replace(" ", "").split(",")
            home = GPSCoord(float(parts[0]), float(parts[1]))
        else:
            home = SITL_HOME
            print(f"  Using: {home}")

        self.waypoints.set_home(home)

        # Destination waypoints
        print(f"\n  Enter destination GPS coordinates (up to 5).")
        print("  Format: LAT,LON")
        print("  Type 'done' to finish, 'demo' for demo waypoints\n")

        count = 0
        while count < 5:
            wp_input = input(f"  WP{count+1}/5: ").strip().lower()

            if wp_input == 'done' or wp_input == '':
                if count == 0:
                    print("  Need at least 1 waypoint!")
                    continue
                break

            if wp_input == 'demo':
                for wp in DEMO_WAYPOINTS:
                    ned = self.waypoints.add_waypoint(wp)
                    count += 1
                print(f"  Added 5 demo waypoints")
                break

            try:
                parts = wp_input.replace(" ", "").split(",")
                wp = GPSCoord(float(parts[0]), float(parts[1]))
                ned = self.waypoints.add_waypoint(wp)
                print(f"    → NED: N={ned.north:.1f}m E={ned.east:.1f}m"
                      f" ({ned.horizontal_distance:.1f}m away)")
                count += 1
            except (ValueError, IndexError):
                print("    Invalid! Use: LAT,LON")

        self.waypoints.print_mission_summary()

        confirm = input("\n  Start mission? (y/n): ").strip().lower()
        if confirm != 'y':
            print("  Cancelled.")
            sys.exit(0)

    def input_coordinates_programmatic(self, home_lat, home_lon, waypoints_list):
        """Set coordinates from code (for testing)."""
        self.waypoints.set_home(GPSCoord(home_lat, home_lon))
        for lat, lon in waypoints_list:
            self.waypoints.add_waypoint(GPSCoord(lat, lon))

    # ==============================================================
    # VIO → FC POSITION FORWARDING (real drone only)
    # ==============================================================

    def _start_vio_sender(self):
        """Send VIO position to FC (only needed for real drone, not SITL)."""
        if self.sitl_mode:
            logger.info("SITL mode — skipping VIO→FC forwarding (SITL has its own position)")
            return

        self._vio_send_running = True
        self._vio_send_thread = threading.Thread(
            target=self._vio_send_loop, daemon=True, name="vio-sender"
        )
        self._vio_send_thread.start()
        logger.info(f"VIO→FC sender started at {VIO_SEND_RATE} Hz")

    def _vio_send_loop(self):
        interval = 1.0 / VIO_SEND_RATE
        while self._vio_send_running:
            data = self.odom.get()
            if data.confidence > 30:
                self.bridge.send_vision_position(
                    data.x, data.y, data.z,
                    data.roll, data.pitch, data.yaw
                )
                self.bridge.send_vision_speed(data.vx, data.vy, data.vz)
            time.sleep(interval)

    def _stop_vio_sender(self):
        self._vio_send_running = False
        if self._vio_send_thread:
            self._vio_send_thread.join(timeout=2.0)

    # ==============================================================
    # MAIN MISSION
    # ==============================================================

    def run(self):
        """Execute the full mission."""
        self.mission_start_time = time.time()
        self.mission_active = True

        try:
            # ---- INIT ----
            self._phase_connect()
            self._phase_init()

            # ---- TAKEOFF ----
            self._phase_takeoff()

            # ---- NAVIGATE WAYPOINTS ----
            for i in range(self.waypoints.count):
                if not self.mission_active:
                    break
                self._phase_goto_waypoint(i)

            # ---- RETURN HOME ----
            if self.mission_active:
                self._phase_return_home()

            # ---- COMPLETE ----
            elapsed = time.time() - self.mission_start_time
            self._print_results(elapsed)

        except KeyboardInterrupt:
            logger.critical("ABORTED!")
            self.bridge.land()
        except Exception as e:
            logger.critical(f"MISSION ERROR: {e}")
            import traceback
            traceback.print_exc()
            self.bridge.land()
        finally:
            self.mission_active = False
            self._stop_vio_sender()
            self.odom.stop()
            self.bridge.disconnect()

    # ==============================================================
    # MISSION PHASES
    # ==============================================================

    def _phase_connect(self):
        """Connect to FC."""
        logger.info("=== CONNECTING ===")
        if not self.bridge.connect():
            raise RuntimeError("Connection failed!")
        if not self.bridge.wait_for_heartbeat():
            raise RuntimeError("No heartbeat!")

        if self.sitl_mode:
            self.bridge.request_all_streams(10)
            time.sleep(2)
            self.bridge.wait_ekf_ready(timeout=60)
        else:
            self.bridge.configure_message_rates()

    def _phase_init(self):
        """Initialize odometry and forwarding."""
        logger.info("=== INITIALIZING ===")
        self.odom.start()
        time.sleep(2.0)
        self._start_vio_sender()

        logger.info(f"Config: kp={self.config.horizontal_gains.kp}, "
                     f"speed={self.config.cruise_max_speed}m/s, "
                     f"accel={self.config.horiz_accel_limit}m/s²")

    def _phase_takeoff(self):
        """Arm + takeoff."""
        logger.info("=== TAKEOFF ===")
        if not self.bridge.set_mode("GUIDED"):
            raise RuntimeError("Cannot set GUIDED mode!")
        time.sleep(1.0)

        arm_retries = 10 if self.sitl_mode else 3
        if not self.bridge.arm(retries=arm_retries):
            raise RuntimeError("Cannot arm!")
        time.sleep(2.0)

        if not self.bridge.takeoff(self.config.takeoff_altitude):
            logger.warning("Takeoff may not have reached full altitude")
        time.sleep(3.0)

    def _phase_goto_waypoint(self, index: int):
        """Fly to waypoint, land, wait, takeoff."""
        gps = self.waypoints.get_waypoint_gps(index)
        ned = self.waypoints.get_waypoint_ned(index)
        self.current_waypoint = index + 1

        logger.info(f"\n{'='*55}")
        logger.info(f"  WAYPOINT {index+1}/{self.waypoints.count}")
        logger.info(f"  GPS: {gps}")
        logger.info(f"  NED: N={ned.north:.1f}m E={ned.east:.1f}m "
                     f"({ned.horizontal_distance:.1f}m)")
        logger.info(f"{'='*55}")

        # ---- FLY using PID FlightController ----
        self.fc.reset()
        arrived = self._fly_to_ned(ned.north, ned.east)

        if arrived:
            logger.info(f"  ✓ Arrived at WP{index+1}")
        else:
            logger.warning(f"  ✗ WP{index+1} timeout, landing at current position")

        # ---- LAND ----
        logger.info(f"  Landing...")
        self.bridge.land()
        self.bridge.wait_landed(timeout=30)
        self.waypoints_completed += 1

        # Log accuracy
        data = self.odom.get()
        err = math.sqrt((data.x - ned.north)**2 + (data.y - ned.east)**2)
        logger.info(f"  WP{index+1} landed — error={err:.2f}m")

        # ---- WAIT ON GROUND ----
        logger.info(f"  Waiting {GROUND_WAIT_TIME}s on ground...")
        time.sleep(GROUND_WAIT_TIME)

        # ---- TAKEOFF for next ----
        if index < self.waypoints.count - 1 or True:  # Always takeoff (RTH)
            logger.info("  Takeoff...")
            self.bridge.set_mode("GUIDED")
            time.sleep(0.5)
            arm_retries = 10 if self.sitl_mode else 3
            self.bridge.arm(retries=arm_retries)
            time.sleep(1.0)
            self.bridge.takeoff(self.config.takeoff_altitude)
            time.sleep(2.0)

    def _phase_return_home(self):
        """Return to home and land."""
        logger.info(f"\n{'='*55}")
        logger.info("  RETURNING HOME")
        logger.info(f"{'='*55}")

        self.fc.reset()
        self._fly_to_ned(0, 0)
        self.bridge.land()
        self.bridge.wait_landed(timeout=30)
        logger.info("  HOME — Landed!")

    # ==============================================================
    # PID NAVIGATION (uses FlightController)
    # ==============================================================

    def _fly_to_ned(self, target_n: float, target_e: float,
                     timeout: float = 120.0) -> bool:
        """
        Fly to NED position using our PID FlightController.
        
        This is the SAME code path for real drone and SITL.
        Only the data source differs (RTAB-Map vs MAVLink position).
        """
        logger.info(f"  Flying to ({target_n:.1f}, {target_e:.1f})...")
        start = time.time()
        dt = 0.1  # 10 Hz control loop

        while time.time() - start < timeout:
            if not self.mission_active:
                return False

            loop_start = time.time()
            data = self.odom.get()

            distance = math.sqrt(
                (target_n - data.x)**2 + (target_e - data.y)**2
            )

            if distance < self.config.arrival_radius:
                self.bridge.send_velocity_ned(0, 0, 0)
                elapsed = time.time() - start
                logger.info(f"  Reached target in {elapsed:.1f}s (dist={distance:.2f}m)")
                return True

            # PID velocity command — THIS IS OUR ACTUAL FlightController
            result = self.fc.compute_waypoint_velocity(
                target_n, target_e, self.config.cruise_altitude,
                data.x, data.y, data.altitude,
                dt=dt
            )
            vx, vy, vz = result[0], result[1], result[2]
            yaw = result[3] if len(result) > 3 else 0.0
            self.bridge.send_velocity_ned_yaw(vx, vy, vz, yaw)

            # Rate-limited logging
            elapsed = time.time() - start
            if int(elapsed) % 3 == 0 and int(elapsed) != int(elapsed - dt):
                speed = math.sqrt(vx**2 + vy**2)
                logger.debug(f"    dist={distance:.1f}m "
                             f"spd={speed:.2f}m/s "
                             f"alt={data.altitude:.1f}m ")

            # Maintain 10 Hz
            sleep_t = dt - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

        logger.warning("  Fly-to timeout!")
        return False

    # ==============================================================
    # RESULTS
    # ==============================================================

    def _print_results(self, elapsed):
        logger.info(f"\n{'='*55}")
        logger.info(f"  MISSION COMPLETE")
        logger.info(f"{'='*55}")
        logger.info(f"  Time:      {elapsed:.0f}s")
        logger.info(f"  Waypoints: {self.waypoints_completed}/{self.waypoints.count}")
        logger.info(f"  Mode:      {'SITL' if self.sitl_mode else 'HARDWARE'}")
        logger.info(f"  PID:       kp={self.config.horizontal_gains.kp}")

    # ==============================================================
    # INTERACTIVE MODE
    # ==============================================================

    def run_interactive(self):
        """
        Interactive flight mode:
          1. Connect + arm + takeoff
          2. Hover and show live NED position
          3. User types NED offset coordinates
          4. Drone flies there
          5. Repeat until user types 'land' or 'home'
        """
        self.mission_active = True
        print("\n" + "=" * 60)
        print("  gNNS DRONE — INTERACTIVE MODE")
        if self.sitl_mode:
            print("  [SITL SIMULATION]")
        print("=" * 60)

        try:
            # ---- Connect ----
            self._phase_connect()
            self._phase_init()

            # ---- Takeoff ----
            self._phase_takeoff()
            time.sleep(2)

            # ---- Position display thread ----
            self._pos_display_running = True
            pos_thread = threading.Thread(target=self._position_display_loop, daemon=True)
            pos_thread.start()

            # ---- Interactive command loop ----
            print("\n" + "=" * 60)
            print("  INTERACTIVE CONTROL")
            print("=" * 60)
            print("  Commands:")
            print("    N,E         → Fly to NED offset (meters). e.g: 10,5")
            print("    land        → Land at current position")
            print("    home        → Fly to (0,0) and land")
            print("    takeoff     → Takeoff again after landing")
            print("    pos         → Show current position")
            print("    hover       → Hold current position")
            print("    quit        → Land and exit")
            print()

            waypoint_count = 0
            flight_log = []

            while self.mission_active:
                try:
                    cmd = input("  CMD> ").strip().lower()
                except EOFError:
                    break

                if not cmd:
                    continue

                # ---- LAND ----
                if cmd == 'land':
                    self._pos_display_running = False
                    print("\n  Landing...")
                    self.bridge.land()
                    self.bridge.wait_landed(timeout=30)
                    data = self.odom.get()
                    print(f"  Landed at N={data.x:.2f} E={data.y:.2f}")
                    print("  Type 'takeoff' to fly again, 'quit' to exit")
                    self._pos_display_running = True

                # ---- TAKEOFF ----
                elif cmd == 'takeoff':
                    self._pos_display_running = False
                    print("\n  Takeoff...")
                    self.bridge.set_mode("GUIDED")
                    time.sleep(0.5)
                    arm_retries = 10 if self.sitl_mode else 3
                    self.bridge.arm(retries=arm_retries)
                    time.sleep(1)
                    self.bridge.takeoff(self.config.takeoff_altitude)
                    time.sleep(3)
                    data = self.odom.get()
                    print(f"  Hovering at N={data.x:.1f} E={data.y:.1f} Alt={data.altitude:.1f}m")
                    self._pos_display_running = True

                # ---- HOME ----
                elif cmd == 'home':
                    self._pos_display_running = False
                    print("\n  Flying to HOME (0, 0)...")
                    self.fc.reset()
                    arrived = self._fly_to_ned(0, 0)
                    if arrived:
                        print("  ✓ Home reached!")
                    self.bridge.land()
                    self.bridge.wait_landed(timeout=30)
                    print("  Landed at home")
                    self._pos_display_running = True

                # ---- POSITION ----
                elif cmd == 'pos':
                    data = self.odom.get()
                    print(f"\n  Position: N={data.x:8.2f}  E={data.y:8.2f}  Alt={data.altitude:.2f}m")
                    print(f"  Velocity: N={data.vx:8.3f}  E={data.vy:8.3f}  D={data.vz:.3f}")
                    print(f"  Speed:    {data.speed_horizontal:.2f} m/s")
                    print(f"  Home:     {data.distance_from_home:.2f}m away")

                # ---- HOVER ----
                elif cmd == 'hover':
                    self.bridge.send_velocity_ned(0, 0, 0)
                    print("  Holding position")

                # ---- QUIT ----
                elif cmd == 'quit' or cmd == 'exit':
                    self._pos_display_running = False
                    print("\n  Landing and shutting down...")
                    self.bridge.land()
                    self.bridge.wait_landed(timeout=30)
                    self.mission_active = False
                    break

                # ---- FLY TO N,E ----
                else:
                    try:
                        parts = cmd.replace(" ", "").split(",")
                        if len(parts) != 2:
                            print("  Invalid! Use: N,E  (e.g. 10,5 or -3.5,8)")
                            continue

                        target_n = float(parts[0])
                        target_e = float(parts[1])

                        data = self.odom.get()
                        dist = math.sqrt((target_n - data.x)**2 + (target_e - data.y)**2)

                        waypoint_count += 1
                        self._pos_display_running = False
                        print(f"\n  WP{waypoint_count}: Flying to N={target_n:.1f} E={target_e:.1f} "
                              f"({dist:.1f}m away)")

                        self.fc.reset()
                        start = time.time()
                        arrived = self._fly_to_ned(target_n, target_e)
                        elapsed = time.time() - start

                        data = self.odom.get()
                        err = math.sqrt((target_n - data.x)**2 + (target_e - data.y)**2)

                        if arrived:
                            print(f"  ✓ Arrived in {elapsed:.1f}s  error={err:.2f}m")
                        else:
                            print(f"  ✗ Timeout after {elapsed:.0f}s  error={err:.2f}m")

                        flight_log.append({
                            "wp": waypoint_count,
                            "target": (target_n, target_e),
                            "error": err,
                            "time": elapsed,
                            "arrived": arrived,
                        })

                        self._pos_display_running = True

                    except ValueError:
                        print("  Invalid! Use: N,E  (e.g. 10,5)")
                        print("  Or: land, home, takeoff, pos, hover, quit")

            # ---- Flight log ----
            if flight_log:
                print(f"\n{'='*55}")
                print("  FLIGHT LOG")
                print(f"{'='*55}")
                print(f"  {'#':<4} {'Target N':>9} {'Target E':>9} {'Error':>7} {'Time':>6} {'Status'}")
                print(f"  {'-'*50}")
                for f in flight_log:
                    s = "✓" if f['arrived'] else "✗"
                    print(f"  {f['wp']:<4} {f['target'][0]:9.1f} {f['target'][1]:9.1f} "
                          f"{f['error']:6.2f}m {f['time']:5.1f}s  {s}")
                print()

        except KeyboardInterrupt:
            print("\n\n  Emergency landing!")
            self.bridge.land()
        except Exception as e:
            print(f"\n  Error: {e}")
            import traceback
            traceback.print_exc()
            self.bridge.land()
        finally:
            self._pos_display_running = False
            self.mission_active = False
            self._stop_vio_sender()
            self.odom.stop()
            self.bridge.disconnect()

    def _position_display_loop(self):
        """Background thread showing live position every 2 seconds."""
        while self._pos_display_running and self.mission_active:
            data = self.odom.get()
            # Use \r to overwrite the line (doesn't interfere with input)
            sys.stdout.write(
                f"\r  📍 N={data.x:7.2f}  E={data.y:7.2f}  "
                f"Alt={data.altitude:4.1f}m  "
                f"Spd={data.speed_horizontal:4.2f}m/s  "
                f"Home={data.distance_from_home:5.1f}m    "
            )
            sys.stdout.flush()
            time.sleep(2.0)


# ==============================================================
# ENTRY POINT
# ==============================================================

def main():
    """Main entry point — same executable for real and SITL."""
    import argparse
    parser = argparse.ArgumentParser(description="gNNS Drone Mission Runner")
    parser.add_argument("--config", default=None, help="MAVLink config YAML path")
    parser.add_argument("--sitl", action="store_true",
                        help="SITL mode (connect to tcp:127.0.0.1:5760)")
    parser.add_argument("--demo", action="store_true",
                        help="Use demo waypoints (skip GPS input)")
    parser.add_argument("--interactive", "-i", action="store_true",
                        help="Interactive mode: takeoff, then enter coordinates live")
    parser.add_argument("--camera", default="ros2",
                        choices=["ros2", "t265_raw", "simulated"],
                        help="Odometry source (ignored in --sitl mode)")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S"
    )

    runner = MissionRunner(config_path=args.config, sitl_mode=args.sitl)

    if args.interactive:
        # Interactive mode: takeoff → hover → user types coordinates
        runner.run_interactive()
    elif args.demo:
        runner.waypoints.set_home(SITL_HOME)
        for wp in DEMO_WAYPOINTS:
            runner.waypoints.add_waypoint(wp)
        runner.waypoints.print_mission_summary()
        runner.run()
    else:
        runner.input_coordinates()
        runner.run()


if __name__ == "__main__":
    main()

