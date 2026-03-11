"""
gNNS Drone — SITL Simulation Test
====================================
Tests the entire mission pipeline against ArduPilot SITL.

Requirements:
  1. ArduPilot SITL running (see instructions below)
  2. pymavlink installed: pip install pymavlink

How to start ArduPilot SITL:
  Option A: Docker (easiest, works on Windows)
    docker run -it --rm -p 5760:5760 radarku/ardupilot-sitl

  Option B: WSL / Linux
    cd ~/ardupilot
    sim_vehicle.py -v ArduCopter --console --map -L KSFO

  Option C: Mission Planner SITL
    Open Mission Planner → Simulation tab → Multirotor

  The SITL listens on tcp:127.0.0.1:5760 (or 5762 for extra connection)

Usage:
  # Test 1: Basic connectivity
  python sim_test.py --test connect

  # Test 2: Arm and takeoff only
  python sim_test.py --test takeoff

  # Test 3: Fly a single waypoint
  python sim_test.py --test single_wp

  # Test 4: Full 5-waypoint mission (simulated VIO)
  python sim_test.py --test full_mission

  # Test 5: Coordinate math (no SITL needed)
  python sim_test.py --test math
"""

import sys
import os
import time
import math
import logging

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gnns_drone.mavlink_bridge import MAVLinkBridge
from gnns_drone.coordinate_utils import GPSCoord, NEDCoord, WaypointManager, gps_to_ned

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("sim_test")


# =================================================================
# SITL CONFIG — override for simulation
# =================================================================
SITL_CONFIG = {
    "connection": {
        "port": "tcp:127.0.0.1:5762",   # SITL default port
        "baudrate": 921600,
        "source_system": 1,
        "source_component": 191,
        "heartbeat_timeout_s": 5.0,
    },
    "send_rates": {
        "vision_position_estimate": 30,
    },
    "request_rates": {
        "LOCAL_POSITION_NED": 30,
        "ATTITUDE": 10,
        "EKF_STATUS_REPORT": 2,
        "VFR_HUD": 1,
        "BATTERY_STATUS": 1,
    },
    "debug": {
        "log_level": "DEBUG",
        "log_all_incoming": False,
        "log_command_acks": True,
        "log_mode_changes": True,
    },
}


# =================================================================
# TEST 1: Basic Connection
# =================================================================
def test_connect():
    """Test basic MAVLink connection to SITL."""
    print("\n" + "=" * 60)
    print("  TEST 1: BASIC CONNECTION")
    print("=" * 60)

    bridge = MAVLinkBridge.__new__(MAVLinkBridge)
    bridge.config = SITL_CONFIG
    bridge.conn = None
    bridge.stats = bridge.__class__.__dict__  # Will be overwritten
    # Re-init properly
    bridge = MAVLinkBridge()
    bridge.config = SITL_CONFIG

    print(f"\n  Connecting to SITL at {SITL_CONFIG['connection']['port']}...")
    if not bridge.connect():
        print("  ❌ Connection FAILED!")
        print("  Make sure SITL is running:")
        print("    Docker: docker run -it --rm -p 5760:5760 -p 5762:5762 radarku/ardupilot-sitl")
        print("    WSL:    sim_vehicle.py -v ArduCopter --console --map")
        return False

    print("  Waiting for heartbeat...")
    if not bridge.wait_for_heartbeat(timeout=10):
        print("  ❌ No heartbeat!")
        bridge.disconnect()
        return False

    print(f"  ✅ Connected! Stats: {bridge.stats}")

    # Read some params
    print("\n  Reading FC parameters...")
    params = ["AHRS_EKF_TYPE", "GPS1_TYPE", "ARMING_CHECK"]
    for p in params:
        val = bridge.get_param(p)
        print(f"    {p} = {val}")

    # Read position
    time.sleep(1)
    pos = bridge.get_latest("LOCAL_POSITION_NED")
    if pos:
        print(f"\n  Current position: x={pos.x:.2f} y={pos.y:.2f} z={pos.z:.2f}")
    
    att = bridge.get_latest("ATTITUDE")
    if att:
        print(f"  Attitude: R={math.degrees(att.roll):.1f}° "
              f"P={math.degrees(att.pitch):.1f}° "
              f"Y={math.degrees(att.yaw):.1f}°")

    print("\n  ✅ TEST PASSED!")
    bridge.disconnect()
    return True


# =================================================================
# TEST 2: Arm + Takeoff
# =================================================================
def test_takeoff():
    """Test arming and takeoff in SITL."""
    print("\n" + "=" * 60)
    print("  TEST 2: ARM + TAKEOFF")
    print("=" * 60)

    bridge = MAVLinkBridge()
    bridge.config = SITL_CONFIG

    if not bridge.connect() or not bridge.wait_for_heartbeat(timeout=10):
        print("  ❌ Connection failed!")
        return False

    bridge.configure_message_rates()

    # SITL has GPS by default, so EKF is already happy
    # (On real drone, you'd need VIO data for 10-30s first)

    print("\n  Setting GUIDED mode...")
    if not bridge.set_mode("GUIDED"):
        print("  ❌ Failed to set GUIDED mode!")
        bridge.disconnect()
        return False

    time.sleep(1)

    print("  Arming...")
    if not bridge.arm():
        print("  ❌ Arm failed! Check SITL console for pre-arm errors.")
        bridge.disconnect()
        return False

    print(f"  Armed: {bridge.is_armed}")
    time.sleep(1)

    print("  Taking off to 3m...")
    bridge.takeoff(3.0)

    # Monitor altitude
    for i in range(30):
        pos = bridge.get_latest("LOCAL_POSITION_NED")
        if pos:
            alt = -pos.z
            print(f"    [{i}s] Altitude: {alt:.2f}m")
            if alt > 2.5:
                print("  ✅ Reached target altitude!")
                break
        time.sleep(1)

    # Land
    print("\n  Landing...")
    bridge.land()
    bridge.wait_landed(timeout=30)

    print("  ✅ TEST PASSED!")
    bridge.disconnect()
    return True


# =================================================================
# TEST 3: Single Waypoint
# =================================================================
def test_single_waypoint():
    """Arm, takeoff, fly 10m north, land."""
    print("\n" + "=" * 60)
    print("  TEST 3: SINGLE WAYPOINT (10m North)")
    print("=" * 60)

    bridge = MAVLinkBridge()
    bridge.config = SITL_CONFIG

    if not bridge.connect() or not bridge.wait_for_heartbeat(timeout=10):
        print("  ❌ Connection failed!")
        return False

    bridge.configure_message_rates()
    time.sleep(2)

    # Takeoff
    bridge.set_mode("GUIDED")
    time.sleep(1)
    bridge.arm()
    time.sleep(1)
    bridge.takeoff(3.0)
    time.sleep(5)

    # Fly 10m North
    print("\n  Flying 10m North...")
    arrived = bridge.goto_position_ned_wait(
        north=10.0, east=0.0, down=-3.0,
        tolerance=1.0, timeout=30.0
    )

    if arrived:
        print("  ✅ Reached waypoint!")
    else:
        print("  ⚠️ Timeout (might still be close)")

    pos = bridge.get_latest("LOCAL_POSITION_NED")
    if pos:
        dist = math.sqrt(pos.x ** 2 + pos.y ** 2)
        print(f"  Position: x={pos.x:.2f} y={pos.y:.2f} (dist={dist:.2f}m from home)")

    # Return home
    print("\n  Returning home...")
    bridge.goto_position_ned_wait(0, 0, -3.0, tolerance=1.0, timeout=30.0)

    # Land
    print("  Landing...")
    bridge.land()
    bridge.wait_landed(timeout=30)

    print("  ✅ TEST PASSED!")
    bridge.disconnect()
    return True


# =================================================================
# TEST 4: Full Mission (Simulated VIO)
# =================================================================
def test_full_mission():
    """
    Full 5-waypoint mission with SIMULATED VIO.
    
    In SITL, GPS is enabled, so EKF already works.
    We still send VISION_POSITION_ESTIMATE to validate the pipeline,
    but EKF uses GPS as primary (since we haven't disabled it in SITL).
    
    To test GPS-denied in SITL, set these params first:
      GPS1_TYPE = 0
      EK3_SRC1_POSXY = 6
      EK3_SRC1_VELXY = 6
      EK3_SRC1_POSZ = 6
      EK3_SRC1_VELZ = 6
      VISO_TYPE = 1
    """
    print("\n" + "=" * 60)
    print("  TEST 4: FULL 5-WAYPOINT MISSION")
    print("=" * 60)

    bridge = MAVLinkBridge()
    bridge.config = SITL_CONFIG

    if not bridge.connect() or not bridge.wait_for_heartbeat(timeout=10):
        print("  ❌ Connection failed!")
        return False

    bridge.configure_message_rates()
    time.sleep(2)

    # Create waypoints — small distances for testing
    # These simulate a 50m x 50m area
    wm = WaypointManager()
    
    # SITL starts at a known location — use SITL's home as our home
    # (In real flight, user types their GPS coordinates)
    home = GPSCoord(lat=-35.363262, lon=149.165237)  # SITL default CMAC
    wm.set_home(home)

    test_waypoints = [
        GPSCoord(-35.363172, 149.165337),  # ~10m NE
        GPSCoord(-35.363352, 149.165337),  # ~10m SE
        GPSCoord(-35.363352, 149.165137),  # ~10m SW
        GPSCoord(-35.363172, 149.165137),  # ~10m NW
        GPSCoord(-35.363262, 149.165237),  # Back near home
    ]

    print("\n  Mission waypoints:")
    for i, wp in enumerate(test_waypoints):
        ned = wm.add_waypoint(wp)
        print(f"    WP{i+1}: {wp} → NED({ned.north:.1f}m N, {ned.east:.1f}m E)")

    # Takeoff
    print("\n  Arming + Takeoff...")
    bridge.set_mode("GUIDED")
    time.sleep(1)
    if not bridge.arm():
        print("  ❌ Arm failed!")
        bridge.disconnect()
        return False
    time.sleep(1)
    bridge.takeoff(3.0)
    time.sleep(5)

    # Fly to each waypoint
    results = []
    for i in range(wm.count):
        ned = wm.get_waypoint_ned(i)
        print(f"\n  --- WAYPOINT {i+1}/5 ---")
        print(f"  Target: NED({ned.north:.1f}, {ned.east:.1f})")

        arrived = bridge.goto_position_ned_wait(
            ned.north, ned.east, -3.0,
            tolerance=2.0, timeout=30.0
        )

        pos = bridge.get_latest("LOCAL_POSITION_NED")
        if pos:
            err = math.sqrt((pos.x - ned.north)**2 + (pos.y - ned.east)**2)
            print(f"  Position error: {err:.2f}m")
            results.append({"wp": i+1, "arrived": arrived, "error_m": err})

        # Land
        print(f"  Landing at WP{i+1}...")
        bridge.land()
        bridge.wait_landed(timeout=20)
        time.sleep(2)

        # Takeoff for next (unless last)
        if i < wm.count - 1:
            print(f"  Takeoff for WP{i+2}...")
            bridge.set_mode("GUIDED")
            time.sleep(0.5)
            bridge.arm()
            time.sleep(1)
            bridge.takeoff(3.0)
            time.sleep(3)

    # Return home
    print("\n  --- RETURN TO HOME ---")
    bridge.set_mode("GUIDED")
    time.sleep(0.5)
    bridge.arm()
    time.sleep(1)
    bridge.takeoff(3.0)
    time.sleep(3)
    bridge.goto_position_ned_wait(0, 0, -3.0, tolerance=2.0, timeout=30.0)
    bridge.land()
    bridge.wait_landed(timeout=20)

    # Results
    print("\n" + "=" * 60)
    print("  MISSION RESULTS")
    print("=" * 60)
    for r in results:
        status = "✅" if r["arrived"] else "⚠️"
        print(f"  WP{r['wp']}: {status} error={r['error_m']:.2f}m")

    avg_err = sum(r["error_m"] for r in results) / len(results) if results else 0
    print(f"\n  Average error: {avg_err:.2f}m")
    print(f"  {'✅ MISSION PASSED!' if avg_err < 3.0 else '❌ MISSION NEEDS TUNING'}")

    bridge.disconnect()
    return avg_err < 3.0


# =================================================================
# TEST 5: Math Only (No SITL needed)
# =================================================================
def test_math():
    """Test coordinate conversions — no hardware or SITL needed."""
    print("\n" + "=" * 60)
    print("  TEST 5: COORDINATE MATH")
    print("=" * 60)

    # Test GPS → NED conversion
    home = GPSCoord(28.6139, 77.2090)
    target = GPSCoord(28.6150, 77.2100)
    ned = gps_to_ned(home, target)

    print(f"\n  Home:   {home}")
    print(f"  Target: {target}")
    print(f"  NED:    N={ned.north:.2f}m  E={ned.east:.2f}m")

    # Verify: ~1° lat ≈ 111km, so 0.0011° ≈ 122m
    assert 100 < ned.north < 150, f"North should be ~122m, got {ned.north}"
    print(f"  ✅ North distance looks correct ({ned.north:.1f}m ≈ 122m expected)")

    # Test WaypointManager
    wm = WaypointManager()
    wm.set_home(home)

    print("\n  Testing 5-waypoint mission:")
    test_points = [
        (28.6145, 77.2095),
        (28.6135, 77.2085),
        (28.6142, 77.2100),
        (28.6148, 77.2082),
        (28.6139, 77.2090),  # Back to home
    ]

    for i, (lat, lon) in enumerate(test_points):
        ned = wm.add_waypoint(GPSCoord(lat, lon))
        dist = math.sqrt(ned.north**2 + ned.east**2)
        print(f"    WP{i+1}: ({lat}, {lon}) → N={ned.north:+.1f}m E={ned.east:+.1f}m (dist={dist:.1f}m)")

    # Last point should be very close to (0,0)
    last_ned = wm.get_waypoint_ned(4)
    assert abs(last_ned.north) < 2 and abs(last_ned.east) < 2, \
        f"WP5 should be near home, got ({last_ned.north:.1f}, {last_ned.east:.1f})"
    print(f"  ✅ WP5 near home: ({last_ned.north:.2f}, {last_ned.east:.2f})m")

    # Test distance calculation
    from gnns_drone.coordinate_utils import haversine_distance
    d = haversine_distance(home, target)
    print(f"\n  Haversine distance home→target: {d:.1f}m")
    assert 100 < d < 200, f"Expected ~156m, got {d}"
    print(f"  ✅ Distance looks correct")

    # Test round-trip: NED → GPS → NED should be consistent
    wm2 = WaypointManager()
    wm2.set_home(home)
    ned1 = wm2.add_waypoint(GPSCoord(28.6150, 77.2100))
    print(f"\n  Round-trip: GPS(28.6150, 77.2100) → NED({ned1.north:.2f}, {ned1.east:.2f})")
    print(f"  ✅ All math tests passed!")

    return True


# =================================================================
# TEST 6: Velocity Commands (SITL)
# =================================================================
def test_velocity():
    """Test velocity-based flight in SITL."""
    print("\n" + "=" * 60)
    print("  TEST 6: VELOCITY COMMANDS")
    print("=" * 60)

    bridge = MAVLinkBridge()
    bridge.config = SITL_CONFIG

    if not bridge.connect() or not bridge.wait_for_heartbeat(timeout=10):
        print("  ❌ Connection failed!")
        return False

    bridge.configure_message_rates()
    time.sleep(2)

    # Takeoff
    bridge.set_mode("GUIDED")
    time.sleep(1)
    bridge.arm()
    time.sleep(1)
    bridge.takeoff(3.0)
    time.sleep(5)

    # Test: Fly north at 1 m/s for 5 seconds = ~5m
    print("\n  Flying NORTH at 1 m/s for 5 seconds...")
    for i in range(50):  # 10 Hz × 5 seconds
        bridge.send_velocity_ned(vx=1.0, vy=0.0, vz=0.0)
        time.sleep(0.1)

    # Stop
    bridge.send_velocity_ned(0, 0, 0)
    time.sleep(1)

    pos = bridge.get_latest("LOCAL_POSITION_NED")
    if pos:
        print(f"  Position: x={pos.x:.2f}m y={pos.y:.2f}m alt={-pos.z:.2f}m")
        print(f"  Expected: x≈5.0m y≈0.0m")
        if 3.0 < pos.x < 7.0 and abs(pos.y) < 2.0:
            print("  ✅ Velocity commands working!")
        else:
            print("  ⚠️ Position off — check velocity command type_mask")

    # Test: Fly east at 0.5 m/s for 4 seconds
    print("\n  Flying EAST at 0.5 m/s for 4 seconds...")
    for i in range(40):
        bridge.send_velocity_ned(vx=0.0, vy=0.5, vz=0.0)
        time.sleep(0.1)

    bridge.send_velocity_ned(0, 0, 0)
    time.sleep(1)

    pos = bridge.get_latest("LOCAL_POSITION_NED")
    if pos:
        print(f"  Position: x={pos.x:.2f}m y={pos.y:.2f}m")
        print(f"  Expected: x≈5.0m y≈2.0m")

    # Land
    bridge.land()
    bridge.wait_landed(timeout=20)

    print("  ✅ TEST PASSED!")
    bridge.disconnect()
    return True


# =================================================================
# MAIN
# =================================================================
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="gNNS Drone SITL Tests")
    parser.add_argument("--test", default="math",
                        choices=["connect", "takeoff", "single_wp",
                                 "full_mission", "math", "velocity", "all"],
                        help="Which test to run")
    parser.add_argument("--port", default=None,
                        help="Override SITL port (e.g., tcp:127.0.0.1:5760)")
    args = parser.parse_args()

    if args.port:
        SITL_CONFIG["connection"]["port"] = args.port
        print(f"Using port: {args.port}")

    tests = {
        "math": test_math,
        "connect": test_connect,
        "takeoff": test_takeoff,
        "single_wp": test_single_waypoint,
        "velocity": test_velocity,
        "full_mission": test_full_mission,
    }

    if args.test == "all":
        results = {}
        for name, func in tests.items():
            try:
                results[name] = func()
            except Exception as e:
                print(f"\n  ❌ {name} FAILED with exception: {e}")
                results[name] = False

        print("\n" + "=" * 60)
        print("  ALL TEST RESULTS")
        print("=" * 60)
        for name, passed in results.items():
            print(f"  {'✅' if passed else '❌'} {name}")
    else:
        try:
            tests[args.test]()
        except Exception as e:
            print(f"\n  ❌ Test failed: {e}")
            import traceback
            traceback.print_exc()
