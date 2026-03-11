#!/usr/bin/env python3
"""
gNNS Drone — SITL Flight Test (pymavlink direct)
==================================================
Arms, takes off, flies to a waypoint, and lands.
No ROS/MAVROS needed — just pymavlink + ArduPilot SITL.

Usage (in WSL terminal 2, while SITL runs in terminal 1):
  python3 sitl/quick_flight_test.py
"""

import time
import math
import sys
from pymavlink import mavutil

# =============================================
# CONNECT
# =============================================
print("=" * 50)
print("  gNNS SITL Flight Test")
print("=" * 50)
print()

print("[1] Connecting to SITL on tcp:127.0.0.1:5760...")
conn = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
conn.wait_heartbeat(timeout=30)
print(f"    Connected! Mode: {conn.flightmode}")
print(f"    System: {conn.target_system}, Component: {conn.target_component}")

# =============================================
# HELPER FUNCTIONS
# =============================================
def set_mode(mode_name):
    """Set flight mode."""
    mode_id = conn.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f"    ERROR: Unknown mode {mode_name}")
        print(f"    Available: {list(conn.mode_mapping().keys())}")
        return False
    conn.mav.set_mode_send(conn.target_system,
                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                           mode_id)
    # Wait for ACK
    for _ in range(30):
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.custom_mode == mode_id:
            print(f"    Mode set: {mode_name}")
            return True
    print(f"    WARNING: Mode {mode_name} may not have set")
    return True

def arm():
    """Arm the drone."""
    # First, disable any problematic sensors that might block arming (like rangefinder/proximity)
    try:
        print("    Clearing parameter RNGFND1_TYPE to unblock arming...")
        conn.mav.param_set_send(conn.target_system, conn.target_component, b'RNGFND1_TYPE', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.5)
        print("    Clearing parameter PRX1_TYPE to unblock arming...")
        conn.mav.param_set_send(conn.target_system, conn.target_component, b'PRX1_TYPE', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(1)
    except Exception as e:
        print(f"    WARNING: Failed to clear params: {e}")

    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    msg = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if msg and msg.result == 0:
        print("    Armed!")
        return True
    
    # If failed, attempt to fetch STATUSTEXT to see why
    reason = "Unknown error"
    status_msg = conn.recv_match(type='STATUSTEXT', blocking=True, timeout=2)
    if status_msg:
        reason = status_msg.text
        
    print(f"    Arm result: {msg} (Reason: {reason})")
    return False

def disarm():
    """Disarm the drone."""
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 21196, 0, 0, 0, 0, 0  # 21196 = force disarm
    )
    time.sleep(1)
    print("    Disarmed")

def takeoff(alt):
    """Send takeoff command."""
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt
    )
    msg = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print(f"    Takeoff command sent (target: {alt}m)")

def send_velocity(vx, vy, vz):
    """Send velocity command in NED frame."""
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # velocity only
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )

def send_position(x, y, z):
    """Send position command in NED frame."""
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,  # position only
        x, y, -z,  # NED: z is negative up
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

def drain_and_get_position():
    """
    Drain ALL buffered messages and return the LATEST position.
    
    This is the KEY FIX: pymavlink buffers messages internally.
    recv_match returns the OLDEST message, not the latest.
    We must drain the buffer to get fresh data.
    """
    latest = None
    while True:
        msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg is None:
            break
        latest = msg

    if latest is None:
        # No buffered messages — wait for the next one
        latest = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2)

    if latest:
        return latest.x, latest.y, -latest.z  # NED z → altitude
    return 0, 0, 0

def drain_buffer():
    """Drain all pending messages to clear the buffer."""
    count = 0
    while True:
        msg = conn.recv_match(blocking=False)
        if msg is None:
            break
        count += 1
    return count

def wait_altitude(target_alt, timeout=20):
    """Wait until drone reaches target altitude."""
    start = time.time()
    while time.time() - start < timeout:
        drain_buffer()  # Clear stale messages first
        time.sleep(0.3)
        x, y, alt = drain_and_get_position()
        if alt > target_alt * 0.85:
            print(f"    Reached {alt:.1f}m (target: {target_alt}m)")
            return True
    x, y, alt = drain_and_get_position()
    print(f"    Timeout at {alt:.1f}m")
    return alt > target_alt * 0.5  # Partial success if >50%

# =============================================
# REQUEST DATA STREAMS
# =============================================
print()
print("[2] Requesting data streams...")
conn.mav.request_data_stream_send(
    conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
)
time.sleep(2)
# Drain initial burst of messages
n = drain_buffer()
print(f"    Streams active (drained {n} initial messages)")

# =============================================
# SET GUIDED MODE
# =============================================
print()
print("[3] Setting GUIDED mode...")
set_mode("GUIDED")
time.sleep(1)

# =============================================
# ARM
# =============================================
print()
print("[4] Arming...")
if not arm():
    print("    FAILED! Check SITL console for errors.")
    sys.exit(1)
time.sleep(2)

# =============================================
# TAKEOFF
# =============================================
print()
print("[5] Taking off to 2.5m...")
takeoff(2.5)
if not wait_altitude(2.5, timeout=20):
    print("    Takeoff incomplete, continuing anyway...")
time.sleep(2)
drain_buffer()

x, y, alt = drain_and_get_position()
print(f"    Position after takeoff: ({x:.1f}, {y:.1f}, {alt:.1f}m)")

# =============================================
# FLY TO WAYPOINT (using velocity commands)
# =============================================
print()
print("[6] Flying to waypoint (10, 5) using velocity commands...")
print("    PID-style navigation (kp=0.6, max_speed=1.5 m/s)")
print()

target_n, target_e = 10.0, 5.0
kp = 0.6  # Same as our flight_config.yaml

for i in range(200):  # 20 seconds at 10Hz
    # CRITICAL: drain buffer to get FRESH position
    drain_buffer()
    time.sleep(0.05)  # Let new messages arrive
    x, y, alt = drain_and_get_position()

    err_n = target_n - x
    err_e = target_e - y
    dist = math.sqrt(err_n**2 + err_e**2)

    if dist < 2.0:
        send_velocity(0, 0, 0)
        print(f"    ARRIVED! dist={dist:.2f}m at ({x:.1f}, {y:.1f})")
        break

    # P-controller
    vx = err_n * kp
    vy = err_e * kp
    speed = math.sqrt(vx**2 + vy**2)
    if speed > 1.5:
        vx = vx / speed * 1.5
        vy = vy / speed * 1.5

    # Altitude hold
    vz = -(2.5 - alt) * 0.8

    send_velocity(vx, vy, vz)

    if i % 10 == 0:
        print(f"    t={i*0.1:.0f}s  pos=({x:6.1f},{y:5.1f},{alt:4.1f})  "
              f"dist={dist:5.1f}m  cmd=({vx:5.2f},{vy:5.2f})")

    time.sleep(0.05)

# Hold position
send_velocity(0, 0, 0)
time.sleep(3)
drain_buffer()
x, y, alt = drain_and_get_position()
print(f"    Final position: ({x:.1f}, {y:.1f}, {alt:.1f}m)")

# =============================================
# LAND
# =============================================
print()
print("[7] Landing...")
set_mode("LAND")

print("    Waiting to touch down...")
for i in range(60):
    drain_buffer()
    time.sleep(0.3)
    x, y, alt = drain_and_get_position()
    if alt < 0.3:
        print(f"    Touchdown! alt={alt:.2f}m")
        break
    if i % 10 == 0:
        print(f"    Descending... alt={alt:.1f}m")

time.sleep(3)

# =============================================
# RESULTS
# =============================================
print()
drain_buffer()
x, y, alt = drain_and_get_position()
err = math.sqrt((x - target_n)**2 + (y - target_e)**2)
print("=" * 50)
print("  FLIGHT TEST COMPLETE!")
print("=" * 50)
print(f"  Final pos:  ({x:.2f}, {y:.2f}, {alt:.2f})")
print(f"  Target was: ({target_n}, {target_e})")
print(f"  Horiz error: {err:.2f}m")
print(f"  Result: {'PASS' if err < 3.0 else 'FAIL'}")
print()
