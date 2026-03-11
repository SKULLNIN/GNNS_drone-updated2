import time
from pymavlink import mavutil

print("Connecting to TCP 5760...")
conn = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
conn.wait_heartbeat()
print("Heartbeat received.")

for p_name in [b'RNGFND1_TYPE', b'PRX1_TYPE']:
    print(f"Disabling {p_name} parameter...")
    conn.mav.param_set_send(conn.target_system, conn.target_component, p_name, 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.5)

# Set mode GUIDED
mode_id = conn.mode_mapping()['GUIDED']
conn.mav.set_mode_send(conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

print("Sending arm command...")
conn.mav.command_long_send(conn.target_system, conn.target_component, 
                           mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                           0, 1, 0, 0, 0, 0, 0, 0)

# Check STATUSTEXT to see why it fails
start_time = time.time()
while time.time() - start_time < 5.0:
    msg = conn.recv_match(type='STATUSTEXT', blocking=True, timeout=1.0)
    if msg:
        print("STATUSTEXT:", msg.text)

print("Done.")
