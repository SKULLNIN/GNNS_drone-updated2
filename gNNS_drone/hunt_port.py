import time
from pymavlink import mavutil

print("Scanning ports...")
ports = [
    "udp:127.0.0.1:14550", 
    "udp:0.0.0.0:14550", 
    "udp:192.168.160.1:14550", 
    "tcp:127.0.0.1:5760",
    "tcp:127.0.0.1:5762"
]
for p in ports:
    print(f"Trying {p}...")
    try:
        conn = mavutil.mavlink_connection(p)
        hb = conn.wait_heartbeat(timeout=3)
        if hb:
            print(f"FOUND ON {p}!")
            break
        else:
            print("No heartbeat")
    except Exception as e:
        print(f"Error: {e}")
