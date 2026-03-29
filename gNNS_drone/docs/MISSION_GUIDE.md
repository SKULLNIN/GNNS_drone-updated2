# Full Mission Guide

End-to-end instructions for running the gNNS autonomous GPS-denied mission
on real hardware: Jetson Nano + CubeOrange FC + Intel RealSense D455f.

---

## 1. Hardware Overview

```
                    ┌──────────────────────────┐
                    │      CubeOrange FC        │
                    │   ArduPilot Copter 4.x    │
                    │   EKF3 (ExternalNav)      │
                    └────────┬─────────────────┘
                             │ TELEM2 serial
                             │ 921600 baud
                    ┌────────┴─────────────────┐
                    │      Jetson Nano          │
                    │  /dev/ttyTHS1             │
                    │                           │
                    │  gnns_drone (Python)       │
                    │    mavlink_bridge.py       │
                    │    mission_runner.py       │
                    │    flight_controller.py    │
                    │    safety.py               │
                    │                           │
                    │  ROS 2 (optional)          │
                    │    realsense2_camera       │
                    │    rtabmap_ros / ORB-SLAM3 │
                    └──┬────────────┬───────────┘
                       │            │
              ┌────────┴──┐   ┌────┴────────┐
              │ D455f     │   │ RPLiDAR A2  │
              │ RGB+Depth │   │ 360 LiDAR   │
              │ + IMU     │   │ /dev/ttyUSB0│
              └───────────┘   └─────────────┘
```

### Components

| Part | Connection | Purpose |
|------|-----------|---------|
| CubeOrange FC | TELEM2 serial to Jetson `/dev/ttyTHS1` | Flight control, EKF3 |
| Jetson Nano | GPIO UART or USB-to-serial | Companion computer |
| Intel RealSense D455f | USB 3.0 to Jetson | RGB + Depth + IMU for VIO |
| RPLiDAR A2 (optional) | USB to Jetson `/dev/ttyUSB0` | 360-degree obstacle avoidance |
| Battery | Analog sensor to FC | Voltage + current monitoring |

---

## 2. One-Time Jetson Setup

### 2.1 Clone and install

```bash
git clone https://github.com/SKULLNIN/GNNS_drone-updated2.git ~/gNNS_drone
cd ~/gNNS_drone/gNNS_drone
chmod +x scripts/jetson_nano/setup_jetson_nano.sh
./scripts/jetson_nano/setup_jetson_nano.sh
```

This installs Python dependencies from `requirements.txt`:
- `pymavlink` -- MAVLink communication
- `pyrealsense2` -- RealSense camera (if using standalone mode)
- `pyyaml` -- Configuration files
- `numpy` -- Math
- `opencv-python` -- Target detection
- `rplidar-roboticia` -- LiDAR (optional)

### 2.2 Serial port access (udev rule)

```bash
sudo cp scripts/jetson_nano/99-gnns-telem.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

This grants non-root access to `/dev/ttyTHS1` (Jetson hardware UART).
If using USB-to-serial instead, update `config/mavlink_config.yaml`:

```yaml
connection:
  port: "/dev/ttyUSB0"
```

### 2.3 ROS 2 installation (for RViz2 streaming and VIO)

```bash
# ROS 2 Humble
# https://docs.ros.org/en/humble/Installation.html

sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-rtabmap-ros
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

---

## 3. One-Time FC Parameter Setup

Apply these parameters to CubeOrange using Mission Planner or QGroundControl.
Full file: `config/fc_params.yaml`.

### 3.1 Serial port

```
SERIAL2_PROTOCOL = 2     # MAVLink2
SERIAL2_BAUD     = 921   # 921600 baud
```

### 3.2 Disable GPS

```
GPS1_TYPE = 0             # No GPS
GPS2_TYPE = 0             # No GPS
```

### 3.3 EKF3 -- External Navigation (VIO)

This is the most critical section. It tells ArduPilot to use
`VISION_POSITION_ESTIMATE` from the Jetson as its position source.

```
EK3_ENABLE      = 1
EK2_ENABLE      = 0
AHRS_EKF_TYPE   = 3       # Use EKF3

# Source Set 1: ExternalNav (value 6) for everything
EK3_SRC1_POSXY  = 6       # Horizontal position from VIO
EK3_SRC1_VELXY  = 6       # Horizontal velocity from VIO
EK3_SRC1_POSZ   = 6       # Vertical position from VIO
EK3_SRC1_VELZ   = 6       # Vertical velocity from VIO
EK3_SRC1_YAW    = 6       # Yaw from VIO

# Source Set 2: Fallback (basic stabilization if VIO dies)
EK3_SRC2_POSXY  = 0       # None
EK3_SRC2_VELXY  = 0       # None
EK3_SRC2_POSZ   = 1       # Barometer
EK3_SRC2_VELZ   = 0       # None
EK3_SRC2_YAW    = 1       # Compass

# Innovation gates (wider = more tolerant of VIO jumps)
EK3_POS_I_GATE  = 500
EK3_VEL_I_GATE  = 500
EK3_HGT_I_GATE  = 500

EK3_GPS_TYPE    = 0        # Disable GPS fusion
EK3_EXTNAV_NOISE = 0.3    # Start at 0.3; lower if VIO is stable
EK3_EXTNAV_DELAY = 20     # VIO processing delay in ms
```

### 3.4 Vision position input

```
VISO_TYPE     = 1          # MAVLink vision position
VISO_ORIENT   = 0          # Camera facing forward; adjust per mounting
VISO_POS_X    = 0.0        # Camera offset forward from CG (meters)
VISO_POS_Y    = 0.0        # Camera offset right from CG
VISO_POS_Z    = 0.0        # Camera offset down from CG
VISO_DELAY_MS = 20         # Must match EK3_EXTNAV_DELAY
```

Camera orientation guide:
- Forward: `VISO_ORIENT = 0`
- Right: `VISO_ORIENT = 4`
- Backward: `VISO_ORIENT = 6`
- Left: `VISO_ORIENT = 2`

### 3.5 Arming

```
ARMING_CHECK    = -17      # Skip GPS arming check (bit 4)
BRD_SAFETYENABLE = 0       # Disable safety switch for development
```

### 3.6 Flight modes

```
FLTMODE1 = 4               # GUIDED (autonomous navigation)
FLTMODE2 = 0               # STABILIZE (manual safety)
FLTMODE3 = 5               # LOITER (hold position)
FLTMODE4 = 6               # RTL
FLTMODE5 = 9               # LAND (emergency)
FLTMODE6 = 2               # ALT_HOLD
```

### 3.7 Speed and navigation limits

```
WPNAV_SPEED    = 300       # 3 m/s horizontal
WPNAV_SPEED_UP = 150       # 1.5 m/s climb
WPNAV_SPEED_DN = 100       # 1 m/s descent
WPNAV_ACCEL    = 100       # 1 m/s^2 acceleration
WPNAV_RADIUS   = 100       # 1 m waypoint acceptance
```

### 3.8 Obstacle avoidance (optional, with LiDAR)

```
PRX_TYPE      = 2           # MAVLink OBSTACLE_DISTANCE
OA_TYPE       = 1           # BendyRuler path planner
OA_MARGIN_MAX = 3           # 3 m braking margin
AVOID_ENABLE  = 7           # All avoidance enabled
AVOID_MARGIN  = 2           # 2 m minimum distance
```

### 3.9 Battery failsafe

```
BATT_MONITOR   = 4          # Analog voltage + current
BATT_CAPACITY  = 5000       # 5000 mAh (adjust to your battery)
BATT_LOW_VOLT  = 14.4       # 4S low (3.6V x 4)
BATT_CRT_VOLT  = 14.0       # 4S critical (3.5V x 4)
BATT_FS_LOW_ACT = 2         # Low: RTL
BATT_FS_CRT_ACT = 1         # Critical: LAND
```

---

## 4. Configuration Files

All config files live in `config/`. Edit these before flying.

### `config/mavlink_config.yaml`

Connection, message rates, and debug settings.

Key fields:
- `connection.port`: `/dev/ttyTHS1` for Jetson UART, `tcp:127.0.0.1:5760` for SITL
- `send_rates.vision_position_estimate`: 30 Hz (do not lower)
- `request_rates.LOCAL_POSITION_NED`: 30 Hz (EKF feedback)

### `config/flight_config.yaml`

PID gains, velocity/acceleration limits, waypoint parameters, takeoff/landing
tuning, and safety thresholds.

Key fields:
- `position_gains.horizontal.kp`: 0.60 (main tuning knob for speed of approach)
- `velocity_limits.cruise_max`: 4.0 m/s
- `waypoint.arrival_radius`: 2.0 m
- `safety.geofence_radius`: 100.0 m

### `config/vio_config.yaml`

Odometry source selection, camera settings, LiDAR parameters.

Key fields:
- `ros2_odom.odom_source`: `"ros2"` for RTAB-Map or `"orbslam3"` for ORB-SLAM3
- `ros2_odom.odom_topic`: `/odom`
- `ros2_odom.odom_frame_convention`: `"ros_enu_to_ned"`

---

## 5. VIO Bring-Up Order

See [docs/ROS2_VIO_SETUP.md](ROS2_VIO_SETUP.md) for detailed instructions.

Summary:

1. **RealSense camera** -- `ros2 launch realsense2_camera rs_launch.py`
2. **IMU + TF** -- Static transforms for camera-to-body frame
3. **ORB-SLAM3** (or RTAB-Map) -- Produces `/odom` topic
4. **gNNS mission** -- Subscribes to `/odom`, sends `VISION_POSITION_ESTIMATE` to FC

Or use `ros_bridge.sh` which handles steps 1-3 automatically:

```bash
./scripts/jetson_nano/ros_bridge.sh --imu --pointcloud
```

---

## 6. Running the Mission

### Option A: One-command launcher

```bash
cd ~/gNNS_drone
./scripts/jetson_nano/run_mission.sh
```

This runs `python -m gnns_drone` with `PYTHONPATH` set correctly.

### Option B: Direct Python

```bash
cd ~/gNNS_drone
python3 -m gnns_drone
```

### Option C: With ros_bridge.sh (camera + VIO + mission)

```bash
./scripts/jetson_nano/ros_bridge.sh --imu
```

### CLI flags

| Flag | Description |
|------|-------------|
| `--sitl` | SITL mode (connect to `tcp:127.0.0.1:5760`) |
| `--demo` | Use 5 demo waypoints (skip GPS input) |
| `--interactive` / `-i` | Interactive mode: takeoff, then enter coordinates live |
| `--vio-source SOURCE` | `ros2`, `orbslam3`, `t265_raw`, or `simulated` |
| `--config PATH` | Custom MAVLink config YAML |

### Coordinate input

When running without `--demo`, the mission prompts for GPS coordinates:

```
  Enter START position (home):
  Home LAT,LON: -35.3632621,149.1652299

  Enter destination GPS coordinates (up to 5).
  Format: LAT,LON
  Type 'done' to finish, 'demo' for demo waypoints

  WP1/5: -35.3631621,149.1653299
    -> NED: N=11.1m E=9.3m (14.5m away)
  WP2/5: done

  Start mission? (y/n): y
```

### Interactive mode

```bash
python3 -m gnns_drone --interactive
```

Commands:
- `N,E` -- Fly to NED offset (e.g. `10,5`)
- `land` -- Land at current position
- `home` -- Fly to (0,0) and land
- `takeoff` -- Takeoff again after landing
- `pos` -- Show current position
- `hover` -- Hold position
- `quit` -- Land and exit

---

## 7. Web Control Panel

```bash
python3 -m gnns_drone.web_control --sitl        # SITL mode
python3 -m gnns_drone.web_control               # Real hardware
```

Open `http://localhost:5000` in a browser.

| Flag | Description |
|------|-------------|
| `--sitl` | SITL mode |
| `--port N` | Web server port (default 5000) |
| `--vio-source SOURCE` | Odometry source |
| `--no-auto-connect` | Skip auto-connect on startup |

Features:
- Live position display (N, E, Alt, Speed)
- Fly-to-coordinates with one click
- Takeoff / Land / Home / Hover buttons
- Real-time flight map with trail
- Flight log table

---

## 8. Mission Flow

```
    ┌─────────────┐
    │   Connect    │  MAVLink to CubeOrange
    │   to FC      │  Wait for heartbeat
    └──────┬──────┘
           │
    ┌──────┴──────┐
    │  Init Odom   │  Start RTAB-Map / ORB-SLAM3
    │  + VIO Fwd   │  Begin sending VISION_POSITION to FC at 30 Hz
    └──────┬──────┘
           │
    ┌──────┴──────┐
    │   Takeoff    │  GUIDED mode -> Arm -> Takeoff to 2.5 m
    └──────┬──────┘
           │
    ┌──────┴──────┐
    │  For each    │  PID velocity control toward waypoint
    │  Waypoint:   │  Slowdown near arrival radius
    │    Fly       │  Precision land (search -> align -> final -> touchdown)
    │    Land      │  Wait 3s on ground
    │    Takeoff   │  Re-arm and climb back to cruise altitude
    └──────┬──────┘
           │
    ┌──────┴──────┐
    │  Return to   │  Fly back to (0,0) NED
    │  Home (RTH)  │  Precision land at home
    └──────┬──────┘
           │
    ┌──────┴──────┐
    │  Mission     │  Print results: waypoints completed, time, errors
    │  Complete    │
    └─────────────┘
```

---

## 9. Safety and Failsafe

The safety monitor (`safety.py`) runs at 2 Hz checking:

| Check | Threshold | Action |
|-------|-----------|--------|
| Odometry stale | > 2 seconds old | LAND (configurable) |
| SLAM confidence low | < 30% for 5 seconds | LAND |
| Geofence breach | > 100 m from home | RTH |
| Altitude ceiling | > 10 m | LAND |
| Battery low | < 20% | RTH |
| Battery critical | < 14.0V | LAND |
| EKF unhealthy | Constant position mode | LAND |
| FC connection lost | No heartbeat for 5s | LAND |

Failsafe actions are wrapped in try/except. If the primary action (e.g., LAND
command) fails, the system falls back to emergency force-disarm.

---

## 10. Diagnostics

```bash
python3 -m gnns_drone.diagnostics
```

Validates:
- FC connection and heartbeat
- MAVLink parameter readback
- EKF status
- Sensor health
- VIO confidence

Run this before every flight to catch configuration errors.

---

## 11. Pre-Flight Checklist

1. **FC parameters applied** -- All params from Section 3 loaded on CubeOrange
2. **Serial cable connected** -- TELEM2 to Jetson `/dev/ttyTHS1`
3. **Camera USB connected** -- D455f on USB 3.0
4. **`rs-enumerate-devices`** -- Camera detected
5. **Config files reviewed** -- `mavlink_config.yaml` port matches hardware
6. **Diagnostics pass** -- `python3 -m gnns_drone.diagnostics`
7. **ROS topics flowing** -- `ros2 topic hz /odom` shows stable rate
8. **EKF converged** -- Mission Planner shows EKF healthy (green)
9. **Battery charged** -- Above 80% recommended
10. **Geofence set** -- `flight_config.yaml` `safety.geofence_radius` appropriate
11. **Safety switch** -- Re-enable `BRD_SAFETYENABLE=1` for actual flights
12. **Pilot ready** -- RC transmitter with STABILIZE on switch for manual override
