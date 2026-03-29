# SITL Simulation Guide

Test the gNNS drone software without hardware using ArduPilot's
Software-In-The-Loop (SITL) simulator. The same Python code runs for
both real hardware and SITL -- only the data source differs.

---

## 1. Prerequisites

### ArduPilot SITL

```bash
# Clone ArduPilot (one time)
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot

# Install build dependencies
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Build ArduCopter SITL
./waf configure --board sitl
./waf copter
```

### Python dependencies

```bash
cd ~/gNNS_drone
pip install -r requirements.txt
```

### Gazebo 11 (optional, for depth camera simulation)

Only needed if you want simulated depth camera data. See Section 7.

---

## 2. Start ArduPilot SITL

Open a terminal and start the SITL vehicle:

```bash
cd ~/ardupilot
sim_vehicle.py -v ArduCopter --no-mavproxy
```

This starts:
- ArduCopter firmware in software simulation
- TCP server on port **5760** (primary) and **5762** (secondary)
- Default home location: **-35.3632621, 149.1652299** (Canberra, Australia)

Wait until you see `APM: EKF3 IMU0 is using GPS` (SITL uses simulated GPS
internally; our code overrides this with `--sitl` flag which uses SITL's
`LOCAL_POSITION_NED` as ground-truth odometry).

---

## 3. Run the Mission in SITL

### Demo mission (5 waypoints, fully automatic)

```bash
cd ~/gNNS_drone
python3 -m gnns_drone --sitl --demo
```

This will:
1. Connect to SITL on `tcp:127.0.0.1:5760`
2. Wait for EKF to initialize (15-30 seconds)
3. Arm and take off to 2.5 m
4. Fly to 5 demo waypoints (~11 m apart), landing at each
5. Return home and land

Demo waypoints (around SITL default home):

| WP | GPS | Approx. NED |
|----|-----|-------------|
| 1 | -35.3631621, 149.1653299 | ~11 m NE |
| 2 | -35.3633621, 149.1653299 | ~11 m SE |
| 3 | -35.3633621, 149.1651299 | ~11 m SW |
| 4 | -35.3631621, 149.1651299 | ~11 m NW |
| 5 | -35.3632121, 149.1652299 | ~5.5 m N |

### Interactive mode (type coordinates live)

```bash
python3 -m gnns_drone --sitl --interactive
```

After takeoff, enter commands at the `CMD>` prompt:

```
CMD> 10,5          # Fly to N=10, E=5
CMD> pos           # Show current position
CMD> hover         # Stop and hold
CMD> home          # Fly back to (0,0) and land
CMD> takeoff       # Takeoff again
CMD> land          # Land at current position
CMD> quit          # Land and exit
```

### Custom waypoints (manual GPS input)

```bash
python3 -m gnns_drone --sitl
```

Enter SITL default home when prompted:
```
Home LAT,LON: -35.3632621,149.1652299
```

Then enter up to 5 destination GPS coordinates.

---

## 4. Web Control Panel (SITL)

```bash
python3 -m gnns_drone.web_control --sitl
```

Open `http://localhost:5000` in your browser.

Features:
- Real-time position display and flight map
- Click to fly to any NED coordinate
- Takeoff / Land / Home / Hover buttons
- Quick-fly presets (N10, E10, NE10, etc.)
- Flight log with arrival error and time

### Scan web (alternative)

```bash
cd sitl
python3 scan_web.py --sitl --port 5001
```

Opens `http://localhost:5001` with an area-scanning interface.

---

## 5. SITL Test Scripts

Located in the `sitl/` directory. Each can be run standalone.

### `sitl/quick_flight_test.py`

Basic smoke test: connect, arm, takeoff, hover, land.

```bash
cd ~/gNNS_drone
python3 sitl/quick_flight_test.py
```

### `sitl/gps_mission.py`

Flies a GPS waypoint mission using ArduPilot's built-in waypoint navigation
(not the gNNS PID controller). Good for verifying SITL is working.

```bash
python3 sitl/gps_mission.py
```

### `sitl/integration_test.py`

Runs an integration test of the full gNNS stack in SITL.

```bash
python3 sitl/integration_test.py
```

### `sitl/area_scanner.py`

Area scanning mission with web UI. Divides an area into a grid and flies
a lawn-mower pattern.

```bash
python3 sitl/area_scanner.py
```

### `sitl/lidar_avoider.py`

Tests LiDAR-based obstacle avoidance (requires Gazebo with obstacles).

```bash
python3 sitl/lidar_avoider.py
```

### `arm_test.py` (project root)

Debug arming failures. Connects to SITL, disables proximity/rangefinder
parameters that block arming, then attempts arm:

```bash
python3 arm_test.py
```

Prints `STATUSTEXT` messages from the FC showing why arming failed.

### `hunt_port.py` (project root)

Scans common MAVLink ports to find where SITL is listening:

```bash
python3 hunt_port.py
```

Tries: `udp:127.0.0.1:14550`, `tcp:127.0.0.1:5760`, `tcp:127.0.0.1:5762`, etc.

---

## 6. SITL Connection Details

| Parameter | Value |
|-----------|-------|
| Primary TCP | `tcp:127.0.0.1:5760` |
| Secondary TCP | `tcp:127.0.0.1:5762` |
| UDP (MAVProxy) | `udp:127.0.0.1:14550` |
| Default home | -35.3632621, 149.1652299, 584 m |
| System ID | 1 |
| Component ID | 1 |

The `--sitl` flag automatically:
- Sets connection to `tcp:127.0.0.1:5760`
- Uses SITL ground-truth position as odometry (no real camera needed)
- Requests all data streams at 10 Hz
- Waits for EKF to initialize (up to 60 s)
- Retries arming up to 10 times (EKF may not be ready immediately)

---

## 7. Gazebo Simulation (Optional)

For simulated depth camera, LiDAR, and visual environment.

### 7.1 Install Gazebo 11 + ROS Noetic

```bash
cd ~/gNNS_drone/sitl
chmod +x setup_gazebo.sh
./setup_gazebo.sh
```

### 7.2 Set up depth camera model

```bash
chmod +x setup_depth_camera.sh
./setup_depth_camera.sh
```

Or manually inject a depth camera into the iris model:

```bash
python3 fix_model.py
```

This reads the default Gazebo `iris_with_ardupilot` model and injects:
- Downward-facing depth camera
- 360-degree LiDAR sensor
- Writes to `~/.gazebo/models/iris_with_depth_camera/model.sdf`

### 7.3 Start full simulation

```bash
cd ~/gNNS_drone
chmod +x start_simulation.sh
./start_simulation.sh
```

This script:
1. Kills stale processes (gzserver, arducopter, roscore)
2. Sources ROS Noetic and Gazebo environments
3. Starts `roscore`
4. Launches Gazebo with the `gnns_depth.world` world
5. Starts ArduPilot SITL with Gazebo frame (`-f gazebo-iris`)

### 7.4 Gazebo worlds

| World | Description |
|-------|-------------|
| `sitl/gazebo_worlds/gnns_depth.world` | Open field with depth camera drone |
| `sitl/gazebo_worlds/gnns_obstacles.world` | Field with obstacles for avoidance testing |

### 7.5 Depth camera parameters

Load `sitl/depth_camera_params.parm` into ArduPilot for SITL depth camera:

```bash
# In MAVProxy or Mission Planner
param load sitl/depth_camera_params.parm
```

---

## 8. SITL Workflow Summary

```
Terminal 1                          Terminal 2
──────────                          ──────────
cd ~/ardupilot                      cd ~/gNNS_drone
sim_vehicle.py -v ArduCopter \
  --no-mavproxy
                                    # Wait until SITL shows "EKF3 IMU0"

                                    # Option A: Auto demo
                                    python3 -m gnns_drone --sitl --demo

                                    # Option B: Interactive
                                    python3 -m gnns_drone --sitl -i

                                    # Option C: Web control
                                    python3 -m gnns_drone.web_control --sitl
                                    # Open http://localhost:5000
```

---

## 9. Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| `Connection failed!` | SITL not running | Start `sim_vehicle.py` first |
| `No heartbeat!` | Wrong port | Run `python3 hunt_port.py` to find the right port |
| `ARM FAILED!` | EKF not ready | Wait longer; use `--sitl` flag (retries 10 times) |
| `ARM FAILED!` | Proximity/rangefinder check | Run `arm_test.py` to disable blocking params |
| EKF takes 30+ seconds | Normal for SITL | `--sitl` waits up to 60 s automatically |
| `Fly-to timeout!` | PID gains too low | Increase `kp` in `config/flight_config.yaml` |
| No position data | Streams not requested | `--sitl` calls `request_all_streams(10)` automatically |
| Connection refused on 5762 | Using wrong port | Use 5760 (primary), not 5762 |
| Web page not loading | Wrong URL | Check `http://localhost:5000` (not https) |
