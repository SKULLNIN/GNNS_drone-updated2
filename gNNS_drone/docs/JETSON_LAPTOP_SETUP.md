# Jetson-to-Laptop Setup Guide

Stream RealSense camera, odometry, and SLAM data from the Jetson to a laptop
running RViz2 over WiFi. Two parallel systems work together:

- **SSH** gives you a live shell into the Jetson for starting/stopping nodes.
- **ROS 2 DDS** automatically streams all topics to any machine on the same
  network with the same `ROS_DOMAIN_ID`.

```
Jetson (D455 + RTAB-Map + gNNS)          Laptop (RViz2)
  publishes /camera/*, /odom, /tf   --->   subscribes automatically
            DDS multicast (domain 42)
```

---

## 1. Network Setup

### Option A: Jetson creates a WiFi hotspot (competition day)

```bash
# On Jetson
nmcli device wifi hotspot ifname wlan0 ssid "DroneNet" password "drone1234"

# Auto-start on boot
nmcli connection modify Hotspot connection.autoconnect yes

# Check Jetson IP (usually 10.42.0.1)
ip addr show wlan0
```

On laptop, connect to `DroneNet` WiFi then verify:

```bash
ping 10.42.0.1
```

### Option B: Both on the same existing WiFi

```bash
# On Jetson
hostname -I
# Example output: 192.168.1.100

# On laptop
ping 192.168.1.100
```

---

## 2. SSH into Jetson

### Basic

```bash
ssh jetson@10.42.0.1          # hotspot
ssh jetson@192.168.1.100      # shared WiFi
```

### With tmux (survives WiFi drops)

```bash
ssh jetson@10.42.0.1
tmux new -s drone

# If WiFi drops later
ssh jetson@10.42.0.1
tmux attach -t drone          # everything still running
```

---

## 3. One-Time Jetson Installation

```bash
cd ~/gNNS_drone

# Python deps + serial port
chmod +x scripts/jetson_nano/setup_jetson_nano.sh
./scripts/jetson_nano/setup_jetson_nano.sh

# ROS 2 Humble
# https://docs.ros.org/en/humble/Installation.html

# RealSense ROS 2 driver
sudo apt install ros-humble-realsense2-camera

# RTAB-Map ROS 2 (launch files are in rtabmap_launch on Humble)
sudo apt install ros-humble-rtabmap-launch

# Fast DDS RMW (default on Humble; install if minimal ROS image)
sudo apt install ros-humble-rmw-fastrtps-cpp

# Verify D455 is detected
rs-enumerate-devices | head -5
```

## 4. One-Time Laptop Installation

```bash
# ROS 2 Humble
# https://docs.ros.org/en/humble/Installation.html

# Visualization
sudo apt install ros-humble-rviz2 ros-humble-rqt ros-humble-image-transport-plugins

# Fast DDS RMW (match Jetson)
sudo apt install ros-humble-rmw-fastrtps-cpp
```

---

## 5. DDS environment (match on both machines)

Scripts set these for you. For `~/.bashrc`, use:

| Variable | Value | Purpose |
|----------|-------|---------|
| `ROS_DOMAIN_ID` | `42` | Same domain on Jetson and laptop |
| `ROS_LOCALHOST_ONLY` | `0` | Required for cross-machine discovery |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | Same RMW on both (default Humble) |

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

## 6. Simple workflow (recommended)

Use two terminals on the **Jetson**, then RViz on the **laptop**. Topic prefix is `/camera/camera/...` (see `d455_launch.sh`).

```bash
# On Jetson — terminal 1
cd ~/gNNS_drone
chmod +x scripts/jetson_nano/d455_launch.sh scripts/jetson_nano/rtabmap_vio.sh
./scripts/jetson_nano/d455_launch.sh

# On Jetson — terminal 2 (after camera is up)
./scripts/jetson_nano/rtabmap_vio.sh

# On laptop
cd ~/gNNS_drone
chmod +x scripts/jetson_nano/laptop_rviz2.sh scripts/jetson_nano/verify_bridge.sh
./scripts/jetson_nano/verify_bridge.sh
./scripts/jetson_nano/laptop_rviz2.sh 42
```

RViz loads [`config/drone_monitor.rviz`](../config/drone_monitor.rviz) when present. If you see **Permission denied**, run `chmod +x` on the script (Git clones may not preserve `+x`).

---

## 7. Optional: all-in-one tmux — `ros_bridge.sh`

`scripts/jetson_nano/ros_bridge.sh` starts everything inside tmux.

### Full mission (camera + VIO + gNNS flight)

```bash
cd ~/gNNS_drone
./scripts/jetson_nano/ros_bridge.sh --imu
```

### Visualization only (camera + VIO, no flight)

```bash
./scripts/jetson_nano/ros_bridge.sh --viz-only --imu --pointcloud
```

### Dry run (show commands without ROS)

```bash
./scripts/jetson_nano/ros_bridge.sh --print-only --viz-only --imu --pointcloud
```

### All flags

| Flag | Description |
|------|-------------|
| `--viz-only` | Skip the gNNS mission, only stream camera + VIO |
| `--imu` | Enable gyro and accelerometer streams |
| `--pointcloud` | Enable depth point cloud |
| `--print-only` | Print what would run, do not start anything |
| `--no-tmux` | Run foreground (for systemd or Docker) |
| `--domain-id N` | Override `ROS_DOMAIN_ID` (default 42) |
| `--rmw NAME` | Override DDS implementation |
| `--odom-source S` | `ros2` (RTAB-Map) or `orbslam3` |
| `--distro NAME` | ROS distro: `humble`, `jazzy`, etc. |
| `--mavlink-out IP:PORT` | Forward MAVLink to laptop for QGroundControl |
| `--session NAME` | tmux session name (default `drone`) |
| `--light-maps` | Lower RTAB-Map cloud bandwidth (`cloud_decimation 8`) |
| `--rtabmap-gui` | Enable `rtabmap_viz` on Jetson (default off, saves GPU/CPU) |
| `--cloud-map` / `--grid-map` | No-op flags (maps publish by default; for script clarity) |

### tmux layout

| Window | Panes |
|--------|-------|
| `ros` | Left: RealSense camera, Right: RTAB-Map or ORB-SLAM3 |
| `mission` | gNNS drone mission (omitted with `--viz-only`) |
| `mavproxy` | MAVLink forwarding (only with `--mavlink-out`) |

Attach: `tmux attach -t drone`
Kill: `tmux kill-session -t drone`

---

## 8. `laptop_rviz2.sh` / `verify_bridge.sh` (reference)

- **`laptop_rviz2.sh`** — optional first argument: domain id (e.g. `42`). Set `ROS_DISTRO` if not `humble`. Lists topics, then starts RViz2 with `config/drone_monitor.rviz` when present.
- **`verify_bridge.sh`** — optional first argument: domain id. Prints `ros2 topic list` and samples `/odom` and camera RGB (`/camera/camera/...` or `/camera/...`).

---

## 9. Manual RViz2 Setup

If you prefer to set up manually instead of using the helper script:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic list              # verify Jetson topics are visible
ros2 run rviz2 rviz2
```

### RViz2 display configuration

Set **Fixed Frame** to `odom`, then add these displays:

| Display Type | Topic | What You See |
|-------------|-------|-------------|
| Image | `/camera/camera/color/image_raw` | Live RGB (with `d455_launch.sh`) |
| PointCloud2 | `/camera/camera/depth/color/points` | Depth cloud |
| Odometry | `/odom` | VIO trail |
| TF | (all) | Coordinate frame tree |
| Map | `/rtabmap/grid_map` or `/map` | 2D occupancy |

Save your config: **File > Save Config As > `drone_monitor.rviz`**

Next time: `ros2 run rviz2 rviz2 -d ~/drone_monitor.rviz`

---

## 10. Verify Data Flow

Run these from the laptop to confirm streaming:

```bash
# List all topics from Jetson
ros2 topic list

# Expected topics (at minimum)
#   /camera/color/image_raw
#   /camera/depth/image_rect_raw
#   /camera/color/camera_info
#   /odom
#   /tf
#   /tf_static

# Check publish rates
ros2 topic hz /camera/color/image_raw
ros2 topic hz /odom

# Watch live odometry
ros2 topic echo /odom --no-arr

# Measure bandwidth per topic
ros2 topic bw /camera/color/image_raw
ros2 topic bw /odom
```

---

## 11. Bandwidth Management

Raw camera over WiFi can be 30-60 MB/s. If WiFi is slow:

### Throttle camera to 5 fps for laptop

```bash
# On Jetson (full rate stays on Jetson, laptop gets 5 Hz)
ros2 run topic_tools throttle messages \
  /camera/color/image_raw 5.0 \
  /camera/color/image_raw/throttled
```

### Compressed transport

```bash
# On Jetson
ros2 run image_transport republish raw \
  --ros-args --remap in:=/camera/color/image_raw \
             --remap out/compressed:=/camera/color/compressed
```

On laptop RViz2, subscribe to `/camera/color/compressed` using the
`image_transport` compressed plugin.

### Other tips

- Omit `--pointcloud` to skip the heavy point cloud stream.
- Reduce camera resolution in `realsense_cmd` launch args if needed.
- Use `ros2 topic bw` to identify which topics consume the most bandwidth.

---

## 12. QGroundControl (Optional)

Forward MAVLink from the Jetson to the laptop for a flight HUD:

```bash
# On Jetson
./scripts/jetson_nano/ros_bridge.sh --mavlink-out 10.42.0.50:14550

# On laptop
# Open QGroundControl — it auto-connects on UDP 14550
# Shows: attitude HUD, battery %, flight mode, altitude
```

---

## 13. Troubleshooting

| Problem | Fix |
|---------|-----|
| `ros2 topic list` is empty on laptop | Check: same `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=0`, same `RMW_IMPLEMENTATION` on both machines |
| Topics visible but no data | Firewall: `sudo ufw allow in proto udp` or disable ufw temporarily |
| RealSense not found | `rs-enumerate-devices`; check USB; try `sudo` |
| RTAB-Map crashes | Verify camera topics: `ros2 topic hz /camera/camera/color/image_raw` (or `/camera/color/...` if using `ros_bridge.sh` only) |
| WiFi drops mid-flight | tmux keeps Jetson processes alive; just SSH back and `tmux attach -t drone` |
| High latency on RViz2 | Omit `--pointcloud`; throttle camera; use compressed transport |
| DDS not discovering | Ensure both machines are on the same subnet; try `ros2 multicast receive` |

---

## 14. Quick Reference Card

```bash
# === JETSON (simple) ===
cd ~/gNNS_drone
./scripts/jetson_nano/d455_launch.sh      # terminal 1
./scripts/jetson_nano/rtabmap_vio.sh      # terminal 2

# === JETSON (tmux all-in-one) ===
./scripts/jetson_nano/ros_bridge.sh --viz-only --imu --pointcloud

# === LAPTOP ===
cd ~/gNNS_drone
chmod +x scripts/jetson_nano/laptop_rviz2.sh scripts/jetson_nano/verify_bridge.sh
./scripts/jetson_nano/verify_bridge.sh
./scripts/jetson_nano/laptop_rviz2.sh 42
ros2 topic hz /odom
```
