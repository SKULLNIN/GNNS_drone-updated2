# RTAB-Map VIO stack on Jetson (RealSense + gNNS)

This guide covers **one integrated path**: Intel RealSense → **`rgbd_odometry`** (RTAB-Map VO) → **`rtabmap`** SLAM, driven by **`scripts/jetson_nano/gnns_vio_stack.sh`**.  
Use it on **Jetson Nano / Orin** (or any Ubuntu + ROS 2 + GPU optional; apt RTAB-Map is **CPU**-bound — see [JETSON_LAPTOP_SETUP.md](JETSON_LAPTOP_SETUP.md) §11 for GPU/CUDA notes).

---

## 1. Prerequisites

| Requirement | Notes |
|-------------|--------|
| **OS** | Ubuntu 22.04 (JetPack / desktop) with **ROS 2 Humble** (or set `ROS_DISTRO` if you use another distro and fix package names). |
| **ROS 2 packages** | `ros-$ROS_DISTRO-realsense2-camera`, `ros-$ROS_DISTRO-rtabmap-launch`, `ros-$ROS_DISTRO-rtabmap-odom`, `ros-$ROS_DISTRO-rtabmap-sync`, `ros-$ROS_DISTRO-rtabmap-slam`, `ros-$ROS_DISTRO-tf2-*` as needed. |
| **RealSense** | D435i / D455 with **USB 3**, firmware OK (`rs-enumerate-devices`). |
| **Workspace** | Clone repo so layout is `.../gNNS_drone/` with `gnns_drone/` and `config/` inside (scripts resolve paths from `scripts/jetson_nano/`). |

---

## 2. One-time setup

```bash
cd ~/gNNS_drone   # or your clone path

# Python deps (mission / gNNS)
chmod +x scripts/jetson_nano/setup_jetson_nano.sh
./scripts/jetson_nano/setup_jetson_nano.sh
# or: pip3 install -r requirements.txt

# Stack script
chmod +x scripts/jetson_nano/gnns_vio_stack.sh
```

**ROS 2 environment** (every new shell, or add to `~/.bashrc`):

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Optional WiFi / large DDS frames (if you use the provided helpers):

```bash
source scripts/jetson_nano/fastdds_wifi_env.sh
```

---

## 3. Default topics and frames

Assumes RealSense launch uses **`camera_name:=camera`**, **`camera_namespace:=camera`** (as in the script).

| Item | Default |
|------|---------|
| Color image | `/camera/camera/color/image_raw` |
| Aligned depth | `/camera/camera/aligned_depth_to_color/image_raw` |
| Camera info | `/camera/camera/color/camera_info` |
| IMU | `/camera/camera/imu` |
| Odometry | `/odom` |
| Map / SLAM NS | `rtabmap` (namespace) |
| TF | `odom` → `camera_link` from VO; `map` from RTAB-Map |

Override with `GNNS_*` env vars (see §5).

---

## 4. Main script: `gnns_vio_stack.sh`

**Working directory:** repository root (`gNNS_drone/`).

### Modes

| Command | What it does |
|---------|----------------|
| `./scripts/jetson_nano/gnns_vio_stack.sh stack` | **Recommended:** RealSense (background) → wait for camera → `rgbd_odometry` → wait `/odom` + TF → RTAB-Map SLAM (+ optional `rtabmap_viz` if `DISPLAY` set). |
| `./scripts/jetson_nano/gnns_vio_stack.sh realsense` | Camera + IMU only (foreground). |
| `./scripts/jetson_nano/gnns_vio_stack.sh rtabmap` | Assumes RealSense **already running**; starts `rgbd_odometry` + RTAB-Map only. |
| `./scripts/jetson_nano/gnns_vio_stack.sh mission --demo` | Runs `python3 -m gnns_drone --vio-source ros2` (after you have `/odom`). |
| `./scripts/jetson_nano/gnns_vio_stack.sh diagnose` | Prints filtered `ros2 topic list` + hints for IMU topic. |
| `./scripts/jetson_nano/gnns_vio_stack.sh help` | Dumps script header (env + usage). |

### Typical session (two terminals)

**Terminal A — full stack**

```bash
cd ~/gNNS_drone
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
./scripts/jetson_nano/gnns_vio_stack.sh stack
```

**Terminal B — mission (optional)**

```bash
cd ~/gNNS_drone
source /opt/ros/humble/setup.bash
./scripts/jetson_nano/gnns_vio_stack.sh mission --demo
```

Ensure `config/vio_config.yaml` uses the same **`odom_topic`** (default `/odom`).

---

## 5. Environment variables (reference)

Set **before** running the script. Defaults are chosen for stability on modest CPUs.

### Core / topics

| Variable | Default | Purpose |
|----------|---------|---------|
| `ROS_DISTRO` | `humble` | ROS install under `/opt/ros/$ROS_DISTRO`. |
| `GNNS_LOG_DIR` | `/tmp` | Log directory for RealSense / VO / RTAB-Map. |
| `GNNS_ODOM_TOPIC` | `/odom` | VO output and SLAM input. |
| `GNNS_RGB_TOPIC` / `GNNS_DEPTH_TOPIC` / `GNNS_INFO_TOPIC` | RealSense paths | Override if your topics differ. |
| `GNNS_IMU_TOPIC` | `/camera/camera/imu` | IMU for **rgbd_odometry** (and SLAM if you enable RTAB-Map IMU). |

### IMU / RTAB-Map SLAM IMU

| Variable | Default | Purpose |
|----------|---------|---------|
| `GNNS_WAIT_IMU` | `0` | `0` = do not block VO on IMU (`wait_imu_to_init:=false`). Use `1` only after `ros2 topic hz …/imu` is stable. |
| `GNNS_RTABMAP_IMU` | `0` | `0` = SLAM does **not** subscribe to raw RealSense IMU (avoids “orientation not set” spam). `1` = use `GNNS_IMU_TOPIC` for SLAM (needs filtered IMU with valid orientation). |
| `GNNS_RTABMAP_IMU_TOPIC` | (see script) | Explicit IMU topic for **rtabmap SLAM** only (e.g. Madgwick output). |

### QoS (important)

| Variable | Default | Purpose |
|----------|---------|---------|
| `GNNS_QOS` | `0` | RTAB-Map image QoS (system default; matches RealSense). |
| `GNNS_QOS_IMU` | `2` | RTAB-Map IMU subscriber often **Best Effort**. |
| `GNNS_QOS_ODOM` | `0` | **rgbd_odometry** image QoS — do **not** force `2` here or VO may get **no** images. |

### Performance / latency

| Variable | Default | Purpose |
|----------|---------|---------|
| `GNNS_RS_FPS` | `15` | RealSense color/depth profile (`640x480xN`). Try `30` on faster Jetson if `/odom` stays stable. |
| `GNNS_ODOM_MAX_RATE` | same as `GNNS_RS_FPS` | Caps `rgbd_odometry` **max_update_rate**. |
| `GNNS_RTABMAP_DETECTION_RATE` | (unset; preset uses `2`) | If set, appends `--Rtabmap/DetectionRate N` (loop closure / hypothesis rate). Higher = snappier, more CPU. |
| `GNNS_STACK_CAMERA_WAIT_SEC` | `10` | Sleep after RealSense start before topic checks. |
| `GNNS_APPROX_SYNC_MAX` | `0.10` | RGB–depth sync window (seconds). |
| `GNNS_QUEUE_SIZE` | `10` | Sync / topic queues. |
| `GNNS_WAIT_FOR_TRANSFORM` | `2.0` | RTAB-Map TF wait tolerance (seconds). |
| `GNNS_ODOM_SENSOR_SYNC` | `1` | RTAB-Map `odom_sensor_sync`; try `0` if timestamps fight. |

### Presets and extra args

| Variable | Purpose |
|----------|---------|
| `GNNS_RTABMAP_PRESET` | `default` \| `recovery` \| `accurate` — VO/SLAM tuning (see script). |
| `RTABMAP_EXTRA_ARGS` | Extra RTAB-Map CLI flags (merged into VO + SLAM where applicable). |
| `RTABMAP_KEEP_DB` | `1` = do not pass `--delete_db_on_start`. |

### Visualization / misc

| Variable | Purpose |
|----------|---------|
| `GNNS_RTABMAP_VIZ` | `0` / `1` — force **rtabmap_viz** off or on (if unset, `DISPLAY` set → viz ON). |
| `GNNS_RTABMAP_NAMESPACE` | RTAB-Map ROS namespace (default `rtabmap`). |
| `GNNS_USE_RGBD_SYNC` | `1` = fuse RGB-D with `rgbd_sync` before VO (advanced). |
| `GNNS_RGBD_TOPIC` | Fused topic when `GNNS_USE_RGBD_SYNC=1`. |

---

## 6. Log files

Default: `GNNS_LOG_DIR` (usually `/tmp`).

| File | Content |
|------|---------|
| `gnns_realsense.log` | RealSense node |
| `gnns_rgbd_odometry.log` | `rgbd_odometry` |
| `gnns_rtabmap.log` | RTAB-Map SLAM launch |

Tail while debugging:

```bash
tail -f /tmp/gnns_rgbd_odometry.log
tail -f /tmp/gnns_rtabmap.log
```

---

## 7. Verification commands

```bash
# Topics
ros2 topic list
./scripts/jetson_nano/gnns_vio_stack.sh diagnose

# Rates
ros2 topic hz /odom
ros2 topic hz /camera/camera/color/image_raw

# TF
ros2 run tf2_ros tf2_echo odom camera_link

# One-shot checks
ros2 topic echo /odom --once
```

---

## 8. Laptop + RViz / DDS (optional)

To view the same topics on a **laptop** over WiFi: match `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=0`, and `RMW_IMPLEMENTATION`. See **[JETSON_LAPTOP_SETUP.md](JETSON_LAPTOP_SETUP.md)** for network, `laptop_rviz2.sh`, `verify_bridge.sh`, and bandwidth tips.

Legacy **two-script** flow (camera in one terminal, RTAB in another):

```bash
./scripts/jetson_nano/d455_launch.sh
./scripts/jetson_nano/rtabmap_vio.sh
```

Prefer **`gnns_vio_stack.sh stack`** for a single coordinated bring-up.

---

## 9. Lag / CPU note

If the stack feels **laggy**, apt RTAB-Map runs on **CPU**. Quick mitigations: lower `GNNS_RS_FPS`, set `GNNS_RTABMAP_VIZ=0`, avoid raising `GNNS_RTABMAP_DETECTION_RATE` until CPU headroom is confirmed. Tuning can be refined later; see script header and §5.

---

## 10. Related documentation

| Doc | Topic |
|-----|--------|
| [JETSON_LAPTOP_SETUP.md](JETSON_LAPTOP_SETUP.md) | Jetson ↔ laptop, DDS, RViz, bandwidth, **GPU/CUDA vs CPU** |
| [ROS2_VIO_SETUP.md](ROS2_VIO_SETUP.md) | Bring-up order, calibration, EKF / FC integration |
| [MISSION_GUIDE.md](MISSION_GUIDE.md) | Full mission, hardware, safety |
| [scripts/jetson_nano/README.md](../scripts/jetson_nano/README.md) | Jetson scripts index + `run_mission.sh` |

---

## 11. Quick copy-paste (Jetson)

```bash
cd ~/gNNS_drone
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
chmod +x scripts/jetson_nano/gnns_vio_stack.sh
./scripts/jetson_nano/gnns_vio_stack.sh stack
```

In another terminal:

```bash
ros2 topic hz /odom
```
