# ROS2 VIO Stack Setup (RealSense + ORB-SLAM3 + RTAB-Map)

Exact bring-up order for RealSense, IMU/TF, ORB-SLAM3, and RTAB-Map with gNNS Drone.

## Prerequisites

- ROS2 Humble or Jazzy
- `realsense2_camera` (ROS2)
- ORB-SLAM3 ROS2 wrapper (RGB-D + inertial)
- `rtabmap_launch` (ROS 2; provides `rtabmap.launch.py` on Humble/Jazzy)
- `nav_msgs`, `sensor_msgs`, `tf2_ros`
- RealSense camera (D435i / D455) with IMU source bridged to ROS2

## RealSense topic namespace (Jetson / gNNS scripts)

Many examples below use a **single** prefix such as `/camera/color/...`. The Jetson launch scripts use `camera_name:=camera` and `camera_namespace:=camera`, which yields a **double** segment **`/camera/camera/...`**:

| Stream | Typical topic |
|--------|----------------|
| Color | `/camera/camera/color/image_raw` |
| Depth (aligned to color) | `/camera/camera/aligned_depth_to_color/image_raw` |
| `camera_info` (use with aligned depth) | `/camera/camera/color/camera_info` |
| Fused IMU (`unite_imu_method:=2`) | `/camera/camera/imu` |

Remap any generic `/camera/...` command to these paths when using `gnns_vio_stack.sh` or `d455_launch.sh`.

## Three-tier RTAB-Map stack

`rtabmap.launch.py` with **`visual_odometry:=true`** starts **rgbd_odometry** and **rtabmap** in one launch. If IMU wait, timestamp sync, or depth/color pairing fails, **rgbd_odometry** may never bootstrap (e.g. F2M ref `-1`, `quality=0`), so the mapper logs **“no odometry is provided”** and ignores images.

The project script **splits** the pipeline into three processes:

1. **RealSense** — color, aligned depth, gyro/accel, fused IMU.
2. **Standalone `rgbd_odometry`** (`ros2 run rtabmap_odom rgbd_odometry`) — publishes `nav_msgs/Odometry` on **`GNNS_ODOM_TOPIC`** (default `/odom`) and TF `odom`→`camera_link`. This node uses **`wait_imu_to_init:=false`** so visual odometry is not blocked while the IMU warms up.
3. **RTAB-Map SLAM** — **`visual_odometry:=false`**, subscribes to that odometry topic and runs mapping / loop closure. IMU is still passed to RTAB-Map for graph constraints when configured.

Run: `./scripts/jetson_nano/gnns_vio_stack.sh stack`  
Logs (default `GNNS_LOG_DIR=/tmp`): `gnns_realsense.log`, `gnns_rgbd_odometry.log`, `gnns_rtabmap.log`.

## Bring-up Order

### 1. RealSense Camera

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Launch RealSense (adjust namespace if needed)
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=false \
  align_depth.enable:=true
```

Ensure topics are published:

- `.../color/image_raw`
- `.../depth/image_rect_raw` or `.../aligned_depth_to_color/image_raw`
- `.../color/camera_info`
- `.../imu` (if using D435i with built-in IMU)

### 2. IMU + TF (Static Transforms)

If IMU is from a separate source (e.g., FC via MAVLink→ROS2 bridge), publish it as `sensor_msgs/Imu`.

Publish static transforms so ORB-SLAM3 knows camera ↔ IMU and camera ↔ base_link:

```bash
# Example: camera_link is identity to imu_link (if IMU is in camera body)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_link imu_link

# camera_link relative to base_link (drone body)
ros2 run tf2_ros static_transform_publisher 0.05 0 0 0 0 0 base_link camera_link
```

Adjust values per your mechanical layout.

### 3. ORB-SLAM3 ROS2 Wrapper (RGB-D + Inertial)

Configure your ORB-SLAM3 ROS2 wrapper for RGB-D + inertial mode:

- **Image topics**: RealSense color and depth (aligned)
- **Camera info**: RealSense color camera_info
- **IMU topic**: `sensor_msgs/Imu` (e.g. `/camera/imu` or `/imu/data`)
- **Odometry output**: `nav_msgs/Odometry` on `/odom` (or set `odom_topic` in `config/vio_config.yaml`)

Example launch (wrapper-dependent; **Jetson** users: use `/camera/camera/...` as in the table above):

```bash
ros2 launch orbslam3_ros rgbd_inertial.launch.py \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  imu_topic:=/camera/imu
```

Verify:

```bash
ros2 topic hz /odom
ros2 topic echo /odom --no-arr
```

### 4. RTAB-Map (Mapping / Loop Closure)

**Pattern A — External odometry (ORB-SLAM3, another VIO node):** RTAB-Map does **not** run its own `rgbd_odometry`. Set **`visual_odometry:=false`** and point **`odom_topic`** at the same `/odom` (or custom topic) your VIO publishes.

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=camera_link \
  visual_odometry:=false \
  odom_topic:=/odom
```

**Pattern B — RTAB-Map’s own RGB-D odometry:** **`visual_odometry:=true`** (default in many tutorials). That starts **`rgbd_odometry` + `rtabmap`** together. For a more robust bring-up on Jetson, use **Pattern C**.

**Pattern C — gNNS split stack (Jetson):** Run **`./scripts/jetson_nano/gnns_vio_stack.sh stack`** so **`rgbd_odometry`** and **RTAB-Map SLAM** are separate processes (see [Three-tier RTAB-Map stack](#three-tier-rtab-map-stack)). FC / `vio_config.yaml` still use **`odom_topic: /odom`** by default.

### 5. gNNS Drone Mission

Set `config/vio_config.yaml`:

```yaml
ros2_odom:
  odom_source: "orbslam3"
  odom_topic: "/odom"
  odom_frame_convention: "ros_enu_to_ned"
```

Run the mission:

```bash
python -m gnns_drone --vio-source orbslam3
# Or rely on config: python -m gnns_drone
```

## Verification

1. `ros2 topic hz /odom` — stable rate (e.g. 30 Hz)
2. `ros2 topic echo /odom` — pose and twist in meters/radians
3. Run mission and check EKF convergence in `EKF_STATUS_REPORT`
4. If axes are wrong, adjust `odom_frame_convention` or FC `VISO_ORIENT`

---

## Calibration and EKF Tuning

Align `config/fc_params.yaml` (or Mission Planner params) with your VIO setup.

### VISO_ORIENT (Camera Orientation)

Matches camera mounting to NED. Values: `0` (default), `2` (Yaw270), `4` (Yaw90), `6` (Yaw180).

- Camera faces **forward**: usually `0`
- Camera faces **right**: `4`
- Camera faces **back**: `6`
- Camera faces **left**: `2`

Verify by flying forward; NED North should increase in the direction of travel.

### VISO_POS_X, VISO_POS_Y, VISO_POS_Z

Offset (meters) from vehicle center of gravity to the camera, in body frame:

- **X**: forward (positive = camera ahead of CG)
- **Y**: right (positive = camera to right of CG)
- **Z**: down (positive = camera below CG)

Measure mechanically or from CAD.

### VISO_DELAY_MS

Processing delay from image capture to VISION_POSITION_ESTIMATE.

To measure:

1. Log timestamps: image capture time vs. time when MAVLink message is sent
2. Typical ORB-SLAM3: 30–80 ms
3. Start with 50 ms; increase if EKF shows high innovation, decrease if sluggish

### EK3_EXTNAV_NOISE

How much the EKF trusts external nav (VIO) vs. IMU.

- **Lower (0.1–0.3)**: EKF trusts VIO more — use when VIO is stable
- **Higher (0.5–1.0)**: EKF filters VIO more — use when VIO is noisy

### EK3_EXTNAV_DELAY

Matches `VISO_DELAY_MS` (in milliseconds). Keep them aligned.

---

## Troubleshooting: IMU / "bad optional access"

If you see crashes or logs mentioning **optional access** / **IMU** while using RealSense + RTAB-Map:

1. **Enable IMU in the RealSense node** — a launch that only sets `enable_color`, `enable_depth`, and `align_depth` does **not** publish gyro/accel. RTAB-Map (or the driver) may then access IMU data that was never enabled.

   ```bash
   ros2 launch realsense2_camera rs_launch.py \
     align_depth.enable:=true \
     enable_color:=true \
     enable_depth:=true \
     enable_gyro:=true \
     enable_accel:=true \
     unite_imu_method:=2
   ```

   `unite_imu_method:=2` publishes a single fused `sensor_msgs/Imu` (typically `.../imu`) at a stable rate.

2. **Verify the topic** before starting RTAB-Map:

   ```bash
   ros2 topic hz /camera/camera/imu
   ros2 topic echo /camera/camera/imu --once
   ```

   Adjust the path to match your `camera_name` / `camera_namespace`.

3. **If you intentionally run visual odometry only** (no IMU fusion), disable IMU wait in RTAB-Map:

   ```bash
   export GNNS_WAIT_IMU=0
   # In launch: wait_imu_to_init:=false
   ```

   The project script `scripts/jetson_nano/gnns_vio_stack.sh` documents `GNNS_WAIT_IMU`.

4. **Firmware / USB** — intermittent IMU can still cause edge-case errors; use a powered USB3 hub, short cable, and current RealSense firmware.

---

## Troubleshooting: "Not enough inliers 0/20" and "no odometry is provided"

Symptoms in the console:

- `rgbd_odometry`: `Registration failed: Not enough inliers 0/20 (matches=...) between -1 and ...`
- `Odom: quality=0`
- `rtabmap`: `RGB-D SLAM mode is enabled ... no odometry is provided. Image 0 is ignored!`
- RViz: fixed frame `odom` but **empty** 3D view (no point cloud / trajectory)

**What it means:** Feature matching finds correspondences (`matches≈50`), but **geometric verification** (pose from RGB + depth) yields **zero** inliers. The `-1` in `between -1 and N` is an invalid / empty **map** reference in Frame-to-Map mode — odometry never bootstraps, so RTAB-Map gets no `/odom`.

**Typical causes:**

1. **Depth ↔ color misalignment** — use **aligned depth to color** for both depth image and the same `camera_info` as color (`aligned_depth_to_color` + `color/camera_info`).
2. **Poor sync** — RGB and depth timestamps too far apart; try smaller `approx_sync_max_interval` or fix frame rate (e.g. 30 Hz for both).
3. **Low light / motion blur** — increase room light or use auto-exposure; avoid moving the camera until VO locks.
4. **Frame-to-Map too strict at start** — try **Frame-to-Frame** odometry and **ORB** features with lower `Vis/MinInliers` (see `gnns_vio_stack.sh` preset `recovery`).

**What to run (project script):**

```bash
# Default is now preset=recovery (F2F + ORB + MinInliers 8). To restore your old F2M/FAST tuning:
GNNS_RTABMAP_PRESET=default ./scripts/jetson_nano/gnns_vio_stack.sh stack
```

**Quick checks:**

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
# Rates should match (e.g. both 15 Hz or both 30 Hz).

ros2 topic echo /camera/camera/color/camera_info --once
ros2 run rqt_image_view rqt_image_view
# Inspect depth: invalid pixels should not cover the whole scene.
```

If problems persist, temporarily use **depth** `image_rect_raw` **with** `depth/camera_info` (not aligned) only if you match RTAB-Map topics consistently — the usual fix is **aligned depth + color camera_info** as in `gnns_vio_stack.sh`.
