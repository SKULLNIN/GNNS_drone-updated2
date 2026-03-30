# ROS2 VIO Stack Setup (RealSense + ORB-SLAM3 + RTAB-Map)

Exact bring-up order for RealSense, IMU/TF, ORB-SLAM3, and RTAB-Map with gNNS Drone.

## Prerequisites

- ROS2 Humble or Jazzy
- `realsense2_camera` (ROS2)
- ORB-SLAM3 ROS2 wrapper (RGB-D + inertial)
- `rtabmap_launch` (ROS 2; provides `rtabmap.launch.py` on Humble/Jazzy)
- `nav_msgs`, `sensor_msgs`, `tf2_ros`
- RealSense camera (D435i / D455) with IMU source bridged to ROS2

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

Example launch (wrapper-dependent):

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

RTAB-Map runs in parallel for mapping. It does **not** need to publish odometry for FC control (ORB-SLAM3 provides that).

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=camera_link \
  visual_odometry:=false \
  odom_topic:=/odom
```

If `visual_odometry:=false`, RTAB-Map can subscribe to external odom (e.g. from ORB-SLAM3) for mapping.

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
