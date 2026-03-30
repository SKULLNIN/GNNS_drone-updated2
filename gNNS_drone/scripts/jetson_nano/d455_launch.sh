#!/usr/bin/env bash
# ================================================================
# d455_launch.sh — RealSense D455 (ROS 2) with IMU + depth + pointcloud
# ================================================================
# Matches camera_namespace used by rtabmap_vio.sh (/camera/camera/...).
# Run on Jetson before rtabmap_vio.sh.
#
# Usage:
#   ./scripts/jetson_nano/d455_launch.sh
# ================================================================
# Do not use set -u — ROS setup.bash references unset variables.
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "$SETUP" ]]; then
    echo "[Error] Missing $SETUP" >&2
    exit 2
fi

# shellcheck source=/dev/null
source "$SETUP"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# shellcheck source=/dev/null
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/fastdds_wifi_env.sh"

exec ros2 launch realsense2_camera rs_launch.py \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  enable_depth:=true \
  enable_color:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  pointcloud.enable:=true \
  depth_module.profile:=640x480x30 \
  rgb_camera.profile:=640x480x30 \
  camera_name:=camera \
  camera_namespace:=camera
