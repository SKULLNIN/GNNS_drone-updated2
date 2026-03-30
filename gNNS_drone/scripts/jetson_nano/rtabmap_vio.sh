#!/usr/bin/env bash
# ================================================================
# rtabmap_vio.sh — RTAB-Map visual odometry + SLAM (ROS 2)
# ================================================================
# Expects D455 topics from d455_launch.sh (/camera/camera/...).
# Publishes /odom, /map, TF, and rtabmap topics.
#
# Usage:
#   ./scripts/jetson_nano/rtabmap_vio.sh
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
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "$HOME/ros2_ws/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

exec ros2 launch rtabmap_launch rtabmap.launch.py \
  visual_odometry:=true \
  frame_id:=camera_link \
  odom_frame_id:=odom \
  map_frame_id:=map \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  imu_topic:=/camera/camera/imu \
  wait_imu_to_init:=true \
  approx_sync:=false \
  publish_tf_map:=true \
  publish_tf_odom:=true \
  odom_topic:=/odom \
  map_topic:=/map \
  rtabmap_viz:=false
