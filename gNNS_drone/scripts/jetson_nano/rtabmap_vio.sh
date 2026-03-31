#!/usr/bin/env bash
# ================================================================
# rtabmap_vio.sh — RTAB-Map visual odometry + SLAM (ROS 2)
# ================================================================
# Expects D455 topics from d455_launch.sh (/camera/camera/...).
# Depth must be aligned to color (same grid as color/camera_info).
# Publishes /odom, /map, TF, and rtabmap topics.
#
# Usage:
#   ./scripts/jetson_nano/rtabmap_vio.sh
#
# Stale or corrupt ~/.ros/rtabmap.db causes VWDictionary "Not found word ... (dict size=0)".
# By default we pass --delete_db_on_start (fresh map each run).
#   export RTABMAP_KEEP_DB=1   # reuse existing database / map
#   export RTABMAP_ARGS="--udebug"   # optional extra rtabmap CLI args (appended)
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

RTAB_RARGS=""
if [[ "${RTABMAP_KEEP_DB:-0}" != "1" ]]; then
    RTAB_RARGS="--delete_db_on_start"
fi
if [[ -n "${RTABMAP_ARGS:-}" ]]; then
    RTAB_RARGS="${RTAB_RARGS:+$RTAB_RARGS }${RTABMAP_ARGS}"
fi

EXTRA=()
if [[ -n "$RTAB_RARGS" ]]; then
    # ros2 launch expects e.g. rtabmap_args:='--delete_db_on_start --udebug'
    EXTRA=( "rtabmap_args:='${RTAB_RARGS}'" )
fi

exec ros2 launch rtabmap_launch rtabmap.launch.py \
  visual_odometry:=true \
  frame_id:=camera_link \
  odom_frame_id:=odom \
  map_frame_id:=map \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  imu_topic:=/camera/camera/imu \
  wait_imu_to_init:=true \
  approx_sync:=true \
  approx_sync_max_interval:=0.5 \
  queue_size:=20 \
  publish_tf_map:=true \
  publish_tf_odom:=true \
  odom_topic:=/odom \
  map_topic:=/map \
  rtabmap_viz:=false \
  "${EXTRA[@]}"
