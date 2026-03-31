#!/usr/bin/env bash
# =============================================================================
# gnns_vio_stack.sh — RealSense + RTAB-Map + gNNS VIO (Jetson / Ubuntu)
# =============================================================================
#
# Integrates your working RTAB-Map launch with the fixes needed for stable IMU:
#
#   "imu bad optional access"  (librealsense / RTAB-Map)
#
# Root cause (typical):
#   • RealSense node launched WITHOUT gyro/accel → no /camera/camera/imu
#   • RTAB-Map waits for IMU (wait_imu_to_init) or tries to fuse missing data
#   • Driver code accesses std::optional IMU frames that were never enabled
#
# Fix:
#   • enable_gyro:=true enable_accel:=true unite_imu_method:=2
#   • This publishes a fused sensor_msgs/Imu on .../imu at stable rate
#
# Usage (from repo root gNNS_drone/):
#
#   chmod +x scripts/jetson_nano/gnns_vio_stack.sh
#
#   # 1) Camera only (debug topics)
#   ./scripts/jetson_nano/gnns_vio_stack.sh realsense
#
#   # 2) RTAB-Map only (after realsense is running in another terminal)
#   ./scripts/jetson_nano/gnns_vio_stack.sh rtabmap
#
#   # 3) Full stack: RealSense (bg) + RTAB-Map (fg) — Ctrl+C stops both
#   ./scripts/jetson_nano/gnns_vio_stack.sh stack
#
#   # 4) gNNS Python mission (needs /odom from RTAB-Map; run after stack)
#   ./scripts/jetson_nano/gnns_vio_stack.sh mission
#   ./scripts/jetson_nano/gnns_vio_stack.sh mission --demo
#
# Environment overrides:
#   ROS_DISTRO=humble
#   ROS_DOMAIN_ID=42
#   GNNS_WAIT_IMU=1          # 0 = RTAB-Map visual-only init (no IMU wait)
#   GNNS_RS_FPS=15           # 15 or 30 (must match your CPU budget)
#   GNNS_PROJECT_ROOT        # path to gNNS_drone parent for mission step
#
# =============================================================================
# Do not use set -u — ROS setup.bash references unset variables.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JETSON_DIR="$SCRIPT_DIR"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-humble}"
SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "$SETUP" ]]; then
  echo "[gnns_vio_stack] ERROR: Missing $SETUP — install ROS 2 ${ROS_DISTRO}" >&2
  exit 2
fi
# shellcheck source=/dev/null
source "$SETUP"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [[ -f "$JETSON_DIR/fastdds_wifi_env.sh" ]]; then
  # shellcheck source=/dev/null
  source "$JETSON_DIR/fastdds_wifi_env.sh"
fi

# -----------------------------------------------------------------------------
# Topic layout — matches RealSense: camera_name:=camera camera_namespace:=camera
# -----------------------------------------------------------------------------
CAM_NS="/camera/camera"
RGB_TOPIC="${GNNS_RGB_TOPIC:-${CAM_NS}/color/image_raw}"
DEPTH_TOPIC="${GNNS_DEPTH_TOPIC:-${CAM_NS}/aligned_depth_to_color/image_raw}"
INFO_TOPIC="${GNNS_INFO_TOPIC:-${CAM_NS}/color/camera_info}"
IMU_TOPIC="${GNNS_IMU_TOPIC:-${CAM_NS}/imu}"

# RTAB-Map waits for first IMU by default; set GNNS_WAIT_IMU=0 if you insist on VO-only
WAIT_IMU="${GNNS_WAIT_IMU:-1}"
if [[ "$WAIT_IMU" == "1" ]]; then
  WAIT_IMU_INIT="true"
else
  WAIT_IMU_INIT="false"
fi

# Frame rate (15 saves CPU on Nano; 30 is smoother for VIO)
RS_FPS="${GNNS_RS_FPS:-15}"
if [[ "$RS_FPS" == "15" ]]; then
  RS_PROFILE="640x480x15"
elif [[ "$RS_FPS" == "30" ]]; then
  RS_PROFILE="640x480x30"
else
  RS_PROFILE="640x480x${RS_FPS}"
fi

# -----------------------------------------------------------------------------
# RTAB-Map CLI args (your tuned set; override with RTABMAP_EXTRA_ARGS)
# -----------------------------------------------------------------------------
RTABMAP_ARGS_BASE="--delete_db_on_start
--Vis/MaxFeatures 500
--Vis/MinInliers 15
--Vis/EstimationType 1
--Vis/MotionThreshold 0.5
--Kp/MaxFeatures 500
--Kp/DetectorStrategy 6
--Kp/MaxDepth 8.0
--Kp/MinDepth 0.3
--OdoF2M/MaxSize 1500
--OdoF2M/BundleAdjustment 1
--Odom/Strategy 0
--Odom/GuessMotion true
--Odom/FilteringStrategy 1
--Odom/Holonomic false
--Mem/STMSize 20
--Mem/RehearsalSimilarity 0.45
--Rtabmap/TimeThr 0
--Rtabmap/DetectionRate 1
--RGBD/ProximityBySpace true
--RGBD/LinearSpeedUpdate 0.05
--RGBD/AngularSpeedUpdate 0.05"

RTAB_RARGS="${RTABMAP_ARGS_BASE//[$'\n']/ }"
if [[ "${RTABMAP_KEEP_DB:-0}" == "1" ]]; then
  RTAB_RARGS="${RTAB_RARGS/--delete_db_on_start/}"
fi
if [[ -n "${RTABMAP_EXTRA_ARGS:-}" ]]; then
  RTAB_RARGS="${RTAB_RARGS} ${RTABMAP_EXTRA_ARGS}"
fi

# Common RealSense launch arguments (IMU required — fixes "bad optional access")
RS_LAUNCH_ARGS=(
  align_depth.enable:=true
  enable_color:=true
  enable_depth:=true
  enable_gyro:=true
  enable_accel:=true
  unite_imu_method:=2
  depth_module.infra_profile:=0x0x0
  "rgb_camera.color_profile:=${RS_PROFILE}"
  "depth_module.depth_profile:=${RS_PROFILE}"
  camera_name:=camera
  camera_namespace:=camera
)

# -----------------------------------------------------------------------------
# RealSense: MUST enable IMU to avoid optional-access / missing-IMU failures
# -----------------------------------------------------------------------------
launch_realsense() {
  echo "[gnns_vio_stack] Launching RealSense with IMU (gyro+accel, fused IMU topic)…"
  echo "  Topics: ${RGB_TOPIC}, ${DEPTH_TOPIC}, ${IMU_TOPIC}"
  exec ros2 launch realsense2_camera rs_launch.py "${RS_LAUNCH_ARGS[@]}"
}

# Background RealSense (for stack mode — do NOT use exec)
launch_realsense_bg() {
  ros2 launch realsense2_camera rs_launch.py "${RS_LAUNCH_ARGS[@]}" &
  echo $!
}

# -----------------------------------------------------------------------------
# RTAB-Map (no exec — so stack mode can trap and kill children)
# -----------------------------------------------------------------------------
launch_rtabmap() {
  echo "[gnns_vio_stack] Launching RTAB-Map…"
  echo "  rgb=${RGB_TOPIC} depth=${DEPTH_TOPIC} imu=${IMU_TOPIC} wait_imu_to_init=${WAIT_IMU_INIT}"
  # shellcheck disable=SC2086
  ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:="${RGB_TOPIC}" \
    depth_topic:="${DEPTH_TOPIC}" \
    camera_info_topic:="${INFO_TOPIC}" \
    imu_topic:="${IMU_TOPIC}" \
    frame_id:=camera_link \
    approx_sync:=true \
    approx_sync_max_interval:=0.5 \
    queue_size:=20 \
    wait_imu_to_init:=${WAIT_IMU_INIT} \
    visual_odometry:=true \
    odom_frame_id:=odom \
    map_frame_id:=map \
    publish_tf_map:=true \
    publish_tf_odom:=true \
    odom_topic:=/odom \
    rtabmap_viz:=true \
    rtabmap_args:="${RTAB_RARGS}"
}

# -----------------------------------------------------------------------------
# Full stack: RealSense background + RTAB-Map foreground
# -----------------------------------------------------------------------------
launch_stack() {
  echo "[gnns_vio_stack] Starting full stack (RealSense → RTAB-Map)…"
  RS_PID="$(launch_realsense_bg)"

  cleanup() {
    echo "[gnns_vio_stack] Shutting down…"
    kill "${RS_PID}" 2>/dev/null || true
    wait "${RS_PID}" 2>/dev/null || true
  }
  trap cleanup EXIT INT TERM

  echo "[gnns_vio_stack] RealSense PID=${RS_PID}"
  echo "[gnns_vio_stack] Waiting for camera + IMU topics (8 s)…"
  sleep 8

  # Foreground RTAB-Map; on exit, trap kills RealSense
  launch_rtabmap
  cleanup
  trap - EXIT INT TERM
}

# -----------------------------------------------------------------------------
# gNNS Python mission (uses RTABMapOdom ros2 mode → /odom)
# -----------------------------------------------------------------------------
launch_mission() {
  local ROOT="${GNNS_PROJECT_ROOT:-$PROJECT_ROOT}"
  export PYTHONPATH="${ROOT}:${PYTHONPATH:-}"
  cd "$ROOT"
  echo "[gnns_vio_stack] gNNS mission from ${ROOT}"
  echo "  Ensure config/vio_config.yaml has odom_topic: /odom (or match your remap)"
  # Pass remaining args to gnns_drone (e.g. --demo --vio-source ros2)
  exec python3 -m gnns_drone --vio-source ros2 "$@"
}

# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
MODE="${1:-help}"
shift || true

case "$MODE" in
  realsense)
    launch_realsense
    ;;
  rtabmap)
    launch_rtabmap
    ;;
  stack)
    launch_stack
    ;;
  mission)
    launch_mission "$@"
    ;;
  help|--help|-h)
    sed -n '1,45p' "$0" | sed 's/^# \{0,1\}//'
    echo ""
    echo "Quick check after 'realsense' terminal is up:"
    echo "  ros2 topic hz ${IMU_TOPIC}"
    echo "  ros2 topic echo ${IMU_TOPIC} --once"
    echo ""
    echo "If IMU still errors, try:"
    echo "  GNNS_WAIT_IMU=0 ./scripts/jetson_nano/gnns_vio_stack.sh rtabmap"
    ;;
  *)
    echo "Unknown mode: $MODE — use: realsense | rtabmap | stack | mission | help" >&2
    exit 1
    ;;
esac
