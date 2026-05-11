#!/usr/bin/env bash
# =============================================================================
# gnns_vio_stack.sh — RealSense + RTAB-Map + gNNS VIO (Jetson / Ubuntu)
# =============================================================================
#
# Common issues fixed in this script:
#
#  (0) Combined RTAB-Map launch (visual_odometry:=true) starts rgbd_odometry + rtabmap
#      in one process tree — IMU wait / sync races can leave VO ref=-1 and kill /odom.
#      Fix: 3-tier stack — RealSense → standalone rgbd_odometry (wait_imu_to_init:=false)
#      → rtabmap SLAM with visual_odometry:=false subscribing to external /odom.
#
#  (1) "Waiting to initialize IMU orientation" forever
#      • RTAB-Map waits for IMU + valid TF when wait_imu_to_init:=true
#      • If IMU topic is wrong, TF missing, or sync fails, it never proceeds
#      • Default: GNNS_WAIT_IMU=0 → wait_imu_to_init:=false (visual odometry
#        starts without IMU gravity alignment). Set GNNS_WAIT_IMU=1 when IMU
#        topic + TF are verified.
#
#  (2) stack mode: RealSense runs but RTAB-Map never starts
#      • Bug: capturing PID with RS_PID="$(launch_realsense_bg)" also captured
#        ALL stdout from `ros2 launch` → wrong PID / broken behavior.
#      • Fix: redirect RealSense logs to a file; echo only the numeric PID.
#
#  (3) invalid rgbd_odometry PID (preset text + PID in one string)
#      • Bug: ODOM_PID="$(launch_rgbd_odometry_bg)" captured ALL stdout, including
#        build_rtab_rargs / echo lines, not just the PID.
#      • Fix: launch_rgbd_odometry_bg sets GNNS_RGBD_ODOM_PID=$!; callers use that.
#
#  (4) RTAB-Map SLAM: "IMU received doesn't have orientation set, it is ignored" (spam)
#      • rtabmap_slam expects sensor_msgs/Imu.orientation filled; RealSense /imu is gyro+accel
#        only (orientation quaternion is 0,0,0,0).
#      • Default: GNNS_RTABMAP_IMU=0 → rtabmap imu_topic remaps to a dummy name (no publisher)
#        so SLAM does not subscribe to raw RealSense IMU. rgbd_odometry still uses GNNS_IMU_TOPIC.
#      • For gravity constraints in SLAM: run imu_filter_madgwick (or similar), then set
#        GNNS_RTABMAP_IMU=1 or GNNS_RTABMAP_IMU_TOPIC=/your/filtered/imu
#
# Usage (from repo root that contains gnns_drone/ and config/):
#
#   chmod +x scripts/jetson_nano/gnns_vio_stack.sh
#
#   ./scripts/jetson_nano/gnns_vio_stack.sh realsense    # camera + IMU only
#   ./scripts/jetson_nano/gnns_vio_stack.sh imu          # Madgwick filter only (RealSense running)
#   ./scripts/jetson_nano/gnns_vio_stack.sh rtabmap      # rgbd_odometry + RTAB (camera up)
#   ./scripts/jetson_nano/gnns_vio_stack.sh ekf          # robot_localization EKF only (/odom + /imu/data)
#   ./scripts/jetson_nano/gnns_vio_stack.sh stack        # RealSense → Madgwick → rgbd_odometry → (EKF) → RTAB
#   ./scripts/jetson_nano/gnns_vio_stack.sh competition  # shorthand: enables IMU+Madgwick+EKF+accurate+30 Hz → stack
#   ./scripts/jetson_nano/gnns_vio_stack.sh mission --demo
#   ./scripts/jetson_nano/gnns_vio_stack.sh diagnose     # Topic list + rates + TF + log tails
#
# Competition profile (shortcut — same defaults as env block below, then runs `stack`):
#   ./scripts/jetson_nano/gnns_vio_stack.sh competition
#   Requires: sudo apt install ros-${ROS_DISTRO}-robot-localization  (EKF enabled)
#
# Competition default (explicit env — equivalent when values match defaults above):
#   GNNS_USE_IMU=1 GNNS_USE_EKF=1 GNNS_RS_FPS=30 GNNS_RTABMAP_PRESET=accurate \
#     ./scripts/jetson_nano/gnns_vio_stack.sh stack
#
# Environment — IMU pipeline (competition profile)
#   GNNS_USE_IMU=1        ENABLE IMU-assisted VO (default).  rgbd_odometry uses wait_imu_to_init +
#                         Odom/GuessMotion true; rtabmap_slam consumes ${GNNS_IMU_FILTERED_TOPIC}.
#                         Set 0 to fall back to pure visual + GuessMotion false.
#   GNNS_IMU_FILTER=1     Launch imu_filter_madgwick → ${GNNS_IMU_FILTERED_TOPIC} (default /imu/data).
#                         Required for valid sensor_msgs/Imu.orientation. Set 0 to skip Madgwick.
#   GNNS_IMU_FILTERED_TOPIC=/imu/data
#   GNNS_MADGWICK_USE_MAG=0|1   Use magnetometer for yaw correction (D455 has none → 0).
#   GNNS_MADGWICK_GAIN=0.05     Beta gain (lower = smoother, higher = more responsive).
#   GNNS_MADGWICK_WORLD_FRAME=enu|nwu|ned
#   GNNS_GYRO_FPS=200 GNNS_ACCEL_FPS=250    RealSense IMU rates (D455 max).
#   GNNS_REALSENSE_INITIAL_RESET=1   Power-cycle RealSense at launch (fixes HID bad optional access).
#   GNNS_REALSENSE_ENABLE_SYNC=1     Frame stereo+IMU in one clock domain (lower jitter).
#   GNNS_HEALTH_HZ_GATE=1            After topic is up, also verify minimum publish rate.
#
# Environment — EKF (robot_localization, optional smoother)
#   GNNS_USE_EKF=0|1      Launch ekf_node fusing /odom + /imu/data → /odometry/filtered.
#                         Needs: sudo apt install ros-${ROS_DISTRO}-robot-localization
#   GNNS_EKF_CONFIG=<path>            Default: config/ekf_vio.yaml (15-state, 3D).
#   GNNS_EKF_ODOM_TOPIC=/odometry/filtered
#
# Environment — legacy / shared
#   GNNS_WAIT_IMU=0|1        auto-set to 1 when GNNS_USE_IMU=1 + GNNS_IMU_FILTER=1.
#   GNNS_IMU_TOPIC=/.../imu  override raw IMU subscriber (used by Madgwick + raw-IMU fallback).
#   GNNS_RTABMAP_IMU=0|1 — legacy SLAM IMU toggle (0=dummy; honored when GNNS_USE_IMU=0).
#   GNNS_RTABMAP_IMU_TOPIC=/... — optional explicit imu topic for rtabmap SLAM (overrides USE_IMU).
#   GNNS_RTABMAP_VIZ — unset + DISPLAY set → rtabmap_viz ON; unset + no DISPLAY → OFF.
#                        Set 0 or 1 explicitly to override (SSH: export GNNS_RTABMAP_VIZ=0).
#   GNNS_LOG_DIR=/tmp        RealSense + Madgwick + rgbd_odom + EKF + RTAB-Map logs
#   GNNS_ODOM_TOPIC=/odom    rgbd_odometry output + rtabmap input
#   GNNS_WAIT_FOR_TRANSFORM=2.0   rtabmap TF wait (seconds; larger tolerates stamp skew)
#   GNNS_RTABMAP_NAMESPACE=rtabmap  must match ros2 launch namespace (for param hooks)
#   GNNS_RTABMAP_VIZ_SUBSCRIBE_ODOM=1  set rtabmap_viz subscribe_odom=true (uses /odom vs TF stamp)
#   GNNS_QOS / GNNS_QOS_IMU=2 / GNNS_QOS_ODOM=0   QoS overrides (do NOT force qos_odom=2)
#   GNNS_ODOM_SENSOR_SYNC=1|0  rtabmap odom_sensor_sync (try 0 if odom/image timestamps fight)
#   GNNS_STACK_CAMERA_WAIT_SEC=10  sleep after RealSense starts before checking topics
#   GNNS_ODOM_FRAME / GNNS_CAMERA_FRAME  must match rgbd_odometry TF (default odom / camera_link)
#
#   GPU/CUDA — ros-*-rtabmap-* packages use CPU OpenCV; no env flag moves this stack to the GPU.
#             Build OpenCV+RTAB-Map from source with CUDA to use the GPU; see docs/JETSON_LAPTOP_SETUP.md §11.
#
#   GNNS_RTABMAP_PRESET=default|recovery|accurate
#       recovery — F2F, lower MinInliers, ORB; use when GNNS_USE_IMU=0 and bootstrap fails.
#       accurate — F2M + GFTT, MinInliers 12, GuessMotion on (with IMU); best for competition.
#   GNNS_RS_FPS — default 15; on Jetson Orin try 30 after `ros2 topic hz /odom` is stable
#   GNNS_RS_IMU_STREAMS=1            Set 0 if HID Motion Sensor Failure — gyro/accel off (visual VO only).
#   GNNS_RS_UNITE_IMU_METHOD=2       Try 0 or 1 if HID / bad optional access persists.
#   GNNS_ODOM_MAX_RATE — cap rgbd_odometry publish rate (Hz); default = GNNS_RS_FPS
#   GNNS_RTABMAP_DETECTION_RATE — loop-closure / hypothesis checks per second (default preset=2)
#   GNNS_APPROX_SYNC_MAX — default 0.10 s (motion/jitter tolerant; 0.04 only if CPU keeps up)
#   GNNS_QUEUE_SIZE — default 10 for rgbd_odometry + rtabmap (balance lag vs starvation)
#   GNNS_USE_RGBD_SYNC=0|1 — opt-in fused rgbd_image stream (rtabmap_sync/rgbd_sync)
#   GNNS_RGBD_TOPIC — output topic when GNNS_USE_RGBD_SYNC=1 (default /gnns_rgbd_image)
#
# Quick VIO test (two terminals on Jetson):
#   Terminal A: ./scripts/jetson_nano/gnns_vio_stack.sh realsense
#   Terminal B:
#     ros2 topic hz /camera/camera/imu
#     GNNS_USE_IMU=1 GNNS_RTABMAP_VIZ=0 ./scripts/jetson_nano/gnns_vio_stack.sh rtabmap
#
# =============================================================================
# Do not use set -u — ROS setup.bash references unset variables.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JETSON_DIR="$SCRIPT_DIR"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# argv alias: `competition` → apply IMU+Madgwick+EKF+accurate+30fps+health gate, then `stack`
if [[ "${1:-}" == "competition" ]]; then
  export GNNS_USE_IMU="${GNNS_USE_IMU:-1}"
  export GNNS_IMU_FILTER="${GNNS_IMU_FILTER:-1}"
  export GNNS_USE_EKF="${GNNS_USE_EKF:-1}"
  export GNNS_RS_FPS="${GNNS_RS_FPS:-30}"
  export GNNS_RTABMAP_PRESET="${GNNS_RTABMAP_PRESET:-accurate}"
  export GNNS_HEALTH_HZ_GATE="${GNNS_HEALTH_HZ_GATE:-1}"
  shift
  set -- stack "$@"
  echo "[gnns_vio_stack] mode=competition (use IMU+Madgwick+EKF, ${GNNS_RS_FPS:-30} fps, preset=${GNNS_RTABMAP_PRESET:-accurate})" >&2
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  case "$(lsb_release -cs 2>/dev/null)" in
    noble)  ROS_DISTRO="jazzy"  ;;  # Ubuntu 24.04
    jammy)  ROS_DISTRO="humble" ;;  # Ubuntu 22.04
    *)      ROS_DISTRO="humble" ;;
  esac
fi
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

LOG_DIR="${GNNS_LOG_DIR:-/tmp}"
mkdir -p "$LOG_DIR"
RS_LOG="${LOG_DIR}/gnns_realsense.log"
RT_LOG="${LOG_DIR}/gnns_rtabmap.log"
ODOM_LOG="${LOG_DIR}/gnns_rgbd_odometry.log"
MAD_LOG="${LOG_DIR}/gnns_madgwick.log"
EKF_LOG="${LOG_DIR}/gnns_ekf.log"
# Set only by launch_rgbd_odometry_bg — read ODOM_PID from this; do not capture the function with $(...).
GNNS_RGBD_ODOM_PID=""
# Set only by launch_rgbd_sync_bg when GNNS_USE_RGBD_SYNC=1
GNNS_RGBD_SYNC_PID=""

# RGB/depth sync window (seconds). Too tight → dropped pairs under motion → VO stalls → TF freezes vs images.
APPROX_SYNC_MAX="${GNNS_APPROX_SYNC_MAX:-0.10}"
# Per-topic and sync queue depth (too small → starvation; too large → lag)
QUEUE_SIZE="${GNNS_QUEUE_SIZE:-10}"
# TF lookup tolerance for rtabmap (seconds). Helps minor stamp skew; does not fix multi‑second TF stalls.
WAIT_FOR_TRANSFORM="${GNNS_WAIT_FOR_TRANSFORM:-2.0}"
# RTAB-Map launch namespace (must match ros2 launch rtabmap.launch.py)
RTABMAP_NS="${GNNS_RTABMAP_NAMESPACE:-rtabmap}"
# rtabmap: qos applies to images; qos_imu is separate (see rtabmap.launch.py).
# Forcing qos=2 on rgbd_odometry breaks subscription to RealSense (no /odom) — use GNNS_QOS_ODOM=0.
GNNS_QOS="${GNNS_QOS:-0}"
GNNS_QOS_IMU="${GNNS_QOS_IMU:-2}"
GNNS_QOS_ODOM="${GNNS_QOS_ODOM:-0}"
# Sync rtabmap sensor callbacks with odometry timestamps (can worsen TF lookup if odom stalls)
GNNS_ODOM_SENSOR_SYNC="${GNNS_ODOM_SENSOR_SYNC:-1}"
if [[ "${GNNS_ODOM_SENSOR_SYNC}" == "1" ]]; then
  ODOM_SENSOR_SYNC_ARG="true"
else
  ODOM_SENSOR_SYNC_ARG="false"
fi

# -----------------------------------------------------------------------------
# IMU pipeline (NEW — competition profile)
# Madgwick fills sensor_msgs/Imu.orientation from raw gyro+accel, so rgbd_odometry
# can do wait_imu_to_init + GuessMotion (big accuracy win) and rtabmap_slam stops
# spamming "IMU received doesn't have orientation set, it is ignored".
# -----------------------------------------------------------------------------
GNNS_IMU_FILTER="${GNNS_IMU_FILTER:-1}"
GNNS_IMU_FILTERED_TOPIC="${GNNS_IMU_FILTERED_TOPIC:-/imu/data}"
GNNS_MADGWICK_USE_MAG="${GNNS_MADGWICK_USE_MAG:-0}"
GNNS_MADGWICK_GAIN="${GNNS_MADGWICK_GAIN:-0.05}"
GNNS_MADGWICK_WORLD_FRAME="${GNNS_MADGWICK_WORLD_FRAME:-enu}"
GNNS_USE_IMU="${GNNS_USE_IMU:-1}"
GNNS_REALSENSE_INITIAL_RESET="${GNNS_REALSENSE_INITIAL_RESET:-1}"
GNNS_REALSENSE_ENABLE_SYNC="${GNNS_REALSENSE_ENABLE_SYNC:-1}"
GNNS_GYRO_FPS="${GNNS_GYRO_FPS:-200}"
GNNS_ACCEL_FPS="${GNNS_ACCEL_FPS:-250}"
GNNS_IMU_QUEUE_SIZE="${GNNS_IMU_QUEUE_SIZE:-200}"
GNNS_HEALTH_HZ_GATE="${GNNS_HEALTH_HZ_GATE:-1}"
GNNS_USE_EKF="${GNNS_USE_EKF:-0}"
GNNS_EKF_CONFIG="${GNNS_EKF_CONFIG:-$PROJECT_ROOT/config/ekf_vio.yaml}"
GNNS_EKF_ODOM_TOPIC="${GNNS_EKF_ODOM_TOPIC:-/odometry/filtered}"
GNNS_MADGWICK_PID=""
GNNS_EKF_PID=""
# RealSense motion module (HID): D455 sometimes reports
# "HID Motion Sensor Failure! bad optional access" on Jetson/USB — see docs.
GNNS_RS_IMU_STREAMS="${GNNS_RS_IMU_STREAMS:-1}"
GNNS_RS_UNITE_IMU_METHOD="${GNNS_RS_UNITE_IMU_METHOD:-2}"

# -----------------------------------------------------------------------------
# Topic layout — matches RealSense: camera_name:=camera camera_namespace:=camera
# If your IMU is on e.g. /camera/imu, run:
#   export GNNS_IMU_TOPIC=/camera/imu
# -----------------------------------------------------------------------------
if [[ "${GNNS_RS_IMU_STREAMS}" != "1" ]]; then
  GNNS_USE_IMU="0"
  GNNS_IMU_FILTER="0"
  echo "[gnns_vio_stack] GNNS_RS_IMU_STREAMS=${GNNS_RS_IMU_STREAMS} — RealSense IMU disabled; GNNS_USE_IMU=0 GNNS_IMU_FILTER=0 (visual VO only)." >&2
fi

CAM_NS="/camera/camera"
RGB_TOPIC="${GNNS_RGB_TOPIC:-${CAM_NS}/color/image_raw}"
DEPTH_TOPIC="${GNNS_DEPTH_TOPIC:-${CAM_NS}/aligned_depth_to_color/image_raw}"
INFO_TOPIC="${GNNS_INFO_TOPIC:-${CAM_NS}/color/camera_info}"
IMU_TOPIC="${GNNS_IMU_TOPIC:-${CAM_NS}/imu}"
# IMU source consumed by rgbd_odometry / rtabmap_slam:
#   GNNS_IMU_FILTER=1 → Madgwick output ${GNNS_IMU_FILTERED_TOPIC} (has orientation quaternion)
#   GNNS_IMU_FILTER=0 → raw ${IMU_TOPIC} (orientation=(0,0,0,0); rtabmap_slam will spam)
if [[ "${GNNS_IMU_FILTER}" == "1" ]]; then
  IMU_SOURCE_FOR_VO="${GNNS_IMU_FILTERED_TOPIC}"
else
  IMU_SOURCE_FOR_VO="${IMU_TOPIC}"
fi
# rtabmap_slam IMU topic resolution
#   explicit GNNS_RTABMAP_IMU_TOPIC wins,
#   else if GNNS_USE_IMU=1 use filtered topic,
#   else legacy GNNS_RTABMAP_IMU=1 raw topic,
#   else dummy (disabled).
if [[ -n "${GNNS_RTABMAP_IMU_TOPIC:-}" ]]; then
  RTABMAP_SLAM_IMU_TOPIC="${GNNS_RTABMAP_IMU_TOPIC}"
elif [[ "${GNNS_USE_IMU}" == "1" && "${GNNS_IMU_FILTER}" == "1" ]]; then
  RTABMAP_SLAM_IMU_TOPIC="${GNNS_IMU_FILTERED_TOPIC}"
elif [[ "${GNNS_RTABMAP_IMU:-0}" == "1" ]]; then
  RTABMAP_SLAM_IMU_TOPIC="${IMU_TOPIC}"
else
  RTABMAP_SLAM_IMU_TOPIC="/gnns/rtabmap_imu_disabled"
fi
# rgbd_odometry publishes here; rtabmap SLAM subscribes (must match config/vio_config.yaml)
ODOM_TOPIC="${GNNS_ODOM_TOPIC:-/odom}"
# TF frame names (must match rgbd_odometry -p odom_frame_id / frame_id)
ODOM_FRAME_FOR_TF="${GNNS_ODOM_FRAME:-odom}"
CAMERA_FRAME_FOR_TF="${GNNS_CAMERA_FRAME:-camera_link}"

# Optional rgbd_sync (rtabmap_sync): fused rgbd_image topic for odom + SLAM when GNNS_USE_RGBD_SYNC=1
GNNS_USE_RGBD_SYNC="${GNNS_USE_RGBD_SYNC:-0}"
GNNS_RGBD_TOPIC="${GNNS_RGBD_TOPIC:-/gnns_rgbd_image}"

# Default: do NOT wait for IMU (fixes endless "Waiting to initialize IMU orientation")
# Set GNNS_WAIT_IMU=1 only after `ros2 topic hz /camera/camera/imu` is stable.
# When GNNS_USE_IMU=1 and GNNS_IMU_FILTER=1 we KNOW Madgwick will publish a valid
# orientation-bearing IMU, so wait_imu_to_init becomes safe and high-accuracy.
WAIT_IMU="${GNNS_WAIT_IMU:-0}"
if [[ "${GNNS_USE_IMU}" == "1" && "${GNNS_IMU_FILTER}" == "1" ]]; then
  WAIT_IMU="1"
fi
if [[ "$WAIT_IMU" == "1" ]]; then
  WAIT_IMU_INIT="true"
else
  WAIT_IMU_INIT="false"
fi

# rtabmap_viz: if GNNS_RTABMAP_VIZ is unset, enable when DISPLAY is set (local monitor).
if [[ -z "${GNNS_RTABMAP_VIZ+x}" ]]; then
  if [[ -n "${DISPLAY:-}" ]]; then
    GNNS_RTABMAP_VIZ=1
    echo "[gnns_vio_stack] GNNS_RTABMAP_VIZ unset + DISPLAY set → rtabmap_viz ON (set GNNS_RTABMAP_VIZ=0 to disable)" >&2
  else
    GNNS_RTABMAP_VIZ=0
  fi
fi
if [[ "${GNNS_RTABMAP_VIZ:-0}" == "1" ]]; then
  RTABMAP_VIZ="true"
else
  RTABMAP_VIZ="false"
fi

# Frame rate: 15 Hz default gives 66 ms/frame budget for VO (safe on most CPUs).
# Set GNNS_RS_FPS=30 only after confirming stable /odom rate: ros2 topic hz /odom
RS_FPS="${GNNS_RS_FPS:-15}"
if [[ "$RS_FPS" == "15" ]]; then
  RS_PROFILE="640x480x15"
elif [[ "$RS_FPS" == "30" ]]; then
  RS_PROFILE="640x480x30"
else
  RS_PROFILE="640x480x${RS_FPS}"
fi

# rgbd_odometry max_update_rate (Hz) — match camera FPS on Jetson; laptop may stay at 15.
ODOM_MAX_RATE="${GNNS_ODOM_MAX_RATE:-${RS_FPS}}"

# -----------------------------------------------------------------------------
# RTAB-Map CLI args (override with RTABMAP_EXTRA_ARGS / GNNS_RTABMAP_PRESET)
# -----------------------------------------------------------------------------
# Preset "recovery" addresses:
#   • OdometryF2M: "Not enough inliers 0/20 (matches=N) between -1 and …"
#     → geometric check fails (depth mismatch, sync, or F2M map ref -1).
#   • rtabmap: "no odometry is provided. Image 0 is ignored"
#     → consequence of failed rgbd_odometry (quality=0).
#
# recovery: F2F odometry (Strategy 1), lower MinInliers, ORB instead of FAST,
#           slightly looser motion threshold — better bootstrap in dim light.
# -----------------------------------------------------------------------------
# default: F2M for map quality; tuned for motion robustness and CPU budget at 15 Hz.
# MinInliers/MotionThreshold relaxed so fast camera moves don't immediately fail VO.
# BA disabled (too expensive per frame); ResetCountdown auto-recovers after 2 bad frames.
# GuessMotion disabled — unreliable velocity prior without verified IMU.
RTABMAP_ARGS_DEFAULT="--delete_db_on_start
--Vis/MaxFeatures 600
--Vis/MinInliers 12
--Vis/EstimationType 1
--Vis/MotionThreshold 1.0
--Kp/MaxFeatures 600
--Kp/DetectorStrategy 6
--Kp/MaxDepth 8.0
--Kp/MinDepth 0.3
--OdoF2M/MaxSize 2000
--OdoF2M/BundleAdjustment 0
--Odom/Strategy 0
--Odom/GuessMotion true
--Odom/KalmanProcessNoise 0.001
--Odom/KalmanMeasurementNoise 0.01
--Odom/FilteringStrategy 1
--Odom/Holonomic false
--Odom/ResetCountdown 2
--Mem/STMSize 20
--Mem/RehearsalSimilarity 0.45
--Rtabmap/TimeThr 0
--Rtabmap/DetectionRate 2
--RGBD/ProximityBySpace true
--RGBD/LinearSpeedUpdate 0.05
--RGBD/AngularSpeedUpdate 0.05"

# recovery: F2F (no reference map needed), permissive thresholds, auto-reset after 2 bad frames.
RTABMAP_ARGS_RECOVERY="--delete_db_on_start
--Vis/MaxFeatures 800
--Vis/MinInliers 8
--Vis/EstimationType 1
--Vis/MotionThreshold 1.0
--Kp/MaxFeatures 800
--Kp/DetectorStrategy 3
--Kp/MaxDepth 8.0
--Kp/MinDepth 0.2
--OdoF2M/MaxSize 1500
--OdoF2M/BundleAdjustment 0
--Odom/Strategy 1
--Odom/GuessMotion false
--Odom/FilteringStrategy 1
--Odom/Holonomic false
--Odom/ResetCountdown 2
--Mem/STMSize 20
--Mem/RehearsalSimilarity 0.45
--Rtabmap/TimeThr 0
--Rtabmap/DetectionRate 2
--RGBD/ProximityBySpace true
--RGBD/LinearSpeedUpdate 0.05
--RGBD/AngularSpeedUpdate 0.05"

# accurate: GFTT corners, F2M odometry, particle filter, loop closure throttled, sharper grid.
# BA disabled for speed; slightly more patient reset (3 bad frames before reference reset).
RTABMAP_ARGS_ACCURATE="--delete_db_on_start
--Vis/MaxFeatures 600
--Vis/MinInliers 12
--Vis/EstimationType 1
--Vis/MotionThreshold 0.7
--Kp/MaxFeatures 600
--Kp/DetectorStrategy 0
--Kp/MaxDepth 6.0
--Kp/MinDepth 0.3
--OdoF2M/MaxSize 2000
--OdoF2M/BundleAdjustment 0
--Odom/Strategy 0
--Odom/GuessMotion true
--Odom/KalmanProcessNoise 0.001
--Odom/KalmanMeasurementNoise 0.01
--Odom/FilteringStrategy 2
--Odom/Holonomic false
--Odom/ResetCountdown 3
--Mem/STMSize 30
--Mem/RehearsalSimilarity 0.40
--Rtabmap/TimeThr 0
--Rtabmap/DetectionRate 2
--RGBD/ProximityBySpace true
--RGBD/LinearSpeedUpdate 0.02
--RGBD/AngularSpeedUpdate 0.02
--Grid/CellSize 0.05
--Grid/ClusterRadius 0.1"

# Built when launching RTAB-Map (see build_rtab_rargs)
RTAB_RARGS=""
# Same preset but without Odom/* and OdoF2M/* — the rtabmap SLAM node does not declare those
# ROS2 parameters; passing them in rtabmap_args crashes with ParameterNotDeclaredException.
RTAB_SLAM_RARGS=""

# Remove odometry-node-only CLI flags (rgbd_odometry accepts them; rtabmap slam does not).
strip_rtabargs_for_slam_node() {
  local s="$1"
  # Each flag is "--Name value" with a single-token value (true/false/numbers).
  s=$(echo "$s" | sed -E 's/--Odom\/[^ ]+ [^ ]+//g')
  s=$(echo "$s" | sed -E 's/--OdoF2M\/[^ ]+ [^ ]+//g')
  echo "$s" | tr -s ' ' | sed 's/^ *//;s/ *$//'
}

build_rtab_rargs() {
  # default = F2M (more stable when moving); use recovery if bootstrap fails in poor conditions
  local preset="${GNNS_RTABMAP_PRESET:-default}"
  local base
  case "$preset" in
    default)
      base="$RTABMAP_ARGS_DEFAULT"
      echo "[gnns_vio_stack] RTAB-Map preset: default (F2M, motion-tuned MinInliers 8)" >&2
      ;;
    recovery)
      base="$RTABMAP_ARGS_RECOVERY"
      echo "[gnns_vio_stack] RTAB-Map preset: recovery (F2F, ORB, MinInliers 8)" >&2
      ;;
    accurate)
      base="$RTABMAP_ARGS_ACCURATE"
      echo "[gnns_vio_stack] RTAB-Map preset: accurate (F2M, GFTT, MinInliers 12, DetectionRate 2)" >&2
      ;;
    *)
      echo "[gnns_vio_stack] ERROR: unknown GNNS_RTABMAP_PRESET='${preset}' (use: default|recovery|accurate)" >&2
      exit 1
      ;;
  esac
  RTAB_RARGS="${base//[$'\n']/ }"
  if [[ "${RTABMAP_KEEP_DB:-0}" == "1" ]]; then
    RTAB_RARGS="${RTAB_RARGS/--delete_db_on_start/}"
  fi
  # IMU-aware auto-revert: presets now default to GuessMotion=true (IMU prior).
  # If IMU is off, restore GuessMotion=false so VO doesn't trust a non-existent prior.
  if [[ "${GNNS_USE_IMU}" != "1" ]]; then
    RTAB_RARGS="${RTAB_RARGS} --Odom/GuessMotion false"
  fi
  if [[ -n "${RTABMAP_EXTRA_ARGS:-}" ]]; then
    RTAB_RARGS="${RTAB_RARGS} ${RTABMAP_EXTRA_ARGS}"
  fi
  if [[ -n "${GNNS_RTABMAP_DETECTION_RATE:-}" ]]; then
    RTAB_RARGS="${RTAB_RARGS} --Rtabmap/DetectionRate ${GNNS_RTABMAP_DETECTION_RATE}"
  fi
  RTAB_SLAM_RARGS="$(strip_rtabargs_for_slam_node "${RTAB_RARGS}")"
}

# Common RealSense launch arguments
# initial_reset often clears transient "HID Motion Sensor Failure! bad optional access".
# GNNS_RS_IMU_STREAMS=0 skips motion module entirely (visual odometry only).
if [[ "${GNNS_REALSENSE_INITIAL_RESET}" == "1" ]]; then RS_INITIAL_RESET="true"; else RS_INITIAL_RESET="false"; fi
if [[ "${GNNS_REALSENSE_ENABLE_SYNC}" == "1" ]]; then RS_ENABLE_SYNC="true"; else RS_ENABLE_SYNC="false"; fi
RS_LAUNCH_ARGS=(
  align_depth.enable:=true
  publish_tf:=true
  enable_color:=true
  enable_depth:=true
  "initial_reset:=${RS_INITIAL_RESET}"
  "enable_sync:=${RS_ENABLE_SYNC}"
  depth_module.infra_profile:=0x0x0
  "rgb_camera.color_profile:=${RS_PROFILE}"
  "depth_module.depth_profile:=${RS_PROFILE}"
  camera_name:=camera
  camera_namespace:=camera
)
if [[ "${GNNS_RS_IMU_STREAMS:-1}" == "1" ]]; then
  RS_LAUNCH_ARGS+=(
    enable_gyro:=true
    enable_accel:=true
    "unite_imu_method:=${GNNS_RS_UNITE_IMU_METHOD}"
    "gyro_fps:=${GNNS_GYRO_FPS}"
    "accel_fps:=${GNNS_ACCEL_FPS}"
  )
else
  RS_LAUNCH_ARGS+=(
    enable_gyro:=false
    enable_accel:=false
  )
fi

# -----------------------------------------------------------------------------
# Wait until a ROS topic has at least one message (timeout seconds)
# -----------------------------------------------------------------------------
wait_for_topic() {
  local topic="$1"
  local timeout="${2:-45}"
  local elapsed=0
  echo "[gnns_vio_stack] Waiting for topic (up to ${timeout}s): ${topic}"
  while [[ "$elapsed" -lt "$timeout" ]]; do
    if ros2 topic echo "$topic" --once >/dev/null 2>&1; then
      echo "[gnns_vio_stack] OK: ${topic} is publishing."
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  echo "[gnns_vio_stack] WARN: timeout waiting for ${topic}" >&2
  return 1
}

# -----------------------------------------------------------------------------
# Wait until TF connects odom -> camera_link (rgbd_odometry must publish publish_tf)
# Without this, rtabmap sees "two unconnected trees" while /odom topic may still publish.
# -----------------------------------------------------------------------------
# Before rgbd_odometry: ensure RealSense is publishing (avoids "Did not receive data" forever).
wait_for_camera_rgb_depth_or_exit() {
  if [[ "${GNNS_USE_RGBD_SYNC}" == "1" ]]; then
    return 0
  fi
  echo "[gnns_vio_stack] Waiting for camera topics (color + depth)…"
  if ! wait_for_topic "${RGB_TOPIC}" 60; then
    echo "[gnns_vio_stack] ERROR: no messages on ${RGB_TOPIC}. Is RealSense running?" >&2
    echo "--- last 30 lines of ${RS_LOG} ---" >&2
    tail -n 30 "${RS_LOG}" 2>/dev/null >&2 || true
    exit 1
  fi
  if ! wait_for_topic "${DEPTH_TOPIC}" 60; then
    echo "[gnns_vio_stack] ERROR: no messages on ${DEPTH_TOPIC}." >&2
    echo "--- last 30 lines of ${RS_LOG} ---" >&2
    tail -n 30 "${RS_LOG}" 2>/dev/null >&2 || true
    exit 1
  fi
}

wait_for_tf_odom_camera_link() {
  local max_wait="${1:-90}"
  echo "[gnns_vio_stack] Waiting for TF ${ODOM_FRAME_FOR_TF} -> ${CAMERA_FRAME_FOR_TF} (up to ${max_wait}s)…"
  # Do NOT use head -N: tf2_echo may print many lines before the first "Translation".
  # grep -m 1 stops at the first match (same pattern you see when tf2_echo works manually).
  if PARENT="${ODOM_FRAME_FOR_TF}" CHILD="${CAMERA_FRAME_FOR_TF}" \
    timeout "${max_wait}" bash -c 'ros2 run tf2_ros tf2_echo "$PARENT" "$CHILD" -r 10 2>&1 | grep -m 1 -q Translation'; then
    echo "[gnns_vio_stack] OK: TF ${ODOM_FRAME_FOR_TF} -> ${CAMERA_FRAME_FOR_TF} (same tree as rgbd_odometry)."
    return 0
  fi
  echo "[gnns_vio_stack] ERROR: TF ${ODOM_FRAME_FOR_TF} -> ${CAMERA_FRAME_FOR_TF} never connected (two TF trees)." >&2
  echo "  rgbd_odometry must publish odom→camera_link (publish_tf=true). Check: ${ODOM_LOG}" >&2
  echo "  Run: ros2 run tf2_tools view_frames   # or: ros2 run tf2_ros tf2_echo odom camera_link" >&2
  return 1
}

# -----------------------------------------------------------------------------
# RealSense: MUST enable IMU to avoid optional-access / missing-IMU failures
# -----------------------------------------------------------------------------
launch_realsense() {
  echo "[gnns_vio_stack] Launching RealSense with IMU (gyro+accel, fused IMU topic)…"
  echo "  Topics: ${RGB_TOPIC}, ${DEPTH_TOPIC}, ${IMU_TOPIC}"
  exec ros2 launch realsense2_camera rs_launch.py "${RS_LAUNCH_ARGS[@]}"
}

# Background RealSense — MUST redirect stdout/stderr so PID capture is not corrupted
launch_realsense_bg() {
  ros2 launch realsense2_camera rs_launch.py "${RS_LAUNCH_ARGS[@]}" >>"${RS_LOG}" 2>&1 &
  echo $!
}

# Kill a PID and any children (ros2 launch leaves component nodes as children)
kill_process_tree() {
  local root="$1"
  [[ -z "$root" ]] || ! [[ "$root" =~ ^[0-9]+$ ]] && return 0
  local c
  for c in $(pgrep -P "$root" 2>/dev/null || true); do
    kill_process_tree "$c"
  done
  if kill -0 "$root" 2>/dev/null; then
    kill -TERM "$root" 2>/dev/null || true
    sleep 0.3
    kill -KILL "$root" 2>/dev/null || true
  fi
}

# -----------------------------------------------------------------------------
# require_ros_pkg <pkg> <install-hint> — fail fast with actionable hint
# -----------------------------------------------------------------------------
require_ros_pkg() {
  local pkg="$1"; local hint="$2"
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "[gnns_vio_stack] ERROR: ROS 2 package '${pkg}' not found." >&2
    echo "  Install: ${hint}" >&2
    exit 2
  fi
}

# -----------------------------------------------------------------------------
# wait_for_topic_hz <topic> <min_hz> <window_sec> — verify publish rate
# Parses `ros2 topic hz --window N` (line: "average rate: <hz>"). Returns 0 if
# observed average >= min_hz, else 1. Use after wait_for_topic so we don't
# declare ready on a topic that's only sputtering.
# -----------------------------------------------------------------------------
wait_for_topic_hz() {
  local topic="$1"; local min_hz="$2"; local window="${3:-6}"
  if [[ "${GNNS_HEALTH_HZ_GATE}" != "1" ]]; then
    return 0
  fi
  echo "[gnns_vio_stack] Checking rate of ${topic} (need >= ${min_hz} Hz over ${window}s)…"
  local out
  out="$(timeout "$((window + 4))" ros2 topic hz "$topic" --window "$window" 2>&1 | grep -m 1 'average rate' || true)"
  if [[ -z "$out" ]]; then
    echo "[gnns_vio_stack] WARN: no rate samples for ${topic} (gate skipped)" >&2
    return 0
  fi
  local hz
  hz="$(echo "$out" | sed -E 's/.*average rate:[[:space:]]*([0-9]+\.[0-9]+).*/\1/')"
  if awk -v a="$hz" -v b="$min_hz" 'BEGIN{exit !(a+0 >= b+0)}'; then
    echo "[gnns_vio_stack] OK: ${topic} averaging ${hz} Hz (>= ${min_hz})."
    return 0
  fi
  echo "[gnns_vio_stack] WARN: ${topic} averaging ${hz} Hz (< ${min_hz})" >&2
  return 1
}

# -----------------------------------------------------------------------------
# imu_filter_madgwick — fuses raw gyro+accel from RealSense into orientation-bearing
# /imu/data (or GNNS_IMU_FILTERED_TOPIC). This is what makes wait_imu_to_init and
# Odom/GuessMotion safe + accurate. Without this, rtabmap_slam spams "orientation
# not set" and rgbd_odometry has no gravity prior.
# -----------------------------------------------------------------------------
launch_madgwick_bg() {
  require_ros_pkg imu_filter_madgwick "sudo apt install -y ros-${ROS_DISTRO}-imu-filter-madgwick"
  local use_mag="false"
  [[ "${GNNS_MADGWICK_USE_MAG}" == "1" ]] && use_mag="true"
  echo "[gnns_vio_stack] Launching imu_filter_madgwick → ${GNNS_IMU_FILTERED_TOPIC}…" >&2
  echo "  raw=${IMU_TOPIC}  use_mag=${use_mag}  gain=${GNNS_MADGWICK_GAIN}  world_frame=${GNNS_MADGWICK_WORLD_FRAME}" >&2
  echo "  Log: ${MAD_LOG}" >&2
  ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args \
    -p use_mag:=${use_mag} \
    -p publish_tf:=false \
    -p world_frame:=${GNNS_MADGWICK_WORLD_FRAME} \
    -p gain:=${GNNS_MADGWICK_GAIN} \
    -r imu/data_raw:=${IMU_TOPIC} \
    -r imu/data:=${GNNS_IMU_FILTERED_TOPIC} \
    >>"${MAD_LOG}" 2>&1 &
  GNNS_MADGWICK_PID=$!
}

# -----------------------------------------------------------------------------
# robot_localization ekf_node — fuse /odom (position) with /imu/data (orientation,
# angular velocity, linear acceleration) into /odometry/filtered. This is the
# competition-grade smoother (15-state EKF in 3D mode).
# -----------------------------------------------------------------------------
launch_ekf_bg() {
  require_ros_pkg robot_localization "sudo apt install -y ros-${ROS_DISTRO}-robot-localization"
  if [[ ! -f "${GNNS_EKF_CONFIG}" ]]; then
    echo "[gnns_vio_stack] ERROR: GNNS_EKF_CONFIG not found: ${GNNS_EKF_CONFIG}" >&2
    exit 2
  fi
  echo "[gnns_vio_stack] Launching robot_localization ekf_node → ${GNNS_EKF_ODOM_TOPIC}…" >&2
  echo "  config=${GNNS_EKF_CONFIG}" >&2
  echo "  Log:    ${EKF_LOG}" >&2
  ros2 run robot_localization ekf_node --ros-args \
    --params-file "${GNNS_EKF_CONFIG}" \
    -r odometry/filtered:=${GNNS_EKF_ODOM_TOPIC} \
    >>"${EKF_LOG}" 2>&1 &
  GNNS_EKF_PID=$!
}

# -----------------------------------------------------------------------------
# Optional: rgbd_sync (rtabmap_sync) — fuse color+depth into one rgbd_image topic
# -----------------------------------------------------------------------------
# When GNNS_USE_RGBD_SYNC=1, odom + rtabmap subscribe_rgbd use GNNS_RGBD_TOPIC.
launch_rgbd_sync_bg() {
  echo "[gnns_vio_stack] Launching rgbd_sync (rtabmap_sync) → ${GNNS_RGBD_TOPIC}…" >&2
  echo "  Log: ${ODOM_LOG}" >&2
  # shellcheck disable=SC2086
  ros2 run rtabmap_sync rgbd_sync --ros-args \
    -p approx_sync:=true \
    -p approx_sync_max_interval:=${APPROX_SYNC_MAX} \
    -p topic_queue_size:=${QUEUE_SIZE} \
    -p sync_queue_size:=${QUEUE_SIZE} \
    -p qos:=${GNNS_QOS_ODOM} \
    -r rgb/image:=${RGB_TOPIC} \
    -r depth/image:=${DEPTH_TOPIC} \
    -r rgb/camera_info:=${INFO_TOPIC} \
    -r rgbd_image:=${GNNS_RGBD_TOPIC} \
    >>"${ODOM_LOG}" 2>&1 &
  GNNS_RGBD_SYNC_PID=$!
}

# -----------------------------------------------------------------------------
# Standalone rgbd_odometry (rtabmap_odom package) — decoupled from rtabmap SLAM
# -----------------------------------------------------------------------------
# Publishes nav_msgs/Odometry on ODOM_TOPIC and TF odom→camera_link.
# When GNNS_WAIT_IMU=0, VO starts without IMU gravity alignment (default).
# When GNNS_WAIT_IMU=1, pass wait_imu_to_init + always_check_imu_tf for IMU fusion.
launch_rgbd_odometry_bg() {
  build_rtab_rargs
  local odom_rargs="${RTAB_RARGS}"
  odom_rargs="${odom_rargs//--delete_db_on_start/}"
  odom_rargs="$(echo "${odom_rargs}" | tr -s ' ' | sed 's/^ *//;s/ *$//')"

  # IMU policy:
  #   GNNS_USE_IMU=1  → wait for IMU + always_check_imu_tf=true; remap to filtered topic.
  #   GNNS_USE_IMU=0  → fall back to legacy GNNS_WAIT_IMU behavior (raw IMU, no wait).
  local wait_imu_odom="false"
  local check_imu_tf="false"
  local imu_for_vo="${IMU_TOPIC}"
  if [[ "${GNNS_USE_IMU}" == "1" ]]; then
    wait_imu_odom="true"
    check_imu_tf="true"
    imu_for_vo="${IMU_SOURCE_FOR_VO}"
  elif [[ "${WAIT_IMU}" == "1" ]]; then
    wait_imu_odom="true"
    check_imu_tf="true"
  fi

  echo "[gnns_vio_stack] Launching rgbd_odometry (rtabmap_odom)…" >&2
  echo "  Log: ${ODOM_LOG}" >&2
  echo "  → ${ODOM_TOPIC}  max_update_rate=${ODOM_MAX_RATE} Hz  (wait_imu_to_init=${wait_imu_odom})" >&2
  echo "  IMU subscriber: ${imu_for_vo}  qos_imu=${GNNS_QOS_IMU}  topic_queue_imu=${GNNS_IMU_QUEUE_SIZE}" >&2

  if [[ "${GNNS_USE_RGBD_SYNC}" == "1" ]]; then
    # shellcheck disable=SC2086
    ros2 run rtabmap_odom rgbd_odometry ${odom_rargs} \
      --ros-args \
      -p frame_id:=camera_link \
      -p odom_frame_id:=odom \
      -p publish_tf:=true \
      -p wait_imu_to_init:=${wait_imu_odom} \
      -p always_check_imu_tf:=${check_imu_tf} \
      -p subscribe_rgbd:=true \
      -p approx_sync:=true \
      -p approx_sync_max_interval:=${APPROX_SYNC_MAX} \
      -p sync_queue_size:=${QUEUE_SIZE} \
      -p topic_queue_size:=${QUEUE_SIZE} \
      -p qos:=${GNNS_QOS_ODOM} \
      -p qos_camera_info:=${GNNS_QOS_ODOM} \
      -p qos_imu:=${GNNS_QOS_IMU} \
      -p max_update_rate:=${ODOM_MAX_RATE}.0 \
      -p expected_update_rate:=${ODOM_MAX_RATE}.0 \
      -r rgbd_image:=${GNNS_RGBD_TOPIC} \
      -r imu:=${imu_for_vo} \
      -r odom:=${ODOM_TOPIC} \
      >>"${ODOM_LOG}" 2>&1 &
  else
    # shellcheck disable=SC2086
    ros2 run rtabmap_odom rgbd_odometry ${odom_rargs} \
      --ros-args \
      -p frame_id:=camera_link \
      -p odom_frame_id:=odom \
      -p publish_tf:=true \
      -p wait_imu_to_init:=${wait_imu_odom} \
      -p always_check_imu_tf:=${check_imu_tf} \
      -p approx_sync:=true \
      -p approx_sync_max_interval:=${APPROX_SYNC_MAX} \
      -p sync_queue_size:=${QUEUE_SIZE} \
      -p topic_queue_size:=${QUEUE_SIZE} \
      -p qos:=${GNNS_QOS_ODOM} \
      -p qos_camera_info:=${GNNS_QOS_ODOM} \
      -p qos_imu:=${GNNS_QOS_IMU} \
      -p max_update_rate:=${ODOM_MAX_RATE}.0 \
      -p expected_update_rate:=${ODOM_MAX_RATE}.0 \
      -r rgb/image:=${RGB_TOPIC} \
      -r depth/image:=${DEPTH_TOPIC} \
      -r rgb/camera_info:=${INFO_TOPIC} \
      -r imu:=${imu_for_vo} \
      -r odom:=${ODOM_TOPIC} \
      >>"${ODOM_LOG}" 2>&1 &
  fi
  GNNS_RGBD_ODOM_PID=$!
}

# -----------------------------------------------------------------------------
# rtabmap_viz: upstream launch omits subscribe_odom; set via param when node appears.
# Uses /odom topic instead of TF-at-image-stamp (mitigates TF extrapolation spam when VO lags).
# -----------------------------------------------------------------------------
enable_rtabmap_viz_subscribe_odom_bg() {
  # Note: ros2 param set does NOT reinitialize DDS subscriptions at runtime.
  # This single-attempt sets the parameter value for diagnostics only.
  # The real fix for viz TF errors is keeping VO alive (Odom/ResetCountdown).
  [[ "${RTABMAP_VIZ}" != "true" ]] && return 0
  [[ "${GNNS_RTABMAP_VIZ_SUBSCRIBE_ODOM:-1}" != "1" ]] && return 0
  (
    local i=0
    local max=120
    while [[ "$i" -lt "$max" ]]; do
      if ros2 node list 2>/dev/null | grep -qF "/${RTABMAP_NS}/rtabmap_viz"; then
        ros2 param set "/${RTABMAP_NS}/rtabmap_viz" subscribe_odom true 2>/dev/null && \
          echo "[gnns_vio_stack] Set /${RTABMAP_NS}/rtabmap_viz subscribe_odom=true" >&2
        exit 0
      fi
      sleep 0.25
      i=$((i + 1))
    done
  ) &
}

# -----------------------------------------------------------------------------
# RTAB-Map SLAM only (external odometry — no rgbd_odometry in this launch)
# -----------------------------------------------------------------------------
launch_rtabmap_slam() {
  build_rtab_rargs
  enable_rtabmap_viz_subscribe_odom_bg
  echo "[gnns_vio_stack] Launching RTAB-Map SLAM (visual_odometry:=false, uses ${ODOM_TOPIC})…"
  echo "  Log: ${RT_LOG}"
  echo "  rgb=${RGB_TOPIC} depth=${DEPTH_TOPIC}  imu(vo)=${IMU_TOPIC}  imu(rtabmap_slam)=${RTABMAP_SLAM_IMU_TOPIC}"
  echo "  wait_imu_to_init=${WAIT_IMU_INIT}  rtabmap_viz=${RTABMAP_VIZ}"
  echo "  odom_sensor_sync=${ODOM_SENSOR_SYNC_ARG}  wait_for_transform=${WAIT_FOR_TRANSFORM}s  qos=${GNNS_QOS}  qos_imu=${GNNS_QOS_IMU}  qos_odom=Reliable  namespace=${RTABMAP_NS}"
  if [[ "${GNNS_USE_RGBD_SYNC}" == "1" ]]; then
    # shellcheck disable=SC2086
    ros2 launch rtabmap_launch rtabmap.launch.py \
      namespace:=${RTABMAP_NS} \
      rgb_topic:="${RGB_TOPIC}" \
      depth_topic:="${DEPTH_TOPIC}" \
      camera_info_topic:="${INFO_TOPIC}" \
      imu_topic:="${RTABMAP_SLAM_IMU_TOPIC}" \
      frame_id:=camera_link \
      approx_sync:=true \
      approx_sync_max_interval:=${APPROX_SYNC_MAX} \
      queue_size:=${QUEUE_SIZE} \
      qos:=${GNNS_QOS} \
      qos_imu:=${GNNS_QOS_IMU} \
      subscribe_rgbd:=true \
      rgbd_topic:=${GNNS_RGBD_TOPIC} \
      rgbd_sync:=false \
      wait_imu_to_init:=${WAIT_IMU_INIT} \
      visual_odometry:=false \
      odom_frame_id:=odom \
      map_frame_id:=map \
      publish_tf_map:=true \
      publish_tf_odom:=false \
      odom_topic:=${ODOM_TOPIC} \
      odom_sensor_sync:=${ODOM_SENSOR_SYNC_ARG} \
      wait_for_transform:=${WAIT_FOR_TRANSFORM} \
      qos_odom:=1 \
      rtabmap_viz:=${RTABMAP_VIZ} \
      rtabmap_args:="${RTAB_SLAM_RARGS}" 2>&1 | tee -a "${RT_LOG}"
  else
    # shellcheck disable=SC2086
    ros2 launch rtabmap_launch rtabmap.launch.py \
      namespace:=${RTABMAP_NS} \
      rgb_topic:="${RGB_TOPIC}" \
      depth_topic:="${DEPTH_TOPIC}" \
      camera_info_topic:="${INFO_TOPIC}" \
      imu_topic:="${RTABMAP_SLAM_IMU_TOPIC}" \
      frame_id:=camera_link \
      approx_sync:=true \
      approx_sync_max_interval:=${APPROX_SYNC_MAX} \
      queue_size:=${QUEUE_SIZE} \
      qos:=${GNNS_QOS} \
      qos_imu:=${GNNS_QOS_IMU} \
      subscribe_rgbd:=false \
      wait_imu_to_init:=${WAIT_IMU_INIT} \
      visual_odometry:=false \
      odom_frame_id:=odom \
      map_frame_id:=map \
      publish_tf_map:=true \
      publish_tf_odom:=false \
      odom_topic:=${ODOM_TOPIC} \
      odom_sensor_sync:=${ODOM_SENSOR_SYNC_ARG} \
      wait_for_transform:=${WAIT_FOR_TRANSFORM} \
      qos_odom:=1 \
      rtabmap_viz:=${RTABMAP_VIZ} \
      rtabmap_args:="${RTAB_SLAM_RARGS}" 2>&1 | tee -a "${RT_LOG}"
  fi
}

# -----------------------------------------------------------------------------
# rgbd_odometry (bg) + RTAB-Map SLAM — use when RealSense is already running
# -----------------------------------------------------------------------------
launch_rtabmap_with_odom() {
  echo "[gnns_vio_stack] RTAB-Map session (rgbd_odometry → ${ODOM_TOPIC} → rtabmap)…"
  echo "[gnns_vio_stack] rgbd_odometry log: ${ODOM_LOG}"
  echo "[gnns_vio_stack] RTAB-Map log:       ${RT_LOG}"
  if [[ "${GNNS_USE_RGBD_SYNC}" == "1" ]]; then
    echo "[gnns_vio_stack] GNNS_USE_RGBD_SYNC=1 → rgbd_sync → ${GNNS_RGBD_TOPIC}"
  fi

  : >"${ODOM_LOG}"
  : >"${RT_LOG}"

  # Optional Madgwick (assumes RealSense is already publishing raw IMU on ${IMU_TOPIC})
  if [[ "${GNNS_IMU_FILTER}" == "1" ]]; then
    : >"${MAD_LOG}"
    if wait_for_topic "${IMU_TOPIC}" 15; then
      launch_madgwick_bg
      if ! wait_for_topic "${GNNS_IMU_FILTERED_TOPIC}" 20; then
        echo "[gnns_vio_stack] WARN: Madgwick did not publish ${GNNS_IMU_FILTERED_TOPIC}" >&2
      fi
    else
      echo "[gnns_vio_stack] WARN: raw IMU ${IMU_TOPIC} unavailable — skipping Madgwick" >&2
    fi
  fi

  if [[ "${GNNS_USE_RGBD_SYNC}" == "1" ]]; then
    launch_rgbd_sync_bg
    echo "[gnns_vio_stack] Waiting for fused RGB-D topic ${GNNS_RGBD_TOPIC}…"
    if ! wait_for_topic "${GNNS_RGBD_TOPIC}" 45; then
      echo "[gnns_vio_stack] ERROR: no messages on ${GNNS_RGBD_TOPIC}. Check ${ODOM_LOG}" >&2
      exit 1
    fi
  else
    wait_for_camera_rgb_depth_or_exit
  fi

  launch_rgbd_odometry_bg
  ODOM_PID="${GNNS_RGBD_ODOM_PID}"
  if ! [[ "$ODOM_PID" =~ ^[0-9]+$ ]]; then
    echo "[gnns_vio_stack] ERROR: invalid rgbd_odometry PID '${ODOM_PID}' — check ${ODOM_LOG}" >&2
    exit 1
  fi

  cleanup_odom() {
    echo "[gnns_vio_stack] Stopping rgbd_odometry (PID=${ODOM_PID})…"
    kill_process_tree "${ODOM_PID}"
    if [[ -n "${GNNS_EKF_PID:-}" ]] && [[ "${GNNS_EKF_PID}" =~ ^[0-9]+$ ]]; then
      echo "[gnns_vio_stack] Stopping EKF (PID=${GNNS_EKF_PID})…"
      kill_process_tree "${GNNS_EKF_PID}"
    fi
    if [[ -n "${GNNS_RGBD_SYNC_PID:-}" ]] && [[ "${GNNS_RGBD_SYNC_PID}" =~ ^[0-9]+$ ]]; then
      echo "[gnns_vio_stack] Stopping rgbd_sync (PID=${GNNS_RGBD_SYNC_PID})…"
      kill_process_tree "${GNNS_RGBD_SYNC_PID}"
    fi
    if [[ -n "${GNNS_MADGWICK_PID:-}" ]] && [[ "${GNNS_MADGWICK_PID}" =~ ^[0-9]+$ ]]; then
      echo "[gnns_vio_stack] Stopping Madgwick (PID=${GNNS_MADGWICK_PID})…"
      kill_process_tree "${GNNS_MADGWICK_PID}"
    fi
  }
  trap cleanup_odom EXIT INT TERM

  echo "[gnns_vio_stack] rgbd_odometry PID=${ODOM_PID}"
  echo "[gnns_vio_stack] Waiting for odometry on ${ODOM_TOPIC}…"
  if ! wait_for_topic "${ODOM_TOPIC}" 60; then
    echo "[gnns_vio_stack] ERROR: no messages on ${ODOM_TOPIC}. Is the camera running?" >&2
    echo "  See: ${ODOM_LOG}" >&2
    echo "--- last 40 lines of ${ODOM_LOG} ---" >&2
    tail -n 40 "${ODOM_LOG}" 2>/dev/null >&2 || true
    exit 1
  fi
  if ! wait_for_tf_odom_camera_link 90; then
    exit 1
  fi
  local odom_min_hz="$(awk -v f="$ODOM_MAX_RATE" 'BEGIN{printf "%.1f", f/2}')"
  wait_for_topic_hz "${ODOM_TOPIC}" "${odom_min_hz}" 6 || \
    echo "[gnns_vio_stack] WARN: ${ODOM_TOPIC} below ${odom_min_hz} Hz — VO may be unhealthy" >&2

  if [[ "${GNNS_USE_EKF}" == "1" ]]; then
    : >"${EKF_LOG}"
    launch_ekf_bg
    if ! wait_for_topic "${GNNS_EKF_ODOM_TOPIC}" 30; then
      echo "[gnns_vio_stack] ERROR: EKF never published ${GNNS_EKF_ODOM_TOPIC}" >&2
      echo "--- last 30 lines of ${EKF_LOG} ---" >&2
      tail -n 30 "${EKF_LOG}" 2>/dev/null >&2 || true
      exit 1
    fi
  fi

  launch_rtabmap_slam
  cleanup_odom
  trap - EXIT INT TERM
}

# -----------------------------------------------------------------------------
# Full stack: RealSense background → rgbd_odometry → RTAB-Map foreground
# -----------------------------------------------------------------------------
launch_stack() {
  echo "[gnns_vio_stack] Starting full stack (RealSense → rgbd_odometry → RTAB-Map)…"
  echo "[gnns_vio_stack] RealSense log:      ${RS_LOG}"
  echo "[gnns_vio_stack] rgbd_odometry log:  ${ODOM_LOG}"
  echo "[gnns_vio_stack] RTAB-Map log:       ${RT_LOG}"
  if [[ "${GNNS_USE_RGBD_SYNC}" == "1" ]]; then
    echo "[gnns_vio_stack] GNNS_USE_RGBD_SYNC=1 → rgbd_sync → ${GNNS_RGBD_TOPIC}"
  fi

  : >"${RS_LOG}"
  : >"${ODOM_LOG}"
  : >"${RT_LOG}"

  RS_PID="$(launch_realsense_bg)"
  if ! [[ "$RS_PID" =~ ^[0-9]+$ ]]; then
    echo "[gnns_vio_stack] ERROR: invalid RealSense PID '${RS_PID}' (launch bug?) — check ${RS_LOG}" >&2
    exit 1
  fi

  ODOM_PID=""
  cleanup() {
    echo "[gnns_vio_stack] Shutting down…"
    if [[ -n "${GNNS_EKF_PID:-}" ]] && [[ "${GNNS_EKF_PID}" =~ ^[0-9]+$ ]]; then
      kill_process_tree "${GNNS_EKF_PID}"
    fi
    if [[ -n "${ODOM_PID}" ]] && [[ "$ODOM_PID" =~ ^[0-9]+$ ]]; then
      kill_process_tree "${ODOM_PID}"
    fi
    if [[ -n "${GNNS_RGBD_SYNC_PID:-}" ]] && [[ "${GNNS_RGBD_SYNC_PID}" =~ ^[0-9]+$ ]]; then
      kill_process_tree "${GNNS_RGBD_SYNC_PID}"
    fi
    if [[ -n "${GNNS_MADGWICK_PID:-}" ]] && [[ "${GNNS_MADGWICK_PID}" =~ ^[0-9]+$ ]]; then
      kill_process_tree "${GNNS_MADGWICK_PID}"
    fi
    kill_process_tree "${RS_PID}"
  }
  trap cleanup EXIT INT TERM

  echo "[gnns_vio_stack] RealSense PID=${RS_PID}"
  local cam_wait="${GNNS_STACK_CAMERA_WAIT_SEC:-10}"
  echo "[gnns_vio_stack] Waiting for camera streams (${cam_wait} s)…"
  sleep "${cam_wait}"

  if [[ "${GNNS_IMU_FILTER}" == "1" || "${WAIT_IMU}" == "1" ]]; then
    if ! wait_for_topic "$IMU_TOPIC" 30; then
      echo "[gnns_vio_stack] IMU not seen on ${IMU_TOPIC}" >&2
      echo "  Run: ros2 topic list | grep -i imu" >&2
      echo "  Or retry with: GNNS_USE_IMU=0 GNNS_IMU_FILTER=0 GNNS_WAIT_IMU=0 $0 stack" >&2
      exit 1
    fi
    wait_for_topic_hz "$IMU_TOPIC" 100 6 || true
  fi

  if [[ "${GNNS_IMU_FILTER}" == "1" ]]; then
    : >"${MAD_LOG}"
    launch_madgwick_bg
    echo "[gnns_vio_stack] Madgwick PID=${GNNS_MADGWICK_PID}"
    if ! wait_for_topic "${GNNS_IMU_FILTERED_TOPIC}" 30; then
      echo "[gnns_vio_stack] ERROR: Madgwick never published ${GNNS_IMU_FILTERED_TOPIC}" >&2
      echo "--- last 30 lines of ${MAD_LOG} ---" >&2
      tail -n 30 "${MAD_LOG}" 2>/dev/null >&2 || true
      exit 1
    fi
    wait_for_topic_hz "${GNNS_IMU_FILTERED_TOPIC}" 50 6 || true
  fi

  if [[ "${GNNS_USE_RGBD_SYNC}" == "1" ]]; then
    launch_rgbd_sync_bg
    echo "[gnns_vio_stack] Waiting for fused RGB-D topic ${GNNS_RGBD_TOPIC}…"
    if ! wait_for_topic "${GNNS_RGBD_TOPIC}" 45; then
      echo "[gnns_vio_stack] ERROR: no messages on ${GNNS_RGBD_TOPIC}. Check ${ODOM_LOG} and ${RS_LOG}" >&2
      exit 1
    fi
  else
    wait_for_camera_rgb_depth_or_exit
  fi

  launch_rgbd_odometry_bg
  ODOM_PID="${GNNS_RGBD_ODOM_PID}"
  if ! [[ "$ODOM_PID" =~ ^[0-9]+$ ]]; then
    echo "[gnns_vio_stack] ERROR: invalid rgbd_odometry PID '${ODOM_PID}' — check ${ODOM_LOG}" >&2
    exit 1
  fi
  echo "[gnns_vio_stack] rgbd_odometry PID=${ODOM_PID}"
  echo "[gnns_vio_stack] Waiting for odometry on ${ODOM_TOPIC}…"
  if ! wait_for_topic "${ODOM_TOPIC}" 60; then
    echo "[gnns_vio_stack] ERROR: no odometry on ${ODOM_TOPIC}. Check ${ODOM_LOG} and ${RS_LOG}" >&2
    echo "--- last 40 lines of ${ODOM_LOG} ---" >&2
    tail -n 40 "${ODOM_LOG}" 2>/dev/null >&2 || true
    exit 1
  fi
  if ! wait_for_tf_odom_camera_link 90; then
    exit 1
  fi
  local odom_min_hz="$(awk -v f="$ODOM_MAX_RATE" 'BEGIN{printf "%.1f", f/2}')"
  wait_for_topic_hz "${ODOM_TOPIC}" "${odom_min_hz}" 6 || \
    echo "[gnns_vio_stack] WARN: ${ODOM_TOPIC} below ${odom_min_hz} Hz — VO may be unhealthy" >&2

  if [[ "${GNNS_USE_EKF}" == "1" ]]; then
    : >"${EKF_LOG}"
    launch_ekf_bg
    echo "[gnns_vio_stack] EKF PID=${GNNS_EKF_PID}"
    if ! wait_for_topic "${GNNS_EKF_ODOM_TOPIC}" 30; then
      echo "[gnns_vio_stack] ERROR: EKF never published ${GNNS_EKF_ODOM_TOPIC}" >&2
      echo "--- last 30 lines of ${EKF_LOG} ---" >&2
      tail -n 30 "${EKF_LOG}" 2>/dev/null >&2 || true
      exit 1
    fi
    wait_for_topic_hz "${GNNS_EKF_ODOM_TOPIC}" 25 6 || true
  fi

  launch_rtabmap_slam
  cleanup
  trap - EXIT INT TERM
}

# -----------------------------------------------------------------------------
# gNNS Python mission (odometry source from env, default ros2 → /odom)
# -----------------------------------------------------------------------------
launch_mission() {
  local ROOT="${GNNS_PROJECT_ROOT:-$PROJECT_ROOT}"
  export PYTHONPATH="${ROOT}:${PYTHONPATH:-}"
  cd "$ROOT"
  local _vio="${GNNS_MISSION_VIO_SOURCE:-ros2}"
  echo "[gnns_vio_stack] gNNS mission from ${ROOT}"
  echo "  Odometry: --vio-source ${_vio}  (override with GNNS_MISSION_VIO_SOURCE)"
  echo "  Ensure config/vio_config.yaml matches (odom_topic / odom_source)."
  exec python3 -m gnns_drone --vio-source "${_vio}" "$@"
}

# -----------------------------------------------------------------------------
# Standalone modes: launch Madgwick or EKF only (RealSense / rgbd_odometry must
# already be running). Useful for incremental bring-up and debugging.
# -----------------------------------------------------------------------------
cmd_imu() {
  echo "[gnns_vio_stack] IMU filter session — Madgwick → ${GNNS_IMU_FILTERED_TOPIC}"
  echo "  Requires: RealSense publishing on ${IMU_TOPIC}"
  : >"${MAD_LOG}"
  if ! wait_for_topic "${IMU_TOPIC}" 15; then
    echo "[gnns_vio_stack] ERROR: raw IMU ${IMU_TOPIC} not seen — start RealSense first." >&2
    exit 1
  fi
  launch_madgwick_bg
  echo "[gnns_vio_stack] Madgwick PID=${GNNS_MADGWICK_PID}"
  trap 'kill_process_tree "${GNNS_MADGWICK_PID}"' EXIT INT TERM
  if ! wait_for_topic "${GNNS_IMU_FILTERED_TOPIC}" 30; then
    echo "[gnns_vio_stack] ERROR: Madgwick never published ${GNNS_IMU_FILTERED_TOPIC}" >&2
    tail -n 30 "${MAD_LOG}" 2>/dev/null >&2 || true
    exit 1
  fi
  wait_for_topic_hz "${GNNS_IMU_FILTERED_TOPIC}" 50 6 || true
  echo "[gnns_vio_stack] IMU filter ready. Ctrl+C to stop."
  wait "${GNNS_MADGWICK_PID}" 2>/dev/null || true
}

cmd_ekf() {
  echo "[gnns_vio_stack] EKF session — ${GNNS_EKF_ODOM_TOPIC}"
  echo "  Requires: ${ODOM_TOPIC} and ${GNNS_IMU_FILTERED_TOPIC} both publishing"
  : >"${EKF_LOG}"
  if ! wait_for_topic "${ODOM_TOPIC}" 30; then
    echo "[gnns_vio_stack] ERROR: ${ODOM_TOPIC} not seen — run rtabmap/stack first." >&2
    exit 1
  fi
  if ! wait_for_topic "${GNNS_IMU_FILTERED_TOPIC}" 30; then
    echo "[gnns_vio_stack] ERROR: ${GNNS_IMU_FILTERED_TOPIC} not seen — run imu/stack first." >&2
    exit 1
  fi
  launch_ekf_bg
  echo "[gnns_vio_stack] EKF PID=${GNNS_EKF_PID}"
  trap 'kill_process_tree "${GNNS_EKF_PID}"' EXIT INT TERM
  if ! wait_for_topic "${GNNS_EKF_ODOM_TOPIC}" 30; then
    echo "[gnns_vio_stack] ERROR: EKF never published ${GNNS_EKF_ODOM_TOPIC}" >&2
    tail -n 30 "${EKF_LOG}" 2>/dev/null >&2 || true
    exit 1
  fi
  wait_for_topic_hz "${GNNS_EKF_ODOM_TOPIC}" 25 6 || true
  echo "[gnns_vio_stack] EKF ready. Ctrl+C to stop."
  wait "${GNNS_EKF_PID}" 2>/dev/null || true
}

# -----------------------------------------------------------------------------
# Print health summary: topic list, rates, TF, log tails
# -----------------------------------------------------------------------------
cmd_diagnose() {
  echo "=== ros2 topic list (camera / imu / odom / rtabmap) ==="
  ros2 topic list 2>/dev/null | grep -E -i 'camera|imu|odom|rtabmap|filtered' || true
  echo ""
  echo "=== Average rates over 5s (skipped if topic absent) ==="
  for t in "${IMU_TOPIC}" "${GNNS_IMU_FILTERED_TOPIC}" "${ODOM_TOPIC}" "${GNNS_EKF_ODOM_TOPIC}"; do
    if ros2 topic info "$t" >/dev/null 2>&1; then
      printf "  %-32s " "$t"
      timeout 7 ros2 topic hz "$t" --window 5 2>/dev/null | grep -m 1 'average rate' || echo "(no samples)"
    fi
  done
  echo ""
  echo "=== TF odom -> camera_link ==="
  timeout 3 ros2 run tf2_ros tf2_echo "${ODOM_FRAME_FOR_TF}" "${CAMERA_FRAME_FOR_TF}" 2>&1 | grep -m 1 'Translation' || echo "  (no transform — VO TF not connected)"
  echo ""
  for L in "${RS_LOG}" "${MAD_LOG}" "${ODOM_LOG}" "${EKF_LOG}" "${RT_LOG}"; do
    [[ -f "$L" ]] || continue
    echo "--- tail -n 10 ${L} ---"
    tail -n 10 "$L" 2>/dev/null || true
    echo ""
  done
  echo "=== If IMU path is wrong, set e.g. ==="
  echo "  export GNNS_IMU_TOPIC=/camera/imu      # raw IMU"
  echo "  export GNNS_IMU_FILTERED_TOPIC=/imu/data   # Madgwick output"
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
    launch_rtabmap_with_odom
    ;;
  imu)
    cmd_imu
    ;;
  ekf)
    cmd_ekf
    ;;
  stack)
    launch_stack
    ;;
  mission)
    launch_mission "$@"
    ;;
  diagnose)
    cmd_diagnose
    ;;
  help|--help|-h)
    sed -n '1,90p' "$0" | sed 's/^# \{0,1\}//'
    echo ""
    echo "Commands:  realsense | rtabmap | imu | ekf | stack | competition | mission | diagnose | help"
    echo ""
    echo "Logs (stack / rtabmap): ${RS_LOG} , ${MAD_LOG} , ${ODOM_LOG} , ${EKF_LOG} , ${RT_LOG}"
    ;;
  *)
    echo "Unknown mode: $MODE — use: realsense | rtabmap | imu | ekf | stack | competition | mission | diagnose | help" >&2
    exit 1
    ;;
esac
