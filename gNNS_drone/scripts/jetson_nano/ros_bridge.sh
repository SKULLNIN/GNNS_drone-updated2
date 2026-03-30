#!/usr/bin/env bash
# ================================================================
# ros_bridge.sh — Jetson ROS 2 bridge for gNNS Drone
# ================================================================
# Launches RealSense + RTAB-Map (or ORB-SLAM3) + optional mission
# inside a tmux session, with DDS configured for multi-machine
# streaming to a laptop running RViz2.
#
# Usage:
#   ./ros_bridge.sh                          # full mission
#   ./ros_bridge.sh --viz-only               # camera + VIO only
#   ./ros_bridge.sh --viz-only --imu --pointcloud  # everything for RViz2
#   ./ros_bridge.sh --print-only             # dry run, no ROS needed
#   ./ros_bridge.sh --no-tmux                # foreground (for systemd)
#   ./ros_bridge.sh --light-maps             # lower RTAB-Map cloud bandwidth
#   ./ros_bridge.sh --rtabmap-gui            # enable rtabmap_viz on Jetson (heavy)
#
# On laptop:
#   ./scripts/jetson_nano/laptop_rviz2.sh 42
# ================================================================
# Do not use set -u — ROS setup.bash uses unset vars (e.g. AMENT_TRACE_SETUP_FILES).
set -eo pipefail

# ---- Defaults ----
ROS_DISTRO_DEFAULT="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_DEFAULT}/setup.bash"
OVERLAY_SETUP=""
DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RMW="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# Multi-machine DDS: never use localhost-only for the bridge (ignore prior env).
LOCALHOST="0"
ODOM_SOURCE="ros2"
TMUX_SESSION="drone"
USE_TMUX=1
PRINT_ONLY=0
VIZ_ONLY=0
POINTCLOUD=0
IMU=0
MAVLINK_OUT=""
RTABMAP_GUI_VAL="false"
RTAB_LIGHT_MAPS=0

MISSION_ARGS=()
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Optional workspace overlay
for overlay in "$HOME/ros2_ws/install/setup.bash" \
               "$PROJECT_ROOT/ros2_ws/install/setup.bash"; do
    if [[ -f "$overlay" ]]; then
        OVERLAY_SETUP="$overlay"
        break
    fi
done

# ---- Helpers ----
require_arg() {
    if [[ -z "${2:-}" ]]; then
        echo "[Error] $1 requires a value" >&2
        exit 2
    fi
}

# ---- Argument parsing ----
while [[ $# -gt 0 ]]; do
    case "$1" in
        --domain-id)     require_arg "$1" "${2:-}"; DOMAIN_ID="$2"; shift 2 ;;
        --rmw)           require_arg "$1" "${2:-}"; RMW="$2"; shift 2 ;;
        --odom-source)   require_arg "$1" "${2:-}"; ODOM_SOURCE="$2"; shift 2 ;;
        --distro)        require_arg "$1" "${2:-}"; ROS_DISTRO_DEFAULT="$2"
                         ROS_SETUP="/opt/ros/${ROS_DISTRO_DEFAULT}/setup.bash"; shift 2 ;;
        --session)       require_arg "$1" "${2:-}"; TMUX_SESSION="$2"; shift 2 ;;
        --mavlink-out)   require_arg "$1" "${2:-}"; MAVLINK_OUT="$2"; shift 2 ;;
        --no-tmux)       USE_TMUX=0; shift ;;
        --print-only)    PRINT_ONLY=1; shift ;;
        --viz-only)      VIZ_ONLY=1; shift ;;
        --pointcloud)    POINTCLOUD=1; shift ;;
        --imu)           IMU=1; shift ;;
        --cloud-map)     shift ;;  # RTAB-Map publishes cloud/grid by default; for documentation
        --grid-map)      shift ;;
        --light-maps)    RTAB_LIGHT_MAPS=1; shift ;;
        --rtabmap-gui)   RTABMAP_GUI_VAL="true"; shift ;;
        --)              shift; MISSION_ARGS+=("$@"); break ;;
        -*)              echo "[Error] Unknown flag: $1" >&2; exit 2 ;;
        *)               MISSION_ARGS+=("$1"); shift ;;
    esac
done

# Export DDS variables
export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY="$LOCALHOST"
export RMW_IMPLEMENTATION="$RMW"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/fastdds_wifi_env.sh"

# ---- Verify ROS setup file exists ----
if [[ "$PRINT_ONLY" != "1" && ! -f "$ROS_SETUP" ]]; then
    echo "[Error] Missing: $ROS_SETUP" >&2
    echo "Install ROS 2 ${ROS_DISTRO_DEFAULT} or pass --distro <name>" >&2
    exit 2
fi

# ROS 2 Humble/Jazzy: rtabmap.launch.py lives in rtabmap_launch, not rtabmap_ros.
gnns_rtabmap_launch_package() {
    local distro="$ROS_DISTRO_DEFAULT"
    local opt_share="/opt/ros/${distro}/share"
    local ws_share=""
    if [[ -n "${OVERLAY_SETUP:-}" ]]; then
        ws_share="$(cd "$(dirname "$OVERLAY_SETUP")/.." && pwd)/share"
    fi
    local s
    for s in "$ws_share" "$opt_share"; do
        [[ -z "$s" || ! -d "$s" ]] && continue
        if [[ -f "$s/rtabmap_launch/launch/rtabmap.launch.py" ]]; then
            echo "rtabmap_launch"
            return 0
        fi
        if [[ -f "$s/rtabmap_ros/launch/rtabmap.launch.py" ]]; then
            echo "rtabmap_ros"
            return 0
        fi
    done
    echo "rtabmap_launch"
}

# ================================================================
# Command generators (each prints a bash snippet for a tmux pane)
# ================================================================

dds_env_snippet() {
    cat <<EOF
export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY="$LOCALHOST"
export RMW_IMPLEMENTATION="$RMW"
EOF
}

realsense_cmd() {
    local args="enable_color:=true enable_depth:=true"
    args+=" enable_infra1:=false enable_infra2:=false"
    args+=" align_depth.enable:=true"
    [[ "$IMU" == "1" ]] && args+=" enable_gyro:=true enable_accel:=true"
    [[ "$POINTCLOUD" == "1" ]] && args+=" pointcloud.enable:=true"
    cat <<EOF
source "$ROS_SETUP"
${OVERLAY_SETUP:+source "$OVERLAY_SETUP"}
$(dds_env_snippet)
ros2 launch realsense2_camera rs_launch.py $args
EOF
}

rtabmap_cmd() {
    local rtab_pkg
    rtab_pkg="$(gnns_rtabmap_launch_package)"
    if [[ "$RTAB_LIGHT_MAPS" == "1" ]]; then
        cat <<EOF
source "$ROS_SETUP"
${OVERLAY_SETUP:+source "$OVERLAY_SETUP"}
$(dds_env_snippet)
ros2 launch ${rtab_pkg} rtabmap.launch.py \\
  rgb_topic:=/camera/color/image_raw \\
  depth_topic:=/camera/depth/image_rect_raw \\
  camera_info_topic:=/camera/color/camera_info \\
  frame_id:=camera_link \\
  approx_sync:=true \\
  odom_frame_id:=odom \\
  visual_odometry:=true \\
  odom_topic:=/odom \\
  map_topic:=/map \\
  publish_tf_map:=true \\
  publish_tf_odom:=true \\
  rtabmap_viz:=${RTABMAP_GUI_VAL} \\
  rtabmap_args:='cloud_decimation 8'
EOF
    else
        cat <<EOF
source "$ROS_SETUP"
${OVERLAY_SETUP:+source "$OVERLAY_SETUP"}
$(dds_env_snippet)
ros2 launch ${rtab_pkg} rtabmap.launch.py \\
  rgb_topic:=/camera/color/image_raw \\
  depth_topic:=/camera/depth/image_rect_raw \\
  camera_info_topic:=/camera/color/camera_info \\
  frame_id:=camera_link \\
  approx_sync:=true \\
  odom_frame_id:=odom \\
  visual_odometry:=true \\
  odom_topic:=/odom \\
  map_topic:=/map \\
  publish_tf_map:=true \\
  publish_tf_odom:=true \\
  rtabmap_viz:=${RTABMAP_GUI_VAL}
EOF
    fi
}

orbslam3_cmd() {
    cat <<EOF
echo "[FATAL] ORB-SLAM3 ROS2 wrapper not configured."
echo "  1. Install ORB-SLAM3 + its ROS2 wrapper"
echo "  2. Update this function with your launch command"
exit 1
EOF
}

mission_cmd() {
    local mission_args_str=""
    if [[ ${#MISSION_ARGS[@]} -gt 0 ]]; then
        printf -v mission_args_str '%q ' "${MISSION_ARGS[@]}"
    fi
    cat <<EOF
source "$ROS_SETUP"
${OVERLAY_SETUP:+source "$OVERLAY_SETUP"}
$(dds_env_snippet)
cd "$PROJECT_ROOT"
python3 -m gnns_drone --vio-source "$ODOM_SOURCE" $mission_args_str
EOF
}

mavproxy_cmd() {
    cat <<EOF
mavproxy.py \\
  --master=/dev/ttyACM0 \\
  --baudrate=921600 \\
  --out=udp:${MAVLINK_OUT}
EOF
}

# ================================================================
# --print-only: show what would run and exit
# ================================================================
if [[ "$PRINT_ONLY" == "1" ]]; then
    echo "=== --print-only (no processes started) ==="
    echo "--- Pane 1: RealSense ---"
    realsense_cmd
    echo "--- Pane 2: RTAB-Map or ORB-SLAM3 ---"
    if [[ "$ODOM_SOURCE" == "ros2" ]]; then
        rtabmap_cmd
    else
        orbslam3_cmd
    fi
    if [[ "$VIZ_ONLY" != "1" ]]; then
        echo "--- Mission ---"
        mission_cmd
    else
        echo "--- Mission: skipped (--viz-only) ---"
    fi
    [[ -n "$MAVLINK_OUT" ]] && echo "--- MAVProxy ---" && mavproxy_cmd
    echo "=== On laptop: ./scripts/jetson_nano/laptop_rviz2.sh $DOMAIN_ID ==="
    exit 0
fi

# ================================================================
# --no-tmux: run foreground (for systemd / Docker)
# ================================================================
if [[ "$USE_TMUX" == "0" ]]; then
    trap 'kill $(jobs -p) 2>/dev/null; wait' EXIT INT TERM

    echo "[ros_bridge] Starting foreground (no tmux)..."
    bash -c "$(realsense_cmd)" &
    sleep 3

    if [[ "$ODOM_SOURCE" == "ros2" ]]; then
        bash -c "$(rtabmap_cmd)" &
    else
        bash -c "$(orbslam3_cmd)" &
    fi
    sleep 2

    if [[ "$VIZ_ONLY" != "1" ]]; then
        bash -c "$(mission_cmd)" &
    fi

    [[ -n "$MAVLINK_OUT" ]] && bash -c "$(mavproxy_cmd)" &

    echo "[ros_bridge] All processes started. Ctrl+C to stop."
    wait
    exit 0
fi

# ================================================================
# tmux session
# ================================================================
if tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    echo "[ros_bridge] tmux session '$TMUX_SESSION' already exists."
    echo "  Attach:  tmux attach -t $TMUX_SESSION"
    echo "  Kill:    tmux kill-session -t $TMUX_SESSION"
    exit 1
fi

echo "[ros_bridge] Creating tmux session '$TMUX_SESSION'..."

# Window 1: ros (realsense | odom)
tmux new-session -d -s "$TMUX_SESSION" -n ros
tmux send-keys -t "${TMUX_SESSION}:ros" "$(realsense_cmd)" C-m

tmux split-window -h -t "${TMUX_SESSION}:ros"
sleep 1
if [[ "$ODOM_SOURCE" == "ros2" ]]; then
    tmux send-keys -t "${TMUX_SESSION}:ros.1" "sleep 4 && $(rtabmap_cmd)" C-m
else
    tmux send-keys -t "${TMUX_SESSION}:ros.1" "sleep 4 && $(orbslam3_cmd)" C-m
fi

# Window 2: mission (unless --viz-only)
if [[ "$VIZ_ONLY" != "1" ]]; then
    tmux new-window -t "$TMUX_SESSION" -n mission
    tmux send-keys -t "${TMUX_SESSION}:mission" "sleep 8 && $(mission_cmd)" C-m
fi

# Window 3: mavproxy (if --mavlink-out)
if [[ -n "$MAVLINK_OUT" ]]; then
    tmux new-window -t "$TMUX_SESSION" -n mavproxy
    tmux send-keys -t "${TMUX_SESSION}:mavproxy" "$(mavproxy_cmd)" C-m
fi

echo ""
echo "  ┌─────────────────────────────────────────┐"
echo "  │  tmux session: $TMUX_SESSION            │"
echo "  │  ROS_DOMAIN_ID=$DOMAIN_ID               │"
echo "  │  RMW=$RMW                               │"
echo "  │  Attach: tmux attach -t $TMUX_SESSION   │"
echo "  │                                         │"
echo "  │  On laptop:                             │"
echo "  │    ./scripts/jetson_nano/laptop_rviz2.sh $DOMAIN_ID │"
echo "  └─────────────────────────────────────────┘"
echo ""

tmux select-window -t "${TMUX_SESSION}:ros"
tmux attach -t "$TMUX_SESSION"
