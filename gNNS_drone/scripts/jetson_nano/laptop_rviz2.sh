#!/usr/bin/env bash
# ================================================================
# laptop_rviz2.sh — Laptop-side RViz2 viewer for gNNS Drone
# ================================================================
# Sets up DDS environment and launches RViz2 to visualize topics
# streamed from the Jetson over WiFi.
#
# Usage:
#   ./laptop_rviz2.sh              # domain 42, humble
#   ./laptop_rviz2.sh 42           # explicit domain
#   ./laptop_rviz2.sh 42 jazzy     # domain + distro
#   ./laptop_rviz2.sh humble       # just distro
# ================================================================
set -euo pipefail

DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

if [[ $# -eq 0 ]]; then
    :
elif [[ $# -eq 1 ]]; then
    if [[ "$1" =~ ^[0-9]+$ ]]; then
        DOMAIN_ID="$1"
    elif [[ "$1" =~ ^(humble|jazzy|iron|foxy)$ ]]; then
        ROS_DISTRO="$1"
    else
        echo "[Error] Unknown argument: $1 (use a number for domain, or humble|jazzy|...)" >&2
        exit 2
    fi
else
    DOMAIN_ID="$1"
    ROS_DISTRO="$2"
fi

RMW="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f "$SETUP" ]]; then
    echo "[Error] Missing: $SETUP" >&2
    echo "Install ROS 2 ${ROS_DISTRO} on this machine, or pass distro as second arg." >&2
    exit 2
fi

export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="$RMW"

# shellcheck source=/dev/null
source "$SETUP"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Optional workspace overlay (same pattern as ros_bridge.sh on Jetson)
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "$HOME/ros2_ws/install/setup.bash"
elif [[ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "[Error] 'ros2' not in PATH after sourcing $SETUP" >&2
    exit 2
fi

echo ""
echo "  ┌──────────────────────────────────────┐"
echo "  │  gNNS Drone — Laptop RViz2 Viewer    │"
echo "  │  ROS_DOMAIN_ID=$DOMAIN_ID            │"
echo "  │  RMW=$RMW                            │"
echo "  │  Distro=$ROS_DISTRO                  │"
echo "  └──────────────────────────────────────┘"
echo ""
echo "  Listing topics from Jetson..."
ros2 topic list 2>/dev/null || echo "  (no topics yet — is the Jetson running?)"
echo ""
echo "  Launching RViz2..."
echo "  Set Fixed Frame → odom"
echo "  Add: Image (/camera/color/image_raw), PointCloud2, Odometry (/odom), TF"
echo ""

ros2 run rviz2 rviz2
