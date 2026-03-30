#!/usr/bin/env bash
# ================================================================
# laptop_rviz2.sh — Laptop RViz2 for gNNS (Jetson topics over DDS)
# ================================================================
# Uses Fast DDS by default (same as d455_launch.sh / rtabmap_vio.sh).
# Optional first argument: ROS_DOMAIN_ID (number), e.g. 42
#
# Usage:
#   chmod +x scripts/jetson_nano/laptop_rviz2.sh   # if Permission denied
#   ./scripts/jetson_nano/laptop_rviz2.sh
#   ./scripts/jetson_nano/laptop_rviz2.sh 42
# ================================================================
# Do not use set -u — ROS setup.bash references unset variables.
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "$SETUP" ]]; then
    echo "[Error] Missing $SETUP — install ROS 2 $ROS_DISTRO" >&2
    exit 2
fi

if [[ "${1:-}" =~ ^[0-9]+$ ]]; then
    export ROS_DOMAIN_ID="$1"
    shift
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# shellcheck source=/dev/null
source "$SETUP"
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "$HOME/ros2_ws/install/setup.bash"
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "[Error] ros2 not in PATH after sourcing $SETUP" >&2
    exit 2
fi

echo ""
echo "  gNNS — Laptop RViz2  |  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  |  RMW=$RMW_IMPLEMENTATION"
echo ""
echo "  Topics from Jetson:"
ros2 topic list 2>/dev/null || echo "  (none — is the Jetson running?)"
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_CFG="$(cd "$SCRIPT_DIR/../.." && pwd)/config/drone_monitor.rviz"
if [[ -f "$RVIZ_CFG" ]]; then
    echo "  Launching RViz2 with: $RVIZ_CFG"
    exec ros2 run rviz2 rviz2 -d "$RVIZ_CFG"
else
    echo "  [Warn] No drone_monitor.rviz — starting blank RViz2"
    exec ros2 run rviz2 rviz2
fi
