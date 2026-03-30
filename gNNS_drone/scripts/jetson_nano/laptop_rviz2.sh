#!/usr/bin/env bash
# ================================================================
# laptop_rviz2.sh — Laptop-side RViz2 viewer for gNNS Drone
# ================================================================
# Sets up DDS environment and launches RViz2 to visualize topics
# streamed from the Jetson over WiFi.
#
# Usage:
#   ./laptop_rviz2.sh              # domain 42, humble, full RViz layout
#   ./laptop_rviz2.sh 42           # explicit domain
#   ./laptop_rviz2.sh 42 jazzy     # domain + distro
#   ./laptop_rviz2.sh humble       # just distro
#   ./laptop_rviz2.sh --topics-only
#   ./laptop_rviz2.sh 42 --topics-only
#   ./laptop_rviz2.sh --cyclone-iface wlan0
# ================================================================
set -euo pipefail

DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
TOPICS_ONLY=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# shellcheck source=cyclonedds_env.sh
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/cyclonedds_env.sh"

# ---- Argument parsing (domain / distro / flags) ----
POSITIONAL=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --topics-only) TOPICS_ONLY=1; shift ;;
        --cyclone-iface)
            if [[ -z "${2:-}" ]]; then echo "[Error] --cyclone-iface needs a value" >&2; exit 2; fi
            export CYCLONE_IFACE="$2"
            shift 2
            ;;
        -*) echo "[Error] Unknown flag: $1" >&2; exit 2 ;;
        *) POSITIONAL+=("$1"); shift ;;
    esac
done
set -- "${POSITIONAL[@]}"

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

gnns_set_cyclonedds_uri

# ament/ros2: force Python 3 used by CLI and RViz2 (fixes mixed venv/conda interpreter errors)
if _py="$(which python3 2>/dev/null)" && [[ -n "$_py" ]]; then
    export AMENT_PYTHON_EXECUTABLE="$_py"
fi
unset _py

# shellcheck source=/dev/null
source "$SETUP"

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

if [[ "$TOPICS_ONLY" == "1" ]]; then
    echo "  (--topics-only: exiting)"
    exit 0
fi

RVIZ_CFG="${PROJECT_ROOT}/config/drone_monitor.rviz"
RVIZ_ARGS=()
if [[ -f "$RVIZ_CFG" ]]; then
    RVIZ_ARGS=(-d "$RVIZ_CFG")
    echo "  Using RViz config: $RVIZ_CFG"
else
    echo "  [Warn] Missing $RVIZ_CFG — starting blank RViz2"
fi

echo ""
echo "  Launching RViz2..."
echo "  Tip: Fixed Frame can be 'odom' or 'map' depending on what you visualize."
echo ""

ros2 run rviz2 rviz2 "${RVIZ_ARGS[@]}"
