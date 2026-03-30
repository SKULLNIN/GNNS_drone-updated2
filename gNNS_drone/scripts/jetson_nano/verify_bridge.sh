#!/usr/bin/env bash
# ================================================================
# verify_bridge.sh — Quick DDS / topic check (run on laptop)
# ================================================================
# Do not use set -u — ROS setup.bash references unset variables.
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "$SETUP" ]]; then
    echo "[Error] Missing $SETUP" >&2
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

echo ""
echo "=== gNNS bridge check | ROS_DOMAIN_ID=$ROS_DOMAIN_ID | RMW=$RMW_IMPLEMENTATION ==="
echo ""
echo "--- ros2 topic list ---"
ros2 topic list 2>/dev/null || true
echo ""

echo "--- /odom rate (5s max) ---"
timeout 5s ros2 topic hz /odom 2>/dev/null | head -5 || echo "[WARN] /odom not seen or not publishing"
echo ""

echo "--- camera RGB (namespaced /camera/camera/...) ---"
timeout 5s ros2 topic hz /camera/camera/color/image_raw 2>/dev/null | head -5 || \
  timeout 5s ros2 topic hz /camera/color/image_raw 2>/dev/null | head -5 || \
  echo "[WARN] camera color topic not seen"
echo ""
echo "Done."
