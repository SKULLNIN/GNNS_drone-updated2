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
# shellcheck source=/dev/null
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/fastdds_wifi_env.sh"

echo ""
echo "=== gNNS bridge check | ROS_DOMAIN_ID=$ROS_DOMAIN_ID | RMW=$RMW_IMPLEMENTATION ==="
echo ""
echo "--- ros2 topic list ---"
ros2 topic list 2>/dev/null || true
echo ""

echo "--- /odom rate (5s max) ---"
timeout 5s ros2 topic hz /odom 2>/dev/null | head -5 || echo "[WARN] /odom not seen or not publishing"
echo ""

echo "--- camera RGB rate (8s max; needs messages on this machine) ---"
_IMG=""
if ros2 topic list 2>/dev/null | grep -qx '/camera/camera/color/image_raw'; then
    _IMG=/camera/camera/color/image_raw
elif ros2 topic list 2>/dev/null | grep -qx '/camera/camera/color/image_rect_color'; then
    _IMG=/camera/camera/color/image_rect_color
elif ros2 topic list 2>/dev/null | grep -qx '/camera/color/image_raw'; then
    _IMG=/camera/color/image_raw
fi
if [[ -n "$_IMG" ]]; then
    echo "Topic: $_IMG"
    ros2 topic info "$_IMG" -v 2>/dev/null | head -25 || true
    echo ""
    timeout 8s ros2 topic hz "$_IMG" 2>/dev/null | head -8 || echo "[WARN] no samples (WiFi/DDS/QoS/firewall?)"
else
    echo "[WARN] no common color image topic in ros2 topic list"
fi
echo ""
echo "Done."
