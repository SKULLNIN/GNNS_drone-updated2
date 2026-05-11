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
#   RQT_IMAGE=1 ./scripts/jetson_nano/laptop_rviz2.sh   # also open rqt_image_view
#
# If RViz dies with "qt.qpa.xcb: could not connect to display": DISPLAY points at an X
# server you cannot access (common: SSH without -Y, Cursor remote terminal, or wrong :0).
# Fix: run from the laptop's desktop terminal, or ssh -Y user@host, or match the session's DISPLAY.
#   GNNS_SKIP_RVIZ_DISPLAY_CHECK=1  # skip xdpyinfo preflight (not recommended)
# ================================================================
# Do not use set -u — ROS setup.bash references unset variables.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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
# shellcheck source=/dev/null
source "$SCRIPT_DIR/fastdds_wifi_env.sh"

if ! command -v ros2 >/dev/null 2>&1; then
    echo "[Error] ros2 not in PATH after sourcing $SETUP" >&2
    exit 2
fi

_gnns_rviz_display_ready() {
    if [[ "${GNNS_SKIP_RVIZ_DISPLAY_CHECK:-0}" == "1" ]]; then
        return 0
    fi
    if [[ -n "${WAYLAND_DISPLAY:-}" ]] && [[ -z "${DISPLAY:-}" ]]; then
        return 0
    fi
    if [[ -z "${DISPLAY:-}" ]]; then
        echo "[Error] No DISPLAY (and no WAYLAND for Qt). RViz cannot open a window here." >&2
        echo "  • Run from the machine's graphical terminal, or: ssh -Y user@host" >&2
        echo "  • Cursor/SSH: do not rely on DISPLAY=:0 unless that X server is really yours." >&2
        return 1
    fi
    if command -v xdpyinfo >/dev/null 2>&1; then
        if xdpyinfo >/dev/null 2>&1; then
            return 0
        fi
    elif command -v xset >/dev/null 2>&1; then
        if timeout 3 xset q >/dev/null 2>&1; then
            return 0
        fi
    else
        echo "[Warn] No xdpyinfo/xset — cannot verify DISPLAY=$DISPLAY (install x11-utils or x11-xserver-utils)." >&2
        return 0
    fi
    echo "[Error] DISPLAY=$DISPLAY is unreachable (same as: qt.qpa.xcb: could not connect to display)." >&2
    echo "  • Plain SSH → use: ssh -Y user@laptop   then rerun this script" >&2
    echo "  • On the laptop desktop: echo \$DISPLAY (often :0 or :1) and run from that session" >&2
    echo "  • Wrong machine: RViz must run where the monitor is (same ROS_DOMAIN_ID as Jetson)." >&2
    return 1
}

echo ""
echo "  gNNS — Laptop RViz2  |  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  |  RMW=$RMW_IMPLEMENTATION"
if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
    echo "  FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
else
    echo "  (no fastdds_wifi_images.xml — clone path wrong or GNNS_SKIP_FASTDDS_WIFI=1)"
fi
echo ""
echo "  Topics from Jetson:"
TLIST=$(ros2 topic list 2>/dev/null || true)
if [[ -z "$TLIST" ]]; then
    echo "  (none — Jetson off, wrong ROS_DOMAIN_ID, firewall, or RMW mismatch)"
else
    echo "$TLIST"
fi
echo ""

IMG_TOPIC=""
if echo "$TLIST" | grep -qx '/camera/camera/color/image_raw'; then
    IMG_TOPIC=/camera/camera/color/image_raw
elif echo "$TLIST" | grep -qx '/camera/camera/color/image_rect_color'; then
    IMG_TOPIC=/camera/camera/color/image_rect_color
elif echo "$TLIST" | grep -qx '/camera/color/image_raw'; then
    IMG_TOPIC=/camera/color/image_raw
fi

if [[ -n "$IMG_TOPIC" ]]; then
    echo "  [OK] Color image topic: $IMG_TOPIC"
    echo "       In RViz → Displays → Camera → Image Topic, set this if the preview is blank."
    echo "       Checking message rate (8 s) — if this hangs at ~0 Hz, RViz will not show video:"
    timeout 8s ros2 topic hz "$IMG_TOPIC" 2>/dev/null | head -10 || true
    echo ""
    echo "  If rate is 0 but the topic exists: WiFi UDP loss (try Ethernet), or Linux fragment buffers:"
    echo "    sudo sysctl -w net.ipv4.ipfrag_time=3"
    echo "    sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728"
    echo "  Fallback viewer (same DDS subscriber stack as RViz Image):"
    echo "    ros2 run rqt_image_view rqt_image_view --ros-args -r image:=$IMG_TOPIC"
else
    echo "  [WARN] No standard color image topic — start d455_launch.sh on Jetson (see README)."
fi

if echo "$TLIST" | grep -q '/camera/camera/color/image_raw'; then
    :
elif echo "$TLIST" | grep -qE '/camera/color/image_raw'; then
    echo ""
    echo "  [WARN] Found /camera/color/... but not /camera/camera/..."
    echo "        Pick the topic that exists in RViz, or use d455_launch.sh (double camera namespace)."
fi
if echo "$TLIST" | grep -q '^/odom$'; then
    echo "  [OK] /odom is visible (start rtabmap_vio.sh on Jetson if it should move)."
else
    echo "  [WARN] /odom not in list — run rtabmap_vio.sh on Jetson after D455 is up."
fi
echo ""
echo "  RViz: Image + PointCloud + Odom use Best Effort (matches RealSense; do not use Reliable"
echo "        on Image — it will not match the camera publisher). If blank, enable"
echo "        display 'Camera (image_rect_color)' or set Camera → Topic to match ros2 topic list."
echo "  If TF errors: Fixed Frame → map or camera_link."
echo "  Image preview: select the Camera display in the left tree — thumbnail shows under properties."
echo ""

if ! _gnns_rviz_display_ready; then
    exit 3
fi

RVIZ_CFG="$(cd "$SCRIPT_DIR/../.." && pwd)/config/drone_monitor.rviz"
if [[ -f "$RVIZ_CFG" ]]; then
    echo "  Launching RViz2 with: $RVIZ_CFG"
    if [[ "${RQT_IMAGE:-0}" == "1" ]] && [[ -n "$IMG_TOPIC" ]]; then
        ros2 run rqt_image_view rqt_image_view --ros-args -r image:="$IMG_TOPIC" &
    fi
    exec ros2 run rviz2 rviz2 -d "$RVIZ_CFG"
else
    echo "  [Warn] No drone_monitor.rviz — starting blank RViz2"
    exec ros2 run rviz2 rviz2
fi
