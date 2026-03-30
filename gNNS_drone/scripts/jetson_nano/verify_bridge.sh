#!/usr/bin/env bash
# ================================================================
# verify_bridge.sh — Laptop-side ROS 2 bridge / Jetson topic checks
# ================================================================
# Run on the laptop (same WiFi as Jetson) after starting ros_bridge.sh
# on the Jetson. Verifies DDS env and that key topics exist (and briefly
# samples rates).
#
# Usage:
#   ./verify_bridge.sh
#   ./verify_bridge.sh 42                    # ROS domain ID
#   ./verify_bridge.sh --cyclone-iface wlan0
#   ./verify_bridge.sh --no-hz               # skip hz sampling
# ================================================================
set -euo pipefail

DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
DO_HZ=1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# shellcheck source=cyclonedds_env.sh
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/cyclonedds_env.sh"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-hz) DO_HZ=0; shift ;;
        --cyclone-iface)
            [[ -n "${2:-}" ]] || { echo "[Error] --cyclone-iface needs value" >&2; exit 2; }
            export CYCLONE_IFACE="$2"
            shift 2
            ;;
        -*)
            echo "[Error] Unknown flag: $1" >&2
            exit 2
            ;;
        *)
            if [[ "$1" =~ ^[0-9]+$ ]]; then
                DOMAIN_ID="$1"
            else
                echo "[Error] Unknown argument: $1" >&2
                exit 2
            fi
            shift
            ;;
    esac
done

ROS_DISTRO="${ROS_DISTRO:-humble}"
SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "$SETUP" ]]; then
    echo "[Error] Missing $SETUP — install ROS 2 $ROS_DISTRO" >&2
    exit 2
fi

export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

gnns_set_cyclonedds_uri

# shellcheck source=/dev/null
source "$SETUP"

if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "$HOME/ros2_ws/install/setup.bash"
elif [[ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
fi

PASS=0
FAIL=0
fail() { echo "[FAIL] $*"; FAIL=$((FAIL + 1)); }
ok() { echo "[ OK ] $*"; PASS=$((PASS + 1)); }

echo ""
echo "=== gNNS bridge verification (laptop) ==="
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>}"
echo ""

[[ "$ROS_LOCALHOST_ONLY" == "0" ]] && ok "ROS_LOCALHOST_ONLY=0 (multicast OK)" || fail "ROS_LOCALHOST_ONLY must be 0 for laptop bridge"
[[ -n "${CYCLONEDDS_URI:-}" ]] && ok "CYCLONEDDS_URI is set" || fail "CYCLONEDDS_URI not set"

if ! command -v ros2 >/dev/null 2>&1; then
    fail "ros2 CLI not found"
    echo "Summary: $PASS passed, $FAIL failed"
    exit 1
fi

mapfile -t TOPICS < <(ros2 topic list 2>/dev/null || true)
if [[ ${#TOPICS[@]} -eq 0 ]]; then
    fail "No topics visible (Jetson not running or DDS/firewall issue)"
    echo "Summary: $PASS passed, $FAIL failed"
    exit 1
fi
ok "Topic list non-empty (${#TOPICS[@]} topics)"

have_topic() {
    local t="$1"
    local x
    for x in "${TOPICS[@]}"; do
        [[ "$x" == "$t" ]] && return 0
    done
    return 1
}

check_topic() {
    local t="$1"
    local label="${2:-$t}"
    if have_topic "$t"; then
        ok "Topic $label"
    else
        fail "Missing topic $label ($t)"
    fi
}

check_topic "/odom" "VIO /odom"
check_topic "/camera/color/image_raw" "camera RGB"

warn_topic() {
    local t="$1"
    local label="$2"
    if have_topic "$t"; then
        ok "Topic $label"
    else
        echo "[WARN] Missing $label ($t) — may appear after RTAB-Map warms up"
    fi
}

warn_topic "/rtabmap/cloud_map" "RTAB-Map cloud_map"
warn_topic "/map" "2D occupancy /map"
warn_topic "/rtabmap/grid_map" "RTAB-Map grid_map"

if [[ "$DO_HZ" == "1" ]]; then
    echo ""
    echo "--- Sample rates (5s max per topic) ---"
    for t in /odom /camera/color/image_raw; do
        if have_topic "$t"; then
            echo "  hz $t:"
            timeout 6s ros2 topic hz "$t" 2>/dev/null | head -5 || true
        fi
    done
fi

echo ""
if [[ $FAIL -eq 0 ]]; then
    echo "Summary: all checks passed ($PASS)"
    exit 0
fi
echo "Summary: $PASS passed, $FAIL failed — fix network/DDS or start Jetson stack"
exit 1
