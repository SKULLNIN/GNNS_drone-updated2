#!/usr/bin/env bash
# ============================================================================
# gNNS Drone — Ubuntu 22.04 Jetson / desktop setup (idempotent, re-runnable)
# ----------------------------------------------------------------------------
# Mirrors scripts/laptop_ubuntu24/setup_laptop_ubuntu24.sh for Ubuntu 22.04
# (Jammy): ROS 2 Humble, same RealSense + RTAB-Map + optional SITL path.
#
# Targets NVIDIA Jetson (L4T / JetPack on Jammy) or any Ubuntu 22.04 machine.
#
#   * Base apt deps (build tools, vision/serial runtime, user groups)
#   * System Python 3.10 venv at .venv-py310 (ROS Humble / rclpy)
#   * Python 3.12 from deadsnakes PPA + .venv-py312 (newer pip stack; no rclpy)
#   * Intel librealsense2 (RealSense D435i / D455) — on aarch64, dkms may
#     need manual steps; see Intel Jetson docs if apt fails
#   * ROS 2 Humble + realsense2-camera + RTAB-Map + DDS RMWs
#   * Optional: ArduPilot SITL + Gazebo Harmonic + ardupilot_gazebo plugin
#
# Usage:
#   bash scripts/jetson_ubuntu22/setup_jetson_ubuntu22.sh [--minimal|--with-ros|--with-sitl|--all] [--tune-network] [--yes]
#
# Flags:
#   --minimal        Base apt + both Python venvs + librealsense only
#   --with-ros       Also install ROS 2 Humble stack
#   --with-sitl      Also install ArduPilot SITL + Gazebo Harmonic
#   --all            --with-ros + --with-sitl  (default if no flag given)
#   --tune-network   Persist DDS-over-WiFi sysctl tweaks
#   --yes            Non-interactive apt (DEBIAN_FRONTEND=noninteractive)
#   -h, --help       Show this help
# ============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Pretty logging
# ---------------------------------------------------------------------------
log()   { printf '\n\033[1;36m[setup]\033[0m %s\n' "$*"; }
ok()    { printf '\033[1;32m  [ok]\033[0m %s\n' "$*"; }
warn()  { printf '\033[1;33m  [warn]\033[0m %s\n' "$*" >&2; }
err()   { printf '\033[1;31m  [err]\033[0m %s\n' "$*" >&2; }
die()   { err "$*"; exit 1; }

# ---------------------------------------------------------------------------
# Arg parsing
# ---------------------------------------------------------------------------
WITH_ROS=0
WITH_SITL=0
TUNE_NETWORK=0
ASSUME_YES=0
EXPLICIT_MODE=0

print_help() { sed -n '2,40p' "$0"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --minimal)      EXPLICIT_MODE=1; WITH_ROS=0; WITH_SITL=0 ;;
    --with-ros)     EXPLICIT_MODE=1; WITH_ROS=1 ;;
    --with-sitl)    EXPLICIT_MODE=1; WITH_SITL=1 ;;
    --all)          EXPLICIT_MODE=1; WITH_ROS=1; WITH_SITL=1 ;;
    --tune-network) TUNE_NETWORK=1 ;;
    --yes|-y)       ASSUME_YES=1 ;;
    -h|--help)      print_help; exit 0 ;;
    *)              die "Unknown argument: $1 (try --help)" ;;
  esac
  shift
done

if [[ "$EXPLICIT_MODE" -eq 0 ]]; then
  WITH_ROS=1
  WITH_SITL=1
fi

if [[ "$ASSUME_YES" -eq 1 ]]; then
  export DEBIAN_FRONTEND=noninteractive
fi

# ---------------------------------------------------------------------------
# Resolve repo root (this script lives at <repo>/scripts/jetson_ubuntu22/)
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
REQ_FILE="$REPO_ROOT/requirements.txt"
JETSON_SCRIPT_DIR="$REPO_ROOT/scripts/jetson_nano"
[[ -f "$REQ_FILE" ]] || die "Cannot find $REQ_FILE — run this script from inside the gNNS_drone checkout."

BASHRC_MARKER="# >>> gnns_drone jetson_ubuntu22 setup >>>"
BASHRC_END_MARKER="# <<< gnns_drone jetson_ubuntu22 setup <<<"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
need_sudo() {
  if [[ "$(id -u)" -ne 0 ]]; then
    command -v sudo >/dev/null 2>&1 || die "sudo required but not installed."
    SUDO="sudo"
  else
    SUDO=""
  fi
}

apt_install() {
  $SUDO apt-get install -y --no-install-recommends "$@"
}

ensure_bashrc_block() {
  local rcfile="$HOME/.bashrc"
  touch "$rcfile"
  if grep -qF "$BASHRC_MARKER" "$rcfile"; then
    awk -v start="$BASHRC_MARKER" -v end="$BASHRC_END_MARKER" '
      BEGIN { skip=0 }
      $0 == start { skip=1; next }
      $0 == end   { skip=0; next }
      skip == 0   { print }
    ' "$rcfile" > "$rcfile.tmp" && mv "$rcfile.tmp" "$rcfile"
  fi
  {
    echo ""
    echo "$BASHRC_MARKER"
    cat
    echo "$BASHRC_END_MARKER"
  } >> "$rcfile"
  ok "Updated $rcfile (gnns_drone block)"
}

# ===========================================================================
# 1. Preflight
# ===========================================================================
preflight() {
  log "Preflight checks"
  if ! command -v lsb_release >/dev/null 2>&1; then
    warn "lsb_release missing; cannot verify Ubuntu version."
  else
    local ver
    ver="$(lsb_release -rs)"
    if [[ "$ver" != "22.04" ]]; then
      warn "Detected Ubuntu $ver — this script targets 22.04 (Jammy). Continuing anyway."
    else
      ok "Ubuntu 22.04 detected."
    fi
  fi
  local arch
  arch="$(dpkg --print-architecture 2>/dev/null || echo unknown)"
  if [[ "$arch" == "arm64" ]]; then
    ok "ARM64 detected — typical for Jetson. If librealsense dkms fails, use Intel's Jetson build instructions."
  fi
  need_sudo
  log "Refreshing apt index"
  $SUDO apt-get update -y
}

# ===========================================================================
# 2. Base apt packages
# ===========================================================================
install_base_apt() {
  log "Installing base apt packages"
  apt_install \
    build-essential git curl wget ca-certificates gnupg lsb-release \
    software-properties-common pkg-config cmake ninja-build unzip \
    python3 python3-venv python3-pip python3-dev \
    ffmpeg v4l-utils libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 udev \
    setserial usbutils tmux

  for grp in dialout plugdev video; do
    if getent group "$grp" >/dev/null 2>&1; then
      if id -nG "$USER" | tr ' ' '\n' | grep -qx "$grp"; then
        ok "User '$USER' already in group '$grp'"
      else
        $SUDO usermod -aG "$grp" "$USER"
        ok "Added '$USER' to group '$grp' (re-login required)"
      fi
    fi
  done
}

# ===========================================================================
# 3. Python 3.12 from deadsnakes (optional second venv; not for rclpy)
# ===========================================================================
install_python312() {
  log "Installing Python 3.12 (deadsnakes PPA)"
  if ! command -v add-apt-repository >/dev/null 2>&1; then
    apt_install software-properties-common
  fi
  if ! grep -rqs "deadsnakes" /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
    $SUDO add-apt-repository -y ppa:deadsnakes/ppa
    $SUDO apt-get update -y
  else
    ok "deadsnakes PPA already present"
  fi
  if ! apt_install python3.12 python3.12-venv python3.12-dev; then
    warn "python3.12 install failed for this release."
    warn "Falling back: .venv-py312 will be skipped; only .venv-py310 will be created."
    return 1
  fi
  ok "Python 3.12 installed: $(python3.12 --version 2>&1)"
}

# ===========================================================================
# 4. Two venvs at repo root
# ===========================================================================
create_venv() {
  local py="$1"
  local target="$2"
  if [[ ! -x "$(command -v "$py")" ]]; then
    warn "$py not available; skipping $target"
    return 1
  fi
  if [[ -d "$target" ]]; then
    ok "venv exists: $target"
  else
    log "Creating venv $target with $py"
    "$py" -m venv "$target"
  fi
  # shellcheck disable=SC1091
  "$target/bin/python" -m pip install --upgrade pip wheel setuptools
  "$target/bin/python" -m pip install -r "$REQ_FILE"
  ok "Installed requirements.txt into $target"
}

create_venvs() {
  log "Creating Python venvs at repo root"
  # ROS 2 Humble on Jammy uses Python 3.10 — use this venv with rclpy.
  create_venv python3 "$REPO_ROOT/.venv-py310" || warn ".venv-py310 setup failed"
  create_venv python3.12 "$REPO_ROOT/.venv-py312" || warn ".venv-py312 setup failed (Python 3.12 missing?)"
}

# ===========================================================================
# 5. librealsense (RealSense D435i / D455)
# ===========================================================================
install_librealsense() {
  log "Installing Intel librealsense2 (RealSenseAI apt repo)"
  local keyring="/etc/apt/keyrings/librealsenseai.gpg"
  local listfile="/etc/apt/sources.list.d/librealsense.list"
  $SUDO mkdir -p /etc/apt/keyrings

  if [[ -f "$listfile" ]] && grep -q "librealsense.intel.com" "$listfile" 2>/dev/null; then
    warn "Removing legacy librealsense.intel.com apt source (incompatible GPG key)."
    $SUDO rm -f "$listfile"
  fi
  [[ -f /etc/apt/keyrings/librealsense.gpg ]] && $SUDO rm -f /etc/apt/keyrings/librealsense.gpg

  if ! curl -fsSL https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
       | $SUDO gpg --dearmor -o "$keyring"; then
    warn "Could not fetch librealsenseai signing key — skipping apt repo."
    warn "Build from source: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md"
    return 0
  fi

  local codename
  codename="$(lsb_release -cs 2>/dev/null || echo jammy)"
  echo "deb [signed-by=$keyring] https://librealsense.realsenseai.com/Debian/apt-repo ${codename} main" \
    | $SUDO tee "$listfile" >/dev/null

  if ! $SUDO apt-get update -y; then
    warn "apt-get update failed after adding librealsense repo."
  fi
  if ! apt_install librealsense2-utils librealsense2-dev librealsense2-dkms; then
    warn "librealsense apt install failed."
    warn "On Jetson, prefer: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md"
  else
    ok "librealsense2 installed (try 'realsense-viewer')"
  fi
}

# ===========================================================================
# 6. ROS 2 Humble
# ===========================================================================
install_ros2_humble() {
  log "Installing ROS 2 Humble"
  local keyring="/usr/share/keyrings/ros-archive-keyring.gpg"
  local listfile="/etc/apt/sources.list.d/ros2.list"
  if [[ ! -f "$keyring" ]]; then
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | $SUDO gpg --dearmor -o "$keyring"
  fi
  if [[ ! -f "$listfile" ]]; then
    local codename
    codename="$(lsb_release -cs 2>/dev/null || echo jammy)"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=$keyring] http://packages.ros.org/ros2/ubuntu $codename main" \
      | $SUDO tee "$listfile" >/dev/null
    $SUDO apt-get update -y
  fi

  apt_install ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool

  apt_install \
    ros-humble-realsense2-camera \
    ros-humble-rtabmap-launch ros-humble-rtabmap-ros \
    ros-humble-rtabmap-odom ros-humble-rtabmap-sync ros-humble-rtabmap-slam \
    ros-humble-rmw-cyclonedds-cpp ros-humble-rmw-fastrtps-cpp \
    ros-humble-tf2-tools ros-humble-image-transport-plugins \
    ros-humble-rqt ros-humble-rqt-common-plugins || \
      warn "Some ROS 2 packages were not available; check 'apt-cache search ros-humble-...'"

  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    $SUDO rosdep init || true
  fi
  rosdep update || warn "rosdep update failed (offline?)"

  cat <<'EOF' | ensure_bashrc_block
# Source ROS 2 Humble (Ubuntu 22.04 / Jetson)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
EOF
  ok "ROS 2 Humble installed and wired into ~/.bashrc"
}

# ===========================================================================
# 7. ArduPilot SITL + Gazebo Harmonic
# ===========================================================================
install_sitl_gazebo() {
  log "Installing ArduPilot SITL + Gazebo Harmonic"

  local apdir="$HOME/ardupilot"
  if [[ ! -d "$apdir/.git" ]]; then
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$apdir"
  else
    ok "ardupilot repo already present at $apdir"
    git -C "$apdir" submodule update --init --recursive || true
  fi
  if [[ -x "$apdir/Tools/environment_install/install-prereqs-ubuntu.sh" ]]; then
    ( cd "$apdir" && Tools/environment_install/install-prereqs-ubuntu.sh -y ) \
      || warn "ArduPilot install-prereqs returned non-zero (check log above)."
  fi

  local gz_keyring="/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg"
  local gz_listfile="/etc/apt/sources.list.d/gazebo-stable.list"
  if [[ ! -f "$gz_keyring" ]]; then
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      | $SUDO gpg --dearmor -o "$gz_keyring"
  fi
  if [[ ! -f "$gz_listfile" ]]; then
    local codename
    codename="$(lsb_release -cs 2>/dev/null || echo jammy)"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=$gz_keyring] http://packages.osrfoundation.org/gazebo/ubuntu-stable $codename main" \
      | $SUDO tee "$gz_listfile" >/dev/null
    $SUDO apt-get update -y
  fi
  apt_install gz-harmonic libgz-sim8-dev rapidjson-dev

  local gz_ws="$HOME/gz_ws"
  local plugin_src="$gz_ws/src/ardupilot_gazebo"
  mkdir -p "$gz_ws/src"
  if [[ ! -d "$plugin_src/.git" ]]; then
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$plugin_src"
  else
    ok "ardupilot_gazebo already present"
  fi
  (
    export GZ_VERSION="${GZ_VERSION:-harmonic}"
    cd "$plugin_src"
    mkdir -p build
    cd build
    cmake ..
    make -j"$(nproc)"
  ) || warn "ardupilot_gazebo build failed — inspect $plugin_src/build/"

  cat <<EOF | ensure_bashrc_block
# ArduPilot + Gazebo Harmonic plugin paths
export GZ_VERSION=\${GZ_VERSION:-harmonic}
export GZ_SIM_SYSTEM_PLUGIN_PATH=\${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:$plugin_src/build
export GZ_SIM_RESOURCE_PATH=\${GZ_SIM_RESOURCE_PATH:-}:$plugin_src/models:$plugin_src/worlds
export PATH=\$PATH:$apdir/Tools/autotest
EOF

  if [[ "$WITH_ROS" -eq 1 ]]; then
    apt_install ros-humble-mavros ros-humble-mavros-extras || \
      warn "ros-humble-mavros not available; skip if not needed."
    local geoinst="$REPO_ROOT/gnns_drone/autonomy_core/interface/scripts/install_geographiclib_datasets.sh"
    if [[ -x "$geoinst" ]]; then
      $SUDO "$geoinst" || warn "GeographicLib datasets install returned non-zero."
    fi
  fi
  ok "ArduPilot SITL + Gazebo Harmonic ready"
}

# ===========================================================================
# 8. Optional network tuning (DDS over Wi-Fi)
# ===========================================================================
tune_network() {
  log "Persisting DDS-over-WiFi sysctl tweaks"
  local f="/etc/sysctl.d/99-gnns-drone.conf"
  $SUDO tee "$f" >/dev/null <<'EOF'
# gNNS Drone: improve DDS reliability over WiFi (matches docs/JETSON_LAPTOP_SETUP.md)
net.ipv4.ipfrag_time = 3
net.ipv4.ipfrag_high_thresh = 134217728
EOF
  $SUDO sysctl --system >/dev/null
  ok "Wrote $f"
}

# ===========================================================================
# 8b. Jetson serial / udev (optional hint)
# ===========================================================================
jetson_serial_hint() {
  local rules="$JETSON_SCRIPT_DIR/99-gnns-telem.rules"
  if [[ -f "$rules" ]]; then
    log "Jetson UART: if you use TELEM on /dev/ttyTHS1, install udev rules:"
    printf '  sudo cp %q /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger\n' "$rules"
  fi
}

# ===========================================================================
# 9. Postflight smoke test + summary
# ===========================================================================
smoke_test() {
  log "Postflight smoke test"
  local py312="$REPO_ROOT/.venv-py312/bin/python"
  if [[ -x "$py312" ]]; then
    if ( cd "$REPO_ROOT" && "$py312" -c "import gnns_drone.coordinate_utils, gnns_drone.safety, gnns_drone.mavlink_bridge; print('gnns_drone imports OK on', __import__('sys').version.split()[0])" ); then
      ok "Python 3.12 venv imports gnns_drone modules"
    else
      warn "gnns_drone import failed in .venv-py312 (check requirements.txt and PYTHONPATH)"
    fi
  fi
  local py310="$REPO_ROOT/.venv-py310/bin/python"
  if [[ -x "$py310" ]]; then
    if ( cd "$REPO_ROOT" && "$py310" -c "import gnns_drone.coordinate_utils, gnns_drone.safety, gnns_drone.mavlink_bridge; print('gnns_drone imports OK on', __import__('sys').version.split()[0])" ); then
      ok "Python 3.10 venv imports gnns_drone modules"
    else
      warn "gnns_drone import failed in .venv-py310"
    fi
  fi
  if [[ "$WITH_ROS" -eq 1 ]] && [[ -f /opt/ros/humble/setup.bash ]]; then
    bash -c 'source /opt/ros/humble/setup.bash && ros2 doctor --report' || \
      warn "ros2 doctor reported issues (often benign on first install)."
  fi
}

print_summary() {
  cat <<EOF

============================================================================
  gNNS Drone — Ubuntu 22.04 (Jetson / Humble) setup complete
============================================================================
Repo:           $REPO_ROOT
Python 3.10:    source $REPO_ROOT/.venv-py310/bin/activate    # ROS Humble / rclpy
Python 3.12:    source $REPO_ROOT/.venv-py312/bin/activate    # pip stack (no rclpy)

Which one to use:
  - ROS 2 / rclpy in same process as gnns_drone  -> .venv-py310
  - Pure pymavlink / OpenCV / SciPy CLI tools     -> .venv-py312 (or 3.10)

ROS 2:          $( [[ "$WITH_ROS" -eq 1 ]]  && echo "Humble installed; ROS_DISTRO=humble in ~/.bashrc"  || echo "skipped (--minimal)" )
SITL:           $( [[ "$WITH_SITL" -eq 1 ]] && echo "ArduPilot + Gazebo Harmonic installed"            || echo "skipped" )

Next steps:
  1. Open a new shell (so ~/.bashrc and group changes take effect).
  2. Verify hardware:
       lsusb | grep -i 'Intel\\|Pixhawk\\|FTDI'
       realsense-viewer            # if librealsense installed
  3. VIO stack (Jetson or 22.04 PC with RealSense):
       source /opt/ros/humble/setup.bash
       ./scripts/jetson_nano/gnns_vio_stack.sh stack
       # or odom-only: ./scripts/jetson_nano/gnns_odom_stack.sh stack

Notes:
  * scripts/jetson_nano/*.sh default ROS_DISTRO from Ubuntu codename:
    jammy -> humble, noble -> jazzy. This script sets ROS_DISTRO=humble.
  * Laptop Ubuntu 24 flow remains: scripts/laptop_ubuntu24/setup_laptop_ubuntu24.sh + gnns_ubuntu24.sh

============================================================================
EOF
}

# ===========================================================================
# Main
# ===========================================================================
main() {
  preflight
  install_base_apt
  install_python312 || true
  create_venvs
  install_librealsense
  if [[ "$WITH_ROS" -eq 1 ]];  then install_ros2_humble; fi
  if [[ "$WITH_SITL" -eq 1 ]]; then install_sitl_gazebo; fi
  if [[ "$TUNE_NETWORK" -eq 1 ]]; then tune_network; fi
  jetson_serial_hint
  smoke_test
  print_summary
}

main "$@"
