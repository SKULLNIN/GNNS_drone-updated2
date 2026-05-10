#!/usr/bin/env bash
# ============================================================================
# gNNS Drone — Ubuntu 24.04 laptop setup (idempotent, re-runnable)
# ----------------------------------------------------------------------------
# Sets up a brand-new Ubuntu 24.04 laptop for the gNNS Drone codebase:
#   * Base apt deps (build tools, vision/serial runtime, user groups)
#   * System Python 3.12 venv at .venv-py312 (ROS-friendly: works with rclpy)
#   * Python 3.14 from deadsnakes PPA + .venv-py314 (the pip stack we
#     validated on Windows; cannot import rclpy)
#   * Intel librealsense2 (RealSense D435i / D455)
#   * ROS 2 Jazzy + realsense2-camera + RTAB-Map + DDS RMWs
#   * ArduPilot SITL + Gazebo Harmonic + ardupilot_gazebo plugin
#
# Why a new script:
#   The existing scripts under scripts/jetson_nano/*.sh default to ROS_DISTRO
#   "humble" (Ubuntu 22.04 only) and sitl/setup_*.sh use ROS Noetic + Gazebo
#   11 (Ubuntu 20.04 only). Neither pair is available on 24.04. This script
#   targets the supported 24.04 stack: ROS 2 Jazzy + Gazebo Harmonic, and
#   exports ROS_DISTRO=jazzy so Jetson helper scripts (which default
#   to jazzy on noble / humble on jammy) stay aligned on this laptop.
#
# Usage:
#   bash scripts/laptop_ubuntu24/setup_laptop_ubuntu24.sh [--minimal|--with-ros|--with-sitl|--all] [--tune-network] [--yes]
#
# Flags:
#   --minimal        Base apt + both Python venvs + librealsense only
#   --with-ros       Also install ROS 2 Jazzy stack
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

# Default to --all when no mode flag is given (matches plan: full dev workstation).
if [[ "$EXPLICIT_MODE" -eq 0 ]]; then
  WITH_ROS=1
  WITH_SITL=1
fi

if [[ "$ASSUME_YES" -eq 1 ]]; then
  export DEBIAN_FRONTEND=noninteractive
fi

# ---------------------------------------------------------------------------
# Resolve repo root (this script lives at <repo>/scripts/laptop_ubuntu24/)
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
REQ_FILE="$REPO_ROOT/requirements.txt"
[[ -f "$REQ_FILE" ]] || die "Cannot find $REQ_FILE — run this script from inside the gNNS_drone checkout."

BASHRC_MARKER="# >>> gnns_drone laptop_ubuntu24 setup >>>"
BASHRC_END_MARKER="# <<< gnns_drone laptop_ubuntu24 setup <<<"

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
    # Replace existing block in-place
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
    if [[ "$ver" != "24.04" ]]; then
      warn "Detected Ubuntu $ver — this script targets 24.04. Continuing anyway."
    else
      ok "Ubuntu 24.04 detected."
    fi
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

  # Hardware groups for MAVLink USB / serial / RealSense.
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
# 3. Python 3.14 from deadsnakes
# ===========================================================================
install_python314() {
  log "Installing Python 3.14 (deadsnakes PPA)"
  if ! command -v add-apt-repository >/dev/null 2>&1; then
    apt_install software-properties-common
  fi
  if ! grep -rqs "deadsnakes" /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
    $SUDO add-apt-repository -y ppa:deadsnakes/ppa
    $SUDO apt-get update -y
  else
    ok "deadsnakes PPA already present"
  fi
  if ! apt_install python3.14 python3.14-venv python3.14-dev; then
    warn "deadsnakes does not yet provide python3.14 for this Ubuntu release."
    warn "Falling back: .venv-py314 will be skipped; only .venv-py312 will be created."
    return 1
  fi
  ok "Python 3.14 installed: $(python3.14 --version 2>&1)"
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
  create_venv python3.12 "$REPO_ROOT/.venv-py312" || warn ".venv-py312 setup failed"
  create_venv python3.14 "$REPO_ROOT/.venv-py314" || warn ".venv-py314 setup failed (Python 3.14 missing?)"
}

# ===========================================================================
# 5. librealsense (RealSense D435i / D455)
# ===========================================================================
install_librealsense() {
  log "Installing Intel librealsense2 (RealSenseAI apt repo)"
  # Official packages and signing key moved from librealsense.intel.com to
  # librealsense.realsenseai.com. The old Intel URL can break apt (NO_PUBKEY
  # FB0B24895113F120) and block all later apt-get update steps — see:
  # https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md
  local keyring="/etc/apt/keyrings/librealsenseai.gpg"
  local listfile="/etc/apt/sources.list.d/librealsense.list"
  $SUDO mkdir -p /etc/apt/keyrings

  if [[ -f "$listfile" ]] && grep -q "librealsense.intel.com" "$listfile" 2>/dev/null; then
    warn "Removing legacy librealsense.intel.com apt source (incompatible GPG key)."
    $SUDO rm -f "$listfile"
  fi
  # Orphan key from old instructions
  [[ -f /etc/apt/keyrings/librealsense.gpg ]] && $SUDO rm -f /etc/apt/keyrings/librealsense.gpg

  if ! curl -fsSL https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
       | $SUDO gpg --dearmor -o "$keyring"; then
    warn "Could not fetch librealsenseai signing key — skipping apt repo."
    warn "Build from source: https://github.com/IntelRealSense/librealsense"
    return 0
  fi

  local codename
  codename="$(lsb_release -cs 2>/dev/null || echo noble)"
  echo "deb [signed-by=$keyring] https://librealsense.realsenseai.com/Debian/apt-repo ${codename} main" \
    | $SUDO tee "$listfile" >/dev/null

  if ! $SUDO apt-get update -y; then
    warn "apt-get update failed after adding librealsense repo."
  fi
  if ! apt_install librealsense2-utils librealsense2-dev librealsense2-dkms; then
    warn "librealsense apt install failed on Ubuntu 24.04."
    warn "Fallback: build from source — https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md"
  else
    ok "librealsense2 installed (try 'realsense-viewer')"
  fi
}

# ===========================================================================
# 6. ROS 2 Jazzy
# ===========================================================================
install_ros2_jazzy() {
  log "Installing ROS 2 Jazzy"
  local keyring="/usr/share/keyrings/ros-archive-keyring.gpg"
  local listfile="/etc/apt/sources.list.d/ros2.list"
  if [[ ! -f "$keyring" ]]; then
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | $SUDO gpg --dearmor -o "$keyring"
  fi
  if [[ ! -f "$listfile" ]]; then
    local codename
    codename="$(lsb_release -cs 2>/dev/null || echo noble)"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=$keyring] http://packages.ros.org/ros2/ubuntu $codename main" \
      | $SUDO tee "$listfile" >/dev/null
    $SUDO apt-get update -y
  fi

  apt_install ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool

  # Code-path packages used by this repo.
  apt_install \
    ros-jazzy-realsense2-camera \
    ros-jazzy-rtabmap-launch ros-jazzy-rtabmap-ros \
    ros-jazzy-rtabmap-odom ros-jazzy-rtabmap-sync ros-jazzy-rtabmap-slam \
    ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-tf2-tools ros-jazzy-image-transport-plugins \
    ros-jazzy-rqt ros-jazzy-rqt-common-plugins || \
      warn "Some ROS 2 packages were not available; check 'apt-cache search ros-jazzy-...'"

  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    $SUDO rosdep init || true
  fi
  rosdep update || warn "rosdep update failed (offline?)"

  cat <<'EOF' | ensure_bashrc_block
# Source ROS 2 Jazzy and pin ROS_DISTRO so the Jetson scripts in
# scripts/jetson_nano/*.sh (which read ${ROS_DISTRO:-humble}) pick the
# right distro on this laptop without modification.
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
EOF
  ok "ROS 2 Jazzy installed and wired into ~/.bashrc"
}

# ===========================================================================
# 7. ArduPilot SITL + Gazebo Harmonic
# ===========================================================================
install_sitl_gazebo() {
  log "Installing ArduPilot SITL + Gazebo Harmonic"

  # --- ArduPilot ---
  local apdir="$HOME/ardupilot"
  if [[ ! -d "$apdir/.git" ]]; then
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$apdir"
  else
    ok "ardupilot repo already present at $apdir"
    git -C "$apdir" submodule update --init --recursive || true
  fi
  if [[ -x "$apdir/Tools/environment_install/install-prereqs-ubuntu.sh" ]]; then
    ( cd "$apdir" && Tools/environment_install/install-prereqs-ubuntu.sh -y ) \
      || warn "ArduPilot install-prereqs returned non-zero (often safe on 24.04)."
  fi

  # --- Gazebo Harmonic (replaces Noetic-era Gazebo 11) ---
  local gz_keyring="/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg"
  local gz_listfile="/etc/apt/sources.list.d/gazebo-stable.list"
  if [[ ! -f "$gz_keyring" ]]; then
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      | $SUDO gpg --dearmor -o "$gz_keyring"
  fi
  if [[ ! -f "$gz_listfile" ]]; then
    local codename
    codename="$(lsb_release -cs 2>/dev/null || echo noble)"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=$gz_keyring] http://packages.osrfoundation.org/gazebo/ubuntu-stable $codename main" \
      | $SUDO tee "$gz_listfile" >/dev/null
    $SUDO apt-get update -y
  fi
  apt_install gz-harmonic libgz-sim8-dev rapidjson-dev

  # --- ardupilot_gazebo plugin ---
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

  # MAVROS for Jazzy + GeographicLib datasets used by interface scripts.
  if [[ "$WITH_ROS" -eq 1 ]]; then
    apt_install ros-jazzy-mavros ros-jazzy-mavros-extras || \
      warn "ros-jazzy-mavros not available; skip if not needed."
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
# 9. Postflight smoke test + summary
# ===========================================================================
smoke_test() {
  log "Postflight smoke test"
  local py314="$REPO_ROOT/.venv-py314/bin/python"
  if [[ -x "$py314" ]]; then
    if ( cd "$REPO_ROOT" && "$py314" -c "import gnns_drone.coordinate_utils, gnns_drone.safety, gnns_drone.mavlink_bridge; print('gnns_drone imports OK on', __import__('sys').version.split()[0])" ); then
      ok "Python 3.14 venv imports gnns_drone modules"
    else
      warn "gnns_drone import failed in .venv-py314 (check requirements.txt and PYTHONPATH)"
    fi
  fi
  local py312="$REPO_ROOT/.venv-py312/bin/python"
  if [[ -x "$py312" ]]; then
    if ( cd "$REPO_ROOT" && "$py312" -c "import gnns_drone.coordinate_utils, gnns_drone.safety, gnns_drone.mavlink_bridge; print('gnns_drone imports OK on', __import__('sys').version.split()[0])" ); then
      ok "Python 3.12 venv imports gnns_drone modules"
    else
      warn "gnns_drone import failed in .venv-py312"
    fi
  fi
  if [[ "$WITH_ROS" -eq 1 ]] && [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # shellcheck disable=SC1091
    bash -c 'source /opt/ros/jazzy/setup.bash && ros2 doctor --report' || \
      warn "ros2 doctor reported issues (often benign on first install)."
  fi
}

print_summary() {
  cat <<EOF

============================================================================
  gNNS Drone — Ubuntu 24.04 setup complete
============================================================================
Repo:           $REPO_ROOT
Python 3.12:    source $REPO_ROOT/.venv-py312/bin/activate    # ROS / rclpy
Python 3.14:    source $REPO_ROOT/.venv-py314/bin/activate    # pip stack only

Which one to use:
  - ROS 2 nodes / anything that imports rclpy  -> .venv-py312
  - Pure pymavlink / OpenCV / SciPy CLI tools  -> .venv-py314 (or 3.12)

ROS 2:          $( [[ "$WITH_ROS" -eq 1 ]]  && echo "Jazzy installed; ROS_DISTRO=jazzy in ~/.bashrc"  || echo "skipped (--minimal)" )
SITL:           $( [[ "$WITH_SITL" -eq 1 ]] && echo "ArduPilot + Gazebo Harmonic installed"           || echo "skipped" )

Next steps:
  1. Open a new shell (so ~/.bashrc and group changes take effect).
  2. Verify hardware:
       lsusb | grep -i 'Intel\\|Pixhawk\\|FTDI'
       realsense-viewer            # if librealsense installed
  3. Run the project:
       source $REPO_ROOT/.venv-py314/bin/activate
       python -m gnns_drone --sitl --demo

Notes:
  * If you need rclpy in the same process as gnns_drone, switch to
    .venv-py312 — Python 3.14 cannot import rclpy on Jazzy.
  * scripts/jetson_nano/*.sh pick ROS_DISTRO from Ubuntu codename (noble=jazzy,
    jammy=humble) unless ROS_DISTRO is already set; this script also sets
    ROS_DISTRO=jazzy in ~/.bashrc on the laptop.
  * The Noetic-based scripts (sitl/setup_*.sh, start_simulation.sh) refuse to
    run on Ubuntu 24.04 — use gnns_ubuntu24.sh instead.

Run full SITL + Gazebo Harmonic + demo mission:
  bash $REPO_ROOT/gnns_ubuntu24.sh fly
============================================================================
EOF
}

# ===========================================================================
# Main
# ===========================================================================
main() {
  preflight
  install_base_apt
  install_python314 || true
  create_venvs
  install_librealsense
  if [[ "$WITH_ROS" -eq 1 ]];  then install_ros2_jazzy; fi
  if [[ "$WITH_SITL" -eq 1 ]]; then install_sitl_gazebo; fi
  if [[ "$TUNE_NETWORK" -eq 1 ]]; then tune_network; fi
  smoke_test
  print_summary
}

main "$@"
