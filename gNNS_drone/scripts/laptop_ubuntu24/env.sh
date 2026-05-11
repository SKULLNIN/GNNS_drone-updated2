# shellcheck shell=bash
# ============================================================================
# gNNS Drone — shared env + helpers for Ubuntu 24.04 (ROS 2 Jazzy + GZ Harmonic)
# Source from repo root:  source scripts/laptop_ubuntu24/env.sh
# Or use:                 bash gnns_ubuntu24.sh <subcommand>
# ============================================================================

# Repo paths (this file lives at <repo>/scripts/laptop_ubuntu24/env.sh)
if [[ -n "${BASH_SOURCE[0]:-}" ]]; then
  _GNNS_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  export REPO_ROOT="$(cd "${_GNNS_ENV_DIR}/../.." && pwd)"
  unset _GNNS_ENV_DIR
else
  export REPO_ROOT="${REPO_ROOT:-$(pwd)}"
fi

export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export GZ_VERSION="${GZ_VERSION:-harmonic}"
export ARDUPILOT_HOME="${ARDUPILOT_HOME:-${HOME}/ardupilot}"
export GZ_PLUGIN="${GZ_PLUGIN:-${HOME}/gz_ws/src/ardupilot_gazebo}"

# Gazebo resource / plugin search path (Harmonic + ardupilot_gazebo build)
if [[ -z "${GZ_SIM_SYSTEM_PLUGIN_PATH:-}" ]]; then
  export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_PLUGIN}/build"
else
  case ":${GZ_SIM_SYSTEM_PLUGIN_PATH}:" in
    *":${GZ_PLUGIN}/build:"*) ;;
    *) export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH}:${GZ_PLUGIN}/build" ;;
  esac
fi
_gnns_res="${GZ_PLUGIN}/models:${GZ_PLUGIN}/worlds"
if [[ -z "${GZ_SIM_RESOURCE_PATH:-}" ]]; then
  export GZ_SIM_RESOURCE_PATH="$_gnns_res"
else
  case ":${GZ_SIM_RESOURCE_PATH}:" in
    *":${GZ_PLUGIN}/models:"*) ;;
    *) export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${_gnns_res}" ;;
  esac
fi
unset _gnns_res

export PATH="${PATH}:${ARDUPILOT_HOME}/Tools/autotest"

export GNNS_VENV="${GNNS_VENV:-${REPO_ROOT}/.venv-py312}"
export GNNS_TMUX_SESSION="${GNNS_TMUX_SESSION:-gnns}"
export GNNS_WEB_PORT="${GNNS_WEB_PORT:-5000}"
export GNNS_MISSION_VIO_SOURCE="${GNNS_MISSION_VIO_SOURCE:-simulated}"

# ---------------------------------------------------------------------------
gnns_help() {
  cat <<'EOF'
gNNS Drone — Ubuntu 24.04 orchestrator (ROS 2 Jazzy + Gazebo Harmonic)

Usage:
  bash gnns_ubuntu24.sh <subcommand> [args]

Subcommands:
  install    Run full laptop setup (apt, ROS 2 Jazzy, ArduPilot, GZ Harmonic, venvs)
  doctor     Verify versions, paths, ports, Python venv (optional: --with-mavros)
  gazebo     Start gz sim (default world: iris_runway.sdf) [--world PATH] [--headless]
  sitl       Start ArduPilot SITL for Harmonic JSON plugin [-- extra sim_vehicle args]
  mission    Run demo mission against tcp:127.0.0.1:5760 (uses .venv-py312)
  web        Start web_control on GNNS_WEB_PORT (default 5000)
  fly        tmux: gazebo -> sitl -> mission [--web] [--headless] [--no-attach]
  clean      Kill gnns tmux session and common simulation processes
  status     Show tmux session, gnns processes and listening ports
  help       This message

Do NOT use start_simulation.sh or sitl/setup_*.sh on 24.04 — they target Noetic + Gazebo 11.
EOF
}

# ---------------------------------------------------------------------------
gnns_doctor() {
  local with_mavros=0
  local a
  for a in "$@"; do
    if [[ "$a" == "--with-mavros" ]]; then with_mavros=1; fi
  done

  local err=0

  echo "[doctor] Ubuntu"
  if command -v lsb_release >/dev/null 2>&1; then
    local ver
    ver="$(lsb_release -rs 2>/dev/null || true)"
    if [[ "$ver" != "24.04" ]]; then
      echo "  [warn] Expected 24.04, got: ${ver:-unknown}" >&2
    else
      echo "  [ok] Ubuntu 24.04"
    fi
  else
    echo "  [warn] lsb_release missing" >&2
  fi

  echo "[doctor] ROS 2 Jazzy"
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    echo "  [ok] /opt/ros/jazzy/setup.bash"
  else
    echo "  [err] Missing /opt/ros/jazzy — run: bash gnns_ubuntu24.sh install" >&2
    err=1
  fi

  echo "[doctor] Gazebo (Harmonic / gz-sim 8)"
  if ! command -v gz >/dev/null 2>&1; then
    echo "  [err] 'gz' not in PATH — run install" >&2
    err=1
  elif timeout 5 gz sim --versions 2>&1 | grep -q 'libgz-sim8'; then
    echo "  [ok] gz sim reports libgz-sim8 (Harmonic)"
  elif timeout 5 gz sim --help >/dev/null 2>&1; then
    echo "  [warn] gz sim works but libgz-sim8 not detected in 'gz sim --versions'" >&2
  else
    echo "  [err] gz sim not working — install gz-harmonic" >&2
    err=1
  fi

  echo "[doctor] ArduPilot + sim_vehicle"
  if [[ ! -d "$ARDUPILOT_HOME" ]]; then
    echo "  [err] ARDUPILOT_HOME not found: $ARDUPILOT_HOME" >&2
    err=1
  else
    echo "  [ok] ARDUPILOT_HOME=$ARDUPILOT_HOME"
  fi
  if command -v sim_vehicle.py >/dev/null 2>&1; then
    echo "  [ok] sim_vehicle.py in PATH"
  else
    echo "  [err] sim_vehicle.py not in PATH (add ${ARDUPILOT_HOME}/Tools/autotest)" >&2
    err=1
  fi

  echo "[doctor] ardupilot_gazebo plugin"
  if [[ -f "${GZ_PLUGIN}/build/libArduPilotPlugin.so" ]]; then
    echo "  [ok] ${GZ_PLUGIN}/build/libArduPilotPlugin.so"
  else
    echo "  [err] Missing libArduPilotPlugin.so — build ${GZ_PLUGIN}/build (run install --with-sitl)" >&2
    err=1
  fi
  if [[ -f "${GZ_PLUGIN}/config/gazebo-iris-gimbal.parm" ]]; then
    echo "  [ok] gazebo-iris-gimbal.parm"
  else
    echo "  [warn] Missing ${GZ_PLUGIN}/config/gazebo-iris-gimbal.parm" >&2
  fi

  echo "[doctor] Python venv (rclpy / mission)"
  if [[ -x "${GNNS_VENV}/bin/python" ]]; then
    # gnns_drone is not installed into site-packages; it is imported relative
    # to REPO_ROOT. Run the import test from REPO_ROOT so a normal CWD (e.g.
    # the user's $HOME) does not falsely fail doctor and block 'fly'.
    if ( cd "$REPO_ROOT" && "${GNNS_VENV}/bin/python" -c "import gnns_drone.mission_runner" ); then
      echo "  [ok] ${GNNS_VENV} imports gnns_drone.mission_runner"
    else
      echo "  [err] venv cannot import gnns_drone — recreate venv (install)" >&2
      err=1
    fi
  else
    echo "  [err] Missing venv at $GNNS_VENV" >&2
    err=1
  fi

  echo "[doctor] Ports (SITL TCP 5760)"
  if command -v ss >/dev/null 2>&1; then
    if ss -ltn 2>/dev/null | grep -qE ':5760\b'; then
      echo "  [warn] Something is listening on 5760 (stop other SITL or run clean)" >&2
    else
      echo "  [ok] TCP 5760 appears free"
    fi
  else
    echo "  [warn] ss not installed; skipped port check" >&2
  fi

  if [[ "$with_mavros" -eq 1 ]]; then
    echo "[doctor] GeographicLib (MAVROS)"
    if [[ -f /usr/share/GeographicLib/geoids/egm96-5.pgm ]]; then
      echo "  [ok] egm96-5 geoid"
    else
      echo "  [err] Missing /usr/share/GeographicLib/geoids/egm96-5.pgm" >&2
      err=1
    fi
  fi

  echo "[doctor] tmux (for fly)"
  if command -v tmux >/dev/null 2>&1; then
    echo "  [ok] tmux"
  else
    echo "  [err] tmux missing — apt install tmux or re-run install" >&2
    err=1
  fi

  if [[ "$err" -ne 0 ]]; then
    echo "[doctor] FAILED — fix errors above, then: bash gnns_ubuntu24.sh install" >&2
    return 1
  fi
  echo "[doctor] All checks passed"
  return 0
}

# ---------------------------------------------------------------------------
gnns_gazebo() {
  local world="iris_runway.sdf"
  local headless=0
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --world) world="${2:?}"; shift 2 ;;
      --headless) headless=1; shift ;;
      *) shift ;;
    esac
  done
  local -a gzargs=(sim -v4)
  if [[ "$headless" -eq 0 ]]; then
    gzargs+=(-r)
  fi
  gzargs+=("$world")
  # No exec: allows tmux / wrappers to run commands after gz exits.
  gz "${gzargs[@]}"
}

# ---------------------------------------------------------------------------
gnns_sitl() {
  cd "$ARDUPILOT_HOME" || return 1
  # Matches ArduPilot/ardupilot_gazebo README (Gazebo Harmonic + JSON plugin + gimbal parm).
  exec sim_vehicle.py -v ArduCopter -f JSON \
    --add-param-file="${GZ_PLUGIN}/config/gazebo-iris-gimbal.parm" \
    --no-mavproxy \
    "$@"
}

# ---------------------------------------------------------------------------
gnns_mission() {
  # MissionRunner forces source='sitl' when --sitl is given, so we do not
  # pass --vio-source here. Run in a subshell so the venv activation cannot
  # leak into the orchestrator's strict env.
  (
    # shellcheck disable=SC1091
    source "${GNNS_VENV}/bin/activate"
    cd "$REPO_ROOT"
    exec python -m gnns_drone --sitl --demo "$@"
  )
}

# ---------------------------------------------------------------------------
gnns_web() {
  (
    # shellcheck disable=SC1091
    source "${GNNS_VENV}/bin/activate"
    cd "$REPO_ROOT"
    exec python -m gnns_drone.web_control --sitl --port "${GNNS_WEB_PORT}" "$@"
  )
}

# ---------------------------------------------------------------------------
# Poll until 'gz topic -l' returns at least one topic, meaning gz-transport
# is up. Avoids the fragile 'sleep 3' guess and matches what the Harmonic
# plugin needs before sim_vehicle.py can establish the JSON link.
_gnns_wait_gazebo_ready() {
  local timeout="${1:-60}"
  local deadline=$(( $(date +%s) + timeout ))
  while (( $(date +%s) < deadline )); do
    if timeout 3 gz topic -l 2>/dev/null | grep -q .; then
      return 0
    fi
    sleep 1
  done
  return 1
}

# ---------------------------------------------------------------------------
_gnns_wait_sitl_heartbeat() {
  local timeout="${1:-90}"
  GNNS_SITL_TIMEOUT="$timeout" "${GNNS_VENV}/bin/python" - <<'PY'
import os, sys, time
from pymavlink import mavutil
t = float(os.environ.get("GNNS_SITL_TIMEOUT", "90"))
deadline = time.time() + t
while time.time() < deadline:
    try:
        m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", timeout=3)
        m.wait_heartbeat(timeout=5)
        sys.exit(0)
    except Exception:
        time.sleep(1.5)
sys.stderr.write(f"gnns: no heartbeat on tcp:127.0.0.1:5760 after {t}s\n")
sys.exit(1)
PY
}

# ---------------------------------------------------------------------------
# Apply readable, mouse-friendly tmux session options for the gnns session.
_gnns_tmux_style() {
  local s="$1"
  tmux set-option -t "$s" mouse on            >/dev/null 2>&1 || true
  tmux set-option -t "$s" history-limit 20000 >/dev/null 2>&1 || true
  tmux set-option -t "$s" status-interval 2   >/dev/null 2>&1 || true
  tmux set-option -t "$s" status-left  "[gnns] " >/dev/null 2>&1 || true
  tmux set-option -t "$s" status-right "%H:%M:%S " >/dev/null 2>&1 || true
  tmux set-window-option -t "$s" automatic-rename off >/dev/null 2>&1 || true
}

# ---------------------------------------------------------------------------
gnns_fly() {
  local no_attach=0
  local with_web=0
  local headless=0
  local a
  for a in "$@"; do
    case "$a" in
      --no-attach) no_attach=1 ;;
      --web)       with_web=1 ;;
      --headless)  headless=1 ;;
    esac
  done

  gnns_doctor || return 1

  command -v tmux >/dev/null 2>&1 || { echo "tmux required" >&2; return 1; }

  tmux has-session -t "$GNNS_TMUX_SESSION" 2>/dev/null && tmux kill-session -t "$GNNS_TMUX_SESSION"

  local gz_args=()
  (( headless == 1 )) && gz_args+=(--headless)

  # 1) Gazebo Harmonic
  tmux new-session -d -s "$GNNS_TMUX_SESSION" -n gazebo \
    "bash -lc 'source \"$REPO_ROOT/scripts/laptop_ubuntu24/env.sh\"; gnns_gazebo ${gz_args[*]}; echo; echo \"[gazebo exited]\"; exec bash'"
  _gnns_tmux_style "$GNNS_TMUX_SESSION"

  echo "[gnns] Waiting for Gazebo transport (gz topic -l) ..."
  if ! _gnns_wait_gazebo_ready "${GNNS_GAZEBO_TIMEOUT:-60}"; then
    echo "[gnns] Gazebo did not come up — attach: tmux attach -t $GNNS_TMUX_SESSION" >&2
    return 1
  fi
  echo "[gnns] Gazebo is up."

  # 2) ArduPilot SITL (only after gz is ready)
  tmux new-window -t "$GNNS_TMUX_SESSION" -n sitl \
    "bash -lc 'source \"$REPO_ROOT/scripts/laptop_ubuntu24/env.sh\"; cd \"$ARDUPILOT_HOME\" && sim_vehicle.py -v ArduCopter -f JSON --add-param-file=\"$GZ_PLUGIN/config/gazebo-iris-gimbal.parm\" --no-mavproxy; echo; echo \"[sitl exited]\"; exec bash'"

  echo "[gnns] Waiting for SITL heartbeat on tcp:127.0.0.1:5760 ..."
  if ! _gnns_wait_sitl_heartbeat "${GNNS_SITL_TIMEOUT:-120}"; then
    echo "[gnns] No heartbeat — attach: tmux attach -t $GNNS_TMUX_SESSION" >&2
    return 1
  fi
  echo "[gnns] SITL heartbeat received."

  # 3) Mission
  tmux new-window -t "$GNNS_TMUX_SESSION" -n mission \
    "bash -lc 'source \"$REPO_ROOT/scripts/laptop_ubuntu24/env.sh\"; source \"${GNNS_VENV}/bin/activate\"; cd \"$REPO_ROOT\" && python -m gnns_drone --sitl --demo; echo; echo \"[mission exited]\"; exec bash'"

  # 4) Optional: web control UI
  if (( with_web == 1 )); then
    tmux new-window -t "$GNNS_TMUX_SESSION" -n web \
      "bash -lc 'source \"$REPO_ROOT/scripts/laptop_ubuntu24/env.sh\"; source \"${GNNS_VENV}/bin/activate\"; cd \"$REPO_ROOT\" && python -m gnns_drone.web_control --sitl --port \"${GNNS_WEB_PORT}\"; echo; echo \"[web exited]\"; exec bash'"
  fi

  # Focus on the mission window so the user lands on the most relevant output.
  tmux select-window -t "${GNNS_TMUX_SESSION}:mission" 2>/dev/null || true

  echo ""
  echo "============================================================================"
  echo "  gNNS: tmux session '$GNNS_TMUX_SESSION'"
  if (( with_web == 1 )); then
    echo "  Windows: gazebo | sitl | mission | web (http://localhost:${GNNS_WEB_PORT})"
  else
    echo "  Windows: gazebo | sitl | mission   (add --web to start web UI too)"
  fi
  echo "  Attach:  tmux attach -t $GNNS_TMUX_SESSION"
  echo "  Switch:  Ctrl+b then 0/1/2/3       Detach: Ctrl+b then d"
  echo "  Clean:   bash gnns_ubuntu24.sh clean"
  echo "============================================================================"

  if [[ "$no_attach" -eq 0 ]]; then
    tmux attach -t "$GNNS_TMUX_SESSION"
  fi
}

# ---------------------------------------------------------------------------
gnns_status() {
  echo "[status] tmux:"
  tmux ls 2>/dev/null || echo "  (no tmux server running)"
  echo
  echo "[status] processes:"
  pgrep -af '[g]z sim|[g]z-sim-server|[s]im_vehicle.py|[a]rducopter|python -m gnns_drone' \
    || echo "  (none)"
  echo
  echo "[status] ports:"
  if command -v ss >/dev/null 2>&1; then
    ss -ltn 2>/dev/null | awk 'NR==1 || /:5760|:5762|:5763|:14550|:5000/'
  fi
}

# ---------------------------------------------------------------------------
gnns_clean() {
  tmux kill-session -t "$GNNS_TMUX_SESSION" 2>/dev/null || true
  # Harmonic launches gz via a ruby wrapper; the server is gz-sim-server.
  pkill -f '[g]z sim'         2>/dev/null || true
  pkill -f '[g]z-sim-server'  2>/dev/null || true
  pkill -f 'ruby .* gz '      2>/dev/null || true
  pkill -f '[s]im_vehicle.py' 2>/dev/null || true
  pkill -f '[a]rducopter'     2>/dev/null || true
  pkill -f 'python -m gnns_drone' 2>/dev/null || true
  rm -rf /tmp/gnns_drone-* 2>/dev/null || true
  echo "[gnns] clean done"
}
