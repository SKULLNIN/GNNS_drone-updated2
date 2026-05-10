# Ubuntu 24.04 — run the drone end-to-end

This is the **only** supported path on Ubuntu 24.04 (noble): **ROS 2 Jazzy + Gazebo Harmonic + ArduPilot SITL**. The Noetic / Gazebo Classic 11 scripts (`start_simulation.sh`, `sitl/setup_*.sh`) are guarded and will refuse to run on 24.04.

---

## Step 0 — Prerequisites (one minute)

1. Fresh Ubuntu 24.04 with `sudo` access.
2. ~10 GB free disk.
3. Internet (apt repos, GitHub).
4. Get the code:
   ```bash
   git clone <your-repo-url> gNNS_drone
   cd gNNS_drone
   ```

> All commands below run from the repo root (`gNNS_drone/`) unless noted.

---

## Step 1 — One-time install (~15-30 min)

```bash
bash gnns_ubuntu24.sh install
```

This calls [scripts/laptop_ubuntu24/setup_laptop_ubuntu24.sh](../scripts/laptop_ubuntu24/setup_laptop_ubuntu24.sh) with `--all` and installs:

- Base apt deps + `tmux` + Python 3.12 / 3.14 venvs
- Intel **librealsense2** (RealSenseAI apt repo)
- **ROS 2 Jazzy** desktop + RTAB-Map + realsense2-camera + DDS RMWs
- **ArduPilot** at `~/ardupilot` (`Tools/environment_install/install-prereqs-ubuntu.sh`)
- **Gazebo Harmonic** (`gz-harmonic`, `libgz-sim8-dev`, `rapidjson-dev`)
- **ardupilot_gazebo** plugin built at `~/gz_ws/src/ardupilot_gazebo/build`
- Bashrc block: `ROS_DISTRO=jazzy`, `GZ_VERSION=harmonic`, `GZ_SIM_*` paths, `ardupilot/Tools/autotest` on `PATH`

---

## Step 2 — Open a fresh shell

```bash
exec bash -l
```

This is required because:

- the installer adds you to `dialout` / `plugdev` / `video` groups, and
- it appended a block to `~/.bashrc` (ROS / GZ paths).

---

## Step 3 — Verify

```bash
cd ~/gNNS_drone   # wherever you cloned it
bash gnns_ubuntu24.sh doctor
```

Expected output ends with `[doctor] All checks passed`. If anything reports `[err]`, fix that first.

Optional MAVROS / geoid check:

```bash
bash gnns_ubuntu24.sh doctor --with-mavros
```

---

## Step 4A — Fly (recommended, one command)

```bash
bash gnns_ubuntu24.sh fly
```

This:

1. Re-runs `doctor`.
2. Starts a `tmux` session **`gnns`** with three windows: **gazebo**, **sitl**, **mission**.
3. Waits for the **MAVLink heartbeat on `tcp:127.0.0.1:5760`** (90 s).
4. Launches `python -m gnns_drone --sitl --demo` (`MissionRunner` already forces `vio-source=sitl`).
5. Attaches you to the tmux session.

Tmux cheatsheet:

| Action | Keys |
|---|---|
| Switch window | `Ctrl+B` then `0` / `1` / `2` |
| Detach | `Ctrl+B` then `D` |
| Re-attach | `tmux attach -t gnns` |

Run in background without attaching:

```bash
bash gnns_ubuntu24.sh fly --no-attach
tmux attach -t gnns      # later
```

---

## Step 4B — Manual (three terminals)

Use this if you want full control over each tier.

**Terminal 1 — Gazebo Harmonic**

```bash
bash gnns_ubuntu24.sh gazebo
# or, with a custom world:
bash gnns_ubuntu24.sh gazebo --world ~/gz_ws/src/ardupilot_gazebo/worlds/iris_runway.sdf
# or headless:
bash gnns_ubuntu24.sh gazebo --headless
```

**Terminal 2 — ArduPilot SITL**

```bash
bash gnns_ubuntu24.sh sitl
```

Equivalent to:

```bash
cd ~/ardupilot
sim_vehicle.py -v ArduCopter -f JSON \
  --add-param-file=$HOME/gz_ws/src/ardupilot_gazebo/config/gazebo-iris-gimbal.parm \
  --no-mavproxy
```

Wait until SITL prints `IMU0 is using GPS` or similar steady-state messages.

**Terminal 3 — Mission**

```bash
bash gnns_ubuntu24.sh mission
# or, for the interactive UI:
bash gnns_ubuntu24.sh web      # open http://localhost:5000
```

---

## Step 5 — Stop everything

```bash
bash gnns_ubuntu24.sh clean
```

Kills the `gnns` tmux session, `gz sim` / `gz-sim-server`, `sim_vehicle.py`, `arducopter`, and any `python -m gnns_drone` process.

---

## Useful overrides

| Variable | Default | Purpose |
|---|---|---|
| `ARDUPILOT_HOME` | `$HOME/ardupilot` | Where ArduPilot was cloned |
| `GZ_PLUGIN` | `$HOME/gz_ws/src/ardupilot_gazebo` | ardupilot_gazebo checkout |
| `GNNS_VENV` | `<repo>/.venv-py312` | Python venv used by mission/web |
| `GNNS_TMUX_SESSION` | `gnns` | tmux session name for `fly` |
| `GNNS_WEB_PORT` | `5000` | Port for `web_control` |
| `GNNS_SITL_TIMEOUT` | `90` | Heartbeat wait inside `fly` (seconds) |

Source the env in your own shell to call the helpers directly:

```bash
source scripts/laptop_ubuntu24/env.sh
gnns_doctor
gnns_gazebo --headless &
gnns_sitl &
gnns_mission
```

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `gz: command not found` | `bash gnns_ubuntu24.sh install` then `exec bash -l` |
| `libArduPilotPlugin.so missing` | Rebuild: `cd $HOME/gz_ws/src/ardupilot_gazebo && rm -rf build && mkdir build && cd build && GZ_VERSION=harmonic cmake .. && make -j$(nproc)` |
| `no heartbeat on tcp:127.0.0.1:5760` | Make sure `sim_vehicle.py` is actually running in window **sitl**; check there for build errors |
| `Refusing to run on Ubuntu 24.04` from `start_simulation.sh` or `sitl/setup_*.sh` | Expected. Use `gnns_ubuntu24.sh` instead — those scripts are Noetic / GZ 11 only |
| `apt update` fails on `librealsense.intel.com` | Already fixed in `setup_laptop_ubuntu24.sh`; re-run `bash gnns_ubuntu24.sh install` |
| Stale ports / orphan `gz-sim-server` | `bash gnns_ubuntu24.sh clean` |

---

## What is happening under the hood

```
gnns_ubuntu24.sh fly
        │
        ├── doctor (preflight)
        │
        ├── tmux 'gnns' window 0: gz sim -v4 -r iris_runway.sdf
        │       (ardupilot_gazebo plugin loaded via GZ_SIM_SYSTEM_PLUGIN_PATH)
        │
        ├── tmux 'gnns' window 1: sim_vehicle.py -v ArduCopter -f JSON
        │       (talks to gazebo plugin via JSON socket; serves MAVLink on tcp:5760)
        │
        ├── wait for MAVLink heartbeat on tcp:127.0.0.1:5760
        │
        └── tmux 'gnns' window 2: python -m gnns_drone --sitl --demo
                (MissionRunner connects to tcp:5760, runs DEMO_WAYPOINTS)
```
