# Jetson Nano — run gNNS Drone

Run the mission directly on the Jetson Nano with minimal setup.

## Jetson + laptop (D455 + RTAB-Map + RViz)

Simple two-process flow (see `docs/JETSON_LAPTOP_SETUP.md`):

```bash
# Jetson — two terminals
./scripts/jetson_nano/d455_launch.sh
./scripts/jetson_nano/rtabmap_vio.sh

# Laptop
chmod +x scripts/jetson_nano/laptop_rviz2.sh scripts/jetson_nano/verify_bridge.sh
./scripts/jetson_nano/laptop_rviz2.sh 42
```

Use `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=0`, and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` on both machines (scripts set these).

### Integrated stack (RealSense + RTAB-Map + optional gNNS mission)

Use **`scripts/jetson_nano/gnns_vio_stack.sh`** when you want one script that:

- Launches RealSense with **gyro + accel + fused IMU** (`unite_imu_method:=2`) — required to avoid missing-IMU / `bad optional access` style failures when RTAB-Map expects `/camera/camera/imu`.
- Starts **`rgbd_odometry` as its own process** (`rtabmap_odom` package) with **`wait_imu_to_init:=false`** on the VO node, then launches **RTAB-Map SLAM** with **`visual_odometry:=false`** so mapping subscribes to external `/odom` — avoids combined-launch IMU/sync races that yield **no odometry**.
- Uses the same tuned `rtabmap_args` presets (aligned depth, queue, approx sync).
- Optionally runs the Python mission with `--vio-source ros2` after `/odom` is live.

```bash
chmod +x scripts/jetson_nano/gnns_vio_stack.sh

# Terminal A — full stack (camera in background, RTAB-Map + viz in foreground)
./scripts/jetson_nano/gnns_vio_stack.sh stack

# Terminal B — FC + mission (after /odom is publishing)
./scripts/jetson_nano/gnns_vio_stack.sh mission --demo
```

See the script header for `realsense` / `rtabmap` / `mission` / `diagnose` modes.

**Stuck on “Waiting to initialize IMU orientation”?**  
By default the script sets `GNNS_WAIT_IMU=0` so RTAB-Map does **not** block on IMU. After `ros2 topic hz /camera/camera/imu` works, you can use `GNNS_WAIT_IMU=1 ./scripts/jetson_nano/gnns_vio_stack.sh stack`. If your IMU topic differs, set `GNNS_IMU_TOPIC` (run `./scripts/jetson_nano/gnns_vio_stack.sh diagnose`).

**Stack mode: RealSense runs but RTAB-Map does not**  
Fixed: RealSense logs are redirected so the background PID is correct. Check `/tmp/gnns_realsense.log`, `/tmp/gnns_rgbd_odometry.log`, and `/tmp/gnns_rtabmap.log` (or `GNNS_LOG_DIR`).

## Quick run

From the **project root** on the Jetson Nano:

```bash
# Make scripts executable (once)
chmod +x scripts/jetson_nano/run_mission.sh
chmod +x scripts/jetson_nano/setup_jetson_nano.sh

# Run full mission (enter GPS at prompts)
./scripts/jetson_nano/run_mission.sh

# Demo with built-in waypoints (no GPS input)
./scripts/jetson_nano/run_mission.sh --demo
```

You can run from any directory; the script switches to the project root automatically.

## One-time setup on the Nano

1. **Clone/copy the repo** onto the Nano (e.g. `~/gNNS_drone`).

2. **Install dependencies:**
   ```bash
   cd ~/gNNS_drone
   ./scripts/jetson_nano/setup_jetson_nano.sh
   ```
   Or manually:
   ```bash
   pip3 install -r requirements.txt
   ```

3. **Serial port (CubeOrange TELEM2):**
   - Default config uses `/dev/ttyTHS1` (Jetson hardware UART).
   - To use without root:
     ```bash
     sudo cp scripts/jetson_nano/99-gnns-telem.rules /etc/udev/rules.d/
     sudo udevadm control --reload-rules && sudo udevadm trigger
     ```
   - If you use a USB–serial adapter, set `connection.port` in `config/mavlink_config.yaml` (e.g. `/dev/ttyUSB0`).

4. **ROS2 (if using RTAB-Map / RealSense):**  
   Source your ROS2 workspace before running the mission if odometry comes from ROS2.

## Options

Pass any mission-runner arguments after the script:

| Command | Description |
|--------|-------------|
| `./scripts/jetson_nano/run_mission.sh` | Normal mission (enter home + 5 waypoints) |
| `./scripts/jetson_nano/run_mission.sh --demo` | Demo waypoints (no input) |
| `./scripts/jetson_nano/run_mission.sh --config /path/to/mavlink_config.yaml` | Custom MAVLink config |
| `./scripts/jetson_nano/run_mission.sh --interactive` | Takeoff, then enter coordinates live |

SITL is for development on a PC; on the Nano you run against the real FC.

## Project root

The launcher resolves the project root from the script path (`scripts/jetson_nano/` → two levels up). Config paths in the code (e.g. `config/mavlink_config.yaml`) are relative to that root, so no `PYTHONPATH` or install is required when using this script.
