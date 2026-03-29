# gNNS Drone — GPS-deNied Navigation System

A GPS-denied autonomous quadcopter navigation system using Visual Inertial Odometry (VIO),
LiDAR SLAM, and custom MAVLink messaging between Jetson Nano and CubeOrange FC.

## Architecture

```
Jetson Nano (Companion Computer)
├── VIO Node (RealSense T265/D435i)
├── MAVLink Bridge (custom messages)
├── Navigation Controller
├── Diagnostics & Debug Monitor
└── ROS2 Integration Layer

CubeOrange (Flight Controller)
├── ArduPilot Copter
├── EKF3 (ExternalNav source)
└── Custom MAVLink Dialect
```

## Quick Start

**On a development machine (Windows/Linux):**
```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Run diagnostics first (validates FC connection + sensors)
python -m gnns_drone.diagnostics

# 3. Run the mission
python -m gnns_drone
```

**On Jetson Nano (one-command run):**
```bash
# One-time: make executable, then run
chmod +x scripts/jetson_nano/run_mission.sh
./scripts/jetson_nano/run_mission.sh          # Full mission (enter GPS)
./scripts/jetson_nano/run_mission.sh --demo   # Demo waypoints
```
See [scripts/jetson_nano/README.md](scripts/jetson_nano/README.md) for setup and options.

## ROS2 VIO Stack (RealSense + ORB-SLAM3 + RTAB-Map)

For RealSense with ORB-SLAM3 VIO and RTAB-Map mapping:

1. Set `config/vio_config.yaml` → `ros2_odom.odom_source: "orbslam3"` (or `"ros2"` for RTAB-Map odom)
2. Follow exact bring-up order in [docs/ROS2_VIO_SETUP.md](docs/ROS2_VIO_SETUP.md): RealSense → IMU/TF → ORB-SLAM3 → RTAB-Map (includes calibration and EKF tuning)
3. Run: `python -m gnns_drone --vio-source orbslam3`

## Project Structure

```
gNNS_drone/
├── config/                  # All configuration files
│   ├── fc_params.yaml       # ArduPilot parameters
│   ├── mavlink_config.yaml  # MAVLink connection settings
│   └── vio_config.yaml      # VIO/sensor settings
├── mavlink_dialect/         # Custom MAVLink XML definitions
│   └── gnns_drone.xml       # Custom messages for gNNS
├── gnns_drone/              # Main Python package
│   ├── __init__.py
│   ├── mavlink_bridge.py    # MAVLink communication layer
│   ├── vio_tracker.py       # VIO position tracking
│   ├── navigator.py         # Waypoint navigation logic
│   ├── mission_runner.py    # Main mission entry point
│   ├── diagnostics.py       # Debug & parameter validation
│   ├── coordinate_utils.py  # GPS ↔ NED conversion
│   └── safety.py            # Failsafe systems
├── ros2_ws/                 # ROS2 workspace (optional)
│   └── src/gnns_drone_ros/
├── tests/                   # Unit & integration tests
├── scripts/
│   └── jetson_nano/         # Run on Jetson Nano (run_mission.sh, setup, udev)
├── requirements.txt
└── README.md
```

## Documentation

| Guide | Description |
|-------|-------------|
| [Jetson-to-Laptop Setup](docs/JETSON_LAPTOP_SETUP.md) | Network, SSH, DDS, `ros_bridge.sh`, `laptop_rviz2.sh`, RViz2 displays, bandwidth, troubleshooting |
| [Full Mission Guide](docs/MISSION_GUIDE.md) | Hardware overview, FC parameters, config files, VIO bringup, running the mission, web control, safety, pre-flight checklist |
| [SITL Simulation Guide](docs/SITL_GUIDE.md) | ArduPilot SITL setup, test scripts, Gazebo, connection details, troubleshooting |
| [ROS2 VIO Setup](docs/ROS2_VIO_SETUP.md) | RealSense + ORB-SLAM3 / RTAB-Map bring-up order, calibration, EKF tuning |
