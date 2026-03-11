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

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Run diagnostics first (validates FC connection + sensors)
python -m gnns_drone.diagnostics

# 3. Run the mission
python -m gnns_drone.mission_runner
```

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
├── scripts/                 # Utility scripts
├── requirements.txt
└── README.md
```
