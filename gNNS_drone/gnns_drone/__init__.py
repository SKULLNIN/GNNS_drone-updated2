"""
gNNS Drone — GPS-deNied Navigation System
==========================================
Main package for autonomous drone navigation without GPS.

Architecture:
  Jetson Nano (this code) ←MAVLink→ CubeOrange (ArduPilot)
  
Components:
  - mavlink_bridge: Communication layer with FC (handles all MAVLink)
  - vio_tracker: Visual Inertial Odometry position tracking
  - navigator: Waypoint navigation controller
  - diagnostics: Parameter validation & debugging tools
  - safety: Failsafe systems
  - coordinate_utils: GPS ↔ NED math
"""

__version__ = "0.1.0"
__project__ = "gNNS Drone"
