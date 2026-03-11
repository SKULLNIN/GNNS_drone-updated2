#!/bin/bash
# =================================================================
# gNNS Drone — Gazebo SITL Quick Setup
# =================================================================
# Run this in WSL2 to install everything needed for Gazebo sim.
# After this, you'll have a 3D drone flying in Gazebo!
#
# Usage:
#   chmod +x setup_gazebo.sh
#   ./setup_gazebo.sh
# =================================================================

set -e

echo "=========================================="
echo " gNNS Gazebo SITL Setup"
echo "=========================================="

# 1. Install Gazebo + ROS packages
echo ""
echo "[1/4] Installing Gazebo & ROS packages..."
sudo apt-get update
sudo apt-get install -y \
  gazebo11 \
  libgazebo11-dev \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-plugins \
  ros-noetic-mavros \
  ros-noetic-mavros-extras \
  python3-catkin-tools

# MAVROS geographic datasets
if [ ! -f /usr/share/GeographicLib/geoids/egm96-5.pgm ]; then
  echo "[1b] Installing MAVROS geographic datasets..."
  wget -q https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O /tmp/install_geo.sh
  chmod +x /tmp/install_geo.sh
  sudo /tmp/install_geo.sh
fi

echo "  [OK] Gazebo & ROS installed"

# 2. Install ardupilot_gazebo plugin
echo ""
echo "[2/4] Installing ardupilot_gazebo plugin..."

if [ ! -d "$HOME/ardupilot_gazebo" ]; then
  cd ~
  git clone https://github.com/khancyr/ardupilot_gazebo.git
  cd ardupilot_gazebo
  mkdir -p build && cd build
  cmake ..
  make -j$(nproc)
  sudo make install
  echo "  [OK] Plugin built and installed"
else
  echo "  Already installed"
fi

# Set environment
grep -q "ardupilot_gazebo" ~/.bashrc 2>/dev/null || {
  echo '' >> ~/.bashrc
  echo '# ArduPilot Gazebo' >> ~/.bashrc
  echo 'export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}' >> ~/.bashrc
  echo 'export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}' >> ~/.bashrc
}
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}

# 3. Ensure ArduPilot SITL is on PATH
echo ""
echo "[3/4] Checking ArduPilot SITL..."
grep -q "ardupilot/Tools/autotest" ~/.bashrc 2>/dev/null || {
  echo 'export PATH=$HOME/ardupilot/Tools/autotest:$PATH' >> ~/.bashrc
}
export PATH=$HOME/ardupilot/Tools/autotest:$PATH

if [ -f ~/ardupilot/Tools/autotest/sim_vehicle.py ]; then
  echo "  [OK] sim_vehicle.py found"
else
  echo "  ERROR: ArduPilot not found at ~/ardupilot"
  echo "  Run: git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot"
  exit 1
fi

# 4. Source ROS
echo ""
echo "[4/4] Sourcing ROS..."
source /opt/ros/noetic/setup.bash
echo "  [OK] ROS Noetic sourced"

echo ""
echo "=========================================="
echo " SETUP COMPLETE!"
echo "=========================================="
echo ""
echo " HOW TO RUN (3 terminals):"
echo ""
echo " Terminal 1 — Gazebo + SITL:"
echo "   source ~/.bashrc"
echo "   gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world &"
echo "   sleep 5"
echo "   ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console"
echo ""
echo " Terminal 2 — MAVROS:"
echo "   source /opt/ros/noetic/setup.bash"
echo "   roslaunch mavros apm.launch fcu_url:='udp://127.0.0.1:14551@14555'"
echo ""
echo " Terminal 3 — Flight test:"
echo "   python3 /mnt/c/Users/guutu/OneDrive/Desktop/mavlink/gNNS_drone/sitl/gazebo_flight_test.py"
echo ""
echo " OR simplified (no MAVROS, pymavlink direct):"
echo ""
echo " Terminal 1:"
echo "   source ~/.bashrc"
echo "   gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world &"
echo "   sleep 5"
echo "   ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --no-mavproxy"
echo ""
echo " Terminal 2:"
echo "   python3 /mnt/c/Users/guutu/OneDrive/Desktop/mavlink/gNNS_drone/sitl/gazebo_flight_test.py"
echo ""
echo "=========================================="
