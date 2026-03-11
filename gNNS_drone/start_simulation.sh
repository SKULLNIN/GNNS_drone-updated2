#!/bin/bash

# 1. KILL EVERYTHING STALE
echo "Cleaning up processes..."
killall -9 gzserver gzclient rosmaster roscore arducopter mavproxy.py xterm python3 2>/dev/null
pkill -9 -f gazebo
pkill -9 -f arducopter
sleep 2

# 2. SOURCE ENVIRONMENTS
source /opt/ros/noetic/setup.bash
source /usr/share/gazebo/setup.bash

# 3. SET PATHS
export GAZEBO_MODEL_PATH=$HOME/.gazebo/models:/usr/share/gazebo-11/models
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export LD_LIBRARY_PATH=/opt/ros/noetic/lib:$LD_LIBRARY_PATH
export DISPLAY=:0

# 4. START ROS CORE
echo "Starting roscore..."
roscore &
sleep 5

# 5. START GAZEBO
echo "Starting Gazebo..."
# Using the full path to the plugin to be absolutely sure
gazebo --verbose -s /opt/ros/noetic/lib/libgazebo_ros_api_plugin.so \
  /mnt/c/Users/guutu/OneDrive/Desktop/mavlink/gNNS_drone/sitl/gazebo_worlds/gnns_depth.world &
sleep 10

# 6. START ARDUPILOT
echo "Starting ArduPilot..."
cd /root/ardupilot
sim_vehicle.py -v ArduCopter -f gazebo-iris --no-mavproxy &

echo "Simulation sequence started. Check terminals."
