#!/bin/bash
# =================================================================
# gNNS Drone — ArduPilot SITL + MAVROS + Gazebo Setup
# =================================================================
# Adapted from: https://github.com/ktelegenov/scripts/blob/main/noetic-sitl.md
# Modified for: ArduPilot (CubeOrange) instead of PX4
#               ROS Noetic + Gazebo 11
#               GPS-denied VIO-based navigation
#
# REQUIREMENTS:
#   - Ubuntu 20.04 (native or WSL2 with GUI)
#   - ROS Noetic installed
#   - ~10GB disk space
#
# WHAT THIS SETS UP:
#   1. ArduPilot SITL (simulates your CubeOrange FC)
#   2. MAVROS (ROS bridge to MAVLink — same protocol as pymavlink)
#   3. Gazebo 11 (3D physics simulation with camera/LiDAR)
#   4. ardupilot_gazebo (drone model + sensor plugins)
#   5. gNNS catkin workspace (our Python nodes as ROS packages)
#
# USAGE:
#   chmod +x setup_sitl_workspace.sh
#   ./setup_sitl_workspace.sh
# =================================================================

set -e  # Exit on error

echo "============================================"
echo " gNNS Drone — SITL Workspace Setup"
echo " ArduPilot + MAVROS + Gazebo"
echo "============================================"
echo ""

# =================================================================
# STEP 1: System dependencies
# =================================================================
echo "[1/7] Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y \
  git \
  python3-pip \
  python3-rosdep \
  python3-catkin-tools \
  python3-rosinstall \
  python3-rosinstall-generator \
  ros-noetic-mavros \
  ros-noetic-mavros-extras \
  ros-noetic-mavros-msgs \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-plugins \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-rqt \
  ros-noetic-rviz

# Install geographic datasets for MAVROS (required!)
echo "[1b] Installing MAVROS geographic datasets..."
wget -q https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh

echo "  [OK] System deps installed"

# =================================================================
# STEP 2: Install ArduPilot SITL
# =================================================================
echo ""
echo "[2/7] Installing ArduPilot SITL..."

if [ ! -d "$HOME/ardupilot" ]; then
  cd ~
  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
  cd ardupilot
  Tools/environment_install/install-prereqs-ubuntu.sh -y
  # Reload PATH so sim_vehicle.py is available
  . ~/.profile
else
  echo "  ArduPilot already exists at ~/ardupilot"
  cd ~/ardupilot
  git pull
  git submodule update --init --recursive
fi

# Ensure sim_vehicle.py is on PATH
# It lives at ~/ardupilot/Tools/autotest/sim_vehicle.py
if ! grep -q "ardupilot/Tools/autotest" ~/.bashrc; then
  echo '' >> ~/.bashrc
  echo '# ArduPilot SITL tools' >> ~/.bashrc
  echo 'export PATH=$HOME/ardupilot/Tools/autotest:$PATH' >> ~/.bashrc
fi
export PATH=$HOME/ardupilot/Tools/autotest:$PATH

# Verify sim_vehicle.py can be found
if command -v sim_vehicle.py &> /dev/null; then
  echo "  sim_vehicle.py found at: $(which sim_vehicle.py)"
else
  echo "  WARNING: sim_vehicle.py not found on PATH!"
  echo "  You can run it directly: ~/ardupilot/Tools/autotest/sim_vehicle.py"
fi

echo "  [OK] ArduPilot SITL ready"

# =================================================================
# STEP 3: Install ardupilot_gazebo plugin
# =================================================================
echo ""
echo "[3/7] Installing ardupilot_gazebo plugin..."

if [ ! -d "$HOME/ardupilot_gazebo" ]; then
  cd ~
  git clone https://github.com/khancyr/ardupilot_gazebo.git
  cd ardupilot_gazebo
  mkdir -p build && cd build
  cmake ..
  make -j$(nproc)
  sudo make install
else
  echo "  ardupilot_gazebo already exists"
fi

# Set Gazebo model path
if ! grep -q "ardupilot_gazebo" ~/.bashrc; then
  echo "" >> ~/.bashrc
  echo "# ArduPilot Gazebo models" >> ~/.bashrc
  echo 'export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}' >> ~/.bashrc
  echo 'export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}' >> ~/.bashrc
fi
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}

echo "  [OK] ardupilot_gazebo installed"

# =================================================================
# STEP 4: Create catkin workspace
# =================================================================
echo ""
echo "[4/7] Creating gNNS catkin workspace..."

GNNS_WS="$HOME/gnns_ws"

if [ ! -d "$GNNS_WS" ]; then
  mkdir -p $GNNS_WS/src
  cd $GNNS_WS
  catkin init
  catkin config --extend /opt/ros/noetic
  catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

echo "  Workspace: $GNNS_WS"
echo "  [OK] catkin workspace ready"

# =================================================================
# STEP 5: Create gNNS MAVROS launch package
# =================================================================
echo ""
echo "[5/7] Creating gNNS launch package..."

LAUNCH_PKG="$GNNS_WS/src/gnns_sitl_launch"

if [ ! -d "$LAUNCH_PKG" ]; then
  cd $GNNS_WS/src
  catkin_create_pkg gnns_sitl_launch rospy roscpp
fi

# Create launch directory
mkdir -p $LAUNCH_PKG/launch
mkdir -p $LAUNCH_PKG/config
mkdir -p $LAUNCH_PKG/worlds
mkdir -p $LAUNCH_PKG/scripts

echo "  [OK] Launch package created"

# =================================================================
# STEP 6: Create config files
# =================================================================
echo ""
echo "[6/7] Creating configuration files..."

# --- MAVROS APM config for SITL ---
cat > $LAUNCH_PKG/config/apm_sitl_config.yaml << 'YAML_END'
# ===========================================
# MAVROS config for ArduPilot SITL
# Adapted for gNNS GPS-denied drone
# ===========================================

# Connection to SITL
conn:
  heartbeat_rate: 1.0
  timeout: 10.0
  timesync_rate: 0.0

# Enable vision position input
vision_position:
  tf:
    listen: false
  frame_id: "map"
  child_frame_id: "base_link"

# Local position
local_position:
  frame_id: "map"
  tf:
    send: true
    frame_id: "map"
    child_frame_id: "base_link"

# IMU
imu:
  frame_id: "base_link"

# Set rates for messages we need
plugin_lists:
  plugin_blacklist:
    - 'safety_area'
    - 'actuator_control'
    - 'hil'
    - 'debug_value'
    - 'obstacle_distance'
    - 'rangefinder'
  plugin_whitelist: []

# Request stream rates (same as our mavlink_config.yaml)
startup_data_rate: 10

# EKF source config
# These will be set by our setup script
YAML_END

# --- Main SITL launch file ---
cat > $LAUNCH_PKG/launch/gnns_sitl.launch << 'LAUNCH_END'
<launch>
    <!-- ============================================ -->
    <!-- gNNS Drone — ArduPilot SITL + MAVROS Launch  -->
    <!-- ============================================ -->

    <!-- Arguments -->
    <arg name="fcu_url" default="udp://127.0.0.1:14551@14555"/>
    <arg name="gcs_url" default=""/>
    <arg name="tgt_system" default="1"/>
    <arg name="tgt_component" default="1"/>
    <arg name="vehicle" default="ArduCopter"/>
    <arg name="gazebo_world" default="$(find gnns_sitl_launch)/worlds/gnns_landing.world"/>
    <arg name="enable_gazebo" default="true"/>

    <!-- ============================================ -->
    <!-- MAVROS — ROS bridge to ArduPilot             -->
    <!-- ============================================ -->
    <include file="$(find mavros)/launch/node.launch">
        <arg name="pluginlists_yaml" value="$(find mavros)/launch/apm_pluginlists.yaml"/>
        <arg name="config_yaml" value="$(find gnns_sitl_launch)/config/apm_sitl_config.yaml"/>
        <arg name="fcu_url" value="$(arg fcu_url)"/>
        <arg name="gcs_url" value="$(arg gcs_url)"/>
        <arg name="tgt_system" value="$(arg tgt_system)"/>
        <arg name="tgt_component" value="$(arg tgt_component)"/>
    </include>

    <!-- ============================================ -->
    <!-- Gazebo World (optional, for visual testing)  -->
    <!-- ============================================ -->
    <group if="$(arg enable_gazebo)">
        <include file="$(find gazebo_ros)/launch/empty_world.launch">
            <arg name="world_name" value="$(arg gazebo_world)"/>
            <arg name="paused" value="false"/>
            <arg name="gui" value="true"/>
            <arg name="verbose" value="false"/>
        </include>
    </group>

    <!-- ============================================ -->
    <!-- gNNS VIO Simulator Node                      -->
    <!-- Sends fake VISION_POSITION_ESTIMATE via MAVROS -->
    <!-- ============================================ -->
    <node pkg="gnns_sitl_launch" type="vio_sim_node.py" name="vio_simulator"
          output="screen" respawn="false">
        <param name="publish_rate" value="30.0"/>
        <param name="noise_std_pos" value="0.01"/>
        <param name="noise_std_yaw" value="0.005"/>
    </node>

</launch>
LAUNCH_END

# --- gNNS-only launch (no Gazebo, MAVROS only) ---
cat > $LAUNCH_PKG/launch/gnns_mavros_only.launch << 'LAUNCH_END'
<launch>
    <!-- Lightweight: just MAVROS + VIO sim, no Gazebo -->
    <arg name="fcu_url" default="udp://127.0.0.1:14551@14555"/>

    <include file="$(find mavros)/launch/node.launch">
        <arg name="pluginlists_yaml" value="$(find mavros)/launch/apm_pluginlists.yaml"/>
        <arg name="config_yaml" value="$(find gnns_sitl_launch)/config/apm_sitl_config.yaml"/>
        <arg name="fcu_url" value="$(arg fcu_url)"/>
        <arg name="gcs_url" value=""/>
        <arg name="tgt_system" value="1"/>
        <arg name="tgt_component" value="1"/>
    </include>

    <node pkg="gnns_sitl_launch" type="vio_sim_node.py" name="vio_simulator"
          output="screen">
        <param name="publish_rate" value="30.0"/>
    </node>
</launch>
LAUNCH_END

echo "  [OK] Config files created"

# =================================================================
# STEP 7: Create VIO simulator node
# =================================================================
echo ""
echo "[7/7] Creating VIO simulator node..."

cat > $LAUNCH_PKG/scripts/vio_sim_node.py << 'PYTHON_END'
#!/usr/bin/env python3
"""
gNNS Drone — VIO Simulator for SITL
=====================================
Subscribes to MAVROS local position (from SITL's GPS-based EKF)
and republishes as vision_position_estimate (simulating RTAB-Map).

This lets us test GPS-denied navigation by:
  1. Disabling GPS in ArduPilot params
  2. Feeding position via MAVROS vision_pose topic
  3. ArduPilot uses this as VIO source

In real flight: RTAB-Map → pymavlink → VISION_POSITION_ESTIMATE
In simulation:  Gazebo    → this node → MAVROS vision_pose → ArduPilot SITL
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.srv import ParamSet, ParamSetRequest
from mavros_msgs.msg import ParamValue
import tf.transformations as tft

class VIOSimulator:
    def __init__(self):
        rospy.init_node('vio_simulator', anonymous=True)

        self.rate = rospy.get_param('~publish_rate', 30.0)
        self.noise_std_pos = rospy.get_param('~noise_std_pos', 0.01)
        self.noise_std_yaw = rospy.get_param('~noise_std_yaw', 0.005)

        # Subscribe to SITL's ground truth position
        self.local_pos_sub = rospy.Subscriber(
            '/mavros/local_position/odom', Odometry, self.odom_cb
        )

        # Publish vision position (this is what RTAB-Map would send)
        self.vision_pub = rospy.Publisher(
            '/mavros/vision_pose/pose', PoseStamped, queue_size=10
        )

        # State monitoring
        self.state_sub = rospy.Subscriber(
            '/mavros/state', State, self.state_cb
        )

        self.current_state = State()
        self.latest_odom = None
        self.configured = False

        rospy.loginfo("VIO Simulator started (rate=%.1f Hz, noise=%.3fm)",
                      self.rate, self.noise_std_pos)

    def state_cb(self, msg):
        self.current_state = msg

    def odom_cb(self, msg):
        self.latest_odom = msg

    def configure_ekf_for_vio(self):
        """Set ArduPilot params for GPS-denied VIO navigation."""
        rospy.loginfo("Configuring EKF for VIO-only navigation...")
        rospy.wait_for_service('/mavros/param/set', timeout=10)

        set_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)

        params = {
            # Use EKF3
            'AHRS_EKF_TYPE': 3,
            'EK3_ENABLE': 1,
            'EK2_ENABLE': 0,

            # ExternalNav as position/velocity source
            'EK3_SRC1_POSXY': 6,   # ExternalNav
            'EK3_SRC1_VELXY': 6,   # ExternalNav
            'EK3_SRC1_POSZ': 6,    # ExternalNav
            'EK3_SRC1_VELZ': 6,    # ExternalNav
            'EK3_SRC1_YAW': 6,     # ExternalNav

            # Enable vision position input
            'VISO_TYPE': 1,

            # Disable GPS
            'GPS1_TYPE': 0,
            'GPS2_TYPE': 0,

            # Skip GPS arming check
            'ARMING_CHECK': 16310,  # All except GPS (bit 15)
        }

        for name, value in params.items():
            try:
                req = ParamSetRequest()
                req.param_id = name
                req.value = ParamValue()
                req.value.integer = int(value) if isinstance(value, int) else 0
                req.value.real = float(value)
                resp = set_param(req)
                if resp.success:
                    rospy.loginfo("  Set %s = %s", name, value)
                else:
                    rospy.logwarn("  Failed to set %s", name)
            except Exception as e:
                rospy.logwarn("  Error setting %s: %s", name, e)

        rospy.loginfo("EKF configured for VIO! Restart SITL for params to take effect.")
        self.configured = True

    def run(self):
        """Main loop: forward position as vision estimate at configured rate."""
        rate = rospy.Rate(self.rate)

        # Wait for FC connection
        rospy.loginfo("Waiting for FC connection...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
        rospy.loginfo("FC connected!")

        # Configure EKF for VIO (first run only)
        if not self.configured:
            try:
                self.configure_ekf_for_vio()
            except Exception as e:
                rospy.logwarn("Could not configure EKF: %s", e)
                rospy.logwarn("Continue with existing params...")

        rospy.loginfo("Publishing vision position at %.1f Hz...", self.rate)

        while not rospy.is_shutdown():
            if self.latest_odom is not None:
                # Create vision pose from ground truth + noise
                vision_msg = PoseStamped()
                vision_msg.header.stamp = rospy.Time.now()
                vision_msg.header.frame_id = "map"

                # Add small noise to simulate real VIO (RTAB-Map)
                vision_msg.pose.position.x = (
                    self.latest_odom.pose.pose.position.x +
                    np.random.normal(0, self.noise_std_pos)
                )
                vision_msg.pose.position.y = (
                    self.latest_odom.pose.pose.position.y +
                    np.random.normal(0, self.noise_std_pos)
                )
                vision_msg.pose.position.z = (
                    self.latest_odom.pose.pose.position.z +
                    np.random.normal(0, self.noise_std_pos)
                )

                # Orientation with small yaw noise
                q = self.latest_odom.pose.pose.orientation
                euler = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
                noisy_yaw = euler[2] + np.random.normal(0, self.noise_std_yaw)
                q_noisy = tft.quaternion_from_euler(euler[0], euler[1], noisy_yaw)
                vision_msg.pose.orientation.x = q_noisy[0]
                vision_msg.pose.orientation.y = q_noisy[1]
                vision_msg.pose.orientation.z = q_noisy[2]
                vision_msg.pose.orientation.w = q_noisy[3]

                self.vision_pub.publish(vision_msg)

            rate.sleep()


if __name__ == '__main__':
    try:
        node = VIOSimulator()
        node.run()
    except rospy.ROSInterruptException:
        pass
PYTHON_END

chmod +x $LAUNCH_PKG/scripts/vio_sim_node.py

# =================================================================
# Create Gazebo world with landing targets
# =================================================================
cat > $LAUNCH_PKG/worlds/gnns_landing.world << 'WORLD_END'
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="gnns_landing_world">

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Landing targets (5 colored pads) -->
    <!-- Target 1: 10m North, 5m East -->
    <model name="landing_pad_1">
      <static>true</static>
      <pose>10 5 0.001 0 0 0</pose>
      <link name="pad">
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.5</radius><length>0.002</length></cylinder>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Target 2: -5m North, 10m East -->
    <model name="landing_pad_2">
      <static>true</static>
      <pose>-5 10 0.001 0 0 0</pose>
      <link name="pad">
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.5</radius><length>0.002</length></cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Target 3: -10m North, -5m East -->
    <model name="landing_pad_3">
      <static>true</static>
      <pose>-10 -5 0.001 0 0 0</pose>
      <link name="pad">
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.5</radius><length>0.002</length></cylinder>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Target 4: 8m North, -8m East -->
    <model name="landing_pad_4">
      <static>true</static>
      <pose>8 -8 0.001 0 0 0</pose>
      <link name="pad">
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.5</radius><length>0.002</length></cylinder>
          </geometry>
          <material>
            <ambient>1 1 0 1</ambient>
            <diffuse>1 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Target 5: 3m North, 3m East (near home) -->
    <model name="landing_pad_5">
      <static>true</static>
      <pose>3 3 0.001 0 0 0</pose>
      <link name="pad">
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.5</radius><length>0.002</length></cylinder>
          </geometry>
          <material>
            <ambient>1 0 1 1</ambient>
            <diffuse>1 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

  </world>
</sdf>
WORLD_END

# =================================================================
# Create quick-test script
# =================================================================
cat > $LAUNCH_PKG/scripts/test_offboard.py << 'PYTHON_END'
#!/usr/bin/env python3
"""
gNNS Drone — MAVROS OFFBOARD Test
===================================
Arms, takes off, flies a square, lands.
Adapted from ktelegenov's offb example for ArduPilot.

Run:  rosrun gnns_sitl_launch test_offboard.py
"""
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node('gnns_offboard_test')

    state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
    pos_pub = rospy.Publisher('/mavros/setpoint_position/local',
                              PoseStamped, queue_size=10)
    arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    rate = rospy.Rate(20)

    # Wait for FC connection
    rospy.loginfo("Waiting for FC connection...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    rospy.loginfo("Connected!")

    # Define waypoints (match our landing pads)
    waypoints = [
        (0, 0, 2.5),     # Hover at home
        (10, 5, 2.5),    # WP1: near pad 1
        (-5, 10, 2.5),   # WP2: near pad 2
        (-10, -5, 2.5),  # WP3: near pad 3
        (8, -8, 2.5),    # WP4: near pad 4
        (3, 3, 2.5),     # WP5: near pad 5
        (0, 0, 2.5),     # Return home
        (0, 0, 0.0),     # Land
    ]

    # Send initial setpoints (required before GUIDED mode)
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2.5

    for i in range(100):
        if rospy.is_shutdown():
            return
        pos_pub.publish(pose)
        rate.sleep()

    # Set GUIDED mode
    rospy.loginfo("Setting GUIDED mode...")
    mode_client(custom_mode="GUIDED")
    rospy.sleep(1)

    # Arm
    rospy.loginfo("Arming...")
    arm_client(value=True)
    rospy.sleep(2)

    # Fly waypoints
    for wp_idx, (x, y, z) in enumerate(waypoints):
        rospy.loginfo("Flying to WP%d: (%.1f, %.1f, %.1f)", wp_idx, x, y, z)

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # Fly for 5 seconds per waypoint
        for _ in range(100):
            if rospy.is_shutdown():
                return
            pos_pub.publish(pose)
            rate.sleep()

        rospy.loginfo("  Reached WP%d", wp_idx)

        # Simulate landing pause at each pad (except first and last)
        if 1 <= wp_idx <= 5:
            rospy.loginfo("  Simulating landing at WP%d...", wp_idx)
            # Descend
            pose.pose.position.z = 0.0
            for _ in range(60):
                pos_pub.publish(pose)
                rate.sleep()
            rospy.loginfo("  Landed! Waiting 2s...")
            rospy.sleep(2)
            # Take off again
            pose.pose.position.z = 2.5
            for _ in range(60):
                pos_pub.publish(pose)
                rate.sleep()
            rospy.loginfo("  Takeoff complete")

    # Disarm
    rospy.loginfo("Disarming...")
    arm_client(value=False)
    rospy.loginfo("Mission complete!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
PYTHON_END

chmod +x $LAUNCH_PKG/scripts/test_offboard.py

# =================================================================
# Build workspace
# =================================================================
echo ""
echo "[BUILD] Building catkin workspace..."
cd $GNNS_WS
catkin build

# Source workspace
if ! grep -q "gnns_ws" ~/.bashrc; then
  echo "" >> ~/.bashrc
  echo "# gNNS workspace" >> ~/.bashrc
  echo "source $GNNS_WS/devel/setup.bash" >> ~/.bashrc
fi
source $GNNS_WS/devel/setup.bash

echo ""
echo "============================================"
echo " SETUP COMPLETE!"
echo "============================================"
echo ""
echo " Your workspace: $GNNS_WS"
echo ""
echo " HOW TO RUN:"
echo " ─────────────────────────────────────"
echo ""
echo " Terminal 1 — Start ArduPilot SITL:"
echo "   sim_vehicle.py -v ArduCopter --console --map --out=udp:127.0.0.1:14551"
echo ""
echo "   (sim_vehicle.py is at ~/ardupilot/Tools/autotest/sim_vehicle.py)"
echo "   (If not on PATH, run: export PATH=\$HOME/ardupilot/Tools/autotest:\$PATH)"
echo ""
echo " Terminal 2 — Start MAVROS + VIO Sim:"
echo "   roslaunch gnns_sitl_launch gnns_mavros_only.launch"
echo ""
echo " Terminal 3 — Run test:"
echo "   rosrun gnns_sitl_launch test_offboard.py"
echo ""
echo " WITH GAZEBO (Full 3D simulation):"
echo "   Terminal 1: sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out=udp:127.0.0.1:14551"
echo "   Terminal 2: roslaunch gnns_sitl_launch gnns_sitl.launch"
echo "   Terminal 3: rosrun gnns_sitl_launch test_offboard.py"
echo ""
echo " MONITOR:"
echo "   rostopic echo /mavros/state"
echo "   rostopic echo /mavros/local_position/pose"
echo "   rostopic echo /mavros/vision_pose/pose"
echo "   rqt"
echo ""
echo "============================================"
