#!/bin/bash
# ============================================================
# gNNS Drone — Depth Camera Setup for Gazebo Classic 11
# ============================================================
# Sets up depth camera simulation WITHOUT ROS 2.
# Uses Gazebo Classic's native depth camera sensor plugin.
#
# What this does:
#   1. Installs required Gazebo plugins (if missing)
#   2. Copies the depth-camera Iris model to Gazebo model path
#   3. Sets up ArduPilot proximity sensor parameters
#   4. Installs Python deps for reading depth data
#
# Run: bash sitl/setup_depth_camera.sh
# ============================================================

set -e
echo ""
echo "============================================"
echo "  gNNS Drone — Depth Camera Setup"
echo "  Gazebo Classic 11 (no ROS 2 needed)"
echo "============================================"
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
GAZEBO_MODELS="${HOME}/.gazebo/models"
IRIS_DEPTH_MODEL="${GAZEBO_MODELS}/iris_with_depth_camera"

# ── Step 1: Check Gazebo ────────────────────────────────────
echo "[1/5] Checking Gazebo..."
if ! command -v gazebo &> /dev/null; then
    echo "ERROR: Gazebo not found! Install with:"
    echo "  sudo apt install gazebo11 libgazebo11-dev"
    exit 1
fi
GAZEBO_VER=$(gazebo --version | grep "version" | awk '{print $4}')
echo "  Gazebo version: $GAZEBO_VER ✓"

# ── Step 2: Install dependencies ────────────────────────────
echo "[2/5] Installing Python dependencies..."
pip3 install -q numpy opencv-python-headless 2>/dev/null || true

# Check for Gazebo dev headers (needed for custom plugins)
if ! dpkg -l | grep -q libgazebo11-dev; then
    echo "  Installing libgazebo11-dev..."
    sudo apt-get install -y libgazebo11-dev 2>/dev/null || echo "  WARNING: Could not install libgazebo11-dev"
fi

echo "  Dependencies OK ✓"

# ── Step 3: Create Iris model with depth camera ─────────────
echo "[3/5] Creating Iris drone model with depth camera..."
mkdir -p "$IRIS_DEPTH_MODEL"

# Model config
cat > "${IRIS_DEPTH_MODEL}/model.config" << 'MODELCFG'
<?xml version="1.0"?>
<model>
  <name>iris_with_depth_camera</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>
    ArduPilot Iris drone with downward-facing depth camera
    for obstacle detection (gNNS Drone project).
  </description>
</model>
MODELCFG

# SDF model — includes Iris + depth camera sensor
cat > "${IRIS_DEPTH_MODEL}/model.sdf" << 'MODELSDF'
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="iris_with_depth_camera">

    <!-- Include base Iris model -->
    <include>
      <uri>model://iris_with_ardupilot</uri>
    </include>

    <!-- ================================================== -->
    <!-- DEPTH CAMERA (RealSense D435 style) -->
    <!-- Mounted facing DOWN on the drone body -->
    <!-- ================================================== -->
    <link name="depth_camera_link">
      <pose>0 0 -0.05 0 1.5708 0</pose>  <!-- Facing DOWN (90° pitch) -->
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz>
        </inertia>
      </inertial>

      <visual name="depth_camera_visual">
        <geometry>
          <box><size>0.03 0.09 0.025</size></box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>

      <sensor name="depth_camera" type="depth">
        <update_rate>15</update_rate>
        <camera name="depth_cam">
          <!-- RealSense D435-like specs (reduced res for performance) -->
          <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80 degrees -->
          <image>
            <width>320</width>
            <height>240</height>
            <format>R_FLOAT32</format>
          </image>
          <clip>
            <near>0.2</near>
            <far>10.0</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.005</stddev>
          </noise>
          <depth_camera>
            <output>depths</output>
          </depth_camera>
        </camera>

        <!-- Publishes depth image on Gazebo transport topic -->
        <plugin name="depth_camera_plugin" filename="libgazebo_ros_depth_camera.so">
          <alwaysOn>true</alwaysOn>
          <updateRate>15</updateRate>
          <imageTopicName>/gnns/depth/image</imageTopicName>
          <depthImageTopicName>/gnns/depth/image_raw</depthImageTopicName>
          <pointCloudTopicName>/gnns/depth/points</pointCloudTopicName>
          <cameraInfoTopicName>/gnns/depth/camera_info</cameraInfoTopicName>
          <frameName>depth_camera_link</frameName>
          <pointCloudCutoff>0.2</pointCloudCutoff>
          <pointCloudCutoffMax>10.0</pointCloudCutoffMax>
        </plugin>
      </sensor>

      <!-- RGB camera for visual reference -->
      <sensor name="rgb_camera" type="camera">
        <update_rate>10</update_rate>
        <camera name="rgb_cam">
          <horizontal_fov>1.3962634</horizontal_fov>
          <image>
            <width>320</width>
            <height>240</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>50.0</far>
          </clip>
        </camera>
      </sensor>
    </link>

    <!-- Attach camera to drone body -->
    <joint name="depth_camera_joint" type="fixed">
      <parent>iris_with_ardupilot::iris::base_link</parent>
      <child>depth_camera_link</child>
    </joint>

  </model>
</sdf>
MODELSDF

echo "  Model created at: $IRIS_DEPTH_MODEL ✓"

# ── Step 4: Update world file ───────────────────────────────
echo "[4/5] Creating depth camera world file..."
WORLD_FILE="${SCRIPT_DIR}/gazebo_worlds/gnns_depth.world"
mkdir -p "$(dirname "$WORLD_FILE")"

cat > "$WORLD_FILE" << 'WORLDSDF'
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="gnns_depth_world">

    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.4 0.6 0.9 1</background>
      <shadows>true</shadows>
    </scene>

    <include><uri>model://sun</uri></include>

    <!-- Ground -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- ========== OBSTACLES (depth camera will see these) ========== -->

    <!-- Tree 1: 8m North, 3m East -->
    <model name="tree_1">
      <static>true</static>
      <pose>8 3 1 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><cylinder><radius>0.3</radius><length>2</length></cylinder></geometry></collision>
        <visual name="vis"><geometry><cylinder><radius>0.3</radius><length>2</length></cylinder></geometry>
          <material><ambient>0.2 0.5 0.1 1</ambient></material></visual>
      </link>
    </model>

    <!-- Tree 2: -5m N, 7m E -->
    <model name="tree_2">
      <static>true</static>
      <pose>-5 7 1.5 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><cylinder><radius>0.4</radius><length>3</length></cylinder></geometry></collision>
        <visual name="vis"><geometry><cylinder><radius>0.4</radius><length>3</length></cylinder></geometry>
          <material><ambient>0.2 0.5 0.1 1</ambient></material></visual>
      </link>
    </model>

    <!-- Building: 5m N, -6m E -->
    <model name="building_1">
      <static>true</static>
      <pose>5 -6 1 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>3 2 2</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>3 2 2</size></box></geometry>
          <material><ambient>0.6 0.4 0.3 1</ambient></material></visual>
      </link>
    </model>

    <!-- Wall: -8m N, -4m E -->
    <model name="wall_1">
      <static>true</static>
      <pose>-8 -4 0.5 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>4 0.3 1</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>4 0.3 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material></visual>
      </link>
    </model>

    <!-- Rocks: 3m N, 10m E -->
    <model name="rock_1">
      <static>true</static>
      <pose>3 10 0.3 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><sphere><radius>0.6</radius></sphere></geometry></collision>
        <visual name="vis"><geometry><sphere><radius>0.6</radius></sphere></geometry>
          <material><ambient>0.4 0.35 0.3 1</ambient></material></visual>
      </link>
    </model>

    <model name="rock_2">
      <static>true</static>
      <pose>4 10.5 0.25 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>
        <visual name="vis"><geometry><sphere><radius>0.5</radius></sphere></geometry>
          <material><ambient>0.4 0.35 0.3 1</ambient></material></visual>
      </link>
    </model>

    <!-- Tall box obstacle: 0m N, 8m E -->
    <model name="crate_1">
      <static>true</static>
      <pose>0 8 0.5 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>1 1 1</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>1 1 1</size></box></geometry>
          <material><ambient>0.6 0.5 0.2 1</ambient></material></visual>
      </link>
    </model>

    <!-- ========== DRONE with DEPTH CAMERA ========== -->
    <include>
      <uri>model://iris_with_depth_camera</uri>
      <pose>0 0 0.05 0 0 0</pose>
    </include>

  </world>
</sdf>
WORLDSDF

echo "  World file: $WORLD_FILE ✓"

# ── Step 5: ArduPilot params for proximity sensor ───────────
echo "[5/5] Setting up ArduPilot parameters..."
PARAMS_FILE="${SCRIPT_DIR}/depth_camera_params.parm"

cat > "$PARAMS_FILE" << 'PARAMS'
# ArduPilot parameters for depth camera obstacle avoidance
# Load with: sim_vehicle.py --add-param-file=sitl/depth_camera_params.parm

# Rangefinder (downward) — reads from depth camera via plugin
RNGFND1_TYPE 1
RNGFND1_PIN 0
RNGFND1_SCALING 10
RNGFND1_MIN_CM 20
RNGFND1_MAX_CM 1000

# Proximity sensor — for lateral obstacle detection
PRX1_TYPE 2
PRX_LOG_RAW 1

# Object avoidance
AVOID_ENABLE 7
AVOID_MARGIN 2

# EKF — trust rangefinder for altitude
EK2_RNG_USE_HGT 70
PARAMS

echo "  Params file: $PARAMS_FILE ✓"

echo ""
echo "============================================"
echo "  SETUP COMPLETE!"
echo "============================================"
echo ""
echo "  To launch:"
echo "    # Terminal 1 — SITL with depth camera world:"
echo "    sim_vehicle.py -v ArduCopter --no-mavproxy \\"
echo "      --add-param-file=$PARAMS_FILE"
echo ""
echo "    # Terminal 2 — Start Gazebo with obstacles:"
echo "    gazebo --verbose $WORLD_FILE"
echo ""
echo "    # Terminal 3 — Start web scanner:"
echo "    cd $PROJECT_DIR"
echo "    python3 sitl/scan_web.py --sitl"
echo ""
echo "  Open: http://localhost:5001"
echo "============================================"
