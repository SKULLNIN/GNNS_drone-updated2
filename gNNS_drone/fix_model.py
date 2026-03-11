import os

src = '/usr/share/gazebo-11/models/iris_with_ardupilot/model.sdf'
dst = '/root/.gazebo/models/iris_with_depth_camera/model.sdf'

with open(src, 'r') as f:
    text = f.read()

# Replace the outer model name
text = text.replace('name="iris_demo"', 'name="iris_with_depth_camera"')
text = text.replace("name='iris_demo'", "name='iris_with_depth_camera'")

# Replace the IMU scope name used by the plugin
text = text.replace('iris_demo::', 'iris_with_depth_camera::')

# Now add the depth camera and joint before the end of the model tag
depth_camera_xml = """
    <!-- ================================================ -->
    <!-- DEPTH CAMERA (RealSense D435 style, facing DOWN) -->
    <!-- ================================================ -->
    <link name="depth_camera_link">
      <pose>0 0 0.1 0 1.5708 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.000001</ixx><iyy>0.000001</iyy><izz>0.000001</izz>
        </inertia>
      </inertial>

      <visual name="cam_vis">
        <geometry><box><size>0.03 0.09 0.025</size></box></geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>

      <sensor name="depth_camera" type="depth">
        <update_rate>15</update_rate>
        <camera name="depth_cam">
          <horizontal_fov>1.3962634</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.2</near>
            <far>10.0</far>
          </clip>
        </camera>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <plugin name="depth_cam_plugin" filename="libgazebo_ros_depth_camera.so">
          <alwaysOn>true</alwaysOn>
          <updateRate>15</updateRate>
          <cameraName>gnns</cameraName>
          <imageTopicName>depth/image_raw</imageTopicName>
          <cameraInfoTopicName>depth/camera_info</cameraInfoTopicName>
          <depthImageTopicName>depth/image_depth_raw</depthImageTopicName>
          <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
          <pointCloudTopicName>depth/points</pointCloudTopicName>
          <frameName>depth_camera_link</frameName>
          <pointCloudCutoff>0.2</pointCloudCutoff>
          <pointCloudCutoffMax>10.0</pointCloudCutoffMax>
        </plugin>
      </sensor>
    </link>

    <!-- ================================================ -->
    <!-- 360 LIDAR (For Obstacle Avoidance) -->
    <!-- ================================================ -->
    <link name="lidar_link">
      <pose>0 0 0.12 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz></inertia>
      </inertial>
      <visual name="lidar_vis">
        <geometry><cylinder><radius>0.03</radius><length>0.04</length></cylinder></geometry>
        <material><ambient>0 0 0 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <collision name="lidar_col">
        <geometry><cylinder><radius>0.03</radius><length>0.04</length></cylinder></geometry>
      </collision>
      <sensor name="lidar" type="ray">
        <pose>0 0 0 0 0 0</pose>
        <visualize>true</visualize>
        <always_on>true</always_on>
        <update_rate>10</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>72</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159265</min_angle>
              <max_angle>3.14159265</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.15</min>
            <max>12.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="lidar_plugin" filename="libgazebo_ros_laser.so">
          <robotNamespace>/gnns</robotNamespace>
          <topicName>scan</topicName>
          <frameName>lidar_link</frameName>
          <gaussianNoise>0.005</gaussianNoise>
          <alwaysOn>true</alwaysOn>
          <updateRate>10</updateRate>
        </plugin>
      </sensor>
    </link>

    <joint name="lidar_joint" type="fixed">
      <parent>iris::base_link</parent>
      <child>lidar_link</child>
    </joint>

    <!-- Attach to the base_link of the sub-model 'iris' (which comes from model://iris_with_standoffs) -->
    <joint name="depth_camera_joint" type="fixed">
      <parent>iris::base_link</parent>
      <child>depth_camera_link</child>
    </joint>

  </model>
"""

# Replace the closing tag and add our block
text = text.replace('  </model>', depth_camera_xml)

with open(dst, 'w') as f:
    f.write(text)

print("SUCCESS: model.sdf updated with correct IMU scoping and depth camera!")
