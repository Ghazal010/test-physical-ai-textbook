---
title: Week 6 - Gazebo Simulation Basics
description: Introduction to physics simulation with Gazebo for robotics applications
sidebar_position: 1
---

# Week 6: Gazebo Simulation Basics

## Learning Objectives
- Understand physics simulation fundamentals (gravity, collisions, friction)
- Set up Gazebo environment and configure it properly
- Work with URDF and SDF robot description formats
- Create robot models from scratch for simulation
- Simulate sensors like LiDAR, depth cameras, IMUs
- Build a complete simulated robot environment

## Prerequisites Check
- Understanding of ROS 2 concepts
- Knowledge of URDF robot description format
- Basic understanding of coordinate systems and transformations

## Theoretical Concepts: Physics Simulation Fundamentals

### Introduction to Physics Simulation

Physics simulation in robotics is crucial for testing algorithms, validating designs, and training AI systems before deployment on real hardware. Gazebo provides a realistic physics engine that simulates:

- **Gravity**: The constant downward force affecting all objects
- **Collisions**: Detection and response when objects make contact
- **Friction**: Resistance that opposes relative motion between surfaces
- **Inertia**: Resistance of objects to changes in motion
- **Damping**: Energy dissipation that reduces oscillations over time

### Key Physics Concepts in Gazebo

**Rigid Body Dynamics**: In Gazebo, objects are treated as rigid bodies with properties like mass, center of mass, and inertia tensor. The physics engine calculates how forces affect these bodies over time.

**Collision Detection**: Gazebo uses geometric algorithms to detect when objects intersect or come into contact. Different collision algorithms offer trade-offs between accuracy and performance.

**Contact Physics**: When collisions are detected, Gazebo calculates the forces and torques that result from the contact, considering factors like friction and restitution (bounciness).

### Gazebo Architecture

Gazebo consists of several key components:

- **Physics Engine**: Handles the actual physics calculations (ODE, Bullet, SimBody)
- **Rendering Engine**: Provides visual representation (OpenGL, Ogre)
- **Sensor System**: Simulates various sensor types
- **Communication Layer**: Uses transport mechanisms for inter-process communication
- **Plugin System**: Allows extending functionality

## Step-by-Step Tutorials: Gazebo Setup and Configuration

### Tutorial 1: Installing and Setting Up Gazebo

First, let's ensure Gazebo is properly installed and configured:

```bash
# Install Gazebo Garden (or latest version)
sudo apt update
sudo apt install ros-humble-gazebo-*

# Install additional packages for ROS 2 integration
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control

# Verify installation
gz --version
```

### Tutorial 2: Creating Your First Gazebo World

Let's create a simple world file to understand the structure:

```xml
<!-- File: robot_control_package/worlds/simple_room.sdf -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_room">
    <!-- Include a default lighting setup -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Define a simple room with walls -->
    <model name="wall_1">
      <pose>0 5 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <self_collide>false</self_collide>
        <kinematic>false</kinematic>
        <gravity>true</gravity>
      </link>
    </model>

    <!-- Add a simple box object -->
    <model name="obstacle_box">
      <pose>2 0 0.5 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode>
                <max_vel>100</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Tutorial 3: Running Your First Gazebo Simulation

Launch the simulation:

```bash
# Launch Gazebo with your world
gz sim -r worlds/simple_room.sdf

# Or if using ROS 2 launch system
ros2 launch gazebo_ros gazebo.launch.py world:=path/to/your/world.sdf
```

### Tutorial 4: Integrating URDF with Gazebo

To use your URDF robot models in Gazebo, you need to add Gazebo-specific elements. Here's how to modify a URDF for Gazebo:

```xml
<!-- File: robot_control_package/urdf/gazebo_robot.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="gazebo_robot">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="10"/>
      <inertia ixx="0.416" ixy="0" ixz="0" iyy="1.0416" iyz="0" izz="1.25"/>
    </inertial>
  </link>

  <!-- Wheels -->
  <xacro:macro name="wheel" params="prefix parent xyz">
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>

      <collision>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </collision>

      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0025"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <xacro:wheel prefix="left" parent="base_link" xyz="0 0.15 0"/>
  <xacro:wheel prefix="right" parent="base_link" xyz="0 -0.15 0"/>

  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <!-- Gazebo plugin for differential drive -->
  <gazebo>
    <plugin filename="libgazebo_ros_diff_drive.so" name="diff_drive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_wheel_tf>true</publish_wheel_tf>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </gazebo>

  <!-- Add a camera sensor -->
  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.01"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo camera sensor -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.089</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <topic_name>camera/image_raw</topic_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### Tutorial 5: Launching Robot in Gazebo

Create a launch file to spawn your robot in Gazebo:

```python
# File: robot_control_package/launch/spawn_robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_xacro_file = LaunchConfiguration('robot_xacro_file', default='gazebo_robot.urdf.xacro')

    # Paths
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_robot_control = FindPackageShare('robot_control_package')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', PathJoinSubstitution([pkg_robot_control, 'urdf', robot_xacro_file])])
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'gazebo_robot',
            '-x', '0', '-y', '0', '-z', '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('robot_xacro_file', default_value='gazebo_robot.urdf.xacro', description='Robot URDF file'),
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

## Code Examples with Explanations

### Example 1: Gazebo Plugin for Custom Sensor

Create a custom sensor plugin for Gazebo:

```cpp
// File: robot_control_package/include/custom_sensor_plugin.hpp
#ifndef CUSTOM_SENSOR_PLUGIN_H
#define CUSTOM_SENSOR_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <boost/bind.hpp>

namespace gazebo
{
  class CustomSensorPlugin : public SensorPlugin
  {
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
      // Get the range sensor
      this->parentSensor =
        std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);

      if (!this->parentSensor)
      {
        gzerr << "CustomSensorPlugin requires a RaySensor.\n";
        return;
      }

      // Initialize ROS if not already initialized
      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "gazebo_custom_sensor",
                 ros::init_options::NoSigintHandler);
      }

      // Create ROS node
      this->rosNode.reset(new ros::NodeHandle("gazebo"));

      // Create publisher
      this->pub = this->rosNode->advertise<sensor_msgs::Range>(
          "/custom_range_sensor", 1);

      // Connect to sensor update event
      this->updateConnection = this->parentSensor->ConnectUpdated(
          boost::bind(&CustomSensorPlugin::OnUpdate, this));

      // Make sure sensor is active
      this->parentSensor->SetActive(true);
    }

    public: void OnUpdate()
    {
      // Get range from sensor
      double range = this->parentSensor->Range(0);

      // Create and publish ROS message
      sensor_msgs::Range msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "custom_sensor_frame";
      msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
      msg.field_of_view = 0.1;
      msg.min_range = 0.01;
      msg.max_range = 10.0;
      msg.range = range;

      this->pub.publish(msg);
    }

    private: sensors::RaySensorPtr parentSensor;
    private: ros::NodeHandlePtr rosNode;
    private: ros::Publisher pub;
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(CustomSensorPlugin)
}
#endif
```

### Example 2: Python Interface for Gazebo Control

Create a Python interface to control Gazebo simulation:

```python
# File: robot_control_package/robot_control_package/gazebo_interface.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32
from gazebo_msgs.srv import SetEntityState, GetEntityState
from gazebo_msgs.msg import ModelState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class GazeboInterface(Node):
    """
    Interface to control and monitor Gazebo simulation
    """

    def __init__(self):
        super().__init__('gazebo_interface')

        # Create service clients
        self.set_state_client = self.create_client(
            SetEntityState, '/world/set_entity_state'
        )
        self.get_state_client = self.create_client(
            GetEntityState, '/world/get_entity_state'
        )

        # Wait for services to be available
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set state service not available, waiting again...')

        while not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get state service not available, waiting again...')

        # Create publishers for simulation control
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sim_status_publisher = self.create_publisher(Float32, 'sim_status', 10)

        # Timer for periodic state updates
        self.state_timer = self.create_timer(0.1, self.update_simulation_state)

        # Internal state
        self.robot_pose = Pose()
        self.simulation_time = 0.0

        self.get_logger().info('Gazebo interface initialized')

    def update_simulation_state(self):
        """
        Update simulation state by getting robot position
        """
        # Get current robot state from Gazebo
        request = GetEntityState.Request()
        request.name = 'gazebo_robot'
        request.reference_frame = 'world'

        future = self.get_state_client.call_async(request)
        future.add_done_callback(self.get_state_callback)

    def get_state_callback(self, future):
        """
        Handle response from get state service
        """
        try:
            response = future.result()
            if response.success:
                self.robot_pose = response.state.pose
                self.get_logger().debug(
                    f'Robot position: ({self.robot_pose.position.x:.2f}, '
                    f'{self.robot_pose.position.y:.2f}, {self.robot_pose.position.z:.2f})'
                )

                # Publish simulation status (time)
                status_msg = Float32()
                status_msg.data = self.simulation_time
                self.sim_status_publisher.publish(status_msg)

                self.simulation_time += 0.1
            else:
                self.get_logger().error(f'Failed to get robot state: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def move_robot_to_position(self, target_x, target_y, target_z=0.0):
        """
        Move robot to a specific position in the simulation
        """
        request = SetEntityState.Request()
        request.state.name = 'gazebo_robot'
        request.state.pose.position.x = target_x
        request.state.pose.position.y = target_y
        request.state.pose.position.z = target_z
        request.state.pose.orientation.w = 1.0  # No rotation
        request.state.reference_frame = 'world'

        future = self.set_state_client.call_async(request)
        future.add_done_callback(self.move_callback)

    def move_callback(self, future):
        """
        Handle response from move service
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Robot moved successfully')
            else:
                self.get_logger().error(f'Move failed: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Move service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GazeboInterface()

    # Example: Move robot to a new position after 5 seconds
    import threading
    def delayed_move():
        import time
        time.sleep(5)
        node.move_robot_to_position(2.0, 2.0, 0.0)

    move_thread = threading.Thread(target=delayed_move, daemon=True)
    move_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3: Advanced World with Multiple Models

Create a more complex world with multiple robots and objects:

```xml
<!-- File: robot_control_package/worlds/complex_world.sdf -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="complex_world">
    <!-- Physics engine configuration -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Custom environment -->
    <model name="table">
      <pose>3 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.5 0.8 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>50</mass>
          <inertia>
            <ixx>6.833</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>12.583</iyy>
            <iyz>0</iyz>
            <izz>18.416</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add some objects on the table -->
    <model name="object_1">
      <pose>3.3 0.2 1.25 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.0056</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0056</iyy>
            <iyz>0</iyz>
            <izz>0.0025</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="object_2">
      <pose>2.7 -0.2 1.25 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.3</mass>
          <inertia>
            <ixx>0.0006</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0006</iyy>
            <iyz>0</iyz>
            <izz>0.0006</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add a wall -->
    <model name="wall">
      <pose>0 -4 1 0 0 1.57</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>8 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>8 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Robot will be spawned separately -->
  </world>
</sdf>
```

## Hands-On Exercises: Robot Model Creation

### Exercise 1: Create a Custom Robot for Simulation

Create a differential drive robot with sensors for Gazebo simulation:

```xml
<!-- File: robot_control_package/urdf/custom_gazebo_robot.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="custom_gazebo_robot">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.04" />
  <xacro:property name="base_length" value="0.4" />
  <xacro:property name="base_width" value="0.3" />
  <xacro:property name="base_height" value="0.15" />

  <!-- Macro for wheel -->
  <xacro:macro name="gazebo_wheel" params="prefix parent xyz">
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>

      <collision>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>

      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0025"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${base_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="silver">
        <color rgba="0.75 0.75 0.75 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 ${base_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 ${base_height/2}" rpy="0 0 0"/>
      <mass value="10"/>
      <inertia ixx="0.416" ixy="0" ixz="0" iyy="1.0416" iyz="0" izz="1.25"/>
    </inertial>
  </link>

  <!-- Wheels -->
  <xacro:gazebo_wheel prefix="left" parent="base_link" xyz="0 ${base_width/2-0.02} 0"/>
  <xacro:gazebo_wheel prefix="right" parent="base_link" xyz="0 -${base_width/2-0.02} 0"/>

  <!-- Casters -->
  <link name="front_caster">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/>
    </inertial>
  </link>

  <joint name="front_caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="front_caster"/>
    <origin xyz="${base_length/2-0.03} 0 ${wheel_radius-0.03}" rpy="0 0 0"/>
  </joint>

  <!-- Sensors -->
  <link name="laser_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.04"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="2e-5"/>
    </inertial>
  </link>

  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Silver</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>

  <gazebo reference="front_caster">
    <material>Gazebo/Black</material>
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
  </gazebo>

  <gazebo reference="laser_link">
    <material>Gazebo/Red</material>
  </gazebo>

  <!-- Differential drive plugin -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.26</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_wheel_tf>true</publish_wheel_tf>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </gazebo>

  <!-- Laser plugin -->
  <gazebo reference="laser_link">
    <sensor name="laser" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="laser_controller" filename="libgazebo_ros_laser.so">
        <topic_name>scan</topic_name>
        <frame_name>laser_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### Exercise 2: Simulation Environment with Dynamic Objects

Create a simulation environment with moving objects:

```xml
<!-- File: robot_control_package/worlds/dynamic_world.sdf -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="dynamic_world">
    <!-- Physics -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Static obstacles -->
    <model name="static_obstacle_1">
      <pose>-2 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Moving object with joint -->
    <model name="moving_object">
      <pose>0 3 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>5.0</mass>
          <inertia>
            <ixx>0.5416</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.5416</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>

      <!-- Prismatic joint to move the object back and forth -->
      <joint name="slider" type="prismatic">
        <parent>world</parent>
        <child>link</child>
        <axis>
          <xyz>1 0 0</xyz>
          <limit>
            <lower>-3</lower>
            <upper>3</upper>
          </limit>
        </axis>
      </joint>
    </model>

    <!-- Add a simple controller for the moving object -->
    <gazebo>
      <plugin name="slider_controller" filename="libgazebo_ros_p3d.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>100</updateRate>
        <bodyName>moving_object::link</bodyName>
        <topicName>slider_position</topicName>
        <gaussianNoise>0.0</gaussianNoise>
        <frameName>world</frameName>
      </plugin>
    </gazebo>

    <!-- A simple maze -->
    <model name="maze_wall_1">
      <pose>0 2 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>4 0.1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>4 0.1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

## Common Pitfalls and Solutions

### Pitfall 1: Physics Instability
**Problem**: Objects behave unrealistically, jitter, or explode in simulation.

**Solutions**:
- Adjust physics parameters (step size, solver iterations)
- Ensure proper mass and inertia values
- Use appropriate friction and damping coefficients
- Check for intersecting collision geometries

```xml
<!-- Proper physics configuration -->
<physics name="default" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Pitfall 2: Performance Issues
**Problem**: Simulation runs slowly or becomes unresponsive.

**Solutions**:
- Optimize collision meshes (use simpler shapes)
- Reduce physics update rate if possible
- Limit the number of complex sensors
- Use appropriate visual vs collision geometry

### Pitfall 3: URDF/SDF Conversion Issues
**Problem**: Robot doesn't appear correctly in Gazebo or has incorrect physics properties.

**Solutions**:
- Ensure all links have proper inertial properties
- Check that Gazebo plugins are correctly specified
- Verify joint limits and types match physical constraints

## Review Questions

1. What are the key differences between URDF and SDF formats?
2. Explain how physics simulation works in Gazebo.
3. What are the important parameters to consider when defining collision properties?
4. How do you add sensors to a robot model for simulation?
5. What are the best practices for optimizing simulation performance?

## Project Assignment: Complete Simulation Environment

Create a complete simulation environment that includes:
1. A differential drive robot with multiple sensors (LIDAR, camera, IMU)
2. A custom world with static and dynamic obstacles
3. Proper physics properties for all objects
4. ROS 2 integration for controlling the robot
5. Sensor data publishing and visualization

Your environment should:
- Be stable and physically realistic
- Include at least 3 different sensor types
- Have appropriate collision and visual properties
- Be optimized for real-time simulation
- Include a launch file to start the complete simulation

## Further Resources

- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [ROS 2 with Gazebo](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [URDF to SDF Conversion](http://gazebosim.org/tutorials?tut=ros_urdf)
- [Physics Engine Comparison](http://gazebosim.org/tutorials?tut=physics)
- [Sensor Simulation](http://gazebosim.org/tutorials?tut=multiple_sensors)

:::info
Remember that simulation is only as good as your models. Take time to properly tune physics parameters and validate your simulation against real-world behavior when possible.
:::