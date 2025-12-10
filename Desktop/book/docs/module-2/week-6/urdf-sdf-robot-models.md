---
title: Week 6 - URDF and SDF Robot Models
description: Working with URDF and SDF formats for robot description in simulation
sidebar_position: 2
---

# Week 6: URDF and SDF Robot Models

## Learning Objectives
- Understand the structure and components of URDF robot descriptions
- Create and modify URDF models for robotic systems
- Work with SDF (Simulation Description Format) for Gazebo simulation
- Implement multi-joint robot models with proper kinematics
- Validate and debug robot models in simulation environments

## Prerequisites Check
- Completion of Week 3-5 (ROS 2 Fundamentals)
- Understanding of ROS 2 packages and file structures
- Basic knowledge of 3D coordinate systems and transformations
- Experience with XML file editing

## Theoretical Concepts: Robot Description Formats

### URDF (Unified Robot Description Format)

URDF is an XML-based format used in ROS to describe robots. It defines the physical and visual properties of a robot, including:

- **Links**: Rigid parts of the robot (e.g., chassis, wheels, arms)
- **Joints**: Connections between links that define kinematic relationships
- **Visual**: How the robot appears in visualization tools
- **Collision**: Collision properties for physics simulation
- **Inertial**: Mass, center of mass, and inertia properties
- **Materials**: Visual appearance properties

### SDF (Simulation Description Format)

SDF is Gazebo's native format for describing simulation environments, robots, and objects. While URDF is ROS-specific, SDF is more general-purpose and can describe:

- Complete simulation worlds with physics properties
- Robot models with sensor configurations
- Environment objects and lighting
- Plugin configurations for simulation behavior

### Key Differences Between URDF and SDF

| Aspect | URDF | SDF |
|--------|------|-----|
| Purpose | Robot description | Simulation environment description |
| Scope | Individual robot | Complete simulation world |
| Integration | ROS-centric | Gazebo-centric |
| Complexity | Focused on robot structure | Broader simulation features |

## Step-by-Step Tutorials: Creating Robot Models

### Basic URDF Robot Model

Let's start with a simple differential drive robot model:

```xml
<?xml version="1.0"?>
<robot name="simple_diff_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
      <origin rpy="1.57075 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.57075 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black"/>
      <origin rpy="1.57075 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.57075 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Base to left wheel joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.0 0.2 0.0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Base to right wheel joint -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0.0 -0.2 0.0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Optional: Add caster wheel for stability -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.2 0.0 -0.05"/>
  </joint>
</robot>
```

### Advanced URDF with Xacro Macros

Xacro is a macro language for XML that makes URDF more maintainable:

```xml
<?xml version="1.0"?>
<robot name="advanced_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.05" />
  <xacro:property name="base_length" value="0.5" />
  <xacro:property name="base_width" value="0.3" />
  <xacro:property name="base_height" value="0.15" />

  <!-- Material definitions -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Macro for creating wheels -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black"/>
        <origin rpy="${rpy}"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <origin rpy="${rpy}"/>
      </collision>
      <inertial>
        <mass value="0.2"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Macro for creating sensors -->
  <xacro:macro name="lidar_sensor" params="name parent xyz rpy">
    <joint name="${name}_joint" type="fixed">
      <parent link="${parent}"/>
      <child link="${name}_link"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
    </joint>

    <link name="${name}_link">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="0.05"/>
        </geometry>
        <material name="black"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.05" length="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
      </inertial>
    </link>

    <!-- Gazebo plugin for LIDAR simulation -->
    <gazebo reference="${name}_link">
      <sensor type="ray" name="${name}_sensor">
        <pose>0 0 0 0 0 0</pose>
        <visualize>false</visualize>
        <update_rate>10</update_rate>
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
            <max>30.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <plugin name="gazebo_ros_${name}_controller" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <namespace>/</namespace>
            <remapping>~/out:=${name}/scan</remapping>
          </ros>
          <output_type>sensor_msgs/LaserScan</output_type>
        </plugin>
      </sensor>
    </gazebo>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Create wheels using macro -->
  <xacro:wheel prefix="left" parent="base_link" xyz="0.0 ${base_width/2 + wheel_width/2} 0.0" rpy="${M_PI/2} 0 0"/>
  <xacro:wheel prefix="right" parent="base_link" xyz="0.0 -${base_width/2 + wheel_width/2} 0.0" rpy="${M_PI/2} 0 0"/>

  <!-- Add caster wheel -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-${base_length/2} 0.0 -${base_height/2 + 0.05}"/>
  </joint>

  <!-- Add LIDAR sensor -->
  <xacro:lidar_sensor name="laser" parent="base_link" xyz="${base_length/2 - 0.05} 0.0 ${base_height/2}" rpy="0 0 0"/>

  <!-- Gazebo-specific configurations -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <update_rate>30</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>${base_width + wheel_width}</wheel_separation>
      <wheel_diameter>${2 * wheel_radius}</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>
</robot>
```

### Converting URDF to SDF for Gazebo

```bash
# Convert URDF to SDF using gazebo's converter
gz sdf -p robot.urdf > robot.sdf

# Or use the older command if using older Gazebo
gzsdf print robot.urdf > robot.sdf
```

### Basic SDF World File

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Physics engine -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Your robot model -->
    <include>
      <uri>model://simple_diff_drive_robot</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>

    <!-- Additional objects -->
    <model name="table">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="table_link">
        <pose>0 0 0.5 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.4 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- A simple obstacle -->
    <model name="obstacle">
      <pose>-1 0 0.2 0 0 0</pose>
      <link name="obstacle_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.3 0.3 0.4</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.3 0.3 0.4</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.8 1</ambient>
            <diffuse>0.4 0.4 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.01</iyy>
            <iyz>0.0</iyz>
            <izz>0.01</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Code Examples with Explanations

### URDF Validation and Debugging

```python
import xml.etree.ElementTree as ET
import subprocess
import os
from typing import Dict, List, Tuple

class URDFValidator:
    """
    Tool for validating and debugging URDF models
    """

    def __init__(self, urdf_path: str):
        self.urdf_path = urdf_path
        self.tree = ET.parse(urdf_path)
        self.root = self.tree.getroot()

    def validate_structure(self) -> Dict[str, List[str]]:
        """
        Validate basic URDF structure and common issues
        """
        errors = []
        warnings = []

        # Check if robot name is defined
        robot_name = self.root.get('name')
        if not robot_name:
            errors.append("Robot name is not defined")

        # Check for at least one link
        links = self.root.findall('.//link')
        if not links:
            errors.append("No links found in URDF")

        # Check for at least one joint (if more than one link)
        joints = self.root.findall('.//joint')
        if len(links) > 1 and not joints:
            errors.append("Multiple links found but no joints to connect them")

        # Check for proper joint connections
        link_names = [link.get('name') for link in links]
        for joint in joints:
            parent = joint.find('parent')
            child = joint.find('child')

            if parent is None or child is None:
                errors.append(f"Joint {joint.get('name')} missing parent or child")
            else:
                parent_name = parent.get('link')
                child_name = child.get('link')

                if parent_name not in link_names:
                    errors.append(f"Joint {joint.get('name')} references non-existent parent link: {parent_name}")
                if child_name not in link_names:
                    errors.append(f"Joint {joint.get('name')} references non-existent child link: {child_name}")
                if parent_name == child_name:
                    errors.append(f"Joint {joint.get('name')} connects link to itself")

        # Check for floating-point values in critical parameters
        self._check_inertial_values(errors, warnings)

        return {
            'errors': errors,
            'warnings': warnings
        }

    def _check_inertial_values(self, errors: List[str], warnings: List[str]):
        """
        Check inertial values for common issues
        """
        for link in self.root.findall('.//link'):
            inertial = link.find('inertial')
            if inertial is not None:
                mass = inertial.find('mass')
                if mass is not None:
                    try:
                        mass_value = float(mass.get('value'))
                        if mass_value <= 0:
                            errors.append(f"Link {link.get('name')} has non-positive mass: {mass_value}")
                    except ValueError:
                        errors.append(f"Link {link.get('name')} has invalid mass value")

                inertia = inertial.find('inertia')
                if inertia is not None:
                    # Check diagonal elements of inertia matrix
                    for param in ['ixx', 'iyy', 'izz']:
                        try:
                            value = float(inertia.get(param))
                            if value <= 0:
                                errors.append(f"Link {link.get('name')} has non-positive {param}: {value}")
                        except ValueError:
                            errors.append(f"Link {link.get('name')} has invalid {param} value")

    def check_kinematic_chain(self) -> Dict[str, any]:
        """
        Check if the kinematic chain is properly formed
        """
        joints = self.root.findall('.//joint')
        links = self.root.findall('.//link')

        # Build parent-child relationships
        parent_map = {}
        child_map = {}

        for joint in joints:
            parent_elem = joint.find('parent')
            child_elem = joint.find('child')

            if parent_elem is not None and child_elem is not None:
                parent_name = parent_elem.get('link')
                child_name = child_elem.get('link')

                parent_map[child_name] = parent_name
                if parent_name not in child_map:
                    child_map[parent_name] = []
                child_map[parent_name].append(child_name)

        # Find base link (has no parent)
        all_link_names = {link.get('name') for link in links}
        child_names = set(parent_map.keys())
        base_links = all_link_names - child_names

        if len(base_links) == 0:
            return {
                'valid': False,
                'error': 'No base link found (all links have parents)'
            }
        elif len(base_links) > 1:
            return {
                'valid': False,
                'error': f'Multiple base links found: {base_links}'
            }

        return {
            'valid': True,
            'base_link': base_links.pop(),
            'total_links': len(links),
            'total_joints': len(joints)
        }

    def visualize_with_check(self) -> bool:
        """
        Try to visualize the URDF using check_urdf command
        """
        try:
            result = subprocess.run(
                ['check_urdf', self.urdf_path],
                capture_output=True,
                text=True
            )
            return result.returncode == 0
        except FileNotFoundError:
            print("check_urdf command not found. Please install ROS urdf package.")
            return False
        except Exception as e:
            print(f"Error running check_urdf: {e}")
            return False

    def generate_urdf_report(self) -> str:
        """
        Generate a comprehensive report about the URDF
        """
        validation_result = self.validate_structure()
        kinematic_result = self.check_kinematic_chain()

        report = f"URDF Report for: {self.urdf_path}\n"
        report += "=" * 50 + "\n\n"

        report += f"Robot Name: {self.root.get('name', 'Unknown')}\n"
        report += f"Total Links: {len(self.root.findall('.//link'))}\n"
        report += f"Total Joints: {len(self.root.findall('.//joint'))}\n"
        report += f"Base Link: {kinematic_result.get('base_link', 'Unknown')}\n\n"

        report += "Validation Results:\n"
        report += f"  Errors: {len(validation_result['errors'])}\n"
        report += f"  Warnings: {len(validation_result['warnings'])}\n\n"

        if validation_result['errors']:
            report += "Errors Found:\n"
            for error in validation_result['errors']:
                report += f"  - {error}\n"

        if validation_result['warnings']:
            report += "Warnings Found:\n"
            for warning in validation_result['warnings']:
                report += f"  - {warning}\n"

        if kinematic_result['valid']:
            report += f"\nKinematic Chain: Valid\n"
            report += f"Base Link: {kinematic_result['base_link']}\n"
        else:
            report += f"\nKinematic Chain: Invalid\n"
            report += f"Error: {kinematic_result['error']}\n"

        return report

# Example usage
def validate_urdf_example():
    """
    Example of using URDF validator
    """
    # Create a simple URDF file for testing
    test_urdf_content = '''<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
  </link>

  <link name="wheel">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <origin xyz="0 0.2 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>'''

    # Write test URDF to file
    with open('/tmp/test_robot.urdf', 'w') as f:
        f.write(test_urdf_content)

    # Validate the URDF
    validator = URDFValidator('/tmp/test_robot.urdf')
    report = validator.generate_urdf_report()
    print(report)

    # Clean up
    os.remove('/tmp/test_robot.urdf')
```

### Robot State Publisher Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

class RobotStatePublisher(Node):
    """
    Robot state publisher for publishing joint states and transforms
    """

    def __init__(self):
        super().__init__('robot_state_publisher')

        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', 10
        )

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for publishing state
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Example joint names and positions
        self.joint_names = ['left_wheel_joint', 'right_wheel_joint']
        self.joint_positions = [0.0, 0.0]
        self.joint_velocities = [0.0, 0.0]
        self.joint_efforts = [0.0, 0.0]

        self.get_logger().info('Robot State Publisher started')

    def publish_joint_states(self):
        """
        Publish joint states message
        """
        # Create joint state message
        msg = JointState()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Publish joint states
        self.joint_state_publisher.publish(msg)

        # Publish transforms
        self.publish_transforms()

    def publish_transforms(self):
        """
        Publish transforms for robot links
        """
        # Example: Publish transform for wheel links
        # In a real implementation, you'd calculate these based on forward kinematics

        # Left wheel transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'left_wheel'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.2  # offset to the right
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

        # Right wheel transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'right_wheel'

        t.transform.translation.x = 0.0
        t.transform.translation.y = -0.2  # offset to the left
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

class URDFRobotController:
    """
    Controller for URDF-based robot with simulation integration
    """

    def __init__(self, robot_description_param: str = 'robot_description'):
        # Initialize ROS node for controller
        self.node = rclpy.create_node('urdf_robot_controller')

        # Get robot description from parameter
        self.node.declare_parameter(robot_description_param, '')
        self.robot_description = self.node.get_parameter(robot_description_param).value

        # Publishers for joint commands
        self.joint_command_publishers = {}

        # Robot configuration based on URDF
        self.robot_config = self._parse_robot_config()

    def _parse_robot_config(self):
        """
        Parse robot configuration from URDF (simplified)
        """
        config = {
            'joints': {},
            'links': {},
            'materials': {}
        }

        # In practice, you'd parse the URDF XML to extract joint limits, etc.
        # This is a simplified version
        config['joints'] = {
            'left_wheel_joint': {
                'type': 'continuous',
                'limits': {'min': -float('inf'), 'max': float('inf')}
            },
            'right_wheel_joint': {
                'type': 'continuous',
                'limits': {'min': -float('inf'), 'max': float('inf')}
            }
        }

        return config

    def send_velocity_commands(self, left_vel: float, right_vel: float):
        """
        Send velocity commands to robot wheels
        """
        # In a real implementation, you'd publish to appropriate topics
        # based on your robot's controller configuration
        pass

    def calculate_forward_kinematics(self, joint_angles: dict):
        """
        Calculate forward kinematics based on joint angles
        """
        # Simplified forward kinematics for differential drive
        # In practice, you'd implement the full kinematic model
        pass

    def calculate_inverse_kinematics(self, target_pose: dict):
        """
        Calculate inverse kinematics to reach target pose
        """
        # Simplified inverse kinematics
        # In practice, you'd implement the full inverse kinematic solution
        pass

def main(args=None):
    rclpy.init(args=args)

    # Create robot state publisher
    state_publisher = RobotStatePublisher()

    try:
        rclpy.spin(state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-On Exercises: Robot Model Creation

### Exercise 1: Create a Simple Arm Robot

Create a URDF model for a simple 3-DOF robotic arm:

```xml
<?xml version="1.0"?>
<robot name="simple_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Constants -->
  <xacro:property name="M_PI" value="3.14159265359"/>
  <xacro:property name="arm_base_radius" value="0.1"/>
  <xacro:property name="arm_base_height" value="0.2"/>
  <xacro:property name="link1_length" value="0.3"/>
  <xacro:property name="link2_length" value="0.3"/>
  <xacro:property name="link3_length" value="0.2"/>
  <xacro:property name="link_radius" value="0.05"/>

  <!-- Materials -->
  <material name="red">
    <color rgba="0.8 0.2 0.2 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.2 0.8 0.2 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.2 0.2 0.8 1.0"/>
  </material>
  <material name="gray">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="${arm_base_radius}" length="${arm_base_height}"/>
      </geometry>
      <material name="gray"/>
      <origin xyz="0 0 ${arm_base_height/2}"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${arm_base_radius}" length="${arm_base_height}"/>
      </geometry>
      <origin xyz="0 0 ${arm_base_height/2}"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Joint 1: Base rotation -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 ${arm_base_height}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI}" upper="${M_PI}" effort="100" velocity="3.0"/>
  </joint>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="${link_radius}" length="${link1_length}"/>
      </geometry>
      <material name="red"/>
      <origin xyz="0 0 ${link1_length/2}" rpy="1.5708 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${link_radius}" length="${link1_length}"/>
      </geometry>
      <origin xyz="0 0 ${link1_length/2}" rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Joint 2: Shoulder joint -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 ${link1_length} 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="3.0"/>
  </joint>

  <!-- Link 2 -->
  <link name="link2">
    <visual>
      <geometry>
        <cylinder radius="${link_radius}" length="${link2_length}"/>
      </geometry>
      <material name="green"/>
      <origin xyz="0 0 ${link2_length/2}" rpy="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${link_radius}" length="${link2_length}"/>
      </geometry>
      <origin xyz="0 0 ${link2_length/2}" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Joint 3: Elbow joint -->
  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0 ${link2_length} 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="3.0"/>
  </joint>

  <!-- Link 3 (end effector) -->
  <link name="link3">
    <visual>
      <geometry>
        <cylinder radius="${link_radius}" length="${link3_length}"/>
      </geometry>
      <material name="blue"/>
      <origin xyz="0 0 ${link3_length/2}" rpy="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${link_radius}" length="${link3_length}"/>
      </geometry>
      <origin xyz="0 0 ${link3_length/2}" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Add a simple gripper (fixed) -->
  <joint name="gripper_joint" type="fixed">
    <parent link="link3"/>
    <child link="gripper"/>
    <origin xyz="0 ${link3_length} 0" rpy="0 0 0"/>
  </joint>

  <link name="gripper">
    <visual>
      <geometry>
        <box size="0.05 0.02 0.05"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.02 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/simple_arm</robotNamespace>
    </plugin>
  </gazebo>

  <!-- Transmission for ROS Control -->
  <transmission name="tran1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint1">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor1">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="tran2">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint2">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor2">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="tran3">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint3">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor3">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
</robot>
```

### Exercise 2: URDF Validation and Testing Script

Create a Python script to validate and test URDF models:

```python
import xml.etree.ElementTree as ET
import subprocess
import sys
import argparse
from pathlib import Path
from typing import Dict, List, Tuple

class URDFAnalyzer:
    """
    Comprehensive URDF analyzer and validator
    """

    def __init__(self, urdf_file: str):
        self.urdf_file = Path(urdf_file)
        if not self.urdf_file.exists():
            raise FileNotFoundError(f"URDF file not found: {urdf_file}")

        self.tree = ET.parse(self.urdf_file)
        self.root = self.tree.getroot()

    def analyze_robot(self) -> Dict:
        """
        Perform comprehensive analysis of the robot model
        """
        analysis = {
            'basic_info': self._get_basic_info(),
            'links': self._analyze_links(),
            'joints': self._analyze_joints(),
            'materials': self._analyze_materials(),
            'sensors': self._analyze_sensors(),
            'gazebo_elements': self._analyze_gazebo_elements(),
            'issues': self._find_common_issues()
        }

        return analysis

    def _get_basic_info(self) -> Dict:
        """
        Extract basic robot information
        """
        return {
            'name': self.root.get('name', 'unnamed'),
            'has_xacro': 'xacro' in self.root.get('xmlns:xacro', ''),
            'total_elements': len(list(self.root.iter()))
        }

    def _analyze_links(self) -> Dict:
        """
        Analyze all links in the robot
        """
        links = {}
        link_elements = self.root.findall('.//link')

        for link_elem in link_elements:
            link_name = link_elem.get('name')
            link_info = {
                'visual_count': len(link_elem.findall('visual')),
                'collision_count': len(link_elem.findall('collision')),
                'inertial_present': link_elem.find('inertial') is not None,
                'geometry_types': []
            }

            # Check geometry types
            for geom_elem in link_elem.findall('.//geometry/*'):
                link_info['geometry_types'].append(geom_elem.tag)

            links[link_name] = link_info

        return {
            'count': len(links),
            'details': links
        }

    def _analyze_joints(self) -> Dict:
        """
        Analyze all joints in the robot
        """
        joints = {}
        joint_elements = self.root.findall('.//joint')

        joint_types = {}
        for joint_elem in joint_elements:
            joint_name = joint_elem.get('name')
            joint_type = joint_elem.get('type')
            joint_info = {
                'type': joint_type,
                'has_parent': joint_elem.find('parent') is not None,
                'has_child': joint_elem.find('child') is not None,
                'has_axis': joint_elem.find('axis') is not None
            }

            joints[joint_name] = joint_info

            # Count joint types
            if joint_type not in joint_types:
                joint_types[joint_type] = 0
            joint_types[joint_type] += 1

        return {
            'count': len(joints),
            'types': joint_types,
            'details': joints
        }

    def _analyze_materials(self) -> Dict:
        """
        Analyze materials in the robot
        """
        materials = {}
        material_elements = self.root.findall('.//material')

        for mat_elem in material_elements:
            mat_name = mat_elem.get('name')
            color_elem = mat_elem.find('color')
            if color_elem is not None:
                materials[mat_name] = {
                    'color': color_elem.get('rgba')
                }

        return {
            'count': len(materials),
            'details': materials
        }

    def _analyze_sensors(self) -> Dict:
        """
        Analyze sensors in the robot (gazebo plugins)
        """
        sensors = []

        # Look for sensor plugins in gazebo elements
        gazebo_elements = self.root.findall('.//gazebo')
        for gazebo_elem in gazebo_elements:
            sensor_elem = gazebo_elem.find('.//sensor')
            if sensor_elem is not None:
                sensor_info = {
                    'name': sensor_elem.get('name'),
                    'type': sensor_elem.get('type'),
                    'parent_link': gazebo_elem.get('reference')
                }
                sensors.append(sensor_info)

        return {
            'count': len(sensors),
            'details': sensors
        }

    def _analyze_gazebo_elements(self) -> Dict:
        """
        Analyze Gazebo-specific elements
        """
        gazebo_elements = self.root.findall('.//gazebo')

        plugins = []
        for gazebo_elem in gazebo_elements:
            plugin_elem = gazebo_elem.find('.//plugin')
            if plugin_elem is not None:
                plugin_info = {
                    'name': plugin_elem.get('name'),
                    'filename': plugin_elem.get('filename')
                }
                plugins.append(plugin_info)

        return {
            'count': len(gazebo_elements),
            'plugins_count': len(plugins),
            'plugins': plugins
        }

    def _find_common_issues(self) -> List[str]:
        """
        Find common URDF issues
        """
        issues = []

        # Check for duplicate names
        link_names = [l.get('name') for l in self.root.findall('.//link')]
        joint_names = [j.get('name') for j in self.root.findall('.//joint')]

        all_names = link_names + joint_names
        if len(all_names) != len(set(all_names)):
            issues.append("Duplicate names found in links/joints")

        # Check for joints with same parent and child
        for joint in self.root.findall('.//joint'):
            parent = joint.find('parent')
            child = joint.find('child')
            if parent is not None and child is not None:
                if parent.get('link') == child.get('link'):
                    issues.append(f"Joint '{joint.get('name')}' connects a link to itself")

        # Check for links without geometry
        for link in self.root.findall('.//link'):
            name = link.get('name')
            has_visual = len(link.findall('visual')) > 0
            has_collision = len(link.findall('collision')) > 0
            if not has_visual and not has_collision:
                issues.append(f"Link '{name}' has no visual or collision geometry")

        return issues

    def run_external_validation(self) -> Dict:
        """
        Run external URDF validation tools
        """
        results = {}

        # Try check_urdf
        try:
            result = subprocess.run(
                ['check_urdf', str(self.urdf_file)],
                capture_output=True,
                text=True,
                timeout=10
            )
            results['check_urdf'] = {
                'success': result.returncode == 0,
                'output': result.stdout,
                'error': result.stderr
            }
        except subprocess.TimeoutExpired:
            results['check_urdf'] = {
                'success': False,
                'error': 'Validation timed out'
            }
        except FileNotFoundError:
            results['check_urdf'] = {
                'success': False,
                'error': 'check_urdf command not found (install ros-<distro>-urdfdom-tools)'
            }
        except Exception as e:
            results['check_urdf'] = {
                'success': False,
                'error': str(e)
            }

        return results

    def generate_report(self) -> str:
        """
        Generate a comprehensive analysis report
        """
        analysis = self.analyze_robot()
        external_results = self.run_external_validation()

        report = f"URDF Analysis Report: {self.urdf_file.name}\n"
        report += "=" * 60 + "\n\n"

        # Basic info
        report += f"Robot Name: {analysis['basic_info']['name']}\n"
        report += f"Has Xacro: {analysis['basic_info']['has_xacro']}\n"
        report += f"Total XML Elements: {analysis['basic_info']['total_elements']}\n\n"

        # Links
        report += f"Links: {analysis['links']['count']}\n"
        for name, details in list(analysis['links']['details'].items())[:5]:  # Show first 5
            report += f"  - {name}: {details['visual_count']} visual, {details['collision_count']} collision\n"
        if len(analysis['links']['details']) > 5:
            report += f"  ... and {len(analysis['links']['details']) - 5} more\n"
        report += "\n"

        # Joints
        report += f"Joints: {analysis['joints']['count']}\n"
        for jtype, count in analysis['joints']['types'].items():
            report += f"  - {jtype}: {count}\n"
        report += "\n"

        # Materials
        report += f"Materials: {analysis['materials']['count']}\n\n"

        # Sensors
        report += f"Sensors: {analysis['sensors']['count']}\n"
        for sensor in analysis['sensors']['details']:
            report += f"  - {sensor['name']} ({sensor['type']}) on {sensor['parent_link']}\n"
        report += "\n"

        # Gazebo elements
        report += f"Gazebo Elements: {analysis['gazebo_elements']['count']}\n"
        report += f"Gazebo Plugins: {analysis['gazebo_elements']['plugins_count']}\n"
        for plugin in analysis['gazebo_elements']['plugins']:
            report += f"  - {plugin['name']}: {plugin['filename']}\n"
        report += "\n"

        # Issues
        if analysis['issues']:
            report += f"Issues Found: {len(analysis['issues'])}\n"
            for issue in analysis['issues']:
                report += f"  - {issue}\n"
        else:
            report += "Issues Found: None\n"
        report += "\n"

        # External validation
        if 'check_urdf' in external_results:
            check_result = external_results['check_urdf']
            report += "External Validation (check_urdf):\n"
            report += f"  Success: {check_result['success']}\n"
            if check_result['error']:
                report += f"  Error: {check_result['error'][:200]}...\n"  # Truncate long errors
            report += "\n"

        return report

def main():
    parser = argparse.ArgumentParser(description='URDF Robot Model Analyzer')
    parser.add_argument('urdf_file', help='Path to URDF file to analyze')
    parser.add_argument('--output', '-o', help='Output file for report')
    parser.add_argument('--validate', action='store_true', help='Run external validation')

    args = parser.parse_args()

    try:
        analyzer = URDFAnalyzer(args.urdf_file)
        report = analyzer.generate_report()

        if args.output:
            with open(args.output, 'w') as f:
                f.write(report)
            print(f"Report saved to {args.output}")
        else:
            print(report)

    except Exception as e:
        print(f"Error analyzing URDF: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
```

## Common Pitfalls and Solutions

### Pitfall 1: Incorrect Inertial Parameters
**Problem**: Robots behave unrealistically in simulation due to incorrect mass or inertia values.

**Solution**:
- Calculate inertial properties based on actual geometry
- Use CAD software to compute accurate inertial values
- Follow the parallel axis theorem for complex shapes

```xml
<!-- Correct way to define inertial properties -->
<link name="arm_link">
  <inertial>
    <!-- Mass should reflect actual weight -->
    <mass value="0.5"/>
    <!-- Inertia values should be calculated for the specific geometry -->
    <!-- For a cylinder: ixx = iyy = m*(3*r² + h²)/12, izz = m*r²/2 -->
    <inertia ixx="0.001" ixy="0.0" ixz="0.0"
             iyy="0.001" iyz="0.0"
             izz="0.0005"/>
  </inertial>
</link>
```

### Pitfall 2: Poor Joint Limit Definitions
**Problem**: Joints allow impossible or damaging configurations.

**Solution**:
- Define realistic joint limits based on physical constraints
- Consider safety margins in limit definitions
- Use appropriate joint types (revolute vs continuous)

```xml
<!-- Properly constrained joint -->
<joint name="shoulder_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <!-- Realistic limits with safety margin -->
  <limit lower="${-M_PI/2 + 0.1}" upper="${M_PI/2 - 0.1}"
         effort="50" velocity="2.0"/>
</joint>
```

### Pitfall 3: Missing or Incorrect Transmissions
**Problem**: Robot cannot be controlled through ROS when used with ros_control.

**Solution**:
- Always include transmission elements for controllable joints
- Use correct hardware interfaces
- Match transmission names with controller configurations

```xml
<!-- Proper transmission definition -->
<transmission name="arm_joint1_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint1">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="joint1_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Pitfall 4: Inconsistent Units
**Problem**: Mixing different unit systems causes unexpected behavior.

**Solution**:
- Always use consistent units (typically meters for distance, radians for angles)
- Document the unit system being used
- Be careful when importing models from CAD software

## Review Questions

1. What are the main differences between URDF and SDF formats?
2. Explain the purpose of each component in a URDF model (links, joints, etc.).
3. What are the essential elements needed for a valid URDF model?
4. How do you validate a URDF model for correctness?
5. What is the role of transmissions in URDF models?

## Project Assignment: Complete Robot Model

Create a complete robot model that:
1. Defines a multi-joint robot with at least 4 DOF
2. Includes proper inertial, visual, and collision properties
3. Implements Gazebo plugins for simulation
4. Adds at least 2 sensors (e.g., camera, LIDAR)
5. Includes proper transmissions for ROS control
6. Validates successfully with URDF checking tools

Your robot should be able to:
- Be visualized in RViz without errors
- Be simulated in Gazebo with realistic physics
- Be controlled through ROS joint controllers
- Include proper safety limits and constraints

## Further Resources

- [URDF/XML Format Documentation](http://wiki.ros.org/urdf/XML)
- [SDF Specification](http://sdformat.org/spec)
- [Gazebo Robot Model Tutorial](http://gazebosim.org/tutorials?tut=ros_urdf)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [Robot State Publisher](http://wiki.ros.org/robot_state_publisher)

:::tip
When creating robot models, start simple and gradually add complexity. Always test each component individually before integrating everything together. Use visualization tools like RViz to verify your model looks correct before running simulations.
:::