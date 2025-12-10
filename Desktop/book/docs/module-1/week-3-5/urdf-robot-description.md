---
title: Week 3-5 - URDF for Robot Description
description: Understanding Unified Robot Description Format for humanoid and mobile robots
sidebar_position: 4
---

# Week 3-5: URDF for Robot Description

## Learning Objectives
- Understand the structure and components of URDF (Unified Robot Description Format)
- Create robot models for both mobile and humanoid robots
- Define kinematic chains, joints, and physical properties
- Visualize and validate URDF models

## Prerequisites Check
- Basic understanding of ROS 2 concepts
- Knowledge of coordinate systems and transformations
- Basic XML knowledge helpful but not required

## Theoretical Concepts: URDF Fundamentals

### What is URDF?

URDF (Unified Robot Description Format) is an XML-based format used to describe robots in ROS. It defines the physical and visual properties of a robot, including:

- **Links**: Rigid parts of the robot (e.g., chassis, wheels, arms)
- **Joints**: Connections between links (e.g., revolute, prismatic, fixed)
- **Visual elements**: How the robot appears in simulation
- **Collision elements**: Physical collision boundaries
- **Inertial properties**: Mass, center of mass, and inertia tensors

### URDF Structure

A basic URDF file has this structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links define rigid bodies -->
  <link name="link_name">
    <visual>
      <!-- Visual appearance -->
    </visual>
    <collision>
      <!-- Collision properties -->
    </collision>
    <inertial>
      <!-- Mass properties -->
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="joint_name" type="joint_type">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <origin xyz="x y z" rpy="roll pitch yaw"/>
  </joint>
</robot>
```

### Link Elements

Links represent rigid parts of the robot:

- **visual**: Defines how the link appears in visualizations
- **collision**: Defines collision boundaries for physics simulation
- **inertial**: Defines mass properties for dynamics simulation

### Joint Types

- **fixed**: No movement between parent and child
- **revolute**: Single axis rotation with limits
- **continuous**: Single axis rotation without limits
- **prismatic**: Single axis translation with limits
- **floating**: 6 DOF movement
- **planar**: Movement on a plane

### Coordinate Systems

URDF uses the right-hand rule with:
- X: Forward
- Y: Left
- Z: Up

## Step-by-Step Tutorials: Creating Robot Models

### Tutorial 1: Simple Mobile Robot

Let's create a simple differential drive robot:

```xml
<!-- File: robot_control_package/urdf/simple_robot.urdf -->
<?xml version="1.0"?>
<robot name="simple_diff_drive_robot">
  <!-- Base link - the main body of the robot -->
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

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- Joints connecting wheels to base -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Optional: Add a camera on top of the robot -->
  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
```

### Tutorial 2: Adding Gazebo-Specific Elements

To make the robot work in Gazebo simulation, we need to add Gazebo-specific elements:

```xml
<!-- File: robot_control_package/urdf/simple_robot.gazebo.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Gazebo plugin for differential drive -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
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

  <!-- Gazebo material for the robot -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="camera_link">
    <material>Gazebo/Red</material>
  </gazebo>
</robot>
```

### Tutorial 3: Creating a Simple Humanoid Robot

Let's create a basic humanoid robot with head, torso, arms, and legs:

```xml
<!-- File: robot_control_package/urdf/simple_humanoid.urdf -->
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso (main body) -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 1.0"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 1.0"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <mass value="15"/>
      <inertia ixx="1.25" ixy="0" ixz="0" iyy="1.65" iyz="0" izz="0.575"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="2"/>
      <inertia ixx="0.018" ixy="0" ixz="0" iyy="0.018" iyz="0" izz="0.018"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left upper arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1"/>
      <inertia ixx="0.0075" ixy="0" ixz="0" iyy="0.0075" iyz="0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- Right upper arm -->
  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1"/>
      <inertia ixx="0.0075" ixy="0" ixz="0" iyy="0.0075" iyz="0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- Left shoulder joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.7" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Right shoulder joint -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.7" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Left lower arm -->
  <link name="left_lower_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.006" ixy="0" ixz="0" iyy="0.006" iyz="0" izz="0.0016"/>
    </inertial>
  </link>

  <!-- Right lower arm -->
  <link name="right_lower_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.006" ixy="0" ixz="0" iyy="0.006" iyz="0" izz="0.0016"/>
    </inertial>
  </link>

  <!-- Left elbow joint -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="10" velocity="1"/>
  </joint>

  <!-- Right elbow joint -->
  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="0" effort="10" velocity="1"/>
  </joint>

  <!-- Left hip (upper leg) -->
  <link name="left_upper_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.07" length="0.5"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.07" length="0.5"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="2"/>
      <inertia ixx="0.0417" ixy="0" ixz="0" iyy="0.0417" iyz="0" izz="0.0049"/>
    </inertial>
  </link>

  <!-- Right hip (upper leg) -->
  <link name="right_upper_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.07" length="0.5"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.07" length="0.5"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="2"/>
      <inertia ixx="0.0417" ixy="0" ixz="0" iyy="0.0417" iyz="0" izz="0.0049"/>
    </inertial>
  </link>

  <!-- Left hip joint -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.07 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <!-- Right hip joint -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.07 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <!-- Left lower leg -->
  <link name="left_lower_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.0313" ixy="0" ixz="0" iyy="0.0313" iyz="0" izz="0.0027"/>
    </inertial>
  </link>

  <!-- Right lower leg -->
  <link name="right_lower_leg">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.0313" ixy="0" ixz="0" iyy="0.0313" iyz="0" izz="0.0027"/>
    </inertial>
  </link>

  <!-- Left knee joint -->
  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="0" effort="20" velocity="1"/>
  </joint>

  <!-- Right knee joint -->
  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="0" effort="20" velocity="1"/>
  </joint>

  <!-- Left foot -->
  <link name="left_foot">
    <visual>
      <origin xyz="0 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.4 0.4 0.4 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.00052" ixy="0" ixz="0" iyy="0.00145" iyz="0" izz="0.00187"/>
    </inertial>
  </link>

  <!-- Right foot -->
  <link name="right_foot">
    <visual>
      <origin xyz="0 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.4 0.4 0.4 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.00052" ixy="0" ixz="0" iyy="0.00145" iyz="0" izz="0.00187"/>
    </inertial>
  </link>

  <!-- Left ankle joint -->
  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Right ankle joint -->
  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>
</robot>
```

## Code Examples with Explanations

### Example 1: URDF with Xacro Macros

Xacro (XML Macros) allows for more readable and maintainable URDF files:

```xml
<!-- File: robot_control_package/urdf/robot_with_xacro.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_with_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.05" />
  <xacro:property name="base_length" value="0.5" />
  <xacro:property name="base_width" value="0.3" />
  <xacro:property name="base_height" value="0.2" />

  <!-- Macro for wheel -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy joint_axis">
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>

      <collision>
        <origin xyz="${xyz}" rpy="${rpy}"/>
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
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="${joint_axis}"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="10"/>
      <inertia ixx="0.416" ixy="0" ixz="0" iyy="1.0416" iyz="0" izz="1.25"/>
    </inertial>
  </link>

  <!-- Use the wheel macro to create both wheels -->
  <xacro:wheel prefix="left" parent="base_link"
               xyz="0 0.15 0" rpy="${M_PI/2} 0 0"
               joint_axis="0 1 0"/>

  <xacro:wheel prefix="right" parent="base_link"
               xyz="0 -0.15 0" rpy="${M_PI/2} 0 0"
               joint_axis="0 1 0"/>

  <!-- Camera -->
  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
```

### Example 2: URDF with Transmission Elements

To control joints in simulation and real robots, we need transmission elements:

```xml
<!-- File: robot_control_package/urdf/robot_with_transmissions.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_with_transmissions">

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

  <!-- Left wheel -->
  <link name="left_wheel">
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

  <!-- Right wheel -->
  <link name="right_wheel">
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

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Transmissions for ROS Control -->
  <transmission name="left_wheel_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_wheel_joint">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_wheel_motor">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="right_wheel_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="right_wheel_joint">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="right_wheel_motor">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <!-- Gazebo plugin for ROS Control -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/simple_robot</robotNamespace>
    </plugin>
  </gazebo>

</robot>
```

### Example 3: URDF with Sensors

Adding sensors to your robot model:

```xml
<!-- File: robot_control_package/urdf/robot_with_sensors.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_with_sensors">

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

  <!-- Left wheel -->
  <link name="left_wheel">
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

  <!-- Right wheel -->
  <link name="right_wheel">
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

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Camera sensor -->
  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>

  <!-- LIDAR sensor -->
  <link name="lidar_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.2"/>
      <inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="2e-4"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo sensor plugins -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera_sensor">
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.01</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <topic_name>camera/image_raw</topic_name>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="lidar_link">
    <sensor type="ray" name="lidar_sensor">
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
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
        <topic_name>scan</topic_name>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Hands-On Exercises: Robot Model Development

### Exercise 1: Create a Custom Mobile Robot

Create a URDF file for a robot with the following specifications:
- Rectangular base (0.6m x 0.4m x 0.2m)
- Four wheels (0.1m radius, 0.05m width)
- Two caster wheels (0.05m radius) at front and back
- RGBD camera on a pan-tilt unit

```xml
<!-- File: robot_control_package/urdf/custom_robot.urdf -->
<?xml version="1.0"?>
<robot name="custom_robot">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="silver">
        <color rgba="0.75 0.75 0.75 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <mass value="15"/>
      <inertia ixx="0.75" ixy="0" ixz="0" iyy="1.35" iyz="0" izz="1.875"/>
    </inertial>
  </link>

  <!-- Drive wheels -->
  <link name="front_left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.008"/>
    </inertial>
  </link>

  <link name="front_right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.008"/>
    </inertial>
  </link>

  <link name="rear_left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.008"/>
    </inertial>
  </link>

  <link name="rear_right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.008"/>
    </inertial>
  </link>

  <!-- Caster wheels -->
  <link name="front_caster">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/>
    </inertial>
  </link>

  <link name="rear_caster">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/>
    </inertial>
  </link>

  <!-- Camera pan-tilt unit -->
  <link name="camera_pan_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.2"/>
      <inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="2e-4"/>
    </inertial>
  </link>

  <link name="camera_tilt_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="0.04"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="0.04"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="5e-5" ixy="0" ixz="0" iyy="5e-5" iyz="0" izz="1e-4"/>
    </inertial>
  </link>

  <link name="rgbd_camera">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.05 0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.05 0.03"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.3"/>
      <inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="front_left_wheel"/>
    <origin xyz="0.15 0.18 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="front_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="front_right_wheel"/>
    <origin xyz="0.15 -0.18 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="rear_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_left_wheel"/>
    <origin xyz="-0.15 0.18 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="rear_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_right_wheel"/>
    <origin xyz="-0.15 -0.18 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="front_caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="front_caster"/>
    <origin xyz="0.25 0 0.05" rpy="0 0 0"/>
  </joint>

  <joint name="rear_caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="rear_caster"/>
    <origin xyz="-0.25 0 0.05" rpy="0 0 0"/>
  </joint>

  <joint name="camera_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="camera_pan_link"/>
    <origin xyz="0.25 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="1" velocity="1"/>
  </joint>

  <joint name="camera_tilt_joint" type="revolute">
    <parent link="camera_pan_link"/>
    <child link="camera_tilt_link"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.785" upper="0.785" effort="1" velocity="1"/>
  </joint>

  <joint name="rgbd_camera_joint" type="fixed">
    <parent link="camera_tilt_link"/>
    <child link="rgbd_camera"/>
    <origin xyz="0 0 0.03" rpy="0 0 0"/>
  </joint>

</robot>
```

### Exercise 2: Robot State Publisher Node

Create a ROS 2 node to publish the robot's state:

```python
# File: robot_control_package/robot_control_package/robot_state_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class RobotStatePublisher(Node):
    """
    Publish joint states and transforms for robot visualization
    """

    def __init__(self):
        super().__init__('robot_state_publisher')

        # Create publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Create transform broadcaster for TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create timer to publish states
        self.timer = self.create_timer(0.1, self.publish_states)

        # Internal state
        self.time = 0.0

        self.get_logger().info('Robot State Publisher initialized')

    def publish_states(self):
        """
        Publish joint states and transforms
        """
        # Create joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'

        # Define joint names
        joint_names = [
            'left_wheel_joint', 'right_wheel_joint',
            'camera_pan_joint', 'camera_tilt_joint'
        ]
        joint_state.name = joint_names

        # Define joint positions (with some oscillation for visualization)
        self.time += 0.1
        positions = [
            math.sin(self.time),           # Left wheel
            math.cos(self.time),           # Right wheel
            0.5 * math.sin(0.5 * self.time),  # Camera pan
            0.3 * math.cos(0.3 * self.time)   # Camera tilt
        ]
        joint_state.position = positions

        # Publish joint states
        self.joint_state_publisher.publish(joint_state)

        # Publish transforms for each link
        self.publish_transforms(joint_state)

    def publish_transforms(self, joint_state):
        """
        Publish transforms for visualization
        """
        # Base to left wheel
        t = TransformStamped()
        t.header.stamp = joint_state.header.stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'left_wheel'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.15
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # Base to right wheel
        t = TransformStamped()
        t.header.stamp = joint_state.header.stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'right_wheel'
        t.transform.translation.x = 0.0
        t.transform.translation.y = -0.15
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # Base to camera pan
        t = TransformStamped()
        t.header.stamp = joint_state.header.stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_pan_link'
        t.transform.translation.x = 0.25
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.25
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()

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

## Common Pitfalls and Solutions

### Pitfall 1: Incorrect Inertial Properties
**Problem**: Robot behaves unrealistically in simulation due to incorrect mass properties.

**Solution**:
- Calculate inertial properties properly for geometric shapes
- Use CAD software to export accurate inertial properties
- Ensure units are consistent (kg for mass, kg*m² for inertia)

```xml
<!-- Correct calculation for a box -->
<inertial>
  <mass value="1.0"/>
  <!-- For a box: Ixx = 1/12 * m * (h² + d²), etc. -->
  <inertia ixx="0.0083" ixy="0" ixz="0" iyy="0.0104" iyz="0" izz="0.0021"/>
</inertial>
```

### Pitfall 2: Joint Limit Issues
**Problem**: Robot joints move beyond physical limits or don't move as expected.

**Solution**:
- Define appropriate joint limits based on physical constraints
- Consider safety margins in limit definitions
- Use proper joint types for intended motion

### Pitfall 3: Collision Mesh Issues
**Problem**: Robot passes through objects or has unexpected collisions.

**Solution**:
- Ensure collision meshes are properly sized and positioned
- Use simpler collision geometries when possible
- Verify that collision and visual elements are aligned

## Review Questions

1. What are the main components of a URDF file?
2. Explain the difference between visual, collision, and inertial elements in URDF.
3. What are the different joint types available in URDF and when would you use each?
4. How do you add sensors to a robot model in URDF?
5. What is the purpose of the robot state publisher in ROS 2?

## Project Assignment: Complete Robot Model

Create a complete robot model that includes:
1. A base with appropriate drive mechanism (wheels, tracks, etc.)
2. At least 3 different sensor types (camera, LIDAR, IMU)
3. Proper inertial properties for all links
4. Transmission definitions for controlled joints
5. Gazebo plugins for simulation
6. A working robot state publisher node

Your model should be:
- Physically realistic with proper mass properties
- Functionally correct with appropriate joint limits
- Ready for simulation in Gazebo
- Well-organized using Xacro macros where appropriate

## Further Resources

- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF.html)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [Robot State Publisher](https://github.com/ros/robot_state_publisher)
- [Gazebo Integration](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
- [Inertial Calculations](https://en.wikipedia.org/wiki/List_of_moments_of_inertia)

<details>
<summary>Advanced URDF Concepts</summary>

URDF is just one part of the robot description ecosystem. SDF (Simulation Description Format) is used directly by Gazebo, while URDF is typically converted to SDF for simulation. Xacro macros make complex robots manageable, and ROS 2 Control provides the interface between your robot description and actual hardware control.

</details>