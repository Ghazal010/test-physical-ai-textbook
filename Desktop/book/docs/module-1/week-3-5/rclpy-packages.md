---
title: Week 3-5 - Building ROS 2 Packages with rclpy
description: Creating and managing ROS 2 packages using Python client library
sidebar_position: 3
---

# Week 3-5: Building ROS 2 Packages with rclpy

## Learning Objectives
- Create ROS 2 packages using the ament_python build system
- Understand the structure of ROS 2 Python packages
- Build modular and reusable ROS 2 components
- Implement proper package organization and dependencies

## Prerequisites Check
- Understanding of ROS 2 architecture and communication patterns
- Python programming knowledge
- Basic understanding of package management

## Theoretical Concepts: ROS 2 Package Structure

### Package Organization

A ROS 2 package is a reusable software module that contains nodes, libraries, and other resources. The standard structure for a Python-based ROS 2 package includes:

```
my_robot_package/
├── package.xml          # Package metadata
├── setup.py             # Python package setup
├── setup.cfg            # Installation configuration
├── my_robot_package/    # Main Python module
│   ├── __init__.py
│   ├── node_example.py
│   └── utility_functions.py
├── launch/              # Launch files
│   └── robot_launch.py
├── config/              # Configuration files
│   └── robot_params.yaml
└── test/                # Unit tests
    └── test_node.py
```

### Build System: ament_python

The `ament_python` build system is used for Python packages in ROS 2:

- **setup.py**: Defines package metadata and dependencies
- **setup.cfg**: Configuration for installation
- **entry_points**: Console scripts that become executable commands
- **ament**: The build and test infrastructure for ROS 2

### Package.xml Manifest

The `package.xml` file contains metadata about the package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>Example robot package</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache License 2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Step-by-Step Tutorials: Creating a Complete ROS 2 Package

### Tutorial 1: Creating a New Package from Scratch

Let's create a complete robot control package:

```bash
# Navigate to your workspace
cd ~/ros2_workspace/src

# Create a new package
ros2 pkg create --build-type ament_python robot_control_package --dependencies rclpy std_msgs geometry_msgs sensor_msgs

# Navigate to the new package
cd robot_control_package
```

### Tutorial 2: Setting up the Package Structure

First, let's create the main package files:

```python
# File: robot_control_package/setup.py
from setuptools import find_packages, setup

package_name = 'robot_control_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch', ['launch/robot_control_launch.py']),
        # Include config files
        ('share/' + package_name + '/config', ['config/robot_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A comprehensive robot control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = robot_control_package.robot_controller:main',
            'robot_monitor = robot_control_package.robot_monitor:main',
            'path_planner = robot_control_package.path_planner:main',
            'sensor_processor = robot_control_package.sensor_processor:main',
        ],
    },
)
```

```ini
# File: robot_control_package/setup.cfg
[develop]
script-dir=$base/lib/robot_control_package
[install]
install-scripts=$base/lib/robot_control_package
```

### Tutorial 3: Creating the Main Robot Controller Node

```python
# File: robot_control_package/robot_control_package/robot_controller.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from robot_control_package.utils import RobotState, calculate_velocity

class RobotController(Node):
    """
    Main robot controller node that handles movement commands and safety
    """

    def __init__(self):
        super().__init__('robot_controller')

        # Declare parameters with defaults
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('robot_radius', 0.3)

        # Get parameter values
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.robot_radius = self.get_parameter('robot_radius').value

        # Create QoS profiles
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', reliable_qos)
        self.status_publisher = self.create_publisher(String, 'robot_status', reliable_qos)

        # Subscriptions
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'navigation_velocity',
            self.cmd_vel_callback,
            reliable_qos
        )

        self.laser_subscription = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.laser_callback,
            best_effort_qos
        )

        self.emergency_subscription = self.create_subscription(
            Bool,
            'emergency_stop',
            self.emergency_callback,
            reliable_qos
        )

        # Timer for safety checks
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        # Internal state
        self.current_cmd_vel = Twist()
        self.laser_data = None
        self.emergency_stop_active = False
        self.robot_state = RobotState.STOPPED

        self.get_logger().info('Robot Controller initialized')

    def cmd_vel_callback(self, msg):
        """
        Handle incoming velocity commands
        """
        if not self.emergency_stop_active:
            # Apply safety limits
            msg.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
            msg.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))

            self.current_cmd_vel = msg
            self.robot_state = RobotState.MOVING

            # Publish the command
            self.cmd_vel_publisher.publish(msg)
        else:
            # If emergency stop is active, send stop command
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            self.robot_state = RobotState.EMERGENCY_STOPPED

    def laser_callback(self, msg):
        """
        Handle laser scan data for obstacle detection
        """
        self.laser_data = msg

    def emergency_callback(self, msg):
        """
        Handle emergency stop commands
        """
        self.emergency_stop_active = msg.data
        if self.emergency_stop_active:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            # Send stop command immediately
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            self.robot_state = RobotState.EMERGENCY_STOPPED
        else:
            self.get_logger().info('Emergency stop deactivated')

    def safety_check(self):
        """
        Perform safety checks based on sensor data
        """
        if self.laser_data and not self.emergency_stop_active:
            # Check for obstacles in front of the robot
            min_distance = min(self.laser_data.ranges)

            if min_distance < self.safety_distance:
                # Reduce speed or stop based on proximity
                reduction_factor = min_distance / self.safety_distance
                reduced_cmd = Twist()
                reduced_cmd.linear.x = self.current_cmd_vel.linear.x * reduction_factor
                reduced_cmd.angular.z = self.current_cmd_vel.angular.z * reduction_factor

                self.cmd_vel_publisher.publish(reduced_cmd)

                if min_distance < self.robot_radius:
                    # Too close, stop completely
                    stop_cmd = Twist()
                    self.cmd_vel_publisher.publish(stop_cmd)
                    self.get_logger().warn(f'OBSTACLE TOO CLOSE: {min_distance:.2f}m')

        # Publish status
        status_msg = String()
        status_msg.data = f'State: {self.robot_state.name}, Vel: ({self.current_cmd_vel.linear.x:.2f}, {self.current_cmd_vel.angular.z:.2f})'
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down robot controller...')
    finally:
        # Send stop command before shutdown
        stop_cmd = Twist()
        node.cmd_vel_publisher.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Tutorial 4: Creating Utility Modules

```python
# File: robot_control_package/robot_control_package/utils.py
from enum import Enum
import math

class RobotState(Enum):
    """
    Robot state enumeration
    """
    STOPPED = "STOPPED"
    MOVING = "MOVING"
    IDLE = "IDLE"
    EMERGENCY_STOPPED = "EMERGENCY_STOPPED"
    CHARGING = "CHARGING"

def calculate_velocity(distance, max_velocity=1.0, threshold=1.0):
    """
    Calculate appropriate velocity based on distance to obstacle

    Args:
        distance: Distance to obstacle in meters
        max_velocity: Maximum velocity
        threshold: Distance at which to start reducing speed

    Returns:
        Appropriate velocity based on safety distance
    """
    if distance <= 0:
        return 0.0

    if distance >= threshold:
        return max_velocity
    else:
        # Linear reduction in velocity as distance decreases
        ratio = distance / threshold
        return max_velocity * ratio

def distance_2d(point1, point2):
    """
    Calculate 2D distance between two points

    Args:
        point1: Tuple (x1, y1) or dict with 'x', 'y' keys
        point2: Tuple (x2, y2) or dict with 'x', 'y' keys

    Returns:
        Distance between points
    """
    if isinstance(point1, dict):
        x1, y1 = point1['x'], point1['y']
    else:
        x1, y1 = point1[0], point1[1]

    if isinstance(point2, dict):
        x2, y2 = point2['x'], point2['y']
    else:
        x2, y2 = point2[0], point2[1]

    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def normalize_angle(angle):
    """
    Normalize angle to [-pi, pi] range

    Args:
        angle: Angle in radians

    Returns:
        Normalized angle in [-pi, pi]
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class PIDController:
    """
    Simple PID controller for robot control
    """

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, dt=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error):
        """
        Compute PID output

        Args:
            error: Current error value

        Returns:
            PID controller output
        """
        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * self.dt
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0
        d_term = self.kd * derivative

        # Store current error for next iteration
        self.prev_error = error

        return p_term + i_term + d_term

def validate_twist_command(twist, max_linear=1.0, max_angular=1.0):
    """
    Validate and limit twist command values

    Args:
        twist: geometry_msgs.msg.Twist object
        max_linear: Maximum linear velocity
        max_angular: Maximum angular velocity

    Returns:
        Validated and limited twist command
    """
    # Limit linear velocity
    twist.linear.x = max(-max_linear, min(max_linear, twist.linear.x))
    twist.linear.y = max(-max_linear, min(max_linear, twist.linear.y))
    twist.linear.z = max(-max_linear, min(max_linear, twist.linear.z))

    # Limit angular velocity
    twist.angular.x = max(-max_angular, min(max_angular, twist.angular.x))
    twist.angular.y = max(-max_angular, min(max_angular, twist.angular.y))
    twist.angular.z = max(-max_angular, min(max_angular, twist.angular.z))

    return twist
```

### Tutorial 5: Creating Additional Nodes

```python
# File: robot_control_package/robot_control_package/robot_monitor.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
import time

class RobotMonitor(Node):
    """
    Monitor robot status and system health
    """

    def __init__(self):
        super().__init__('robot_monitor')

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscriptions for monitoring
        self.status_subscription = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            qos
        )

        self.battery_subscription = self.create_subscription(
            BatteryState,
            'battery_status',
            self.battery_callback,
            qos
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_monitor_callback,
            qos
        )

        # Publishers for alerts
        self.alert_publisher = self.create_publisher(String, 'system_alerts', qos)
        self.health_publisher = self.create_publisher(Float32, 'system_health', qos)

        # Timer for system checks
        self.health_timer = self.create_timer(1.0, self.system_health_check)

        # Internal state
        self.last_status = "Unknown"
        self.battery_level = 100.0
        self.last_cmd_time = time.time()
        self.last_cmd_vel = Twist()

        # System thresholds
        self.low_battery_threshold = 20.0
        self.inactivity_threshold = 30.0  # seconds

        self.get_logger().info('Robot Monitor initialized')

    def status_callback(self, msg):
        """
        Handle robot status updates
        """
        self.last_status = msg.data
        self.last_cmd_time = time.time()

    def battery_callback(self, msg):
        """
        Handle battery status updates
        """
        self.battery_level = msg.percentage * 100.0  # Convert from 0-1 to 0-100

        if self.battery_level < self.low_battery_threshold:
            alert_msg = String()
            alert_msg.data = f'LOW BATTERY WARNING: {self.battery_level:.1f}% remaining'
            self.alert_publisher.publish(alert_msg)
            self.get_logger().warn(alert_msg.data)

    def cmd_vel_monitor_callback(self, msg):
        """
        Monitor velocity commands
        """
        self.last_cmd_vel = msg

    def system_health_check(self):
        """
        Perform system health checks
        """
        current_time = time.time()

        # Check for inactivity
        time_since_last_cmd = current_time - self.last_cmd_time
        if time_since_last_cmd > self.inactivity_threshold:
            alert_msg = String()
            alert_msg.data = f'INACTIVITY ALERT: No commands for {time_since_last_cmd:.1f}s'
            self.alert_publisher.publish(alert_msg)
            self.get_logger().warn(alert_msg.data)

        # Calculate system health score (0-100)
        health_score = 100.0

        # Reduce health for low battery
        if self.battery_level < 50:
            health_score -= (50 - self.battery_level) * 0.5

        # Reduce health for inactivity
        if time_since_last_cmd > 10:
            health_score -= min(20, (time_since_last_cmd - 10) * 0.5)

        health_score = max(0, min(100, health_score))

        # Publish health score
        health_msg = Float32()
        health_msg.data = health_score
        self.health_publisher.publish(health_msg)

        self.get_logger().info(f'System health: {health_score:.1f}, Battery: {self.battery_level:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down robot monitor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Tutorial 6: Creating a Path Planning Node

```python
# File: robot_control_package/robot_control_package/path_planner.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
import math
from typing import List, Tuple

class PathPlanner(Node):
    """
    Simple path planning node using A* algorithm
    """

    def __init__(self):
        super().__init__('path_planner')

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.path_publisher = self.create_publisher(Path, 'global_plan', qos)
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', qos)

        # Subscriptions
        self.start_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.start_callback,
            qos
        )

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            'move_base_simple/goal',
            self.goal_callback,
            qos
        )

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            qos
        )

        # Internal state
        self.map_data = None
        self.start_pose = None
        self.goal_pose = None
        self.resolution = 0.05  # meters per cell

        self.get_logger().info('Path Planner initialized')

    def start_callback(self, msg):
        """
        Handle initial pose (start position)
        """
        self.start_pose = msg.pose.pose
        self.get_logger().info('Received start pose')

    def goal_callback(self, msg):
        """
        Handle goal pose
        """
        self.goal_pose = msg.pose
        self.get_logger().info('Received goal pose')

        # Plan path if we have both start and goal
        if self.start_pose and self.goal_pose and self.map_data:
            self.plan_path()

    def map_callback(self, msg):
        """
        Handle map data
        """
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height}, res: {self.resolution}m')

    def plan_path(self):
        """
        Plan path from start to goal using A* algorithm
        """
        if not self.map_data:
            self.get_logger().error('No map data available for path planning')
            return

        # Convert poses to map coordinates
        start_map = self.pose_to_map_coords(self.start_pose)
        goal_map = self.pose_to_map_coords(self.goal_pose)

        if not start_map or not goal_map:
            self.get_logger().error('Could not convert poses to map coordinates')
            return

        # Run A* algorithm
        path = self.a_star(start_map, goal_map)

        if path:
            # Convert path back to world coordinates
            world_path = self.path_to_world_coords(path)

            # Publish the path
            self.publish_path(world_path)

            self.get_logger().info(f'Path planned with {len(path)} waypoints')
        else:
            self.get_logger().error('Could not find a path to goal')

    def pose_to_map_coords(self, pose) -> Tuple[int, int]:
        """
        Convert world coordinates to map cell coordinates
        """
        if not self.map_data:
            return None

        # Calculate map coordinates
        map_x = int((pose.position.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        map_y = int((pose.position.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # Check bounds
        if (0 <= map_x < self.map_data.info.width and
            0 <= map_y < self.map_data.info.height):
            return (map_x, map_y)
        else:
            return None

    def path_to_world_coords(self, path: List[Tuple[int, int]]) -> List[Tuple[float, float]]:
        """
        Convert map cell coordinates back to world coordinates
        """
        if not self.map_data:
            return []

        world_path = []
        for map_x, map_y in path:
            world_x = map_x * self.map_data.info.resolution + self.map_data.info.origin.position.x
            world_y = map_y * self.map_data.info.resolution + self.map_data.info.origin.position.y
            world_path.append((world_x, world_y))

        return world_path

    def a_star(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Simple A* path planning algorithm
        """
        # This is a simplified implementation
        # In a real system, you'd want a more sophisticated algorithm

        # For this example, we'll use a simple direct path with obstacle avoidance
        path = [start]

        # Calculate direction vector
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]

        # Move in steps toward the goal
        current = start
        step_size = 1  # Move one cell at a time

        while current != goal:
            # Determine next step
            next_x = current[0] + (1 if dx > 0 else -1 if dx < 0 else 0)
            next_y = current[1] + (1 if dy > 0 else -1 if dy < 0 else 0)

            next_cell = (next_x, next_y)

            # Check if next cell is valid (not occupied)
            if self.is_valid_cell(next_cell):
                path.append(next_cell)
                current = next_cell
            else:
                # Simple obstacle avoidance - try alternative path
                # In a real implementation, this would be more sophisticated
                break

            # Update remaining distance
            dx = goal[0] - current[0]
            dy = goal[1] - current[1]

            # Prevent infinite loops
            if len(path) > 1000:  # Arbitrary limit
                self.get_logger().warn('Path planning exceeded maximum iterations')
                break

        # If we reached the goal, return path; otherwise return empty path
        return path if current == goal else []

    def is_valid_cell(self, cell: Tuple[int, int]) -> bool:
        """
        Check if a map cell is valid (not occupied)
        """
        if not self.map_data:
            return False

        x, y = cell

        # Check bounds
        if not (0 <= x < self.map_data.info.width and 0 <= y < self.map_data.info.height):
            return False

        # Get index in the data array
        index = y * self.map_data.info.width + x

        # Check if the cell is occupied (value > 50 means occupied in occupancy grid)
        return self.map_data.data[index] < 50

    def publish_path(self, path: List[Tuple[float, float]]):
        """
        Publish the planned path
        """
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'  # Assuming map frame

        for x, y in path:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0  # No rotation

            path_msg.poses.append(pose_stamped)

        self.path_publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down path planner...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Code Examples with Explanations

### Example 1: Configuration Management

```python
# File: robot_control_package/robot_control_package/config_manager.py
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
import yaml
import os

class ConfigManager(Node):
    """
    Manage robot configuration using parameter files
    """

    def __init__(self):
        super().__init__('config_manager')

        # Declare parameters
        self.declare_parameter('config_file_path', '/default/config.yaml')

        # Load configuration
        self.load_configuration()

        # Publisher for config changes
        self.config_publisher = self.create_publisher(String, 'config_updates', 10)

        self.get_logger().info('Configuration Manager initialized')

    def load_configuration(self):
        """
        Load configuration from file and set as parameters
        """
        config_file = self.get_parameter('config_file_path').value

        if os.path.exists(config_file):
            with open(config_file, 'r') as file:
                config_data = yaml.safe_load(file)

            # Set parameters from config file
            for key, value in config_data.items():
                if isinstance(value, dict):
                    # Handle nested parameters
                    for sub_key, sub_value in value.items():
                        param_name = f'{key}.{sub_key}'
                        self.declare_parameter(param_name, sub_value)
                        self.get_logger().info(f'Set parameter {param_name} = {sub_value}')
                else:
                    self.declare_parameter(key, value)
                    self.get_logger().info(f'Set parameter {key} = {value}')
        else:
            self.get_logger().warn(f'Config file not found: {config_file}')
            # Set default values
            self.set_default_parameters()

    def set_default_parameters(self):
        """
        Set default parameters if config file is not available
        """
        defaults = {
            'robot.max_linear_velocity': 1.0,
            'robot.max_angular_velocity': 1.0,
            'safety.safety_distance': 0.5,
            'navigation.planning_frequency': 5.0
        }

        for param_name, default_value in defaults.items():
            self.declare_parameter(param_name, default_value)

    def update_parameter(self, param_name: str, value):
        """
        Update a parameter at runtime
        """
        try:
            param = Parameter(param_name, Parameter.Type.NOT_SET, value)
            result = self.set_parameters([param])

            if all(success for success in result):
                self.get_logger().info(f'Updated parameter {param_name} = {value}')

                # Publish update notification
                update_msg = String()
                update_msg.data = f'PARAM_UPDATE: {param_name}={value}'
                self.config_publisher.publish(update_msg)

                return True
            else:
                self.get_logger().error(f'Failed to update parameter {param_name}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error updating parameter {param_name}: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = ConfigManager()

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

### Example 2: Launch File for Package

```python
# File: robot_control_package/launch/robot_control_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Robot controller node
    robot_controller = Node(
        package='robot_control_package',
        executable='robot_controller',
        name='robot_controller',
        parameters=[
            {'max_linear_velocity': 1.0},
            {'max_angular_velocity': 1.0},
            {'safety_distance': 0.5},
            {'robot_radius': 0.3},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Robot monitor node
    robot_monitor = Node(
        package='robot_control_package',
        executable='robot_monitor',
        name='robot_monitor',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Path planner node
    path_planner = Node(
        package='robot_control_package',
        executable='path_planner',
        name='path_planner',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Sensor processor node
    sensor_processor = Node(
        package='robot_control_package',
        executable='sensor_processor',
        name='sensor_processor',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        declare_use_sim_time,
        robot_controller,
        robot_monitor,
        path_planner,
        sensor_processor
    ])
```

### Example 3: Testing Framework

```python
# File: robot_control_package/test/test_robot_controller.py
import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from robot_control_package.robot_controller import RobotController

class TestRobotController(unittest.TestCase):
    """
    Unit tests for RobotController node
    """

    def setUp(self):
        """
        Set up the test environment
        """
        if not rclpy.ok():
            rclpy.init()

        self.node = RobotController()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        """
        Clean up after tests
        """
        self.node.destroy_node()
        rclpy.shutdown()

    def test_initial_state(self):
        """
        Test that the node initializes with correct default values
        """
        self.assertEqual(self.node.robot_state.name, 'STOPPED')
        self.assertEqual(self.node.max_linear_vel, 1.0)
        self.assertEqual(self.node.max_angular_vel, 1.0)
        self.assertEqual(self.node.safety_distance, 0.5)
        self.assertEqual(self.node.robot_radius, 0.3)

    def test_velocity_limiting(self):
        """
        Test that velocity commands are properly limited
        """
        # Create a command with values exceeding limits
        cmd = Twist()
        cmd.linear.x = 2.0  # Exceeds max of 1.0
        cmd.angular.z = -1.5  # Exceeds max of 1.0

        # Apply limiting function (would be called internally in real usage)
        from robot_control_package.utils import validate_twist_command
        limited_cmd = validate_twist_command(cmd, max_linear=1.0, max_angular=1.0)

        self.assertEqual(limited_cmd.linear.x, 1.0)
        self.assertEqual(limited_cmd.angular.z, -1.0)

    def test_safety_distance(self):
        """
        Test that safety distance parameter is properly set
        """
        self.assertEqual(self.node.safety_distance, 0.5)

def test_main():
    """
    Main test function to run all tests
    """
    unittest.main()

if __name__ == '__main__':
    test_main()
```

## Hands-On Exercises: Building Reusable Components

### Exercise 1: Create a Sensor Processing Node

Create a node that processes sensor data from multiple sources:

```python
# File: robot_control_package/robot_control_package/sensor_processor.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Imu, BatteryState
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Vector3
import numpy as np
from collections import deque
import statistics

class SensorProcessor(Node):
    """
    Process and analyze data from multiple sensors
    """

    def __init__(self):
        super().__init__('sensor_processor')

        # QoS profiles
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Declare parameters
        self.declare_parameter('laser_scan_history_size', 5)
        self.declare_parameter('imu_history_size', 10)
        self.declare_parameter('publish_frequency', 10.0)

        # Get parameter values
        laser_history_size = self.get_parameter('laser_scan_history_size').value
        imu_history_size = self.get_parameter('imu_history_size').value
        publish_freq = self.get_parameter('publish_frequency').value

        # Publishers
        self.processed_laser_publisher = self.create_publisher(Float32MultiArray, 'processed_laser', reliable_qos)
        self.fused_imu_publisher = self.create_publisher(Vector3, 'fused_imu', reliable_qos)
        self.sensor_status_publisher = self.create_publisher(String, 'sensor_status', reliable_qos)

        # Subscriptions
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'laser_scan',
            self.laser_callback,
            sensor_qos
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            'imu_data',
            self.imu_callback,
            sensor_qos
        )

        self.battery_subscription = self.create_subscription(
            BatteryState,
            'battery_status',
            self.battery_callback,
            sensor_qos
        )

        # Data storage with circular buffers
        self.laser_history = deque(maxlen=laser_history_size)
        self.imu_history = deque(maxlen=imu_history_size)

        # Timers
        self.processing_timer = self.create_timer(1.0/publish_freq, self.process_sensors)

        # Internal state
        self.battery_level = 100.0
        self.last_battery_time = self.get_clock().now()

        self.get_logger().info('Sensor Processor initialized')

    def laser_callback(self, msg):
        """
        Handle laser scan data
        """
        # Store in history for processing
        self.laser_history.append(msg)

    def imu_callback(self, msg):
        """
        Handle IMU data
        """
        # Store in history for processing
        self.imu_history.append(msg)

    def battery_callback(self, msg):
        """
        Handle battery data
        """
        self.battery_level = msg.percentage * 100.0
        self.last_battery_time = self.get_clock().now()

    def process_sensors(self):
        """
        Process sensor data and publish results
        """
        # Process laser data
        if self.laser_history:
            processed_laser = self.process_laser_data()
            if processed_laser is not None:
                self.processed_laser_publisher.publish(processed_laser)

        # Process IMU data
        if self.imu_history:
            fused_imu = self.process_imu_data()
            if fused_imu is not None:
                self.fused_imu_publisher.publish(fused_imu)

        # Publish sensor status
        status_msg = String()
        status_msg.data = (
            f'Laser scans: {len(self.laser_history)}, '
            f'IMU readings: {len(self.imu_history)}, '
            f'Battery: {self.battery_level:.1f}%'
        )
        self.sensor_status_publisher.publish(status_msg)

    def process_laser_data(self):
        """
        Process laser scan data to extract useful information
        """
        if not self.laser_history:
            return None

        # Get the most recent laser scan
        latest_scan = self.laser_history[-1]

        # Calculate some statistics
        valid_ranges = [r for r in latest_scan.ranges if 0 < r < latest_scan.range_max]

        if not valid_ranges:
            return None

        # Calculate statistics
        min_distance = min(valid_ranges) if valid_ranges else float('inf')
        max_distance = max(valid_ranges) if valid_ranges else 0
        avg_distance = statistics.mean(valid_ranges) if valid_ranges else 0

        # Create processed laser message
        processed_msg = Float32MultiArray()
        processed_msg.data = [
            min_distance,
            max_distance,
            avg_distance,
            len(valid_ranges),  # number of valid readings
            latest_scan.angle_min,
            latest_scan.angle_max,
            latest_scan.angle_increment
        ]

        return processed_msg

    def process_imu_data(self):
        """
        Process IMU data to get averaged orientation
        """
        if not self.imu_history:
            return None

        # Calculate average orientation from recent readings
        avg_orientation = Vector3()

        # Extract and average the orientation values
        orientations = []
        for imu_msg in self.imu_history:
            orientations.append([
                imu_msg.orientation.x,
                imu_msg.orientation.y,
                imu_msg.orientation.z
            ])

        if orientations:
            avg_orientation.x = statistics.mean([o[0] for o in orientations])
            avg_orientation.y = statistics.mean([o[1] for o in orientations])
            avg_orientation.z = statistics.mean([o[2] for o in orientations])

        return avg_orientation

def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down sensor processor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 2: Create a Behavior Manager

Create a node that manages robot behaviors based on conditions:

```python
# File: robot_control_package/robot_control_package/behavior_manager.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist, PoseStamped
from enum import Enum
import time

class BehaviorState(Enum):
    IDLE = "IDLE"
    PATROLLING = "PATROLLING"
    EXPLORING = "EXPLORING"
    RETURNING_HOME = "RETURNING_HOME"
    CHARGING = "CHARGING"
    EMERGENCY = "EMERGENCY"

class BehaviorManager(Node):
    """
    Manage robot behaviors based on sensor input and conditions
    """

    def __init__(self):
        super().__init__('behavior_manager')

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', qos)
        self.behavior_publisher = self.create_publisher(String, 'current_behavior', qos)
        self.goal_publisher = self.create_publisher(PoseStamped, 'move_base_simple/goal', qos)

        # Subscriptions
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_status',
            self.sensor_status_callback,
            qos
        )

        self.battery_subscription = self.create_subscription(
            Float32,
            'system_health',
            self.battery_callback,
            qos
        )

        self.obstacle_subscription = self.create_subscription(
            Bool,
            'obstacle_detected',
            self.obstacle_callback,
            qos
        )

        # Timer for behavior updates
        self.behavior_timer = self.create_timer(0.1, self.update_behavior)

        # Internal state
        self.current_behavior = BehaviorState.IDLE
        self.system_health = 100.0
        self.obstacle_detected = False
        self.last_behavior_change = time.time()
        self.behavior_start_time = time.time()

        # Configuration parameters
        self.low_health_threshold = 30.0
        self.charging_health_threshold = 20.0
        self.min_behavior_duration = 5.0  # seconds

        self.get_logger().info('Behavior Manager initialized')

    def sensor_status_callback(self, msg):
        """
        Handle sensor status updates
        """
        # Parse sensor status to extract relevant information
        if 'OBSTACLE' in msg.data:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def battery_callback(self, msg):
        """
        Handle battery/health updates
        """
        self.system_health = msg.data

    def obstacle_callback(self, msg):
        """
        Handle obstacle detection
        """
        self.obstacle_detected = msg.data

    def update_behavior(self):
        """
        Update robot behavior based on current state and conditions
        """
        current_time = time.time()

        # Check for emergency conditions first
        if self.system_health < 10.0:
            self.set_behavior(BehaviorState.EMERGENCY)
        elif self.system_health < self.charging_health_threshold:
            self.set_behavior(BehaviorState.RETURNING_HOME)
        elif self.obstacle_detected and self.current_behavior != BehaviorState.EMERGENCY:
            self.handle_obstacle()
        else:
            # Normal behavior progression
            self.manage_normal_behaviors(current_time)

        # Execute current behavior
        self.execute_current_behavior()

    def set_behavior(self, new_behavior):
        """
        Safely change the current behavior
        """
        if new_behavior != self.current_behavior:
            current_time = time.time()

            # Check minimum duration to prevent rapid switching
            if (current_time - self.last_behavior_change) >= self.min_behavior_duration:
                self.get_logger().info(f'Changing behavior from {self.current_behavior.value} to {new_behavior.value}')

                self.current_behavior = new_behavior
                self.behavior_start_time = current_time
                self.last_behavior_change = current_time

                # Publish behavior change
                behavior_msg = String()
                behavior_msg.data = self.current_behavior.value
                self.behavior_publisher.publish(behavior_msg)

    def handle_obstacle(self):
        """
        Handle obstacle detection
        """
        if self.current_behavior not in [BehaviorState.EMERGENCY, BehaviorState.CHARGING]:
            # For now, just stop when obstacle is detected
            # In a real system, you might implement avoidance behaviors
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)

    def manage_normal_behaviors(self, current_time):
        """
        Manage normal behavior progression
        """
        time_in_behavior = current_time - self.behavior_start_time

        if self.current_behavior == BehaviorState.IDLE:
            # After being idle for a while, start patrolling
            if time_in_behavior > 10.0:  # 10 seconds idle
                self.set_behavior(BehaviorState.PATROLLING)

        elif self.current_behavior == BehaviorState.PATROLLING:
            # Patrol for a while, then explore
            if time_in_behavior > 60.0:  # 1 minute patrolling
                self.set_behavior(BehaviorState.EXPLORING)

        elif self.current_behavior == BehaviorState.EXPLORING:
            # Explore for a while, then return to patrol
            if time_in_behavior > 120.0:  # 2 minutes exploring
                self.set_behavior(BehaviorState.PATROLLING)

    def execute_current_behavior(self):
        """
        Execute the commands for the current behavior
        """
        cmd_vel = Twist()

        if self.current_behavior == BehaviorState.IDLE:
            # Stay still
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        elif self.current_behavior == BehaviorState.PATROLLING:
            # Simple patrol pattern - move forward with occasional turns
            cmd_vel.linear.x = 0.5  # Move forward at 0.5 m/s
            cmd_vel.angular.z = 0.1 * (time.time() % 10 > 5)  # Gentle turns occasionally

        elif self.current_behavior == BehaviorState.EXPLORING:
            # Exploration pattern - more random movement
            cmd_vel.linear.x = 0.3
            cmd_vel.angular.z = 0.2 * (time.time() % 4 > 2)  # More frequent turns

        elif self.current_behavior == BehaviorState.RETURNING_HOME:
            # In a real system, this would navigate to a charging station
            cmd_vel.linear.x = -0.2  # Move back slowly
            cmd_vel.angular.z = 0.0

        elif self.current_behavior == BehaviorState.EMERGENCY:
            # Emergency stop
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        # If no obstacle is detected, publish the command
        if not self.obstacle_detected:
            self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down behavior manager...')
    finally:
        # Send stop command before shutdown
        stop_cmd = Twist()
        node.cmd_vel_publisher.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Pitfalls and Solutions

### Pitfall 1: Import Issues in ROS 2 Packages
**Problem**: Python import errors when running ROS 2 nodes.

**Solution**:
- Ensure proper `__init__.py` files in all directories
- Use correct import paths relative to package
- Check setup.py includes all necessary modules

```python
# File: robot_control_package/robot_control_package/__init__.py
# This file makes the directory a Python package
# It can be empty or contain package initialization code

# Example of what might go in __init__.py:
from .utils import RobotState, calculate_velocity, PIDController
from .robot_controller import RobotController

__all__ = [
    'RobotState',
    'calculate_velocity',
    'PIDController',
    'RobotController'
]
```

### Pitfall 2: Package Dependencies Not Resolved
**Problem**: Missing dependencies cause runtime errors.

**Solution**:
- List all dependencies in package.xml
- Use correct dependency types (build_depend, exec_depend, test_depend)
- Test builds in clean environments

### Pitfall 3: Entry Point Configuration Issues
**Problem**: Console scripts don't work due to incorrect entry point configuration.

**Solution**:
- Use correct format in setup.py: `'executable_name = package.module:function'`
- Ensure the function has the correct signature
- Test the executable after building

## Review Questions

1. What are the essential files needed for a ROS 2 Python package?
2. Explain the difference between build_depend and exec_depend in package.xml.
3. How do you create console scripts that become executable ROS 2 nodes?
4. What is the purpose of the setup.py file in ament_python packages?
5. How can you organize code in a ROS 2 package for better maintainability?

## Project Assignment: Complete Robot Control Package

Create a complete robot control package that includes:
1. A main controller node with safety features
2. A sensor processing node that fuses multiple sensor inputs
3. A behavior management node that selects appropriate behaviors
4. A monitoring node that tracks system health
5. Proper configuration files and launch files
6. Unit tests for critical components

Your package should:
- Follow ROS 2 best practices for Python packages
- Include proper error handling and logging
- Use appropriate QoS settings
- Be organized in a modular, reusable way
- Include documentation and examples

## Further Resources

- [ROS 2 Python Packages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [ament_python Documentation](https://github.com/ament/ament_python)
- [ROS 2 Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [ROS 2 Testing](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Unit-Testing-Python.html)

:::info
Remember that well-structured ROS 2 packages are essential for maintainable and reusable robotic systems. Take time to organize your code properly with clear separation of concerns and appropriate abstraction layers.
:::