---
title: Week 3-5 - Nodes, Topics, and Services
description: Deep dive into ROS 2 communication patterns and node management
sidebar_position: 2
---

# Week 3-5: Nodes, Topics, and Services

## Learning Objectives
- Create and manage ROS 2 nodes effectively
- Implement publisher-subscriber communication patterns
- Build custom ROS 2 services for request-response communication
- Understand Quality of Service (QoS) policies and their impact

## Prerequisites Check
- Understanding of basic ROS 2 architecture
- Basic Python programming skills
- Completion of ROS 2 architecture content

## Theoretical Concepts: Nodes, Topics, and Services

### Nodes in Depth

A ROS 2 node is an executable that uses ROS 2 client library to communicate with other nodes. Each node:
- Runs a single process
- Contains at least one publisher, subscriber, service, client, or action
- Has a unique name within the ROS graph
- Can be written in any supported language
- Manages its own lifecycle and resources

**Node Lifecycle**:
1. **Initialization**: Node is created and initialized
2. **Configuration**: Parameters and interfaces are configured
3. **Activation**: Node becomes active and starts processing
4. **Deactivation**: Node stops processing but remains configured
5. **Cleanup**: Resources are released

### Topics and Publishers/Subscribers

Topics enable asynchronous, decoupled communication using the publish-subscribe pattern:

**Publishers**:
- Send messages to named topics
- Don't know who (if anyone) is subscribed
- Can have multiple publishers for the same topic
- Messages are sent immediately when published

**Subscribers**:
- Receive messages from named topics
- Don't know who published the messages
- Can have multiple subscribers for the same topic
- Process messages in callback functions

**Message Types**:
- Defined using `.msg` files
- Generated for different languages
- Include primitive types and custom structures
- Serialized for network transmission

### Services and Clients

Services provide synchronous request-response communication:

**Services**:
- Request-response pattern
- Blocking until response is received
- One client talks to one server at a time
- Good for operations that must complete before continuing

**Message Types**:
- Defined using `.srv` files
- Include both request and response parts
- Request part: parameters sent by client
- Response part: return values sent by server

### Quality of Service (QoS) Policies

QoS policies allow you to configure communication behavior:

**Reliability Policy**:
- `RELIABLE`: All messages are delivered (with retries)
- `BEST_EFFORT`: Messages are sent without guarantees

**Durability Policy**:
- `TRANSIENT_LOCAL`: Messages are stored for late-joining subscribers
- `VOLATILE`: Messages are not stored, only sent to current subscribers

**History Policy**:
- `KEEP_LAST`: Store only the most recent messages
- `KEEP_ALL`: Store all messages (use with caution)

**Depth**: Number of messages to store in history queue

## Step-by-Step Tutorials: Node Creation and Communication

### Tutorial 1: Advanced Node Implementation

Let's create a more sophisticated node with parameters and multiple interfaces:

```python
# File: my_first_package/my_first_package/advanced_node.py
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Int32
from example_interfaces.srv import SetBool

class AdvancedNode(Node):
    """
    An advanced node demonstrating parameters, multiple interfaces, and QoS
    """

    def __init__(self):
        super().__init__('advanced_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('sensor_frequency', 10)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('robot_name').value
        self.sensor_frequency = self.get_parameter('sensor_frequency').value

        # Create QoS profile for different types of data
        # For critical control data, use reliable communication
        control_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # For sensor data, best effort might be acceptable
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Create publishers
        self.status_publisher = self.create_publisher(String, 'robot_status', control_qos)
        self.sensor_publisher = self.create_publisher(Int32, 'sensor_data', sensor_qos)

        # Create subscribers
        self.command_subscription = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            control_qos
        )

        # Create service
        self.service = self.create_service(
            SetBool,
            'enable_robot',
            self.enable_robot_callback
        )

        # Create timer for periodic tasks
        self.timer = self.create_timer(1.0/self.sensor_frequency, self.sensor_timer_callback)

        # Internal state
        self.is_enabled = True
        self.command_history = []

        self.get_logger().info(
            f'Advanced Node initialized for robot: {self.robot_name}, '
            f'max speed: {self.max_speed}, frequency: {self.sensor_frequency}Hz'
        )

    def command_callback(self, msg):
        """
        Handle incoming commands
        """
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Add to history
        self.command_history.append(command)
        if len(self.command_history) > 10:  # Keep only last 10 commands
            self.command_history.pop(0)

        # Process command
        if command == 'stop':
            self.stop_robot()
        elif command == 'reset':
            self.reset_robot()
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def enable_robot_callback(self, request, response):
        """
        Service callback to enable/disable robot
        """
        self.is_enabled = request.data
        response.success = True
        response.message = f'Robot {"enabled" if self.is_enabled else "disabled"}'

        self.get_logger().info(response.message)
        return response

    def sensor_timer_callback(self):
        """
        Publish sensor data periodically
        """
        if not self.is_enabled:
            return

        # Simulate sensor reading
        import random
        sensor_value = random.randint(0, 100)

        msg = Int32()
        msg.data = sensor_value
        self.sensor_publisher.publish(msg)

        # Also publish status
        status_msg = String()
        status_msg.data = f'{self.robot_name}: OK, sensor={sensor_value}'
        self.status_publisher.publish(status_msg)

    def stop_robot(self):
        """
        Stop robot operations
        """
        self.get_logger().info('Stopping robot...')
        # In a real robot, this would send stop commands to actuators

    def reset_robot(self):
        """
        Reset robot to initial state
        """
        self.get_logger().info('Resetting robot...')
        # In a real robot, this would reset all systems

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedNode()

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

### Tutorial 2: Custom Message Types

Create custom message types for your application:

```bash
# Create a new package for custom messages
cd ~/ros2_workspace/src
ros2 pkg create --build-type ament_cmake custom_messages

# Create the message directory
mkdir -p custom_messages/msg
mkdir -p custom_messages/srv
```

Create a custom message file:

```# File: custom_messages/msg/RobotPose.msg
# Custom message for robot pose information
float64 x
float64 y
float64 theta
string robot_name
int32 battery_level
bool is_charging
```

Create a custom service file:

```# File: custom_messages/srv/NavigationGoal.srv
# Custom service for navigation goals
float64 x
float64 y
float64 theta
string description
---
bool success
string message
float64 time_to_complete
```

Update the CMakeLists.txt:

```cmake
# File: custom_messages/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(custom_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define message files
set(msg_files
  "msg/RobotPose.msg"
)

# Define service files
set(srv_files
  "srv/NavigationGoal.srv"
)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()
```

### Tutorial 3: Publisher with Custom Message

```python
# File: custom_messages/custom_messages/custom_publisher.py
import rclpy
from rclpy.node import Node
from custom_messages.msg import RobotPose
import math
import time

class CustomMessagePublisher(Node):
    """
    Publisher using custom message type
    """

    def __init__(self):
        super().__init__('custom_message_publisher')

        # Create publisher with custom message type
        self.publisher = self.create_publisher(RobotPose, 'robot_pose', 10)

        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.publish_pose)

        self.time_offset = time.time()
        self.battery_level = 100  # Start with full battery
        self.is_charging = False

        self.get_logger().info('Custom Message Publisher has started')

    def publish_pose(self):
        """
        Publish robot pose with custom message
        """
        current_time = time.time() - self.time_offset

        # Simulate robot moving in a circle
        msg = RobotPose()
        msg.x = 2.0 * math.cos(current_time)
        msg.y = 2.0 * math.sin(current_time)
        msg.theta = current_time  # Robot orientation
        msg.robot_name = 'simulated_robot'
        msg.battery_level = max(0, self.battery_level)
        msg.is_charging = self.is_charging

        # Simulate battery drain
        if not self.is_charging:
            self.battery_level -= 0.01
            if self.battery_level <= 20:
                self.is_charging = True  # Robot goes to charging station
        else:
            self.battery_level = min(100, self.battery_level + 0.05)
            if self.battery_level >= 95:
                self.is_charging = False  # Fully charged, resume normal operation

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published pose: ({msg.x:.2f}, {msg.y:.2f}, {msg.theta:.2f}), '
            f'Battery: {msg.battery_level:.1f}%, Charging: {msg.is_charging}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = CustomMessagePublisher()

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

### Tutorial 4: Service Server with Custom Service

```python
# File: custom_messages/custom_messages/navigation_service.py
import rclpy
from rclpy.node import Node
from custom_messages.srv import NavigationGoal
import time
import random

class NavigationService(Node):
    """
    Navigation service server with custom service type
    """

    def __init__(self):
        super().__init__('navigation_service')

        # Create service with custom service type
        self.srv = self.create_service(
            NavigationGoal,
            'navigate_to_goal',
            self.navigate_to_goal_callback
        )

        self.get_logger().info('Navigation Service Server has started')

    def navigate_to_goal_callback(self, request, response):
        """
        Handle navigation requests
        """
        self.get_logger().info(
            f'Received navigation request to ({request.x}, {request.y}, {request.theta}): {request.description}'
        )

        # Simulate navigation process
        # In a real robot, this would involve path planning and execution
        time_to_complete = random.uniform(5.0, 15.0)  # Random time between 5-15 seconds

        # Simulate navigation process (could be interrupted)
        start_time = time.time()
        while time.time() - start_time < time_to_complete:
            # Check for cancellation or other conditions
            time.sleep(0.1)  # Small sleep to allow other operations

        # Navigation completed successfully
        response.success = True
        response.message = f'Navigation to ({request.x}, {request.y}) completed successfully'
        response.time_to_complete = time_to_complete

        self.get_logger().info(f'Navigation completed in {time_to_complete:.2f} seconds')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NavigationService()

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

## Code Examples with Explanations

### Example 1: Node with Lifecycle Management

```python
# File: my_first_package/my_first_package/lifecycle_node.py
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import String

class LifecycleExampleNode(LifecycleNode):
    """
    Example of a lifecycle node with proper state management
    """

    def __init__(self):
        super().__init__('lifecycle_example_node')

        # Initialize publishers, but don't create them yet
        self.pub = None
        self.timer = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Configure the node - create publishers, subscribers, etc.
        """
        self.get_logger().info(f'Configuring {self.get_name()}')

        # Create publisher
        self.pub = self.create_publisher(String, 'lifecycle_chatter', 10)

        # Create timer (but don't start it yet)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.timer.cancel()  # Don't start timer yet

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Activate the node - start timers, enable functionality
        """
        self.get_logger().info(f'Activating {self.get_name()}')

        # Start the timer
        self.timer.reset()

        # Call parent class method
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Deactivate the node - stop timers, disable functionality
        """
        self.get_logger().info(f'Deactivating {self.get_name()}')

        # Stop the timer
        self.timer.cancel()

        # Call parent class method
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Clean up the node - destroy publishers, subscribers, etc.
        """
        self.get_logger().info(f'Cleaning up {self.get_name()}')

        # Destroy publisher
        self.destroy_publisher(self.pub)
        self.pub = None

        # Destroy timer
        self.destroy_timer(self.timer)
        self.timer = None

        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        """
        Timer callback function
        """
        msg = String()
        msg.data = f'Lifecycle node message at {self.get_clock().now().nanoseconds}'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = LifecycleExampleNode()

    # In a real application, you would control the lifecycle manually
    # For this example, we'll just spin
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

### Example 2: Multi-Node System with Coordination

```python
# File: my_first_package/my_first_package/robot_coordinator.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from example_interfaces.srv import SetBool

class RobotCoordinator(Node):
    """
    Coordinate multiple robot subsystems
    """

    def __init__(self):
        super().__init__('robot_coordinator')

        # Publishers for different subsystems
        self.movement_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, 'coordinator_status', 10)

        # Subscriptions for monitoring
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_status',
            self.sensor_status_callback,
            10
        )

        self.navigation_subscription = self.create_subscription(
            String,
            'navigation_status',
            self.navigation_status_callback,
            10
        )

        # Services for subsystem control
        self.movement_client = self.create_client(SetBool, 'enable_movement')
        self.sensor_client = self.create_client(SetBool, 'enable_sensors')

        # Wait for services to be available
        while not self.movement_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Movement service not available, waiting...')

        while not self.sensor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Sensor service not available, waiting...')

        # Timer for coordination tasks
        self.timer = self.create_timer(1.0, self.coordination_callback)

        # Internal state
        self.movement_enabled = False
        self.sensors_enabled = False
        self.robot_mode = 'idle'  # idle, navigating, avoiding_obstacles, etc.

        self.get_logger().info('Robot Coordinator Node has started')

    def sensor_status_callback(self, msg):
        """
        Handle sensor status updates
        """
        self.get_logger().info(f'Sensor status: {msg.data}')

    def navigation_status_callback(self, msg):
        """
        Handle navigation status updates
        """
        self.get_logger().info(f'Navigation status: {msg.data}')

    def coordination_callback(self):
        """
        Main coordination logic
        """
        # Example coordination logic
        if self.robot_mode == 'idle':
            self.check_for_tasks()
        elif self.robot_mode == 'navigating':
            self.monitor_navigation()
        elif self.robot_mode == 'avoiding_obstacles':
            self.handle_obstacles()

        # Publish coordinator status
        status_msg = String()
        status_msg.data = f'Mode: {self.robot_mode}, Movement: {self.movement_enabled}, Sensors: {self.sensors_enabled}'
        self.status_publisher.publish(status_msg)

    def check_for_tasks(self):
        """
        Check if there are tasks to perform
        """
        # In a real system, this would check for goals, requests, etc.
        # For this example, we'll simulate a task occasionally
        import random
        if random.random() < 0.1:  # 10% chance per second
            self.start_navigation_task()

    def start_navigation_task(self):
        """
        Start a navigation task
        """
        self.get_logger().info('Starting navigation task')
        self.robot_mode = 'navigating'

        # Enable movement if not already enabled
        if not self.movement_enabled:
            self.enable_movement(True)

    def monitor_navigation(self):
        """
        Monitor navigation progress
        """
        # In a real system, this would check navigation progress
        # For this example, we'll simulate completion
        import random
        if random.random() < 0.05:  # 5% chance per second
            self.get_logger().info('Navigation task completed')
            self.robot_mode = 'idle'

    def enable_movement(self, enable: bool):
        """
        Enable or disable robot movement
        """
        request = SetBool.Request()
        request.data = enable

        future = self.movement_client.call_async(request)
        future.add_done_callback(self.movement_response_callback)

    def movement_response_callback(self, future):
        """
        Handle movement enable/disable response
        """
        try:
            response = future.result()
            if response.success:
                self.movement_enabled = response.message.startswith('enabled')
                self.get_logger().info(f'Movement {"enabled" if self.movement_enabled else "disabled"}')
            else:
                self.get_logger().error(f'Failed to enable movement: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotCoordinator()

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

### Example 3: Quality of Service Configuration

```python
# File: my_first_package/my_first_package/qos_examples.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Int32
import time

class QoSExamplesNode(Node):
    """
    Demonstrate different QoS configurations
    """

    def __init__(self):
        super().__init__('qos_examples_node')

        # Different QoS profiles for different use cases

        # Reliable communication for critical data
        self.critical_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Best effort for high-frequency sensor data
        self.sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Keep all for configuration data that must be preserved
        self.config_qos = QoSProfile(
            depth=100,  # Large depth for history
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL
        )

        # Publishers with different QoS
        self.critical_publisher = self.create_publisher(String, 'critical_data', self.critical_qos)
        self.sensor_publisher = self.create_publisher(Int32, 'sensor_stream', self.sensor_qos)
        self.config_publisher = self.create_publisher(String, 'config_data', self.config_qos)

        # Create timers for different data types
        self.critical_timer = self.create_timer(2.0, self.publish_critical_data)
        self.sensor_timer = self.create_timer(0.01, self.publish_sensor_data)  # High frequency
        self.config_timer = self.create_timer(10.0, self.publish_config_data)  # Low frequency

        # Counter for tracking
        self.critical_counter = 0
        self.sensor_counter = 0
        self.config_counter = 0

        self.get_logger().info('QoS Examples Node has started')

    def publish_critical_data(self):
        """
        Publish critical data with reliable QoS
        """
        msg = String()
        msg.data = f'Critical message #{self.critical_counter} at {time.time()}'
        self.critical_publisher.publish(msg)
        self.critical_counter += 1
        self.get_logger().info(f'Published critical data: {msg.data}')

    def publish_sensor_data(self):
        """
        Publish high-frequency sensor data with best-effort QoS
        """
        import random
        msg = Int32()
        msg.data = random.randint(0, 1000)
        self.sensor_publisher.publish(msg)
        self.sensor_counter += 1

        # Log every 100 messages to avoid spam
        if self.sensor_counter % 100 == 0:
            self.get_logger().info(f'Published sensor data #{self.sensor_counter}: {msg.data}')

    def publish_config_data(self):
        """
        Publish configuration data with TRANSIENT_LOCAL durability
        """
        msg = String()
        msg.data = f'Config message #{self.config_counter} - Robot settings updated'
        self.config_publisher.publish(msg)
        self.config_counter += 1
        self.get_logger().info(f'Published config data: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = QoSExamplesNode()

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

## Hands-On Exercises: Building Communication Systems

### Exercise 1: Sensor Fusion Node

Create a node that subscribes to multiple sensor topics and publishes fused information:

```python
# File: my_first_package/my_first_package/sensor_fusion.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Vector3
import statistics

class SensorFusionNode(Node):
    """
    Fuse data from multiple sensors to create a more reliable estimate
    """

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscriptions to different sensors
        self.temperature_sub1 = self.create_subscription(
            Float32,
            'temperature_sensor_1',
            self.temp1_callback,
            10
        )

        self.temperature_sub2 = self.create_subscription(
            Float32,
            'temperature_sensor_2',
            self.temp2_callback,
            10
        )

        self.temperature_sub3 = self.create_subscription(
            Float32,
            'temperature_sensor_3',
            self.temp3_callback,
            10
        )

        # Publisher for fused data
        self.fused_temp_publisher = self.create_publisher(Float32, 'fused_temperature', 10)
        self.status_publisher = self.create_publisher(String, 'fusion_status', 10)

        # Storage for sensor readings
        self.temp_readings = {
            'sensor1': [],
            'sensor2': [],
            'sensor3': []
        }

        # Parameters for fusion
        self.max_readings = 10  # Keep last 10 readings for each sensor
        self.confidence_threshold = 2.0  # Max deviation to trust sensor

        self.get_logger().info('Sensor Fusion Node has started')

    def temp1_callback(self, msg):
        self.update_sensor_reading('sensor1', msg.data)

    def temp2_callback(self, msg):
        self.update_sensor_reading('sensor2', msg.data)

    def temp3_callback(self, msg):
        self.update_sensor_reading('sensor3', msg.data)

    def update_sensor_reading(self, sensor_name, reading):
        """
        Update sensor reading and perform fusion
        """
        # Add new reading
        self.temp_readings[sensor_name].append(reading)

        # Keep only the most recent readings
        if len(self.temp_readings[sensor_name]) > self.max_readings:
            self.temp_readings[sensor_name] = self.temp_readings[sensor_name][-self.max_readings:]

        # Perform sensor fusion
        self.perform_fusion()

    def perform_fusion(self):
        """
        Perform sensor fusion using weighted average based on sensor reliability
        """
        # Calculate statistics for each sensor to assess reliability
        sensor_stats = {}
        for sensor, readings in self.temp_readings.items():
            if len(readings) > 0:
                mean_val = statistics.mean(readings)
                if len(readings) > 1:
                    stdev = statistics.stdev(readings) if len(readings) > 1 else 0
                else:
                    stdev = 0

                sensor_stats[sensor] = {
                    'mean': mean_val,
                    'stdev': stdev,
                    'reliability': 1.0 / (1.0 + stdev) if stdev > 0 else 1.0  # Higher reliability for lower variance
                }

        if len(sensor_stats) == 0:
            return

        # Calculate weighted average
        total_weight = sum(stats['reliability'] for stats in sensor_stats.values())
        if total_weight == 0:
            return

        weighted_sum = sum(
            stats['mean'] * stats['reliability']
            for stats in sensor_stats.values()
        )

        fused_temp = weighted_sum / total_weight

        # Publish fused result
        temp_msg = Float32()
        temp_msg.data = fused_temp
        self.fused_temp_publisher.publish(temp_msg)

        # Publish status
        status_msg = String()
        status_msg.data = f'Fused temp: {fused_temp:.2f}°C from {len(sensor_stats)} sensors'
        self.status_publisher.publish(status_msg)

        self.get_logger().info(f'Fused temperature: {fused_temp:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()

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

### Exercise 2: Service-Based Parameter Server

Create a service that manages parameters for multiple nodes:

```python
# File: my_first_package/my_first_package/parameter_server.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool, SetInt, SetFloat, GetBool, GetInt, GetFloat
from std_msgs.msg import String

class ParameterServer(Node):
    """
    Centralized parameter server using services
    """

    def __init__(self):
        super().__init__('parameter_server')

        # Store parameters in a dictionary
        self.parameters = {
            'robot.max_speed': 1.0,
            'robot.min_distance': 0.5,
            'sensor.range': 3.0,
            'navigation.planning_frequency': 5.0,
            'safety.enabled': True,
            'debug.level': 1
        }

        # Service servers for different parameter types
        self.set_bool_srv = self.create_service(SetBool, 'set_bool_param', self.set_bool_param)
        self.set_int_srv = self.create_service(SetInt, 'set_int_param', self.set_int_param)
        self.set_float_srv = self.create_service(SetFloat, 'set_float_param', self.set_float_param)

        self.get_bool_srv = self.create_service(GetBool, 'get_bool_param', self.get_bool_param)
        self.get_int_srv = self.create_service(GetInt, 'get_int_param', self.get_int_param)
        self.get_float_srv = self.create_service(GetFloat, 'get_float_param', self.get_float_param)

        # Publisher for parameter change notifications
        self.change_publisher = self.create_publisher(String, 'parameter_changes', 10)

        self.get_logger().info('Parameter Server has started')

    def set_bool_param(self, request, response):
        """
        Set a boolean parameter
        """
        param_name = request.message  # Using message field as parameter name
        param_value = request.data

        self.parameters[param_name] = param_value

        response.success = True
        response.message = f'Parameter {param_name} set to {param_value}'

        # Notify of parameter change
        self.publish_parameter_change(param_name, str(param_value))

        self.get_logger().info(f'Set bool parameter {param_name} = {param_value}')
        return response

    def set_int_param(self, request, response):
        """
        Set an integer parameter
        """
        param_name = request.message  # Using message field as parameter name
        param_value = request.data

        self.parameters[param_name] = param_value

        response.success = True
        response.message = f'Parameter {param_name} set to {param_value}'

        # Notify of parameter change
        self.publish_parameter_change(param_name, str(param_value))

        self.get_logger().info(f'Set int parameter {param_name} = {param_value}')
        return response

    def set_float_param(self, request, response):
        """
        Set a float parameter
        """
        param_name = request.message  # Using message field as parameter name
        param_value = request.data

        self.parameters[param_name] = param_value

        response.success = True
        response.message = f'Parameter {param_name} set to {param_value}'

        # Notify of parameter change
        self.publish_parameter_change(param_name, str(param_value))

        self.get_logger().info(f'Set float parameter {param_name} = {param_value}')
        return response

    def get_bool_param(self, request, response):
        """
        Get a boolean parameter
        """
        param_name = request.message  # Using message field as parameter name

        if param_name in self.parameters:
            response.success = True
            response.data = bool(self.parameters[param_name])
            response.message = f'Parameter {param_name} retrieved'
        else:
            response.success = False
            response.data = False
            response.message = f'Parameter {param_name} not found'

        self.get_logger().info(f'Get bool parameter {param_name} = {response.data}')
        return response

    def get_int_param(self, request, response):
        """
        Get an integer parameter
        """
        param_name = request.message  # Using message field as parameter name

        if param_name in self.parameters:
            response.success = True
            response.data = int(self.parameters[param_name])
            response.message = f'Parameter {param_name} retrieved'
        else:
            response.success = False
            response.data = 0
            response.message = f'Parameter {param_name} not found'

        self.get_logger().info(f'Get int parameter {param_name} = {response.data}')
        return response

    def get_float_param(self, request, response):
        """
        Get a float parameter
        """
        param_name = request.message  # Using message field as parameter name

        if param_name in self.parameters:
            response.success = True
            response.data = float(self.parameters[param_name])
            response.message = f'Parameter {param_name} retrieved'
        else:
            response.success = False
            response.data = 0.0
            response.message = f'Parameter {param_name} not found'

        self.get_logger().info(f'Get float parameter {param_name} = {response.data}')
        return response

    def publish_parameter_change(self, param_name, param_value):
        """
        Publish notification about parameter change
        """
        msg = String()
        msg.data = f'PARAM_CHANGE: {param_name}={param_value}'
        self.change_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterServer()

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

### Pitfall 1: Memory Management in Long-Running Nodes
**Problem**: Nodes that run for extended periods can accumulate memory usage.

**Solution**:
- Implement proper cleanup in callbacks
- Limit history and buffer sizes
- Use circular buffers for data storage

```python
class MemoryManagedNode(Node):
    """
    Example of memory management in ROS 2 nodes
    """

    def __init__(self):
        super().__init__('memory_managed_node')

        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.callback,
            10
        )

        # Use circular buffer to limit memory usage
        self.max_buffer_size = 100
        self.message_buffer = []

    def callback(self, msg):
        """
        Process message with memory management
        """
        # Add new message
        self.message_buffer.append(msg.data)

        # Maintain buffer size limit
        if len(self.message_buffer) > self.max_buffer_size:
            # Remove oldest messages
            self.message_buffer = self.message_buffer[-self.max_buffer_size:]
```

### Pitfall 2: Inconsistent QoS Settings
**Problem**: Publishers and subscribers with mismatched QoS policies don't communicate.

**Solution**:
- Document QoS requirements for each topic
- Use consistent QoS settings across the system
- Test with different QoS combinations

### Pitfall 3: Blocking Service Calls
**Problem**: Synchronous service calls can block the node's execution.

**Solution**:
- Use asynchronous service calls when possible
- Implement timeouts
- Handle service unavailability gracefully

```python
def call_service_async(self, client, request):
    """
    Call service asynchronously with timeout handling
    """
    future = client.call_async(request)

    # Add timeout handling
    timer = self.create_timer(5.0, lambda: self.handle_timeout(future))

    future.add_done_callback(
        lambda f: self.handle_service_response(f, timer)
    )

def handle_timeout(self, future):
    """
    Handle service call timeout
    """
    if not future.done():
        self.get_logger().warn('Service call timed out')
        future.set_exception(TimeoutError('Service call timed out'))

def handle_service_response(self, future, timer):
    """
    Handle service response
    """
    timer.cancel()  # Cancel timeout timer
    try:
        response = future.result()
        # Process response
    except Exception as e:
        self.get_logger().error(f'Service call failed: {e}')
```

## Review Questions

1. What are the key differences between ROS 2 nodes and traditional processes?
2. Explain the publisher-subscriber communication pattern and when to use it.
3. When would you choose services over topics for communication?
4. What are Quality of Service (QoS) policies and why are they important?
5. How do lifecycle nodes differ from regular nodes and when should you use them?

## Project Assignment: Multi-Node Robot System

Create a complete multi-node robot system that includes:
1. A sensor node that publishes sensor data
2. A processing node that fuses sensor information
3. A control node that makes decisions based on processed data
4. A parameter server that manages system configuration
5. A monitoring node that tracks system status

Your system should:
- Use appropriate QoS settings for different data types
- Include proper error handling and logging
- Implement both topic-based and service-based communication
- Use custom message types where appropriate
- Include a launch file to start the entire system

## Further Resources

- [ROS 2 QoS Examples](https://docs.ros.org/en/humble/Tutorials/Quality-Of-Service/)
- [ROS 2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [ROS 2 Parameters Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Introducing-Parameters.html)
- [ROS 2 Lifecycle Nodes](https://docs.ros.org/en/humble/Tutorials/Advanced-Launch/Using-Lifecycle-Nodes.html)

<details>
<summary>DDS and Middleware</summary>

DDS (Data Distribution Service) is the underlying middleware that powers ROS 2's communication. Understanding DDS concepts like DDS entities (Domain, Participant, Publisher, Subscriber, Topic, DataWriter, DataReader) can help you optimize your ROS 2 applications. DDS provides built-in support for discovery, data modeling, and QoS policies that make ROS 2 more robust and production-ready than ROS 1.

</details>