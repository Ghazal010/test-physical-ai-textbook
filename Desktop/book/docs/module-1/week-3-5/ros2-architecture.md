---
title: Week 3-5 - ROS 2 Architecture
description: Understanding ROS 2 architecture and core concepts
sidebar_position: 1
---

# Week 3-5: ROS 2 Architecture

## Learning Objectives
- Master ROS 2 architecture and core concepts
- Understand the differences between ROS 1 and ROS 2
- Learn about DDS (Data Distribution Service) as the backbone of ROS 2
- Implement basic ROS 2 communication patterns

## Prerequisites Check
- Completion of Week 1-2 content
- Basic Python programming knowledge
- Understanding of client-server concepts

## Theoretical Concepts: ROS 2 Architecture

### What is ROS 2?

ROS 2 (Robot Operating System 2) is the second generation of the Robot Operating System, designed for production robotics applications. Unlike ROS 1, which was built on a custom peer-to-peer communication system, ROS 2 is built on DDS (Data Distribution Service), providing improved reliability, security, and real-time capabilities.

### Key Differences from ROS 1

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Communication | Custom peer-to-peer | DDS-based |
| Middleware | roscpp/rosjava/ros... | DDS implementations |
| Security | None | Built-in security |
| Real-time | Limited | Real-time capable |
| Multi-robot | Complex setup | Native support |
| Lifecycle | No formal lifecycle | Lifecycle nodes |
| Deployment | Research-focused | Production-ready |

### DDS (Data Distribution Service)

DDS is an OMG (Object Management Group) standard for real-time, high-performance data distribution. In ROS 2, DDS provides:

- **Data-centricity**: Data is king; systems are organized around data rather than functions
- **Quality of Service (QoS)**: Configurable policies for reliability, durability, etc.
- **Discovery**: Automatic discovery of participants in the network
- **Language and platform independence**: Works across different languages and OSes

### Core Architecture Components

#### Nodes
Nodes are the fundamental units of computation in ROS 2. Each node:
- Runs a single process
- Communicates with other nodes through topics, services, or actions
- Has a unique name within the ROS graph
- Can be written in any supported language (C++, Python, etc.)

#### Topics
Topics enable asynchronous, many-to-many communication between nodes:
- Publisher-subscriber pattern
- Data is published to named topics
- Multiple nodes can subscribe to the same topic
- Decouples publishers from subscribers in time and space

#### Services
Services provide synchronous, request-response communication:
- Client-server pattern
- Request-response model
- Blocking calls until response is received
- Good for operations that must complete before continuing

#### Actions
Actions provide asynchronous request-goal-result communication:
- Goal-request-result pattern
- Supports long-running operations with feedback
- Can be preempted before completion
- Ideal for navigation and manipulation tasks

## Step-by-Step Tutorials: ROS 2 Setup and Basic Communication

### Tutorial 1: Setting up Your First ROS 2 Workspace

Let's create a basic ROS 2 workspace and set up your development environment:

```bash
# Create a workspace directory
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace

# Source ROS 2 installation (assuming Humble Hawksbill)
source /opt/ros/humble/setup.bash

# Create a basic package
cd src
ros2 pkg create --build-type ament_python my_first_package --dependencies rclpy std_msgs

# Navigate to the new package
cd my_first_package
```

### Tutorial 2: Creating Your First Publisher Node

Create a simple publisher that sends messages to a topic:

```python
# File: my_first_package/my_first_package/simple_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    """
    A simple publisher node that sends messages to a topic
    """

    def __init__(self):
        super().__init__('simple_publisher')

        # Create a publisher
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create a timer to publish messages at regular intervals
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to keep track of messages
        self.i = 0

        self.get_logger().info('Simple Publisher Node has started')

    def timer_callback(self):
        """
        Callback function that publishes messages
        """
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    simple_publisher = SimplePublisher()

    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        simple_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Tutorial 3: Creating Your First Subscriber Node

Create a subscriber that receives messages from the publisher:

```python
# File: my_first_package/my_first_package/simple_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    """
    A simple subscriber node that receives messages from a topic
    """

    def __init__(self):
        super().__init__('simple_subscriber')

        # Create a subscription
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Simple Subscriber Node has started')

    def listener_callback(self, msg):
        """
        Callback function that processes received messages
        """
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    simple_subscriber = SimpleSubscriber()

    try:
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        simple_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Tutorial 4: Setting up the Package for Execution

Update the package setup files to make your nodes executable:

```python
# File: my_first_package/setup.py
from setuptools import find_packages, setup

package_name = 'my_first_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A simple ROS 2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_first_package.simple_publisher:main',
            'simple_subscriber = my_first_package.simple_subscriber:main',
        ],
    },
)
```

### Tutorial 5: Building and Running Your Nodes

Build and run your ROS 2 nodes:

```bash
# Navigate to your workspace
cd ~/ros2_workspace

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build your package
colcon build --packages-select my_first_package

# Source the newly built package
source install/setup.bash

# Run the publisher in one terminal
ros2 run my_first_package simple_publisher

# Run the subscriber in another terminal
ros2 run my_first_package simple_subscriber
```

## Code Examples with Explanations

### Example 1: Advanced Publisher with Custom Message Types

```python
# File: my_first_package/my_first_package/advanced_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import math
import time

class AdvancedPublisher(Node):
    """
    An advanced publisher that sends complex data structures
    """

    def __init__(self):
        super().__init__('advanced_publisher')

        # Create multiple publishers for different data types
        self.position_publisher = self.create_publisher(Point, 'robot_position', 10)
        self.sensor_publisher = self.create_publisher(Float64MultiArray, 'sensor_readings', 10)

        # Create a timer with variable rate
        self.timer = self.create_timer(0.1, self.publish_data)

        self.time_offset = time.time()
        self.get_logger().info('Advanced Publisher Node has started')

    def publish_data(self):
        """
        Publish multiple types of data
        """
        current_time = time.time() - self.time_offset

        # Publish robot position (simulated circular motion)
        position_msg = Point()
        position_msg.x = 2.0 * math.cos(current_time)
        position_msg.y = 2.0 * math.sin(current_time)
        position_msg.z = 0.0
        self.position_publisher.publish(position_msg)

        # Publish sensor readings (simulated sensor array)
        sensor_msg = Float64MultiArray()
        sensor_values = []
        for i in range(8):  # 8 sensors
            # Simulate sensor reading with some variation
            base_value = 1.0 + 0.1 * math.sin(current_time + i)
            noise = 0.05 * (2 * (i % 2) - 1)  # Simple noise pattern
            sensor_values.append(base_value + noise)
        sensor_msg.data = sensor_values
        self.sensor_publisher.publish(sensor_msg)

        self.get_logger().info(f'Published position: ({position_msg.x:.2f}, {position_msg.y:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedPublisher()

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

### Example 2: Service Server Implementation

```python
# File: my_first_package/my_first_package/simple_service_server.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SimpleServiceServer(Node):
    """
    A simple service server that adds two integers
    """

    def __init__(self):
        super().__init__('simple_service_server')

        # Create a service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Simple Service Server has started')

    def add_two_ints_callback(self, request, response):
        """
        Callback function for the service
        """
        response.sum = request.a + request.b
        self.get_logger().info(f'Request received: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SimpleServiceServer()

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

### Example 3: Service Client Implementation

```python
# File: my_first_package/my_first_package/simple_service_client.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SimpleServiceClient(Node):
    """
    A simple service client that calls the add_two_ints service
    """

    def __init__(self):
        super().__init__('simple_service_client')

        # Create a client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        """
        Send a request to the service
        """
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    node = SimpleServiceClient()

    # Send a request
    future = node.send_request(2, 3)

    try:
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        node.get_logger().info(f'Result: {response.sum}')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 4: Action Server Implementation

```python
# File: my_first_package/my_first_package/simple_action_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class SimpleActionServer(Node):
    """
    A simple action server that calculates Fibonacci sequence
    """

    def __init__(self):
        super().__init__('simple_action_server')

        # Create an action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

        self.get_logger().info('Simple Action Server has started')

    def execute_callback(self, goal_handle):
        """
        Execute the action callback
        """
        self.get_logger().info('Executing goal...')

        # Create feedback and result messages
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Check if the goal should be cancelled
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            result = Fibonacci.Result()
            result.sequence = []
            return result

        # Generate the Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled during execution')
                result = Fibonacci.Result()
                result.sequence = feedback_msg.sequence
                return result

            # Update feedback
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

        # Complete the goal
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')

        return result

def main(args=None):
    rclpy.init(args=args)
    node = SimpleActionServer()

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

### Exercise 1: Temperature Monitoring System

Create a system that monitors temperature sensors and alerts when thresholds are exceeded:

```python
# File: my_first_package/my_first_package/temp_monitor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String

class TemperatureMonitor(Node):
    """
    Monitor temperature sensors and publish alerts
    """

    def __init__(self):
        super().__init__('temperature_monitor')

        # Subscribe to temperature topic
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10
        )

        # Publish alerts
        self.alert_publisher = self.create_publisher(String, 'temp_alerts', 10)

        # Set temperature thresholds
        self.high_threshold = 30.0  # degrees Celsius
        self.low_threshold = 10.0   # degrees Celsius

        self.get_logger().info('Temperature Monitor Node has started')

    def temperature_callback(self, msg):
        """
        Process temperature readings
        """
        temp = msg.data

        if temp > self.high_threshold:
            alert_msg = String()
            alert_msg.data = f'HIGH TEMP ALERT: {temp:.2f}C > {self.high_threshold}C'
            self.alert_publisher.publish(alert_msg)
            self.get_logger().warn(alert_msg.data)
        elif temp < self.low_threshold:
            alert_msg = String()
            alert_msg.data = f'LOW TEMP ALERT: {temp:.2f}C < {self.low_threshold}C'
            self.alert_publisher.publish(alert_msg)
            self.get_logger().warn(alert_msg.data)
        else:
            self.get_logger().info(f'Temperature normal: {temp:.2f}C')

class TemperatureSimulator(Node):
    """
    Simulate temperature sensor readings
    """

    def __init__(self):
        super().__init__('temperature_simulator')

        # Create publisher for temperature readings
        self.publisher = self.create_publisher(Float32, 'temperature', 10)

        # Create timer to publish simulated readings
        self.timer = self.create_timer(1.0, self.publish_temperature)

        # Simulate temperature with some variation
        self.base_temp = 20.0
        self.variation = 0.0
        self.get_logger().info('Temperature Simulator Node has started')

    def publish_temperature(self):
        """
        Publish simulated temperature readings
        """
        import random

        # Add some random variation
        self.variation += random.uniform(-0.5, 0.5)
        self.variation = max(-5.0, min(5.0, self.variation))  # Limit variation

        temp = self.base_temp + self.variation + random.uniform(-1.0, 1.0)

        msg = Float32()
        msg.data = temp
        self.publisher.publish(msg)

        self.get_logger().info(f'Published temperature: {temp:.2f}C')

def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    monitor = TemperatureMonitor()
    simulator = TemperatureSimulator()

    try:
        # Create an executor to run both nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(monitor)
        executor.add_node(simulator)

        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 2: Robot Command Interface

Create a system that accepts high-level commands and translates them to low-level motor commands:

```python
# File: my_first_package/my_first_package/robot_commander.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class RobotCommander(Node):
    """
    Translate high-level commands to low-level motor commands
    """

    def __init__(self):
        super().__init__('robot_commander')

        # Subscribe to high-level commands
        self.command_subscription = self.create_subscription(
            String,
            'high_level_commands',
            self.command_callback,
            10
        )

        # Publish low-level motor commands
        self.motor_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info('Robot Commander Node has started')

    def command_callback(self, msg):
        """
        Translate high-level commands to motor commands
        """
        command = msg.data.lower()

        # Create motor command message
        motor_cmd = Twist()

        if command == 'forward':
            motor_cmd.linear.x = 0.5  # Move forward at 0.5 m/s
            self.get_logger().info('Moving forward')
        elif command == 'backward':
            motor_cmd.linear.x = -0.5  # Move backward at 0.5 m/s
            self.get_logger().info('Moving backward')
        elif command == 'left':
            motor_cmd.angular.z = 0.5  # Turn left at 0.5 rad/s
            self.get_logger().info('Turning left')
        elif command == 'right':
            motor_cmd.angular.z = -0.5  # Turn right at 0.5 rad/s
            self.get_logger().info('Turning right')
        elif command == 'stop':
            # Stop all movement (velocities remain 0)
            self.get_logger().info('Stopping')
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return

        # Publish the motor command
        self.motor_publisher.publish(motor_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RobotCommander()

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

### Pitfall 1: Node Naming Conflicts
**Problem**: Multiple nodes with the same name cause conflicts in the ROS graph.

**Solution**:
- Use unique node names
- Implement node name parameters
- Use namespaces for organization

```python
def main(args=None):
    rclpy.init(args=args)

    # Get node name from command line arguments or use default
    node_name = 'robot_commander'
    if len(sys.argv) > 1:
        node_name = f"{node_name}_{sys.argv[1]}"  # Add unique identifier

    node = RobotCommander(node_name)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Pitfall 2: Memory Leaks with Callbacks
**Problem**: Improper handling of subscriptions and publications can lead to memory leaks.

**Solution**:
- Always destroy nodes properly
- Use context managers where possible
- Monitor resource usage

```python
class ProperlyManagedNode(Node):
    """
    Example of proper resource management in a ROS 2 node
    """

    def __init__(self):
        super().__init__('properly_managed_node')

        # Store references to subscriptions and publishers
        self.subscriber = self.create_subscription(
            String,
            'topic',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(String, 'output_topic', 10)

        # Create timers
        self.timer = self.create_timer(0.1, self.timer_callback)

    def destroy_node(self):
        """
        Properly clean up resources
        """
        # Cancel timers
        if self.timer:
            self.timer.cancel()

        # Destroy subscriptions and publishers
        if self.subscriber:
            self.subscriber.destroy()

        if self.publisher:
            self.publisher.destroy()

        # Call parent destroy method
        super().destroy_node()
```

### Pitfall 3: Quality of Service Mismatch
**Problem**: Publishers and subscribers with incompatible QoS settings don't communicate.

**Solution**:
- Understand QoS policies
- Match QoS settings between publishers and subscribers
- Use appropriate QoS for your use case

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Define QoS for real-time critical data
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)

# Use consistent QoS in publisher and subscriber
publisher = self.create_publisher(String, 'critical_topic', qos_profile)
subscription = self.create_subscription(String, 'critical_topic', callback, qos_profile)
```

## Review Questions

1. What is the main difference between ROS 1 and ROS 2 in terms of communication?
2. Explain the publisher-subscriber communication pattern in ROS 2.
3. What is DDS and why is it important for ROS 2?
4. Describe the differences between topics, services, and actions.
5. What are Quality of Service (QoS) policies and why are they important?

## Project Assignment: Simple Robot Control System

Create a complete robot control system that includes:
1. A sensor node that publishes simulated sensor data
2. A processing node that interprets sensor data
3. A command node that sends movement commands
4. A safety node that monitors and enforces safety constraints

Your system should:
- Use proper ROS 2 communication patterns
- Include error handling and logging
- Implement at least one service for configuration
- Use appropriate QoS settings for different data types
- Include a launch file to start all nodes together

## Further Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [DDS Specification](https://www.omg.org/spec/DDS/)
- [ROS 2 Design](https://design.ros2.org/)
- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)

:::warning
Remember that ROS 2 is designed for production robotics. Always consider safety, reliability, and real-time constraints when designing your robotic systems.
:::