---
title: Week 3-5 - Publisher-Subscriber Tutorial
description: Comprehensive tutorial on implementing publisher-subscriber patterns in ROS 2
sidebar_position: 5
---

# Week 3-5: Publisher-Subscriber Tutorial

## Learning Objectives
- Implement robust publisher-subscriber communication patterns
- Handle various message types and data structures
- Design efficient message passing for real-time robotics applications
- Debug and optimize communication performance

## Prerequisites Check
- Understanding of basic ROS 2 concepts
- Knowledge of nodes, topics, and messages
- Basic Python programming skills

## Theoretical Concepts: Advanced Publisher-Subscriber Patterns

### Message Passing Fundamentals

In ROS 2, the publisher-subscriber pattern enables asynchronous, decoupled communication between nodes:

- **Publishers** send messages to named topics without knowing who (if anyone) is subscribed
- **Subscribers** receive messages from named topics without knowing who published them
- **Topics** serve as channels for message passing
- **Messages** are the data structures passed between nodes

### Quality of Service (QoS) Considerations

Different types of data require different QoS policies:

- **Reliability**: Whether all messages must be delivered (RELIABLE) or best-effort delivery is acceptable (BEST_EFFORT)
- **Durability**: Whether late-joining subscribers should receive previous messages (TRANSIENT_LOCAL) or only new ones (VOLATILE)
- **History**: How many messages to store (KEEP_LAST vs KEEP_ALL)
- **Depth**: Maximum number of messages in the queue

### Message Types and Serialization

ROS 2 supports various message types:
- Primitive types: int8, int16, int32, int64, uint8, uint16, uint32, uint64, float32, float64, string, bool
- Complex types: arrays, nested messages, constants
- Standard messages: geometry_msgs, sensor_msgs, nav_msgs, etc.

## Step-by-Step Tutorials: Advanced Communication Patterns

### Tutorial 1: Multi-Topic Publisher with Synchronization

Create a publisher that coordinates multiple related data streams:

```python
# File: robot_control_package/robot_control_package/multi_topic_publisher.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import BatteryState
import time
import threading

class MultiTopicPublisher(Node):
    """
    Publish coordinated data to multiple topics with proper timing
    """

    def __init__(self):
        super().__init__('multi_topic_publisher')

        # Define QoS profiles for different data types
        # Reliable for critical control data
        control_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Best effort for high-frequency sensor data
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publishers for different data types
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', control_qos)
        self.battery_publisher = self.create_publisher(BatteryState, 'battery_status', control_qos)
        self.status_publisher = self.create_publisher(String, 'robot_status', control_qos)
        self.imu_publisher = self.create_publisher(Vector3, 'imu_data', sensor_qos)
        self.distance_publisher = self.create_publisher(Float32, 'distance_to_obstacle', sensor_qos)

        # Timer for coordinated publishing
        self.publish_timer = self.create_timer(0.1, self.publish_coordinated_data)

        # Internal state tracking
        self.simulation_time = time.time()
        self.battery_level = 100.0
        self.velocity_command = Twist()
        self.obstacle_distance = 2.0

        self.get_logger().info('Multi-topic publisher initialized')

    def publish_coordinated_data(self):
        """
        Publish coordinated data to multiple topics
        """
        current_time = self.get_simulation_time()

        # Publish velocity command
        self.publish_velocity_command(current_time)

        # Publish battery status
        self.publish_battery_status(current_time)

        # Publish robot status
        self.publish_robot_status(current_time)

        # Publish IMU data (simulated)
        self.publish_imu_data(current_time)

        # Publish distance to obstacle
        self.publish_distance_data(current_time)

        # Update simulation state
        self.update_simulation_state()

    def publish_velocity_command(self, timestamp):
        """
        Publish velocity commands with coordination
        """
        # Simulate a velocity pattern (circle motion)
        self.velocity_command.linear.x = 0.5 + 0.2 * self.get_simulation_time() % 2
        self.velocity_command.angular.z = 0.3 * (self.get_simulation_time() % 4 - 2)

        self.velocity_publisher.publish(self.velocity_command)

    def publish_battery_status(self, timestamp):
        """
        Publish battery status with realistic discharge
        """
        battery_msg = BatteryState()
        battery_msg.header.stamp = timestamp
        battery_msg.header.frame_id = 'base_link'
        battery_msg.percentage = self.battery_level / 100.0  # Convert to 0-1 range
        battery_msg.voltage = 12.6 - (12.6 - 11.0) * (100 - self.battery_level) / 100.0
        battery_msg.current = -1.0  # Discharging
        battery_msg.charge = -1.0  # Not implemented
        battery_msg.capacity = -1.0  # Not implemented
        battery_msg.design_capacity = -1.0  # Not implemented
        battery_msg.percentage = self.battery_level / 100.0
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

        self.battery_publisher.publish(battery_msg)

    def publish_robot_status(self, timestamp):
        """
        Publish comprehensive robot status
        """
        status_msg = String()
        status_msg.data = (
            f'Time: {self.get_simulation_time():.2f}s, '
            f'Battery: {self.battery_level:.1f}%, '
            f'Vel: ({self.velocity_command.linear.x:.2f}, {self.velocity_command.angular.z:.2f}), '
            f'Dist: {self.obstacle_distance:.2f}m'
        )
        self.status_publisher.publish(status_msg)

    def publish_imu_data(self, timestamp):
        """
        Publish simulated IMU data
        """
        import random
        imu_msg = Vector3()
        imu_msg.x = 0.1 * random.uniform(-1, 1)  # Small acceleration in x
        imu_msg.y = 0.1 * random.uniform(-1, 1)  # Small acceleration in y
        imu_msg.z = 9.8 + 0.1 * random.uniform(-1, 1)  # Gravity + small variation

        self.imu_publisher.publish(imu_msg)

    def publish_distance_data(self, timestamp):
        """
        Publish distance to obstacle (simulated)
        """
        distance_msg = Float32()
        # Simulate distance that varies over time
        base_distance = 2.0 + 0.5 * (self.get_simulation_time() % 10 - 5) / 5
        noise = 0.1 * (2 * (self.get_simulation_time() * 10) % 2 - 1)  # Simple noise
        distance_msg.data = max(0.1, base_distance + noise)  # Ensure positive distance

        self.distance_publisher.publish(distance_msg)

    def update_simulation_state(self):
        """
        Update internal simulation state
        """
        # Discharge battery over time
        self.battery_level = max(0, self.battery_level - 0.01)

        # Simulate obstacle distance variation
        import math
        self.obstacle_distance = 2.0 + 0.5 * math.sin(self.get_simulation_time())

    def get_simulation_time(self):
        """
        Get current simulation time as ROS message
        """
        current_time = time.time()
        from builtin_interfaces.msg import Time
        ros_time = Time()
        ros_time.sec = int(current_time)
        ros_time.nanosec = int((current_time - int(current_time)) * 1e9)
        return ros_time

def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down multi-topic publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Tutorial 2: Advanced Subscriber with Message Filtering

Create a subscriber that processes and filters incoming messages:

```python
# File: robot_control_package/robot_control_package/advanced_subscriber.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from collections import deque
import statistics
import time

class AdvancedSubscriber(Node):
    """
    Advanced subscriber with message filtering, buffering, and analysis
    """

    def __init__(self):
        super().__init__('advanced_subscriber')

        # QoS profiles
        control_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        sensor_qos = QoSProfile(
            depth=20,  # Larger buffer for sensor data
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # Subscriptions
        self.velocity_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            control_qos
        )

        self.battery_subscription = self.create_subscription(
            BatteryState,
            'battery_status',
            self.battery_callback,
            control_qos
        )

        self.status_subscription = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            control_qos
        )

        self.distance_subscription = self.create_subscription(
            Float32,
            'distance_to_obstacle',
            self.distance_callback,
            sensor_qos
        )

        # Publishers for processed data
        self.filtered_distance_publisher = self.create_publisher(Float32, 'filtered_distance', control_qos)
        self.analyzed_status_publisher = self.create_publisher(String, 'analyzed_status', control_qos)

        # Data buffers for analysis
        self.velocity_buffer = deque(maxlen=10)
        self.battery_buffer = deque(maxlen=20)
        self.distance_buffer = deque(maxlen=10)
        self.status_buffer = deque(maxlen=5)

        # Timers for analysis
        self.analysis_timer = self.create_timer(1.0, self.perform_analysis)

        # Internal state
        self.last_velocity_time = time.time()
        self.velocity_change_threshold = 0.1
        self.low_battery_threshold = 20.0

        self.get_logger().info('Advanced subscriber initialized')

    def velocity_callback(self, msg):
        """
        Handle velocity messages with filtering and analysis
        """
        # Add to buffer
        self.velocity_buffer.append({
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z,
            'timestamp': self.get_clock().now()
        })

        # Check for significant velocity changes
        if len(self.velocity_buffer) >= 2:
            prev_vel = self.velocity_buffer[-2]
            curr_vel = self.velocity_buffer[-1]

            linear_change = abs(curr_vel['linear_x'] - prev_vel['linear_x'])
            angular_change = abs(curr_vel['angular_z'] - prev_vel['angular_z'])

            if linear_change > self.velocity_change_threshold or angular_change > self.velocity_change_threshold:
                self.get_logger().info(
                    f'Significant velocity change: '
                    f'Linear: {prev_vel["linear_x"]:.2f} -> {curr_vel["linear_x"]:.2f}, '
                    f'Angular: {prev_vel["angular_z"]:.2f} -> {curr_vel["angular_z"]:.2f}'
                )

        self.last_velocity_time = time.time()

    def battery_callback(self, msg):
        """
        Handle battery status messages with threshold checking
        """
        # Add to buffer
        battery_data = {
            'level': msg.percentage * 100,  # Convert back to percentage
            'voltage': msg.voltage,
            'timestamp': self.get_clock().now()
        }
        self.battery_buffer.append(battery_data)

        # Check for low battery
        if battery_data['level'] < self.low_battery_threshold:
            self.get_logger().warn(f'LOW BATTERY: {battery_data["level"]:.1f}%')

    def status_callback(self, msg):
        """
        Handle status messages
        """
        # Add to buffer
        status_data = {
            'message': msg.data,
            'timestamp': self.get_clock().now()
        }
        self.status_buffer.append(status_data)

    def distance_callback(self, msg):
        """
        Handle distance messages with filtering
        """
        # Add to buffer
        self.distance_buffer.append({
            'distance': msg.data,
            'timestamp': self.get_clock().now()
        })

        # Apply simple filtering (moving average)
        if len(self.distance_buffer) >= 3:
            recent_distances = [d['distance'] for d in list(self.distance_buffer)[-3:]]
            filtered_distance = statistics.mean(recent_distances)

            # Publish filtered distance
            filtered_msg = Float32()
            filtered_msg.data = filtered_distance
            self.filtered_distance_publisher.publish(filtered_msg)

    def perform_analysis(self):
        """
        Perform periodic analysis of buffered data
        """
        analysis_results = []

        # Analyze velocity patterns
        if len(self.velocity_buffer) >= 5:
            linear_velocities = [v['linear_x'] for v in self.velocity_buffer]
            angular_velocities = [v['angular_z'] for v in self.velocity_buffer]

            avg_linear = statistics.mean(linear_velocities)
            avg_angular = statistics.mean(angular_velocities)
            linear_variance = statistics.variance(linear_velocities) if len(linear_velocities) > 1 else 0

            analysis_results.append(f'Avg Vel: L={avg_linear:.2f}, A={avg_angular:.2f}, Var={linear_variance:.4f}')

        # Analyze battery trends
        if len(self.battery_buffer) >= 5:
            battery_levels = [b['level'] for b in self.battery_buffer]
            battery_trend = battery_levels[-1] - battery_levels[0]  # Change over period

            analysis_results.append(f'Battery trend: {battery_trend:.2f}% over period')

        # Analyze distance patterns
        if len(self.distance_buffer) >= 5:
            distances = [d['distance'] for d in self.distance_buffer]
            avg_distance = statistics.mean(distances)
            min_distance = min(distances)

            analysis_results.append(f'Distance: Avg={avg_distance:.2f}m, Min={min_distance:.2f}m')

        # Publish analysis results
        if analysis_results:
            analysis_msg = String()
            analysis_msg.data = ' | '.join(analysis_results)
            self.analyzed_status_publisher.publish(analysis_msg)
            self.get_logger().info(f'Analysis: {analysis_msg.data}')

    def check_health_status(self):
        """
        Check overall system health based on buffered data
        """
        health_issues = []

        # Check for velocity issues
        if time.time() - self.last_velocity_time > 5.0:
            health_issues.append('No velocity commands for 5s')

        # Check battery level
        if self.battery_buffer and self.battery_buffer[-1]['level'] < self.low_battery_threshold:
            health_issues.append(f'Battery low: {self.battery_buffer[-1]["level"]:.1f}%')

        # Check distance to obstacles
        if self.distance_buffer and min(d['distance'] for d in self.distance_buffer) < 0.5:
            health_issues.append('Obstacle proximity detected')

        return health_issues

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down advanced subscriber...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Tutorial 3: Message Aggregation Node

Create a node that aggregates messages from multiple sources:

```python
# File: robot_control_package/robot_control_package/message_aggregator.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, String, Int32
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import BatteryState, LaserScan
from custom_messages.msg import RobotPose  # Assuming we have this custom message
from collections import defaultdict, deque
import time

class MessageAggregator(Node):
    """
    Aggregate messages from multiple sources into comprehensive reports
    """

    def __init__(self):
        super().__init__('message_aggregator')

        # QoS profiles
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        sensor_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # Subscriptions for various data sources
        self.velocity_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            reliable_qos
        )

        self.battery_subscription = self.create_subscription(
            BatteryState,
            'battery_status',
            self.battery_callback,
            reliable_qos
        )

        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            sensor_qos
        )

        self.imu_subscription = self.create_subscription(
            Vector3,
            'imu_data',
            self.imu_callback,
            sensor_qos
        )

        self.distance_subscription = self.create_subscription(
            Float32,
            'distance_to_obstacle',
            self.distance_callback,
            sensor_qos
        )

        # Publishers
        self.comprehensive_status_publisher = self.create_publisher(
            String, 'comprehensive_status', reliable_qos
        )

        self.robot_pose_publisher = self.create_publisher(
            RobotPose, 'aggregated_robot_pose', reliable_qos
        )

        # Data storage
        self.data_buffers = defaultdict(lambda: deque(maxlen=10))
        self.last_update_times = defaultdict(float)

        # Timers
        self.aggregation_timer = self.create_timer(0.5, self.aggregate_data)

        self.get_logger().info('Message aggregator initialized')

    def velocity_callback(self, msg):
        """
        Handle velocity messages
        """
        self.data_buffers['velocity'].append({
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'linear_z': msg.linear.z,
            'angular_x': msg.angular.x,
            'angular_y': msg.angular.y,
            'angular_z': msg.angular.z,
            'timestamp': time.time()
        })
        self.last_update_times['velocity'] = time.time()

    def battery_callback(self, msg):
        """
        Handle battery messages
        """
        self.data_buffers['battery'].append({
            'level': msg.percentage * 100,
            'voltage': msg.voltage,
            'current': msg.current,
            'timestamp': time.time()
        })
        self.last_update_times['battery'] = time.time()

    def laser_callback(self, msg):
        """
        Handle laser scan messages
        """
        # Process laser data to extract useful information
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if 0 < r < msg.range_max]
            if valid_ranges:
                min_distance = min(valid_ranges) if valid_ranges else float('inf')
                avg_distance = sum(valid_ranges) / len(valid_ranges) if valid_ranges else 0
            else:
                min_distance = float('inf')
                avg_distance = 0
        else:
            min_distance = float('inf')
            avg_distance = 0

        self.data_buffers['laser'].append({
            'min_distance': min_distance,
            'avg_distance': avg_distance,
            'num_readings': len(msg.ranges),
            'timestamp': time.time()
        })
        self.last_update_times['laser'] = time.time()

    def imu_callback(self, msg):
        """
        Handle IMU messages
        """
        self.data_buffers['imu'].append({
            'accel_x': msg.x,
            'accel_y': msg.y,
            'accel_z': msg.z,
            'timestamp': time.time()
        })
        self.last_update_times['imu'] = time.time()

    def distance_callback(self, msg):
        """
        Handle distance messages
        """
        self.data_buffers['distance'].append({
            'distance': msg.data,
            'timestamp': time.time()
        })
        self.last_update_times['distance'] = time.time()

    def aggregate_data(self):
        """
        Aggregate data from all sources and publish comprehensive status
        """
        # Check if we have recent data from all sources
        current_time = time.time()
        timeout_threshold = 2.0  # seconds

        recent_sources = []
        for source, last_time in self.last_update_times.items():
            if current_time - last_time <= timeout_threshold:
                recent_sources.append(source)

        if not recent_sources:
            return  # No recent data to aggregate

        # Build comprehensive status message
        status_parts = []

        # Add velocity information
        if 'velocity' in recent_sources and self.data_buffers['velocity']:
            latest_vel = self.data_buffers['velocity'][-1]
            status_parts.append(
                f'Vel: ({latest_vel["linear_x"]:.2f}, {latest_vel["angular_z"]:.2f})'
            )

        # Add battery information
        if 'battery' in recent_sources and self.data_buffers['battery']:
            latest_bat = self.data_buffers['battery'][-1]
            status_parts.append(f'Bat: {latest_bat["level"]:.1f}% ({latest_bat["voltage"]:.1f}V)')

        # Add laser information
        if 'laser' in recent_sources and self.data_buffers['laser']:
            latest_laser = self.data_buffers['laser'][-1]
            status_parts.append(f'Laser: Min={latest_laser["min_distance"]:.2f}m')

        # Add IMU information
        if 'imu' in recent_sources and self.data_buffers['imu']:
            latest_imu = self.data_buffers['imu'][-1]
            status_parts.append(f'IMU: Z={latest_imu["accel_z"]:.2f}')

        # Add distance information
        if 'distance' in recent_sources and self.data_buffers['distance']:
            latest_dist = self.data_buffers['distance'][-1]
            status_parts.append(f'Dist: {latest_dist["distance"]:.2f}m')

        # Publish comprehensive status
        if status_parts:
            status_msg = String()
            status_msg.data = f'[Aggregated] {" | ".join(status_parts)}'
            self.comprehensive_status_publisher.publish(status_msg)

        # Also publish a RobotPose message if we have position data
        self.publish_robot_pose()

    def publish_robot_pose(self):
        """
        Publish aggregated robot pose information
        """
        # This is a simplified example - in a real system, you'd integrate
        # odometry, IMU, and other sensors to estimate pose
        if self.data_buffers['velocity']:
            latest_vel = self.data_buffers['velocity'][-1]

            pose_msg = RobotPose()
            pose_msg.x = 0.0  # Would come from odometry integration
            pose_msg.y = 0.0  # Would come from odometry integration
            pose_msg.theta = 0.0  # Would come from IMU integration
            pose_msg.robot_name = 'aggregated_robot'
            pose_msg.battery_level = 100.0  # Would come from battery data
            pose_msg.is_charging = False

            # Update with latest battery data if available
            if self.data_buffers['battery']:
                latest_bat = self.data_buffers['battery'][-1]
                pose_msg.battery_level = latest_bat['level']

            self.robot_pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MessageAggregator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down message aggregator...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Code Examples with Explanations

### Example 1: Real-time Message Processing with Threading

```python
# File: robot_control_package/robot_control_package/realtime_processor.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
import threading
import queue
import time
from collections import deque

class RealTimeMessageProcessor(Node):
    """
    Process messages in real-time with separate processing threads
    """

    def __init__(self):
        super().__init__('realtime_processor')

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscriptions
        self.sensor_subscription = self.create_subscription(
            Float32,
            'sensor_data',
            self.sensor_callback,
            qos
        )

        # Publishers
        self.processed_publisher = self.create_publisher(Float32, 'processed_data', qos)
        self.status_publisher = self.create_publisher(String, 'processor_status', qos)

        # Thread-safe queues for message processing
        self.input_queue = queue.Queue(maxsize=100)
        self.output_queue = queue.Queue(maxsize=100)

        # Processing buffers
        self.processing_buffer = deque(maxlen=10)
        self.results_buffer = deque(maxlen=10)

        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_messages, daemon=True)
        self.processing_thread.start()

        # Timer for status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # Statistics
        self.messages_processed = 0
        self.messages_dropped = 0
        self.start_time = time.time()

        self.get_logger().info('Real-time message processor initialized')

    def sensor_callback(self, msg):
        """
        Handle incoming sensor messages
        """
        try:
            # Add to processing queue
            self.input_queue.put_nowait({
                'data': msg.data,
                'timestamp': time.time()
            })
        except queue.Full:
            self.messages_dropped += 1
            self.get_logger().warn('Input queue full, message dropped')

    def process_messages(self):
        """
        Process messages in a separate thread
        """
        while rclpy.ok():
            try:
                # Get message from input queue
                msg_data = self.input_queue.get(timeout=0.1)

                # Process the message (simulated processing time)
                processed_value = self.process_sensor_data(msg_data['data'])

                # Add to output queue
                result = {
                    'processed': processed_value,
                    'original': msg_data['data'],
                    'processing_time': time.time() - msg_data['timestamp']
                }

                try:
                    self.output_queue.put_nowait(result)
                    self.messages_processed += 1
                except queue.Full:
                    self.messages_dropped += 1

                self.output_queue.task_done()

            except queue.Empty:
                continue  # No messages to process, continue loop

    def process_sensor_data(self, raw_data):
        """
        Process raw sensor data
        """
        # Simulate some processing (e.g., filtering, calibration, etc.)
        import math
        # Apply some transformation
        processed = raw_data * 1.1 + 0.05 * math.sin(time.time())
        return processed

    def publish_status(self):
        """
        Publish processing status
        """
        try:
            # Get results from output queue
            while True:
                result = self.output_queue.get_nowait()

                # Publish processed data
                processed_msg = Float32()
                processed_msg.data = result['processed']
                self.processed_publisher.publish(processed_msg)

                # Add to results buffer for statistics
                self.results_buffer.append(result)

        except queue.Empty:
            pass  # No results to publish

        # Calculate statistics
        runtime = time.time() - self.start_time
        processing_rate = self.messages_processed / runtime if runtime > 0 else 0

        status_msg = String()
        status_msg.data = (
            f'Processed: {self.messages_processed}, '
            f'Dropped: {self.messages_dropped}, '
            f'Rate: {processing_rate:.2f} Hz, '
            f'Buffer: {self.input_queue.qsize()}'
        )
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RealTimeMessageProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down real-time processor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Message Validation and Error Handling

```python
# File: robot_control_package/robot_control_package/validated_subscriber.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
import math

class ValidatedSubscriber(Node):
    """
    Subscribe to messages with validation and error handling
    """

    def __init__(self):
        super().__init__('validated_subscriber')

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscriptions
        self.velocity_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.validated_velocity_callback,
            qos
        )

        self.battery_subscription = self.create_subscription(
            BatteryState,
            'battery_status',
            self.validated_battery_callback,
            qos
        )

        self.sensor_subscription = self.create_subscription(
            Float32,
            'sensor_data',
            self.validated_sensor_callback,
            qos
        )

        # Publishers for validation results
        self.validation_publisher = self.create_publisher(String, 'validation_results', qos)

        # Statistics
        self.validation_stats = {
            'total_messages': 0,
            'valid_messages': 0,
            'invalid_messages': 0,
            'error_messages': 0
        }

        self.get_logger().info('Validated subscriber initialized')

    def validated_velocity_callback(self, msg):
        """
        Handle velocity messages with validation
        """
        self.validation_stats['total_messages'] += 1

        # Validate message
        is_valid, errors = self.validate_twist_message(msg)

        if is_valid:
            self.validation_stats['valid_messages'] += 1
            self.process_valid_velocity(msg)
        else:
            self.validation_stats['invalid_messages'] += 1
            self.handle_invalid_velocity(msg, errors)

    def validated_battery_callback(self, msg):
        """
        Handle battery messages with validation
        """
        self.validation_stats['total_messages'] += 1

        # Validate message
        is_valid, errors = self.validate_battery_message(msg)

        if is_valid:
            self.validation_stats['valid_messages'] += 1
            self.process_valid_battery(msg)
        else:
            self.validation_stats['invalid_messages'] += 1
            self.handle_invalid_battery(msg, errors)

    def validated_sensor_callback(self, msg):
        """
        Handle sensor messages with validation
        """
        self.validation_stats['total_messages'] += 1

        # Validate message
        is_valid, errors = self.validate_sensor_message(msg)

        if is_valid:
            self.validation_stats['valid_messages'] += 1
            self.process_valid_sensor(msg)
        else:
            self.validation_stats['invalid_messages'] += 1
            self.handle_invalid_sensor(msg, errors)

    def validate_twist_message(self, msg):
        """
        Validate Twist message
        """
        errors = []

        # Check for NaN or infinity values
        if math.isnan(msg.linear.x) or math.isinf(msg.linear.x):
            errors.append(f"Invalid linear.x: {msg.linear.x}")
        if math.isnan(msg.linear.y) or math.isinf(msg.linear.y):
            errors.append(f"Invalid linear.y: {msg.linear.y}")
        if math.isnan(msg.linear.z) or math.isinf(msg.linear.z):
            errors.append(f"Invalid linear.z: {msg.linear.z}")
        if math.isnan(msg.angular.x) or math.isinf(msg.angular.x):
            errors.append(f"Invalid angular.x: {msg.angular.x}")
        if math.isnan(msg.angular.y) or math.isinf(msg.angular.y):
            errors.append(f"Invalid angular.y: {msg.angular.y}")
        if math.isnan(msg.angular.z) or math.isinf(msg.angular.z):
            errors.append(f"Invalid angular.z: {msg.angular.z}")

        # Check for extreme values
        max_linear_vel = 5.0  # m/s
        max_angular_vel = 5.0  # rad/s
        if abs(msg.linear.x) > max_linear_vel:
            errors.append(f"Linear velocity too high: {msg.linear.x}")
        if abs(msg.angular.z) > max_angular_vel:
            errors.append(f"Angular velocity too high: {msg.angular.z}")

        return len(errors) == 0, errors

    def validate_battery_message(self, msg):
        """
        Validate BatteryState message
        """
        errors = []

        # Check percentage range
        if msg.percentage < 0.0 or msg.percentage > 1.0:
            errors.append(f"Invalid percentage: {msg.percentage}")

        # Check voltage range (assuming 12V system)
        if msg.voltage < 10.0 or msg.voltage > 14.4:
            errors.append(f"Invalid voltage: {msg.voltage}")

        return len(errors) == 0, errors

    def validate_sensor_message(self, msg):
        """
        Validate Float32 sensor message
        """
        errors = []

        # Check for NaN or infinity
        if math.isnan(msg.data) or math.isinf(msg.data):
            errors.append(f"Invalid sensor data: {msg.data}")

        # Check for reasonable range (example: distance sensor 0-10m)
        if msg.data < 0.0 or msg.data > 10.0:
            errors.append(f"Sensor data out of range: {msg.data}")

        return len(errors) == 0, errors

    def process_valid_velocity(self, msg):
        """
        Process valid velocity message
        """
        self.get_logger().debug(f'Valid velocity: ({msg.linear.x:.2f}, {msg.angular.z:.2f})')

    def process_valid_battery(self, msg):
        """
        Process valid battery message
        """
        self.get_logger().debug(f'Valid battery: {msg.percentage*100:.1f}%')

    def process_valid_sensor(self, msg):
        """
        Process valid sensor message
        """
        self.get_logger().debug(f'Valid sensor: {msg.data:.2f}')

    def handle_invalid_velocity(self, msg, errors):
        """
        Handle invalid velocity message
        """
        error_msg = f'Invalid velocity message: {", ".join(errors)}'
        self.get_logger().error(error_msg)
        self.publish_validation_error(error_msg)

    def handle_invalid_battery(self, msg, errors):
        """
        Handle invalid battery message
        """
        error_msg = f'Invalid battery message: {", ".join(errors)}'
        self.get_logger().error(error_msg)
        self.publish_validation_error(error_msg)

    def handle_invalid_sensor(self, msg, errors):
        """
        Handle invalid sensor message
        """
        error_msg = f'Invalid sensor message: {", ".join(errors)}'
        self.get_logger().error(error_msg)
        self.publish_validation_error(error_msg)

    def publish_validation_error(self, error_msg):
        """
        Publish validation error
        """
        validation_msg = String()
        validation_msg.data = error_msg
        self.validation_publisher.publish(validation_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ValidatedSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down validated subscriber...')
        node.get_logger().info(f'Validation stats: {node.validation_stats}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3: Message Synchronization

```python
# File: robot_control_package/robot_control_package/message_synchronizer.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
from collections import deque

class MessageSynchronizer(Node):
    """
    Synchronize messages from multiple topics based on timestamps
    """

    def __init__(self):
        super().__init__('message_synchronizer')

        # QoS profile
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscriptions
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos
        )

        self.velocity_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            qos
        )

        self.distance_subscription = self.create_subscription(
            Float32,
            'distance_to_obstacle',
            self.distance_callback,
            qos
        )

        # Publishers
        self.synchronized_publisher = self.create_publisher(
            String, 'synchronized_data', qos
        )

        # Message buffers with timestamps
        self.laser_buffer = deque(maxlen=10)
        self.velocity_buffer = deque(maxlen=10)
        self.distance_buffer = deque(maxlen=10)

        # Synchronization parameters
        self.sync_window = 0.1  # seconds to match messages

        # Timer for synchronization
        self.sync_timer = self.create_timer(0.05, self.synchronize_messages)

        self.get_logger().info('Message synchronizer initialized')

    def laser_callback(self, msg):
        """
        Handle laser scan messages
        """
        self.laser_buffer.append({
            'data': msg,
            'timestamp': time.time()
        })

    def velocity_callback(self, msg):
        """
        Handle velocity messages
        """
        self.velocity_buffer.append({
            'data': msg,
            'timestamp': time.time()
        })

    def distance_callback(self, msg):
        """
        Handle distance messages
        """
        self.distance_buffer.append({
            'data': msg,
            'timestamp': time.time()
        })

    def synchronize_messages(self):
        """
        Synchronize messages from different topics
        """
        if not (self.laser_buffer and self.velocity_buffer and self.distance_buffer):
            return  # Need messages from all sources

        # Get the most recent messages
        latest_laser = self.laser_buffer[-1]
        latest_velocity = self.velocity_buffer[-1]
        latest_distance = self.distance_buffer[-1]

        # Find messages within sync window
        laser_match = self.find_closest_message(latest_velocity['timestamp'], self.laser_buffer)
        distance_match = self.find_closest_message(latest_velocity['timestamp'], self.distance_buffer)

        if laser_match and distance_match:
            # All messages are within sync window
            self.publish_synchronized_data(
                latest_velocity['data'],
                laser_match['data'],
                distance_match['data']
            )

    def find_closest_message(self, target_time, buffer):
        """
        Find the message in buffer closest to target_time within sync window
        """
        closest_msg = None
        min_diff = float('inf')

        for msg in buffer:
            diff = abs(msg['timestamp'] - target_time)
            if diff < self.sync_window and diff < min_diff:
                min_diff = diff
                closest_msg = msg

        return closest_msg

    def publish_synchronized_data(self, velocity_msg, laser_msg, distance_msg):
        """
        Publish synchronized data combination
        """
        if laser_msg.ranges:
            min_distance = min([r for r in laser_msg.ranges if 0 < r < laser_msg.range_max] or [float('inf')])
        else:
            min_distance = float('inf')

        sync_msg = String()
        sync_msg.data = (
            f'Synced: Vel=({velocity_msg.linear.x:.2f},{velocity_msg.angular.z:.2f}), '
            f'LaserMin={min_distance:.2f}m, Dist={distance_msg.data:.2f}m'
        )

        self.synchronized_publisher.publish(sync_msg)
        self.get_logger().debug(f'Published synchronized data: {sync_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MessageSynchronizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down message synchronizer...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-On Exercises: Communication Optimization

### Exercise 1: Message Compression and Efficient Publishing

Create a system that efficiently publishes high-frequency sensor data:

```python
# File: robot_control_package/robot_control_package/efficient_publisher.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import time
from collections import deque

class EfficientPublisher(Node):
    """
    Efficiently publish high-frequency data using optimized message formats
    """

    def __init__(self):
        super().__init__('efficient_publisher')

        # QoS profile optimized for high-frequency data
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publishers
        self.compressed_publisher = self.create_publisher(Float32MultiArray, 'compressed_data', qos)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, 'pointcloud_data', qos)

        # Timer for high-frequency publishing
        self.publish_timer = self.create_timer(0.01, self.publish_efficient_data)  # 100 Hz

        # Data generation state
        self.time_offset = time.time()
        self.data_counter = 0

        self.get_logger().info('Efficient publisher initialized')

    def publish_efficient_data(self):
        """
        Publish data efficiently using compressed formats
        """
        current_time = time.time() - self.time_offset

        # Publish compressed sensor array
        self.publish_compressed_sensor_data(current_time)

        # Publish point cloud data
        self.publish_pointcloud_data(current_time)

        self.data_counter += 1

    def publish_compressed_sensor_data(self, timestamp):
        """
        Publish sensor data in compressed format
        """
        # Simulate sensor array data (e.g., 64 sensors)
        num_sensors = 64
        sensor_data = np.random.normal(0, 1, num_sensors).astype(np.float32)

        # Add some pattern to make it more realistic
        for i in range(num_sensors):
            sensor_data[i] += 0.5 * np.sin(timestamp + i * 0.1)

        # Create compressed message
        compressed_msg = Float32MultiArray()
        compressed_msg.data = sensor_data.tolist()

        self.compressed_publisher.publish(compressed_msg)

    def publish_pointcloud_data(self, timestamp):
        """
        Publish point cloud data efficiently
        """
        # Create a simple point cloud (e.g., for LIDAR simulation)
        num_points = 360  # 360 degree scan
        angles = np.linspace(0, 2*np.pi, num_points)

        # Simulate distances with some objects
        distances = 2.0 + 0.5 * np.sin(4 * angles)  # Base with some variation
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        z = np.zeros_like(x)  # 2D scan

        # Create PointCloud2 message
        points = np.column_stack((x, y, z)).astype(np.float32)

        # Create PointCloud2 message manually for efficiency
        cloud_msg = PointCloud2()
        cloud_msg.header = Header()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = 'laser_frame'

        # Set up fields
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 * 4 bytes per point
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True

        # Pack the data
        cloud_msg.data = points.tobytes()

        self.pointcloud_publisher.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EfficientPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down efficient publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 2: Adaptive Message Rate Control

Create a system that adapts message rates based on system load:

```python
# File: robot_control_package/robot_control_package/adaptiverate_publisher.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float32
import time
import threading
import psutil

class AdaptiveRatePublisher(Node):
    """
    Publish messages at adaptive rates based on system load
    """

    def __init__(self):
        super().__init__('adaptive_rate_publisher')

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publishers
        self.status_publisher = self.create_publisher(String, 'adaptive_status', qos)
        self.load_publisher = self.create_publisher(Float32, 'system_load', qos)

        # Initial timer
        self.publish_rate = 10.0  # Hz
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_adaptive_data)

        # System monitoring
        self.last_cpu_percent = 0
        self.adaptation_lock = threading.Lock()

        # Adaptive parameters
        self.min_rate = 1.0    # Minimum publish rate (Hz)
        self.max_rate = 50.0   # Maximum publish rate (Hz)
        self.load_threshold = 80.0  # CPU load threshold for rate reduction

        # Start system monitoring thread
        self.monitor_thread = threading.Thread(target=self.monitor_system, daemon=True)
        self.monitor_thread.start()

        self.get_logger().info('Adaptive rate publisher initialized')

    def publish_adaptive_data(self):
        """
        Publish data at current adaptive rate
        """
        status_msg = String()
        status_msg.data = (
            f'Rate: {self.publish_rate:.2f}Hz, '
            f'CPU: {self.last_cpu_percent:.1f}%, '
            f'Time: {time.time():.2f}'
        )
        self.status_publisher.publish(status_msg)

        load_msg = Float32()
        load_msg.data = self.last_cpu_percent / 100.0  # Normalize to 0-1
        self.load_publisher.publish(load_msg)

    def monitor_system(self):
        """
        Monitor system resources in a separate thread
        """
        while rclpy.ok():
            # Get current CPU usage
            cpu_percent = psutil.cpu_percent(interval=1)

            with self.adaptation_lock:
                self.last_cpu_percent = cpu_percent

                # Adjust publish rate based on CPU load
                if cpu_percent > self.load_threshold:
                    # Reduce rate when CPU is overloaded
                    self.publish_rate = max(self.min_rate, self.publish_rate * 0.9)
                elif cpu_percent < self.load_threshold * 0.7:
                    # Increase rate when CPU has capacity
                    self.publish_rate = min(self.max_rate, self.publish_rate * 1.05)

                # Update timer with new rate
                try:
                    self.timer.timer_period_ns = int(1e9 / self.publish_rate)
                except:
                    # If timer is already destroyed, break the loop
                    break

            time.sleep(1)  # Monitor every second

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveRatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down adaptive rate publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Pitfalls and Solutions

### Pitfall 1: Message Queue Overflow
**Problem**: Message queues fill up and cause memory issues or message drops.

**Solution**:
- Use appropriate queue depths based on expected message rates
- Implement message dropping strategies when queues are full
- Monitor queue sizes and adjust publishing rates accordingly

```python
def check_queue_status(self):
    """
    Check queue status and adjust behavior if needed
    """
    queue_size = self.input_queue.qsize()
    if queue_size > self.input_queue.maxsize * 0.8:  # 80% full
        self.get_logger().warn(f'Input queue {queue_size}/{self.input_queue.maxsize} - reducing rate')
        # Implement rate reduction logic here
```

### Pitfall 2: Inconsistent QoS Settings
**Problem**: Publishers and subscribers with incompatible QoS settings don't communicate.

**Solution**:
- Document QoS requirements for each topic
- Use consistent QoS settings across the system
- Test with different QoS combinations during development

### Pitfall 3: Message Ordering Issues
**Problem**: Messages arrive out of order, causing incorrect processing.

**Solution**:
- Use sequence numbers in messages when order matters
- Implement message buffering and reordering if needed
- Use reliable QoS when message order is critical

## Review Questions

1. How do Quality of Service (QoS) policies affect message passing in ROS 2?
2. What are the differences between RELIABLE and BEST_EFFORT communication?
3. How can you optimize message publishing for high-frequency data?
4. What techniques can be used to validate incoming messages?
5. How do you handle message synchronization from multiple topics?

## Project Assignment: Communication System Optimization

Create a complete communication system that includes:
1. A publisher that efficiently handles high-frequency sensor data
2. A subscriber that validates and processes incoming messages
3. A message aggregator that combines data from multiple sources
4. An adaptive rate controller that adjusts based on system load
5. Proper error handling and recovery mechanisms

Your system should:
- Demonstrate efficient message passing patterns
- Include validation and error handling
- Show adaptive communication based on system conditions
- Be optimized for real-time robotics applications
- Include comprehensive logging and monitoring

## Further Resources

- [ROS 2 Quality of Service](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [ROS 2 Publisher-Subscriber Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Message Passing Performance](https://discourse.ros.org/t/ros2-performance-analysis/12345)
- [Real-time ROS 2 Programming](https://docs.ros.org/en/humble/Tutorials/Advanced/Real-Time-Programming.html)

:::info
Efficient message passing is crucial for real-time robotics applications. Always consider the trade-offs between reliability, performance, and resource usage when designing your communication architecture.
:::