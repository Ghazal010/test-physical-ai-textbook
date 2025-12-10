---
title: Week 13 - System Integration
description: Integrating all modules into a complete autonomous humanoid system
sidebar_position: 2
---

# Week 13: System Integration

## Learning Objectives
- Understand the principles of complex system integration in robotics
- Implement modular integration patterns for large robotic systems
- Design robust communication architectures between subsystems
- Create comprehensive testing and validation procedures
- Optimize system performance across integrated components
- Troubleshoot and debug integrated robotic systems

## Prerequisites Check
- Completion of all previous modules (1-4)
- Understanding of ROS 2 communication patterns
- Experience with simulation and real robot deployment
- Knowledge of software architecture patterns

## Theoretical Concepts: System Integration Architecture

### Integration Patterns in Robotics

System integration in robotics involves connecting multiple specialized subsystems into a cohesive, functional whole. The main integration patterns include:

1. **Centralized Integration**: A central coordinator manages all subsystems
2. **Decentralized Integration**: Subsystems communicate directly with each other
3. **Service-Oriented Integration**: Subsystems provide services to each other
4. **Event-Driven Integration**: Subsystems react to events from other components

### Communication Architecture

The communication architecture defines how information flows between subsystems:

- **ROS Topics**: Asynchronous message passing for continuous data streams
- **ROS Services**: Synchronous request-response for discrete operations
- **ROS Actions**: Asynchronous goal-oriented operations with feedback
- **Shared Memory**: High-performance data sharing for critical paths

### Integration Challenges

Key challenges in system integration include:

- **Timing Constraints**: Ensuring real-time performance across subsystems
- **Data Consistency**: Maintaining synchronized state across components
- **Error Propagation**: Preventing failures from cascading through the system
- **Resource Management**: Coordinating shared resources like CPU and memory
- **Testing Complexity**: Validating the behavior of the integrated system

## Step-by-Step Tutorials: Integration Implementation

### Modular Integration Framework

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
import threading
import queue
from typing import Dict, List, Callable, Any, Optional
from dataclasses import dataclass
from enum import Enum

class SystemState(Enum):
    """
    System states for integration management
    """
    INITIALIZING = "initializing"
    IDLE = "idle"
    ACTIVE = "active"
    ERROR = "error"
    SHUTTING_DOWN = "shutting_down"

@dataclass
class IntegrationConfig:
    """
    Configuration for system integration
    """
    heartbeat_rate: float = 10.0  # Hz
    timeout_threshold: float = 5.0  # seconds
    max_message_queue_size: int = 100
    enable_monitoring: bool = True
    enable_logging: bool = True

class IntegrationModule:
    """
    Base class for integration modules
    """

    def __init__(self, name: str, node: Node):
        self.name = name
        self.node = node
        self.active = False
        self.last_heartbeat = self.node.get_clock().now()
        self.message_queue = queue.Queue(maxsize=10)

    def initialize(self) -> bool:
        """
        Initialize the module
        """
        raise NotImplementedError

    def process(self):
        """
        Main processing loop for the module
        """
        raise NotImplementedError

    def shutdown(self):
        """
        Clean shutdown of the module
        """
        raise NotImplementedError

    def send_message(self, topic: str, message: Any):
        """
        Send message to other modules
        """
        pass

    def receive_message(self, topic: str) -> Any:
        """
        Receive message from other modules
        """
        try:
            return self.message_queue.get_nowait()
        except queue.Empty:
            return None

class PerceptionModule(IntegrationModule):
    """
    Perception subsystem integration module
    """

    def __init__(self, node: Node):
        super().__init__("perception", node)

        # Subscribers for sensor data
        self.image_sub = node.create_subscription(
            sensor_msgs.msg.Image, '/camera/rgb/image_raw',
            self.image_callback, 10
        )
        self.laser_sub = node.create_subscription(
            sensor_msgs.msg.LaserScan, '/scan',
            self.laser_callback, 10
        )

        # Publishers for processed data
        self.objects_pub = node.create_publisher(
            vision_msgs.msg.Detection2DArray, '/perception/objects', 10
        )
        self.scene_pub = node.create_publisher(
            std_msgs.msg.String, '/perception/scene_description', 10
        )

        # State tracking
        self.latest_image = None
        self.latest_scan = None
        self.object_detections = []

    def initialize(self) -> bool:
        """
        Initialize perception module
        """
        self.node.get_logger().info("Initializing perception module")
        self.active = True
        return True

    def image_callback(self, msg):
        """
        Process incoming image data
        """
        self.latest_image = msg
        # Process image for object detection (simplified)
        detections = self.process_image(msg)
        self.object_detections.extend(detections)

        # Publish detections
        detection_msg = vision_msgs.msg.Detection2DArray()
        detection_msg.header = msg.header
        detection_msg.detections = detections
        self.objects_pub.publish(detection_msg)

    def laser_callback(self, msg):
        """
        Process incoming laser scan data
        """
        self.latest_scan = msg
        # Process scan for obstacle detection (simplified)
        obstacles = self.process_scan(msg)

        # Publish scene description
        scene_msg = std_msgs.msg.String()
        scene_msg.data = f"Detected {len(obstacles)} obstacles"
        self.scene_pub.publish(scene_msg)

    def process_image(self, image_msg):
        """
        Process image for object detection
        """
        # Simplified object detection (in practice, use trained models)
        return []

    def process_scan(self, scan_msg):
        """
        Process laser scan for obstacle detection
        """
        # Simplified obstacle detection
        obstacles = []
        for i, range_val in enumerate(scan_msg.ranges):
            if scan_msg.range_min < range_val < scan_msg.range_max:
                if range_val < 1.0:  # Obstacle within 1 meter
                    obstacles.append({
                        'angle': scan_msg.angle_min + i * scan_msg.angle_increment,
                        'distance': range_val
                    })
        return obstacles

    def process(self):
        """
        Main processing loop for perception
        """
        # In practice, this would run continuous perception processing
        pass

    def shutdown(self):
        """
        Shutdown perception module
        """
        self.active = False

class NavigationModule(IntegrationModule):
    """
    Navigation subsystem integration module
    """

    def __init__(self, node: Node):
        super().__init__("navigation", node)

        # Action client for navigation
        self.nav_client = rclpy.action.ActionClient(
            node, nav2_msgs.action.NavigateToPose, 'navigate_to_pose'
        )

        # Subscribers
        self.goal_sub = node.create_subscription(
            geometry_msgs.msg.PoseStamped, '/goal_pose',
            self.goal_callback, 10
        )
        self.odom_sub = node.create_subscription(
            nav_msgs.msg.Odometry, '/odom',
            self.odom_callback, 10
        )

        # Publishers
        self.cmd_vel_pub = node.create_publisher(
            geometry_msgs.msg.Twist, '/cmd_vel', 10
        )

        # State tracking
        self.current_pose = None
        self.navigation_active = False
        self.goal_queue = queue.Queue()

    def initialize(self) -> bool:
        """
        Initialize navigation module
        """
        self.node.get_logger().info("Initializing navigation module")
        self.active = True
        return True

    def goal_callback(self, msg):
        """
        Handle incoming navigation goals
        """
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error("Navigation server not available")
            return

        goal_msg = nav2_msgs.action.NavigateToPose.Goal()
        goal_msg.pose = msg

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def odom_callback(self, msg):
        """
        Update current pose from odometry
        """
        self.current_pose = msg.pose.pose

    def navigation_feedback_callback(self, feedback_msg):
        """
        Handle navigation feedback
        """
        self.node.get_logger().info(
            f"Navigation progress: {feedback_msg.feedback.distance_remaining:.2f}m remaining"
        )

    def goal_response_callback(self, future):
        """
        Handle navigation goal response
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Navigation goal rejected')
            return

        self.navigation_active = True
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """
        Handle navigation result
        """
        result = future.result().result
        status = future.result().status

        if status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('Navigation succeeded')
        else:
            self.node.get_logger().info(f'Navigation failed with status: {status}')

        self.navigation_active = False

    def process(self):
        """
        Main processing loop for navigation
        """
        # Handle queued navigation goals
        try:
            while not self.goal_queue.empty():
                goal = self.goal_queue.get_nowait()
                self.execute_navigation(goal)
        except queue.Empty:
            pass

    def execute_navigation(self, goal):
        """
        Execute navigation to goal
        """
        # Simplified navigation execution
        pass

    def shutdown(self):
        """
        Shutdown navigation module
        """
        self.active = False

class InteractionModule(IntegrationModule):
    """
    Human-robot interaction subsystem integration module
    """

    def __init__(self, node: Node):
        super().__init__("interaction", node)

        # Subscribers
        self.voice_cmd_sub = node.create_subscription(
            std_msgs.msg.String, '/voice_command',
            self.voice_command_callback, 10
        )
        self.text_cmd_sub = node.create_subscription(
            std_msgs.msg.String, '/text_command',
            self.text_command_callback, 10
        )

        # Publishers
        self.response_pub = node.create_publisher(
            std_msgs.msg.String, '/response', 10
        )
        self.emotion_pub = node.create_publisher(
            std_msgs.msg.String, '/emotion', 10
        )

        # State tracking
        self.conversation_history = []
        self.current_intent = None

    def initialize(self) -> bool:
        """
        Initialize interaction module
        """
        self.node.get_logger().info("Initializing interaction module")
        self.active = True
        return True

    def voice_command_callback(self, msg):
        """
        Process voice commands
        """
        self.process_command(msg.data, source="voice")

    def text_command_callback(self, msg):
        """
        Process text commands
        """
        self.process_command(msg.data, source="text")

    def process_command(self, command: str, source: str):
        """
        Process incoming command and generate response
        """
        # Simplified command processing (in practice, use NLP/LLM)
        self.conversation_history.append({
            'source': source,
            'command': command,
            'timestamp': self.node.get_clock().now()
        })

        # Generate response
        response = self.generate_response(command)

        # Publish response
        response_msg = std_msgs.msg.String()
        response_msg.data = response
        self.response_pub.publish(response_msg)

    def generate_response(self, command: str) -> str:
        """
        Generate appropriate response to command
        """
        # Simplified response generation
        if "hello" in command.lower():
            return "Hello! How can I help you?"
        elif "move" in command.lower() or "go" in command.lower():
            return "I will navigate to the specified location."
        else:
            return "I understand your command."

    def process(self):
        """
        Main processing loop for interaction
        """
        # Handle ongoing interactions
        pass

    def shutdown(self):
        """
        Shutdown interaction module
        """
        self.active = False

class SystemIntegrator(Node):
    """
    Main system integrator that coordinates all modules
    """

    def __init__(self):
        super().__init__('system_integrator')

        # Integration configuration
        self.config = IntegrationConfig()

        # System state management
        self.state = SystemState.INITIALIZING
        self.modules: Dict[str, IntegrationModule] = {}

        # Publishers for system status
        self.status_pub = self.create_publisher(
            std_msgs.msg.String, '/system_status', 10
        )
        self.health_pub = self.create_publisher(
            std_msgs.msg.Bool, '/system_healthy', 10
        )

        # Initialize modules
        self.initialize_modules()

        # Start integration timer
        self.integration_timer = self.create_timer(
            1.0 / self.config.heartbeat_rate,
            self.integration_loop
        )

        self.get_logger().info("System integrator initialized")

    def initialize_modules(self):
        """
        Initialize all integration modules
        """
        # Create and initialize modules
        perception_module = PerceptionModule(self)
        navigation_module = NavigationModule(self)
        interaction_module = InteractionModule(self)

        modules_to_init = [
            ("perception", perception_module),
            ("navigation", navigation_module),
            ("interaction", interaction_module)
        ]

        for name, module in modules_to_init:
            if module.initialize():
                self.modules[name] = module
                self.get_logger().info(f"Module {name} initialized successfully")
            else:
                self.get_logger().error(f"Failed to initialize module {name}")

        # Check if all modules initialized
        if len(self.modules) == len(modules_to_init):
            self.state = SystemState.IDLE
            self.get_logger().info("All modules initialized successfully")
        else:
            self.state = SystemState.ERROR
            self.get_logger().error("Failed to initialize all modules")

    def integration_loop(self):
        """
        Main integration loop that coordinates all modules
        """
        if self.state in [SystemState.ERROR, SystemState.SHUTTING_DOWN]:
            return

        # Process each module
        for name, module in self.modules.items():
            try:
                module.process()
            except Exception as e:
                self.get_logger().error(f"Error processing module {name}: {e}")
                self.state = SystemState.ERROR

        # Check system health
        healthy = self.check_system_health()
        health_msg = Bool()
        health_msg.data = healthy
        self.health_pub.publish(health_msg)

        # Update status
        status_msg = String()
        status_msg.data = f"State: {self.state.value}, Modules: {len(self.modules)}"
        self.status_pub.publish(status_msg)

    def check_system_health(self) -> bool:
        """
        Check overall system health
        """
        if self.state == SystemState.ERROR:
            return False

        # Check if all modules are active
        for name, module in self.modules.items():
            if not module.active:
                return False

        return True

    def shutdown(self):
        """
        Shutdown the entire integrated system
        """
        self.get_logger().info("Shutting down integrated system")
        self.state = SystemState.SHUTTING_DOWN

        # Shutdown all modules
        for name, module in self.modules.items():
            try:
                module.shutdown()
            except Exception as e:
                self.get_logger().error(f"Error shutting down module {name}: {e}")

def main(args=None):
    rclpy.init(args=args)
    integrator = SystemIntegrator()

    try:
        rclpy.spin(integrator)
    except KeyboardInterrupt:
        pass
    finally:
        integrator.shutdown()
        integrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Integration with State Management

```python
import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Callable, Any, Optional
from dataclasses import dataclass
from enum import Enum
import time
import json

class IntegrationState(Enum):
    """
    Detailed integration states
    """
    STARTING = "starting"
    INITIALIZING_MODULES = "initializing_modules"
    CONNECTING_INTERFACES = "connecting_interfaces"
    VALIDATING_INTEGRATION = "validating_integration"
    OPERATIONAL = "operational"
    DEGRADED = "degraded"
    FAILING = "failing"
    SHUTTING_DOWN = "shutting_down"

@dataclass
class IntegrationEvent:
    """
    Event for system integration communication
    """
    event_type: str
    source_module: str
    data: Dict[str, Any]
    timestamp: float
    priority: int = 0  # 0=low, 1=normal, 2=high, 3=critical

class EventManager:
    """
    Event management system for integration
    """

    def __init__(self):
        self.event_queue = asyncio.Queue()
        self.subscribers: Dict[str, List[Callable]] = {}
        self.event_history = []
        self.max_history_size = 1000

    async def publish_event(self, event: IntegrationEvent):
        """
        Publish an event to the event queue
        """
        await self.event_queue.put(event)

        # Add to history
        self.event_history.append(event)
        if len(self.event_history) > self.max_history_size:
            self.event_history.pop(0)

    def subscribe_to_event(self, event_type: str, callback: Callable):
        """
        Subscribe to events of a specific type
        """
        if event_type not in self.subscribers:
            self.subscribers[event_type] = []
        self.subscribers[event_type].append(callback)

    async def process_events(self):
        """
        Process events from the queue
        """
        while True:
            try:
                event = await asyncio.wait_for(self.event_queue.get(), timeout=0.1)

                # Notify subscribers
                if event.event_type in self.subscribers:
                    for callback in self.subscribers[event.event_type]:
                        try:
                            callback(event)
                        except Exception as e:
                            print(f"Error in event callback: {e}")
            except asyncio.TimeoutError:
                continue  # No events to process, continue loop

class InterfaceManager:
    """
    Interface management for system integration
    """

    def __init__(self):
        self.interfaces: Dict[str, Dict] = {}
        self.connections: Dict[str, List[str]] = {}
        self.interface_validators: Dict[str, Callable] = {}

    def register_interface(self, name: str, interface_config: Dict):
        """
        Register a new interface
        """
        self.interfaces[name] = interface_config
        self.connections[name] = []

    def connect_interfaces(self, source: str, target: str):
        """
        Connect two interfaces
        """
        if source in self.interfaces and target in self.interfaces:
            if target not in self.connections[source]:
                self.connections[source].append(target)
                print(f"Connected {source} -> {target}")
        else:
            raise ValueError(f"Interface {source} or {target} not registered")

    def validate_interface(self, name: str, data: Any) -> bool:
        """
        Validate data for an interface
        """
        if name in self.interface_validators:
            return self.interface_validators[name](data)
        return True  # Default to true if no validator

    def add_validator(self, interface_name: str, validator: Callable):
        """
        Add a validator for an interface
        """
        self.interface_validators[interface_name] = validator

class IntegrationMonitor:
    """
    Monitor for integrated system health
    """

    def __init__(self):
        self.metrics: Dict[str, List] = {}
        self.alerts: List[Dict] = []
        self.max_metrics_history = 1000

    def record_metric(self, metric_name: str, value: Any):
        """
        Record a system metric
        """
        if metric_name not in self.metrics:
            self.metrics[metric_name] = []

        self.metrics[metric_name].append({
            'timestamp': time.time(),
            'value': value
        })

        # Maintain history size
        if len(self.metrics[metric_name]) > self.max_metrics_history:
            self.metrics[metric_name].pop(0)

    def check_threshold(self, metric_name: str, threshold: float, operator: str = 'gt'):
        """
        Check if metric exceeds threshold
        """
        if metric_name not in self.metrics or not self.metrics[metric_name]:
            return False

        latest_value = self.metrics[metric_name][-1]['value']

        if operator == 'gt':
            return latest_value > threshold
        elif operator == 'lt':
            return latest_value < threshold
        elif operator == 'eq':
            return latest_value == threshold
        elif operator == 'ge':
            return latest_value >= threshold
        elif operator == 'le':
            return latest_value <= threshold

        return False

    def generate_alert(self, alert_type: str, message: str, severity: str = 'warning'):
        """
        Generate an alert
        """
        alert = {
            'timestamp': time.time(),
            'type': alert_type,
            'message': message,
            'severity': severity
        }
        self.alerts.append(alert)
        print(f"[{severity.upper()}] {alert_type}: {message}")

    def get_health_score(self) -> float:
        """
        Calculate overall system health score (0.0 to 1.0)
        """
        if not self.metrics:
            return 0.5  # Default if no metrics

        # Calculate based on various metrics
        scores = []

        # CPU usage (lower is better, max 80%)
        if 'cpu_usage' in self.metrics:
            cpu_usage = self.metrics['cpu_usage'][-1]['value']
            cpu_score = max(0, (80 - cpu_usage) / 80)  # Invert so lower usage is better
            scores.append(cpu_score)

        # Memory usage (lower is better, max 85%)
        if 'memory_usage' in self.metrics:
            mem_usage = self.metrics['memory_usage'][-1]['value']
            mem_score = max(0, (85 - mem_usage) / 85)
            scores.append(mem_score)

        # Error rate (lower is better)
        if 'error_rate' in self.metrics:
            error_rate = self.metrics['error_rate'][-1]['value']
            error_score = max(0, (1 - error_rate) if error_rate <= 1 else 0)
            scores.append(error_score)

        return sum(scores) / len(scores) if scores else 0.5

class AdvancedSystemIntegrator:
    """
    Advanced system integration with comprehensive monitoring
    """

    def __init__(self):
        self.state = IntegrationState.STARTING
        self.event_manager = EventManager()
        self.interface_manager = InterfaceManager()
        self.monitor = IntegrationMonitor()
        self.executor = ThreadPoolExecutor(max_workers=8)

        # Module registry
        self.modules: Dict[str, Any] = {}
        self.module_states: Dict[str, IntegrationState] = {}

        # Integration tasks
        self.integration_tasks = []

        # Start event processing
        self.event_task = asyncio.create_task(self.process_events())

    async def process_events(self):
        """
        Process events in the background
        """
        await self.event_manager.process_events()

    def register_module(self, name: str, module_instance: Any):
        """
        Register a module with the integrator
        """
        self.modules[name] = module_instance
        self.module_states[name] = IntegrationState.STARTING

        # Register module interfaces
        if hasattr(module_instance, 'get_interfaces'):
            interfaces = module_instance.get_interfaces()
            for interface_name, interface_config in interfaces.items():
                full_name = f"{name}.{interface_name}"
                self.interface_manager.register_interface(full_name, interface_config)

    def connect_modules(self, source_module: str, source_interface: str,
                       target_module: str, target_interface: str):
        """
        Connect interfaces between modules
        """
        source_full = f"{source_module}.{source_interface}"
        target_full = f"{target_module}.{target_interface}"

        self.interface_manager.connect_interfaces(source_full, target_full)

    def start_integration(self):
        """
        Start the integration process
        """
        print("Starting system integration...")

        # Initialize modules
        self.state = IntegrationState.INITIALIZING_MODULES
        for name, module in self.modules.items():
            try:
                if hasattr(module, 'initialize'):
                    module.initialize()
                    self.module_states[name] = IntegrationState.INITIALIZING_MODULES
                    print(f"Initialized module: {name}")
            except Exception as e:
                print(f"Error initializing module {name}: {e}")
                self.module_states[name] = IntegrationState.FAILING

        # Connect interfaces
        self.state = IntegrationState.CONNECTING_INTERFACES
        self.setup_module_connections()

        # Validate integration
        self.state = IntegrationState.VALIDATING_INTEGRATION
        if self.validate_integration():
            self.state = IntegrationState.OPERATIONAL
            print("System integration validated successfully")
        else:
            self.state = IntegrationState.FAILING
            print("System integration validation failed")

    def setup_module_connections(self):
        """
        Setup connections between modules based on requirements
        """
        # Example connections - in practice, this would be configured
        # based on system requirements
        connections = [
            # Perception -> Navigation (obstacle data)
            ('perception', 'objects', 'navigation', 'obstacles'),
            # Interaction -> Navigation (destination goals)
            ('interaction', 'destination', 'navigation', 'goal'),
            # Navigation -> Control (motion commands)
            ('navigation', 'commands', 'control', 'motion'),
        ]

        for src_mod, src_int, tgt_mod, tgt_int in connections:
            try:
                self.connect_modules(src_mod, src_int, tgt_mod, tgt_int)
            except Exception as e:
                print(f"Error connecting {src_mod}.{src_int} to {tgt_mod}.{tgt_int}: {e}")

    def validate_integration(self) -> bool:
        """
        Validate that the integration is working correctly
        """
        # Check that all modules are operational
        operational_modules = sum(
            1 for state in self.module_states.values()
            if state in [IntegrationState.OPERATIONAL, IntegrationState.DEGRADED]
        )

        total_modules = len(self.module_states)

        if operational_modules == 0:
            return False

        # Check interface connections
        connected_interfaces = sum(
            len(connections) for connections in self.interface_manager.connections.values()
        )

        # Basic validation: at least some interfaces should be connected
        if connected_interfaces == 0 and total_modules > 1:
            return False

        return True

    def run_integration_loop(self):
        """
        Run the main integration loop
        """
        while self.state not in [IntegrationState.SHUTTING_DOWN, IntegrationState.FAILING]:
            # Monitor system health
            self.monitor_system()

            # Process integration tasks
            self.process_integration_tasks()

            # Update metrics
            self.update_metrics()

            # Sleep briefly to avoid busy waiting
            time.sleep(0.1)

    def monitor_system(self):
        """
        Monitor the integrated system
        """
        # Record system metrics
        self.monitor.record_metric('integration_state', self.state.value)
        self.monitor.record_metric('module_count', len(self.modules))

        # Check for issues
        if self.monitor.check_threshold('error_rate', 0.1, 'gt'):
            self.monitor.generate_alert('high_error_rate', 'Error rate exceeds threshold', 'critical')

        if self.monitor.check_threshold('cpu_usage', 90, 'gt'):
            self.monitor.generate_alert('high_cpu_usage', 'CPU usage exceeds 90%', 'warning')

        # Update module states
        for name, module in self.modules.items():
            if hasattr(module, 'get_state'):
                module_state = module.get_state()
                self.module_states[name] = module_state

    def update_metrics(self):
        """
        Update system metrics
        """
        # This would integrate with system monitoring tools
        # For now, we'll simulate some metrics
        import random

        self.monitor.record_metric('cpu_usage', random.uniform(20, 80))
        self.monitor.record_metric('memory_usage', random.uniform(30, 70))
        self.monitor.record_metric('error_rate', random.uniform(0, 0.05))

    def process_integration_tasks(self):
        """
        Process integration-specific tasks
        """
        # Process any pending integration tasks
        completed_tasks = []
        for i, task in enumerate(self.integration_tasks):
            if task.done():
                completed_tasks.append(i)

        # Remove completed tasks
        for i in reversed(completed_tasks):
            self.integration_tasks.pop(i)

    def get_system_health_report(self) -> Dict[str, Any]:
        """
        Generate a comprehensive system health report
        """
        report = {
            'timestamp': time.time(),
            'integration_state': self.state.value,
            'module_states': {name: state.value for name, state in self.module_states.items()},
            'total_modules': len(self.modules),
            'operational_modules': sum(1 for state in self.module_states.values()
                                     if state == IntegrationState.OPERATIONAL),
            'health_score': self.monitor.get_health_score(),
            'recent_alerts': self.monitor.alerts[-10:],  # Last 10 alerts
            'metrics_summary': {
                name: metrics[-1]['value'] if metrics else None
                for name, metrics in self.monitor.metrics.items()
            }
        }

        return report

    def shutdown(self):
        """
        Shutdown the integrated system
        """
        self.state = IntegrationState.SHUTTING_DOWN

        # Shutdown all modules
        for name, module in self.modules.items():
            try:
                if hasattr(module, 'shutdown'):
                    module.shutdown()
            except Exception as e:
                print(f"Error shutting down module {name}: {e}")

        # Shutdown executor
        self.executor.shutdown(wait=True)

        print("System integration shutdown complete")

# Example module implementations
class ExamplePerceptionModule:
    """
    Example perception module implementation
    """

    def __init__(self):
        self.state = IntegrationState.STARTING
        self.objects_detected = 0

    def initialize(self):
        """
        Initialize the perception module
        """
        print("Initializing perception module...")
        self.state = IntegrationState.OPERATIONAL
        return True

    def get_interfaces(self):
        """
        Get module interfaces
        """
        return {
            'objects': {
                'type': 'publisher',
                'message_type': 'detection_array',
                'frequency': 10.0
            },
            'scene': {
                'type': 'publisher',
                'message_type': 'scene_description',
                'frequency': 1.0
            }
        }

    def get_state(self):
        """
        Get current module state
        """
        return self.state

    def shutdown(self):
        """
        Shutdown the module
        """
        self.state = IntegrationState.SHUTTING_DOWN
        print("Perception module shutdown")

class ExampleNavigationModule:
    """
    Example navigation module implementation
    """

    def __init__(self):
        self.state = IntegrationState.STARTING
        self.active_goals = 0

    def initialize(self):
        """
        Initialize the navigation module
        """
        print("Initializing navigation module...")
        self.state = IntegrationState.OPERATIONAL
        return True

    def get_interfaces(self):
        """
        Get module interfaces
        """
        return {
            'goal': {
                'type': 'subscriber',
                'message_type': 'pose_stamped',
                'frequency': 1.0
            },
            'commands': {
                'type': 'publisher',
                'message_type': 'twist',
                'frequency': 50.0
            },
            'obstacles': {
                'type': 'subscriber',
                'message_type': 'detection_array',
                'frequency': 10.0
            }
        }

    def get_state(self):
        """
        Get current module state
        """
        return self.state

    def shutdown(self):
        """
        Shutdown the module
        """
        self.state = IntegrationState.SHUTTING_DOWN
        print("Navigation module shutdown")

def main_integration_example():
    """
    Example of using the advanced integration system
    """
    # Create the integrator
    integrator = AdvancedSystemIntegrator()

    # Create and register modules
    perception = ExamplePerceptionModule()
    navigation = ExampleNavigationModule()

    integrator.register_module('perception', perception)
    integrator.register_module('navigation', navigation)

    # Start integration
    integrator.start_integration()

    # Run for a while to demonstrate operation
    try:
        integrator.run_integration_loop()
    except KeyboardInterrupt:
        print("\nShutting down...")
        integrator.shutdown()

if __name__ == "__main__":
    main_integration_example()
```

## Code Examples with Explanations

### Communication Architecture Implementation

```python
import asyncio
import zmq
import json
import threading
from typing import Dict, Any, Callable, Optional
from dataclasses import dataclass
import time

@dataclass
class Message:
    """
    Standardized message format for system communication
    """
    source: str
    destination: str
    message_type: str
    data: Dict[str, Any]
    timestamp: float
    correlation_id: Optional[str] = None
    priority: int = 1  # 0=low, 1=normal, 2=high

class CommunicationBus:
    """
    Central communication bus for system integration
    """

    def __init__(self):
        # ZMQ context for high-performance messaging
        self.context = zmq.Context()

        # Publishers for different message types
        self.publishers: Dict[str, zmq.Socket] = {}

        # Subscribers for different message types
        self.subscribers: Dict[str, zmq.Socket] = {}

        # Callback registry
        self.callbacks: Dict[str, List[Callable]] = {}

        # Message queues for different priorities
        self.message_queues = {
            0: asyncio.Queue(),  # Low priority
            1: asyncio.Queue(),  # Normal priority
            2: asyncio.Queue()   # High priority
        }

        # Start message processing loop
        self.processing_task = None
        self.running = False

    def setup_publisher(self, message_type: str, port: int = None):
        """
        Setup a publisher for a specific message type
        """
        if message_type not in self.publishers:
            socket = self.context.socket(zmq.PUB)
            if port:
                socket.bind(f"tcp://*:{port}")
            else:
                port = socket.bind_to_random_port("tcp://*")
            self.publishers[message_type] = socket
            print(f"Setup publisher for {message_type} on port {port}")

    def setup_subscriber(self, message_type: str, address: str):
        """
        Setup a subscriber for a specific message type
        """
        if message_type not in self.subscribers:
            socket = self.context.socket(zmq.SUB)
            socket.connect(address)
            socket.setsockopt_string(zmq.SUBSCRIBE, message_type)
            self.subscribers[message_type] = socket

    def subscribe(self, message_type: str, callback: Callable):
        """
        Subscribe to messages of a specific type
        """
        if message_type not in self.callbacks:
            self.callbacks[message_type] = []
        self.callbacks[message_type].append(callback)

    def send_message(self, message: Message):
        """
        Send a message through the communication bus
        """
        # Serialize message
        serialized_msg = json.dumps({
            'source': message.source,
            'destination': message.destination,
            'type': message.message_type,
            'data': message.data,
            'timestamp': message.timestamp,
            'correlation_id': message.correlation_id,
            'priority': message.priority
        })

        # Send via appropriate publisher
        if message.message_type in self.publishers:
            publisher = self.publishers[message.message_type]
            publisher.send_string(f"{message.message_type} {serialized_msg}")

        # Also queue for local processing
        self.message_queues[message.priority].put_nowait(message)

    async def process_messages(self):
        """
        Process messages from all queues
        """
        self.running = True
        while self.running:
            # Process high priority messages first
            await self._process_queue(2)
            await self._process_queue(1)
            await self._process_queue(0)

            await asyncio.sleep(0.001)  # Brief pause to prevent busy waiting

    async def _process_queue(self, priority: int):
        """
        Process messages from a specific priority queue
        """
        queue = self.message_queues[priority]
        while not queue.empty():
            try:
                message = queue.get_nowait()

                # Deliver to registered callbacks
                if message.message_type in self.callbacks:
                    for callback in self.callbacks[message.message_type]:
                        try:
                            # Call callback in executor to prevent blocking
                            await asyncio.get_event_loop().run_in_executor(
                                None, callback, message
                            )
                        except Exception as e:
                            print(f"Error in callback: {e}")
            except asyncio.QueueEmpty:
                break

    def start_processing(self):
        """
        Start the message processing loop
        """
        if not self.processing_task:
            self.processing_task = asyncio.create_task(self.process_messages())

    def stop_processing(self):
        """
        Stop the message processing loop
        """
        self.running = False
        if self.processing_task:
            self.processing_task.cancel()

class ModuleCommunicator:
    """
    Communication interface for individual modules
    """

    def __init__(self, module_name: str, bus: CommunicationBus):
        self.module_name = module_name
        self.bus = bus
        self.response_handlers: Dict[str, asyncio.Future] = {}

    def setup_publisher(self, message_type: str, port: int = None):
        """
        Setup publisher for this module
        """
        self.bus.setup_publisher(f"{self.module_name}.{message_type}", port)

    def setup_subscriber(self, source_module: str, message_type: str, address: str):
        """
        Setup subscriber to another module
        """
        self.bus.setup_subscriber(f"{source_module}.{message_type}", address)

    def send_request(self, destination_module: str, message_type: str,
                     data: Dict[str, Any], timeout: float = 5.0) -> str:
        """
        Send a request and return correlation ID for response tracking
        """
        correlation_id = f"{self.module_name}_{int(time.time() * 1000000)}"

        message = Message(
            source=self.module_name,
            destination=destination_module,
            message_type=f"{destination_module}.{message_type}",
            data=data,
            timestamp=time.time(),
            correlation_id=correlation_id,
            priority=2  # Requests are high priority
        )

        # Create future for response
        future = asyncio.Future()
        self.response_handlers[correlation_id] = future

        # Send message
        self.bus.send_message(message)

        # Set timeout
        asyncio.create_task(self._set_timeout(correlation_id, timeout))

        return correlation_id

    async def _set_timeout(self, correlation_id: str, timeout: float):
        """
        Set timeout for response
        """
        await asyncio.sleep(timeout)
        if correlation_id in self.response_handlers:
            future = self.response_handlers[correlation_id]
            if not future.done():
                future.set_exception(asyncio.TimeoutError(f"Request {correlation_id} timed out"))

    def send_notification(self, destination_module: str, message_type: str,
                         data: Dict[str, Any]):
        """
        Send a notification (no response expected)
        """
        message = Message(
            source=self.module_name,
            destination=destination_module,
            message_type=f"{destination_module}.{message_type}",
            data=data,
            timestamp=time.time(),
            priority=1  # Normal priority
        )
        self.bus.send_message(message)

    def register_handler(self, message_type: str, handler: Callable):
        """
        Register handler for incoming messages
        """
        full_message_type = f"{self.module_name}.{message_type}"
        self.bus.subscribe(full_message_type, handler)

    async def wait_for_response(self, correlation_id: str) -> Dict[str, Any]:
        """
        Wait for response to a request
        """
        if correlation_id in self.response_handlers:
            future = self.response_handlers[correlation_id]
            try:
                result = await future
                del self.response_handlers[correlation_id]
                return result
            except asyncio.TimeoutError:
                del self.response_handlers[correlation_id]
                raise
        else:
            raise ValueError(f"No request with correlation ID {correlation_id}")

# Example usage
async def example_communication():
    """
    Example of using the communication system
    """
    # Create communication bus
    bus = CommunicationBus()
    bus.start_processing()

    # Create module communicators
    perception_comm = ModuleCommunicator("perception", bus)
    navigation_comm = ModuleCommunicator("navigation", bus)

    # Setup communication channels
    perception_comm.setup_publisher("objects_detected")
    navigation_comm.setup_publisher("navigation_status")

    # Subscribe to messages
    def handle_objects_detected(message: Message):
        print(f"Navigation received: {message.data}")
        # Process objects and update navigation plan

    navigation_comm.register_handler("objects_detected", handle_objects_detected)

    # Send a test message
    test_data = {
        "objects": [
            {"type": "chair", "position": [1.0, 2.0, 0.0]},
            {"type": "table", "position": [3.0, 1.0, 0.0]}
        ]
    }

    perception_comm.send_notification("navigation", "objects_detected", test_data)

    # Keep running for demonstration
    await asyncio.sleep(2)

    # Cleanup
    bus.stop_processing()

if __name__ == "__main__":
    asyncio.run(example_communication())
```

### Integration Testing Framework

```python
import unittest
import asyncio
import time
from typing import Dict, List, Any
from dataclasses import dataclass
import threading

@dataclass
class IntegrationTestResult:
    """
    Result of an integration test
    """
    test_name: str
    passed: bool
    duration: float
    errors: List[str]
    metrics: Dict[str, Any]

class IntegrationTestSuite:
    """
    Comprehensive integration testing framework
    """

    def __init__(self):
        self.tests: List[Callable] = []
        self.results: List[IntegrationTestResult] = []
        self.setup_functions: List[Callable] = []
        self.teardown_functions: List[Callable] = []

    def add_test(self, test_func: Callable):
        """
        Add a test function to the suite
        """
        self.tests.append(test_func)

    def add_setup(self, setup_func: Callable):
        """
        Add a setup function
        """
        self.setup_functions.append(setup_func)

    def add_teardown(self, teardown_func: Callable):
        """
        Add a teardown function
        """
        self.teardown_functions.append(teardown_func)

    def run_all_tests(self) -> List[IntegrationTestResult]:
        """
        Run all tests in the suite
        """
        self.results = []

        for test_func in self.tests:
            result = self.run_test(test_func)
            self.results.append(result)

        return self.results

    def run_test(self, test_func: Callable) -> IntegrationTestResult:
        """
        Run a single test
        """
        start_time = time.time()
        errors = []

        try:
            # Run setup functions
            for setup_func in self.setup_functions:
                setup_func()

            # Run the test
            test_func()

            # Run teardown functions
            for teardown_func in self.teardown_functions:
                teardown_func()

        except Exception as e:
            errors.append(str(e))

        duration = time.time() - start_time

        result = IntegrationTestResult(
            test_name=test_func.__name__,
            passed=len(errors) == 0,
            duration=duration,
            errors=errors,
            metrics={}
        )

        return result

    def get_summary(self) -> Dict[str, Any]:
        """
        Get summary of test results
        """
        total_tests = len(self.results)
        passed_tests = sum(1 for result in self.results if result.passed)
        failed_tests = total_tests - passed_tests

        total_duration = sum(result.duration for result in self.results)

        return {
            'total_tests': total_tests,
            'passed_tests': passed_tests,
            'failed_tests': failed_tests,
            'success_rate': passed_tests / total_tests if total_tests > 0 else 0,
            'total_duration': total_duration,
            'average_duration': total_duration / total_tests if total_tests > 0 else 0
        }

    def print_report(self):
        """
        Print a detailed test report
        """
        summary = self.get_summary()

        print("\n" + "="*60)
        print("INTEGRATION TEST REPORT")
        print("="*60)
        print(f"Total Tests: {summary['total_tests']}")
        print(f"Passed: {summary['passed_tests']}")
        print(f"Failed: {summary['failed_tests']}")
        print(f"Success Rate: {summary['success_rate']:.2%}")
        print(f"Total Duration: {summary['total_duration']:.2f}s")
        print(f"Average Duration: {summary['average_duration']:.2f}s")
        print("-"*60)

        for result in self.results:
            status = "PASS" if result.passed else "FAIL"
            print(f"{result.test_name:<30} {status:<10} {result.duration:.2f}s")
            if result.errors:
                for error in result.errors:
                    print(f"  ERROR: {error}")

        print("="*60)

class SystemIntegrationTester:
    """
    Specialized tester for robotic system integration
    """

    def __init__(self):
        self.test_suite = IntegrationTestSuite()

        # Add common setup and teardown
        self.test_suite.add_setup(self.setup_test_environment)
        self.test_suite.add_teardown(self.teardown_test_environment)

        # Add specific integration tests
        self.add_integration_tests()

    def setup_test_environment(self):
        """
        Setup function for integration tests
        """
        print("Setting up test environment...")
        # Initialize mock systems, reset state, etc.

    def teardown_test_environment(self):
        """
        Teardown function for integration tests
        """
        print("Tearing down test environment...")
        # Clean up resources, reset systems, etc.

    def add_integration_tests(self):
        """
        Add specific integration tests
        """
        self.test_suite.add_test(self.test_perception_navigation_integration)
        self.test_suite.add_test(self.test_interaction_navigation_integration)
        self.test_suite.add_test(self.test_full_system_integration)
        self.test_suite.add_test(self.test_error_recovery)
        self.test_suite.add_test(self.test_performance_under_load)

    def test_perception_navigation_integration(self):
        """
        Test integration between perception and navigation
        """
        # Simulate object detection
        detected_objects = [
            {"type": "obstacle", "position": [1.0, 0.0, 0.0], "distance": 1.0}
        ]

        # Verify navigation system receives and processes the information
        # This would involve checking internal state or using mocks
        assert len(detected_objects) > 0, "Should detect objects"

        # Simulate navigation response to obstacles
        planned_path = self.calculate_avoidance_path(detected_objects)
        assert planned_path is not None, "Should calculate avoidance path"

    def test_interaction_navigation_integration(self):
        """
        Test integration between interaction and navigation
        """
        # Simulate voice command for navigation
        voice_command = "Go to the kitchen"

        # Process command and extract destination
        destination = self.extract_destination_from_command(voice_command)
        assert destination == "kitchen", "Should extract correct destination"

        # Verify navigation system receives destination
        navigation_goal_set = self.verify_navigation_goal_set(destination)
        assert navigation_goal_set, "Navigation goal should be set"

    def test_full_system_integration(self):
        """
        Test complete system integration
        """
        # Simulate a complete user interaction flow
        # 1. Voice command received
        # 2. Command processed by interaction system
        # 3. Navigation goal set
        # 4. Perception system activated
        # 5. Path planning executed
        # 6. Navigation executed
        # 7. Completion reported

        flow_completed = self.simulate_complete_interaction_flow()
        assert flow_completed, "Complete interaction flow should complete successfully"

    def test_error_recovery(self):
        """
        Test system's ability to recover from errors
        """
        # Simulate an error in one subsystem
        error_occurred = self.simulate_subsystem_error()
        assert error_occurred, "Error should occur"

        # Verify system can recover
        recovery_successful = self.verify_error_recovery()
        assert recovery_successful, "System should recover from error"

    def test_performance_under_load(self):
        """
        Test system performance under load
        """
        import time

        start_time = time.time()

        # Simulate multiple concurrent operations
        operations = []
        for i in range(10):  # Simulate 10 concurrent operations
            operation_result = self.simulate_concurrent_operation(i)
            operations.append(operation_result)

        end_time = time.time()
        duration = end_time - start_time

        # Verify all operations completed successfully
        all_successful = all(operations)
        assert all_successful, "All operations should complete successfully"

        # Verify performance is acceptable
        assert duration < 5.0, f"Operations took too long: {duration}s"

    # Helper methods (simulated implementations)
    def calculate_avoidance_path(self, obstacles):
        """
        Simulated path calculation
        """
        return [{"x": 2.0, "y": 0.0}, {"x": 2.0, "y": 1.0}]  # Simple path around obstacle

    def extract_destination_from_command(self, command):
        """
        Simulated destination extraction
        """
        if "kitchen" in command.lower():
            return "kitchen"
        return None

    def verify_navigation_goal_set(self, destination):
        """
        Simulated verification
        """
        return True  # Assume verification passes

    def simulate_complete_interaction_flow(self):
        """
        Simulated complete flow
        """
        return True  # Assume flow completes successfully

    def simulate_subsystem_error(self):
        """
        Simulated error
        """
        return True  # Simulate that an error occurred

    def verify_error_recovery(self):
        """
        Simulated recovery verification
        """
        return True  # Assume recovery was successful

    def simulate_concurrent_operation(self, operation_id):
        """
        Simulated concurrent operation
        """
        import time
        time.sleep(0.1)  # Simulate processing time
        return True  # Assume operation succeeds

    def run_tests(self):
        """
        Run all integration tests and return results
        """
        results = self.test_suite.run_all_tests()
        self.test_suite.print_report()
        return results

def run_integration_tests():
    """
    Function to run the complete integration test suite
    """
    tester = SystemIntegrationTester()
    results = tester.run_tests()
    return results

# Example of using the tester
if __name__ == "__main__":
    results = run_integration_tests()
```

## Hands-On Exercises: Integration Implementation

### Exercise 1: Real-time Integration Pipeline

Create a real-time integration pipeline that connects multiple subsystems:

```python
import asyncio
import threading
import queue
import time
from typing import Dict, List, Callable, Any
from dataclasses import dataclass

@dataclass
class IntegrationPipelineConfig:
    """
    Configuration for the integration pipeline
    """
    buffer_size: int = 100
    processing_rate: float = 30.0  # Hz
    timeout: float = 1.0  # seconds
    enable_monitoring: bool = True

class RealTimeIntegrationPipeline:
    """
    Exercise: Real-time integration pipeline connecting multiple subsystems
    """

    def __init__(self, config: IntegrationPipelineConfig):
        self.config = config
        self.active = False

        # Data queues for different subsystems
        self.perception_queue = queue.Queue(maxsize=config.buffer_size)
        self.navigation_queue = queue.Queue(maxsize=config.buffer_size)
        self.interaction_queue = queue.Queue(maxsize=config.buffer_size)

        # Processed data queues
        self.output_queues: Dict[str, queue.Queue] = {
            'navigation_commands': queue.Queue(maxsize=config.buffer_size),
            'system_status': queue.Queue(maxsize=config.buffer_size)
        }

        # Processing callbacks
        self.processing_callbacks: Dict[str, Callable] = {}

        # Performance metrics
        self.metrics = {
            'processing_rate': 0.0,
            'latency': 0.0,
            'throughput': 0.0
        }

    def register_subsystem(self, name: str, callback: Callable):
        """
        Register a subsystem with its processing callback
        """
        self.processing_callbacks[name] = callback

    def add_perception_data(self, data: Dict[str, Any]):
        """
        Add perception data to the pipeline
        """
        try:
            self.perception_queue.put_nowait(data)
        except queue.Full:
            print("Perception queue full, dropping data")

    def add_navigation_data(self, data: Dict[str, Any]):
        """
        Add navigation data to the pipeline
        """
        try:
            self.navigation_queue.put_nowait(data)
        except queue.Full:
            print("Navigation queue full, dropping data")

    def add_interaction_data(self, data: Dict[str, Any]):
        """
        Add interaction data to the pipeline
        """
        try:
            self.interaction_queue.put_nowait(data)
        except queue.Full:
            print("Interaction queue full, dropping data")

    def get_output(self, output_type: str) -> Any:
        """
        Get processed output from the pipeline
        """
        if output_type in self.output_queues:
            try:
                return self.output_queues[output_type].get_nowait()
            except queue.Empty:
                return None
        return None

    async def run_pipeline(self):
        """
        Run the real-time integration pipeline
        """
        self.active = True
        last_process_time = time.time()

        while self.active:
            start_time = time.time()

            # Process data from all subsystems
            await self._process_perception_data()
            await self._process_navigation_data()
            await self._process_interaction_data()

            # Integrate data across subsystems
            await self._integrate_subsystems()

            # Calculate performance metrics
            process_time = time.time() - start_time
            self.metrics['latency'] = process_time
            self.metrics['processing_rate'] = 1.0 / process_time if process_time > 0 else 0

            # Maintain target processing rate
            target_interval = 1.0 / self.config.processing_rate
            sleep_time = max(0, target_interval - process_time)
            await asyncio.sleep(sleep_time)

    async def _process_perception_data(self):
        """
        Process perception data
        """
        if not self.perception_queue.empty():
            try:
                data = self.perception_queue.get_nowait()
                if 'perception' in self.processing_callbacks:
                    result = self.processing_callbacks['perception'](data)
                    # Add result to integration buffer
            except queue.Empty:
                pass

    async def _process_navigation_data(self):
        """
        Process navigation data
        """
        if not self.navigation_queue.empty():
            try:
                data = self.navigation_queue.get_nowait()
                if 'navigation' in self.processing_callbacks:
                    result = self.processing_callbacks['navigation'](data)
                    # Add result to integration buffer
            except queue.Empty:
                pass

    async def _process_interaction_data(self):
        """
        Process interaction data
        """
        if not self.interaction_queue.empty():
            try:
                data = self.interaction_queue.get_nowait()
                if 'interaction' in self.processing_callbacks:
                    result = self.processing_callbacks['interaction'](data)
                    # Add result to integration buffer
            except queue.Empty:
                pass

    async def _integrate_subsystems(self):
        """
        Integrate data from all subsystems
        """
        # This is where the magic happens - combining data from all subsystems
        # to make coordinated decisions

        # Example integration logic:
        # - If perception detects obstacle AND navigation is active, adjust path
        # - If interaction requests navigation AND no obstacles, set goal
        # - Update system status based on all inputs

        integration_result = {
            'timestamp': time.time(),
            'subsystem_status': {
                'perception': not self.perception_queue.empty(),
                'navigation': not self.navigation_queue.empty(),
                'interaction': not self.interaction_queue.empty()
            }
        }

        # Publish integration result
        try:
            self.output_queues['system_status'].put_nowait(integration_result)
        except queue.Full:
            print("System status queue full")

    def stop_pipeline(self):
        """
        Stop the integration pipeline
        """
        self.active = False

# Example usage and testing
async def test_integration_pipeline():
    """
    Test the integration pipeline
    """
    config = IntegrationPipelineConfig(
        buffer_size=50,
        processing_rate=20.0,  # 20 Hz processing
        timeout=0.5
    )

    pipeline = RealTimeIntegrationPipeline(config)

    # Register processing callbacks
    def perception_callback(data):
        print(f"Processing perception: {data}")
        return {"processed": True, "type": "perception"}

    def navigation_callback(data):
        print(f"Processing navigation: {data}")
        return {"processed": True, "type": "navigation"}

    def interaction_callback(data):
        print(f"Processing interaction: {data}")
        return {"processed": True, "type": "interaction"}

    pipeline.register_subsystem('perception', perception_callback)
    pipeline.register_subsystem('navigation', navigation_callback)
    pipeline.register_subsystem('interaction', interaction_callback)

    # Start pipeline in background
    pipeline_task = asyncio.create_task(pipeline.run_pipeline())

    # Simulate data input
    for i in range(100):
        # Add simulated data
        pipeline.add_perception_data({"objects": [f"object_{i}"], "timestamp": time.time()})
        pipeline.add_navigation_data({"position": [i*0.1, 0, 0], "goal": [5, 5, 0]})
        pipeline.add_interaction_data({"command": f"command_{i}", "source": "user"})

        await asyncio.sleep(0.1)  # 10 Hz input rate

    # Stop pipeline
    pipeline.stop_pipeline()
    await pipeline_task

if __name__ == "__main__":
    asyncio.run(test_integration_pipeline())
```

### Exercise 2: Fault-Tolerant Integration System

Create a fault-tolerant integration system that handles subsystem failures:

```python
import asyncio
import random
from typing import Dict, List, Callable, Optional
from enum import Enum
import time

class SubsystemState(Enum):
    """
    States for subsystems in fault-tolerant system
    """
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    FAILED = "failed"
    RECOVERING = "recovering"

class FaultTolerantIntegrator:
    """
    Exercise: Fault-tolerant system integration
    """

    def __init__(self):
        self.subsystems: Dict[str, Dict] = {}
        self.fallback_strategies: Dict[str, List[Callable]] = {}
        self.health_monitor = SubsystemHealthMonitor()
        self.active = False

    def register_subsystem(self, name: str, primary_func: Callable,
                          fallback_funcs: List[Callable] = None):
        """
        Register a subsystem with primary and fallback functions
        """
        self.subsystems[name] = {
            'primary': primary_func,
            'fallbacks': fallback_funcs or [],
            'state': SubsystemState.HEALTHY,
            'last_heartbeat': time.time(),
            'error_count': 0,
            'success_count': 0
        }

    def register_fallback_strategy(self, subsystem: str, strategy: Callable):
        """
        Register a fallback strategy for a subsystem
        """
        if subsystem not in self.fallback_strategies:
            self.fallback_strategies[system] = []
        self.fallback_strategies[system].append(strategy)

    async def execute_with_fault_tolerance(self, subsystem: str, *args, **kwargs):
        """
        Execute a subsystem operation with fault tolerance
        """
        if subsystem not in self.subsystems:
            raise ValueError(f"Unknown subsystem: {subsystem}")

        subsystem_info = self.subsystems[system]

        # Check if subsystem is healthy
        if subsystem_info['state'] == SubsystemState.FAILED:
            # Try fallback strategies
            return await self._execute_fallback(subsystem, *args, **kwargs)

        # Try primary function
        try:
            result = await self._execute_function(subsystem_info['primary'], *args, **kwargs)
            self._update_success(subsystem)
            return result
        except Exception as e:
            print(f"Primary function failed for {subsystem}: {e}")
            self._update_error(subsystem)

            # Mark as degraded if error threshold exceeded
            if subsystem_info['error_count'] > 3:
                subsystem_info['state'] = SubsystemState.DEGRADED
                print(f"{subsystem} marked as degraded")

            # Try fallback
            return await self._execute_fallback(subsystem, *args, **kwargs)

    async def _execute_function(self, func: Callable, *args, **kwargs):
        """
        Execute a function with timeout
        """
        try:
            if asyncio.iscoroutinefunction(func):
                return await asyncio.wait_for(func(*args, **kwargs), timeout=2.0)
            else:
                # Run sync function in thread pool
                loop = asyncio.get_event_loop()
                return await loop.run_in_executor(None, func, *args, **kwargs)
        except asyncio.TimeoutError:
            raise Exception("Function execution timed out")

    async def _execute_fallback(self, subsystem: str, *args, **kwargs):
        """
        Execute fallback strategies
        """
        subsystem_info = self.subsystems[system]

        # Try registered fallback functions first
        for fallback_func in subsystem_info['fallbacks']:
            try:
                result = await self._execute_function(fallback_func, *args, **kwargs)
                print(f"Fallback successful for {subsystem}")
                return result
            except Exception as e:
                print(f"Fallback function failed: {e}")
                continue

        # Try fallback strategies
        if subsystem in self.fallback_strategies:
            for strategy in self.fallback_strategies[system]:
                try:
                    result = await self._execute_function(strategy, *args, **kwargs)
                    print(f"Fallback strategy successful for {subsystem}")
                    return result
                except Exception as e:
                    print(f"Fallback strategy failed: {e}")
                    continue

        # If all fallbacks fail, mark as failed
        subsystem_info['state'] = SubsystemState.FAILED
        raise Exception(f"All fallback strategies failed for {subsystem}")

    def _update_success(self, subsystem: str):
        """
        Update success metrics for subsystem
        """
        subsystem_info = self.subsystems[system]
        subsystem_info['success_count'] += 1
        subsystem_info['error_count'] = max(0, subsystem_info['error_count'] - 1)

        # If recovered from degraded state
        if subsystem_info['state'] == SubsystemState.DEGRADED:
            if subsystem_info['success_count'] > subsystem_info['error_count'] * 2:
                subsystem_info['state'] = SubsystemState.HEALTHY
                print(f"{subsystem} recovered to healthy state")

    def _update_error(self, subsystem: str):
        """
        Update error metrics for subsystem
        """
        subsystem_info = self.subsystems[system]
        subsystem_info['error_count'] += 1

    async def monitor_subsystems(self):
        """
        Monitor subsystem health and trigger recovery
        """
        while self.active:
            current_time = time.time()

            for name, info in self.subsystems.items():
                # Check heartbeat timeout
                if current_time - info['last_heartbeat'] > 5.0:  # 5 second timeout
                    if info['state'] != SubsystemState.FAILED:
                        print(f"{name} heartbeat timeout, marking as failed")
                        info['state'] = SubsystemState.FAILED

                # Trigger recovery for failed subsystems
                if info['state'] == SubsystemState.FAILED:
                    await self._attempt_recovery(name)

            await asyncio.sleep(1.0)  # Check every second

    async def _attempt_recovery(self, subsystem: str):
        """
        Attempt to recover a failed subsystem
        """
        print(f"Attempting to recover {subsystem}...")

        # Simulate recovery process
        await asyncio.sleep(0.5)

        # Random chance of recovery (in practice, this would be actual recovery logic)
        if random.random() > 0.3:  # 70% chance of recovery
            self.subsystems[subsystem]['state'] = SubsystemState.RECOVERING
            print(f"{subsystem} recovery initiated")

            # Simulate recovery time
            await asyncio.sleep(1.0)

            # Recovery complete
            self.subsystems[subsystem]['state'] = SubsystemState.HEALTHY
            self.subsystems[subsystem]['last_heartbeat'] = time.time()
            print(f"{subsystem} recovered successfully")
        else:
            print(f"{subsystem} recovery failed")

    def start_monitoring(self):
        """
        Start subsystem monitoring
        """
        self.active = True
        return asyncio.create_task(self.monitor_subsystems())

    def stop_monitoring(self):
        """
        Stop subsystem monitoring
        """
        self.active = False

class SubsystemHealthMonitor:
    """
    Health monitoring for subsystems
    """

    def __init__(self):
        self.health_history: Dict[str, List[Dict]] = {}
        self.alerts: List[Dict] = []

    def record_health(self, subsystem: str, state: SubsystemState, metrics: Dict = None):
        """
        Record health status of a subsystem
        """
        if subsystem not in self.health_history:
            self.health_history[subsystem] = []

        record = {
            'timestamp': time.time(),
            'state': state.value,
            'metrics': metrics or {}
        }

        self.health_history[subsystem].append(record)

        # Keep only recent history
        if len(self.health_history[subsystem]) > 100:
            self.health_history[subsystem].pop(0)

    def get_health_score(self, subsystem: str) -> float:
        """
        Calculate health score (0.0 to 1.0) for a subsystem
        """
        if subsystem not in self.health_history:
            return 0.5  # Default score

        recent_records = self.health_history[subsystem][-10:]  # Last 10 records

        healthy_count = sum(1 for record in recent_records
                          if record['state'] == SubsystemState.HEALTHY.value)

        return healthy_count / len(recent_records) if recent_records else 0.5

    def generate_alert(self, subsystem: str, alert_type: str, message: str):
        """
        Generate a health alert
        """
        alert = {
            'timestamp': time.time(),
            'subsystem': subsystem,
            'type': alert_type,
            'message': message
        }
        self.alerts.append(alert)

# Example usage
async def test_fault_tolerant_integration():
    """
    Test the fault-tolerant integration system
    """
    integrator = FaultTolerantIntegrator()

    # Define subsystem functions
    async def perception_primary(data):
        # Simulate occasional failures
        if random.random() < 0.1:  # 10% failure rate
            raise Exception("Perception sensor error")
        return {"objects": ["detected"], "status": "ok"}

    async def perception_fallback1(data):
        # Simpler perception with higher success rate
        return {"objects": ["fallback_detected"], "status": "degraded"}

    async def perception_fallback2(data):
        # Even simpler fallback
        return {"objects": ["basic_detection"], "status": "minimal"}

    async def navigation_primary(goal):
        if random.random() < 0.15:  # 15% failure rate
            raise Exception("Navigation planning failed")
        return {"path": ["waypoint1", "waypoint2"], "status": "ok"}

    async def navigation_fallback(goal):
        return {"path": ["direct_route"], "status": "simplified"}

    # Register subsystems
    integrator.register_subsystem(
        'perception',
        perception_primary,
        [perception_fallback1, perception_fallback2]
    )
    integrator.register_subsystem(
        'navigation',
        navigation_primary,
        [navigation_fallback]
    )

    # Start monitoring
    monitor_task = integrator.start_monitoring()

    # Test operations with potential failures
    for i in range(20):
        try:
            # Test perception
            perception_result = await integrator.execute_with_fault_tolerance(
                'perception', {'sensor_data': f'data_{i}'}
            )
            print(f"Perception result {i}: {perception_result}")

            # Test navigation
            navigation_result = await integrator.execute_with_fault_tolerance(
                'navigation', {'goal': [i, i, 0]}
            )
            print(f"Navigation result {i}: {navigation_result}")

        except Exception as e:
            print(f"Operation failed: {e}")

        await asyncio.sleep(0.5)

    # Stop monitoring
    integrator.stop_monitoring()
    await monitor_task

if __name__ == "__main__":
    asyncio.run(test_fault_tolerant_integration())
```

## Common Pitfalls and Solutions

### Pitfall 1: Tight Coupling Between Subsystems
**Problem**: Subsystems become too dependent on each other, making the system brittle.

**Solution**:
- Use well-defined interfaces and APIs
- Implement loose coupling with message-based communication
- Use dependency injection for subsystem dependencies

```python
class DecoupledIntegration:
    """
    Example of loose coupling between subsystems
    """

    def __init__(self):
        self.message_bus = MessageBus()  # Central communication hub
        self.subsystems = {}

    def register_subsystem(self, name: str, subsystem: object):
        """
        Register subsystem with loose coupling
        """
        self.subsystems[name] = subsystem

        # Subscribe to relevant messages
        if hasattr(subsystem, 'get_subscriptions'):
            for msg_type in subsystem.get_subscriptions():
                self.message_bus.subscribe(msg_type, getattr(subsystem, 'handle_message'))
```

### Pitfall 2: Performance Bottlenecks
**Problem**: Integration layer becomes a performance bottleneck.

**Solution**:
- Use asynchronous processing where possible
- Implement parallel processing for independent tasks
- Optimize critical communication paths

### Pitfall 3: Debugging Complex Interactions
**Problem**: Difficult to trace issues across subsystems.

**Solution**:
- Implement comprehensive logging with correlation IDs
- Use distributed tracing
- Create system visualization tools

## Review Questions

1. What are the main patterns for system integration in robotics?
2. How do you design communication architectures for integrated systems?
3. What are the key challenges in integrating multiple subsystems?
4. How do you implement fault tolerance in integrated systems?
5. What testing strategies are effective for integrated systems?

## Project Assignment: Complete Integration System

Create a complete integration system that:
1. Connects all subsystems (perception, navigation, interaction) with proper interfaces
2. Implements fault-tolerant communication between components
3. Provides real-time performance monitoring and metrics
4. Includes comprehensive testing and validation procedures
5. Demonstrates graceful degradation when subsystems fail
6. Implements proper error handling and recovery mechanisms

Your integration system should handle:
- Real-time data flow between subsystems
- Performance optimization under load
- Failure detection and recovery
- System state monitoring and reporting
- Clean shutdown procedures

## Further Resources

- [ROS 2 Design](https://design.ros2.org/)
- [Microservices Architecture Patterns](https://microservices.io/)
- [System Integration Best Practices](https://ieeexplore.ieee.org/document/8765432)
- [Real-time Systems Integration](https://www.cs.cmu.edu/~gallmeister/realtime-book.pdf)
- [Fault-Tolerant Systems](https://www.computer.org/csdl/magazine/co/2015/09/mco2015090006/13rRUy8M7b6)

:::tip
Always design your integration architecture with modularity and testability in mind. The integration layer should be as thin as possible while providing robust communication and coordination between subsystems.
:::