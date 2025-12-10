# Week 12: Embodied AI & Humanoid Control

## Learning Objectives
By the end of this week, students will be able to:
- Understand the principles of embodied AI and how it differs from traditional AI
- Implement humanoid robot control systems using ROS 2
- Design voice-activated control systems for humanoid robots
- Integrate perception, planning, and control for embodied AI systems
- Apply machine learning techniques to humanoid robot behavior

## Introduction to Embodied AI

Embodied AI represents a paradigm shift from traditional AI systems that operate in virtual environments to AI systems that interact with the physical world through robotic bodies. Unlike digital AI that processes text, images, or other digital data, embodied AI must handle the complexities of real-world physics, sensor noise, actuator limitations, and dynamic environments.

### Key Principles of Embodied AI

1. **Embodiment**: The AI system has a physical form that interacts with the environment
2. **Situatedness**: The AI system exists in a specific environment and context
3. **Emergence**: Complex behaviors emerge from simple sensorimotor interactions
4. **Groundedness**: Concepts and understanding are grounded in physical experience

### Differences from Traditional AI

Traditional AI systems operate in well-defined, controlled environments with discrete inputs and outputs. Embodied AI systems must handle:
- Continuous sensor data streams
- Uncertain and noisy information
- Real-time decision making
- Physical constraints and dynamics
- Safety and reliability requirements

## Humanoid Robot Control Systems

Humanoid robots present unique challenges in control engineering due to their complex kinematics, balance requirements, and multi-degree-of-freedom systems. Unlike wheeled robots or manipulator arms, humanoid robots must maintain balance while performing tasks, requiring sophisticated control algorithms.

### Key Components of Humanoid Control

1. **Balance Control**: Maintaining center of mass within support polygon
2. **Locomotion Control**: Walking, stepping, and gait generation
3. **Manipulation Control**: Arm and hand movement coordination
4. **Whole-Body Control**: Coordinated movement of all joints

### Control Architecture

```
High-Level Commands
        ↓
Behavior Manager
        ↓
Motion Planning
        ↓
Trajectory Generation
        ↓
Low-Level Control
        ↓
Robot Hardware
```

## Voice-Activated Control Systems

Integrating voice recognition with humanoid robot control creates natural human-robot interaction. This system combines speech recognition, natural language processing, and robot control to enable intuitive command interfaces.

### System Architecture

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal
import openai
import numpy as np
import math

class CompleteVoiceControlNode(Node):
    def __init__(self):
        super().__init__('complete_voice_control_node')

        # Publishers for different robot components
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, '/joint_trajectory', 10
        )

        # Subscriber for voice commands
        self.voice_subscriber = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10
        )

        # Robot state subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Initialize robot state
        self.current_joint_positions = {}
        self.robot_pose = None
        self.is_balanced = True

        # Command mapping for humanoid control
        self.command_map = {
            'walk forward': self.walk_forward,
            'walk backward': self.walk_backward,
            'turn left': self.turn_left,
            'turn right': self.turn_right,
            'raise arm': self.raise_arm,
            'lower arm': self.lower_arm,
            'wave': self.wave,
            'dance': self.dance,
            'balance': self.balance_robot,
            'stop': self.stop_robot,
        }

        self.get_logger().info('Complete Voice Control Node initialized')

    def joint_state_callback(self, msg):
        """Callback to update current joint positions"""
        for i, name in enumerate(msg.name):
            self.current_joint_positions[name] = msg.position[i]

    def voice_command_callback(self, msg):
        """Process voice commands and execute corresponding robot actions"""
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received voice command: {command}')

        # Parse command and execute
        if command in self.command_map:
            self.command_map[command]()
        else:
            # Try to parse more complex commands
            self.parse_complex_command(command)

    def parse_complex_command(self, command):
        """Parse complex voice commands with parameters"""
        # Example: "walk 2 meters forward" or "turn 90 degrees right"
        words = command.split()

        if 'walk' in command:
            if 'forward' in command or 'ahead' in command:
                distance = self.extract_distance(command)
                self.walk_distance(distance)
            elif 'backward' in command or 'back' in command:
                distance = self.extract_distance(command)
                self.walk_distance(-distance)

        elif 'turn' in command or 'rotate' in command:
            if 'left' in command:
                angle = self.extract_angle(command)
                self.turn_angle(-angle)
            elif 'right' in command:
                angle = self.extract_angle(command)
                self.turn_angle(angle)

        elif 'move' in command and ('arm' in command or 'hand' in command):
            self.parse_arm_command(command)

    def extract_distance(self, command):
        """Extract distance from command string"""
        words = command.split()
        for i, word in enumerate(words):
            if word.replace('.', '').isdigit():
                try:
                    distance = float(word)
                    # Check if next word indicates units
                    if i + 1 < len(words):
                        unit = words[i + 1].lower()
                        if 'meter' in unit:
                            return distance
                        elif 'foot' in unit or 'feet' in unit:
                            return distance * 0.3048  # Convert feet to meters
                        elif 'inch' in unit:
                            return distance * 0.0254  # Convert inches to meters
                    return distance  # Assume meters by default
                except ValueError:
                    continue
        return 1.0  # Default distance if none specified

    def extract_angle(self, command):
        """Extract angle from command string"""
        words = command.split()
        for i, word in enumerate(words):
            if word.replace('.', '').isdigit():
                try:
                    angle = float(word)
                    # Check if next word indicates units
                    if i + 1 < len(words):
                        unit = words[i + 1].lower()
                        if 'degree' in unit:
                            return math.radians(angle)
                        elif 'radian' in unit:
                            return angle
                    return math.radians(angle)  # Default to degrees
                except ValueError:
                    continue
        return math.radians(90)  # Default 90 degrees if none specified

    def parse_arm_command(self, command):
        """Parse arm movement commands"""
        if 'raise' in command or 'up' in command:
            self.raise_arm()
        elif 'lower' in command or 'down' in command:
            self.lower_arm()
        elif 'wave' in command:
            self.wave()
        elif 'point' in command:
            self.point_at_direction(command)

    def walk_forward(self):
        """Execute forward walking motion"""
        self.get_logger().info('Executing walk forward command')
        # Send velocity command for forward movement
        twist = Twist()
        twist.linear.x = 0.5  # m/s
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

        # For actual humanoid walking, we'd need more complex gait patterns
        self.execute_walk_gait('forward')

    def walk_backward(self):
        """Execute backward walking motion"""
        self.get_logger().info('Executing walk backward command')
        twist = Twist()
        twist.linear.x = -0.5  # m/s
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.execute_walk_gait('backward')

    def turn_left(self):
        """Execute left turn"""
        self.get_logger().info('Executing turn left command')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # rad/s
        self.cmd_vel_publisher.publish(twist)

    def turn_right(self):
        """Execute right turn"""
        self.get_logger().info('Executing turn right command')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.5  # rad/s
        self.cmd_vel_publisher.publish(twist)

    def walk_distance(self, distance):
        """Walk a specific distance"""
        self.get_logger().info(f'Walking distance: {distance} meters')
        # Calculate time needed based on velocity
        velocity = 0.5  # m/s
        duration = abs(distance) / velocity

        twist = Twist()
        twist.linear.x = velocity if distance > 0 else -velocity
        twist.angular.z = 0.0

        # Publish command for duration
        self.execute_timed_command(twist, duration)

    def turn_angle(self, angle):
        """Turn by a specific angle"""
        self.get_logger().info(f'Turning angle: {math.degrees(angle)} degrees')
        # Calculate time needed based on angular velocity
        ang_velocity = 0.5  # rad/s
        duration = abs(angle) / ang_velocity

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = ang_velocity if angle > 0 else -ang_velocity

        # Publish command for duration
        self.execute_timed_command(twist, duration)

    def raise_arm(self):
        """Raise the robot's arm"""
        self.get_logger().info('Raising arm')
        # Create joint trajectory for arm raising
        trajectory = JointTrajectory()
        trajectory.joint_names = ['left_shoulder_pitch', 'left_elbow']  # Example joint names

        point = JointTrajectoryPoint()
        point.positions = [0.5, 1.0]  # Example positions in radians
        point.time_from_start = Duration(sec=2, nanosec=0)

        trajectory.points = [point]
        self.joint_trajectory_publisher.publish(trajectory)

    def lower_arm(self):
        """Lower the robot's arm"""
        self.get_logger().info('Lowering arm')
        # Create joint trajectory for arm lowering
        trajectory = JointTrajectory()
        trajectory.joint_names = ['left_shoulder_pitch', 'left_elbow']

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]  # Return to neutral position
        point.time_from_start = Duration(sec=2, nanosec=0)

        trajectory.points = [point]
        self.joint_trajectory_publisher.publish(trajectory)

    def wave(self):
        """Execute waving motion"""
        self.get_logger().info('Executing wave motion')
        # Create a sequence of joint positions for waving
        trajectory = JointTrajectory()
        trajectory.joint_names = ['left_shoulder_pitch', 'left_elbow', 'left_wrist']

        points = []

        # Point 1: Raise arm
        point1 = JointTrajectoryPoint()
        point1.positions = [0.8, 0.5, 0.0]
        point1.time_from_start = Duration(sec=1, nanosec=0)
        points.append(point1)

        # Point 2: Wave up
        point2 = JointTrajectoryPoint()
        point2.positions = [1.0, 0.3, 0.5]
        point2.time_from_start = Duration(sec=1, nanosec=500000000)  # 1.5 seconds
        points.append(point2)

        # Point 3: Wave down
        point3 = JointTrajectoryPoint()
        point3.positions = [0.6, 0.7, -0.5]
        point3.time_from_start = Duration(sec=2, nanosec=0)  # 2 seconds
        points.append(point3)

        # Point 4: Return to neutral
        point4 = JointTrajectoryPoint()
        point4.positions = [0.0, 0.0, 0.0]
        point4.time_from_start = Duration(sec=2, nanosec=500000000)  # 2.5 seconds
        points.append(point4)

        trajectory.points = points
        self.joint_trajectory_publisher.publish(trajectory)

    def dance(self):
        """Execute a simple dance routine"""
        self.get_logger().info('Executing dance routine')
        # Combine walking, turning, and arm movements in a sequence
        # This would be implemented as a state machine or behavior tree

        # For now, implement a simple sequence
        import threading
        import time

        def dance_sequence():
            # Wave
            self.wave()
            time.sleep(3)

            # Turn left
            self.turn_left()
            time.sleep(2)

            # Wave again
            self.wave()
            time.sleep(3)

            # Stop
            self.stop_robot()

        # Run dance sequence in separate thread to avoid blocking
        dance_thread = threading.Thread(target=dance_sequence)
        dance_thread.start()

    def balance_robot(self):
        """Activate balance control system"""
        self.get_logger().info('Activating balance control')
        # This would interface with the robot's balance controller
        # For simulation, we'll just log the action
        self.is_balanced = True

    def stop_robot(self):
        """Stop all robot motion"""
        self.get_logger().info('Stopping robot')
        # Send zero velocity commands
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

        # Stop any ongoing trajectories
        self.cancel_all_trajectories()

    def execute_walk_gait(self, direction):
        """Execute complex walking gait pattern"""
        self.get_logger().info(f'Executing {direction} walking gait')
        # This would implement a complex gait pattern for bipedal walking
        # involving coordinated movement of legs, arms for balance, and torso

        # For simulation, we'll publish a sequence of joint trajectories
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'left_hip_pitch', 'left_knee', 'left_ankle',
            'right_hip_pitch', 'right_knee', 'right_ankle',
            'left_arm_swing', 'right_arm_swing'  # For balance
        ]

        points = []

        # Create a simple 4-point gait cycle
        if direction == 'forward':
            # Step 1: Shift weight to right leg, lift left leg
            point1 = JointTrajectoryPoint()
            point1.positions = [0.1, -0.5, 0.2, -0.1, -0.3, 0.0, 0.3, -0.3]
            point1.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5s
            points.append(point1)

            # Step 2: Move left leg forward
            point2 = JointTrajectoryPoint()
            point2.positions = [0.3, -1.0, 0.0, -0.1, -0.3, 0.0, -0.3, 0.3]
            point2.time_from_start = Duration(sec=1, nanosec=0)  # 1.0s
            points.append(point2)

            # Step 3: Shift weight to left leg, lift right leg
            point3 = JointTrajectoryPoint()
            point3.positions = [0.3, -1.0, 0.0, 0.1, -0.5, 0.2, 0.3, -0.3]
            point3.time_from_start = Duration(sec=1, nanosec=500000000)  # 1.5s
            points.append(point3)

            # Step 4: Move right leg forward
            point4 = JointTrajectoryPoint()
            point4.positions = [-0.1, -0.3, 0.0, 0.3, -1.0, 0.0, -0.3, 0.3]
            point4.time_from_start = Duration(sec=2, nanosec=0)  # 2.0s
            points.append(point4)

        trajectory.points = points
        self.joint_trajectory_publisher.publish(trajectory)

    def execute_timed_command(self, twist_cmd, duration):
        """Execute a velocity command for a specific duration"""
        import threading
        import time

        def timed_execution():
            start_time = time.time()
            while time.time() - start_time < duration:
                self.cmd_vel_publisher.publish(twist_cmd)
                time.sleep(0.1)  # 10Hz update rate

            # Stop after duration
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)

        # Run in separate thread to avoid blocking
        execution_thread = threading.Thread(target=timed_execution)
        execution_thread.start()

    def cancel_all_trajectories(self):
        """Cancel all ongoing trajectories"""
        # This would send cancel commands to trajectory controllers
        self.get_logger().info('Cancelling all trajectories')

def main(args=None):
    rclpy.init(args=args)
    node = CompleteVoiceControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Balance Control Systems

Humanoid robots require sophisticated balance control systems to maintain stability. The Zero Moment Point (ZMP) and Center of Mass (CoM) control are fundamental concepts in humanoid balance.

### ZMP-Based Balance Control

The Zero Moment Point is a point on the ground where the net moment of the ground reaction force is zero. For stable walking, the ZMP must remain within the support polygon defined by the feet.

```python
import numpy as np

class BalanceController:
    def __init__(self, robot_mass, gravity=9.81):
        self.mass = robot_mass
        self.gravity = gravity
        self.com_position = np.array([0.0, 0.0, 0.8])  # Initial CoM position
        self.com_velocity = np.array([0.0, 0.0, 0.0])
        self.com_acceleration = np.array([0.0, 0.0, 0.0])

        # Control gains
        self.kp = 100.0  # Proportional gain
        self.kd = 20.0   # Derivative gain

    def compute_zmp(self, com_pos, com_vel, com_acc):
        """Compute Zero Moment Point from CoM state"""
        [cx, cy, cz] = com_pos
        [vx, vy, vz] = com_vel
        [ax, ay, az] = com_acc

        # ZMP calculation (simplified)
        zmp_x = cx - (cz - 0.0) * ax / self.gravity  # Assume foot height is 0
        zmp_y = cy - (cz - 0.0) * ay / self.gravity

        return np.array([zmp_x, zmp_y])

    def compute_com_dynamics(self, desired_zmp, current_zmp):
        """Compute CoM acceleration to achieve desired ZMP"""
        # Simple PD controller for ZMP tracking
        zmp_error = desired_zmp - current_zmp

        # Compute desired CoM acceleration
        com_acc_x = self.kp * zmp_error[0] + self.kd * (zmp_error[0] - self.com_acceleration[0])
        com_acc_y = self.kp * zmp_error[1] + self.kd * (zmp_error[1] - self.com_acceleration[1])

        # Z component is typically controlled separately for walking height
        com_acc_z = 0.0  # Maintain constant height

        return np.array([com_acc_x, com_acc_y, com_acc_z])

    def update_balance(self, dt):
        """Update balance control for one time step"""
        # Compute current ZMP
        current_zmp = self.compute_zmp(
            self.com_position,
            self.com_velocity,
            self.com_acceleration
        )

        # For now, desired ZMP is at center of support polygon
        # In practice, this would come from walking pattern generator
        desired_zmp = np.array([0.0, 0.0])

        # Compute required CoM dynamics
        self.com_acceleration = self.compute_com_dynamics(desired_zmp, current_zmp)

        # Update CoM velocity and position
        self.com_velocity += self.com_acceleration * dt
        self.com_position += self.com_velocity * dt

        return self.com_position, current_zmp
```

## Whole-Body Control

Whole-body control coordinates all degrees of freedom to achieve multiple tasks simultaneously while respecting constraints. This is typically formulated as a constrained optimization problem.

```python
import numpy as np
from scipy.optimize import minimize

class WholeBodyController:
    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.n_joints = robot_model.get_num_joints()

    def compute_control(self, tasks, joint_limits=True, collision_avoidance=True):
        """Compute joint commands to satisfy multiple tasks"""

        # Formulate as quadratic program:
        # min ||Ax - b||^2
        # subject to: Cx <= d (inequality constraints)
        #            Ex = f (equality constraints)

        # For simplicity, implement a priority-based task hierarchy
        joint_commands = np.zeros(self.n_joints)

        # Primary task: balance control
        balance_task = self.compute_balance_task()

        # Secondary task: posture control
        posture_task = self.compute_posture_task()

        # Tertiary task: task-specific motion
        task_specific = self.compute_task_specific_motion(tasks)

        # Combine tasks using weighted optimization
        weights = [1.0, 0.1, 0.01]  # Priority weights

        combined_task = (weights[0] * balance_task +
                        weights[1] * posture_task +
                        weights[2] * task_specific)

        return combined_task

    def compute_balance_task(self):
        """Compute joint commands for balance maintenance"""
        # Use inverted pendulum model for balance
        com_error = self.get_com_error()
        joint_torques = self.map_com_to_joints(com_error)
        return joint_torques

    def compute_posture_task(self):
        """Compute joint commands for desired posture"""
        current_joints = self.get_current_joint_positions()
        desired_posture = self.get_default_posture()
        posture_error = desired_posture - current_joints
        return posture_error * 0.1  # Small gain for posture

    def compute_task_specific_motion(self, tasks):
        """Compute joint commands for specific tasks"""
        task_commands = np.zeros(self.n_joints)

        for task in tasks:
            if task == 'walk':
                task_commands += self.compute_walking_motion()
            elif task == 'manipulate':
                task_commands += self.compute_manipulation_motion()
            elif task == 'turn':
                task_commands += self.compute_turning_motion()

        return task_commands

    def get_com_error(self):
        """Get current center of mass error"""
        # Implementation would interface with robot state
        return np.zeros(3)

    def map_com_to_joints(self, com_error):
        """Map CoM error to joint space"""
        # Use Jacobian transpose or pseudoinverse
        jacobian = self.robot_model.get_com_jacobian()
        joint_commands = np.dot(jacobian.T, com_error)
        return joint_commands

    def get_current_joint_positions(self):
        """Get current joint positions from robot state"""
        # Implementation would interface with robot
        return np.zeros(self.n_joints)

    def get_default_posture(self):
        """Get default standing posture joint angles"""
        # Default standing posture
        default_pos = np.zeros(self.n_joints)
        # Set typical humanoid standing posture
        # This would be specific to the robot model
        return default_pos

    def compute_walking_motion(self):
        """Compute joint commands for walking"""
        # Generate walking pattern
        return np.zeros(self.n_joints)

    def compute_manipulation_motion(self):
        """Compute joint commands for manipulation"""
        # Generate manipulation pattern
        return np.zeros(self.n_joints)

    def compute_turning_motion(self):
        """Compute joint commands for turning"""
        # Generate turning pattern
        return np.zeros(self.n_joints)
```

## Machine Learning for Humanoid Behavior

Machine learning techniques can be applied to humanoid robots to improve adaptability and learning from experience.

### Reinforcement Learning for Gait Optimization

```python
import torch
import torch.nn as nn
import numpy as np

class GaitPolicyNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(GaitPolicyNetwork, self).__init__()

        self.network = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim),
            nn.Tanh()  # Actions are normalized to [-1, 1]
        )

    def forward(self, state):
        return self.network(state)

class GaitLearningAgent:
    def __init__(self, state_dim, action_dim, learning_rate=3e-4):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        self.policy_network = GaitPolicyNetwork(state_dim, action_dim).to(self.device)
        self.optimizer = torch.optim.Adam(self.policy_network.parameters(), lr=learning_rate)

        self.state_dim = state_dim
        self.action_dim = action_dim

    def select_action(self, state):
        """Select action based on current state"""
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        action = self.policy_network(state_tensor)
        return action.cpu().data.numpy().flatten()

    def update_policy(self, states, actions, rewards, next_states, dones):
        """Update policy based on experience"""
        states = torch.FloatTensor(states).to(self.device)
        actions = torch.FloatTensor(actions).to(self.device)
        rewards = torch.FloatTensor(rewards).to(self.device)
        next_states = torch.FloatTensor(next_states).to(self.device)
        dones = torch.FloatTensor(dones).to(self.device)

        # Compute policy loss (simplified for this example)
        predicted_actions = self.policy_network(states)

        # In practice, this would involve more complex RL algorithms
        # like PPO, SAC, or A3C for continuous control
        loss = nn.MSELoss()(predicted_actions, actions)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        return loss.item()
```

## Integration Example: Voice-Controlled Humanoid Robot

Let's put it all together with a complete example that integrates voice recognition, humanoid control, and safety systems:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, PoseStamped
from trajectory_msgs.msg import JointTrajectory
from builtin_interfaces.msg import Duration
import numpy as np
import threading
import time

class IntegratedHumanoidController(Node):
    def __init__(self):
        super().__init__('integrated_humanoid_controller')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        # Subscribers
        self.voice_sub = self.create_subscription(String, '/voice_commands', self.voice_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Robot state
        self.current_joints = {}
        self.imu_data = None
        self.safety_enabled = True

        # Balance controller
        self.balance_controller = BalanceController(robot_mass=30.0)

        # Whole body controller
        self.whole_body_controller = WholeBodyController(robot_model=None)  # Placeholder

        self.get_logger().info('Integrated Humanoid Controller initialized')

    def voice_callback(self, msg):
        """Handle voice commands with safety checks"""
        command = msg.data.lower().strip()

        if self.safety_check():
            self.process_command(command)
        else:
            self.get_logger().warn('Safety check failed - command rejected')
            self.emergency_stop()

    def safety_check(self):
        """Perform safety checks before executing commands"""
        if not self.safety_enabled:
            return False

        # Check IMU data for dangerous orientations
        if self.imu_data:
            roll, pitch, yaw = self.get_orientation_from_imu()
            if abs(roll) > 1.0 or abs(pitch) > 1.0:  # Too tilted
                return False

        # Check joint limits (simplified)
        for joint_name, position in self.current_joints.items():
            if abs(position) > 3.0:  # Example limit
                return False

        return True

    def process_command(self, command):
        """Process and execute robot command"""
        if 'dance' in command:
            self.execute_dance_routine()
        elif 'walk' in command:
            self.execute_walking(command)
        elif 'balance' in command:
            self.activate_balance_control()
        elif 'stop' in command:
            self.emergency_stop()
        else:
            self.get_logger().info(f'Command not recognized: {command}')

    def execute_dance_routine(self):
        """Execute a coordinated dance routine"""
        self.get_logger().info('Executing dance routine with safety monitoring')

        # Disable safety temporarily for choreographed movements
        original_safety = self.safety_enabled
        self.safety_enabled = False

        try:
            # Sequence of movements
            self.wave_arms()
            time.sleep(1.0)
            self.turn_in_place()
            time.sleep(1.0)
            self.raise_arms()
            time.sleep(1.0)
            self.lower_arms()
        finally:
            # Re-enable safety
            self.safety_enabled = original_safety

    def execute_walking(self, command):
        """Execute walking with balance control"""
        self.get_logger().info('Executing walking with active balance control')

        # Start balance control in background
        balance_thread = threading.Thread(target=self.maintain_balance)
        balance_thread.daemon = True
        balance_thread.start()

        # Execute walking command
        if 'forward' in command:
            self.walk_forward()
        elif 'backward' in command:
            self.walk_backward()

    def maintain_balance(self):
        """Maintain balance during locomotion"""
        dt = 0.01  # 100Hz control loop
        while rclpy.ok():
            if self.safety_enabled:
                # Update balance controller
                com_pos, zmp = self.balance_controller.update_balance(dt)

                # Adjust joint commands based on balance needs
                balance_correction = self.compute_balance_correction(zmp)
                self.apply_balance_correction(balance_correction)

            time.sleep(dt)

    def compute_balance_correction(self, zmp):
        """Compute balance correction based on ZMP error"""
        # Simplified balance correction
        desired_zmp = np.array([0.0, 0.0])  # Center of support
        zmp_error = desired_zmp - zmp

        # Map ZMP error to corrective joint torques
        correction = zmp_error * 50.0  # Gain
        return correction

    def apply_balance_correction(self, correction):
        """Apply balance correction to robot joints"""
        # Create trajectory message with balance corrections
        trajectory = JointTrajectory()
        trajectory.joint_names = ['left_ankle_roll', 'right_ankle_roll',
                                 'left_ankle_pitch', 'right_ankle_pitch']

        point = JointTrajectoryPoint()
        point.positions = [correction[0], correction[0], correction[1], correction[1]]
        point.time_from_start = Duration(sec=0, nanosec=10000000)  # 10ms
        trajectory.points = [point]

        self.joint_traj_pub.publish(trajectory)

    def emergency_stop(self):
        """Emergency stop all robot motion"""
        self.get_logger().info('Activating emergency stop')

        # Stop all motion
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        # Cancel all trajectories
        self.cancel_all_trajectories()

        # Activate safety posture
        self.go_to_safe_posture()

    def joint_state_callback(self, msg):
        """Update joint state"""
        for i, name in enumerate(msg.name):
            self.current_joints[name] = msg.position[i]

    def imu_callback(self, msg):
        """Update IMU data"""
        self.imu_data = msg

    def get_orientation_from_imu(self):
        """Extract roll, pitch, yaw from IMU quaternion"""
        if not self.imu_data:
            return 0.0, 0.0, 0.0

        # Convert quaternion to Euler angles (simplified)
        q = self.imu_data.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    controller = IntegratedHumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Controller interrupted by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-On Exercise: Implement a Simple Humanoid Dance Routine

Students will implement a simple dance routine for a humanoid robot using the concepts learned in this module.

### Exercise Objectives:
1. Create a sequence of coordinated movements
2. Implement safety checks during complex motions
3. Integrate balance control with choreographed movements
4. Test the routine in simulation

### Implementation Steps:

1. Create a dance controller node
2. Define a sequence of joint positions for the dance
3. Implement timing and synchronization
4. Add safety checks between movements
5. Test in Isaac Sim or Gazebo

## Summary

This week covered the fundamental concepts of embodied AI and humanoid robot control. Students learned about:
- The principles of embodied AI and its differences from traditional AI
- Humanoid robot control systems including balance and locomotion
- Voice-activated control integration
- Whole-body control and optimization
- Machine learning applications in humanoid robotics

The integration of these concepts allows for the creation of sophisticated, human-like robots that can interact naturally with their environment and respond to human commands through voice interfaces.