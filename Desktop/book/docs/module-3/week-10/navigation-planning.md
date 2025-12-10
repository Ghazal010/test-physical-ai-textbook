---
title: Week 10 - Navigation and Planning
description: Path planning and navigation using Nav2 and sim-to-real transfer techniques
sidebar_position: 1
---

# Week 10: Navigation and Planning

## Learning Objectives
- Use Nav2 for path planning and autonomous navigation
- Understand bipedal humanoid movement challenges and navigation solutions
- Implement obstacle avoidance systems for safe navigation
- Generate synthetic data for training navigation systems
- Apply sim-to-real transfer techniques for real-world deployment
- Build complete autonomous navigation pipelines

## Prerequisites Check
- Understanding of ROS 2 navigation concepts
- Experience with sensor integration and perception
- Knowledge of Isaac Sim and Isaac ROS integration
- Basic understanding of path planning algorithms

## Theoretical Concepts: Navigation and Planning Fundamentals

### Introduction to Navigation in Robotics

Navigation in robotics encompasses the ability of a robot to move from one location to another while avoiding obstacles and adapting to its environment. Modern robotic navigation systems typically include:

- **Localization**: Determining the robot's position in the environment
- **Mapping**: Creating or using a representation of the environment
- **Path Planning**: Computing a safe route from start to goal
- **Motion Planning**: Generating trajectories that respect robot dynamics
- **Control**: Executing motions to follow planned paths
- **Recovery**: Handling failures and replanning when needed

### Nav2 Architecture and Components

Nav2 (Navigation 2) is the next-generation navigation framework for ROS 2, designed to be more modular, flexible, and robust than its predecessor. Key components include:

**Planners**:
- **Global Planner**: Computes long-term path from start to goal
- **Local Planner**: Generates short-term trajectories while avoiding obstacles
- **Controller**: Tracks the trajectory with robot-specific dynamics

**Behaviors**:
- **Recovery Behaviors**: Actions taken when navigation fails
- **Safety Behaviors**: Emergency stopping and collision prevention
- **Calibration Behaviors**: Sensor and system calibration procedures

**Coordination**:
- **Action Server**: Coordinates the navigation process
- **Life Cycle Manager**: Manages state transitions of components
- **Behavior Tree**: Orchestrates complex navigation behaviors

### Path Planning Algorithms

**Global Path Planners**:
- **A* (A-star)**: Heuristic search algorithm optimal for grid-based maps
- **Dijkstra**: Unweighted shortest path algorithm
- **RRT (Rapidly-exploring Random Trees)**: Sampling-based for complex spaces
- **Theta***: Any-angle path planning for smoother paths

**Local Path Planners**:
- **DWA (Dynamic Window Approach)**: Considers robot dynamics
- **TEB (Timed Elastic Band)**: Optimizes trajectories over time
- **MPC (Model Predictive Control)**: Predictive control approach

### Bipedal Navigation Challenges

Humanoid robots present unique navigation challenges:

- **Balance**: Maintaining stability during movement
- **Foot placement**: Careful planning of step locations
- **Center of Mass**: Managing COM during gait transitions
- **Dynamic walking**: Handling the transition between static and dynamic states
- **Terrain adaptation**: Adjusting gait for different surfaces

### Sim-to-Real Transfer Techniques

Successfully transferring navigation capabilities from simulation to reality requires:

- **Domain Randomization**: Varying simulation parameters to improve robustness
- **System Identification**: Modeling real-world discrepancies
- **Adaptive Control**: Adjusting parameters based on real-world performance
- **Validation**: Extensive testing in both simulation and reality

## Step-by-Step Tutorials: Navigation Implementation

### Tutorial 1: Setting up Nav2 for Differential Drive Robot

Let's start by configuring Nav2 for a differential drive robot:

```yaml
# File: robot_control_package/config/nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    likelihood_max_dist: 2.0
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.5
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    interrupt_on_battery_percentage: 0.0
    battery_low_percentage: 0.10
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_consistent_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # DWB parameters
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.0
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      use_interpolation: false
      regulate_frequency: 20
      max_allowed_time_to_collision_up_to_carrot: 1.0
      carrot_planner:
        plugin: "nav2_regulated_pure_pursuit_controller::CarrotPlan"
        sampling_delta: 0.025
        min_obstacle_clearance: 0.51
        use_navfn: false

controller_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        map_subscribe_transient_local: True
      always_send_full_costmap: True
  local_costmap_client:
    ros__parameters:
      use_sim_time: True
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
  global_costmap_client:
    ros__parameters:
      use_sim_time: True
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "turtlebot3_world.yaml"

map_saver:
  ros__parameters:
    use_sim_time: True
    save_map_timeout: 5000
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_behaviors::BackUp"
      backup_dist: 0.15
      backup_speed: 0.025
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
      drive_on_heading_dist: 1.0
      drive_on_heading_angle_tolerance: 0.785
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
      min_vel_trans: 0.0
      max_vel_trans: 0.5
      min_vel_rot: 0.0
      max_vel_rot: 1.0
      trans_stopped_velocity: 0.5
      rot_stopped_velocity: 0.5
      death_zone: 0.0
    wait:
      plugin: "nav2_behaviors::Wait"
      wait_duration: 1.0

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
```

### Tutorial 2: Creating a Navigation Node

Let's create a navigation node that uses Nav2:

```python
# File: robot_control_package/robot_control_package/navigation_node.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import time
import threading

class NavigationNode(Node):
    """
    Navigation node using Nav2 for autonomous navigation
    """

    def __init__(self):
        super().__init__('navigation_node')

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

        # Navigation action client
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Subscriptions
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            reliable_qos
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            sensor_qos
        )

        # Publishers
        self.navigation_status_publisher = self.create_publisher(
            String, 'navigation_status', reliable_qos
        )
        self.navigation_goal_publisher = self.create_publisher(
            PoseStamped, 'navigation_goal', reliable_qos
        )
        self.navigation_feedback_publisher = self.create_publisher(
            String, 'navigation_feedback', reliable_qos
        )

        # TF2 components
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal state
        self.current_pose = None
        self.scan_data = None
        self.navigation_active = False
        self.navigation_goal = None
        self.navigation_thread = None

        # Navigation parameters
        self.min_distance_to_goal = 0.5  # meters
        self.obstacle_distance_threshold = 0.8  # meters
        self.safety_margin = 0.3  # meters

        # Timers
        self.status_timer = self.create_timer(1.0, self.publish_navigation_status)

        self.get_logger().info('Navigation node initialized')

    def odom_callback(self, msg):
        """
        Handle odometry data to track current position
        """
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """
        Handle laser scan data for obstacle detection
        """
        self.scan_data = msg

    def navigate_to_pose(self, x, y, theta):
        """
        Navigate to a specific pose using Nav2
        """
        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        quat = self.yaw_to_quaternion(theta)
        goal_msg.pose.pose.orientation = quat

        # Send goal
        self.navigation_active = True
        self.navigation_goal = (x, y, theta)

        self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        ).add_done_callback(self.navigation_result_callback)

        # Publish goal for visualization
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header = goal_msg.pose.header
        goal_pose_msg.pose = goal_msg.pose.pose
        self.navigation_goal_publisher.publish(goal_pose_msg)

        self.get_logger().info(f'Navigation goal sent: ({x:.2f}, {y:.2f}, {theta:.2f})')
        return True

    def navigation_feedback_callback(self, feedback_msg):
        """
        Handle navigation feedback
        """
        feedback_str = f"Navigation feedback: {feedback_msg.feedback.current_pose}"
        self.get_logger().debug(feedback_str)

        # Publish feedback
        feedback_msg_pub = String()
        feedback_msg_pub.data = feedback_str
        self.navigation_feedback_publisher.publish(feedback_msg_pub)

    def navigation_result_callback(self, future):
        """
        Handle navigation result
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Navigation goal rejected')
                self.navigation_active = False
                return

            self.get_logger().info('Navigation goal accepted')

            # Get result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.navigation_final_result_callback)

        except Exception as e:
            self.get_logger().error(f'Navigation goal error: {e}')
            self.navigation_active = False

    def navigation_final_result_callback(self, future):
        """
        Handle final navigation result
        """
        try:
            result = future.result().result
            self.get_logger().info(f'Navigation completed with result: {result}')
        except Exception as e:
            self.get_logger().error(f'Navigation result error: {e}')
        finally:
            self.navigation_active = False

    def yaw_to_quaternion(self, yaw):
        """
        Convert yaw angle to quaternion
        """
        from math import sin, cos

        qx = 0.0
        qy = 0.0
        qz = sin(yaw / 2.0)
        qw = cos(yaw / 2.0)

        quat = Quaternion()
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw

        return quat

    def check_obstacles(self):
        """
        Check for obstacles in the path using LIDAR data
        """
        if not self.scan_data:
            return False

        # Check for obstacles in front of the robot
        ranges = self.scan_data.ranges
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment

        # Check central front sector (±30 degrees)
        front_start = int((len(ranges) // 2) - (30 / 180 * math.pi) / angle_increment)
        front_end = int((len(ranges) // 2) + (30 / 180 * math.pi) / angle_increment)

        front_ranges = ranges[max(0, front_start):min(len(ranges), front_end)]

        # Find minimum distance in front
        if front_ranges:
            min_front_dist = min([r for r in front_ranges if 0 < r < float('inf')], default=float('inf'))

            if min_front_dist < self.obstacle_distance_threshold:
                self.get_logger().warn(f'Obstacle detected: {min_front_dist:.2f}m')
                return True

        return False

    def get_current_position(self):
        """
        Get current robot position
        """
        if self.current_pose:
            return (
                self.current_pose.position.x,
                self.current_pose.position.y,
                self.quaternion_to_yaw(self.current_pose.orientation)
            )
        return None

    def quaternion_to_yaw(self, orientation):
        """
        Convert quaternion to yaw angle
        """
        from math import atan2, asin

        # Simplified conversion for z-axis rotation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = atan2(siny_cosp, cosy_cosp)

        return yaw

    def publish_navigation_status(self):
        """
        Publish navigation status
        """
        status_msg = String()

        if self.navigation_active:
            status = "Navigating"
            if self.navigation_goal:
                goal_str = f"Goal: ({self.navigation_goal[0]:.2f}, {self.navigation_goal[1]:.2f})"
                status += f" - {goal_str}"
        else:
            status = "Idle"

        # Add obstacle information
        has_obstacles = self.check_obstacles()
        obstacle_status = "Obstacles: Yes" if has_obstacles else "Obstacles: No"

        # Add position information
        pos = self.get_current_position()
        if pos:
            pos_str = f"Pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
        else:
            pos_str = "Pos: Unknown"

        status_msg.data = f"{status} | {obstacle_status} | {pos_str}"
        self.navigation_status_publisher.publish(status_msg)

    def execute_navigation_sequence(self, waypoints):
        """
        Execute a sequence of navigation waypoints
        """
        self.get_logger().info(f'Executing navigation sequence with {len(waypoints)} waypoints')

        for i, waypoint in enumerate(waypoints):
            self.get_logger().info(f'Navigating to waypoint {i+1}/{len(waypoints)}: {waypoint}')

            # Navigate to waypoint
            success = self.navigate_to_pose(waypoint[0], waypoint[1], waypoint[2])

            if not success:
                self.get_logger().error(f'Failed to navigate to waypoint {i+1}')
                return False

            # Wait for navigation to complete
            while self.navigation_active:
                time.sleep(0.1)

            # Check if navigation was successful
            # In a real implementation, you'd check the result more thoroughly

        self.get_logger().info('Navigation sequence completed')
        return True

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    # Example: Navigate to a specific location after startup
    import threading
    def delayed_navigation():
        time.sleep(5)  # Wait for systems to initialize
        node.navigate_to_pose(2.0, 2.0, 0.0)  # Navigate to (2,2) with 0 heading

    nav_thread = threading.Thread(target=delayed_navigation, daemon=True)
    nav_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down navigation node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Tutorial 3: Advanced Path Planning with Local and Global Planners

Create a node that interfaces with both local and global planners:

```python
# File: robot_control_package/robot_control_package/advanced_path_planning.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
import math
from scipy.spatial import KDTree
import threading

class AdvancedPathPlanningNode(Node):
    """
    Advanced path planning node with local and global planning capabilities
    """

    def __init__(self):
        super().__init__('advanced_path_planning_node')

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

        # Action client for path planning
        self.path_planner_client = ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose'
        )

        # Services for costmap access
        self.global_costmap_client = self.create_client(
            GetCostmap, 'global_costmap/costmap/get_costmap'
        )
        self.local_costmap_client = self.create_client(
            GetCostmap, 'local_costmap/costmap/get_costmap'
        )

        # Subscriptions
        self.initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.initial_pose_callback,
            reliable_qos
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            sensor_qos
        )

        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,
            'global_costmap/costmap',
            self.costmap_callback,
            sensor_qos
        )

        # Publishers
        self.global_plan_publisher = self.create_publisher(Path, 'global_plan', reliable_qos)
        self.local_plan_publisher = self.create_publisher(Path, 'local_plan', reliable_qos)
        self.path_quality_publisher = self.create_publisher(Float32, 'path_quality', reliable_qos)
        self.planning_status_publisher = self.create_publisher(String, 'planning_status', reliable_qos)

        # TF2 components
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal state
        self.initial_pose = None
        self.scan_data = None
        self.costmap_data = None
        self.global_plan = None
        self.planning_active = False

        # Path planning parameters
        self.planning_frequency = 1.0  # Hz
        self.lookahead_distance = 1.0  # meters
        self.path_smoothing_factor = 0.3
        self.obstacle_clearance = 0.5  # meters

        # Timers
        self.planning_timer = self.create_timer(
            1.0 / self.planning_frequency, self.periodic_path_planning
        )

        self.get_logger().info('Advanced Path Planning node initialized')

    def initial_pose_callback(self, msg):
        """
        Handle initial pose for localization
        """
        self.initial_pose = msg.pose.pose

    def scan_callback(self, msg):
        """
        Handle laser scan data for local planning
        """
        self.scan_data = msg

    def costmap_callback(self, msg):
        """
        Handle costmap data for global planning
        """
        self.costmap_data = msg

    def request_path_to_pose(self, start_pose, goal_pose):
        """
        Request path from start to goal using Nav2 path planner
        """
        # Wait for action server
        if not self.path_planner_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Path planner action server not available')
            return None

        # Create goal
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = PoseStamped()
        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.header.stamp = self.get_clock().now().to_msg()
        goal_msg.start.pose = start_pose

        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose = goal_pose

        # Send goal
        self.planning_active = True

        future = self.path_planner_client.send_goal_async(goal_msg)
        future.add_done_callback(self.path_planning_result_callback)

        self.get_logger().info('Path planning request sent')
        return future

    def path_planning_result_callback(self, future):
        """
        Handle path planning result
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Path planning goal rejected')
                self.planning_active = False
                return

            self.get_logger().info('Path planning goal accepted')

            # Get result
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.path_planning_final_result_callback)

        except Exception as e:
            self.get_logger().error(f'Path planning goal error: {e}')
            self.planning_active = False

    def path_planning_final_result_callback(self, future):
        """
        Handle final path planning result
        """
        try:
            result = future.result().result
            if result.path.poses:
                self.global_plan = result.path
                self.get_logger().info(f'Path computed with {len(result.path.poses)} waypoints')

                # Publish global plan
                self.global_plan_publisher.publish(self.global_plan)

                # Evaluate path quality
                path_quality = self.evaluate_path_quality(self.global_plan)
                quality_msg = Float32()
                quality_msg.data = path_quality
                self.path_quality_publisher.publish(quality_msg)

                self.get_logger().info(f'Path quality: {path_quality:.2f}')

            else:
                self.get_logger().warn('Computed path is empty')
                self.global_plan = None

        except Exception as e:
            self.get_logger().error(f'Path planning result error: {e}')
        finally:
            self.planning_active = False

    def evaluate_path_quality(self, path):
        """
        Evaluate the quality of a computed path
        """
        if not path or not path.poses:
            return 0.0

        # Calculate path length
        total_length = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i-1].pose.position
            p2 = path.poses[i].pose.position
            dist = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            total_length += dist

        # Calculate smoothness (deviation from straight line)
        if len(path.poses) >= 2:
            start = path.poses[0].pose.position
            end = path.poses[-1].pose.position
            straight_line_dist = math.sqrt((end.x - start.x)**2 + (end.y - start.y)**2)

            if straight_line_dist > 0:
                efficiency_ratio = straight_line_dist / total_length if total_length > 0 else 1.0
            else:
                efficiency_ratio = 1.0
        else:
            efficiency_ratio = 1.0

        # Calculate safety based on proximity to obstacles
        safety_score = self.evaluate_path_safety(path)

        # Weighted combination of factors
        quality = 0.4 * efficiency_ratio + 0.6 * safety_score

        return max(0.0, min(1.0, quality))

    def evaluate_path_safety(self, path):
        """
        Evaluate path safety based on proximity to obstacles
        """
        if not self.costmap_data:
            return 0.8  # Assume medium safety if no costmap available

        # Convert path to costmap coordinates and check for high-cost areas
        safe_segments = 0
        total_segments = max(1, len(path.poses) - 1)

        for i in range(len(path.poses) - 1):
            pose = path.poses[i].pose.position

            # Convert world coordinates to costmap cell coordinates
            try:
                map_x = int((pose.x - self.costmap_data.info.origin.position.x) / self.costmap_data.info.resolution)
                map_y = int((pose.y - self.costmap_data.info.origin.position.y) / self.costmap_data.info.resolution)

                # Check if within costmap bounds
                if (0 <= map_x < self.costmap_data.info.width and
                    0 <= map_y < self.costmap_data.info.height):

                    # Get cost from costmap
                    map_idx = map_y * self.costmap_data.info.width + map_x
                    cost = self.costmap_data.data[map_idx]

                    # Check if segment is safe (cost below threshold)
                    if cost < 50:  # Below lethal obstacle threshold
                        safe_segments += 1
            except Exception:
                continue

        safety_ratio = safe_segments / total_segments
        return safety_ratio

    def periodic_path_planning(self):
        """
        Periodically re-plan path if needed
        """
        if self.planning_active:
            return  # Don't plan while planning is active

        # Only plan if we have initial pose and a goal
        if not self.initial_pose:
            return

        # For this example, we'll plan to a fixed goal
        # In a real system, this would come from navigation goals
        goal_pose = self.create_default_goal()
        self.request_path_to_pose(self.initial_pose, goal_pose)

    def create_default_goal(self):
        """
        Create a default goal pose for demonstration
        """
        from geometry_msgs.msg import Pose

        goal = Pose()
        goal.position.x = 5.0  # Fixed goal at (5,0)
        goal.position.y = 0.0
        goal.position.z = 0.0

        # Face along positive x-axis
        goal.orientation.w = 1.0
        goal.orientation.x = 0.0
        goal.orientation.y = 0.0
        goal.orientation.z = 0.0

        return goal

    def generate_local_path(self, current_pose, global_plan):
        """
        Generate local path based on global plan and local obstacles
        """
        if not global_plan or not global_plan.poses:
            return None

        # Find closest point on global plan to current position
        current_pos = current_pose.position
        closest_idx = 0
        min_dist = float('inf')

        for i, pose in enumerate(global_plan.poses):
            p = pose.pose.position
            dist = math.sqrt((p.x - current_pos.x)**2 + (p.y - current_pos.y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Generate local path from current position to a point ahead on global plan
        local_path = Path()
        local_path.header.stamp = self.get_clock().now().to_msg()
        local_path.header.frame_id = 'map'

        # Add current position
        start_pose = PoseStamped()
        start_pose.header = local_path.header
        start_pose.pose = current_pose
        local_path.poses.append(start_pose)

        # Add points from global plan up to lookahead distance
        start_idx = closest_idx
        current_dist = 0.0

        for i in range(start_idx, min(len(global_plan.poses), start_idx + 20)):
            if current_dist >= self.lookahead_distance:
                break

            pose_stamped = PoseStamped()
            pose_stamped.header = local_path.header
            pose_stamped.pose = global_plan.poses[i].pose

            if i > start_idx:
                prev_pos = local_path.poses[-1].pose.position
                curr_pos = pose_stamped.pose.position
                segment_dist = math.sqrt((curr_pos.x - prev_pos.x)**2 + (curr_pos.y - prev_pos.y)**2)
                current_dist += segment_dist

            local_path.poses.append(pose_stamped)

        return local_path

    def smooth_path(self, path):
        """
        Smooth the path using a simple smoothing algorithm
        """
        if not path or len(path.poses) < 3:
            return path

        smoothed_path = Path()
        smoothed_path.header = path.header

        # Copy first and last points
        smoothed_path.poses.append(path.poses[0])

        # Smooth intermediate points
        for i in range(1, len(path.poses) - 1):
            prev_pt = path.poses[i-1].pose.position
            curr_pt = path.poses[i].pose.position
            next_pt = path.poses[i+1].pose.position

            # Simple smoothing: weighted average of neighboring points
            smoothed_pose = PoseStamped()
            smoothed_pose.header = path.header

            alpha = self.path_smoothing_factor
            smoothed_pose.pose.position.x = (1 - alpha) * curr_pt.x + alpha * 0.5 * (prev_pt.x + next_pt.x)
            smoothed_pose.pose.position.y = (1 - alpha) * curr_pt.y + alpha * 0.5 * (prev_pt.y + next_pt.y)
            smoothed_pose.pose.position.z = curr_pt.z

            # Keep original orientation for now
            smoothed_pose.pose.orientation = path.poses[i].pose.orientation

            smoothed_path.poses.append(smoothed_pose)

        # Copy last point
        if len(path.poses) > 1:
            smoothed_path.poses.append(path.poses[-1])

        return smoothed_path

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedPathPlanningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down advanced path planning node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Tutorial 4: Obstacle Avoidance and Recovery Behaviors

Implement obstacle avoidance and recovery behaviors:

```python
# File: robot_control_package/robot_control_package/obstacle_avoidance.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Bool, Float32
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np
from collections import deque
import threading

class ObstacleAvoidanceNode(Node):
    """
    Obstacle avoidance and recovery behaviors for navigation
    """

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

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

        # Subscriptions
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            reliable_qos
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            sensor_qos
        )

        self.path_subscription = self.create_subscription(
            Path,
            'global_plan',
            self.path_callback,
            reliable_qos
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', reliable_qos)
        self.avoidance_status_publisher = self.create_publisher(String, 'avoidance_status', reliable_qos)
        self.recovery_status_publisher = self.create_publisher(String, 'recovery_status', reliable_qos)
        self.avoidance_path_publisher = self.create_publisher(Path, 'avoidance_path', reliable_qos)

        # TF2 components
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal state
        self.current_pose = None
        self.scan_data = None
        self.global_path = None
        self.current_velocity = Twist()
        self.avoidance_active = False
        self.recovery_active = False
        self.emergency_stop = False

        # Obstacle avoidance parameters
        self.min_obstacle_distance = 0.6  # meters
        self.avoidance_gain = 1.0
        self.forward_speed = 0.3  # m/s
        self.rotation_speed = 0.5  # rad/s
        self.emergency_stop_distance = 0.3  # meters

        # Recovery parameters
        self.recovery_attempts = 0
        self.max_recovery_attempts = 3
        self.recovery_timeout = 10.0  # seconds

        # Data buffers
        self.velocity_buffer = deque(maxlen=10)
        self.obstacle_history = deque(maxlen=5)

        # Timers
        self.avoidance_timer = self.create_timer(0.1, self.avoidance_control_loop)  # 10 Hz

        self.get_logger().info('Obstacle Avoidance node initialized')

    def odom_callback(self, msg):
        """
        Handle odometry data
        """
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

        # Add to velocity buffer for analysis
        self.velocity_buffer.append({
            'linear': msg.twist.twist.linear.x,
            'angular': msg.twist.twist.angular.z,
            'timestamp': self.get_clock().now()
        })

    def scan_callback(self, msg):
        """
        Handle laser scan data for obstacle detection
        """
        self.scan_data = msg

    def path_callback(self, msg):
        """
        Handle global path for reference
        """
        self.global_path = msg

    def avoidance_control_loop(self):
        """
        Main avoidance control loop
        """
        if not self.scan_data:
            return

        # Check for obstacles
        obstacle_info = self.detect_obstacles()

        if self.emergency_stop or obstacle_info['emergency']:
            self.execute_emergency_stop()
        elif obstacle_info['obstacle_detected']:
            if not self.avoidance_active:
                self.get_logger().info('Obstacle detected, activating avoidance')

            self.avoidance_active = True
            avoidance_cmd = self.compute_avoidance_command(obstacle_info)
            self.cmd_vel_publisher.publish(avoidance_cmd)
        else:
            # No obstacles, deactivate avoidance
            if self.avoidance_active:
                self.get_logger().info('No obstacles detected, deactivating avoidance')

            self.avoidance_active = False

            # Publish zero velocity to stop any ongoing avoidance
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)

        # Publish status
        status_msg = String()
        if self.emergency_stop:
            status_msg.data = "EMERGENCY STOP - Obstacle too close"
        elif self.avoidance_active:
            status_msg.data = f"Avoiding obstacle - Left: {obstacle_info['left_clear']:.2f}m, Right: {obstacle_info['right_clear']:.2f}m"
        else:
            status_msg.data = "Clear path, no avoidance active"

        self.avoidance_status_publisher.publish(status_msg)

    def detect_obstacles(self):
        """
        Detect obstacles from laser scan data
        """
        if not self.scan_data:
            return {
                'obstacle_detected': False,
                'emergency': False,
                'closest_distance': float('inf'),
                'left_clear': float('inf'),
                'right_clear': float('inf')
            }

        ranges = self.scan_data.ranges
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment

        # Calculate indices for different sectors
        total_angles = len(ranges)
        front_center = total_angles // 2

        # Front sector (±45 degrees)
        front_start = max(0, front_center - int(45 * math.pi / 180 / angle_increment))
        front_end = min(total_angles, front_center + int(45 * math.pi / 180 / angle_increment))

        # Left and right sectors
        left_start = max(0, front_center - int(90 * math.pi / 180 / angle_increment))
        left_end = front_center
        right_start = front_center
        right_end = min(total_angles, front_center + int(90 * math.pi / 180 / angle_increment))

        # Find closest obstacles in each sector
        front_ranges = [r for r in ranges[front_start:front_end] if 0 < r < float('inf')]
        left_ranges = [r for r in ranges[left_start:left_end] if 0 < r < float('inf')]
        right_ranges = [r for r in ranges[right_start:right_end] if 0 < r < float('inf')]

        closest_front = min(front_ranges) if front_ranges else float('inf')
        closest_left = min(left_ranges) if left_ranges else float('inf')
        closest_right = min(right_ranges) if right_ranges else float('inf')

        # Check for emergency condition
        emergency = closest_front < self.emergency_stop_distance
        obstacle_detected = closest_front < self.min_obstacle_distance

        return {
            'obstacle_detected': obstacle_detected,
            'emergency': emergency,
            'closest_distance': closest_front,
            'left_clear': closest_left,
            'right_clear': closest_right
        }

    def compute_avoidance_command(self, obstacle_info):
        """
        Compute avoidance command based on obstacle information
        """
        cmd = Twist()

        if obstacle_info['closest_distance'] < self.emergency_stop_distance:
            # Emergency stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.emergency_stop = True
            return cmd

        # Normal avoidance behavior
        if obstacle_info['closest_distance'] < self.min_obstacle_distance:
            # Determine turn direction based on clearer side
            if obstacle_info['left_clear'] > obstacle_info['right_clear']:
                # Turn left (positive angular velocity)
                cmd.angular.z = min(self.rotation_speed,
                                   self.avoidance_gain * (obstacle_info['left_clear'] - obstacle_info['right_clear']))
            else:
                # Turn right (negative angular velocity)
                cmd.angular.z = max(-self.rotation_speed,
                                   -self.avoidance_gain * (obstacle_info['right_clear'] - obstacle_info['left_clear']))

            # Reduce forward speed when turning
            cmd.linear.x = self.forward_speed * 0.5
        else:
            # Clear path, move forward
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        # Limit velocities
        cmd.linear.x = max(-self.forward_speed, min(self.forward_speed, cmd.linear.x))
        cmd.angular.z = max(-self.rotation_speed, min(self.rotation_speed, cmd.angular.z))

        return cmd

    def execute_emergency_stop(self):
        """
        Execute emergency stop procedure
        """
        if not self.emergency_stop:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            self.emergency_stop = True

        # Publish zero velocity
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)

        # Check if it's safe to resume
        if self.scan_data:
            ranges = [r for r in self.scan_data.ranges if 0 < r < float('inf')]
            if ranges:
                min_dist = min(ranges)
                if min_dist > self.min_obstacle_distance * 1.5:  # Safety margin
                    self.get_logger().info('Emergency condition cleared, resuming')
                    self.emergency_stop = False

    def check_recovery_needed(self):
        """
        Check if recovery behavior is needed
        """
        if not self.velocity_buffer:
            return False

        # Check if robot is stuck (low velocity for extended period)
        recent_velocities = list(self.velocity_buffer)[-5:]  # Last 5 measurements
        avg_linear = sum(v['linear'] for v in recent_velocities) / len(recent_velocities)
        avg_angular = sum(v['angular'] for v in recent_velocities) / len(recent_velocities)

        # If robot has very low velocity for a while, it might be stuck
        if abs(avg_linear) < 0.05 and abs(avg_angular) < 0.05:
            return True

        return False

    def execute_recovery_behavior(self):
        """
        Execute recovery behavior when robot is stuck
        """
        if self.recovery_active:
            return  # Already in recovery

        if self.recovery_attempts >= self.max_recovery_attempts:
            self.get_logger().error('Maximum recovery attempts reached')
            return

        self.get_logger().info(f'Executing recovery behavior, attempt {self.recovery_attempts + 1}')
        self.recovery_active = True
        self.recovery_attempts += 1

        # Recovery thread to avoid blocking the main loop
        recovery_thread = threading.Thread(target=self.perform_recovery_action)
        recovery_thread.daemon = True
        recovery_thread.start()

    def perform_recovery_action(self):
        """
        Perform actual recovery action in a separate thread
        """
        try:
            # Back up slightly
            backup_cmd = Twist()
            backup_cmd.linear.x = -0.2  # Move backward
            backup_cmd.angular.z = 0.0

            start_time = self.get_clock().now()
            timeout = rclpy.duration.Duration(seconds=2.0)

            while (self.get_clock().now() - start_time) < timeout and self.recovery_active:
                self.cmd_vel_publisher.publish(backup_cmd)
                time.sleep(0.1)

            # Rotate in place
            rotate_cmd = Twist()
            rotate_cmd.linear.x = 0.0
            rotate_cmd.angular.z = 0.5  # Rotate

            start_time = self.get_clock().now()
            timeout = rclpy.duration.Duration(seconds=2.0)

            while (self.get_clock().now() - start_time) < timeout and self.recovery_active:
                self.cmd_vel_publisher.publish(rotate_cmd)
                time.sleep(0.1)

            # Stop
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)

        except Exception as e:
            self.get_logger().error(f'Recovery action error: {e}')
        finally:
            self.recovery_active = False
            self.get_logger().info('Recovery behavior completed')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down obstacle avoidance node...')
    finally:
        # Send stop command before shutdown
        stop_cmd = Twist()
        node.cmd_vel_publisher.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import time  # Added for the threading sleep
    main()
```

## Code Examples with Explanations

### Example 1: Bipedal Navigation Controller

```python
# File: robot_control_package/robot_control_package/bipedal_navigation.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Pose, Point
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String, Float32
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np
from collections import deque
import threading

class BipedalNavigationController(Node):
    """
    Navigation controller specifically designed for bipedal humanoid robots
    """

    def __init__(self):
        super().__init__('bipedal_navigation_controller')

        # QoS profiles
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Subscriptions
        self.joint_states_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            reliable_qos
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            reliable_qos
        )

        # Publishers
        self.footstep_publisher = self.create_publisher(Pose, 'desired_footstep', reliable_qos)
        self.balance_publisher = self.create_publisher(Twist, 'balance_command', reliable_qos)
        self.bipedal_status_publisher = self.create_publisher(String, 'bipedal_status', reliable_qos)

        # TF2 components
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal state
        self.joint_positions = {}
        self.joint_velocities = {}
        self.imu_data = None
        self.desired_velocity = Twist()
        self.current_gait_phase = 0.0
        self.step_count = 0

        # Bipedal-specific parameters
        self.step_length = 0.3  # meters
        self.step_height = 0.05  # meters
        self.step_period = 1.0  # seconds
        self.com_height = 0.8  # Center of mass height
        self.foot_spacing = 0.2  # Distance between feet

        # Balance control parameters
        self.balance_kp = 1.0
        self.balance_kd = 0.1
        self.max_balance_correction = 0.1  # radians

        # Data buffers
        self.com_trajectory = deque(maxlen=100)
        self.support_polygon = []

        # Timers
        self.gait_timer = self.create_timer(0.01, self.gait_control_loop)  # 100 Hz for gait control
        self.balance_timer = self.create_timer(0.02, self.balance_control_loop)  # 50 Hz for balance

        self.get_logger().info('Bipedal Navigation Controller initialized')

    def joint_states_callback(self, msg):
        """
        Handle joint state updates for bipedal robot
        """
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

    def imu_callback(self, msg):
        """
        Handle IMU data for balance control
        """
        self.imu_data = msg

    def gait_control_loop(self):
        """
        Main gait control loop for bipedal locomotion
        """
        if not self.imu_data:
            return

        # Update gait phase based on desired velocity
        linear_speed = math.sqrt(
            self.desired_velocity.linear.x**2 +
            self.desired_velocity.linear.y**2
        )

        # Adjust step frequency based on desired speed
        current_step_period = max(0.5, self.step_period * (1.0 - linear_speed * 0.5))

        # Update gait phase
        self.current_gait_phase += 2 * math.pi / (current_step_period * 100)  # 100 Hz timer
        if self.current_gait_phase >= 2 * math.pi:
            self.current_gait_phase -= 2 * math.pi

        # Calculate footstep based on gait phase
        footstep = self.calculate_footstep_from_gait_phase()

        # Publish desired footstep
        self.footstep_publisher.publish(footstep)

        # Calculate support polygon for balance
        self.calculate_support_polygon()

        # Update step counter
        if self.current_gait_phase < 0.1 and hasattr(self, '_last_phase_positive'):
            if self._last_phase_positive:
                self.step_count += 1
        self._last_phase_positive = self.current_gait_phase > 0

    def calculate_footstep_from_gait_phase(self):
        """
        Calculate desired footstep based on current gait phase
        """
        footstep = Pose()

        # Determine which foot is swing foot based on gait phase
        if 0 <= self.current_gait_phase < math.pi:
            # Left foot is swing foot (right foot is stance)
            swing_side = 'left'
        else:
            # Right foot is swing foot (left foot is stance)
            swing_side = 'right'

        # Calculate step position based on desired velocity and gait phase
        phase_normalized = self.current_gait_phase / (2 * math.pi)

        # Forward position based on desired velocity
        forward_offset = self.desired_velocity.linear.x * self.step_period * phase_normalized
        lateral_offset = 0  # For now, keep feet under body

        # Vertical position for foot trajectory (cycloid or similar)
        if swing_side == 'left':
            footstep.position.x = forward_offset
            footstep.position.y = self.foot_spacing / 2
        else:
            footstep.position.x = forward_offset
            footstep.position.y = -self.foot_spacing / 2

        # Height trajectory (parabolic lift and place)
        if (0 <= self.current_gait_phase < math.pi/2) or (math.pi <= self.current_gait_phase < 3*math.pi/2):
            # Lift phase
            if swing_side == 'left':
                footstep.position.z = self.step_height * math.sin(self.current_gait_phase)
            else:
                footstep.position.z = self.step_height * math.sin(self.current_gait_phase - math.pi)
        else:
            # Place phase
            if swing_side == 'left':
                footstep.position.z = self.step_height * math.sin(self.current_gait_phase)
            else:
                footstep.position.z = self.step_height * math.sin(self.current_gait_phase - math.pi)

        # Orientation to keep foot level
        footstep.orientation.w = 1.0  # No rotation for now

        return footstep

    def calculate_support_polygon(self):
        """
        Calculate the support polygon for balance control
        """
        # For a biped, support polygon is typically the convex hull of contact points
        # In single support: point at stance foot
        # In double support: line between feet
        # For simplicity, we'll calculate based on gait phase

        if 0 <= self.current_gait_phase < math.pi:
            # Right foot is stance foot
            self.support_polygon = [
                Point(x=0.0, y=-self.foot_spacing/2, z=0.0),  # Right foot
            ]
        else:
            # Left foot is stance foot
            self.support_polygon = [
                Point(x=0.0, y=self.foot_spacing/2, z=0.0),  # Left foot
            ]

        # In double support phase, both feet would be included
        # This is simplified for single support only

    def balance_control_loop(self):
        """
        Balance control loop to maintain stability
        """
        if not self.imu_data:
            return

        # Get roll and pitch from IMU
        quat = self.imu_data.orientation
        roll, pitch, _ = self.quaternion_to_euler(quat)

        # Calculate balance corrections
        roll_correction = -self.balance_kp * roll - self.balance_kd * self.imu_data.angular_velocity.x
        pitch_correction = -self.balance_kp * pitch - self.balance_kd * self.imu_data.angular_velocity.y

        # Limit corrections
        roll_correction = max(-self.max_balance_correction,
                             min(self.max_balance_correction, roll_correction))
        pitch_correction = max(-self.max_balance_correction,
                              min(self.max_balance_correction, pitch_correction))

        # Create balance command
        balance_cmd = Twist()
        balance_cmd.linear.x = roll_correction  # Could be used for hip roll adjustment
        balance_cmd.linear.y = pitch_correction  # Could be used for hip pitch adjustment
        balance_cmd.angular.z = 0.0  # Foot placement adjustment

        # Publish balance command
        self.balance_publisher.publish(balance_cmd)

        # Publish status
        status_msg = String()
        status_msg.data = (
            f"Gait Phase: {self.current_gait_phase:.2f}, "
            f"Roll: {math.degrees(roll):.2f}°, "
            f"Pitch: {math.degrees(pitch):.2f}°, "
            f"Steps: {self.step_count}"
        )
        self.bipedal_status_publisher.publish(status_msg)

    def quaternion_to_euler(self, quaternion):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        """
        import math

        # Simplified conversion for small angles
        q = quaternion
        roll = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x*q.x + q.y*q.y))
        pitch = math.asin(2*(q.w*q.y - q.z*q.x))
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

        return roll, pitch, yaw

    def set_desired_velocity(self, linear_x, linear_y, angular_z):
        """
        Set desired navigation velocity
        """
        self.desired_velocity.linear.x = linear_x
        self.desired_velocity.linear.y = linear_y
        self.desired_velocity.angular.z = angular_z

def main(args=None):
    rclpy.init(args=args)
    node = BipedalNavigationController()

    # Example: Set a slow forward velocity
    node.set_desired_velocity(0.1, 0.0, 0.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down bipedal navigation controller...')
    finally:
        # Send zero velocity to stop
        node.set_desired_velocity(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Sim-to-Real Transfer Node

```python
# File: robot_control_package/robot_control_package/sim_to_real_transfer.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool, Float32
from cv_bridge import CvBridge
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
import math
from collections import deque
import threading

class SimToRealTransferNode(Node):
    """
    Node for facilitating sim-to-real transfer of navigation capabilities
    """

    def __init__(self):
        super().__init__('sim_to_real_transfer_node')

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

        # Subscriptions
        self.sim_scan_subscription = self.create_subscription(
            LaserScan,
            'sim_scan',
            self.sim_scan_callback,
            sensor_qos
        )

        self.real_scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.real_scan_callback,
            sensor_qos
        )

        self.sim_image_subscription = self.create_subscription(
            Image,
            'sim_camera/image_raw',
            self.sim_image_callback,
            sensor_qos
        )

        self.real_image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.real_image_callback,
            sensor_qos
        )

        # Publishers
        self.calibrated_cmd_publisher = self.create_publisher(Twist, 'calibrated_cmd_vel', reliable_qos)
        self.transfer_status_publisher = self.create_publisher(String, 'transfer_status', reliable_qos)
        self.similarity_metrics_publisher = self.create_publisher(Float32, 'sim_real_similarity', reliable_qos)

        # Internal components
        self.cv_bridge = CvBridge()
        self.sim_scan_data = None
        self.real_scan_data = None
        self.sim_image_data = None
        self.real_image_data = None

        # Transfer parameters
        self.domain_gap_threshold = 0.3  # Maximum acceptable domain gap
        self.adaptation_learning_rate = 0.1
        self.similarity_buffer = deque(maxlen=20)

        # Calibration parameters (will be adjusted during transfer)
        self.linear_scale_factor = 1.0
        self.angular_scale_factor = 1.0
        self.sensor_noise_model = {'std': 0.05, 'bias': 0.0}

        # Data collection for domain adaptation
        self.sim_data_buffer = deque(maxlen=100)
        self.real_data_buffer = deque(maxlen=100)

        # Timers
        self.transfer_timer = self.create_timer(0.1, self.sim_to_real_transfer_loop)

        self.get_logger().info('Sim-to-Real Transfer node initialized')

    def sim_scan_callback(self, msg):
        """
        Handle simulation laser scan data
        """
        self.sim_scan_data = msg
        self.sim_data_buffer.append({
            'ranges': list(msg.ranges),
            'timestamp': msg.header.stamp
        })

    def real_scan_callback(self, msg):
        """
        Handle real robot laser scan data
        """
        self.real_scan_data = msg
        self.real_data_buffer.append({
            'ranges': list(msg.ranges),
            'timestamp': msg.header.stamp
        })

    def sim_image_callback(self, msg):
        """
        Handle simulation camera image
        """
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.sim_image_data = cv_image
        except Exception as e:
            self.get_logger().error(f'Sim image conversion error: {e}')

    def real_image_callback(self, msg):
        """
        Handle real camera image
        """
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.real_image_data = cv_image
        except Exception as e:
            self.get_logger().error(f'Real image conversion error: {e}')

    def sim_to_real_transfer_loop(self):
        """
        Main sim-to-real transfer loop
        """
        if not self.sim_scan_data or not self.real_scan_data:
            return

        # Calculate similarity between sim and real data
        similarity = self.calculate_sim_real_similarity()

        # Update similarity buffer
        self.similarity_buffer.append(similarity)

        # Adapt parameters based on similarity
        self.adapt_parameters(similarity)

        # Publish transfer status
        status_msg = String()
        status_msg.data = (
            f"Sim-to-Real Transfer - "
            f"Similarity: {similarity:.3f}, "
            f"Linear Scale: {self.linear_scale_factor:.3f}, "
            f"Angular Scale: {self.angular_scale_factor:.3f}"
        )
        self.transfer_status_publisher.publish(status_msg)

        # Publish similarity metric
        similarity_msg = Float32()
        similarity_msg.data = similarity
        self.similarity_metrics_publisher.publish(similarity_msg)

        # Log if similarity is below threshold
        if similarity < self.domain_gap_threshold:
            self.get_logger().warn(f'Large domain gap detected: {similarity:.3f}')

    def calculate_sim_real_similarity(self):
        """
        Calculate similarity between simulation and real sensor data
        """
        if not self.sim_scan_data or not self.real_scan_data:
            return 0.0

        # Compare laser scan data using a simple distance metric
        sim_ranges = np.array([r if 0 < r < float('inf') else 10.0 for r in self.sim_scan_data.ranges])
        real_ranges = np.array([r if 0 < r < float('inf') else 10.0 for r in self.real_scan_data.ranges])

        # Ensure arrays are the same size
        min_len = min(len(sim_ranges), len(real_ranges))
        sim_ranges = sim_ranges[:min_len]
        real_ranges = real_ranges[:min_len]

        # Calculate normalized distance between scans
        distance_diff = np.abs(sim_ranges - real_ranges)
        normalized_diff = np.mean(distance_diff) / 10.0  # Normalize by max range

        # Convert to similarity (higher is more similar)
        similarity = max(0.0, 1.0 - normalized_diff)

        return similarity

    def adapt_parameters(self, similarity):
        """
        Adapt navigation parameters based on sim-to-real similarity
        """
        # Adjust scaling factors based on domain gap
        if similarity < self.domain_gap_threshold:
            # Increase scaling if domain gap is large
            self.linear_scale_factor *= (1.0 + self.adaptation_learning_rate * (self.domain_gap_threshold - similarity))
            self.angular_scale_factor *= (1.0 + self.adaptation_learning_rate * (self.domain_gap_threshold - similarity))
        else:
            # Gradually return to nominal values if domain gap is small
            self.linear_scale_factor = max(0.8, min(1.2, self.linear_scale_factor * 0.999))
            self.angular_scale_factor = max(0.8, min(1.2, self.angular_scale_factor * 0.999))

        # Keep scaling factors within reasonable bounds
        self.linear_scale_factor = max(0.5, min(2.0, self.linear_scale_factor))
        self.angular_scale_factor = max(0.5, min(2.0, self.angular_scale_factor))

    def calibrate_command(self, raw_cmd):
        """
        Calibrate navigation command for real robot characteristics
        """
        calibrated_cmd = Twist()

        # Apply scaling factors
        calibrated_cmd.linear.x = raw_cmd.linear.x * self.linear_scale_factor
        calibrated_cmd.linear.y = raw_cmd.linear.y * self.linear_scale_factor
        calibrated_cmd.linear.z = raw_cmd.linear.z * self.linear_scale_factor

        calibrated_cmd.angular.x = raw_cmd.angular.x * self.angular_scale_factor
        calibrated_cmd.angular.y = raw_cmd.angular.y * self.angular_scale_factor
        calibrated_cmd.angular.z = raw_cmd.angular.z * self.angular_scale_factor

        # Apply additional calibration based on domain gap
        domain_gap_factor = max(0.8, min(1.0, 1.0 - (self.domain_gap_threshold - np.mean(self.similarity_buffer)) if self.similarity_buffer else 0.0))
        calibrated_cmd.linear.x *= domain_gap_factor
        calibrated_cmd.angular.z *= domain_gap_factor

        return calibrated_cmd

    def get_transfer_readiness(self):
        """
        Get readiness level for sim-to-real transfer
        """
        if len(self.similarity_buffer) < 10:
            return "INSUFFICIENT_DATA"

        avg_similarity = np.mean(list(self.similarity_buffer))
        if avg_similarity > 0.7:
            return "READY"
        elif avg_similarity > 0.4:
            return "CAUTIOUS"
        else:
            return "NOT_READY"

def main(args=None):
    rclpy.init(args=args)
    node = SimToRealTransferNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down sim-to-real transfer node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3: Complete Navigation Pipeline

```python
# File: robot_control_package/robot_control_package/complete_navigation_pipeline.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, Imu, JointState
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Bool, Float32
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np
from collections import deque
import threading

class CompleteNavigationPipelineNode(Node):
    """
    Complete navigation pipeline integrating all components
    """

    def __init__(self):
        super().__init__('complete_navigation_pipeline_node')

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

        # Subscriptions
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            reliable_qos
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            sensor_qos
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            sensor_qos
        )

        self.joint_states_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            reliable_qos
        )

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            'move_base_simple/goal',
            self.goal_callback,
            reliable_qos
        )

        self.initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.initial_pose_callback,
            reliable_qos
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', reliable_qos)
        self.global_plan_publisher = self.create_publisher(Path, 'global_plan', reliable_qos)
        self.local_plan_publisher = self.create_publisher(Path, 'local_plan', reliable_qos)
        self.system_status_publisher = self.create_publisher(String, 'system_status', reliable_qos)
        self.safety_status_publisher = self.create_publisher(Bool, 'safety_status', reliable_qos)

        # TF2 components
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal state
        self.current_pose = None
        self.scan_data = None
        self.imu_data = None
        self.joint_positions = {}
        self.navigation_goal = None
        self.initial_pose = None
        self.system_active = True
        self.safety_engaged = False

        # Navigation state machine
        self.navigation_state = 'IDLE'  # IDLE, PLANNING, NAVIGATING, AVOIDING, RECOVERING, ERROR
        self.previous_state = 'IDLE'

        # Navigation parameters
        self.linear_velocity = 0.5  # m/s
        self.angular_velocity = 0.5  # rad/s
        self.min_obstacle_distance = 0.6  # meters
        self.goal_tolerance = 0.5  # meters
        self.rotation_threshold = 0.2  # radians

        # Data buffers
        self.pose_history = deque(maxlen=50)
        self.velocity_history = deque(maxlen=20)
        self.obstacle_history = deque(maxlen=10)

        # Timers
        self.main_control_timer = self.create_timer(0.1, self.main_control_loop)  # 10 Hz
        self.safety_timer = self.create_timer(0.05, self.safety_monitoring_loop)  # 20 Hz
        self.status_timer = self.create_timer(1.0, self.publish_system_status)  # 1 Hz

        self.get_logger().info('Complete Navigation Pipeline node initialized')

    def odom_callback(self, msg):
        """
        Handle odometry data
        """
        self.current_pose = msg.pose.pose

        # Add to history
        self.pose_history.append({
            'position': (msg.pose.pose.position.x, msg.pose.pose.position.y),
            'timestamp': self.get_clock().now()
        })

        # Add velocity to history
        self.velocity_history.append({
            'linear': msg.twist.twist.linear.x,
            'angular': msg.twist.twist.angular.z,
            'timestamp': self.get_clock().now()
        })

    def scan_callback(self, msg):
        """
        Handle laser scan data
        """
        self.scan_data = msg

    def imu_callback(self, msg):
        """
        Handle IMU data
        """
        self.imu_data = msg

    def joint_states_callback(self, msg):
        """
        Handle joint states
        """
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def goal_callback(self, msg):
        """
        Handle navigation goal
        """
        self.navigation_goal = msg.pose
        self.get_logger().info(f'Navigation goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        # Transition to planning state
        if self.navigation_state == 'IDLE':
            self.navigation_state = 'PLANNING'

    def initial_pose_callback(self, msg):
        """
        Handle initial pose
        """
        self.initial_pose = msg.pose.pose
        self.get_logger().info('Initial pose set')

    def main_control_loop(self):
        """
        Main navigation control loop
        """
        if not self.system_active or not self.current_pose:
            return

        # State machine for navigation
        if self.navigation_state == 'IDLE':
            self.handle_idle_state()
        elif self.navigation_state == 'PLANNING':
            self.handle_planning_state()
        elif self.navigation_state == 'NAVIGATING':
            self.handle_navigating_state()
        elif self.navigation_state == 'AVOIDING':
            self.handle_avoiding_state()
        elif self.navigation_state == 'RECOVERING':
            self.handle_recovering_state()
        elif self.navigation_state == 'ERROR':
            self.handle_error_state()

        # Check for state transitions
        self.check_state_transitions()

    def handle_idle_state(self):
        """
        Handle idle state - waiting for goal
        """
        # Publish zero velocity
        cmd = Twist()
        self.cmd_vel_publisher.publish(cmd)

    def handle_planning_state(self):
        """
        Handle planning state - compute path to goal
        """
        if self.navigation_goal:
            # In a real implementation, this would call the path planner
            # For this example, we'll just transition to navigating
            self.get_logger().info('Path planning completed')
            self.navigation_state = 'NAVIGATING'
        else:
            # No goal, return to idle
            self.navigation_state = 'IDLE'

    def handle_navigating_state(self):
        """
        Handle navigating state - follow path to goal
        """
        if not self.navigation_goal:
            self.navigation_state = 'IDLE'
            return

        # Check for obstacles
        if self.scan_data and self.detect_obstacles():
            self.navigation_state = 'AVOIDING'
            return

        # Calculate command to reach goal
        cmd = self.compute_navigation_command()
        self.cmd_vel_publisher.publish(cmd)

        # Check if goal is reached
        if self.is_goal_reached():
            self.get_logger().info('Goal reached!')
            self.navigation_state = 'IDLE'

    def handle_avoiding_state(self):
        """
        Handle obstacle avoidance state
        """
        if self.scan_data:
            cmd = self.compute_avoidance_command()
            self.cmd_vel_publisher.publish(cmd)

            # Check if obstacle is cleared
            if not self.detect_obstacles():
                self.navigation_state = 'NAVIGATING'

    def handle_recovering_state(self):
        """
        Handle recovery state
        """
        # In a real implementation, this would execute recovery behaviors
        # For now, we'll just return to navigating after a short time
        self.get_logger().info('Executing recovery behavior...')

        # Recovery command (backup and rotate)
        cmd = Twist()
        cmd.linear.x = -0.2  # Backup
        cmd.angular.z = 0.5  # Rotate
        self.cmd_vel_publisher.publish(cmd)

        # After 2 seconds, return to navigation
        if not hasattr(self, 'recovery_start_time'):
            self.recovery_start_time = self.get_clock().now()

        if (self.get_clock().now() - self.recovery_start_time).nanoseconds / 1e9 > 2.0:
            self.navigation_state = 'NAVIGATING'
            delattr(self, 'recovery_start_time')

    def handle_error_state(self):
        """
        Handle error state
        """
        # Emergency stop
        cmd = Twist()
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().error('System in error state - stopped')

    def check_state_transitions(self):
        """
        Check for necessary state transitions
        """
        # Log state changes
        if self.navigation_state != self.previous_state:
            self.get_logger().info(f'State transition: {self.previous_state} -> {self.navigation_state}')
            self.previous_state = self.navigation_state

    def compute_navigation_command(self):
        """
        Compute navigation command to reach goal
        """
        cmd = Twist()

        if not self.current_pose or not self.navigation_goal:
            return cmd

        # Calculate distance and angle to goal
        dx = self.navigation_goal.position.x - self.current_pose.position.x
        dy = self.navigation_goal.position.y - self.current_pose.position.y
        distance_to_goal = math.sqrt(dx*dx + dy*dy)

        # Calculate desired heading
        desired_yaw = math.atan2(dy, dx)

        # Get current yaw from orientation
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)

        # Calculate heading error
        heading_error = self.normalize_angle(desired_yaw - current_yaw)

        # Simple proportional controller
        if distance_to_goal > self.goal_tolerance:
            cmd.linear.x = min(self.linear_velocity, distance_to_goal * 0.5)
            cmd.angular.z = max(-self.angular_velocity, min(self.angular_velocity, heading_error * 1.0))
        else:
            # Near goal, focus on orientation
            cmd.angular.z = max(-self.angular_velocity, min(self.angular_velocity, heading_error * 2.0))

        return cmd

    def compute_avoidance_command(self):
        """
        Compute obstacle avoidance command
        """
        cmd = Twist()

        if not self.scan_data:
            return cmd

        # Simple reactive avoidance
        ranges = self.scan_data.ranges
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment

        # Check front sector for obstacles
        front_center = len(ranges) // 2
        front_sector = ranges[max(0, front_center-10):min(len(ranges), front_center+10)]

        # Find minimum distance in front
        valid_ranges = [r for r in front_sector if 0 < r < float('inf')]
        if valid_ranges:
            min_front_dist = min(valid_ranges)

            if min_front_dist < self.min_obstacle_distance:
                # Too close, rotate away
                left_avg = np.mean([r for r in ranges[:len(ranges)//2] if 0 < r < float('inf')] or [10.0])
                right_avg = np.mean([r for r in ranges[len(ranges)//2:] if 0 < r < float('inf')] or [10.0])

                if left_avg > right_avg:
                    cmd.angular.z = self.angular_velocity * 0.5  # Turn left
                else:
                    cmd.angular.z = -self.angular_velocity * 0.5  # Turn right
            else:
                # Obstacle far enough, continue forward
                cmd.linear.x = self.linear_velocity * 0.3  # Slower when avoiding

        return cmd

    def detect_obstacles(self):
        """
        Detect obstacles in front of robot
        """
        if not self.scan_data:
            return False

        ranges = self.scan_data.ranges
        front_center = len(ranges) // 2
        front_sector = ranges[max(0, front_center-15):min(len(ranges), front_center+15)]

        valid_ranges = [r for r in front_sector if 0 < r < float('inf')]
        if valid_ranges:
            min_front_dist = min(valid_ranges)
            return min_front_dist < self.min_obstacle_distance

        return False

    def is_goal_reached(self):
        """
        Check if navigation goal is reached
        """
        if not self.current_pose or not self.navigation_goal:
            return False

        dx = self.navigation_goal.position.x - self.current_pose.position.x
        dy = self.navigation_goal.position.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        return distance <= self.goal_tolerance

    def quaternion_to_yaw(self, orientation):
        """
        Convert quaternion to yaw angle
        """
        import math
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi] range
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def safety_monitoring_loop(self):
        """
        Continuously monitor safety conditions
        """
        safety_ok = True

        # Check for emergency stop conditions
        if self.scan_data:
            ranges = [r for r in self.scan_data.ranges if 0 < r < float('inf')]
            if ranges and min(ranges) < 0.2:  # Emergency stop distance
                safety_ok = False
                if not self.safety_engaged:
                    self.get_logger().warn('EMERGENCY STOP - Very close obstacle detected!')

        # Check IMU for abnormal readings (if available)
        if self.imu_data:
            # Check for excessive tilt (indicating potential fall)
            roll, pitch, _ = self.quaternion_to_euler(self.imu_data.orientation)
            if abs(roll) > 1.0 or abs(pitch) > 1.0:  # ~57 degrees
                safety_ok = False
                if not self.safety_engaged:
                    self.get_logger().warn('EMERGENCY STOP - Excessive tilt detected!')

        # Update safety status
        if not safety_ok and not self.safety_engaged:
            self.safety_engaged = True
            self.get_logger().error('SAFETY SYSTEM ENGAGED - EMERGENCY STOP!')

            # Send emergency stop command
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)

            # Set error state
            self.navigation_state = 'ERROR'
        elif safety_ok and self.safety_engaged:
            self.safety_engaged = False
            self.get_logger().info('Safety system disengaged - resuming normal operation')

        # Publish safety status
        safety_msg = Bool()
        safety_msg.data = safety_ok
        self.safety_status_publisher.publish(safety_msg)

    def quaternion_to_euler(self, quaternion):
        """
        Convert quaternion to Euler angles
        """
        import math

        q = quaternion
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def publish_system_status(self):
        """
        Publish overall system status
        """
        status_msg = String()

        # Basic status
        status_parts = [
            f"State: {self.navigation_state}",
            f"Pose: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})" if self.current_pose else "Pose: Unknown",
            f"Goal: ({self.navigation_goal.position.x:.2f}, {self.navigation_goal.position.y:.2f})" if self.navigation_goal else "Goal: None",
            f"Safety: {'OK' if not self.safety_engaged else 'ENGAGED'}",
            f"Obstacles: {'Yes' if self.scan_data and self.detect_obstacles() else 'No'}" if self.scan_data else "Unknown"
        ]

        status_msg.data = " | ".join(status_parts)
        self.system_status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CompleteNavigationPipelineNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down complete navigation pipeline...')

        # Send stop command before shutdown
        stop_cmd = Twist()
        node.cmd_vel_publisher.publish(stop_cmd)

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-On Exercises: Navigation System Implementation

### Exercise 1: Create a Custom Path Planner

Implement a custom path planner that works with Nav2:

```python
# File: robot_control_package/robot_control_package/custom_path_planner.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np
from scipy.spatial import distance
import threading

class CustomPathPlannerNode(Node):
    """
    Custom path planner implementation
    """

    def __init__(self):
        super().__init__('custom_path_planner_node')

        # QoS profiles
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Action server for path planning
        from rclpy.action import ActionServer
        self.path_planning_server = ActionServer(
            self,
            ComputePathToPose,
            'compute_path_to_pose_custom',
            self.execute_path_planning
        )

        # Subscriptions
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,
            'global_costmap/costmap',
            self.costmap_callback,
            reliable_qos
        )

        # Publishers
        self.planned_path_publisher = self.create_publisher(Path, 'custom_planned_path', reliable_qos)
        self.planner_status_publisher = self.create_publisher(String, 'custom_planner_status', reliable_qos)

        # TF2 components
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal state
        self.costmap_data = None
        self.costmap_resolution = 0.05
        self.planner_active = False

        # Planning parameters
        self.planning_algorithm = 'astar'  # 'astar', 'rrt', 'dijkstra'
        self.smoothing_iterations = 10
        self.smoothing_weight = 0.1

        self.get_logger().info('Custom Path Planner node initialized')

    def costmap_callback(self, msg):
        """
        Handle costmap data
        """
        self.costmap_data = msg
        self.costmap_resolution = msg.info.resolution

    def execute_path_planning(self, goal_handle):
        """
        Execute path planning action
        """
        self.get_logger().info('Executing custom path planning')

        start = goal_handle.request.start.pose
        goal = goal_handle.request.goal.pose

        # Convert world coordinates to costmap coordinates
        start_map = self.world_to_map_coords(start.position.x, start.position.y)
        goal_map = self.world_to_map_coords(goal.position.x, goal.position.y)

        if not start_map or not goal_map:
            self.get_logger().error('Could not convert coordinates to map space')
            result = ComputePathToPose.Result()
            result.path = Path()
            goal_handle.abort()
            return result

        # Plan path using selected algorithm
        path = self.plan_path(start_map, goal_map)

        if path:
            # Convert map path back to world coordinates
            world_path = self.map_path_to_world(path)

            # Smooth the path
            smoothed_path = self.smooth_path(world_path)

            # Create Path message
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for point in smoothed_path:
                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose.position.x = point[0]
                pose_stamped.pose.position.y = point[1]
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.w = 1.0
                path_msg.poses.append(pose_stamped)

            # Publish planned path
            self.planned_path_publisher.publish(path_msg)

            # Create result
            result = ComputePathToPose.Result()
            result.path = path_msg

            goal_handle.succeed()
            self.get_logger().info(f'Path planning succeeded with {len(path_msg.poses)} waypoints')
        else:
            self.get_logger().error('Could not find a path to goal')
            result = ComputePathToPose.Result()
            result.path = Path()
            goal_handle.abort()

        return result

    def plan_path(self, start, goal):
        """
        Plan path using selected algorithm
        """
        if self.planning_algorithm == 'astar':
            return self.astar_plan(start, goal)
        elif self.planning_algorithm == 'dijkstra':
            return self.dijkstra_plan(start, goal)
        else:
            return self.astar_plan(start, goal)  # Default to A*

    def astar_plan(self, start, goal):
        """
        A* path planning algorithm
        """
        if not self.costmap_data:
            return None

        # Create a simple grid for A* (in practice, you'd use the actual costmap)
        height = self.costmap_data.info.height
        width = self.costmap_data.info.width

        # Check if start and goal are valid
        if (not self.is_valid_cell(start[0], start[1], width, height) or
            not self.is_valid_cell(goal[0], goal[1], width, height)):
            return None

        # Check if start or goal are in obstacle space
        start_idx = start[1] * width + start[0]
        goal_idx = goal[1] * width + goal[0]

        if (self.costmap_data.data[start_idx] > 50 or  # Lethal cost
            self.costmap_data.data[goal_idx] > 50):
            return None

        # A* implementation
        open_set = [(0, start)]  # (f_score, position)
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        visited = set()

        while open_set:
            # Sort by f_score (this is inefficient; use heapq in real implementation)
            open_set.sort(key=lambda x: x[0])
            current_f, current = open_set.pop(0)

            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            visited.add(current)

            # Check neighbors
            for neighbor in self.get_neighbors(current, width, height):
                if neighbor in visited:
                    continue

                # Check if neighbor is valid (not an obstacle)
                neighbor_idx = neighbor[1] * width + neighbor[0]
                if neighbor_idx < len(self.costmap_data.data):
                    if self.costmap_data.data[neighbor_idx] > 50:  # Lethal cost
                        continue

                tentative_g_score = g_score.get(current, float('inf')) + 1

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)

                    # Add to open set if not already there
                    if neighbor not in [item[1] for item in open_set]:
                        open_set.append((f_score[neighbor], neighbor))

        return None  # No path found

    def dijkstra_plan(self, start, goal):
        """
        Dijkstra path planning algorithm (simplified)
        """
        # For this example, we'll use the same structure as A* but without heuristic
        # In a real implementation, Dijkstra would use uniform cost instead of A*'s heuristic
        return self.astar_plan(start, goal)

    def get_neighbors(self, pos, width, height):
        """
        Get valid neighbors for a position (8-connectivity)
        """
        x, y = pos
        neighbors = []

        # 8-connected neighborhood
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue  # Skip current position

                nx, ny = x + dx, y + dy

                if self.is_valid_cell(nx, ny, width, height):
                    neighbors.append((nx, ny))

        return neighbors

    def is_valid_cell(self, x, y, width, height):
        """
        Check if a cell is valid (within bounds)
        """
        return 0 <= x < width and 0 <= y < height

    def heuristic(self, pos1, pos2):
        """
        Heuristic function for A* (Euclidean distance)
        """
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def world_to_map_coords(self, x, y):
        """
        Convert world coordinates to map cell coordinates
        """
        if not self.costmap_data:
            return None

        origin_x = self.costmap_data.info.origin.position.x
        origin_y = self.costmap_data.info.origin.position.y
        resolution = self.costmap_data.info.resolution

        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)

        if (0 <= map_x < self.costmap_data.info.width and
            0 <= map_y < self.costmap_data.info.height):
            return (map_x, map_y)

        return None

    def map_path_to_world(self, map_path):
        """
        Convert map path to world coordinates
        """
        if not self.costmap_data:
            return []

        origin_x = self.costmap_data.info.origin.position.x
        origin_y = self.costmap_data.info.origin.position.y
        resolution = self.costmap_data.info.resolution

        world_path = []
        for x, y in map_path:
            world_x = x * resolution + origin_x
            world_y = y * resolution + origin_y
            world_path.append((world_x, world_y))

        return world_path

    def smooth_path(self, path):
        """
        Smooth the path using an iterative algorithm
        """
        if len(path) < 3:
            return path

        # Convert to numpy array for easier manipulation
        points = np.array(path)

        # Iterative smoothing
        for _ in range(self.smoothing_iterations):
            for i in range(1, len(points) - 1):
                # Calculate smoothing: mix current point with neighbors
                prev_point = points[i-1]
                next_point = points[i+1]

                # Apply smoothing formula
                new_point = (1 - self.smoothing_weight) * points[i] + \
                           self.smoothing_weight * 0.5 * (prev_point + next_point)

                points[i] = new_point

        # Convert back to list of tuples
        return [(point[0], point[1]) for point in points]

def main(args=None):
    rclpy.init(args=args)
    node = CustomPathPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down custom path planner...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 2: Implement Recovery Behaviors

Create a comprehensive recovery behavior system:

```python
# File: robot_control_package/robot_control_package/recovery_behaviors.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np
from collections import deque
import threading
import time

class RecoveryBehaviorsNode(Node):
    """
    Comprehensive recovery behaviors for navigation
    """

    def __init__(self):
        super().__init__('recovery_behaviors_node')

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

        # Subscriptions
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            reliable_qos
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            sensor_qos
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            reliable_qos
        )

        # Publishers
        self.recovery_cmd_publisher = self.create_publisher(Twist, 'recovery_cmd_vel', reliable_qos)
        self.recovery_status_publisher = self.create_publisher(String, 'recovery_status', reliable_qos)
        self.recovery_trigger_publisher = self.create_publisher(Bool, 'recovery_triggered', reliable_qos)

        # TF2 components
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal state
        self.current_pose = None
        self.scan_data = None
        self.last_cmd_vel = Twist()
        self.recovery_active = False
        self.recovery_behavior = None
        self.recovery_start_time = None
        self.recovery_attempt_count = 0
        self.max_recovery_attempts = 3

        # Recovery detection parameters
        self.stuck_velocity_threshold = 0.01  # m/s
        self.stuck_time_threshold = 5.0  # seconds
        self.rotation_stuck_threshold = 0.05  # rad/s
        self.obstacle_too_close_threshold = 0.3  # meters

        # Data buffers for detection
        self.velocity_buffer = deque(maxlen=50)
        self.position_buffer = deque(maxlen=20)
        self.cmd_buffer = deque(maxlen=20)

        # Available recovery behaviors
        self.recovery_behaviors = {
            'clear_costmap': self.clear_costmap_recovery,
            'backup': self.backup_recovery,
            'rotate_in_place': self.rotate_in_place_recovery,
            'wander': self.wander_recovery,
            'reset_localization': self.reset_localization_recovery
        }

        # Timers
        self.recovery_detection_timer = self.create_timer(1.0, self.detect_recovery_conditions)
        self.recovery_execution_timer = self.create_timer(0.1, self.execute_recovery)

        self.get_logger().info('Recovery Behaviors node initialized')

    def odom_callback(self, msg):
        """
        Handle odometry data
        """
        self.current_pose = msg.pose.pose

        # Add to velocity buffer
        lin_vel = msg.twist.twist.linear.x
        ang_vel = msg.twist.twist.angular.z
        self.velocity_buffer.append({
            'linear': abs(lin_vel),
            'angular': abs(ang_vel),
            'timestamp': self.get_clock().now()
        })

        # Add to position buffer
        self.position_buffer.append({
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'timestamp': self.get_clock().now()
        })

    def scan_callback(self, msg):
        """
        Handle laser scan data
        """
        self.scan_data = msg

    def cmd_vel_callback(self, msg):
        """
        Monitor command velocity
        """
        self.last_cmd_vel = msg
        self.cmd_buffer.append({
            'linear': abs(msg.linear.x),
            'angular': abs(msg.angular.z),
            'timestamp': self.get_clock().now()
        })

    def detect_recovery_conditions(self):
        """
        Detect when recovery behaviors are needed
        """
        if self.recovery_active:
            return  # Don't trigger new recovery while one is active

        # Check if robot is stuck (low velocity for extended period)
        is_stuck = self.is_robot_stuck()

        # Check for obstacle too close
        too_close_to_obstacle = self.is_too_close_to_obstacle()

        # Check for oscillation
        is_oscillating = self.is_oscillating()

        if is_stuck or too_close_to_obstacle or is_oscillating:
            # Determine appropriate recovery behavior
            if too_close_to_obstacle:
                self.recovery_behavior = 'backup'
            elif is_oscillating:
                self.recovery_behavior = 'rotate_in_place'
            else:
                self.recovery_behavior = 'wander'  # General stuck recovery

            self.trigger_recovery()

    def is_robot_stuck(self):
        """
        Check if robot appears to be stuck
        """
        if len(self.velocity_buffer) < 10:
            return False

        # Check recent velocities
        recent_velocities = list(self.velocity_buffer)[-10:]
        avg_linear = sum(v['linear'] for v in recent_velocities) / len(recent_velocities)
        avg_angular = sum(v['angular'] for v in recent_velocities) / len(recent_velocities)

        # Check if robot has barely moved in a while
        if len(self.position_buffer) >= 10:
            old_pos = self.position_buffer[0]
            current_pos = self.position_buffer[-1]
            distance_moved = math.sqrt(
                (current_pos['x'] - old_pos['x'])**2 +
                (current_pos['y'] - old_pos['y'])**2
            )

            time_elapsed = (current_pos['timestamp'] - old_pos['timestamp']).nanoseconds / 1e9

            if time_elapsed > 0:
                avg_speed = distance_moved / time_elapsed
                if avg_speed < self.stuck_velocity_threshold:
                    return True

        return (avg_linear < self.stuck_velocity_threshold and
                avg_angular < self.rotation_stuck_threshold)

    def is_too_close_to_obstacle(self):
        """
        Check if robot is too close to obstacles
        """
        if not self.scan_data:
            return False

        ranges = [r for r in self.scan_data.ranges if 0 < r < float('inf')]
        if ranges:
            min_range = min(ranges)
            return min_range < self.obstacle_too_close_threshold

        return False

    def is_oscillating(self):
        """
        Check if robot is oscillating (common in local planners)
        """
        if len(self.cmd_buffer) < 5:
            return False

        recent_cmds = list(self.cmd_buffer)[-5:]

        # Check for alternating positive/negative commands (oscillation)
        linear_cmds = [c['linear'] for c in recent_cmds]
        angular_cmds = [c['angular'] for c in recent_cmds]

        # Simple oscillation detection: alternating signs
        sign_changes = 0
        for i in range(1, len(angular_cmds)):
            if (angular_cmds[i-1] > 0.1 and angular_cmds[i] < -0.1) or \
               (angular_cmds[i-1] < -0.1 and angular_cmds[i] > 0.1):
                sign_changes += 1

        return sign_changes >= 2

    def trigger_recovery(self):
        """
        Trigger recovery behavior
        """
        if self.recovery_attempt_count >= self.max_recovery_attempts:
            self.get_logger().error('Maximum recovery attempts reached')
            return

        if self.recovery_behavior in self.recovery_behaviors:
            self.recovery_active = True
            self.recovery_start_time = self.get_clock().now()
            self.recovery_attempt_count += 1

            self.get_logger().info(
                f'Executing recovery behavior: {self.recovery_behavior} '
                f'(attempt {self.recovery_attempt_count}/{self.max_recovery_attempts})'
            )

            # Publish recovery trigger
            trigger_msg = Bool()
            trigger_msg.data = True
            self.recovery_trigger_publisher.publish(trigger_msg)

            # Execute recovery in a separate thread to avoid blocking
            recovery_thread = threading.Thread(target=self.execute_specific_recovery)
            recovery_thread.daemon = True
            recovery_thread.start()

    def execute_specific_recovery(self):
        """
        Execute the specific recovery behavior
        """
        try:
            behavior_func = self.recovery_behaviors[self.recovery_behavior]
            behavior_func()
        except Exception as e:
            self.get_logger().error(f'Error in recovery behavior {self.recovery_behavior}: {e}')
        finally:
            self.finish_recovery()

    def execute_recovery(self):
        """
        Main recovery execution loop
        """
        if self.recovery_active:
            # Recovery is handled in separate thread, just monitor timeout
            elapsed = (self.get_clock().now() - self.recovery_start_time).nanoseconds / 1e9
            if elapsed > 10.0:  # 10 second timeout
                self.get_logger().warn('Recovery behavior timed out')
                self.finish_recovery()

    def finish_recovery(self):
        """
        Finish recovery behavior
        """
        self.recovery_active = False
        self.recovery_start_time = None

        # Publish stop command
        stop_cmd = Twist()
        self.recovery_cmd_publisher.publish(stop_cmd)

        self.get_logger().info('Recovery behavior completed')

        # Publish status
        status_msg = String()
        status_msg.data = f"Recovery completed: {self.recovery_behavior}"
        self.recovery_status_publisher.publish(status_msg)

    def backup_recovery(self):
        """
        Move robot backward to clear obstacles
        """
        cmd = Twist()
        cmd.linear.x = -0.2  # Move backward
        cmd.angular.z = 0.0

        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=2.0)

        while (self.get_clock().now() - start_time) < duration and self.recovery_active:
            self.recovery_cmd_publisher.publish(cmd)
            time.sleep(0.1)

    def rotate_in_place_recovery(self):
        """
        Rotate robot in place to find clearer direction
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Rotate at 0.5 rad/s

        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=4.0)  # Rotate 180 degrees

        while (self.get_clock().now() - start_time) < duration and self.recovery_active:
            self.recovery_cmd_publisher.publish(cmd)
            time.sleep(0.1)

    def wander_recovery(self):
        """
        Wander in a random direction to escape local minima
        """
        # Choose a random direction
        import random
        turn_angle = random.uniform(-math.pi, math.pi)
        distance = random.uniform(0.5, 1.5)  # meters

        # First rotate to desired direction
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5 if turn_angle > 0 else -0.5

        rotation_time = abs(turn_angle) / 0.5
        start_time = self.get_clock().now()

        while ((self.get_clock().now() - start_time).nanoseconds / 1e9 < rotation_time
               and self.recovery_active):
            self.recovery_cmd_publisher.publish(cmd)
            time.sleep(0.1)

        # Then move forward
        cmd.angular.z = 0.0
        cmd.linear.x = 0.3  # Move forward

        movement_time = distance / 0.3
        start_time = self.get_clock().now()

        while ((self.get_clock().now() - start_time).nanoseconds / 1e9 < movement_time
               and self.recovery_active):
            self.recovery_cmd_publisher.publish(cmd)
            time.sleep(0.1)

    def clear_costmap_recovery(self):
        """
        Simulate clearing costmaps (in real system, would call service)
        """
        # In a real system, this would call services to clear costmaps
        # For simulation, just wait
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=1.0)

        while (self.get_clock().now() - start_time) < duration and self.recovery_active:
            time.sleep(0.1)

    def reset_localization_recovery(self):
        """
        Simulate resetting localization (in real system, would call service)
        """
        # In a real system, this would call localization reset services
        # For simulation, just wait
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=2.0)

        while (self.get_clock().now() - start_time) < duration and self.recovery_active:
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = RecoveryBehaviorsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down recovery behaviors node...')

        # Stop any active recovery
        stop_cmd = Twist()
        node.recovery_cmd_publisher.publish(stop_cmd)

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Pitfalls and Solutions

### Pitfall 1: Navigation Performance Issues
**Problem**: Navigation system performs poorly in real-world environments with dynamic obstacles.

**Solutions**:
- Implement proper costmap configuration with inflation layers
- Tune local planner parameters for your specific robot
- Use appropriate sensor fusion for reliable obstacle detection
- Implement proper recovery behaviors

```python
def tune_costmap_parameters(self):
    """
    Properly tune costmap parameters for your robot
    """
    # Example costmap tuning
    tuned_params = {
        'inflation_radius': 0.55,  # Adjust based on robot size and safety requirements
        'cost_scaling_factor': 3.0,  # Higher for more conservative planning
        'obstacle_range': 2.5,  # How far to sense obstacles
        'raytrace_range': 3.0,  # How far to clear free space
        'footprint_padding': 0.02  # Small padding for safety
    }
    return tuned_params
```

### Pitfall 2: Sim-to-Real Transfer Failure
**Problem**: Navigation system that works in simulation fails in real world.

**Solutions**:
- Use domain randomization in simulation
- Account for sensor noise and latency differences
- Implement adaptive controllers that adjust to real conditions
- Extensive testing in both simulation and reality

### Pitfall 3: Oscillation and Local Minima
**Problem**: Robot gets stuck oscillating between conflicting commands.

**Solutions**:
- Implement proper oscillation detection
- Use appropriate recovery behaviors
- Tune planner parameters to avoid oscillation
- Implement hysteresis in decision making

## Review Questions

1. What are the key components of the Nav2 navigation stack?
2. Explain the difference between global and local planners in navigation.
3. How do you handle obstacle avoidance in real-time navigation?
4. What are the challenges specific to bipedal robot navigation?
5. Describe the sim-to-real transfer process and its importance.

## Project Assignment: Complete Navigation System

Create a complete navigation system that includes:
1. Global path planning with Nav2 integration
2. Local path planning and trajectory generation
3. Obstacle avoidance and recovery behaviors
4. Bipedal-specific navigation adaptations
5. Sim-to-real transfer capabilities
6. Comprehensive safety and monitoring systems

Your system should:
- Demonstrate robust navigation in various environments
- Include proper error handling and recovery
- Show adaptation from simulation to real world
- Provide comprehensive logging and monitoring
- Be optimized for your specific robot platform
- Include launch files for easy deployment

## Further Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [Navigation Tuning Guide](https://navigation.ros.org/configuration/index.html)
- [ROS 2 Navigation Tutorials](https://navigation.ros.org/tutorials/index.html)
- [Path Planning Algorithms](https://planning.cs.uiuc.edu/)
- [Mobile Robot Navigation](https://www.springer.com/gp/book/9783030474514)

:::info
Successful navigation requires careful integration of perception, planning, and control. Always test extensively in simulation before deploying to real robots, and implement robust safety mechanisms to prevent accidents.
:::