---
title: Week 7 - Multi-Sensor Fusion
description: Implementing multi-sensor fusion for comprehensive environment perception
sidebar_position: 2
---

# Week 7: Multi-Sensor Fusion

## Learning Objectives
- Implement multi-sensor fusion in simulation for complex perception tasks
- Understand sensor fusion algorithms and their applications in robotics
- Create comprehensive environment awareness systems
- Integrate data from multiple sensor types effectively
- Build robust perception systems that handle sensor failures gracefully

## Prerequisites Check
- Understanding of individual sensor types (LIDAR, cameras, IMU, etc.)
- Knowledge of ROS 2 communication patterns
- Basic understanding of coordinate transformations
- Experience with simulation environments

## Theoretical Concepts: Multi-Sensor Fusion Fundamentals

### Introduction to Sensor Fusion

Sensor fusion is the process of combining data from multiple sensors to achieve better accuracy, reliability, and robustness than could be achieved by using any single sensor alone. In robotics, sensor fusion is essential because:

- **Redundancy**: Multiple sensors provide backup when one fails
- **Complementarity**: Different sensors provide different types of information
- **Accuracy**: Combining sensors can reduce overall error
- **Robustness**: Systems become less susceptible to environmental conditions

### Types of Sensor Fusion

**1. Data-Level Fusion (Low-Level)**
- Combines raw sensor data directly
- Requires precise timing synchronization
- Computationally intensive but preserves all information

**2. Feature-Level Fusion**
- Extracts features from each sensor first
- Combines features to form a more complete picture
- Good balance between performance and information preservation

**3. Decision-Level Fusion**
- Each sensor makes independent decisions
- Combines decisions using voting or other methods
- Computationally efficient but may lose information

### Common Fusion Algorithms

**Kalman Filters**
- Optimal for linear systems with Gaussian noise
- Predict-correct cycle for state estimation
- Variants: Extended KF, Unscented KF for nonlinear systems

**Particle Filters**
- Non-parametric approach for non-Gaussian distributions
- Represents probability distribution with particles
- Good for multimodal distributions

**Bayesian Networks**
- Probabilistic graphical models
- Handle uncertainty and dependencies
- Good for complex decision making

### Sensor Characteristics and Complementarity

Different sensors have complementary strengths and weaknesses:

| Sensor Type | Strengths | Weaknesses | Best Use Cases |
|-------------|-----------|------------|----------------|
| LIDAR | Precise distance, works in dark | Expensive, affected by weather | Mapping, obstacle detection |
| Cameras | Rich visual information | Light dependent, ambiguous depth | Recognition, scene understanding |
| IMU | High-frequency orientation | Drifts over time | Stabilization, short-term motion |
| GPS | Absolute positioning | Indoors, low update rate | Outdoor navigation |
| Sonar | Simple, low-cost | Limited range, narrow beam | Close-range detection |

## Step-by-Step Tutorials: Implementing Sensor Fusion

### Tutorial 1: Basic Sensor Fusion Node

Let's create a basic sensor fusion node that combines LIDAR and camera data:

```python
# File: robot_control_package/robot_control_package/basic_fusion.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import threading
import queue

class BasicFusionNode(Node):
    """
    Basic sensor fusion node combining LIDAR and camera data
    """

    def __init__(self):
        super().__init__('basic_fusion_node')

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
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            sensor_qos
        )

        self.camera_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.camera_callback,
            sensor_qos
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            reliable_qos
        )

        # Publishers
        self.fused_publisher = self.create_publisher(
            PointCloud2, 'fused_pointcloud', reliable_qos
        )
        self.fusion_status_publisher = self.create_publisher(
            String, 'fusion_status', reliable_qos
        )

        # Internal state
        self.cv_bridge = CvBridge()
        self.camera_info = None
        self.lidar_data = None
        self.camera_data = None
        self.fusion_queue = queue.Queue(maxsize=10)

        # Camera-LIDAR calibration parameters (would be loaded from file in real system)
        self.camera_to_lidar = np.eye(4)  # Identity as placeholder
        self.projection_matrix = np.eye(3)  # Placeholder

        # Timers
        self.fusion_timer = self.create_timer(0.1, self.perform_fusion)

        self.get_logger().info('Basic fusion node initialized')

    def lidar_callback(self, msg):
        """
        Handle LIDAR data
        """
        self.lidar_data = msg
        self.get_logger().debug(f'Received LIDAR scan with {len(msg.ranges)} points')

    def camera_callback(self, msg):
        """
        Handle camera data
        """
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.camera_data = {
                'image': cv_image,
                'timestamp': msg.header.stamp,
                'encoding': msg.encoding
            }
            self.get_logger().debug(f'Received camera image: {cv_image.shape}')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def camera_info_callback(self, msg):
        """
        Handle camera info for intrinsic parameters
        """
        self.camera_info = msg
        # Update projection matrix
        self.projection_matrix = np.array(msg.p).reshape(3, 4)[:, :3]

    def perform_fusion(self):
        """
        Perform basic sensor fusion
        """
        if not self.lidar_data or not self.camera_data or not self.camera_info:
            return  # Need data from all sensors

        try:
            # Create 2D points from LIDAR data projected to image plane
            points_2d = self.project_lidar_to_image(self.lidar_data)

            # Process camera image for features
            image_features = self.extract_image_features(self.camera_data['image'])

            # Combine LIDAR range data with image features
            fused_data = self.combine_sensor_data(points_2d, image_features, self.lidar_data)

            # Publish fused result
            self.publish_fused_data(fused_data)

            # Publish status
            status_msg = String()
            status_msg.data = f'Fusion performed: {len(fused_data)} combined points'
            self.fusion_status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Fusion error: {e}')

    def project_lidar_to_image(self, lidar_msg):
        """
        Project LIDAR points to camera image coordinates
        """
        if not self.camera_info:
            return []

        points_2d = []
        ranges = lidar_msg.ranges
        angle_min = lidar_msg.angle_min
        angle_increment = lidar_msg.angle_increment

        for i, range_val in enumerate(ranges):
            if not (lidar_msg.range_min <= range_val <= lidar_msg.range_max):
                continue

            # Convert polar to Cartesian coordinates
            angle = angle_min + i * angle_increment
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            z = 0  # Assuming 2D LIDAR

            # Transform to camera frame (simplified - would use actual calibration in real system)
            # This is a placeholder transformation
            camera_x = x
            camera_y = y
            camera_z = z

            # Project to image coordinates
            if camera_z > 0:  # Avoid division by zero
                u = int((camera_x * self.camera_info.p[0] / camera_z) + self.camera_info.p[2])
                v = int((camera_y * self.camera_info.p[5] / camera_z) + self.camera_info.p[6])

                if (0 <= u < self.camera_info.width) and (0 <= v < self.camera_info.height):
                    points_2d.append((u, v, range_val))

        return points_2d

    def extract_image_features(self, image):
        """
        Extract simple features from camera image
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Simple edge detection
        edges = cv2.Canny(gray, 50, 150)

        # Find contours (potential objects)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        features = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small contours
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                features.append({
                    'bbox': (x, y, w, h),
                    'center': (x + w//2, y + h//2),
                    'area': cv2.contourArea(contour)
                })

        return features

    def combine_sensor_data(self, lidar_points_2d, image_features, lidar_data):
        """
        Combine LIDAR and camera data
        """
        combined_data = []

        for lidar_point in lidar_points_2d:
            u, v, range_val = lidar_point

            # Find corresponding image features
            for feature in image_features:
                x, y, w, h = feature['bbox']

                # Check if LIDAR point is near image feature
                if (x <= u <= x + w) and (y <= v <= y + h):
                    combined_data.append({
                        'image_coords': (u, v),
                        'lidar_range': range_val,
                        'feature_center': feature['center'],
                        'feature_area': feature['area'],
                        'confidence': self.calculate_confidence(u, v, range_val)
                    })

        return combined_data

    def calculate_confidence(self, u, v, range_val):
        """
        Calculate confidence based on sensor data quality
        """
        # Simple confidence calculation
        # In real system, this would be more sophisticated
        confidence = 1.0

        # Reduce confidence for longer ranges (LIDAR uncertainty increases)
        if range_val > 5.0:
            confidence *= (5.0 / range_val)

        # Reduce confidence near image edges
        if u < 50 or u > 590 or v < 50 or v > 430:  # Assuming 640x480 image
            confidence *= 0.8

        return max(0.0, min(1.0, confidence))

    def publish_fused_data(self, fused_data):
        """
        Publish fused data as a simplified format
        """
        # In a real system, you'd create a proper PointCloud2 message
        # For this example, we'll just log the data
        self.get_logger().debug(f'Fused {len(fused_data)} data points')

def main(args=None):
    rclpy.init(args=args)
    node = BasicFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down basic fusion node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Tutorial 2: Kalman Filter for Sensor Fusion

Implement a Kalman filter for fusing IMU and visual odometry data:

```python
# File: robot_control_package/robot_control_package/kalman_fusion.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, Vector3Stamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
from scipy.linalg import block_diag

class KalmanFusionNode(Node):
    """
    Kalman filter based sensor fusion for position estimation
    """

    def __init__(self):
        super().__init__('kalman_fusion_node')

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscriptions
        self.visual_odom_subscription = self.create_subscription(
            Odometry,
            'visual_odom',
            self.visual_odom_callback,
            qos
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            qos
        )

        self.ground_truth_subscription = self.create_subscription(
            Odometry,
            'ground_truth',
            self.ground_truth_callback,
            qos
        )

        # Publishers
        self.fused_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'fused_pose', qos
        )
        self.fused_odom_publisher = self.create_publisher(
            Odometry, 'fused_odom', qos
        )

        # Kalman filter initialization
        self.initialize_kalman_filter()

        # Internal state
        self.visual_odom_data = None
        self.imu_data = None
        self.ground_truth_data = None

        # Timers
        self.fusion_timer = self.create_timer(0.02, self.kalman_update)  # 50 Hz

        self.get_logger().info('Kalman fusion node initialized')

    def initialize_kalman_filter(self):
        """
        Initialize Kalman filter state and matrices
        """
        # State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
        # Position, velocity, orientation, angular velocity
        self.state_dim = 12
        self.obs_dim = 9  # [x, y, z, roll, pitch, yaw, p, q, r]

        # State vector [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
        self.x = np.zeros((self.state_dim, 1))  # State estimate
        self.P = np.eye(self.state_dim) * 1000  # Error covariance (large initial uncertainty)

        # Process noise covariance
        self.Q = np.eye(self.state_dim)
        self.Q[0:3, 0:3] *= 0.1   # Position process noise
        self.Q[3:6, 3:6] *= 1.0   # Velocity process noise
        self.Q[6:9, 6:9] *= 0.01  # Orientation process noise
        self.Q[9:12, 9:12] *= 0.1 # Angular velocity process noise

        # Measurement noise covariance
        self.R = np.eye(self.obs_dim)
        self.R[0:3, 0:3] *= 0.1   # Position measurement noise
        self.R[3:6, 3:6] *= 0.01  # Orientation measurement noise
        self.R[6:9, 6:9] *= 0.05  # Angular velocity measurement noise

        # Measurement matrix (maps state to measurement space)
        self.H = np.zeros((self.obs_dim, self.state_dim))
        self.H[0:3, 0:3] = np.eye(3)  # Position measurements
        self.H[3:6, 6:9] = np.eye(3)  # Orientation measurements
        self.H[6:9, 9:12] = np.eye(3) # Angular velocity measurements

        self.prev_time = None

    def visual_odom_callback(self, msg):
        """
        Handle visual odometry measurements
        """
        self.visual_odom_data = msg

    def imu_callback(self, msg):
        """
        Handle IMU measurements
        """
        self.imu_data = msg

    def ground_truth_callback(self, msg):
        """
        Handle ground truth for validation (in simulation)
        """
        self.ground_truth_data = msg

    def kalman_predict(self, dt):
        """
        Prediction step of Kalman filter
        """
        # State transition model (simplified)
        F = np.eye(self.state_dim)

        # Position updates based on velocity
        F[0:3, 3:6] = np.eye(3) * dt

        # Orientation updates based on angular velocity
        # This is a simplified model - in reality, rotation integration is more complex
        F[6:9, 9:12] = np.eye(3) * dt

        # Predict state
        self.x = F @ self.x

        # Predict error covariance
        self.P = F @ self.P @ F.T + self.Q

    def kalman_update(self):
        """
        Update step of Kalman filter with sensor measurements
        """
        if not self.imu_data and not self.visual_odom_data:
            return

        # Get current time for dt calculation
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        self.prev_time = current_time

        if dt <= 0:
            return

        # Prediction step
        self.kalman_predict(dt)

        # Prepare measurement vector
        z = self.get_measurement_vector()
        if z is None:
            return

        # Innovation (measurement residual)
        y = z - self.H @ self.x

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state estimate
        self.x = self.x + K @ y

        # Update error covariance
        I = np.eye(self.state_dim)
        self.P = (I - K @ self.H) @ self.P

        # Publish results
        self.publish_fused_state()

    def get_measurement_vector(self):
        """
        Create measurement vector from sensor data
        """
        z = np.zeros((self.obs_dim, 1))

        # Get position from visual odometry
        if self.visual_odom_data:
            pos = self.visual_odom_data.pose.pose.position
            z[0] = pos.x
            z[1] = pos.y
            z[2] = pos.z

        # Get orientation and angular velocity from IMU
        if self.imu_data:
            # Orientation (simplified - would need proper quaternion to Euler conversion)
            orientation = self.imu_data.orientation
            # Convert quaternion to Euler angles (simplified)
            z[3] = 0  # roll (would calculate properly)
            z[4] = 0  # pitch
            z[5] = 0  # yaw

            # Angular velocity
            ang_vel = self.imu_data.angular_velocity
            z[6] = ang_vel.x
            z[7] = ang_vel.y
            z[8] = ang_vel.z

        return z

    def publish_fused_state(self):
        """
        Publish the fused state estimate
        """
        # Create PoseWithCovarianceStamped message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Set position
        pose_msg.pose.pose.position.x = float(self.x[0])
        pose_msg.pose.pose.position.y = float(self.x[1])
        pose_msg.pose.pose.position.z = float(self.x[2])

        # Set orientation (simplified - would properly convert from Euler to quaternion)
        pose_msg.pose.pose.orientation.w = 1.0  # Identity quaternion as placeholder

        # Set covariance from error covariance matrix
        for i in range(6):  # Position and orientation covariance
            for j in range(6):
                idx = i * 6 + j
                if idx < len(pose_msg.pose.covariance):
                    if i < 3 and j < 3:  # Position covariance
                        pose_msg.pose.covariance[idx] = float(self.P[i, j])
                    else:
                        pose_msg.pose.covariance[idx] = 0.1  # Placeholder

        self.fused_pose_publisher.publish(pose_msg)

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose = pose_msg.pose

        self.fused_odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Kalman fusion node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Tutorial 3: Particle Filter for Multi-Sensor Fusion

Implement a particle filter for more complex fusion scenarios:

```python
# File: robot_control_package/robot_control_package/particle_fusion.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, PointStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.spatial.distance import cdist
import random

class ParticleFusionNode(Node):
    """
    Particle filter based sensor fusion for robust state estimation
    """

    def __init__(self):
        super().__init__('particle_fusion_node')

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscriptions
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            qos
        )

        # Publishers
        self.particle_cloud_publisher = self.create_publisher(
            PoseArray, 'particle_cloud', qos
        )
        self.best_estimate_publisher = self.create_publisher(
            PointStamped, 'best_estimate', qos
        )

        # Initialize particle filter
        self.initialize_particle_filter()

        # Internal state
        self.lidar_data = None
        self.map_occupancy = None  # Would come from SLAM or known map

        # Timers
        self.fusion_timer = self.create_timer(0.1, self.particle_filter_update)

        self.get_logger().info('Particle fusion node initialized')

    def initialize_particle_filter(self):
        """
        Initialize particle filter with random particles
        """
        self.num_particles = 1000
        self.state_dim = 3  # x, y, theta for 2D pose

        # Initialize particles randomly in a region
        self.particles = np.random.uniform(-5, 5, (self.num_particles, self.state_dim))
        self.particles[:, 2] = np.random.uniform(-np.pi, np.pi, self.num_particles)  # theta

        # Initialize weights uniformly
        self.weights = np.ones(self.num_particles) / self.num_particles

        # Motion model noise
        self.motion_noise = [0.1, 0.1, 0.05]  # x, y, theta noise

        # Sensor model parameters
        self.sensor_noise = 0.1
        self.max_range = 10.0

    def lidar_callback(self, msg):
        """
        Handle LIDAR data for sensor update
        """
        self.lidar_data = msg

    def particle_filter_update(self):
        """
        Main particle filter update cycle
        """
        if self.lidar_data is None:
            return

        # Prediction step - move particles based on motion model
        self.predict_particles()

        # Update step - reweight particles based on sensor data
        self.update_particles()

        # Resample particles
        self.resample_particles()

        # Publish results
        self.publish_results()

    def predict_particles(self):
        """
        Prediction step: move particles based on motion model
        """
        # This is a simplified motion model
        # In a real system, you'd use odometry or control inputs
        for i in range(self.num_particles):
            # Add random motion to simulate robot movement
            delta_x = np.random.normal(0, self.motion_noise[0])
            delta_y = np.random.normal(0, self.motion_noise[1])
            delta_theta = np.random.normal(0, self.motion_noise[2])

            self.particles[i, 0] += delta_x
            self.particles[i, 1] += delta_y
            self.particles[i, 2] += delta_theta

            # Normalize angle
            self.particles[i, 2] = self.normalize_angle(self.particles[i, 2])

    def update_particles(self):
        """
        Update step: reweight particles based on sensor likelihood
        """
        if self.lidar_data is None:
            return

        # Convert LIDAR ranges to Cartesian points in sensor frame
        angles = np.array([
            self.lidar_data.angle_min + i * self.lidar_data.angle_increment
            for i in range(len(self.lidar_data.ranges))
        ])

        # Only use valid ranges (not inf or nan)
        valid_ranges = np.array([
            r if 0 < r < self.lidar_data.range_max else self.max_range
            for r in self.lidar_data.ranges
        ])

        for i in range(self.num_particles):
            particle_weight = self.calculate_particle_weight(
                self.particles[i], angles, valid_ranges
            )
            self.weights[i] = particle_weight

        # Normalize weights
        if np.sum(self.weights) > 0:
            self.weights /= np.sum(self.weights)
        else:
            # If all weights are zero, reset to uniform
            self.weights = np.ones(self.num_particles) / self.num_particles

    def calculate_particle_weight(self, particle, angles, ranges):
        """
        Calculate likelihood of particle given sensor data
        """
        x, y, theta = particle

        # Transform LIDAR beams to world coordinates for this particle
        predicted_points = []
        for angle, range_val in zip(angles, ranges):
            if range_val < self.max_range:
                # Calculate where this beam should intersect in world frame
                beam_angle = theta + angle
                pred_x = x + range_val * np.cos(beam_angle)
                pred_y = y + range_val * np.sin(beam_angle)
                predicted_points.append([pred_x, pred_y])

        if not predicted_points:
            return 1.0

        predicted_points = np.array(predicted_points)

        # In a real system, you'd compare with a known map
        # For this example, we'll use a simple likelihood model
        # This is a simplified version - real implementation would be more complex

        # Calculate weight based on how well the predicted measurements match
        # This is a placeholder implementation
        weight = 1.0

        # Simple model: prefer particles that predict reasonable range values
        for range_val in ranges:
            if range_val < 0.2:  # Very close readings might indicate collision
                weight *= 0.1
            elif range_val > self.max_range * 0.9:  # Very far readings are less informative
                weight *= 0.8

        return weight

    def resample_particles(self):
        """
        Resample particles based on their weights
        """
        # Systematic resampling
        indices = self.systematic_resample()

        # Resample particles
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles

    def systematic_resample(self):
        """
        Systematic resampling algorithm
        """
        indices = np.zeros(self.num_particles, dtype=int)
        cumulative_sum = np.cumsum(self.weights)

        # Generate random starting point
        random_start = np.random.random() / self.num_particles

        # Generate equally spaced points
        points = (np.arange(self.num_particles) + random_start) / self.num_particles

        # Find corresponding particles
        i, j = 0, 0
        while i < self.num_particles:
            if points[i] < cumulative_sum[j]:
                indices[i] = j
                i += 1
            else:
                j += 1

        return indices

    def normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi] range
        """
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def publish_results(self):
        """
        Publish particle cloud and best estimate
        """
        # Publish particle cloud
        particle_msg = PoseArray()
        particle_msg.header.stamp = self.get_clock().now().to_msg()
        particle_msg.header.frame_id = 'map'

        for i in range(self.num_particles):
            pose = Pose()
            pose.position.x = float(self.particles[i, 0])
            pose.position.y = float(self.particles[i, 1])
            pose.position.z = 0.0

            # Convert orientation to quaternion (simplified)
            theta = self.particles[i, 2]
            pose.orientation.z = np.sin(theta / 2)
            pose.orientation.w = np.cos(theta / 2)

            particle_msg.poses.append(pose)

        self.particle_cloud_publisher.publish(particle_msg)

        # Publish best estimate (mean of particles)
        mean_state = np.average(self.particles, axis=0, weights=self.weights)

        best_estimate_msg = PointStamped()
        best_estimate_msg.header.stamp = self.get_clock().now().to_msg()
        best_estimate_msg.header.frame_id = 'map'
        best_estimate_msg.point.x = float(mean_state[0])
        best_estimate_msg.point.y = float(mean_state[1])
        best_estimate_msg.point.z = 0.0

        self.best_estimate_publisher.publish(best_estimate_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ParticleFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down particle fusion node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Code Examples with Explanations

### Example 1: Advanced Multi-Sensor Fusion Node

```python
# File: robot_control_package/robot_control_package/advanced_fusion.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from collections import deque
import threading

class AdvancedFusionNode(Node):
    """
    Advanced multi-sensor fusion node with multiple sensor types
    """

    def __init__(self):
        super().__init__('advanced_fusion_node')

        # QoS profiles
        sensor_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Subscriptions for multiple sensor types
        self.lidar_subscription = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, sensor_qos
        )

        self.imu_subscription = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, sensor_qos
        )

        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, reliable_qos
        )

        self.gps_subscription = self.create_subscription(
            NavSatFix, 'gps/fix', self.gps_callback, reliable_qos
        )

        # Publishers
        self.fused_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'fused_pose', reliable_qos
        )

        self.fused_cloud_publisher = self.create_publisher(
            PointCloud2, 'fused_cloud', reliable_qos
        )

        self.status_publisher = self.create_publisher(
            String, 'fusion_status', reliable_qos
        )

        # Data buffers for temporal fusion
        self.lidar_buffer = deque(maxlen=5)
        self.imu_buffer = deque(maxlen=10)
        self.odom_buffer = deque(maxlen=10)
        self.gps_buffer = deque(maxlen=5)

        # Fusion state
        self.fusion_state = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),  # quaternion
            'velocity': np.array([0.0, 0.0, 0.0]),
            'confidence': 0.0
        }

        # Sensor health monitoring
        self.sensor_health = {
            'lidar': True,
            'imu': True,
            'odom': True,
            'gps': True
        }

        # Timers
        self.fusion_timer = self.create_timer(0.05, self.perform_advanced_fusion)  # 20 Hz
        self.health_timer = self.create_timer(1.0, self.check_sensor_health)

        # Threading for heavy computations
        self.fusion_lock = threading.Lock()

        self.get_logger().info('Advanced fusion node initialized')

    def lidar_callback(self, msg):
        """
        Handle LIDAR data with timestamp buffering
        """
        with self.fusion_lock:
            self.lidar_buffer.append({
                'data': msg,
                'timestamp': self.get_clock().now()
            })

    def imu_callback(self, msg):
        """
        Handle IMU data with proper orientation handling
        """
        with self.fusion_lock:
            # Extract orientation quaternion
            orientation = np.array([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ])

            # Extract angular velocity
            angular_vel = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])

            self.imu_buffer.append({
                'orientation': orientation,
                'angular_velocity': angular_vel,
                'linear_acceleration': np.array([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ]),
                'timestamp': self.get_clock().now()
            })

    def odom_callback(self, msg):
        """
        Handle odometry data
        """
        with self.fusion_lock:
            self.odom_buffer.append({
                'pose': np.array([
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z
                ]),
                'orientation': np.array([
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                ]),
                'twist': np.array([
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z
                ]),
                'timestamp': self.get_clock().now()
            })

    def gps_callback(self, msg):
        """
        Handle GPS data
        """
        with self.fusion_lock:
            # Convert lat/lon to local coordinates (simplified)
            # In real system, you'd use proper coordinate transformation
            local_x = (msg.longitude - self.get_parameter_or_set('reference_lon', 0.0).value) * 111320  # approx
            local_y = (msg.latitude - self.get_parameter_or_set('reference_lat', 0.0).value) * 111320  # approx

            self.gps_buffer.append({
                'position': np.array([local_x, local_y, msg.altitude]),
                'position_covariance': np.array(msg.position_covariance).reshape(3, 3),
                'timestamp': self.get_clock().now()
            })

    def perform_advanced_fusion(self):
        """
        Perform advanced sensor fusion with multiple algorithms
        """
        with self.fusion_lock:
            if not self.all_sensors_available():
                self.publish_status("Insufficient sensor data for fusion")
                return

            try:
                # 1. Fuse position data (GPS + Odometry)
                fused_position = self.fuse_position_data()

                # 2. Fuse orientation data (IMU + Odometry)
                fused_orientation = self.fuse_orientation_data()

                # 3. Fuse velocity data (Odometry + IMU)
                fused_velocity = self.fuse_velocity_data()

                # 4. Calculate overall confidence
                confidence = self.calculate_fusion_confidence()

                # Update fusion state
                self.fusion_state['position'] = fused_position
                self.fusion_state['orientation'] = fused_orientation
                self.fusion_state['velocity'] = fused_velocity
                self.fusion_state['confidence'] = confidence

                # Publish results
                self.publish_fused_results()

            except Exception as e:
                self.get_logger().error(f'Fusion error: {e}')
                self.publish_status(f"Fusion failed: {str(e)}")

    def fuse_position_data(self):
        """
        Fuse position data from GPS and odometry
        """
        if not (self.gps_buffer and self.odom_buffer):
            # Fallback to odometry if GPS unavailable
            if self.odom_buffer:
                return self.odom_buffer[-1]['pose']
            else:
                return self.fusion_state['position']

        # Get most recent data
        gps_data = self.gps_buffer[-1]
        odom_data = self.odom_buffer[-1]

        # Simple weighted fusion (in real system, use Kalman filter)
        gps_weight = 0.3  # Lower weight for GPS due to lower frequency
        odom_weight = 0.7  # Higher weight for odometry

        fused_pos = (
            gps_weight * gps_data['position'] +
            odom_weight * odom_data['pose']
        )

        return fused_pos

    def fuse_orientation_data(self):
        """
        Fuse orientation data from IMU and odometry
        """
        if not (self.imu_buffer and self.odom_buffer):
            # Fallback to current orientation
            return self.fusion_state['orientation']

        # Get most recent data
        imu_data = self.imu_buffer[-1]
        odom_data = self.odom_buffer[-1]

        # For now, use IMU orientation as it's typically more accurate
        # In real system, use proper sensor fusion
        return imu_data['orientation']

    def fuse_velocity_data(self):
        """
        Fuse velocity data from odometry and IMU
        """
        if not self.odom_buffer:
            return self.fusion_state['velocity']

        # Use odometry velocity as primary source
        return self.odom_buffer[-1]['twist']

    def calculate_fusion_confidence(self):
        """
        Calculate overall fusion confidence based on sensor availability and quality
        """
        confidence = 0.0

        # Add confidence based on sensor availability
        if self.gps_buffer:
            confidence += 0.25  # GPS available
        if self.imu_buffer:
            confidence += 0.30  # IMU available
        if self.odom_buffer:
            confidence += 0.30  # Odometry available
        if self.lidar_buffer:
            confidence += 0.15  # LIDAR available

        # Adjust for sensor health
        for sensor, is_healthy in self.sensor_health.items():
            if not is_healthy:
                confidence *= 0.5  # Reduce confidence if sensor is unhealthy

        return min(1.0, confidence)

    def all_sensors_available(self):
        """
        Check if all critical sensors are available
        """
        return bool(self.odom_buffer and self.imu_buffer)

    def check_sensor_health(self):
        """
        Check sensor health and update status
        """
        current_time = self.get_clock().now()

        # Check if sensors are providing recent data
        if self.lidar_buffer:
            time_since_lidar = (current_time - self.lidar_buffer[-1]['timestamp']).nanoseconds / 1e9
            self.sensor_health['lidar'] = time_since_lidar < 1.0  # 1 second timeout

        if self.imu_buffer:
            time_since_imu = (current_time - self.imu_buffer[-1]['timestamp']).nanoseconds / 1e9
            self.sensor_health['imu'] = time_since_imu < 0.1  # 100ms timeout

        if self.odom_buffer:
            time_since_odom = (current_time - self.odom_buffer[-1]['timestamp']).nanoseconds / 1e9
            self.sensor_health['odom'] = time_since_odom < 0.1  # 100ms timeout

        if self.gps_buffer:
            time_since_gps = (current_time - self.gps_buffer[-1]['timestamp']).nanoseconds / 1e9
            self.sensor_health['gps'] = time_since_gps < 5.0  # 5 seconds timeout

        # Log sensor status
        healthy_sensors = sum(1 for healthy in self.sensor_health.values() if healthy)
        total_sensors = len(self.sensor_health)
        self.publish_status(f"Sensor health: {healthy_sensors}/{total_sensors} sensors healthy")

    def publish_fused_results(self):
        """
        Publish fused results
        """
        # Publish fused pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.pose.position.x = float(self.fusion_state['position'][0])
        pose_msg.pose.pose.position.y = float(self.fusion_state['position'][1])
        pose_msg.pose.pose.position.z = float(self.fusion_state['position'][2])

        pose_msg.pose.pose.orientation.x = float(self.fusion_state['orientation'][0])
        pose_msg.pose.pose.orientation.y = float(self.fusion_state['orientation'][1])
        pose_msg.pose.pose.orientation.z = float(self.fusion_state['orientation'][2])
        pose_msg.pose.pose.orientation.w = float(self.fusion_state['orientation'][3])

        # Set covariance based on confidence
        confidence = self.fusion_state['confidence']
        cov_scale = 1.0 - confidence  # Lower covariance for higher confidence
        for i in range(36):
            pose_msg.pose.covariance[i] = cov_scale * 0.1

        self.fused_pose_publisher.publish(pose_msg)

    def publish_status(self, status_msg):
        """
        Publish fusion status
        """
        msg = String()
        msg.data = status_msg
        self.status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down advanced fusion node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Sensor Validation and Fault Detection

```python
# File: robot_control_package/robot_control_package/sensor_validation.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from std_msgs.msg import String, Bool
from collections import deque
import numpy as np
import statistics

class SensorValidationNode(Node):
    """
    Validate sensor data and detect faults
    """

    def __init__(self):
        super().__init__('sensor_validation_node')

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscriptions
        self.lidar_subscription = self.create_subscription(
            LaserScan, 'scan', self.lidar_validation_callback, qos
        )

        self.imu_subscription = self.create_subscription(
            Imu, 'imu/data', self.imu_validation_callback, qos
        )

        self.gps_subscription = self.create_subscription(
            NavSatFix, 'gps/fix', self.gps_validation_callback, qos
        )

        # Publishers
        self.lidar_health_publisher = self.create_publisher(Bool, 'lidar_health', qos)
        self.imu_health_publisher = self.create_publisher(Bool, 'imu_health', qos)
        self.gps_health_publisher = self.create_publisher(Bool, 'gps_health', qos)
        self.validation_report_publisher = self.create_publisher(String, 'validation_report', qos)

        # Data validation buffers
        self.lidar_ranges_buffer = deque(maxlen=10)
        self.imu_orientation_buffer = deque(maxlen=10)
        self.gps_position_buffer = deque(maxlen=10)

        # Validation parameters
        self.lidar_range_threshold = 30.0  # Maximum valid range
        self.imu_orientation_threshold = 0.5  # Maximum orientation change per second
        self.gps_variance_threshold = 10.0  # Maximum position variance

        # Timers
        self.validation_timer = self.create_timer(0.5, self.perform_validation)

        self.get_logger().info('Sensor validation node initialized')

    def lidar_validation_callback(self, msg):
        """
        Validate LIDAR data
        """
        # Check for valid ranges
        valid_ranges = [r for r in msg.ranges if 0 < r < msg.range_max]

        if not valid_ranges:
            self.get_logger().warn('LIDAR: No valid ranges detected')
            return

        # Store for statistical validation
        self.lidar_ranges_buffer.append(valid_ranges)

    def imu_validation_callback(self, msg):
        """
        Validate IMU data
        """
        # Check for NaN or infinity values
        if (np.isnan(msg.orientation.x) or np.isnan(msg.orientation.y) or
            np.isnan(msg.orientation.z) or np.isnan(msg.orientation.w)):
            self.get_logger().warn('IMU: NaN orientation detected')
            return

        # Store for validation
        orientation = np.array([
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        ])
        self.imu_orientation_buffer.append(orientation)

    def gps_validation_callback(self, msg):
        """
        Validate GPS data
        """
        # Check for valid status
        if msg.status.status != 0:  # STATUS_FIX
            self.get_logger().warn(f'GPS: Invalid status {msg.status.status}')
            return

        # Check for valid coordinates
        if np.isnan(msg.latitude) or np.isnan(msg.longitude):
            self.get_logger().warn('GPS: Invalid coordinates')
            return

        # Store for validation
        position = np.array([msg.latitude, msg.longitude, msg.altitude])
        self.gps_position_buffer.append(position)

    def perform_validation(self):
        """
        Perform statistical validation of sensor data
        """
        lidar_healthy = self.validate_lidar_data()
        imu_healthy = self.validate_imu_data()
        gps_healthy = self.validate_gps_data()

        # Publish health status
        lidar_health_msg = Bool()
        lidar_health_msg.data = lidar_healthy
        self.lidar_health_publisher.publish(lidar_health_msg)

        imu_health_msg = Bool()
        imu_health_msg.data = imu_healthy
        self.imu_health_publisher.publish(imu_health_msg)

        gps_health_msg = Bool()
        gps_health_msg.data = gps_healthy
        self.gps_health_publisher.publish(gps_health_msg)

        # Publish validation report
        report_msg = String()
        report_msg.data = f"LIDAR:{'OK' if lidar_healthy else 'FAIL'}, " \
                         f"IMU:{'OK' if imu_healthy else 'FAIL'}, " \
                         f"GPS:{'OK' if gps_healthy else 'FAIL'}"
        self.validation_report_publisher.publish(report_msg)

    def validate_lidar_data(self):
        """
        Validate LIDAR data using statistical methods
        """
        if len(self.lidar_ranges_buffer) < 2:
            return True  # Not enough data to validate

        # Check for sudden changes in average range
        avg_ranges = [statistics.mean(ranges) for ranges in self.lidar_ranges_buffer]
        if len(avg_ranges) >= 2:
            recent_change = abs(avg_ranges[-1] - avg_ranges[-2])
            if recent_change > 5.0:  # Threshold for significant change
                self.get_logger().warn(f'LIDAR: Sudden range change detected: {recent_change:.2f}m')

        # Check for consistent extreme values
        all_ranges = []
        for ranges in self.lidar_ranges_buffer:
            all_ranges.extend(ranges)

        if all_ranges:
            range_variance = statistics.variance(all_ranges) if len(all_ranges) > 1 else 0
            if range_variance > 100:  # High variance might indicate issues
                self.get_logger().warn(f'LIDAR: High range variance detected: {range_variance:.2f}')

        return True  # LIDAR is considered healthy if it passes basic checks

    def validate_imu_data(self):
        """
        Validate IMU data for consistency
        """
        if len(self.imu_orientation_buffer) < 2:
            return True

        # Check orientation change rate
        prev_quat = self.imu_orientation_buffer[-2]
        curr_quat = self.imu_orientation_buffer[-1]

        # Calculate angular difference
        dot_product = np.dot(prev_quat, curr_quat)
        angle_diff = 2 * np.arccos(min(abs(dot_product), 1.0))  # Clamp to avoid numerical errors

        # Assuming 100Hz IMU, this is change per 10ms
        if angle_diff > self.imu_orientation_threshold * 0.01:  # Scale by time
            self.get_logger().warn(f'IMU: Rapid orientation change: {angle_diff:.4f} rad')

        return True

    def validate_gps_data(self):
        """
        Validate GPS data for accuracy and consistency
        """
        if len(self.gps_position_buffer) < 3:
            return True

        # Calculate position variance
        positions = np.array(self.gps_position_buffer)
        position_variance = np.var(positions, axis=0)
        max_variance = np.max(position_variance)

        if max_variance > self.gps_variance_threshold:
            self.get_logger().warn(f'GPS: High position variance: {max_variance:.4f}')

        return True

def main(args=None):
    rclpy.init(args=args)
    node = SensorValidationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down sensor validation node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-On Exercises: Complex Fusion Implementation

### Exercise 1: Create a Robust Fusion System

Create a comprehensive sensor fusion system that handles sensor failures gracefully:

```python
# File: robot_control_package/robot_control_package/robust_fusion.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import numpy as np
from collections import defaultdict, deque
import threading
import time

class RobustFusionNode(Node):
    """
    Robust sensor fusion system that handles sensor failures gracefully
    """

    def __init__(self):
        super().__init__('robust_fusion_node')

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscriptions
        self.lidar_subscription = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, qos
        )

        self.imu_subscription = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, qos
        )

        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos
        )

        # Publishers
        self.fused_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'fused_pose', qos
        )

        self.fallback_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'fallback_pose', qos
        )

        self.system_status_publisher = self.create_publisher(String, 'system_status', qos)

        # Data storage with timestamps
        self.sensor_data = defaultdict(lambda: deque(maxlen=10))
        self.sensor_last_update = defaultdict(float)
        self.sensor_health = defaultdict(bool)

        # Fallback strategies
        self.fallback_modes = {
            'full': ['lidar', 'imu', 'odom'],  # All sensors available
            'reduced': ['imu', 'odom'],        # LIDAR failed
            'minimal': ['odom'],               # Only odometry available
            'dead_reckoning': []               # No sensors, use prediction
        }

        # Current fusion mode
        self.current_mode = 'full'

        # Lock for thread safety
        self.fusion_lock = threading.Lock()

        # Timers
        self.fusion_timer = self.create_timer(0.1, self.robust_fusion_update)
        self.health_check_timer = self.create_timer(1.0, self.check_sensor_health)

        self.get_logger().info('Robust fusion node initialized')

    def lidar_callback(self, msg):
        """
        Handle LIDAR data with timestamp tracking
        """
        with self.fusion_lock:
            self.sensor_data['lidar'].append({
                'data': msg,
                'timestamp': time.time()
            })
            self.sensor_last_update['lidar'] = time.time()

    def imu_callback(self, msg):
        """
        Handle IMU data
        """
        with self.fusion_lock:
            self.sensor_data['imu'].append({
                'data': msg,
                'timestamp': time.time()
            })
            self.sensor_last_update['imu'] = time.time()

    def odom_callback(self, msg):
        """
        Handle odometry data
        """
        with self.fusion_lock:
            self.sensor_data['odom'].append({
                'data': msg,
                'timestamp': time.time()
            })
            self.sensor_last_update['odom'] = time.time()

    def check_sensor_health(self):
        """
        Check sensor health and update fusion mode
        """
        current_time = time.time()
        timeout_threshold = 2.0  # seconds

        with self.fusion_lock:
            # Update sensor health based on update times
            for sensor_name in ['lidar', 'imu', 'odom']:
                last_update = self.sensor_last_update[sensor_name]
                self.sensor_health[sensor_name] = (
                    current_time - last_update < timeout_threshold
                )

            # Determine current fusion mode based on sensor health
            available_sensors = [s for s, healthy in self.sensor_health.items() if healthy]

            if set(available_sensors) == set(['lidar', 'imu', 'odom']):
                self.current_mode = 'full'
            elif set(available_sensors) == set(['imu', 'odom']):
                self.current_mode = 'reduced'
            elif set(available_sensors) == set(['odom']):
                self.current_mode = 'minimal'
            else:
                self.current_mode = 'dead_reckoning'

        # Log mode changes
        self.get_logger().info(f'Fusion mode: {self.current_mode}')

    def robust_fusion_update(self):
        """
        Perform fusion based on current system health
        """
        with self.fusion_lock:
            try:
                if self.current_mode == 'full':
                    pose = self.full_fusion()
                elif self.current_mode == 'reduced':
                    pose = self.reduced_fusion()
                elif self.current_mode == 'minimal':
                    pose = self.minimal_fusion()
                else:  # dead_reckoning
                    pose = self.dead_reckoning_fusion()

                # Publish appropriate result
                if self.current_mode in ['full', 'reduced']:
                    self.fused_pose_publisher.publish(pose)
                else:
                    self.fallback_pose_publisher.publish(pose)

                # Publish system status
                status_msg = String()
                status_msg.data = f"Mode: {self.current_mode}, Sensors: {dict(self.sensor_health)}"
                self.system_status_publisher.publish(status_msg)

            except Exception as e:
                self.get_logger().error(f'Fusion update error: {e}')

    def full_fusion(self):
        """
        Perform full sensor fusion when all sensors are available
        """
        # Get most recent data from all sensors
        lidar_data = self.sensor_data['lidar'][-1]['data'] if self.sensor_data['lidar'] else None
        imu_data = self.sensor_data['imu'][-1]['data'] if self.sensor_data['imu'] else None
        odom_data = self.sensor_data['odom'][-1]['data'] if self.sensor_data['odom'] else None

        # Create fused pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Use odometry as primary position source
        if odom_data:
            pose_msg.pose.pose = odom_data.pose.pose
            # Improve covariance based on additional sensor data
            pose_msg.pose.covariance = list(np.array(odom_data.pose.covariance) * 0.5)  # Better accuracy

        return pose_msg

    def reduced_fusion(self):
        """
        Fusion with IMU and odometry (LIDAR failed)
        """
        # Get available data
        imu_data = self.sensor_data['imu'][-1]['data'] if self.sensor_data['imu'] else None
        odom_data = self.sensor_data['odom'][-1]['data'] if self.sensor_data['odom'] else None

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Use odometry for position, IMU for orientation refinement
        if odom_data:
            pose_msg.pose.pose = odom_data.pose.pose
            # Increase uncertainty due to missing LIDAR
            pose_msg.pose.covariance = list(np.array(odom_data.pose.covariance) * 2.0)

        return pose_msg

    def minimal_fusion(self):
        """
        Fusion with only odometry (IMU and LIDAR failed)
        """
        odom_data = self.sensor_data['odom'][-1]['data'] if self.sensor_data['odom'] else None

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        if odom_data:
            pose_msg.pose.pose = odom_data.pose.pose
            # Significantly increase uncertainty
            pose_msg.pose.covariance = list(np.array(odom_data.pose.covariance) * 5.0)

        return pose_msg

    def dead_reckoning_fusion(self):
        """
        Dead reckoning when no sensors are available
        """
        # Use the last known pose and predict based on last known velocity
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # In a real system, you'd predict position based on last known velocity
        # For this example, we'll return a zero pose with high uncertainty
        pose_msg.pose.covariance = [100.0] * 36  # Very high uncertainty

        return pose_msg

def main(args=None):
    rclpy.init(args=args)
    node = RobustFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down robust fusion node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Exercise 2: Multi-Sensor Data Association

Implement data association for multi-sensor fusion:

```python
# File: robot_control_package/robot_control_package/data_association.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PointStamped, PoseArray
from std_msgs.msg import String
import numpy as np
from scipy.spatial.distance import cdist
from sklearn.cluster import DBSCAN
import threading

class DataAssociationNode(Node):
    """
    Perform data association between multiple sensors
    """

    def __init__(self):
        super().__init__('data_association_node')

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscriptions
        self.lidar_subscription = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, qos
        )

        self.vision_subscription = self.create_subscription(
            PointCloud2, 'vision_pointcloud', self.vision_callback, qos
        )

        # Publishers
        self.associated_objects_publisher = self.create_publisher(
            PoseArray, 'associated_objects', qos
        )

        self.association_report_publisher = self.create_publisher(
            String, 'association_report', qos
        )

        # Data storage
        self.lidar_points = []
        self.vision_points = []
        self.association_threshold = 0.5  # meters

        # Lock for thread safety
        self.association_lock = threading.Lock()

        # Timers
        self.association_timer = self.create_timer(0.5, self.perform_association)

        self.get_logger().info('Data association node initialized')

    def lidar_callback(self, msg):
        """
        Process LIDAR scan and extract points
        """
        with self.association_lock:
            # Convert LIDAR scan to Cartesian points
            self.lidar_points = []
            for i, range_val in enumerate(msg.ranges):
                if msg.range_min <= range_val <= msg.range_max:
                    angle = msg.angle_min + i * msg.angle_increment
                    x = range_val * np.cos(angle)
                    y = range_val * np.sin(angle)
                    self.lidar_points.append([x, y])

    def vision_callback(self, msg):
        """
        Process vision point cloud
        """
        # Note: In a real system, you'd properly parse the PointCloud2 message
        # For this example, we'll simulate some vision points
        with self.association_lock:
            # Simulate vision points (in real system, parse PointCloud2)
            self.vision_points = [
                [1.0, 0.5], [2.0, -0.5], [3.0, 1.0]  # Example points
            ]

    def perform_association(self):
        """
        Perform data association between LIDAR and vision points
        """
        with self.association_lock:
            if not self.lidar_points or not self.vision_points:
                return

            try:
                # Convert to numpy arrays
                lidar_array = np.array(self.lidar_points)
                vision_array = np.array(self.vision_points)

                # Calculate distance matrix between all points
                distance_matrix = cdist(lidar_array, vision_array)

                # Find associations within threshold
                associations = []
                for lidar_idx in range(len(lidar_array)):
                    min_dist_idx = np.argmin(distance_matrix[lidar_idx])
                    min_dist = distance_matrix[lidar_idx, min_dist_idx]

                    if min_dist <= self.association_threshold:
                        associations.append((lidar_idx, min_dist_idx, min_dist))

                # Publish associated objects
                self.publish_associated_objects(associations, lidar_array, vision_array)

                # Publish association report
                report_msg = String()
                report_msg.data = f"Found {len(associations)} associations out of {len(lidar_array)} LIDAR points"
                self.association_report_publisher.publish(report_msg)

            except Exception as e:
                self.get_logger().error(f'Association error: {e}')

    def publish_associated_objects(self, associations, lidar_array, vision_array):
        """
        Publish associated objects as PoseArray
        """
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'map'

        for lidar_idx, vision_idx, distance in associations:
            # Calculate fused position (simple average for this example)
            fused_x = (lidar_array[lidar_idx, 0] + vision_array[vision_idx, 0]) / 2
            fused_y = (lidar_array[lidar_idx, 1] + vision_array[vision_idx, 1]) / 2

            pose = PointStamped()
            pose.header = pose_array.header
            pose.point.x = fused_x
            pose.point.y = fused_y
            pose.point.z = 0.0  # Assuming 2D

            # Add to pose array as well
            pose_msg = Pose()
            pose_msg.position.x = fused_x
            pose_msg.position.y = fused_y
            pose_msg.position.z = 0.0
            pose_array.poses.append(pose_msg)

        self.associated_objects_publisher.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = DataAssociationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down data association node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Pitfalls and Solutions

### Pitfall 1: Synchronization Issues
**Problem**: Sensor data arrives at different times, causing temporal misalignment.

**Solution**:
- Implement proper timestamp management
- Use message filters for synchronization
- Apply temporal interpolation when needed

```python
def synchronize_sensor_data(self, lidar_msg, imu_msg, odom_msg):
    """
    Synchronize sensor data using timestamps
    """
    # Calculate time differences
    lidar_time = lidar_msg.header.stamp.sec + lidar_msg.header.stamp.nanosec / 1e9
    imu_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec / 1e9
    odom_time = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec / 1e9

    # Find the closest timestamps
    target_time = (lidar_time + imu_time + odom_time) / 3

    # Interpolate if needed (simplified example)
    return {
        'timestamp': target_time,
        'lidar': self.interpolate_lidar(lidar_msg, target_time),
        'imu': self.interpolate_imu(imu_msg, target_time),
        'odom': self.interpolate_odom(odom_msg, target_time)
    }
```

### Pitfall 2: Coordinate System Mismatches
**Problem**: Different sensors use different coordinate frames, causing incorrect fusion.

**Solution**:
- Use TF2 for coordinate transformations
- Maintain proper frame conventions
- Validate transformations regularly

### Pitfall 3: Sensor Failure Handling
**Problem**: System fails when individual sensors fail.

**Solution**:
- Implement graceful degradation
- Use redundant sensors where possible
- Monitor sensor health continuously

## Review Questions

1. What are the main types of sensor fusion and when would you use each?
2. Explain the differences between Kalman filters and particle filters for sensor fusion.
3. How do you handle sensor synchronization in a multi-sensor system?
4. What are the key considerations for validating sensor data quality?
5. Describe how to implement graceful degradation when sensors fail.

## Project Assignment: Comprehensive Multi-Sensor Fusion System

Create a complete multi-sensor fusion system that includes:
1. Integration of at least 4 different sensor types (LIDAR, camera, IMU, GPS/odometry)
2. Multiple fusion algorithms (Kalman filter, particle filter, and rule-based)
3. Sensor validation and health monitoring
4. Data association between different sensor modalities
5. Robust handling of sensor failures and degraded modes
6. Performance monitoring and logging

Your system should:
- Demonstrate real-time fusion capabilities
- Include proper error handling and recovery
- Show graceful degradation when sensors fail
- Provide comprehensive logging and visualization
- Be optimized for computational efficiency

## Further Resources

- [Probabilistic Robotics by Thrun, Burgard, and Fox](https://mitpress.mit.edu/books/probabilistic-robotics)
- [Sensor Fusion Tutorial](https://www.mathworks.com/help/fusion/ug/sensor-fusion-in-navigation.html)
- [Kalman Filter Implementation](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)
- [Multi-Sensor Fusion in ROS](https://navigation.ros.org/setup_guides/fusion/index.html)
- [SLAM and Sensor Fusion](https://github.com/SMRT-AIST/interactive_slam)

:::info
Sensor fusion is critical for robust robotics applications. Always validate your fusion algorithms with real sensor data and consider the computational requirements for real-time operation.
:::