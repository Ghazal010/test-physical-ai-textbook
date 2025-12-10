---
title: Week 1-2 - Sensors and Perception
description: Understanding different sensor systems and perception in robotics
sidebar_position: 2
---

# Week 1-2: Sensors and Perception

## Learning Objectives
- Identify various sensor types and their applications
- Understand how robots perceive their environment
- Learn about sensor data processing and interpretation
- Implement basic perception algorithms

## Prerequisites Check
- Understanding of basic Python programming
- Knowledge of arrays and data structures
- Basic understanding of coordinate systems

## Theoretical Concepts: Sensor Systems in Robotics

### Overview of Robot Sensors

Robots rely on various sensors to perceive and interact with their environment. These sensors provide the "senses" that allow robots to navigate, manipulate objects, and respond to their surroundings. Understanding sensor systems is crucial for developing effective robotic applications.

### LIDAR Sensors

LIDAR (Light Detection and Ranging) sensors emit laser pulses and measure the time it takes for the light to return after reflecting off objects. This creates accurate distance measurements that form the basis for 2D or 3D maps of the environment.

**Applications**:
- Navigation and mapping
- Obstacle detection
- Localization
- 3D reconstruction

**Advantages**:
- High accuracy in distance measurements
- Works in various lighting conditions
- Provides dense spatial information

**Limitations**:
- Expensive compared to other sensors
- Can be affected by reflective surfaces
- Limited ability to identify object types

### Camera Systems

Cameras provide rich visual information that enables object recognition, color detection, and complex scene understanding. Modern robots often use multiple cameras for stereo vision, wide-angle coverage, or specialized tasks.

**Types of Camera Systems**:
- **RGB Cameras**: Provide color images for object recognition
- **Depth Cameras**: Provide distance information for each pixel
- **Stereo Cameras**: Use two cameras to calculate depth through triangulation
- **Thermal Cameras**: Detect heat signatures for special applications

### Inertial Measurement Units (IMUs)

IMUs combine accelerometers, gyroscopes, and sometimes magnetometers to measure a robot's orientation, acceleration, and angular velocity. These sensors are essential for maintaining balance and understanding motion.

**Components**:
- **Accelerometer**: Measures linear acceleration
- **Gyroscope**: Measures angular velocity
- **Magnetometer**: Measures magnetic field (compass function)

### Force and Torque Sensors

These sensors measure the forces and torques applied to robotic systems, crucial for manipulation tasks and ensuring safe interaction with the environment.

### Other Specialized Sensors

- **Ultrasonic Sensors**: Use sound waves for distance measurement (short range)
- **Infrared Sensors**: Detect proximity and distance (short range)
- **GPS**: Provides global positioning information
- **Encoders**: Measure joint angles and wheel rotations

## Step-by-Step Tutorials: Sensor Integration

### Tutorial 1: Multi-Sensor Data Collection

Let's create a system that collects and processes data from multiple sensors:

```python
import numpy as np
import time
from dataclasses import dataclass
from typing import Dict, List, Optional
import matplotlib.pyplot as plt

@dataclass
class LIDARData:
    """Data structure for LIDAR sensor readings"""
    distances: List[float]
    angles: List[float]
    timestamp: float

@dataclass
class CameraData:
    """Data structure for camera sensor readings"""
    image: np.ndarray
    width: int
    height: int
    timestamp: float

@dataclass
class IMUData:
    """Data structure for IMU sensor readings"""
    orientation: tuple  # (roll, pitch, yaw)
    acceleration: tuple  # (x, y, z)
    angular_velocity: tuple  # (x, y, z)
    timestamp: float

class MultiSensorSystem:
    """
    A system to simulate and integrate data from multiple sensors
    """
    def __init__(self):
        self.lidar = self._initialize_lidar()
        self.camera = self._initialize_camera()
        self.imu = self._initialize_imu()
        self.sensor_data_log = []

    def _initialize_lidar(self):
        """Initialize LIDAR parameters"""
        return {
            'min_range': 0.1,
            'max_range': 30.0,
            'fov': 360,
            'resolution': 1.0,
            'num_beams': 360
        }

    def _initialize_camera(self):
        """Initialize camera parameters"""
        return {
            'width': 640,
            'height': 480,
            'fov': 60,
            'fps': 30
        }

    def _initialize_imu(self):
        """Initialize IMU parameters"""
        return {
            'acceleration_noise': 0.01,
            'gyro_noise': 0.001,
            'compass_accuracy': 1.0
        }

    def collect_sensor_data(self) -> Dict[str, any]:
        """
        Collect data from all sensors simultaneously
        """
        timestamp = time.time()

        # Collect LIDAR data
        lidar_distances = self._simulate_lidar_scan()
        lidar_data = LIDARData(
            distances=lidar_distances,
            angles=[i * self.lidar['resolution'] for i in range(len(lidar_distances))],
            timestamp=timestamp
        )

        # Collect camera data
        camera_image = self._simulate_camera_frame()
        camera_data = CameraData(
            image=camera_image,
            width=self.camera['width'],
            height=self.camera['height'],
            timestamp=timestamp
        )

        # Collect IMU data
        imu_data = self._simulate_imu_data(timestamp)

        # Log the data
        sensor_reading = {
            'lidar': lidar_data,
            'camera': camera_data,
            'imu': imu_data,
            'timestamp': timestamp
        }

        self.sensor_data_log.append(sensor_reading)
        return sensor_reading

    def _simulate_lidar_scan(self) -> List[float]:
        """Simulate LIDAR data with environmental features"""
        distances = []
        for i in range(self.lidar['num_beams']):
            angle = i * self.lidar['resolution']
            angle_rad = np.radians(angle)

            # Simulate environment with obstacles
            distance = 5.0  # Base distance

            # Add some environmental features (walls, objects)
            distance += 1.0 * np.sin(4 * angle_rad)  # Simulate 4 walls
            distance += 0.5 * np.sin(8 * angle_rad)  # Simulate more detail
            distance += 0.1 * np.random.random()     # Add sensor noise

            # Ensure within range
            distance = max(self.lidar['min_range'],
                          min(self.lidar['max_range'], distance))
            distances.append(distance)

        return distances

    def _simulate_camera_frame(self) -> np.ndarray:
        """Simulate a camera frame with objects"""
        width, height = self.camera['width'], self.camera['height']
        frame = np.zeros((height, width, 3), dtype=np.uint8)

        # Add some objects to the scene
        # Red rectangle (obstacle)
        cv2.rectangle(frame, (100, 100), (200, 200), (255, 0, 0), -1)

        # Green circle (target)
        cv2.circle(frame, (400, 300), 50, (0, 255, 0), -1)

        # Blue triangle (landmark)
        triangle_points = np.array([[500, 100], [550, 200], [450, 200]], np.int32)
        cv2.fillPoly(frame, [triangle_points], (0, 0, 255))

        return frame

    def _simulate_imu_data(self, timestamp: float) -> IMUData:
        """Simulate IMU data with realistic values"""
        # Simulate a robot that's mostly stable but with small movements
        orientation = (
            np.random.uniform(-0.05, 0.05),   # roll (small tilt)
            np.random.uniform(-0.05, 0.05),   # pitch (small tilt)
            np.random.uniform(-0.1, 0.1)      # yaw (slow rotation)
        )

        # Simulate acceleration (mostly gravity + small movements)
        acceleration = (
            np.random.uniform(-0.1, 0.1),     # x acceleration
            np.random.uniform(-0.1, 0.1),     # y acceleration
            np.random.uniform(9.7, 10.1)      # z acceleration (gravity + small)
        )

        # Simulate angular velocity (mostly stable)
        angular_velocity = (
            np.random.uniform(-0.01, 0.01),   # x angular velocity
            np.random.uniform(-0.01, 0.01),   # y angular velocity
            np.random.uniform(-0.05, 0.05)    # z angular velocity
        )

        return IMUData(
            orientation=orientation,
            acceleration=acceleration,
            angular_velocity=angular_velocity,
            timestamp=timestamp
        )

    def process_sensor_data(self, sensor_reading: Dict[str, any]) -> Dict[str, any]:
        """
        Process raw sensor data to extract meaningful information
        """
        results = {}

        # Process LIDAR data - detect obstacles
        lidar_data = sensor_reading['lidar']
        obstacles = self._detect_obstacles(lidar_data.distances, threshold=1.5)
        results['obstacles'] = obstacles

        # Process camera data - detect colored objects
        camera_data = sensor_reading['camera']
        objects = self._detect_colored_objects(camera_data.image)
        results['objects'] = objects

        # Process IMU data - calculate orientation status
        imu_data = sensor_reading['imu']
        orientation_status = self._analyze_orientation(imu_data.orientation)
        results['orientation_status'] = orientation_status

        return results

    def _detect_obstacles(self, distances: List[float], threshold: float = 1.5) -> List[Dict]:
        """Detect obstacles in LIDAR data"""
        obstacles = []
        for i, distance in enumerate(distances):
            if distance < threshold:
                obstacle = {
                    'angle': i,
                    'distance': distance,
                    'direction': 'front' if abs(i - 180) < 45 else
                                'left' if i < 90 or i > 270 else
                                'right' if i < 180 else 'back'
                }
                obstacles.append(obstacle)
        return obstacles

    def _detect_colored_objects(self, image: np.ndarray) -> List[Dict]:
        """Detect colored objects in camera image"""
        objects = []

        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        # Define color ranges
        color_ranges = {
            'red': ([0, 50, 50], [10, 255, 255]),
            'red2': ([170, 50, 50], [180, 255, 255]),  # Wrap around hue
            'green': ([40, 50, 50], [80, 255, 255]),
            'blue': ([100, 50, 50], [130, 255, 255])
        }

        for color_name, (lower, upper) in color_ranges.items():
            if color_name == 'red2':  # Combine red ranges
                continue

            lower = np.array(lower)
            upper = np.array(upper)

            # Create mask for this color
            if color_name == 'red':  # Also include red2 range
                mask1 = cv2.inRange(hsv, lower, upper)
                lower2 = np.array([170, 50, 50])
                upper2 = np.array([180, 255, 255])
                mask2 = cv2.inRange(hsv, lower2, upper2)
                mask = mask1 + mask2
            else:
                mask = cv2.inRange(hsv, lower, upper)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 100:  # Minimum area threshold
                    x, y, w, h = cv2.boundingRect(contour)
                    obj = {
                        'color': color_name,
                        'x': x,
                        'y': y,
                        'width': w,
                        'height': h,
                        'area': area
                    }
                    objects.append(obj)

        return objects

    def _analyze_orientation(self, orientation: tuple) -> str:
        """Analyze IMU orientation data"""
        roll, pitch, yaw = orientation
        max_tilt = max(abs(roll), abs(pitch))

        if max_tilt > 0.2:
            return "tilted"
        elif max_tilt > 0.1:
            return "slightly_tilted"
        else:
            return "stable"

# Example usage
sensor_system = MultiSensorSystem()

# Collect a sample of sensor data
sensor_reading = sensor_system.collect_sensor_data()
processed_data = sensor_system.process_sensor_data(sensor_reading)

print(f"Detected {len(processed_data['obstacles'])} obstacles")
print(f"Detected {len(processed_data['objects'])} objects")
print(f"Orientation status: {processed_data['orientation_status']}")
```

## Code Examples with Explanations

### Sensor Data Fusion Example

This example shows how to combine data from multiple sensors to create a more complete understanding of the environment:

```python
class SensorFusion:
    """
    Combine data from multiple sensors to create a unified perception
    """
    def __init__(self):
        self.confidence_threshold = 0.7
        self.fusion_window = 0.1  # Time window for data fusion (seconds)

    def fuse_sensor_data(self, lidar_data: LIDARData,
                        camera_data: CameraData,
                        imu_data: IMUData) -> Dict:
        """
        Fuse data from multiple sensors to create a comprehensive view
        """
        # Create a unified environment representation
        environment = {
            'timestamp': max(lidar_data.timestamp,
                           camera_data.timestamp,
                           imu_data.timestamp),
            'objects': [],
            'obstacles': [],
            'robot_pose': self._estimate_robot_pose(imu_data),
            'spatial_map': self._create_spatial_map(lidar_data)
        }

        # Add camera-detected objects to the environment
        camera_objects = self._detect_objects_in_camera(camera_data.image)
        for obj in camera_objects:
            # Project camera objects into 3D space using LIDAR data
            projected_obj = self._project_to_3d(obj, lidar_data)
            environment['objects'].append(projected_obj)

        # Add LIDAR-detected obstacles
        lidar_obstacles = self._detect_obstacles_in_lidar(lidar_data.distances)
        environment['obstacles'].extend(lidar_obstacles)

        return environment

    def _estimate_robot_pose(self, imu_data: IMUData) -> Dict:
        """
        Estimate robot pose (position and orientation) from IMU data
        """
        # In a real system, this would integrate acceleration data
        # and combine with other sensors for better accuracy
        return {
            'position': {'x': 0, 'y': 0, 'z': 0},  # Simplified
            'orientation': {
                'roll': imu_data.orientation[0],
                'pitch': imu_data.orientation[1],
                'yaw': imu_data.orientation[2]
            }
        }

    def _create_spatial_map(self, lidar_data: LIDARData) -> List[Dict]:
        """
        Create a spatial map from LIDAR data
        """
        map_points = []
        for i, distance in enumerate(lidar_data.distances):
            angle_rad = np.radians(lidar_data.angles[i])
            x = distance * np.cos(angle_rad)
            y = distance * np.sin(angle_rad)

            if distance < 30.0:  # Within sensor range
                map_points.append({
                    'x': x,
                    'y': y,
                    'distance': distance,
                    'angle': lidar_data.angles[i]
                })

        return map_points

    def _project_to_3d(self, camera_obj: Dict, lidar_data: LIDARData) -> Dict:
        """
        Project a camera object into 3D space using LIDAR data
        """
        # Simplified projection - in reality, this would require
        # camera calibration and sensor fusion algorithms
        center_x = camera_obj['x'] + camera_obj['width'] // 2
        center_y = camera_obj['y'] + camera_obj['height'] // 2

        # Estimate distance using LIDAR (simplified)
        # In reality, this would use more sophisticated algorithms
        angle_estimate = (center_x / camera_data.width) * 360
        lidar_idx = int(angle_estimate)
        if 0 <= lidar_idx < len(lidar_data.distances):
            distance = lidar_data.distances[lidar_idx]
        else:
            distance = 2.0  # Default distance if out of bounds

        return {
            'color': camera_obj['color'],
            'type': 'object',
            'estimated_distance': distance,
            'position_2d': (center_x, center_y),
            'confidence': 0.8  # Estimated confidence
        }
```

### Perception Pipeline Example

This example demonstrates a complete perception pipeline:

```python
class PerceptionPipeline:
    """
    A complete perception pipeline that processes sensor data
    """
    def __init__(self):
        self.feature_detectors = {
            'edges': self._detect_edges,
            'corners': self._detect_corners,
            'blobs': self._detect_blobs
        }
        self.classifiers = {
            'color': self._classify_by_color,
            'shape': self._classify_by_shape
        }

    def run_perception(self, sensor_data: Dict) -> Dict:
        """
        Run the complete perception pipeline
        """
        results = {
            'features': {},
            'classifications': {},
            'environment_map': {},
            'navigation_data': {}
        }

        # Process LIDAR data
        if 'lidar' in sensor_data:
            lidar_map = self._process_lidar_data(sensor_data['lidar'])
            results['environment_map']['lidar'] = lidar_map

        # Process camera data
        if 'camera' in sensor_data:
            camera_features = self._process_camera_data(sensor_data['camera'])
            results['features'] = camera_features

        # Process IMU data
        if 'imu' in sensor_data:
            imu_state = self._process_imu_data(sensor_data['imu'])
            results['navigation_data']['pose'] = imu_state

        # Fuse all data for comprehensive understanding
        results['fused_perception'] = self._fuse_perception_data(results)

        return results

    def _process_lidar_data(self, lidar_data: LIDARData) -> Dict:
        """
        Process LIDAR data to extract environment features
        """
        # Detect obstacles
        obstacles = self._detect_obstacles(lidar_data.distances, threshold=1.0)

        # Create occupancy grid
        occupancy_grid = self._create_occupancy_grid(lidar_data)

        # Detect planar surfaces (like floors, walls)
        surfaces = self._detect_surfaces(lidar_data.distances)

        return {
            'obstacles': obstacles,
            'occupancy_grid': occupancy_grid,
            'surfaces': surfaces
        }

    def _process_camera_data(self, camera_data: CameraData) -> Dict:
        """
        Process camera data to extract visual features
        """
        image = camera_data.image

        # Apply various feature detectors
        features = {}
        for feature_name, detector in self.feature_detectors.items():
            features[feature_name] = detector(image)

        # Apply classifiers
        classifications = {}
        for classifier_name, classifier in self.classifiers.items():
            classifications[classifier_name] = classifier(image)

        return {
            'features': features,
            'classifications': classifications
        }

    def _detect_edges(self, image: np.ndarray) -> List:
        """Detect edges in the image"""
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        return cv2.findNonZero(edges) if cv2.findNonZero(edges) is not None else []

    def _detect_corners(self, image: np.ndarray) -> List:
        """Detect corners in the image"""
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        corners = cv2.goodFeaturesToTrack(gray, 25, 0.01, 10)
        return corners if corners is not None else []

    def _classify_by_color(self, image: np.ndarray) -> Dict:
        """Classify image regions by color"""
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        color_ranges = {
            'red': ((0, 50, 50), (10, 255, 255)),
            'yellow': ((20, 50, 50), (30, 255, 255)),
            'green': ((40, 50, 50), (80, 255, 255)),
            'blue': ((100, 50, 50), (130, 255, 255))
        }

        color_areas = {}
        for color_name, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            area = cv2.countNonZero(mask)
            color_areas[color_name] = area

        return color_areas
```

## Hands-On Exercises: Perception Implementation

### Exercise 1: Environment Mapping

Create a program that combines LIDAR and camera data to create an environment map:

```python
class EnvironmentMapper:
    """
    Create a map of the environment using sensor data
    """
    def __init__(self, map_resolution: float = 0.1):
        self.resolution = map_resolution
        self.map_width = 100  # meters
        self.map_height = 100  # meters
        self.grid_map = np.zeros((int(self.map_width/self.resolution),
                                 int(self.map_height/self.resolution)))

    def update_map_with_lidar(self, lidar_data: LIDARData, robot_pose: Dict):
        """
        Update the map with LIDAR data
        """
        for i, distance in enumerate(lidar_data.distances):
            angle = np.radians(lidar_data.angles[i])

            # Convert to global coordinates
            global_x = robot_pose['x'] + distance * np.cos(angle + robot_pose['yaw'])
            global_y = robot_pose['y'] + distance * np.sin(angle + robot_pose['yaw'])

            # Convert to grid coordinates
            grid_x = int((global_x + self.map_width/2) / self.resolution)
            grid_y = int((global_y + self.map_height/2) / self.resolution)

            # Mark as occupied if within bounds
            if 0 <= grid_x < self.grid_map.shape[0] and 0 <= grid_y < self.grid_map.shape[1]:
                self.grid_map[grid_x, grid_y] = 1  # Occupied

    def update_map_with_camera(self, camera_data: CameraData,
                              lidar_data: LIDARData, robot_pose: Dict):
        """
        Update the map with camera object detections
        """
        # Detect objects in camera
        objects = self._detect_objects_in_camera(camera_data.image)

        for obj in objects:
            # Estimate distance using LIDAR data
            # This is a simplified approach - real systems use more sophisticated methods
            angle_to_obj = obj['x'] / camera_data.width * 360  # Approximate angle
            lidar_idx = int(angle_to_obj)
            if 0 <= lidar_idx < len(lidar_data.distances):
                distance = lidar_data.distances[lidar_idx]
            else:
                distance = 2.0  # Default if out of bounds

            # Convert to global coordinates
            angle_rad = np.radians(angle_to_obj + np.degrees(robot_pose['yaw']))
            global_x = robot_pose['x'] + distance * np.cos(angle_rad)
            global_y = robot_pose['y'] + distance * np.sin(angle_rad)

            # Convert to grid coordinates and mark as object
            grid_x = int((global_x + self.map_width/2) / self.resolution)
            grid_y = int((global_y + self.map_height/2) / self.resolution)

            if 0 <= grid_x < self.grid_map.shape[0] and 0 <= grid_y < self.grid_map.shape[1]:
                self.grid_map[grid_x, grid_y] = 2  # Object detected

    def get_traversable_area(self) -> np.ndarray:
        """
        Get areas that are likely traversable
        """
        # Invert the occupancy grid to show free space
        free_space = (self.grid_map == 0).astype(int)
        return free_space

    def find_path_to_goal(self, start: tuple, goal: tuple) -> List[tuple]:
        """
        Simple path planning to a goal (using A* or similar algorithm)
        """
        # Simplified implementation - real systems use more sophisticated path planners
        path = [start]

        # Move in straight line toward goal (avoiding obvious obstacles)
        current = start
        while np.linalg.norm(np.array(current) - np.array(goal)) > 1.0:
            direction = np.array(goal) - np.array(current)
            direction = direction / np.linalg.norm(direction)  # Normalize
            next_point = tuple(current + direction * 1.0)  # Move 1m in that direction

            # Check if the next point is traversable
            grid_x = int((next_point[0] + self.map_width/2) / self.resolution)
            grid_y = int((next_point[1] + self.map_height/2) / self.resolution)

            if (0 <= grid_x < self.grid_map.shape[0] and
                0 <= grid_y < self.grid_map.shape[1] and
                self.grid_map[grid_x, grid_y] == 0):  # Free space
                path.append(next_point)
                current = next_point
            else:
                # Simple obstacle avoidance - move around
                break  # Simplified for this example

        return path
```

### Exercise 2: Object Tracking

Implement a simple object tracking system:

```python
class ObjectTracker:
    """
    Track objects across multiple sensor readings
    """
    def __init__(self, max_displacement: float = 0.5, history_length: int = 10):
        self.max_displacement = max_displacement
        self.history_length = history_length
        self.tracked_objects = {}  # id -> history of positions
        self.next_id = 0

    def update_with_sensor_data(self, sensor_data: Dict) -> Dict:
        """
        Update object tracking with new sensor data
        """
        new_objects = []

        # Process LIDAR data for object-like clusters
        if 'lidar' in sensor_data:
            lidar_objects = self._extract_objects_from_lidar(sensor_data['lidar'])
            new_objects.extend(lidar_objects)

        # Process camera data for visual objects
        if 'camera' in sensor_data:
            camera_objects = self._extract_objects_from_camera(sensor_data['camera'])
            new_objects.extend(camera_objects)

        # Associate new objects with existing tracks
        updated_tracks = self._associate_objects_with_tracks(new_objects)

        return updated_tracks

    def _extract_objects_from_lidar(self, lidar_data: LIDARData) -> List[Dict]:
        """
        Extract object-like clusters from LIDAR data
        """
        objects = []
        # Simple clustering algorithm to group nearby LIDAR points
        points = []
        for i, distance in enumerate(lidar_data.distances):
            if distance < 5.0:  # Only consider close points
                angle = np.radians(lidar_data.angles[i])
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                points.append((x, y, distance, angle))

        # Simple clustering: group nearby points
        clusters = []
        for point in points:
            assigned = False
            for cluster in clusters:
                # Check if point is close to existing cluster
                cluster_center = np.mean([p[:2] for p in cluster], axis=0)
                dist_to_cluster = np.linalg.norm(np.array(point[:2]) - cluster_center)

                if dist_to_cluster < 0.5:  # 50cm threshold
                    cluster.append(point)
                    assigned = True
                    break

            if not assigned:
                clusters.append([point])

        # Create objects from clusters
        for i, cluster in enumerate(clusters):
            if len(cluster) > 2:  # Only consider clusters with multiple points
                center = np.mean([p[:2] for p in cluster], axis=0)
                avg_distance = np.mean([p[2] for p in cluster])

                obj = {
                    'type': 'lidar_object',
                    'position': center,
                    'distance': avg_distance,
                    'size': len(cluster),
                    'confidence': min(0.9, len(cluster) / 10.0)  # Confidence based on cluster size
                }
                objects.append(obj)

        return objects

    def _associate_objects_with_tracks(self, new_objects: List[Dict]) -> Dict:
        """
        Associate new objects with existing tracks
        """
        # For each existing track, find the closest new object
        assignments = {}

        for track_id, track_history in self.tracked_objects.items():
            if track_history:
                last_position = track_history[-1]['position']

                # Find closest new object
                closest_obj = None
                min_dist = float('inf')

                for obj in new_objects:
                    dist = np.linalg.norm(np.array(last_position) - np.array(obj['position']))
                    if dist < min_dist and dist < self.max_displacement:
                        min_dist = dist
                        closest_obj = obj

                if closest_obj:
                    # Update the track with the new object
                    track_history.append(closest_obj)
                    # Keep only recent history
                    if len(track_history) > self.history_length:
                        track_history = track_history[-self.history_length:]
                    assignments[track_id] = track_history
                    # Remove from new_objects to avoid double assignment
                    if closest_obj in new_objects:
                        new_objects.remove(closest_obj)

        # Create new tracks for unassigned objects
        for obj in new_objects:
            new_track_id = self.next_id
            self.next_id += 1
            self.tracked_objects[new_track_id] = [obj]
            assignments[new_track_id] = [obj]

        return assignments
```

## Common Pitfalls and Solutions

### Pitfall 1: Sensor Calibration Issues
**Problem**: Sensors are not properly calibrated, leading to inaccurate data.

**Solutions**:
- Implement regular calibration routines
- Use calibration patterns and known reference points
- Monitor sensor drift over time

```python
def calibrate_sensor_pair(lidar_data: LIDARData, camera_data: CameraData,
                         calibration_data: Dict) -> Dict:
    """
    Calibrate the relationship between LIDAR and camera data
    """
    # Apply transformation matrix to align coordinate systems
    transformation_matrix = calibration_data['lidar_to_camera']

    # Transform LIDAR points to camera coordinate system
    calibrated_lidar = []
    for i, distance in enumerate(lidar_data.distances):
        angle = np.radians(lidar_data.angles[i])
        x = distance * np.cos(angle)
        y = distance * np.sin(angle)
        z = 0  # Assuming 2D LIDAR

        # Apply transformation
        point_3d = np.array([x, y, z, 1])  # Homogeneous coordinates
        transformed_point = transformation_matrix @ point_3d
        calibrated_lidar.append(transformed_point[:3])  # Back to 3D

    return {
        'calibrated_lidar': calibrated_lidar,
        'camera_data': camera_data
    }
```

### Pitfall 2: Time Synchronization
**Problem**: Sensor data from different sources has different timestamps, causing misalignment.

**Solutions**:
- Implement proper time synchronization
- Use interpolation for data alignment
- Buffer data for temporal consistency

### Pitfall 3: Data Association
**Problem**: Difficulty in matching sensor readings that correspond to the same environmental feature.

**Solutions**:
- Implement data association algorithms (nearest neighbor, Mahalanobis distance)
- Use consistent coordinate systems
- Apply uncertainty models

## Review Questions

1. What are the main types of sensors used in robotics and their primary applications?
2. Explain the advantages and limitations of LIDAR sensors compared to camera systems.
3. How does sensor fusion improve robotic perception compared to using individual sensors?
4. What is the difference between raw sensor data and processed perception data?
5. Describe the challenges in time synchronization across multiple sensors.

## Project Assignment: Perception System Integration

Create a comprehensive perception system that:
1. Integrates data from at least two different sensor types
2. Implements basic object detection and tracking
3. Creates a simple environment map
4. Provides navigation-relevant information
5. Handles sensor noise and uncertainty

Your system should include:
- Sensor data collection and preprocessing
- Feature extraction from different modalities
- Data association and fusion
- Environment representation
- Basic path planning based on perception data

## Further Resources

- [Robotics, Vision and Control by Peter Corke](https://link.springer.com/book/10.1007/978-3-642-20144-8)
- [Probabilistic Robotics by Thrun, Burgard, and Fox](https://mitpress.mit.edu/books/probabilistic-robotics)
- [OpenCV Documentation](https://docs.opencv.org/)
- [PCL (Point Cloud Library) Documentation](https://pointclouds.org/)
- [Sensor Fusion Tutorial](https://www.mathworks.com/help/fusion/ug/sensor-fusion-in-navigation.html)

:::info
Sensor systems are the eyes, ears, and skin of robots. The quality of your robot's perception directly impacts its ability to interact safely and effectively with the physical world.
:::