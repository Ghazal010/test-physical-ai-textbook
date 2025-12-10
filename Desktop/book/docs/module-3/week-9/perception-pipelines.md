---
title: Week 9 - Perception Pipelines
description: Building comprehensive perception pipelines using Isaac ROS
sidebar_position: 2
---

# Week 9: Perception Pipelines

## Learning Objectives
- Design and implement perception pipelines using Isaac ROS
- Integrate multiple sensors for comprehensive environment understanding
- Optimize perception systems for real-time performance
- Evaluate perception pipeline performance and accuracy
- Implement sensor fusion techniques for robust perception

## Prerequisites Check
- Completion of Week 8 (Isaac Sim Basics)
- Understanding of Isaac ROS packages and nodes
- Basic knowledge of computer vision concepts
- Experience with ROS 2 communication patterns

## Theoretical Concepts: Perception Pipeline Architecture

### Overview of Perception Pipelines

A perception pipeline in robotics is a systematic approach to processing sensor data to understand the environment. In the context of NVIDIA Isaac, perception pipelines leverage hardware acceleration and optimized algorithms to achieve real-time performance.

Key components of a perception pipeline:
- **Data Acquisition**: Collection of raw sensor data (cameras, LiDAR, IMU, etc.)
- **Preprocessing**: Calibration, noise reduction, and data normalization
- **Feature Extraction**: Identification of relevant features in the data
- **Object Detection**: Recognition and classification of objects
- **Tracking**: Maintaining object identities across frames
- **Fusion**: Combining information from multiple sensors
- **Post-processing**: Refinement and decision making

### Isaac ROS Perception Stack

NVIDIA Isaac ROS provides optimized perception packages that leverage GPU acceleration:

- **Isaac ROS Visual SLAM**: Simultaneous localization and mapping
- **Isaac ROS Detection**: Object detection and classification
- **Isaac ROS Image Pipeline**: Image preprocessing and enhancement
- **Isaac ROS Point Cloud Processing**: 3D point cloud manipulation
- **Isaac ROS Sensor Bridge**: Hardware interface and data synchronization

## Step-by-Step Tutorials: Building Perception Pipelines

### Basic Perception Pipeline

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import cv2
import numpy as np
from cv_bridge import CvBridge
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class BasicPerceptionPipeline(Node):
    """
    Basic perception pipeline combining camera and depth data
    """

    def __init__(self):
        super().__init__('basic_perception_pipeline')

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()

        # QoS profile for synchronized subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers for camera and depth data
        self.image_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw', qos_profile=qos_profile
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth/image_raw', qos_profile=qos_profile
        )

        # Synchronize camera and depth messages
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.image_depth_callback)

        # Publisher for detections
        self.detection_pub = self.create_publisher(Detection2DArray, '/perception/detections', 10)

        # Publishers for processed data
        self.segmented_pub = self.create_publisher(Image, '/perception/segmented', 10)

        self.get_logger().info("Basic Perception Pipeline initialized")

    def image_depth_callback(self, image_msg, depth_msg):
        """
        Process synchronized image and depth data
        """
        try:
            # Convert ROS images to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            # Perform basic segmentation
            segmented_image = self.perform_segmentation(cv_image)

            # Detect objects in the segmented image
            detections = self.detect_objects(segmented_image, cv_depth, depth_msg.header)

            # Publish detections
            self.detection_pub.publish(detections)

            # Publish segmented image
            segmented_msg = self.bridge.cv2_to_imgmsg(segmented_image, encoding='bgr8')
            segmented_msg.header = image_msg.header
            self.segmented_pub.publish(segmented_msg)

        except Exception as e:
            self.get_logger().error(f"Error in image_depth_callback: {e}")

    def perform_segmentation(self, image):
        """
        Perform basic color-based segmentation
        """
        # Convert to HSV for better color segmentation
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges for segmentation (example: red objects)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Apply mask to original image
        segmented = cv2.bitwise_and(image, image, mask=mask)

        return segmented

    def detect_objects(self, segmented_image, depth_image, header):
        """
        Detect objects in segmented image and estimate 3D positions
        """
        # Find contours in segmented image
        gray = cv2.cvtColor(segmented_image, cv2.COLOR_BGR2GRAY)
        contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = Detection2DArray()
        detections.header = header

        for contour in contours:
            # Filter by area to remove noise
            area = cv2.contourArea(contour)
            if area > 100:  # Minimum area threshold
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate center of object in image
                center_x, center_y = x + w//2, y + h//2

                # Estimate depth at object center
                if center_y < depth_image.shape[0] and center_x < depth_image.shape[1]:
                    depth = depth_image[center_y, center_x]

                    # Create detection
                    detection = Detection2D()
                    detection.header = header
                    detection.bbox.center.x = center_x
                    detection.bbox.center.y = center_y
                    detection.bbox.size_x = w
                    detection.bbox.size_y = h

                    # Add classification hypothesis
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = "object"
                    hypothesis.hypothesis.score = 0.8  # Confidence score
                    detection.results.append(hypothesis)

                    # Estimate 3D position (simplified)
                    # In practice, use proper camera calibration
                    detection.bbox.center.theta = depth  # Store depth as theta for now

                    detections.detections.append(detection)

        return detections

def main(args=None):
    rclpy.init(args=args)
    node = BasicPerceptionPipeline()

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

### Advanced Perception Pipeline with Isaac ROS

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, Imu
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
from cv_bridge import CvBridge
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from isaac_ros_perceptor_interfaces.msg import Detection2DArray, Detection3DArray

class AdvancedPerceptionPipeline(Node):
    """
    Advanced perception pipeline using Isaac ROS packages
    """

    def __init__(self):
        super().__init__('advanced_perception_pipeline')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Synchronized subscribers for multiple sensors
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/rgb/image_rect_color', qos_profile=sensor_qos
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth/image_rect', qos_profile=sensor_qos
        )
        self.camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/rgb/camera_info', qos_profile=sensor_qos
        )
        self.pointcloud_sub = message_filters.Subscriber(
            self, PointCloud2, '/camera/depth/color/points', qos_profile=sensor_qos
        )

        # Synchronize all sensor data
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.camera_info_sub, self.pointcloud_sub],
            queue_size=10, slop=0.2
        )
        self.sync.registerCallback(self.multi_sensor_callback)

        # Publishers for processed data
        self.detection_2d_pub = self.create_publisher(Detection2DArray, '/perception/detection_2d', 10)
        self.detection_3d_pub = self.create_publisher(Detection3DArray, '/perception/detection_3d', 10)
        self.visualization_pub = self.create_publisher(MarkerArray, '/perception/visualization', 10)

        # Initialize perception components
        self.object_detector = self.initialize_object_detector()
        self.segmentation_model = self.initialize_segmentation_model()
        self.tracker = self.initialize_tracker()

        self.get_logger().info("Advanced Perception Pipeline initialized")

    def initialize_object_detector(self):
        """
        Initialize object detection model (placeholder for Isaac ROS detector)
        """
        # In practice, this would initialize an Isaac ROS object detection node
        # For now, return a placeholder
        return {"initialized": True, "model": "isaac_ros_dnn_detection"}

    def initialize_segmentation_model(self):
        """
        Initialize segmentation model (placeholder for Isaac ROS segmentation)
        """
        # In practice, this would initialize an Isaac ROS segmentation node
        return {"initialized": True, "model": "isaac_ros_segmentation"}

    def initialize_tracker(self):
        """
        Initialize object tracker
        """
        return {
            "trackers": {},
            "next_id": 0,
            "max_displacement": 50  # pixels
        }

    def multi_sensor_callback(self, rgb_msg, depth_msg, camera_info_msg, pointcloud_msg):
        """
        Process synchronized multi-sensor data
        """
        try:
            # Convert images
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            # Perform object detection
            detections_2d = self.perform_object_detection(cv_rgb, rgb_msg.header)

            # Perform 3D localization
            detections_3d = self.localize_detections_in_3d(
                detections_2d, cv_depth, camera_info_msg, pointcloud_msg.header
            )

            # Track objects over time
            tracked_detections = self.track_detections(detections_3d, rgb_msg.header)

            # Publish results
            self.detection_2d_pub.publish(detections_2d)
            self.detection_3d_pub.publish(tracked_detections)

            # Publish visualization
            visualization = self.create_visualization_markers(tracked_detections, rgb_msg.header)
            self.visualization_pub.publish(visualization)

        except Exception as e:
            self.get_logger().error(f"Error in multi_sensor_callback: {e}")

    def perform_object_detection(self, image, header):
        """
        Perform object detection on image
        """
        # Placeholder for Isaac ROS object detection
        # In practice, this would call Isaac ROS detection node
        # For now, simulate detection

        detections = Detection2DArray()
        detections.header = header

        # Simulate some detections (in practice, these come from DNN)
        height, width = image.shape[:2]

        # Create sample detections
        for i in range(3):
            x = np.random.randint(0, width // 2)
            y = np.random.randint(0, height // 2)
            w = np.random.randint(50, 150)
            h = np.random.randint(50, 150)

            detection = Detection2D()
            detection.header = header
            detection.bbox.center.x = x + w // 2
            detection.bbox.center.y = y + h // 2
            detection.bbox.size_x = w
            detection.bbox.size_y = h

            # Add classification results
            class_hypothesis = ObjectHypothesisWithPose()
            class_hypothesis.hypothesis.class_id = f"object_{i}"
            class_hypothesis.hypothesis.score = 0.8 + (0.2 * np.random.random())
            detection.results.append(class_hypothesis)

            detections.detections.append(detection)

        return detections

    def localize_detections_in_3d(self, detections_2d, depth_image, camera_info, header):
        """
        Localize 2D detections in 3D space using depth information
        """
        detections_3d = Detection3DArray()
        detections_3d.header = header

        for detection_2d in detections_2d.detections:
            detection_3d = Detection3D()
            detection_3d.header = header

            # Get 2D bounding box center
            center_x = int(detection_2d.bbox.center.x)
            center_y = int(detection_2d.bbox.center.y)

            # Get depth at center point (with bounds checking)
            if center_y < depth_image.shape[0] and center_x < depth_image.shape[1]:
                depth = depth_image[center_y, center_x]

                # Convert 2D point to 3D using camera intrinsics
                # Simplified: use basic pinhole camera model
                # In practice, use proper camera calibration
                fx = camera_info.k[0]  # focal length x
                fy = camera_info.k[4]  # focal length y
                cx = camera_info.k[2]  # principal point x
                cy = camera_info.k[5]  # principal point y

                # Calculate 3D position
                pos_x = (center_x - cx) * depth / fx
                pos_y = (center_y - cy) * depth / fy
                pos_z = depth

                detection_3d.bbox.center.position.x = pos_x
                detection_3d.bbox.center.position.y = pos_y
                detection_3d.bbox.center.position.z = pos_z

                # Copy 2D bbox info
                detection_3d.bbox.size.x = detection_2d.bbox.size_x * depth / fx
                detection_3d.bbox.size.y = detection_2d.bbox.size_y * depth / fy
                detection_3d.bbox.size.z = depth  # Approximate depth extent

                # Copy detection results
                detection_3d.results = detection_2d.results

                detections_3d.detections.append(detection_3d)

        return detections_3d

    def track_detections(self, detections_3d, header):
        """
        Track detections over time
        """
        # Update tracker with new detections
        tracker = self.tracker

        # For each detection, find closest existing track or create new one
        updated_detections = Detection3DArray()
        updated_detections.header = header

        for detection in detections_3d.detections:
            # Find closest existing track
            best_match = None
            min_distance = float('inf')

            for track_id, track_info in tracker["trackers"].items():
                # Calculate distance to last known position
                pos = detection.bbox.center.position
                last_pos = track_info["position"]

                distance = np.sqrt(
                    (pos.x - last_pos.x)**2 +
                    (pos.y - last_pos.y)**2 +
                    (pos.z - last_pos.z)**2
                )

                if distance < min_distance and distance < tracker["max_displacement"]:
                    min_distance = distance
                    best_match = track_id

            if best_match is not None:
                # Update existing track
                tracker["trackers"][best_match]["position"] = detection.bbox.center.position
                tracker["trackers"][best_match]["last_seen"] = header.stamp
            else:
                # Create new track
                new_id = tracker["next_id"]
                tracker["trackers"][new_id] = {
                    "position": detection.bbox.center.position,
                    "last_seen": header.stamp,
                    "id": new_id
                }
                tracker["next_id"] += 1

                # Add ID to detection
                id_hypothesis = ObjectHypothesisWithPose()
                id_hypothesis.hypothesis.class_id = f"track_{new_id}"
                id_hypothesis.hypothesis.score = 1.0
                detection.results.append(id_hypothesis)

            updated_detections.detections.append(detection)

        return updated_detections

    def create_visualization_markers(self, detections_3d, header):
        """
        Create visualization markers for detected objects
        """
        markers = MarkerArray()

        for i, detection in enumerate(detections_3d.detections):
            # Create marker for bounding box
            marker = Marker()
            marker.header = header
            marker.ns = "detections"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Set position and size
            marker.pose.position = detection.bbox.center.position
            marker.pose.orientation.w = 1.0
            marker.scale.x = detection.bbox.size.x
            marker.scale.y = detection.bbox.size.y
            marker.scale.z = detection.bbox.size.z

            # Set color based on confidence
            confidence = detection.results[0].hypothesis.score if detection.results else 0.8
            marker.color.r = 1.0
            marker.color.g = 1.0 - confidence
            marker.color.b = 1.0 - confidence
            marker.color.a = 0.7

            markers.markers.append(marker)

        return markers

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedPerceptionPipeline()

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

### Multi-Sensor Fusion Pipeline

```python
import threading
import queue
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

@dataclass
class SensorData:
    """
    Container for sensor data with timestamp and source
    """
    source: str
    data: object
    timestamp: float
    frame_id: str

class MultiSensorFusionPipeline:
    """
    Advanced multi-sensor fusion pipeline for comprehensive perception
    """

    def __init__(self):
        # Queues for different sensor data
        self.rgb_queue = queue.Queue(maxsize=10)
        self.depth_queue = queue.Queue(maxsize=10)
        self.lidar_queue = queue.Queue(maxsize=10)
        self.imu_queue = queue.Queue(maxsize=10)

        # Thread for processing
        self.processing_thread = None
        self.is_running = False

        # Sensor synchronization
        self.synchronization_window = 0.05  # 50ms window for synchronization

        # Fusion results
        self.fusion_results = {}

    def start_processing(self):
        """
        Start the multi-sensor fusion processing
        """
        self.is_running = True
        self.processing_thread = threading.Thread(target=self._process_sensors)
        self.processing_thread.start()

    def stop_processing(self):
        """
        Stop the multi-sensor fusion processing
        """
        self.is_running = False
        if self.processing_thread:
            self.processing_thread.join()

    def add_sensor_data(self, sensor_data: SensorData):
        """
        Add sensor data to the appropriate queue
        """
        if sensor_data.source == 'rgb':
            try:
                self.rgb_queue.put_nowait(sensor_data)
            except queue.Full:
                # Drop oldest data if queue is full
                try:
                    self.rgb_queue.get_nowait()
                    self.rgb_queue.put_nowait(sensor_data)
                except queue.Empty:
                    pass
        elif sensor_data.source == 'depth':
            try:
                self.depth_queue.put_nowait(sensor_data)
            except queue.Full:
                try:
                    self.depth_queue.get_nowait()
                    self.depth_queue.put_nowait(sensor_data)
                except queue.Empty:
                    pass
        elif sensor_data.source == 'lidar':
            try:
                self.lidar_queue.put_nowait(sensor_data)
            except queue.Full:
                try:
                    self.lidar_queue.get_nowait()
                    self.lidar_queue.put_nowait(sensor_data)
                except queue.Empty:
                    pass
        elif sensor_data.source == 'imu':
            try:
                self.imu_queue.put_nowait(sensor_data)
            except queue.Full:
                try:
                    self.imu_queue.get_nowait()
                    self.imu_queue.put_nowait(sensor_data)
                except queue.Empty:
                    pass

    def _process_sensors(self):
        """
        Main processing loop for sensor fusion
        """
        while self.is_running:
            # Get synchronized sensor data
            synchronized_data = self._get_synchronized_data()

            if synchronized_data:
                # Perform fusion
                fusion_result = self._fuse_sensor_data(synchronized_data)

                # Store result
                self.fusion_results[time.time()] = fusion_result

                # Publish result (in ROS context)
                self._publish_fusion_result(fusion_result)

            time.sleep(0.01)  # 10ms sleep

    def _get_synchronized_data(self) -> Optional[Dict[str, SensorData]]:
        """
        Get synchronized data from all sensors within time window
        """
        # Get latest data from each sensor
        latest_data = {}

        # Get latest RGB data
        try:
            while not self.rgb_queue.empty():
                latest_data['rgb'] = self.rgb_queue.get_nowait()
        except queue.Empty:
            pass

        # Get latest depth data
        try:
            while not self.depth_queue.empty():
                latest_data['depth'] = self.depth_queue.get_nowait()
        except queue.Empty:
            pass

        # Get latest LiDAR data
        try:
            while not self.lidar_queue.empty():
                latest_data['lidar'] = self.lidar_queue.get_nowait()
        except queue.Empty:
            pass

        # Get latest IMU data
        try:
            while not self.imu_queue.empty():
                latest_data['imu'] = self.imu_queue.get_nowait()
        except queue.Empty:
            pass

        if len(latest_data) < 2:  # Need at least 2 sensors for fusion
            return None

        # Find reference timestamp (latest RGB or depth)
        ref_time = None
        if 'rgb' in latest_data:
            ref_time = latest_data['rgb'].timestamp
        elif 'depth' in latest_data:
            ref_time = latest_data['depth'].timestamp
        else:
            # Use the latest available timestamp
            timestamps = [data.timestamp for data in latest_data.values()]
            ref_time = max(timestamps)

        # Check if all data is within synchronization window
        synchronized = {}
        for source, data in latest_data.items():
            if abs(data.timestamp - ref_time) <= self.synchronization_window:
                synchronized[source] = data

        return synchronized if len(synchronized) >= 2 else None

    def _fuse_sensor_data(self, synchronized_data: Dict[str, SensorData]):
        """
        Fuse synchronized sensor data
        """
        # Example fusion: combine camera and LiDAR for object detection
        if 'rgb' in synchronized_data and 'lidar' in synchronized_data:
            # Project LiDAR points to camera image
            # Perform 2D-3D object detection fusion
            # Return fused detections

            # Placeholder for fusion result
            fusion_result = {
                'timestamp': synchronized_data['rgb'].timestamp,
                'sources': list(synchronized_data.keys()),
                'fused_objects': self._detect_fused_objects(synchronized_data),
                'environment_map': self._create_environment_map(synchronized_data)
            }

            return fusion_result

        # For other combinations, implement appropriate fusion
        return {
            'timestamp': next(iter(synchronized_data.values())).timestamp,
            'sources': list(synchronized_data.keys()),
            'data': synchronized_data
        }

    def _detect_fused_objects(self, synchronized_data: Dict[str, SensorData]) -> List[Dict]:
        """
        Detect objects using fused sensor data
        """
        # Placeholder implementation
        # In practice, this would use Isaac ROS fusion algorithms
        return [
            {
                'id': 1,
                'type': 'obstacle',
                'position_3d': {'x': 1.0, 'y': 0.5, 'z': 0.0},
                'confidence': 0.9,
                'sources': ['rgb', 'lidar']
            }
        ]

    def _create_environment_map(self, synchronized_data: Dict[str, SensorData]) -> Dict:
        """
        Create environment map from fused data
        """
        # Placeholder implementation
        return {
            'type': 'occupancy_grid',
            'resolution': 0.1,
            'data': []
        }

    def _publish_fusion_result(self, fusion_result):
        """
        Publish fusion result (placeholder for ROS publisher)
        """
        # In ROS context, this would publish to appropriate topics
        pass
```

### Real-time Performance Optimization

```python
import numpy as np
import cv2
from concurrent.futures import ThreadPoolExecutor, as_completed
import time

class OptimizedPerceptionPipeline:
    """
    Optimized perception pipeline for real-time performance
    """

    def __init__(self, max_workers=4):
        self.max_workers = max_workers
        self.executor = ThreadPoolExecutor(max_workers=max_workers)

        # Pre-allocated arrays for performance
        self.temp_arrays = {
            'hsv': None,
            'gray': None,
            'processed': None
        }

        # Performance metrics
        self.metrics = {
            'frame_count': 0,
            'processing_times': [],
            'average_fps': 0
        }

    def process_frame_async(self, image):
        """
        Process frame asynchronously using thread pool
        """
        future = self.executor.submit(self._process_single_frame, image)
        return future

    def _process_single_frame(self, image):
        """
        Process a single frame with optimized operations
        """
        start_time = time.time()

        # Resize image if too large (optimize for performance)
        h, w = image.shape[:2]
        if h > 640 or w > 640:
            scale = min(640/h, 640/w)
            new_w, new_h = int(w*scale), int(h*scale)
            image = cv2.resize(image, (new_w, new_h))

        # Ensure temp arrays are the right size
        if (self.temp_arrays['hsv'] is None or
            self.temp_arrays['hsv'].shape[:2] != image.shape[:2]):
            self.temp_arrays['hsv'] = np.empty((image.shape[0], image.shape[1], 3), dtype=np.uint8)

        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV, dst=self.temp_arrays['hsv'])

        # Perform segmentation
        mask = self._optimized_segmentation(hsv)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter and process contours
        objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Minimum area threshold
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate center
                center_x, center_y = x + w//2, y + h//2

                objects.append({
                    'center': (center_x, center_y),
                    'bbox': (x, y, w, h),
                    'area': area
                })

        # Update metrics
        processing_time = time.time() - start_time
        self.metrics['processing_times'].append(processing_time)
        self.metrics['frame_count'] += 1

        # Keep only recent metrics (last 100 frames)
        if len(self.metrics['processing_times']) > 100:
            self.metrics['processing_times'] = self.metrics['processing_times'][-100:]

        # Calculate average FPS
        if self.metrics['processing_times']:
            avg_processing_time = np.mean(self.metrics['processing_times'])
            self.metrics['average_fps'] = 1.0 / avg_processing_time if avg_processing_time > 0 else 0

        return {
            'objects': objects,
            'processing_time': processing_time,
            'fps': 1.0 / processing_time if processing_time > 0 else 0
        }

    def _optimized_segmentation(self, hsv_image):
        """
        Optimized segmentation using vectorized operations
        """
        # Define color ranges (example: red objects)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create masks using vectorized operations
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Apply morphological operations to clean up the mask
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask

    def get_performance_metrics(self):
        """
        Get performance metrics
        """
        return {
            'average_fps': self.metrics['average_fps'],
            'frame_count': self.metrics['frame_count'],
            'current_processing_time': self.metrics['processing_times'][-1] if self.metrics['processing_times'] else 0
        }

    def shutdown(self):
        """
        Shutdown the pipeline
        """
        self.executor.shutdown(wait=True)
```

## Hands-On Exercises: Perception Pipeline Implementation

### Exercise 1: Basic Object Detection Pipeline

Create a perception pipeline that combines camera and depth data to detect and localize objects:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectionPipeline(Node):
    """
    Exercise: Basic object detection pipeline
    """

    def __init__(self):
        super().__init__('object_detection_pipeline')

        self.bridge = CvBridge()

        # Subscribe to camera topics
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10
        )

        # Publisher for detected object positions
        self.object_pos_pub = self.create_publisher(PointStamped, '/detected_object', 10)

        # Store camera info
        self.camera_info = None
        self.latest_depth = None

        self.get_logger().info("Object Detection Pipeline initialized")

    def camera_info_callback(self, msg):
        """
        Store camera calibration info
        """
        self.camera_info = msg

    def depth_callback(self, msg):
        """
        Store latest depth image
        """
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")

    def image_callback(self, msg):
        """
        Process camera image to detect objects
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform color-based segmentation
            segmented = self.segment_by_color(cv_image)

            # Find objects in segmented image
            contours, _ = cv2.findContours(
                cv2.cvtColor(segmented, cv2.COLOR_BGR2GRAY),
                cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)

                    # Calculate center of object
                    center_x, center_y = x + w//2, y + h//2

                    # Get depth at center point
                    if (self.latest_depth is not None and
                        center_y < self.latest_depth.shape[0] and
                        center_x < self.latest_depth.shape[1]):

                        depth = self.latest_depth[center_y, center_x]

                        # Convert 2D pixel coordinates to 3D world coordinates
                        if self.camera_info and depth > 0:
                            # Use camera intrinsics to convert to 3D
                            fx = self.camera_info.k[0]
                            fy = self.camera_info.k[4]
                            cx = self.camera_info.k[2]
                            cy = self.camera_info.k[5]

                            # Calculate 3D position
                            pos_x = (center_x - cx) * depth / fx
                            pos_y = (center_y - cy) * depth / fy
                            pos_z = depth

                            # Publish detected object position
                            point_msg = PointStamped()
                            point_msg.header = Header()
                            point_msg.header.stamp = msg.header.stamp
                            point_msg.header.frame_id = self.camera_info.header.frame_id
                            point_msg.point.x = pos_x
                            point_msg.point.y = pos_y
                            point_msg.point.z = pos_z

                            self.object_pos_pub.publish(point_msg)

                            self.get_logger().info(
                                f"Detected object at: ({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f})"
                            )
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def segment_by_color(self, image):
        """
        Segment image by color (example: red objects)
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red color
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create masks
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Apply mask to image
        segmented = cv2.bitwise_and(image, image, mask=mask)

        return segmented

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionPipeline()

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

### Exercise 2: Performance-Optimized Pipeline

Create a perception pipeline optimized for real-time performance:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from threading import Lock
from collections import deque
import time

class OptimizedPerceptionNode(Node):
    """
    Exercise: Performance-optimized perception pipeline
    """

    def __init__(self):
        super().__init__('optimized_perception_node')

        self.bridge = CvBridge()
        self.lock = Lock()

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.optimized_image_callback, 5
        )

        # Publishers for performance metrics
        self.fps_pub = self.create_publisher(Float32, '/perception/fps', 10)
        self.processing_time_pub = self.create_publisher(Float32, '/perception/processing_time', 10)

        # Frame rate control
        self.frame_counter = 0
        self.start_time = time.time()
        self.processing_times = deque(maxlen=30)  # Keep last 30 processing times

        # Pre-allocated arrays for performance
        self.working_arrays = {
            'gray': None,
            'hsv': None,
            'mask': None
        }

        # Processing parameters
        self.process_every_n_frames = 3  # Process every 3rd frame to save CPU
        self.target_resolution = (320, 240)  # Target resolution for processing

        self.get_logger().info("Optimized Perception Node initialized")

    def optimized_image_callback(self, msg):
        """
        Optimized image callback that processes frames efficiently
        """
        # Throttle processing to save CPU
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            return

        start_time = time.time()

        try:
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize image for faster processing
            h, w = cv_image.shape[:2]
            if w > self.target_resolution[0] or h > self.target_resolution[1]:
                scale = min(
                    self.target_resolution[0] / w,
                    self.target_resolution[1] / h
                )
                new_w, new_h = int(w * scale), int(h * scale)
                cv_image = cv2.resize(cv_image, (new_w, new_h))

            # Process image
            result = self.optimized_process_image(cv_image)

            # Calculate and publish performance metrics
            processing_time = time.time() - start_time
            self.processing_times.append(processing_time)

            # Calculate FPS
            elapsed = time.time() - self.start_time
            fps = self.frame_counter / elapsed if elapsed > 0 else 0

            # Publish metrics
            fps_msg = Float32()
            fps_msg.data = float(fps)
            self.fps_pub.publish(fps_msg)

            time_msg = Float32()
            time_msg.data = float(processing_time)
            self.processing_time_pub.publish(time_msg)

            if self.frame_counter % 30 == 0:  # Log every 30 processed frames
                avg_processing = np.mean(self.processing_times) if self.processing_times else 0
                self.get_logger().info(
                    f"FPS: {fps:.2f}, Avg Processing Time: {avg_processing*1000:.2f}ms, "
                    f"Objects detected: {len(result['objects']) if result else 0}"
                )

        except Exception as e:
            self.get_logger().error(f"Error in optimized callback: {e}")

    def optimized_process_image(self, image):
        """
        Optimized image processing function
        """
        h, w = image.shape[:2]

        # Ensure working arrays are the right size
        if (self.working_arrays['hsv'] is None or
            self.working_arrays['hsv'].shape[0] != h or
            self.working_arrays['hsv'].shape[1] != w):
            self.working_arrays['hsv'] = np.empty((h, w, 3), dtype=np.uint8)

        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV, dst=self.working_arrays['hsv'])

        # Create color mask (example: blue objects)
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])

        # Use pre-allocated mask if possible
        if (self.working_arrays['mask'] is None or
            self.working_arrays['mask'].shape[0] != h or
            self.working_arrays['mask'].shape[1] != w):
            self.working_arrays['mask'] = np.zeros((h, w), dtype=np.uint8)

        mask = cv2.inRange(hsv, lower_blue, upper_blue, dst=self.working_arrays['mask'])

        # Find contours efficiently
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Process contours with area filtering
        objects = []
        min_area = 100
        max_contours = 10  # Limit number of contours to process

        for i, contour in enumerate(contours):
            if i >= max_contours:
                break

            area = cv2.contourArea(contour)
            if area >= min_area:
                # Get bounding rectangle
                x, y, w_roi, h_roi = cv2.boundingRect(contour)

                # Calculate center
                center_x, center_y = x + w_roi//2, y + h_roi//2

                objects.append({
                    'center': (center_x, center_y),
                    'bbox': (x, y, w_roi, h_roi),
                    'area': area
                })

        return {
            'objects': objects,
            'image_shape': (h, w)
        }

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedPerceptionNode()

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

### Pitfall 1: Sensor Data Synchronization
**Problem**: Sensor data from different sources arrives at different times, making fusion difficult.

**Solution**:
- Use message filters for time synchronization
- Implement interpolation for temporal alignment
- Use buffering strategies to match data rates

```python
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile, ReliabilityPolicy

def setup_synchronized_subscribers(self):
    """
    Properly synchronize multiple sensor streams
    """
    # Define QoS for sensor data (often best effort for sensors)
    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5
    )

    # Create subscribers
    rgb_sub = Subscriber(self, Image, '/camera/color/image_raw', qos_profile=sensor_qos)
    depth_sub = Subscriber(self, Image, '/camera/depth/image_raw', qos_profile=sensor_qos)
    imu_sub = Subscriber(self, Imu, '/imu/data', qos_profile=sensor_qos)

    # Synchronize with appropriate slop (time tolerance)
    sync = ApproximateTimeSynchronizer(
        [rgb_sub, depth_sub, imu_sub],
        queue_size=10,
        slop=0.1  # 100ms tolerance
    )
    sync.registerCallback(self.synchronized_callback)
```

### Pitfall 2: Memory Management in Real-time Systems
**Problem**: Perception pipelines can consume excessive memory, leading to performance degradation.

**Solution**:
- Pre-allocate arrays and reuse them
- Use memory pools for frequently allocated objects
- Implement proper cleanup and garbage collection

```python
class MemoryEfficientPipeline:
    """
    Memory-efficient perception pipeline
    """

    def __init__(self):
        # Pre-allocate working arrays
        self.working_memory = {
            'input_buffer': np.zeros((720, 1280, 3), dtype=np.uint8),
            'processed_buffer': np.zeros((720, 1280, 3), dtype=np.uint8),
            'temp_buffer': np.zeros((720, 1280), dtype=np.uint8),
            'feature_buffer': np.zeros(1000, dtype=np.float32),  # For features
        }

        # Use object pooling for detections
        self.detection_pool = []
        self.max_pool_size = 100

    def get_detection_object(self):
        """
        Get detection object from pool or create new one
        """
        if self.detection_pool:
            return self.detection_pool.pop()
        else:
            return Detection2D()  # Create new if pool is empty

    def return_detection_object(self, detection):
        """
        Return detection object to pool for reuse
        """
        if len(self.detection_pool) < self.max_pool_size:
            # Reset object before returning to pool
            detection.bbox.center.x = 0
            detection.bbox.center.y = 0
            detection.bbox.size_x = 0
            detection.bbox.size_y = 0
            detection.results.clear()
            self.detection_pool.append(detection)
```

### Pitfall 3: Computational Complexity
**Problem**: Complex perception algorithms may not run in real-time.

**Solution**:
- Use multi-threading for parallel processing
- Implement level-of-detail processing
- Use hardware acceleration (GPU, dedicated vision chips)

## Review Questions

1. What are the key components of a perception pipeline in robotics?
2. How does Isaac ROS enhance perception capabilities compared to standard ROS packages?
3. What are the challenges in synchronizing data from multiple sensors?
4. How can you optimize perception pipelines for real-time performance?
5. What role does sensor fusion play in robust perception systems?

## Project Assignment: Complete Perception System

Create a complete perception system that:
1. Integrates data from at least 3 different sensors (camera, depth, LiDAR, or IMU)
2. Implements object detection and classification
3. Performs 3D localization of detected objects
4. Tracks objects over time with stable IDs
5. Provides visualization of detection results
6. Includes performance monitoring and optimization

Your system should handle:
- Real-time processing at 10+ FPS
- Robust operation in varying lighting conditions
- Proper handling of sensor failures or missing data
- Memory-efficient processing
- Integration with ROS 2 navigation stack

## Further Resources

- [NVIDIA Isaac ROS Perception Documentation](https://nvidia-isaac-ros.github.io/released/perception/index.html)
- [ROS 2 Perception Tutorials](https://navigation.ros.org/tutorials/docs/get_backwards_compat.html)
- [Computer Vision for Robotics](https://www.cs.cmu.edu/~16373/s21/lectures.html)
- [Sensor Fusion Techniques](https://www.mathworks.com/help/fusion/ug/sensor-fusion-in-robotics.html)
- [Real-time Perception Systems](https://arxiv.org/list/cs.CV/recent)

:::tip
Remember to validate your perception pipeline with real-world data. Simulated data can only take you so far - testing with actual sensors in real environments is crucial for robust perception systems.
:::