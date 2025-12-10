---
title: Week 13 - CAPSTONE PROJECT
description: Building an autonomous humanoid robot with voice commands and navigation
sidebar_position: 1
---

# Week 13: CAPSTONE PROJECT

## Learning Objectives
- Integrate all modules into a complete autonomous humanoid system
- Build an autonomous humanoid that receives and executes voice commands
- Implement path planning using Nav2 for navigation
- Develop obstacle navigation capabilities
- Implement object identification with computer vision
- Integrate object manipulation functionality
- Test, debug, and optimize the complete system
- Present the final project according to guidelines

## Prerequisites Check
- Completion of all previous modules (1-4)
- Understanding of ROS 2, Gazebo, Isaac Sim, and VLA systems
- Experience with perception pipelines and navigation
- Knowledge of voice-to-action and language-to-action systems

## Theoretical Concepts: System Integration

### Holistic Robot Architecture

The capstone project represents the culmination of all previous learning modules, integrating:

- **Physical Foundation**: ROS 2-based control system (Module 1)
- **Digital Twin**: Simulation environment for testing (Module 2)
- **AI Brain**: Perception and decision-making (Module 3)
- **Human Interface**: Voice and language interaction (Module 4)

### Architecture Overview

The complete system consists of several interconnected subsystems:

1. **Perception System**: Processes sensor data to understand the environment
2. **Planning System**: Determines optimal paths and actions
3. **Control System**: Executes physical movements
4. **Interaction System**: Handles human-robot communication
5. **Integration Layer**: Coordinates all subsystems

### Key Integration Challenges

- **Timing Synchronization**: Ensuring all systems operate in harmony
- **Data Flow Management**: Efficient routing of information between subsystems
- **Error Handling**: Robust fallback mechanisms when components fail
- **Performance Optimization**: Balancing real-time requirements with computational demands

## Step-by-Step Tutorials: Complete System Integration

### Complete Autonomous Humanoid System

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration
import numpy as np
import cv2
from cv_bridge import CvBridge
import speech_recognition as sr
import openai
from openai import OpenAI
import json
import threading
import queue
from typing import Dict, List, Optional, Tuple

class AutonomousHumanoid(Node):
    """
    Complete autonomous humanoid system integrating all modules
    """

    def __init__(self):
        super().__init__('autonomous_humanoid')

        # Initialize subsystems
        self.bridge = CvBridge()
        self.voice_recognizer = sr.Recognizer()
        self.openai_client = OpenAI(api_key='your-api-key-here')  # Replace with your key

        # State management
        self.current_task = "idle"
        self.voice_command_queue = queue.Queue()
        self.navigation_goal_queue = queue.Queue()
        self.system_status = {
            'perception': 'ok',
            'navigation': 'ok',
            'voice': 'ok',
            'manipulation': 'ok'
        }

        # Subscribers for sensor data
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, 10
        )
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Publishers for commands and visualization
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.voice_cmd_pub = self.create_publisher(String, '/voice_command', 10)
        self.visualization_pub = self.create_publisher(MarkerArray, '/visualization', 10)

        # Voice recognition setup
        self.setup_voice_recognition()

        # Navigation setup
        self.setup_navigation()

        # Computer vision setup
        self.setup_computer_vision()

        # Start processing threads
        self.start_processing_threads()

        self.get_logger().info("Autonomous Humanoid System initialized")

    def setup_voice_recognition(self):
        """
        Setup voice recognition and command processing
        """
        # Adjust for ambient noise
        with sr.Microphone() as source:
            self.voice_recognizer.adjust_for_ambient_noise(source)

        # Start voice recognition thread
        self.voice_thread = threading.Thread(target=self.voice_recognition_loop)
        self.voice_thread.daemon = True

    def setup_navigation(self):
        """
        Setup navigation system with Nav2 integration
        """
        # Initialize navigation parameters
        self.nav_params = {
            'min_distance_to_obstacle': 0.5,
            'max_speed': 0.5,
            'rotation_speed': 0.5,
            'goal_tolerance': 0.2
        }

    def setup_computer_vision(self):
        """
        Setup computer vision for object detection
        """
        # Initialize object detection model (simplified)
        self.object_detector = {
            'model_loaded': True,
            'classes': ['person', 'chair', 'table', 'bottle', 'cup']
        }

    def start_processing_threads(self):
        """
        Start background processing threads
        """
        self.voice_thread.start()
        self.command_processor_thread = threading.Thread(target=self.process_commands)
        self.command_processor_thread.daemon = True
        self.command_processor_thread.start()

    def image_callback(self, msg):
        """
        Process camera images for object detection
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection
            detections = self.detect_objects(cv_image)

            # Update system status based on detections
            if detections:
                self.system_status['perception'] = 'detection_active'

            # Publish visualization markers
            self.publish_detection_markers(detections, msg.header)

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def laser_callback(self, msg):
        """
        Process laser scan for obstacle detection
        """
        try:
            # Find minimum distance in laser scan
            if len(msg.ranges) > 0:
                min_distance = min([r for r in msg.ranges if r > msg.range_min and r < msg.range_max])

                if min_distance < self.nav_params['min_distance_to_obstacle']:
                    self.system_status['navigation'] = 'obstacle_detected'
                    self.stop_robot()
                else:
                    self.system_status['navigation'] = 'clear_path'

        except Exception as e:
            self.get_logger().error(f"Error in laser callback: {e}")

    def odom_callback(self, msg):
        """
        Process odometry data for localization
        """
        # Update robot position and orientation
        self.current_position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'orientation': msg.pose.pose.orientation
        }

    def voice_recognition_loop(self):
        """
        Continuous voice recognition loop
        """
        with sr.Microphone() as source:
            while rclpy.ok():
                try:
                    # Listen for audio
                    audio = self.voice_recognizer.listen(source, timeout=1, phrase_time_limit=5)

                    # Recognize speech
                    text = self.voice_recognizer.recognize_google(audio)

                    if text:
                        self.get_logger().info(f"Recognized: {text}")

                        # Add to command queue
                        self.voice_command_queue.put(text)

                        # Publish recognized command
                        cmd_msg = String()
                        cmd_msg.data = text
                        self.voice_cmd_pub.publish(cmd_msg)

                except sr.WaitTimeoutError:
                    # No speech detected, continue listening
                    pass
                except sr.UnknownValueError:
                    self.get_logger().info("Could not understand audio")
                except sr.RequestError as e:
                    self.get_logger().error(f"Speech recognition error: {e}")
                except Exception as e:
                    self.get_logger().error(f"Voice recognition error: {e}")

    def process_commands(self):
        """
        Process commands from voice recognition
        """
        while rclpy.ok():
            try:
                if not self.voice_command_queue.empty():
                    command = self.voice_command_queue.get_nowait()

                    # Process the command using LLM
                    action_plan = self.process_voice_command_with_llm(command)

                    if action_plan:
                        self.execute_action_plan(action_plan)

            except queue.Empty:
                # No commands to process, sleep briefly
                pass
            except Exception as e:
                self.get_logger().error(f"Command processing error: {e}")

    def process_voice_command_with_llm(self, command: str) -> Optional[Dict]:
        """
        Process voice command using LLM to generate action plan
        """
        try:
            prompt = f"""
            You are an intelligent robot assistant. Convert the following human command
            into a structured action plan for a humanoid robot. The robot has capabilities
            for navigation, object detection, manipulation, and voice interaction.

            Command: "{command}"

            Respond with a JSON object containing:
            - "action_type": "navigation" | "manipulation" | "detection" | "conversation"
            - "target_location": [x, y, z] coordinates if navigation
            - "target_object": name of object if manipulation/detection
            - "action_sequence": list of specific actions to execute
            - "priority": "high" | "medium" | "low"

            Example response:
            {{
                "action_type": "navigation",
                "target_location": [2.0, 1.5, 0.0],
                "target_object": "red cup",
                "action_sequence": ["move_to_location", "detect_object", "grasp_object"],
                "priority": "medium"
            }}
            """

            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=300,
                temperature=0.3
            )

            response_text = response.choices[0].message.content

            # Extract JSON from response
            start_idx = response_text.find('{')
            end_idx = response_text.rfind('}') + 1

            if start_idx != -1 and end_idx != 0:
                json_str = response_text[start_idx:end_idx]
                action_plan = json.loads(json_str)
                return action_plan

        except json.JSONDecodeError:
            self.get_logger().error("Could not parse LLM response as JSON")
        except Exception as e:
            self.get_logger().error(f"LLM processing error: {e}")

        return None

    def execute_action_plan(self, action_plan: Dict):
        """
        Execute the action plan generated by LLM
        """
        action_type = action_plan.get('action_type', 'unknown')

        if action_type == 'navigation':
            target_location = action_plan.get('target_location', [0, 0, 0])
            self.navigate_to_location(target_location)

        elif action_type == 'manipulation':
            target_object = action_plan.get('target_object')
            self.manipulate_object(target_object)

        elif action_type == 'detection':
            target_object = action_plan.get('target_object')
            self.detect_object(target_object)

        elif action_type == 'conversation':
            self.respond_to_conversation(action_plan)

    def navigate_to_location(self, target_location: List[float]):
        """
        Navigate to specified location using Nav2
        """
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = target_location[0]
        goal_msg.pose.position.y = target_location[1]
        goal_msg.pose.position.z = target_location[2]

        # Set orientation to face forward
        goal_msg.pose.orientation.w = 1.0

        self.nav_goal_pub.publish(goal_msg)
        self.get_logger().info(f"Navigating to: {target_location}")

    def detect_objects(self, image):
        """
        Detect objects in image using computer vision
        """
        # Convert to RGB for processing
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Simple color-based detection (in practice, use trained models)
        detections = []

        # Detect red objects
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = cv2.bitwise_or(mask1, mask2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                center_x, center_y = x + w//2, y + h//2

                detection = {
                    'class': 'red_object',
                    'confidence': 0.8,
                    'bbox': [x, y, w, h],
                    'center': [center_x, center_y]
                }
                detections.append(detection)

        return detections

    def publish_detection_markers(self, detections, header):
        """
        Publish visualization markers for detected objects
        """
        markers = MarkerArray()

        for i, detection in enumerate(detections):
            marker = Marker()
            marker.header = header
            marker.ns = "detections"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position based on detection center (simplified)
            marker.pose.position.x = detection['center'][0] / 100.0  # Scale down for visualization
            marker.pose.position.y = detection['center'][1] / 100.0
            marker.pose.position.z = 0.5  # Fixed height

            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7

            markers.markers.append(marker)

        self.visualization_pub.publish(markers)

    def stop_robot(self):
        """
        Stop robot movement
        """
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)

    def respond_to_conversation(self, action_plan):
        """
        Respond to conversational requests
        """
        # In practice, this would use text-to-speech
        response = action_plan.get('response', 'I understand your request.')
        self.get_logger().info(f"Response: {response}")

def main(args=None):
    rclpy.init(args=args)
    humanoid = AutonomousHumanoid()

    try:
        rclpy.spin(humanoid)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced System Integration with Isaac Sim

```python
import omni
from pxr import Gf, UsdGeom, PhysxSchema
import carb
import numpy as np
from typing import Dict, List, Optional
import asyncio

class IsaacSimHumanoidIntegration:
    """
    Advanced integration with Isaac Sim for the complete humanoid system
    """

    def __init__(self, stage):
        self.stage = stage
        self.humanoid_prims = {}
        self.sensor_configs = {}
        self.control_systems = {}

    def setup_humanoid_model(self, model_path: str, position: Gf.Vec3f):
        """
        Setup humanoid model in Isaac Sim
        """
        # Import humanoid model
        humanoid_prim = omni.usd.get_context().get_stage().DefinePrim(
            f"/World/Humanoid", "Xform"
        )

        # Add USD reference to humanoid model
        humanoid_prim.GetReferences().AddReference(model_path)

        # Set initial position
        xform_api = UsdGeom.Xformable(humanoid_prim)
        xform_api.AddTranslateOp().Set(position)

        # Store reference for later use
        self.humanoid_prims['main'] = humanoid_prim

        # Setup physics properties
        self.setup_physics_properties(humanoid_prim)

    def setup_physics_properties(self, prim):
        """
        Setup physics properties for humanoid
        """
        # Apply rigid body properties
        physx_rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        physx_rigid_api.GetSolverPositionIterationCountAttr().Set(8)
        physx_rigid_api.GetSolverVelocityIterationCountAttr().Set(4)

    def setup_sensors(self):
        """
        Setup various sensors for the humanoid
        """
        # Camera sensor
        self.setup_camera_sensor()

        # LIDAR sensor
        self.setup_lidar_sensor()

        # IMU sensor
        self.setup_imu_sensor()

    def setup_camera_sensor(self):
        """
        Setup RGB camera sensor
        """
        # Create camera prim
        camera_prim = self.stage.DefinePrim("/World/Humanoid/Camera", "Camera")

        # Configure camera properties
        camera_api = UsdGeom.Camera(camera_prim)
        camera_api.GetFocalLengthAttr().Set(24.0)
        camera_api.GetHorizontalApertureAttr().Set(36.0)
        camera_api.GetVerticalApertureAttr().Set(20.25)

        # Add ROS bridge component for camera
        from omni.isaac.ros_bridge.scripts import ROSBridgeHelper
        ROSBridgeHelper.add_ros_bridge_to_stage(
            camera_prim,
            "rgb_camera_publisher",
            "sensor_msgs/msg/Image"
        )

        self.sensor_configs['camera'] = camera_prim

    def setup_lidar_sensor(self):
        """
        Setup LIDAR sensor
        """
        # Create LIDAR prim
        lidar_prim = self.stage.DefinePrim("/World/Humanoid/Lidar", "Lidar")

        # Configure LIDAR properties
        # In practice, this would use Isaac Sim's LIDAR schema

        self.sensor_configs['lidar'] = lidar_prim

    def setup_imu_sensor(self):
        """
        Setup IMU sensor
        """
        # Create IMU prim
        imu_prim = self.stage.DefinePrim("/World/Humanoid/IMU", "Imu")

        self.sensor_configs['imu'] = imu_prim

    def setup_control_interfaces(self):
        """
        Setup control interfaces for humanoid joints
        """
        # Define joint control interfaces
        joint_names = [
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle',
            'left_shoulder', 'left_elbow', 'left_wrist',
            'right_shoulder', 'right_elbow', 'right_wrist'
        ]

        for joint_name in joint_names:
            # Create joint control interface
            control_interface = self.create_joint_controller(joint_name)
            self.control_systems[joint_name] = control_interface

    def create_joint_controller(self, joint_name: str):
        """
        Create controller for a specific joint
        """
        # This would interface with Isaac Sim's control system
        # In practice, use Isaac Sim's control schemas
        controller = {
            'joint_name': joint_name,
            'control_type': 'position',  # position, velocity, or effort
            'limits': {'min': -2.0, 'max': 2.0},
            'gains': {'p': 100.0, 'i': 0.1, 'd': 10.0}
        }
        return controller

    def integrate_with_nav2(self):
        """
        Integrate with Nav2 for navigation planning
        """
        # Setup Nav2 interfaces in Isaac Sim
        # This involves creating path planning and obstacle avoidance systems
        nav2_integration = {
            'path_planner': 'navfn',
            'local_planner': 'dwa_local_planner',
            'costmap': 'obstacle_layer',
            'transforms': ['map', 'odom', 'base_link']
        }
        return nav2_integration

    def integrate_voice_system(self):
        """
        Integrate voice recognition and response system
        """
        # Setup audio input/output in simulation
        # This would involve virtual microphone and speaker systems
        voice_system = {
            'microphone': 'virtual_microphone',
            'speaker': 'virtual_speaker',
            'recognition_model': 'whisper_simulated',
            'response_synthesis': 'text_to_speech_simulated'
        }
        return voice_system

    def run_simulation_loop(self):
        """
        Main simulation loop integrating all systems
        """
        # This would run the complete simulation loop
        # integrating perception, planning, control, and interaction
        pass

class SystemIntegrationTester:
    """
    Comprehensive system integration tester
    """

    def __init__(self):
        self.test_results = {
            'perception': {'passed': 0, 'failed': 0},
            'navigation': {'passed': 0, 'failed': 0},
            'voice': {'passed': 0, 'failed': 0},
            'integration': {'passed': 0, 'failed': 0}
        }

    def run_comprehensive_tests(self):
        """
        Run all integration tests
        """
        tests = [
            self.test_perception_integration,
            self.test_navigation_integration,
            self.test_voice_integration,
            self.test_end_to_end_scenario
        ]

        for test_func in tests:
            self.get_logger().info(f"Running {test_func.__name__}")
            try:
                test_func()
                self.get_logger().info(f"{test_func.__name__} PASSED")
            except Exception as e:
                self.get_logger().error(f"{test_func.__name__} FAILED: {e}")

    def test_perception_integration(self):
        """
        Test perception system integration
        """
        # Test object detection
        # Test sensor fusion
        # Test environment understanding
        pass

    def test_navigation_integration(self):
        """
        Test navigation system integration
        """
        # Test path planning
        # Test obstacle avoidance
        # Test localization
        pass

    def test_voice_integration(self):
        """
        Test voice system integration
        """
        # Test voice recognition
        # Test command processing
        # Test response generation
        pass

    def test_end_to_end_scenario(self):
        """
        Test complete end-to-end scenario
        """
        # Simulate complete user interaction
        # Voice command -> Processing -> Action -> Verification
        pass

    def generate_test_report(self):
        """
        Generate comprehensive test report
        """
        report = "System Integration Test Report\n"
        report += "=" * 40 + "\n\n"

        for system, results in self.test_results.items():
            total = results['passed'] + results['failed']
            if total > 0:
                success_rate = (results['passed'] / total) * 100
                report += f"{system.title()} System:\n"
                report += f"  Tests Passed: {results['passed']}/{total} ({success_rate:.1f}%)\n"
                report += f"  Tests Failed: {results['failed']}/{total}\n\n"

        return report
```

## Code Examples with Explanations

### Multi-Modal Interaction System

```python
import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Dict, List, Callable, Any

@dataclass
class InteractionContext:
    """
    Context for multi-modal human-robot interaction
    """
    user_intent: str
    detected_objects: List[Dict]
    robot_state: Dict
    environment_map: Dict
    conversation_history: List[str]

class MultiModalInteractionSystem:
    """
    Multi-modal interaction system combining voice, vision, and action
    """

    def __init__(self):
        self.interaction_context = InteractionContext(
            user_intent="",
            detected_objects=[],
            robot_state={},
            environment_map={},
            conversation_history=[]
        )

        self.modality_processors = {
            'voice': self.process_voice_input,
            'vision': self.process_vision_input,
            'touch': self.process_touch_input,
            'action': self.process_action_feedback
        }

        self.executor = ThreadPoolExecutor(max_workers=4)
        self.event_loop = asyncio.new_event_loop()

    def process_multimodal_input(self, inputs: Dict[str, Any]) -> Dict[str, Any]:
        """
        Process multi-modal inputs simultaneously
        """
        results = {}

        # Process each modality in parallel
        futures = {}
        for modality, data in inputs.items():
            if modality in self.modality_processors:
                future = self.executor.submit(self.modality_processors[modality], data)
                futures[modality] = future

        # Collect results
        for modality, future in futures.items():
            try:
                results[modality] = future.result(timeout=5.0)
            except Exception as e:
                results[modality] = {'error': str(e)}

        # Integrate results across modalities
        integrated_result = self.integrate_modalities(results)

        return integrated_result

    def process_voice_input(self, audio_data) -> Dict[str, Any]:
        """
        Process voice input and extract meaning
        """
        # In practice, use speech-to-text and NLP
        recognized_text = "Move to the red chair"  # Simulated recognition

        # Extract intent using NLP
        intent = self.extract_intent(recognized_text)

        return {
            'text': recognized_text,
            'intent': intent,
            'confidence': 0.9
        }

    def process_vision_input(self, image_data) -> Dict[str, Any]:
        """
        Process vision input and detect objects/environment
        """
        # In practice, use computer vision models
        detected_objects = [
            {'class': 'chair', 'confidence': 0.85, 'position': [1.2, 0.5, 0.0]},
            {'class': 'table', 'confidence': 0.92, 'position': [2.0, 1.0, 0.0]}
        ]

        return {
            'objects': detected_objects,
            'environment': 'indoor_office',
            'lighting': 'normal'
        }

    def process_touch_input(self, touch_data) -> Dict[str, Any]:
        """
        Process touch/proximity input
        """
        # For humanoid robots, this could be tactile sensors
        return {
            'contact_points': [],
            'pressure': 0.0,
            'surface_type': 'unknown'
        }

    def process_action_feedback(self, action_data) -> Dict[str, Any]:
        """
        Process feedback from executed actions
        """
        return {
            'success': True,
            'metrics': {'execution_time': 0.5, 'accuracy': 0.95},
            'side_effects': []
        }

    def integrate_modalities(self, modality_results: Dict[str, Any]) -> Dict[str, Any]:
        """
        Integrate information from multiple modalities
        """
        # Combine voice intent with visual context
        voice_result = modality_results.get('voice', {})
        vision_result = modality_results.get('vision', {})

        if 'intent' in voice_result and 'objects' in vision_result:
            intent = voice_result['intent']
            objects = vision_result['objects']

            # Match intent with detected objects
            target_object = self.match_intent_to_object(intent, objects)

            return {
                'integrated_intent': intent,
                'target_object': target_object,
                'action_plan': self.generate_action_plan(intent, target_object),
                'confidence': min(voice_result.get('confidence', 0.8), 0.9)
            }

        return modality_results

    def match_intent_to_object(self, intent: Dict, objects: List[Dict]) -> Dict:
        """
        Match user intent to specific detected objects
        """
        # Extract object references from intent
        if 'target_class' in intent:
            target_class = intent['target_class']
            for obj in objects:
                if obj['class'] == target_class:
                    return obj

        # If no exact match, return the most relevant object
        return objects[0] if objects else {}

    def generate_action_plan(self, intent: Dict, target_object: Dict) -> List[Dict]:
        """
        Generate step-by-step action plan
        """
        plan = []

        if intent.get('action_type') == 'navigate_to':
            plan.append({
                'action': 'path_planning',
                'target': target_object.get('position', [0, 0, 0])
            })
            plan.append({
                'action': 'navigation_execution',
                'speed': 'moderate'
            })

        elif intent.get('action_type') == 'grasp_object':
            plan.append({
                'action': 'approach_object',
                'target': target_object
            })
            plan.append({
                'action': 'grasp_execution',
                'object': target_object
            })

        return plan

    def update_interaction_context(self, new_inputs: Dict[str, Any]):
        """
        Update the interaction context with new information
        """
        integrated_result = self.process_multimodal_input(new_inputs)

        # Update context based on integration results
        if 'integrated_intent' in integrated_result:
            self.interaction_context.user_intent = integrated_result['integrated_intent']

        if 'target_object' in integrated_result:
            self.interaction_context.detected_objects = [integrated_result['target_object']]

        # Add to conversation history
        if 'text' in new_inputs.get('voice', {}):
            self.interaction_context.conversation_history.append(
                new_inputs['voice']['text']
            )

    def get_response(self, user_input: Dict[str, Any]) -> str:
        """
        Generate natural language response to user
        """
        self.update_interaction_context(user_input)

        # Generate response based on current context
        if self.interaction_context.user_intent:
            action_plan = self.interaction_context.user_intent.get('action_plan', [])
            if action_plan:
                return f"I will execute the following plan: {[step['action'] for step in action_plan]}"

        return "I understand. Processing your request."
```

### Performance Optimization and System Monitoring

```python
import psutil
import time
from collections import deque
import threading
from dataclasses import dataclass
from typing import Dict, List, Callable

@dataclass
class PerformanceMetrics:
    """
    Data class for system performance metrics
    """
    timestamp: float
    cpu_usage: float
    memory_usage: float
    gpu_usage: float
    network_io: Dict[str, float]
    sensor_frequency: Dict[str, float]
    processing_latency: Dict[str, float]
    system_load: float

class SystemMonitor:
    """
    System monitoring and performance optimization
    """

    def __init__(self):
        self.metrics_history = deque(maxlen=100)  # Keep last 100 metrics
        self.performance_thresholds = {
            'cpu_usage': 80.0,      # % CPU usage
            'memory_usage': 85.0,   # % memory usage
            'processing_latency': 0.1,  # seconds
            'sensor_frequency': 10.0   # Hz
        }
        self.optimization_callbacks = []

        # Start monitoring thread
        self.monitoring_active = True
        self.monitoring_thread = threading.Thread(target=self.monitoring_loop)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()

    def monitoring_loop(self):
        """
        Continuous monitoring loop
        """
        while self.monitoring_active:
            metrics = self.collect_metrics()
            self.metrics_history.append(metrics)

            # Check for performance issues
            self.check_performance_issues(metrics)

            # Apply optimizations if needed
            self.apply_optimizations(metrics)

            time.sleep(1.0)  # Monitor every second

    def collect_metrics(self) -> PerformanceMetrics:
        """
        Collect system performance metrics
        """
        timestamp = time.time()

        # CPU usage
        cpu_usage = psutil.cpu_percent(interval=0.1)

        # Memory usage
        memory_info = psutil.virtual_memory()
        memory_usage = memory_info.percent

        # GPU usage (if available)
        gpu_usage = self.get_gpu_usage()

        # Network I/O
        net_io = psutil.net_io_counters()
        network_io = {
            'bytes_sent': net_io.bytes_sent,
            'bytes_recv': net_io.bytes_recv,
            'packets_sent': net_io.packets_sent,
            'packets_recv': net_io.packets_recv
        }

        # Sensor frequencies (estimated)
        sensor_frequency = {
            'camera': 15.0,  # Hz
            'lidar': 10.0,   # Hz
            'imu': 100.0     # Hz
        }

        # Processing latency (estimated)
        processing_latency = {
            'perception': 0.05,   # seconds
            'planning': 0.02,     # seconds
            'control': 0.01       # seconds
        }

        # System load
        system_load = sum([cpu_usage, memory_usage]) / 2.0

        return PerformanceMetrics(
            timestamp=timestamp,
            cpu_usage=cpu_usage,
            memory_usage=memory_usage,
            gpu_usage=gpu_usage,
            network_io=network_io,
            sensor_frequency=sensor_frequency,
            processing_latency=processing_latency,
            system_load=system_load
        )

    def get_gpu_usage(self) -> float:
        """
        Get GPU usage (placeholder - implement based on your GPU)
        """
        try:
            # For NVIDIA GPUs, you could use nvidia-ml-py
            # This is a simplified placeholder
            import subprocess
            result = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                gpu_usage = float(result.stdout.strip())
                return gpu_usage
        except:
            pass
        return 0.0  # Return 0 if GPU not available

    def check_performance_issues(self, metrics: PerformanceMetrics):
        """
        Check for performance issues based on thresholds
        """
        issues = []

        if metrics.cpu_usage > self.performance_thresholds['cpu_usage']:
            issues.append(f"High CPU usage: {metrics.cpu_usage}%")

        if metrics.memory_usage > self.performance_thresholds['memory_usage']:
            issues.append(f"High memory usage: {metrics.memory_usage}%")

        for sensor, freq in metrics.sensor_frequency.items():
            if freq < self.performance_thresholds['sensor_frequency']:
                issues.append(f"Low {sensor} frequency: {freq}Hz")

        for component, latency in metrics.processing_latency.items():
            if latency > self.performance_thresholds['processing_latency']:
                issues.append(f"High {component} latency: {latency}s")

        if issues:
            self.handle_performance_issues(issues, metrics)

    def handle_performance_issues(self, issues: List[str], metrics: PerformanceMetrics):
        """
        Handle performance issues
        """
        print(f"Performance issues detected: {issues}")

        # Log the issues
        for issue in issues:
            print(f"[PERFORMANCE] {issue} at {time.ctime(metrics.timestamp)}")

    def apply_optimizations(self, metrics: PerformanceMetrics):
        """
        Apply optimizations based on current metrics
        """
        optimizations_applied = []

        # If CPU usage is high, reduce processing frequency
        if metrics.cpu_usage > 85:
            optimizations_applied.append("Reduced processing frequency")
            # In practice, this would adjust system parameters

        # If memory usage is high, trigger garbage collection
        if metrics.memory_usage > 90:
            optimizations_applied.append("Triggered memory optimization")
            # In practice, this would free up memory

        if optimizations_applied:
            print(f"Applied optimizations: {optimizations_applied}")

    def add_optimization_callback(self, callback: Callable):
        """
        Add callback for optimization events
        """
        self.optimization_callbacks.append(callback)

    def get_performance_summary(self) -> Dict[str, float]:
        """
        Get summary of recent performance metrics
        """
        if not self.metrics_history:
            return {}

        recent_metrics = list(self.metrics_history)[-10:]  # Last 10 metrics

        summary = {
            'avg_cpu_usage': sum(m.cpu_usage for m in recent_metrics) / len(recent_metrics),
            'avg_memory_usage': sum(m.memory_usage for m in recent_metrics) / len(recent_metrics),
            'avg_gpu_usage': sum(m.gpu_usage for m in recent_metrics) / len(recent_metrics),
            'max_cpu_usage': max(m.cpu_usage for m in recent_metrics),
            'max_memory_usage': max(m.memory_usage for m in recent_metrics),
        }

        return summary

    def stop_monitoring(self):
        """
        Stop the monitoring system
        """
        self.monitoring_active = False
        if self.monitoring_thread.is_alive():
            self.monitoring_thread.join()
```

## Hands-On Exercises: Complete System Implementation

### Exercise 1: Voice Command to Navigation Pipeline

Implement a complete pipeline from voice command to navigation execution:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from openai import OpenAI
import json
import re

class VoiceNavigationPipeline(Node):
    """
    Exercise: Voice command to navigation pipeline
    """

    def __init__(self):
        super().__init__('voice_navigation_pipeline')

        # Initialize OpenAI client
        self.openai_client = OpenAI(api_key='your-api-key-here')  # Replace with your key

        # Subscribe to voice commands
        self.voice_sub = self.create_subscription(
            String, '/voice_command', self.voice_command_callback, 10
        )

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Location mapping (in a real system, this would be learned)
        self.location_map = {
            'kitchen': [2.0, 1.0, 0.0],
            'living room': [0.0, 2.0, 0.0],
            'bedroom': [-2.0, 1.0, 0.0],
            'office': [1.0, -2.0, 0.0],
            'dining room': [-1.0, -1.0, 0.0]
        }

        self.get_logger().info("Voice Navigation Pipeline initialized")

    def voice_command_callback(self, msg):
        """
        Process voice command and initiate navigation
        """
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Process command with LLM to extract navigation intent
        navigation_goal = self.extract_navigation_goal(command)

        if navigation_goal:
            self.navigate_to_goal(navigation_goal)
        else:
            self.get_logger().info("Could not extract navigation goal from command")

    def extract_navigation_goal(self, command: str) -> list:
        """
        Extract navigation goal from voice command using LLM
        """
        try:
            prompt = f"""
            Extract navigation goal from the following command:
            "{command}"

            If the command contains a destination, return the coordinates.
            If it refers to a named location, map it to coordinates.
            If no navigation is requested, return an empty list.

            Available locations and coordinates:
            - kitchen: [2.0, 1.0, 0.0]
            - living room: [0.0, 2.0, 0.0]
            - bedroom: [-2.0, 1.0, 0.0]
            - office: [1.0, -2.0, 0.0]
            - dining room: [-1.0, -1.0, 0.0]

            Return only the coordinates as a JSON array [x, y, z].
            If uncertain, return [].
            """

            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=100,
                temperature=0.1
            )

            response_text = response.choices[0].message.content

            # Extract coordinates from response
            coords_match = re.search(r'\[([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)\]', response_text)
            if coords_match:
                x = float(coords_match.group(1))
                y = float(coords_match.group(2))
                z = float(coords_match.group(3))
                return [x, y, z]

            # Check if it's a named location
            for location_name, coords in self.location_map.items():
                if location_name.lower() in command.lower():
                    return coords

        except Exception as e:
            self.get_logger().error(f"Error extracting navigation goal: {e}")

        return []  # Return empty list if no goal found

    def navigate_to_goal(self, goal_position: list):
        """
        Send navigation goal to Nav2
        """
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = goal_position[0]
        goal_msg.pose.pose.position.y = goal_position[1]
        goal_msg.pose.pose.position.z = goal_position[2]
        # Set orientation to face forward
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )

        send_goal_future.add_done_callback(self.navigation_goal_response_callback)

    def navigation_goal_response_callback(self, future):
        """
        Handle navigation goal response
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_feedback_callback(self, feedback_msg):
        """
        Handle navigation feedback
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation progress: {feedback.distance_remaining:.2f}m remaining')

    def navigation_result_callback(self, future):
        """
        Handle navigation result
        """
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceNavigationPipeline()

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

### Exercise 2: Complete System Integration Test

Create a comprehensive test for the entire system:

```python
import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
import threading
import time
from typing import Dict, Any

class SystemIntegrationTest(Node):
    """
    Exercise: Complete system integration test
    """

    def __init__(self):
        super().__init__('system_integration_test')

        # Test results
        self.test_results = {
            'perception': False,
            'navigation': False,
            'voice': False,
            'integration': False
        }

        # Test subscribers to monitor system behavior
        self.voice_response_sub = self.create_subscription(
            String, '/voice_response', self.voice_response_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Test publishers to stimulate the system
        self.voice_cmd_pub = self.create_publisher(String, '/voice_command', 10)

        # Test state tracking
        self.received_voice_response = False
        self.moved_robot = False
        self.detected_obstacles = False
        self.processed_images = False
        self.updated_odometry = False

        self.get_logger().info("System Integration Test initialized")

    def voice_response_callback(self, msg):
        """
        Track voice response from the system
        """
        self.received_voice_response = True
        self.test_results['voice'] = True

    def cmd_vel_callback(self, msg):
        """
        Track robot movement commands
        """
        if msg.linear.x != 0 or msg.angular.z != 0:
            self.moved_robot = True

    def scan_callback(self, msg):
        """
        Track obstacle detection
        """
        if len(msg.ranges) > 0:
            self.detected_obstacles = True
            self.test_results['perception'] = True

    def image_callback(self, msg):
        """
        Track image processing
        """
        self.processed_images = True

    def odom_callback(self, msg):
        """
        Track localization updates
        """
        self.updated_odometry = True

    def run_integration_tests(self):
        """
        Run complete integration tests
        """
        self.get_logger().info("Starting system integration tests...")

        # Test 1: Voice command processing
        self.test_voice_command()
        time.sleep(2)

        # Test 2: Navigation response
        self.test_navigation_response()
        time.sleep(2)

        # Test 3: Perception system
        self.test_perception_system()
        time.sleep(2)

        # Compile results
        self.test_results['navigation'] = self.moved_robot
        self.test_results['integration'] = all(self.test_results.values())

        self.print_test_results()

    def test_voice_command(self):
        """
        Test voice command processing
        """
        cmd_msg = String()
        cmd_msg.data = "move to the kitchen"
        self.voice_cmd_pub.publish(cmd_msg)
        self.get_logger().info("Published voice command: 'move to the kitchen'")

    def test_navigation_response(self):
        """
        Test navigation system response
        """
        # The navigation test depends on the system responding to voice commands
        # and executing navigation
        pass

    def test_perception_system(self):
        """
        Test perception system
        """
        # The perception test is validated by sensor data callbacks
        pass

    def print_test_results(self):
        """
        Print comprehensive test results
        """
        print("\n" + "="*50)
        print("SYSTEM INTEGRATION TEST RESULTS")
        print("="*50)

        for system, passed in self.test_results.items():
            status = "PASS" if passed else "FAIL"
            print(f"{system.capitalize():<12}: {status}")

        overall_result = "PASS" if all(self.test_results.values()) else "FAIL"
        print(f"\n{'Overall':<12}: {overall_result}")
        print("="*50)

def run_integration_tests():
    """
    Function to run integration tests
    """
    rclpy.init()

    test_node = SystemIntegrationTest()

    # Run tests in a separate thread to allow ROS spinning
    def run_tests():
        time.sleep(1)  # Allow systems to initialize
        test_node.run_integration_tests()

    test_thread = threading.Thread(target=run_tests)
    test_thread.start()

    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()
        test_thread.join()

# Unit tests for individual components
class TestPerceptionSystem(unittest.TestCase):
    """
    Unit tests for perception system components
    """

    def test_object_detection(self):
        """
        Test object detection functionality
        """
        # Mock image processing
        import numpy as np
        import cv2

        # Create a test image with a red rectangle (simulated object)
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(test_image, (200, 200), (300, 300), (0, 0, 255), -1)  # Red rectangle

        # Simple red object detection
        hsv = cv2.cvtColor(test_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = cv2.bitwise_or(mask1, mask2)

        # Check if red object is detected
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected = len(contours) > 0

        self.assertTrue(detected, "Red object should be detected in the test image")

    def test_laser_processing(self):
        """
        Test laser scan processing
        """
        # Simulate laser scan data
        ranges = [float('inf')] * 360  # 360 degree scan
        ranges[180] = 1.0  # Obstacle at 180 degrees, 1m away

        # Process scan to detect obstacles
        min_distance = min(ranges) if ranges else float('inf')
        has_obstacle = min_distance < 2.0  # Obstacle within 2m

        self.assertTrue(has_obstacle, "Obstacle should be detected at 1m distance")

class TestNavigationSystem(unittest.TestCase):
    """
    Unit tests for navigation system components
    """

    def test_path_planning(self):
        """
        Test basic path planning logic
        """
        # Simple path planning test
        start = (0, 0)
        goal = (5, 5)

        # Calculate simple path (Manhattan distance)
        path = []
        x, y = start
        while (x, y) != goal:
            if x < goal[0]:
                x += 1
            elif x > goal[0]:
                x -= 1
            if y < goal[1]:
                y += 1
            elif y > goal[1]:
                y -= 1
            path.append((x, y))

        self.assertEqual(path[-1], goal, "Path should reach the goal")
        self.assertLess(len(path), 50, "Path should be reasonably efficient")

if __name__ == '__main__':
    # Run unit tests
    unittest.main(argv=[''], exit=False, verbosity=2)

    # Run integration tests
    print("\nRunning system integration tests...")
    run_integration_tests()
```

## Common Pitfalls and Solutions

### Pitfall 1: Integration Complexity
**Problem**: Connecting all subsystems creates complex interactions and dependencies.

**Solution**:
- Use modular design with well-defined interfaces
- Implement comprehensive error handling
- Create thorough testing procedures

```python
class ModularSystemIntegrator:
    """
    Modular approach to system integration
    """

    def __init__(self):
        self.modules = {
            'perception': self.initialize_perception_module(),
            'navigation': self.initialize_navigation_module(),
            'interaction': self.initialize_interaction_module(),
            'control': self.initialize_control_module()
        }

        self.connect_modules()

    def initialize_perception_module(self):
        """
        Initialize perception module with clear interface
        """
        return {
            'process': self.perception_process,
            'subscribe': ['/camera/rgb/image_raw', '/scan'],
            'publish': ['/perception/objects', '/perception/scene']
        }

    def initialize_navigation_module(self):
        """
        Initialize navigation module with clear interface
        """
        return {
            'process': self.navigation_process,
            'subscribe': ['/perception/objects', '/goal_pose'],
            'publish': ['/cmd_vel', '/navigation/status']
        }

    def connect_modules(self):
        """
        Connect modules through ROS topics
        """
        # This ensures each module only depends on the interfaces
        # of other modules, not their internal implementations
        pass
```

### Pitfall 2: Performance Bottlenecks
**Problem**: System becomes slow when all components run simultaneously.

**Solution**:
- Implement priority-based processing
- Use multi-threading for parallel processing
- Optimize critical paths

### Pitfall 3: Debugging Complex Interactions
**Problem**: Difficult to debug when multiple systems interact.

**Solution**:
- Implement comprehensive logging
- Use visualization tools
- Create isolated testing environments

## Review Questions

1. What are the key components that must be integrated in the capstone project?
2. How does the voice-to-action pipeline work in the complete system?
3. What are the main challenges in integrating perception, navigation, and interaction?
4. How do you test a complete autonomous humanoid system?
5. What performance considerations are important for the integrated system?

## Project Assignment: Complete Autonomous Humanoid

Build a complete autonomous humanoid robot system that:
1. Receives and processes voice commands using speech recognition
2. Plans and executes navigation to specified locations using Nav2
3. Navigates around obstacles autonomously
4. Identifies objects in the environment using computer vision
5. Manipulates objects when requested (simulation only)
6. Integrates all modules into a cohesive system
7. Includes comprehensive testing and validation
8. Presents the final system according to project guidelines

Your system should demonstrate:
- End-to-end functionality from voice command to action execution
- Robust performance in various scenarios
- Proper error handling and fallback behaviors
- Real-time performance suitable for autonomous operation
- Integration of all four course modules

## Further Resources

- [ROS 2 Navigation Stack](https://navigation.ros.org/)
- [OpenAI API Documentation](https://platform.openai.com/docs/api-reference)
- [Computer Vision in Robotics](https://www.cs.cmu.edu/~16373/s21/lectures.html)
- [Humanoid Robot Control](https://humanoid.ros.org/)
- [System Integration Best Practices](https://arxiv.org/list/cs.RO/recent)

:::warning
Ensure proper safety measures are in place when testing on real robots. Always have emergency stop procedures and human supervision during testing of integrated systems.
:::