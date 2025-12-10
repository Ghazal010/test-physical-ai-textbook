---
title: Week 12 - Language-to-Action
description: Translating natural language to ROS 2 actions and implementing multi-modal interaction
sidebar_position: 1
---

# Week 12: Language-to-Action

## Learning Objectives
- Use LLMs (GPT models) for cognitive planning in robotics
- Translate natural language to ROS 2 actions for robot control
- Implement multi-modal interaction (speech, gesture, vision)
- Create context understanding and memory systems
- Build conversational robot interfaces for natural human-robot interaction

## Prerequisites Check
- Completion of Week 11 (Voice-to-Action)
- Understanding of LLM integration with robotics
- Experience with ROS 2 action servers and clients
- Knowledge of natural language processing concepts

## Theoretical Concepts: Language-to-Action Pipeline

### Cognitive Planning with LLMs

Large Language Models (LLMs) can serve as cognitive planners for robots by translating high-level natural language instructions into executable action sequences. This involves:

1. **Understanding**: Parsing natural language to extract intent and relevant information
2. **Reasoning**: Using world knowledge to determine appropriate actions
3. **Planning**: Sequencing actions to achieve the requested goal
4. **Execution**: Converting high-level plans into low-level robot commands

### Natural Language to Robot Actions

The translation from natural language to robot actions involves several key components:

- **Intent Recognition**: Identifying the user's goal from language
- **Entity Extraction**: Identifying objects, locations, and parameters
- **Action Mapping**: Converting linguistic concepts to robot capabilities
- **Context Integration**: Using environmental and historical context

### Multi-Modal Interaction Framework

Effective human-robot interaction requires combining multiple modalities:

- **Visual**: Understanding the environment through perception
- **Linguistic**: Processing natural language commands
- **Contextual**: Maintaining conversation and task context
- **Embodied**: Understanding spatial relationships and physical constraints

## Step-by-Step Tutorials: Language-to-Action Implementation

### Basic Language-to-Action System

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
from typing import Dict, List, Optional

class LanguageToActionNode(Node):
    """
    Basic language-to-action system for robotics
    """

    def __init__(self):
        super().__init__('language_to_action_node')

        # Initialize OpenAI client
        self.openai_client = OpenAI(api_key='your-api-key-here')  # Replace with your key

        # Subscribe to natural language commands
        self.command_sub = self.create_subscription(
            String, '/natural_language_command', self.command_callback, 10
        )

        # Publisher for system responses
        self.response_pub = self.create_publisher(String, '/system_response', 10)

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Context storage
        self.conversation_context = []
        self.object_locations = {}  # Store known object locations
        self.robot_state = {}       # Store robot state information

        self.get_logger().info("Language-to-Action node initialized")

    def command_callback(self, msg):
        """
        Process natural language command and convert to robot actions
        """
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Add to conversation context
        self.conversation_context.append({
            'speaker': 'user',
            'text': command,
            'timestamp': self.get_clock().now().to_msg()
        })

        # Process command with LLM to extract action plan
        action_plan = self.process_command_with_llm(command)

        if action_plan:
            self.execute_action_plan(action_plan)
        else:
            response_msg = String()
            response_msg.data = "I couldn't understand that command. Could you please rephrase?"
            self.response_pub.publish(response_msg)

    def process_command_with_llm(self, command: str) -> Optional[Dict]:
        """
        Process command using LLM to generate action plan
        """
        try:
            # Create context for the LLM
            context = self.build_context(command)

            prompt = f"""
            You are an intelligent robot assistant that can understand natural language commands
            and convert them into structured action plans for a humanoid robot.

            The robot has these capabilities:
            - Navigation to specific locations
            - Object detection and recognition
            - Basic manipulation (in simulation)
            - Voice response

            Conversation context:
            {context}

            User command: "{command}"

            Please respond with a JSON object containing:
            {{
                "intent": "navigation|manipulation|detection|conversation|other",
                "target_location": [x, y, z] coordinates if navigation,
                "target_object": "object_name" if manipulation/detection,
                "action_sequence": [
                    {{
                        "action": "navigate|detect|grasp|speak|wait",
                        "parameters": {{"key": "value"}}
                    }}
                ],
                "confidence": 0.0 to 1.0
            }}

            If the command is unclear or impossible, return an empty JSON object: {{}}
            """

            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=500,
                temperature=0.3
            )

            response_text = response.choices[0].message.content

            # Extract JSON from response
            start_idx = response_text.find('{')
            end_idx = response_text.rfind('}') + 1

            if start_idx != -1 and end_idx != 0:
                json_str = response_text[start_idx:end_idx]
                action_plan = json.loads(json_str)

                # Add to context
                self.conversation_context.append({
                    'speaker': 'system',
                    'text': json_str,
                    'timestamp': self.get_clock().now().to_msg()
                })

                return action_plan

        except json.JSONDecodeError:
            self.get_logger().error("Could not parse LLM response as JSON")
        except Exception as e:
            self.get_logger().error(f"LLM processing error: {e}")

        return None

    def build_context(self, current_command: str) -> str:
        """
        Build context from conversation history and environment
        """
        context_parts = []

        # Add recent conversation history
        recent_context = self.conversation_context[-5:]  # Last 5 exchanges
        for exchange in recent_context:
            speaker = exchange['speaker']
            text = exchange['text']
            context_parts.append(f"{speaker}: {text}")

        # Add known object locations
        if self.object_locations:
            obj_info = "Known objects: " + ", ".join([f"{name} at {loc}" for name, loc in self.object_locations.items()])
            context_parts.append(obj_info)

        # Add current robot state
        if self.robot_state:
            state_info = f"Robot state: {self.robot_state}"
            context_parts.append(state_info)

        return "\n".join(context_parts)

    def execute_action_plan(self, action_plan: Dict):
        """
        Execute the action plan generated by LLM
        """
        intent = action_plan.get('intent', 'unknown')
        action_sequence = action_plan.get('action_sequence', [])

        self.get_logger().info(f"Executing action plan with intent: {intent}")

        # Execute each action in sequence
        for i, action in enumerate(action_sequence):
            action_type = action.get('action', 'unknown')
            parameters = action.get('parameters', {})

            if action_type == 'navigate':
                self.execute_navigation_action(parameters)
            elif action_type == 'detect':
                self.execute_detection_action(parameters)
            elif action_type == 'grasp':
                self.execute_grasp_action(parameters)
            elif action_type == 'speak':
                self.execute_speak_action(parameters)
            elif action_type == 'wait':
                self.execute_wait_action(parameters)

            # Small delay between actions
            self.get_logger().info(f"Completed action {i+1}/{len(action_sequence)}: {action_type}")

    def execute_navigation_action(self, parameters: Dict):
        """
        Execute navigation action
        """
        target_pos = parameters.get('target_position', [0.0, 0.0, 0.0])

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = target_pos[0]
        goal_msg.pose.pose.position.y = target_pos[1]
        goal_msg.pose.pose.position.z = target_pos[2]
        goal_msg.pose.pose.orientation.w = 1.0  # Default orientation

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        send_goal_future.add_done_callback(self.navigation_goal_response_callback)

    def execute_detection_action(self, parameters: Dict):
        """
        Execute object detection action
        """
        target_object = parameters.get('target_object', 'unknown')
        self.get_logger().info(f"Looking for object: {target_object}")

        # In practice, this would trigger perception system
        # For now, we'll simulate detection
        detected = self.simulate_object_detection(target_object)

        if detected:
            response_msg = String()
            response_msg.data = f"I found the {target_object}."
            self.response_pub.publish(response_msg)
        else:
            response_msg = String()
            response_msg.data = f"I couldn't find the {target_object}."
            self.response_pub.publish(response_msg)

    def execute_grasp_action(self, parameters: Dict):
        """
        Execute grasp action (simulation only)
        """
        target_object = parameters.get('target_object', 'unknown')
        self.get_logger().info(f"Attempting to grasp: {target_object}")

        # Simulate grasp action
        success = self.simulate_grasp_action(target_object)

        response_msg = String()
        if success:
            response_msg.data = f"I successfully grasped the {target_object}."
        else:
            response_msg.data = f"I couldn't grasp the {target_object}."
        self.response_pub.publish(response_msg)

    def execute_speak_action(self, parameters: Dict):
        """
        Execute speak action
        """
        text = parameters.get('text', 'Hello')
        self.get_logger().info(f"Speaking: {text}")

        response_msg = String()
        response_msg.data = text
        self.response_pub.publish(response_msg)

    def execute_wait_action(self, parameters: Dict):
        """
        Execute wait action
        """
        duration = parameters.get('duration', 1.0)
        self.get_logger().info(f"Waiting for {duration} seconds")

        # In ROS 2, we typically don't block, so we'd use a timer
        # For this example, we'll just log the action

    def navigation_feedback_callback(self, feedback_msg):
        """
        Handle navigation feedback
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation progress: {feedback.distance_remaining:.2f}m remaining')

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

    def navigation_result_callback(self, future):
        """
        Handle navigation result
        """
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            response_msg = String()
            response_msg.data = "I have reached the destination."
            self.response_pub.publish(response_msg)
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')
            response_msg = String()
            response_msg.data = "I couldn't reach the destination."
            self.response_pub.publish(response_msg)

    def simulate_object_detection(self, target_object: str) -> bool:
        """
        Simulate object detection (in practice, use perception system)
        """
        # Simulate detection success based on object type
        import random
        return random.random() > 0.3  # 70% success rate

    def simulate_grasp_action(self, target_object: str) -> bool:
        """
        Simulate grasp action (in practice, use manipulation system)
        """
        # Simulate grasp success
        import random
        return random.random() > 0.2  # 80% success rate

def main(args=None):
    rclpy.init(args=args)
    node = LanguageToActionNode()

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

Now let me create the "llm-integration.md" file that should also be part of week 12 according to the implementation plan:
