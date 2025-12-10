---
title: Week 12 - LLM Integration
description: Integrating large language models for cognitive planning and natural interaction
sidebar_position: 2
---

# Week 12: LLM Integration

## Learning Objectives
- Integrate large language models (LLMs) for cognitive planning in robotics
- Implement context-aware language processing for robotics applications
- Design memory systems for maintaining conversation and task context
- Create multimodal interfaces combining language with perception and action
- Optimize LLM usage for real-time robotic applications

## Prerequisites Check
- Understanding of basic NLP concepts
- Experience with API integration (OpenAI, Hugging Face, etc.)
- Knowledge of ROS 2 service and action patterns
- Familiarity with context management in conversational systems

## Theoretical Concepts: LLMs in Robotics

### Cognitive Architecture for Language-Enabled Robots

Large Language Models serve as the "cognitive layer" in modern robotic systems, providing:

1. **High-Level Reasoning**: Abstract thinking and planning capabilities
2. **Natural Language Understanding**: Processing human instructions and queries
3. **World Knowledge**: Access to vast amounts of general knowledge
4. **Context Management**: Maintaining conversation and task state
5. **Action Selection**: Choosing appropriate robot behaviors

### Multimodal LLM Integration

Modern robotics applications require LLMs to process multiple modalities:

- **Text**: Natural language commands and responses
- **Vision**: Image and video input for grounding language in perception
- **Action**: Robot behaviors and environmental changes
- **Context**: Historical interactions and environmental state

### Context and Memory Systems

For effective human-robot interaction, LLM-based systems need:

- **Short-term Memory**: Recent conversation and task context
- **Long-term Memory**: Persistent knowledge about the environment and user preferences
- **Episodic Memory**: Specific interaction episodes for learning
- **Semantic Memory**: General knowledge about the world and tasks

## Step-by-Step Tutorials: LLM Integration

### Advanced LLM Integration with Context Management

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from openai import OpenAI
import json
import asyncio
import threading
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass
from datetime import datetime
import pickle
import os

@dataclass
class ConversationTurn:
    """
    Data structure for conversation turns
    """
    speaker: str  # 'user' or 'robot'
    text: str
    timestamp: datetime
    context: Dict[str, Any] = None
    embeddings: List[float] = None

@dataclass
class RobotState:
    """
    Data structure for robot state
    """
    position: List[float]
    orientation: List[float]
    battery_level: float
    active_tasks: List[str]
    detected_objects: List[Dict[str, Any]]

class LLMContextManager:
    """
    Advanced context management for LLM integration
    """

    def __init__(self, max_context_length: int = 20, memory_file: str = "robot_memory.pkl"):
        self.max_context_length = max_context_length
        self.conversation_history: List[ConversationTurn] = []
        self.robot_state: RobotState = None
        self.environment_state: Dict[str, Any] = {}
        self.memory_file = memory_file
        self.load_memory()

    def add_conversation_turn(self, speaker: str, text: str, context: Dict[str, Any] = None):
        """
        Add a turn to the conversation history
        """
        turn = ConversationTurn(
            speaker=speaker,
            text=text,
            timestamp=datetime.now(),
            context=context
        )
        self.conversation_history.append(turn)

        # Maintain maximum context length
        if len(self.conversation_history) > self.max_context_length:
            self.conversation_history = self.conversation_history[-self.max_context_length:]

    def get_recent_context(self, num_turns: int = 5) -> str:
        """
        Get recent conversation context as a string
        """
        recent_turns = self.conversation_history[-num_turns:]
        context_strs = []

        for turn in recent_turns:
            speaker_prefix = "User" if turn.speaker == "user" else "Robot"
            context_strs.append(f"{speaker_prefix}: {turn.text}")

        return "\n".join(context_strs)

    def update_robot_state(self, state: RobotState):
        """
        Update the robot's state in context
        """
        self.robot_state = state

    def update_environment_state(self, env_state: Dict[str, Any]):
        """
        Update the environment state in context
        """
        self.environment_state.update(env_state)

    def build_complete_context(self, current_command: str) -> str:
        """
        Build complete context for LLM processing
        """
        context_parts = []

        # Add conversation history
        if self.conversation_history:
            context_parts.append("Recent conversation:")
            context_parts.append(self.get_recent_context())
            context_parts.append("")

        # Add robot state
        if self.robot_state:
            context_parts.append("Current robot state:")
            context_parts.append(f"- Position: {self.robot_state.position}")
            context_parts.append(f"- Battery: {self.robot_state.battery_level}%")
            context_parts.append(f"- Active tasks: {self.robot_state.active_tasks}")
            context_parts.append("")

        # Add environment state
        if self.environment_state:
            context_parts.append("Environment information:")
            for key, value in self.environment_state.items():
                context_parts.append(f"- {key}: {value}")
            context_parts.append("")

        # Add current command
        context_parts.append(f"Current user command: {current_command}")

        return "\n".join(context_parts)

    def save_memory(self):
        """
        Save context and memory to file
        """
        memory_data = {
            'conversation_history': self.conversation_history,
            'robot_state': self.robot_state,
            'environment_state': self.environment_state
        }

        with open(self.memory_file, 'wb') as f:
            pickle.dump(memory_data, f)

    def load_memory(self):
        """
        Load context and memory from file
        """
        if os.path.exists(self.memory_file):
            try:
                with open(self.memory_file, 'rb') as f:
                    memory_data = pickle.load(f)

                self.conversation_history = memory_data.get('conversation_history', [])
                self.robot_state = memory_data.get('robot_state')
                self.environment_state = memory_data.get('environment_state', {})
            except Exception as e:
                print(f"Error loading memory: {e}")

class AdvancedLLMIntegrationNode(Node):
    """
    Advanced LLM integration with context management
    """

    def __init__(self):
        super().__init__('advanced_llm_integration_node')

        # Initialize OpenAI client
        self.openai_client = OpenAI(api_key='your-api-key-here')  # Replace with your key

        # Context manager
        self.context_manager = LLMContextManager()

        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/natural_language_command', self.command_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, 10
        )

        # Publishers
        self.response_pub = self.create_publisher(String, '/system_response', 10)
        self.action_plan_pub = self.create_publisher(String, '/action_plan', 10)

        # State tracking
        self.latest_image = None
        self.system_active = True

        # Memory management timer
        self.memory_save_timer = self.create_timer(30.0, self.save_memory_periodically)

        self.get_logger().info("Advanced LLM Integration node initialized")

    def command_callback(self, msg):
        """
        Process natural language command with full context
        """
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Add to conversation history
        self.context_manager.add_conversation_turn('user', command)

        # Process with LLM using full context
        action_plan = self.process_command_with_context(command)

        if action_plan:
            self.execute_action_plan(action_plan)
        else:
            response_msg = String()
            response_msg.data = "I'm not sure how to help with that. Could you provide more details?"
            self.response_pub.publish(response_msg)

    def image_callback(self, msg):
        """
        Process incoming images for visual context
        """
        self.latest_image = msg
        # In practice, this would trigger image analysis
        # and update environment state

    def process_command_with_context(self, command: str) -> Optional[Dict]:
        """
        Process command using LLM with full context
        """
        try:
            # Build complete context
            full_context = self.context_manager.build_complete_context(command)

            # Create a more sophisticated prompt
            prompt = f"""
            You are an intelligent robot assistant with access to the following information:

            {full_context}

            Based on the user's command and the current context, please provide a structured action plan.
            Consider the robot's current state, environment, and conversation history.

            Respond with a JSON object containing:
            {{
                "intent": "navigation|manipulation|detection|conversation|question_answering|other",
                "confidence": 0.0 to 1.0,
                "required_capabilities": ["list", "of", "required", "robot", "capabilities"],
                "action_sequence": [
                    {{
                        "action_type": "navigate|detect|grasp|speak|wait|analyze_image",
                        "parameters": {{"key": "value"}},
                        "confidence": 0.0 to 1.0
                    }}
                ],
                "reasoning": "Brief explanation of your reasoning",
                "follow_up_questions": ["list", "of", "clarifying", "questions", "if", "needed"]
            }}

            If the command is unclear, respond with clarifying questions.
            If the task is impossible given current state, explain why.
            """

            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=800,
                temperature=0.4
            )

            response_text = response.choices[0].message.content

            # Extract JSON from response
            start_idx = response_text.find('{')
            end_idx = response_text.rfind('}') + 1

            if start_idx != -1 and end_idx != 0:
                json_str = response_text[start_idx:end_idx]
                action_plan = json.loads(json_str)

                # Add to conversation history
                self.context_manager.add_conversation_turn('system', json_str)

                return action_plan

        except json.JSONDecodeError:
            self.get_logger().error("Could not parse LLM response as JSON")
        except Exception as e:
            self.get_logger().error(f"LLM processing error: {e}")

        return None

    def execute_action_plan(self, action_plan: Dict):
        """
        Execute the action plan with error handling and monitoring
        """
        intent = action_plan.get('intent', 'unknown')
        action_sequence = action_plan.get('action_sequence', [])
        confidence = action_plan.get('confidence', 0.5)

        self.get_logger().info(f"Executing {intent} action plan with {confidence:.2f} confidence")

        # Check if confidence is high enough to proceed
        if confidence < 0.6:
            self.get_logger().warn(f"Low confidence ({confidence:.2f}) for action plan")

        # Execute each action in sequence
        successful_actions = 0
        for i, action in enumerate(action_sequence):
            action_type = action.get('action_type', 'unknown')
            parameters = action.get('parameters', {})
            action_confidence = action.get('confidence', 1.0)

            if action_confidence < 0.5:
                self.get_logger().warn(f"Low confidence ({action_confidence:.2f}) for {action_type} action")

            success = self.execute_single_action(action_type, parameters)

            if success:
                successful_actions += 1
                self.get_logger().info(f"Action {i+1}/{len(action_sequence)} ({action_type}) completed successfully")
            else:
                self.get_logger().error(f"Action {i+1}/{len(action_sequence)} ({action_type}) failed")
                # Optionally, stop execution on failure or continue based on requirements

        # Publish completion status
        completion_msg = String()
        completion_msg.data = f"Completed {successful_actions}/{len(action_sequence)} actions for {intent} task"
        self.response_pub.publish(completion_msg)

    def execute_single_action(self, action_type: str, parameters: Dict) -> bool:
        """
        Execute a single action and return success status
        """
        try:
            if action_type == 'navigate':
                return self.execute_navigation_action(parameters)
            elif action_type == 'detect':
                return self.execute_detection_action(parameters)
            elif action_type == 'speak':
                return self.execute_speak_action(parameters)
            elif action_type == 'analyze_image':
                return self.execute_image_analysis_action(parameters)
            elif action_type == 'wait':
                return self.execute_wait_action(parameters)
            else:
                self.get_logger().warn(f"Unknown action type: {action_type}")
                return False
        except Exception as e:
            self.get_logger().error(f"Error executing {action_type} action: {e}")
            return False

    def execute_navigation_action(self, parameters: Dict) -> bool:
        """
        Execute navigation action
        """
        target_pos = parameters.get('target_position', [0.0, 0.0, 0.0])

        # In practice, this would use navigation stack
        # For this example, we'll just log the action
        self.get_logger().info(f"Planning navigation to: {target_pos}")

        # Publish action for navigation system
        action_msg = String()
        action_msg.data = json.dumps({
            'action': 'navigate',
            'target': target_pos
        })
        self.action_plan_pub.publish(action_msg)

        return True  # Simulate success

    def execute_detection_action(self, parameters: Dict) -> bool:
        """
        Execute detection action
        """
        target_object = parameters.get('target_object', 'unknown')
        self.get_logger().info(f"Looking for object: {target_object}")

        # Publish action for perception system
        action_msg = String()
        action_msg.data = json.dumps({
            'action': 'detect',
            'target_object': target_object
        })
        self.action_plan_pub.publish(action_msg)

        return True  # Simulate success

    def execute_speak_action(self, parameters: Dict) -> bool:
        """
        Execute speak action
        """
        text = parameters.get('text', 'Hello')
        self.get_logger().info(f"Speaking: {text}")

        response_msg = String()
        response_msg.data = text
        self.response_pub.publish(response_msg)

        return True

    def execute_image_analysis_action(self, parameters: Dict) -> bool:
        """
        Execute image analysis action
        """
        analysis_type = parameters.get('analysis_type', 'description')
        self.get_logger().info(f"Analyzing image for: {analysis_type}")

        # In practice, this would process the latest image
        # For this example, we'll just log the action
        if self.latest_image:
            self.get_logger().info("Image analysis would process latest image here")
        else:
            self.get_logger().warn("No image available for analysis")

        return True

    def execute_wait_action(self, parameters: Dict) -> bool:
        """
        Execute wait action
        """
        duration = parameters.get('duration', 1.0)
        self.get_logger().info(f"Waiting for {duration} seconds")

        # In ROS 2, we typically don't block, so we'd use a timer
        # For this example, we'll just log the action

        return True

    def save_memory_periodically(self):
        """
        Periodically save memory to file
        """
        try:
            self.context_manager.save_memory()
            self.get_logger().info("Memory saved successfully")
        except Exception as e:
            self.get_logger().error(f"Error saving memory: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedLLMIntegrationNode()

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

Now let me update the sidebar to include these new week 12 files:
