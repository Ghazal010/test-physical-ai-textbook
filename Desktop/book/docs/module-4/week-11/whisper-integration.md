---
title: Week 11 - Whisper Integration
description: Integrating OpenAI Whisper for voice recognition in robotics
sidebar_position: 2
---

# Week 11: Whisper Integration

## Learning Objectives
- Understand OpenAI Whisper for voice recognition
- Build speech-to-text pipelines for robotics
- Integrate voice commands with ROS 2
- Create voice-controlled robot interactions

## Prerequisites Check
- Completion of Week 10 (Navigation and Planning)
- Understanding of ROS 2 communication patterns
- Basic knowledge of audio processing concepts
- Python programming skills

## Theoretical Concepts: Voice Recognition

### Introduction to OpenAI Whisper

OpenAI Whisper is a robust automatic speech recognition (ASR) system trained on a vast dataset of diverse audio. For robotics applications, Whisper provides accurate transcription of human speech, enabling voice-controlled robot interactions.

Key features of Whisper for robotics:
- Multilingual support
- Robustness to accents and background noise
- Timestamped word-level alignment
- Speaker identification capabilities

### Speech-to-Text Pipeline Architecture

For robotics applications, a typical speech-to-text pipeline includes:

1. **Audio Capture**: Microphone array or single microphone input
2. **Preprocessing**: Noise reduction, audio normalization
3. **Transcription**: Whisper model processing
4. **Post-processing**: Command parsing and validation
5. **Action Mapping**: Converting text to robot commands

## Step-by-Step Tutorials: Whisper Integration

### Setting up Whisper for Robotics

```python
import rospy
import whisper
import torch
import pyaudio
import wave
import numpy as np
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

class WhisperRobotInterface:
    """
    Interface for integrating Whisper ASR with ROS 2 for voice-controlled robotics
    """

    def __init__(self):
        # Initialize Whisper model
        rospy.init_node('whisper_robot_interface')

        # Load Whisper model (using medium model for balance of accuracy and speed)
        rospy.loginfo("Loading Whisper model...")
        self.model = whisper.load_model("medium")

        # Audio parameters
        self.rate = 16000  # Sample rate
        self.chunk = 1024  # Buffer size
        self.format = pyaudio.paInt16
        self.channels = 1

        # ROS publishers and subscribers
        self.audio_sub = rospy.Subscriber('/audio', AudioData, self.audio_callback)
        self.command_pub = rospy.Publisher('/voice_command', String, queue_size=10)

        # Audio buffer for processing
        self.audio_buffer = []
        self.buffer_size = 4  # seconds of audio to process

        rospy.loginfo("Whisper Robot Interface initialized")

    def audio_callback(self, audio_data):
        """
        Callback for audio data from microphone
        """
        # Convert audio data to numpy array
        audio_array = np.frombuffer(audio_data.data, dtype=np.int16)
        self.audio_buffer.extend(audio_array)

        # Limit buffer size to prevent memory issues
        max_buffer_size = self.rate * self.buffer_size  # samples for buffer_size seconds
        if len(self.audio_buffer) > max_buffer_size:
            self.audio_buffer = self.audio_buffer[-max_buffer_size:]

    def transcribe_audio(self, audio_segment):
        """
        Transcribe audio segment using Whisper
        """
        # Convert to float32 and normalize
        audio_float = np.array(audio_segment, dtype=np.float32) / 32768.0

        # Transcribe using Whisper
        result = self.model.transcribe(audio_float, fp16=torch.cuda.is_available())

        return result["text"].strip()

    def process_voice_command(self):
        """
        Process accumulated audio buffer and extract voice commands
        """
        if len(self.audio_buffer) < self.rate * 0.5:  # At least 0.5 seconds of audio
            return None

        # Transcribe the audio
        transcription = self.transcribe_audio(self.audio_buffer)

        # Clear the buffer after processing
        self.audio_buffer.clear()

        if transcription:
            rospy.loginfo(f"Transcribed: {transcription}")

            # Publish the transcription as a voice command
            cmd_msg = String()
            cmd_msg.data = transcription
            self.command_pub.publish(cmd_msg)

            return transcription

        return None

    def run(self):
        """
        Main loop for processing audio and transcribing speech
        """
        rate = rospy.Rate(10)  # Process audio every 100ms

        while not rospy.is_shutdown():
            self.process_voice_command()
            rate.sleep()

def main():
    try:
        interface = WhisperRobotInterface()
        interface.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Whisper Robot Interface shutdown")

if __name__ == '__main__':
    main()
```

### Advanced Whisper Configuration for Robotics

```python
import whisper
import torch
from dataclasses import dataclass
from typing import Optional, Union

@dataclass
class WhisperConfig:
    """
    Configuration for Whisper ASR in robotics applications
    """
    model_size: str = "medium"  # tiny, base, small, medium, large
    language: str = "en"        # Language code
    temperature: float = 0.0    # Temperature for sampling
    patience: float = 1.0       # Patience for beam search
    suppress_tokens: str = "-1" # Tokens to suppress
    initial_prompt: Optional[str] = None  # Initial prompt for context
    condition_on_previous_text: bool = True  # Condition on previous text
    vad_filter: bool = True     # Voice activity detection filter

class AdvancedWhisperInterface:
    """
    Advanced Whisper interface with configurable parameters for robotics
    """

    def __init__(self, config: WhisperConfig):
        self.config = config
        self.model = whisper.load_model(config.model_size)
        self.options = whisper.DecodingOptions(
            language=config.language,
            temperature=config.temperature,
            patience=config.patience,
            suppress_tokens=config.suppress_tokens,
            initial_prompt=config.initial_prompt,
            condition_on_previous_text=config.condition_on_previous_text,
            vad_filter=config.vad_filter
        )

    def transcribe_with_options(self, audio: Union[str, np.ndarray]) -> str:
        """
        Transcribe audio with custom options
        """
        if isinstance(audio, str):
            # Load audio from file
            audio = whisper.load_audio(audio)

        # Pad/trim audio to fit model requirements
        audio = whisper.pad_or_trim(audio)

        # Convert to log-Mel spectrogram
        mel = whisper.log_mel_spectrogram(audio).to(self.model.device)

        # Decode audio
        result = self.model.decode(mel, self.options)

        return result.text
```

## Code Examples with Explanations

### Real-time Voice Command Processing

```python
import threading
import time
from queue import Queue
import numpy as np

class RealTimeVoiceProcessor:
    """
    Real-time voice processing system for robotics
    """

    def __init__(self, whisper_interface):
        self.whisper_interface = whisper_interface
        self.audio_queue = Queue()
        self.command_queue = Queue()
        self.is_running = False

        # Lock for thread safety
        self.processing_lock = threading.Lock()

    def start_processing(self):
        """
        Start real-time audio processing
        """
        self.is_running = True

        # Start audio processing thread
        self.processing_thread = threading.Thread(target=self._process_audio_loop)
        self.processing_thread.start()

        # Start command processing thread
        self.command_thread = threading.Thread(target=self._process_commands_loop)
        self.command_thread.start()

    def _process_audio_loop(self):
        """
        Continuously process audio from queue
        """
        while self.is_running:
            if not self.audio_queue.empty():
                audio_segment = self.audio_queue.get()

                with self.processing_lock:
                    try:
                        transcription = self.whisper_interface.transcribe_audio(audio_segment)
                        if transcription:
                            self.command_queue.put(transcription)
                    except Exception as e:
                        rospy.logerr(f"Error in audio processing: {e}")

            time.sleep(0.01)  # Small delay to prevent busy waiting

    def _process_commands_loop(self):
        """
        Process transcribed commands
        """
        while self.is_running:
            if not self.command_queue.empty():
                command = self.command_queue.get()

                # Parse and execute command
                self._execute_robot_command(command)

            time.sleep(0.05)  # Process commands at 20Hz

    def _execute_robot_command(self, command: str):
        """
        Parse and execute robot command from voice input
        """
        # Simple command parser - in practice, this would be more sophisticated
        command_lower = command.lower()

        if "move forward" in command_lower:
            rospy.loginfo("Moving robot forward")
            # Publish movement command
        elif "turn left" in command_lower:
            rospy.loginfo("Turning robot left")
            # Publish turn command
        elif "stop" in command_lower:
            rospy.loginfo("Stopping robot")
            # Publish stop command
        else:
            rospy.loginfo(f"Unknown command: {command}")

    def stop_processing(self):
        """
        Stop real-time processing
        """
        self.is_running = False
        if hasattr(self, 'processing_thread'):
            self.processing_thread.join()
        if hasattr(self, 'command_thread'):
            self.command_thread.join()
```

### Voice Command Grammar for Robotics

```python
import re
from typing import Dict, List, Tuple

class VoiceCommandGrammar:
    """
    Grammar for parsing voice commands in robotics
    """

    def __init__(self):
        # Define command patterns
        self.command_patterns = {
            'move': [
                r'move\s+(forward|backward|up|down|left|right)',
                r'go\s+(forward|backward|up|down|left|right)',
                r'move\s+to\s+(?P<direction>forward|backward|up|down|left|right)'
            ],
            'turn': [
                r'turn\s+(left|right|around)',
                r'rotate\s+(left|right)',
                r'pivot\s+(left|right)'
            ],
            'stop': [
                r'stop',
                r'hold',
                r'freeze',
                r'pause'
            ],
            'navigation': [
                r'go\s+to\s+(?P<location>\w+)',
                r'navigate\s+to\s+(?P<location>\w+)',
                r'move\s+to\s+(?P<location>\w+)'
            ],
            'action': [
                r'pick\s+up\s+(?P<object>\w+)',
                r'grab\s+(?P<object>\w+)',
                r'get\s+(?P<object>\w+)',
                r'place\s+(?P<object>\w+)\s+on\s+(?P<surface>\w+)',
                r'put\s+(?P<object>\w+)\s+on\s+(?P<surface>\w+)'
            ]
        }

    def parse_command(self, transcription: str) -> Tuple[str, Dict]:
        """
        Parse voice command and extract parameters
        """
        transcription = transcription.strip().lower()

        for command_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, transcription)
                if match:
                    return command_type, match.groupdict()

        return 'unknown', {}

    def execute_parsed_command(self, command_type: str, params: Dict):
        """
        Execute parsed command
        """
        if command_type == 'move':
            direction = params.get('direction', 'forward')
            rospy.loginfo(f"Moving robot {direction}")
            # Publish appropriate movement command
        elif command_type == 'turn':
            direction = params.get('direction', 'left')
            rospy.loginfo(f"Turning robot {direction}")
            # Publish turn command
        elif command_type == 'stop':
            rospy.loginfo("Stopping robot")
            # Publish stop command
        elif command_type == 'navigation':
            location = params.get('location', 'default')
            rospy.loginfo(f"Navigating to {location}")
            # Publish navigation command
        elif command_type == 'action':
            rospy.loginfo(f"Performing action with parameters: {params}")
            # Execute complex action based on parameters
        else:
            rospy.loginfo(f"Unknown command type: {command_type}")
```

## Hands-On Exercises: Voice Command Implementation

### Exercise 1: Basic Voice Command Recognition

Implement a simple voice command recognition system:

```python
class BasicVoiceCommandSystem:
    """
    Basic voice command system for robotics
    """

    def __init__(self):
        self.whisper_model = whisper.load_model("base")
        self.command_history = []
        self.active_commands = set([
            "move forward", "move backward", "turn left", "turn right",
            "stop", "go home", "find object", "follow me"
        ])

    def recognize_command(self, audio_data):
        """
        Recognize command from audio data
        """
        # Convert audio to numpy array and normalize
        audio_array = np.frombuffer(audio_data, dtype=np.int16)
        audio_float = audio_array.astype(np.float32) / 32768.0

        # Transcribe with Whisper
        result = self.whisper_model.transcribe(audio_float)
        transcription = result["text"].strip().lower()

        # Check if transcription matches known commands
        recognized_command = self._match_command(transcription)

        if recognized_command:
            self.command_history.append({
                'transcription': result["text"],
                'command': recognized_command,
                'timestamp': rospy.Time.now()
            })

            return recognized_command
        else:
            return None

    def _match_command(self, transcription):
        """
        Match transcription to known commands
        """
        # Simple fuzzy matching - in practice, use more sophisticated NLP
        for known_cmd in self.active_commands:
            if known_cmd in transcription:
                return known_cmd

        # Try to find similar commands using edit distance
        for known_cmd in self.active_commands:
            if self._similarity(transcription, known_cmd) > 0.7:
                return known_cmd

        return None

    def _similarity(self, s1, s2):
        """
        Calculate similarity between two strings
        """
        # Simple implementation - in practice, use proper string similarity algorithm
        s1_words = set(s1.split())
        s2_words = set(s2.split())

        intersection = s1_words.intersection(s2_words)
        union = s1_words.union(s2_words)

        return len(intersection) / len(union) if union else 0

# Example usage
def test_voice_recognition():
    """
    Test function for voice recognition
    """
    system = BasicVoiceCommandSystem()

    # Simulate some audio data (in practice, this would come from microphone)
    # For testing, we'll simulate with known phrases
    test_phrases = [
        "please move forward",
        "turn left now",
        "stop immediately",
        "go home please"
    ]

    for phrase in test_phrases:
        # In real implementation, this would be actual audio data
        # Here we simulate the audio-to-text conversion
        recognized = system._match_command(phrase.lower())
        print(f"Input: '{phrase}' -> Recognized: {recognized}")
```

### Exercise 2: Advanced Voice Command Processing

Create an advanced system that can handle complex voice commands:

```python
import json
from dataclasses import dataclass
from typing import Optional

@dataclass
class CommandContext:
    """
    Context for voice command processing
    """
    robot_position: Tuple[float, float, float]
    robot_orientation: Tuple[float, float, float, float]  # quaternion
    detected_objects: List[Dict]
    current_task: str
    user_preferences: Dict

class AdvancedVoiceProcessor:
    """
    Advanced voice processing system with context awareness
    """

    def __init__(self):
        self.whisper_model = whisper.load_model("medium")
        self.context = self._initialize_context()
        self.command_parser = VoiceCommandGrammar()

    def _initialize_context(self):
        """
        Initialize command context
        """
        return CommandContext(
            robot_position=(0.0, 0.0, 0.0),
            robot_orientation=(0.0, 0.0, 0.0, 1.0),
            detected_objects=[],
            current_task="idle",
            user_preferences={}
        )

    def process_voice_command(self, audio_data):
        """
        Process voice command with context awareness
        """
        # Transcribe audio
        transcription = self._transcribe_audio(audio_data)

        if not transcription:
            return None

        # Parse command with context
        command_type, params = self.command_parser.parse_command(transcription)

        if command_type == 'unknown':
            # Try to use LLM to interpret the command
            interpreted_command = self._interpret_command(transcription)
            if interpreted_command:
                command_type, params = interpreted_command
            else:
                rospy.loginfo(f"Could not interpret command: {transcription}")
                return None

        # Execute command with context
        result = self._execute_command_with_context(command_type, params)

        return {
            'transcription': transcription,
            'command_type': command_type,
            'parameters': params,
            'execution_result': result
        }

    def _transcribe_audio(self, audio_data):
        """
        Transcribe audio using Whisper
        """
        audio_array = np.frombuffer(audio_data, dtype=np.int16)
        audio_float = audio_array.astype(np.float32) / 32768.0

        result = self.whisper_model.transcribe(audio_float)
        return result["text"].strip()

    def _interpret_command(self, transcription):
        """
        Use additional AI to interpret complex commands
        """
        # This is a simplified implementation
        # In practice, this would use a more sophisticated NLP model
        # or even an LLM API call to interpret complex commands

        # For now, return None to indicate interpretation failed
        return None

    def _execute_command_with_context(self, command_type, params):
        """
        Execute command considering current context
        """
        # Update context based on command
        if command_type == 'navigation':
            location = params.get('location', 'unknown')
            # Update robot position after navigation
            self.context.current_task = f"navigating_to_{location}"

        # Execute the actual command (publish to ROS topics)
        self._publish_command(command_type, params)

        return "success"

    def _publish_command(self, command_type, params):
        """
        Publish command to ROS system
        """
        # Implementation would publish to appropriate ROS topics
        # based on command type and parameters
        rospy.loginfo(f"Publishing command: {command_type} with params: {params}")
```

## Common Pitfalls and Solutions

### Pitfall 1: Audio Quality Issues
**Problem**: Poor audio quality leads to inaccurate transcriptions.

**Solution**:
- Use directional microphones to reduce background noise
- Implement noise reduction algorithms
- Use audio preprocessing techniques

```python
def preprocess_audio(audio_data, sample_rate=16000):
    """
    Preprocess audio data for better Whisper performance
    """
    import scipy.signal as signal

    # Apply noise reduction
    # This is a simple example - in practice, use more sophisticated methods
    b, a = signal.butter(8, 0.04, 'highpass')  # High-pass filter to remove low-frequency noise
    filtered_audio = signal.filtfilt(b, a, audio_data)

    # Normalize audio
    max_val = np.max(np.abs(filtered_audio))
    if max_val > 0:
        filtered_audio = filtered_audio / max_val

    return filtered_audio
```

### Pitfall 2: Real-time Processing Latency
**Problem**: Whisper processing can be slow, causing delays in real-time applications.

**Solution**:
- Use smaller Whisper models for faster processing
- Implement audio buffering and batch processing
- Use GPU acceleration when available

```python
class OptimizedWhisperProcessor:
    """
    Optimized Whisper processor for real-time robotics applications
    """

    def __init__(self):
        # Use smaller model for faster processing
        self.model = whisper.load_model("base")

        # Audio buffer for batch processing
        self.audio_buffer = np.array([])
        self.buffer_duration = 2.0  # Process every 2 seconds

    def add_audio_chunk(self, chunk):
        """
        Add audio chunk to buffer
        """
        chunk_array = np.frombuffer(chunk, dtype=np.int16).astype(np.float32) / 32768.0
        self.audio_buffer = np.concatenate([self.audio_buffer, chunk_array])

        # Process if buffer is full
        if len(self.audio_buffer) >= self.buffer_duration * 16000:  # 16kHz sample rate
            return self._process_buffer()
        return None

    def _process_buffer(self):
        """
        Process accumulated audio buffer
        """
        if len(self.audio_buffer) == 0:
            return None

        # Transcribe the buffer
        result = self.model.transcribe(self.audio_buffer)

        # Clear buffer
        self.audio_buffer = np.array([])

        return result["text"].strip()
```

### Pitfall 3: Context Loss in Conversations
**Problem**: Voice commands lose context when processing is done in isolation.

**Solution**:
- Maintain conversation context
- Use session-based processing
- Implement dialogue management

## Review Questions

1. What are the key advantages of OpenAI Whisper for robotics applications?
2. How does the Whisper speech-to-text pipeline integrate with ROS 2?
3. What are the main challenges in real-time voice processing for robots?
4. How can you improve voice recognition accuracy in noisy environments?
5. What is the role of context in voice command interpretation for robots?

## Project Assignment: Voice Command System

Create a complete voice command system for a robot that:
1. Captures audio from a microphone array
2. Processes audio using Whisper for speech-to-text conversion
3. Parses commands using a grammar-based approach
4. Executes appropriate robot actions based on recognized commands
5. Handles errors gracefully and provides feedback
6. Maintains conversation context for multi-turn interactions

Your system should be able to:
- Recognize basic navigation commands (move forward, turn, stop, etc.)
- Process object manipulation commands
- Handle ambiguous or unclear commands
- Provide audio feedback to the user
- Log command history for debugging

## Further Resources

- [OpenAI Whisper Documentation](https://github.com/openai/whisper)
- [Audio Processing in Robotics](https://www.robotics.org/content-detail.cfm/Industrial-Robotics-Featured-Article/Audio-Processing-in-Robotics/981)
- [ROS Audio Common Packages](http://wiki.ros.org/audio_common)
- [Speech Recognition in Robotics Research](https://arxiv.org/list/cs.RO/recent)
- [PyAudio Documentation](https://pyaudio.readthedocs.io/)

:::warning
Be mindful of privacy considerations when implementing voice recognition systems. Always inform users when voice data is being captured and processed, and implement appropriate data handling practices.
:::