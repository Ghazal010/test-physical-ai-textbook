---
title: Troubleshooting
description: Common issues and solutions for the Physical AI & Humanoid Robotics course
sidebar_position: 2
---

# Troubleshooting

## Overview

This troubleshooting guide addresses common issues encountered during the Physical AI & Humanoid Robotics course. Each section provides diagnostic steps and solutions for specific problems.

## Common Installation Issues

### ROS 2 Installation Problems

#### Issue: Package Not Found During Installation
**Symptoms**: `apt install ros-humble-desktop` returns "package not found"

**Solutions**:
1. **Verify Repository Setup**:
```bash
# Check if ROS repository is properly added
apt policy | grep ros

# If not found, re-add the repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

2. **Check Ubuntu Version Compatibility**:
```bash
# Verify Ubuntu version
lsb_release -a

# ROS 2 Humble requires Ubuntu 22.04 or 20.04
# For other versions, check ROS 2 compatibility
```

#### Issue: Python Package Installation Failures
**Symptoms**: `pip install` fails with compilation errors or dependency conflicts

**Solutions**:
```bash
# Upgrade pip first
pip3 install --upgrade pip setuptools

# Install system dependencies for common packages
sudo apt update
sudo apt install python3-dev python3-numpy python3-yaml python3-empy

# Try installing with user flag to avoid permission issues
pip3 install --user package_name

# For OpenCV issues specifically
sudo apt install python3-opencv
```

### Isaac Sim Installation Issues

#### Issue: Isaac Sim Won't Launch
**Symptoms**: Isaac Sim crashes on startup or displays "Failed to initialize GPU"

**Solutions**:
1. **Verify GPU Compatibility**:
```bash
# Check GPU and driver
nvidia-smi

# Verify CUDA version compatibility
nvcc --version

# Check if GPU meets minimum requirements
# Isaac Sim requires Turing architecture (RTX 20xx series) or newer
```

2. **Install Missing Dependencies**:
```bash
# Install required libraries
sudo apt install mesa-utils libgl1-mesa-glx libgl1-mesa-dri

# Verify OpenGL support
glxinfo | grep "OpenGL version"
```

3. **Reset Isaac Sim Configuration**:
```bash
# Backup and remove configuration
mv ~/.nvidia-isaac-sim ~/.nvidia-isaac-sim-backup

# Restart Isaac Sim to generate new configuration
```

## Simulation Environment Issues

### Gazebo Simulation Problems

#### Issue: Robot Falls Through Ground or Jitters
**Symptoms**: Robot model falls through floor or exhibits unstable physics behavior

**Solutions**:
1. **Check Inertial Properties**:
```xml
<!-- Verify mass and inertia values in URDF -->
<link name="base_link">
  <inertial>
    <mass value="1.0"/>  <!-- Should be positive and realistic -->
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>  <!-- Should be positive -->
  </inertial>
</link>
```

2. **Adjust Physics Parameters**:
```xml
<!-- In world file, adjust physics parameters -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- Smaller steps for stability -->
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

#### Issue: LIDAR/Depth Camera Data Issues
**Symptoms**: Sensor data shows artifacts, incorrect ranges, or no data

**Solutions**:
1. **Check Sensor Configuration**:
```xml
<!-- Verify sensor parameters in URDF/SDF -->
<sensor type="ray" name="laser_sensor">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>  <!-- Number of beams -->
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>  <!-- -π radians -->
        <max_angle>3.14159</max_angle>   <!-- π radians -->
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>  <!-- Minimum range -->
      <max>30.0</max> <!-- Maximum range -->
      <resolution>0.01</resolution>
    </range>
  </ray>
</sensor>
```

2. **Verify Plugin Configuration**:
```xml
<!-- Check Gazebo plugin settings -->
<plugin name="gazebo_ros_laser" filename="libgazebo_ros_ray_sensor.so">
  <ros>
    <namespace>/laser</namespace>
    <remapping>~/out:=scan</remapping>
  </ros>
  <output_type>sensor_msgs/LaserScan</output_type>
</plugin>
```

### Isaac Sim Performance Issues

#### Issue: Low Frame Rate in Isaac Sim
**Symptoms**: Simulation runs slowly or with low FPS

**Solutions**:
1. **Adjust Rendering Quality**:
   - In Isaac Sim, go to Window > Settings > Renderer
   - Reduce rendering quality during development
   - Disable ray tracing and advanced lighting for testing

2. **Optimize Scene Complexity**:
```python
# Reduce physics complexity in USD scenes
# Simplify collision meshes
# Reduce number of active lights
```

3. **Monitor Resource Usage**:
```bash
# Monitor GPU usage
nvidia-smi -l 1

# Monitor CPU usage
htop
```

## ROS 2 Communication Issues

### Topic and Service Problems

#### Issue: Nodes Cannot Communicate
**Symptoms**: Publishers and subscribers don't connect, no data exchange occurs

**Solutions**:
1. **Check ROS Domain ID**:
```bash
# Ensure all terminals use the same domain
echo $ROS_DOMAIN_ID

# Set domain if needed
export ROS_DOMAIN_ID=0
```

2. **Verify Network Configuration**:
```bash
# Check if multicast is working
ros2 topic list

# For multi-machine setup, check RMW implementation
echo $RMW_IMPLEMENTATION

# Test basic communication
ros2 topic echo /chatter std_msgs/msg/String &
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello'"
```

#### Issue: TF Transform Issues
**Symptoms**: "No transform found" errors, incorrect robot visualization

**Solutions**:
1. **Check TF Tree**:
```bash
# Install tf2 tools
sudo apt install ros-humble-tf2-tools

# View TF tree
ros2 run tf2_tools view_frames

# Check specific transforms
ros2 run tf2_ros tf2_echo base_link laser_frame
```

2. **Verify Robot State Publisher**:
```bash
# Ensure joint states are published
ros2 topic echo /joint_states sensor_msgs/msg/JointState

# Check robot state publisher is running
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="..."
```

## Computer Vision Issues

### OpenCV and Image Processing Problems

#### Issue: Camera Images Not Displaying Properly
**Symptoms**: Black images, wrong colors, or distorted visuals

**Solutions**:
1. **Check Image Encoding**:
```python
import cv2
from cv_bridge import CvBridge

# Verify image encoding
bridge = CvBridge()
cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')  # or 'rgb8'
```

2. **Verify Camera Calibration**:
```yaml
# Check camera info is published with correct parameters
# Camera intrinsic and extrinsic parameters should match physical camera
```

#### Issue: Object Detection Not Working
**Symptoms**: No objects detected or false detections

**Solutions**:
1. **Check Image Quality**:
```python
# Verify image has valid data
if cv_image is not None and cv_image.size > 0:
    # Process image
    pass
else:
    print("Invalid image data")
```

2. **Adjust Detection Parameters**:
```python
# For color-based detection, adjust HSV ranges
lower_color = np.array([0, 50, 50])   # Adjust based on lighting
upper_color = np.array([10, 255, 255]) # Adjust based on lighting
```

## Voice and Language Integration Issues

### Whisper Integration Problems

#### Issue: Poor Voice Recognition Accuracy
**Symptoms**: Commands not recognized or incorrect transcriptions

**Solutions**:
1. **Check Audio Input**:
```python
import pyaudio

# Verify microphone is working
p = pyaudio.PyAudio()
print("Available audio devices:")
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    print(f"Device {i}: {info['name']} - Input: {info['maxInputChannels']}")
```

2. **Adjust Recognition Parameters**:
```python
import speech_recognition as sr

# Adjust for ambient noise
with sr.Microphone() as source:
    r.adjust_for_ambient_noise(source, duration=1.0)
    audio = r.listen(source, timeout=5, phrase_time_limit=10)
```

#### Issue: LLM Integration Failures
**Symptoms**: API errors, timeout, or incorrect responses

**Solutions**:
1. **Verify API Configuration**:
```python
from openai import OpenAI

# Check API key and network connectivity
client = OpenAI(api_key='your-api-key')

try:
    response = client.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[{"role": "user", "content": "test"}],
        timeout=30
    )
    print("API connection successful")
except Exception as e:
    print(f"API connection failed: {e}")
```

## Performance Optimization

### General Performance Issues

#### Issue: System Running Slowly
**Symptoms**: High CPU/GPU usage, slow response times, application lag

**Solutions**:
1. **Monitor Resource Usage**:
```bash
# CPU and memory usage
htop

# GPU usage
nvidia-smi

# Disk I/O
iotop
```

2. **Optimize ROS 2 Launch Files**:
```xml
<!-- Reduce unnecessary nodes -->
<!-- Use appropriate QoS settings -->
<!-- Consider using Fast DDS for better performance -->
```

3. **Adjust Simulation Settings**:
```bash
# Reduce simulation update rate
# Simplify robot models during development
# Use lighter-weight sensors for testing
```

## Network and Communication Issues

### Multi-Machine Setup Problems

#### Issue: Cross-Machine ROS 2 Communication Failure
**Symptoms**: Nodes on different machines cannot communicate

**Solutions**:
1. **Check Network Configuration**:
```bash
# Verify machines can ping each other
ping other_machine_ip

# Check firewall settings
sudo ufw status

# Ensure both machines use same ROS domain
export ROS_DOMAIN_ID=0
```

2. **Configure DDS Middleware**:
```bash
# Use Cyclone DDS for better multi-machine support
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
```

## Common Error Messages and Solutions

### "Segmentation Fault" Errors
**Causes**: Memory access violations, usually in C++ nodes
**Solutions**:
- Run with debug symbols: `gdb --args ros2 run package node`
- Check for memory leaks in custom code
- Verify all pointers are properly initialized

### "Permission Denied" Errors
**Causes**: File permission issues or missing access rights
**Solutions**:
- Check file permissions: `ls -la filename`
- Add user to appropriate groups: `sudo usermod -a -G dialout $USER`
- Use proper ROS 2 workspace setup

### "Address Already in Use" Errors
**Causes**: Port conflicts or zombie processes
**Solutions**:
- Find and kill conflicting processes: `lsof -i :port_number`
- Restart the ROS 2 daemon: `ros2 daemon stop && ros2 daemon start`

## Debugging Strategies

### Systematic Debugging Approach
1. **Isolate the Problem**: Test individual components separately
2. **Check Logs**: Examine ROS 2 logs and system messages
3. **Verify Dependencies**: Ensure all required packages are installed
4. **Test with Minimal Examples**: Use simple test cases first
5. **Check Documentation**: Refer to official documentation for configuration

### Useful Debugging Commands
```bash
# ROS 2 debugging
ros2 doctor                    # Check ROS 2 installation health
ros2 run demo_nodes_cpp talker # Test basic functionality
ros2 node list                 # Verify nodes are running
ros2 topic info /topic_name    # Check topic status

# System debugging
journalctl -f                  # Monitor system logs
dmesg                          # Check kernel messages
systemctl status ros-humble-ros-core  # Check ROS services
```

## Getting Additional Help

### When to Seek Help
- Issues persist after following troubleshooting steps
- Error messages are unclear or unfamiliar
- Performance issues significantly impact learning

### Resources for Help
- Course forums and discussion boards
- Official ROS 2 documentation and answers
- Isaac Sim community and support
- Gazebo simulation tutorials and forums
- GitHub issues for specific packages

Remember: Most issues in robotics development are solvable with systematic troubleshooting. Take time to understand error messages and verify each component independently.