---
title: Development Environment Setup
description: Complete guide to setting up your development environment for Physical AI & Humanoid Robotics
---

# Development Environment Setup

This guide provides detailed instructions for setting up your complete development environment for the Physical AI & Humanoid Robotics course.

## Development Environment Overview

The development environment for this course consists of multiple integrated tools and frameworks that work together to create a comprehensive robotics development platform.

### Core Components

- **ROS 2 (Robot Operating System 2)**: The communication framework for robotics applications
- **Gazebo**: Physics-based simulation environment
- **NVIDIA Isaac**: AI and perception tools for robotics
- **Unity**: Alternative simulation and visualization platform
- **Docker**: Containerization for consistent development environments

## ROS 2 Development Environment

### Installing ROS 2 Humble Hawksbill

The course uses ROS 2 Humble Hawksbill, which is an LTS (Long Term Support) version providing stability for the duration of the course.

```bash
# Add ROS 2 repository key
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-cv-bridge ros-humble-tf2-tools ros-humble-tf2-eigen
```

### Setting up ROS 2 Environment

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Add to your bashrc to make permanent
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Workspace Management

### Creating a ROS 2 Workspace

```bash
# Create workspace directory
mkdir -p ~/robotics_ws/src
cd ~/robotics_ws

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
echo "source ~/robotics_ws/install/setup.bash" >> ~/.bashrc
```

## Python Development Environment

### Virtual Environment Setup

```bash
# Create virtual environment
cd ~/robotics_ws
python3 -m venv venv
source venv/bin/activate

# Install required packages
pip install --upgrade pip
pip install numpy matplotlib opencv-python transforms3d pyquaternion
pip install rclpy sensor-msgs geometry-msgs
```

### ROS 2 Python Packages

```bash
# Install ROS 2 Python interfaces
sudo apt install python3-ros-interfaces python3-ros-numpy
pip install rospkg catkin_pkg
```

## Simulation Environment Setup

### Gazebo Installation

```bash
# Install Gazebo Garden
sudo apt install ignition-garden

# Or install Gazebo Harmonic (recommended)
sudo apt install gz-harmonic
```

### Unity Robotics Setup

For Unity integration, install the Unity Hub and Unity 2022.3 LTS:

1. Download Unity Hub from [Unity's website](https://unity.com/download)
2. Install Unity 2022.3 LTS through Unity Hub
3. Install the Unity Robotics Hub package
4. Import the ROS-TCP-Connector package

## NVIDIA Isaac Setup

### Isaac ROS Dependencies

```bash
# Install Isaac ROS dependencies
sudo apt install ros-humble-isaac-ros-gems ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-a100 ros-humble-isaac-ros-bitmask-encoder
```

### CUDA Setup

```bash
# Verify CUDA installation
nvidia-smi
nvcc --version

# Install PyCUDA if needed
pip install pycuda
```

## Development Tools

### IDE Configuration

Recommended IDEs for this course:

1. **VS Code** with ROS extensions:
   ```bash
   code --install-extension ms-iot.vscode-ros
   code --install-extension ms-python.python
   code --install-extension redhat.vscode-yaml
   ```

2. **PyCharm** with ROS plugins

### Git Configuration for ROS Projects

```bash
# Configure Git for ROS projects
git config --global core.precomposeunicode true
git config --global core.ignorecase false

# Create .gitignore for ROS projects
cat > ~/.gitignore_global << EOF
# ROS specific
*.pyc
*.so
build/
install/
log/
*.log
devel/

# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
env/
venv/
.venv/
pip-log.txt
pip-delete-this-directory.txt
.tox/
.coverage
.coverage.*
.cache
nosetests.xml
coverage.xml
*.cover
*.log
.gitattributes
.pytest_cache/
EOF

git config --global core.excludesfile ~/.gitignore_global
```

## Docker Environment (Alternative)

For students who prefer containerized development:

```bash
# Install Docker
sudo apt install docker.io
sudo usermod -aG docker $USER

# Install Docker Compose
sudo apt install docker-compose

# ROS 2 Docker image
docker pull osrf/ros:humble-desktop-full

# Run ROS 2 container
docker run -it --rm --name ros2_env osrf/ros:humble-desktop-full
```

## Verification Steps

### Test ROS 2 Installation

```bash
# Test ROS 2
ros2 topic list
ros2 service list
```

### Test Python Packages

```bash
python3 -c "import rclpy; import cv2; import numpy as np; print('ROS 2 Python environment ready')"
```

### Test Simulation

```bash
# Launch Gazebo
gz sim -v 4
```

## Troubleshooting

### Common Issues

1. **Permission errors**: Make sure to add your user to the correct groups:
   ```bash
   sudo usermod -a -G dialout $USER
   sudo usermod -a -G docker $USER
   ```

2. **CUDA issues**: Verify NVIDIA drivers and CUDA installation:
   ```bash
   nvidia-smi
   nvcc --version
   ```

3. **Python path issues**: Ensure ROS 2 environment is sourced before Python commands.

## Development Workflow

### Project Structure

```
~/robotics_ws/
├── src/                    # Source code
│   ├── robot_controller/   # Custom controllers
│   ├── perception_nodes/   # Perception modules
│   └── simulation_tests/   # Simulation tests
├── build/                  # Build artifacts
├── install/                # Install space
└── venv/                   # Python virtual environment
```

### Best Practices

1. Always source your ROS 2 environment before development
2. Use separate terminals for different components (simulation, nodes, etc.)
3. Regularly update packages: `sudo apt update && sudo apt upgrade`
4. Use version control for your custom code

## Next Steps

After completing your development environment setup:

1. Review the [Hardware Requirements](/docs/resources/hardware-requirements) for optimal performance
2. Explore the [Troubleshooting Guide](/docs/resources/troubleshooting) for common issues
3. Begin with [Module 1: The Robotic Nervous System](/docs/module-1/week-1-2/introduction-to-physical-ai)