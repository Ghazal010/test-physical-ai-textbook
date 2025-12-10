---
title: Prerequisites
description: Required knowledge and setup for the Physical AI & Humanoid Robotics course
sidebar_position: 2
---

# Prerequisites

## Required Background Knowledge

### Programming Skills
- **Python Programming**: Basic to intermediate level understanding
  - Variables, data types, and control structures
  - Functions, classes, and object-oriented programming
  - File I/O and error handling
  - Libraries and modules
  - Basic understanding of data structures (lists, dictionaries, etc.)

### Mathematics
- Basic algebra and geometry
- Understanding of coordinate systems (Cartesian, polar)
- Basic calculus concepts (derivatives for understanding motion)
- Linear algebra fundamentals (vectors and matrices)

### Computer Science Fundamentals
- Basic understanding of algorithms and data structures
- Experience with version control (Git)
- Command line interface proficiency
- Basic understanding of operating systems

## Software Prerequisites

### Operating System
- **Ubuntu 20.04 LTS** or **Ubuntu 22.04 LTS** (recommended)
- Alternative: Virtual machine or Docker container with Ubuntu
- Windows users: WSL2 (Windows Subsystem for Linux) with Ubuntu

### Required Software Installation

#### Python Environment
```bash
# Install Python 3.8 or later
sudo apt update
sudo apt install python3 python3-pip python3-venv

# Verify installation
python3 --version
```

#### Version Control
```bash
# Install Git
sudo apt install git

# Configure Git (replace with your information)
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

#### Essential Tools
```bash
# Install essential build tools
sudo apt install build-essential cmake pkg-config

# Install additional utilities
sudo apt install curl wget unzip htop iotop
```

### ROS 2 Prerequisites

#### System Dependencies
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

#### Python Dependencies
```bash
pip3 install -U argcomplete numpy matplotlib opencv-python
```

## Hardware Requirements

### Minimum Specifications
- **CPU**: Intel i5 or AMD Ryzen 5 (4 cores, 8 threads minimum)
- **RAM**: 8 GB DDR4
- **Storage**: 50 GB free space (SSD recommended)
- **GPU**: Integrated graphics or entry-level dedicated GPU
- **Network**: Stable internet connection

### Recommended Specifications
- **CPU**: Intel i7 or AMD Ryzen 7 (6+ cores, 12+ threads)
- **RAM**: 16 GB DDR4 (32 GB for intensive simulation)
- **Storage**: 100+ GB SSD storage
- **GPU**: NVIDIA RTX series (RTX 3060 or better) with CUDA support
- **Network**: High-speed internet connection

## Installation Preparation

### System Check
Before installing the course software stack, verify your system:

```bash
# Check Ubuntu version
lsb_release -a

# Check available disk space
df -h

# Check RAM
free -h

# Check CPU
lscpu
```

### Creating a Workspace
```bash
# Create a workspace directory
mkdir -p ~/robotics_ws/src
cd ~/robotics_ws

# Create virtual environment for Python packages
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
```

## Prior Knowledge Assessment

To ensure you're ready for this course, you should be comfortable with:

### Python Programming Test
```python
# You should be able to write and understand code like this:
def calculate_distance(point1, point2):
    """Calculate Euclidean distance between two points"""
    import math
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

# Example usage
start_pos = (0, 0)
end_pos = (3, 4)
distance = calculate_distance(start_pos, end_pos)
print(f"Distance: {distance}")  # Should output 5.0
```

### Basic ROS Concepts (Helpful but Not Required)
- Understanding of nodes, topics, and services
- Basic familiarity with message passing
- Conceptual understanding of robot simulation

## Setup Verification

### Python Environment Test
```bash
# Test Python environment
python3 -c "import numpy, matplotlib, cv2; print('Python environment ready')"
```

### Git Test
```bash
# Test Git functionality
git clone https://github.com/ros2/examples ~/test_ros2_examples
ls ~/test_ros2_examples
rm -rf ~/test_ros2_examples
```

## Alternative Setup Options

### Docker Environment
For students with incompatible systems:
```bash
# Install Docker
sudo apt install docker.io
sudo usermod -aG docker $USER

# Log out and back in, then test
docker run hello-world
```

### Cloud-Based Development
- AWS EC2 instances with GPU support
- Google Cloud Platform with Deep Learning VMs
- GitHub Codespaces with Ubuntu environment

## Troubleshooting Common Issues

### Python Package Installation Issues
```bash
# If pip fails, try upgrading setuptools
pip install --upgrade setuptools

# For permission issues, use user flag
pip install --user package_name
```

### System Resource Constraints
- Close unnecessary applications before running simulations
- Consider using lighter-weight alternatives for initial learning
- Use cloud resources for intensive computations

## Getting Help

If you encounter issues with prerequisites:

1. Check the [Troubleshooting Guide](/docs/resources/troubleshooting)
2. Visit the course forums
3. Attend virtual office hours
4. Consult the [Hardware Requirements](/docs/resources/hardware-requirements) page for system-specific guidance

## Next Steps

Once you've verified all prerequisites:

1. Proceed to install [ROS 2](https://docs.ros.org/en/humble/Installation.html)
2. Set up your [Development Environment](/docs/resources/development-environment)
3. Begin with [Week 1-2 Introduction to Physical AI](/docs/module-1/week-1-2/introduction-to-physical-ai)

Remember: It's normal to encounter setup challenges. Take your time with the installation process, and don't hesitate to seek help from the course community when needed.