---
title: Hardware Requirements
description: Detailed hardware specifications for the Physical AI & Humanoid Robotics course
sidebar_position: 1
---

# Hardware Requirements

## Overview

This course involves intensive computation, simulation, and visualization tasks that require specific hardware capabilities. The following specifications ensure optimal performance for all course activities.

## System Requirements

### Minimum Requirements

#### CPU
- **Type**: Multi-core processor
- **Minimum**: Intel Core i5 or AMD Ryzen 5 with 4 cores/8 threads
- **Architecture**: x86_64 (64-bit)
- **Clock Speed**: 2.5 GHz or higher

#### RAM
- **Minimum**: 8 GB DDR4
- **Recommended**: 16 GB DDR4 for optimal performance
- **Type**: DDR4 2400 MHz or higher

#### Storage
- **Minimum**: 50 GB free space
- **Recommended**: 100+ GB free space
- **Type**: SSD recommended (NVMe preferred)
- **Note**: Simulation environments and datasets can be large

#### Graphics Processing Unit (GPU)
- **Minimum**: Integrated graphics (Intel HD Graphics 620 or equivalent)
- **Dedicated GPU**: NVIDIA GTX 1060 (6GB) or AMD RX 580 (8GB)
- **CUDA Support**: Required for NVIDIA Isaac Sim (CUDA 11.8+)

#### Network
- **Type**: Ethernet or Wi-Fi 5 (802.11ac) minimum
- **Speed**: 100 Mbps minimum, 1 Gbps recommended
- **Stability**: Reliable connection required for development

### Recommended Requirements

#### CPU
- **Type**: Multi-core processor
- **Recommended**: Intel Core i7/i9 or AMD Ryzen 7/9 with 6+ cores/12+ threads
- **Architecture**: x86_64 (64-bit)
- **Clock Speed**: 3.0 GHz or higher with boost capabilities

#### RAM
- **Recommended**: 32 GB DDR4
- **For Intensive Simulation**: 64 GB DDR4
- **Type**: DDR4 3200 MHz or higher

#### Storage
- **Recommended**: 500+ GB NVMe SSD
- **Type**: NVMe M.2 2280 SSD
- **Speed**: Sequential read 3000+ MB/s, write 2000+ MB/s

#### Graphics Processing Unit (GPU)
- **Recommended**: NVIDIA RTX 3060 (12GB) or higher
- **For Isaac Sim**: NVIDIA RTX 3080 (10GB) or RTX 4080 (16GB)
- **VRAM**: 8GB minimum, 16GB recommended for complex simulations
- **CUDA Cores**: 3584+ cores
- **RT Cores**: 2nd generation or higher (for ray tracing in Isaac Sim)

#### Network
- **Type**: Gigabit Ethernet preferred
- **Speed**: 1 Gbps
- **Latency**: &lt;10ms for real-time simulation

## Component-Specific Requirements

### For ROS 2 Development
- **CPU Cores**: 4+ (for parallel compilation)
- **RAM**: 8GB+ (for multiple ROS nodes)
- **Storage**: SSD for fast compilation and file access

### For Gazebo Simulation
- **CPU**: Multi-core with good single-thread performance
- **RAM**: 16GB+ for complex environments
- **GPU**: Dedicated graphics with OpenGL 4.3+ support
- **VRAM**: 4GB+ for high-quality rendering

### For NVIDIA Isaac Sim
- **GPU**: NVIDIA RTX series (20xx or 40xx generation)
- **VRAM**: 10GB+ minimum, 16GB+ recommended
- **CUDA**: Version 11.8 or higher
- **Driver**: NVIDIA driver 520+ (Linux) or 531+ (Windows)
- **CPU**: 8+ cores recommended for physics simulation

### For Computer Vision Tasks
- **CPU**: Multi-core with good floating-point performance
- **GPU**: CUDA-capable NVIDIA GPU recommended
- **RAM**: 16GB+ for processing high-resolution images
- **Storage**: Fast SSD for dataset access

## System Compatibility

### Operating System Support
- **Primary**: Ubuntu 20.04 LTS or 22.04 LTS
- **Alternative**: Ubuntu running in WSL2 (Windows)
- **Virtualization**: VM with GPU passthrough (NVIDIA vGPU)

### GPU Compatibility Matrix

| GPU Series | ROS 2 Support | Gazebo Support | Isaac Sim Support | VRAM Min | Performance |
|------------|---------------|----------------|-------------------|----------|-------------|
| NVIDIA RTX 40xx | ✅ | ✅ | ✅ | 10GB | Excellent |
| NVIDIA RTX 30xx | ✅ | ✅ | ✅ | 8GB | Very Good |
| NVIDIA RTX 20xx | ✅ | ✅ | ✅ | 8GB | Good |
| NVIDIA GTX 16xx | ✅ | ✅ | ❌ | 6GB | Adequate |
| AMD Radeon RX 6000 | ✅ | ✅ | ❌ | 8GB | Good |
| Intel Arc A770 | ✅ | ✅ | ❌ | 8GB | Good |

### CPU Architecture
- **Supported**: x86_64 (Intel/AMD 64-bit)
- **Experimental**: ARM64 (with limitations on Isaac Sim)
- **Not Supported**: 32-bit architectures

## Budget Considerations

### Budget-Friendly Setup (Under $1000)
- CPU: AMD Ryzen 5 5600X or Intel i5-12400F
- RAM: 16GB DDR4-3200
- Storage: 500GB NVMe SSD
- GPU: NVIDIA GTX 1660 Super or RTX 3060
- This setup meets minimum requirements for most coursework

### Recommended Setup ($1500-2500)
- CPU: AMD Ryzen 7 5800X or Intel i7-12700K
- RAM: 32GB DDR4-3600
- Storage: 1TB+ NVMe SSD
- GPU: NVIDIA RTX 3070 or RTX 4070
- Excellent for all course requirements

### High-Performance Setup ($3000+)
- CPU: AMD Ryzen 9 5900X or Intel i9-12900K
- RAM: 64GB DDR4-3600
- Storage: 2TB+ NVMe SSD
- GPU: NVIDIA RTX 4080 or RTX 4090
- Ideal for research and advanced projects

## Alternative Options

### Cloud-Based Development
- **AWS EC2**: p3/p4 instances with NVIDIA V100/A100 GPUs
- **Google Cloud**: A2 instances with NVIDIA A100 GPUs
- **Azure**: NC/ND series with NVIDIA GPUs
- **Benefits**: No hardware investment, scalable resources
- **Drawbacks**: Ongoing costs, network dependency

### Virtual Machines
- **Requirements**: Hardware acceleration support (Intel VT-x/AMD-V)
- **GPU Passthrough**: For Isaac Sim (VFIO/IGPU)
- **Minimum**: 8GB RAM allocated to VM
- **Storage**: 50GB+ virtual disk

### Container-Based Development
- **Docker**: With NVIDIA Container Toolkit
- **Requirements**: Host system meets hardware requirements
- **Benefits**: Isolated environments, easy setup
- **Performance**: Near-native with proper configuration

## Troubleshooting Hardware Issues

### Common GPU Problems
```bash
# Check NVIDIA GPU detection
nvidia-smi

# Verify CUDA installation
nvcc --version

# Check OpenGL support
glxinfo | grep "OpenGL version"
```

### Memory Issues
```bash
# Check available RAM
free -h

# Monitor memory usage during simulation
htop
```

### Storage Space Management
```bash
# Check available disk space
df -h

# Clean ROS 2 build files to save space
rm -rf build/ log/
```

## Hardware Upgrade Path

If starting with minimum requirements, consider upgrading in this order:
1. **RAM**: From 8GB to 16GB+ (immediate impact)
2. **Storage**: From HDD to SSD (significant speed improvement)
3. **GPU**: For Isaac Sim and advanced graphics
4. **CPU**: For faster compilation and processing

## Verification Steps

Before starting the course, verify your hardware meets requirements:

```bash
# Check system information
uname -a
lscpu
free -h
df -h

# For NVIDIA GPU users
nvidia-smi
nvidia-smi -q -d SUPPORTED_CLOCKS

# Check OpenGL support
glxinfo | grep -E "(OpenGL|renderer)"
```

## Support and Recommendations

- **University Labs**: Check if your institution provides access to high-performance workstations
- **Student Discounts**: NVIDIA, AMD, and other vendors offer academic pricing
- **Refurbished Options**: Consider certified refurbished high-performance systems
- **Rental Services**: Short-term GPU rental for intensive projects

## Notes for Specific Use Cases

### For Real Robot Development
- Additional requirements for real-time control systems
- Real-time kernel considerations
- Low-latency network interfaces

### For Research Projects
- Consider dual GPU setups for training and simulation
- High-speed storage arrays for large datasets
- Redundant power supplies for reliability

This hardware specification is designed to support all aspects of the Physical AI & Humanoid Robotics course while providing a foundation for future robotics development and research.