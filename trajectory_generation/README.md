# Manipulator UAV Task Planning System

A ROS package designed for task planning functionality, including task scheduling and path planning. It integrates with the ROS system to manage tasks effectively.

Table of Contents

1. [Overview](#overview)
2. [System Requirements](#system-requirements)
3. [Installation Guide](#installation-guide)
4. [Demonstration Process](#demonstration-process)
5. [Usage Instructions](#usage-instructions)

## Overview

T `trajectory_generation` is a ROS package primarily designed for task planning functionality. Its core features include task scheduling, path planning, and integration with the ROS system. Inputs typically consist of task requests (e.g., target points or task instructions), while outputs are planning results (e.g., paths or action sequences). The package facilitates inter-module communication through ROS messages and services, making it suitable for task management in UAV or robotic systems.

## System Requirements

### Software Dependencies

- ROS (Robot Operating System) version: Noetic
- CMake version: 3.16 or higher
- Python version: 3.8 or higher

### Operating System

- Ubuntu 20.04 LTS (Recommended)
- Ubuntu 18.04 LTS
- Other Linux distributions

### Hardware Requirements

- Processor: Intel i5 or higher
- Memory: 8GB or higher
- Storage: At least 10GB free space

## Installation Guide

1. Clone the repository:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/your-repo/trajectory_generation.git（注意修改）
   ```
2. Install dependencies:
   ```bash
   sudo apt-get update
   sudo apt-get install ros-noetic-desktop-full
   ```
3. Build the project:
   ```bash
   cd ~/catkin_ws
   catkin build
   ```

**Estimated installation time**: Approximately 10 minutes (depending on network and hardware performance).

## Demonstration Process

1. Start ROS core:
   ```bash
   roscore
   ```
2. Start the task planning system:
   ```bash
   roslaunch trajectory_generation trajectory_generation.launch
   ```
3. Expected output:
   - Task planning success logs
   - Visualized path results

**Demonstration time**: Approximately 5 minutes.

## Usage Instructions

### Processing Custom Data

1. Modify the configuration file `config/corridor.yaml` to match your data (typically, only `corridor.yaml` needs adjustment; other configuration files like `objects.yaml` and `obstacle.yaml` usually require no changes).
2. Restart the task planning node.

---

For any issues, contact: your-email@example.com（注意修改）
