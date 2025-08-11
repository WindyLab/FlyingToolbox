# Airbox Monitor

Table of Contents

1. [Overview](#overview)
2. [System Requirements](#system-requirements)
3. [Installation Guide](#installation-guide)
4. [Demo Process](#demo-process)
5. [Usage Instructions](#usage-instructions)
6. [Functional Modules](#functional-modules)
7. [Notes](#notes)

## Overview
A multi-UAV monitoring and trajectory planning system based on ROS and Qt. The system can monitor the positions of multiple UAVs and supports interactive trajectory planning and collision detection.

## System Requirements

### Software Dependencies

- **ROS**: Version Noetic
- **Qt**: Version 5.12
- **Catkin**: Build tool

### Operating System

- **Ubuntu**: 20.04 LTS

### Test Environment

- **Hardware**: Intel Core i7, 16GB RAM, NVIDIA Geforce 1060
- **Software**: ROS Noetic, Qt 5.12

### Special Hardware Requirements

- **NVIDIA Graphics Card**: For GPU acceleration
- **UAV Communication Module**: Such as MAVLink

## Installation Guide

### Step-by-Step Installation

1. **Install ROS Noetic**:
   ```bash
   sudo apt-get install ros-noetic-desktop-full
   ```
2. **Install Qt 5.12**:
   ```bash
   sudo apt-get install qt5-default
   ```
3. **Clone the Project Repository**:
   ```bash
   git clone https://github.com/your-repo/task_planning.git(注意需要修改)
   ```
4. **Build the Project**:
   ```bash
   cd task_planning
   catkin build
   source devel/setup.bash
   ```

### Installation Time

- **Standard Computer**: Approximately 30 minutes

## Demo Process

### Steps to Run

1. **Start ROS Core**:
   ```bash
   roscore
   ```
2. **Start the Monitoring System**:
   ```bash
   roslaunch airbox_monitor task_planning.launch
   ```

### Expected Output

- Real-time display of UAV positions
- Trajectory planning visualization
- Collision detection warnings

### Demo Duration

- **Full Demo**: Approximately 10 minutes

## Usage Instructions

### Reproducing Paper Results

1. Download the paper dataset
2. Configure the system according to the parameters in the paper
3. Run the system and record the results

## Functional Modules

### 1. Real-time Monitoring (UAV_monitor)

- Display real-time UAV positions
- Coordinate transformation and visualization
- Position validity checks

### 2. Trajectory Planning (Task Plan)

- Add waypoints by clicking
- Support for multi-UAV trajectory planning
- Allow editing and deleting trajectories
- Support collision detection
- Automatically generate task configuration files

## Notes

### Changing Background Images

1. **Copy the image to the `resources/images` folder**
   - Recommended resolution: 1000x600 pixels
2. **Update the `images.qrc` file**
   ```xml
   <file>images/new_background.png</file>
   ```
3. **Modify the `main_window.cpp` file**
   ```cpp
   QPixmap pixmap(":/images/new_background.png");
   ```
4. **Rebuild and Launch**
   ```bash
   catkin build
   source devel/setup.bash
   roslaunch airbox_monitor airbox_monitor.launch
   ```
