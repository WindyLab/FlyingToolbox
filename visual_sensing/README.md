# Aerial Vision Toolbox

## Table of Contents

1. [Overview](#overview)
2. [System Requirements](#system-requirements)
3. [Installation Guide](#installation-guide)
4. [Demo Workflow](#demo-workflow)
5. [Usage Instructions](#usage-instructions)

## Overview

This toolbox is designed for aerial vision tasks, including camera calibration, real-time image processing, and relative position calculation between drones. It supports ROS (Noetic) and is optimized for NVIDIA Jetson platforms.

## System Requirements

- **Operating System**: Ubuntu 20.04 LTS
- **Software Dependencies**: ROS (Noetic), CMake 3.16+, Eigen3, OpenCV 4.2+
- **Hardware Requirements**: ROS-compatible drone platform, camera or visual sensor
- **Tested Environments**: NVIDIA Jetson Xavier NX and Windows x64 platforms

## Installation Guide

1. Clone this repository to your local workspace:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/your_repository/visual_sensing.git（需要修改）
   ```
2. Install dependencies:

   - Ubuntu:
     ```bash
     sudo apt-get install ros-noetic-eigen-conversions
     ```
3. Build the project:

   ```bash
   catkin build
   ```

   **Estimated duration**: Approximately 5-15 minutes (depending on hardware performance).

## Demo Workflow

1. Start the ROS core (if using ROS):
   ```bash
   roscore
   ```
2. Launch the camera or visual sensor:
   ```bash
   rosrun visual_sensing test_realsense
   ```
3. Run the image processing program to calculate the relative position between the two drones:
   ```bash
   rosrun visual_sensing realsense
   ```
4. View the detection results:
   - ROS environment:
     ```bash
     rostopic echo /tool_box_pos
     ```
   - Non-ROS environment: View the program output directly.
     **Expected output**: Displays the toolbox's position, orientation, and the number of detected tools.
     **Demo duration**: Approximately 2-5 minutes.

## Usage Instructions

1. **Process custom data**: Input camera data into the program or subscribe to the ROS topic `/tool_box_pos` to obtain toolbox information.
2. **Reproduce paper results**: Run the detection program using the provided test dataset.
3. **Customize tool IDs**: Modify the tool ID definitions in `toolbox.h` to match the actual toolbox tags.

### Program Function Descriptions

- **`test_realsense`**: Used to open the camera or visual sensor and publish image topics.

  - Usage:
    ```bash
    rosrun visual_sensing test_realsense
    ```
  - Output: Publishes the image topic `/camera/realsense`.
- **`cali_camera`**: Used for camera calibration to calculate the camera's position in the working drone's coordinate system.

  - Usage:
    ```bash
    rosrun visual_sensing cali_camera
    ```
  - Output: Generates the calibration parameter file `camera_params.yaml`.
- **`realsense`**: Used for actual docking to calculate the relative position between two drones.

  - Usage:
    ```bash
    rosrun visual_sensing realsense
    ```
  - Output: Publishes the relative position topic `/tool_box_pos`.
