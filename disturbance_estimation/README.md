# Disturbance Estimator Toolbox

Table of Contents
-----------------

1. [Overview](#overview)
2. [System Requirements](#system-requirements)
3. [Installation Guide](#installation-guide)
4. [Usage Instructions](#usage-instructions)
5. [Functional Modules](#functional-modules)
6. [Notes](#notes)

## Overview

The disturbance_estimation is a disturbance estimation module based on ROS and neural networks (NN), primarily designed for real-time prediction and compensation of external disturbances in UAV or robotic systems. Its core functionalities include neural network model inference (e.g., nn_predict-related files), matrix operation optimization (e.g., elementwiseOperationInPlace), and interaction with other system modules via ROS messages (e.g., motor_state.msg). The project is built using CMake and relies on MATLAB Coder-generated code (e.g., rtwtypes.h), making it suitable for robust control scenarios in dynamic environments.

## System Requirements

### Software Dependencies

- ROS Noetic
- Catkin Tools
- Python 3.8 or later

### Operating System

- Ubuntu 20.04 LTS (Recommended)

### Test Environment

- Tested on Ubuntu 20.04 LTS with ROS Noetic

### Special Hardware Requirements

- No special hardware requirements

## Installation Guide

1. Navigate to the Catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   ```
2. Clone the repository:
   ```bash
   git clone https://github.com/your-repo/disturbance_estimation.git(这里需要修改)
   ```
3. Build the project:
   ```bash
   cd ~/catkin_ws && catkin build
   ```

**Estimated Installation Time**: Approximately 10-15 minutes (depending on network speed and hardware performance).

## Usage Instructions

1. **Build the Package**:
   ```bash
   cd ~/catkin_ws && catkin build disturbance_estimation
   ```
2. **Launch the Node**:
   ```bash
   roslaunch disturbance_estimation start.launch
   ```
3. **Configure Parameters**:
   - Modify `launch/start.launch` to adjust ROS topics and neural model paths.
4. **Monitor Output**:
   - Use `rostopic echo /mavros/disturbance_est/dist_est` to view real-time disturbance estimates.

## Functional Modules

1. **Neural Network Inference (`nn_predict`)** - Handles real-time disturbance prediction using trained models.
2. **Matrix Operations (`elementwiseOperationInPlace`)** - Optimized matrix operations for performance-critical sections.

## Notes

- Ensure MATLAB Coder-generated files (e.g., `rtwtypes.h`) are correctly linked.
- The module is designed for real-time operation; avoid blocking calls in critical paths.
- Tested on ROS Noetic and Ubuntu 20.04.

1. Run the launch file:
   ```bash
   roslaunch disturbance_estimation start.launch
   ```
2. Expected Output:
   - Disturbance estimation node starts
   - Real-time estimation results are logged to the terminal and log files

**Estimated Demo Time**: Approximately 1-2 minutes (depending on hardware performance).

---

For any issues, please contact: your-email@example.com(这里需要修改)
