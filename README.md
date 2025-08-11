<div align="center">
    <h1>Proximal cooperative aerial manipulation with vertically stacked drones</h1>
    <br>Huazi Cao, Jiahao Shen, Yin Zhang, Zheng Fu, Cunjia Liu, Sihao Sun</br>
</div>

This GitHub repository provides open-source materials for **FlyingToolbox**, including mechanical design files and source code for disturbance estimation, visual sensing, trajectory generation, and task planning.  **FlyingToolbox** is a cooperative aerial manipulation system and consists of a toolbox MAV and a manipulator MAV.  The toolbox MAV carries a toolbox that contains a set of end-effector tools for dedicated tasks such as grasping, cutting, and inspection. The manipulator MAV flies above the toolbox MAV and can autonomously dock with any tool using its robotic arm. After completing a manipulation task, the manipulator MAV can return the tool back to the toolbox MAV or switch to another to perform a different task. 

# File Summary


<div align="center">

| Folder names | Contents |
|:--------:|:----:|
| 3D Modeling Files | Files for the mechanical design |
| disturbance_estimation | Source code used for disturbance estimation |
| visual_sensing | Source code used for visual sensing |
| trajectory_generation | Source code used for trajectory generation|
| task_planning |  Source code used for task planning|

</div>

# System Requirements

### Hardware Requirements

1. Disturbance estimation
This package requires only a computer with enough RAM to support the operations defined by a user. For minimal performance, this will be a computer with about 2 GB of RAM. Computers can be either personal computers (PCs) or onboard computing devices such as NUC and NX systems.  For optimal performance, we recommend a computer with the following specs:

- RAM: 8+ GB  

- CPU: 4+ cores, 1.7GHz/core

In addition to a computer, the following hardware equipment is required for this package.

|                    Usage                     |                     Hardware equipments                      |
| :------------------------------------------: | :----------------------------------------------------------: |
|    Estimated disturbance force and torque    |                    Motion capture system                     |
| Acquisition of true disturbance force values | Motion capture system , [T-Motor 501-X motor and ESC integrated kit](https://item.taobao.com/item.htm?_u=a22o13el43d2&id=606577444356&pisk=g1czsB_zQQdrNiZ8EjNFbs5vdWP81W-6xXZQ-203PuqkpzOhT2ugRUKpyxznAmLpy7g3YDobD6aBeHCE00giF8iIFk43mm05NkO8LDusXD1BwYahTmgLKDGEJ6zn-DL8AUpjy4FLtht6TCg-yHSI8Wlzro0mkyslqKwlUIuWlht61BJloW-yfD1Z3KR07PV3rJfhnEzarzV3Ku0Duy4Cq9foxE84yP5hxzqhoI4Lm9q3-zjcmPUGt_4u-Eu0DP5utWm3iEzKgvm-vGagEEjRPXZdWHyuj4qVt1bTq8mw6oChFj4zXO3z0qgjgzyzjJmsgzGoc2cSw-pPEW3jQDknjn67qAzZ0RMyb_oZ2y0U-0RO7ocr8jy-hgXu77rzIbmcWs3aKxl4NbxdBqg4qRPjhKK7d7orBlekHnnEuuina-jyckMslb2qjn1qvJknNSckmCSyaMUmMxlKUMXUErU4fE84Ln01IOT4bXBdp-2gulT8K9BLErU4fE8Vp9eRsrr6yJf..&spm=a1z09.2.0.0.78ef2e8dyWEVzq),  [ALPHA DATA LINK v2 acquisition system](https://item.taobao.com/item.htm?_u=a22o13el23c8&id=643443876894&pisk=guX4s6m2T-e2Z78viT9Z8Yk81dJvKdzQSOT6jGjMcEY06nwiQGSHfoUTD_-G5aETDKsM7N7WyAt_MPHNzZshGI_1GFxMrajjhFwv_NSCJNM_HstiQas9sN6NXA-GjNEv5oF5Dip9IyaQQJsADPu1bdW2mbcl2himm7Omg8SbVyaQdRPmqdz4RNMFacekYHvMmCDiZu-emnvMsEAu4hxsofDcSuryDHkiSnYiq8x9rfDMoFxkrnKKsVcDSuSkyhkMIdbMEu-tgT_AW2tHiu0xcOTtXAAJ-iYrIvmpoIX7pUMiuTx2JySXz3t5UnR2-CbCUn6cPGXfHQFqidI5TN5G-zGXo6-FzB148x7FkhjwjZyKYUXVbTRANqcDYK82ttbovYIes_Wyht4t9gsyoBJ5N7UX1K7V9ed0wzQN4E_G3Q04PF1CVtAP-zMPWC5GhLX0rJSPTVKle_WOgVcwi3KyRury_zjItXZy8Oht6QAH4eZvsfh9i3KyRurr6fdxK38QDCf..&spm=a1z09.2.0.0.78ef2e8dyWEVzq&skuId=4633424884192), [HWT905 IMU](https://detail.tmall.com/item.htm?abbucket=20&id=598664650003&mi_id=rY7NVMPyiDSRF7ikY3XbWG7n6kimayiMKO03bL11Q_GXctqFmoiCLXO3KC6-5WytpaBv7pU7eNnbZRJFg21fo6IE5feMqmVfRZDd56BVSng&ns=1&skuId=4439943127599&spm=a21n57.1.hoverItem.1&utparam=%7B%22aplus_abtest%22%3A%22567b61e09056866734aae87654842d08%22%7D&xxc=taobaoSearch) or other higher-precision IMU |

2. Visual sensing

This package requires only a computer with enough RAM to support the operations defined by a user. For minimal performance, this will be a computer with about 2 GB of RAM. Computers can be either personal computers (PCs) or onboard computing devices such as NUC and NX systems.  For optimal performance, we recommend a computer with the following specs:

- RAM: 8+ GB  

- CPU: 4+ cores, 1.7GHz/core

This package must be used in conjunction with onboard cameras. We have successfully tested it with the following cameras: **Intel RealSense L515** and **D435i**.

3. Trajectory generation

This package requires only a computer with enough RAM to support the operations defined by a user. For minimal performance, this will be a computer with about 2 GB of RAM. Computers can be either personal computers (PCs) or onboard computing devices such as NUC and NX systems.  For optimal performance, we recommend a computer with the following specs:

- RAM: 8+ GB  

- CPU: 4+ cores, 1.7GHz/core


4. Task planning

This package requires only a computer with enough RAM to support the operations defined by a user. For minimal performance, this will be a computer with about 2 GB of RAM. Computers can be either personal computers (PCs) or onboard computing devices such as NUC and NX systems.  For optimal performance, we recommend a computer with the following specs:

- RAM: 8+ GB  

- CPU: 4+ cores, 1.7GHz/core

### Software Requirements

The repository development version is tested on *Linux* operating systems. The developmental version of the package has been tested on the following systems: Ubuntu 18.04, Ubuntu 20.04. In addition, all code in the package  were developed for ROS (Melodic on Ubuntu 18.04 or Noetic on Ubuntu 20.04). 

# Code download and compilation

1. Clone this repository to your local workspace:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/WindyLab/FlyingToolbox.git
   ```

2. Install dependencies:


   - Disturbance estimation:
      This package utilizes MATLAB Coder for neural network code generation. For detailed technical specifications, please refer to the documentation at: https://ww2.mathworks.cn/help/coder/deep-learning-code-generation-fundamentals.html?s_tid=CRUX_lftnav. Additionally, the following dependencies must be installed to execute the generated code.

 ```bash
     sudo apt-get install ros-noetic-serial ros-noetic-eigen-conversions
     sudo apt-get install libomp-dev      # OpenMP
 ```
   - Visual sensing:
     
     This package utilizes the OpenCV library and Intel RealSense SDK for its core functionality. The following dependencies are required for proper operation:
     
     ```bash
          sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf ros-noetic-eigen-conversions
          sudo apt-get install librealsense2-dev #RealSense SDK
          sudo apt-get install libopencv-dev #OpenCV
     ```
     
   - Trajectory generation:
     ```bash
     sudo apt-get install ros-noetic-eigen-conversions
     ```
     Additionally, this package requires OSQP as a dependency. For installation and usage instructions, please refer to: https://github.com/osqp/osqp.git.
     
   - Task planning:
     ```bash
     # Qt5 
     sudo apt-get install qtbase5-dev             
     sudo apt-get install libqt5widgets5-dev      
     sudo apt-get install qt5-qmake              
     sudo apt-get install qtdeclarative5-dev     
     ```
     Additionally, this package requires MAVROS as a dependency. For installation and usage instructions, please refer to: https://github.com/mavlink/mavros.git.
3. Build the project:

   ```bash
   catkin build 
   ```

   **Estimated duration**: Approximately 5-15 minutes (depending on hardware performance).



# Demo Execution

### 1. Disturbance estimation

- **disturbance_est**: This node can be directly used for disturbance estimation. Once the motion capture system publishes the position and attitude information of both the MAVs, the node will automatically calculate the magnitude of disturbance forces and moments during docking process.

  - Usage:

    ```bash
    rosrun disturbance_estimation disturbance_est
    ```

  - Output: Publishes the disturbance force and torque estimates to `/mavros/disturbance_estimate/force` and `/mavros/disturbance_estimate/torque` respectively.

- **motor_state_uart**: This node is responsible for publishing motor status information, including rotation speed, current, voltage, and temperature readings. Its operation depends on the [T-Motor ALPHA Series ESC Data Acquisition Card](https://item.taobao.com/item.htm?_u=a22o13el23c8&id=643443876894&pisk=guX4s6m2T-e2Z78viT9Z8Yk81dJvKdzQSOT6jGjMcEY06nwiQGSHfoUTD_-G5aETDKsM7N7WyAt_MPHNzZshGI_1GFxMrajjhFwv_NSCJNM_HstiQas9sN6NXA-GjNEv5oF5Dip9IyaQQJsADPu1bdW2mbcl2himm7Omg8SbVyaQdRPmqdz4RNMFacekYHvMmCDiZu-emnvMsEAu4hxsofDcSuryDHkiSnYiq8x9rfDMoFxkrnKKsVcDSuSkyhkMIdbMEu-tgT_AW2tHiu0xcOTtXAAJ-iYrIvmpoIX7pUMiuTx2JySXz3t5UnR2-CbCUn6cPGXfHQFqidI5TN5G-zGXo6-FzB148x7FkhjwjZyKYUXVbTRANqcDYK82ttbovYIes_Wyht4t9gsyoBJ5N7UX1K7V9ed0wzQN4E_G3Q04PF1CVtAP-zMPWC5GhLX0rJSPTVKle_WOgVcwi3KyRury_zjItXZy8Oht6QAH4eZvsfh9i3KyRurr6fdxK38QDCf..&spm=a1z09.2.0.0.78ef2e8dyWEVzq&skuId=4633424884192) hardware device.

  - Usage:

    ```bash
    rosrun disturbance_estimation motor_state_uart
    ```

    The port number and baud rate may vary, and users need to modify them according to their actual device configuration.

  - Output: Publishes motor status data to the `/motor_state` topic.

- **disturbance_est_imu**:  This node is designed for ground truth disturbance calculation. Before use, ensure the motor_state_uart node is already running.

  - Usage:

    ```bash
    #Set/Configure the parameters based on the actual rotor speed-thrust model.
    #<node pkg="disturbance_estimation" type="disturbance_est_imu" name="disturbance_est_imu_node" output="screen">
    #    <param name="p1" type="double" value="-6.939e-7" />
    #    <param name="p2" type="double" value="3.127e-4" />
    #    <param name="p3" type="double" value="-0.2451" />
    #    <param name="p4" type="double" value="1.1122" />
    #    <param name="m" type="double" value="7.844" />
    #</node>
    # Then
    roslaunch disturbance_estimation start.launch
    ```

    Among these parameters, p1-p4 are derived from the curve fitting results of motor speed versus thrust, while m represents the mass of the toolbox MAV. These parameters require adjustment based on actual operating conditions.

  - Output: Publishes disturbance force and torque ground truth estimates to /disturbance_truth/force and /disturbance_truth/torque respectively.

### 2. Visual sensing

- **`test_realsense`**: This node is designed to capture real-time images from the camera and publish the image data as a ROS topic.

  - Usage:

    ```bash
    rosrun visual_sensing test_realsense
    ```

  - Output: Publishes the image topic `/camera/realsense`.

- **`cali_camera`**: This node performs camera hand-eye calibration. It requires both a motion capture system and a specially designed calibration target (see diagram below).

  <img src="images\fig_cail.jpg" width="60%">

  Before running this node, it is necessary to first select the output poses of the manipulator MAV, calibration tool, and camera in the motion capture system. The corresponding topics are `/vicon/Turing/Turing`, `/vicon/ruler/ruler`, and `/vicon/realsense/realsense`, respectively. Note that when placing the `calibration tool`, its coordinate system must align with the toolbox MAV's coordinate system. Then, run the following program to generate the parameter file `calibration_trans_drone_cam.txt`.

  - Usage:

    ```bash
    rosrun visual_sensing cali_camera
    ```

 - **`realsense`**: This node is used to obtain the relative distance information between the upper and lower MAVs.


- **`realsense`**: This node is used to obtain the relative distance information between the upper and lower MAVs.

  Before using this node, first check and modify the parameters in line 580 of the `realsense.cc` file to ensure the file exists and the path is correct. Copy the calibration results from the previous step into this file. After running `catkin build`, use the following command to perform relative position estimation between the two MAVs.

  - Usage:

    ```bash
    rosrun visual_sensing realsense -i x -f y
    ```

  - **x**: Indicates the x-th hole, with possible values of 0, 1, 2, or 3. If not set, the default value is 1.

  - **y**: Determines whether to save real-time images locally. If set to 0, no images are saved. If set to a non-zero value, images are saved to the path specified in line 379 of the `realsense.cc` file.

  - Output: Publishes the relative position topic `/tool_box_pos`.

  <img src="images\fig_docking_with_visual_sensing.jpg" width="60%">

### 3. Trajectory planning

Before running this package, you need task planning result.

- **Input**: corridor.yaml (generated from task planning)
- **Output**: The trajectory_result provides the MAV's real-time desired position, desired velocity, and desired acceleration.

Run the following code to launch the trajectory planning.

```bash
     roslaunch trajectory_generation trajectory_generation.launch
```


### 4. Task planning

Run the following code to launch the QT interface for task planning.
```bash
roslaunch task_planning task_planning.launch
```
<img src="images\Task_Planning.png" width="60%">

**Task Planning Workflow:** First, select a MAV ID from the dropdown menu on the right, then click on the visualization interface to set waypoints—each click adds a new waypoint to the table below (where parameters like task type and duration can be modified), with repeated clicks constructing the complete mission sequence. To program additional MAVs, simply select different IDs and repeat the waypoint editing process. After configuring all MAV trajectories, click the Collision Detectionbutton to automatically validate paths and receive conflict alerts. Once collision-free operation is confirmed, click Generate Taskto output the multi-MAV coordination configuration file corridor.yaml.







