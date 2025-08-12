# Visual Sensing

This package is deployed on the manipulator MAV to enable perception of the end-effector tool positions on the toolbox MAV.

### System requirements

This package operates on Ubuntu 18.04 or 20.04 systems with both [ROS installed](https://wiki.ros.org/ROS/Installation) and [Mavros configured](https://docs.px4.io/v1.14/zh/ros/mavros_installation.html), requiring at least 2GB of RAM. Our testing has verified stable and smooth performance on both an Intel NUC12WSHi5 onboard computer and an NVIDIA Jetson Xavier NX development kit featuring a 6-core NVIDIA Carmel ARMv8.2 64-bit CPU cluster.  This package must be used in conjunction with onboard cameras. We have successfully tested it with the following cameras: Intel RealSense L515 and D435i.

### Install dependencies & build 

This package utilizes the OpenCV library and Intel RealSense SDK for its core functionality. The following dependencies are required for proper operation:

  ```bash
sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf ros-noetic-eigen-conversions
sudo apt-get install librealsense2-dev #RealSense SDK
sudo apt-get install libopencv-dev #OpenCV
  ```

 Build the package:

   ```bash
catkin build 
   ```

### Demo execution

- **`test_realsense`**: This node is designed to capture real-time images from the camera and publish the image data as a ROS topic. Its operation depends on the Intel RealSense camera.

  - Usage:

    ```bash
    rosrun visual_sensing test_realsense
    ```

  - Output: Publishes the image topic `/camera/realsense`.

- **`cali_camera`**: This node performs camera hand-eye calibration. It requires both a motion capture system and a specially designed calibration target (see the figure below).

<div align="center">
  <img src="readme_images\fig_cail.jpg" width="60%">
</div>


  Before running this node, it is necessary to first select the output poses of the manipulator MAV, calibration tool, and camera in the motion capture system. The corresponding topics are `/vicon/Turing/Turing`, `/vicon/ruler/ruler`, and `/vicon/realsense/realsense`, respectively. Note that when placing the `calibration tool`, its coordinate system must align with the toolbox MAV's coordinate system. Then, run the following program to generate the parameter file `calibration_trans_drone_cam.txt`.

  - Usage:

    ```bash
    rosrun visual_sensing cali_camera
    ```

- **`realsense`**: This node is used to obtain the relative distance information between the upper and lower MAVs (see the figure below). Before using this node, first check and modify the parameters in line 580 of the `realsense.cc` file to ensure the file exists and the path is correct. Copy the calibration results from the previous step into this file. After running `catkin build`, use the following command to perform relative position estimation between the two MAVs.

  - Usage:

    ```bash
    rosrun visual_sensing realsense -i x -f y
    ```

  - **x**: Indicates the x-th hole, with possible values of 0, 1, 2, or 3. If not set, the default value is 1.

  - **y**: Determines whether to save real-time images locally. If set to 0, no images are saved. If set to a non-zero value, images are saved to the path specified in line 379 of the `realsense.cc` file.

  - Output: Publishes the relative position topic `/tool_box_pos`.

<div align="center">
  <img src="readme_images\fig_docking_with_visual_sensing.jpg" width="60%">
</div>

