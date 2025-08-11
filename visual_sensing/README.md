# visual sensing

### Hardware 

This package must be used in conjunction with onboard cameras. We have successfully tested it with the following cameras: **Intel RealSense L515** and **D435i**.

### Method
The design of the onboard vision system involves six steps: three offline steps (QR code array design based ArUco https://www.2weima.com/aruco.html, camera calibration, and hand-eye calibration) and three online steps (QR code detection, 6D pose estimation, and filtering at 50 Hz).

<img src="images\fig_method.jpg" width="100%">

### Description for the nodes

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

