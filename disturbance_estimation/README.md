# Disturbance Estimation

This package is used for estimating the downwash disturbance received by the lower MAV during vertical-stack proximal cooperation.

### System requirements

This package operates on Ubuntu 18.04 or 20.04 systems with both [ROS installed](https://wiki.ros.org/ROS/Installation) and [Mavros configured](https://docs.px4.io/v1.14/zh/ros/mavros_installation.html), requiring at least 2GB of RAM. Our testing has verified stable and smooth performance on both an Intel NUC12WSHi5 onboard computer and an NVIDIA Jetson Xavier NX development kit featuring a 6-core NVIDIA Carmel ARMv8.2 64-bit CPU cluster. The disturbance_estimation package supports collecting ground truth data of disturbance forces acting on the UAV. When ground truth force data  is unnecessary and the package is solely used for disturbance  estimation, it only requires position information of both the toolbox  MAV and manipulator MAV, which we recommend obtaining through a motion  capture system. For cases requiring ground truth disturbance force  collection, in addition to the aforementioned position data, the package also needs rotor speed information from the toolbox MAV and its  acceleration data. We recommend using a combination of a [T-Motor 501-X motor and ESC integrated kit](https://item.taobao.com/item.htm?_u=a22o13el43d2&id=606577444356&pisk=g1czsB_zQQdrNiZ8EjNFbs5vdWP81W-6xXZQ-203PuqkpzOhT2ugRUKpyxznAmLpy7g3YDobD6aBeHCE00giF8iIFk43mm05NkO8LDusXD1BwYahTmgLKDGEJ6zn-DL8AUpjy4FLtht6TCg-yHSI8Wlzro0mkyslqKwlUIuWlht61BJloW-yfD1Z3KR07PV3rJfhnEzarzV3Ku0Duy4Cq9foxE84yP5hxzqhoI4Lm9q3-zjcmPUGt_4u-Eu0DP5utWm3iEzKgvm-vGagEEjRPXZdWHyuj4qVt1bTq8mw6oChFj4zXO3z0qgjgzyzjJmsgzGoc2cSw-pPEW3jQDknjn67qAzZ0RMyb_oZ2y0U-0RO7ocr8jy-hgXu77rzIbmcWs3aKxl4NbxdBqg4qRPjhKK7d7orBlekHnnEuuina-jyckMslb2qjn1qvJknNSckmCSyaMUmMxlKUMXUErU4fE84Ln01IOT4bXBdp-2gulT8K9BLErU4fE8Vp9eRsrr6yJf..&spm=a1z09.2.0.0.78ef2e8dyWEVzq) and a [ALPHA DATA LINK v2 acquisition system](https://item.taobao.com/item.htm?_u=a22o13el23c8&id=643443876894&pisk=guX4s6m2T-e2Z78viT9Z8Yk81dJvKdzQSOT6jGjMcEY06nwiQGSHfoUTD_-G5aETDKsM7N7WyAt_MPHNzZshGI_1GFxMrajjhFwv_NSCJNM_HstiQas9sN6NXA-GjNEv5oF5Dip9IyaQQJsADPu1bdW2mbcl2himm7Omg8SbVyaQdRPmqdz4RNMFacekYHvMmCDiZu-emnvMsEAu4hxsofDcSuryDHkiSnYiq8x9rfDMoFxkrnKKsVcDSuSkyhkMIdbMEu-tgT_AW2tHiu0xcOTtXAAJ-iYrIvmpoIX7pUMiuTx2JySXz3t5UnR2-CbCUn6cPGXfHQFqidI5TN5G-zGXo6-FzB148x7FkhjwjZyKYUXVbTRANqcDYK82ttbovYIes_Wyht4t9gsyoBJ5N7UX1K7V9ed0wzQN4E_G3Q04PF1CVtAP-zMPWC5GhLX0rJSPTVKle_WOgVcwi3KyRury_zjItXZy8Oht6QAH4eZvsfh9i3KyRurr6fdxK38QDCf..&spm=a1z09.2.0.0.78ef2e8dyWEVzq&skuId=4633424884192) to  acquire rotor speed measurements, while employing a HWT905 IMU for acceleration  data acquisition.

### Install dependencies & build 

 This package utilizes MATLAB Coder for neural network code generation. For detailed technical specifications, please refer to the documentation at: https://ww2.mathworks.cn/help/coder/deep-learning-code-generation-fundamentals.html?s_tid=CRUX_lftnav. Additionally, the following dependencies must be installed to execute the generated code.

 ```bash
sudo apt-get install ros-noetic-serial ros-noetic-eigen-conversions
sudo apt-get install libomp-dev      # OpenMP
 ```

 Build the package:

   ```bash
catkin build 
   ```

### Demo execution

This package contains three nodes, whose functionalities and usage methods are described below:

- **disturbance_est**: This node is used for disturbance estimation. Once the motion capture system publishes the pose information of both the MAVs, the node will automatically calculate the magnitude of disturbance forces and torque during docking process.

  - Usage:

    ```bash
    roslaunch disturbance_estimation disturbance_estimation.launch
    ```

  - Output: Publishes the disturbance force and torque estimates to `/mavros/disturbance_estimate/force` and `/mavros/disturbance_estimate/torque` respectively.

- **motor_state_uart**: This node is responsible for publishing motor status. Its operation depends on the [T-Motor ALPHA Series ESC Data Acquisition Card](https://item.taobao.com/item.htm?_u=a22o13el23c8&id=643443876894&pisk=guX4s6m2T-e2Z78viT9Z8Yk81dJvKdzQSOT6jGjMcEY06nwiQGSHfoUTD_-G5aETDKsM7N7WyAt_MPHNzZshGI_1GFxMrajjhFwv_NSCJNM_HstiQas9sN6NXA-GjNEv5oF5Dip9IyaQQJsADPu1bdW2mbcl2himm7Omg8SbVyaQdRPmqdz4RNMFacekYHvMmCDiZu-emnvMsEAu4hxsofDcSuryDHkiSnYiq8x9rfDMoFxkrnKKsVcDSuSkyhkMIdbMEu-tgT_AW2tHiu0xcOTtXAAJ-iYrIvmpoIX7pUMiuTx2JySXz3t5UnR2-CbCUn6cPGXfHQFqidI5TN5G-zGXo6-FzB148x7FkhjwjZyKYUXVbTRANqcDYK82ttbovYIes_Wyht4t9gsyoBJ5N7UX1K7V9ed0wzQN4E_G3Q04PF1CVtAP-zMPWC5GhLX0rJSPTVKle_WOgVcwi3KyRury_zjItXZy8Oht6QAH4eZvsfh9i3KyRurr6fdxK38QDCf..&spm=a1z09.2.0.0.78ef2e8dyWEVzq&skuId=4633424884192) hardware device.

  - Usage:

    ```bash
    rosrun disturbance_estimation motor_state_uart _port:="/dev/ttyUSB0" _baud:=115200
    ```

    The port number and baud rate may vary, and users need to modify them according to their actual device configuration.

  - Output: Publishes motor status data to the `/motor_state` topic.

- **disturbance_est_imu**:  This node is designed for ground truth disturbance calculation. Before running this node, ensure that the **motor_state_uart** node is already running.

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
    roslaunch disturbance_estimation disturbance_groundtruth.launch
    ```

    Among these parameters, p1-p4 are derived from the curve fitting results of motor speed versus thrust, while m represents the mass of the toolbox MAV. These parameters require adjustment based on actual operating conditions.

  - Output: Publishes disturbance force and torque ground truth estimates to /disturbance_truth/force and /disturbance_truth/torque respectively.

