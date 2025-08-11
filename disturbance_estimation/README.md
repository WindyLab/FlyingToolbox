# Disturbance estimation

This package is used for estimating the downwash disturbance received by the lower MAV during vertical-stack proximal cooperation.

### Hardware
The onboard computer of this system utilizes the NVIDIA AI development kit Jetson Xavier NX, running Ubuntu 20.04 and the Noetic ROS robot operating system.  

If disturbance ground truth collection is not required, only the disturbance estimation function can be used. In this case, it is sufficient to publish the position and attitude information of the working UAV and the toolbox UAV in the motion capture system to enable disturbance estimation.  

If disturbance ground truth calculation is needed, motor speed feedback data and IMU data are required. The system we tested employs the [T-Motor 501-X motor and ESC integrated kit](https://item.taobao.com/item.htm?_u=a22o13el43d2&id=606577444356&pisk=g1czsB_zQQdrNiZ8EjNFbs5vdWP81W-6xXZQ-203PuqkpzOhT2ugRUKpyxznAmLpy7g3YDobD6aBeHCE00giF8iIFk43mm05NkO8LDusXD1BwYahTmgLKDGEJ6zn-DL8AUpjy4FLtht6TCg-yHSI8Wlzro0mkyslqKwlUIuWlht61BJloW-yfD1Z3KR07PV3rJfhnEzarzV3Ku0Duy4Cq9foxE84yP5hxzqhoI4Lm9q3-zjcmPUGt_4u-Eu0DP5utWm3iEzKgvm-vGagEEjRPXZdWHyuj4qVt1bTq8mw6oChFj4zXO3z0qgjgzyzjJmsgzGoc2cSw-pPEW3jQDknjn67qAzZ0RMyb_oZ2y0U-0RO7ocr8jy-hgXu77rzIbmcWs3aKxl4NbxdBqg4qRPjhKK7d7orBlekHnnEuuina-jyckMslb2qjn1qvJknNSckmCSyaMUmMxlKUMXUErU4fE84Ln01IOT4bXBdp-2gulT8K9BLErU4fE8Vp9eRsrr6yJf..&spm=a1z09.2.0.0.78ef2e8dyWEVzq) as the power unit, which supports motor speed feedback. Combined with the [ALPHA DATA LINK v2 acquisition system](https://item.taobao.com/item.htm?_u=a22o13el23c8&id=643443876894&pisk=guX4s6m2T-e2Z78viT9Z8Yk81dJvKdzQSOT6jGjMcEY06nwiQGSHfoUTD_-G5aETDKsM7N7WyAt_MPHNzZshGI_1GFxMrajjhFwv_NSCJNM_HstiQas9sN6NXA-GjNEv5oF5Dip9IyaQQJsADPu1bdW2mbcl2himm7Omg8SbVyaQdRPmqdz4RNMFacekYHvMmCDiZu-emnvMsEAu4hxsofDcSuryDHkiSnYiq8x9rfDMoFxkrnKKsVcDSuSkyhkMIdbMEu-tgT_AW2tHiu0xcOTtXAAJ-iYrIvmpoIX7pUMiuTx2JySXz3t5UnR2-CbCUn6cPGXfHQFqidI5TN5G-zGXo6-FzB148x7FkhjwjZyKYUXVbTRANqcDYK82ttbovYIes_Wyht4t9gsyoBJ5N7UX1K7V9ed0wzQN4E_G3Q04PF1CVtAP-zMPWC5GhLX0rJSPTVKle_WOgVcwi3KyRury_zjItXZy8Oht6QAH4eZvsfh9i3KyRurr6fdxK38QDCf..&spm=a1z09.2.0.0.78ef2e8dyWEVzq&skuId=4633424884192), the speed and temperature data of multiple motors can be transmitted to the onboard computer via serial communication. The IMU sensor can be the one integrated into the flight controller, but our tests revealed that its noise level is relatively high, leading to significant errors in the obtained ground truth. We recommend using the [HWT905 IMU](https://detail.tmall.com/item.htm?abbucket=20&id=598664650003&mi_id=rY7NVMPyiDSRF7ikY3XbWG7n6kimayiMKO03bL11Q_GXctqFmoiCLXO3KC6-5WytpaBv7pU7eNnbZRJFg21fo6IE5feMqmVfRZDd56BVSng&ns=1&skuId=4439943127599&spm=a21n57.1.hoverItem.1&utparam=%7B%22aplus_abtest%22%3A%22567b61e09056866734aae87654842d08%22%7D&xxc=taobaoSearch) or a higher-precision IMU for better accuracy.

### Method

The estimation of downwash-induced disturbance forces is accomplished using an offline-trained neural network model. The training process for this neural network consists of two steps: First, a Gaussian profile is employed to model the downwash flow field. Subsequently, 15 trajectories containing ground truth disturbance data are used as the training set to complete the neural network training. Once the neural network model is obtained, it can be utilized to estimate the downwash disturbance between two MAVs during vertical stacked flight operations.

<img src="images\fig_disturbace_method.jpg" width="100%">

### Description for the nodes

- **disturbance_est**: This node can be directly used for disturbance estimation. Once the motion capture system publishes the position and attitude information of both the working UAV and the toolbox UAV, the node will automatically calculate the magnitude of disturbance forces and moments during the dual-UAV docking process.

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
    
    Among these parameters, p1-p4 are derived from the curve fitting results of motor speed versus thrust, while m represents the mass of the toolbox UAV. These parameters require adjustment based on actual operating conditions.
  - Output: Publishes disturbance force and torque ground truth estimates to /disturbance_truth/force and /disturbance_truth/torque respectively.
