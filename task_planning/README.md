# Task Planning

This package facilitates mission planning operations for the FlyingToolbox system.

### System Requirements

This package operates on Ubuntu 18.04 or 20.04 systems with both [ROS installed](https://wiki.ros.org/ROS/Installation) and [Mavros configured](https://docs.px4.io/v1.14/zh/ros/mavros_installation.html), requiring at least 2GB of RAM. Our testing has verified stable and smooth performance on both an Intel NUC12WSHi5 onboard computer and an NVIDIA Jetson Xavier NX development kit featuring a 6-core NVIDIA Carmel ARMv8.2 64-bit CPU cluster. 

### Install dependencies & build 

This package requires MAVROS as a dependency. For installation and usage instructions, please refer to: https://github.com/mavlink/mavros.git. Execute the following commands to  install all dependencies.

```bash
# Qt5 
sudo apt-get install qtbase5-dev             
sudo apt-get install libqt5widgets5
sudo apt-get install qt5-qmake              
sudo apt-get install qtdeclarative5-dev
```

### Demo execution

Run the following code to launch the QT interface ( see following figure) for task planning.

```bash
roslaunch task_planning task_planning.launch
```

<div align="center">
<img src="readme_images\Task_Planning.png" width="60%">
</div>

Task Planning Workflow: First, select a MAV ID from the dropdown menu on the right, then click on the visualization interface to set waypoints—each click adds a new waypoint to the table below (where parameters like task type and duration can be modified), with repeated clicks constructing the complete mission sequence. To program additional MAVs, simply select different IDs and repeat the waypoint editing process. After configuring all MAV trajectories, click the Collision Detectionbutton to automatically validate paths and receive conflict alerts. Once collision-free operation is confirmed, click Generate Task to output the multi-MAV coordination configuration file corridor.yaml.

