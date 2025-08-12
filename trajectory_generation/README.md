# Trajectory Generation

This package is used to implement trajectory generation for MAVs. The operation of this package depends on the results from the task_planning package.

### System requirements

This package operates on Ubuntu 18.04 or 20.04 systems with both [ROS installed](https://wiki.ros.org/ROS/Installation) and [Mavros configured](https://docs.px4.io/v1.14/zh/ros/mavros_installation.html), requiring at least 2GB of RAM. Our testing has verified stable and smooth performance on both an Intel NUC12WSHi5 onboard computer and an NVIDIA Jetson Xavier NX development kit featuring a 6-core NVIDIA Carmel ARMv8.2 64-bit CPU cluster. 

### Install dependencies & build 

This package requires OSQP as a dependency. For installation and usage instructions, please refer to: https://github.com/osqp/osqp.git. Additionally, since it involves substantial computations, it also  depends on the ROS Eigen library. Execute the following commands to  install all dependencies.

```bash
sudo apt-get install ros-noetic-eigen-conversions
```

 Build the package:

   ```bash
catkin build 
   ```

### Demo execution

Before running this package, you need task planning result from the task_planning package.

Run the following code to launch the trajectory planning.

```bash
roslaunch trajectory_generation trajectory_generation.launch
```

  - Output: Publishes the reference via  `/trajectory_result` service messages.
