<div align="center">
    <h1>Proximal cooperative aerial manipulation with vertically stacked drones</h1>
    <br>
    <a href="" target="_blank">Huazi Cao</a>,
	<a href="" target="_blank">Jiahao Shen</a>
	<a href="https://oliviazhang1996.github.io " target="_blank">Yin Zhang</a>,
	<a href="" target="_blank">Zheng Fu</a>,
	<a href=" " target="_blank">Cunjia Liu</a>,
	<a href=" " target="_blank">Sihao Sun</a>,
	<a href="https://shiyuzhao.westlake.edu.cn/Shiyu_Zhao.htm" target="_blank">Shiyu Zhao</a>,
    <p>
        <h45>
            <br>
           		<img src='figure/LOGOzhongban.png' alt='Windy Lab' width="20%">
            <br>
        </h5>
    </p>
</div>




This GitHub repository provides open-source materials for **FlyingToolbox**, including mechanical design files and source code for disturbance estimation, visual sensing, trajectory generation, and task planning.


#  Introduction for FlyingToolbox
 **FlyingToolbox** is a cooperative aerial manipulation system and consists of a toolbox MAV and a manipulator MAV.  The toolbox MAV carries a toolbox that contains a set of end-effector tools for dedicated tasks such as grasping, cutting, and inspection. The manipulator MAV flies above the toolbox MAV and can autonomously dock with any tool using its robotic arm. After completing a manipulation task, the manipulator MAV can return the tool back to the toolbox MAV or switch to another to perform a different task. 

<img src="figure\fig_structure_sys.jpg" width="100%">



# File Summary

The correspondence between folder names and their contents is as follows:

<div align="center">

| Contents | folder names |
|:--------:|:----:|
| Files for the mechanical design | 3D Modeling Files |
| Source code used for disturbance estimation | disturbance_estimation |
| Source code used for visual sensing | visual_sensing |
| Source code used for trajectory generation| trajectory_generation |
| Source code used for task planning | task_planning |

</div>


# System Requirements

### Hardware Requirements

This repository requires only a computer with enough RAM to support the operations defined by a user. For minimal performance, this will be a computer with about 2 GB of RAM. Computers can be either personal computers (PCs) or onboard computing devices such as NUC and NX systems.  For optimal performance, we recommend a computer with the following specs:

- RAM: 8+ GB  

- CPU: 4+ cores, 1.7GHz/core



### Software Requirements

The repository development version is tested on *Linux* operating systems. The developmental version of the package has been tested on the following systems: Ubuntu 18.04, Ubuntu 20.04. In addition, all code in the package  were developed for ROS (Melodic on Ubuntu 18.04 or Noetic on Ubuntu 20.04). 


# Installation Guide
We demonstrate the package installation using a Noetic/Ubuntu 20.04 environment as an example.

1. Clone this repository to your local workspace:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/WindyLab/FlyingToolbox.git
   ```

2. Install dependencies:

   - Disturbance estimation:
     ```bash
     sudo apt-get install ros-noetic-serial ros-noetic-eigen-conversions
     sudo apt-get install libomp-dev      # OpenMP
     ```
   - Visual sensing:
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

All packages in this repository must follow standard ROS operating procedures. The execution step for each package is as follows:

- Disturbance estimation:
     ```bash
     roslaunch disturbance_estimation start.launch
     ```
- Visual sensing:
     ```bash
     roscore # If the ROS master is not running
     rosrun visual_sensing realsense
     ```
- Trajectory generation:
     ```bash
     roslaunch trajectory_generation trajectory_generation.launch
     ```
- Task planning:
     ```bash
     roslaunch task_planning task_planning.launch 
     ```



