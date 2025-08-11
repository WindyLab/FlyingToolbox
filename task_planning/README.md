# Task Planning

This package facilitates mission planning operations for the FlyingToolbox system.

### Hardware

This package operates without requiring additional hardware components.

### Method

Developed based on task requirements and a state machine architecture, this package enables discrete waypoint planning while ensuring safe multi-MAV collaborative operations. The manipulator MAV's operational workflow is implemented as a five-state finite state machine (Move, Wait, Operate, Mount Tool, Switch Tool, and Release Tool), while the toolbox MAV follows a simplified two-state model (Move and Wait). All generated discrete waypoint paths for each MAV are stored in a corridor.yaml configuration file, maintaining spatial-temporal coordination parameters and safety constraints for the entire multi-rotor system.

<img src="images\fig_task_plan_methoad.jpg" width="100%">



### Description for the nodes

##### Trajectory Planning (Task Plan)

- Add waypoints by clicking

- Support for multi-UAV trajectory planning

- Allow editing and deleting trajectories

- Support collision detection

- Automatically generate task configuration files
  ![UAV_Monitor](.\images\Task_Planning.png)

The operational workflow proceeds as follows: First, select the UAV number from the dropdown menu on the right, then click any position on the visualization interface above to set waypoints. Each click adds new waypoint information to the table below, where parameters like task type and execution duration can be modified. Repeated clicking generates a complete task sequence. To program additional UAVs, simply select different identification numbers and repeat the waypoint editing process.

After completing all UAV task sequences, click the `Collision Detection` button to automatically perform collision verification. The system will display risk alerts if potential conflicts are detected.

When the trajectory validation confirms collision-free operation, clicking the `Generate Task` button produces the required `corridor.yaml` configuration file containing the coordinated multi-UAV mission parameters.

- Usage:
  ```bash
  roslaunch task_planning task_planning.launch
  ```

##### 	Real-time Monitoring (UAV_monitor)

- Display real-time UAV positions

- Coordinate transformation and visualization

- Position validity checks

  ![UAV_Monitor](.\images\UAV_Monitor.png)

  Modify the topic name in the `task_planning.launch` file to match the target UAV's designation that requires monitoring. Click the `Connect` button to establish a connection with the ROS system, and the corresponding UAV's position will be displayed in the interface shown above. The software can be exited by clicking `Quit`.

