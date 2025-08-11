# Trajectory generation

This package is used to implement trajectory generation for MAVs. The operation of this package depends on the results from the task_planning package.

### Method
The operation procedure of the manipulator MAV is represented as a simple state machine with five states: move, wait, operate, mount tool, switch tool, and release tool. The operation of the toolbox MAV corresponds to two states: move and wait. Motion trajectories for both the manipulator MAV and toolbox MAV are generated using a Bezier curve-based method, which incorporates flight corridor and trajectory generation. Safe flight corridors are established by considering only static obstacles, ensuring that MAVs do not enter the same area simultaneously to avoid collisions.
<img src="images\fig_traj_gen_methoad.jpg" width="100%">



Before running this code, you need to save the task planning results as a YAML file in the launch folder and rename it to corridor.yaml.

- **Input**: corridor.yaml (generated from task planning)
- **Output**: The trajectory_result provides the MAV's real-time desired position, desired velocity, and desired acceleration.

