# `ros2_control` workflow
 1. CM and controllers configuration (YAML)
 2. Definition of an HW interface class for HW components
 3. [URDF description](ros2_control_URDF.md) of HW components

---
---
---
---
---
## Appunti

Usually 3 packages:
1. `description` package, store URDF description files, rviz configurations and meshes
2. `bringup` package, holds launch files and runtime configurations for robots
3. `hardware` package, implements the hardware interfaces

The launch file starts the `CM` node, either using a default [`ros2_control` node](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp) (recommended) (#TODO see example rrbot position only launch)

> NOTE: You could alternatively use a script to create setup a [skeleton of the “hardware_interface” package by using the scripts](https://stoglrobotics.github.io/ros_team_workspace/use-cases/setup_robot_ros2_control_hardware.html) provided by one of our maintainers.