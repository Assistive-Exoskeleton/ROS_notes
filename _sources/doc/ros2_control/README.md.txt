# ROS2 Control

```{toctree}
---
maxdepth: 1
hidden:
---

ros2_control_concepts
ros2_control_URDF
ros2_control_HI
ros2_control_UI
ros2_controllers
```


Theory and concepts
- [x] [`ros2_control` concepts](ros2_control_concepts.md)

Programming and application
- [x] [`ros2_control` URDF](ros2_control_URDF.md)
- [x] [`ros2_control` hardware interface](ros2_control_HI.md)
- [x] [`ros2_control` user interfaces](ros2_control_UI.md)
- [x] [`ros2_control` controllers](ros2_controllers.md)


<h2> Installation </h2>

`ros2_control` is now released for galactic:
```sh
$ sudo apt install ros-galactic-ros2-control
$ sudo apt install ros-galactic-ros2-controllers
# maybe also control-toolbox?
```

<h2> Workflow </h2>

Usually the main packages are:

1. `description` package, store URDF description files, rviz configurations and meshes
2. `bringup` package, holds launch files and runtime configurations for robots
3. `hardware` package, implements the hardware interfaces
4. `controllers` package, implements custom controllers.

The workflow can be thought as follows:
1. Implement the `description` package. It contains:
   - The [robot URDF](../Robot_description/URDF.md) ([XACRO](../Robot_description/XACRO.md)) defining the robot structure and physical properties.
   - The [`ros2_control` URDF](ros2_control_URDF.md) (XACRO) defining the logical components of the robot and how they're managed by `ros2_control` framework.
2. Implement the [`hardware` package](ros2_control_HI.md), containing the hardware resources that will be requested by the controllers and managed by the RM.
3. Implement the `controllers`, either in the bringup package or in a more specific package for controllers if they're built. It contains:
   - The [controllers `YAML`](ros2_controllers.md#configuring-controller-and-cm-yaml) config file, declaring the CM and controllers configuration (i.e. `update` rate, controllers to be activated and their parameters...)
   - optionally, write [custom controllers](ros2_controllers.md) if they're not provided by `ros2_controllers` package. 
4. Implement the `bringup` package, with launch files starting `CM` node and controllers using a default [ros2_control node](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp)

