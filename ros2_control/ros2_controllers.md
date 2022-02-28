# Controllers
`ros2_control` framework uses `namespaces` to sort controllers according to the type of interface they use. Standard controllers use [common hardware interface definitions](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp), like:
- `position_controllers`: `hardware_interface::HW_IF_POSITION`
- `velocity_controllers`: `hardware_interface::HW_IF_VELOCITY`
- `acceleration_controllers`: `hardware_interface::HW_IF_ACCELERATION`
- `effort_controllers`: `hardware_interface::HW_IF_EFFORT`
- ...

## Available [controllers](https://github.com/ros-controls/ros2_controllers)
- `position_controllers`: collection of controllers that use the `position` command interface, but may accept different joint-level commands at controller level (e.g. control position to achieve a set velocity.)
- `velocity_controllers`: collection of controllers that use the `velocity` command interface, but may accept different joint-level commands at controller level (e.g. control `velocity` to achieve a set `position`.)
- `effort_controllers`: collection of controllers that use the `effort` command interface, but may accept different joint-level commands at controller level (e.g. control `effort` to achieve a set `position`.)
- `diff_drive_controller`: controller for mobile robots with differential drive.
- `forward_command_controller`: {descrizione troppo simile a `effort`?}
- [`joint_trajectory_controller`](http://control.ros.org/ros2_controllers/joint_trajectory_controller/doc/userdoc.html): controller for executing joint state trajectories on a group of joints. Trajectories are specified as a set of waypoints to be reached at specific time instants. Waypoints may consist of `position` and optionally `velocity` or `acceleration`. The controller can work with different **trajectory representations**. By default a spline interpolator is provided. Depending on the way-point specification, the interpolator uses:
  - **Linear** strategy: only position is specified. Guarantees continuity at position level. It's discouraged because of discontinuous velocity.
  - **Cubic** strategy: position and velocity are specified. Guarantees continuity at velocity level.
  - **Quintic** strategy: position, velocity and acceleration are specified. Guarantees continuity at acceleration level.

## Available broadcasters
- [`joint_state_broadcaster`](http://control.ros.org/ros2_controllers/joint_state_broadcaster/doc/userdoc.html): reads all state interfaces and reports them on `/joint_states` and `/dynamic_joint_states`. It's not a real controller and takes no command.
- [IMU sensor broadcaster](http://control.ros.org/ros2_controllers/imu_sensor_broadcaster/doc/userdoc.html): the controller is a wrapper around IMUSensor semantic component.

## Writing a new controller
As hardware interfaces, controllers are libraries dynamically loaded by CM through `pluginlib`. The [implementation](http://control.ros.org/ros2_controllers/doc/writing_new_controller.html) is very similar to that of an hardware interface, but the `update` method is used instead of `read` and `write` methods. Note it should be implemented with **real-time** constraints in mind. When this method is called, the state interfaces have the most recent values from the hardware, and new commands for the hardware should be written into command interfaces.

## Configuring controller and CM (YAML)
In the `bringup` package, include a `config/<robot_name_controllers>.yaml` file. Inside, configure the controller manager and controllers parameters.
```yaml
#example
controller_manager:
    
    ros__parameters:
        update_rate: <Hz>
    
    joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster
    
    forward_position_controller:
        type: forward_command_controller/ForwardCommandController
    
forward_position_controller:
    ros__parameters:
        joints:
            - joint1
            - joint2
        interface_name: position
#...
```
See more examples on how to write it. [Diffbot](https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_bringup/config/diffbot_controllers.yaml) contains a custom controller.