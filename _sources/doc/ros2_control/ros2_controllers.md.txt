# Controllers
The controllers in the `ros2_control` framework have the same functionality as defined in control theory. They compare a reference value with the measured output and, based on this error, calculate a system’s input.

The controlles are objects derived from `controller_interface` [package](https://github.com/ros-controls/ros2_control/tree/master/controller_interface) in `ros2_control`.


As with [hardware resources/interfaces](ros2_control_HI.md), controllers are dynamic libraries exported as plugins using `pluginlib`-library. Moreover, controllers’ lifecycle is based on the [LifecycleNode](https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp) Class, implementing the state-machine [lifecycle](../Standard_Libraries/rcl.md#lifecycle).

When executing the control-loop `update()` method is called. The method can access the latest hardware states and enable the controller to write the hardware’s command interfaces.

## Controller interface
Once system is bootstrapped and a controller is loaded, it can claim logical components and access their interfaces.
- **Generic access**: a controller can access a single interface value via a query to the RM for the respective key. For example, if `joint_name/position` is available, it enables the controller to claim the handle that allows to set `position` values on `joint_name` during the controller execution.
  ```cpp
  //example
  void MyController::init(... resource_manager)
  {
    InterfaceCommandHandle joint_name_position_cmd = resource_manager->claim_command_interface("joint_name/position");
    InterfaceStateHandle joint_name_velocity_state = resource_manager->claim_state_interface("joint_name/velocity");
  }
  ```
- **Semantic components**: for more complex setups, there might be quite many handles accumulated. Semantic components wrap a non-zero amount of keys and provide a more meaningful API on top of it. The implementation of classes for the logical components provides more insights on how to interpret the keys, that are pointers of the respective interfaces.
  ```cpp
  //example
  void MyController::init(... resource_manager)
  {
    FTSensor6D ft_sensor(resource_manager, 
      "sensor1/fx", "sensor1/fy", "sensor1/fz",   //forces
      "sensor1/tx", "sensor1/ty", "sensor1/tz");  //torques

    std::vector<double> torque = ft_sensor.get_torque_values();

    Camera cam(resource_manager, "camera1/data", "camera1/size");
    cv::Mat img = cam.get_image();

  }
  ```
## Standard controllers
`ros2_control` framework uses `namespaces` to sort controllers according to the type of interface they use. Standard controllers use [common hardware interface definitions](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp), like:
- `position_controllers`: `hardware_interface::HW_IF_POSITION`
- `velocity_controllers`: `hardware_interface::HW_IF_VELOCITY`
- `acceleration_controllers`: `hardware_interface::HW_IF_ACCELERATION`
- `effort_controllers`: `hardware_interface::HW_IF_EFFORT`
- ...

Available [`ros2_controllers`](https://github.com/ros-controls/ros2_controllers) (most still need to be ported from ROS to ROS2, check [`ros_controllers`](https://github.com/ros-controls/ros_controllers)):
|controller|description|
|-|-|
`forward_command_controller`| Generic type of controller that simply forwards the reference to its output (joint) without making any processing. It has a `type` parameter that is used to specialize it into the different interfaces (`position`, `velocity`...).
`position_controllers`| Collection of controllers that use the `position` **command interface**, but may accept different joint-level commands at controller level (e.g. control `position` to achieve a set `velocity`.)
`velocity_controllers`|Collection of controllers that use the `velocity` command interface, but may accept different joint-level commands at controller level (e.g. control `velocity` to achieve a set `position`.)
`effort_controllers`|Collection of controllers that use the `effort` command interface, but may accept different joint-level commands at controller level (e.g. control `effort` to achieve a set `position`.)
`diff_drive_controller`|controller for mobile robots with differential drive.
[`joint_trajectory_controller`](http://control.ros.org/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)|Controller for executing joint state trajectories on a group of joints. Trajectories are specified as a set of waypoints to be reached at specific time instants. Waypoints may consist of `position` and optionally `velocity` or `acceleration`. The controller can work with different **trajectory representations**. By default a spline interpolator is provided.

The available **broadcasters** are:
|broadcaster|description|
|-|-|
[`joint_state_broadcaster`](http://control.ros.org/ros2_controllers/joint_state_broadcaster/doc/userdoc.html)| reads all state interfaces and reports them on `/joint_states` and `/dynamic_joint_states`. It's not a real controller and takes no command.
[IMU sensor broadcaster](http://control.ros.org/ros2_controllers/imu_sensor_broadcaster/doc/userdoc.html)| the controller is a wrapper around IMUSensor semantic component.

## Writing a controller
As hardware interfaces, controllers are libraries dynamically loaded by CM through `pluginlib`. The [implementation](http://control.ros.org/ros2_controllers/doc/writing_new_controller.html) is very similar to that of an hardware interface, but the `update` method is used instead of `read` and `write` methods. Note it should be implemented with **real-time** constraints in mind. When this method is called, the state interfaces have the most recent values from the hardware, and new commands for the hardware should be written into command interfaces.

### Configuring controller and CM (YAML)
#TODO

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