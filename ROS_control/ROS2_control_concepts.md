# ROS2_control
## Contents
- [ROS2_control](#ros2_control)
  - [Contents](#contents)
- [Architecture](#architecture)
  - [Controller manager](#controller-manager)
  - [Resource Manager](#resource-manager)
  - [Controllers](#controllers)
  - [Control loop](#control-loop)
  - [User interfaces](#user-interfaces)
- [Hardware components](#hardware-components)
  - [Hardware description in URDF](#hardware-description-in-urdf)
- [Running the framework](#running-the-framework)
# Architecture
The following figure shows the architecture:

![ros_control_architecture](./images/ros2_control_architecture.png)

## Controller manager
The `controller_manager` ([CM](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/controller_manager.cpp)) connects the **controllers** and **HW-abstraction** sides of ros2_control framework. It also serves as the entry-point for users through **ROS services**.

CM has two main purposes:
1. **Robot resource management**: CM has access to the **hardware components** (through [Resource Manager](#resource-manager)).
2. **Controller management**: CM manages the **lifecycle** (e.g., loading, activating, deactivating, unloading) and **updates** of controllers and the required interfaces.

The execution of the **control-loop** is managed by the CM’s `update()` method. The method reads data from the hardware components, updates outputs of all active controllers, and writes the result to the components.

```#TODO
The CM has an API based on **ROS services** for:
- Controller lifecycle management
  ``
  load_controller
  unload_controller
  switch_controller
  ``
- Queries
  ``
  list_controllers
  list_controller_types
  ``
- Other
  ``
  reload_controller_libraries
  ``
```

## Resource Manager
The Resource Manager ([RM](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/src/resource_manager.cpp)) **abstracts physical hardware** and its drivers (called hardware components) for the ros2_control framework. The RM loads the components using [`pluginlib`](../Standard_Libraries/pluginlib.md)-library, manages their lifecycle and components’ state and command interfaces. This abstraction provided by RM enables re-usability of implemented hardware components, and flexible hardware application for state and command interfaces.

In the control loop execution, the RM’s `read()` and `write()` methods deal with communication to the hardware components.

## Controllers
The controllers in the ros2_control framework have the same functionality as defined in control theory. They compare the reference value with the measured output and, based on this error, calculate a system’s input.

The controlles are objects derived from `controller_interface` [package](https://github.com/ros-controls/ros2_control/tree/master/controller_interface) in `ros2_control` and exported as plugins using `pluginlib`-library.
The controllers’ lifecycle is based on the [LifecycleNode-Class](https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp) implementing the state machine as described in the [Node Lifecycle Design](https://design.ros2.org/articles/node_lifecycle.html) document

When executing the control-loop `update()` method is called. The method can access the latest hardware states and enable the controller to write the hardware’s command interfaces.

## Control loop
In the most basic form, the loop consists in:
1. `read` **state** from HW
2. update controllers through [CM](#controller-manager)
3. `write` **command** to HW

## User interfaces
Users interact with the ros2_control framework using [Controller Manager](#controller-manager)'s services.
While service calls can be used directly from the command line or via nodes, there exists a user-friendly Command Line Interface (CLI) which integrates with the [ros2 cli](https://github.com/ros-controls/ros2_control/tree/master/ros2controlcli). This supports auto-complete and has a range of common commands available.

# Hardware components
The [hardware components](https://github.com/ros-controls/roadmap/blob/master/design_drafts/hardware_access.md) realize communication to physical hardware and represent its abstraction in the ros2_control framework. The components have to be exported as plugins using `pluginlib`-library. The [Resource Manager](#resource-manager) dynamically loads those plugins and manages their lifecycle.

There are three basic types of components:
- **System**: Complex (multi-DOF) robotic hardware. The main difference between the Actuator component is the possibility to use complex transmissions like needed for humanoid robot’s hands. This component has reading and writing capabilities. It is used when there is only one logical communication channel to the hardware
- **Sensor**: Robotic hardware is used for sensing its environment. A sensor component is related to a joint (e.g., encoder) or a link (e.g., force-torque sensor). This component type has only reading capabilities.
- **Actuator**: Simple (1 DOF) robotic hardware like motors, valves, and similar. An actuator implementation is related to only one joint. This component type has reading and writing capabilities. Reading is not mandatory if not possible. The actuator type can also be used with a multi-DOF robot if its hardware enables modular design

## Hardware description in URDF
The ros2_control framework uses the `<ros2_control>`-tag in the robot’s URDF file to describe its components, i.e., the hardware setup. The chosen structure enables tracking together multiple xacro-macros into one without any changes.
Check [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos).

# Running the framework
To run ros2_control framework:
1. Create a YAML file with the configuration of the controller manager and a controller.
2. Extend the robot URDF description with the `<ros2_control>` tags. use XML macro (`xacro`) files instead of URDF directly.
3. Create a launch file to start the node (recommended default `ros2_control_node`) with [Controller Manager](#controller-manager)

NOTE: You could alternatively use a script to create setup a [skeleton of the “hardware_interface” package by using the scripts](https://stoglrobotics.github.io/ros_team_workspace/use-cases/setup_robot_ros2_control_hardware.html) provided by one of our maintainers.