# ROS2_control
- [ROS2_control](#ros2_control)
- [Architecture](#architecture)
  - [Controller manager (CM)](#controller-manager-cm)
  - [Resource Manager (RM)](#resource-manager-rm)
  - [Hardware components](#hardware-components)
  - [Controllers](#controllers)
    - [Controller interface](#controller-interface)
  - [Control loop](#control-loop)
# Architecture
The primary motivation of ros_control is the lack of realtime-safe communication layer in ROS. The following figure shows the architecture:

![ros_control_architecture](./images/ros2_control_architecture.png)

## Controller manager ([CM](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/controller_manager.cpp))
The `controller_manager` (CM) has three main purposes:
1. CM connects the **controllers** and **hardware components** abstraction (through [Resource Manager](#resource-manager)).
2. CM **manages controllers**': their **lifecycle** (i.e. loading, activating, deactivating, unloading) and their **update**.
3. CM also offers ROS **services** to the ROS world and the **user**.

Parameter|description
-|-
`robot_description` [string]|String with the URDF robot description. This is usually the result of parsed files by `xacro`
`update_rate` [double]|Frequency of CM **real-time** `update()` control-loop. This loop reads data from hardware, updates all active controllers and writes commands to hardware.
[other optional parameters](http://control.ros.org/ros2_control/controller_manager/doc/userdoc.html#parameters)

There are 2 helper scripts to interact with CM at launch:
- **`spawner`**: loads, configures and start a controller on startup
  ```powershell
  $ ros2 run controller_manager spawner [-options] <controller_name>
  # -options:
  #   -p <param_file>: parameter file to be loaded into controller node before configure
  #   -t <controller_type>: type of controller. If not provided it should exist in CM namespace.
  #   --stopped: load controller without starting it
  #   ... 
  ```
- **`unspawner`**: stops and unloads a controller
  ```powershell
  $ ros2 run controller_manager unspawner [-options] <controller_name>
  ```


## Resource Manager ([RM](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/src/resource_manager.cpp))
The `resource_manager` **abstracts physical hardware** and its drivers ([**hardware components**](#hardware-components)) for the `ros2_control` framework. 

RM has different purposes:
- parses the [`ros2_control` URDF](ros2_control_URDF.md) and loads/instantiate the respective logic components (i.e. `joint` and `sensor`) and HW components (`system`, `sensor` and `actuator`) using [`pluginlib`](../Standard_Libraries/pluginlib.md)-library
- manages component's lifecycle
- manages `state` and `command` **interfaces**. 

RM abstracts hardware resources from their logical components, such that a controller does not have to know which hardware commands which joint. It also keeps a ledger of controllers and the claimed resources, in order to distribute them. This abstraction enables re-usability of implemented hardware components, and flexible hardware application for `state` and `command` interfaces.

RM internally maintains a mapping of each individual hardware resource and their interfaces. This mapping can be indexed through a simple `_logical_component_/_interface_name_` lookup (e.g. `joint_name/position`)

In the control loop execution, the RM’s `read()` and `write()` methods deal with communication to the hardware components.

## Hardware components
The [hardware components / resources](https://github.com/ros-controls/roadmap/blob/master/design_drafts/hardware_access.md) realize communication to physical hardware and represent its abstraction in the `ros2_control` framework. The components have to be exported as **plugins** using `pluginlib`-library. The [RM](#resource-manager-rm) dynamically loads those plugins at runtime and manages their lifecycle, allowing flexibility. The hardware is composed and configured solely in [URDF](ros2_control_URDF.md/#hardware-components-description).

There are three basic types of hardware components/resources:
- **System**: Complex (multi-DOF) robotic hardware. The main difference between the Actuator component is the possibility to use complex transmissions like needed for humanoid robot’s hands. This component has reading and writing capabilities. It is used when there is only one logical communication channel to the hardware
- **Sensor**: Robotic hardware is used for sensing its environment. A sensor component is related to a joint (e.g., encoder) or a link (e.g., force-torque sensor). This component type has only reading capabilities.
- **Actuator**: Simple (1 DOF) robotic hardware like motors, valves, and similar. An actuator implementation is related to only one joint. This component type has reading and writing capabilities. Reading is not mandatory if not possible. The actuator type can also be used with a multi-DOF robot if its hardware enables modular design

## Controllers
The controllers in the `ros2_control` framework have the same functionality as defined in control theory. They compare the reference value with the measured output and, based on this error, calculate a system’s input.

The controlles are objects derived from `controller_interface` [package](https://github.com/ros-controls/ros2_control/tree/master/controller_interface) in `ros2_control` and exported as plugins using `pluginlib`-library.
The controllers’ lifecycle is based on the [LifecycleNode-Class](https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp) implementing the state machine as described in the [Node Lifecycle Design](https://design.ros2.org/articles/node_lifecycle.html) document

When executing the control-loop `update()` method is called. The method can access the latest hardware states and enable the controller to write the hardware’s command interfaces.

### Controller interface
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

## Control loop
In the most basic form, the loop consists in:
1. **read state** from HW through [RM](#resource-manager-rm) `read()` method
2. update controllers through [CM](#controller-manager-cm) `update()` method
3. **write command** to HW through [RM](#resource-manager-rm) `write()` method