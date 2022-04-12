# URDF

## URDF Structure 
<pre>
Robot
├── link1
│   ├── visual
│   │   ├── geometry
│   │   ├── origin
│   │   └── material
│   └── [inertial]
│       ├── mass
│       └── inertia
├── link2
│   └── ...
├── joint1
│   ├── axis
│   ├── origin
│   ├── parent
│   ├── child
│   ├── limit
│   └── dynamics
├── joint2
│   └── ...
<span├──  style="color:red">ros2_control
│   ├── hardware
│   │   └── plugin
│   └── joint
│       ├── command_interface
│       ├── state_interface1
│       ├── [state_interface2]
│       └── [state_interface3]
└── gazebo
    └── plugin
        └── parameters</span>
</pre>

## Special Characters in Joint Names

Joint names should not contain any of the following special characters: -,[,],(,)

## Safety Limits

Some URDFs have safety limits set in addition to the joint limits of the robot. Here’s an example of the safety controller specified for the Panda head pan joint:

```xml
<safety_controller k_position="100" k_velocity="1.5" soft_lower_limit="-2.857" soft_upper_limit="2.857"/>
```

The “soft_lower_limit” field and the “soft_upper_limit” field specify the joint position limits for this joint. MoveIt will compare these limits to the hard limits for the joint specified in the URDF and choose the limits that are more conservative.

```{note}
If the “soft_lower_limit” and the “soft_upper_limit” in the safety_controller are set to 0.0, your joint will be unable to move. MoveIt relies on you to specify the correct robot model.
```


## Add ros2_control tag to a URDF

To use `ros2_control` with your robot, you need to add some additional elements to your URDF.
You should include the tag `<ros2_control>` to access and control the robot interfaces. We should
include

 - a specific `<plugin>` for our robot
 - `<joint>` tag including the robot controllers: commands and states.

For example, if you want to include an effort controller you have to write:
```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="To_Change">
    <command_interface name="effort">
      <param name="min">-1000</param>
      <param name="max">1000</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

## Add the gazebo_ros2_control plugin

In addition to the `ros2_control` tags, a Gazebo plugin needs to be added to your URDF that
actually parses the `ros2_control` tags and loads the appropriate hardware interfaces and
controller manager. By default the `gazebo_ros2_control` plugin is very simple, though it is also
extensible via an additional plugin architecture to allow power users to create their own custom
robot hardware interfaces between `ros2_control` and Gazebo.

```xml
<gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <robot_param>robot_description</robot_param>
      <robot_param_node>robot_state_publisher</robot_param_node>
      <parameters>$(find package_name)/config/file.yaml</parameters>
    </plugin>
</gazebo>
```

The `gazebo_ros2_control` `<plugin>` tag also has the following optional child elements:

 - `<robot_param>`: The location of the `robot_description` (URDF) on the parameter server, defaults to `robot_description`
 - `<robot_param_node>`: Name of the node where the `robot_param` is located, defaults to `robot_state_publisher`
 - `<parameters>`: YAML file with the configuration of the controllers

Use the tag `<parameters>` inside `<plugin>` to set the YAML file with the controller configuration.

This controller publishes the state of all resources registered to a
`hardware_interface::StateInterface` to a topic of type `sensor_msgs/msg/JointState`.
The following is a basic configuration of the controller.

```yaml
joint_state_controller:
  ros__parameters:
    type: joint_state_controller/JointStateController
```

This controller creates an action called `/cart_pole_controller/follow_joint_trajectory` of type `control_msgs::action::FollowJointTrajectory`.

```yaml
cart_pole_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
       - slider_to_cart
    write_op_modes:
       - slider_to_cart
```

### Default gazebo_ros2_control Behavior

By default, without a `<plugin>` tag, `gazebo_ros2_control` will attempt to get all of the information it needs to interface with a ros2_control-based controller out of the URDF. This is sufficient for most cases, and good for at least getting started.

The default behavior provides the following ros2_control interfaces:

 - hardware_interface::JointStateInterface
 - hardware_interface::EffortJointInterface
 - hardware_interface::VelocityJointInterface

### **Advanced**: custom gazebo_ros2_control Simulation Plugins

The `gazebo_ros2_control` Gazebo plugin also provides a pluginlib-based interface to implement custom interfaces between Gazebo and `ros2_control` for simulating more complex mechanisms (nonlinear springs, linkages, etc).

These plugins must inherit `gazebo_ros2_control::GazeboSystemInterface` which implements a simulated `ros2_control`
`hardware_interface::SystemInterface`. SystemInterface provides API-level access to read and command joint properties.

The respective GazeboSystemInterface sub-class is specified in a URDF model and is loaded when the
robot model is loaded. For example, the following XML will load the default plugin:
```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  ...
<ros2_control>
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    ...
  </plugin>
</gazebo>
```
