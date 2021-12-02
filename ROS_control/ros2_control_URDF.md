# `ros2_control` URDF

## Hardware components description
Each [hardware component](ROS2_control_concepts.md#hardware-components) is described in URDF using `<ros2_control>` tag.

```XML
<ros2_control name="myComponent" type="hardware_component_type">
    
    <hardware>
        <plugin>hardware_plugin_path_class</plugin>
        <param name="param_name">value</param>
        ...
    </hardware>

    <joint name="joint_name">
        <command_interface name="command_interface_name">
            <param ...>...</param>
        </command_interface>
        <state_interface name="state_interface_name"/>
    </joint>

    <sensor name="sensor_name">
        <state_interface name="state_interface_name"/>
    </sensor>

    <transmission name="transmission_name">
        <plugin>transmission_path_class</plugin>
        <param ...>...</param>
    </transmission>

</ros2_control>
```

- `<hardware>`: declares the hardware components implemented as plugins. 
    - `<plugin>`: name of the plugin class that implements the component
- `<param>`: parameters for the current element. 
  > param names are not predefined and each component may define its names.
- `joint`: declares a robot joint. Joint names must be compatible with the controller's configuration. It has:
  - `command_interface`: communication channel to send commands to the joint (i.e. to actuator)
  - `state_interface`: communication channel to receive states from the joint (i.e. from sensor)
- `sensor`: declares a robot sensor. Being read-only, sensors only have a `state_interface`.
- `transmission`: URDF element describing the relationship between an actuator and a joint {e.g. motoriduttore}