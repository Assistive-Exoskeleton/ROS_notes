# `ros2_control` URDF
The ros2_control framework uses the `<ros2_control>` tag in the robot’s URDF file to describe its components, i.e., the hardware setup.

```XML
<ros2_control name="myRobot" type="hardware_component_type">

    <hardware>
        <plugin>hardware_plugin_path_class</plugin>
        <param name="param_name">value</param>
        ...
    </hardware>

    <joint name="jointName">
        <command_interface name="command_interface_name">
            <param name="param_name">value</param>
        </command_interface>
        <state_interface name="state_interface_name">
    </joint>
</ros2_control>
```

- `<hardware>`: declares the hardware components implemented as plugins. 
    - `<plugin>`: name of the plugin class that implements the component
- `param`: names are not predefined and each component may define its names. Examples:
  - hardware plugins  `start_duration`, `stop_duration` and `slowdown` {come mai questi parametri?}
  -  
- `joint`: declares the joint. It has:
  - `command_interface`: communication channel to send commands to the joint.
  - `state_interface`: communication channel to receive states from the joint. 