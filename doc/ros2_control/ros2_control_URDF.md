# `ros2_control` URDF

## Hardware components description
```{note}
Use XML macro (`XACRO`) files instead of URDF directly.
```
1. Each [hardware component](ros2_control_HI.md#hardware-interface) is described in URDF using `<ros2_control>` tag.
    ```XML
    <ros2_control name="my_component" type="hardware_component_type">

    </ros2_control>
    <!-- other components ... -->
    ```
    |Attribute|Description|
    |-|-|
    |`name`|name of the hardware component
    |`type`|hardware component type (i.e. `system`, `sensor`, `actuator`)
2. Next, the hardware component is declared with the `<hardware>` tag:
    ```XML
    ...
        <hardware>
            <plugin>hw_plugin_path_class</plugin>
            <param name="my_param">value</param>
        </hardware>
    ...
    ```
    |Tag|Description|
    |-|-|
    |`<plugin>`|name of the plugin class that implements the component/hardware interface HI (e.g. `HI_pkg_name/HI_name`)
    |`<param>`|parameters for the current element (not predefined, each element can have its own)

3. A robot **joint** is a logical component, an abstraction between controller instance and the underlying hardware. A signle joint may be controlled by multiple motors with a non-trivial transmission interface, yet a controller only cares about joint values. A joint can be declared with the `<joint>` tag:
    ```XML
    ...
        <joint name="my_joint">
                <command_interface name="interface_type">
                    <param>...</param>
                </command_interface>
                <state_interface name="interface_type">
        </joint>
    ...
    ```
    |Tag|Description|
    |-|-|
    |`<command_interface>`|communication channel to send commands to the joint (i.e. to actuator)
    |`<state_interface>`|communication channel to receive states from the joint (i.e. from sensor)

    > **Interface types**: `"position"`, `"velocity"` and `"effort"` interface types are defined in `hardware_interface/types/hardware_interface_type_values.hpp`. However any interface type can be created defining a custom string (e.g. `"position_in_degrees"`)

    ```{note}
    joint names must be compatible with the controller's configuration
    ```

- A `<sensor>` tag can be used to declare a read-only component (similar to joint but can have only `<state_interface>`)
- A `<transmission>` tag can be used to define the URDF element describing actuator-joint relationship (e.g. gearboxes {=motoriduttori})
    ```XML
    ...
        <transmission name="my_transmission">
            <plugin>transmission_path_class</plugin>
            <param>...</param>    
        </transmission>
    ...
    ```