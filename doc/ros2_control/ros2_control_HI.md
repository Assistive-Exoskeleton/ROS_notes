# Hardware components
The [hardware components / resources](https://github.com/ros-controls/roadmap/blob/master/design_drafts/hardware_access.md) realize communication to physical hardware and represent its abstraction in the `ros2_control` framework. Hardware components are libraries (plugins) defined using `pluginlib`. the [RM](ros2_control_concepts.md#resource-manager-rm) dynamically loads those plugins at runtime and manages their [lifecycle](../Standard_Libraries/rcl.md#lifecycle), allowing flexibility. 


The hardware is composed and configured in [URDF](ros2_control_URDF.md#hardware-components-description).

There are 3 basic **[types](http://control.ros.org/api/namespacehardware__interface.html)** of hardware components/resources:
- **System**: Complex (multi-DOF) robotic hardware. The main difference between the Actuator component is the possibility to use complex transmissions like needed for humanoid robot’s hands. It has reading and writing capabilities. Used when there is only one logical communication channel to the hardware
- **Sensor**: Robotic hardware used for sensing its environment. A sensor component is related to a joint (e.g. encoder) or a link (e.g. force-torque sensor). This component type has only reading capabilities.
- **Actuator**: Simple (1 DOF) robotic hardware like motors, valves, and similar. An actuator implementation is related to only one joint. This component type has reading and writing capabilities. The actuator type can also be used with a multi-DOF robot if its hardware enables modular design

# Writing a Hardware interface
## Building hardware package
As with usual packages, use `ros2 pkg create` to generate an `ament_cmake` build-type package for the hardware interface. 
```powershell
$ ros2 pkg create --ament_cmake <hardware_pkg>
```

The package structure will be something like:
```
<hardware_pkg>
   CMakeLists.txt
   package.xml
   <hardware_pkg>.xml
   
   include/<hardware_pkg>/
      visibility_control.h
      <hardware_resource>.hpp
      ...
   
   src/
      <hardware_resource>.cpp
      ...
```

## `visibility_control.h`
`visibility_control.h` is a header containing various macros used for [visibility control](https://gcc.gnu.org/wiki/Visibility).

These macros are useful when dealing with **shared libraries** (e.g. ros2 plugins) that are dynamically loaded at runtime. The purposes are manifold:
- Improves load times of shared libraries
- Optimizes code which becomes faster and lighter
- Reduces chances of symbols collision between libraries
- Manages compatibility between [Windows](https://docs.microsoft.com/en-us/cpp/cpp/dllexport-dllimport?view=msvc-170) shared libraries (i.e. `DLL`, Dynamic Link Libraries) and `ELF` systems (e.g. **Linux**) shared libraries (i.e. `DSO`, Dynamic Shared Objects)

It also changes the **visibility** of certain symbols of `rclcpp` library, that the library itself cannot have, but a code in a different package in which the `rclcpp` header is imported must have in order to link.

## `<hardware_resource>.hpp`
Header guards are used to encapsulate the header file:
```c++
#ifndef <HARDWARE_PKG>__<HARDWARE_RESOURCE>_HPP_
#define <HARDWARE_PKG>__<HARDWARE_RESOURCE>_HPP_

//...

#endif //<HARDWARE_PKG>__<HARDWARE_RESOURCE>_HPP_
```
Includes:
- The custom hardware resource will extend a **base hardware interface class**:
  ```c++
  #include "hardware_interface/<type>_interface.hpp" 
  /* hardware_interface::<Type>Interface */ 
  ```
  Where:
  - `<type>` is `system`, `sensor` or `actuator`
  - `<Type>` is `System`, `Sensor` or `Actuator`
  
- Other headers for the hardware interfaces and their respective classes are:
  ```c++
  #include "hardware_interface/handle.hpp"
  /* hardware_interface::StateInterface
     hardware_interface::CommandInterface */

  #include "hardware_interface/hardware_info.hpp"
  /* hardware_interface::HardwareInfo */
  
  #include "hardware_interface/types/hardware_interface_return_values.hpp"
   /* hardware_interface::return_type */
  ```
- The hardware resource lifecycle is managed through node lifecycle methods:
  ```c++
  #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
  /* rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface */

  #include "rclcpp_lifecycle/state.hpp"  
  /* rclcpp_lifecycle::State */
  ```

The hardware resource class is encapsulated in the `namespace` of hardware package written in *snake_case* (lowercase with underscores)
```c++
//includes...
namespace hardware_pkg
{
   //...
}
```

Then, the class of the hardware component/resource is declared, by extending the base class of the chosen hardware component:
```c++
//includes, namespace...
class MyHardwareResource : public hardware_interface::SystemInterface //or Actuator, Sensor
{
   //...
} 
```

The hardware resource [lifecycle](../Standard_Libraries/rcl.md#lifecycle) is managed through `LifecycleNodeInterface` callbacks, extended by `hardware_interface::<Type>Interface`, which are overridden:
```c++
CallbackReturn on_configure(State &) override;
CallbackReturn on_cleanup(State &) override;
CallbackReturn on_shutdown(State &) override;
CallbackReturn on_activate(State &) override;
CallbackReturn on_deactivate(State &) override;
CallbackReturn on_error(State &) override;
```
Where:
- `CallbackReturn` is the return value of `LifecycleNodeInterface`:
   ```c++
   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
   //SUCCESS ; FAILURE ; ERROR
   ```
- `State` represents the previous state of the lifecycle from which the callback was called:
   ```c++
   rclcpp_lifecycle::State
   //e.g. Unconfigured, Inactive, Active, Finalized
   ```

Also `<InterfaceType>Interface` methods are overridden:
```c++
CallbackReturn on_init(HardwareInfo &) override;
std::vector<StateInterface> export_state_interfaces() override;
std::vector<CommandInterface> export_command_interfaces() override;
return_type prepare_command_mode_switch(/**/) override;
return_type perform_command_mode_switch(/**/) override;
return_type read() override;
return_type write() override;
```
Where:
- `HardwareInfo` is a struct containing parsed URDF data for `ros2_control`:
  ```c++
  hardware_interface::HardwareInfo
  //name, type, hardware_class_type, hardware_parameters, joints, sensors, transmissions...
  ```
- `StateInterface` is a read-only handle:
  ```c++
  hardware_interface::StateInterface
  ```
- `CommandInterface` is a read-write handle:
  ```c++
  hardware_interface::CommandInterface
  ```
- `return_type` is the return type of `hardware_interface`:
  ```c++
  hardware_interface::return_type
  //OK = 0, ERROR = 1
  ```

## `<hardware_resource>.cpp`
Again, include the `<hardware_resource>.hpp` header and encapsulate the class definition in the package `namespace`.

Then implement the methods:
- **`on_init`**: lifecycle callback that instantiates the resource and sets the state to `Unconfigured`. Initializes all member variables and process parameters from the `hardware_interface::HardwareInfo` argument. The parent method `on_init` is called first:
  ```c++
  CallbackReturn MyHardwareResource::on_init(HardwareInfo & info){
     hardware_interface::SystemInterface::on_init(info)
     //now process 'info_'
  }
  ```
  This generates a `info_` attribute, which is the `HardwareInfo` struct containing the parsed `ros2control` URDF.
- **`on_configure`**: lifecycle callback for loading configuration and performing setup (e.g. resetting variables, setting up hardware communication...). It if succeeds the resource is `Inactive`.
- **`on_cleanup`**: lifecycle callback used to clear all state and return to the initial functionality after creation, from `Inactive` to `Unconfigured`.
- **`export_state_interfaces`** and **`export_command_interfaces`**: methods defining interfaces that hardware offers, might be standard or custom. Custom interfaces are only available to custom controllers, but the robot can still function with standard controllers if other standard interfaces are present.
- **`on_activate`**: lifecycle callback used to make any final preparation (requiring short time) to start executing. This may include acquiring resources that are only held when the node is actually active (e.g. hardware access). If succeeds, the state becomes `Active`
- **`on_deactivate`**: lifecycle callback used to reverse changes performed in `Activating` state. If succeeds, the state becomes `Inactive`.
- **`on_shutdown`**: lifecycle callback doing cleanup before node destruction (`Finalized` state).
- **`on_error`**: used to handle errors from any lifecycle state. If it succeeds, the state reverts to `Unconfigured`, otherwise it becomes `Finalized` in preparation for destruction.
- **`read`**: reads the states from hardware storing them in internal variables defined in `export_state_interfaces`.
- **`write`**: commands hardware based on values stored in internal variables defined in `export_command_interfaces`.

Then include `pluginlib/class_list_macros.hpp` and add a `PLUGINLIB_EXPORT_CLASS` with parameters:
1. Hardware interface class `my_hardware_pkg::MyHardwareResource`
2. Base class `hardware_interface::<Type>Interface`

## `<hardware_pkg>.xml`
This files contains the definition of the library and class that have to be visible for `pluginlib`. The structure is like:
```XML
<libary path="<hardware_pkg>">
   <class name="<hardware_pkg>/<HardwareResource>"
          type="<hardware_pkg>::<HardwareResource>"
          base_class_type="hardware_interface::<Type>Interface">
      <description>hardware pkg description</description>
   </class>
   <!-- other classes in this package ... -->
</library>
```
> Note: the plugin `name` defines the hardware interface type when the RM searches for it.

## `CMakeLists.txt`
To add compile directives:
1. Dependencies, at least:
   ```CMake
   find_package(ament_cmake REQUIRED)
   find_package(hardware_interface REQUIRED)
   find_package(pluginlib REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(rclcpp_lifecycle REQUIRED)
   # other dependencies ...
   ```
2. Compile directive for a shared library:
   ```CMake
   add_library(
      ${PROJECT_NAME}
      SHARED
      src/<hardware_resource>.cpp
      # other source files to be compiled ...
   )
   ```
3. Include directories for the library:
   ```CMake
   target_include_directories(
      ${PROJECT_NAME}
      PRIVATE
      include
      # other include directories
   )
   ```
4. Ament dependencies needed by the library, at least:
   ```CMake
   ament_target_dependencies(
      ${PROJECT_NAME}
      hardware_interface
      pluginlib
      rclcpp
      rclcpp_lifecycle
   )
   ```
5. Export for pluginlib description file:
   ```CMake
   pluginlib_export_plugin_description_file(hardware_interface <hardware_pkg>.xml)
   ```
6. Install directives for targets and include directories:
   ```CMake
   install(
      TARGETS ${PROJECT_NAME}
      DESTINATION lib
   )
   install(
      DIRECTORY include/
      DESTINATION include
   )
   ```
7. (optional) export directives
   ```CMake
   ament_export_include_directories(
      include
   )
   ament_export_libraries(
      ${PROJECT_NAME}
   )
   ament_export_dependencies(
      hardware_interface
      pluginlib
      rclcpp
      rclcpp_lifecycle
   )
   ```
8. ```CMake
   ament_package()
   ```

## `package.xml`
Add dependencies:
```XML
<depend>hardware_interface</depend>
<depend>pluginlib</depend>
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
```

# Testing #TODO
**test to check if controller can be found and loaded**
1. Create a `test/` folder and add a file `test_load_<HI_name>.cpp`
2. you can copy this [`load_generic_system_2dof`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/test/fake_components/test_generic_system.cpp#L441-L446).
3. Change the name of copied test and in the last line where HI type is specified put the name defined in  `HI_package.xml`, e.g. `HI_package/HI_name`

**CMake**
1. In the test section add the following dependencies: `ament_cmake_gmock`, `hardware_interface`.
```CMake
#???
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()
```
2. Add compile definitions for the tests using the `ament_add_gmock` directive. For details, [see how it is done for fake hardware in the ros2_control package](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/CMakeLists.txt).

**Package.xml**
1.  Add at least into the `<depend>` tag: `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`.
2.  Add at least into the `<test_depend>` tag: `ament_cmake_gmock` and `hardware_interface`. #FIXME mica non poteva esserci sia in depend che in test_depend?

**Compile and test**:
1.  compile using `colcon build`
2.  source the `setup.bash` in the `install` folder
3.  test with `colcon test <HI_package>` to check if the controller can be found through `pluginlib` and loaded by the CM.


## Fake components
Fake components are trivial simulations of hardware components, with an ideal behavior of mirroring commands to states. This fake HI can be added instead of the true HI for offline testing (i.e. launch, controllers, broadcaster, integrations with MoveIt) without hardware. [More info here](http://control.ros.org/ros2_control/hardware_interface/doc/fake_components_userdoc.html).