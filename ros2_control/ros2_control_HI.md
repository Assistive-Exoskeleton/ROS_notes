# Writing an Hardware interface
In ros2, hardware system components are libraries (plugins) dynamically loaded by the CM using `pluginlib`.

1. **Building HI package**. As with usual packages, use `ros2 pkg create` to generate an `ament_cmake` build-type package. See `--help` of command to get more info: there's an option to create library source files compile rules.
2. **Preparing src files**: the structure of the package should be:
   ```c
   <package_name>
        CMakeLists.txt
        package.xml
        visibility_control.h         //optional export rules for Windows
        include/<package_name>/
                <robot_HI_name>.hpp
        src/
                <robot_HI_name>.cpp
    
   ```
   > Note: `visibility_control.h` file can be copied from an existing controller package, changing the prefix to `<package_name>`. 

3. **Adding declarations in `.hpp` file**:
   1. Use header guards (`#ifndef`, `#define` preprocessor directives)
   2. Include `"hardware_interface/<interface_type>_interface.hpp`, where `<interface_type>` is `Actuator`, `Sensor` or `System`. Include also `visibility_control.h` if using it.
   3. Define a unique `namespace`, usually the package name written in `snake_case` (lowercase with underscores)
   4. Define the class of the HI by **extending** `<interface_type>Interface` class:
      ```c++
      //example
      class HI_name : public hardware_interface::SystemInterface
      ```
   5. Add a constructor without parameters of the following methods implementing `LifecycleNodeInterface`: `on_configure`, `on_cleanup`, `on_shutdown`, `on_activate`, `on_deactivate`, `on_error`. <br>Override `<interface_type>Interface` definition: `on_init`, `export_state_interfaces`, `export_command_interfaces`, `read`, `write`.
      > More info: [doxygen](http://control.ros.org/api/namespacehardware__interface.html).
4. **Adding definitions in `.cpp` file**:
   1. Include the header file and add a namespace definition
   2. Implement `on_init` method. Here, initialize all member variables and process parameters from the `info` argument.<br> 
   First the parents `on_init` is often called to process standard values (e.g. name): `hardware_interface::SystemInterface::on_init(info)`.<br>
   Return `CallbackReturn::SUCCESS` or `CallbackReturn::ERROR` depending if all worked fine or not.
   3. Write the `on_configure`/`on_cleanup` methods where usually **communication** with hardware is set-up so that hardware can be activated/deactivated.
   4. Implement `export_state_interfaces` and `export_command_interfaces` methods defining interfaces that hardware offers. Some might be already existing and standard, others might be custom. Custom interfaces are only available to custom controllers, but the robot can still function with standard controllers if other standard interfaces are present. 
   5. Implement `on_activate`/`on_deactivate` method, where hardware "power" is enabled/disabled.
   6. Implement `on_shutdown` method.
   7. Implement `on_error` method to handle errors from all states
   8. Implement `read` getting the states from hardware and storing them in internal variables defined in `export_state_interfaces`
   9. Implement `write` commanding the hardware based on values stored in internal variables defined in `export_command_interfaces`
   10. Include `pluginlib/class_list_macros.hpp` header. Add a `PLUGINLIB_EXPORT_CLASS` macro at end of file, with two parameters:
       1.  hardware interface class `HI_package::HI_name`
       2.  base class `hardware::interface::SystemInterface`

 1. **Write export definition for pluginlib**: Create `HI_package.xml` file in the package and add a definition of the library and HI class that have to be visible for `pluginlib`. It's similar to this [fake component](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/fake_components_plugin_description.xml) definition. Usually the plugin name is `HI_package/HI_name`. This defines the HI type when the RM searches for it. 
 2. **Write a test to check if controller can be found and loaded**
    1. Create a `test/` folder and add a file `test_load_<HI_name>.cpp`
    2. you can copy this [`load_generic_system_2dof`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/test/fake_components/test_generic_system.cpp#L441-L446).
    3. Change the name of copied test and in the last line where HI type is specified put the name defined in `HI_package.xml`, e.g. `HI_package/HI_name`
 3. **Add compile directives to `CMakeLists.txt`**:
    1. Under `find_package(ament_cmake REQUIRED)` add further dependencies. Those are at least: `hardware_interface`, `pluginlib`, `rclcpp` and `rclcpp_lifecycle`.
    2. Add a compile directive for a shared library providing the `HI_name.cpp` file as the source.
    3. Add include directories for the library (usually only `include`).
    4. Add ament dependencies needed by the library. You should add at least those listed under 1.
    5. Export for pluginlib description file using the following command:
       ```cmake
       pluginlib_export_plugin_description_file(hardware_interface <HI_package>.xml)
       ```
    6. Add install directives for targets and include directory.
    7. In the test section add the following dependencies: `ament_cmake_gmock`, `hardware_interface`.
    8. Add compile definitions for the tests using the `ament_add_gmock` directive. For details, [see how it is done for fake hardware in the ros2_control package](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/CMakeLists.txt).
    9. (optional) Add your hardware interface's library into `ament_export_libraries` before `ament_package()`.
 4.  **Add dependencies to `package.xml`**:
     1.  Add at least into the `<depend>` tag: `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`.
     2.  Add at least into the `<test_depend>` tag: `ament_cmake_gmock` and `hardware_interface`. #FIXME mica non poteva esserci sia in depend che in test_depend?
 5.  **Compile and test**:
     1.  compile using `colcon build`
     2.  source the `setup.bash` in the `install` folder
     3.  test with `colcon test <HI_package>` to check if the controller can be found through `pluginlib` and loaded by the CM.


## Fake components
Fake components are trivial simulations of hardware components, with an ideal behavior of mirroring commands to states. This fake HI can be added instead of the true HI for offline testing (i.e. launch, controllers, broadcaster, integrations with MoveIt) without hardware. [More info here](http://control.ros.org/ros2_control/hardware_interface/doc/fake_components_userdoc.html).