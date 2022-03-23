# ROS Notes
Notes for ROS and ROS packages.

ROS theory and concepts
- [x] [ROS2 concepts](ROS2_concepts.md)
- [x] [ROS2 filesystem](ROS2_filesystem.md)

ROS programming and application
- [ ] [ROS2 standard libraries](Standard_Libraries)
  - [x] [ROS2 command-line tools](Standard_Libraries/ros2cli.md)
- [ ] [ROS2 launch files](launch.md) 
- [ ] [Robot description](Robot_description)

Packages:
- [ ] [`ros_control`](ROS_control)
- [ ] [`rViz`](rViz)
- [x] [`rQt`](rQt)
- [ ] [`gazebo`](gazebo)

[Raspberry](Raspberry):
- [ ] [Installation](Raspberry/installation.md)
- [ ] [Servo motor simple test](Raspberry/Rpi_servo.md)

# References
## docs.ros.org
- [ ] [Concepts](https://docs.ros.org/en/galactic/Concepts/)
  - [x] [About Quality of Service settings](https://docs.ros.org/en/galactic/Concepts/About-Quality-of-Service-Settings.html)
  - [x] [About ROS 2 client libraries](https://docs.ros.org/en/galactic/Concepts/About-ROS-2-Client-Libraries.html)
  - [x] [About ROS 2 interfaces](https://docs.ros.org/en/galactic/Concepts/About-ROS-Interfaces.html)
  - [ ] [About parameters in ROS 2](https://docs.ros.org/en/galactic/Concepts/About-ROS-2-Parameters.html)
  - [ ] ...
- [ ] [Tutorials](https://docs.ros.org/en/galactic/Tutorials.html)
  - [x] [Configuring your ROS 2 environment](https://docs.ros.org/en/galactic/Tutorials/Configuring-ROS2-Environment.html)
  - [x] [Introducing turtlesim and rqt](https://docs.ros.org/en/galactic/Tutorials/Turtlesim/Introducing-Turtlesim.html)
  - [x] [Understanding ROS 2 nodes](https://docs.ros.org/en/galactic/Tutorials/Understanding-ROS2-Nodes.html)
  - [x] [Understanding ROS 2 topics](https://docs.ros.org/en/galactic/Tutorials/Topics/Understanding-ROS2-Topics.html)
  - [x] [Understanding ROS 2 services](https://docs.ros.org/en/galactic/Tutorials/Services/Understanding-ROS2-Services.html)
  - [x] [Understanding ROS 2 parameters](https://docs.ros.org/en/galactic/Tutorials/Services/Understanding-ROS2-Services.html)
  - [x] [Understanding ROS 2 actions](https://docs.ros.org/en/galactic/Tutorials/Understanding-ROS2-Actions.html)
  - [x] [Using rqt_console](https://docs.ros.org/en/galactic/Tutorials/Rqt-Console/Using-Rqt-Console.html)
  - [x] [Creating a launch file](https://docs.ros.org/en/galactic/Tutorials/Launch-Files/Creating-Launch-Files.html)
  - [ ] [Recording and playing back data](https://docs.ros.org/en/galactic/Tutorials/Ros2bag/Recording-And-Playing-Back-Data.html)
  - [x] [Creating a workspace](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html)
  - [x] [Creating your first ROS 2 package](https://docs.ros.org/en/galactic/Tutorials/Creating-Your-First-ROS2-Package.html)
  - [x] [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
  - [x] [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
  - [x] [Writing a simple service and client (C++)](https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Cpp-Service-And-Client.html)
  - [x] [ Writing a simple service and client (Python)](https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Py-Service-And-Client.html)
  - [x] [Creating custom ROS 2 msg and srv files](https://docs.ros.org/en/galactic/Tutorials/Custom-ROS2-Interfaces.html)
  - [ ] [Expanding on ROS 2 interfaces](https://docs.ros.org/en/galactic/Tutorials/Single-Package-Define-And-Use-Interface.html)
  - [ ] [Using parameters in a class (C++)](https://docs.ros.org/en/galactic/Tutorials/Using-Parameters-In-A-Class-CPP.html)
  - [ ] [Using parameters in a class (Python)](https://docs.ros.org/en/galactic/Tutorials/Using-Parameters-In-A-Class-Python.html)
  - [ ] [Getting started with ros2doctor](https://docs.ros.org/en/galactic/Tutorials/Getting-Started-With-Ros2doctor.html)
  - [x] [Creating and Using Plugins (C++)](https://docs.ros.org/en/galactic/Tutorials/Pluginlib.html)
  - [x] [Creating an action](https://docs.ros.org/en/galactic/Tutorials/Actions/Creating-an-Action.html)
  - [ ] [Writing an action server and client (C++)](https://docs.ros.org/en/galactic/Tutorials/Actions/Writing-a-Cpp-Action-Server-Client.html)
  - [ ] [Writing an action server and client (Python)](https://docs.ros.org/en/galactic/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html)
  - [x] [Launching/monitoring multiple nodes with Launch](https://docs.ros.org/en/galactic/Tutorials/Launch-system.html)
  - [ ] [Using ROS2 launch for large projects](https://docs.ros.org/en/galactic/Tutorials/Launch-Files/Using-ROS2-Launch-For-Large-Projects.html)
  - [ ] [Composing multiple nodes in a single process](https://docs.ros.org/en/galactic/Tutorials/Composition.html)
  - [ ] [Using colcon to build packages](https://docs.ros.org/en/galactic/Tutorials/Colcon-Tutorial.html)
  - [ ] [Monitoring for parameter changes (C++)](https://docs.ros.org/en/galactic/Tutorials/Monitoring-For-Parameter-Changes-CPP.html)
  - [ ] [tf2 Tutorials](https://docs.ros.org/en/galactic/Tutorials/Tf2/Tf2-Main.html)
    - [ ] ...
  - [ ] [URDF Tutorials](https://docs.ros.org/en/galactic/Tutorials/URDF/URDF-Main.html)
    - [ ] [Building a Visual Robot Model with URDF from Scratch](https://docs.ros.org/en/galactic/Tutorials/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html)
    - [ ] ...
  - [ ] ...
  - [ ] [Building realtime Linux for ROS 2](https://docs.ros.org/en/galactic/Tutorials/Building-Realtime-rt_preempt-kernel-for-ROS-2.html)
  - [ ] [Use quality-of-service settings to handle lossy networks](https://docs.ros.org/en/galactic/Tutorials/Quality-of-Service.html)
  - [ ] ...
  - [ ] [Real-time programming in ROS 2](https://docs.ros.org/en/galactic/Tutorials/Real-Time-Programming.html)
  - [ ] ...
- [ ] [How-to Guides](https://docs.ros.org/en/galactic/How-To-Guides.html)
  - [ ] ...

## docs.ros2.org
- [x] [rclpy](https://docs.ros2.org/latest/api/rclpy/index.html)
- [ ] [rclcpp](https://docs.ros2.org/latest/api/rclcpp/)

## design.ros2.org
- [x] [`Interface Definition Language (IDL)`](https://design.ros2.org/articles/idl_interface_definition.html)

## wiki.ros.org
- [ ] [urdf/XML](http://wiki.ros.org/urdf/XML)
  - [x] [model](http://wiki.ros.org/urdf/XML/model)
  - [ ] ...
- [ ] [rviz/UserGuide](http://wiki.ros.org/rviz/UserGuide)
- [ ] [pluginlib](http://wiki.ros.org/pluginlib)
  - [x] [Tutorials](http://wiki.ros.org/pluginlib/Tutorials)
    - [x] [Writing and Using a Simple Plugin](http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin)
- [ ] ros_control
  - [ ] tutorials
    - [x] [Create your own hardware interface](http://wiki.ros.org/ros_control/Tutorials/Create%20your%20own%20hardware%20interface)

## index.ros.org

## sir.upc.edu/projects/rostutorials/
- [ ] ...
- [ ] [Tutorial 10: Robot Control](https://sir.upc.edu/projects/rostutorials/10-gazebo_control_tutorial/index.html)

## github.com
- [x] [`launch`](https://github.com/ros2/launch/blob/master/launch/doc/source/architecture.rst)
- [x] [`ros2cli`](https://github.com/ros2/ros2cli)
- [x] [`common_interfaces`](https://github.com/ros2/common_interfaces)
- [x] [`example_interfaces`](https://github.com/ros2/example_interfaces)
- [x] [`robot_state_publisher`](https://github.com/ros/robot_state_publisher/tree/ros2)
- [x] [`joint_state_publisher`](https://github.com/ros/joint_state_publisher/tree/ros2)
- [ ] [`ros2_control_demos`](https://github.com/ros-controls/ros2_control_demos)
- [ ] ros_controls [`roadmap`](https://github.com/ros-controls/roadmap)
  - [ ] [`design_drafts`](https://github.com/ros-controls/roadmap/tree/master/design_drafts)
    - [x] [components_architecture_and_urdf_examples.md](https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md)
    - [x] [hardware access through controllers](https://github.com/ros-controls/roadmap/blob/master/design_drafts/hardware_access.md)
- [x] [ubuntu robotics cheat_sheets for CLI](https://github.com/ubuntu-robotics/ros2_cheats_sheet/blob/master/cli/cli_cheats_sheet.pdf)

## control.ros.org
- [x] [getting started](https://control.ros.org/getting_started.html)
- [x] core functionalities
  - [x] [controller_manager](http://control.ros.org/ros2_control/controller_manager/doc/userdoc.html)
  - [x] [Hardware components](http://control.ros.org/ros2_control/hardware_interface/doc/hardware_components_userdoc.html)
  - [x] [Fake components](http://control.ros.org/ros2_control/hardware_interface/doc/fake_components_userdoc.html)  
  - [x] [ros2_control CLI](http://control.ros.org/ros2_control/ros2controlcli/doc/userdoc.html) 
## gazebosim.org
- [ ] [Gazebo ros control](http://gazebosim.org/tutorials/?tut=ros_control)

## Others
- [ ] [`ros_control` video presentation](https://vimeo.com/107507546)
- [ ] [Designing a ROS2 Robot](https://jeffzzq.medium.com/designing-a-ros2-robot-7c31a62c535a)