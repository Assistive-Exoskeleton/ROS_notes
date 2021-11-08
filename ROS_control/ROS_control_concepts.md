# ROS Control
## Contents
- [ROS Control](#ros-control)
  - [Contents](#contents)
- [Description](#description)
- [Setting up a Robot](#setting-up-a-robot)
  - [Hardware abstraction](#hardware-abstraction)
  - [Controllers](#controllers)
  - [Simulation backend](#simulation-backend)

# Description
`ros_control` is a framework for (real-time) control of robots using ROS.

`ros_control` framework comprises different repositories:
- **ros_control**: main interfaces and components of the framework
- **ros_controllers**: widely used controllers
- **control_toolbox**: widely used control theory implementations (e.g. **PID**) used by controllers
- **realtime_tools**: general toolkit for realtime support (e.g. realtime buffers and publishers)
- **control_msgs**: common messages


set of packages that include **controller interfaces**, **controller managers**, **transmission** and **hardware interfaces**.

![ros_control](./images/ros_control.png)

# Setting up a Robot
The **RobotHW** abstraction is the space that talks to the hardware, provides resources and handles resource conflicts (by default exclusive resource ownership).

The **Controllers** don't talk directly to HW, but require resources that are provided by the hardware abstraction

## Hardware abstraction
We have notions of
- **resources** (e.g. velocity-controlled wheel). Resources are nothing else than pointers to the raw data that have some semantics.
- **hardware interface**: a set of similar resources 
- **robot**: a set of interfaces
- **ros_control interfaces**: note they're not classical interfaces like topics, services or actions, but like pointers so that they're real-time safe. 

```C++
class MyRobot :
    public hardware_interface::RobotHW //inherit
    {
        public:
            MyRobot(); //constructor

            //talk to HW
            void read();
            void write();

            //if exclusive resource ownership is not good enough, you can implement a virtual function
            virtual bool checkForConflict(...) const
    };

```

The package `hardware_interface` implements all building blocks for constructing robot HW abstraction

We have:
- Read-only resources and interfaces:
  - Joint state
  - IMU
  - Force-torque sensor
- Read-write resources and interfaces:
  - Position joint
  - Velocity joint
  - Effort joint

## Controllers
HW abstraction is connected to controllers by ros_control interfaces. Controllers also have their interfaces called **controller interfaces** (that are typical custom ROS interfaces like topics, services, actions)

>Note: we can have multiple controllers access the same interface, but they will be mutually exclusive during running.

We have:
- a **plugin interface** that implements the controller **lifecycle**: it's a simple state-machine that has 2 states: **stopped** and **running**, with transitions between states:
  - **load/unload**: loads and initialize, setup ROS interfaces, checks requisites:
    - configuration (URDF robot description, ROS params of the controller)
    - hardware resource **existence**
  - **run/stop**: resource **conflict handling**.
- **computation** of the controller, which has a **real-time** and a **non real-time** components.
  - **update**: real-time safe (deterministic requirement of the implementation) computation, executed periodically
  - **callbacks**: non real-time computation executed asynchronously



## Simulation backend
Uses `ros_simulation/gazebo_ros_pkgs/gazebo_ros_control`, which is a plugin of Gazebo for ros_control.
- Default plugin: 
  - populates joint interfaces from URDF
  - Reads transmission and joint limits specs
  ```XML
  <gazebo>
    <plugin name='gazebo_ros_control' filename='libgazebo_ros_control.so'>
        <robotNamespace>/my_robot</robotNamespace>
    </plugin>
  </gazebo>
  ```
- Custom plugin

With this we can test ros_control without coding a robotHW.

