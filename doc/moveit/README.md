# MoveIt 
MoveIt 2 is the robotic manipulation platform for ROS 2 led by [PickNik Robotics](https://picknik.ai/).

For further informations:
* [concepts](concepts.md)
* [MSA - moveit setup assistant](MSA.md)
* [URDF/SRDF](SRDF.md)
* [Perception Pipeline](perception.md)
* MoveIt plugins:
  * [Motion Planning](moveit_motion_planning.md)
  * [Kinematics](moveit_kinematics.md)

<h2>Terminology</h2>


What is the difference between MoveIt 1 and MoveIt 2?

* MoveIt 2 is currently (April 2020) a straight port of MoveIt 1 but for ROS 2.
* ROS 2 is a total rewrite of the popular robotics middleware that addresses many unfulfilled needs of industry / commercial users in ROS 1.
* MoveIt 2 will hopefully fork from MoveIt 1 soon to take advantage of the new features in ROS 2. See the [roadmap](https://moveit.ros.org/documentation/contributing/roadmap/) or [status update](https://moveit.ros.org/ros2/moveit/2021/06/08/moveit-vs-moveit2.html).

What is the difference between MoveIt and ROS?

* MoveIt runs on top of ROS (Robot Operating System).
* ROS is an open-source meta-operating system for robots that provides low-level functionality like a build system, message passing, device drivers and some integrated capabilities like navigation.
* MoveIt is a primary source of the functionality for manipulation (and mobile manipulation) in ROS.
* MoveIt builds on the ROS messaging and build systems and utilizes some of the common tools in ROS like the ROS Visualizer (Rviz) and the ROS robot format (URDF).
* MoveIt is a common entry point into ROS, especially through the use of the MoveIt Setup Assistant for configuring new robots.





```{toctree}
---
maxdepth: 2
hidden:
---

concepts
MSA
SRDF
perception
moveit_motion_planning
moveit_kinematics
```



