# RQt (formerly RQt2)
rqt is a Qt-based framework for GUI development for ROS. It consists of three parts/metapackages:

* rqt - core infrastucture modules.
* **rqt_common_plugins** - Backend tools for building tools.
* **rqt_robot_plugins*** - Tools for interacting with robots during runtime. They need to be installed separately.
  * [rqt_moveit](rqt_moveit.md)
  * [rqt_robot_dashboard](rqt_robot_dashboard.md)
  * [rqt_robot_monitor](rqt_robot_monitor.md)
  * [rqt_robot_steering](rqt_robot_steering.md)
  * [rqt_runtime_monitor](rqt_runtime_monitor.md)
  * [rqt_tf_tree](rqt_tf_tree.md)
  

\*<font size="1">Only ROS2 compatible plugins are listed.</font>

## Advantages
RQt implements various tools and interfaces in the form of plugins and makes it easier to manage all of them in a single screen layout. 

Users can create their own plugins for RQt with either Python or C++. Over 20 plugins were created in ROS 1 and these plugins are currently being ported to ROS 2 (work in progress as of march 2022).

Compared to building your own GUIs from scratch:

* Standardized common procedures for GUI (start-shutdown hook, restore previous states).
* Multiple widgets can be docked in a single window.
* Easily turn your existing Qt widgets into RQt plugins. (see [this repo](https://github.com/Assistive-Exoskeleton/Templates_ROS2) for easily create a basic RQt plugin compatible with ROS2)