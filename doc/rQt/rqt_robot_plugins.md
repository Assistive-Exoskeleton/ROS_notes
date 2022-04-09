# rqt_moveit
An rqt-based tool that assists monitoring tasks for MoveIt! motion planner developers and users. Currently the following items are monitored if they are either running, existing or published:

* Node: `/move_group`
* Parameter: `[/robot_description, /robot_description_semantic]`
* Topic: Following types are monitored. Published "names" are ignored:\
[sensor_msgs/PointCloud, sensor_msgs/PointCloud2, sensor_msgs/Image, sensor_msgs/CameraInfo]

# rqt_robot_dashboard
rqt_robot_dashboard provides an infrastructure for building robot dashboard plugins in rqt.

# rqt_robot_monitor
rqt_robot_monitor displays diagnostics_agg topics messages that are published by diagnostic_aggregator. rqt_robot_monitor is a direct port to rqt of robot_monitor. All diagnostics are fall into one of three tree panes depending on the status of diagnostics (normal, warning, error/stale). Status are shown in trees to represent their hierarchy. Worse status dominates the higher level status.

    Ex. 'Computer' category has 3 sub devices. 2 are green but 1 is error. Then 'Computer' becomes error. 

You can look at the detail of each status by double-clicking the tree nodes.
Currently re-usable API to other pkgs are not explicitly provided.

# rqt_robot_steering
rqt_robot_steering provides a GUI plugin for steering a robot using Twist messages.

## Usage Example (with care-o-bot)

As noted, rqt_robot_steering publishes geometry_msgs/Twist as a name of /cmd_vel by default. This name may vary per robot/application; e.g. [Care-o-bot](http://wiki.ros.org/Robots/Care-O-bot) also publishes geometry_msgs/Twist but as base_controller/command (see [cob_teleop#Published_Topics](http://wiki.ros.org/cob_teleop#Published_Topics). So, type that name into the topic field at the top of rqt_robot_steering GUI and it gets applied immediately.

To verify, use also [rqt_topic](http://wiki.ros.org/rqt_topic) (or run from commandline rostopic echo) to see if the topics base_controller/command are actually published. 

# rqt_runtime_monitor
rqt_runtime_monitor provides a GUI plugin viewing DiagnosticsArray messages.

# rqt_tf_tree
rqt_tf_tree provides a GUI plugin for visualizing the ROS TF frame tree.