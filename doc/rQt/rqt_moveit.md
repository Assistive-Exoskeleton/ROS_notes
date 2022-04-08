# RQt moveit

An rqt-based tool that assists monitoring tasks for MoveIt! motion planner developers and users. Currently the following items are monitored if they are either running, existing or published:

* Node: `/move_group`
* Parameter: `[/robot_description, /robot_description_semantic]`
* Topic: Following types are monitored. Published "names" are ignored:\
[sensor_msgs/PointCloud, sensor_msgs/PointCloud2, sensor_msgs/Image, sensor_msgs/CameraInfo]
