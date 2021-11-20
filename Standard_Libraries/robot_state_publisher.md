# Robot state publisher #TODO
[`robot_state_publisher`](https://github.com/ros/robot_state_publisher/tree/ros2) uses the URDF specified by the parameter `robot_description` and the joint positions from the topic `/joint_states` to calculate the forward kinematics of the robot and publish the results via `tf`.

## Topics
Robot State Publisher **publishes** on:
- `/tf_static [tf2_msgs/msg/TFMessage]`: information about `fixed` joint data
- `/tf [tf2_msgs/msg/TFMessage]`: information about `movable` joint data, when `/joint_states` data is available.
- `/robot_description [std_msgs/msg/String]`: description of the robot URDF. Republishes the value set in the **`robot_description` parameter**, which is useful for getting informed of dynamic changes to the URDF. Uses *transient local* QoS.

And **subscribes** to:
- `/joint_states [sensor_msgs/msg/JointState]`: The joint state updates to the robot poses. The RobotStatePublisher class takes these updates, does transformations and then publishes the results on the `tf2` topics.

## Parameters
- `robot_description [string]`: The original description of the robot in URDF form. This **must** be set at startup time. Updates to it are reflected in `robot_description` topic.
- `publish_frequency [double]`: maximum frequency at which non-static transforms (e.g. joint states) will be published to /tf. Default 20Hz
- `ignore_timestamp [bool]` - Whether to accept all joint states no matter what the timestamp (`true`), or to only publish joint state updates if they are newer than the last publish_frequency (`false`). Defaults to `false`.
- `frame_prefix [string]` - An arbitrary prefix to add to the published tf2 frames. Defaults to the empty string.