# Joint State Publisher #TODO [link](https://github.com/ros/joint_state_publisher/tree/ros2/joint_state_publisher)
Given a URDF (either passed on the command-line or via the `/robot_description` topic), the `joint_state_publisher` node
will continually publish `sensor_msgs/msg/JointState` messages for all of the movable joints in the URDF to the `/joint_states` topic. In combination with `robot_state_publisher`, this ensures that there is a valid transform for all joints, even when the joint doesn't have encoder data.

## Published Topics
- `/joint_states [sensor_msgs/msg/JointState]` - The state of all of the movable joints in the system.

## Subscribed Topics
- (optional) `/robot_description [std_msgs/msg/String]` - If no URDF is given on the command-line, then this node will listen on the `/robot_description` topic for the URDF to be published.  Once it has been received at least once, this node will start to publish joint values to `/joint_states`.
- (optional) `/any_topic [sensor_msgs/msg/JointState]` - If the `sources_list` parameter is not empty, then every named topic in this parameter will be subscribed to for joint state updates.  Do *not* add the default `/joint_states` topic to this list, as it will end up in an endless loop!

## Parameters
- `rate` (int) - The rate at which to publish updates to the `/joint_states` topic.  Defaults to 10.
- `publish_default_positions` (bool) - Whether to publish a default position for each movable joint to the `/joint_states` topic.  Defaults to True.
- `publish_default_velocities` (bool) - Whether to publish a default velocity for each movable joint to the `/joint_states` topic.  Defaults to False.
- `publish_default_efforts` (bool) - Whether to publish a default effort for each movable joint to the `/joint_states` topic.  Defaults to False.
- `use_mimic_tags` (bool) - Whether to honor `<mimic>` tags in the URDF.  Defaults to True.
- `use_smallest_joint_limits` (bool) - Whether to honor `<safety_controller>` tags in the URDF.  Defaults to True.
- `source_list` (array of strings) - Each string in this array represents a topic name.  For each string, create a subscription to the named topic of type `sensor_msgs/msg/JointStates`.  Publication to that topic will update the joints named in the message.  Defaults to an empty array.
- `delta` (double) - How much to automatically move joints during each iteration.  Defaults to 0.0.

# `joint_state_publisher_gui`
GUI (using Qt libraries) to change data about the position of joints.