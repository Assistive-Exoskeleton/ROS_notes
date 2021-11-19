# Robot state publisher #TODO
[`robot_state_publisher`](https://github.com/ros/robot_state_publisher/tree/ros2)  contains the Robot State Publisher, a `Node` and a class to publish the state of a robot to `tf2`.

At startup time, Robot State Publisher is supplied with a kinematic tree model (URDF) of the robot. It then subscribes to the joint_states topic (of type `sensor_msgs/msg/JointState`) to get individual joint states. These joint states are used to update the kinematic tree model, and the resulting 3D poses are then published to tf2.