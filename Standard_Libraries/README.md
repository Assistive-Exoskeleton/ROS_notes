# ROS2 standard libraries
Main libraries found:
- `ros2cli` ([`ros2/ros2cli`](https://github.com/ros2/ros2cli)): contains the ROS2 **command line interface** tools.
- `common_interfaces` ([`ros2/common_interfaces`](https://github.com/ros2/common_interfaces)): Set of packages containing common interface files (`.msg` and `.srv`). Main sources are:
  - [`std_msgs`](https://github.com/ros2/common_interfaces/tree/master/std_msgs)
  - [`std_srvs`](https://github.com/ros2/common_interfaces/tree/master/std_srvs)
  - [msg_examples](https://github.com/ros2/example_interfaces)
  
  > These are **primitive types**, just for prototyping or to build more complex types. Final interfaces should have a **semantic meaning**.

- [x] `rcl` ([rlc.md](rcl.md)): ROS client library
- [ ] `pluginlib` ([pluginlib.md](pluginlib.md))
- [ ] `robot_state_publisher` ([`robot_state_publisher.md](robot_state_publisher.md))
- [ ] `tf` ([`tf.md`](tf.md))