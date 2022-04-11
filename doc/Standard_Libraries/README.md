# ROS2 standard libraries
Main libraries found:
- `common_interfaces` ([`ros2/common_interfaces`](https://github.com/ros2/common_interfaces)): Set of packages containing common interface files (`.msg` and `.srv`). Main sources are:
  - [`std_msgs`](https://github.com/ros2/common_interfaces/tree/master/std_msgs)
  - [`std_srvs`](https://github.com/ros2/common_interfaces/tree/master/std_srvs)
  - [msg_examples](https://github.com/ros2/example_interfaces)
  
  > These are **primitive types**, just for prototyping or to build more complex types. Final interfaces should have a **semantic meaning**.

  - [`sensor_msgs`](https://github.com/ros2/common_interfaces/tree/master/sensor_msgs)
  - ...


```{toctree}
---
maxdepth: 2
---

rcl
pluginlib
robot_state_publisher
joint_state_publisher
tf
ros2cli
```