# 1. ros2 language
Main libraries and how to code
- [1. ros2 language](#1-ros2-language)
- [2. Standard libraries](#2-standard-libraries)
  - [2.1. `ros2cli` (command line interface)](#21-ros2cli-command-line-interface)
  - [2.2. `rcl` (ROS client library)](#22-rcl-ros-client-library)
  - [2.3. `common_interfaces`](#23-common_interfaces)
    - [2.3.1. `std_msgs`](#231-std_msgs)
    - [2.3.2. `std_srvs`](#232-std_srvs)
  - [2.4. `robot_state_publisher`](#24-robot_state_publisher)
  - [2.5. `tf2`](#25-tf2)
- [3. Launch](#3-launch)
  - [3.1. Architecture of `launch`](#31-architecture-of-launch)
  - [3.2. Architecture of `launch_ros`](#32-architecture-of-launch_ros)
  - [3.3. Launch files](#33-launch-files)
  - [3.4. Launch for large projects](#34-launch-for-large-projects)

# 2. Standard libraries
## 2.1. `ros2cli` (command line interface)
[`ros2cli`](https://github.com/ros2/ros2cli) contains the ROS2 command line interface tools. 
## 2.2. `rcl` (ROS client library)
`rcl` provides the standard APIs for interacting with ROS2 (*node*, *topic*, *service*, *action*, *parameter*, *logging*, *context*, ...). The language-specific libraries are built on the [C-library `rcl`](https://github.com/ros2/rcl).

||C++|Python|
|-|---|------|
|source repository|[`rclcpp`](https://github.com/ros2/rclcpp)|[`rclpy`](https://github.com/ros2/rclpy)
|documentation|[rclcpp_doc](https://docs.ros2.org/latest/api/rclcpp/)|[rclpy_doc](https://docs.ros2.org/latest/api/rclpy/index.html)

import with:
- C++
    ```cpp
    #include "rclcpp/rclcpp.hpp"
    ```
- Python
    ```python
    import rclpy
    ```

A typical ROS program consists of the following operations:
1. **Initialization**: done by calling `init` for a particular **`Context`**. Must be done before any ROS node is generated
2. **Nodes generation**: either using `create_node` or by instantiating a `Node`. 
3. **Process node callbacks**: to process work waiting to be executed, the `spin` functions are used.
4. **Shutdown**: when finished with a previously initialized **context**, the `shutdown` function is called, invalidating all entities derived from the context.

Main APIs (python version):
- **`init`**: initialize ROS communications for a given context
- **`shutdown`**: shutdown a previously initialized context
- **`spin`**: execute work and block until the context associated with the executor is shutdown. Callbacks are executed by the provided `executor`. 
  |parameter|type|description|
  |---------|----|-----------|
  |`node`|`Node`|node to add to the executor to check for work
  |`executor`|`Executor`|executor to use (defaults to global executor if `None`)
- **`Node`**:
  - `__init__`: initialize `Node`
    |parameter|type|description|
    |---------|----|-----------|
    |`node_name`|`str`|name of the node
    |`context`|`Context`|optional context
    |`cli_args`|`List(str)`|optional list of command-line arguments
    |`namespace`|`str`|optional namespace to which relative topic/service names will be prefixed
  - `create_publisher->Publisher`: create a new publisher
    parameter|type|description|
    |--------|----|-----------|
    |`msg_type`|`.msg` imported object|type of ros msg it publishes
    |`topic`|`str`|topic to which it publishes.
    |`qos_profile`|`Union(QosProfile,int)`|[`QoS_profile`](https://docs.ros2.org/latest/api/rclpy/api/qos.html#rclpy.qos.QoSProfile) or history depth/queue size (`int`) to apply to a publisher.
  - `create_subscription->Subscription`: create a subscription:
    |parameter|type|description|
    |--------|----|-----------|
    |`msg_type`|`.msg` imported object|type of ros msg it subscribes to
    |`topic`|`str`|topic to which it subscribes 
    |`qos_profile`|`Union(QosProfile,int)`|[`QoS_profile`](https://docs.ros2.org/latest/api/rclpy/api/qos.html#rclpy.qos.QoSProfile) or history depth/queue size (`int`) to apply to the subscription.
    |`callback`|`Callable`|user-defined callback, called when a message is received by the subscription
    |`raw`|`bool`| If True, then received messages will be stored in raw binary representation.
  - `create_service->Service`: create a new service server:
    |parameter|type|description|
    |---------|----|-----------|
    |`srv_type`|`.srv` imported object|type of service interface
    |`srv_name`|`str`|name of the service
    |`callback`|`Callable`|user-defined callback, called when a service request is received by the server
    |`qos_profile`|`QoSProfile`|Quality of service profile to be applied to the service's server
  - `create_client->Client`: create a new service client
    |parameter|type|description|
    |---------|----|-----------|
    |`srv_type`|`.srv` imported object|type of service interface
    |`srv_name`|`str`|name of the service
    |`qos_profile`|`QoSProfile`|Quality of service profile to be applied to the service's client

  - `create_timer->Timer`: create a new timer, which calls a callback function every period
    |parameter|type|description|
    |---------|----|-----------|
    |`timer_period_sec`|`float`|Period (s) of timer
    |`callback`|`Callable`|user-defined callback, called when the timer expires.
  - `get_logger()->RcutilsLogger`: gets the node logger
  - `destroy_node()`: explicitly destroys the node and frees resources (subscribers, publishers...). Note that this is not always required as resources may be freed by Python's garbage collector.
    > To destroy individual resources use `destroy_<resource> (<resource>)` (e.g. `destroy_publisher(publisher)`) 
- **`Publisher`**:
  - `get_subscription_count()->int`: get amount of subscribers that the publisher has
  - `publish`: publishes a message
    |parameter|type|description|
    |---------|----|-----------|
    |`msg`|`Union(MsgType,bytes)`|ROS message to publish
  - `destroy()`
  - `@property topic_name->str`

- **`Subscription`**:
  - `destroy()`
  - `@property topic_name->str`
- **`Service`**: ROS service server
  - `send_response`: send a service response
    |parameter|type|description|
    |---------|----|-----------|
    |`response`|`SrvTypeResponse`|service response|
    |`header`||capsule pointing to the service header from the original request
  - `destroy()`
- **`Client`**: ROS service client
  - `call->SrvTypeResponse`: Make a service request and await result
    |parameter|type|description|
    |---------|----|-----------|
    |`request`|`SrvTypeRequest`|service request
  - `service_is_ready()->bool`: check if there's a service server ready
  - `wait_for_service->bool`: wait until service is ready (`True` if it is)
    |parameter|type|description|
    |---------|----|-----------|
    |`timeout_sec`|`float`|timeout, returning `False` if it expires|
  - `destroy()`
- **`Context`**: A context encapsulates the lifecycle of init and shutdown.
- **`RcutilsLogger`**:
  - `log`: log a message with a specified severity
    |parameter|type|description|
    |---------|----|-----------|
    |`message`|`str`|message to log|
    |`severity`|`LoggingSeverity`|severity of the message (`DEBUG`, `INFO`, `WARN`, `ERROR`, `FATAL` ...)
    > There are also individual functions depending on severity: `RcutilsLogger.debug(message)`, `RcutilsLogger.info(message)` ... They simply call the `log` function with the `severity` argument predefined.
  


## 2.3. `common_interfaces`
[`common_interfaces`](https://github.com/ros2/common_interfaces) is a set of packages containing common interface files (.msg and .srv).
### 2.3.1. `std_msgs`
[`std_msgs`](https://github.com/ros2/common_interfaces/tree/master/std_msgs) provides many basic msg types. Note however that primitive types don't convey semantic meaning to their contents: they simply have a "*data*" field. They should not be used for long-term usage, but just for prototyping or for embedding them in another message. 
> Note: it's **deprecated** and it's advised to use [msg_examples](https://github.com/ros2/example_interfaces) equivalent instead

### 2.3.2. `std_srvs`
[`std_srvs`](https://github.com/ros2/common_interfaces/tree/master/std_srvs) provides several service definitions for standard but simple ROS services.

## 2.4. `robot_state_publisher`
[`robot_state_publisher`](https://github.com/ros/robot_state_publisher/tree/ros2) contains the Robot State Publisher, a node and a class to publish the state of a robot to `tf2`.

At startup time, Robot State Publisher is supplied with a kinematic tree model (URDF) of the robot. It then subscribes to the joint_states topic (of type `sensor_msgs/msg/JointState`) to get individual joint states. These joint states are used to update the kinematic tree model, and the resulting 3D poses are then published to tf2.

## 2.5. `tf2`
[`tf2`](https://github.com/ros/geometry2) is the second generation of the transform library, which lets the user keep track of multiple coordinate frames over time.

# 3. Launch
The [launch system](https://docs.ros.org/en/galactic/Tutorials/Launch-system.html) in ROS 2 is responsible for helping the user describe the configuration of their system (programs to run, where to run them, arguments, ...) and then execute it as described. It is also responsible for monitoring the state of the processes launched, and reporting and/or reacting to changes in the state of those processes.

Launch files written in Python can start and stop different nodes as well as trigger and act on various events. The package providing this framework is [`launch_ros`](https://github.com/ros2/launch_ros), which uses the non-ROS-specific [`launch`](https://github.com/ros2/launch) framework underneath.

## 3.1. Architecture of `launch`
The main classes are:
- **`launch.LaunchDescription`**: it encapsulates the intent of the user as a list of discrete **`launch.Action`**'s.
- **`launch.Action`**: actions can have direct effect (run a process or set a configuration variable) or yield additional actions. 

  Actions may also have arguments affecting their behavior. This is where `launch.Substitution`s can be used to provide more flexibility when describing reusable launch descriptions.

  Basic actions (`launch.actions.`):
  - `DeclareLaunchArgument`: declares a `LaunchDescription` argument, with a `name`, `default_value` and `description`. This argument can be exposed via command line option for a root launch description, or as action configuration.

- **`launch.Substitution`**: entities evaluated during launch (e.g. a configuration, an environment variable, an arbitrary Python expression...).
  
  Basic substitutions (`launch.substitutions.`):
  - `LaunchConfiguration`: This substitution gets a launch configuration value, as a string, by name.
  - `Command`: Substitution that gets the output of a command as a string.
  - `EnvironmentVariable`: This substitution gets an environment variable value, as a string, by name.
  - `FindExecutable`: This substitution locates the full path to an executable on the PATH if it exists.
  - `PathJoinSubstitution`: Substitution that join paths, in a platform independent way.

- **`launch.LaunchService`**: long-running activity that handles the event loop and dispatches actions.

## 3.2. Architecture of `launch_ros`
The main classes are:
- **`launch_ros.Action`**, with basic actions (`launch_ros.actions`):
  - `Node`: executes a Node.

## 3.3. Launch files
Launch files are inside `launch/` directory, with the `.launch.py` suffix.

Launch files should define the `generate_launch_description()` function, which returns a `launch.LaunchDescription()` to be used by the `ros2 launch` command.

```Python
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            "arg_name",
            default_value = '''<value>''',
            description = "description",       
        )
    )
    #...

    # Generate nodes
    nodes = []
    nodes.append(
      my_node = launch_ros.actions.Node(
        package="pkg_name",
        executable="exec_name",
        name="my_name",
        output='''????''',
        arguments='''???''',
        parameters='''???''',
      )
    )



    return launch.LaunchDescription(declared_arguments + nodes)
```

## 3.4. Launch for large projects
