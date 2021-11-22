# `rlc` (ROS Client Library)
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

## Nodes and `Node` class
Main methods:
  - `__init__`: initialize `Node`
    |parameter|type|description|
    |---------|----|-----------|
    |`node_name`|`str`|name of the node
    |`context`|`Context`|optional context
    |`cli_args`|`List(str)`|optional list of command-line arguments
    |`namespace`|`str`|optional namespace to which relative topic/service names will be prefixed
    ```python
    #Example
    class myNode(Node):
      __init__(self):
        super.__init__('myNodeName')
    ```
  - `create_publisher->Publisher`: create a new publisher
    parameter|type|description|
    |--------|----|-----------|
    |`msg_type`|`.msg` imported object|type of ros msg it publishes
    |`topic`|`str`|topic to which it publishes.
    |`qos_profile`|`Union(QosProfile,int)`|[`QoS_profile`](https://docs.ros2.org/latest/api/rclpy/api/qos.html#rclpy.qos.QoSProfile) or history depth/queue size (`int`) to apply to a publisher.
    ```python
    #Example
    myNode.my_pub = myNode.create_publisher(MyType,'myTopic',10)
    ```
  - `create_subscription->Subscription`: create a subscription:
    |parameter|type|description|
    |--------|----|-----------|
    |`msg_type`|`.msg` imported object|type of ros msg it subscribes to
    |`topic`|`str`|topic to which it subscribes 
    |`qos_profile`|`Union(QosProfile,int)`|[`QoS_profile`](https://docs.ros2.org/latest/api/rclpy/api/qos.html#rclpy.qos.QoSProfile) or history depth/queue size (`int`) to apply to the subscription.
    |`callback`|`Callable`|user-defined callback, called when a message is received by the subscription
    |`raw`|`bool`| If True, then received messages will be stored in raw binary representation.
    ```python
    #Example
    myNode.my_sub = myNode.create_subscription(MyType,'myTopic',10, my_sub_callback)
    def my_sub_callback(message):
      #message is like a struct that can be accessed
      #through message.fieldname
      ...
    ```
  - `create_service->Service`: create a new service server:
    |parameter|type|description|
    |---------|----|-----------|
    |`srv_type`|`.srv` imported object|type of service interface
    |`srv_name`|`str`|name of the service
    |`callback`|`Callable [SrvTypeRequest, SrvTypeResponse]`|user-defined callback, called when a service request is received by the server. You can send request and response as parameters.
    |`qos_profile`|`QoSProfile`|Quality of service profile to be applied to the service's server
    ```python
    #Example
    myNode.my_srv = myNode.create_service(MyType,'my_service', my_srv_callback)
    def my_srv_callback(request, response):
      ...
      #request and response are like struct that can be accessed
      #through request.fieldname or response.fieldname
      return response

    ```
  - `create_client->Client`: create a new service client
    |parameter|type|description|
    |---------|----|-----------|
    |`srv_type`|`.srv` imported object|type of service interface
    |`srv_name`|`str`|name of the service
    |`qos_profile`|`QoSProfile`|Quality of service profile to be applied to the service's client
    ```python
    #Example
    class MyCli(Node):
      #...
      self.my_cli = self.create_client(MyType,'my_service')

      request = MyType.Request() #this method is generated from the .srv file using rosidl generators. See ROS_concepts/interfaces.
    
      def send_request(self):
        #access request field by request.<fieldname>
        #for example we can use:
        self.future = myNode.my_cli.call_async(request)
    
    #...
    cli_node = MyCli()
    #we can get results as:
    if cli_node.future.done():
      try:
        response = cli_node.my_cli.future.result()
      except Exception as e:
        #exception, request failed
      else:
        #request succeeded, use response to access result

    ```

  - `create_timer->Timer`: create a new timer, which calls a callback function every period
    |parameter|type|description|
    |---------|----|-----------|
    |`timer_period_sec`|`float`|Period (s) of timer
    |`callback`|`Callable`|user-defined callback, called when the timer expires.
  - `get_logger()->RcutilsLogger`: gets the node logger
  - `destroy_node()`: explicitly destroys the node and frees resources (subscribers, publishers...). Note that this is not always required as resources may be freed by Python's garbage collector.
    > To destroy individual resources use `destroy_<resource> (<resource>)` (e.g. `destroy_publisher(publisher)`)

## Topics
There are two main classes:

1. **`Publisher`**, with main methods:
   - `get_subscription_count()->int`: get amount of subscribers that the publisher has
   - `publish`: publishes a message
        |parameter|type|description|
        |---------|----|-----------|
        |`msg`|`Union(MsgType,bytes)`|ROS message to publish
   - `destroy()`
   - `@property topic_name->str`

2. **`Subscription`**, with main methods:
   - `destroy()`
   - `@property topic_name->str`

## Services
There are two main classes:
1. **`Service`**: ROS service server, with main methods:
   - `send_response`: send a service response
        |parameter|type|description|
        |---------|----|-----------|
        |`response`|`SrvTypeResponse`|service response|
        |`header`||capsule pointing to the service header from the original request
   - `destroy()`
2. **`Client`**: ROS service client, with main methods:
   - `call->SrvTypeResponse`: Make a service request and await result
        |parameter|type|description|
        |---------|----|-----------|
        |`request`|`SrvTypeRequest`|service request
   - `call_async->Future`: make a service request and asyncronously get the result. The `Future` object completes when the request does.
        |parameter|type|description|
        |---------|----|-----------|
        |`request`|`SrvTypeRequest`|service request
   - `service_is_ready()->bool`: check if there's a service server ready
   - `wait_for_service->bool`: wait until service is ready (`True` if it is)
        |parameter|type|description|
        |---------|----|-----------|
        |`timeout_sec`|`float`|timeout, returning `False` if it expires|
   - `destroy()`

## `Context` (#TODO)
A `Context` encapsulates the lifecycle of init and shutdown. The context is used in the creation of top level entities like nodes and guard conditions, as well as to shutdown a specific instance of init.

## `RcutilsLogger`
Main class for logging information about a node. It's instantiated by `Node.get_logger()` method. Main methods:
- `log`: log a message with a specified severity
    |parameter|type|description|
    |---------|----|-----------|
    |`message`|`str`|message to log|
    |`severity`|`LoggingSeverity`|severity of the message (`DEBUG`, `INFO`, `WARN`, `ERROR`, `FATAL` ...)
> There are also individual functions depending on severity: `RcutilsLogger.debug(message)`, `RcutilsLogger.info(message)` ... They simply call the `log` function with the `severity` argument predefined.

## Execution and callbacks
Callbacks are units of work like subscription callbacks, timer callbacks, service calls, and received client responses. There are two components that control the execution of callbacks: **executors** and **callback groups**.

- **`Executor`**: controls the threading model used to process callbacks (which thread callbacks get executed in).
- **`CallbackGroup`**: used to enforce concurrency rules for callbacks.

## Quality of service
**[QoS](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html) policies** allow to tune communication between nodes. A set of QoS policies combine to form a **QoS profile**. ROS2 provides predefined QoS profiles to simplify the choice of correct policies for a given use case.

> A QoS profile can be applied independently to each instance of publishers, subscribers, service servers and clients. However, if different profiles are used it is possible that they will be incompatible, preventing the delivery of messages.

The base QoS profile currently includes settings for the following policies:

- **History**
  - **Keep last**: only store up to N samples, configurable via the queue depth option.
  - **Keep all**: store all samples, subject to the configured resource limits of the underlying middleware.

- **Depth**
  - **Queue size**: only honored if the “history” policy was set to “keep last”.

- **Reliability**
  - **Best effort**: attempt to deliver samples, but may lose them if the network is not robust.
  - **Reliable**: guarantee that samples are delivered, may retry multiple times.

- **Durability**
  - **Transient local**: the publisher becomes responsible for persisting samples for “late-joining” subscriptions.
  - **Volatile**: no attempt is made to persist samples.

- **Deadline**
  - **Duration**: the expected maximum amount of time between subsequent messages being published to a topic

- **Lifespan**
  - **Duration**: the maximum amount of time between the publishing and the reception of a message without the message being considered stale or expired (expired messages are silently dropped and are effectively never received).

- **Liveliness**
  - **Automatic**: the system will consider all of the node’s publishers to be alive for another “lease duration” when any one of its publishers has published a message.
  - **Manual by topic**: the system will consider the publisher to be alive for another “lease duration” if it manually asserts that it is still alive (via a call to the publisher API).

- **Lease Duration**
  - **Duration**: the maximum period of time a publisher has to indicate that it is alive before the system considers it to have lost liveliness (losing liveliness could be an indication of a failure).

For each of the policies that is not a duration, there is also the option of “system default”, which uses the default of the underlying middleware. For each of the policies that is a duration, there also exists a “default” option that means the duration is unspecified, which the underlying middleware will usually interpret as an infinitely long duration.