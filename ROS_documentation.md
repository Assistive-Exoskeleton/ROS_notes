# 1. ROS documentation
## 1.1. Contents
- [1. ROS documentation](#1-ros-documentation)
  - [1.1. Contents](#11-contents)
- [2. Nodes](#2-nodes)
  - [2.1. ROScore](#21-roscore)
- [3. Master](#3-master)
- [4. Parameter server](#4-parameter-server)
- [5. Messages](#5-messages)
- [6. Topics](#6-topics)
- [7. Services](#7-services)
- [9. Community level](#9-community-level)

# 2. Nodes
Useful commands for **nodes**:
- to manage running nodes use `rosnode`:
  ```powershell
  #rosnode help
  $ rosnode -h
  #list active nodes
  $ rosnode list [namespace]
  #print node info
  $ rosnode info <node>
  ...
  ```
- run a single node:
    ```powershell
    $ rosrun [package_name] [node_name]
    ```
- run nodes from launch files (`roscore` included):
  ```powershell
  $ roslaunch [package_name] [launch_file.launch]
  ```
  The launch file consists of some tags:
  ```XML
  <launch> <!--identifies the file as launch-->
    <group ns="namespace_name">
      <node pkg="pkg_name" name="node_name" type="node_type"/>
    </group>
    ...
  </launch>
  ```

## 2.1. ROScore
`roscore` is a collection of [nodes](#nodes) and programs that are pre-requisites of a ROS-based system. You **must** have a roscore running in order for ROS nodes to communicate.
> Note: `roslaunch` automatically launches also `roscore`

`roscore` starts up:
- a ROS [Master](#master)
- a ROS [Parameter server](#parameter-server)
- a `rosout` logging node (collects nodes debugging output)

# 3. Master
# 4. Parameter server
Useful commands for **parameters**:
- To manage parameters use `rosparam`:
  ```powershell
  #rosparam help
  $ rosparam -h
  
  #list parameter names
  $ rosparam list

  #set and get parameters
  $ rosparam set <parameter> <args>
  $ rosparam get <parameter>

  #manage parameters from files
  $ rosparam load [file_name] [namespace]
  $ rosparam dump [file_name] [namespace]
  ```
# 5. Messages
ROS messages structures used by topics are stored in `package/msg/*.msg` files.

To look at details of a message you can use:
```powershell
$ rosmsg show <msg_type> #for topics
$ rossrv show <service_type> #for services
```

# 6. Topics
Useful commands for **topics**:
- To manage topics use `rostopic`:
  ```powershell
  #rostopic help
  $ rostopic -h

  #list active topics
  $ rostopic list [topic] [-options]
  #   -options:
  #     -v: verbose, display also type, publishers
  #         and subscribers

  #print topic info
  $ rostopic info <topic>

  #print topic/message type
  $ rostopic type <topic>

  #publish onto a topic
  $ rostopic pub [options] <topic> <type> <args>
  #   -options:
  #     -1: publish just once
  #     -r <hz>: publish continuously at rate <hz>

  #subscribe shell to a topic and print output
  $ rostopic echo <topic>

  ...
  ```


# 7. Services
In ROS, services are defined using `package/srv/*.srv` files

Useful commands for **services**:
- To manage services use `rosservice`
  ```powershell
  #rosservice help
  $ rosservice -h

  #list active services
  $ rosservice list

  #print info about a service
  $ rosservice info <service>

  #print service/messages type
  $ rosservice type <service>

  #call a service
  $ rosservice call <service> [args]

  ...
  ```
# 9. Community level