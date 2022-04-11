# Concepts

## Nodes
Useful commands for **nodes**:
- to manage running nodes use `rosnode`:
  ```sh
  #rosnode help
  $ rosnode -h
  #list active nodes
  $ rosnode list [namespace]
  #print node info
  $ rosnode info <node>
  ...
  ```
- run a single node:
    ```sh
    $ rosrun [package_name] [node_name]
    ```
- run nodes from launch files (`roscore` included):
  ```sh
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

### ROScore
`roscore` is a collection of [Nodes](#nodes) and programs that are pre-requisites of a ROS-based system. You **must** have a roscore running in order for ROS nodes to communicate.
```{note}
`roslaunch` automatically launches also `roscore`
```

`roscore` starts up:
- a ROS [Master](#master)
- a ROS [Parameter server](#parameter-server)
- a `rosout` logging node (collects nodes debugging output)

## Master
The ROS Master provides naming and registration services to the rest of the nodes in the ROS system. It tracks publishers and subscribers to topics as well as services. The role of the Master is to enable individual ROS nodes to locate one another. Once these nodes have located each other they communicate with each other peer-to-peer. Nodes connect to other nodes directly; the Master only provides lookup information, much like a DNS server.

The Master also provides the Parameter Server.

## Parameter server
Useful commands for **parameters**:
- To manage parameters use `rosparam`:
  ```sh
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
## Messages
ROS messages structures used by topics are stored in `package/msg/*.msg` files.

To look at details of a message you can use:
```sh
$ rosmsg show <msg_type> #for topics
$ rossrv show <service_type> #for services
```

## Topics
Useful commands for **topics**:
- To manage topics use `rostopic`:
  ```sh
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


## Services
In ROS, services are defined using `package/srv/*.srv` files

Useful commands for **services**:
- To manage services use `rosservice`
  ```sh
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
## Community level