# Command Line Interface
**Contents**:
- [Command Line Interface](#command-line-interface)
	- [Help and documentation](#help-and-documentation)
	- [Launch and run](#launch-and-run)
	- [Nodes](#nodes)
	- [Interfaces](#interfaces)
	- [Topics](#topics)
	- [Services](#services)
	- [Parameters](#parameters)
	- [Actions](#actions)
	- [Packages](#packages)

[`ros2cli`](https://github.com/ros2/ros2cli) is the command line interface library. All `ros2cli` tools start with the prefix `ros2` followed by a **command**, a **verb** and (possibly) **arguments**.

## Help and documentation
For any **command** the documentation is accessible through:
```powershell
$ ros2 <command> --help
```
Also for any **verb** the documentation is accessible through:
```powershell
ros2 <command> <verb> -h
```

## Launch and run
To run an executable:
```powershell
$ ros2 run <package_name> <executable_name>
```
See also [ROS2 node configuration via command line arguments](https://design.ros2.org/articles/ros_command_line_arguments.html)

To launch a `.launch.py` file:
```powershell
$ ros2 launch <package_name> <launch_file.launch.py>
```

## Nodes
Nodes cli tools use the command `ros2 node`:
- list running nodes:
	```powershell
	$ ros2 node list
	```
- info about nodes (subscribers, publishers, services, and actions that interact with that node):
	```powershell
	$ ros2 node info <node_name>
	```

## Interfaces
The various ROS2 interfaces (messages, topics, services, actions) cli tools use the command `ros2 interface`:
- list all available interface types:
	```powershell
	$ ros2 interface list [-options]
	# -options:
	#   --only-msgs: list only messages
	#	--only-srvs: list only services
	#	--only-actions: list only actions 
	```
- output the interface definition:
  ```powershell
  $ ros2 interface show <interface_type>
  ```

> Each interface has its own command (e.g. `topic`, `service`, `action`). Note that **arguments** passed to an interface call (e.g. `topic pub`, `service call`, `action send_goal`) are written in `YAML` format, a dictionary-like syntax similar to json.

## Topics
Topics cli tools use the command `ros2 topic`:
- list running topics:
	```powershell
	$ ros2 topic list [-options]
	#-options:
	#	-t: list also topic types
	```
- Display info about a topic (type, count of publishers and subscribers)
	```powershell
	$ ros2 topic info <topic_name>
	```
- add an echo subscriber for a topic and echo the output on the terminal:
	```powershell
	$ ros2 topic echo <topic_name>
	```
- publish data to a topic from command line
	```powershell
	$ ros2 topic pub [--once | --rate] <topic_name> <msg_type> ['<args>']
	#--once: publish the message only once then exit
	#--rate <Hz>: decide a frequency of publishing
	```
- view the rate at which data is published:
	```powershell
	$ ros2 topic hz <topic_name>
	```

## Services
Services cli tools use the command `ros2 service`:
- list active services:
	```powershell
	$ ros2 service list [-options]
	#-options:
	#	-t: list also service types
	```
- call a service:
	```powershell
	$ ros2 service call <service_name> <service_type> ['<args>']
	```

## Parameters
Parameters cli tools use the command `ros2 param`:
- list available parameters:
	```powershell
	$ ros2 param list
	```
- see a parameter type and current value:
	```powershell
	$ ros2 param get <node_name> <parameter_name>
	```
- set a parameter value:
	```powershell
	$ ros2 param set <node_name> <parameter_name> <value>
	```
- save current parameter into a file in yaml format:
	```powershell
	$ ros2 param dump <node_name>
	```
- load parameter from file:
	```powershell
	$ ros2 param load <node_name> <parameter_file>
	```

## Actions
Actions cli tools use the command `ros2 action`:
- list active actions:
	```powershell
	$ ros2 action list [-options]
	#-options:
	#	-t: list also action types
	```
- display action info (clients and servers nodes)
	```powershell
	$ ros2 action info <action_name>
	```
- send an action goal
	```powershell
	$ ros2 action send_goal <action_name> <action_type> ['<args>'] [-options]
	#-options:
	#	--feedback: also display feedback
	```

## Packages
Packages cli tools use the command `ros2 pkg`:
- list available packages:
	```powershell
	$ ros2 pkg list
	```
- create a new package:
	```powershell
	$ ros2 pkg create ...
	```