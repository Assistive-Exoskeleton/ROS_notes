# ROS2 documentation
## Contents
- [ROS2 documentation](#ros2-documentation)
	- [Contents](#contents)
- [ROS2 basic operations](#ros2-basic-operations)
	- [Nodes](#nodes)
	- [Messages](#messages)
	- [Topics](#topics)
	- [Services](#services)
	- [Parameters](#parameters)
	- [Actions](#actions)
- [Packages](#packages)
	- [Package structure](#package-structure)
	- [Packages in a workspace](#packages-in-a-workspace)
	- [Customize package xml](#customize-package-xml)


# ROS2 basic operations
- Launch an executable:
	```powershell
	$ ros2 run <package_name> <executable_name>
	```
	See also [ROS2 node configuration via command line arguments](https://design.ros2.org/articles/ros_command_line_arguments.html)

## Nodes
Useful commands for **nodes**:
- list running nodes:
	```powershell
	$ ros2 node list
	```
- info about nodes, returns a list of subscribers, publishers, services, and actions that interact with that node:
	```powershell
	$ ros2 node info <node_name>
	```

## Messages
To look at details of a message you can use:
```powershell
$ ros2 interface show <msg_type>
```

## Topics
Useful commands for **topics**:
- list running topics:
	```powershell
	$ ros2 topic list [-options]
	
	#-options:
	#	-t: list also topic types in square brackets
	```

- Display info about a topic, such as type, count of publishers and subscribers.
	```powershell
	$ ros2 topic info <topic_name>
	```

- add a subscriber node for a topic and echo the output on the terminal:
	```powershell
	$ ros2 topic echo <topic_name>
	```

- inspect a topic message type
	```powershell
	$ ros2 interface show <msg_type>
	```

- publish data onto a topic from command line
	```powershell
	$ ros2 topic pub [--once | --rate] <topic_name> <msg_type> '<args>'

	#--once: publish the message only once then exit
	#--rate <Hz>: decide a frequency of publishing
	```

- view the rate at which data is published:
	```powershell
	$ ros2 topic hz <topic_name>
	```

## Services
Useful commands for **services**:
- list active services:
	```powershell
	$ ros2 service list [-options]

	-options:
		-t: 'list also service types in square brackets'
	```

- see type of a service:
	```powershell
	$ ros2 service type <service_name>
	```

- find all services for a specific type:
	```powershell
	$ ros2 service find <service_type>
	```

- inspect a service type:
	```powershell
	$ ros2 interface show <service_type>
	```
	This will return something like:
	```
	<input arguments structure to call the service>
	---
	<type of response from the call>
	```

- call a service:
	```powershell
	$ ros2 service call <service_name> <service_type> ['<arguments>']
	```
	>Note: arguments are optional, for example Empty types have no arguments

## Parameters
Useful commands for **parameters**:
- list parameters of nodes:
	```powershell
	$ ros2 param list
	```

- see a parameter type and current value:
	```powershell
	$ ros2 param get <node_name> <parameter_name>
	```

- set a parameter value at runtime:
	```powershell
	$ ros2 param set <node_name> <parameter_name> <value>
	```

- log current node parameter state into a file:
	```powershell
	$ ros2 param dump <node_name>
	```

- load parameter state from file:
	```powershell
	$ ros2 param load <node_name> <parameter_file>
	```

## Actions
Useful commands for **actions**:
- list active actions:
	```powershell
	$ ros2 action list [-options]

	-options:
		-t: 'list also action types in square brackets'
	```

- display action info, such as clients and servers nodes
	```powershell
	$ ros2 action info <action_name>
	```
- inspect an action type structure:
	```powershell
	$ ros2 interface show <action_type>
	```
	This will return something like:
	```shell
	<structure (type and name) of goal request>
	---
	<structure of the result>
	---
	<structure of the feedback>
	```
- send an action goal
	```powershell
	$ ros2 action send_goal <action_name> <action_type> '<values>' [-options]

	-options:
		--feedback: 'also display feedback'
	```
	Again `'<values>'` is in YAML format.

# Packages
A package can be considered a container for your ROS2 code, organized for installation or sharing. 

Package creation in ROS2 uses **ament** as build system and [colcon](https://docs.ros.org/en/rolling/Tutorials/Colcon-Tutorial.html) as build tool. Officially supported methods for creating a package are **CMake** or **Python**.

## Package structure
Minimum required contents for a package:

|CMake				|Python				|Description|
|-------------------|-------------------|-----------|
|`package.xml`		|`package.xml`		|contains meta-information about the package
|`CMakeLists.txt`	|`setup.py`			|description on how to build/install the package
|					|`setup.cfg`		|required when a package has executables, so `ros2 run` can find them
|					|`/<package_name>`	|directory with the same name as the package, containing `__init__.py` 

## Packages in a workspace
A single workspace can contain many packages, of different build types, but no nested packages. Best practice is to have a `/src` folder and creating packages in there.

Example of workspace:
```
workspace_folder/
	src/
		package_1/
			CMakeLists.txt
			package.xml
		package_2/
			setup.py
			package.xml
			resource/package_2
		...
		package_n/
			CMakeLists.txt
			package.xml
```

Useful commands for **packages**:
- create a package (inside `workspace/src`):
	```powershell
	$ ros2 pkg create [-options] <package_name>

	-options:
		--build-type {ament_python|ament_cmake}: 'choose the type of method for building the package'
		--node-name <my_node>: 'creates a simple Hello World type executable in the package' 
	```
- build the workspace (inside `workspace/` main folder):
	```powershell
	$ colcon build [-options]
	
	-options:
		--packages-select <package_name>: 'build only selected package'
	```

- use the new package and executable, adding the workspace to the path (inside `workspace/` main folder):
	```powershell
	$ . install/local_setup.bash
	```

- solve dependencies (inside `workspace/` main folder):
	```powershell
	$ rosdep install -i --from-path src --rosdistro <my_distro> -y
	```
	`<my_distro>` for example can be `rolling`.

## Customize package xml
The main lines to edit in `package.xml` are:
```XML
<maintainer email="user@todo.todo">user</maintainer>
<description>TODO: Package description</description>
<license>TODO: License declaration</license>
```

>Note: when building with python, also the `setup.py` file must be edited with these fileds, and they need to match exactly in both files.