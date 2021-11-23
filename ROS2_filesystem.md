# ROS2 filesystem
**Contents**:
- [ROS2 filesystem](#ros2-filesystem)
- [Environment variables](#environment-variables)
- [Packages](#packages)
	- [Package structure](#package-structure)
	- [Workspaces](#workspaces)
	- [Command line tools](#command-line-tools)
	- [Package Manifest (`package.xml`)](#package-manifest-packagexml)
		- [Dependencies](#dependencies)
		- [Metapackages](#metapackages)
	- [CMakeLists.txt #TODO](#cmakeliststxt-todo)

# Environment variables
In Linux and Unix based systems, environment variables are a set of dynamic named values, stored within the system that are used by applications launched in shells or subshells. In simple words, an environment variable is a variable with a **name** and an associated **value**.

Sourcing ROS 2 setup files will set several environment variables necessary for operating ROS2. A list of variables can be printed through:
```powershell
$ printenv | grep -i ROS
```

# Packages
A package can be considered a container for the ROS2 code, organized for installation or sharing. 

Package creation in ROS2 uses **ament** as build system and [colcon](https://docs.ros.org/en/rolling/Tutorials/Colcon-Tutorial.html) as build tool. Officially supported methods for creating a package are **CMake** or **Python**.

## Package structure
Minimum required contents for a package:

|CMake				|Python				|Description|
|-------------------|-------------------|-----------|
|`package.xml`		|`package.xml`		|contains meta-information about the package
|`CMakeLists.txt`	|`setup.py`			|description on how to build/install the package
|					|`setup.cfg`		|required when a package has executables, so `ros2 run` can find them
|					|`/<package_name>`	|directory with the same name as the package, containing `__init__.py`

ROS2 packages often contain also:
|directory|description|
|---------|-----------|
|`include/pkg_name`|include headers|
|`src/`|source files|
|`msg/`|message types|
|`srv/`|service types|
|`action/`|action types|
|`scripts/`|executable scripts|

## Workspaces
A single workspace can contain many packages, of different build types, but no nested packages. Best practice is to have a `/src` folder and creating packages in there:
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

## Command line tools
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
		--symlink-install: 'use symlinks instead of copying files where possible'
	```

- use the new package and executable, adding the workspace to the path (inside `workspace/` main folder):
	```powershell
	$ . install/local_setup.bash
	```

- `rosdep`: install system dependencies of a package. 
  
  To install and update:
  ```powershell
  # installation on ROS Noetic
  $ sudo apt install python3-rosdep
  # Initialize rosdep (call only once after installation)
  $ sudo rosdep init
  # Update rosdep (DO NOT CALL AS SUDO)
  $ rosdep update
  ```
  > Note: `python3-rosdep2` is not for ROS2, it's an older version of rosdep

  To install dependencies of all packages in workspace use (from `workspace/`):
  ```powershell
  $ rosdep install --from-paths src --ignore-src -r -y --rosdistro <my_distro>
  
  --from-paths: 'the arguments will be considered paths to be searched, acting on all catkin packages found there in'
  --ignore-src, -i: 'rosdep will ignore keys that are found to be catkin or ament packages anywhere in the ROS_PACKAGE_PATH, AMENT_PREFIX_PATH or in any of the directories given by the --from-paths option'
  -r: 'Continue installing despite errors'
  -y: 'Tell the package manager to default to y (yes) or fail when installing'
  --rosdistro <my_distro>: 'for example rolling, galactic...'
  ```

## Package Manifest (`package.xml`)
The package manifest is an XML file called `package.xml` that defines properties about the package, such as *package name*, *version*, *authors*, *maintainers* and [dependencies](#dependencies). These information are important when the package gets released to the ROS community.

Each `package.xml` has a `<package>` tag as root tag:
```XML
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">

</package>
```
The minimal tags to make the manifest complete are:
```XML
<package format="3">
  <name>pkg_name</name>
  <version>pkg_version</version>
  <description>pkg_description</description>
  <maintainer email="my_email">my_name</maintainer>
  <license>pkg_license</license>
</package>
```
Other tags:
```XML
<author></author>

<url></url>

<export>
   <build_type>ament_cmake</build_type>
</export>

<export>
   <build_type>ament_python</build_type>
</export>
```

>Note: when building with python, also the `setup.py` file must be edited with these fileds, and they need to match exactly in both files.

### Dependencies
The `package.xml` with minimal tags doesn't specify **dependencies**. There are 6 types of dependencies:
> **Note**, the tag:
> ```XML
> <depend>pkg_name</depend>
> ```
> is the most used and includes build, build-export and execution dependencies.
 
- **Build dependencies**: packages needed (at build-time) to build this package. This may be including headers from these packages at compilation time, linking against libraries from these packages.
  ```XML
  <build_depend>pkg_name</build_depend>
  ```
- **Build export dependencies**: packages needed to build libraries against this package.
  ```XML
  <build_export_depend>pkg_name</build_export_depend>
  ```
- **Execution dependencies**: packages needed to run code in this package.
  ```XML
  <exec_depend>pkg_name</exec_depend>
  ```
- **Test dependencies**: specify only *additional* dependencies for unit test. Do not duplicate dependencies already mentioned.
  ```XML
  <test_depend>pkg_name</test_depend>
  ```
- **Build tool dependencies**: specify build system tools (e.g. `ament_cmake`, there's none for `ament_python`).
  ```XML
  <buildtool_depend>pkg_name</buildtool_depend>
  ```
- **Documentation dependencies**: documentation tools needed to generate documentation
  ```XML
  <doc_depend>pkg_name</doc_depend>
  ```

### Metapackages
It's often convenient to group multiple packages as a single logical package or **metapackage**. A metapackage is a normal package with the following tag in `package.xml`:
```XML
<export>
  <metapackage />
</export>
```

## CMakeLists.txt #TODO