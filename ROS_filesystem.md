# 1. Contents
- [1. Contents](#1-contents)
- [2. Environment variables](#2-environment-variables)
- [3. Packages](#3-packages)
  - [3.1. Catkin](#31-catkin)
  - [3.2. Common files and directories](#32-common-files-and-directories)
  - [3.3. Command line tools](#33-command-line-tools)
  - [3.4. Package manifest](#34-package-manifest)
    - [3.4.1. Dependencies](#341-dependencies)
    - [3.4.2. Metapackages](#342-metapackages)
  - [3.5. CMakeLists.txt](#35-cmakeliststxt)
- [4. Message types](#4-message-types)
- [5. Service types](#5-service-types)

# 2. Environment variables
In Linux and Unix based systems, environment variables are a set of dynamic named values, stored within the system that are used by applications launched in shells or subshells. In simple words, an environment variable is a variable with a **name** and an associated **value**.

Environment variables serve a variety of purposes for ROS:
- **Finding packages**
  - `ROS_ROOT` and `ROS_PACKAGE_PATH` enable ROS to locate packages and stacks in the filesystem. 
  - `PYTHONPATH` enables python to find ROS libraries.
- **Effecting a Node runtime**
  - `ROS_MASTER_URI` tells a [Node](#nodes) where the [Master](#master) is.
  - `ROS_IP` and `ROS_HOSTNAME` affect the network address of a [Node](#nodes)
  - `ROS_NAMESPACE` changes the namespace of a [Node](#nodes).
  - `ROS_LOG_DIR` lets you set the directory where log files are written
- **Modifying the build system**

See [ROS guide](http://wiki.ros.org/ROS/EnvironmentVariables) for a list of ROS environment variables, or use:
```powershell
$ printenv | grep ROS
```

# 3. Packages
Software in ROS is organized in packages, containing anything that constitutes a module. Packages are directories descended from `ROS_PACKAGE_PATH` that have a `package.xml` file in it.

```powershell
my_package/
  package.xml
```

## 3.1. Catkin
Catkin is the official build system for ROS. A catkin **workspace** is like:
```powershell
workspace_folder/         #WORKSPACE
  build/                  #BUILD SPACE
  devel/                  #DEVEL SPACE
  install/                #INSTALL SPACE
  src/                    #SOURCE SPACE
    CMakeLists.txt        #'Toplevel' CMake file
    pkg_1/
      CMakeLists.txt      #pkg_1 CMake file
      package.xml         #pkg_1 manifest
    ...
    pkg_n/
      CMakeLists.txt      #pkg_n CMake file
      package.xml         #pkg_n manifest
```
To create a catkin **workspace** just generate the two directories:
```powershell
$ mkdir -p <workspace_name>/src
```
To build the workspace (empty or existing) use (from `workspace/`):
  ```powershell
  $ catkin_make [make_targets]
  ``` 
There are three layers of the build process:
1. First, packages in the **source space** (`workspace/src/`) are built the **build space** (`workspace/build/`), where CMake and catkin keep cache and intermediate build files.
2. Build targets are then placed in the **devel space** (`workspace/devel/` or `CATKIN_DEVEL_PREFIX`) prior to be installed. This provides an useful testing environment that doesn't require installing. It contains all **generated files** (libraries, executables, code...). Also there are `setup.*sh` scripts to add this devel space to the **environment**:
   ```powershell
   $ source devel/setup.bash
   ```
3. Once targets are built, they can be **installed** into the **install space** (`workspace/install/` or `CMAKE_INSTALL_PREFIX`). This can be done with:
   ```powershell
   $ catkin_make install # might need sudo
   ```
  
To create a catkin **package** you can also use:
```powershell
$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```



## 3.2. Common files and directories
ROS packages follow a common structure:
```powershell
CMakeLists.txt:   #CMake build file
package.xml       #package manifest
----------
include/pkg_name  #C++ include headers
msg/              #folder containing message types
src/              #source files
srv/              #folder containing service types
scripts/          #executable scripts
```



## 3.3. Command line tools
To manage packages:
- `rospack`: find and retrieve information about packages
  ```powershell
  $ rospack help
  $ rospack list #list all packages
  $ rospack find <pkg> #print path to pkg
  $ rospack depends <pkg> #list all dependencies
  $ rospack depends1 <pkg> #list all first-order dependencies
  $ rospack depends-on <pkg> #list all packages that depend on pkg
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

  To install dependencies of a package and also **build** it with catkin use:
  ```powershell
  $ rosdep install <package_name>
  ```
  To install dependencies of all packages in workspace use (from `workspace/`):
  ```powershell
  $ rosdep install --from-paths src --ignore-src -r -y
  
  --from-paths: 'the arguments will be considered paths to be searched, acting on all catkin packages found there in'
  --ignore-src: 'rosdep will ignore keys that are found to be catkin or ament packages anywhere in the ROS_PACKAGE_PATH, AMENT_PREFIX_PATH or in any of the directories given by the --from-paths option'
  -r: 'Continue installing despite errors'
  -y: 'Tell the package manager to default to y (yes) or fail when installing'
  ```
- `roscd`: `cd` into a ROS package
  ```powershell
  $ roscd <pkg>
  $ roscd log #folder with log files
  ```
- `rosls`: `ls` into a ROS package
  ```powershell
  $ rosls <pkg>
  ```

## 3.4. Package manifest
The package manifest is an XML file called `package.xml` that defines properties about the package, such as *package name*, *version*, *authors*, *maintainers* and [dependencies](#dependencies). These information are important when the package gets released to the ROS community.

Each `package.xml` has a `<package>` tag as root tag:
```XML
<package format="2">

</package>
```
The minimal tags to make the manifest complete are:
```XML
<package format="2">
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
```
### 3.4.1. Dependencies
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
- **Build tool dependencies**: specify build system tools (e.g. `catkin`).
  ```XML
  <buildtool_depend>pkg_name</buildtool_depend>
  ```
- **Documentation dependencies**: documentation tools needed to generate documentation
  ```XML
  <doc_depend>pkg_name</doc_depend>
  ```

### 3.4.2. Metapackages
It's often convenient to group multiple packages as a single logical package or **metapackage**. A metapackage is a normal package with the following tag in `package.xml`:
```XML
<export>
  <metapackage />
</export>
```
Other than a required `<buildtool_depends>` on `catkin`, metapackages can only have *execution dependencies* on packages of which they group.

## 3.5. CMakeLists.txt
The file `CMakeLists.txt` is the input to the CMake build system for building software packages.

In order to work, it has to follow this exact format in the exact order:
1. **Required CMake version** (`catkin` requires > 2.8.3):
   ```CMake
   cmake_minimum_required(VERSION 2.8.3)
   ```

2. **Package name**:
   ```CMake
   project(<name>)
   ```
   Note in CMake you can reference the project name later, using the variable `${PROJECT_NAME}`.

3. **Find other CMake/Catkin packages needed for build**:
   ```CMake
   find_package(catkin REQUIRED)
   ```
   If your project depends on other wet packages, they are automatically turned into components (in terms of CMake) of catkin:
   ```CMake
   find_package(catkin REQUIRED COMPONENTS <pkg_name>)
   ```
4. **Enable Python module support**:
   ```CMake
   catkin_python_setup()
   ```
5. **Message/Service/Action Generators**: Messages (.msg), services (.srv), and actions (.action) files in ROS require a special preprocessor build step before being built and used by ROS packages.
   
   There are three macros provided to handle messages, services, and actions respectively:
   ```CMake
   add_message_files()
   add_service_files()
   add_action_files()
   ```
6. **Invoke message/service/action generation**: Previous macros must be followed by a call to the macro that invokes generation:
   ```CMake
   generate_messages()
   ```
7. **Specify package build info export**:
   ```CMake
   catkin_package(
   INCLUDE_DIRS include #exported include paths
   LIBRARIES ${PROJECT_NAME} #exported libraries
   CATKIN_DEPENDS <pkg> #catkin dependencies
   DEPENDS <pkg> #non-catkin dependencies
   )
   ```

8. **Libraries/Executables to build**: Build targets can take many forms, but usually they represent one of two possibilties:
    - **Executable Target** - programs we can run
    - **Library Target** - libraries that can be used by executable targets at build and/or runtime

    ```CMake
    add_library()
    add_executable()
    target_link_libraries()
    ```

9. **Tests to build**:
    ```CMake
    catkin_add_gtest()
    ```

10. **Install rules**:
    ```CMake
    install()
    ```

# 4. Message types
# 5. Service types
