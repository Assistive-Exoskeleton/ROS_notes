# RQt (formerly RQt2)
rqt is a Qt-based framework for GUI development for ROS.

## Advantages
RQt implements various tools and interfaces in the form of plugins and makes it easier to manage all of them in a single screen layout. 

Users can create their own plugins for RQt with either Python or C++. Over 20 plugins were created in ROS 1 and these plugins are currently being ported to ROS 2 (work in progress as of march 2022).

Compared to building your own GUIs from scratch:

* Standardized common procedures for GUI (start-shutdown hook, restore previous states).
* Multiple widgets can be docked in a single window.
* Easily turn your existing Qt widgets into RQt plugins. (see [this repo](https://github.com/Assistive-Exoskeleton/Templates_ROS2) for easily create a basic RQt plugin compatible with ROS2)

## RQt plugins
RQt consists of three parts/metapackages:

* rqt - core infrastucture modules.
* **rqt_common_plugins** - Backend tools for building tools.
* **rqt_robot_plugins*** - Tools for interacting with robots during runtime. They need to be installed separately.

  ```{toctree}
  ---
  maxdepth: 1
  ---

  rqt_robot_plugins
  ```

\*<font size="1">Only ROS2 compatible plugins are listed.</font>



## RQt perspectives

A perspective is a combination of RQT plugins in a defined configuration. In the Perspectives menu you can save, import, export and set perspectives as default. 

A perspective saves the plugins loaded, their layout, and where supported, their settings and last-saved initial parameters (such as what topic we were last plotting).

### Save a perspective 

When you’re happy with a dashboard configuration, you can save the perspective by selecting `Perspectives > Create Perspective`, giving it a name, and asking it to clone the current perspective. These perspectives are saved locally and persist between sessions.

To export a perspective for closer management such as sharing or persisting to a repository, select `Perspectives > Export` and give it a name with the filename extension, .perspective.

### Load a perspective 

A perspective can be loaded interactively in RQT by selecting Perspectives, import. However it’s useful to launch them directly from the command-line, which allows us to wrap them in a script that can be roslaunched:
``` shell
rqt -p <path> 
```
or
``` shell
rqt --perspective-file <path>
```
If you want to load a perspective from a launch file (ROS2), add following node:

``` python
from launch.substitutions import EnvironmentVariable

    ... code ...

node_name = Node(
    package="rqt_gui",
    executable="rqt_gui",
    respawn = "false",
    output = "screen",
    arguments = ["--perspective-file", [EnvironmentVariable('PWD'), "/src/perspectives/FILENAME.perspective"]]
)
```
In this example `FILENAME.perspective` is placed into `"/WORKSPACE/src/perspectives"`

Loading perspectives is useful when is necessary to load multiple rqt plugins into the same rqt instance. 