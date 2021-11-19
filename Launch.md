# Launch
Launch libraries and launch files.

**Table of contents:**
- [Launch](#launch)
  - [Architecture of `launch`](#architecture-of-launch)
  - [Architecture of `launch_ros`](#architecture-of-launch_ros)
  - [Launch files](#launch-files)
  - [Launch for large projects](#launch-for-large-projects)
  
The [launch system](https://docs.ros.org/en/galactic/Tutorials/Launch-system.html) in ROS 2 is responsible for helping the user describe the configuration of their system (programs to run, where to run them, arguments, ...) and then execute it as described. It is also responsible for monitoring the state of the processes launched, and reporting and/or reacting to changes in the state of those processes.

Launch files written in Python can start and stop different nodes as well as trigger and act on various events. The package providing this framework is [`launch_ros`](https://github.com/ros2/launch_ros), which uses the non-ROS-specific [`launch`](https://github.com/ros2/launch) framework underneath.

## Architecture of `launch`
The main classes are:
- **`launch.LaunchDescription`**: it encapsulates the intent of the user as a list of discrete **`launch.Action`**'s.
- **`launch.Action`**: actions can have direct effect (run a process or set a configuration variable) or yield additional actions. 

  Actions may also have arguments affecting their behavior. This is where `launch.Substitution`-s can be used to provide more flexibility when describing reusable launch descriptions.

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

## Architecture of `launch_ros`
The main classes are:
- **`launch_ros.Action`**, with basic actions (`launch_ros.actions`):
  - `Node`: executes a Node.

## Launch files
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

## Launch for large projects
