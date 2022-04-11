# `ros2_control` User Interfaces
Users interact with the `ros2_control` framework using [Controller Manager](ros2_control_concepts.md#controller-manager)'s **services**.
While service calls can be used directly from the command line or via nodes, there exists a user-friendly [`ros2_control CLI`](https://github.com/ros-controls/ros2_control/tree/master/ros2controlcli) which integrates with the [ros2 cli](../Standard_Libraries/ros2cli.md). This supports auto-complete and has a range of common commands available.

## `ros2_control` [CLI](https://github.com/ros-controls/ros2_control/tree/master/ros2controlcli)
The user can interact with the **CM** through command line interface. The basic command is **control**:
```sh
$ ros2 control <verb> [-options]
# -options:
#       -h: help
#       -c <CM>: specify controller manager ROS node
#       --include-hidden-nodes
```

### Queries
- `list_controllers`: output a lists of loaded controllers, their type and status:
  ```sh
  $ ros2 control list_controllers [-options]
  ```
- `list_controller_types`: list available controllers types and their base classes
  ```sh
  $ ros2 control list_controller_types [-options]
  ```
- `list_hardware_interfaces`: output the list of interface types (i.e. `state` and `command` interfaces) for loaded controllers and their status
  ```sh
  $ ros2 control list_hardware_interfaces [-options]
  ```

### Controller lifecycle management
- `load_controller`: load a controller in the CM
  ```sh
  $ ros2 control load_controller [-options] <controller_name>
  # -options
  #     --set_state {configure,start}: set the state of the loaded controller
- `reload_controller_libraries`: reloads controller libraries (?)
  ```sh
  $ ros2 control reload_controller_libraries [-options]
  # -options
  #     --force-kill: force stop loaded controllers
  ```
- `set_controller_state`: set the state of a controller
  ```sh
  $ ros2 control set_controller_state [-options] <controller_name> <state>
  # <state> = {configure,start,stop}: state of the controller to be set
  ```
- `switch_controllers`: switch controllers in a CM:
  ```sh
  $ ros2 control switch_controllers [-options]
  # -options:
  #     --stop [STOP[STOP ...]]: name of the controllers to be stopped
  #     --start [START [START ...]]: name of the controllers to be started
  #     --strict: strict switch
  #     --start-asap: start the controllers as soon as possible
  #     --switch-timeout <timeout>: timeout for switching controllers 
  ```
- `unload_controller`: unloads a controller in the CM:
  ```sh
  $ ros2 control load_controller [-options] <controller_name>
  ```