# XACRO
XACRO is an XML macro language, used to construct shorter and more readable XML files.

## URDF with XACRO
The `xacro` program runs all the macros and outputs the result.
```sh
#example
$ xacro model.xacro > model.urdf
```

The URDF can also be generated through a launch file. Note it might take time to generate.
```python
from launch.substitutions import Command
robot_description_content = Command(
    ['xacro ', str(path_to_xacro)]
)# equivalent to $xacro model.xacro
```

### Writing the XACRO file
At the top the `namespace` must be specified in order for the file to parse properly.
```XML
<?xml version="1.0"?>

<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
```
- **properties** act as **constants**:
  ```XML
  <xacro:property name="my_prop" value="my_val">
  ```
  These can then be expanded as `${my_prop}`:
  ```sh
  ${my_prop} -> my_val
  ```
- Arbitrarily complex expressions can be expanded with `${expression}`. It can contain properties, numbers, (`+`,`-`,`*`,`/`,`sin`,`cos`) and other operators
- **macros** can be defined as:
  ```XML
  <xacro:macro name="my_macro">
    content
  </xacro:macro>
  ```
  And then expanded with `<xacro:my_macro />`:
  ```XML
  <xacro:my_macro /> -> content
  ```
  It can also be parametrized:
  ```XML
  <xacro:macro name="my_macro" params="my_param">
    content and ${my_param}
  </xacro:macro>
  ```
  So that the parametrized expression is used as:
  ```XML
  <xacro:my_macro my_param="my_val">
  ```
  Parameters can also be entire blocks. They're defined as:
  ```XML
  <xacro:macro name="my_macro" params="my_param *my_block">
    content
    <xacro:insert_block name="my_block">
    other content
  </xacro:macro>
  ```
  And used as:
  ```XML
  <xacro:my_macro my_param="my_val">
    this is my parameter block
  </xacro:my_macro>
  ```

Other common tricks include:
- using a name **prefix** to get similarly named joints:
  ```XML
  <xacro:macro name="my_macro" params="prefix">
    ...example (right and left leg links)
    <link name="${prefix}_leg">
    ...
  </xacro:macro>
  <xacro:my_macro prefix="right">
  <xacro:my_macro prefix="left">
  ```
- use math to calculate values. Useful in case the size of the robot needs to be changed
  ```XML
  <origin xyz="0 0 -${length/2}" rpy="0 ${pi/2} 0"/>
  ```
- use a "reflect" parameter (+1 or -1) in case of simmetries of the robot
- ...