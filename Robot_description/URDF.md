# URDF
**Table of contents**
- [URDF](#urdf)
- [Description](#description)
  - [Link](#link)
  - [Joint](#joint)

# Description
The Unified Robotic Description Format (**URDF**) is an `XML` file format used in ROS to describe all elements of a robot. The drawback is that only *tree* structures with rigid links can be represented, ruling out parallel robots and flexible elements.

The specification covers:
- Kinematic and dynamic description of the robot
- Viual representation of the robot
- Collision model of the robot

The description of a robot consists of a set of [link elements](#link) and a set of [joint elements](#joint):
```XML
<robot name="my_robot">
    <link>...</link>
    <link>...</link>
    ...
    <joint>...</joint>
    <joint>...</joint>
</robot>
```

## Link
Links represent the skeleton of the robot. The link element describes a **rigid body** with an inertia, visual features and collision properties.

Attributes:
- `name`: the name of the link (required)
  ```XML
  <link name="my_link">
  ```

Elements:
- `<inertial>`: inertial properties of the link
  - `<origin>`: pose of the inertial reference frame relative to the reference frame of the link (needs to be at the center of gravity)
    - `xyz`: represents the offset
    - `rpy`: represents the fixed axis roll, pitch and yaw angles in radians
  - `<mass>`: mass of the link represented by the value attribute (`<mass value="my_mass">`)
  - `<inertia>`: 3x3 rotational inertia matrix, represented in the inertia frame. Only 6 DOF as attributes: `ixx`, `ixy`, `ixz`, `iyy`, `iyz`, `izz`.
- `<visual>`: visual properties of the link (e.g. shape), with attribute `name`. 
  > Note: multiple instances can exist: the union of the geometry they define forms the final representation.
  - `<origin>`: reference frame of the visual element with respect to that of the link. Same elements as inertial origin.
  - `<geometry>`: shape of the visual object
    - `<box>`, with attributes `size` and origin in the center.
    - ...
    - `<mesh>`: trimesh element specified by a `filename` and an optional `scale`.
  - `<material>`: material of the element, with attribute `name`
    - `<color>`: rgba color
    - `<texture>`: texture from `filename`
- `<collision>`: collision properties, with `name`.
  > Note: multiple instances can exists. The final collision model is the union of the instances.
  - `<origin>`: reference frame of collision element relative to that of the link. Same elements as inertial origin.
  - `<geometry`> same elements as visual geometry.

## Joint
The joint element describes the kinematics and dynamics of the joint, also specifying safety limits of the joint.

Attributes:
- `name`: unique name of the joint
- `type`: type of joint (`"revolute"`, `"continuous"`, `"prismatic"`, `"fixed"`, `"floating"`, `"planar"`)

Elements:
- `<origin>`: transform from parent link to child link. The joint is located at the origin of the child link (same attributes `xyz` and `rpy` as before)
- `<parent>`: parent link name (`link` attribute)
- `<child>`: child link name (`link` attribute)
- `<axis>`: joint axis specified in the joint frame. It's the functional axis (rotation, translation... depending on the joint type), with attribute `xyz`
- `<calibration>`: reference positions of the joint, used to calibrate the absolute position
  - `rising`: when joint moves in a positive direction, the reference position will trigger a rising edge
  - `falling`: when joint moves in a positive direction, this reference position will trigger a falling edge.
- `<dynamics>`: speficy physical properties of the joint.
  - `damping`
  - `friction`
- `<limit>`: 
  - `lower`: lower joint limit (omit if joint is continuous)
  - `upper` upper joint limit (omit if continuous)
  - `effort`: maximum joint effort
  - `velocity`: maximum joint velocity
- `<mimic>`: used to specify that the defined joint mimics another existing joint.
- `<safety_controller>`: more strict boundary than lower and upper joint limits, specifying when the safety controller starts limiting the position of the joint (soft limit).