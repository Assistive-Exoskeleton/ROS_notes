# SRDF and URDF


## URDF

MoveIt starts with a URDF (Universal Robot Description Format), the native format for describing robots in ROS. In this tutorial, you will find resources for the URDF, important tips and also a list of MoveIt specific requirements.

**Resources available:**

* [URDF principles](../gazebo/URDF.md)
* [SOLIDWORKS URDF Plugin](https://wiki.ros.org/sw_urdf_exporter) - A plugin that lets you generate a URDF directly from a SOLIDWORKS model.

### Collision Checking

MoveIt uses the meshes specified in the URDF for collision checking. The URDF allows you to specify two sets of meshes separately for visualization and collision checking. In general, the visualization meshes can be detailed and pretty, but the collision meshes should be much less detailed. The number of triangles in a mesh affects the amount of time it takes to collision check a robot link. The number of triangles in the whole robot should be on the order of a few thousand.

### Test your URDF

It is very important to test your URDF out and make sure things are ok. The easiest ways are:
* VsCode extension [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros), then use URDF preview. 
* Build basic ROS2 package and launch `description` package; the easiest way to check joints domains is to include `robot_state_publisher_gui` package into lauch file. (check [templates repository](https://github.com/Assistive-Exoskeleton/Templates_ROS2) to easily create basic ROS packages).  
* Matlab tools for ROS/ROS2. 


## SRDF

The SRDF or Semantic Robot Description Format complement the URDF and specifies joint groups, default robot configurations, additional collision checking information, and additional transforms that may be needed to completely specify the robot’s pose. The recommended way to generate a SRDF is using the MoveIt Setup Assistant.

### SRDF Structure

```xml
<?xml version="1.0"?>

 <!-- This does not replace URDF, and is not an extension of URDF.
      This is a format for representing semantic information about the robot structure.
      A URDF file must exist for this robot as well, where the joints and the links that are referenced are defined -->
 <robot name="some robot">

   <group name="group1">
      <!-- when a link is specified, the parent joint of that link (if it exists) is automatically included -->
      <link name="..."/>
      <link name="..."/>

      <!-- when a joint is specified, the child link of that joint (which will always exist) is automatically included -->
      <joint name="..." />

      <!-- when a chain is specified, all the links along the chain (including endpoints) are included in the group. Additionally, all the joints that are parents to included links are also included. This means that joints along the chain and the parent joint of the base link are included in the group -->
      <chain base_link="l_shoulder_pan_link" tip_link="l_wrist_roll_link"/>
      <chain base_link="r_shoulder_pan_link" tip_link="r_wrist_roll_link"/>
   </group>

   <!-- groups can also be formed by referencing to already defined group names -->
   <group name="arms">
      <group name="left_arm"/>
      <group name="right_arm"/>
      <link name="..." />
   </group>

   <!-- define a named state/configuration of a group -->
   <group_state name="name of this state" group="name of group the state is for">
      <joint name="name of joint in group" value="" />
      <!-- all joints must be specified for the state to be valid -->
   </group_state>

   <!-- Define how the robot moves in its environment, i.e., connection to robot's root link -->
   <virtual_joint name="world_joint" type="planar" parent_frame="some fixed frame" child_link="robot's root link name"/> <!-- type can be planar, floating or fixed -->

   <!-- We can then include the virtual joint in groups -->
   <group name="whole_body">
      <group name="arms"/>
      <joint name="world_joint"/>
   </group>


   <!-- define end effectors -->
   <end_effector name="some diff name" parent_link="..." group="group_name"/>

   <!-- By default it is assumed that any link of the robot could potentially come into collision with any other link in the robot. This tag disables collision checking between a specified pair of links. There can be many such tags in this file.-->
   <disable_collisions link1="link1" link2="link2" />

</robot>
```

For further information, check [official wiki](https://wiki.ros.org/srdf). 

### Virtual Joints

The URDF contains information only about the physical joints on the robot. Often, additional joints need to be defined to specify the pose of the root link on the robot with respect to a world coordinate system. In such cases, a virtual joint is used to specify this connection. E.g., a mobile robot like the PR2 that moves around in the plane is specified using a planar virtual joint that attaches the world coordinate frame to the frame of the robot. A fixed robot (like an industrial manipulator) should be attached to the world using a fixed joint.


### Passive Joints

Passive joints are unactuated joints on a robot, e.g. passive casters on a differential drive robots. They are specified separately in the SRDF to make sure that different components in the motion planning or control pipelines know that the joints cannot be directly controlled. If your robot has unactuated casters, they should be specified as passive casters.


### Groups

a ‘Group’ (sometimes called ‘JointGroup’ or ‘Planning Group’) is a central concept in MoveIt. MoveIt always acts on a particular group. MoveIt will only consider moving the joints in the group that it is planning for – other joints are left stationary. (A motion plan where all joints in the robot may move can be achieved by creating a group containing all joints.) A group is simply a collection of joints and links. Each group can be specified in one of several different ways:


### Collection of Joints

A group can be specified as a collection of joints. All the child links of each joint are automatically included in the group.


### Collection of Links

A group can also be specified as a collection of links. All the parent joints of the links are also included in the group.


### Serial Chain

A serial chain is specified using the base link and the tip link. The tip link in a chain is the child link of the last joint in the chain. The base link in a chain is the parent link for the first joint in the chain.


### Collection of Sub-Groups

A group can also be a collection of groups. E.g., you can define left_arm and right_arm as two groups and then define a new group called both_arms that includes these two groups.


### End-Effectors

Certain groups in a robot can be given a special designation as an end-effector. An end-effector is typically connected to another group (like an arm) through a fixed joint. Note that when specifying groups that are end-effectors, it’s important to make sure that there are no common links between the end-effector and the parent group it is connected to.


### Self-Collisions

The Default Self-Collision Matrix Generator (part of Setup Assistant) searches for pairs of links on the robot that can safely be disabled from collision checking, decreasing motion planning processing time. These pairs of links are disabled when they are always in collision, never in collision, in collision in the robot’s default position or when the links are adjacent to each other on the kinematic chain. The sampling density specifies how many random robot positions to check for self collision. Higher densities require more computation time while lower densities have a higher possibility of disabling pairs that should not be disabled. The default value is 10,000 collision checks. Collision checking is done in parallel to decrease processing time.


### Robot Poses

The SRDF can also store fixed configurations of the robot. A typical example of the SRDF in this case is in defining a HOME position for a manipulator. The configuration is stored with a string id, which can be used to recover the configuration later.


