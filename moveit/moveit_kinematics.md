# MoveIt - kinematics plugin 

MoveIt uses a plugin infrastructure, especially targeted towards allowing users to write their own inverse kinematics algorithms. Forward kinematics and finding jacobians is integrated within the RobotState class itself. The default inverse kinematics plugin for MoveIt is configured using the `KDL` numerical jacobian-based solver. This plugin is automatically configured by the MoveIt Setup Assistant.

### IKFast Plugin

Often, users may choose to implement their own kinematics solvers, e.g. the PR2 has its own kinematics solvers. A popular approach to implementing such a solver is using the IKFast package to generate the C++ code needed to work with your particular robot.

## IKFast Kinematics Solver

[qui](https://moveit.picknik.ai/galactic/doc/examples/ikfast/ikfast_tutorial.html?highlight=ikfast#ikfast-kinematics-solver)

[//]: #TODO

## Collision Checking

Collision checking in MoveIt is configured inside a Planning Scene using the `CollisionWorld` object. MoveIt is set up so that users never really have to worry about how collision checking is happening. Collision checking in MoveIt is mainly carried out using the `FCL` package - the primary collision checking library of MoveIt.

MoveIt supports collision checking for different types of objects including:


- Meshes - you can use either `.stl` (standard triangle language) or `.dae` (digital asset exchange) formats to describe objects such as robot links.

- Primitive Shapes - e.g. boxes, cylinders, cones, spheres and planes

- Octomap - the `Octomap` object can be directly used for collision checking

## Allowed Collision Matrix (ACM)

Collision checking is a very expensive operation often accounting for close to 90% of the computational expense during motion planning. The `Allowed Collision Matrix` or `ACM` encodes a binary value corresponding to the need to check for collision between pairs of bodies (which could be on the robot or in the world). If the value corresponding to two bodies is set to `true` in the ACM, it specifies that a collision check between the two bodies is either not required or wanted. The collision checking would not be required if, e.g., the two bodies are always so far away that they can never collide with each other. Alternatively, the two bodies could be in contact with each other by default, in which case the collision detection should be disabled for the pair in the ACM.