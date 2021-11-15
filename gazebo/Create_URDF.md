# URDF Structure 
<pre>
Robot
├── link1
│   ├── visual
│   │   ├── geometry
│   │   ├── origin
│   │   └── material
│   └── [inertial]
│       ├── mass
│       └── inertia
├── link2
│   └── ...
├── joint1
│   ├── axis
│   ├── origin
│   ├── parent
│   ├── child
│   ├── limit
│   └── dynamics
├── joint2
│   └── ...
<span├──  style="color:red">ros2_control
│   ├── hardware
│   │   └── plugin
│   └── joint
│       ├── command_interface
│       ├── state_interface1
│       ├── [state_interface2]
│       └── [state_interface3]
└── gazebo
    └── plugin
        └── parameters</span>
</pre>
