# RQt robot steering

rqt_robot_steering provides a GUI plugin for steering a robot using Twist messages.

## Usage Example (with care-o-bot)

As noted, rqt_robot_steering publishes geometry_msgs/Twist as a name of /cmd_vel by default. This name may vary per robot/application; e.g. [Care-o-bot](http://wiki.ros.org/Robots/Care-O-bot) also publishes geometry_msgs/Twist but as base_controller/command (see [cob_teleop#Published_Topics](http://wiki.ros.org/cob_teleop#Published_Topics). So, type that name into the topic field at the top of rqt_robot_steering GUI and it gets applied immediately.

To verify, use also [rqt_topic](http://wiki.ros.org/rqt_topic) (or run from commandline rostopic echo) to see if the topics base_controller/command are actually published. 