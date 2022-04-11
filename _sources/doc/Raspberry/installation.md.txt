# ROS installation
Remember to install ROS on Raspberry or using SSH to connect from a network machine

``` bash
ssh ubuntu@<ip_address>
```


## Setup locale
``` bash
sudo locale-gen it_IT it_IT.UTF-8
sudo update-locale LC_ALL=it_IT.UTF-8 LANG=it_IT.UTF-8
export LANG=it_IT.UTF-8
```
## Setup sources
``` bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```
## Install ROS2 core packages
After adding ROS2 repo to the source list, update with
``` bash
sudo apt update
```
and install ROS2
``` bash
sudo apt install ros-foxy-ros-base
```
```{note}
Is not installed ros-foxy-desktop, is installed ros-foxy-ros-base, which contains no GUI tools, just the bare minimum need to write and execute ROS2 programs.
```

## Install colcon (build tool)
To build ROS2 package install colcon.

ROS2 uses colcon as a build tool (and ament as the build system). When installed the ROS2 core packages, colcon is not here, so install it manually.
``` bash
sudo apt install python3-colcon-common-extensions
```

## Auto-completion for ROS2 command line tools
ROS2 comes with a lot of useful command line tools, and if you want to use auto-completion for those tools, you’ll need to install python3-argcomplete.
``` bash
sudo apt install python3-argcomplete
```

## Setup your environment for ROS2
Source ROS environment variables at each terminal opening with 
``` bash
source /opt/ros/foxy/setup.bash
```
or add this line to .bashrc file
``` bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```
