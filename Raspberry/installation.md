# Install ROS on raspberry 

## Prerequisites

> ⚠ This guide is written for ROS2 Foxy over Ubuntu 20.04 server. 

Install Ubuntu 20.04 **server** using rpi-manager:

```bash
sudo snap install rpi-imager
```

### Getting setup with Wi-Fi

If is not possible to use the Ethernet connection, a preliminary setup is required.

With the SD card still inserted in the laptop, open a file manager and locate the “system-boot” partition on the card. It contains initial configuration files that load during the first boot process.

Edit the network-config file to add Wi-Fi credentials.
``` c
wifis:
  wlan0:
    dhcp4: true
    optional: true
    access-points:
      "home network":
        password: "123456789"
```

> ⚠ be careful to keep indentation spaces (2 spaces for each indentation, don't use tabs)

Save the file and extract the card from your laptop.

> Note ⓘ: During the first boot, Raspberry Pi will try to connect to this network. It will fail the first time around. Simply reboot `sudo reboot` and it will work.

### Set static IP
To set a static IP you need to replace the

> dhcp4: true

line in the network-config file with lines that specify the intended IP address as well as its default gateway and DNS server. You can do this for either the eth0 or wlan0 interface (or both). **It is important to get the indenting right for this work correctly.**

For example, if is planned to give the pi the address 192.168.1.23 in the 192.168.1.0/24 subnet with a default gateway of 192.168.1.1 and a DNS server of 192.168.1.53 then the following text would work. The same structure works for both the eth0 or wlan0 sections in the file:

``` c
ethernets:
  eth0:
    addresses:
      - 192.168.101.23/24
    gateway4: 192.168.1.1
    nameservers:
      addresses: [192.168.1.23]
    optional: true
```

### Change Network config after first boot

Edit 50-clound-init file with

``` bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

## ROS installation
Remember to install ROS on Raspberry or using SSH to connect from a network machine

``` bash
ssh ubuntu@<ip_address>
```

### Setup locale
``` bash
sudo locale-gen it_IT it_IT.UTF-8
sudo update-locale LC_ALL=it_IT.UTF-8 LANG=it_IT.UTF-8
export LANG=it_IT.UTF-8
```
### Setup sources
``` bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```
### Install ROS2 core packages
After adding ROS2 repo to the source list, update with
``` bash
sudo apt update
```
and install ROS2
``` bash
sudo apt install ros-foxy-ros-base
```
> Note ⓘ: is not installed ros-foxy-desktop, is installed ros-foxy-ros-base, which contains no GUI tools, just the bare minimum need to write and execute ROS2 programs.

### Install colcon (build tool)
To build ROS2 package install colcon.

ROS2 uses colcon as a build tool (and ament as the build system). When installed the ROS2 core packages, colcon is not here, so install it manually.
``` bash
sudo apt install python3-colcon-common-extensions
```

### Auto-completion for ROS2 command line tools
ROS2 comes with a lot of useful command line tools, and if you want to use auto-completion for those tools, you’ll need to install python3-argcomplete.
``` bash
sudo apt install python3-argcomplete
```

### Setup your environment for ROS2
Source ROS environment variables at each terminal opening with 
``` bash
source /opt/ros/foxy/setup.bash
```
or add this line to .bashrc file
``` bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```
