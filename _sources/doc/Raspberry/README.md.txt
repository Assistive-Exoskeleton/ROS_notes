# Install ROS on Raspberry


<h2> Prerequisites </h2>

```{warning}
This guide is written for ROS2 Foxy over Ubuntu 20.04 server. 
```

Install Ubuntu 20.04 **server** using rpi-manager:

```bash
sudo snap install rpi-imager
```

<h3> Getting setup with Wi-Fi </h3>

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

```{note}
During the first boot, Raspberry Pi will try to connect to this network. It will fail the first time around. Simply reboot `sudo reboot` and it will work.
```

<h3> Set static IP </h3>

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

<h3> Change Network config after first boot </h3>

Edit 50-clound-init file with

``` bash
sudo nano /etc/netplan/50-cloud-init.yaml
```



```{toctree}
---
maxdepth: 2
hidden:
---

installation
shared_network
Rpi_servo
```