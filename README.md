# dio_ros_driver
ROS Driver for accessing DIO with Linux-based manner

# Overvew
In the golf-cart project, DIO module is used for several purpose.  
This system serves DIO ROS driver for at most 8 ports.

`dio_ros_driver` sends topics which includes boolean value read from corresponding DI port.
It receives topics which includes boolean value and write DO port.
Topics are allocated to respective port.


## Behavior
```
$ roslaunch dio_ros_driver dio_ros_driver.launch chip_name:="gpiochip0" access_frequency:=10.0
```

After executing this command, `dio_ros_driver_node` will run without any message.
You can observe `/startbutton` topic. `/startbutton` is boolean data to show that selected button is pushed.


## Arguments
* `chip_name`: to select `gpiodchip[0-9]` in `/dev` directory
* `din_default_value`: initial boolean value for DI ports, in other words, this signal decides active low or not. (default: false, if this value is true, active low is enabled.)
* `dout_default_value`: initial boolean value for DO ports
* `access_frequency`: access frequency (default: )

## Topics
* `/dio/din[0-7]`: Boolean value read from DI ports
* `/dio/din_status`: Status and all ports' value from DI ports
* `/dio/dout[0-7]`: Boolean value written into DO ports 
* `/dio/dout_status`: Status and all ports' value from DO ports


## Config file
In [`port_list.yaml`](./msg/port_list.yaml), port offset is listed as below.

```
din_ports:
  - 72
  - 73
  - 74
  - 75
  - 76
  - 77
  - 78
  - 79

dout_ports:
  - 105
  - 106
  - 107
  - 108
  - 109
  - 110
  - 111
  - 112
```

Each offset is assigned to ordering number. This `port_list.yaml` indicates that 0th DI port is corresponded to 72th offset of the DI module.  


# Prerequisite
System configuration:
* OS: Ubuntu18.04
* ROS: ROS Melodic
* Other library: libgpiod


What you have to do for using this package is shown as below

1. Installing the related packages
1. Changing the permission of gpiochip you want to use


## Installing the related packages.
You have to install `libgpiod` package as below before using this package.

```
$ sudo apt install libgpiod1 libgpiod-dev libgpiod-doc gpiod
```

After installing all package, you can detect gpio chip as follow.

```
$ sudo gpiodetect
```

## Changing the permission of gpiochip
You access `/dev/gpiochip0` via `libgiod` package.
Non-root user cannot acccess `/dev/gpiochip0` without `sudo`.

You have to add new group and edit startup configuration file.

1. Add new group

```
$ sudo adduser --group gpio
$ sudo usermod -aG gpio autoware
$ reboot # to apply setting.
```

2. Change permission of `/dev/gpiochip0`
You have to change the permission of gpio chip file.
Please open `/etc/rc.local` and edit as below.

```
#!/bin/sh -e

chown root.gpio /dev/gpiochip0
chmod g+rw /dev/gpiochip0
exit 0
```

Add execution permission to the `/etc/rc.local`
```
$ sudo chmod +x /etc/rc.local
$ reboot
```


