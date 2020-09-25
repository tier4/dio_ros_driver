# dio_ros_driver
ROS Driver for accessing DIO with libgpiod

# Overvew
In the golf-cart project, DIO module is used for several purpose.  
This system serves DIO ROS driver for at most 8 ports.

`dio_ros_driver` sends topics which includes boolean value read from corresponding DI port.
It receives topics which includes boolean value and write DO port.
Topics are allocated to respective port.


## Execution
```
$ roslaunch dio_ros_driver dio_ros_driver.launch chip_name:="gpiochip0" access_frequency:=10.0
```

After executing this command, `dio_ros_driver_node` will run without any message.
You can observe topics such as `/dio/din[0-7]`, `/dio/din_status`, `/dio/dout[0-7]`. `/dio/dout_status`. `/dio/din[0-7]` and `/dio/dout_status` has boolean value which shows the state of corresponding DI and DO port.  `/dio/din_status` and `/dio/dout_status` indicates status and all values of respective ports.


## Arguments
* `chip_name`: to select `gpiodchip[0-9]` in `/dev` directory
* `access_frequency`: to set access frequency (default: 10.0)
* `din_value_inverse`: to inverse raw value from DIN port. If this value is true, raw value 0 is converted into value 1 when publishing topic (default: false)
* `dout_value_inverse`: to inverse raw value from DOUT port. If this value is true, raw value 0 is converted into value 1 when setting value (default: false)
* `dout_default_value`: initial boolean value for DO ports (default: true)
  * default value is true because initial value from DO port of ADLINK's MVP-6100 has **1 (true)**

## Topics
* `/dio/din[0-7]`
  * message type: `dio_ros_driver/DIOPort`
  * description: Boolean value read from DI ports
* `/dio/din_status`
  * message type: `dio_ros_driver/DIOStatus`
  * description: Status and all ports' value from DI ports
* `/dio/dout[0-7]`
  * message type: `dio_ros_driver/DIOPort`
  * descritpion: Boolean value written into DO ports
* `/dio/dout_status`
  * message type: `dio_ros_driver/DIOStatus`
  * description: Status and all ports' value from DO ports

**Note(1):** If `status` field of `/dio/din_status` is not equaled to 0, all DI ports cannot be accessed and any DI topic cannot published. 

**Note(2):** If `status` field of `/dio/dout_status` is not equaled to 0, all DO ports cannot be accessed and updated.


## Config file
In [`port_list.yaml`](./msg/port_list.yaml), port offset is listed as below. 

```
din_ports:
  - 72 # DIN port 0
  - 73 # DIN port 1
  - 74 # DIN port 2
  - 75 # DIN port 3
  - 76 # DIN port 4
  - 77 # DIN port 5
  - 78 # DIN port 6
  - 79 # DIN port 7

dout_ports:
  - 105 # DOUT port 0
  - 106 # DOUT port 1
  - 107 # DOUT port 2
  - 108 # DOUT port 3
  - 109 # DOUT port 4
  - 110 # DOUT port 5
  - 111 # DOUT port 6
  - 112 # DOUT port 7
```

Each offset is assigned to ordering number. This `port_list.yaml` indicates that 0th DI port is corresponded to 72th offset of the DI module.  
The list is defined based on DIO module coupled with ADLINK's MVP-6100 series.

## Constraints
* This node read data from DI port and write value to DO port periodically. If port value is updated several times in less period than access cycle, the node would use the last value when updating ports.


# Setup
## Prerequisite
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

## Build package and execute ROS node
Build `dio_ros_package` with `colcon` command.
```
$ cd <working directory>
$ source /opt/ros/melodic/setup.bash
$ git clone git@github.com:tier4/dio_ros_driver.git # if you clone repository via ssh
$ colcon build
$ source install/setup.bash
$ roslaunch dio_ros_driver dio_ros_driver.launch
```

After the command sequence, you can find `dio_ros_driver` node and topics.

## Running `dio_ros_driver` on ADLINK's MVP-6100
All ports of DIO, served by ADLINK's MVP-6100, has value which is active low.

The following table shows relation between port value and polarity.

| port value | polarity |
| ---        | ---      |
| 0          | positive |
| 1          | negative |


**Caution:** After startup, initial value would be 1 as negative polarity. It is strongly recommended that **1** should be set on DO ports during shutting down. If inversion option is enabled, **0** must be set.


# Design
Please refer to [class diagram](./design/class.md), [sequence diagram](./design/sequence.md), and [activity diagram](./design/activity.md).



