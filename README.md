# dio_ros_driver
ROS Driver for accessing DIO with Linux-based manner

# Overvew
In the golf-cart project, usage of DIO is required.  
This system serves DIO ROS driver for a single port.

You can choose gpio chip and gpio line with giving arguments as below. 

```
$ roslaunch dio_ros_driver dio_ros_driver.launch chip_name:="gpiochip0" line_offset:=32
```

After executing this command, `dio_ros_driver_node` will run.
You can observe `/startbutton` topic. `/startbutton` is boolean data to show that selected button is pushed.


## Arguments
* `chip_name`: to select `gpiodchip[0-9]` in `/dev` directory
* `line_offset`: to select GPIO line from selected `gpiochip[0-9]`
* `di_active_low`: if true, DI line is active low

## Topics
* `/dio_ros_driver/startbutton_raw`: raw DI value from libgpiod
* `/startbutton`: detection of startbutton pushed


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


