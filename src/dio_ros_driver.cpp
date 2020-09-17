/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Package: dio_ros_dirver
 * File Name: dio_ros_dirver.cpp
 * Author: Takayuki AKAMINE
 * Description: ROS driver for DI module.
 *              This driver sends DI value topic.
 */
#include <cstring>
#include <iostream>
#include <boost/bind.hpp>

#include <dio_ros_driver/DIOPort.h>
#include <dio_ros_driver/DIOStatus.h>

#include "dio_ros_driver/dio_ros_driver.hpp"

#include <ros/ros.h>

namespace dio_ros_driver
{
  DIO_ROSDriver::DIO_ROSDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
      : nh_(nh),
        pnh_(pnh),
        din_port_publisher_array_(),
        dout_port_subscriber_array_(),
        din_status_publisher_(),
        dout_status_publisher_(),
        din_accessor_(),
        dout_accessor_(),
        access_frequency_(1.0),
        chip_name_("chipname0"),
        active_low_(false),
        write_update_mutex_(),
        dout_user_update_(),
        dio_chip_(NULL)
  {
    // load general parameters.
    pnh_.param<double>("access_frequency", access_frequency_, 1.0);
    pnh_.param<std::string>("chip_name", chip_name_, "gpiochip0");
    pnh_.param<bool>("active_low", active_low_, false);

    // prepare publishers
    for (uint32_t i = 0; i < MAX_PORT_NUM; i++)
    {
      std::string topic_name = "/dio/din" + std::to_string(i);
      din_port_publisher_array_[i] = nh_.advertise<dio_ros_driver::DIOPort>(topic_name, 1, false);
    }
    // prepare subscribers
    for (uint32_t i = 0; i < MAX_PORT_NUM; i++)
    {
      std::string topic_name = "/dio/dout" + std::to_string(i);
      dout_port_subscriber_array_[i] = nh_.subscribe<dio_ros_driver::DIOPort>(topic_name,
                                                                              1,
                                                                              boost::bind(&DIO_ROSDriver::requestUserWrite, this, _1, i));
    }

    din_status_publisher_ = nh_.advertise<dio_ros_driver::DIOStatus>("/dio/din_status", 1, false);
    dout_status_publisher_ = nh_.advertise<dio_ros_driver::DIOStatus>("/dio/dout_status", 1, false);
  }

  int DIO_ROSDriver::init(void)
  {
    dio_chip_ = gpiod_chip_open_by_name(chip_name_.c_str());
    din_accessor_.setDIOChip(dio_chip_);
    dout_accessor_.setDIOChip(dio_chip_);

    initAccessor("/dio/din_ports", din_accessor_);
    initAccessor("/dio/dout_ports", dout_accessor_);

    return 0;
  }

  void DIO_ROSDriver::initAccessor(const std::string param_name, DIO_AccessorBase &dio_accessor)
  {
    // get port number array.
    std::vector<int32_t> offset_array;
    nh_.getParam(param_name, offset_array);

    for (const auto &offset : offset_array)
    {
      dio_accessor.addPort(offset);
    }
    return;
  }

  void DIO_ROSDriver::requestUserWrite(const dio_ros_driver::DIOPort::ConstPtr &dout_topic, const uint32_t &port_id)
  {
    write_update_mutex_.lock();
    dout_user_update_[port_id].update_ = true;
    dout_user_update_[port_id].value_ = dout_topic->value;
    write_update_mutex_.unlock();
  }

  void DIO_ROSDriver::run(void)
  {
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate loop_rate(access_frequency_);

    dio_status din_status = din_accessor_.getStatus();
    dio_status dout_status = dout_accessor_.getStatus();
    dio_ros_driver::DIOStatus din_status_topic;
    dio_ros_driver::DIOStatus dout_status_topic;

    while (ros::ok())
    {
      // read data from DIN ports.
      if (din_status.status_ == 0)
        readDINPorts();
      // write data to DOUT ports.
      if (dout_status.status_ == 0)
        writeDOUTPorts();
      // publish din_status
      din_status = din_accessor_.getStatus();
      din_status_topic.raw_value = din_status.value_;
      din_status_topic.status = din_status.status_;
      din_status_publisher_.publish(din_status_topic);
      // publish dout_status
      dout_status = dout_accessor_.getStatus();
      dout_status_topic.raw_value = dout_status.value_;
      dout_status_topic.status = dout_status.status_;
      dout_status_publisher_.publish(dout_status_topic);

      loop_rate.sleep();
    }
  }

  void DIO_ROSDriver::readDINPorts(void)
  {
    int32_t read_value;
    dio_ros_driver::DIOPort din_port;
    for (uint32_t i = 0; i < MAX_PORT_NUM; i++)
    {
      read_value = din_accessor_.readPort(i);
      if (read_value < 0)
      {
        ROS_ERROR("DI read error %d", i);
      }
      else
      {
        din_port.value = static_cast<bool>(read_value);
        din_port_publisher_array_[i].publish(din_port);
      }
    }
  }

  void DIO_ROSDriver::writeDOUTPorts(void)
  {
    write_update_mutex_.lock();
    for (uint32_t i = 0; i < MAX_PORT_NUM; i++)
    {
      if (dout_user_update_[i].update_ == true)
      {
        int32_t ret_code = dout_accessor_.writePort(i, dout_user_update_[i].value_);
        if (ret_code == -1)
        {
          ROS_ERROR("DOUT write error %d", i);
        }
        dout_user_update_[i].update_ = false;
      }
    }
    write_update_mutex_.unlock();
  }

} // namespace dio_ros_driver
