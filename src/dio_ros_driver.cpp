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

#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include "dio_ros_driver/dio_ros_driver.hpp"
#include "dio_ros_driver/dio_user_handler.h"

namespace dio_ros_driver
{
  DIO_ROSDriver::DIO_ROSDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
      : nh_(nh),
        pnh_(pnh),
        pressing_period_threshold_(1.0),
        chip_name_("chipname0"),
        line_offset_(0),
        di_active_low_(false)
  {
    // load global parameter from NodeHandler.
    pnh.param<double>("pressing_period_threshold", pressing_period_threshold_, 1.0);
    pnh.param<std::string>("chip_name", chip_name_, "gpiochip0");
    pnh.param<int>("line_offset", line_offset_, 0);
    pnh.param<bool>("di_active_low", di_active_low_, false);

    // prepare publishers
    button_pub_ = nh_.advertise<std_msgs::Bool>("startbutton", 10);
    button_raw_pub_ = nh_.advertise<std_msgs::Bool>("startbutton_raw", 10);
  }

  int DIO_ROSDriver::initDIOAccess(void)
  {
    if (!init_di(chip_name_.c_str(), static_cast<uint32_t>(line_offset_)))
    {
      ROS_ERROR("initDIOAccess is failed.");
      return -1;
    }
    return 0;
  }

  void DIO_ROSDriver::run(void)
  {
    constexpr double sleep_sec = 0.02;
    ros::Rate loop_rate(1.0 / sleep_sec);
    double pressing_period = 0.0;
    std_msgs::Bool startbutton_raw_msg;
    std_msgs::Bool startbutton_msg;

    while (ros::ok())
    {
      const bool di_line_value = static_cast<bool>(read_di_line());
      const bool is_pressed = (di_active_low_ == true) ? !di_line_value
                                                       : di_line_value;

      if (is_pressed)
      {
        pressing_period += sleep_sec;
      }
      else
      {
        pressing_period = 0.0;
      }

      startbutton_raw_msg.data = di_line_value;
      startbutton_msg.data = is_pressed;

      // publish
      if (pressing_period > pressing_period_threshold_)
      {
        button_pub_.publish(startbutton_msg);
      }
      button_raw_pub_.publish(startbutton_raw_msg);
      loop_rate.sleep();
    }
  }

} // namespace dio_ros_driver
