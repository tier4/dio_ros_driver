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
#include <iostream>
#include <cstring>

#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include "dio_ros_driver/dio_user_handler.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "dio_ros_driver");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // set param
  // threshold to detect DI pressed.
  double pressing_period_threshold;
  pnh.param<double>("pressing_period_threshold",
                    pressing_period_threshold, 1.0);
  // chipname
  std::string chip_name;
  pnh.param<std::string>("chip_name",
                         chip_name, "gpiochip0");
  // line offset
  int line_offset;
  pnh.param<int>("line_offset",
                 line_offset, 0);

  bool di_active_low;
  pnh.param<bool>("di_active_low",
                  di_active_low, false);

  // set publisher
  // startbutton is directed to Autoware.
  ros::Publisher button_pub =
    nh.advertise<std_msgs::Bool>("startbutton", 10);
  // startbutton_raw is a topic for debugging.
  ros::Publisher button_raw_pub =
    pnh.advertise<std_msgs::Bool>("startbutton_raw", 10);

  if (!init_di(chip_name.c_str(),
               static_cast<unsigned int>(line_offset))) {
      std::cerr << "init_di cannot <-- ERROR\n" << std::endl;
      return -1;
  }

  // handling button pushed.
  constexpr double sleep_sec = 0.02;
  ros::Rate loop_rate(1.0 / sleep_sec);
  uint32_t pressed_count = 0;
  std_msgs::Bool startbutton_raw_msg;
  std_msgs::Bool startbutton_msg;
  while (ros::ok()) {
    const bool di_line_value = static_cast<bool>(read_di_line());
    const bool is_pressed = (di_active_low == true) ? !di_line_value
                                                     : di_line_value;
    pressed_count = (is_pressed) ? pressed_count + 1 : 0;
    const double pressing_period = pressed_count * sleep_sec;

    startbutton_raw_msg.data = di_line_value;
    startbutton_msg.data = is_pressed;

    // publish topics of button(DI) pushed.
    button_raw_pub.publish(startbutton_raw_msg);
    if (pressing_period > pressing_period_threshold) {
      button_pub.publish(startbutton_msg);
    }
    loop_rate.sleep();
  }

  return 0;
}

