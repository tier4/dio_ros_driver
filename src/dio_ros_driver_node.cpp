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

#include "ros/ros.h"

#include "dio_ros_driver/dio_ros_driver.hpp"
#include <csignal>

// for terminate signal processing.
static std::shared_ptr<dio_ros_driver::DIO_ROSDriver> dio_ros_driver_ptr;
extern "C" void terminate_handler(int signum) { dio_ros_driver_ptr->terminate(signum); }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dio_ros_driver");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::shared_ptr<dio_ros_driver::DIO_ROSDriver> dio_ros_driver;
  dio_ros_driver = std::make_shared<dio_ros_driver::DIO_ROSDriver>(nh, pnh);
  dio_ros_driver_ptr = dio_ros_driver;
  signal(SIGTERM, terminate_handler);

  dio_ros_driver->init();

  dio_ros_driver->run();

  return 0;
}
