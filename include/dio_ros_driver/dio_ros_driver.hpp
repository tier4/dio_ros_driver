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
 * Package: dio_ros_driver
 * File Name: dio_driver.hpp
 * Author: Takayuki AKAMINE
 * Description: Header file for dio_ros_driver
 */

#ifndef __DIO_ROS_DRIVER_HPP__
#define __DIO_ROS_DRIVER_HPP__

#include <ros/ros.h>

namespace dio_ros_driver
{
  class DIO_ROSDriver
  {
  public:
    DIO_ROSDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
    ~DIO_ROSDriver()
    {
    }

    int initDIOAccess(); // !<@brief DIO Accessor Initialization
    void run();

  private:
    // callbacks
    void publishButtonStatus();

    // Publisher
    ros::NodeHandle nh_;            //!< @brief ros node handle
    ros::NodeHandle pnh_;           //!< @brief ros node handle
    ros::Publisher button_pub_;     //!< @brief ros publisher
    ros::Publisher button_raw_pub_; //!< @brief ros publisher

    // Variables.
    double pressing_period_threshold_; //!< @brief pressing period.
    std::string chip_name_;            // !<@brief DIO Chip Name
    int line_offset_;                  // !<@brief Line offset
    bool di_active_low_;               // !<@brief DI Active Low Enabler.
  };
} // namespace dio_ros_driver

#endif
