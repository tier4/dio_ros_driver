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
#include <cstdint>
#include <array>
#include <vector>
#include <mutex>

#include "din_accessor.hpp"
#include "dout_accessor.hpp"
#include <dio_ros_driver/DIOPort.h>

namespace dio_ros_driver
{
  typedef struct dout_update
  {
    bool update_;
    bool value_;
    dout_update() : update_(false), value_() {}
    dout_update(const bool &update_, const bool &value_);
  } dout_update;

  class DIO_ROSDriver
  {
  public:
    DIO_ROSDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
    ~DIO_ROSDriver()
    {
    }

    int init(void); // !<@brief DIO Accessor Initialization
    void run(void);

  private:
    // callbacks
    void readDINPorts(void);
    void writeDOUTPorts(void);
    void requestUserWrite(const dio_ros_driver::DIOPort::ConstPtr &dout_topic, const uint32_t &port_id);

    // Node handler
    ros::NodeHandle nh_;  //!< @brief ros node handle
    ros::NodeHandle pnh_; //!< @brief ros node handle

    // Publisher and subscribers.
    std::array<ros::Publisher, MAX_PORT_NUM> din_port_publisher_array_;    //!< @brief ros publisher array
    std::array<ros::Subscriber, MAX_PORT_NUM> dout_port_subscriber_array_; //!< @brief ros publisher array
    ros::Publisher din_status_publisher_;
    ros::Publisher dout_status_publisher_;

    // Access handler
    DINAccessor *din_accessor_;
    DOUTAccessor *dout_accessor_;

    // Variables for parametr.
    double access_frequency_; //!< @brief pressing period.
    std::string chip_name_;   // !<@brief DIO Chip Name
    bool active_low_;         // !<@brief Active Low Enabler.

    // variable for sharing between callbacks
    std::mutex write_update_mutex_;
    std::array<dout_update, MAX_PORT_NUM> dout_user_update_;
    gpiod_chip *dio_chip_;

    std::vector<int32_t> din_offset_array_;
    std::vector<int32_t> dout_offset_array_;
  };
} // namespace dio_ros_driver

#endif
