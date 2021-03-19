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

/**
 * @package dio_ros_driver
 * @file dio_driver.hpp
 * @brief DIO ROS Driver class
 * @author Takayuki AKAMINE
 */

#ifndef __DIO_ROS_DRIVER_HPP__
#define __DIO_ROS_DRIVER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "dio_ros_driver/msg/dio_port.hpp"

#include <string>
#include <cstdint>
#include <array>
#include <vector>
#include <mutex>
#include <memory>

#include "dio_ros_driver/din_accessor.hpp"
#include "dio_ros_driver/dout_accessor.hpp"
#include "dio_ros_driver/dio_diagnostic_updater.hpp"

namespace dio_ros_driver {
typedef struct dout_update {
  bool update_;
  bool value_;
  dout_update() : update_(false), value_() {}
  dout_update(const bool &update_, const bool &value_);
} dout_update;  // !<@brief indicator of update and value for updating port

class DIO_ROSDriver : public rclcpp::Node {
 public:
  DIO_ROSDriver(const std::string & node_name, const rclcpp::NodeOptions & options);  // !<@brief Constructor
  ~DIO_ROSDriver() {}                                                    // !<@brief Destructor

  int init(void);                 // !<@brief DIO Accessor Initialization.
  void update(void);                 // !<@brief main routine of this node.
  void terminate(int singal_id);  // !<@brief terminate processing.

 private:
  // callbacks
  void addAccessorPorts(const std::vector<uint32_t> offset_array,
      std::shared_ptr<DIO_AccessorBase> dio_accessor);           // !<@brief Add ports to given accessor.
  void receiveWriteRequest(const dio_ros_driver::msg::DIOPort::SharedPtr &dout_topic,
         const uint32_t &port_id);                               // !<@brief receive user write request.
  void readDINPorts(void);                                                         // !<@brief read all DI port and send them as topics
  void writeDOUTPorts(void);                                                       // !<@brief DO ports by value according to received request

  // Publisher and subscribers.
  std::array<rclcpp::Publisher<dio_ros_driver::msg::DIOPort>::SharedPtr, MAX_PORT_NUM> din_port_publisher_array_;     // !<@brief ros publishers array for DIN ports
  std::array<rclcpp::Subscription<dio_ros_driver::msg::DIOPort>::SharedPtr, MAX_PORT_NUM> dout_port_subscriber_array_;  // !<@brief ros subscribers array for DOUT ports

  // Timer callback
  rclcpp::TimerBase::SharedPtr dio_update_timer_;  // !<@brief Timer for DIO update.

  // Access handler.
  std::shared_ptr<DINAccessor> din_accessor_;    // !<@brief DIN Accessor.
  std::shared_ptr<DOUTAccessor> dout_accessor_;  // !<@brief DOUT Accessor.

  // Diagnostic updater
  std::shared_ptr<DIO_DiagnosticUpdater> dio_diag_updater_;  // !<@brief DIO's diagnostic updater.

  // Variables for parametr.
  uint32_t access_frequency_;  // !<@brief pressing period.
  std::string chip_name_;    // !<@brief DIO Chip Name
  std::vector<uint32_t> din_port_offset_;
  bool din_value_inverse_;   // !<@brief DIN value inverse enabler.
  std::vector<uint32_t> dout_port_offset_;
  bool dout_value_inverse_;  // !<@brief DOUT value inverse enabler.
  bool dout_default_value_;  // !<@brief DOUT defaule value

  // variable for sharing between callbacks
  std::mutex write_update_mutex_;                           // !<@brief mutex.
  std::array<dout_update, MAX_PORT_NUM> dout_user_update_;  // !<@brief update list.
  gpiod_chip *dio_chip_;                                    // !<@brief chip descriptor
};
}  // namespace dio_ros_driver

#endif
