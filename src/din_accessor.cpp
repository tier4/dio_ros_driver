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
 * @file din_accessor.cpp
 * @brief DIN Accessor class
 * @author Takayuki AKAMINE
 * 
 */

#include "dio_ros_driver/din_accessor.hpp"
#include <iostream>

namespace dio_ros_driver {

  /**
   * @brief Constructor of DIN Accessor 
   * call super class's constructor.
   */
  DINAccessor::DINAccessor(void) : DIO_AccessorBase() {}

  /**
   * @brief warn that this accessor for DI port.
   * only warn that this is DI port accessor.
   * @param[in] port_id     to specify the target port, but unused.
   * @param[in] port_value  to give value
   * @retval 0 always return 0 because it does not influences on the system.
   */
  int32_t DINAccessor::writePort(const uint16_t &port_id, const bool &port_value) {
    if (port_id >= dio_port_num_) {
      setAccessorStatus(ERROR_ACCESSOR_ILLEGAL_PORT_ACCESS);
      return -1;
    }
    setAccessorStatus(WARN_PORT_SETTING_VALU_ON_DIN_VALUE);
    setPortStatus(port_id, WARNING_WRITE_VALUE_TO_DIN_PORT);

    return 0;
  }

  /**
   * @brief set direction
   * set input direction for DI port.
   * @param[in] port targeted DI port.
   */
  int32_t DINAccessor::setDirection(const dio_port_descriptor &port) {
    std::string port_name = "/dio/din" + std::to_string(port.port_offset_);
    return gpiod_line_request_input(port.dio_line_, port_name.c_str());
  }

}  // namespace dio_ros_driver
