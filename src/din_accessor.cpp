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
   * @brief initialize for handling DI ports
   * Set DIO chip descriptor and inverse option for accessing DO port.
   * @param[in] dio_chip_descriptor DIO chip descriptor
   * @param[in] din_value_inverse inverse boolean value option
   */
  void DINAccessor::initialize(gpiod_chip *const dio_chip_descriptor, const bool &din_value_inverse) {
    DIO_AccessorBase::initialize(dio_chip_descriptor, din_value_inverse);
  }

  /**
   * @brief warn that this accessor for DI port.
   * only warn that this is DI port accessor.
   * @param[in] port_id     to specify the target port, but unused.
   * @param[in] port_value  to give value
   * @retval 0 always return 0 because it does not influences on the system.
   */
  int32_t DINAccessor::writePort(const uint16_t &port_id, const bool &port_value) {
    std::cerr << "cannot write din port: " << port_id << std::endl;
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
