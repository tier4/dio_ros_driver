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
 * File Name: din_accessor.cpp
 * Author: Takayuki AKAMINE
 * Description: Body fine for din accessor
 */

#include "dio_ros_driver/din_accessor.hpp"
#include <iostream>

namespace dio_ros_driver
{

  /**
   * @brief Constructor of DIN Accessor 
   * call super class's constructor.
   */
  DINAccessor::DINAccessor(void) : DIO_AccessorBase()
  {
  }

  void DINAccessor::initialize(gpiod_chip *const dio_chip_descriptor, const bool &din_value_inverse)
  {
    DIO_AccessorBase::initialize(dio_chip_descriptor, din_value_inverse);
  }

  int32_t DINAccessor::writePort(const uint16_t &port_id, const bool &port_value)
  {
    std::cerr << "cannot write din port: " << port_id << std::endl;
    return 0;
  }

  int32_t DINAccessor::setDirection(const dio_port_descriptor &port)
  {
    std::string port_name = "/dio/din" + std::to_string(port.port_offset_);
    return gpiod_line_request_input(port.dio_line_, port_name.c_str());
  }

} // namespace dio_ros_driver