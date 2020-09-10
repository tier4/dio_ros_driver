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
 * Description: Body fine for dout accessor
 */

#include "dio_ros_driver/dout_accessor.hpp"
#include <iostream>

namespace dio_ros_driver
{
  DOUTAccessor::DOUTAccessor(gpiod_chip *const dio_chip_descriptor) : DIO_AccessorBase(dio_chip_descriptor)
  {
  }

  int32_t DOUTAccessor::writePort(const uint16_t &port_id, const bool &port_value)
  {

    if (gpiod_line_set_value(dio_ports_set[port_id].dio_line_, (int)port_value) != 0)
    {
      setErrorCode(port_id, NOT_SET_LINE_VALUE);
      return -1;
    }
    return 0;
  }

  int32_t DOUTAccessor::setDirection(const dio_port_descriptor &port)
  {
    std::string port_name = "/dio/dout" + std::to_string(port.port_offset_);
    return gpiod_line_request_output(port.dio_line_, port_name.c_str(), 0);
  }

} // namespace dio_ros_driver