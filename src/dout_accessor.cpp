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
  /**
   * @brief DOUT Constructor
   * Execute super class's constructor 
   * to initialize member variables
   */
  DOUTAccessor::DOUTAccessor() : DIO_AccessorBase()
  {
  }

  /**
   * @brief DOUT Initializer method
   * Set DIO chip descriptor and specify the initial
   * value of DO ports.
   * @param dio_chip_descriptor[in] DIO chip descriptor
   * @param dout_value_inverse[in]  Convert boolean value
   * @param dout_default_value[in]  Initial value of DO ports
   */
  void DOUTAccessor::initialize(gpiod_chip *const dio_chip_descriptor, const bool &dout_value_inverse, const bool &dout_default_value)
  {
    DIO_AccessorBase::initialize(dio_chip_descriptor, dout_value_inverse);
    dout_default_value_ = dout_default_value;
  }

  /**
   * @brief write given value to targeted port.
   * main function of DOUTAccessor
   * @param[in] port_id     to specify the targeted port.
   * @param[in] port_value  to give value.
   * @retval 0 success in writing value to port.
   * @retval -1 failed in writing value to port.
   */
  int32_t DOUTAccessor::writePort(const uint16_t &port_id, const bool &port_value)
  {
    const bool writing_value = value_inverse_ ^ port_value;

    if (gpiod_line_set_value(dio_ports_set.at(port_id).dio_line_, static_cast<int32_t>(writing_value)) != 0)
    {
      setErrorCode((0x0001 << port_id), NOT_SET_LINE_VALUE);
      return -1;
    }
    return 0;
  }

  /**
   * @brief reset all ports
   * reset all ports by default value of DO ports.
   * @retval 0 success in resetting
   * @retval -1 failed in resetting
   */
  int32_t DOUTAccessor::resetAllPorts(void)
  {
    for (uint32_t i = 0; i < getNumOfPorts(); i++)
    {
      if (writePort(static_cast<uint16_t>(i), dout_default_value_) == -1)
      {
        return -1;
      }
    }
    return 0;
  }

  /**
   * @brief set direction
   * set output direction for DO ports.
   * @param[in] port targeted DO port.
   */
  int32_t DOUTAccessor::setDirection(const dio_port_descriptor &port)
  {
    const bool initial_value = value_inverse_ ^ dout_default_value_;
    std::string port_name = "/dio/dout" + std::to_string(port.port_offset_);
    return gpiod_line_request_output(port.dio_line_, port_name.c_str(), static_cast<int32_t>(initial_value));
  }

} // namespace dio_ros_driver