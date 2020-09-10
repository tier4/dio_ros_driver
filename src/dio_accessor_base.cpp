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
 * File Name: dio_accessor_base.hpp
 * Author: Takayuki AKAMINE
 * Description: Header file for dio_ros_driver
 */

#include "dio_ros_driver/dio_accessor_base.hpp"
#include <iostream>
#include <ros/ros.h>

namespace dio_ros_driver
{
  DIO_AccessorBase::DIO_AccessorBase(gpiod_chip *const dio_chip_descriptor)
      : dio_chip_descriptor_(dio_chip_descriptor),
        dio_port_num_(0)
  {
    if (dio_chip_descriptor == NULL)
    {
      dio_status_.status_ = static_cast<uint16_t>(0x00FF);
      dio_status_.status_ |= static_cast<uint16_t>(NOT_OPEN_DIOCHIP << 8);
    }
    else
    {
      dio_status_.status_ = (static_cast<uint16_t>(NO_DIO_ERROR) << 8) | 0x0000;
    }
  }

  void DIO_AccessorBase::setErrorCode(const uint16_t &port_id, const ERROR_CODE &error_code)
  {
    // set error port
    dio_status_.status_ = static_cast<uint16_t>(0x0001 << port_id);
    // set error code.
    dio_status_.status_ |= static_cast<uint16_t>(error_code << 8);
    return;
  }

  bool DIO_AccessorBase::addPort(const uint16_t &port_id, const uint16_t &port_offset)
  {
    // check status.
    if (dio_status_.status_ != 0)
    {
      return 1;
    }
    // check index value.
    if (port_id >= MAX_PORT_NUM)
    {
      std::cerr << "[dio_ros_driver] addPort error due to illegal index access." << std::endl;
      return 1;
    }
    // get line descriptor from gpiochip
    dio_ports_set[port_id].port_offset_ = port_offset;
    dio_ports_set[port_id].dio_line_ = gpiod_chip_get_line(dio_chip_descriptor_, port_offset);
    if (dio_ports_set[port_id].dio_line_ == NULL)
    {
      std::cerr << "[dio_ros_driver] cannot get line #" << port_offset << std::endl;
      setErrorCode(port_id, NOT_GET_LINE);
      return 1;
    }
    // set direction to descriptor
    if (setDirection(dio_ports_set[port_id]) != 0)
    {
      std::cerr << "[dio_ros_driver] cannot set direction #" << port_offset << std::endl;
      setErrorCode(port_id, NOT_SET_LINE_DIRECTION);
      return 1;
    }
    // add line to bulk;
    dio_port_num_++;

    return 0;
  }

  int32_t DIO_AccessorBase::readPort(const uint16_t &port_id)
  {
    int32_t din_value;
    din_value = gpiod_line_get_value(dio_ports_set[port_id].dio_line_);
    if (din_value < 0)
    {
      setErrorCode(port_id, NOT_GET_LINE_VALUE);
    }
    return din_value;
  }

  dio_status DIO_AccessorBase::getStatus(void)
  {
    // check status
    if (dio_status_.status_ != 0 ||
        dio_port_num_ == 0)
    {
      return dio_status_;
    }

    // get all value.
    dio_status_.value_ = 0;
    int32_t read_value;
    for (uint32_t i = 0; i < dio_port_num_; i++)
    {
      read_value = readPort(i);
      if (read_value < 0)
      {
        std::cerr << "[dio_ros_driver] failed in reading all ports.";
      }
      dio_status_.value_ |= (static_cast<uint16_t>(read_value) << i);
    }
    return dio_status_;
  }

} // namespace dio_ros_driver