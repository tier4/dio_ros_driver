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
 * @file dio_accessor_base.cpp
 * @brief DIO Accessor base class
 * @author Takayuki AKAMINE
 */

#include <ros/ros.h>
#include <iostream>
#include "dio_ros_driver/dio_accessor_base.hpp"

namespace dio_ros_driver {

  /**
   * @brief Constructor of DIO Accessor
   * Set initial value on each member
   */
  DIO_AccessorBase::DIO_AccessorBase(void)
      : dio_chip_descriptor_(nullptr),
        dio_port_num_(0),
        dio_ports_set(),
        dio_status_() {
    setErrorCode(0xFFFF, NOT_OPEN_DIOCHIP);
  }

  /**
   * @brief initialize for handling DIO port
   * set chip descriptor and inverse option.
   * @param[in] dio_chip_descriptor dio chip descriptor
   * @param[in] value_inverse inverse enabler option
   */
  void DIO_AccessorBase::initialize(gpiod_chip *const dio_chip_descriptor, const bool &value_inverse) {
    if (dio_chip_descriptor != nullptr)
      setErrorCode(0x0000, NO_DIO_ERROR);
    dio_chip_descriptor_ = dio_chip_descriptor;
    value_inverse_ = value_inverse;
    return;
  }

  /**
   * @brief add port to access
   * register port to access as gpio line.
   * @param[in] port_offset
   * @retval 0 success
   * @retval 1 failed in adding port due to error
   */
  bool DIO_AccessorBase::addPort(const uint16_t &port_offset) {
    // check status.
    if (dio_status_.status_ != 0) {
      return 1;
    }
    // check index value.
    if (dio_port_num_ >= MAX_PORT_NUM) {
      std::cerr << "[dio_ros_driver] addPort error due to illegal index access." << std::endl;
      return 1;
    }
    // get line descriptor from gpiochip.
    dio_port_descriptor &dio_port = dio_ports_set.at(dio_port_num_);
    dio_port.port_offset_ = port_offset;
    dio_port.dio_line_ = gpiod_chip_get_line(dio_chip_descriptor_, port_offset);
    if (dio_port.dio_line_ == nullptr) {
      std::cerr << "[dio_ros_driver] cannot get line #" << port_offset << std::endl;
      setErrorCode((0x0001 << dio_port_num_), NOT_GET_LINE);
      return 1;
    }
    // set direction to descriptor.
    if (setDirection(dio_port) != 0) {
      std::cerr << "[dio_ros_driver] cannot set direction #" << port_offset << std::endl;
      setErrorCode((0x0001 << dio_port_num_), NOT_SET_LINE_DIRECTION);
      return 1;
    }
    // succeeded and increment registered port number.
    dio_port_num_++;

    return 0;
  }

  /**
   * @brief getter of port to access
   * @retval the number of registered port
   */
  uint32_t DIO_AccessorBase::getNumOfPorts(void) {
    return dio_port_num_;
  }

  /**
   * @brief read single DI/DO port
   * @param[in] port_id DI/DO port ID(0-7)
   * @retval -1 failure in reading port value
   * @retval others read value from selected port
   */
  int32_t DIO_AccessorBase::readPort(const uint16_t &port_id) {
    int32_t read_value;
    read_value = gpiod_line_get_value(dio_ports_set.at(port_id).dio_line_);
    if (read_value < 0) {
      setErrorCode((0x0001 << port_id), NOT_GET_LINE_VALUE);
    }
    read_value = (static_cast<int>(value_inverse_) ^ read_value) & 0x0001;

    return read_value;
  }

  /**
   * @brief get accessor status and values from all ports
   * @return dio_status_
   */
  dio_status DIO_AccessorBase::getStatus(void) {
    // check status
    if (dio_status_.status_ != 0 ||
        dio_port_num_ == 0) {
      return dio_status_;
    }

    // get all value.
    dio_status_.user_value_ = 0;
    for (uint32_t i = 0; i < dio_port_num_; i++) {
      int32_t read_value = readPort(i);
      if (read_value < 0) {
        std::cerr << "[dio_ros_driver] failed in reading all ports.";
      }
      dio_status_.user_value_ |= (static_cast<uint16_t>(read_value) << i);
    }
    return dio_status_;
  }

  /**
   * @brief release all of registered ports
   * @retval no return value
   */
  void DIO_AccessorBase::releaseAllPorts(void) {
    if (dio_port_num_ == 0) {
      return;
    }
    for (uint32_t i = 0; i < dio_port_num_; i++) {
      dio_port_descriptor &dio_port = dio_ports_set.at(i);
      gpiod_line_release(dio_port.dio_line_);
    }
    dio_port_num_ = 0;
    return;
  }

  /**
   * @brief set error code to send user application.
   * @param[in] port_bitmap bitmap data indicating failed port
   * @param[in] error_code error code indicating failure mode
   */
  void DIO_AccessorBase::setErrorCode(const uint16_t &port_bitmap, const ERROR_CODE &error_code) {
    // set error port
    dio_status_.status_ = port_bitmap;
    // set error code.
    dio_status_.status_ |= static_cast<uint16_t>(error_code << 8);
    return;
  }

}  // namespace dio_ros_driver
