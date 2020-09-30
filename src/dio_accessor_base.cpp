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
        dio_ports_set_(),
        accessor_status_() {
  }

  /**
   * @brief initialize for handling DIO port
   * set chip descriptor and inverse option.
   * @param[in] dio_chip_descriptor dio chip descriptor
   * @param[in] value_inverse inverse enabler option
   */
  void DIO_AccessorBase::initialize(gpiod_chip *const dio_chip_descriptor, const bool &value_inverse) {
    dio_chip_descriptor_ = dio_chip_descriptor;
    value_inverse_ = value_inverse;
    if (dio_chip_descriptor_ == nullptr) {
      setAccessorStatus(ERROR_ACCESSOR_NOT_OPEN_CHIP);
    } else {
      setAccessorStatus(WARN_ACCESSOR_NO_PORT_INITIALIZED);
    }
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
    // check DIO chip is opened
    if (dio_chip_descriptor_ == nullptr) {
      setAccessorStatus(ERROR_ACCESSOR_NOT_OPEN_CHIP);
      return 1;
    }

    // check index value in range.
    if (dio_port_num_ == MAX_PORT_NUM) {
      setAccessorStatus(ERROR_ACCESSOR_NOT_ADD_PORT_TO_MAX);
      return 1;
    }

    // get line descriptor from gpiochip.
    dio_port_descriptor &dio_port = dio_ports_set_.at(dio_port_num_);
    dio_port.port_offset_ = port_offset;
    dio_port.dio_line_ = gpiod_chip_get_line(dio_chip_descriptor_, port_offset);

    // status check
    if (dio_port.dio_line_ == nullptr) {
      setPortStatus(dio_port_num_, ERROR_FAILED_GETTING_PORT_AS_LINE);
      setAccessorStatus(ERROR_PORT_FAILED_GETTING_PORT_AS_LINE);
      return 1;
    }

    // set direction to descriptor.
    if (setDirection(dio_port) != 0) {
      setPortStatus(dio_port_num_, ERROR_FAILED_SETTING_PORT_DIRECTION);
      setAccessorStatus(ERROR_PORT_FAILED_SETTING_PORT_DIRECTION);
      return 1;
    }

    // succeeded and increment registered port number.
    setPortStatus(dio_port_num_, PORT_RUNNING);
    setAccessorStatus(NORMAL_OPERATION);
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
   * @brief getter of port's offset on GPIO chip
   * @retval ports's offset (if error occurs, 0xFFFF)
   */
  uint16_t DIO_AccessorBase::getPortOffset(const uint16_t& port_id) {
    try {
      return dio_ports_set_.at(port_id).port_offset_;
    }
    catch (std::out_of_range excep_oor)
    {
      setAccessorStatus(ERROR_ACCESSOR_ILLEGAL_PORT_ACCESS);
      return 0xFFFF;
    }
  }

  /**
   * @brief read single DI/DO port
   * @param[in] port_id DI/DO port ID(0-7)
   * @retval -1 failure in reading port value
   * @retval others read value from selected port
   */
  int32_t DIO_AccessorBase::readPort(const uint16_t &port_id) {
    int32_t read_value;
    if (port_id >= dio_port_num_) {
      setAccessorStatus(ERROR_ACCESSOR_ILLEGAL_PORT_ACCESS);
      return -1;
    }

    // read value from selected DI/DO port.
    dio_port_descriptor dio_port = dio_ports_set_.at(port_id);
    read_value = gpiod_line_get_value(dio_port.dio_line_);
    if (read_value < 0) {
      setPortStatus(port_id, ERROR_FAILED_GETTING_VALUE_FROM_PORT);
      setAccessorStatus(ERROR_PORT_FAILED_GETTING_VALUE_FROM_PORT);
    }
    read_value = (static_cast<int>(value_inverse_) ^ read_value) & 0x0001;

    return read_value;
  }

  /**
   * @brief get accessor status and values from all ports
   * @retval accessor_status_
   */
  accessor_status DIO_AccessorBase::getAccessorStatus(void) {
    // get all value.
    accessor_status_.user_value_ = 0;
    for (uint32_t i = 0; i < dio_port_num_; i++) {
      int32_t read_value = readPort(i);
      if (read_value < 0) {
        setAccessorStatus(ERROR_ACCESSOR_FAILED_IN_GETTING_ALL_VALUE);
      }
      accessor_status_.user_value_ |= (static_cast<uint16_t>(read_value) << i);
    }
    return accessor_status_;
  }

  /**
   * @brief getter of port's status
   * @param[in] port_id port_id [0 - (MAX_PORT_NUM-1)]
   * @retval port status
   */
  uint16_t DIO_AccessorBase::getPortStatus(const uint16_t& port_id) {
    try {
      return dio_ports_set_.at(port_id).status_;
    }
    catch (std::out_of_range excep_oor)
    {
      setAccessorStatus(ERROR_ACCESSOR_ILLEGAL_PORT_ACCESS);
      return ERROR_UNDEFINED_PORT_ACCESSED;
    }
  }

  /**
   * @brief setter of accessor's status.
   * @param[in] status setting status value as uint16_t
   */
  void DIO_AccessorBase::setAccessorStatus(const uint16_t &status) {

    const uint16_t current_status_code = accessor_status_.status_;
    if ((current_status_code & 0x1000) == 0x1000) {
      // not update Error status once status is changed into error.
      return;
    }
    accessor_status_.status_ = status;
    return;
  }

  /**
   * @brief setter of port's status.
   * @param[in] port_id port id [0 - (MAX_PORT_NUM-1)]
   * @param[in] status setting status value as uint16_t
   */
  void DIO_AccessorBase::setPortStatus(const uint16_t &port_id, const uint16_t &status) {
    try {
      const uint16_t current_status_code = dio_ports_set_.at(port_id).status_;
      if ((current_status_code & 0x1000) == 0x1000) {
        // not update error status once status is changed into error.
        return;
      }
      dio_ports_set_.at(port_id).status_ = status;
    }
    catch (std::out_of_range excep_oor)
    {
      accessor_status_.status_ = ERROR_ACCESSOR_FAILED_IN_RESETTING_DOUT_PORT;
      return;
    }
    return;
  }

  /**
   * @brief release all of registered ports
   * @retval no return value
   */
  void DIO_AccessorBase::releaseAllPorts(void) {

    for (uint32_t i = 0; i < dio_port_num_; i++) {
      dio_port_descriptor &dio_port = dio_ports_set_.at(i);
      gpiod_line_release(dio_port.dio_line_);
    }
    dio_port_num_ = 0;
    return;
  }


}  // namespace dio_ros_driver
