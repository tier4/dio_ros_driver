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
 * File Name: dio_accessor_base.cpp
 * Author: Takayuki AKAMINE
 * Description: Header file for dio_ros_driver
 */

#ifndef __DIO_ACCESSOR_BASE_HPP__
#define __DIO_ACCESSOR_BASE_HPP__

extern "C"
{
#include <gpiod.h>
}
#include <cstdint>
#include <array>

namespace dio_ros_driver
{
  constexpr uint16_t MAX_PORT_NUM = 8;

  typedef struct dio_port_descriptor
  {
    uint16_t port_offset_;
    gpiod_line *dio_line_;

    dio_port_descriptor() : port_offset_(), dio_line_() {}
    dio_port_descriptor(const uint16_t &port_offest, gpiod_line *dio_line) : port_offset_(port_offest), dio_line_(dio_line) {}
  } dio_port_descriptor;

  typedef struct dio_status
  {
    uint16_t status_;
    uint16_t value_;
    dio_status() : status_(0), value_(0) {}
    dio_status(const uint16_t &status, const uint16_t &value) : status_(status), value_(value) {}
  } dio_status;

  enum ERROR_CODE
  {
    NO_DIO_ERROR,
    NOT_OPEN_DIOCHIP,
    NOT_GET_LINE,
    NOT_SET_LINE_DIRECTION,
    NOT_GET_LINE_VALUE,
    NOT_SET_LINE_VALUE
  };

  class DIO_AccessorBase
  {
  public:
    bool addPort(const uint16_t &port_offset);
    uint32_t getNumOfPorts(void);
    virtual int32_t readPort(const uint16_t &port_id);
    virtual int32_t writePort(const uint16_t &port_id, const bool &port_value) = 0;
    dio_status getStatus(void);

  protected:
    DIO_AccessorBase(void);
    virtual void initialize(gpiod_chip *const dio_chip_descriptor);
    void setErrorCode(const uint16_t &port_id, const ERROR_CODE &error_code);
    virtual int32_t setDirection(const dio_port_descriptor &port) = 0;

    gpiod_chip *dio_chip_descriptor_;
    std::array<dio_port_descriptor, MAX_PORT_NUM> dio_ports_set;
    uint32_t dio_port_num_;
    dio_status dio_status_;
  };
} // namespace dio_ros_driver

#endif