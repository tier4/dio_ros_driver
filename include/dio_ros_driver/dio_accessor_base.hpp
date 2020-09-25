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
 * @file dio_accessor_base.hpp
 * @brief DIO Accessor base class
 * @author Takayuki AKAMINE
 */

#ifndef __DIO_ACCESSOR_BASE_HPP__
#define __DIO_ACCESSOR_BASE_HPP__

extern "C" {
#include <gpiod.h>
}
#include <cstdint>
#include <array>

namespace dio_ros_driver {
constexpr uint16_t MAX_PORT_NUM = 8;

typedef struct dio_port_descriptor {
  uint16_t port_offset_;
  gpiod_line *dio_line_;

  dio_port_descriptor() : port_offset_(), dio_line_() {}
  dio_port_descriptor(const uint16_t &port_offest, gpiod_line *dio_line) : port_offset_(port_offest), dio_line_(dio_line) {}
} dio_port_descriptor;

typedef struct dio_status {
  uint16_t status_;
  uint16_t user_value_;
  dio_status() : status_(0), user_value_(0) {}
  dio_status(const uint16_t &status, const uint16_t &user_value) : status_(status), user_value_(user_value) {}
} dio_status;

enum ERROR_CODE {
  NO_DIO_ERROR,
  NOT_OPEN_DIOCHIP,
  NOT_GET_LINE,
  NOT_SET_LINE_DIRECTION,
  NOT_GET_LINE_VALUE,
  NOT_SET_LINE_VALUE
};

class DIO_AccessorBase {
 public:
  bool addPort(const uint16_t &port_offset);                                       // !<@brief add port to access
  uint32_t getNumOfPorts(void);                                                    // !<@brief getter of port to access
  int32_t readPort(const uint16_t &port_id);                                       // !<@brief read single DI/DO port
  virtual int32_t writePort(const uint16_t &port_id, const bool &port_value) = 0;  // !<@brief abstract method of writing port
  dio_status getStatus(void);                                                      // !<@brief get accessor status and values from all ports
  void releaseAllPorts(void);                                                      // !<@brief release all of registered ports

 protected:
  DIO_AccessorBase(void);                                                                     // !<@brief Constructor of DIO Accessor
  virtual void initialize(gpiod_chip *const dio_chip_descriptor, const bool &value_inverse);  // !<@brief initialize for handling DIO port
  void setErrorCode(const uint16_t &port_id, const ERROR_CODE &error_code);                   // !<@brief set error code to send user application.
  virtual int32_t setDirection(const dio_port_descriptor &port) = 0;                          // !<@brief abstract method of set direction

  gpiod_chip *dio_chip_descriptor_;                             // !<@brief chip descriptor
  std::array<dio_port_descriptor, MAX_PORT_NUM> dio_ports_set;  // !<@brief port list to access
  uint32_t dio_port_num_;                                       // !<@brief the number of ports to access
  dio_status dio_status_;                                       // !<@brief ports status
  bool value_inverse_;                                          // !<@brief inverse option
};
}  // namespace dio_ros_driver

#endif
