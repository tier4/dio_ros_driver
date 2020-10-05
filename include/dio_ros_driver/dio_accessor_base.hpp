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


enum ACCESSOR_STATUS_CODE {
  NORMAL_OPERATION                              = 0x0000,
  WARN_ACCESSOR_NO_PORT_INITIALIZED             = 0x0100,
  WARN_PORT_SETTING_VALU_ON_DIN_VALUE           = 0x0101,
  ERROR_ACCESSOR_NOT_OPEN_CHIP                  = 0x1000,
  ERROR_ACCESSOR_NOT_ADD_PORT_TO_MAX            = 0x1001,
  ERROR_ACCESSOR_ILLEGAL_PORT_ACCESS            = 0x1002,
  ERROR_ACCESSOR_FAILED_IN_GETTING_ALL_VALUE    = 0x1003,
  ERROR_ACCESSOR_FAILED_IN_RESETTING_DOUT_PORT  = 0x1004,
  ERROR_PORT_FAILED_GETTING_PORT_AS_LINE        = 0x1005,
  ERROR_PORT_FAILED_SETTING_PORT_DIRECTION      = 0x1006,
  ERROR_PORT_FAILED_GETTING_VALUE_FROM_PORT     = 0x1007,
  ERROR_PORT_FAILED_SETTING_VALUE_TO_PORT       = 0x1008,
};  // !<@brief status code of accessor.

typedef struct accessor_status {
  uint16_t status_;
  uint16_t user_value_;
  accessor_status() : status_(WARN_ACCESSOR_NO_PORT_INITIALIZED), user_value_(0) {}
  accessor_status(const uint16_t &status, const uint16_t &user_value) : status_(status), user_value_(user_value) {}
} accessor_status;  // !<@brief accessor(module-level) status

enum PORT_STATUS {
  NOT_PORT_INITIALIZED                   = 0x0000,
  PORT_RUNNING                           = 0x0001,
  WARNING_WRITE_VALUE_TO_DIN_PORT        = 0x0100,
  ERROR_FAILED_GETTING_PORT_AS_LINE      = 0x1000,
  ERROR_FAILED_SETTING_PORT_DIRECTION    = 0x1001,
  ERROR_FAILED_GETTING_VALUE_FROM_PORT   = 0x1002,
  ERROR_FAILED_SETTING_VALUE_TO_PORT     = 0x1003,
  ERROR_UNDEFINED_PORT_ACCESSED          = 0x1004,
};  // !<@brief port-level status code.

typedef struct dio_port_descriptor {
  uint16_t port_offset_;
  uint16_t status_;
  gpiod_line *dio_line_;

  dio_port_descriptor() : port_offset_(0xFFFF), status_(NOT_PORT_INITIALIZED), dio_line_() {}
  dio_port_descriptor(const uint16_t &port_offest, const uint16_t &status, gpiod_line *dio_line) : port_offset_(port_offest), status_(status), dio_line_(dio_line) {}
} dio_port_descriptor;  // !<@brief port descriptor and status


class DIO_AccessorBase {
 public:
  virtual void initialize(gpiod_chip *const dio_chip_descriptor,
        const bool &value_inverse);              // !<@brief initialize for handling DIO port
  bool addPort(const uint16_t &port_offset);                       // !<@brief add port to access
  uint32_t getNumOfPorts(void);                                    // !<@brief getter of port to access
  uint16_t getPortOffset(const uint16_t &port_id);                 // !<brief getter of port offset
  int32_t readPort(const uint16_t &port_id);                       // !<@brief read single DI/DO port
  virtual int32_t writePort(const uint16_t &port_id,
                            const bool &port_value) = 0;           // !<@brief abstract method of writing port
  accessor_status getAccessorStatus(void);                         // !<@brief get accessor status and values from all ports
  uint16_t getPortStatus(const uint16_t &port_id);                 // !<@brief get port status
  void releaseAllPorts(void);                                      // !<@brief release all of registered ports

 protected:
  DIO_AccessorBase(void);                                               // !<@brief Constructor of DIO Accessor
  virtual int32_t setDirection(const dio_port_descriptor &port) = 0;    // !<@brief abstract method of set direction
  void setAccessorStatus(const uint16_t &status);                       // !<@brief setter of accessor's status
  void setPortStatus(const uint16_t &port_id, const uint16_t &status);  // !<@brief setter of port's status

  gpiod_chip *dio_chip_descriptor_;                              // !<@brief chip descriptor
  std::array<dio_port_descriptor, MAX_PORT_NUM> dio_ports_set_;  // !<@brief port list to access
  uint32_t dio_port_num_;                                        // !<@brief the number of ports to access
  accessor_status accessor_status_;                              // !<@brief ports status
  bool value_inverse_;                                           // !<@brief inverse option
};
}  // namespace dio_ros_driver

#endif
