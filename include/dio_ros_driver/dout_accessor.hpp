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
 * @file dout_accessor.hpp
 * @brief DOUT Accessor class
 * @author: Takayuki AKAMINE
 */

#ifndef __DOUT_ACCESSOR_HPP__
#define __DOUT_ACCESSOR_HPP__

extern "C" {
#include <gpiod.h>
}
#include <cstdint>

#include "dio_accessor_base.hpp"

namespace dio_ros_driver {
class DOUTAccessor : public DIO_AccessorBase {
 public:
  DOUTAccessor();                                                                                                          // !<@brief DOUT Accessor Constructor
  ~DOUTAccessor() {}                                                                                                       // !<@brief DOUT Accessor Destructor
  void initialize(gpiod_chip *const dio_chip_descriptor, const bool &dout_value_inverse, const bool &dout_default_value);  // !<@brief initialize DOUT Accessor
  int32_t writePort(const uint16_t &port_id, const bool &port_value) override;                                             // !<@brief write given value to targeted port
  int32_t resetAllPorts(void);                                                                                             // !<@brief reset all ports by default value

 private:
  int32_t setDirection(const dio_port_descriptor &port) override;  // !<brief set direction
  bool dout_default_value_;                                        // !<@brief initial value
};
}  // namespace dio_ros_driver

#endif
