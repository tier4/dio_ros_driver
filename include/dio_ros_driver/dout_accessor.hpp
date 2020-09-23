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
 * File Name: dout_accessor.hpp
 * Author: Takayuki AKAMINE
 * Description: Header file for dio_ros_driver
 */

#ifndef __DOUT_ACCESSOR_HPP__
#define __DOUT_ACCESSOR_HPP__

#include "dio_accessor_base.hpp"

extern "C"
{
#include <gpiod.h>
}
#include <cstdint>

namespace dio_ros_driver
{
  class DOUTAccessor : public DIO_AccessorBase
  {
  public:
    DOUTAccessor();
    ~DOUTAccessor() {}
    void initialize(gpiod_chip *const dio_chip_descriptor, const bool &dout_default_value);
    int32_t writePort(const uint16_t &port_id, const bool &port_value) override;
    int32_t resetAllPorts(void);

  private:
    int32_t setDirection(const dio_port_descriptor &port) override;
    bool dout_default_value_;
  };
} // namespace dio_ros_driver

#endif
