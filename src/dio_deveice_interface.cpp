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
 * @file dio_deveice_interface.hpp
 * @brief DIO_DeviceInterface class header
 * @author Satoshi INOUE
 */

#include "dio_ros_driver/dio_deveice_interface.hpp"

extern "C" {
  #include <gpiod.h>
}

namespace dio_ros_driver {

  // ----------------------------------------------------------------------------------------------------------------------------
  // DIO_Device 

  DIO_Device::DIO_Device() {}

  DIO_Device::~DIO_Device() {}

  int DIO_Device::gpiod_line_request_input(struct gpiod_line *line, const char *consumer) {
    return ::gpiod_line_request_input(line, consumer);
  }

  int DIO_Device::gpiod_line_request_output(struct gpiod_line *line, const char *consumer, int default_val) {
    return ::gpiod_line_request_output(line, consumer, default_val);
  }

  int DIO_Device::gpiod_line_get_value(struct gpiod_line *line) {
    return ::gpiod_line_get_value(line);
  }

  int DIO_Device::gpiod_line_set_value(struct gpiod_line *line, int value) {
    return ::gpiod_line_set_value(line, value);
  }

  struct gpiod_line *DIO_Device::gpiod_chip_get_line(struct gpiod_chip *chip, unsigned int offset) {
    return ::gpiod_chip_get_line(chip, offset);
  }

  struct gpiod_chip *DIO_Device::gpiod_chip_open_by_name(const char *name) {
    return ::gpiod_chip_open_by_name(name);
  }

  void DIO_Device::gpiod_line_release(struct gpiod_line *line) {
    return ::gpiod_line_release(line);
  }

  void DIO_Device::gpiod_chip_close(struct gpiod_chip *chip) {
    return ::gpiod_chip_close(chip);
  }


  // ----------------------------------------------------------------------------------------------------------------------------
  // DIO_DeviceMock 

  DIO_DeviceMock::DIO_DeviceMock() {}

  DIO_DeviceMock::~DIO_DeviceMock() {}

  int DIO_DeviceMock::gpiod_line_request_input(struct gpiod_line *line, const char *consumer) {
    throw "Not implemented exception";
  }

  int DIO_DeviceMock::gpiod_line_request_output(struct gpiod_line *line, const char *consumer, int default_val) {
    throw "Not implemented exception";
  }

  int DIO_DeviceMock::gpiod_line_get_value(struct gpiod_line *line) {
    throw "Not implemented exception";
  }

  int DIO_DeviceMock::gpiod_line_set_value(struct gpiod_line *line, int value) {
    throw "Not implemented exception";
  }

  struct gpiod_line *DIO_DeviceMock::gpiod_chip_get_line(struct gpiod_chip *chip, unsigned int offset) {
    throw "Not implemented exception";
  }

  struct gpiod_chip *DIO_DeviceMock::gpiod_chip_open_by_name(const char *name) {
    throw "Not implemented exception";
  }

  void DIO_DeviceMock::gpiod_line_release(struct gpiod_line *line) {
    throw "Not implemented exception";
  }

  void DIO_DeviceMock::gpiod_chip_close(struct gpiod_chip *chip) {
    throw "Not implemented exception";
  }
}  // namespace dio_ros_driver
