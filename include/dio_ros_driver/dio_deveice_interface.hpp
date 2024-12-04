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

#ifndef __DIO_DEVEICE_INTERFACE_HPP__
#define __DIO_DEVEICE_INTERFACE_HPP__

extern "C" {
#include <gpiod.h>
}

namespace dio_ros_driver {


class DIO_DeviceInterface {
 public:
  virtual ~DIO_DeviceInterface() = default;

  /**
   * @brief Reserve a single line, set the direction to input.
   * @param line GPIO line object.
   * @param consumer Name of the consumer.
   * @return 0 if the line was properly reserved, -1 on failure.
   */
  virtual int gpiod_line_request_input(struct gpiod_line *line, const char *consumer) = 0;

  /**
   * @brief Reserve a single line, set the direction to output.
   * @param line GPIO line object.
   * @param consumer Name of the consumer.
   * @param default_val Initial line value.
   * @return 0 if the line was properly reserved, -1 on failure.
   */
  virtual int gpiod_line_request_output(struct gpiod_line *line, const char *consumer, int default_val) = 0;

  /**
   * @brief Read current value of a single GPIO line.
   * @param line GPIO line object.
   * @return 0 or 1 if the operation succeeds. On error this routine returns -1
   *         and sets the last error number.
   */
  virtual int gpiod_line_get_value(struct gpiod_line *line) = 0;

  /**
   * @brief Set the value of a single GPIO line.
   * @param line GPIO line object.
   * @param value New value.
   * @return 0 is the operation succeeds. In case of an error this routine
   *         returns -1 and sets the last error number.
   */
  virtual int gpiod_line_set_value(struct gpiod_line *line, int value) = 0;

  /**
   * @brief Get the handle to the GPIO line at given offset.
   * @param chip The GPIO chip object.
   * @param offset The offset of the GPIO line.
   * @return Pointer to the GPIO line handle or NULL if an error occured.
   */
  virtual struct gpiod_line * gpiod_chip_get_line(struct gpiod_chip *chip, unsigned int offset) = 0;

  /**
   * @brief Open a gpiochip by name.
   * @param name Name of the gpiochip to open.
   * @return GPIO chip handle or NULL if an error occurred.
   *
   * This routine appends name to '/dev/' to create the path.
   */
  virtual struct gpiod_chip *gpiod_chip_open_by_name(const char *name) = 0;

  /**
   * @brief Release a previously reserved line.
   * @param line GPIO line object.
   */
  virtual void gpiod_line_release(struct gpiod_line *line) = 0;

  /**
   * @brief Close a GPIO chip handle and release all allocated resources.
   * @param chip The GPIO chip object.
   */
  virtual void gpiod_chip_close(struct gpiod_chip *chip) = 0;
};


class DIO_Device : public DIO_DeviceInterface
{
  public:
    DIO_Device();
    ~DIO_Device();

    int gpiod_line_request_input(struct gpiod_line *line, const char *consumer) override;

    int gpiod_line_request_output(struct gpiod_line *line, const char *consumer, int default_val) override;

    int gpiod_line_get_value(struct gpiod_line *line) override;

    int gpiod_line_set_value(struct gpiod_line *line, int value) override;

    struct gpiod_line * gpiod_chip_get_line(struct gpiod_chip *chip, unsigned int offset) override;

    struct gpiod_chip *gpiod_chip_open_by_name(const char *name) override;

    void gpiod_line_release(struct gpiod_line *line) override;

    void gpiod_chip_close(struct gpiod_chip *chip) override;
};

class DIO_DeviceMock : public DIO_DeviceInterface
{
  public:
    DIO_DeviceMock();
    ~DIO_DeviceMock();

    int gpiod_line_request_input(struct gpiod_line *line, const char *consumer) override;

    int gpiod_line_request_output(struct gpiod_line *line, const char *consumer, int default_val) override;

    int gpiod_line_get_value(struct gpiod_line *line) override;

    int gpiod_line_set_value(struct gpiod_line *line, int value) override;

    struct gpiod_line * gpiod_chip_get_line(struct gpiod_chip *chip, unsigned int offset) override;

    struct gpiod_chip *gpiod_chip_open_by_name(const char *name) override;

    void gpiod_line_release(struct gpiod_line *line) override;

    void gpiod_chip_close(struct gpiod_chip *chip) override;
};

}  // namespace dio_ros_driver

#endif
