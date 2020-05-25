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
 * Package: dio_ros_dirver
 * File Name: dio_user_handler.c
 * Author: Takayuki AKAMINE
 * Description: Wrapper module for libgpiod.
 */


#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <gpiod.h>

#include <stdio.h>

#include "dio_ros_driver/dio_user_handler.h"

// variables under scope of this file.
// Update assignment of DI module as system spefication.
static const char* const DI_Modname = "di_handler";
static struct gpiod_chip *DI_Chip;
static struct gpiod_line *DI_Line;

/*
 * @brief: Initialize DI line.
 * @param[in]: chip_name, string for selecting gpiochip.
 *             line_offset, integer for selecting DI line.
 * @return: flag indicating DI initialized.
 *  if true (1), the initialization is succeeded.
 *  if false(0), the initialization is failed.
 */
uint32_t init_di(const char* chip_name, const uint32_t line_offset) {
  // open DI chip.
  DI_Chip = gpiod_chip_open_by_name(chip_name);
  if (DI_Chip == NULL) {
    fprintf(stderr, "Cannot open %s\n", chip_name);
    return 0;  // failed
  }

  // get DI Line(Port) from DI Chip.
  DI_Line = gpiod_chip_get_line(DI_Chip, line_offset);
  if (DI_Line == NULL) {
    fprintf(stderr, "Cannot get %d line of %s\n", line_offset, chip_name);
    gpiod_chip_close(DI_Chip);
    return 0;  // failed.
  }

  // set all lines as input ports.
  if (gpiod_line_request_input(DI_Line, DI_Modname) != 0) {
    gpiod_chip_close(DI_Chip);
    fprintf(stderr, "Cannot execute request bulk on %s\n", chip_name);
    return 0;  // failure.
  }
  return 1;  // succeeded.
}

/*
 * @brief: Reset DI line.
 * @param[in]: Nan(void).
 * @return: flag indicating DI reset.
 */
uint32_t reset_di(void) {
  // release lines.
  if (DI_Line != NULL) {
    gpiod_line_release(DI_Line);
  }
  // close DI chip.
  if (DI_Chip != NULL) {
    gpiod_chip_close(DI_Chip);
    return 1;  // succeeded.
  } else {
    return 0;  // failed (not opened anyway).
  }
}

/*
 * @brief: Read a single channel of digital input.
 * @param[void]: Nan(void).
 * @return: the status(1 or 0) of the specified DI channel.
 */ 
uint32_t read_di_line(void) {
  struct gpiod_line_bulk bulk;
  sint32_t di_value;
  sint32_t ret_val_gpiod_func;

  gpiod_line_bulk_init(&bulk);
  gpiod_line_bulk_add(&bulk, DI_Line);

  ret_val_gpiod_func = gpiod_line_get_value_bulk(&bulk, &di_value);

  if (ret_val_gpiod_func == -1) {
    return 0;
  }

  return di_value;
}


