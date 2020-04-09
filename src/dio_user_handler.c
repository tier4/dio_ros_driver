/*
  Copyright (c) 2020 TierIV.
  Package: dio_ros_dirver
  File Name: dio_ros_dirver.hpp
  Author: Takayuki AKAMINE
  Description: 
 */


#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <gpiod.h>

#include <stdio.h>

#include "dio_ros_driver/dio_user_handler.h"

// variables under scope of this file.
// Update assignment of DI module as system spefication.
static const char* const DI_MODNAME = "di_handler";
static struct gpiod_chip *DI_CHIP;
static struct gpiod_line *DI_LINE;

/*
 * @brief: Initialize DIO module.
 * @param[in]: Nan(void)
 * @return: flag indicating DIO initialized.\
 *  if true (1), the initialization is succeeded.
 *  if false(0), the initialization is failed.
 */
uint32_t init_di(const char* chip_name, const uint32_t line_offset){
  uint32_t i = 0;

  // open DI chip.
  DI_CHIP = gpiod_chip_open_by_name(chip_name);
  if (DI_CHIP == NULL) {
    fprintf(stderr, "Cannot open %s\n", chip_name);
    return 0; // failed
  }

  // get DI Line(Port) from DI Chip.
  DI_LINE = gpiod_chip_get_line(DI_CHIP, line_offset);
  if (DI_LINE == NULL) {
    fprintf(stderr, "Cannot get %d line of %s\n", line_offset, chip_name);
    gpiod_chip_close(DI_CHIP);
    return 0; // failed.
  }

  // set all lines as input ports.
  if (gpiod_line_request_input(DI_LINE, DI_MODNAME) != 0) {
    gpiod_chip_close(DI_CHIP);
    fprintf(stderr, "Cannot execute request bulk on %s\n", chip_name);
    return 0; // failure.
  }
  return 1; // succeeded.
}

/*
 * @brief: Reset all DIO module.
 * @param[in]: Nan(void)
 * @return: flag indicating DIO reset.
 */
uint32_t reset_di(void) {
  // release lines.
  if (DI_LINE != NULL) {
    gpiod_line_release(DI_LINE);
  }
  // close DI chip.
  if (DI_CHIP != NULL) {
    gpiod_chip_close(DI_CHIP);
    return 1; // succeeded.
  } else {
    return 0; // failed (not opened anyway).
  }
}

/*
 * @brief: Read a single channel of digital input.
 * @param[int]: ch (value specifies the DI channel to read.)
 * @return: the status(TRUE or FALSE) of the specified DI channel.
 */ 
uint32_t read_di_line(void) {
  struct gpiod_line_bulk bulk;
  sint32_t di_value;
  sint32_t ret_val_gpiod_func;

  gpiod_line_bulk_init(&bulk);
  gpiod_line_bulk_add(&bulk, DI_LINE);

  ret_val_gpiod_func = gpiod_line_get_value_bulk(&bulk, &di_value);

  if (ret_val_gpiod_func == -1) {
    return 0;
  }

  return di_value;
}


