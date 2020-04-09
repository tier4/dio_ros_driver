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
// Update assignment of DI chipname as system spefication.
static const char* const DI_CHIPNAME = "gpiochip0";
static struct gpiod_chip *DI_CHIP;
static struct gpiod_line_bulk DI_LINES_BULK;
static const sint32_t DI_PORT_ARRAY[] = {CN_DI0, CN_DI1, CN_DI2, CN_DI3,
                                    CN_DI4, CN_DI5, CN_DI6, CN_DI7};

/*
 * @brief: Initialize DIO module.
 * @param[in]: Nan(void)
 * @return: flag indicating DIO initialized.\
 *  if true (1), the initialization is succeeded.
 *  if false(0), the initialization is failed.
 */
uint32_t init_di(void){
  uint32_t i = 0;
  struct gpiod_line *DI_LINE;

  // open DI chip.
  DI_CHIP = gpiod_chip_open_by_name(DI_CHIPNAME);
  if (DI_CHIP == NULL) {
    fprintf(stderr, "Cannot open %s\n", DI_CHIPNAME);
    return 0; // failed
  }

  // initilize bulk.
  gpiod_line_bulk_init(&DI_LINES_BULK);

  // get DI Line(Port) from DI Chip.
  for (i = 0; i < DI_PORT_NUM; i++) {
    DI_LINE = gpiod_chip_get_line(DI_CHIP, DI_PORT_ARRAY[i]);
    if (DI_LINE == NULL) {
      fprintf(stderr, "Cannot get %d line of %s\n", DI_PORT_ARRAY[i], DI_CHIPNAME);
      gpiod_chip_close(DI_CHIP);
      return 0; // failed.
    }
    gpiod_line_bulk_add(&DI_LINES_BULK, DI_LINE);
  }

  // set all lines as input ports.
  if (gpiod_line_request_bulk_input(&DI_LINES_BULK, DI_MODNAME) != 0) {
    gpiod_chip_close(DI_CHIP);
    fprintf(stderr, "Cannot execute request bulk on %s\n", DI_CHIPNAME);
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
  if (gpiod_line_bulk_num_lines(&DI_LINES_BULK) > 0) {
    gpiod_line_release_bulk(&DI_LINES_BULK);
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
uint32_t read_di_line(uint32_t ch) {
  // return 0 if accessing out-of-range.
  struct gpiod_line *target_di_line;
  sint32_t target_line_value;
  
  if (ch >= DI_PORT_NUM ) {
    return 0;
  }

  // read port with gpiod_line_get_value.
  target_di_line = gpiod_line_bulk_get_line(&DI_LINES_BULK, ch);
  target_line_value = gpiod_line_get_value(target_di_line);
  if (target_line_value == -1) {
    return 0;
  } else {
    return target_line_value;
  }
}

/*
 * @brief: getter function of all of DI's value
 * @param[in]: Nan(void)
 * @return: return all DI's value(0or1)
            if failure occurs, return 0xFFFF0000.
 */
uint32_t read_all_di_lines(void){
  sint32_t di_value_array[DI_PORT_NUM];
  sint32_t ret_val_gpiod_func;
  uint32_t di_port_value;
  uint32_t i;
  
  ret_val_gpiod_func = gpiod_line_get_value_bulk(&DI_LINES_BULK, di_value_array);

  if (ret_val_gpiod_func == -1) {
    return 0xFFFF0000; // error value;
  }
  // make integer-type return value from array.
  for (i = 0; i < DI_PORT_NUM; i++) {
    di_port_value = di_value_array[i] << i;
  }
  return di_port_value;
}

