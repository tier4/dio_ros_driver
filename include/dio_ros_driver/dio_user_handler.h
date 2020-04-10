/*
  Copyright (c) 2020 TierIV.
  Package: dio_ros_dirver
  File Name: dio_ros_dirver.hpp
  Author: Takayuki AKAMINE
  Description: Header file for dio_user_handler.c
 */

#ifndef __DIO_USER_HANDLER__
#define __DIO_USER_HANDLER__


#ifdef __cplusplus
extern "C" {

#endif

typedef unsigned int   uint32_t;
typedef signed int     sint32_t;

/*
 * @brief: Initialize DIO module.
 * @param[in]: Nan(void)
 * @return: flag indicating DIO initialized.
 */
uint32_t init_di(const char* chip_name, const uint32_t line_offset);

/*
 * @brief: Reset all DIO module.
 * @param[in]: Nan(void)
 * @return: flag indicating DIO reset.
 */
uint32_t reset_di(void);

/*
 * @brief: Read a single channel of digital input.
 * @param[int]: ch (value specifies the DI channel to read.)
 * @return: the status(1 or 0) of the specified DI channel.
 */ 
uint32_t read_di_line(void);

#ifdef __cplusplus
}
#endif

#endif