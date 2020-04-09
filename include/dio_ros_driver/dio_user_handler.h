/*
  Copyright (c) 2020 TierIV.
  Package: dio_ros_dirver
  File Name: dio_ros_dirver.hpp
  Author: Takayuki AKAMINE
  Description: Header file for dio_user_handler.c
 */

#ifndef __DIO_USER_HANDLER__
#define __DIO_USER_HANDLER__


// Assignment number for DI's GPIO
#define DI_PORT_NUM 8
enum DI_GPIO_NUM {
  CN_DI0 = 0,
  CN_DI1 = 1,
  CN_DI2 = 2,
  CN_DI3 = 3,
  CN_DI4 = 4,
  CN_DI5 = 5,
  CN_DI6 = 6,
  CN_DI7 = 7,
};


/***
enum DO_GPIO_NUM {
  CN_DO0 = 253,
  CN_DO1 = 254,
  CN_DO2 = 255,
  CN_DO3 = 256,
  CN_DO4 = 257,
  CN_DO5 = 258,
  CN_DO6 = 259,
  CN_DO7 = 260,
};
***/

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
uint32_t init_di(void);

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
uint32_t read_di_line(uint32_t ch);

/*
 * @brief: getter function of all of DI's value
 * @param[in]: Nan(void)
 * @return: return all DI's value(0or1)
 */
uint32_t read_all_di_lines(void);

#ifdef __cplusplus
}
#endif

#endif
