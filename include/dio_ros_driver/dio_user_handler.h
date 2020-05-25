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
 * File Name: dio_user_handler.h
 * Author: Takayuki AKAMINE
 * Description: Header file for dio_user_handler.c
 */

#ifndef INCLUDE_DIO_ROS_DRIVER_DIO_USER_HANDLER_H_
#define INCLUDE_DIO_ROS_DRIVER_DIO_USER_HANDLER_H_


#ifdef __cplusplus
extern "C" {

#endif

typedef unsigned int   uint32_t;
typedef signed int     sint32_t;

/*
 * @brief: Initialize DI line.
 * @param[in]: chip_name, string for selecting gpiochip.
 *             line_offset, integer for selecting DI line.
 * @return: flag indicating DI initialized.
 *  if true (1), the initialization is succeeded.
 *  if false(0), the initialization is failed.
 */
uint32_t init_di(const char* chip_name, const uint32_t line_offset);

/*
 * @brief: Reset DI line.
 * @param[in]: Nan(void).
 * @return: flag indicating DI reset.
 */
uint32_t reset_di(void);

/*
 * @brief: Read a single channel of digital input.
 * @param[void]: Nan(void).
 * @return: the status(1 or 0) of the specified DI channel.
 */ 
uint32_t read_di_line(void);

#ifdef __cplusplus
}
#endif

#endif  // INCLUDE_DIO_ROS_DRIVER_DIO_USER_HANDLER_H_
