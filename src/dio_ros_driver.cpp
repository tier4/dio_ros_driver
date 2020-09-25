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
 * @package dio_ros_dirver
 * @file dio_ros_dirver.cpp
 * @brief DIO ROS Driver class
 * @author Takayuki AKAMINE
 */

#include <dio_ros_driver/DIOPort.h>
#include <dio_ros_driver/DIOStatus.h>
#include <ros/ros.h>

#include <cstring>
#include <iostream>
#include <cstdlib>
#include <boost/bind.hpp>

#include "dio_ros_driver/dio_ros_driver.hpp"

namespace dio_ros_driver {
/**
 * @brief Constructor of DIO_ROSDriver
 * Prepare ROS specific variable and initialize variable
 * @param nh node handler
 * @param pnh private node handler
 */
DIO_ROSDriver::DIO_ROSDriver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh),
      pnh_(pnh),
      din_port_publisher_array_(),
      dout_port_subscriber_array_(),
      din_status_publisher_(),
      dout_status_publisher_(),
      din_accessor_(),
      dout_accessor_(),
      access_frequency_(1.0),
      chip_name_("chipname0"),
      din_value_inverse_(false),
      dout_value_inverse_(false),
      dout_default_value_(false),
      write_update_mutex_(),
      dout_user_update_(),
      dio_chip_(nullptr) {
  // load general parameters.
  pnh_.param<double>("access_frequency", access_frequency_, 1.0);
  pnh_.param<std::string>("chip_name", chip_name_, "gpiochip0");
  pnh_.param<bool>("din_value_inverse", din_value_inverse_, false);
  pnh_.param<bool>("dout_value_inverse", dout_value_inverse_, false);
  pnh_.param<bool>("dout_default_value", dout_default_value_, false);

  // prepare publishers
  for (uint32_t i = 0; i < MAX_PORT_NUM; i++) {
    std::string topic_name = "/dio/din" + std::to_string(i);
    din_port_publisher_array_.at(i) = nh_.advertise<dio_ros_driver::DIOPort>(topic_name, 1, false);
  }
  // prepare subscribers
  for (uint32_t i = 0; i < MAX_PORT_NUM; i++) {
    std::string topic_name = "/dio/dout" + std::to_string(i);
    dout_port_subscriber_array_.at(i) = nh_.subscribe<dio_ros_driver::DIOPort>(topic_name,
                                                                               1,
                                                                               boost::bind(&DIO_ROSDriver::receiveWriteRequest, this, _1, i));
  }

  din_status_publisher_ = nh_.advertise<dio_ros_driver::DIOStatus>("/dio/din_status", 1, false);
  dout_status_publisher_ = nh_.advertise<dio_ros_driver::DIOStatus>("/dio/dout_status", 1, false);
}

/**
 * @brief DIO Accessor Initialization.
 * initialization to access dio chip and add ports to access.
 * @retval 0 always 0, but update status topic if any error occur
 */
int DIO_ROSDriver::init(void) {
  dio_chip_ = gpiod_chip_open_by_name(chip_name_.c_str());
  din_accessor_.initialize(dio_chip_, din_value_inverse_);
  dout_accessor_.initialize(dio_chip_, dout_value_inverse_, dout_default_value_);

  addAccessorPorts("/dio/din_ports", din_accessor_);
  addAccessorPorts("/dio/dout_ports", dout_accessor_);

  return 0;
}

/**
 * @brief main routine of this node
 * update DO port and send topic periodically based on given access_frequency
 */
void DIO_ROSDriver::run(void) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate loop_rate(access_frequency_);

  dio_status din_status = din_accessor_.getStatus();
  dio_status dout_status = dout_accessor_.getStatus();

  while (ros::ok()) {
    // read data from DIN ports.
    if (din_status.status_ == 0)
      readDINPorts();
    // write data to DOUT ports.
    if (dout_status.status_ == 0)
      writeDOUTPorts();
    // publish din_status
    updateStatus(din_accessor_, din_status_publisher_);
    // publish dout_status
    updateStatus(dout_accessor_, dout_status_publisher_);

    loop_rate.sleep();
  }
}

/**
 * @brief terminate processing
 * After receiving SIGTERM signal, this method will be called.
 * @param signal_id signal id which is expected by SIGTERM.
 */
void DIO_ROSDriver::terminate(int signal_id) {
  ROS_INFO("Terminate process");
  int32_t exit_status = 0;
  const dio_status dout_status = dout_accessor_.getStatus();
  // any other thread cannot access dout_accessor_ object.
  write_update_mutex_.lock();
  // check if DIO chip opened
  if (dio_chip_ == nullptr) {
    ROS_INFO("DIO chip is not opened. This program will be closed without any DIO access");
    exit_status = 0;
    std::exit(exit_status);
  }

  // check if any occurs at ports
  if (dout_accessor_.getNumOfPorts() == 0) {
    ROS_INFO("Any out port are not accessed on this program. Close DIO chip.");
    exit_status = 0;
    goto CLOSE_DIO_CHIP;
  }

  if (dout_status.status_ != 0) {
    ROS_ERROR("Unable to access to some failed port. Close the program immediately");
    exit_status = -1;
    goto CLOSE_DIO_CHIP;
  }

  if (dout_accessor_.resetAllPorts() != 0) {
    ROS_ERROR("Failed in access to any port.  Close the program immediately");
    exit_status = -1;
    goto CLOSE_DIO_CHIP;
  }
  dout_accessor_.releaseAllPorts();
  exit_status = 0;

CLOSE_DIO_CHIP:
  gpiod_chip_close(dio_chip_);

  std::exit(exit_status);
}

/// Private methods.
/**
 * @brief add access port based on given parameter (after load .yaml)
 * @param[in] param_name ROS parameter name to list port id and offset
 * @param[out] dio_accessor dio accessor to update
 */
void DIO_ROSDriver::addAccessorPorts(const std::string param_name, DIO_AccessorBase &dio_accessor) {
  // get port number array.
  std::vector<int32_t> offset_array;
  nh_.getParam(param_name, offset_array);

  for (const auto &offset : offset_array) {
    dio_accessor.addPort(offset);
  }
  return;
}

/**
 * @brief callback function to hold and notify write request
 * @param[in] dout_topic topic to update port from application
 * @param[in] port_id targeted port number.
 */
void DIO_ROSDriver::receiveWriteRequest(const dio_ros_driver::DIOPort::ConstPtr &dout_topic, const uint32_t &port_id) {
  write_update_mutex_.lock();
  dout_update &targeted_dout_update = dout_user_update_.at(port_id);
  targeted_dout_update.update_ = true;
  targeted_dout_update.value_ = dout_topic->value;
  write_update_mutex_.unlock();
}

/**
 * @brief convert read value to topic
 * read values from all DI ports and send them as respective topic to application node
 */
void DIO_ROSDriver::readDINPorts(void) {
  dio_ros_driver::DIOPort din_port;
  din_port.header.stamp = ros::Time::now();
  for (uint32_t i = 0; i < din_accessor_.getNumOfPorts(); i++) {
    int32_t read_value = din_accessor_.readPort(i);
    if (read_value < 0) {
      ROS_ERROR("DI read error %d", i);
    } else {
      din_port.value = static_cast<bool>(read_value);
      din_port_publisher_array_.at(i).publish(din_port);
    }
  }
}

/**
 * @brief write value to DO port
 * update value of DO port according to user's request.
 */
void DIO_ROSDriver::writeDOUTPorts(void) {
  write_update_mutex_.lock();
  for (uint32_t i = 0; i < dout_accessor_.getNumOfPorts(); i++) {
    if (dout_user_update_.at(i).update_ == true) {
      int32_t ret_code = dout_accessor_.writePort(i, dout_user_update_.at(i).value_);
      if (ret_code == -1) {
        ROS_ERROR("DOUT write error %d", i);
      }
      dout_user_update_.at(i).update_ = false;
    }
  }
  write_update_mutex_.unlock();
}

/**
 * @brief update status and send it as topic
 * read status and values from all ports and send them with same topic
 */
void DIO_ROSDriver::updateStatus(DIO_AccessorBase &dio_accessor, ros::Publisher &status_publisher) {
  dio_ros_driver::DIOStatus dio_status_topic;
  dio_status updated_dio_status = dio_accessor.getStatus();
  dio_status_topic.user_value = updated_dio_status.user_value_;
  dio_status_topic.status = updated_dio_status.status_;
  dio_status_topic.header.stamp = ros::Time::now();
  status_publisher.publish(dio_status_topic);
  return;
}

}  // namespace dio_ros_driver
