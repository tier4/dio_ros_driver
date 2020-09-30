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
 * @file dio_accessor_base.cpp
 * @brief DIO Accessor base class
 * @author Takayuki AKAMINE
 */

#include <diagnostic_updater/diagnostic_updater.h>

#include <memory>
#include <string>
#include <map>

#include "dio_ros_driver/dio_accessor_diagnosis.hpp"
#include "dio_ros_driver/dio_accessor_base.hpp"


namespace dio_ros_driver {

/**
 * @brief constructor of accessor diagnosis
 * assign given acceossor's pointer to member variable.
 */
DIO_AccessorDiagnosis::DIO_AccessorDiagnosis(std::shared_ptr<DIO_AccessorBase> dio_accessor) :
  dio_accessor_(dio_accessor) {}

void DIO_AccessorDiagnosis::checkDIOAccessStatus(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  // check dio accessor pointer is not nullptr
  if (dio_accessor_ == nullptr) {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Not initialized");
    return;
  }

  // check accessor and module status
  const accessor_status status = dio_accessor_->getAccessorStatus();
  const uint16_t status_code = status.status_;

  if ((status_code & 0x1100) == 0x0000) {
    // no error.
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Work fine.");
  } else if ((status_code & 0x1000) == 0x1000) {
    // error.
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Error! DIO Access is no longer available (status code: 0x%04x)", status_code);
  } else if ((status_code & 0x0100) == 0x0100) {
    // warning.
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Warning! Some wrong operation (status code: 0x%04x)", status_code);
  } else {
    // unknown error, but not reach unless this program does not bug.
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Undefined behavior");
  }

  // set user value.
  stat.add("status", status.status_);
  stat.add("user_value", status.user_value_);

  // add port offset and status
  for (uint16_t i = 0; i < MAX_PORT_NUM; i++) {
    std::string port_idx = ("port[" + std::to_string(i) + "]");
    const uint16_t port_offset = dio_accessor_->getPortOffset(i);
    const uint16_t port_status = dio_accessor_->getPortStatus(i);

    stat.addf(port_idx, "offset: %d, status: 0x%04x", port_offset, port_status);
  }

    return;
}


}  // namespace dio_ros_driver
