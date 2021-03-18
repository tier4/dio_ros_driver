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
 * @file dio_diagnosis.hpp
 * @brief DIO Diagnosis class
 * @author Takayuki AKAMINE
 */


#include <unistd.h>
#include "dio_ros_driver/dio_diagnostic_updater.hpp"
#include "dio_ros_driver/dio_accessor_diagnosis.hpp"
#include "dio_ros_driver/din_accessor.hpp"
#include "dio_ros_driver/dout_accessor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>


namespace dio_ros_driver {

/**
 * @brief constructor
 * this constructor initializes diagnostic callback functions.
 * @param[in] din_accessor pointer of DIN Accessor.
 * @param[in] dout_accessor pointer of DOUT Accessor.
 */
DIO_DiagnosticUpdater::DIO_DiagnosticUpdater(std::shared_ptr<rclcpp::Node> node_ptr ,
                                             std::shared_ptr<DINAccessor> din_accessor, std::shared_ptr<DOUTAccessor> dout_accessor) :
diag_updater_(node_ptr),
hostname_(),
din_diagnosis_(nullptr),
dout_diagnosis_(nullptr) {
  gethostname(hostname_, sizeof(hostname_));
  diag_updater_.setHardwareID(hostname_);

  // set callbacks for diagnosising DIN
  din_diagnosis_ = std::make_shared<DIO_AccessorDiagnosis>(din_accessor);
  diag_updater_.add("DIN Access", din_diagnosis_.get(), &DIO_AccessorDiagnosis::checkDIOAccessStatus);

  // set callbacks for diagnosising DIN
  dout_diagnosis_ = std::make_shared<DIO_AccessorDiagnosis>(dout_accessor);
  diag_updater_.add("DOUT Access", dout_diagnosis_.get(), &DIO_AccessorDiagnosis::checkDIOAccessStatus);
}

/**
 * @brief update diagnosis
 * update diagnostic status and send topic for diagnosis.
 */
void DIO_DiagnosticUpdater::update(void) {
  //diag_updater_.update();
}

/**
 * @brief force update diagnosis
 * force update diagnostic status and send topic for diagnosis.
 */
void DIO_DiagnosticUpdater::force_update(void) {
  diag_updater_.force_update();
}

}  // namespace dio_ros_driver
