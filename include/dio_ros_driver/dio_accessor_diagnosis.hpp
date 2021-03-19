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
 * @file dio_accessor_diagnosis.hpp
 * @brief DIO Accessor diagnosis class
 * @author Takayuki AKAMINE
 */

#ifndef __DIO_ACCESSOR_DIAGNOSTIC_HPP__
#define __DIO_ACCESSOR_DIAGNOSTIC_HPP__

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <memory>
#include "dio_ros_driver/dio_accessor_base.hpp"

namespace dio_ros_driver {


class DIO_AccessorDiagnosis {
 public:
  explicit DIO_AccessorDiagnosis(std::shared_ptr<DIO_AccessorBase> dio_accessor);  //  !<@brief constructor.
  ~DIO_AccessorDiagnosis() {}                                                       // !<@brief destructor
  void checkDIOAccessStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);    //  !<@brief accessor's status callback.

 private:
  std::shared_ptr<DIO_AccessorBase> dio_accessor_;   //  !<@brief pointer of DIO_AccessorBase object.
};
}  // namespace dio_ros_driver


#endif
