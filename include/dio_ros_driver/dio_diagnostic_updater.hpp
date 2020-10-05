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
 * @file dio_diagnostic_updater.hpp
 * @brief DIO Diagnostic updater class
 * @author Takayuki AKAMINE
 */

#ifndef __DIO_DIAGNOSTIC_UPDATER_HPP__
#define __DIO_DIAGNOSTIC_UPDATER_HPP__

#include <diagnostic_updater/diagnostic_updater.h>

#include <memory>
#include <map>
#include "dio_ros_driver/dio_accessor_diagnosis.hpp"
#include "dio_ros_driver/din_accessor.hpp"
#include "dio_ros_driver/dout_accessor.hpp"

namespace dio_ros_driver {

class DIO_DiagnosticUpdater {
 public:
  DIO_DiagnosticUpdater(std::shared_ptr<DINAccessor> din_accessor,
      std::shared_ptr<DOUTAccessor> dout_accessor);  //  !<@brief constructor
  ~DIO_DiagnosticUpdater() {}                                          //  !<@brief destructor

  void update(void);         //  !<@brief update diagnosis result.
  void force_update(void);   //  !<@brief force update diagnosis result.

 private:
  // diagnostic member.
  diagnostic_updater::Updater diag_updater_;               // !<@brief wrapped diagnostic updater.
  char hostname_[HOST_NAME_MAX + 1];                       // !<@brief hostname
  std::shared_ptr<DIO_AccessorDiagnosis> din_diagnosis_;   // !<@brief diagnostic object pointer for DINAccessor.
  std::shared_ptr<DIO_AccessorDiagnosis> dout_diagnosis_;  // !<@brief diagnostic object pointer for DOUTAccessor.
};
}  // namespace dio_ros_driver

#endif
