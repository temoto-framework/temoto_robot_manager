/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2019 TeMoto Telerobotics
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
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "temoto_robot_manager/robot_common_procedures.h"
#include <iostream>

namespace temoto_robot_manager
{
CommonProcedure::CommonProcedure(const std::string& name, const YAML::Node& common_conf) : name_(name)
{
  this->executable_ = common_conf["executable"].as<std::string>();
  this->executable_type_ = common_conf["executable_type"].as<std::string>();
  // setFromConfig(common_conf["executable_type"], this->executable_type_);
  if (common_conf["args"])
  {
    this->args_ = common_conf["args"].as<std::string>();
  }
  if (executable_type_ == "ros")
  {
    this->package_name_ = common_conf["package_name"].as<std::string>();
  }
  this->procedure_defined_ = true;
}
}

