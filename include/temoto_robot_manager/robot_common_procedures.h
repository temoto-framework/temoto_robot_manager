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

#ifndef TEMOTO_ROBOT_MANAGER__ROBOT_COMMO_PROC_H
#define TEMOTO_ROBOT_MANAGER__ROBOT_COMMO_PROC_H

#include "temoto_core/common/temoto_id.h"

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace temoto_robot_manager
{
class CommonProcedure
{
public:
  CommonProcedure(const std::string& name, const YAML::Node& common_conf);

  std::string getName() const
  {
    return name_;
  }

  std::string getPackageName() const
  {
    return package_name_;
  }

  std::string getExecutable() const
  {
    return executable_;
  }

  std::string getExecutableType() const
  {
    return executable_type_;
  }

  std::string getArgs() const
  {
    return args_;
  }

  bool isDefined() const 
  {
    return procedure_defined_;
  }

  bool isLoaded() const 
  {
    return procedure_loaded_;
  }

  void setLoaded(bool loaded)
  {
    procedure_loaded_ = loaded;
  }

protected:
  std::string name_;
  std::string package_name_;
  std::string executable_;
  std::string executable_type_;
  std::string args_;
  bool procedure_defined_;
  bool procedure_loaded_;
};

}

#endif

