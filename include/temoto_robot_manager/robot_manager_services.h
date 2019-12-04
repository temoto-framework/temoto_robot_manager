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

/* Author: Veiko Vunder */

#ifndef TEMOTO_ROBOT_MANAGER__ROBOT_MANAGER_SERVICES_H
#define TEMOTO_ROBOT_MANAGER__ROBOT_MANAGER_SERVICES_H

#include "temoto_robot_manager/RobotLoad.h"
#include "temoto_robot_manager/RobotPlan.h"
#include "temoto_robot_manager/RobotExecute.h"
#include "temoto_robot_manager/RobotSetTarget.h"
#include "temoto_robot_manager/RobotSetMode.h"
#include "temoto_robot_manager/RobotGetVizInfo.h"
#include "temoto_robot_manager/RobotGetTarget.h"

#include <string>

namespace robot_manager
{
namespace srv_name
{
const std::string MANAGER = "robot_manager";
const std::string SYNC_TOPIC = "/temoto_robot_manager/" + MANAGER + "/sync";

const std::string SERVER_LOAD = "load";
const std::string SERVER_PLAN = "plan";
const std::string SERVER_EXECUTE = "execute";
const std::string SERVER_GET_VIZ_INFO = "get_visualization_info";
const std::string SERVER_SET_TARGET = "set_target";

const std::string SERVER_GET_TARGET = "get_target";

const std::string SERVER_SET_MODE = "set_mode";
}

namespace modes
{
const std::string AUTO = "auto";
const std::string NAVIGATION = "navigation";
const std::string MANIPULATION = "manipulation";
}
}

static bool operator==(const temoto_robot_manager::RobotLoad::Request& r1,
                       const temoto_robot_manager::RobotLoad::Request& r2)
{
  return (r1.robot_name == r2.robot_name);
}
#endif
