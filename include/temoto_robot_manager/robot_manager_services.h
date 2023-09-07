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

#ifndef TEMOTO_ROBOT_MANAGER__ROBOT_MANAGER_SERVICES_H
#define TEMOTO_ROBOT_MANAGER__ROBOT_MANAGER_SERVICES_H

#include "temoto_robot_manager/RobotLoad.h"
#include "temoto_robot_manager/RobotPlanManipulation.h"
#include "temoto_robot_manager/RobotExecutePlan.h"
#include "temoto_robot_manager/RobotSetTarget.h"
#include "temoto_robot_manager/RobotSetMode.h"
#include "temoto_robot_manager/RobotGetVizInfo.h"
#include "temoto_robot_manager/RobotGetTarget.h"
#include "temoto_robot_manager/RobotGetNamedTargets.h"
#include "temoto_robot_manager/RobotNavigationGoal.h"
#include "temoto_robot_manager/RobotGripperControlPosition.h"
#include "temoto_robot_manager/RobotGetConfig.h"
#include "temoto_robot_manager/CustomRequest.h"
#include "temoto_robot_manager/CustomRequestPreempt.h"
#include "temoto_robot_manager/CustomFeedback.h"
#include "temoto_robot_manager/NavigationFeedback.h"
#include "temoto_robot_manager/RobotCancelNavigationGoal.h"

#include <string>

namespace temoto_robot_manager
{
namespace srv_name
{
const std::string MANAGER = "robot_manager";
const std::string SYNC_TOPIC = "/temoto_robot_manager/" + MANAGER + "/sync";

const std::string SERVER_LOAD = MANAGER + "/" + "load";
const std::string SERVER_PLAN = MANAGER + "/" + "plan";
const std::string SERVER_EXECUTE = MANAGER + "/" + "execute";
const std::string SERVER_GET_VIZ_INFO = MANAGER + "/" + "get_visualization_info";
const std::string SERVER_GET_CONFIG = MANAGER + "/" + "get_config";
const std::string SERVER_SET_MANIPULATION_TARGET = MANAGER + "/" + "set_manipulation_target";
const std::string SERVER_GET_MANIPULATION_TARGET = MANAGER + "/" + "get_manipulation_target";
const std::string SERVER_GET_MANIPULATION_NAMED_TARGETS = MANAGER + "/" + "get_manipulation_named_targets";
const std::string SERVER_NAVIGATION_GOAL = MANAGER + "/" + "navigation_goal";
const std::string SERVER_SET_MODE = MANAGER + "/" + "set_mode";
const std::string SERVER_GRIPPER_CONTROL_POSITION = MANAGER + "/" + "gripper_control_position";
const std::string SERVER_CANCEL_NAVIGATION_GOAL = MANAGER + "/" + "cancel_navigation_goal";

const std::string NAVIGATION_FEEDBACK = MANAGER + "/" + "navigation_feedback";

}



namespace channels
{
namespace custom
{
  const std::string REQUEST = srv_name::MANAGER + "/" + "custom_request";
  const std::string PREEMPT = srv_name::MANAGER + "/" + "custom_preempt";
  const std::string FEEDBACK = srv_name::MANAGER + "/" + "custom_feedback";
}
}

}

static bool operator==(const temoto_robot_manager::RobotLoad::Request& r1,
                       const temoto_robot_manager::RobotLoad::Request& r2)
{
  return (r1.robot_name == r2.robot_name);
}

static bool operator==(const temoto_robot_manager::CustomRequest::Request& r1,
                       const temoto_robot_manager::CustomRequest::Request& r2)
{
  return (r1.robot_name == r2.robot_name &&
          r1.custom_feature_name == r2.custom_feature_name);
}

// bool operator==(const temoto_robot_manager::CustomRequestPreempt::Request& rp1,
//                 const temoto_robot_manager::CustomRequest::Request& r2)
// {
//   return (rp1.robot_name == r2.robot_name &&
//           rp1.custom_feature_name == r2.custom_feature_name);
// }

// bool operator==(const temoto_robot_manager::CustomRequest::Request& r1,
//                 const temoto_robot_manager::CustomRequestPreempt::Request& rp2)
// {
//   return (rp2 == r1);
// }

#endif

