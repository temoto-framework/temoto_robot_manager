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

#ifndef TEMOTO_ROBOT_MANAGER__ROBOT_MANAGER_INTERFACE_H
#define TEMOTO_ROBOT_MANAGER__ROBOT_MANAGER_INTERFACE_H

#include "temoto_core/common/base_subsystem.h"
#include "rr/ros1_resource_registrar.h"
#include "temoto_robot_manager/robot_manager_services.h"
#include "yaml-cpp/yaml.h"
#include <vector>
#include <string>
#include <ctime>

namespace temoto_robot_manager
{
class RobotManagerInterface : public temoto_core::BaseSubsystem
{
public:
  RobotManagerInterface(bool initialize_interface = false)
  : unique_suffix_(std::to_string(createID()))
  , initialized_(false)
  {
    if (initialize_interface)
    {
      initialize();
    }
  }

  void initialize()
  {
    if (!initialized_)
    {
      rr_name_ = TEMOTO_LOG_ATTR.getSubsystemNameWithSlash() + GET_CLASS_NAME + "_" + unique_suffix_;
      resource_registrar_ = std::make_unique<temoto_resource_registrar::ResourceRegistrarRos1>(rr_name_);
      resource_registrar_->init();

      client_plan_ =
        nh_.serviceClient<RobotPlanManipulation>(srv_name::SERVER_PLAN);
      client_exec_ =
        nh_.serviceClient<RobotExecutePlan>(srv_name::SERVER_EXECUTE);
      client_viz_info_ =
        nh_.serviceClient<RobotGetVizInfo>(srv_name::SERVER_GET_VIZ_INFO);
      client_set_manipulation_target_ =
        nh_.serviceClient<RobotSetTarget>(srv_name::SERVER_SET_MANIPULATION_TARGET);
      client_get_manipulation_target_ =
        nh_.serviceClient<RobotGetTarget>(srv_name::SERVER_GET_MANIPULATION_TARGET);
      client_get_manipulation_named_targets_ =
        nh_.serviceClient<RobotGetNamedTargets>(srv_name::SERVER_GET_MANIPULATION_NAMED_TARGETS);
      client_navigation_goal_ =
        nh_.serviceClient<RobotNavigationGoal>(srv_name::SERVER_NAVIGATION_GOAL);
      client_gripper_control_position_ =
        nh_.serviceClient<RobotGripperControlPosition>(srv_name::SERVER_GRIPPER_CONTROL_POSITION);
      client_get_robot_config_ =
        nh_.serviceClient<RobotGetConfig>(srv_name::SERVER_GET_CONFIG);

      initialized_ = true;
    }
    else
    {
      TEMOTO_WARN_STREAM_("The Robot Manager interface is already initialized");
    }
  }

  unsigned int createID()
  {
    std::srand(std::time(nullptr));
    return std::rand();
  }

  YAML::Node getRobotConfig(const std::string& robot_name)
  try
  {
    temoto_robot_manager::RobotGetConfig msg;
    msg.request.robot_name = robot_name;
    if (client_get_robot_config_.call(msg))
    {
      if (!msg.response.success)
      {
        throw TEMOTO_ERRSTACK("Could not get the config of robot '" + robot_name + "'");
      }
      return YAML::Load(msg.response.robot_config);
    }
    else
    {
      throw TEMOTO_ERRSTACK("Unable to reach robot_manager");
    }
  }
  catch(const std::exception& e)
  {
    throw TEMOTO_ERRSTACK(e.what());
  }

  void loadRobot(const std::string& robot_name)
  try
  {
    temoto_robot_manager::RobotLoad load_robot_msg;
    load_robot_msg.request.robot_name = robot_name;
    
    resource_registrar_->call<RobotLoad>(srv_name::MANAGER
    , srv_name::SERVER_LOAD
    , load_robot_msg
    , std::bind(&RobotManagerInterface::statusInfoCb, this, std::placeholders::_1, std::placeholders::_2));

    allocated_robots_.push_back(load_robot_msg);
  }
  catch(resource_registrar::TemotoErrorStack e)
  {
    throw FWD_TEMOTO_ERRSTACK(e);
  }

  void planManipulation(const std::string& robot_name
  , const std::string& planning_group
  , const geometry_msgs::PoseStamped& pose)
  {
    temoto_robot_manager::RobotPlanManipulation msg;
    msg.request.goal_target = msg.request.POSE_STAMPED;
    msg.request.target_pose = pose;
    msg.request.planning_group = planning_group;
    msg.request.robot_name = robot_name;
    
    if (!client_plan_.call(msg))
    {
      throw TEMOTO_ERRSTACK("Unable to reach robot_manager");
    }

    if (!msg.response.success)
    {
      throw TEMOTO_ERRSTACK("Unsuccessful attempt to 'planManipulation'");
    }
  }

  void planManipulation(const std::string& robot_name
  , const std::string& planning_group
  , const std::string& named_target_pose)
  {
    temoto_robot_manager::RobotPlanManipulation msg;
    msg.request.goal_target = msg.request.NAMED_TARGET_POSE;
    msg.request.named_target = named_target_pose;
    msg.request.planning_group = planning_group;
    msg.request.robot_name = robot_name;
    
    if (!client_plan_.call(msg))
    {
      throw TEMOTO_ERRSTACK("Unable to reach robot_manager");
    }
    
    if (!msg.response.success)
    {
      throw TEMOTO_ERRSTACK("Unsuccessful attempt to invoke 'planManipulation'");
    }
  }

  void planManipulation(const std::string& robot_name
  , const std::string& planning_group
  , const std::vector<double> &joint_state_target)
  {
    temoto_robot_manager::RobotPlanManipulation msg;
    msg.request.goal_target = msg.request.JOINT_STATE;
    msg.request.joint_state_target = joint_state_target;
    msg.request.planning_group = planning_group;
    msg.request.robot_name = robot_name;

    if (!client_plan_.call(msg))
    {
      throw TEMOTO_ERRSTACK("Unable to reach robot_manager");
    }

    if (!msg.response.success)
    {
      throw TEMOTO_ERRSTACK("Unsuccessful attempt to invoke 'planManipulation'");
    }
  }

  void executePlan(const std::string& robot_name)
  {
    temoto_robot_manager::RobotExecutePlan msg;
    msg.request.robot_name = robot_name;
    if (!client_exec_.call(msg))
    {
      throw TEMOTO_ERRSTACK("Unable to reach robot_manager");
    }

    if (!msg.response.success)
    {
      throw TEMOTO_ERRSTACK("Unsuccessful attempt to invoke 'executePlan'");
    }
  }

  std::string getMoveitRvizConfig()
  {
    temoto_robot_manager::RobotGetVizInfo msg;
    if (!client_viz_info_.call(msg))
    {
      throw TEMOTO_ERRSTACK("Unable to reach robot_manager");
    }

    if (!msg.response.success)
    {
      throw TEMOTO_ERRSTACK("Unsuccessful attempt to invoke 'getMoveitRvizConfig'");
    }

    return msg.response.info;
  }

  void setTarget(const std::string& object_name)
  {
    temoto_robot_manager::RobotSetTarget msg;
    msg.request.object_name = object_name;
    if (!client_set_manipulation_target_.call(msg))
    {
      throw TEMOTO_ERRSTACK("Unable to reach robot_manager");
    }

    if (!msg.response.success)
    {
      throw TEMOTO_ERRSTACK("Unsuccessful attempt to invoke 'setTarget'");
    }
  }

 geometry_msgs::PoseStamped getEndEffPose(const std::string& robot_name, const std::string& planning_group)
 {
    temoto_robot_manager::RobotGetTarget msg; 
    msg.request.robot_name = robot_name;
    msg.request.planning_group = planning_group;
    msg.request.get_current_state = msg.request.END_EFFECTOR;
    if (!client_get_manipulation_target_.call(msg))
    {
      throw TEMOTO_ERRSTACK("Unable to reach robot_manager");
    }

    if (!msg.response.success)
    {
      throw TEMOTO_ERRSTACK("Unsuccessful attempt to invoke 'getEndEffPose'");
    }
    return msg.response.pose;
  }

  std::vector<double> getCurrentJointValues(const std::string& robot_name, const std::string& planning_group)
 {
    temoto_robot_manager::RobotGetTarget msg;
    msg.request.robot_name = robot_name;
    msg.request.planning_group = planning_group;
    msg.request.get_current_state = msg.request.JOINT_STATE;
    if (!client_get_manipulation_target_.call(msg))
    {
      throw TEMOTO_ERRSTACK("Unable to reach robot_manager");
    }

    if (!msg.response.success)
    {
      throw TEMOTO_ERRSTACK("Unsuccessful attempt to invoke 'getEndEffPose'");
    }
    return msg.response.joint_values;
  }

 std::vector<std::string> getNamedTargets(const std::string& robot_name, const std::string& planning_group)
 {
    temoto_robot_manager::RobotGetNamedTargets msg; 
    msg.request.robot_name = robot_name;
    msg.request.planning_group = planning_group;
    if (!client_get_manipulation_named_targets_.call(msg))
    {
      throw TEMOTO_ERRSTACK("Unable to reach robot_manager");
    }
    if (!msg.response.success)
    {
      throw TEMOTO_ERRSTACK("Unsuccessful attempt to invoke 'getEndEffPose'");
    }    
    return msg.response.named_target_poses;
  }

  void navigationGoal(const std::string& robot_name
  , const geometry_msgs::PoseStamped& pose)
  {
    temoto_robot_manager::RobotNavigationGoal msg;
    msg.request.target_pose = pose;
    
    if (pose.header.frame_id.empty())
    {
      throw TEMOTO_ERRSTACK("Reference frame is not defined");
    }

    msg.request.target_pose.header.frame_id = pose.header.frame_id;
    msg.request.robot_name = robot_name;
    if (!client_navigation_goal_.call(msg))
    {
      throw TEMOTO_ERRSTACK("Unable to reach robot_manager");
    }

    if (!msg.response.success)
    {
      throw TEMOTO_ERRSTACK("Unsuccessful attempt to invoke 'navigationGoal'");
    }
  }

  void controlGripperPosition(const std::string& robot_name, const float& position)
  {
    temoto_robot_manager::RobotGripperControlPosition msg;    
    msg.request.robot_name = robot_name;
    msg.request.control = position;

    if (!client_gripper_control_position_.call(msg))
    {
      throw TEMOTO_ERRSTACK("Unable to reach robot_manager");
    }

    if (!msg.response.success)
    {
      throw TEMOTO_ERRSTACK("Unsuccessful attempt to invoke 'controlGripperPosition'");
    }
  }

  const std::string& getName() const
  {
    return rr_name_;
  }

  void statusInfoCb(RobotLoad srv_msg, temoto_resource_registrar::Status status_msg)
  try
  {
    TEMOTO_DEBUG_STREAM_("status info was received");
    TEMOTO_DEBUG_STREAM_(srv_msg.request);

    // auto robot_it = std::find_if(
    //   allocated_robots_.begin(),
    //   allocated_robots_.end(),
    //   [&](const RobotLoad& loaded_robot) -> bool {
    //     return loaded_robot.response.temoto_metadata.request_id == srv_msg.response.temoto_metadata.request_id;
    //   });

    // if (robot_it != allocated_robots_.end())
    // {
    //   TEMOTO_WARN_STREAM_("Sending a request to unload the failed robot ...");
    //   resource_registrar_->unload(srv_name::MANAGER
    //   , robot_it->response.temoto_metadata.request_id);

    //   TEMOTO_DEBUG_("Requesting to load the same robot again ...");

    //   // this call automatically updates the response in allocated robots vec
    //   resource_registrar_->call<RobotLoad>(srv_name::MANAGER
    //   , srv_name::SERVER_LOAD
    //   , *robot_it
    //   , std::bind(&RobotManagerInterface::statusInfoCb, this, std::placeholders::_1, std::placeholders::_2));
    // }
    // else
    // {
    //   TEMOTO_WARN_("The status info regards a resource that was not allocated from this interface.");
    // }
  }
  catch (resource_registrar::TemotoErrorStack e)
  {
    throw FWD_TEMOTO_ERRSTACK(e);
  }

  ~RobotManagerInterface()
  {
    // Shutdown robot manager clients.
    client_load_.shutdown();
    client_plan_.shutdown();
    client_exec_.shutdown();
    client_viz_info_.shutdown();
    client_set_manipulation_target_.shutdown();
    client_get_manipulation_target_.shutdown();
    client_navigation_goal_.shutdown();
    client_gripper_control_position_.shutdown();

    TEMOTO_DEBUG_("RobotManagerInterface destroyed.");
  }

private:

  std::string rr_name_;
  std::string unique_suffix_;
  bool initialized_;
  std::vector<RobotLoad> allocated_robots_;

  ros::NodeHandle nh_;
  ros::ServiceClient client_load_;
  ros::ServiceClient client_plan_;
  ros::ServiceClient client_exec_;
  ros::ServiceClient client_viz_info_;
  ros::ServiceClient client_set_manipulation_target_;  
  ros::ServiceClient client_get_manipulation_target_;
  ros::ServiceClient client_get_manipulation_named_targets_;
  ros::ServiceClient client_navigation_goal_;
  ros::ServiceClient client_gripper_control_position_;
  ros::ServiceClient client_get_robot_config_; 

  std::unique_ptr<temoto_resource_registrar::ResourceRegistrarRos1> resource_registrar_;
};
} // namespace
#endif