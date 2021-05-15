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
  , has_owner_(false)
  , initialized_(false)
  {
    class_name_ = __func__;
    if (initialize_interface)
    {
      initialize();
    }
  }

  unsigned int createID()
  {
    std::srand(std::time(nullptr));
    return std::rand();
  }

  void initialize(const BaseSubsystem& owner)
  {
    if (!initialized_)
    {
      initializeBase(owner);
      log_group_ = "interfaces." + owner.subsystem_name_;
      rr_name_ = owner.class_name_ + "/" + class_name_ + "_" + unique_suffix_;
      has_owner_ = true;
      initialize();
    }
    else
    {
      TEMOTO_WARN_STREAM_("The Robot Manager interface is already initialized");
    }
  }

  void initialize()
  {
    if (!initialized_)
    {
      if (!has_owner_)
      {
        rr_name_ = class_name_ + "_" + unique_suffix_;
      }
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

  YAML::Node getRobotConfig(const std::string& robot_name)
  {
    temoto_robot_manager::RobotGetConfig msg;
    msg.request.robot_name = robot_name;
    if (!client_get_robot_config_.call(msg))
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == temoto_core::trr::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
    else
    {
      try
      {
        YAML::Node robot_config = YAML::Load(msg.response.robot_config);
        robot_config["robot_absolute_namespace"] = msg.response.robot_absolute_namespace;
        return robot_config;
      }
      catch(const std::exception& e)
      {
        throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, e.what());
      }
    }
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
  catch(temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }


  void planManipulation(const std::string& robot_name, std::string planning_group = "")
  {
    temoto_robot_manager::RobotPlanManipulation msg;
    msg.request.use_default_target = true;
    msg.request.use_named_target = false;
    msg.request.planning_group = planning_group;
    msg.request.robot_name = robot_name;
    if (!client_plan_.call(msg))
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == temoto_core::trr::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
  }

  void planManipulation(const std::string& robot_name
  , const std::string& planning_group
  , const geometry_msgs::PoseStamped& pose)
  {
    temoto_robot_manager::RobotPlanManipulation msg;
    msg.request.use_default_target = false;
    msg.request.use_named_target = false;
    msg.request.target_pose = pose;
    msg.request.planning_group = planning_group;
    msg.request.robot_name = robot_name;
    
    if (!client_plan_.call(msg))
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == temoto_core::trr::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
  }

  void planManipulation(const std::string& robot_name
  , const std::string& planning_group
  , const std::string& named_target_pose)
  {
    temoto_robot_manager::RobotPlanManipulation msg;
    msg.request.use_default_target = false;
    msg.request.use_named_target = true;
    msg.request.named_target = named_target_pose;
    msg.request.planning_group = planning_group;
    msg.request.robot_name = robot_name;
    
    if (!client_plan_.call(msg))
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == temoto_core::trr::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
  }

  void executePlan(const std::string& robot_name)
  {
    temoto_robot_manager::RobotExecutePlan msg;
    msg.request.robot_name = robot_name;
    if (!client_exec_.call(msg))
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == temoto_core::trr::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
  }

  std::string getMoveitRvizConfig()
  {
    temoto_robot_manager::RobotGetVizInfo msg;
    if (!client_viz_info_.call(msg))
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == temoto_core::trr::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
    return msg.response.info;
  }

  void setTarget(const std::string& object_name)
  {
    temoto_robot_manager::RobotSetTarget msg;
    msg.request.object_name = object_name;
    if (!client_set_manipulation_target_.call(msg))
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == temoto_core::trr::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
  }

 geometry_msgs::Pose getEndEffPose(const std::string& robot_name)
 {
    geometry_msgs::Pose pose;
    temoto_robot_manager::RobotGetTarget msg; 
    msg.request.robot_name = robot_name;
    client_get_manipulation_target_.call(msg);
    pose = msg.response.pose;
    
    return pose;
  }

  bool navigationGoal(const std::string& robot_name
  , const std::string& reference_frame
  , const geometry_msgs::PoseStamped& pose)
  {
    temoto_robot_manager::RobotNavigationGoal msg; 
    msg.request.reference_frame = reference_frame;
    msg.request.target_pose = pose;
    msg.request.robot_name = robot_name;
    if (client_navigation_goal_.call(msg))
    {
      TEMOTO_DEBUG_("The server was reached");
      return msg.response.success;
    }
    else
    {
      TEMOTO_ERROR("Failed to reach the server");
      return false;
    }

  }

  void controlGripperPosition(const std::string& robot_name, const float& position)
  {
    temoto_robot_manager::RobotGripperControlPosition msg;    
    msg.request.robot_name = robot_name;
    msg.request.control = position;

    if (client_gripper_control_position_.call(msg))
    {
      TEMOTO_DEBUG_("Call to move the gripper was successful");
    }
    else
    {
      TEMOTO_ERROR("Failed to reach the server for gripper control"); 
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
  catch (temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
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
  bool has_owner_;
  bool initialized_;
  std::vector<RobotLoad> allocated_robots_;

  ros::NodeHandle nh_;
  ros::ServiceClient client_load_;
  ros::ServiceClient client_plan_;
  ros::ServiceClient client_exec_;
  ros::ServiceClient client_viz_info_;
  ros::ServiceClient client_set_manipulation_target_;  
  ros::ServiceClient client_get_manipulation_target_;
  ros::ServiceClient client_navigation_goal_;
  ros::ServiceClient client_gripper_control_position_;
  ros::ServiceClient client_get_robot_config_; 

  std::unique_ptr<temoto_resource_registrar::ResourceRegistrarRos1> resource_registrar_;
};
} // namespace
#endif

