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

#ifndef TEMOTO_ROBOT_MANAGER__ROBOT_MANAGER_INTERFACE_H
#define TEMOTO_ROBOT_MANAGER__ROBOT_MANAGER_INTERFACE_H

#include "temoto_core/rmp/resource_manager.h"
#include "temoto_core/common/temoto_id.h"
#include "temoto_core/common/console_colors.h"
#include "temoto_robot_manager/robot_manager_services.h"
#include <vector>
#include <string>

namespace robot_manager
{

template <class OwnerAction>
class RobotManagerInterface : public temoto_core::BaseSubsystem
{
public:
  RobotManagerInterface()
  {
    class_name_ = __func__;
  }

  void initialize(OwnerAction* action)
  {
    initializeBase(action);
    log_group_ = "interfaces." + action->getName();
    name_ = action->getName() + "/robot_manager_interface";
    std::string prefix = temoto_core::common::generateLogPrefix(log_subsys_, log_class_, __func__);

    // create resource manager
    resource_manager_ = std::unique_ptr<temoto_core::rmp::ResourceManager<RobotManagerInterface>>(
        new temoto_core::rmp::ResourceManager<RobotManagerInterface>(name_, this));

    // ensure that resource_manager was created
    validateInterface(prefix);

    // register status callback function
    // resource_manager_->registerStatusCb(&RobotManagerInterface::statusInfoCb);
    //client_load_ =
      //  nh_.serviceClient<temoto_robot_manager::RobotLoad>(robot_manager::srv_name::SERVER_LOAD);
    client_plan_ =
        nh_.serviceClient<temoto_robot_manager::RobotPlan>(robot_manager::srv_name::SERVER_PLAN);
    client_exec_ =
        nh_.serviceClient<temoto_robot_manager::RobotExecute>(robot_manager::srv_name::SERVER_EXECUTE);
    client_viz_info_ =
        nh_.serviceClient<temoto_robot_manager::RobotGetVizInfo>(robot_manager::srv_name::SERVER_GET_VIZ_INFO);
    client_set_target_ =
        nh_.serviceClient<temoto_robot_manager::RobotSetTarget>(robot_manager::srv_name::SERVER_SET_TARGET);
    client_get_target_ =
        nh_.serviceClient<temoto_robot_manager::RobotGetTarget>(robot_manager::srv_name::SERVER_GET_TARGET);
  }

  void loadRobot(std::string robot_name = "")
  {
    
    std::string prefix = temoto_core::common::generateLogPrefix(log_subsys_, log_class_, __func__);
  
    // Contact the "Context Manager", pass the gesture specifier and if successful, get
    // the name of the topic
    temoto_robot_manager::RobotLoad load_srvc;
    load_srvc.request.robot_name = robot_name;
  
    
    try
    {
      TEMOTO_INFO_STREAM(robot_manager::srv_name::MANAGER);
      TEMOTO_INFO_STREAM(robot_manager::srv_name::SERVER_LOAD);
      resource_manager_->template call<temoto_robot_manager::RobotLoad>(
          robot_manager::srv_name::MANAGER, robot_manager::srv_name::SERVER_LOAD, load_srvc);
          
    }
    catch(temoto_core::error::ErrorStack& error_stack)
    {
      //TEMOTO_INFO_STREAM("========== here ==========");
      //TEMOTO_INFO_STREAM(error_stack.size);
      throw FORWARD_ERROR(error_stack);
    }
  }

  void plan(std::string planning_group = "")
  {
    std::string prefix = temoto_core::common::generateLogPrefix(log_subsys_, log_class_, __func__);

    temoto_robot_manager::RobotPlan msg;
    msg.request.use_default_target = true;
    msg.request.planning_group = planning_group;

    if (!client_plan_.call(msg))
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == temoto_core::rmp::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
  }

  void plan(const geometry_msgs::PoseStamped& pose, std::string planning_group = "")
  {
    std::string prefix = temoto_core::common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_robot_manager::RobotPlan msg;
    msg.request.use_default_target = false;
    msg.request.target_pose = pose;
    msg.request.planning_group = planning_group;
    
    if (!client_plan_.call(msg))
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == temoto_core::rmp::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
  }

  void execute()
  {
    std::string prefix = temoto_core::common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_robot_manager::RobotExecute msg;
    if (!client_exec_.call(msg))
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == temoto_core::rmp::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
  }

  std::string getMoveitRvizConfig()
  {
    std::string prefix = temoto_core::common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_robot_manager::RobotGetVizInfo msg;
    if (!client_viz_info_.call(msg))
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == temoto_core::rmp::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
    return msg.response.info;
  }

  void setTarget(std::string object_name)
  {
    std::string prefix = temoto_core::common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());

    temoto_robot_manager::RobotSetTarget msg;
    msg.request.object_name = object_name;
    if (!client_set_target_.call(msg))
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Service call returned false.");
    }
    else if (msg.response.code == temoto_core::rmp::status_codes::FAILED)
    {
      throw FORWARD_ERROR(msg.response.error_stack);
    }
  }


// ====== Test - Function to get the pose of the eef respect to something ====

 geometry_msgs::Pose getEndEffPose(std::string object_name,std::string respect_to_link)
 {
    std::string prefix = temoto_core::common::generateLogPrefix(log_subsys_, log_class_, __func__);
    TEMOTO_DEBUG("%s", prefix.c_str());
    geometry_msgs::Pose pose;
    
    TEMOTO_INFO_STREAM("=====getEndEffPose====== " << object_name << respect_to_link);

    temoto_robot_manager::RobotGetTarget msg; 

    msg.request.ref_joint = object_name;
    msg.request.respect_to = respect_to_link;

    //client_get_target_.call(msg);
    TEMOTO_INFO_STREAM(client_get_target_.call(msg));
    TEMOTO_INFO_STREAM("=====GET TARGET ======");

    TEMOTO_INFO_STREAM(msg.response.pose);
    pose = msg.response.pose;    
    
     return pose;
  }
  
// END TEST FUNCTION
  
  

  /**
   * @brief validateInterface()
   * @param sensor_type
   */
  void validateInterface(std::string& log_prefix)
  {
    if (!resource_manager_)
    {
      throw CREATE_ERROR(temoto_core::error::Code::UNINITIALIZED, "Interface is not initalized.");
    }
  }

  const std::string& getName() const
  {
    return name_;
  }

  ~RobotManagerInterface()
  {
    // Shutdown robot manager clients.
    client_load_.shutdown();
    client_plan_.shutdown();
    client_exec_.shutdown();
    client_viz_info_.shutdown();
    client_set_target_.shutdown();

    TEMOTO_DEBUG("RobotManagerInterface destroyed.");
  }

private:
  std::string name_;
  std::string log_class_, log_subsys_, log_group_;

  ros::NodeHandle nh_;
  ros::ServiceClient client_load_;
  ros::ServiceClient client_plan_;
  ros::ServiceClient client_exec_;
  ros::ServiceClient client_viz_info_;
  ros::ServiceClient client_set_target_;
  
  ros::ServiceClient client_get_target_;


  std::unique_ptr<temoto_core::rmp::ResourceManager<RobotManagerInterface>> resource_manager_;
};

} // namespace
#endif
