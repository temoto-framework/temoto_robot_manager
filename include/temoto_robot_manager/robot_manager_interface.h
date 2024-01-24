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

#include "rr/ros1_resource_registrar.h"
#include "temoto_robot_manager/robot_manager_services.h"
#include "yaml-cpp/yaml.h"
#include <vector>
#include <string>
#include <ctime>
#include <optional>

namespace temoto_robot_manager
{
class RobotManagerInterface
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
      
      client_cancel_navigation_goal_ =
        nh_.serviceClient<RobotCancelNavigationGoal>(srv_name::SERVER_CANCEL_NAVIGATION_GOAL);

      client_custom_request_ = 
        nh_.serviceClient<CustomRequest>(channels::custom::REQUEST);
      client_custom_request_preempt_ = 
        nh_.serviceClient<CustomRequestPreempt>(channels::custom::PREEMPT);
      custom_feedback_ = nh_.subscribe(channels::custom::FEEDBACK, 1, &RobotManagerInterface::customFeedback, this);

      navigation_feedback_ = nh_.subscribe(srv_name::NAVIGATION_FEEDBACK, 1, &RobotManagerInterface::navigationFeedbackCb, this);
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

  bool invokeCustomFeature(CustomRequest& custom_request)
  {
    custom_request.request.client_id = rr_name_;

    if (!client_custom_request_.call(custom_request))
    {
      custom_request.response.accepted = false;
      custom_request.response.message = "Unable to reach CustomRequest server"; 
      return false;
    }

    if (custom_request.response.accepted)
    {
      custom_request.request.request_id = custom_request.response.request_id;
      std::lock_guard<std::mutex> lock(custom_queries_mutex_);
      ongoing_custom_queries_.insert({custom_request.response.request_id, CustomQuery(custom_request)});
    }
    
    return custom_request.response.accepted;
  }

  std::optional<CustomFeedback> getCustomFeatureFeedback(const std::string& request_id)
  {
    std::lock_guard<std::mutex> lock(custom_queries_mutex_);
    auto ongoing_query_it = ongoing_custom_queries_.find(request_id);

    if (ongoing_query_it == ongoing_custom_queries_.end())
    {
      return {};
      //throw TEMOTO_ERRSTACK("Could not find the request in the list of ongoing requests");
    }

    if (ongoing_query_it->second.feedback.status == CustomFeedback::FINISHED)
    {
      auto feedback = ongoing_query_it->second.feedback;
      ongoing_custom_queries_.erase(ongoing_query_it);
      return feedback;
    }
    else
    {
      return ongoing_query_it->second.feedback;
    }
  }

  bool preemptCustomFeature(const std::string& request_id)
  {
    std::lock_guard<std::mutex> lock(custom_queries_mutex_);
    auto ongoing_query_it = ongoing_custom_queries_.find(request_id);

    if (ongoing_query_it == ongoing_custom_queries_.end())
    {
      throw TEMOTO_ERRSTACK("Could not find the request in the list of ongoing requests");
    }

    CustomRequestPreempt preempt_srv_msg;
    const auto& ongoing_req = ongoing_query_it->second.request.request;

    preempt_srv_msg.request.custom_feature_name = ongoing_req.custom_feature_name;
    preempt_srv_msg.request.robot_name = ongoing_req.robot_name;
    preempt_srv_msg.request.priority = ongoing_req.priority;
    preempt_srv_msg.request.request_id = ongoing_query_it->first;

    if (!client_custom_request_preempt_.call(preempt_srv_msg))
    {
      throw TEMOTO_ERRSTACK("Unable to reach the CustomRequestPreempt server");
    }

    return preempt_srv_msg.response.accepted;
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

  // TODO: Erase?
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

  bool navigationGoal(RobotNavigationGoal& goal)
  {
    if (goal.request.target_pose.header.frame_id.empty())
    {
      throw TEMOTO_ERRSTACK("Reference frame is not defined");
    }

    if (!client_navigation_goal_.call(goal))
    {
      throw TEMOTO_ERRSTACK("Unable to reach robot_manager");
    }

    if (!goal.response.success)
    {
      TEMOTO_INFO_("Goal response not success");
    }

    ongoing_navigation_queries_.insert({goal.request.robot_name, NavigationQuery(goal)});

    // wait to finish execution
    while (getNavigationFeedback(goal.request.robot_name)->status != NavigationFeedback::FINISHED
          && getNavigationFeedback(goal.request.robot_name)->status != NavigationFeedback::CANCELLED)
    {
      ros::Duration(1).sleep();
    }

    auto ongoing_query_it = ongoing_navigation_queries_.find(goal.request.robot_name);
    if (ongoing_query_it != ongoing_navigation_queries_.end())
    {
      ongoing_navigation_queries_.erase(ongoing_query_it);
    }

    if (getNavigationFeedback(goal.request.robot_name)->status == NavigationFeedback::CANCELLED)
    {
      return false;
    }
    return goal.response.success;
  }

  std::optional<NavigationFeedback> getNavigationFeedback(const std::string& robot_name)
  {
    std::lock_guard<std::mutex> lock(navigation_queries_mutex_);
    auto ongoing_query_it = ongoing_navigation_queries_.find(robot_name);

    if (ongoing_query_it == ongoing_navigation_queries_.end())
    {
      TEMOTO_INFO_STREAM_("There's no ongoing query for the " << robot_name << " robot");
      return {};
      //throw TEMOTO_ERRSTACK("Could not find the request in the list of ongoing requests");
    }

    return ongoing_query_it->second.feedback;
  }

  bool cancelNavigationGoal(RobotCancelNavigationGoal& cancel_goal)
  {
    if (!client_cancel_navigation_goal_.call(cancel_goal))
    {
      throw TEMOTO_ERRSTACK("Unable to reach the CancelNavigationGoal server");
    }

    return cancel_goal.response.result;
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
    client_cancel_navigation_goal_.shutdown();
    client_gripper_control_position_.shutdown();

    TEMOTO_DEBUG_("RobotManagerInterface destroyed.");
  }

private:

  struct CustomQuery
  {
    CustomQuery(const CustomRequest& cr) : request{cr}{}
    CustomRequest request;
    CustomFeedback feedback;
  };

  struct NavigationQuery
  {
    NavigationQuery(const RobotNavigationGoal& nr) : request{nr}{}
    RobotNavigationGoal request;
    NavigationFeedback feedback;
  };

  void customFeedback(const CustomFeedback& msg)
  {
    std::lock_guard<std::mutex> lock(custom_queries_mutex_);
    auto ongoing_query_it = ongoing_custom_queries_.find(msg.request_id);

    if (ongoing_query_it != ongoing_custom_queries_.end())
    {
      ongoing_query_it->second.feedback = msg;
    }
  }

  void navigationFeedbackCb(const NavigationFeedback& msg)
  {
    std::lock_guard<std::mutex> lock(custom_queries_mutex_);
    auto ongoing_nav_query_it = ongoing_navigation_queries_.find(msg.robot_name);
    if (ongoing_nav_query_it != ongoing_navigation_queries_.end())
    {
      ongoing_nav_query_it->second.feedback = msg;
    }
    return;
  }

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
  ros::ServiceClient client_cancel_navigation_goal_;
  ros::ServiceClient client_gripper_control_position_;
  ros::ServiceClient client_get_robot_config_;

  ros::ServiceClient client_custom_request_;
  ros::ServiceClient client_custom_request_preempt_;
  ros::Subscriber custom_feedback_;
  std::mutex custom_queries_mutex_;
  std::map<std::string, CustomQuery> ongoing_custom_queries_;

  ros::Subscriber navigation_feedback_;
  std::mutex navigation_queries_mutex_;
  std::map<std::string, NavigationQuery> ongoing_navigation_queries_;

  std::unique_ptr<temoto_resource_registrar::ResourceRegistrarRos1> resource_registrar_;
};

} // namespace
#endif