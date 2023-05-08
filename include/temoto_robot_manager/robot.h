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

#ifndef TEMOTO_ROBOT_MANAGER__ROBOT_H
#define TEMOTO_ROBOT_MANAGER__ROBOT_H

#include "class_loader/class_loader.hpp"
#include "temoto_process_manager/process_manager_services.hpp"
#include "rr/ros1_resource_registrar.h"
#include "temoto_robot_manager/robot_config.h"
#include "temoto_robot_manager/robot_manager.h"
#include "temoto_robot_manager/robot_features.h"
#include "temoto_robot_manager/GripperControl.h"
#include "temoto_robot_manager/custom_plugin_base.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string>
#include <map>
#include <vector>
#include <memory>

namespace temoto_robot_manager
{

RmCustomFeedbackWrap : RmCustomFeedback
{
  std::string robot_name;
  std::string custom_feature_name;
  std::string request_id;
};

typedef std::function<void(const RmCustomFeedbackWrap&)> CustomFeatureUpdateCb;

class Robot
{
public:

  Robot(RobotConfigPtr config_
  , const std::string& resource_id
  , temoto_resource_registrar::ResourceRegistrarRos1& resource_registrar
  , CustomFeatureUpdateCb custom_feature_update_cb_);

  virtual ~Robot();
  void load();
  void recover(const std::string& parent_query_id);
  void addPlanningGroup(const std::string& planning_group_name);
  void removePlanningGroup(const std::string& planning_group_name);
  void planManipulationPath(const std::string& planning_group_name, const geometry_msgs::PoseStamped& target_pose);
  void planManipulationPath(const std::string& planning_group_name, const std::string& named_target);
  void planManipulationPath(const std::string& planning_group_name, const std::vector<double> &joint_state_target);

  void executeManipulationPath();
  
  geometry_msgs::PoseStamped getManipulationTarget(const std::string& planning_group_name);
  std::vector<double> getCurrentJointValues(const std::string& planning_group_name);
  std::vector<std::string> getNamedTargetPoses(const std::string& planning_group_name);
  
  void goalNavigation(const geometry_msgs::PoseStamped& target_pose);
  void controlGripper(const std::string& robot_name, const float position);

  void invokeCustomFeature(const std::string& custom_feature_name, const RmCustomRequest& request);
  void preemptCustomFeature(const std::string& custom_feature_name);
  
  std::string getName() const
  {
    return config_->getName();
  }

  RobotConfigPtr getConfig()
  {
    return config_;
  }

  bool isLocal() const;

  bool isRobotOperational() const;

  bool isInError() const;

  // return all the information required to visualize this robot
  std::string getVizInfo();

  void setResourceId(const std::string& resource_id);

private:
  void waitForHardware();
  void loadHardware();
  void loadUrdf();
  void loadManipulationController();
  void loadManipulationDriver();
  void loadNavigationController();
  void loadNavigationDriver();
  void loadGripperController();
  void loadGripperDriver();
  void loadCustomController(const std::string& feature_name);
  void loadCustomDriver(const std::string& feature_name);

  temoto_process_manager::LoadProcess rosExecute(const std::string& package_name
  , const std::string& executable
  , const std::string& args = "");

  void resourceStatusCb(temoto_process_manager::LoadProcess srv_msg
  , temoto_resource_registrar::Status status_msg);

  void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);

  void waitForParam(const std::string& param);
  void waitForTopic(const std::string& topic);
  bool isTopicAvailable(const std::string& topic);
  void setRobotOperational(bool robot_operational);
  void setInError(bool state_in_error);

  ros::NodeHandle nh_;
  std::string robot_resource_id_;
  bool robot_operational_;
  bool robot_loaded_;
  bool state_in_error_;
  mutable std::recursive_mutex robot_operational_mutex_;
  mutable std::recursive_mutex robot_state_in_error_mutex_;
  RobotConfigPtr config_;
  temoto_resource_registrar::ResourceRegistrarRos1& resource_registrar_;

  // Manipulation related
  bool is_plan_valid_;
  moveit::planning_interface::MoveGroupInterface::Plan last_plan;
  std::map<std::string, std::unique_ptr<moveit::planning_interface::MoveGroupInterface>> planning_groups_;

  // Navigation related
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  ros::Subscriber localized_pose_sub_;
  geometry_msgs::PoseWithCovarianceStamped current_pose_navigation_;

  // Custom related
  struct CustomPluginHelper
  {
    std::shared_ptr<CustomPluginBase> plugin;
    std::shared_ptr<class_loader::ClassLoader> class_loader;
  };
  std::map<std::string, CustomPluginHelper> custom_feature_plugins_;
  CustomFeatureUpdateCb custom_feature_update_cb_ = NULL;

  ros::ServiceClient client_gripper_control_;
};
}

#endif


