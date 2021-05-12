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

#include "temoto_core/common/base_subsystem.h"
#include "temoto_er_manager/temoto_er_manager_services.h"
#include "rr/ros1_resource_registrar.h"
#include "temoto_robot_manager/robot_config.h"
#include "temoto_robot_manager/robot_manager.h"
#include "temoto_robot_manager/robot_features.h"
#include "temoto_robot_manager/GripperControl.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>
#include <map>
#include <vector>

namespace temoto_robot_manager
{

// Forward declaration
class RobotManager;

class Robot : public temoto_core::BaseSubsystem
{
public:
  Robot(RobotConfigPtr config_
  , temoto_resource_registrar::ResourceRegistrarRos1& resource_registrar
  , temoto_core::BaseSubsystem& b);

  virtual ~Robot();
  void addPlanningGroup(const std::string& planning_group_name);
  void removePlanningGroup(const std::string& planning_group_name);
  void planManipulationPath(std::string& planning_group_name, const geometry_msgs::PoseStamped& target_pose);
  void planManipulationPath(std::string& planning_group_name, const std::string& named_target);  

  void executeManipulationPath();
  
  geometry_msgs::Pose getManipulationTarget();
  void goalNavigation(const std::string& reference_frame, const geometry_msgs::PoseStamped& target_pose);
  void controlGripper(const std::string& robot_name, const float position);
  
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

  // return all the information required to visualize this robot
  std::string getVizInfo();

  //bool hasResource(temoto_core::temoto_id::ID resource_id);

private:
  void waitForHardware();
  void load();
  void loadHardware();
  void loadUrdf();
  void loadManipulationController();
  void loadManipulationDriver();
  void loadNavigationController();
  void loadNavigationDriver();
  void loadGripperController();
  void loadGripperDriver();

  temoto_er_manager::LoadExtResource rosExecute(const std::string& package_name
  , const std::string& executable
  , const std::string& args = "");

  void resourceStatusCb(temoto_er_manager::LoadExtResource srv_msg
  , temoto_resource_registrar::Status status_msg);

  void waitForParam(const std::string& param);
  void waitForTopic(const std::string& topic);
  bool isTopicAvailable(const std::string& topic);
  void setRobotOperational(bool robot_operational);

  ros::NodeHandle nh_;
  bool robot_operational_;
  mutable std::recursive_mutex robot_operational_mutex_;
  RobotConfigPtr config_;
  temoto_resource_registrar::ResourceRegistrarRos1& resource_registrar_;

  // Manipulation related
  bool is_plan_valid_;
  moveit::planning_interface::MoveGroupInterface::Plan last_plan;
  std::map<std::string, std::unique_ptr<moveit::planning_interface::MoveGroupInterface>> planning_groups_;

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient_;

  ros::ServiceClient client_gripper_control_;
};
}

#endif


