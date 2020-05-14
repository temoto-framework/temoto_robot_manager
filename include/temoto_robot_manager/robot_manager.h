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

#ifndef TEMOTO_ROBOT_MANAGER__ROBOT_MANAGER_H
#define TEMOTO_ROBOT_MANAGER__ROBOT_MANAGER_H

#include "temoto_core/common/temoto_id.h"
#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/trr/resource_registrar.h"
#include "temoto_core/trr/config_synchronizer.h"
#include "temoto_core/ConfigSync.h"
#include "temoto_er_manager/temoto_er_manager_services.h"
#include "temoto_robot_manager/robot_manager_services.h"
#include "temoto_robot_manager/robot.h"
#include "temoto_robot_manager/robot_config.h"
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <mutex>
#include <vector>
#include <map>
#include <boost/filesystem/operations.hpp>

namespace robot_manager
{
// Forward declaration
class Robot;

typedef std_msgs::String PayloadType;

class RobotManager : public temoto_core::BaseSubsystem
{
public:
  RobotManager();

  const std::string& getName() const
  {
    return subsystem_name_;
  }

private:

  /**
   * @brief Callback for loading a robot
   * @param Request that specifies the robot's parameters
   * @param Returns which robot got loaded
   */
  void loadCb(temoto_robot_manager::RobotLoad::Request& req, temoto_robot_manager::RobotLoad::Response& res);

  /**
   * @brief Callback for unloading a robot
   * @param Request that specifies which robot to unload
   * @param Returns status information
   */
  void unloadCb(temoto_robot_manager::RobotLoad::Request& req, temoto_robot_manager::RobotLoad::Response& res);

  /**
   * @brief Service callback that plans using moveit
   * @param A gesture specifier message
   * @param Returns a topic where the requested gesture messages
   * are going to be published
   * @return
   */
  bool planManipulationPathCb(temoto_robot_manager::RobotPlanManipulation::Request& req, temoto_robot_manager::RobotPlanManipulation::Response& res);

  /**
   * @brief Service that executes the moveit plan
   * @param LoadGesture request message
   * @param LoadGesture response message
   * @return
   */

  bool execManipulationPathCb(temoto_robot_manager::RobotExecutePlan::Request& req, temoto_robot_manager::RobotExecutePlan::Response& res);

  bool getManipulationTargetCb(temoto_robot_manager::RobotGetTarget::Request& req, temoto_robot_manager::RobotGetTarget::Response& res);

  bool goalNavigationCb(temoto_robot_manager::RobotGoal::Request& req, temoto_robot_manager::RobotGoal::Response& res);

  bool gripperControlPositionCb(temoto_robot_manager::RobotGripperControlPosition::Request& req, temoto_robot_manager::RobotGripperControlPosition::Response& res);

  bool getRobotConfigCb(temoto_robot_manager::RobotGetConfig::Request& req, temoto_robot_manager::RobotGetConfig::Response& res);
  
  bool setModeCb(temoto_robot_manager::RobotSetMode::Request& req, temoto_robot_manager::RobotSetMode::Response& res);

  void syncCb(const temoto_core::ConfigSync& msg, const PayloadType& payload);

  void advertiseConfig(RobotConfigPtr config);

  void advertiseConfigs(RobotConfigs configs);

  RobotConfigs parseRobotConfigs(const YAML::Node& config);
  
  RobotConfigs parseRobotConfigs(const YAML::Node& config, RobotConfigs configs);  

  RobotConfigPtr findRobot(const std::string& robot_name, const RobotConfigs& robot_infos);

  bool getVizInfoCb(temoto_robot_manager::RobotGetVizInfo::Request& req,
                    temoto_robot_manager::RobotGetVizInfo::Response& res);

  void statusInfoCb(temoto_core::ResourceStatus& srv);

  void loadLocalRobot(RobotConfigPtr info_ptr, temoto_core::temoto_id::ID resource_id);

  void readRobotDescription(const std::string& path_file_rob_description);

  void findRobotDescriptionFiles(boost::filesystem::path current_dir);

  typedef std::shared_ptr<Robot> RobotPtr;
  typedef std::map<temoto_core::temoto_id::ID, RobotPtr> Robots;
  RobotPtr active_robot_;
  Robots loaded_robots_;
  RobotConfigs local_configs_;
  RobotConfigs remote_configs_;

  std::string mode_;
  geometry_msgs::PoseStamped default_target_pose_;

  ros::NodeHandle nh_;
  ros::ServiceServer server_plan_;
  ros::ServiceServer server_exec_;
  ros::ServiceServer server_get_viz_cfg_;
  ros::ServiceServer server_set_manipulation_target_;
  ros::ServiceServer server_get_manipulation_target_;
  ros::ServiceServer server_set_mode_;
  ros::ServiceServer server_navigation_goal_;
  ros::ServiceServer server_gripper_control_position_;
  ros::ServiceServer server_get_robot_config_;

  ros::ServiceClient client_plan_;
  ros::ServiceClient client_exec_;
  ros::ServiceClient client_get_viz_cfg_;
  ros::ServiceClient client_set_manipulation_target_;
  ros::ServiceClient client_get_manipulation_target_;
  ros::ServiceClient client_set_mode_;
  ros::ServiceClient client_navigation_goal_;
  ros::ServiceClient client_gripper_control_position_;

  ros::Subscriber target_pose_sub_;
  // temoto_robot_manager::LoadGesture hand_srv_msg_;
  
  // Keeps robot_infos in sync with other managers
  temoto_core::trr::ConfigSynchronizer<RobotManager, PayloadType> config_syncer_;

  // Resource manager for contacting process manager
  temoto_core::trr::ResourceRegistrar<RobotManager> resource_registrar_;
  std::mutex default_pose_mutex_;

  tf2_ros::TransformListener tf2_listener;
  tf2_ros::Buffer tf2_buffer;

  ros::Publisher marker_publisher_;
};
}

#endif

