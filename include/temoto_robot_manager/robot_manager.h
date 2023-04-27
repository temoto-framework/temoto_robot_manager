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

#ifndef TEMOTO_ROBOT_MANAGER__ROBOT_MANAGER_H
#define TEMOTO_ROBOT_MANAGER__ROBOT_MANAGER_H

#include "rr/ros1_resource_registrar.h"
#include "temoto_core/common/base_subsystem.h"
#include "temoto_core/trr/config_synchronizer.h"
#include "temoto_core/ConfigSync.h"
#include "temoto_process_manager/process_manager_services.hpp"
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

namespace temoto_robot_manager
{
// Forward declaration
class Robot;

typedef std_msgs::String PayloadType;

class RobotManager : public temoto_core::BaseSubsystem
{
public:
  RobotManager(const std::string& config_base_path, bool restore_from_catalog);

  const std::string& getName() const
  {
    return subsystem_name_;
  }

private:

  /**
   * @brief Restores the state of the Robot Manager via RR Catalog
   * 
   */
  void restoreState();

  /**
   * @brief Callback for loading a robot
   * @param Request that specifies the robot's parameters
   * @param Returns which robot got loaded
   */
  void loadCb(RobotLoad::Request& req, RobotLoad::Response& res);

  /**
   * @brief Callback for unloading a robot
   * @param Request that specifies which robot to unload
   * @param Returns status information
   */
  void unloadCb(RobotLoad::Request& req, RobotLoad::Response& res);

  /**
   * @brief Service callback that plans using moveit
   * @param A gesture specifier message
   * @param Returns a topic where the requested gesture messages
   * are going to be published
   * @return
   */
  bool planManipulationPathCb(RobotPlanManipulation::Request& req, RobotPlanManipulation::Response& res);

  bool execManipulationPathCb(RobotExecutePlan::Request& req, RobotExecutePlan::Response& res);

  bool getManipulationTargetCb(RobotGetTarget::Request& req, RobotGetTarget::Response& res);

  bool getManipulationNamedTargetsCb(RobotGetNamedTargets::Request& req, RobotGetNamedTargets::Response& res);

  bool goalNavigationCb(RobotNavigationGoal::Request& req, RobotNavigationGoal::Response& res);

  bool gripperControlPositionCb(RobotGripperControlPosition::Request& req, RobotGripperControlPosition::Response& res);

  bool getRobotConfigCb(RobotGetConfig::Request& req, RobotGetConfig::Response& res);
  
  bool setModeCb(RobotSetMode::Request& req, RobotSetMode::Response& res);

  void syncCb(const temoto_core::ConfigSync& msg, const PayloadType& payload);

  void advertiseConfig(RobotConfigPtr config);

  void advertiseConfigs(RobotConfigs configs);

  bool customFeatureCb(CustomRequest::Request& req, CustomRequest::Response& res);

  RobotConfigs parseRobotConfigs(const YAML::Node& config);
  
  RobotConfigs parseRobotConfigs(const YAML::Node& config, RobotConfigs configs);  

  RobotConfigPtr findRobot(const std::string& robot_name, const RobotConfigs& robot_infos);

  bool getVizInfoCb(RobotGetVizInfo::Request& req, RobotGetVizInfo::Response& res);

  void resourceStatusCb(RobotLoad srv_msg, temoto_resource_registrar::Status status_msg);

  void readRobotDescription(const std::string& path_file_rob_description);

  void findRobotDescriptionFiles(boost::filesystem::path current_dir);

  std::shared_ptr<Robot> findLoadedRobot(const std::string& robot_name);

  typedef std::shared_ptr<Robot> RobotPtr;  
  
  std::vector<RobotPtr> loaded_robots_;
  RobotConfigs local_configs_;
  RobotConfigs remote_configs_;

  geometry_msgs::PoseStamped default_target_pose_;

  ros::NodeHandle nh_;
  ros::ServiceServer server_plan_;
  ros::ServiceServer server_exec_;
  ros::ServiceServer server_get_viz_cfg_;
  ros::ServiceServer server_set_manipulation_target_;
  ros::ServiceServer server_get_manipulation_target_;
  ros::ServiceServer server_get_manipulation_named_targets_;
  ros::ServiceServer server_set_mode_;
  ros::ServiceServer server_navigation_goal_;
  ros::ServiceServer server_gripper_control_position_;
  ros::ServiceServer server_get_robot_config_;
  ros::ServiceServer server_custom_feature_;

  ros::ServiceClient client_plan_;
  ros::ServiceClient client_exec_;
  ros::ServiceClient client_get_viz_cfg_;
  ros::ServiceClient client_set_manipulation_target_;
  ros::ServiceClient client_get_manipulation_target_;
  ros::ServiceClient client_get_manipulation_named_targets_;
  ros::ServiceClient client_set_mode_;
  ros::ServiceClient client_navigation_goal_;
  ros::ServiceClient client_gripper_control_position_;

  ros::Publisher pub_custom_feature_feedback_;
  
  // Keeps robot_infos in sync with other managers
  temoto_core::trr::ConfigSynchronizer<RobotManager, PayloadType> config_syncer_;

  temoto_resource_registrar::ResourceRegistrarRos1 resource_registrar_;
  temoto_resource_registrar::Configuration rr_catalog_config_;

  tf2_ros::TransformListener tf2_listener;
  tf2_ros::Buffer tf2_buffer;
};
}

#endif

