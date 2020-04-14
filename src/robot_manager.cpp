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

#include "ros/package.h"
#include "temoto_core/temoto_error/temoto_error.h"
#include "temoto_robot_manager/robot_manager.h"
#include "temoto_er_manager/temoto_er_manager_services.h"
#include <boost/filesystem/operations.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>

namespace robot_manager
{
RobotManager::RobotManager()
  : temoto_core::BaseSubsystem("robot_manager", temoto_core::error::Subsystem::ROBOT_MANAGER, __func__)
  , resource_registrar_(srv_name::MANAGER, this)
  , config_syncer_(srv_name::MANAGER, srv_name::SYNC_TOPIC, &RobotManager::syncCb, this)
  , mode_(modes::AUTO)
  , tf2_listener(tf2_buffer)
{

  //\TODO:Remove, deprecated
  log_class_ = this->class_name_;
  log_subsys_ = this->subsystem_name_;

  // Start the server for loading/unloading robots as resources
  resource_registrar_.addServer<temoto_robot_manager::RobotLoad>(srv_name::SERVER_LOAD, &RobotManager::loadCb,
                                                   &RobotManager::unloadCb);
  resource_registrar_.registerStatusCb(&RobotManager::statusInfoCb);

  // Ask remote robot managers to send their robot config
  config_syncer_.requestRemoteConfigs();

  // Fire up additional servers for performing various actions on a robot.
  server_plan_ =
      nh_.advertiseService(robot_manager::srv_name::SERVER_PLAN, &RobotManager::planManipulationPathCb, this);
  server_exec_ =
      nh_.advertiseService(robot_manager::srv_name::SERVER_EXECUTE, &RobotManager::execManipulationPathCb, this);
  server_get_viz_cfg_ = nh_.advertiseService(robot_manager::srv_name::SERVER_GET_VIZ_INFO,
                                             &RobotManager::getVizInfoCb, this);
  server_set_manipulation_target_ = nh_.advertiseService(robot_manager::srv_name::SERVER_SET_MANIPULATION_TARGET,
                                            &RobotManager::setManipulationTargetCb, this);
  server_set_mode_ = nh_.advertiseService(robot_manager::srv_name::SERVER_SET_MODE,
                                          &RobotManager::setModeCb, this);
  
  server_get_manipulation_target_ = nh_.advertiseService(robot_manager::srv_name::SERVER_GET_MANIPULATION_TARGET, 
                                            &RobotManager::getManipulationTargetCb, this);

  server_navigation_goal_ = nh_.advertiseService(robot_manager::srv_name::SERVER_NAVIGATION_GOAL, 
                                            &RobotManager::goalNavigationCb, this);

  server_gripper_control_position_ = nh_.advertiseService(robot_manager::srv_name::SERVER_GRIPPER_CONTROL_POSITION, 
                                            &RobotManager::gripperControlPositionCb, this);


//TODO: TEMPORARY, REMOVE!
//      seq: 0
//      stamp: 
//        secs: 0
//        nsecs:         0
//      frame_id: "/base_link"
//    pose: 
//      position: 
//        x: 0.539124787814
//        y: -0.592158374795
//        z: 0.717273819597
//      orientation: 
//        x: 0.383115589721
//        y: 0.0150255505036
//        z: -0.0361555479561
//        w: 0.922870226032  
  std::string marker_topic = temoto_core::common::getAbsolutePath("world_to_target_marker");
  // Advertise the marker topic
  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(marker_topic, 10);

//Set the initial default stamped pose in the world frame, until target tracking is not set.
  default_target_pose_.header.stamp = ros::Time::now();
  default_target_pose_.header.frame_id = "world";
  default_target_pose_.pose.position.x = 0.539124787814;
  default_target_pose_.pose.position.y = -0.592158374795;
  default_target_pose_.pose.position.z = 0.717273819597;
  default_target_pose_.pose.orientation.x = 0.383115589721;
  default_target_pose_.pose.orientation.y = 0.0150255505036;
  default_target_pose_.pose.orientation.z = -0.0361555479561;
  default_target_pose_.pose.orientation.w = 0.922870226032;

  // Read the robot config for this manager.
  TEMOTO_INFO_STREAM(temoto_core::common::getTemotoNamespace());

  // Getting the path of the src folder
  const std::string current_node_path = ros::package::getPath(ROS_PACKAGE_NAME);
  std::vector<std::string> current_node_path_tokens;
  
  boost::split(current_node_path_tokens, current_node_path, boost::is_any_of("/"));
  // Remove all tokens up to "src" token. TODO: May potentially cause problems
  // if duplicate "src" tokens are present.
  bool src_token_found = false;
  while(!src_token_found)
  {
    if (current_node_path_tokens.size() == 0)
    {
      break;
    }
    if(current_node_path_tokens.back() != "src")
    {
      current_node_path_tokens.pop_back();
    }
    else
    {
      current_node_path_tokens.pop_back();
      src_token_found = true;
      break;
    }
  }
  std::string source_path_;
  for (const auto& token : current_node_path_tokens)
  {
    source_path_ += token + "/";
  }
  source_path_ += "src/";

  TEMOTO_INFO_STREAM(source_path_);  

  boost::filesystem::path current_dir (source_path_);
  findRobotDescriptionFiles(current_dir);

  TEMOTO_INFO("Robot manager is ready.");
}

void RobotManager::findRobotDescriptionFiles(boost::filesystem::path current_dir)
{ 
  boost::filesystem::directory_iterator end_itr;
  for ( boost::filesystem::directory_iterator itr( current_dir ); itr != end_itr; ++itr )
  {
    if (boost::filesystem::is_regular_file(*itr) && (itr->path().filename() == "robot_description.yaml"))
    {      
      TEMOTO_INFO_STREAM(itr->path().string());   
      readRobotDescription(itr->path().string());
    }
    else if ( boost::filesystem::is_directory(*itr) )
    {
      findRobotDescriptionFiles(*itr);
    }
  }
}

void RobotManager::readRobotDescription(const std::string& path_file_rob_description)
{
  std::ifstream in(path_file_rob_description);
  YAML::Node yaml_config = YAML::Load(in);  
  // Parse the Robots section
  if (yaml_config["Robots"])
  {
    // local_configs_ = parseRobotConfigs(yaml_config);
    local_configs_ = parseRobotConfigs(yaml_config, local_configs_);
    // Debug what was added
    for (auto& config : local_configs_)
    {
      TEMOTO_DEBUG("Added robot: '%s'.", config->getName().c_str());
      TEMOTO_DEBUG_STREAM("CONFIG: \n" << config->toString());
    }
    // Advertise the parsed local robots
    advertiseConfigs(local_configs_);
  }
  TEMOTO_INFO("Robot manager is ready.");
}

void RobotManager::loadLocalRobot(RobotConfigPtr config, temoto_core::temoto_id::ID resource_id)
{
  if (!config)
  {
    throw CREATE_ERROR(temoto_core::error::Code::NULL_PTR, "config == NULL");    
  }

  try
  {
    active_robot_ = std::make_shared<Robot>(config, resource_registrar_, *this);
    loaded_robots_.emplace(resource_id, active_robot_);
    TEMOTO_DEBUG("Robot '%s' loaded.", config->getName().c_str());
  }
  catch (temoto_core::error::ErrorStack& error_stack)
  {
    //\TODO: Should we adjust reliability for only certain type of errors?
    config->adjustReliability(0.0);
    advertiseConfig(config);
    throw FORWARD_ERROR(error_stack);
  }
}

void RobotManager::loadCb(temoto_robot_manager::RobotLoad::Request& req, temoto_robot_manager::RobotLoad::Response& res)
{
  TEMOTO_INFO("Starting to load robot '%s'...", req.robot_name.c_str());  

  // Find the suitable robot and fill the process manager service request
  auto config = findRobot(req.robot_name, local_configs_);
  if (config)
  {
    try
    {
      loadLocalRobot(config, res.trr.resource_id);
      res.trr.code = temoto_core::trr::status_codes::OK;
      res.trr.message = "Robot sucessfully loaded.";
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
    catch (...)
    {
      TEMOTO_ERROR_STREAM("Failed to load local robot: Unknown exception.");
      return;
    }
    return;
  }
  

  // Try to find suitable candidate from remote managers
  config = findRobot(req.robot_name, remote_configs_);
  if (config)
  {
    try
    {
      temoto_robot_manager::RobotLoad load_robot_srvc;
      load_robot_srvc.request.robot_name = req.robot_name;
      TEMOTO_INFO("RobotManager is forwarding request: '%s'", req.robot_name.c_str());

      resource_registrar_.call<temoto_robot_manager::RobotLoad>(robot_manager::srv_name::MANAGER,
                                                  robot_manager::srv_name::SERVER_LOAD,
                                                  load_robot_srvc,
                                                  temoto_core::trr::FailureBehavior::NONE,
                                                  config->getTemotoNamespace());
      TEMOTO_DEBUG("Call to remote RobotManager was sucessful.");
      res.trr = load_robot_srvc.response.trr;
      active_robot_ = std::make_shared<Robot>(config, resource_registrar_, *this);
      loaded_robots_.emplace(load_robot_srvc.response.trr.resource_id, active_robot_);
    }
    catch(temoto_core::error::ErrorStack& error_stack)
    {
      throw FORWARD_ERROR(error_stack);
    }
    catch (...)
    {
      throw CREATE_ERROR(temoto_core::error::Code::UNHANDLED_EXCEPTION, "Exception occured while creating Robot "
                                                           "object.");
    }
    return;
  }
  else
  {
    // no local nor remote robot found
    throw CREATE_ERROR(temoto_core::error::Code::ROBOT_NOT_FOUND,"Robot manager did not find a suitable robot.");
  }
}

void RobotManager::unloadCb(temoto_robot_manager::RobotLoad::Request& req, temoto_robot_manager::RobotLoad::Response& res)
{
  TEMOTO_DEBUG("ROBOT '%s' unloading...", req.robot_name.c_str());
  TEMOTO_WARN_STREAM(req);
  TEMOTO_WARN_STREAM(res);

//  ros::Duration(5).sleep();
for (const auto& r : loaded_robots_)
{
  TEMOTO_WARN_STREAM(r.first);
}

  // search for the robot based on its resource id, remove from map,
  // and clear active_robot_ if the unloaded robot was active.
  auto it = loaded_robots_.find(res.trr.resource_id);
  if (it != loaded_robots_.end())
  {
    TEMOTO_WARN("REMOVING ROBOT");
    if (active_robot_ == it->second)
    {
      active_robot_ = NULL;
    }
    loaded_robots_.erase(it);
  }
  TEMOTO_DEBUG("ROBOT '%s' unloaded.", req.robot_name.c_str());
}

void RobotManager::syncCb(const temoto_core::ConfigSync& msg, const PayloadType& payload)
{
  if (msg.action == temoto_core::trr::sync_action::REQUEST_CONFIG)
  {
    advertiseConfigs(local_configs_);
    return;
  }

  if (msg.action == temoto_core::trr::sync_action::ADVERTISE_CONFIG)
  {
    // Convert the config string to YAML tree and parse
    YAML::Node yaml_config = YAML::Load(payload.data);
    RobotConfigs configs = parseRobotConfigs(yaml_config);

    // TODO hold remote stuff in a map or something keyed by namespace
    for (auto& config : configs)
    {
      config->setTemotoNamespace(msg.temoto_namespace);
    }

    for (auto& config : configs)
    {
      // Check if robot config has to be added or updated
      auto it = std::find_if(remote_configs_.begin(), remote_configs_.end(),
                             [&](const RobotConfigPtr& ri) -> bool { return *ri == *config; });
      if (it != remote_configs_.end())
      {
        TEMOTO_DEBUG("Updating remote robot '%s' at '%s'.", config->getName().c_str(), config->getTemotoNamespace().c_str());
        *it = config;  // overwrite found entry
      }
      else
      {
        TEMOTO_DEBUG("Adding remote robot '%s' at '%s'.", config->getName().c_str(), config->getTemotoNamespace().c_str());
        remote_configs_.push_back(config);
      }
    }
  }
}

void RobotManager::advertiseConfig(RobotConfigPtr config)
{
  // publish all local robots
  YAML::Node yaml_config;
  yaml_config["Robots"].push_back(config->getYAMLConfig());
  PayloadType payload;
  payload.data = YAML::Dump(yaml_config);
  config_syncer_.advertise(payload);
}

void RobotManager::advertiseConfigs(RobotConfigs configs)
{
  // publish all local robots
  YAML::Node yaml_config;
  for (auto& config : configs)
  {
    yaml_config["Robots"].push_back(config->getYAMLConfig());
  }

  // send to other managers if there is anything to send
  if (yaml_config.size())
  {
    PayloadType payload;
    payload.data = YAML::Dump(yaml_config);
    config_syncer_.advertise(payload);
  }
}

RobotConfigs RobotManager::parseRobotConfigs(const YAML::Node& yaml_config)
{
  RobotConfigs configs;

  if (!yaml_config.IsMap())
  {
    // TODO Throw
    TEMOTO_WARN("Unable to parse 'Robots' key from config.");
    return configs;
  }

  YAML::Node robots_node = yaml_config["Robots"];
  if (!robots_node.IsSequence())
  {
    TEMOTO_WARN("The given config does not contain sequence of robots.");
    // TODO Throw
    return configs;
  }

  TEMOTO_DEBUG("Parsing %lu robots.", robots_node.size());

  // go over each robot node in the sequence
  for (YAML::const_iterator node_it = robots_node.begin(); node_it != robots_node.end(); ++node_it)
  {    
    if (!node_it->IsMap())
    {
      TEMOTO_ERROR("Unable to parse the robot config. Parameters in YAML have to be specified in "
                   "key-value pairs.");
      continue;
    }

    try
    {
      RobotConfig config(*node_it, *this);
      
      // TEMOTO_INFO_STREAM(config.toString());  //==ToErase==

      if (std::count_if(configs.begin(), configs.end(),
                        [&](const RobotConfigPtr& ri) { return *ri == config; }) == 0)
      {
        // OK, this is unique config, add it to the configs.
        TEMOTO_INFO("unique '%s'.", config.getName().c_str());
        configs.emplace_back(std::make_shared<RobotConfig>(config));
      }
      else
      {
        TEMOTO_WARN("Ignoring duplicate of robot '%s'.", config.getName().c_str());   
        TEMOTO_INFO("Ignoring duplicate of robot '%s'.", config.getName().c_str());     
      }
    }
    catch (...)
    {
      TEMOTO_WARN("Failed to parse RobotConfig from config.");
      continue;
    }
  }
  return configs;
}

RobotConfigs RobotManager::parseRobotConfigs(const YAML::Node& yaml_config, RobotConfigs configs)
{
  // RobotConfigs configs;

  if (!yaml_config.IsMap())
  {
    // TODO Throw
    TEMOTO_WARN("Unable to parse 'Robots' key from config.");
    return configs;
  }

  YAML::Node robots_node = yaml_config["Robots"];
  if (!robots_node.IsSequence())
  {
    TEMOTO_WARN("The given config does not contain sequence of robots.");
    // TODO Throw
    return configs;
  }

  TEMOTO_DEBUG("Parsing %lu robots.", robots_node.size());

  // go over each robot node in the sequence
  for (YAML::const_iterator node_it = robots_node.begin(); node_it != robots_node.end(); ++node_it)
  {
    if (!node_it->IsMap())
    {
      TEMOTO_ERROR("Unable to parse the robot config. Parameters in YAML have to be specified in "
                   "key-value pairs.");
      continue;
    }

    try
    {
      RobotConfig config(*node_it, *this);
      
      bool compare = false;
      for (const auto& config_compare : configs)
      {
        if (config.getName() == config_compare->getName())
        {
          TEMOTO_INFO("Equal");
          compare = true;
          TEMOTO_INFO(config.getName().c_str());          
        }
      }    
      
      if (std::count_if(configs.begin(), configs.end(),
                        [&](const RobotConfigPtr& ri) { return *ri == config; }) == 0 && compare==false )                       
      {
        // OK, this is unique config, add it to the configs.
        TEMOTO_INFO("unique '%s'.", config.getName().c_str());
        configs.emplace_back(std::make_shared<RobotConfig>(config));        
      }
      else
      {
        TEMOTO_WARN("Ignoring duplicate of robot '%s'.", config.getName().c_str());
        TEMOTO_INFO("Ignoring duplicate of robot '%s'.", config.getName().c_str());
      }
      compare=false;
    }
    catch (...)
    {
      TEMOTO_WARN("Failed to parse RobotConfig from config.");
      continue;
    }
  }
  return configs;
}

bool RobotManager::planManipulationPathCb(temoto_robot_manager::RobotPlanManipulation::Request& req, temoto_robot_manager::RobotPlanManipulation::Response& res)
{  
  TEMOTO_DEBUG("ACTIVE ROBOT...");
  TEMOTO_DEBUG(active_robot_->getName().c_str());
  if (active_robot_->getName().c_str() != req.robot_name)
  {
    auto robot_it = std::find_if(loaded_robots_.begin(), loaded_robots_.end(),
                                 [&](const std::pair<temoto_core::temoto_id::ID, RobotPtr> p) -> bool {
                                  //  return p.second->getName() == "xarm7_robot_sim";
                                  return p.second->getName() == req.robot_name;
                                 });  
    active_robot_ = robot_it->second;
  }    
  TEMOTO_DEBUG(active_robot_->getName().c_str());

  TEMOTO_DEBUG("PLANNING...");
  if (!active_robot_)
  {
    res.error_stack = CREATE_ERROR(temoto_core::error::Code::ROBOT_PLAN_FAIL, "Unable to plan, because no robot "
                                                                 "is loaded.");
    res.code = temoto_core::trr::status_codes::FAILED;
    return true;
  }
  
  if (active_robot_->isLocal())
  {
    geometry_msgs::PoseStamped pose;
    if (req.use_default_target)
    {
      default_pose_mutex_.lock();
      pose = default_target_pose_;
      default_pose_mutex_.unlock();
    }
    else
    {
      pose = req.target_pose;
    }

    TEMOTO_DEBUG_STREAM("Planning goal: " << pose <<std::endl);

    try
    {
      if (req.use_named_target)
      {
        active_robot_->planManipulationPath(req.planning_group, req.named_target);
      }
      else
      {
        active_robot_->planManipulationPath(req.planning_group, pose);        
      }      
    }
    catch (temoto_core::error::ErrorStack(e))
    {
      res.error_stack = FORWARD_ERROR(e);
      res.code = temoto_core::trr::status_codes::OK;
      return true;
    }

    TEMOTO_DEBUG("DONE PLANNING...");
    res.code = temoto_core::trr::status_codes::OK;
  }
  else
  {
    // This robot is present in a remote robot manager, forward the planning command to there.
    std::string topic = "/" + active_robot_->getConfig()->getTemotoNamespace() + "/" +
                        robot_manager::srv_name::SERVER_PLAN;
    ros::ServiceClient client_plan = nh_.serviceClient<temoto_robot_manager::RobotPlanManipulation>(topic);
    temoto_robot_manager::RobotPlanManipulation fwd_plan_srvc;
    fwd_plan_srvc.request = req;
    fwd_plan_srvc.response = res;
    if (client_plan.call(fwd_plan_srvc))
    {
      res = fwd_plan_srvc.response;
    }
    else
    {
      res.code = temoto_core::trr::status_codes::FAILED;
      res.error_stack = CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Call to remote RobotManager "
                                                                    "service failed.");
      return true;
    }
  }

  return true;
}

bool RobotManager::execManipulationPathCb(temoto_robot_manager::RobotExecutePlan::Request& req,
                          temoto_robot_manager::RobotExecutePlan::Response& res)
{
  TEMOTO_INFO("EXECUTING...");
  if (active_robot_)
  {
    if (active_robot_->isLocal())
    {
      active_robot_->executeManipulationPath();
      TEMOTO_DEBUG("DONE EXECUTING...");
      res.message = "Execute command sent to MoveIt";
      res.code = temoto_core::trr::status_codes::OK;
    }
    else
    {
      // This robot is present in a remote robotmanager, forward the command to there.
      std::string topic = "/" + active_robot_->getConfig()->getTemotoNamespace() + "/" +
                          robot_manager::srv_name::SERVER_EXECUTE;
      ros::ServiceClient client_exec = nh_.serviceClient<temoto_robot_manager::RobotExecutePlan>(topic);
      temoto_robot_manager::RobotExecutePlan fwd_exec_srvc;
      fwd_exec_srvc.request = req;
      fwd_exec_srvc.response = res;
      if (client_exec.call(fwd_exec_srvc))
      {
        TEMOTO_DEBUG("Call to remote RobotManager was sucessful.");
        res = fwd_exec_srvc.response;
      }
      else
      {
        TEMOTO_ERROR("Call to remote RobotManager service failed.");
        res.message = "Call to remote RobotManager service failed.";
        res.code = temoto_core::trr::status_codes::FAILED;
      }
    }
  }
  else
  {
    TEMOTO_ERROR("Unable to execute, because the robot is not loaded.");
  }
  return true;
}

bool RobotManager::getVizInfoCb(temoto_robot_manager::RobotGetVizInfo::Request& req,
                                temoto_robot_manager::RobotGetVizInfo::Response& res)
{
  TEMOTO_INFO("GETTING visualization info...");
  // Search for the loaded robot, when its name is specified.
  if (req.robot_name != "")
  {
    auto robot_it = std::find_if(loaded_robots_.begin(), loaded_robots_.end(),
                                 [&](const std::pair<temoto_core::temoto_id::ID, RobotPtr> p) -> bool {
                                   return p.second->getName() == req.robot_name;
                                 });
    if (robot_it != loaded_robots_.end())
    {
      res.info = robot_it->second->getVizInfo();
    }
    else
    {
      res.error_stack = CREATE_ERROR(temoto_core::error::Code::ROBOT_NOT_LOADED,
                                     "The requested robot '%s' is not loaded.", req.robot_name);
      res.code == temoto_core::trr::status_codes::FAILED;
      return true;
    }
  }
  else
  {
    // Robot name is not specified, try to use the active robot.
    if (active_robot_)
    {
      res.info = active_robot_->getVizInfo();
    }
    else
    {
      res.error_stack =
          CREATE_ERROR(temoto_core::error::Code::ROBOT_NOT_LOADED, "No loaded robots found.", req.robot_name);
      res.code == temoto_core::trr::status_codes::FAILED;
      return true;
    }
  }
  res.code = temoto_core::trr::status_codes::OK;
  return true;
}

bool RobotManager::setManipulationTargetCb(temoto_robot_manager::RobotSetTarget::Request& req,
                               temoto_robot_manager::RobotSetTarget::Response& res)
{
  if (active_robot_->isLocal())
  {
    TEMOTO_INFO("Setting target to object '%s'", req.object_name.c_str());

    temoto_context_manager::TrackObject track_object_msg;
    track_object_msg.request.object_name = req.object_name;

    try
    {
      resource_registrar_.call<temoto_context_manager::TrackObject>(temoto_context_manager::srv_name::MANAGER,
                                                    temoto_context_manager::srv_name::TRACK_OBJECT_SERVER,
                                                    track_object_msg);
      TEMOTO_DEBUG("Subscribing to '%s'", track_object_msg.response.object_topic.c_str());
      target_pose_sub_ = nh_.subscribe(track_object_msg.response.object_topic, 1,
                                       &RobotManager::targetPoseCb, this);
    }
    catch (temoto_core::error::ErrorStack& error_stack)
    {
      res.error_stack = FORWARD_ERROR(error_stack);
      res.code = temoto_core::trr::status_codes::FAILED;
    }
  }
  else
  {
    // This is remote robot, forward the set target command
    std::string topic = "/" + active_robot_->getConfig()->getTemotoNamespace() + "/" +
                        robot_manager::srv_name::SERVER_SET_MANIPULATION_TARGET;
    ros::ServiceClient client_mode = nh_.serviceClient<temoto_robot_manager::RobotSetTarget>(topic);
    temoto_robot_manager::RobotSetTarget fwd_target_srvc;
    fwd_target_srvc.request = req;
    fwd_target_srvc.response = res;
    if (client_mode.call(fwd_target_srvc))
    {
      TEMOTO_DEBUG("Call to remote RobotManager was sucessful.");
      res = fwd_target_srvc.response;
    }
    else
    {
      TEMOTO_ERROR("Call to remote RobotManager service failed.");
      res.message = "Call to remote RobotManager service failed.";
      res.code = temoto_core::trr::status_codes::FAILED;
    }
  }
  return true;
}

bool RobotManager::getManipulationTargetCb(temoto_robot_manager::RobotGetTarget::Request& req,
                                          temoto_robot_manager::RobotGetTarget::Response& res)
{
  if (active_robot_->getName().c_str() != req.robot_name)
  {
    auto robot_it = std::find_if(loaded_robots_.begin(), loaded_robots_.end(),
                                 [&](const std::pair<temoto_core::temoto_id::ID, RobotPtr> p) -> bool {
                                  return p.second->getName() == req.robot_name;
                                 });  
    active_robot_ = robot_it->second;
  }

  if (active_robot_->isLocal())
  {    
    res.pose = active_robot_->getManipulationTarget();
  }
  else
  {
    TEMOTO_INFO("robot is not local");
    std::string topic = "/" + active_robot_->getConfig()->getTemotoNamespace() + "/" +
                        robot_manager::srv_name::SERVER_GET_MANIPULATION_TARGET;
    ros::ServiceClient client_mode = nh_.serviceClient<temoto_robot_manager::RobotGetTarget>(topic);
    temoto_robot_manager::RobotGetTarget fwd_get_target_srvc;
    fwd_get_target_srvc.request = req;
    fwd_get_target_srvc.response = res;
    if (client_mode.call(fwd_get_target_srvc))
    {
      TEMOTO_DEBUG("Call to remote RobotManager was sucessful.");
      res = fwd_get_target_srvc.response;
    }
    else
    {
      TEMOTO_ERROR("Call to remote RobotManager service failed.");      
    }    
  }  
  return true;
}

bool RobotManager::goalNavigationCb(temoto_robot_manager::RobotGoal::Request& req, 
                                    temoto_robot_manager::RobotGoal::Response& res)
{
  if (active_robot_->getName().c_str() != req.robot_name)
  {
    auto robot_it = std::find_if(loaded_robots_.begin(), loaded_robots_.end(),
                                 [&](const std::pair<temoto_core::temoto_id::ID, RobotPtr> p) -> bool {
                                  return p.second->getName() == req.robot_name;
                                 });  
    active_robot_ = robot_it->second;
  }
  TEMOTO_DEBUG("GOAL NAVIGATION...");
  if (!active_robot_)
  {
    //TODO: Add the correspondig error, for now using the plan code
    res.error_stack = CREATE_ERROR(temoto_core::error::Code::ROBOT_PLAN_FAIL, "Unable to navigate, because no robot "
                                                                 "is loaded.");                                                              
    res.code = temoto_core::trr::status_codes::FAILED;
    return true;
  }

  if (active_robot_->isLocal())
  {
    active_robot_->goalNavigation("map", req.target_pose);  // The robot would move with respect to this coordinate frame
    TEMOTO_DEBUG("DONE NAVIGATION TO THE GOAL...");        
  }
  else
  {
    std::string topic = "/" + active_robot_->getConfig()->getTemotoNamespace() + "/" +
                        robot_manager::srv_name::SERVER_NAVIGATION_GOAL;
    ros::ServiceClient client_navigation_goal_ = nh_.serviceClient<temoto_robot_manager::RobotGoal>(topic);
    temoto_robot_manager::RobotGoal fwd_goal_srvc;
    fwd_goal_srvc.request = req;
    fwd_goal_srvc.response = res;
    if (client_navigation_goal_.call(fwd_goal_srvc))
    {
      res = fwd_goal_srvc.response;
    }
    else
    {
      res.code = temoto_core::trr::status_codes::FAILED;
      res.error_stack = CREATE_ERROR(temoto_core::error::Code::SERVICE_REQ_FAIL, "Call to remote RobotManager "
                                                                    "service failed.");
      return true;
    }
  }  
  return true;  
}

bool RobotManager::setModeCb(temoto_robot_manager::RobotSetMode::Request& req,
                             temoto_robot_manager::RobotSetMode::Response& res)
{
  TEMOTO_INFO("SET MODE...");
  // input validation
  if (req.mode != modes::AUTO && req.mode != modes::NAVIGATION && req.mode != modes::MANIPULATION && req.mode != modes::GRIPPER)
  {
    TEMOTO_ERROR("Mode '%s' is not supported.", req.mode.c_str());
    res.message = "Mode is not supported.";
    res.code = temoto_core::trr::status_codes::FAILED;
    return true;
  }

  if (active_robot_)
  {
    if (active_robot_->isLocal())
    {
      mode_ = req.mode;
      TEMOTO_DEBUG("Robot mode set to: %s...", mode_.c_str());
      res.message = "Robot mode set to '" + mode_ + "'.";
      res.code = temoto_core::trr::status_codes::OK;
    }
    else
    {
      // This robot is present in a remote robotmanager, forward the command to there.
      std::string topic = "/" + active_robot_->getConfig()->getTemotoNamespace() + "/" +
                          robot_manager::srv_name::SERVER_SET_MODE;
      ros::ServiceClient client_mode = nh_.serviceClient<temoto_robot_manager::RobotSetMode>(topic);
      temoto_robot_manager::RobotSetMode fwd_mode_srvc;
      fwd_mode_srvc.request = req;
      fwd_mode_srvc.response = res;
      if (client_mode.call(fwd_mode_srvc))
      {
        TEMOTO_DEBUG("Call to remote RobotManager was sucessful.");
        res = fwd_mode_srvc.response;
      }
      else
      {
        TEMOTO_ERROR("Call to remote RobotManager service failed.");
        res.message = "Call to remote RobotManager service failed.";
        res.code = temoto_core::trr::status_codes::FAILED;
      }
    }
  }
  else
  {
    TEMOTO_ERROR("Unable to set mode, because the robot is not loaded.");
  }
  return true;
}

// Take palm pose of whichever hand is present, prefer left_hand.
// Store the pose in a class member for later use when planning is requested.
void RobotManager::targetPoseCb(const temoto_context_manager::ObjectContainer& msg)
{
    default_pose_mutex_.lock();
    default_target_pose_ = msg.pose;

    geometry_msgs::TransformStamped tf_world_to_target;
    try
    {
      tf_world_to_target =
          tf2_buffer.lookupTransform("world", msg.pose.header.frame_id, ros::Time(0));
      tf2::doTransform(msg.pose, default_target_pose_, tf_world_to_target);
      default_target_pose_.header.frame_id = "world";
    }
    catch(tf2::TransformException ex)
    {
      TEMOTO_ERROR("%s",ex.what());
    }

//    tf::StampedTransform transform;
//    //default_target_pose_.pose.position.x = transform.getOrigin().x() + msg.pose.pose.position.x;
//    //default_target_pose_.pose.position.y = transform.getOrigin().y() + msg.pose.pose.position.y;
//    //default_target_pose_.pose.position.z = transform.getOrigin().z() + msg.pose.pose.position.z;
//    
//    default_target_pose_.pose.position.x = transform.getOrigin().x();
//    default_target_pose_.pose.position.y = transform.getOrigin().y();
//    default_target_pose_.pose.position.z = transform.getOrigin().z();
//
//    tf::Quaternion q = transform.getRotation().normalized();
//    //default_target_pose_.pose.orientation.x = (double)q.getX() + msg.pose.pose.orientation.x;
//    //default_target_pose_.pose.orientation.y = (double)q.getY() + msg.pose.pose.orientation.y;
//    //default_target_pose_.pose.orientation.z = (double)q.getZ() + msg.pose.pose.orientation.z;
//    //default_target_pose_.pose.orientation.w = (double)q.getW() + msg.pose.pose.orientation.w;
//
//    default_target_pose_.pose.orientation.x = (double)q.getX();
//    default_target_pose_.pose.orientation.y = (double)q.getY();
//    default_target_pose_.pose.orientation.z = (double)q.getZ();
//    default_target_pose_.pose.orientation.w = (double)q.getW();

    temoto_context_manager::ObjectContainer msg2 = msg;
    msg2.marker.header = default_target_pose_.header;
    msg2.marker.pose = default_target_pose_.pose;
    msg2.marker.ns = "blah2346";
    msg2.marker.id = 0;
    //  msg2.marker.type = visualization_msgs::Marker::CUBE;
    // msg2.marker.color.g = 1.0;
    msg2.marker.lifetime = ros::Duration();

    if (marker_publisher_)
    {
      marker_publisher_.publish(msg2.marker);
  }
  else
  {
    TEMOTO_ERROR("no marker publisher");
  }
//    TEMOTO_DEBUG_STREAM(default_target_pose_);

    default_pose_mutex_.unlock();
}

void RobotManager::statusInfoCb(temoto_core::ResourceStatus& srv)
{
  TEMOTO_DEBUG("status info was received");
  TEMOTO_DEBUG_STREAM(srv.request);
  // if any resource should fail, just unload it and try again
  // there is a chance that sensor manager gives us better sensor this time
  // if (srv.request.status_code == temoto_core::trr::status_codes::FAILED &&
  //     srv.request.resource_id == hand_srv_msg_.response.trr.resource_id)
  // {
  //   TEMOTO_WARN("Robot manager detected a hand sensor failure. Unloading and "
  //               "trying again");
  //   try
  //   {
  //     resource_registrar_.unloadClientResource(hand_srv_msg_.response.trr.resource_id);
  //   }
  //   catch (temoto_core::error::ErrorStack& error_stack)
  //   {
  //     TEMOTO_ERROR_STREAM(error_stack);
  //   }

  //   // retry with previous request
  //   try
  //   {
  //     resource_registrar_.call<temoto_robot_manager::LoadGesture>(temoto_context_manager::srv_name::MANAGER,
  //                                                   temoto_context_manager::srv_name::GESTURE_SERVER,
  //                                                   hand_srv_msg_, temoto_core::trr::FailureBehavior::NONE);
  //     TEMOTO_DEBUG("Subscribing to '%s'", hand_srv_msg_.response.topic.c_str());
  //     target_pose_sub_ =
  //         nh_.subscribe(hand_srv_msg_.response.topic, 1, &RobotManager::targetPoseCb, this);
  //   }
  //   catch (temoto_core::error::ErrorStack& error_stack)
  //   {
  //     throw FORWARD_ERROR(error_stack);
  //   }
  // }

  // Check if any of the allocated robots has failed
  // Currently we simply remove the loaded robot if it failed
  if (srv.request.status_code == temoto_core::trr::status_codes::FAILED)
  {
    // was it a remote robot
    if (loaded_robots_.erase(srv.request.resource_id))
    {
      TEMOTO_DEBUG("Removed remote robot, because its status failed.");
      return;
    }

    // check if it was a resource related to a robot feature has failed.
    // unload the robot
    for (auto it = loaded_robots_.begin(); it != loaded_robots_.end(); ++it)
    {
      if (it->second->hasResource(srv.request.resource_id))
      {
        RobotConfigPtr config = it->second->getConfig();
        config->adjustReliability(0.0);
        YAML::Node yaml_config;
        yaml_config["Robots"].push_back(config->getYAMLConfig());
        PayloadType payload;
        payload.data = YAML::Dump(yaml_config);
        std::cout << payload << std::endl;
        config_syncer_.advertise(payload);
        loaded_robots_.erase(it);
        break;
      }
    }
  }
}

RobotConfigPtr RobotManager::findRobot(const std::string& robot_name, const RobotConfigs& configs)
{
  // Local list of devices that follow the requirements
  RobotConfigs candidates;

  // If robot name is unspecified, pick the best one from all configs.
  if (robot_name == "")
  {
    candidates = configs;
  }
  else
  {
    // Find the robot that matches the "name" criteria
    auto it = std::copy_if(configs.begin(), configs.end(), std::back_inserter(candidates),
                           [&](const RobotConfigPtr& s) { return s->getName() == robot_name; });
  }

  // If the list is empty, leave the req empty
  if (candidates.empty())
  {
    return NULL;
  }

  std::sort(candidates.begin(), candidates.end(), [](RobotConfigPtr& rc1, RobotConfigPtr& rc2) 
  {
    return rc1->getReliability() > rc2->getReliability();
  });

  // Get the name of the package and first launchable
  return candidates.front();
}

bool RobotManager::gripperControlPositionCb(temoto_robot_manager::RobotGripperControlPosition::Request& req, 
                                    temoto_robot_manager::RobotGripperControlPosition::Response& res)
{
  TEMOTO_INFO_STREAM("inside of callback function");
  TEMOTO_INFO_STREAM(active_robot_->getName().c_str());
  if (active_robot_->getName().c_str() != req.gripper_name)
  {
    auto robot_it = std::find_if(loaded_robots_.begin(), loaded_robots_.end(),
                                 [&](const std::pair<temoto_core::temoto_id::ID, RobotPtr> p) -> bool {
                                  return p.second->getName() == req.gripper_name;
                                 });  
    active_robot_ = robot_it->second;
  }
  TEMOTO_INFO_STREAM("GRIPPER CONTROL...");
  
  if (!active_robot_)
  {    
    //TODO: Add the correspondig error, for now using the plan code
    TEMOTO_INFO_STREAM("Unable to control because gripper is not loaded");
    return true;
  }

  if (active_robot_->isLocal())
  {
    TEMOTO_INFO_STREAM("gripper is local - ");  
    TEMOTO_INFO_STREAM(req.gripper_name);  
    TEMOTO_INFO_STREAM(req.control);  

    active_robot_->controlGripper(req.gripper_name,req.control);
    TEMOTO_INFO_STREAM("THE GRIPPER MOVED...");
  }
  else
  {
    std::string topic = "/" + active_robot_->getConfig()->getTemotoNamespace() + "/" +
                        robot_manager::srv_name::SERVER_GRIPPER_CONTROL_POSITION;
    ros::ServiceClient client_gripper_control_position_ = nh_.serviceClient<temoto_robot_manager::RobotGripperControlPosition>(topic);
    temoto_robot_manager::RobotGripperControlPosition fwd_gripper_srvc;
    fwd_gripper_srvc.request = req;
    fwd_gripper_srvc.response = res;
    if (client_gripper_control_position_.call(fwd_gripper_srvc))
    {
      res = fwd_gripper_srvc.response;
    }
    else
    {      
      TEMOTO_INFO_STREAM("Call to remote RobotManager / gripper - service failed.");
      return true;
    }
  }  
  return true; 
}

}  // namespace robot_manager
