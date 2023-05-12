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

#include "ros/package.h"
#include "temoto_robot_manager/robot_manager.h"
#include "temoto_process_manager/process_manager_services.hpp"

#include <boost/filesystem/operations.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>

namespace temoto_robot_manager
{

std::string generateId()
{
  return boost::uuids::to_string(boost::uuids::random_generator()());
}

RobotManager::RobotManager(const std::string& config_base_path, bool restore_from_catalog)
: temoto_core::BaseSubsystem("robot_manager", temoto_core::error::Subsystem::ROBOT_MANAGER, __func__)
, resource_registrar_(srv_name::MANAGER)
, config_syncer_(srv_name::MANAGER, srv_name::SYNC_TOPIC, &RobotManager::syncCb, this)
, tf2_listener(tf2_buffer)
{
  /*
   * Configure the RR catalog backup routine
   */
  std::string home_path = std::getenv("HOME");
  std::string rr_catalog_backup_path = home_path + "/.temoto/" + srv_name::MANAGER + ".rrcat";
  rr_catalog_config_.setName(srv_name::MANAGER);
  rr_catalog_config_.setLocation(rr_catalog_backup_path);
  rr_catalog_config_.setSaveOnModify(true);
  rr_catalog_config_.setEraseOnDestruct(true);
  resource_registrar_.updateConfiguration(rr_catalog_config_);
  
  /*
   * Add the LoadRobot server to the resource registrar
   */
  auto server = std::make_unique<Ros1Server<RobotLoad>>(srv_name::SERVER_LOAD
  , std::bind(&RobotManager::loadCb, this, std::placeholders::_1, std::placeholders::_2)
  , std::bind(&RobotManager::unloadCb, this, std::placeholders::_1, std::placeholders::_2));

  resource_registrar_.registerServer(std::move(server));
  resource_registrar_.init();

  /*
   * Find the robot_description.yaml file
   */
  findRobotDescriptionFiles(config_base_path);

  /*
   * Check if this node should be recovered from a previous system failure
   */
  if (restore_from_catalog && boost::filesystem::exists(rr_catalog_backup_path))
  {
    restoreState();
  }

  // resource_registrar_.registerStatusCb(&RobotManager::resourceStatusCb);

  // Ask remote robot managers to send their robot config
  config_syncer_.requestRemoteConfigs();

  /*
   * Fire up additional regular ROS servers for performing various operations on a robot.
   */
  server_plan_ = nh_.advertiseService(
    srv_name::SERVER_PLAN,
    &RobotManager::planManipulationPathCb,
    this);
  server_exec_ = nh_.advertiseService(
    srv_name::SERVER_EXECUTE,
    &RobotManager::execManipulationPathCb,
    this);
  server_get_viz_cfg_ = nh_.advertiseService(
    srv_name::SERVER_GET_VIZ_INFO,
    &RobotManager::getVizInfoCb,
    this);
  server_get_manipulation_target_ = nh_.advertiseService(
    srv_name::SERVER_GET_MANIPULATION_TARGET,
    &RobotManager::getManipulationTargetCb,
    this);
  server_get_manipulation_named_targets_ = nh_.advertiseService(
    srv_name::SERVER_GET_MANIPULATION_NAMED_TARGETS,
    &RobotManager::getManipulationNamedTargetsCb,
    this);
  server_navigation_goal_ = nh_.advertiseService(
    srv_name::SERVER_NAVIGATION_GOAL, 
    &RobotManager::goalNavigationCb,
    this);
  server_gripper_control_position_ = nh_.advertiseService(
    srv_name::SERVER_GRIPPER_CONTROL_POSITION,
    &RobotManager::gripperControlPositionCb,
    this);
  server_get_robot_config_ = nh_.advertiseService(
    srv_name::SERVER_GET_CONFIG,
    &RobotManager::getRobotConfigCb,
    this);

  /*
   * Set up the Custom Feature channel
   */
  server_custom_feature_ = nh_.advertiseService(
    channels::custom::REQUEST,
    &RobotManager::customFeatureCb,
    this);
  server_custom_feature_preempt_ = nh_.advertiseService(
    channels::custom::PREEMPT,
    &RobotManager::customFeaturePreemptCb,
    this);
  pub_custom_feature_feedback_ = nh_.advertise<CustomFeedback>(channels::custom::FEEDBACK, 10);

  TEMOTO_INFO_("Robot manager is ready.\n");
}

bool RobotManager::customFeatureCb(CustomRequest::Request& req, CustomRequest::Response& res)
try
{
  TEMOTO_INFO_("Received an invocation request");
  TEMOTO_DEBUG_STREAM_("Request:\n" << req);
  std::lock_guard<std::mutex> l(mutex_ongoing_custom_requests_);

  auto custom_request_it = std::find_if(
    ongoing_custom_requests_.begin()
  , ongoing_custom_requests_.end()
  , [&](const auto& ongoing_req)
  {
    return req == ongoing_req.second.request;
  });

  /*
   * Check if that request is already processed by a client with higher priority
   */
  if (custom_request_it != ongoing_custom_requests_.end() &&
      custom_request_it->second.request.priority > req.priority)
  {
    res.message = "Request declined as it is already in process by client with higher priority";
    TEMOTO_WARN_STREAM_(res.message << std::endl);

    res.accepted = false;
    return true;
  }
  
  RobotPtr loaded_robot = findLoadedRobot(req.robot_name);

  /*
   * Pre-empt the lower priority request
   */
  if (custom_request_it != ongoing_custom_requests_.end())
  {
    TEMOTO_INFO_("This request is already in process under a lower priority, preempting.");
    loaded_robot->preemptCustomFeature(req.custom_feature_name);
    ongoing_custom_requests_.erase(custom_request_it);
  }

  res.request_id = generateId();
  res.accepted = true;

  RmCustomRequestWrap req_rm;
  req_rm.robot_name = req.robot_name;
  req_rm.custom_feature_name = req.custom_feature_name;
  req_rm.request_id = res.request_id;
  req_rm.data_str = req.data_str;
  req_rm.data_str_array = req.data_str_array;
  req_rm.data_num = req.data_num;
  req_rm.data_num_array = req.data_num_array;
  req_rm.data_pose = RmCustomRequest::PoseStamped{};                    // TODO
  req_rm.data_pose_array = std::vector<RmCustomRequest::PoseStamped>{}; // TODO

  // TODO: Add pose and pose array
  loaded_robot->invokeCustomFeature(req.custom_feature_name, req_rm);

  ongoing_custom_requests_.insert({res.request_id
  , [&]
  {
    CustomRequest srv_msg;
    srv_msg.request = req;
    srv_msg.response = res;
    return srv_msg;
  }()});

  TEMOTO_INFO_("Invocation request successful.\n");
  return true;
}
catch(resource_registrar::TemotoErrorStack& e)
{
  res.accepted = false;
  res.message = std::string("Unable to invoke the custom feature:\n") + e.what();
  TEMOTO_WARN_STREAM_(res.message << std::endl);

  return true;
}

bool RobotManager::customFeaturePreemptCb(CustomRequestPreempt::Request& req, CustomRequestPreempt::Response& res)
try
{
  TEMOTO_INFO_("Received a pre-emption request");
  TEMOTO_DEBUG_STREAM_("Request:\n" << req);
  std::lock_guard<std::mutex> l(mutex_ongoing_custom_requests_);

  auto custom_request_it = std::find_if(
    ongoing_custom_requests_.begin()
  , ongoing_custom_requests_.end()
  , [&](const auto& ongoing_req)
  {
    // return req == ongoing_req.second.request;
    return req.robot_name == ongoing_req.second.request.robot_name &&
           req.custom_feature_name == ongoing_req.second.request.custom_feature_name;
  });

  /*
   * DECLINE: If no ID was provided and priority is low
   */
  if (req.request_id.empty() && (req.priority <= custom_request_it->second.request.priority))
  {
    res.message = "Pre-emption request declined: Priority lower than required";
    TEMOTO_WARN_STREAM_(res.message << std::endl);

    res.accepted = false;
    return true;
  }

  /*
   * DECLINE: If ID was provided but mismatches
   */
  if (!req.request_id.empty() && (req.request_id != custom_request_it->second.response.request_id))
  {
    res.message = "Pre-emption request declined: Request ID mismatch";
    TEMOTO_WARN_STREAM_(res.message << std::endl);
    
    res.accepted = false;
    return true;
  }

  /*
   * ACCEPT: If ID matches or the priority is higher
   */
  RobotPtr loaded_robot = findLoadedRobot(req.robot_name);
  loaded_robot->preemptCustomFeature(req.custom_feature_name);
  ongoing_custom_requests_.erase(custom_request_it);

  TEMOTO_INFO_("Pre-emption request initiated successfully.\n");
  res.accepted = true;
  return true;
}
catch(resource_registrar::TemotoErrorStack& e)
{
  res.accepted = false;
  res.message = std::string("Pre-emption request declined: \n") + e.what();
  TEMOTO_WARN_STREAM_(res.message << std::endl);

  return true;
}

void RobotManager::customFeatureUpdateCb(const RmCustomFeedbackWrap& feedback)
{
  CustomFeedback msg;

  msg.header.stamp = ros::Time::now();
  msg.robot_name = feedback.robot_name;
  msg.custom_feature_name = feedback.custom_feature_name;
  msg.request_id = feedback.request_id;
  msg.status = feedback.status;
  msg.progress = feedback.progress;

  std::lock_guard<std::mutex> l(mutex_pub_custom_feature_feedback_);
  pub_custom_feature_feedback_.publish(msg);
}

void RobotManager::findRobotDescriptionFiles(boost::filesystem::path current_dir)
{
  if (std::string(current_dir.c_str()).empty())
  {
    TEMOTO_WARN_STREAM_("robot_description.yaml base path is empty");
    return;
  }

  boost::filesystem::directory_iterator end_itr;
  for ( boost::filesystem::directory_iterator itr( current_dir ); itr != end_itr; ++itr )
  {
    if (boost::filesystem::is_regular_file(*itr) && (itr->path().filename() == "robot_description.yaml"))
    {      
      TEMOTO_INFO_STREAM_(itr->path().string());   
      readRobotDescription(itr->path().string());
    }
    else if ( boost::filesystem::is_directory(*itr) )
    {
      findRobotDescriptionFiles(*itr);
    }
  }
}

void RobotManager::readRobotDescription(const std::string& path_to_rob_description)
{
  std::ifstream in(path_to_rob_description);
  YAML::Node yaml_config = YAML::Load(in);  
  // Parse the Robots section
  if (yaml_config["Robots"])
  {
    local_configs_ = parseRobotConfigs(yaml_config);

    // Debug what was added
    for (auto& config : local_configs_)
    {
      TEMOTO_DEBUG_STREAM_("Added robot: '" << config->getName() << "'\nCONFIG: " << config->toString());
    }

    // Advertise the parsed local robots
    advertiseConfigs(local_configs_);
  }
}

void RobotManager::loadCb(RobotLoad::Request& req, RobotLoad::Response& res)
{
  TEMOTO_INFO_("Loading robot '%s'...", req.robot_name.c_str());  

  // Find the suitable robot and fill the process manager service request
  auto config = findRobot(req.robot_name, local_configs_);
  if (config)
  try
  {
    auto loaded_robot = std::make_shared<Robot>(config, res.temoto_metadata.request_id, resource_registrar_
    , std::bind(&RobotManager::customFeatureUpdateCb, this, std::placeholders::_1));

    loaded_robot->load();
    loaded_robots_.push_back(loaded_robot);
    TEMOTO_INFO_("Robot '%s' loaded.\n", config->getName().c_str());
    return;
  }
  catch (resource_registrar::TemotoErrorStack& e)
  {
    throw FWD_TEMOTO_ERRSTACK(e);
  }
  catch (...)
  {
    config->adjustReliability(0.0);
    advertiseConfig(config);
    throw TEMOTO_ERRSTACK("Failed to load robot '" + req.robot_name + "'");
  }
    
  
  // Try to find suitable candidate from remote managers
  config = findRobot(req.robot_name, remote_configs_);
  if (!config)
  {
    throw TEMOTO_ERRSTACK("Robot manager did not find a suitable robot.");
  }

  try
  {
    RobotLoad load_robot_srvc;
    load_robot_srvc.request.robot_name = req.robot_name;
    TEMOTO_INFO_("RobotManager is forwarding request to load '%s' to '%s'", req.robot_name.c_str(), config->getTemotoNamespace().c_str());

    resource_registrar_.call<RobotLoad>("/" + config->getTemotoNamespace() + "/" + srv_name::MANAGER
    , srv_name::SERVER_LOAD
    , load_robot_srvc);

    TEMOTO_INFO_("Call to remote RobotManager was sucessful.");
    auto loaded_robot = std::make_shared<Robot>(config, res.temoto_metadata.request_id, resource_registrar_
    , std::bind(&RobotManager::customFeatureUpdateCb, this, std::placeholders::_1));
    loaded_robots_.push_back(loaded_robot);

    return;
  }
  catch(resource_registrar::TemotoErrorStack& e)
  {
    throw FWD_TEMOTO_ERRSTACK(e);
  }
  catch (...)
  {
    throw TEMOTO_ERRSTACK("Exception occured while creating Robot object.");
  }
}

void RobotManager::unloadCb(RobotLoad::Request& req, RobotLoad::Response& res)
{
  TEMOTO_INFO_("Unloading robot '%s' ...", req.robot_name.c_str());

  // search for the robot based on its resource id, remove from map,
  // and clear loaded_robot if the unloaded robot was active.
  auto robot_it = std::find_if(loaded_robots_.begin()
  , loaded_robots_.end()
  , [&](const RobotManager::RobotPtr p) -> bool
    {
      return p->getName() == req.robot_name;
    });
  
  if (robot_it != loaded_robots_.end())
  {
    loaded_robots_.erase(robot_it);
    TEMOTO_INFO_("Robot '%s' unloaded.\n", req.robot_name.c_str());
  }
  else
  {
    throw TEMOTO_ERRSTACK("Unable to unload the robot '" + req.robot_name + "'");
  }
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
        TEMOTO_DEBUG_("Updating remote robot '%s' at '%s'.", config->getName().c_str(), config->getTemotoNamespace().c_str());
        *it = config;  // overwrite found entry
      }
      else
      {
        TEMOTO_DEBUG_("Adding remote robot '%s' at '%s'.", config->getName().c_str(), config->getTemotoNamespace().c_str());
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
    throw TEMOTO_ERRSTACK("Unable to parse 'Robots' key from config.");
  }

  YAML::Node robots_node = yaml_config["Robots"];
  if (!robots_node.IsSequence())
  {
    throw TEMOTO_ERRSTACK("The given config does not contain sequence of robots.");
  }

  TEMOTO_DEBUG_("Parsing %lu robots.", robots_node.size());

  // go over each robot node in the sequence
  for (YAML::const_iterator node_it = robots_node.begin(); node_it != robots_node.end(); ++node_it)
  try
  {
    if (!node_it->IsMap())
    {
      TEMOTO_WARN_("Unable to parse the robot config. Parameters in YAML have to be specified in "
                    "key-value pairs.");
      continue;
    }

    RobotConfig config(*node_it);

    // Check if the config is unique
    if (std::count_if(configs.begin(), configs.end()
    , [&](const RobotConfigPtr& ri) { return ri->getName() == config.getName(); }) == 0)
    {
      TEMOTO_INFO_("Adding robot '%s'", config.getName().c_str());
      configs.emplace_back(std::make_shared<RobotConfig>(config));
    }
    else
    {
      TEMOTO_WARN_("Ignoring duplicate of robot '%s'", config.getName().c_str());       
    }
  }
  catch (resource_registrar::TemotoErrorStack& e)
  {
    TEMOTO_WARN_STREAM_("Failed to parse config: " << e.what());
  }

  return configs;
}

bool RobotManager::planManipulationPathCb(RobotPlanManipulation::Request& req, RobotPlanManipulation::Response& res)
try
{
  RobotPtr loaded_robot = findLoadedRobot(req.robot_name);

  if (loaded_robot->isLocal())
  {
    TEMOTO_DEBUG_STREAM_("Creating a manipulation path for robot '" << loaded_robot->getName() 
      << " with goal pose: " << req.goal_target <<std::endl);

    switch (req.goal_target)
    {
      case RobotPlanManipulation::Request::POSE_STAMPED:
        loaded_robot->planManipulationPath(req.planning_group, req.target_pose);
        break;

      case RobotPlanManipulation::Request::NAMED_TARGET_POSE:
        loaded_robot->planManipulationPath(req.planning_group, req.named_target);
        break;

      case RobotPlanManipulation::Request::JOINT_STATE:
        loaded_robot->planManipulationPath(req.planning_group, req.joint_state_target);
        break;

      default:
        throw TEMOTO_ERRSTACK("Unable to plan because the Goal_target is unknown.");
    }

    TEMOTO_DEBUG_("Done planning.");
  }
  else
  {
    // This robot is present in a remote robot manager, forward the planning command to there.
    std::string topic = "/" + loaded_robot->getConfig()->getTemotoNamespace() + "/" 
      + srv_name::SERVER_PLAN;
    TEMOTO_DEBUG_STREAM_("Forwarding the planning request to remote robot manager at '" << topic << "'.");

    ros::ServiceClient client_plan = nh_.serviceClient<RobotPlanManipulation>(topic);
    RobotPlanManipulation fwd_plan_srvc;
    fwd_plan_srvc.request = req;
    fwd_plan_srvc.response = res;

    if (client_plan.call(fwd_plan_srvc))
    {
      res = fwd_plan_srvc.response;
    }
    else
    {
      throw TEMOTO_ERRSTACK("Call to remote RobotManager service failed.");
    }
  }
  res.success = true;
  return true;
}
catch(resource_registrar::TemotoErrorStack& e)
{
  res.success = false;
  return true;
}


bool RobotManager::execManipulationPathCb(RobotExecutePlan::Request& req, RobotExecutePlan::Response& res)
try
{
  RobotPtr loaded_robot = findLoadedRobot(req.robot_name);

  if (loaded_robot->isLocal())
  {
    TEMOTO_DEBUG_STREAM_("Executing a manipulation path for robot '" << loaded_robot->getName() << " ...");
    loaded_robot->executeManipulationPath();
    TEMOTO_DEBUG_("Done executing.");
  }
  else
  {
    // This robot is present in a remote robotmanager, forward the command to there.
    std::string topic = "/" + loaded_robot->getConfig()->getTemotoNamespace() + "/"
      + srv_name::SERVER_EXECUTE;
    TEMOTO_DEBUG_STREAM_("Forwarding the execution request to remote robot manager at '" << topic << "'.");

    ros::ServiceClient client_exec = nh_.serviceClient<RobotExecutePlan>(topic);
    RobotExecutePlan fwd_exec_srvc;
    fwd_exec_srvc.request = req;
    fwd_exec_srvc.response = res;

    if (client_exec.call(fwd_exec_srvc))
    {
      TEMOTO_DEBUG_("Call to remote RobotManager was sucessful.");
      res = fwd_exec_srvc.response;
    }
    else
    {
      throw TEMOTO_ERRSTACK("Call to remote RobotManager service failed.");
    }
  }
  res.success = true;
  return true;
}
catch(resource_registrar::TemotoErrorStack& e)
{
  res.success = false;
  return true;
}

bool RobotManager::getVizInfoCb(RobotGetVizInfo::Request& req, RobotGetVizInfo::Response& res)
try
{
  TEMOTO_DEBUG_STREAM_("Getting the visualization information of '" << req.robot_name << " ...");
  RobotPtr loaded_robot = findLoadedRobot(req.robot_name);
  res.info = loaded_robot->getVizInfo();
  res.success = true;
  return true;
}
catch(resource_registrar::TemotoErrorStack& e)
{
  res.success = false;
  return true;
}

bool RobotManager::getManipulationTargetCb(RobotGetTarget::Request& req, RobotGetTarget::Response& res)
try
{
  TEMOTO_DEBUG_STREAM_("Getting the manipulation target of '" << req.robot_name << " ...");
  RobotPtr loaded_robot = findLoadedRobot(req.robot_name);

  if (loaded_robot->isLocal())
  {
    switch (req.get_current_state)
    {
      case RobotGetTarget::Request::END_EFFECTOR:
        res.pose = loaded_robot->getManipulationTarget(req.planning_group);
        break;

      case RobotGetTarget::Request::JOINT_STATE:
        res.joint_values = loaded_robot->getCurrentJointValues(req.planning_group);
        break;

      default:
        throw TEMOTO_ERRSTACK("Unable to get manipulation target because the get_current_state is unknown");
    }
  }
  else
  {
    std::string topic = "/" + loaded_robot->getConfig()->getTemotoNamespace() + "/" 
      + srv_name::SERVER_GET_MANIPULATION_TARGET;
    TEMOTO_DEBUG_STREAM_("Forwarding the request to remote robot manager at '" << topic << "'.");

    ros::ServiceClient client_mode = nh_.serviceClient<RobotGetTarget>(topic);
    RobotGetTarget fwd_get_target_srvc;
    fwd_get_target_srvc.request = req;
    fwd_get_target_srvc.response = res;
    if (client_mode.call(fwd_get_target_srvc))
    {
      TEMOTO_DEBUG_("Call to remote RobotManager was sucessful.");
      res = fwd_get_target_srvc.response;
    }
    else
    {
      throw TEMOTO_ERRSTACK("Call to remote RobotManager service failed.");      
    }    
  }
  res.success = true;
  return true;
}
catch(resource_registrar::TemotoErrorStack& e)
{
  res.success = false;
  return true;
}

bool RobotManager::getManipulationNamedTargetsCb(RobotGetNamedTargets::Request& req, RobotGetNamedTargets::Response& res)
try
{
  TEMOTO_DEBUG_STREAM_("Getting the named targets of '" << req.robot_name << " ... with planning group " << req.planning_group );
  RobotPtr loaded_robot = findLoadedRobot(req.robot_name);

  if (loaded_robot->isLocal())
  {    
    res.named_target_poses = loaded_robot->getNamedTargetPoses(req.planning_group);
  }
  else
  {
    std::string topic = "/" + loaded_robot->getConfig()->getTemotoNamespace() + "/" 
      + srv_name::SERVER_GET_MANIPULATION_NAMED_TARGETS;
    TEMOTO_DEBUG_STREAM_("Forwarding the request to remote robot manager at '" << topic << "'.");

    ros::ServiceClient client_mode = nh_.serviceClient<RobotGetNamedTargets>(topic);
    RobotGetNamedTargets fwd_get_named_targets_srvc;
    fwd_get_named_targets_srvc.request = req;
    fwd_get_named_targets_srvc.response = res;
    if (client_mode.call(fwd_get_named_targets_srvc))
    {
      TEMOTO_DEBUG_("Call to remote RobotManager was sucessful.");
      res = fwd_get_named_targets_srvc.response;
    }
    else
    {
      throw TEMOTO_ERRSTACK("Call to remote RobotManager service failed.");      
    }    
  }
  res.success = true;
  return true;
}
catch(resource_registrar::TemotoErrorStack& e)
{
  res.success = false;
  return true;
}

bool RobotManager::goalNavigationCb(RobotNavigationGoal::Request& req, RobotNavigationGoal::Response& res)
try
{
  RobotPtr loaded_robot = findLoadedRobot(req.robot_name);
  if (loaded_robot->isLocal())
  {
    TEMOTO_DEBUG_STREAM_("Navigating '" << req.robot_name << " to pose: " << req.target_pose << " ...");
    loaded_robot->goalNavigation(req.target_pose);  // The robot would move with respect to the coordinate frame defined in the header
    res.success = true;   
  }
  else
  {
    std::string topic = "/" + loaded_robot->getConfig()->getTemotoNamespace() + "/"
      + srv_name::SERVER_NAVIGATION_GOAL;
    TEMOTO_DEBUG_STREAM_("Forwarding the request to remote robot manager at '" << topic << "'.");

    ros::ServiceClient client_navigation_goal_ = nh_.serviceClient<RobotNavigationGoal>(topic);
    RobotNavigationGoal fwd_goal_srvc;
    fwd_goal_srvc.request = req;
    fwd_goal_srvc.response = res;
    if (client_navigation_goal_.call(fwd_goal_srvc))
    {
      res = fwd_goal_srvc.response;
    }
    else
    {
      throw TEMOTO_ERRSTACK("Call to remote RobotManager service failed.");
    }
  }
  res.success = true;
  return true;  
}
catch(resource_registrar::TemotoErrorStack& e)
{
  res.success = false;
  return true;
}

void RobotManager::resourceStatusCb(RobotLoad srv_msg, temoto_resource_registrar::Status status_msg)
{
  TEMOTO_DEBUG_("status info was received");
  TEMOTO_DEBUG_STREAM_(srv_msg.request);

  // Check if any of the allocated robots has failed
  // Currently we simply remove the loaded robot if it failed
  if (true)
  {
    // was it a remote robot
    // if (loaded_robots_.erase(srv.request.resource_id))
    // {
    //   TEMOTO_DEBUG_("Removed remote robot, because its status failed.");
    //   return;
    // }

    // // check if it was a resource related to a robot feature has failed.
    // // unload the robot
    // for (auto it = loaded_robots_.begin(); it != loaded_robots_.end(); ++it)
    // {
    //   if (it->second->hasResource(srv.request.resource_id))
    //   {
    //     RobotConfigPtr config = it->second->getConfig();
    //     config->adjustReliability(0.0);
    //     YAML::Node yaml_config;
    //     yaml_config["Robots"].push_back(config->getYAMLConfig());
    //     PayloadType payload;
    //     payload.data = YAML::Dump(yaml_config);
    //     std::cout << payload << std::endl;
    //     config_syncer_.advertise(payload);
    //     loaded_robots_.erase(it);
    //     break;
    //   }
    // }
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
    auto it = std::copy_if(configs.begin()
    , configs.end()
    , std::back_inserter(candidates)
    , [&](const RobotConfigPtr& s) { return s->getName() == robot_name; });
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

bool RobotManager::gripperControlPositionCb(RobotGripperControlPosition::Request& req
, RobotGripperControlPosition::Response& res)
try
{
  TEMOTO_DEBUG_STREAM_("Commanding the gripper of '" << req.robot_name << " ...");
  RobotPtr loaded_robot = findLoadedRobot(req.robot_name);
  TEMOTO_INFO_STREAM_("GRIPPER CONTROL...");

  if (loaded_robot->isLocal())
  {
    loaded_robot->controlGripper(req.robot_name,req.control);
  }
  else
  {
    std::string topic = "/" + loaded_robot->getConfig()->getTemotoNamespace() + "/"
      + srv_name::SERVER_GRIPPER_CONTROL_POSITION;
    TEMOTO_DEBUG_STREAM_("Forwarding the execution request to remote robot manager at '" << topic << "'.");

    ros::ServiceClient client_gripper_control_position_ = nh_.serviceClient<RobotGripperControlPosition>(topic);
    RobotGripperControlPosition fwd_gripper_srvc;
    fwd_gripper_srvc.request = req;
    fwd_gripper_srvc.response = res;
    if (client_gripper_control_position_.call(fwd_gripper_srvc))
    {
      res = fwd_gripper_srvc.response;
    }
    else
    {      
      throw TEMOTO_ERRSTACK("Call to remote RobotManager service failed.");
    }
  }
  res.success = true;
  return true;
}
catch(resource_registrar::TemotoErrorStack& e)
{
  res.success = false;
  return true;
}

bool RobotManager::getRobotConfigCb(RobotGetConfig::Request& req, RobotGetConfig::Response& res)
{
  TEMOTO_DEBUG_STREAM_("Received a request to send the config of '" << req.robot_name << "'.");
  /*
   * Look for local robot configs
   */ 
  auto local_robot_config_it = std::find_if(
    local_configs_.begin(),
    local_configs_.end(),
    [&](const RobotConfigPtr p) -> bool 
    {
      return p->getName() == req.robot_name;
    });
  
  if (local_robot_config_it != local_configs_.end())
  { 
    TEMOTO_DEBUG_STREAM_("Found the config of '" << req.robot_name << "' in known local robot configs.");
    res.robot_config = (*local_robot_config_it)->getYamlConfigString();
    res.robot_absolute_namespace = (*local_robot_config_it)->getAbsRobotNamespace();
    res.success = true;
    return true;
  }

  /*
   * Look for remote robot configs
   */
  auto remote_robot_config_it = std::find_if(
    remote_configs_.begin(),
    remote_configs_.end(),
    [&](const RobotConfigPtr p) -> bool 
    {
      return p->getName() == req.robot_name;
    });
  
  if (remote_robot_config_it != remote_configs_.end())
  { 
    TEMOTO_DEBUG_STREAM_("Found the config of '" << req.robot_name << "' in known remote robot configs.");
    res.robot_config = (*remote_robot_config_it)->getYamlConfigString();
    res.robot_absolute_namespace = (*remote_robot_config_it)->getAbsRobotNamespace();
    res.success = true;
    return true;
  }

  //TODO: Add the correspondig error, for now using the plan code
  TEMOTO_INFO_STREAM_("Could not find robot '" << req.robot_name << "'");

  res.success = false;
  return true;
}

RobotManager::RobotPtr RobotManager::findLoadedRobot(const std::string& robot_name)
{
  auto robot_it = std::find_if(loaded_robots_.begin()
  , loaded_robots_.end()
  , [&](const RobotManager::RobotPtr p) -> bool
    {
      return p->getName() == robot_name;
    });  
  
  if (robot_it == loaded_robots_.end())
  {
    throw TEMOTO_ERRSTACK("Robot '" + robot_name + "' is not loaded.");
  }
  else if (*robot_it == nullptr)
  {
    throw TEMOTO_ERRSTACK("Robot '" + robot_name + "' is loaded but its configuration is invalid (nullptr).");
  }

  return *robot_it;
}

void RobotManager::restoreState()
{
  /*
   * 1) Read the restored RR catalog
   * 2) Extract all LoadRobot queries
   * 3) Per each LoadRobot query, find its according robot_config (extracted from robot_description.yaml)
   * 4) Instantiate the Robot object via its robot_config
   * 5) Per each Robot, invoke the "recover" method that accepts the ID of the LoadRobot query (parent ID)
   * 5.1) Robot::recover: Register a LoadProcess client to RR
   * 5.2) Robot::recover: get the sub-resource queries (LoadProcess)
   * 5.3) Robot::recover: Recover each robotic feature based on subresource, including assigning status callbacks per resource ID
   */

  resource_registrar_.loadCatalog();
  for (const auto& query : resource_registrar_.getServerQueries<RobotLoad>(srv_name::SERVER_LOAD))
  {
    auto robot_config = findRobot(query.request.robot_name, local_configs_);
    if (!robot_config)
    {
      // TODO: error this robot is not described in robot_description.yaml
      continue;
    }
    auto robot = std::make_shared<Robot>(robot_config, query.response.temoto_metadata.request_id, resource_registrar_
    , std::bind(&RobotManager::customFeatureUpdateCb, this, std::placeholders::_1));
    robot->recover(query.response.temoto_metadata.request_id);
    loaded_robots_.push_back(robot);
  }
}

}  // namespace temoto_robot_manager
