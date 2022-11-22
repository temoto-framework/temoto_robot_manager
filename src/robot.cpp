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

#include "temoto_robot_manager/robot.h"
#include "temoto_core/temoto_error/temoto_error.h"
#include "ros/package.h"

namespace temoto_robot_manager
{
Robot::Robot(RobotConfigPtr config
, const std::string& resource_id
, temoto_resource_registrar::ResourceRegistrarRos1& resource_registrar
, temoto_core::BaseSubsystem& b)
: config_(config)
, robot_resource_id_(resource_id)
, resource_registrar_(resource_registrar)
, is_plan_valid_(false)
, robot_operational_(true)
, state_in_error_(false)
, robot_loaded_(false)
, temoto_core::BaseSubsystem(b)
{
  class_name_ = __func__;
}

Robot::~Robot()
{
  if(isLocal())
  {
    // Unload features
    if (config_->getFeatureURDF().isLoaded())
    {
      TEMOTO_WARN("Unloading URDF Feature.");
      //resource_registrar_.unloadClientResource(config_->getFeatureURDF().getResourceId());
      config_->getFeatureURDF().setLoaded(false);
    }

    if (config_->getFeatureManipulation().isLoaded())
    {
      TEMOTO_WARN("Unloading Manipulation Feature.");
      //resource_registrar_.unloadClientResource(config_->getFeatureManipulation().getResourceId());
      config_->getFeatureManipulation().setLoaded(false);
    }

    if (config_->getFeatureManipulation().isDriverLoaded())
    {
      TEMOTO_WARN("Unloading Manipulation driver Feature.");
      //resource_registrar_.unloadClientResource(config_->getFeatureManipulation().getDriverResourceId());
      config_->getFeatureManipulation().setDriverLoaded(false);
    }

    if (config_->getFeatureNavigation().isLoaded())
    {
      TEMOTO_WARN("Unloading Navigation Feature.");
      //resource_registrar_.unloadClientResource(config_->getFeatureNavigation().getResourceId());
      config_->getFeatureNavigation().setLoaded(false);
    }

    if (config_->getFeatureNavigation().isDriverLoaded())
    {
      TEMOTO_WARN("Unloading Navigation driver Feature.");
      //resource_registrar_.unloadClientResource(config_->getFeatureNavigation().getDriverResourceId());
      config_->getFeatureNavigation().setDriverLoaded(false);
    }

    if (config_->getFeatureGripper().isLoaded())
    {
      TEMOTO_WARN("Unloading Gripper Feature.");
      //resource_registrar_.unloadClientResource(config_->getFeatureGripper().getResourceId());
      config_->getFeatureGripper().setLoaded(false);
    }

    if (config_->getFeatureGripper().isDriverLoaded())
    {
      TEMOTO_WARN("Unloading Gripper driver Feature.");
      //resource_registrar_.unloadClientResource(config_->getFeatureGripper().getDriverResourceId());
      config_->getFeatureGripper().setDriverLoaded(false);
    }
    
    // Remove parameters
    if(nh_.deleteParam(config_->getAbsRobotNamespace()))
    {
      TEMOTO_DEBUG("Parameter(s) removed successfully.");
    }
    else
    {
      TEMOTO_WARN("Parameter(s) not removed.");
    }
  }
  TEMOTO_DEBUG("Robot destructed");
}

void Robot::load()
{
  if (!isLocal())
  {
    return;
  }

  if (!config_->getFeatureURDF().isEnabled() && !config_->getFeatureManipulation().isEnabled() &&
      !config_->getFeatureNavigation().isEnabled() && !config_->getFeatureGripper().isEnabled())
  {
    throw CREATE_ERROR(temoto_core::error::Code::ROBOT_CONFIG_FAIL, "Robot is missing features. Please specify "
                                                       "urdf, manipulation, navigation, gripper sections in "
                                                       "the configuration file.");
  }

  // Load URDF
  if (config_->getFeatureURDF().isEnabled())
  {
    loadUrdf();
  }
 
  /*
   * Load navigation
   */ 
  if (config_->getFeatureNavigation().isDriverEnabled())
  {
    loadNavigationDriver();
  }

  if (config_->getFeatureNavigation().isEnabled())
  {
    loadNavigationController();
  }

  /*
   * Load manipulation
   */ 
  if (config_->getFeatureManipulation().isDriverEnabled())
  {
    loadManipulationDriver();
  }

  if (config_->getFeatureManipulation().isEnabled())
  {
    loadManipulationController();
  }

  /*
   * Load gripper
   */ 
  if (config_->getFeatureGripper().isDriverEnabled())
  {
    loadGripperDriver();
  }  

  if (config_->getFeatureGripper().isEnabled())
  {
    loadGripperController();
  }

  robot_loaded_ = true;
}

void Robot::waitForParam(const std::string& param)
{
  //\TODO: add 30 sec timeout protection.
  while (!nh_.hasParam(param))
  {
    TEMOTO_DEBUG("Waiting for %s ...", param.c_str());
    if (isInError())
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_STATUS_FAIL, "Loading interrupted. The robot is in a failed state.");
    }
    ros::Duration(1).sleep();
  }
  TEMOTO_DEBUG("Parameter '%s' was found.", param.c_str());
}

void Robot::waitForTopic(const std::string& topic)
{
  //\TODO: add 30 sec timeout protection.
  while (!isTopicAvailable(topic))
  {
    TEMOTO_DEBUG("Waiting for %s ...", topic.c_str());
    if (isInError())
    {
      throw CREATE_ERROR(temoto_core::error::Code::SERVICE_STATUS_FAIL, "Loading interrupted. The robot is in a failed state.");
    }
    ros::Duration(1).sleep();
  }
  TEMOTO_DEBUG("Topic '%s' was found.", topic.c_str());
}

bool Robot::isTopicAvailable(const std::string& topic)
{
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  auto it = std::find_if(master_topics.begin()
  , master_topics.end()
  , [&](ros::master::TopicInfo& master_topic) -> bool 
    { 
      return master_topic.name == topic; 
    });
  
  return it != master_topics.end();
}

// Load robot's urdf
void Robot::loadUrdf()
try
{
  FeatureURDF& ftr = config_->getFeatureURDF();
  std::string urdf_path = '/' + ros::package::getPath(ftr.getPackageName()) + '/' + ftr.getExecutable();
  auto load_er_msg = rosExecute("temoto_robot_manager", "urdf_loader.py", urdf_path);

  std::string robot_desc_param = config_->getAbsRobotNamespace() + "/robot_description";
  waitForParam(robot_desc_param);
  ftr.setLoaded(true);
  TEMOTO_DEBUG("Feature 'URDF' loaded.");
}
catch(temoto_core::error::ErrorStack& error_stack)
{
  throw FORWARD_ERROR(error_stack);
}

// Load move group and move group interfaces
void Robot::loadManipulationController()
{
  if (config_->getFeatureManipulation().isLoaded())
  {
    return; // Return if already loaded.
  }

  try
  {
    FeatureManipulation& ftr = config_->getFeatureManipulation();
    rosExecute(ftr.getPackageName(), ftr.getExecutable(), ftr.getArgs());
    //ftr.setResourceId(res_id);
    std::string desc_sem_param = config_->getAbsRobotNamespace() + "/robot_description_semantic";
    waitForParam(desc_sem_param);
    ros::Duration(5).sleep();

    // Add planning groups
    // TODO: read groups from srdf automatically
    for (auto group : ftr.getPlanningGroups())
    {
      TEMOTO_DEBUG("Adding planning group '%s'.", group.c_str());
      addPlanningGroup(group);
    }

    ftr.setLoaded(true);
    TEMOTO_DEBUG("Feature 'Manipulation Controller' loaded.");
  }
  catch(temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

// Load robot driver that will publish joint states and robot state
void Robot::loadManipulationDriver()
{
  if (config_->getFeatureManipulation().isDriverLoaded())
  {
    return; // Return if already loaded.
  }

  try
  {
    FeatureManipulation& ftr = config_->getFeatureManipulation();
    rosExecute(ftr.getDriverPackageName(), ftr.getDriverExecutable(), ftr.getDriverArgs());
    //ftr.setDriverResourceId(res_id);

    std::string joint_states_topic = config_->getAbsRobotNamespace() + "/joint_states";
    waitForTopic(joint_states_topic);

    ftr.setDriverLoaded(true);
    TEMOTO_DEBUG("Feature 'Manipulation Driver' loaded.");
  }
  catch(temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

// Load Move Base
void Robot::loadNavigationController()
{
  if (config_->getFeatureNavigation().isLoaded())
  {
    return; // Return if already loaded.
  }

  try
  {
    FeatureNavigation& ftr = config_->getFeatureNavigation();
    rosExecute(ftr.getPackageName(), ftr.getExecutable(), ftr.getArgs());
    //ftr.setResourceId(res_id);

    // wait for command velocity to be published
    std::string cmd_vel_topic = config_->getAbsRobotNamespace() + "/" + ftr.getCmdVelTopic();
    waitForTopic(cmd_vel_topic);

    // Subscribe to the pose messages
    if (!ftr.getPoseTopic().empty())
    {
      localized_pose_sub_ = nh_.subscribe(config_->getAbsRobotNamespace() + "/" + ftr.getPoseTopic()
      , 1
      , &Robot::robotPoseCallback
      , this);
    }

    ros::Duration(5).sleep();
    ftr.setLoaded(true);
    TEMOTO_DEBUG("Feature 'Navigation Controller' loaded.");
  }
  catch (temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

// Load robot driver that will publish odom
void Robot::loadNavigationDriver()
{
  if (config_->getFeatureNavigation().isDriverLoaded())
  {
    return; // Return if already loaded.
  }

  try
  {
    FeatureNavigation& ftr = config_->getFeatureNavigation();
    rosExecute(ftr.getDriverPackageName(), ftr.getDriverExecutable(), ftr.getDriverArgs());
    //ftr.setDriverResourceId(res_id);
    std::string odom_topic = config_->getAbsRobotNamespace() + "/" + ftr.getOdomTopic();
    waitForTopic(odom_topic);
    ftr.setDriverLoaded(true);
    TEMOTO_DEBUG("Feature 'Navigation Driver' loaded.");        
  }
  catch(temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

void Robot::loadGripperController()
{
  if (config_->getFeatureGripper().isLoaded())
  {
    return; // Return if already loaded.
  }

  try
  {
    FeatureGripper& ftr = config_->getFeatureGripper();
    rosExecute(ftr.getPackageName(), ftr.getExecutable(), ftr.getArgs());          
    //ftr.setResourceId(res_id);
    std::string gripper_topic = config_->getAbsRobotNamespace() + "/gripper_control";
    ros::service::waitForService(gripper_topic,-1);
    ftr.setLoaded(true);
    TEMOTO_DEBUG("Feature 'Gripper Controller' loaded.");
    
  }
  catch(temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

void Robot::loadGripperDriver()
{
  if (config_->getFeatureGripper().isDriverLoaded())
  {
    return; // Return if already loaded.
  }
  try
  {
    FeatureGripper& ftr = config_->getFeatureGripper();
    rosExecute(ftr.getDriverPackageName(), ftr.getDriverExecutable(), ftr.getDriverArgs());
    //ftr.setDriverResourceId(res_id);
    TEMOTO_DEBUG("Feature 'Gripper driver' loaded.");
    ros::Duration(5).sleep();
    TEMOTO_INFO_STREAM("Finish Load Gripper Driver ");
    //

  }
  catch(temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

temoto_er_manager::LoadExtResource Robot::rosExecute(const std::string& package_name
, const std::string& executable
, const std::string& args)
try
{
  temoto_er_manager::LoadExtResource load_proc_srvc;
  load_proc_srvc.request.package_name = package_name;
  load_proc_srvc.request.ros_namespace = config_->getAbsRobotNamespace(); //Execute in robot namespace
  load_proc_srvc.request.action = temoto_er_manager::action::ROS_EXECUTE;
  load_proc_srvc.request.executable = executable;
  load_proc_srvc.request.args = args;

  resource_registrar_.call<temoto_er_manager::LoadExtResource>(temoto_er_manager::srv_name::MANAGER
  , temoto_er_manager::srv_name::SERVER
  , load_proc_srvc
  , std::bind(&Robot::resourceStatusCb, this, std::placeholders::_1, std::placeholders::_2));

  return load_proc_srvc;
}
catch(temoto_core::error::ErrorStack& error_stack)
{
  throw FORWARD_ERROR(error_stack);
}

void Robot::resourceStatusCb(temoto_er_manager::LoadExtResource srv_msg
, temoto_resource_registrar::Status status_msg)
{
  TEMOTO_WARN_STREAM_("Received a status message: " << status_msg.message_);
  if (true /* TODO: check the type of the status message */)
  {
    setInError(true);
    setRobotOperational(false);
  }

  /* 
   * If the status message arrived while the robot was being loaded, then do not
   * run the recovery procedure
   */ 
  if (!robot_loaded_)
  {
    return;
  }
  else
  {
    setInError(false);
    resource_registrar_.unload(temoto_er_manager::srv_name::MANAGER
    , srv_msg.response.temoto_metadata.request_id);

    auto load_er_query = rosExecute(srv_msg.request.package_name
    , srv_msg.request.executable
    , srv_msg.request.args);

    resource_registrar_.registerDependency(temoto_er_manager::srv_name::MANAGER
    , load_er_query.response.temoto_metadata.request_id
    , robot_resource_id_);

    // TODO: make sure that the feature is actually properly loaded and obviously all of that
    // has to be completely restructured
    FeatureNavigation& ftr = config_->getFeatureNavigation();
    if (ftr.getPackageName() == srv_msg.request.package_name &&
        ftr.getExecutable()  == srv_msg.request.executable)
    {
      TEMOTO_WARN_STREAM_("The controller of " << config_->getName() << " crashed, restarting it ...");
      // wait for command velocity to be published
      std::string cmd_vel_topic = config_->getAbsRobotNamespace() + "/" + ftr.getCmdVelTopic();
      waitForTopic(cmd_vel_topic);
    }
    else if (ftr.getDriverPackageName() == srv_msg.request.package_name &&
             ftr.getDriverExecutable()  == srv_msg.request.executable)
    {
      TEMOTO_WARN_STREAM_("The driver of " << config_->getName() << " crashed, restarting it ...");
      // wait for command velocity to be published
      std::string odom_topic = config_->getAbsRobotNamespace() + "/" + ftr.getOdomTopic();
      waitForTopic(odom_topic);
      ftr.setDriverLoaded(true);
    }

     // Send the initial pose
    ros::Publisher pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(config_->getAbsRobotNamespace() + "/initialpose", 10);
    while(pub.getNumSubscribers() < 1)
    {
      ros::Duration(0.5).sleep();
    }
      
    // TODO: the inital pose subscriber sometimes ignores the message, hence the message is sent multiple times ...
    pub.publish(current_pose_navigation_);
    ros::Duration(1).sleep();
    pub.publish(current_pose_navigation_);
    ros::Duration(1).sleep();
    pub.publish(current_pose_navigation_);
    ros::Duration(1).sleep();
    pub.publish(current_pose_navigation_);
    ros::Duration(1).sleep();
    pub.publish(current_pose_navigation_);
    ros::Duration(1).sleep();
    pub.publish(current_pose_navigation_);
    ros::Duration(1).sleep();
    pub.publish(current_pose_navigation_);
    ros::Duration(1).sleep();
    pub.publish(current_pose_navigation_);
    ros::Duration(1).sleep();
    pub.publish(current_pose_navigation_);
    ros::Duration(1).sleep();
    pub.publish(current_pose_navigation_);

    setRobotOperational(true);
  }
}

void Robot::addPlanningGroup(const std::string& planning_group_name)
{
  //Prepare robot description path and a nodehandle, which is in robot's namespace
  std::string rob_desc = config_->getAbsRobotNamespace() + "/robot_description";
  ros::NodeHandle mg_nh(config_->getAbsRobotNamespace());
  moveit::planning_interface::MoveGroupInterface::Options opts(planning_group_name, rob_desc, mg_nh);
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> group(
      new moveit::planning_interface::MoveGroupInterface(opts));
  group->setPlannerId("RRTConnectkConfigDefault");
  //group->setPlannerId("ESTkConfigDefault");
  group->setNumPlanningAttempts(2);
  group->setPlanningTime(5);

  // Playing around with tolerances
  group->setGoalPositionTolerance(0.001);
  group->setGoalOrientationTolerance(0.001);
  group->setGoalJointTolerance(0.001);
  TEMOTO_DEBUG("Active end effector link: %s", group->getEndEffectorLink().c_str());

  planning_groups_.emplace(planning_group_name, std::move(group));
}

void Robot::removePlanningGroup(const std::string& planning_group_name)
{
  planning_groups_.erase(planning_group_name);
}

void Robot::planManipulationPath(std::string& planning_group_name, const geometry_msgs::PoseStamped& target_pose)
{
  if (!planning_groups_.size())
  {
    throw CREATE_ERROR(temoto_core::error::Code::ROBOT_PLAN_FAIL,"Robot has no planning groups.");
  }

  FeatureManipulation& ftr = config_->getFeatureManipulation();

  planning_group_name = (planning_group_name == "") ? ftr.getActivePlanningGroup() : planning_group_name;
  auto group_it = planning_groups_.find(planning_group_name);
  if (group_it == planning_groups_.end())
  {
    throw CREATE_ERROR(temoto_core::error::Code::PLANNING_GROUP_NOT_FOUND, "Planning group '%s' was not found.",
                       planning_group_name.c_str());
  }
  ftr.setActivePlanningGroup(planning_group_name);
  group_it->second->setStartStateToCurrentState();
  group_it->second->setPoseTarget(target_pose);
  
  is_plan_valid_ = static_cast<bool>(group_it->second->plan(last_plan));
  
  TEMOTO_DEBUG("Plan %s",  is_plan_valid_ ? "FOUND" : "FAILED");
  if(!is_plan_valid_)
  {
    throw CREATE_ERROR(temoto_core::error::Code::ROBOT_PLAN_FAIL,"Planning with group '%s' failed.", group_it->first.c_str());
  }
}

void Robot::planManipulationPath(std::string& planning_group_name, const std::string& named_target)
{
  if (!planning_groups_.size())
  {
    throw CREATE_ERROR(temoto_core::error::Code::ROBOT_PLAN_FAIL,"Robot has no planning groups.");
  }

  FeatureManipulation& ftr = config_->getFeatureManipulation();

  planning_group_name = (planning_group_name == "") ? ftr.getActivePlanningGroup() : planning_group_name;
  auto group_it = planning_groups_.find(planning_group_name);
  if (group_it == planning_groups_.end())
  {
    throw CREATE_ERROR(temoto_core::error::Code::PLANNING_GROUP_NOT_FOUND, "Planning group '%s' was not found.",
                       planning_group_name.c_str());
  }
  ftr.setActivePlanningGroup(planning_group_name);
  group_it->second->setStartStateToCurrentState();
  if (!group_it->second->setNamedTarget(named_target))
  {
    is_plan_valid_ = false;
    throw CREATE_ERROR(temoto_core::error::Code::ROBOT_PLAN_FAIL,"Planning to named target pose '%s' failed.", named_target);
  }
  
  is_plan_valid_ = static_cast<bool>(group_it->second->plan(last_plan));
  
  TEMOTO_DEBUG("Plan %s",  is_plan_valid_ ? "FOUND" : "FAILED");
  if(!is_plan_valid_)
  {
    throw CREATE_ERROR(temoto_core::error::Code::ROBOT_PLAN_FAIL,"Planning with group '%s' failed.", group_it->first.c_str());
  }
}

void Robot::executeManipulationPath()
{
  std::string planning_group_name = config_->getFeatureManipulation().getActivePlanningGroup();
  moveit::planning_interface::MoveGroupInterface::Plan empty_plan;
  if (!is_plan_valid_)
  {
    TEMOTO_ERROR("Unable to execute group '%s' without a plan.", planning_group_name.c_str());
    return;
  }
  auto group_it =
      planning_groups_.find(planning_group_name);  ///< Will throw if group does not exist
  if (group_it != planning_groups_.end())
  {
    bool success = false;
    group_it->second->setStartStateToCurrentState();
    group_it->second->setRandomTarget();
    //success = static_cast<bool>(group_it->second->execute(last_plan));
    size_t i=0;
    while (!success && i<3) 
    {
      TEMOTO_INFO_STREAM("Attempt = " << i+1);
      ++i;
      success = static_cast<bool>(group_it->second->execute(last_plan));
    }

    TEMOTO_DEBUG("Execution %s",  success ? "SUCCESSFUL" : "FAILED");
    if(!success)
    {
      throw CREATE_ERROR(temoto_core::error::Code::ROBOT_EXEC_FAIL,"Execute plan with group '%s' failed.", planning_group_name.c_str());
    }
  }
  else
  {
    TEMOTO_ERROR("Planning group '%s' was not found.", planning_group_name.c_str());
    throw CREATE_ERROR(temoto_core::error::Code::PLANNING_GROUP_NOT_FOUND, "Planning group '%s' was not found.",
                       planning_group_name.c_str());
  }
}

geometry_msgs::Pose Robot::getManipulationTarget()
{
  std::string planning_group_name = config_->getFeatureManipulation().getActivePlanningGroup();
  
  auto group_it = planning_groups_.find(planning_group_name);
  TEMOTO_INFO_STREAM(planning_group_name.c_str());

  geometry_msgs::Pose current_pose;
  
  if (group_it != planning_groups_.end())
  {    
    current_pose = group_it->second->getCurrentPose().pose;    
  }
  else 
  {
    //TODO: This section has to utilize temoto error management system
    TEMOTO_ERROR("Planning group '%s' was not found.", planning_group_name.c_str());
  } 
  return current_pose;  
}

std::vector<std::string> Robot::getNamedTargetPoses(std::string& planning_group_name)
{
  if (!planning_groups_.size())
  {
    throw CREATE_ERROR(temoto_core::error::Code::ROBOT_PLAN_FAIL,"Robot has no planning groups.");
  }

  FeatureManipulation& ftr = config_->getFeatureManipulation();

  planning_group_name = (planning_group_name == "") ? ftr.getActivePlanningGroup() : planning_group_name;
  auto group_it = planning_groups_.find(planning_group_name);
  if (group_it == planning_groups_.end())
  {
    throw CREATE_ERROR(temoto_core::error::Code::PLANNING_GROUP_NOT_FOUND, "Planning group '%s' was not found.",
                       planning_group_name.c_str());
  }
  //ftr.setActivePlanningGroup(planning_group_name);

  return group_it->second->getNamedTargets();

}

void Robot::goalNavigation(const std::string& reference_frame, const geometry_msgs::PoseStamped& target_pose)
{
  if (!isRobotOperational())
  {
    throw TEMOTO_ERRSTACK("Could not navigate the robot because robot is not operational");
  }

  FeatureNavigation& ftr = config_->getFeatureNavigation();
  std::string act_rob_ns = config_->getAbsRobotNamespace() + "/move_base";
  MoveBaseClient ac(act_rob_ns, true);    
  
  if (!ac.waitForServer(ros::Duration(5.0)))
  {
    TEMOTO_ERRSTACK("The move_base action server did not come up");
  }

  move_base_msgs::MoveBaseGoal goal;  
  goal.target_pose.pose = target_pose.pose;
  goal.target_pose.header.frame_id = reference_frame;         
  goal.target_pose.header.stamp = ros::Time::now();  
  ac.sendGoal(goal);

  // Wait until either the goal is finished or robot has encountered a system issue
  while((ac.getState() == actionlib::SimpleClientGoalState::PENDING || ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
     && isRobotOperational())
  {
    ros::Duration(1).sleep();
  }

  if (!isRobotOperational())
  {
    ac.cancelGoal();
    throw TEMOTO_ERRSTACK("Could not finish the navigation goal because the robot is not operational");
  }
  else if(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    throw TEMOTO_ERRSTACK("The base failed to move");
  }
}

void Robot::controlGripper(const std::string& robot_name,const float position)
{
  try
  {
    FeatureGripper& ftr = config_->getFeatureGripper();   
    std::string argument = std::to_string(position);
    std::string gripper_topic = config_->getAbsRobotNamespace() + "/gripper_control";
    
    TEMOTO_DEBUG("Feature 'Gripper' loaded.");
    client_gripper_control_ = nh_.serviceClient<temoto_robot_manager::GripperControl>(gripper_topic);
    temoto_robot_manager::GripperControl gripper_srvc;
    gripper_srvc.request.robot_name = robot_name;
    gripper_srvc.request.position = position;    
    if (client_gripper_control_.call(gripper_srvc))
    {
      TEMOTO_DEBUG("Call to gripper control was sucessful.");
    }
    else
    {
      TEMOTO_ERROR("Call to remote RobotManager service failed.");
    }  
  }
  catch(temoto_core::error::ErrorStack& error_stack)
  {
    throw FORWARD_ERROR(error_stack);
  }
}

bool Robot::isLocal() const
{
  if (config_) 
  {
    return config_->getTemotoNamespace() == ::temoto_core::common::getTemotoNamespace();
  }
 return true; // some default that should never reached. 
}

std::string Robot::getVizInfo()
{
  std::string act_rob_ns = config_->getAbsRobotNamespace();
  YAML::Node info;
  YAML::Node rviz = info["RViz"];

  // RViz options
  //TODO: SYNC info about robot features and if thet are actually loaded or not.

  if (config_->getFeatureURDF().isEnabled())
  {
    rviz["urdf"]["robot_description"] = act_rob_ns + "/robot_description";
  }

  if (config_->getFeatureManipulation().isEnabled())
  {
    rviz["manipulation"]["move_group_ns"] = act_rob_ns;
    rviz["manipulation"]["active_planning_group"] =
        config_->getFeatureManipulation().getActivePlanningGroup();
  }

  if (config_->getFeatureNavigation().isEnabled())
  {
    rviz["navigation"]["move_base_ns"] = act_rob_ns;
    rviz["navigation"]["global_planner"] = config_->getFeatureNavigation().getGlobalPlanner();
    rviz["navigation"]["local_planner"] = config_->getFeatureNavigation().getLocalPlanner();
  }

  if (config_->getFeatureGripper().isEnabled())
  {
    rviz["gripper"]["gripper_ns"] = act_rob_ns;    
  }
  
  return YAML::Dump(info);
}

bool Robot::isRobotOperational() const
{
  std::lock_guard<std::recursive_mutex> lock(robot_operational_mutex_);
  return robot_operational_;
}

void Robot::setRobotOperational(bool robot_operational)
{
  std::lock_guard<std::recursive_mutex> lock(robot_operational_mutex_);
  robot_operational_ = robot_operational;
}

void Robot::robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  if (isRobotOperational())
  {
    current_pose_navigation_ = msg;
  }
}

void Robot::recover(const std::string& parent_query_id)
{
  /*
   * 1) get the sub-resource queries (LoadExtResource)
   * 2) Recover each robotic feature based on subresource, including assigning status callbacks per resource ID
   * TODO: this method needs data race protection via mutexes
   */

  auto erm_queries = resource_registrar_.getRosChildQueries<temoto_er_manager::LoadExtResource>(parent_query_id
  , temoto_er_manager::srv_name::SERVER);

  TEMOTO_DEBUG_STREAM_("size of erm_queries: " << erm_queries.size());
  for (const auto& erm_query : erm_queries)
  {
    TEMOTO_DEBUG_STREAM("ERM query: " << erm_query.second.request);
    resource_registrar_.registerClientCallback<temoto_er_manager::LoadExtResource>(temoto_er_manager::srv_name::MANAGER
    , temoto_er_manager::srv_name::SERVER
    , erm_query.second.response.temoto_metadata.request_id
    , std::bind(&Robot::resourceStatusCb, this, std::placeholders::_1, std::placeholders::_2));
  }

  /*
   * TODO: each erm query has to be check against the enabled features
   */

  // Recover URDF feature
  if (config_->getFeatureURDF().isEnabled())
  {
    config_->getFeatureURDF().setLoaded(true);
  }
  
  // Recover the navigation driver
  if (config_->getFeatureNavigation().isDriverEnabled())
  {
    config_->getFeatureNavigation().setDriverLoaded(true);
  }

  // Recover the navigation controller
  if (config_->getFeatureNavigation().isEnabled())
  {
    // Subscribe to the pose messages
    if (!config_->getFeatureNavigation().getPoseTopic().empty())
    {
      localized_pose_sub_ = nh_.subscribe(config_->getAbsRobotNamespace() + "/" + config_->getFeatureNavigation().getPoseTopic()
      , 1
      , &Robot::robotPoseCallback
      , this);
    }
    config_->getFeatureNavigation().setLoaded(true);
  }

  robot_loaded_ = true;
  setRobotOperational(true);
}

void Robot::setResourceId(const std::string& resource_id)
{
  robot_resource_id_ = resource_id;
}

void Robot::setInError(bool state_in_error)
{
  std::lock_guard<std::recursive_mutex> lock(robot_state_in_error_mutex_);
  state_in_error_ = state_in_error;
}

bool Robot::isInError() const
{
  std::lock_guard<std::recursive_mutex> lock(robot_state_in_error_mutex_);
  return state_in_error_;
}
}


