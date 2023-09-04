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
#include "temoto_robot_manager/robot.h"
#include "temoto_robot_manager/custom_plugin_base.h"
#include "temoto_robot_manager/navigation_plugin_base.h"
#include "temoto_resource_registrar/temoto_error.h"

namespace temoto_robot_manager
{
Robot::Robot(RobotConfigPtr config
, const std::string& resource_id
, temoto_resource_registrar::ResourceRegistrarRos1& resource_registrar
, CustomFeatureUpdateCb custom_feature_update_cb)
: config_(config)
, robot_resource_id_(resource_id)
, resource_registrar_(resource_registrar)
, custom_feature_update_cb_(custom_feature_update_cb)
, is_plan_valid_(false)
, robot_operational_(true)
, state_in_error_(false)
, robot_loaded_(false)
, custom_feature_feedback_thread_running_(false)
{}

Robot::~Robot()
{
  if(!isLocal())
  {
    return;
  }

  // Unload features
  if (config_->getFeatureURDF().isLoaded())
  {
    TEMOTO_DEBUG_("Unloading URDF Feature.");
    config_->getFeatureURDF().setLoaded(false);
  }

  for (auto& common_procedure : config_->getCommonProcedures())
  {
    if (!common_procedure.second.isLoaded())
    {
      continue;
    }
    TEMOTO_DEBUG_("Unloading Common Procedure.");
    common_procedure.second.setLoaded(false);
  }

  if (config_->getFeatureManipulation().isLoaded())
  {
    TEMOTO_DEBUG_("Unloading Manipulation Feature.");
    config_->getFeatureManipulation().setLoaded(false);
  }

  if (config_->getFeatureManipulation().isDriverLoaded())
  {
    TEMOTO_DEBUG_("Unloading Manipulation driver Feature.");
    config_->getFeatureManipulation().setDriverLoaded(false);
  }

  if (config_->getFeatureNavigation().isLoaded())
  {
    TEMOTO_DEBUG_("Unloading Navigation Feature.");
    config_->getFeatureNavigation().setLoaded(false);
  }

  if (config_->getFeatureNavigation().isDriverLoaded())
  {
    TEMOTO_DEBUG_("Unloading Navigation driver Feature.");
    config_->getFeatureNavigation().setDriverLoaded(false);
  }

  if (config_->getFeatureGripper().isLoaded())
  {
    TEMOTO_DEBUG_("Unloading Gripper Feature.");
    config_->getFeatureGripper().setLoaded(false);
  }

  if (config_->getFeatureGripper().isDriverLoaded())
  {
    TEMOTO_DEBUG_("Unloading Gripper driver Feature.");
    config_->getFeatureGripper().setDriverLoaded(false);
  }

  // First stop the feedback thread
  custom_feature_feedback_thread_running_ = false;
  while (!custom_feature_feedback_thread_.joinable())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  custom_feature_feedback_thread_.join();

  for (auto& custom_feature : config_->getCustomFeatures())
  {
    /*
     * Unload the controller
     */
    if (custom_feature.second.isLoaded())
    {
      std::lock_guard<std::mutex> l(custom_feature_plugins_mutex_);

      auto custom_feature_plugin_it = custom_feature_plugins_.find(custom_feature.second.getName());
      if (custom_feature_plugin_it == custom_feature_plugins_.end())
      {
        TEMOTO_ERROR_STREAM_("Feature '" << custom_feature.second.getName() 
        << "' of robot '" << config_->getName() << "' not found.");
        continue;
      }

      try
      {
        custom_feature_plugins_.erase(custom_feature_plugin_it);
      }
      catch(resource_registrar::TemotoErrorStack& e)
      {
        TEMOTO_ERROR_STREAM_("Unable to uninitialize plugin '" << custom_feature.second.getName() 
        << "' of robot '" << config_->getName() << "'.");
      }

      custom_feature.second.setLoaded(false);
    }
    else
    {
      TEMOTO_WARN_STREAM_("Plugin '" << custom_feature.second.getName() 
      << "' of robot '" << config_->getName() << "' not loaded.");
    }

    /*
     * Unload the driver
     */
    if (custom_feature.second.isDriverLoaded())
    {
      custom_feature.second.setDriverLoaded(false);
    }
  }
  
  // Remove parameters
  if (config_->getFeatureURDF().isLoaded())
  {
    if(nh_.deleteParam(config_->getAbsRobotNamespace()))
    {
      TEMOTO_DEBUG_("Parameter(s) removed successfully.");
    }
    else
    {
      TEMOTO_WARN_("Parameter(s) not removed.");
    }
  }

  TEMOTO_DEBUG_("Robot destructed");
}

void Robot::load()
{
  if (!isLocal())
  {
    return;
  }

  if (!config_->getFeatureURDF().isEnabled() && !config_->getFeatureManipulation().isEnabled() &&
      !config_->getFeatureNavigation().isEnabled() && !config_->getFeatureGripper().isEnabled() &&
      !config_->getCustomFeatures().begin()->second.isEnabled() &&
      !config_->getCommonProcedures().begin()->second.isDefined())
  {
    throw TEMOTO_ERRSTACK("Robot is missing features. Please specify "
                          "urdf, manipulation, navigation, gripper or custom sections in "
                          "the configuration file.");
  }

  // Load URDF
  if (config_->getFeatureURDF().isEnabled())
  {
    loadUrdf();
  }

  /*
   * Load common procedures
   */
  for (auto& common_procedure : config_->getCommonProcedures())
  {
    if (common_procedure.second.isDefined())
    {
      TEMOTO_INFO_(common_procedure.second.getName());
      loadCommonProcedure(common_procedure.second.getName());
    }
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

  /*
   * Load custom features
   */
  for (auto& custom_feature : config_->getCustomFeatures())
  {
    if (custom_feature.second.isEnabled())
    {
      loadCustomController(custom_feature.second.getName());
    }

    if (custom_feature.second.isDriverEnabled())
    {
      loadCustomDriver(custom_feature.second.getName());
    }
  }

  robot_loaded_ = true;
}

void Robot::waitForParam(const std::string& param)
{
  //\TODO: add 30 sec timeout protection.
  while (!nh_.hasParam(param))
  {
    TEMOTO_DEBUG_("Waiting for %s ...", param.c_str());
    if (isInError())
    {
      throw TEMOTO_ERRSTACK("Loading interrupted. The robot is in a failed state.");
    }
    ros::Duration(1).sleep();
  }
  TEMOTO_DEBUG_("Parameter '%s' was found.", param.c_str());
}

void Robot::waitForTopic(const std::string& topic)
{
  //\TODO: add 30 sec timeout protection.
  while (!isTopicAvailable(topic))
  {
    TEMOTO_DEBUG_("Waiting for %s ...", topic.c_str());
    if (isInError())
    {
      throw TEMOTO_ERRSTACK("Loading interrupted. The robot is in a failed state.");
    }
    ros::Duration(1).sleep();
  }
  TEMOTO_DEBUG_("Topic '%s' was found.", topic.c_str());
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
  std::string urdf_path = ros::package::getPath(ftr.getPackageName()) + '/' + ftr.getExecutable();
  std::string args = ftr.getArgs();
  std::string cmd = urdf_path + " " + args;

  auto load_er_msg = rosExecute("temoto_robot_manager", "urdf_loader.py", cmd);

  std::string robot_desc_param = "/" + config_->getAbsRobotNamespace() + "/robot_description";
  waitForParam(robot_desc_param);
  ftr.setLoaded(true);
  TEMOTO_DEBUG_("Feature 'URDF' loaded.");
}
catch(resource_registrar::TemotoErrorStack& error_stack)
{
  throw FWD_TEMOTO_ERRSTACK(error_stack);
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
    std::string desc_sem_param = "/" + config_->getAbsRobotNamespace() + "/robot_description_semantic";
    waitForParam(desc_sem_param);
    ros::Duration(5).sleep();

    // Add planning groups
    // TODO: read groups from srdf automatically
    for (auto group : ftr.getPlanningGroups())
    {
      TEMOTO_DEBUG_("Adding planning group '%s'.", group.c_str());
      addPlanningGroup(group);
    }

    ftr.setLoaded(true);
    TEMOTO_DEBUG_("Feature 'Manipulation Controller' loaded.");
  }
  catch(resource_registrar::TemotoErrorStack& error_stack)
  {
    throw FWD_TEMOTO_ERRSTACK(error_stack);
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

    std::string joint_states_topic = "/" + config_->getAbsRobotNamespace() + "/joint_states";
    waitForTopic(joint_states_topic);

    ftr.setDriverLoaded(true);
    TEMOTO_DEBUG_("Feature 'Manipulation Driver' loaded.");
  }
  catch(resource_registrar::TemotoErrorStack& error_stack)
  {
    throw FWD_TEMOTO_ERRSTACK(error_stack);
  }
}

// Load Move Base
void Robot::loadNavigationController()
{
  TEMOTO_INFO_("0 ================= Loading Navigation contoller ==================");
  if (config_->getFeatureNavigation().isLoaded())
  {
    return; // Return if already loaded.
  }
  TEMOTO_INFO_("1 ================= Loading Navigation contoller ==================");
  try
  {
    FeatureNavigation& ftr = config_->getFeatureNavigation();
    TEMOTO_INFO_("2 ================= Loading Navigation contoller ==================");
    if (ftr.getExecutableType() == "ros")
    {
      // Previous Implementation 
      rosExecute(ftr.getPackageName(), ftr.getExecutable(), ftr.getArgs());
      // wait for command velocity to be published
      std::string cmd_vel_topic = "/" + config_->getAbsRobotNamespace() + "/" + ftr.getCmdVelTopic();
      waitForTopic(cmd_vel_topic);

      // Subscribe to the pose messages
      if (!ftr.getPoseTopic().empty())
      {
        localized_pose_sub_ = nh_.subscribe("/" + config_->getAbsRobotNamespace() + "/" + ftr.getPoseTopic()
        , 1
        , &Robot::robotPoseCallback
        , this);
      }
    }
    else if (ftr.getExecutableType() == "lib")
    {
      TEMOTO_INFO_("Navigation contoller lib");
      try
      {
        const std::string& plugin_path = ftr.getExecutable();
        TEMOTO_INFO_("Executable");
        TEMOTO_INFO_(plugin_path);
        NavigationPluginHelperPtr plugin_helper = std::make_shared<NavigationPluginHelper>(plugin_path, navigation_feature_update_cb_);

        std::lock_guard<std::mutex> l(navigation_feature_plugins_mutex_);
        // ftr.setLoaded(true);
        

        // /*
        // * Start the custom feature feedback thread
        // */
        // if (navigation_feature_feedback_thread_running_)
        // {
        //   return;
        // }

        // navigation_feature_feedback_thread_running_ = true;
        // navigation_feature_feedback_thread_ = std::thread(
        // [&]
        // {
        //   TEMOTO_DEBUG_("Custom feature feedback thread running");

        //   while (navigation_feature_feedback_thread_running_)
        //   {
        //     std::lock_guard<std::mutex> l(custom_feature_plugins_mutex_);

        //     for (const auto& cfp : custom_feature_plugins_)
        //     {
        //       cfp.second->sendUpdate();
        //       std::this_thread::sleep_for(std::chrono::milliseconds(50));
        //     }

        //     std::this_thread::sleep_for(std::chrono::milliseconds(200));
        //   }

        //   TEMOTO_DEBUG_("Custom feature feedback thread finished");
        // });
      }
      catch(resource_registrar::TemotoErrorStack& error_stack)
      {
        throw FWD_TEMOTO_ERRSTACK(error_stack);
      }
      catch(std::exception& e)
      {
        throw TEMOTO_ERRSTACK(e.what());
      }
      catch(...)
      {
        throw TEMOTO_ERRSTACK("Could not load navigation feature");
      }
    }

    TEMOTO_INFO_("3 ================= Loadng Navigation contoller ==================");

    ros::Duration(5).sleep();
    ftr.setLoaded(true);
    TEMOTO_DEBUG_("Feature 'Navigation Controller' loaded.");
  }
  catch (resource_registrar::TemotoErrorStack& error_stack)
  {
    throw FWD_TEMOTO_ERRSTACK(error_stack);
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
    std::string odom_topic = "/" + config_->getAbsRobotNamespace() + "/" + ftr.getOdomTopic();
    TEMOTO_INFO_(" ===== Loadng Navigation driver =====  Waitinf for topic =====");
    TEMOTO_INFO_(odom_topic);

    waitForTopic(odom_topic);
    ftr.setDriverLoaded(true);
    TEMOTO_DEBUG_("Feature 'Navigation Driver' loaded.");        
  }
  catch(resource_registrar::TemotoErrorStack& error_stack)
  {
    throw FWD_TEMOTO_ERRSTACK(error_stack);
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
    std::string gripper_topic = "/" + config_->getAbsRobotNamespace() + "/gripper_control";
    ros::service::waitForService(gripper_topic,-1);
    ftr.setLoaded(true);
    TEMOTO_DEBUG_("Feature 'Gripper Controller' loaded.");
    
  }
  catch(resource_registrar::TemotoErrorStack& error_stack)
  {
    throw FWD_TEMOTO_ERRSTACK(error_stack);
  }
}

void Robot::loadGripperDriver()
try
{
  if (config_->getFeatureGripper().isDriverLoaded())
  {
    return; // Return if already loaded.
  }
  
  FeatureGripper& ftr = config_->getFeatureGripper();
  rosExecute(ftr.getDriverPackageName(), ftr.getDriverExecutable(), ftr.getDriverArgs());
  ros::Duration(5).sleep();
  TEMOTO_DEBUG_("Feature 'Gripper driver' loaded.");
  ftr.setDriverLoaded(true);
}
catch(resource_registrar::TemotoErrorStack& error_stack)
{
  throw FWD_TEMOTO_ERRSTACK(error_stack);
}

void Robot::loadCustomController(const std::string& feature_name)
try
{
  auto custom_feature_it = config_->getCustomFeatures().find(feature_name);
  if (custom_feature_it == config_->getCustomFeatures().end())
  {
    throw TEMOTO_ERRSTACK("Could not find feature '" + feature_name + "'");
  }

  /*
   * Load the plugin
   *   TODO: Check if the plugin '.so' file even exists. If it doesn't
   *   the class_loader will just crash ...
   */
  const std::string& plugin_path = custom_feature_it->second.getExecutable();
  CustomPluginHelperPtr plugin_helper = std::make_shared<CustomPluginHelper>(plugin_path, custom_feature_update_cb_);

  std::lock_guard<std::mutex> l(custom_feature_plugins_mutex_);
  {
    custom_feature_plugins_.insert({feature_name, plugin_helper});
    config_->getCustomFeatures().at(feature_name).setLoaded(true);
  }

  /*
   * Start the custom feature feedback thread
   */
  if (custom_feature_feedback_thread_running_)
  {
    return;
  }

  custom_feature_feedback_thread_running_ = true;
  custom_feature_feedback_thread_ = std::thread(
  [&]
  {
    TEMOTO_DEBUG_("Custom feature feedback thread running");

    while (custom_feature_feedback_thread_running_)
    {
      std::lock_guard<std::mutex> l(custom_feature_plugins_mutex_);

      for (const auto& cfp : custom_feature_plugins_)
      {
        cfp.second->sendUpdate();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    TEMOTO_DEBUG_("Custom feature feedback thread finished");
  });
}
catch(resource_registrar::TemotoErrorStack& error_stack)
{
  throw FWD_TEMOTO_ERRSTACK(error_stack);
}
catch(std::exception& e)
{
  throw TEMOTO_ERRSTACK(e.what());
}
catch(...)
{
  throw TEMOTO_ERRSTACK("Could not load custom feature '" + feature_name + "'");
}

void Robot::loadCustomDriver(const std::string& feature_name)
{
  // TODO
}

void Robot::invokeCustomFeature(const std::string& custom_feature_name, const RmCustomRequestWrap& request)
try
{
  auto custom_feature_plugin_it = custom_feature_plugins_.find(custom_feature_name);
  if (custom_feature_plugin_it == custom_feature_plugins_.end())
  {
    throw TEMOTO_ERRSTACK("Feature '" + custom_feature_name  + "' of robot '" + config_->getName() + "' not found.");
  }

  custom_feature_plugin_it->second->invoke(request);

  //custom_feature_update_cb_();
}
catch(resource_registrar::TemotoErrorStack& e)
{
  std::string message = "Unable to invoke feature '" + custom_feature_name  + "' of robot '" + config_->getName() + "'.";
  throw FWD_TEMOTO_ERRSTACK_WMSG(e, message);
}


void Robot::preemptCustomFeature(const std::string& custom_feature_name)
try
{
  auto custom_feature_plugin_it = custom_feature_plugins_.find(custom_feature_name);
  if (custom_feature_plugin_it == custom_feature_plugins_.end())
  {
    throw TEMOTO_ERRSTACK("Feature '" + custom_feature_name  + "' of robot '" + config_->getName() + "' not found.");
  }

  custom_feature_plugin_it->second->preempt();
}
catch(resource_registrar::TemotoErrorStack& e)
{
  std::string message = "Unable to pre-empt feature '" + custom_feature_name  + "' of robot '" + config_->getName() + "'.";
  throw FWD_TEMOTO_ERRSTACK_WMSG(e, message);
}

void Robot::loadCommonProcedure(const std::string& procedure_name)
try
{
  auto common_procedure_it = config_->getCommonProcedures().find(procedure_name);
  if (common_procedure_it == config_->getCommonProcedures().end())
  {
    throw TEMOTO_ERRSTACK("Could not find procedure '" + procedure_name + "'");
  }
  if (common_procedure_it->second.isLoaded())
  {
    return; // Return if already loaded.
  }

  CommonProcedure& ftr = common_procedure_it->second;
  if (ftr.getExecutableType() == "ros")
  {
    rosExecute(ftr.getPackageName(), ftr.getExecutable(), ftr.getArgs());
    ftr.setLoaded(true);
    TEMOTO_DEBUG_("Feature 'Common Procedure' loaded.");
  }
  else
  {
    programExecute(ftr.getExecutable(), ftr.getArgs());
    ftr.setLoaded(true);
    TEMOTO_DEBUG_("Feature 'Common Procedure' loaded.");
  }
}
catch(resource_registrar::TemotoErrorStack& error_stack)
{
  throw FWD_TEMOTO_ERRSTACK(error_stack);
}

temoto_process_manager::LoadProcess Robot::rosExecute(const std::string& package_name
, const std::string& executable
, const std::string& args)
try
{
  temoto_process_manager::LoadProcess load_proc_srvc;
  load_proc_srvc.request.package_name = package_name;
  load_proc_srvc.request.ros_namespace = config_->getAbsRobotNamespace(); //Execute in robot namespace
  load_proc_srvc.request.action = temoto_process_manager::action::ROS_EXECUTE;
  load_proc_srvc.request.executable = executable;
  load_proc_srvc.request.args = args;

  resource_registrar_.call<temoto_process_manager::LoadProcess>(temoto_process_manager::srv_name::MANAGER
  , temoto_process_manager::srv_name::SERVER
  , load_proc_srvc
  , std::bind(&Robot::resourceStatusCb, this, std::placeholders::_1, std::placeholders::_2));

  return load_proc_srvc;
}
catch(resource_registrar::TemotoErrorStack& error_stack)
{
  throw FWD_TEMOTO_ERRSTACK(error_stack);
}

temoto_process_manager::LoadProcess Robot::programExecute(const std::string& program_name
, const std::string& args)
try
{
  temoto_process_manager::LoadProcess load_proc_srvc;
  load_proc_srvc.request.action = temoto_process_manager::action::SYS_EXECUTE;
  load_proc_srvc.request.executable = program_name;
  load_proc_srvc.request.args = args;

  resource_registrar_.call<temoto_process_manager::LoadProcess>(temoto_process_manager::srv_name::MANAGER
  , temoto_process_manager::srv_name::SERVER
  , load_proc_srvc
  , std::bind(&Robot::resourceStatusCb, this, std::placeholders::_1, std::placeholders::_2));

  return load_proc_srvc;
}
catch(resource_registrar::TemotoErrorStack& error_stack)
{
  throw FWD_TEMOTO_ERRSTACK(error_stack);
}

void Robot::resourceStatusCb(temoto_process_manager::LoadProcess srv_msg
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
    resource_registrar_.unload(temoto_process_manager::srv_name::MANAGER
    , srv_msg.response.temoto_metadata.request_id);

    auto load_er_query = rosExecute(srv_msg.request.package_name
    , srv_msg.request.executable
    , srv_msg.request.args);

    resource_registrar_.registerDependency(temoto_process_manager::srv_name::MANAGER
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
      std::string cmd_vel_topic = "/" + config_->getAbsRobotNamespace() + "/" + ftr.getCmdVelTopic();
      waitForTopic(cmd_vel_topic);
    }
    else if (ftr.getDriverPackageName() == srv_msg.request.package_name &&
             ftr.getDriverExecutable()  == srv_msg.request.executable)
    {
      TEMOTO_WARN_STREAM_("The driver of " << config_->getName() << " crashed, restarting it ...");
      // wait for command velocity to be published
      std::string odom_topic = "/" + config_->getAbsRobotNamespace() + "/" + ftr.getOdomTopic();
      waitForTopic(odom_topic);
      ftr.setDriverLoaded(true);
    }

     // Send the initial pose
    ros::Publisher pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/" + config_->getAbsRobotNamespace() + "/initialpose", 10);
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
  std::string rob_desc = "/" + config_->getAbsRobotNamespace() + "/robot_description";
  ros::NodeHandle mg_nh("/" + config_->getAbsRobotNamespace());
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
  TEMOTO_DEBUG_("Active end effector link: %s", group->getEndEffectorLink().c_str());

  planning_groups_.emplace(planning_group_name, std::move(group));
}

void Robot::removePlanningGroup(const std::string& planning_group_name)
{
  planning_groups_.erase(planning_group_name);
}

void Robot::planManipulationPath(const std::string& planning_group_name, const geometry_msgs::PoseStamped& target_pose)
{
  if (!planning_groups_.size())
  {
    throw TEMOTO_ERRSTACK("Robot has no planning groups.");
  }

  FeatureManipulation& ftr = config_->getFeatureManipulation();

  std::string planning_group = (planning_group_name.empty()) ? ftr.getActivePlanningGroup() : planning_group_name;
  auto group_it = planning_groups_.find(planning_group);
  if (group_it == planning_groups_.end())
  {
    throw TEMOTO_ERRSTACK("Planning group '" + planning_group + "' was not found.");
  }
  ftr.setActivePlanningGroup(planning_group);
  group_it->second->setStartStateToCurrentState();
  group_it->second->setPoseTarget(target_pose);
  
  is_plan_valid_ = static_cast<bool>(group_it->second->plan(last_plan));
  
  TEMOTO_DEBUG_("Plan %s",  is_plan_valid_ ? "FOUND" : "FAILED");
  if(!is_plan_valid_)
  {
    throw TEMOTO_ERRSTACK("Planning with group '" + group_it->first + "' failed.");
  }
}

void Robot::planManipulationPath(const std::string& planning_group_name, const std::vector<double> &joint_state_target)
{
  if (!planning_groups_.size())
  {
    throw TEMOTO_ERRSTACK("Robot has no planning groups.");
  }

  FeatureManipulation& ftr = config_->getFeatureManipulation();

  std::string planning_group = (planning_group_name.empty()) ? ftr.getActivePlanningGroup() : planning_group_name;
  auto group_it = planning_groups_.find(planning_group);
  if (group_it == planning_groups_.end())
  {
    throw TEMOTO_ERRSTACK("Planning group '" + planning_group + "' was not found.");
  }
  ftr.setActivePlanningGroup(planning_group);
  group_it->second->setStartStateToCurrentState();
  group_it->second->setJointValueTarget(joint_state_target);

  is_plan_valid_ = static_cast<bool>(group_it->second->plan(last_plan));

  TEMOTO_DEBUG_("Plan %s",  is_plan_valid_ ? "FOUND" : "FAILED");
  if(!is_plan_valid_)
  {
    throw TEMOTO_ERRSTACK("Planning with group '" + group_it->first +"' failed.");
  }
}

void Robot::planManipulationPath(const std::string& planning_group_name, const std::string& named_target)
{
  if (!planning_groups_.size())
  {
    throw TEMOTO_ERRSTACK("Robot has no planning groups.");
  }

  FeatureManipulation& ftr = config_->getFeatureManipulation();

  std::string planning_group = (planning_group_name.empty()) ? ftr.getActivePlanningGroup() : planning_group_name;
  auto group_it = planning_groups_.find(planning_group);

  if (group_it == planning_groups_.end())
  {
    throw TEMOTO_ERRSTACK("Planning group '" + planning_group_name + "' was not found.");
  }

  ftr.setActivePlanningGroup(planning_group);
  group_it->second->setStartStateToCurrentState();

  if (!group_it->second->setNamedTarget(named_target))
  {
    is_plan_valid_ = false;
    throw TEMOTO_ERRSTACK("Planning to named target pose '" + named_target + "' failed.");
  }
  
  is_plan_valid_ = static_cast<bool>(group_it->second->plan(last_plan));
  
  TEMOTO_DEBUG_("Plan %s",  is_plan_valid_ ? "FOUND" : "FAILED");
  if(!is_plan_valid_)
  {
    throw TEMOTO_ERRSTACK("Planning with group '" + group_it->first + "' failed.");
  }
}

void Robot::executeManipulationPath()
{
  std::string planning_group_name = config_->getFeatureManipulation().getActivePlanningGroup();
  moveit::planning_interface::MoveGroupInterface::Plan empty_plan;

  if (!is_plan_valid_)
  {
    throw TEMOTO_ERRSTACK("Unable to execute group '" + planning_group_name + "' without a plan.");
  }

  auto group_it = planning_groups_.find(planning_group_name);  ///< Will throw if group does not exist

  if (group_it != planning_groups_.end())
  {
    bool success = false;
    group_it->second->setStartStateToCurrentState();

    // Sometimes the arm doesn't execute the trajectory even when the plan is valid, Most probably a moveit issue. 
    // Re send path - max 3 Attempts
    for (size_t i=0; !success && i<3; i++)
    {
      TEMOTO_DEBUG_STREAM_("Attempt = " << i+1);
      success = static_cast<bool>(group_it->second->execute(last_plan));
    }

    TEMOTO_DEBUG_("Execution %s",  success ? "SUCCESSFUL" : "FAILED");
    if(!success)
    {
      throw TEMOTO_ERRSTACK("Execute plan with group '" + planning_group_name + "' failed.");
    }
  }
  else
  {
    throw TEMOTO_ERRSTACK("Planning group '" + planning_group_name + "' was not found.");
  }
}

geometry_msgs::PoseStamped Robot::getManipulationTarget(const std::string& planning_group_name)
{
  auto group_it = planning_groups_.find(planning_group_name);
  geometry_msgs::PoseStamped current_pose;

  if (group_it == planning_groups_.end())
  {
    throw TEMOTO_ERRSTACK("Planning group '" + planning_group_name + "' was not found.");
  }

  return group_it->second->getCurrentPose();;  
}

std::vector<double> Robot::getCurrentJointValues(const std::string& planning_group_name)
{
  auto group_it = planning_groups_.find(planning_group_name);

  if (group_it == planning_groups_.end())
  {
    throw TEMOTO_ERRSTACK("Planning group '" + planning_group_name + "' was not found.");
  }

  return group_it->second->getCurrentJointValues();
}


std::vector<std::string> Robot::getNamedTargetPoses(const std::string& planning_group_name)
{
  if (planning_groups_.empty())
  {
    throw TEMOTO_ERRSTACK("Robot has no planning groups.");
  }

  FeatureManipulation& ftr = config_->getFeatureManipulation();

  std::string planning_group = (planning_group_name.empty()) ? ftr.getActivePlanningGroup() : planning_group_name;
  auto group_it = planning_groups_.find(planning_group);

  if (group_it == planning_groups_.end())
  {
    throw TEMOTO_ERRSTACK("Planning group '" + planning_group_name + "' was not found.");
  }

  return group_it->second->getNamedTargets();
}

void Robot::goalNavigation(const geometry_msgs::PoseStamped& target_pose)
{
  TEMOTO_INFO_("================= [robot.cpp 1026] goalNavigation ==================");
  if (!isRobotOperational())
  {
    throw TEMOTO_ERRSTACK("Could not navigate the robot because robot is not operational");
  }
  TEMOTO_INFO_("================= [robot.cpp 1031] getFeatureNavigation ==================");
  FeatureNavigation& ftr = config_->getFeatureNavigation();
  RmNavigationRequestWrap request;
  request.goal_pose.header.frame_id = target_pose.header.frame_id;
  request.goal_pose.pose.position.x = target_pose.pose.position.x;
  request.goal_pose.pose.position.y = target_pose.pose.position.y;
  request.goal_pose.pose.position.z = target_pose.pose.position.z;
  request.goal_pose.pose.orientation.x = target_pose.pose.orientation.x;
  request.goal_pose.pose.orientation.y = target_pose.pose.orientation.y;
  request.goal_pose.pose.orientation.z = target_pose.pose.orientation.z;
  request.goal_pose.pose.orientation.w = target_pose.pose.orientation.w;
  TEMOTO_INFO_("================= Before send Goal  ==================");
  navigation_feature_plugin_->sendGoal(request);

  // std::string act_rob_ns = "/" + config_->getAbsRobotNamespace() + "/move_base";
  // MoveBaseClient ac(act_rob_ns, true);
  
  // if (!ac.waitForServer(ros::Duration(5.0)))
  // {
  //   TEMOTO_ERRSTACK("The move_base action server did not come up");
  // }

  // move_base_msgs::MoveBaseGoal goal;
  // goal.target_pose = target_pose;
  // goal.target_pose.header.stamp = ros::Time::now();
  // ac.sendGoal(goal);

  // // Wait until either the goal is finished or robot has encountered a system issue
  // while((ac.getState() == actionlib::SimpleClientGoalState::PENDING || ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
  //    && isRobotOperational())
  // {
  //   ros::Duration(1).sleep();
  // }

  // if (!isRobotOperational())
  // {
  //   ac.cancelGoal();
  //   throw TEMOTO_ERRSTACK("Could not finish the navigation goal because the robot is not operational");
  // }
  // else if(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  // {
  //   throw TEMOTO_ERRSTACK("The base failed to move");
  // }


}

void Robot::controlGripper(const std::string& robot_name,const float position)
try
{
  FeatureGripper& ftr = config_->getFeatureGripper();   
  std::string argument = std::to_string(position);
  std::string gripper_topic = "/" + config_->getAbsRobotNamespace() + "/gripper_control";
  
  TEMOTO_DEBUG_("Feature 'Gripper' loaded.");
  client_gripper_control_ = nh_.serviceClient<temoto_robot_manager::GripperControl>(gripper_topic);
  temoto_robot_manager::GripperControl gripper_srvc;
  gripper_srvc.request.robot_name = robot_name;
  gripper_srvc.request.position = position;

  if (!client_gripper_control_.call(gripper_srvc))
  {
    throw TEMOTO_ERRSTACK("Call to remote RobotManager service failed.");
  }
  
  TEMOTO_DEBUG_("Call to gripper control was sucessful.");
}
catch(resource_registrar::TemotoErrorStack& error_stack)
{
  throw FWD_TEMOTO_ERRSTACK(error_stack);
}

bool Robot::isLocal() const
{
  if (!config_) 
  {
    throw TEMOTO_ERRSTACK("Robot in undefined configuration.");
  }
  return config_->getTemotoNamespace() == TEMOTO_LOG_ATTR.getNs(); 
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
   * 1) get the sub-resource queries (LoadProcess)
   * 2) Recover each robotic feature based on subresource, including assigning status callbacks per resource ID
   * TODO: this method needs data race protection via mutexes
   */

  auto erm_queries = resource_registrar_.getRosChildQueries<temoto_process_manager::LoadProcess>(parent_query_id
  , temoto_process_manager::srv_name::SERVER);

  TEMOTO_DEBUG_STREAM_("size of erm_queries: " << erm_queries.size());
  for (const auto& erm_query : erm_queries)
  {
    TEMOTO_DEBUG_STREAM_("ERM query: " << erm_query.second.request);
    resource_registrar_.registerClientCallback<temoto_process_manager::LoadProcess>(temoto_process_manager::srv_name::MANAGER
    , temoto_process_manager::srv_name::SERVER
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
      localized_pose_sub_ = nh_.subscribe("/" + config_->getAbsRobotNamespace() + "/" + config_->getFeatureNavigation().getPoseTopic()
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


