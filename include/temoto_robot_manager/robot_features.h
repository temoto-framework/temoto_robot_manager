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

#ifndef TEMOTO_ROBOT_MANAGER__ROBOT_FEATURES_H
#define TEMOTO_ROBOT_MANAGER__ROBOT_FEATURES_H

#include "temoto_core/common/temoto_id.h"

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace temoto_robot_manager
{

  // Base class for all features
class RobotFeature
{
public:
  RobotFeature(const std::string& name = "unknown_feature");

  std::string getName() const
  {
    return name_;
  }

  std::string getPackageName() const
  {
    return package_name_;
  }

  std::string getExecutable() const
  {
    return executable_;
  }

  std::string getExecutableType() const
  {
    return executable_type_;
  }

  std::string getArgs() const
  {
    return args_;
  }

  temoto_core::temoto_id::ID getResourceId() const
  {
    return resource_id_;
  }
  
  bool isLoaded() const 
  {
    return feature_loaded_;
  }

  void setLoaded(bool loaded)
  {
    feature_loaded_ = loaded;
  }

  bool isEnabled() const 
  {
    return feature_enabled_;
  }

  void setEnabled(bool enabled)
  {
    feature_enabled_ = enabled;
  }

  void setResourceId(temoto_core::temoto_id::ID id)
  {
    resource_id_ = id;
  }

  bool setFromConfig(const YAML::Node& config, std::string& parameter)
  {
    if (config.IsDefined())
    {
      parameter = config.as<std::string>();
      return true;
    }
    return false;
  }

protected:
  std::string name_;
  std::string package_name_;
  std::string executable_;
  std::string executable_type_;
  std::string args_;
  temoto_core::temoto_id::ID resource_id_;
  bool feature_enabled_;
  bool feature_loaded_;
};

  // Base class for features which contain a robot driver
class FeatureWithDriver : public RobotFeature
{
public:
  FeatureWithDriver(const std::string& name = "unknown_feature_with_driver");

  bool isDriverLoaded() const
  {
    return driver_loaded_;
  }

  void setDriverLoaded(bool loaded)
  {
    driver_loaded_ = loaded;
  }

  bool isDriverEnabled() const
  {
    return driver_enabled_;
  }

  void setDriverEnabled(bool enabled)
  {
    driver_enabled_ = enabled;
  }

  temoto_core::temoto_id::ID getDriverResourceId() const
  {
    return driver_resource_id_;
  }

  void setDriverResourceId(temoto_core::temoto_id::ID id)
  {
    driver_resource_id_ = id;
  }

  std::string getDriverPackageName() const
  {
    return driver_package_name_;
  }

  std::string getDriverExecutable() const
  {
    return driver_executable_;
  }

  std::string getDriverArgs() const
  {
    return driver_args_;
  }

protected:
  bool driver_loaded_;
  bool driver_enabled_;
  std::string driver_package_name_;
  std::string driver_executable_;
  std::string driver_executable_type_;
  std::string driver_args_;
  temoto_core::temoto_id::ID driver_resource_id_;
};

// URDF feature
class FeatureURDF : public RobotFeature
{
  public:
  FeatureURDF();
  FeatureURDF(const YAML::Node& urdf_conf);
  void setArgs(const std::string& args);
};


// Manipulation feature
class FeatureManipulation : public FeatureWithDriver
{
public:
  FeatureManipulation();
  FeatureManipulation(const YAML::Node& manip_conf);

  std::vector<std::string> getPlanningGroups() const
  {
    return planning_groups_;
  }

  std::string getActivePlanningGroup() const
  {
    return active_planning_group_;
  }

  void setActivePlanningGroup(std::string planning_group_name)
  {
    //TODO: check if group exists
    active_planning_group_ = planning_group_name;
  }
  
private:
  std::vector<std::string> planning_groups_;
  std::string active_planning_group_;
};


// Navigation feature
class FeatureNavigation : public FeatureWithDriver
{
public:
  FeatureNavigation();
  FeatureNavigation(const YAML::Node& nav_conf);

  const std::string& getGlobalPlanner() const
  {
    return global_planner_;
  }

  const std::string& getLocalPlanner() const
  {
    return local_planner_;
  }

  const std::string& getOdomTopic() const
  {
    return odom_topic_;
  }

  const std::string& getCmdVelTopic() const
  {
    return cmd_vel_topic_;
  }

  const std::string& getPoseTopic() const
  {
    return pose_topic_;
  }

private:
  std::string global_planner_;
  std::string local_planner_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string pose_topic_;
};

class FeatureGripper : public FeatureWithDriver
{
public:
  FeatureGripper();
  FeatureGripper(const YAML::Node& gripp_conf);
  
};

class FeatureCustom : public FeatureWithDriver
{
public:
  FeatureCustom(const std::string& name, const YAML::Node& yaml_node);
};

// Common Procedures
class CommonProcedure : public RobotFeature
{
public:
  CommonProcedure(const std::string& name, const YAML::Node& common_conf);
};

}

#endif

