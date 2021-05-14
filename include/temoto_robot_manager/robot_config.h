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

#ifndef TEMOTO_ROBOT_MANAGER__TEMOTO_ROBOT_MANAGER__ROBOT_CONFIG_H
#define TEMOTO_ROBOT_MANAGER__TEMOTO_ROBOT_MANAGER__ROBOT_CONFIG_H

#include "temoto_core/common/temoto_log_macros.h"
#include "temoto_core/common/reliability.h"
#include "temoto_core/common/base_subsystem.h"
#include "temoto_robot_manager/robot_features.h"

#include "yaml-cpp/yaml.h"
#include <string>
#include <vector>
#include <map>
#include <ctype.h>
#include <memory>  // shared_ptr

namespace temoto_robot_manager
{

class RobotConfig : temoto_core::BaseSubsystem
{
public:
  /**
   * @brief RobotConfig
   */

  RobotConfig(YAML::Node yaml_config, temoto_core::BaseSubsystem& b);
  
  std::string toString() const;

  /* * * * * * * * * * * *
   *     GETTERS
   * * * * * * * * * * * */

  // Get the temoto namespace where this robot is defined
  std::string getTemotoNamespace() const
  {
    return temoto_namespace_;
  }

  // Get the robot's namespace
  std::string getAbsRobotNamespace() const
  {
    return "/" + temoto_namespace_ + "/robot_manager/robots/" + name_;
  }

  void parseName();
  void parseHardware();

  void parseTemotoNamespace();
  void parseDescription();
  void parseReliability();

  void parseUrdf();
  void parseManipulation();
  void parseNavigation();
  void parseGripper();

  std::string getName() const
  {
    return name_;
  }

  std::string getDescription() const
  {
    return description_;
  }

  float getReliability() const
  {
    return reliability_.getReliability();
  }

  FeatureURDF& getFeatureURDF()
  {
    return feature_urdf_;
  }

  FeatureManipulation& getFeatureManipulation()
  {
    return feature_manipulation_;
  }

  FeatureNavigation& getFeatureNavigation()
  {
    return feature_navigation_;
  }

  FeatureGripper& getFeatureGripper()
  {
    return feature_gripper_;
  }

  void adjustReliability(float reliability)
  {
    reliability_.adjustReliability(reliability);
  }

  void resetReliability(float reliability)
  {
    reliability_.resetReliability(reliability);
  }

  const YAML::Node& getYAMLConfig() const
  {
    return yaml_config_;
  }

  std::string getYamlConfigString() const
  {
    std::stringstream ss;
    ss << getYAMLConfig();
    return ss.str();
  }

  void setTemotoNamespace(std::string temoto_namespace)
  {
    temoto_namespace_ = temoto_namespace;
  }

private:

  std::string log_class_ = "RobotConfig";
  std::string log_subsys_ = "robot_manager";
  std::string log_group_ = "robot_manager";

  std::string temoto_namespace_;
  YAML::Node yaml_config_;

  FeatureURDF feature_urdf_;
  FeatureManipulation feature_manipulation_;
  FeatureNavigation feature_navigation_;
  FeatureGripper feature_gripper_;
  std::vector<RobotFeature*> enabled_features_;
  
  std::string name_;
  std::string description_;
  temoto_core::Reliability reliability_;
};

typedef std::shared_ptr<RobotConfig> RobotConfigPtr;
typedef std::vector<RobotConfigPtr> RobotConfigs;

static bool operator==(const RobotConfig& r1, const RobotConfig& r2)
{
  //compare if the two configs are created from the same YAML config string
  return r1.getYAMLConfig() == r2.getYAMLConfig();

//  return (r1.getTemotoNamespace() == r2.getTemotoNamespace() && r1.getName() == r2.getName() &&
//          r1.getDescription() == r2.getDescription() &&
//          r1.getPlanningGroups() == r2.getPlanningGroups() &&
//          r1.getRobotFeatures() == r2.getRobotFeatures());
}

} // robot_manager namespace

#endif

