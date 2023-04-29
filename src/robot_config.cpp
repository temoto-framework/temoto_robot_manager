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

#include "temoto_robot_manager/robot_config.h"
#include "temoto_resource_registrar/temoto_error.h"
#include "temoto_core/common/tools.h"
#include <boost/algorithm/string/replace.hpp>
#include <string>
#include <vector>

namespace temoto_robot_manager
{
RobotConfig::RobotConfig(YAML::Node yaml_config, temoto_core::BaseSubsystem& b)
: yaml_config_(yaml_config)
, temoto_core::BaseSubsystem(b)
{
  class_name_ = "RobotConfig";

  // Parse mandatory information.
  try
  {
    parseName();
  }
  catch (...)
  {
    throw TEMOTO_ERRSTACK("Unable to parse robot name.");
  }

  // Parse additional information
  parseTemotoNamespace();
  parseDescription();
  parseReliability();

  // Parse robot features
  parseFeatures();
  // parseUrdf();
  // parseNavigation();
  // parseManipulation();
  // parseGripper();
}

void RobotConfig::parseFeatures()
try
{

  if (!yaml_config_["features"].IsDefined())
  {
    throw TEMOTO_ERRSTACK("The robot has no features described");
  }

  YAML::Node features_node = yaml_config_["features"];

  if (!features_node.IsSequence())
  {
    throw TEMOTO_ERRSTACK("The given config does not contain sequence of features.");
  }

  TEMOTO_DEBUG_("Parsing %lu features.", features_node.size());

  // Go over each feature node in the sequence
  for (YAML::const_iterator node_it = features_node.begin(); node_it != features_node.end(); ++node_it)
  {
    TEMOTO_WARN_STREAM_("Feature: " << (*node_it)["name"] << ", " << (*node_it)["type"]);
  }
}
catch (YAML::InvalidNode e)
{
  throw TEMOTO_ERRSTACK("Unable to parse features: " + std::string(e.what()));
}

void RobotConfig::parseName()
try
{
  name_ = yaml_config_["robot_name"].as<std::string>();
}
catch (YAML::InvalidNode e)
{
  name_ = "unnamed_robot";
  throw TEMOTO_ERRSTACK("CONFIG: robot_name NOT FOUND");
}

void RobotConfig::parseTemotoNamespace()
try
{
  if (yaml_config_["temoto_namespace"].IsDefined())
  {
    setTemotoNamespace(yaml_config_["temoto_namespace"].as<std::string>());
  }
  else
  {
    setTemotoNamespace(temoto_core::common::getTemotoNamespace());
  }
}
catch (...)
{
  TEMOTO_WARN("CONFIG: temoto_namespace is ill formated");
}

void RobotConfig::parseDescription()
try
{
  if (yaml_config_["description"].IsDefined())
  {
    description_ = yaml_config_["description"].as<std::string>();
  }
  else
  {
    TEMOTO_WARN("CONFIG: description NOT FOUND");
  }
}
catch (...)
{
  TEMOTO_WARN("CONFIG: description is ill formated");
}

void RobotConfig::parseReliability()
try
{
  if (yaml_config_["reliability"].IsDefined())
  {
    resetReliability(yaml_config_["reliability"].as<float>());
  }
}
catch (...)
{
  TEMOTO_WARN("CONFIG: reliability is ill formated");
}

void RobotConfig::parseUrdf()
try
{
  if (!yaml_config_["urdf"].IsDefined())
  {
    return;
  }

  feature_urdf_ = FeatureURDF(yaml_config_["urdf"]);
  if (!feature_urdf_.getArgs().empty())
  {
    std::string processed_args = feature_urdf_.getArgs();
    boost::replace_all(processed_args, "__ABS_NAMESPACE__", getAbsRobotNamespace());
    
    feature_urdf_.setArgs(processed_args);
    yaml_config_["urdf"]["args"] = processed_args;
  }
  enabled_features_.push_back(&feature_urdf_);
}
catch (...)
{
  throw TEMOTO_ERRSTACK("CONFIG: urdf:{package_name or executable} NOT FOUND");
}

void RobotConfig::parseManipulation()
try
{
  if (!yaml_config_["manipulation"].IsDefined())
  {
    return;
  }

  feature_manipulation_ = FeatureManipulation(yaml_config_["manipulation"]);
  enabled_features_.push_back(&feature_manipulation_);
}
catch (YAML::Exception& e)
{
  TEMOTO_WARN("CONFIG: error parsing manipulation: %s", e.what());
}

void RobotConfig::parseNavigation()
try
{
  if (!yaml_config_["navigation"].IsDefined())
  {
    return;
  }

  feature_navigation_ = FeatureNavigation(yaml_config_["navigation"]);
  enabled_features_.push_back(&feature_navigation_);
}
catch (YAML::Exception e)
{
  throw TEMOTO_ERRSTACK("CONFIG: error parsing navigation: " + std::string(e.what()));
}

void RobotConfig::parseGripper()
try
{
  if (!yaml_config_["gripper"].IsDefined())
  {
    return;
  }
  
  feature_gripper_ = FeatureGripper(yaml_config_["gripper"]);
  enabled_features_.push_back(&feature_gripper_);
}
catch (YAML::Exception& e)
{
  TEMOTO_WARN("CONFIG: error parsing gripper: %s", e.what());
}

std::string RobotConfig::toString() const
{
  std::string ret;
  ret += "ROBOT: " + getName() + "\n";
  ret += "  description : " + getDescription() + "\n";
  ret += "  reliability : " + std::to_string(getReliability()) + "\n";
  ret += "  features: \n";
  ret += feature_urdf_.isEnabled() ? "    urdf\n" : "";
  ret += feature_manipulation_.isEnabled() ? "    manipulation\n" : "";
  ret += feature_navigation_.isEnabled() ? "    navigation\n" : "";
  ret += feature_gripper_.isEnabled() ? "    gripper\n" : "";
  return ret;
}

}  // RobotManager namespace

