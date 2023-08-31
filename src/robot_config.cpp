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
RobotConfig::RobotConfig(YAML::Node yaml_config)
: yaml_config_(yaml_config)
{
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
    std::string feature_name = (*node_it)["name"].as<std::string>();
    std::string feature_type = (*node_it)["type"].as<std::string>();
    TEMOTO_DEBUG_STREAM_("Feature: " << feature_name << ", Type: " << feature_type);
    
    if (feature_type == "urdf")
    {
      parseUrdf(*node_it);
    }
    else if (feature_type == "navigation")
    {
      parseNavigation(*node_it);
    }
    else if (feature_type == "manipulation")
    {
      parseManipulation(*node_it);
    }
    else if (feature_type == "gripper")
    {
      parseGripper(*node_it);
    }
    else if (feature_type == "custom")
    {
      parseCustom(*node_it);
    }
    else if (feature_type == "common")
    {
      parseCommon(*node_it);
    }
    else
    {
      throw TEMOTO_ERRSTACK("Unrecognized feature type '" + feature_type + "'");
    }
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
  TEMOTO_WARN_("CONFIG: temoto_namespace is ill formated");
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
    TEMOTO_WARN_("CONFIG: description NOT FOUND");
  }
}
catch (...)
{
  TEMOTO_WARN_("CONFIG: description is ill formated");
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
  TEMOTO_WARN_("CONFIG: reliability is ill formated");
}

void RobotConfig::parseUrdf(const YAML::Node& yaml_node)
try
{
  feature_urdf_ = FeatureURDF(yaml_node);
  if (!feature_urdf_.getArgs().empty())
  {
    std::string processed_args = feature_urdf_.getArgs();
    boost::replace_all(processed_args, "__ABS_NAMESPACE__", getAbsRobotNamespace());
    
    feature_urdf_.setArgs(processed_args);
    yaml_config_["urdf"]["args"] = processed_args;
  }
}
catch (...)
{
  throw TEMOTO_ERRSTACK("CONFIG: urdf:{package_name or executable} NOT FOUND");
}

void RobotConfig::parseManipulation(const YAML::Node& yaml_node)
try
{
  feature_manipulation_ = FeatureManipulation(yaml_node);
}
catch (YAML::Exception& e)
{
  TEMOTO_WARN_("CONFIG: error parsing manipulation: %s", e.what());
}

void RobotConfig::parseNavigation(const YAML::Node& yaml_node)
try
{
  feature_navigation_ = FeatureNavigation(yaml_node);
}
catch (YAML::Exception e)
{
  throw TEMOTO_ERRSTACK("CONFIG: error parsing navigation: " + std::string(e.what()));
}

void RobotConfig::parseGripper(const YAML::Node& yaml_node)
try
{
  feature_gripper_ = FeatureGripper(yaml_node);
}
catch (YAML::Exception& e)
{
  TEMOTO_WARN_("CONFIG: error parsing gripper: %s", e.what());
}

void RobotConfig::parseCustom(const YAML::Node& yaml_node)
try
{
  std::string feature_name = yaml_node["name"].as<std::string>(); 
  m_feature_custom_.insert({feature_name, FeatureCustom(feature_name, yaml_node)});
}
catch (YAML::Exception& e)
{
  TEMOTO_WARN_("CONFIG: error parsing custom feature: %s", e.what());
}

void RobotConfig::parseCommon(const YAML::Node& yaml_node)
try
{
  std::string procedure_name = yaml_node["name"].as<std::string>();
  m_common_procedures_.insert({procedure_name, CommonProcedure(procedure_name, yaml_node)});
}
catch (YAML::Exception& e)
{
  TEMOTO_WARN_("CONFIG: error parsing common feature: %s", e.what());
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
  for (const auto& custom_feature : m_feature_custom_)
  {
    ret += custom_feature.second.isEnabled() ? std::string("    custom: " + custom_feature.second.getName() + "\n") : "";
  }
  for (const auto& common_procedure : m_common_procedures_)
  {
    ret += common_procedure.second.isDefined() ? std::string("    common: " + common_procedure.second.getName() + "\n") : "";
  }
  return ret;
}

}  // RobotManager namespace

