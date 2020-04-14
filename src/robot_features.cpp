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

#include "temoto_robot_manager/robot_features.h"
#include <iostream>

// ALL OF THESE CLASSES MAY THROW YAML EXCEPTIONS

namespace robot_manager
{
RobotFeature::RobotFeature(const std::string& name) : name_(name), feature_loaded_(false), feature_enabled_(false)
{
}

FeatureWithDriver::FeatureWithDriver(const std::string& name)
  : RobotFeature(name), driver_loaded_(false), driver_enabled_(false)
{
}

FeatureURDF::FeatureURDF() : RobotFeature("urdf")
{
}

FeatureURDF::FeatureURDF(const YAML::Node& urdf_conf) : RobotFeature("urdf")
{
  this->package_name_ = urdf_conf["package_name"].as<std::string>();
  this->executable_ = urdf_conf["executable"].as<std::string>();
  this->feature_enabled_ = true;
}

FeatureManipulation::FeatureManipulation() : FeatureWithDriver("manipulation")
{
}

FeatureManipulation::FeatureManipulation(const YAML::Node& manip_conf)
  : FeatureWithDriver("manipulation")
{
  this->package_name_ = manip_conf["controller"]["package_name"].as<std::string>();
  if (manip_conf["controller"]["executable"])
  {
    this->executable_ = manip_conf["controller"]["executable"].as<std::string>();
  }
  else
  {
    this->executable_ = "move_group.launch";
  }

  if (manip_conf["controller"]["args"])
  {
    this->args_ = manip_conf["controller"]["args"].as<std::string>();
  }

  // parse planning groups
  YAML::Node yaml_groups = manip_conf["controller"]["planning_groups"];
  for (YAML::const_iterator it = yaml_groups.begin(); it != yaml_groups.end(); ++it)
  {
    planning_groups_.emplace_back(it->as<std::string>());
  }
  if(planning_groups_.size())
  {
    active_planning_group_ = planning_groups_.front();
  }
  this->feature_enabled_ = true;

  this->driver_package_name_ = manip_conf["driver"]["package_name"].as<std::string>();
  this->driver_executable_ = manip_conf["driver"]["executable"].as<std::string>();
  if (manip_conf["driver"]["args"])
  {
    this->driver_args_ = manip_conf["driver"]["args"].as<std::string>();
  }
  this->driver_enabled_ = true;
}

FeatureNavigation::FeatureNavigation() : FeatureWithDriver("navigation")
{
}

FeatureNavigation::FeatureNavigation(const YAML::Node& nav_conf) : FeatureWithDriver("navigation")
{
  this->package_name_ = nav_conf["controller"]["package_name"].as<std::string>();
  this->executable_ = nav_conf["controller"]["executable"].as<std::string>();

  if (nav_conf["controller"]["args"])
  {
    this->args_ = nav_conf["controller"]["args"].as<std::string>();
  }
  this->global_planner_ = nav_conf["controller"]["global_planner"].as<std::string>();
  this->local_planner_ = nav_conf["controller"]["local_planner"].as<std::string>();
  this->feature_enabled_ = true;

  this->driver_package_name_ = nav_conf["driver"]["package_name"].as<std::string>();
  this->driver_executable_ = nav_conf["driver"]["executable"].as<std::string>();
  if (nav_conf["driver"]["args"])
  {
    this->driver_args_ = nav_conf["driver"]["args"].as<std::string>();
  }
  this->driver_enabled_ = true;
}

FeatureGripper::FeatureGripper() : FeatureWithDriver("gripper")
{
}

FeatureGripper::FeatureGripper(const YAML::Node& grip_conf)
  : FeatureWithDriver("gripper")
{
  this->package_name_ = grip_conf["controller"]["package_name"].as<std::string>();
  if (grip_conf["controller"]["executable"])
  {
    this->executable_ = grip_conf["controller"]["executable"].as<std::string>();
  }
  else
  {
    this->executable_ = "temoto_gripper_converter.launch";      // For now a defaul value.. 
                                                                // TODO: change to the right one
  }
  if (grip_conf["controller"]["args"])
  {
    this->args_ = grip_conf["controller"]["args"].as<std::string>();
  }
  this->feature_enabled_ = true;
  this->driver_package_name_ = grip_conf["driver"]["package_name"].as<std::string>();
  this->driver_executable_ = grip_conf["driver"]["executable"].as<std::string>();
  if (grip_conf["driver"]["args"])
  {
    this->driver_args_ = grip_conf["driver"]["args"].as<std::string>();
  }
  this->driver_enabled_ = true;
}

// bool operator==(const RobotFeature& rf1, const RobotFeature& rf2)
//{
//  return (rf1.getType() == rf2.getType() && rf1.getPackageName() == rf2.getPackageName() &&
//          rf1.getExecutable() == rf2.getExecutable() && rf1.getArgs() == rf2.getArgs());
//}
}

