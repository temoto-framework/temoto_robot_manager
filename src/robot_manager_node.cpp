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

#include "temoto_robot_manager/robot_manager.h"
#include <boost/program_options.hpp>
#include "temoto_resource_registrar/temoto_logging.h"

using namespace temoto_robot_manager;

int main(int argc, char** argv)
{
  namespace po = boost::program_options;
  po::variables_map vm;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("config-base-path", po::value<std::string>(), "Base path to robot_description.yaml config file.")
    ("restore-from-catalog", po::value<bool>(), "Restore the state of the manager via RR catalog.");

  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  // Get the config path
  std::string config_base_path;
  if (vm.count("config-base-path"))
  {
    config_base_path = vm["config-base-path"].as<std::string>();
  }
  else
  {
    std::cout << "Missing robot_description.yaml base path\n" << desc;
    return 1;
  }

  bool restore_from_catalog{false};
  if (vm.count("restore-from-catalog"))
  {
    restore_from_catalog = vm["restore-from-catalog"].as<bool>();
  }

  TEMOTO_LOG_ATTR.initialize("robot_manager");
  ros::init(argc, argv, TEMOTO_LOG_ATTR.getSubsystemName());
  RobotManager rm(config_base_path, restore_from_catalog);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
}

