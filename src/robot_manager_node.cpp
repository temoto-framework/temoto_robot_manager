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

#include "temoto_robot_manager/robot_manager.h"

using namespace robot_manager;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_manager");

  // Create a SensorManager object
  RobotManager rm;

  // set up some robots
//  rm.pkg_infos_.emplace_back(std::make_shared<PackageInfo>("task_robot", "ur5"));
//  rm.pkg_infos_.back()->addLaunchable({ "ur5.launch", "" });
//
//  rm.pkg_infos_.emplace_back(std::make_shared<PackageInfo>("task_robot", "ur3"));
//  rm.pkg_infos_.back()->addLaunchable({ "ur3.launch", "" });

  //use single threaded spinner for global callback queue
   ros::AsyncSpinner spinner(4);
   spinner.start();
   ros::waitForShutdown();

  return 0;
}

