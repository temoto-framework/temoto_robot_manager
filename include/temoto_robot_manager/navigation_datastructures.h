#ifndef TEMOTO_ROBOT_MANAGER__NAVIGATION_DATASTRUCTURES_H
#define TEMOTO_ROBOT_MANAGER__NAVIGATION_DATASTRUCTURES_H

#include "temoto_robot_manager/rm_datastructures.h"

namespace temoto_robot_manager
{

struct RmNavigationGoal
{
  PoseStamped goal_pose;
};

struct RmNavigationFeedback
{
  uint8_t status;
  double progress;
  PoseStamped base_position;
};

} // temoto_robot_manager

#endif