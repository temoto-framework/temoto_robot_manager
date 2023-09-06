#ifndef TEMOTO_ROBOT_MANAGER__CUSTOM_DATASTRUCTURES_H
#define TEMOTO_ROBOT_MANAGER__CUSTOM_DATASTRUCTURES_H

#include "temoto_robot_manager/rm_datastructures.h"

namespace temoto_robot_manager
{

struct RmCustomRequest
{
  Header header;
  Position position;  
  Orientation orientation;
  Pose pose;
  PoseStamped poseStamped;
  std::string data_str;
  std::vector<std::string> data_str_array;
  double data_num;
  std::vector<double> data_num_array;
  PoseStamped data_pose;
  std::vector<PoseStamped> data_pose_array;
};

struct RmCustomFeedback
{
  uint8_t status;
  double progress;
};

} // temoto_robot_manager

#endif