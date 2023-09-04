#ifndef TEMOTO_ROBOT_MANAGER__CUSTOM_DATASTRUCTURES_H
#define TEMOTO_ROBOT_MANAGER__CUSTOM_DATASTRUCTURES_H

#include <string>
#include <vector>

namespace temoto_robot_manager
{

struct Header
{
  std::string frame_id;
  uint64_t timestamp;
  uint64_t sequence_id;
};

struct Position
{
  double x;
  double y;
  double z;
};

struct Orientation
{
  double x;
  double y;
  double z;
  double w;
};

struct Pose
{
  Position position;
  Orientation orientation;
}; 

struct PoseStamped
{
  Header header;
  Pose pose;
};

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

struct RmNavigationGoal
{
  PoseStamped goal_pose;
};

struct RmNavigationFeedback
{
  uint8_t status;
  PoseStamped base_position;
};

} // temoto_robot_manager

#endif