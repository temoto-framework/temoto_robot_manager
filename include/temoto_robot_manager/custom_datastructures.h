#ifndef TEMOTO_ROBOT_MANAGER__CUSTOM_DATASTRUCTURES_H
#define TEMOTO_ROBOT_MANAGER__CUSTOM_DATASTRUCTURES_H

#include <string>
#include <vector>

namespace temoto_robot_manager
{

struct RmCustomRequest
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

  struct PoseStamped
  {
    Header header;
    Position position;
    Orientation orientation;
  };

  std::string data_str;
  double data_num;
  PoseStamped data_pose;

  std::vector<std::string> data_str_array;
  std::vector<double> data_num_array;
  std::vector<PoseStamped> data_pose_array;
};

struct RmCustomFeedback
{
  uint8_t status;
  double progress;
};

} // temoto_robot_manager

#endif