#ifndef TEMOTO_ROBOT_MANAGER__RM_DATASTRUCTURES_H
#define TEMOTO_ROBOT_MANAGER__RM_DATASTRUCTURES_H

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

} // temoto_robot_manager

#endif