#ifndef TEMOTO_ROBOT_MANAGER__NAVIGATION_PLUGIN_BASE_H
#define TEMOTO_ROBOT_MANAGER__NAVIGATION_PLUGIN_BASE_H

#include "temoto_robot_manager/navigation_datastructures.h"
#include <optional>

namespace temoto_robot_manager
{

class NavigationPluginBase
{
public:
  virtual bool initialize(const std::string& robot_ns) = 0;
  virtual bool sendGoal(RmNavigationGoal goal) = 0;
  virtual std::optional<RmNavigationFeedback> getFeedback() = 0;
  virtual bool cancelGoal() = 0;
  virtual bool deinitialize() = 0;
  virtual ~NavigationPluginBase(){};
};

} // temoto_robot_manager

#endif