#ifndef TEMOTO_ROBOT_MANAGER__CUSTOM_PLUGIN_BASE_H
#define TEMOTO_ROBOT_MANAGER__CUSTOM_PLUGIN_BASE_H

#include "temoto_robot_manager/custom_datastructures.h"
#include <optional>

namespace temoto_robot_manager
{

class CustomPluginBase
{
public:
  virtual bool initialize() = 0;
  virtual bool invoke(const RmCustomRequest& goal) = 0;
  virtual std::optional<RmCustomFeedback> getFeedback() = 0;
  virtual bool preempt() = 0;
  virtual bool deinitialize() = 0;
  virtual ~CustomPluginBase(){};
};

} // temoto_robot_manager

#endif