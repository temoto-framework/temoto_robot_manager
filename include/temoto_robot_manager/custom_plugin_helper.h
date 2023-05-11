#ifndef TEMOTO_ROBOT_MANAGER__CUSTOM_PLUGIN_HELPER_H
#define TEMOTO_ROBOT_MANAGER__CUSTOM_PLUGIN_HELPER_H

#include "class_loader/class_loader.hpp"
#include "temoto_robot_manager/custom_plugin_base.h"
#include <memory>
#include <thread>

namespace temoto_robot_manager
{

struct RmCustomFeedbackWrap : RmCustomFeedback
{
  std::string robot_name;
  std::string custom_feature_name;
  std::string request_id;
};

typedef std::shared_ptr<CustomPluginHelper> CustomPluginHelperPtr;
typedef std::function<void(const RmCustomFeedbackWrap&)> CustomFeatureUpdateCb;

class CustomPluginHelper
{
public:
  enum class State
  {
    NOT_LOADED,
    UNINITIALIZED,
    INITIALIZED,
    PROCESSING,
    STOPPING,
    ERROR
  };

  CustomPluginHelper(const std::string& plugin_path, CustomFeatureUpdateCb update_cb);
  ~CustomPluginHelper();
  void initialize();
  void invoke(const RmCustomRequest& request);
  void preempt();
  void deinitialize();

private:
  State getState() const;
  void setState(State state);

  std::shared_ptr<CustomPluginBase> plugin;
  std::shared_ptr<class_loader::ClassLoader> class_loader;
  std::thread exec_thread_;
  std::string plugin_path_;

  State state_;
  mutable std::mutex mutex_state_;

  CustomFeatureUpdateCb update_cb_;
};
} // temoto_robot_manager namespace
#endif