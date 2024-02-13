#ifndef TEMOTO_ROBOT_MANAGER__NAVIGATION_PLUGIN_HELPER_H
#define TEMOTO_ROBOT_MANAGER__NAVIGATION_PLUGIN_HELPER_H

#include "class_loader/class_loader.hpp"
#include "temoto_robot_manager/navigation_plugin_base.h"
#include <memory>
#include <thread>

namespace temoto_robot_manager
{

struct RmNavigationFeedbackWrap : RmNavigationFeedback
{
  std::string robot_name;
  std::string request_id;
};

struct RmNavigationRequestWrap : RmNavigationGoal
{
  std::string robot_name;
  std::string request_id;
};

class NavigationPluginHelper;

typedef std::shared_ptr<NavigationPluginHelper> NavigationPluginHelperPtr;
typedef std::function<void(const RmNavigationFeedbackWrap&)> NavigationFeatureUpdateCb;

class NavigationPluginHelper
{
public:
  enum class State
  {
    NOT_LOADED,
    UNINITIALIZED,
    INITIALIZED,
    PROCESSING,
    FINISHED,
    STOPPING,
    ERROR
  };

  NavigationPluginHelper(const std::string& plugin_path, const std::string& robot_ns, NavigationFeatureUpdateCb update_cb);
  ~NavigationPluginHelper();
  void initialize();
  void sendGoal(const RmNavigationRequestWrap& request);
  void sendUpdate() const;
  void cancelGoal();
  void deinitialize();
  State getState() const;

private:
  void setState(State state);

  std::shared_ptr<NavigationPluginBase> plugin;
  std::shared_ptr<class_loader::ClassLoader> class_loader;
  std::thread exec_thread_;
  std::string plugin_path_;
  std::string robot_ns_;
  bool is_thread_running_ = false;

  State state_;
  mutable std::mutex mutex_state_;

  std::optional<RmNavigationRequestWrap> current_request_;
  NavigationFeatureUpdateCb update_cb_;
};
} // temoto_robot_manager namespace
#endif