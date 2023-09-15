#include "temoto_robot_manager/navigation_plugin_helper.h"
#include "temoto_resource_registrar/temoto_error.h"
#include <chrono>

#include <iostream>

namespace temoto_robot_manager
{

NavigationPluginHelper::NavigationPluginHelper(const std::string& plugin_path, const std::string& robot_ns, NavigationFeatureUpdateCb update_cb)
: plugin_path_(plugin_path)
, state_(State::NOT_LOADED)
, update_cb_(update_cb)
, current_request_{}
, robot_ns_(robot_ns)
{
try
{
  class_loader = std::make_shared<class_loader::ClassLoader>(plugin_path_, false);

  if (class_loader->getAvailableClasses<NavigationPluginBase>().empty())
  {
    setState(State::ERROR);
    throw TEMOTO_ERRSTACK("Library contains no plugins, check if the path is correct: '" + plugin_path_ + "'");
  }

  std::string plugin_name = class_loader->getAvailableClasses<NavigationPluginBase>().front();
  plugin = class_loader->createSharedInstance<NavigationPluginBase>(plugin_name);
  if (!class_loader->isLibraryLoaded())
  {
    setState(State::ERROR);
    throw TEMOTO_ERRSTACK("Unable to load plugin '" + plugin_path_ + "'");
  }

  setState(State::UNINITIALIZED);  
}
catch(class_loader::ClassLoaderException & e)
{
  throw TEMOTO_ERRSTACK(e.what());
}
}

NavigationPluginHelper::~NavigationPluginHelper()
{
  if (getState() == State::PROCESSING)
  {
    plugin->cancelGoal();

    while (!exec_thread_.joinable())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    exec_thread_.join();
    plugin->deinitialize();
  }

  else if (getState() == State::STOPPING)
  {
    while (!exec_thread_.joinable())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    exec_thread_.join();
    plugin->deinitialize();
  }

  else if(getState() == State::INITIALIZED || getState() == State::FINISHED || getState() == State::ERROR)
  {
    if (exec_thread_.joinable())
    {
      exec_thread_.join();
    }

    plugin->deinitialize();
  }
  
  plugin.reset();
}

void NavigationPluginHelper::initialize()
try
{
  if (getState() != State::UNINITIALIZED && getState() != State::FINISHED)
  {
    setState(State::ERROR);
    throw TEMOTO_ERRSTACK("Cannot initalize the plugin. It has to be in 'UNINITIALIZED' state for that");
  }
  if (!plugin->initialize(robot_ns_))
  {
    setState(State::ERROR);
    plugin.reset();
    throw TEMOTO_ERRSTACK("Unable to initialize the plugin");
  }
  setState(State::INITIALIZED);
}
catch(class_loader::ClassLoaderException & e)
{
  throw TEMOTO_ERRSTACK(e.what());
}

void NavigationPluginHelper::sendGoal(const RmNavigationRequestWrap& request)
{
  initialize();

  if (getState() != State::INITIALIZED)
  {
    setState(State::ERROR);
    throw TEMOTO_ERRSTACK("Cannot send navigation goal. The plugin has to be in 'INITIALIZED' state for that");
  }

  if (exec_thread_.joinable())
  {
    exec_thread_.join();
  }

  current_request_ = request;
  exec_thread_ = std::thread(
  [&]
  {
    if (plugin->sendGoal(request))
    {
      setState(State::FINISHED);  
    }
    else
    {
      setState(State::ERROR);
      //throw TEMOTO_ERRSTACK("Unable to invoke the plugin");
    }

    sendUpdate();
  });

  setState(State::PROCESSING);
  sendUpdate();
}

void NavigationPluginHelper::cancelGoal()
{

  if (getState() == State::FINISHED)
  {
    std::cout << "\033[1;32m [Plugin helper] Nothing to Cancel. Goal finished already\033[0m\n" <<std::endl;
    return;
  }

  if (getState() == State::PROCESSING)
  {
    // setState(State::ERROR);
    std::cout << "\033[1;32m [Plugin helper] State = processing\033[0m\n" <<std::endl;
    // throw TEMOTO_ERRSTACK("Cannot cancel the goal. Plugin has to be in 'PROCESSING' state for that");
    if (!plugin->cancelGoal())
    {
      std::cout << "\033[1;32m [Plugin helper] !plugin->cancelGoal \033[0m\n" <<std::endl;
      setState(State::ERROR);
      throw TEMOTO_ERRSTACK("Unable to cancel goal");
    }

    std::cout << "\033[1;32m [Plugin helper] end cancelgoal \033[0m\n" <<std::endl;
    setState(State::STOPPING);
    sendUpdate();
  }  
}

void NavigationPluginHelper::deinitialize()
try
{
  if (getState() != State::INITIALIZED)
  {
    setState(State::ERROR);
    throw TEMOTO_ERRSTACK("Cannot deinitialize the plugin. It has to be in 'INITIALIZED' state for that");
  }

  if (!plugin->deinitialize())
  {
    setState(State::ERROR);
    throw TEMOTO_ERRSTACK("Unable to deinitialize the plugin");
  }

  setState(State::UNINITIALIZED);
}
catch(...)
{
  throw TEMOTO_ERRSTACK("Unable to deinitialize the plugin");
}

NavigationPluginHelper::State NavigationPluginHelper::getState() const
{
  std::lock_guard<std::mutex> l(mutex_state_);
  return state_;
}

void NavigationPluginHelper::setState(State state)
{
  std::lock_guard<std::mutex> l(mutex_state_);
  state_ = state;
}

void NavigationPluginHelper::sendUpdate() const
{
  auto fb = plugin->getFeedback();
  if (fb.has_value())
  {
    RmNavigationFeedbackWrap fbw;
    
    fbw.robot_name = current_request_->robot_name;
    // fbw.navigation_feature_name = current_request_->navigation_feature_name;
    // fbw.request_id = current_request_->request_id;
    fbw.status = uint8_t(state_);
    fbw.progress = fb->progress;
    fbw.base_position = fb->base_position;
    update_cb_(fbw);
  }
}

}