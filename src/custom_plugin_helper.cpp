#include "temoto_robot_manager/custom_plugin_helper.h"
#include "temoto_resource_registrar/temoto_error.h"
#include <chrono>

namespace temoto_robot_manager
{

CustomPluginHelper::CustomPluginHelper(const std::string& plugin_path, CustomFeatureUpdateCb update_cb)
: plugin_path_(plugin_path)
, state_(State::NOT_LOADED)
, update_cb_(update_cb)
{
try
{
  class_loader = std::make_shared<class_loader::ClassLoader>(plugin_path_, false);

  if (class_loader->getAvailableClasses<CustomPluginBase>().empty())
  {
    setState(State::ERROR);
    throw TEMOTO_ERRSTACK("Library contains no plugins, check if the path is correct: '" + plugin_path_ + "'");
  }

  std::string plugin_name = class_loader->getAvailableClasses<CustomPluginBase>().front();
  plugin = class_loader->createSharedInstance<CustomPluginBase>(plugin_name);
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

CustomPluginHelper::~CustomPluginHelper()
{
  if (getState() == State::PROCESSING)
  {
    plugin->preempt();

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

  else if(getState() == State::INITIALIZED || getState() == State::ERROR)
  {
    if (exec_thread_.joinable())
    {
      exec_thread_.join();
    }

    plugin->deinitialize();
  }
  
  plugin.reset();
}

void CustomPluginHelper::initialize()
try
{
  if (getState() != State::UNINITIALIZED)
  {
    setState(State::ERROR);
    throw TEMOTO_ERRSTACK("Cannot initalize the plugin. It has to be in 'UNINITIALIZED' state for that");
  }

  if (!plugin->initialize())
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

void CustomPluginHelper::invoke(const RmCustomRequest& request)
{
  if (getState() != State::INITIALIZED)
  {
    setState(State::ERROR);
    throw TEMOTO_ERRSTACK("Cannot invoke the plugin. It has to be in 'INITIALIZED' state for that");
  }

  if (exec_thread_.joinable())
  {
    exec_thread_.join();
  }

  exec_thread_ = std::thread(
  [&]
  {
    if (plugin->invoke(request))
    {
      setState(State::INITIALIZED);  
    }
    else
    {
      setState(State::ERROR);
      //throw TEMOTO_ERRSTACK("Unable to invoke the plugin");
    }
  });

  setState(State::PROCESSING);
}

void CustomPluginHelper::preempt()
{
  if (getState() != State::PROCESSING)
  {
    setState(State::ERROR);
    throw TEMOTO_ERRSTACK("Cannot pre-empt the plugin. It has to be in 'PROCESSING' state for that");
  }

  if (!plugin->preempt())
  {
    setState(State::ERROR);
    throw TEMOTO_ERRSTACK("Unable to pre-empt the plugin");
  }

  setState(State::STOPPING);
}

void CustomPluginHelper::deinitialize()
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

CustomPluginHelper::State CustomPluginHelper::getState() const
{
  std::lock_guard<std::mutex> l(mutex_state_);
  return state_;
}

void CustomPluginHelper::setState(State state)
{
  std::lock_guard<std::mutex> l(mutex_state_);
  state_ = state;
}

}