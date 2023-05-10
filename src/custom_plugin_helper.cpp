#include "temoto_robot_manager/custom_plugin_helper.h"
#include "temoto_resource_registrar/temoto_error.h"

namespace temoto_robot_manager
{

CustomPluginHelper::CustomPluginHelper(const std::string& plugin_path)
: plugin_path_(plugin_path)
, state_(State::NOT_LOADED)
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

  setState(State::INITIALIZED);
}
catch(class_loader::ClassLoaderException & e)
{
  throw TEMOTO_ERRSTACK(e.what());
}
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

  // TODO: invoke the plugin in its own thread
  if (!plugin->invoke(request))
  {
    setState(State::ERROR);
    throw TEMOTO_ERRSTACK("Unable to invoke the plugin");
  }

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
    throw TEMOTO_ERRSTACK("Unable to pre-empt plugin");
  }
}

void CustomPluginHelper::deinitialize()
try
{
  plugin.reset();
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