#include "temoto_robot_manager/custom_plugin_helper.h"
#include "temoto_resource_registrar/temoto_error.h"
#include <chrono>

namespace temoto_robot_manager
{

CustomPluginHelper::CustomPluginHelper(const std::string& plugin_path, CustomFeatureUpdateCb update_cb)
: plugin_path_(plugin_path)
, state_(State::NOT_LOADED)
, update_cb_(update_cb)
, current_request_{}
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

void CustomPluginHelper::initialize()
try
{
  if (getState() != State::UNINITIALIZED && getState() != State::FINISHED)
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

void CustomPluginHelper::invoke(const RmCustomRequestWrap& request)
{
  initialize();

  if (getState() != State::INITIALIZED)
  {
    setState(State::ERROR);
    throw TEMOTO_ERRSTACK("Cannot invoke the plugin. It has to be in 'INITIALIZED' state for that");
  }

  if (exec_thread_.joinable())
  {
    exec_thread_.join();
  }

  current_request_ = request;
  exec_thread_ = std::thread(
  [&]
  {
    if (plugin->invoke(request))
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
  sendUpdate();
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

void CustomPluginHelper::sendUpdate() const
{
  auto fb = plugin->getFeedback();
  if (fb.has_value())
  {
    RmCustomFeedbackWrap fbw;

    fbw.robot_name = current_request_->robot_name;
    fbw.custom_feature_name = current_request_->custom_feature_name;
    fbw.request_id = current_request_->request_id;
    fbw.status = uint8_t(state_);
    fbw.progress = fb->progress;

    update_cb_(fbw);
  }
}

}