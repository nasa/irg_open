// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
#ifndef GlobalShaderParamPlugin_h
#define GlobalShaderParamPlugin_h

#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>

#include "std_msgs/msg/string.hpp"
#include "irg_gazebo_plugins/msg/shader_param_update.hpp"

#include <map>
#include <vector>
#include <string>

#include <OgreGpuProgramParams.h> 

namespace irg {
  
class GlobalShaderParamPlugin : public gazebo::VisualPlugin
{
public:
  GlobalShaderParamPlugin();
  ~GlobalShaderParamPlugin();

  virtual void Load(gazebo::rendering::VisualPtr _sensor, sdf::ElementPtr _sdf);

protected:
  //void onShaderUpdateMsg(const std_msgs::String::ConstPtr& msg);
  void onShaderParamUpdate(const irg_gazebo_plugins::msg::ShaderParamUpdate::SharedPtr msg);

  void onPreRender();
  void onAddEntity();
  void onDeleteEntity();
  void onWorldCreated();

  typedef std::vector<Ogre::GpuProgramParametersSharedPtr> GpuProgramParamsList;
  void clearCache();
  void buildCache();
  void cacheParams(int8_t, const std::string& paramName, GpuProgramParamsList& paramsList);

  void handleUpdates();

  void setParam(Ogre::GpuProgramParametersSharedPtr params,
                const std::string& paramName, const std::string& paramValue);

private:
  bool m_hasUpdates;
  bool m_cacheCleared;

  gazebo::event::ConnectionPtr m_preRenderConnection;
  gazebo::event::ConnectionPtr m_entityAddedConnection;
  gazebo::event::ConnectionPtr m_entityDeletedConnection;
  gazebo::event::ConnectionPtr m_worldCreatedConnection;

  gazebo_ros::Node::SharedPtr m_rosNode;
  rclcpp::Subscription<irg_gazebo_plugins::msg::ShaderParamUpdate>::ConstSharedPtr m_subscriber;

  std::map<std::string,GpuProgramParamsList> m_paramsListMap[irg_gazebo_plugins::msg::ShaderParamUpdate::NUM_SHADER_TYPES];
  std::map<std::string,std::string>          m_paramUpdateMap[irg_gazebo_plugins::msg::ShaderParamUpdate::NUM_SHADER_TYPES];

  typedef std::recursive_mutex Mutex;
  typedef std::lock_guard<std::recursive_mutex> Lock;
  Mutex m_mutex;
};

}

#endif // GlobalShaderParamPlugin_h

