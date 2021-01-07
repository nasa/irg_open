// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef GlobalShaderParamPlugin_h
#define GlobalShaderParamPlugin_h

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <irg_gazebo_plugins/ShaderParamUpdate.h>

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
  void onShaderParamUpdate(const irg_gazebo_plugins::ShaderParamUpdate::ConstPtr& msg);

  void onPostRender();
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

  std::string shaderTypeName(int8_t t);

private:
  bool m_hasUpdates;
  bool m_cacheCleared;

  gazebo::event::ConnectionPtr m_postRenderConnection;
  gazebo::event::ConnectionPtr m_entityAddedConnection;
  gazebo::event::ConnectionPtr m_entityDeletedConnection;
  gazebo::event::ConnectionPtr m_worldCreatedConnection;

  std::unique_ptr<ros::NodeHandle> m_nodeHandle;
  ros::Subscriber                  m_subscriber;

  std::map<std::string,GpuProgramParamsList> m_paramsListMap[irg_gazebo_plugins::ShaderParamUpdate::NUM_SHADER_TYPES];
  std::map<std::string,std::string>          m_paramUpdateMap[irg_gazebo_plugins::ShaderParamUpdate::NUM_SHADER_TYPES];

  typedef std::recursive_mutex Mutex;
  typedef std::lock_guard<std::recursive_mutex> Lock;
  Mutex m_mutex;
};

}

#endif // GlobalShaderParamPlugin_h

