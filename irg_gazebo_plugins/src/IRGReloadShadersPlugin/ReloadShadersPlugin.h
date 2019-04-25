// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
#ifndef ReloadShadersPlugin_h
#define ReloadShadersPlugin_h


#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <OGRE/OgreMaterial.h>

namespace irg {

class ReloadShadersPlugin : public gazebo::VisualPlugin
{
public:
  ReloadShadersPlugin();
  ~ReloadShadersPlugin();

  virtual void Load(gazebo::rendering::VisualPtr _sensor, sdf::ElementPtr _sdf);

  void OnUpdate();

private:
  void ReloadShaders(Ogre::MaterialPtr m);

  // Connection to the update event
  gazebo::event::ConnectionPtr m_updateConnection;

  gazebo::common::Timer m_timer;

  time_t mCurrentTime;
};

}


#endif // ReloadShadersPlugin_h
