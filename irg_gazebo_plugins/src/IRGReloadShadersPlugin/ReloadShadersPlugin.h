/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#ifndef ReloadShadersPlugin_h
#define ReloadShadersPlugin_h


#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <OGRE/OgreMaterial.h>

namespace gazebo {

  class ReloadShadersPlugin : public VisualPlugin
  {
  public: 
    ReloadShadersPlugin();
    ~ReloadShadersPlugin();

    virtual void Load(rendering::VisualPtr _sensor, sdf::ElementPtr _sdf);

    void OnUpdate();

  private:
    void ReloadShaders(Ogre::MaterialPtr m);

    // Connection to the update event
    event::ConnectionPtr m_updateConnection;

    common::Timer m_timer;
  };

}


#endif // ReloadShadersPlugin_h
