// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
#ifndef CameraNoisePlugin_h
#define CameraNoisePlugin_h


#include "NoiseCompositorListener.h"
#include <gazebo/common/Plugin.hh>


namespace gazebo {

  class CameraNoisePlugin : public SensorPlugin
  {
  public: 
    CameraNoisePlugin(){}
    ~CameraNoisePlugin(){}

    virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

  private:
    void setupCompositor(gazebo::rendering::CameraPtr camera);

    boost::shared_ptr<NoiseCompositorListener> m_compositor_listener;
  };

}


#endif // CameraNoisePlugin_h
