// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
#ifndef CameraSimSensorPlugin_h
#define CameraSimSensorPlugin_h


#include "CameraCompositor.h"
#include <gazebo/common/Plugin.hh>


namespace irg {

/**
 * @brief The CameraSimSensorPlugin class
 * This SensorPlugin will add a digital camera simulation to any sensor of type
 * 'camera' or 'multicamera' in your SDF code.
 */
class CameraSimSensorPlugin : public gazebo::SensorPlugin
{
public:
  CameraSimSensorPlugin(){}

  virtual void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  CameraCompositor m_camera_compositor;
};

}


#endif // CameraSimSensorPlugin_h
