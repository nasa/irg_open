// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

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
