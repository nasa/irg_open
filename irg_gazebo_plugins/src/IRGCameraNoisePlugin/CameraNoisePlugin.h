// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef CameraNoisePlugin_h
#define CameraNoisePlugin_h


#include "NoiseCompositorListener.h"
#include <gazebo/common/Plugin.hh>


namespace irg {

class CameraNoisePlugin : public gazebo::SensorPlugin
{
public:
  CameraNoisePlugin(){}
  //~CameraNoisePlugin(){}

  virtual void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  void setupCompositor(gazebo::rendering::CameraPtr camera);

  std::shared_ptr<NoiseCompositorListener> m_compositor_listener;
};

}


#endif // CameraNoisePlugin_h
