// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef CameraCompositor_h
#define CameraCompositor_h


#include "CameraCompositorListener.h"
#include <gazebo/rendering/Camera.hh>


namespace irg {

class CameraCompositor
{
public:
  CameraCompositor(){}

  void setupCompositor(sdf::ElementPtr sdf, gazebo::rendering::CameraPtr camera);

private:
  std::shared_ptr<CameraCompositorListener> m_compositor_listener;
};

}


#endif // CameraCompositor_h
