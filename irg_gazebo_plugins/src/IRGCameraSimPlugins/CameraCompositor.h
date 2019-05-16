// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
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
