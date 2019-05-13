// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
#ifndef CameraSimBase_h
#define CameraSimBase_h


#include "CameraCompositorListener.h"
#include <gazebo/rendering/Camera.hh>


namespace irg {

class CameraSimBase
{
public:
  CameraSimBase(){}

protected:
  void setupCompositor(sdf::ElementPtr sdf, gazebo::rendering::CameraPtr camera);

  std::shared_ptr<CameraCompositorListener> m_compositor_listener;
};

}


#endif // CameraSimBase_h
