// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
#include "CameraNoisePlugin.h"
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/ogre_gazebo.h>


using namespace gazebo;
using namespace gazebo::rendering;
using namespace Ogre;

GZ_REGISTER_SENSOR_PLUGIN(CameraNoisePlugin)


void CameraNoisePlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  sensors::CameraSensorPtr camera_sensor = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  if (camera_sensor)
  {
    setupCompositor(camera_sensor->Camera());
  }

  sensors::MultiCameraSensorPtr multi_camera_sensor = std::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor);
  if (multi_camera_sensor)
  {
    for (unsigned int i=0; i<multi_camera_sensor->CameraCount(); i++)
    {
      setupCompositor(multi_camera_sensor->Camera(i));
    }
  }
}

void CameraNoisePlugin::setupCompositor(CameraPtr camera)
{
  if (!m_compositor_listener)
  {
    m_compositor_listener.reset(new NoiseCompositorListener());
  }

  CompositorInstance* ci = Ogre::CompositorManager::getSingleton().addCompositor(
    camera->OgreViewport(), "IRGCameraNoisePlugin");
  if (ci)
  {
    ci->setEnabled(true);
    ci->addListener(m_compositor_listener.get());
  }
}

