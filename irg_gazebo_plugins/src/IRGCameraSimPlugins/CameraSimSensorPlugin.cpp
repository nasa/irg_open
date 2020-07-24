// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "CameraSimSensorPlugin.h"
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>


using namespace irg;
using namespace gazebo;
using namespace gazebo::rendering;
using namespace Ogre;

GZ_REGISTER_SENSOR_PLUGIN(CameraSimSensorPlugin)


void CameraSimSensorPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  if (sdf == nullptr)
  {
    gzerr << "No SDF element specified. CameraSimSensorPlugin won't load." << std::endl;
    return;
  }

  sensors::CameraSensorPtr camera_sensor = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  if (camera_sensor)
  {
    m_camera_compositor.setupCompositor(sdf, camera_sensor->Camera());
  }

  sensors::MultiCameraSensorPtr multi_camera_sensor = std::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor);
  if (multi_camera_sensor)
  {
    for (unsigned int i=0; i<multi_camera_sensor->CameraCount(); i++)
    {
      m_camera_compositor.setupCompositor(sdf, multi_camera_sensor->Camera(i));
    }
  }
}
