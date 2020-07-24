// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ShadowParametersSensorPlugin.h"

using namespace irg;
using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(ShadowParametersSensorPlugin)


void ShadowParametersSensorPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  LoadBase(_sdf);
}

