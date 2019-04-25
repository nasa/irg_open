// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
#include "ShadowParametersSensorPlugin.h"

using namespace irg;
using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(ShadowParametersSensorPlugin)


void ShadowParametersSensorPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  LoadBase(_sdf);
}

