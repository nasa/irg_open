/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#include "ShadowParametersSensorPlugin.h"

using namespace gazebo;
using namespace gazebo::rendering;
GZ_REGISTER_SENSOR_PLUGIN(ShadowParametersSensorPlugin)


void ShadowParametersSensorPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  LoadBase(_sdf);
}

