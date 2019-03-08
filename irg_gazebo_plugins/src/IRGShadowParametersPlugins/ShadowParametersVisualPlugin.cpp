/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#include "ShadowParametersVisualPlugin.h"

using namespace gazebo;
using namespace gazebo::rendering;
GZ_REGISTER_VISUAL_PLUGIN(ShadowParametersVisualPlugin)


void ShadowParametersVisualPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  LoadBase(_sdf);
}
