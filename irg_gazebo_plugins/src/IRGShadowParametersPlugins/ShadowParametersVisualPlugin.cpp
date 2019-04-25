// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
#include "ShadowParametersVisualPlugin.h"

using namespace irg;
using namespace gazebo;
GZ_REGISTER_VISUAL_PLUGIN(ShadowParametersVisualPlugin)


void ShadowParametersVisualPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  LoadBase(_sdf);
}
