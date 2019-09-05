// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

#include "VisibilityPlugin.h"
#include <gazebo/rendering/Visual.hh>

using namespace irg;
using namespace std;
using namespace gazebo;
using namespace sdf;

GZ_REGISTER_VISUAL_PLUGIN(VisibilityPlugin)

VisibilityPlugin::VisibilityPlugin() :
  VisualPlugin()
{
}

VisibilityPlugin::~VisibilityPlugin()
{
}

void VisibilityPlugin::Load(rendering::VisualPtr visual, sdf::ElementPtr element)
{
  if (!element->HasElement("visibility_bitmask")) {
    gzerr << "VisibilityPlugin: you must specify a visibility_bitmask element." << endl;
    return;
  }
  const string mask_str = element->Get<string>("visibility_bitmask");
  const uint32_t visibility_bitmask = std::stoul(mask_str, nullptr, 16);

  visual->SetVisibilityFlags(visibility_bitmask);
}

