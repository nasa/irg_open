// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

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

