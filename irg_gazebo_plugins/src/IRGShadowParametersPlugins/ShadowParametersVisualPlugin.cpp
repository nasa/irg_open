// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ShadowParametersVisualPlugin.h"

using namespace irg;
using namespace gazebo;
GZ_REGISTER_VISUAL_PLUGIN(ShadowParametersVisualPlugin)


void ShadowParametersVisualPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  LoadBase(_sdf);
}
