// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef ShadowParametersVisualPlugin_h
#define ShadowParametersVisualPlugin_h

#include "ShadowParametersPluginBase.h"

namespace irg {

// This will override any params set by ShadowParametersSensorPlugin. This is
// because shadow parameters are set globally in Gazebo--not per view or per
// sensor.
class ShadowParametersVisualPlugin : public ShadowParametersPluginBase, public gazebo::VisualPlugin
{
public:
  ShadowParametersVisualPlugin(){}
  ~ShadowParametersVisualPlugin(){}

  virtual std::string GetClassName(){ return "ShadowParametersVisualPlugin"; }

  virtual void Load(gazebo::rendering::VisualPtr _visual, sdf::ElementPtr _sdf);
};

}

#endif // ShadowParametersVisualPlugin_h
