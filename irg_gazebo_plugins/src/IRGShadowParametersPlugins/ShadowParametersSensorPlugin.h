// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef ShadowParametersSensorPlugin_h
#define ShadowParametersSensorPlugin_h

#include "ShadowParametersPluginBase.h"

namespace irg {

// Any params set by this plugin will be overridden by
// ShadowParametersVisualPlugin. This is because shadow parameters are set
// globally in Gazebo--not per view or per sensor.
class ShadowParametersSensorPlugin : public ShadowParametersPluginBase, public gazebo::SensorPlugin
{
public:
  ShadowParametersSensorPlugin(){}
  ~ShadowParametersSensorPlugin(){}

  virtual std::string GetClassName(){ return "ShadowParametersSensorPlugin"; }

  virtual void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
};

}

#endif // ShadowParametersSensorPlugin_h
