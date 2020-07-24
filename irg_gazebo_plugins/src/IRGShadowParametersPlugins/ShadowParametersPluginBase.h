// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef ShadowParametersPluginBase_h
#define ShadowParametersPluginBase_h

#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"

namespace irg {

// Base class contains meat of the code
class ShadowParametersPluginBase
{
public:
  ShadowParametersPluginBase();
  ~ShadowParametersPluginBase(){}

  virtual std::string GetClassName() = 0;

  void LoadBase(sdf::ElementPtr _sdf);

  void onUpdate();

protected:
  double m_constant_bias;
  double m_slope_scale_bias;

  // Connection to the update event
  gazebo::event::ConnectionPtr m_update_connection;
};

}

#endif // ShadowParametersPluginBase_h
