// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
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
