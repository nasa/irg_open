// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
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
