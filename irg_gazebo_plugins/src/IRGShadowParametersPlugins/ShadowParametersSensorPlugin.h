/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#ifndef ShadowParametersSensorPlugin_h
#define ShadowParametersSensorPlugin_h

#include "ShadowParametersPluginBase.h"

namespace gazebo {

  // Any params set by this plugin will be overridden by
  // ShadowParametersVisualPlugin. This is because shadow parameters are set
  // globally in Gazebo--not per view or per sensor.
  class ShadowParametersSensorPlugin : public ShadowParametersPluginBase, public SensorPlugin
  {
  public: 
    ShadowParametersSensorPlugin(){}
    ~ShadowParametersSensorPlugin(){}

    virtual std::string GetClassName(){ return "ShadowParametersSensorPlugin"; }

    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
  };

}

#endif // ShadowParametersSensorPlugin_h
