/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#ifndef CelestialBodyPlugin_h
#define CelestialBodyPlugin_h

#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <tf2_ros/transform_listener.h>

namespace irg {

  class CelestialBodyPlugin : public gazebo::ModelPlugin
  {
  public: 
    CelestialBodyPlugin();
    ~CelestialBodyPlugin();
    
    virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

    void OnUpdate();

  private:
    gazebo::physics::ModelPtr m_model;

    std::string m_frame;
    double m_radius;
    double m_renderDistance;
    bool m_lightSource;

    // Connection to the update event
    gazebo::event::ConnectionPtr m_updateConnection;

    gazebo::common::Timer m_timer;

    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener m_transformListener;
  };
}

#endif // CelestialBodyPlugin_h
