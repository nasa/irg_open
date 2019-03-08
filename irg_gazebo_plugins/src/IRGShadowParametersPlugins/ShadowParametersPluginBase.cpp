/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#include "ShadowParametersPluginBase.h"

#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RTShaderSystem.hh"

using namespace gazebo;
using namespace gazebo::rendering;


void ShadowParametersPluginBase::LoadBase(sdf::ElementPtr _sdf)
{
  rendering::ScenePtr scene = rendering::get_scene();
  if (!scene) {
    gzerr << GetClassName() << ": scene pointer is NULL" << std::endl;
    return;
  }

  if (_sdf->HasElement("shadow_texture_size")) {
    unsigned int size = _sdf->Get<unsigned int>("shadow_texture_size");
    scene->SetShadowTextureSize(size);
  }

  const bool hasNear = _sdf->HasElement("shadow_near");
  const bool hasFar = _sdf->HasElement("shadow_far");
  if (hasNear || hasFar) {
    double near = RTShaderSystem::Instance()->ShadowNearClip();
    double far = RTShaderSystem::Instance()->ShadowFarClip();
    if (hasNear) {
      near = _sdf->Get<double>("shadow_near");
    }
    if (hasFar) {
      far = _sdf->Get<double>("shadow_far");
    }
    RTShaderSystem::Instance()->SetShadowClipDist(near, far);
  }

  if (_sdf->HasElement("shadow_split_lambda")) {
    double lambda = _sdf->Get<double>("shadow_split_lambda");
    RTShaderSystem::Instance()->SetShadowSplitLambda(lambda);
  }

  if (_sdf->HasElement("shadow_split_padding")) {
    double padding = _sdf->Get<double>("shadow_split_padding");
    RTShaderSystem::Instance()->SetShadowSplitPadding(padding);
  }
}

