// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ShadowParametersPluginBase.h"

#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RTShaderSystem.hh"


using namespace irg;
using namespace gazebo;
using namespace gazebo::rendering;
using namespace Ogre;


ShadowParametersPluginBase::ShadowParametersPluginBase() :
  m_constant_bias(0.0),
  m_slope_scale_bias(0.0)
{
}


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

  bool set_depth_bias = false;
  if (_sdf->HasElement("constant_bias")) {
    m_constant_bias = _sdf->Get<double>("constant_bias");
    set_depth_bias = true;
  }
  if (_sdf->HasElement("slope_scale_bias")) {
    m_slope_scale_bias = _sdf->Get<double>("slope_scale_bias");
    set_depth_bias = true;
  }
  // Set depth bias if user has set either one of its parameters
  if (set_depth_bias)
  {
    // Listen to the update event. This event is broadcast every sim iteration.
    m_update_connection = event::Events::ConnectPreRender(
      boost::bind(&ShadowParametersPluginBase::onUpdate, this));
  }
}

void ShadowParametersPluginBase::onUpdate()
{
  // Set depth bias for rendering shadow maps.
  // This is not done when loading the plugin because the material isn't ready.
  MaterialPtr material = MaterialManager::getSingletonPtr()->getByName("Gazebo/shadow_caster");
  if (!material.isNull())
  {
    Technique* t = material->getTechnique(0);
    if (t != nullptr)
    {
      Pass* p = t->getPass(0);
      if (p != nullptr)
      {
        p->setDepthBias(m_constant_bias, m_slope_scale_bias);

        // disconnect so this method will not be called again
        m_update_connection.reset();
      }
    }
  }
}
