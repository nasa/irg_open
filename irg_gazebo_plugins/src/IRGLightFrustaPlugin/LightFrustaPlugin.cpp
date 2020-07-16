// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "LightFrustaPlugin.h"
#include <gazebo/rendering/rendering.hh>


using namespace irg;
using namespace gazebo;


GZ_REGISTER_SYSTEM_PLUGIN(LightFrustaPlugin)


LightFrustaPlugin::~LightFrustaPlugin() {
  m_update.reset();
}

void LightFrustaPlugin::Load(int _argc, char **_argv) {
  m_update = event::Events::ConnectPreRender(std::bind(&LightFrustaPlugin::disableLightFrusta, this));
}

void LightFrustaPlugin::disableLightFrusta() {
  rendering::ScenePtr scene = rendering::get_scene();
  if (!scene || !scene->Initialized())
  {
    return;
  }

  // Iterate over the children 
  for (uint32_t l = 0; l < scene->LightCount(); l++) {
    rendering::LightPtr light = scene->LightByIndex(l);
    if (!light)
    {
      continue;
    }

    // Get the light's visual
    rendering::VisualPtr visual = scene->GetVisual(light->Name());
    if (!visual)
    {
      continue;
    }

    // Get the scene node and hide its frustum model
    Ogre::SceneNode* node = visual->GetSceneNode();
    // numAttachedObjects() usually returns 0 for link lights. Is it a bug?
    for (auto i = 0; i < node->numAttachedObjects(); i++)
    {
      Ogre::MovableObject* obj = node->getAttachedObject(i);
      if (obj->getMovableType() != "Light")
      {
        obj->setVisible(false);
      }
    }
  }
}

