#include "LightFrustaPlugin.h"
#include <gazebo/rendering/rendering.hh>


using namespace irg;
using namespace gazebo;


GZ_REGISTER_SYSTEM_PLUGIN(LightFrustaPlugin)


LightFrustaPlugin::~LightFrustaPlugin() {
  m_update.reset();
}

void LightFrustaPlugin::Init() {
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

