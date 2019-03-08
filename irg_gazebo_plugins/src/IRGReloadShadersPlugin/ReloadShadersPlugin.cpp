/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#include "ReloadShadersPlugin.h"

#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Heightmap.hh>


using namespace std;
using namespace gazebo;
using namespace Ogre;

#define OGRE_GREATER_THAN_1_8 !(OGRE_VERSION < ((1 << 16) | (9 << 8) | 0))

GZ_REGISTER_VISUAL_PLUGIN(ReloadShadersPlugin)

ReloadShadersPlugin::ReloadShadersPlugin() :
  VisualPlugin()
{
  m_timer.Start();
}

ReloadShadersPlugin::~ReloadShadersPlugin()
{
}

void ReloadShadersPlugin::Load(rendering::VisualPtr _sensor, sdf::ElementPtr _sdf)
{
  // Listen to the update event. This event is broadcast every sim iteration.
  // If result goes out of scope. updates will stop, so it assigned to a member variable.
  m_updateConnection = event::Events::ConnectPreRender(boost::bind(&ReloadShadersPlugin::OnUpdate, this));
}


void ReloadShadersPlugin::OnUpdate()
{
  // Only continue if a second has elapsed
  if (m_timer.GetElapsed().Double() < 1.0)
  {
    return;
  }
  m_timer.Reset();
  m_timer.Start();

  // reload all vertex and fragment programs for every material
  Ogre::ResourceManager::ResourceMapIterator it = Ogre::MaterialManager::getSingleton().getResourceIterator();
  while (it.hasMoreElements())
  {
    ResourcePtr r = it.getNext();
    if (r.isNull())
    {
      continue;
    }
#if OGRE_GREATER_THAN_1_8
    MaterialPtr m = r.staticCast<Material>();
#else 
    MaterialPtr m = r;
#endif

    // HACK: materials starting with these strings cause assertion:
    // /OgreMain/src/OgreGpuProgramParams.cpp:1099: void Ogre::GpuProgramParameters::_writeRawConstants(size_t, const float*, size_t): Assertion `physicalIndex + count <= mFloatConstants.size()' failed
    // There is some suggestion this is an Ogre bug: https://forums.ogre3d.org/viewtopic.php?p=222740
    // It would be better to find a general-purpose fix or workaround.
    if (StringUtil::startsWith(m->getName(), "mobile_base"))
    {
      continue;
    }
    if (StringUtil::startsWith(m->getName(), "fake_sun"))
    {
      continue;
    }
    if (StringUtil::startsWith(m->getName(), "sun_sphere"))
    {
      continue;
    }
    if (StringUtil::startsWith(m->getName(), "global_shader_param_loader"))
    {
      continue;
    }

    ReloadShaders(m);
  }
}

void ReloadShadersPlugin::ReloadShaders(Ogre::MaterialPtr m)
{
  Ogre::Material::TechniqueIterator ti = m->getTechniqueIterator();
  while (ti.hasMoreElements())
  {
    Technique *t = ti.getNext();
    if (t == NULL)
    {
      continue;
    }
    if (!t->isSupported() || !t->isLoaded())
    {
      continue;
    }

    Ogre::Technique::PassIterator pi = t->getPassIterator();
    while (pi.hasMoreElements())
    {
      Pass* p = pi.getNext();
      if (p == NULL)
      {
        continue;
      }
      if (!p->isLoaded())
      {
        continue;
      }

      // Get pointers
      GpuProgramPtr vert;
      vert.setNull();
      if (p->hasVertexProgram())
      {
        vert = p->getVertexProgram();
      }
      GpuProgramPtr frag;
      frag.setNull();
      if (p->hasFragmentProgram())
      {
        frag = p->getFragmentProgram();
      }
      if (vert.isNull() || frag.isNull())
      {
        continue;
      }

      // reload shaders
      vert->reload();
      frag->reload();

      // Ensure that shader gets sent to your GPU
      // https://forums.ogre3d.org/viewtopic.php?t=82115#p519782
      String combinedName = "Vertex Program:" + vert->getName()
        + " Fragment Program:" + frag->getName();
      if (GpuProgramManager::getSingleton().isMicrocodeAvailableInCache(combinedName))
      {
#if OGRE_GREATER_THAN_1_8
        GpuProgramManager::getSingleton().removeMicrocodeFromCache(combinedName);
#else
        gzerr << "ReloadShadersPlugin requires Ogre 1.9... this plugin should not be enabled\n";
#endif
      }
    }
  }
}

