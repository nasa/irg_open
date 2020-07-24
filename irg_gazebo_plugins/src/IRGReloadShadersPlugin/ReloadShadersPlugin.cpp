// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ReloadShadersPlugin.h"

#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Heightmap.hh>


using namespace irg;
using namespace std;
using namespace gazebo;
using namespace Ogre;

#define UPDATE_RATE 1.0
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
  if (m_timer.GetElapsed().Double() < UPDATE_RATE)
  {
    return;
  }
  m_timer.Reset();
  m_timer.Start();

  mCurrentTime = time(NULL);

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

      // Reload shaders if they have changed
      bool sendToGPU = false;
      time_t time = ResourceGroupManager::getSingletonPtr()->resourceModifiedTime(vert->getGroup(), vert->getSourceFile());
      if (frag->isReloadable() && difftime(mCurrentTime, time) < UPDATE_RATE * 2.0 )
      {
        vert->reload();
        sendToGPU = true;
      }
      time = ResourceGroupManager::getSingletonPtr()->resourceModifiedTime(frag->getGroup(), frag->getSourceFile());
      if (frag->isReloadable() && difftime(mCurrentTime, time) < UPDATE_RATE * 2.0 )
      {
        frag->reload();
        sendToGPU = true;
      }

      // Ensure that shader gets sent to your GPU
      // https://forums.ogre3d.org/viewtopic.php?t=82115#p519782
      if (sendToGPU)
      {
        String combinedName = "Vertex Program:" + vert->getName() + " Fragment Program:" + frag->getName();
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
}

