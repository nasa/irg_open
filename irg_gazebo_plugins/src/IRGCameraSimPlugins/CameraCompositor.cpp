// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "CameraCompositor.h"


using namespace irg;
using namespace gazebo;
using namespace gazebo::rendering;
using namespace Ogre;


void CameraCompositor::setupCompositor(sdf::ElementPtr sdf, CameraPtr camera)
{
  // Create the listener
  if (!m_compositor_listener)
  {
    m_compositor_listener.reset(new CameraCompositorListener(sdf));
  }

  // Remove compositor if one has already been applied.
  // This lets us create a new one in case the viewport has been resized, which
  // can happen with a GUIPlugin running on the Gazebo client.
  CompositorPtr compositor = Ogre::CompositorManager::getSingleton().getByName("IRGCameraSim");
  if (compositor.get() != nullptr)
  {
    Ogre::CompositorManager::getSingleton().removeCompositor(
      camera->OgreViewport(), "IRGCameraSim");
  }

  // Create the compositor and attach the listener to it
  CompositorInstance* ci = Ogre::CompositorManager::getSingleton().addCompositor(
    camera->OgreViewport(), "IRGCameraSim");
  if (ci)
  {
    ci->setEnabled(true);
    ci->addListener(m_compositor_listener.get());
  }
}
