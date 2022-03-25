// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "CameraSimVisualPlugin.h"
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>


using namespace irg;
using namespace gazebo;
using namespace gazebo::rendering;
using namespace Ogre;

GZ_REGISTER_VISUAL_PLUGIN(CameraSimVisualPlugin)


CameraSimVisualPlugin::CameraSimVisualPlugin() :
  m_width(0),
  m_height(0)
{
}

void CameraSimVisualPlugin::Load(rendering::VisualPtr visual, sdf::ElementPtr sdf)
{
  m_sdf = sdf;
  if (m_sdf == nullptr)
  {
    gzerr << "No SDF element specified. CameraSimVisualPlugin won't load." << std::endl;
    return;
  }

  rendering::ScenePtr scene = visual->GetScene();
  if (scene == nullptr)
  {
    gzerr << "Scene is null. CameraSimVisualPlugin won't load." << std::endl;
    return;
  }

  // This plugin is intended only for gzclient's UserCamera and not for gzserver
  if (scene->IsServer())
  {
    return;
  }

  // Should this handle multiple user cameras? In what cases are there more than one?
  m_camera = scene->GetUserCamera(0);
  if (m_camera == nullptr)
  {
    gzerr << "Camera is null. CameraSimVisualPlugin won't load." << std::endl;
    return;
  }

  m_width = m_camera->ViewportWidth();
  m_height = m_camera->ViewportHeight();
  m_camera_compositor.setupCompositor(m_sdf, m_camera);

  // Listen to the update event. This event is broadcast every sim iteration.
  m_update_connection = event::Events::ConnectPreRender(
    boost::bind(&CameraSimVisualPlugin::onUpdate, this));
}

void CameraSimVisualPlugin::onUpdate()
{
  // The compositor stops updating if the window is resized
  const unsigned int new_width = m_camera->ViewportWidth();
  const unsigned int new_height = m_camera->ViewportHeight();
  if (m_width != new_width || m_height != new_height)
  {
    m_width = new_width;
    m_height = new_height;
    m_camera_compositor.setupCompositor(m_sdf, m_camera);
  }
}
