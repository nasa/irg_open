// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef CameraSimVisualPlugin_h
#define CameraSimVisualPlugin_h


#include "CameraCompositor.h"
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/UserCamera.hh>


namespace irg {

/**
 * @brief The CameraSimVisualPlugin class
 * This VisualPlugin will add a digital camera simulation to Gazebo's GUI
 * camera. It was more clear to implement this as a GUIPlugin, but that
 * implementation always resulted in a 640x480 black box drawn on top of the
 * contents of the GUI window. I was unable to defeat this bug.
 */
class CameraSimVisualPlugin : public gazebo::VisualPlugin
{
public:
  CameraSimVisualPlugin();

  virtual void Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf) override;

private:
  /// \brief Re-initialize compositor on window resize
  void onUpdate();

private:
  CameraCompositor m_camera_compositor;

  sdf::ElementPtr m_sdf;

  gazebo::rendering::UserCameraPtr m_camera;

  unsigned int m_width;
  unsigned int m_height;

  // Connection to the update event
  gazebo::event::ConnectionPtr m_update_connection;
};

}


#endif // CameraSimVisualPlugin_h
