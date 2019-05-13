// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
#ifndef CameraSimVisualPlugin_h
#define CameraSimVisualPlugin_h


#include "CameraSimBase.h"
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
class CameraSimVisualPlugin : public CameraSimBase, public gazebo::VisualPlugin
{
public:
  CameraSimVisualPlugin();

  virtual void Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf) override;

private:
  /// \brief Re-initialize compositor on window resize
  void onUpdate();

  sdf::ElementPtr m_sdf;

  gazebo::rendering::UserCameraPtr m_camera;

  unsigned int m_width;
  unsigned int m_height;

  // Connection to the update event
  gazebo::event::ConnectionPtr m_update_connection;
};

}


#endif // CameraSimVisualPlugin_h
