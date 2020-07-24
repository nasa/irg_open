// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef IRRADIANCE_MAP_PLUGIN_H
#define IRRADIANCE_MAP_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include "CubemapFilter.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreTexture.h>

namespace irg {

/**
 * @brief The IrradianceMapPlugin class
 * This is a visual plugin for Gazebo that generates an irradiance environment
 * cubemap from the point-of-view of its Visual. You would most likely attach it
 * to a placeholder Visual that is scaled down to zero or otherwise invisible.
 * The final irradiance environment cubemap can be used in any material in your
 * scene. This class renders the environment to a cubemap but lets another class
 * filter that cubemap to produce the final irradiance environment cubemap.
 */
class IrradianceMapPlugin : public gazebo::VisualPlugin
{
public:
  IrradianceMapPlugin();
  ~IrradianceMapPlugin();

  virtual void Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr element) override;

  void onUpdate();

private:
  static int m_index_counter;

  // A unique index to use in "global" Ogre resources, such as texture names
  int m_unique_index;

  gazebo::common::Timer m_timer;

  Ogre::String m_texture_unit_name;

  // This mask will interact with Ogre3D's visibility flags, which can be set
  // with gazebo::rendering::Visual::setVisibilityFlags(). The result of and-ing
  // this mask with that one will determine whether a visual is rendered.
  uint32_t m_visibility_bitmask;
  bool m_use_visibility_bitmask;

  // Connection to the update event
  gazebo::event::ConnectionPtr m_update_connection;

  Ogre::TexturePtr m_texture;

  // Cameras and viewports for capturing the scene
  Ogre::Camera* m_cameras[6];
  Ogre::Viewport* m_viewports[6];

  Ogre::ColourValue m_background_color;

  std::unique_ptr<CubemapFilter> m_cubemap_filter;
};

}

#endif // IRRADIANCE_MAP_PLUGIN_H

