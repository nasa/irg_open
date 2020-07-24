// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef VISIBILITY_PLUGIN_H
#define VISIBILITY_PLUGIN_H

#include <gazebo/common/Plugin.hh>

namespace irg {

/**
 * @brief The VisibilityPlugin class
 * This is a visual plugin for Gazebo that lets you set a bitmask for
 * selectively rendering that visual. During rendering you can set a visibility
 * mask on Ogre's SceneManager or Viewport which will be and-ed with the bitmask
 * on this plugin's Visual.
 */
class VisibilityPlugin : public gazebo::VisualPlugin
{
public:
  VisibilityPlugin();
  ~VisibilityPlugin();

  virtual void Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr element) override;
};

}

#endif // VISIBILITY_PLUGIN_H

