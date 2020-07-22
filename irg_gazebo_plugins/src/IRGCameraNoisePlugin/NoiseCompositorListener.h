// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef NoiseCompositorListener_h
#define NoiseCompositorListener_h

#include <gazebo/rendering/ogre_gazebo.h>

namespace irg {

class NoiseCompositorListener : public Ogre::CompositorInstance::Listener
{
public:
  NoiseCompositorListener(){}
  //~NoiseCompositorListener(){}

  /// \brief Callback that Ogre3D will call before rendering each frame
  virtual void notifyMaterialRender(unsigned int pass_id, Ogre::MaterialPtr& mat) override;
};

}


#endif // NoiseCompositorListener_h
