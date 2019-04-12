// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
#ifndef NoiseCompositorListener_h
#define NoiseCompositorListener_h

#include <gazebo/rendering/ogre_gazebo.h>

class NoiseCompositorListener : public Ogre::CompositorInstance::Listener
{
public:
  NoiseCompositorListener(){}
  //~NoiseCompositorListener(){}

  /// \brief Callback that Ogre3D will call before rendering each frame
  virtual void notifyMaterialRender(unsigned int pass_id, Ogre::MaterialPtr& mat) override;
};


#endif // NoiseCompositorListener_h
