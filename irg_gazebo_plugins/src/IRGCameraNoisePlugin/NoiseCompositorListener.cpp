// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
#include "NoiseCompositorListener.h"
#include <gazebo/common/Assert.hh>


void NoiseCompositorListener::notifyMaterialRender(unsigned int pass_id,
                                                   Ogre::MaterialPtr& mat)
{
  GZ_ASSERT(!mat.isNull(), "Null Ogre3D material");
  Ogre::Technique* technique = mat->getTechnique(0);
  GZ_ASSERT(technique, "Null Ogre3D technique");
  Ogre::Pass* pass = technique->getPass(pass_id);
  GZ_ASSERT(pass, "Null Ogre3D pass");
  Ogre::GpuProgramParametersSharedPtr params = pass->getFragmentProgramParameters();
  GZ_ASSERT(!params.isNull(), "Null Ogre3D material GPU parameters");

  // The noise shader contains a simple pseudorandom number generator based on
  // fragment position that is used 3 times for each fragment. Generate 3
  // random numbers here that will be added the inputs to that generator,
  // making the final random numbers different for each frame.
  Ogre::Vector3 offsets(double(rand()) / double(RAND_MAX),
                        double(rand()) / double(RAND_MAX),
                        double(rand()) / double(RAND_MAX));

  params->setNamedConstant("offsets", offsets);
}
