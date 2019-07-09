// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__
#ifndef NearFarClipPlugin_h
#define NearFarClipPlugin_h


#include <gazebo/gui/GuiPlugin.hh>


namespace irg {

class NearFarClipPlugin : public gazebo::GUIPlugin
{
public:
  NearFarClipPlugin();
  ~NearFarClipPlugin();

  virtual void Load(sdf::ElementPtr _sdf);
};

}

#endif // NearFarClipPlugin_h
