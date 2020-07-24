// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

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
