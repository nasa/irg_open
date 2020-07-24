// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef LightFrustaPlugin_h
#define LightFrustaPlugin_h


#include <gazebo/common/Plugin.hh>


namespace irg {

class LightFrustaPlugin : public gazebo::SystemPlugin {
public:
  LightFrustaPlugin() {}

  ~LightFrustaPlugin();

  void Load(int _argc = 0, char **_argv = NULL) override;

  void disableLightFrusta();

private:
  gazebo::event::ConnectionPtr m_update;
};

}

#endif // LightFrustaPlugin_h
