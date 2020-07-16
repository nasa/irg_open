The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

IRG Gazebo plugins
==================================
IRGLightFrustalugin
-------------------

This is a system plugin that can be used when launching gzclient:

```
gzclient --gui-client-plugin libIRGLightFrustalugin.so
```

This plugin finds lights in your scene and disables visual objects associated
with them, specifically their bright green wireframe frusta.

