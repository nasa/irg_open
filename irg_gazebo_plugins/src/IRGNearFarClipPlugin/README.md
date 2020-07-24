The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

IRG Gazebo plugins
==================================
IRGNearFarClipPlugin
----------------------------------

This is a GUI plugin that can only be use within a Gazebo `<gui>` tag like this:

```
<gui fullscreen='0'>
  .
  .
  .
  <plugin name="near_far_clip" filename="libIRGNearFarClipPlugin.so">
    <near>0.1</near>
    <far>200000.0</far>
  </plugin>
</gui>
```

`<near>` and `<far>` elements are optional and default to the values shown
above. These differ from the gazebo default values of 0.1 and 5000.

