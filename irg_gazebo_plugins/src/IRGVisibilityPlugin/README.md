The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

IRG Gazebo plugins
==================================
IRGVisibilityPlugin
----------------------------------

This plugin calls SetVisibilityFlags() on a Gazebo Visual.

This is a visual plugin that can only be use within a Gazebo `<visual>` tag like this:
```
<visual name="visual">
  .
  .
  .
  <plugin name="visibility" filename="libIRGVisibilityPlugin.so">
    <visibility_bitmask>0x00010000</visibility_bitmask>
  </plugin>
</visual>
```

A bitmask can also be set on Ogre's SceneManager or Viewport. That bitmask will
be and-ed with the one set on your Gazebo Visual to determine whether the Visual
will be rendered.

### Tree traversal doesn't work
This plugin would be more useful if it could also set bitmasks on all children
of your plugin's Visual. However, there is no guarantee that Visuals underneath
your plugin's Visual have been instantiated at the time of loading the plugin.
In practice, you will find that some or none of the child Visuals have their
bitmasks set.

It should also be possible to inspect the SDF tree, find the names of all
Visuals, and set their masks as they are instantiated. However, SDF appears to
have a bug where calling `sdf::Element::GetParent()` on your visual element
always returns `nullptr`. This makes it impossible to traverse the sdf tree
starting from the `sdf::ElementPtr` passed to this plugin's `Load()` method'

Another option could have been to use a model plugin instead of a visual plugin.
However, model plugins are only loaded by gzserver, not gzclient.

