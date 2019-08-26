IRG Gazebo plugins
==================================
IRGIrradianceMapPlugin
----------------------------------

This plugin renders a dynamic cubemap and processes it to create an irradiance
environment map.

This is a visual plugin that can only be use within a Gazebo `<visual>` tag like this:
```
<visual name="visual">
  .
  .
  .
  <plugin name="irradiance_map" filename="libIRGIrradianceMapPlugin.so">
    <texture_unit>irradiance_environment_map</texture_unit>
    <visibility_bitmask>0x0000000f</visibility_bitmask>
  </plugin>
</visual>
```

Your irradiance map can be used globally. Wherever you want it to be used, add
to an Ogre3D material script a texture_unit with a name matching the one passed
to the plugin:
```
  texture_unit irradiance_environment_map
  {
  }
```

Optionally, you can set a `visibility_bitmask` if you want to selectively render
gazebo::rendering::Visuals into your irradiance map. You can set a bitmask on a
Visual by calling `Visual::SetVisibilityFlags()`. If the result of and-ing that
bitmask and your `visibility_bitmask` is non-zero, that Visual will be rendered
into your irradiance map. Otherwise, it will not be rendered.

