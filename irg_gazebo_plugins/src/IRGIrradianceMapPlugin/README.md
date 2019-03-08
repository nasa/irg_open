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
