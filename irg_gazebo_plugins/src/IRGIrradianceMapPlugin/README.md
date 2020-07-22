The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

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
    <background_color>0.1 0.2 0.3</background_color>
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

Optionally, you can set a `background_color` if you want the background (or
"sky") to be rendered with a specific color when it renders into your irradiance
environment map. The default `background_color` is (0.00022, 0.00022, 0.00022),
which is the lux due to starlight described in this paper:
https://www.researchgate.net/publication/238589855_Night_illumination_in_the_visible_NIR_and_SWIR_spectral_bands

