IRG Gazebo plugins
==================================
IRGReloadShadersPlugin
----------------------------------

This is a visual plugin that reloads all shaders every second, so you can save
a vertex or fragment shader and immediately see the updates without restarting
Gazebo. This is intended as a development tool, not something that would be
enabled in a final product. It can be used within a Gazebo <visual> tag like this:

```
<visual name="visual">
  .
  .
  .
  <plugin name="reload_shaders" filename="libIRGReloadShadersPlugin.so" />
  .
  .
  .
</visual>
```

