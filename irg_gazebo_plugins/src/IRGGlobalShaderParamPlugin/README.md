The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

IRG Gazebo plugins
==================================
IRGGlobalShaderParamPlugin
----------------------------------

This Visual Gazebo plugin allows uniform variables in GLSL shaders to be manipulated in real time through a ROS topic. 

Although the plugin is declared in SDF in a visual block, functionality of the plugin is *not* associated with that visual. It is a visual plugin simply because we want it to be able to manipulate uniforms on both the client and server sides (World plugins only work server-side). 

To use the plugin, arbitrarily pick a visual in the world and declare it:

```
<visual name="sphere_0_visual">
  <cast_shadows>1</cast_shadows>
  <geometry>
    <sphere>
      <radius>0.1</radius>
    </sphere>
  </geometry>
  <plugin name="GlobalShaderParam" filename="libIRGGlobalShaderParamPlugin.so">
    <param>
      <type>fragment</type>
      <name>exposureMultiplier</name>
    </param>
  </plugin>
</visual>
```

The above block will initialize the plugin to receive dynamic updates to a uniform named "exposureMultiplier" in a fragment program. **All** materials that have a fragment program containing a uniform variable that matches this parameter name will be updated. 

The plugin listens for irg_gazebo_plugins::ShaderParamUpdate messages on topic `/gazebo/global_shader_param`. 

To use rostopic to publish a uniform value change:
```
rostopic pub --once /gazebo/global_shader_param irg_gazebo_plugins/ShaderParamUpdate '{ shaderType: 1, paramName: "exposureMultiplier", paramValue: "2.0" }'
```
  - `shaderType` is one of:
```
  SHADER_TYPE_VERTEX   = 0
  SHADER_TYPE_FRAGMENT = 1
```
  - `paramName` is name of uniform in shader
  - `paramValue` string should be in Ogre's material script format (i.e. a vec 3 would be "0.5 0.9 1.0")

