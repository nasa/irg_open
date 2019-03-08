IRG Gazebo plugins
==================================
IRG Shadow Parameters Plugins
----------------------------------

These plugins can be used to override Gazebo's shadow map defaults. There is a
<sensor> plugin and a <visual> plugin, which can be used inside their respective
XML tags. Because Gazebo sets up shadows globally (not per-view or per-sensor),
these plugins can interfere with one another. The <visual> plugin will always
override the <sensor> plugin. Use them as shown:
```
<sensor type="camera" name="myShadowyCamera">
  .
  .
  .
  <plugin name="shadow_params" filename="libIRGShadowParametersSensorPlugin.so">
    <shadow_texture_size>2048</shadow_texture_size>
    <shadow_near>0.01</shadow_near>
    <shadow_far>100</shadow_far>
    <shadow_split_lambda>0.5</shadow_split_lambda>
    <shadow_split_padding>2</shadow_split_padding>
  </plugin>
</sensor>

<visual name="visual">
  .
  .
  .
  <plugin name="shadow_params" filename="libIRGShadowParametersVisualPlugin.so">
    <shadow_texture_size>2048</shadow_texture_size>
    <shadow_near>0.01</shadow_near>
    <shadow_far>100</shadow_far>
    <shadow_split_lambda>0.75</shadow_split_lambda>
    <shadow_split_padding>2</shadow_split_padding>
  </plugin>
</visual>
```
Gazebo uses the Parallel Split Shadow Mapping (PSSM) algorithm, which is the
reason for the <shadow_split_lambda> and <shadow_split_padding> tags.

#### XML tags
Use any or all of these tags. If you don't use one, Gazebo will use its own default value.
 - <shadow_texture_size> - A power-of-two texture size. Three textures of this size will be allocated. Maximum size depends on your GPU. Default = 2048
 - <shadow_near> - A floating point near clip distance for shadows (in meters). Must be > 0.0. Default = 0.01
 - <shadow_far> - A floating point far clip distance for shadows (in meters). Must be > shadow_near. Default = 100.0
 - <shadow_split_lambda> - A floating point value in the range {0.0, 1.0} to adjust how shadow split depths are chosen. 0.0 = linear splits. 1.0 = logarithmic splits. Default = 0.75
 - <shadow_split_padding> - A floating point value (in meters) representing shadow map overlap for covering up problems in the PSSM implementation. Must be >= 0.0. Default = 2.0

