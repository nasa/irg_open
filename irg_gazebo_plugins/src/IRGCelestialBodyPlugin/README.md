IRG Gazebo plugins
==================================
IRG Celestial Body Plugin
-------------------
This is a model plugin that can only be use within a Gazebo `<model>` tag like this:

```
<?xml version="1.0"?>
<sdf version='1.6'>
    
  <model name="irg_sun">
    <!-- This pose will be overridden by IRGCelestialBodyPlugin. -->
    <pose>0 0 0  0 0 0</pose>
    <static>0</static>
    <link name='ellipsoid'>
      <gravity>0</gravity>
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry>
          <sphere>
            <radius>1</radius>
          </sphere>
        </geometry>
        <material>
          <script>
            <uri>model://irg_sun/materials/scripts</uri>
            <name>irg/sun</name>
          </script>
        </material>
      </visual>
      <light name='sun' type='directional'>
        <cast_shadows>1</cast_shadows>
        <direction>1 0 0</direction>
        <diffuse>1 1 1 1</diffuse>
        <specular>1 1 1 1</specular>
      </light>
    </link>
    <plugin name="CelestialBody" filename="libIRGCelestialBodyPlugin.so">
      <frame>sun</frame>
      <radius>696392000</radius>
      <render_distance>160000</render_distance>
      <light_source>true</light_source>
    </plugin>
  </model>

</sdf>
```

#### XML tags
 - `<frame>` - (required) Specify the frame to lookup in the tf-tree and apply to this model.
 - `<radius>` - (required) Specify the actual radius of this body, the same radius used in your model file.
 - `<render_distance>` - (required) Distance from origin to render this body. Use a value within your far clipping plane.
 - `<light_source>` - (optional) If true, ignore ephemeris rotation and rotate so light points toward origin.

#### Explanation
This plugin can be used to position a model in the sky using a given frame. The
given frame is looked up relative to `celestial_body_origin`. This will affect
the rotation of a body and its position relative to Gazebo's origin. You will
probably want something like this in your launch file to integrate this frame
into your scene:
```
  <node name="site_to_celestial_body_origin" pkg="tf" type="static_transform_publisher"
    args="0 0 0 0 0 0 site_frame celestial_body_origin 1000" />
  <node name="celestial_body_origin_to_moon" pkg="tf" type="static_transform_publisher"
    args="0 0 0 1.570796327 0 0 celestial_body_origin moon 1000" />
```
The "moon" frame in that example would be the reference frame in your
`irg_planetary_ephemeris` config file.

If `<light_source>` is set to true, the rotation will be set to affect the light
direction instead of affecting the celestial body's rotation. The direction of
the light source in the .sdf file must be (1, 0, 0). Set this to true for the
sun or other stars.

Models that use this plugin can be found in `irg_open/irg_planetary_ephemeris/models/`

#### To-do
There is no mechanism to set this plugin's params in a launch script or at
runtime. They can only be set where they are hardcoded in model files. It would
be possible to set them with global params in ROS1, but ROS2 has no global
parameter server. It would be good to add this flexibility after ROS2 has some
way of doing this, such as a global or namespaced parameter server.
