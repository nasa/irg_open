The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

IRG Gazebo plugins
==================================
link_tracks_plugin
-------------------
This is a visual plugin intented to be attached to a heightfield visual in Gazebo. 
The heightfield must have a GLSL shader expecting a texture that this plugin will
write to. 

```
<visual name="visual">
  .
  .
  .
  <plugin name="link_tracks" filename="libIRGLinkTracksPlugin.so">
    <link_name_0>mobile_base::front_left_wheel_link</link_name_0>
    <link_name_1>mobile_base::rear_left_wheel_link</link_name_1>
    <link_name_2>mobile_base::front_right_wheel_link</link_name_2>
    <link_name_3>mobile_base::rear_right_wheel_link</link_name_3>
    <texture_name>wheelTracks</texture_name>
    <load_image>/tmp/tracks.png</load_image>
    <track_width>4.5</track_width>
    <track_depth>1.0</track_depth>
    <track_exponent>2.0</track_exponent>
  </plugin>
</visual>
```

#### XML tags
 - link_name_X - `string` - REQUIRED - name of links in Gazebo scenegraph
 - texture_name - `string` - REQUIRED - name of texture sampler variable in terrain shader
 - load_image - `string` - A previously saved tracks heightmap to load on plugin initialization.
 - track_width - `double` - A floating point track width (in pixels) greater than 0.0. Default = 4.5
 - track_depth - `double` - The visual "depth" of wheel tracks in the range {0.0, 1.0}. Default = 1.0
 - track_exponent -`double` - A floating point exponent (greater than 0.0) to describe the track shape. 1.0 will make the track appear circular. 2.0 will make it a bit more square. Default = 2.0
 - min_dist_thresh - `double` - Minimum distance the link has to move before drawing a new position in the texture. Default = 0.05
 - draw_enabled - `bool` - Initial global draw state. Default = true
 - altitude_tracking_enabled - `bool` - track altitude of link above terrain and publish on ROS topic. Default = false
 - altitude_threshold - `double` - default altitude threshold for all links. If link height above terrain is greater than this value, drawing of that track will be disabled. Default = 999.9
 - altitude_threshold_X - `double` - altitude threshold for individual link.

#### ROS params
 - /gazebo/plugins/link_tracks/load_image - overrides `load_image` XML tag if set
 
#### Save a PNG image
At any time the plugin is running you can save the tracks heightmap to a PNG 
image by publishing a String ROS topic like this:
```
> rostopic pub -1 /gazebo/plugins/link_tracks/save_image std_msgs/String "/tmp/tracks.png"
```
The image name **must** have the ".png" suffix. (PNG is the only type of image 
Gazebo can save.)

#### Enable/Disable drawing of tracks
Drawing of individual tracks can be enabled/disabled at runtime by sending a 
std_msgs::UIntMultiArray message on topic `/gazebo/plugins/link_tracks/link_enable`. 
The number of elements in the array should match the number of links specified in 
SDF. `0` disables drawing, `1` enables drawing. **Note** if Altitude Tracking is 
enabled, it will override these values. 

A global enable/disable is also provided - a std_msgs::Bool type on topic 
`/gazebo/plugins/link_tracks/draw_enable`. This topic is much easier to use at 
the command line, e.g. to disable drawing:
```
rostopic pub /gazebo/plugins/link_tracks/draw_enable std_msgs/Bool '{ data: false }'
```

#### Altitude Tracking 
When altitude tracking is enabled, each link specified will publish its altitude 
above the terrain on topic `/gazebo/plugins/link_tracks/altitude/<modified_link_name>` 
where `<modified_link_name>` is the value of `link_name_X` with all colons 
replaced with underscores, e.g. if link_name_0 is `lander::l_scoop`, the 
modified link name would be `lander__l_scoop`. 
The datatype for the topic is [geometry_msgs/Vector3Stamped](https://docs.ros.org/api/geometry_msgs/html/msg/Vector3Stamped.html), with X, Y being 2D world position and Z being altitude above 
heightmap. 
With altitude tracking enabled, tracks will only be drawn into the texture if 
altitude is less than the specified `altitude_threshold`. 




