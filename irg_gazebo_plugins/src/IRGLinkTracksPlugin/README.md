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
 - link_name_X - REQUIRED - name of links in Gazebo scenegraph
 - texture_name - REQUIRED - name of texture sampler variable in terrain shader
 - load_image - A previously saved tracks heightmap to load on plugin initialization.
 - track_width - A floating point track width (in pixels) greater than 0.0. Default = 4.5
 - track_depth - The visual "depth" of wheel tracks in the range {0.0, 1.0}. Default = 1.0
 - track_exponent - A floating point exponent (greater than 0.0) to describe the track shape. 1.0 will make the track appear circular. 2.0 will make it a bit more square. Default = 2.0
 - min_dist_thresh - minimum distance the link has to move before drawing a new position in the texture. Default = 0.05

#### ROS params
 - /gazebo/plugins/link_tracks/load_image - overrides `load_image` XML tag if set
 
#### Save a PNG image
At any time the plugin is running you can save the tracks heightmap to a PNG image by publishing a String ROS topic like this:
```
> rostopic pub -1 /wheel_tracks/save_image std_msgs/String "/tmp/tracks.png"
```
The image name **must** have the ".png" suffix. (PNG is the only type of image Gazebo can save.)

#### Enable/Disable drawing of tracks
Drawing of individual tracks can be enabled/disabled at runtime by sending a std_msgs::UIntMultiArray 
message on topic `/gazebo/plugins/link_tracks/link_enable`. The number of elements in the array should match
the number of links specified in SDF. `0` disables drawing, `1` enables drawing. 

For convenience, a global enable/disable is also provided - a std_msgs::Bool type on topic 
`/gazebo/plugins/link_tracks/draw_enable`. This topic is much easier to use at the command line, e.g.
to disable drawing:
```
rostopic pub /gazebo/plugins/link_tracks/draw_enable std_msgs/Bool '{ data: false }'
```



