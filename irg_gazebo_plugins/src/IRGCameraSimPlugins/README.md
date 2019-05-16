IRG Gazebo plugins
==================================
IRG Camera Sim Plugins
----------------------------------
These plugins use an Ogre compositor to simulate the effects of a digital camera
on your scene rendering. The compositor renders the scene to floating point
texture, applies exposure and gamma, rendering the result to a 16-bit image
which simulates the camera sensor, applies noise and sensor gain, rendering the
result to whatever texture or framebuffer the user has defined in his SDF code.
Gamma is not a property of digital cameras, but it was easy to add and gives
the developer a useful option for rendering human-consumable images in the
Gazebo GUI.

IMPORTANT: For these plugins to work correctly your graphics card must support
frame buffer object (FBO) rendering to `PF_FLOAT32_RGB` and `PF_SHORT_RGB`.
Inspect ~/.gazebo/ogre.log to see if you have the right support.

There is a SensorPlugin version that can be use within a SDF
`<sensor type="camera">` or `<sensor type="multicamera">` element like this:

```
<sensor type="multicamera" name="my_camera">
  .
  .
  .
  <plugin name="noise" filename="libIRGCameraSimSensorPlugin.so">
    <exposure>2.0</exposure>
  </plugin>
</sensor>
```

There is also a VisualPlugin that can be used within a SDF `<visual>` element.
```
<!-- Apply camera sim plugin to GUI camera -->
<model name="gui_camera_sim">
  <static>true</static>
  <link name="link">
    <visual name="visual">
      <geometry>
        <empty/>
      </geometry>
      <plugin name="CameraSim" filename="libIRGCameraSimVisualPlugin.so" >
        <topic_uid>gui</topic_uid>
        <gamma>0.5</gamma>
        <read_noise>0.0</read_noise>
        <shot_noise>0.0</shot_noise>
      </plugin>
    </visual>
  </link>
</model>
```
The visual version of the plugin will affect the GUI camera in the Gazebo GUI. A
GUIPlugin would have been a more appropriate choice, but it had a bug where a
black box was drawn on top of the GUI window. I couldn't defeat this bug and
settled on a VisualPlugin instead.

### Parameters
The plugins can be initialized with user-defined values using the following parameters:
 - `<topic_uid>` - Changes topic from `/gazebo/plugins/camera_sim/<topic>` to `/gazebo/plugins/camera_sim/<topic_uid>/<topic>`.
 - `<exposure>` - Multiply original image by this value. Default = 1.0
 - `<gamma>` - Curve original image by this power. Default = 1.0
 - `<read_noise>` - Read noise coefficient. Default = 0.64
 - `<shot_noise>` - Shot noise coefficient. Default = 0.09
 - `<gain>` - Simulate sensor gain by multiplying 16-bit version of exposed image by this value. Default = 1.0

### Topics
Any parameter other than `topic_uid` can be modified at run-time by sending a
message to a ROS topic of type `std_msgs/Float64`. A command-line example:
```
rostopic pub -1 /gazebo/plugins/camera_sim/exposure std_msgs/Float64 1.5
```
If you have defined your `topic_uid` as `foobar`, the example would be:
```
rostopic pub -1 /gazebo/plugins/camera_sim/foobar/exposure std_msgs/Float64 1.5
```
This allows you to define unique messages for each instance

### Noise
This noise part of the plugin is based on gazebo's default
[noise model](http://gazebosim.org/tutorials?tut=sensor_noise&cat=sensors) and
uses a slightly modified version of that model's shader. It lays noise on top of
the original image to produce a new image. Our plugin's noise is affected by the
input image, while Gazebo's default noise is not.

The formula we are using for noise is:

`N = sqrt(read_noise + shot_noise * I)`

where I is the image intensity at a given pixel in the range {0, 4095}. The
first term is a read noise coefficient (default = 0.64), and the second is a
photon shot noise coefficient (default = 0.09). N is the standard deviation
used to choose a random value along a normal distribution for that pixel.
