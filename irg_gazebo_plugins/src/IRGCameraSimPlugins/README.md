The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

IRG Gazebo plugins
==================================
IRG Camera Sim Plugins
----------------------------------
These plugins use an Ogre compositor to simulate the effects of a digital camera
on your scene rendering. The compositor renders the scene to floating point
texture, applies exposure, applies "energy conversion", applies noise, applies
sensor gain, applies gamma, downsamples to a specified bit-depth, and renders
the result to whatever texture or framebuffer the user has defined in his SDF
code. Gamma is not a property of digital cameras, but it was easy to add and
gives the developer a useful tool for rendering human-consumable images in the
Gazebo GUI.

IMPORTANT: For these plugins to work correctly your graphics card must support
frame buffer object (FBO) rendering to `PF_FLOAT32_RGB`. Inspect
~/.gazebo/ogre.log to see if you have the right support.

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
        <read_noise_std_dev>0.0</read_noise_std_dev>
        <shot_noise_coeff>0.0</shot_noise_coeff>
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
 - `<exposure>` - Exposure time (seconds). Default = 1.0
 - `<energy_conversion>` - Pixel value per lux-second in the interval [0.0, 1.0]. A conversion factor from luminous energy to normalized sensor output. Default = 1.0
 - `<read_noise_std_dev>` - Read noise standard deviation (in integer pixel value where pixel values are in the interval [0, 2<sup>adc_bits</sup> - 1] ). Default = 0.8
 - `<shot_noise_coeff>` - Shot noise coefficient (unitless). Default = 0.3
 - `<gain>` - Simulate sensor gain by multiplying 16-bit version of exposed image by this value. Default = 1.0
 - `<gamma>` - Curve final image by this power. Default = 1.0
 - `<adc_bits>` - Bit depth of camera's analog-to-digital converter. Default = 12

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

### Energy conversion
An estimate of final image values would look something like this (don't take this
equation literally--the real math would be much more complicated):
```
image value = lux or radiant power * scene albedo * exposure time
              * (1 / f-number ^ 2) * lens transmittance * sensor pixel area * quantum efficiency
              * sensor gain
```
This camera simulation plugin does not simulate anything between `f-number`
and `quantum efficiency`, inclusive. If you want, you can estimate this entire
part of the pipeline with the `energy_conversion` parameter. If well chosen,
this will allow you to use realistic exposure times to achieve properly exposed
final images.

`energy_conversion` does not allow for a different energy conversion for each color
channel, which might more realistically account for differences in color filters
or be useful if you are working outside the visible spectrum.

`energy_conversion` defaults to 1.0, effectively disabling this feature.

#### Choosing an `energy_conversion` value
There are many ways you could choose an energy conversion value. Here are some
examples:

**Example 1** Assume you are taking photos on Earth and applying the
[Sunny 16 Rule](https://en.wikipedia.org/wiki/Sunny_16_rule) to choose a shutter
speed (`exposure` value) of 1/100. If you want an incoming light value of 1.0
(as is common in computer graphics) then `energy_conversion` must only compensate
for the shutter speed. Set `energy_conversion` to 100.

**Example 2** Say you want to keep using a shutter speed of 1/100 but now you
want more realistic units for your incoming light value, such as lux. On a sunny
day the surface of Earth receives lux = 120000, and Earth's average albedo = 0.3,
so lux entering the camera = 120000 * 0.3 = 36000. You must compensate to achieve
a properly exposed image. Set `energy_conversion` = 100 / 36000 = 0.0027778.

You can build off these examples or find another method of estimating
`energy_conversion`. Other circumstances may include:
* Using a different f-number or exposure.
* Taking pictures on another world with a different sun lux or surface albedo.
* Using units other than lux, perhaps for light outside the visible spectrum.

### Noise
This noise part of the plugin is based on gazebo's default
[noise model](http://gazebosim.org/tutorials?tut=sensor_noise&cat=sensors) and
uses a slightly modified version of that model's shader. It lays noise on top of
the original image to produce a new image. Our plugin's noise is affected by the
input image, while Gazebo's default noise is not.

The formula we are using for noise is:

```
shot_noise_std_dev = shot_noise_coeff ^ 2 * I
std_dev = sqrt(read_noise_std_dev ^ 2 + shot_noise_std_dev ^ 2)
```

where I is the image intensity at a given pixel in the interval
[0, 2<sup>adc_bits</sup> - 1]. `std_dev` is the standard deviation used to
choose a random Gaussian offset for that pixel. Note that the inputs
`read_noise_std_dev` and `shot_noise_coeff` affect the level of noise to apply
to each pixel in the same image intensity interval, and so they are coupled with
your choice of `adc_bits`. Decreasing `adc_bits` will increase the visual impact
of noise.

### Analog-to-digital converter
The voltages coming out of a digital camera's sensor must be converted to a
digital signal by an ADC. This functionality is simulated in the camera sim's
fragment shader.

When setting up a camera in your SDF code, make sure its image format has a bit
depth greater than or equal to that of the adc_bits you set for this plugin. If
your image format has a smaller bit depth, you will lose desired image detail.

#### Writing final image to texture
Internally, GLSL shaders keep color values in the interval [0.0, 1.0]. When our
simulated ADC converts to, say, 12-bits, it keeps the color in the interval
[0.0, 1.0] but it uses 4096 distinct values in that interval. When this result
is written to a 16-bit render target, it uses 4096 distinct values spread
throughout that interval of 65536 values.

An alternative method would be to write to the *lowest* 4096 values in that
interval of 65536 values. This would require modifying the shader and its inputs.

Both versions of this final image would contain the same amount of information.
We currently don't know if computer vision algorithms would behave the same or
differently when processing both of these.

