The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

IRG Gazebo plugins
==================================
IRGCameraNoisePlugin
-------------------
This is a sensor plugin that can only be use within a Gazebo `<sensor type="camera">`
or `<sensor type="multicamera">` tag like this:

```
<sensor type="multicamera" name="my_noisy_camera">
  .
  .
  .
  <plugin name="noise" filename="libIRGCameraNoisePlugin.so" />
</sensor>
```

This noise plugin is based on gazebo's default [noise model](http://gazebosim.org/tutorials?tut=sensor_noise&cat=sensors),
and uses a slightly modified version of that model's shader. It is built as an
Ogre3D Compositor and works by layering noise on top of an image to produce a
new image. Our plugin's noise is affected by the input image, while Gazebo's
default noise is not.

The formula we are using for noise is:

`N = sqrt(0.64 + 0.09 * I)`

where I is the image intensity at a given pixel in the range {0, 4095}. The
first term is read noise, the second is photon shot noise. N is the standard
deviation used to choose a random value along a normal distribution for that pixel.

