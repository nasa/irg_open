// This shader is an adaptation of Gazebo's camera_noise_gaussian_fs.glsl,
// licensed under the Apache License, Version 2.0. It is modified to apply a
// digital camera simulation model, including our own noise formula that is
// dependent on the pixel brightness from the base image.

// This fragment shader will add Gaussian noise to a rendered image.  It's
// intended to be instantiated via Ogre's Compositor framework so that we're
// operating on a previously rendered image.  We're doing it a shader for
// efficiency: a naive CPU implementation slows camera rates from 30Hz to 10Hz,
// while this GPU implementation has no noticable effect on performance.
//
// We're applying additive amplifier noise, as described at:
// http://en.wikipedia.org/wiki/Image_noise#Amplifier_noise_.28Gaussian_noise.29
// This is uncorrelated Gaussian noise added to each pixel.  For each pixel, we
// want to sample a new value from a Gaussian distribution and add it to each
// channel in the input image.
//
// There isn't (as far as I can tell) a way to generate random values in GLSL.
// The GPU vendors apparently don't implement the noise[1-4]() functions that
// are described in the documentation.  What we do have is a deterministic 
// function that does a decent job of appoximating a uniform distribution 
// on [0,1].  But it requires a 2-D vector as input.
//
// So we're doing something mildly complicated:
//
// 1. On the CPU, before each call to this shader, generate 3 random numbers
// that are uniformly distributed on [0,1.0] and pass them here,
// in the `offsets` parameter.
//
// 2. Each time we need a random number here, add one of the CPU-provided
// offsets to our current pixel coordinates and give the resulting vector to our
// pseudo-random number generator.
// 
// 3. Implement the Box-Muller method to sample from a Gaussian distribution.
// Normally each iteration of this method requires 2 uniform inputs and
// gives 2 Gaussian outputs.  We're using 3 uniform inputs, with the 3rd 
// being used to select randomly between the 2 Gaussian outputs.
//
// 4. Having produced a Gaussian sample, we add this value to each channel of
// the input image.

// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;

// Other parameters are set in C++, via
// Ogre::GpuProgramParameters::setNamedConstant()

// Exposure (shutter time) and gamma curve
uniform float exposure;
uniform float gamma;

// Convert light energy to normalized sensor signal
uniform float energy_conversion;

// Random values sampled on the CPU, which we'll use as offsets into our 2-D
// pseudo-random sampler here.
uniform vec3 offsets;
// Mean of the Gaussian distribution that we want to sample from.
//uniform float mean;
// Standard deviation of the Gaussian distribution that we want to sample from.
//uniform float stddev;

// Read noise standard deviation (in integer pixel value where pixel values are
// in the interval [0, 2^adc_bits - 1] )
uniform float read_noise_std_dev;

// Shot noise coefficient (unitless)
uniform float shot_noise_coeff;

// This is the simplest possible simulation of camera sensor gain (or ISO)
uniform float gain;

// Bit-depth of digital camera output
uniform float adc_bits;


#define PI 3.14159265358979323846264

float rand(vec2 co)
{
  // This one-liner can be found in many places, including:
  // http://stackoverflow.com/questions/4200224/random-noise-functions-for-glsl
  // I can't find any explanation for it, but experimentally it does seem to
  // produce approximately uniformly distributed values in the interval [0,1].
  float r = fract(sin(dot(co.xy, vec2(12.9898,78.233))) * 43758.5453);

  // Make sure that we don't return 0.0
  if(r == 0.0)
    return 0.000000000001;
  else
    return r;
}

vec4 gaussrand(float I, vec2 co, float integer_limit)
{
  // Box-Muller method for sampling from the normal distribution
  // http://en.wikipedia.org/wiki/Normal_distribution#Generating_values_from_normal_distribution
  // This method requires 2 uniform random inputs and produces 2 
  // Gaussian random outputs.  We'll take a 3rd random variable and use it to
  // switch between the two outputs.

  float U, V, R, Z;
  // Add in the CPU-supplied random offsets to generate the 3 random values that
  // we'll use.
  U = rand(co + vec2(offsets.x, offsets.x));
  V = rand(co + vec2(offsets.y, offsets.y));
  R = rand(co + vec2(offsets.z, offsets.z));
  // Switch between the two random outputs.
  if(R < 0.5)
    Z = sqrt(-2.0 * log(U)) * sin(2.0 * PI * V);
  else
    Z = sqrt(-2.0 * log(U)) * cos(2.0 * PI * V);

  // Apply the stddev and mean.
  //Z = Z * stddev + mean;

  // Shot noise standard deviation is some constant times the square root of
  // integer pixel intensity.
  float shot_noise_std_dev_squared = shot_noise_coeff * shot_noise_coeff * I * integer_limit;
  // Because standard deviations here are measured in integers, the final stddev
  // is divided by the integer limit to put it in the floating point interval [0.0, 1.0]
  float stddev = sqrt(read_noise_std_dev * read_noise_std_dev + shot_noise_std_dev_squared) / integer_limit;
  Z = Z * stddev;

  // Return it as a vec4, to be added to the input ("true") color.
  return vec4(Z, Z, Z, 0.0);
}

vec3 downsample(vec3 color, float integer_limit)
{
  vec3 integer_limit_vec = vec3(integer_limit);
  return floor(color * integer_limit_vec) / integer_limit_vec;

  // If you are downsampling to, for example, 5 bits and rendering the final
  // image to an 8-bit framebuffer or texture, you would expect the least
  // significant 3 bits of each fragment to be 0. It doesn't work out this way
  // due to floating-point inaccuracy. If we want to final result to be this
  // perfect, we would add this to the result returned above:
  //
  // + vec3(0.5 / pow(2.0, render_target_bit_depth));
  //
  // This would require the user to pass render_target_bit_depth to this shader
  // as a uniform. We don't yet know if this kind of exactness is necessary, so
  // we haven't done it yet.

  // An alternative way to downsample the texture would be:
  //
  // return color * (integer_limit / render_target_integer_limit);
  //
  // This would cause a smaller range of the final image's possible values to
  // be used. The resulting image would look dark to a human viewer but might
  // be appropriate for computer vision algorithms.
}

void main()
{
  vec4 color = texture2D(RT, gl_TexCoord[0].xy);

  // exposure
  color.rgb *= vec3(exposure);

  // convert light power to sensor signal
  color.rgb *= vec3(energy_conversion);

  // Number of possible values in the final image encoded as integers, or the
  // analog-to-digital limit.
  float integer_limit = pow(2.0, adc_bits);

  // luminance noise
  float gray = dot(color.rgb, vec3(0.299, 0.587, 0.114));
  color.rgb += vec3(gaussrand(gray, gl_TexCoord[0].xy, integer_limit));

  // sensor gain
  color.rgb *= vec3(gain);

  // gamma
  // This should be the very last step so that it causes exaggerated banding
  // artifacts after changing to a lower bit-depth. But that would be difficult
  // to compute and this is a purely cosmetic feature that is not part of real
  // digital cameras, so we will forego that level of accuracy.
  color.rgb = pow(color.rgb, vec3(gamma));

  // downsample to simulate camera's analog-to-digital converter
  gl_FragColor = vec4(downsample(color.rgb, integer_limit), color.a);
}

