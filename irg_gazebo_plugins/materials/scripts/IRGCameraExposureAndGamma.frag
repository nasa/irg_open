// The input texture, which is set up by the Ogre Compositor infrastructure.
uniform sampler2D RT;

// Exposure (shutter time) and gamma curve
uniform float exposure;
uniform float gamma;

void main()
{
  vec4 color = texture2D(RT, gl_TexCoord[0].xy);

  color.rgb = color.rgb * vec3(exposure);

  gl_FragColor = vec4(pow(color.rgb, vec3(gamma)), color.a);
}
