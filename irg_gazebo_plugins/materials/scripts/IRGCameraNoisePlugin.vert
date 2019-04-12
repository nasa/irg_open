// This shader is a duplicate of Gazebo's camera_noise_gaussian_vs.glsl,
// licensed under the Apache License, Version 2.0.
void main() 
{
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
  gl_TexCoord[0] = gl_MultiTexCoord0;  
}
