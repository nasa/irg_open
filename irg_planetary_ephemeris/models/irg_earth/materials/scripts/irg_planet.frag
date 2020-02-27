#version 130

uniform vec3 sunIntensity;  // lux (visual spectrum) or watts per square meter (some other spectrum)
uniform sampler2D diffuseMap;
uniform float albedoScale;

// vectors in view-space
in vec3 vsNormal;
in vec3 vsVecToSun;

in vec2 uv;

out vec4 outputCol;

void main()
{
  vec3 color = texture2D(diffuseMap, uv).rgb * albedoScale;

  color *= sunIntensity * max(dot(normalize(vsNormal), vsVecToSun), 0.0);

  outputCol = vec4(color, 1.0);
}
