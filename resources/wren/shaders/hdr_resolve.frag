#version 330 core

precision highp float;

in vec2 texUv;

layout(location = 0) out vec4 result;

uniform sampler2D inputTextures[1];

uniform float exposure;

void main() {
  const float gamma = 2.2;
  vec4 sample = texture(inputTextures[0], texUv);

  // Atmospheric-sky pixels are written by sky_apply.frag with alpha=0
  // because they are already tonemapped + sRGB-encoded inside that
  // shader to match docs/developer/atmospheric-sky-preview.html
  // byte-for-byte.  Pass them through unchanged so we don't
  // double-tonemap.  Alpha is reset to 1 on output for downstream
  // compositing consistency.
  if (sample.a < 0.5) {
    result = vec4(sample.rgb, 1.0);
    return;
  }

  // Exposure tone mapping (regular scene pixels).
  vec3 mapped = vec3(1.0) - exp(-sample.rgb * exposure);
  mapped = pow(mapped, vec3(1.0 / gamma));
  result = vec4(mapped, 1.0);
}
