// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.
// Shared fullscreen-quad vertex shader for the Hillaire 2020 atmospheric
// sky pipeline. Used by the three LUT-baking passes (transmittance,
// multiscattering, sky-view) and the final sky-apply pass.

#version 330 core
layout(location = 0) in vec3 vCoord;

out vec2 texUv;

void main() {
  gl_Position = vec4(vCoord.xy, 0.0, 1.0);
  texUv = vCoord.xy * 0.5 + 0.5;
}
