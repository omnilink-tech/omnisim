// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.
// Atmospheric-sky cube vertex shader.  Mirrors skybox.vert: stamps the
// unit-cube vertex against the camera's infinite-far projection while
// passing the world-space direction through to the fragment stage as
// the view-ray.

#version 330 core
layout(location = 0) in vec3 vCoord;

out vec3 texUv;

layout(std140) uniform CameraTransforms {
  mat4 view;
  mat4 projection;
  mat4 infiniteProjection;
}
cameraTransforms;

void main() {
  gl_Position = cameraTransforms.infiniteProjection * mat4(mat3(cameraTransforms.view)) * vec4(vCoord, 1.0);
  texUv = vCoord;
}
