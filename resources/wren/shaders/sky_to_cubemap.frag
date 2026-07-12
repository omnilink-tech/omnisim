// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.
//
// Sky-view LUT → cubemap face rendering.
//
// Used in two places:
//   1. Seeds the procedural sky cubemap that
//      ``wr_texture_cubemap_bake_specular_irradiance`` then pre-filters
//      into the PBR IBL map.  Without this step, every PBRAppearance
//      in the scene reflects whatever the cubemap path's fallback
//      gives them (a 2×2 uniform pixel block); with this step, they
//      reflect the actual procedural sky.
//   2. (future) Sensor-camera bakes that need a snapshot of the
//      current sky without re-running the full LUT pipeline per
//      sensor frame.
//
// Renders the same colour sky_apply.frag would render for the cube-
// face pixel's view direction, MINUS the sun disc (the disc is a
// hard delta and ruins the pre-filter integral; the diffuse +
// specular IBL pipeline only cares about the smoothly-varying sky
// background, not the disc).
//
// Driven by ``view`` + ``projection`` matrix uniforms in the same
// per-face convention as the existing IBL bakers in TextureCubeMapBaker.

#version 330 core

precision highp float;

// World-space cube vertex position, supplied by bake_cubemap.vert (the
// vertex shader the existing IBL bakers also pair with — we re-use it
// to keep the projection/view matrix uniform convention identical).
in vec3 worldPosition;

layout(location = 0) out vec4 fragColor;

// Texture slot 0: sky-view LUT (2D).
uniform sampler2D inputTextures[1];
#define skyViewLut inputTextures[0]

uniform vec3 sunIlluminance;

const float PI = 3.14159265358979;

void main() {
  // worldPosition is the cube-face vertex in Webots world-space (ENU,
  // Z-up).  The sky-view LUT and sky_apply.frag both work in a Y-up
  // internal frame, so swap world Z (up) into shader Y (up) before
  // computing azimuth/elevation — same remap as sky_apply.frag line
  // 133.  Without this swap, elevation here is the angle from north,
  // not from horizon: the cube's up face samples the LUT's horizon
  // band, which on Mars at high sun is blue-green, contaminating the
  // baked IBL with a green tint that propagates into every PBR
  // material's diffuse channel.
  vec3 viewDir = normalize(vec3(worldPosition.x, worldPosition.z, -worldPosition.y));
  float elevation = max(0.0, asin(clamp(viewDir.y, -1.0, 1.0)));
  float azimuth = atan(viewDir.x, -viewDir.z);
  if (azimuth < 0.0)
    azimuth += 2.0 * PI;
  vec2 uv = vec2(azimuth / (2.0 * PI), elevation / PI + 0.5);
  vec3 sky = texture(skyViewLut, uv).rgb * sunIlluminance;

  // Full white-balance.  Preserves luminance (IBL keeps its
  // directional brightness — bright near sun, dim in shadow) while
  // pulling each pixel's chromaticity almost all the way to neutral
  // grey.  Real cameras white-balance the sky's chromatic load away
  // by firmware so albedos read true; without this, the Hillaire
  // per-channel scattering math (Rayleigh blue + Mie broadband)
  // integrates over a cube that's chromatically biased, and the
  // pre-filter mip pyramid lands that bias on every PBR surface.
  // 0.95 = 5 % residual atmospheric character preserved, just
  // enough that golden-hour still reads warm and Mars still reads
  // alien-coloured in the *sky*, but materials' own colours
  // dominate everywhere else.
  float luma = max(1e-4, dot(sky, vec3(0.2126, 0.7152, 0.0722)));
  vec3 chroma = sky / luma;
  chroma = mix(chroma, vec3(1.0), 1.00);
  sky = chroma * luma;

  fragColor = vec4(sky, 1.0);
}
