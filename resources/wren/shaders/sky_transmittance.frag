// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.
//
// Hillaire 2020 — Transmittance LUT (§5.1).
//
// Two-axis 2D lookup table parameterised on (mu, height):
//   - texUv.x : maps linearly to mu in [-1, 1], where mu is the cosine
//               of the angle between the view direction and the
//               planet-zenith at the sample point.  Linear mapping is
//               cheaper than Bruneton's non-linear remapping at a
//               small accuracy cost near the horizon; a future
//               session can switch to the non-linear parameterisation
//               if quality demands it.
//   - texUv.y : maps linearly to altitude in [0, topRadiusKm -
//               bottomRadiusKm].
//
// Each texel stores the per-channel transmittance integrated from the
// sample point along the view direction until the ray exits the
// atmosphere (or hits the planet ground, in which case transmittance
// is zero).  The integral uses the same Rayleigh + Mie + absorption
// composition as the WebGL preview at
// docs/developer/atmospheric-sky-preview.html, so visual results from
// the preview carry over to the in-Webots LUT once consumed by the
// sky-apply pass.

#version 330 core

precision highp float;

in vec2 texUv;

layout(location = 0) out vec4 fragColor;

uniform float bottomRadiusKm;
uniform float topRadiusKm;
uniform vec3  rayleighScattering;
uniform float rayleighDensityExpScale;
uniform vec3  mieScattering;
uniform vec3  mieAbsorption;
uniform float mieDensityExpScale;
uniform vec3  absorptionExtinction;
uniform float groundAlbedo;

const int OPTICAL_DEPTH_STEPS = 40;
const float METRES_PER_KM = 1000.0;

// Returns the positive far root of the (origin, dir, sphere of radius r)
// intersection.  -1 if no intersection.
float raySphereExit(vec3 origin, vec3 dir, float radius) {
  float b = dot(origin, dir);
  float c = dot(origin, origin) - radius * radius;
  float d = b * b - c;
  if (d < 0.0)
    return -1.0;
  return -b + sqrt(d);
}

// Returns the positive near root of the same intersection, or -1 if
// the closer root is behind the origin (or no intersection).
float raySphereEnter(vec3 origin, vec3 dir, float radius) {
  float b = dot(origin, dir);
  float c = dot(origin, origin) - radius * radius;
  float d = b * b - c;
  if (d < 0.0)
    return -1.0;
  float t = -b - sqrt(d);
  return (t < 0.0) ? -1.0 : t;
}

void main() {
  float mu = texUv.x * 2.0 - 1.0;
  float h = texUv.y * (topRadiusKm - bottomRadiusKm);

  // Anchor the sample point on the +Y axis at altitude h.  Direction
  // lies in the XY plane with elevation arccos(mu) from zenith.
  vec3 origin = vec3(0.0, bottomRadiusKm + h, 0.0);
  vec3 dir = vec3(sqrt(max(0.0, 1.0 - mu * mu)), mu, 0.0);

  // Rays heading into the ground carry zero transmittance — the
  // planet blocks the sun.  Mark and bail.
  if (raySphereEnter(origin, dir, bottomRadiusKm) > 0.0) {
    fragColor = vec4(0.0, 0.0, 0.0, 1.0);
    return;
  }

  float tEnd = raySphereExit(origin, dir, topRadiusKm);
  float dt = tEnd / float(OPTICAL_DEPTH_STEPS);

  vec3 tau = vec3(0.0);
  for (int i = 0; i < OPTICAL_DEPTH_STEPS; ++i) {
    vec3 p = origin + dir * (dt * (float(i) + 0.5));
    float altitude = max(0.0, length(p) - bottomRadiusKm);
    float rayleighDensity = exp(altitude * rayleighDensityExpScale);
    float mieDensity = exp(altitude * mieDensityExpScale);
    // Absorption (ozone on Earth, broad CO2 on Mars) is approximated
    // as scaling with Rayleigh density.  This mismatches Earth's real
    // stratospheric ozone tent (peaks at 25 km) but matches the
    // preview shader's approximation so visual continuity is
    // preserved; the full tent will replace this in a tuning pass.
    vec3 extinction = rayleighScattering * rayleighDensity
                    + (mieScattering + mieAbsorption) * mieDensity
                    + absorptionExtinction * rayleighDensity;
    tau += extinction * dt * METRES_PER_KM;
  }

  fragColor = vec4(exp(-tau), 1.0);
}
