// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.
//
// Hillaire 2020 — Sky-View LUT (§5.3).
//
// Camera-relative 2D lookup table parameterised on (azimuth, elevation):
//   - texUv.x : azimuth around the world-up axis (atan2(x, -z) convention
//               matching the cube-vert reconstruction in sky_apply.frag).
//               Linear, [0, 1] -> [0, 2π].
//   - texUv.y : elevation, linear in [0, 1] -> [-π/2, π/2].  A
//               non-linear horizon-biased mapping (Hillaire §5.3) is a
//               future tuning step — linear is the simpler starting
//               point and the visible difference at our resolutions is
//               small.
//
// Each texel stores the per-channel sky luminance along that view
// direction at the camera's altitude, NORMALISED by sun illuminance.
// sky_apply.frag samples this LUT and multiplies by the actual sun
// illuminance — this way the LUT can be reused across different
// illuminance scales without rebake.
//
// Combines single-scattering with Rayleigh + Cornette-Shanks Mie
// phase functions and multi-scattering sampled from
// `multiScatteringLut` (Hillaire §5.2).
//
// CONVENTION: ``sunDirection`` is the unit vector FROM the camera
// TOWARD the sun (i.e., pointing up at noon).  When viewing the sun,
// dot(viewDir, sunDirection) = +1.  Webots `DirectionalLight.direction`
// is the opposite sign (direction light travels); the C++ side
// (WbWrenAtmosphericSky::setSunDirection) is responsible for handing
// this shader the negated-sign version.

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
uniform float miePhaseG;
uniform vec3  absorptionExtinction;
uniform float groundAlbedo;

uniform vec3  sunDirection;
uniform float cameraHeightKm;

// Texture slot 0: transmittance LUT.  Slot 1: multi-scattering LUT.
uniform sampler2D inputTextures[2];
#define transmittanceLut    inputTextures[0]
#define multiScatteringLut  inputTextures[1]

const int VIEW_MARCH_STEPS = 32;
const float PI = 3.14159265358979;
const float METRES_PER_KM = 1000.0;

float raySphereExit(vec3 o, vec3 d, float r) {
  float b = dot(o, d), c = dot(o, o) - r * r, disc = b * b - c;
  return disc < 0.0 ? -1.0 : -b + sqrt(disc);
}
float raySphereEnter(vec3 o, vec3 d, float r) {
  float b = dot(o, d), c = dot(o, o) - r * r, disc = b * b - c;
  if (disc < 0.0) return -1.0;
  float t = -b - sqrt(disc);
  return t < 0.0 ? -1.0 : t;
}

vec3 tLut(vec3 origin, vec3 dir) {
  float r = length(origin);
  vec3 zenith = origin / r;
  float mu = dot(dir, zenith);
  float h = r - bottomRadiusKm;
  vec2 uv = vec2(mu * 0.5 + 0.5, h / (topRadiusKm - bottomRadiusKm));
  return texture(transmittanceLut, uv).rgb;
}

vec3 msLut(vec3 origin) {
  float r = length(origin);
  vec3 zenith = origin / r;
  float muSun = dot(sunDirection, zenith);
  float h = r - bottomRadiusKm;
  vec2 uv = vec2(muSun * 0.5 + 0.5, h / (topRadiusKm - bottomRadiusKm));
  return texture(multiScatteringLut, uv).rgb;
}

void scatteringValues(vec3 p, out vec3 rSc, out vec3 mSc, out vec3 ext) {
  float alt = max(0.0, length(p) - bottomRadiusKm);
  float rd = exp(alt * rayleighDensityExpScale);
  float md = exp(alt * mieDensityExpScale);
  rSc = rayleighScattering * rd;
  mSc = mieScattering * md;
  ext = rSc + mSc + mieAbsorption * md + absorptionExtinction * rd;
}

float phaseRayleigh(float cosTheta) {
  return (3.0 / (16.0 * PI)) * (1.0 + cosTheta * cosTheta);
}
float phaseMie(float cosTheta, float g) {
  float g2 = g * g;
  float num = (1.0 - g2) * (1.0 + cosTheta * cosTheta);
  float denom = (2.0 + g2) * pow(max(1e-6, 1.0 + g2 - 2.0 * g * cosTheta), 1.5);
  return (3.0 / (8.0 * PI)) * num / denom;
}

void main() {
  float azimuth = texUv.x * 2.0 * PI;
  float elevation = (texUv.y - 0.5) * PI;
  // View direction reconstruction matches sky_apply.frag's atan2.
  // World Y up; -Z forward (VRML camera convention preserved by the
  // rest of the WREN pipeline).
  vec3 viewDir = vec3(cos(elevation) * sin(azimuth),
                      sin(elevation),
                      -cos(elevation) * cos(azimuth));

  // sunDirection is supplied by the C++ side in Webots world space
  // (ENU, Z-up).  viewDir above is reconstructed in the shader's
  // Y-up internal frame, so the raw dot product would be wrong by
  // a 90° axis swap — that's why the IBL bake was reading the LUT
  // as if the sun was always near the horizon, contaminating
  // overhead-zenith samples with horizon-orange and tinting flat
  // ground green on Earth at high sun.  Apply the same swap as
  // sky_apply.frag line 134 so the dot product stays in one frame.
  vec3 sun = normalize(vec3(sunDirection.x, sunDirection.z, -sunDirection.y));

  vec3 origin = vec3(0.0, bottomRadiusKm + cameraHeightKm, 0.0);

  float tExit = raySphereExit(origin, viewDir, topRadiusKm);
  if (tExit <= 0.0) {
    fragColor = vec4(0.0, 0.0, 0.0, 1.0);
    return;
  }
  float tGround = raySphereEnter(origin, viewDir, bottomRadiusKm);
  float tEnd = (tGround > 0.0) ? tGround : tExit;
  float dt = tEnd / float(VIEW_MARCH_STEPS);

  float cosSunView = dot(viewDir, sun);
  float phaseR = phaseRayleigh(cosSunView);
  float phaseM = phaseMie(cosSunView, miePhaseG);

  vec3 transmittanceFromCamera = vec3(1.0);
  vec3 luminance = vec3(0.0);

  for (int j = 0; j < VIEW_MARCH_STEPS; ++j) {
    vec3 p = origin + viewDir * (dt * (float(j) + 0.5));
    vec3 rSc, mSc, ext;
    scatteringValues(p, rSc, mSc, ext);
    vec3 stepT = exp(-ext * dt * METRES_PER_KM);

    vec3 sunT = tLut(p, sun);
    if (raySphereEnter(p, sun, bottomRadiusKm) > 0.0)
      sunT = vec3(0.0);

    // Single-scattering only.  Multi-scattering (Hillaire §5.2) blue-
    // shifts the Mars sky because the multi-scatter LUT integrates
    // sun light through the long atmosphere column where red is
    // already scattered away, leaving blue.  Adding that contribution
    // back to the sky pushed the zenith toward blue and broke parity
    // with docs/developer/atmospheric-sky-preview.html, which the
    // user signed off on.  Single-scatter math here matches the
    // preview's ``sampleSky`` exactly.  The multi-scattering LUT
    // still bakes — kept for potential reuse on Earth where the
    // contribution is more subtle and visually correct.
    vec3 inScatter = sunT * (rSc * phaseR + mSc * phaseM);
    vec3 sIntegral = (inScatter - inScatter * stepT) / max(ext, vec3(1e-10));
    luminance += transmittanceFromCamera * sIntegral;
    transmittanceFromCamera *= stepT;
  }

  // No ground-bounce contribution — preview HTML matches.  The visible
  // ground in-scene is rendered by the ElevationGrid / Solid terrain
  // mesh in front of the skybox; the skybox itself should only carry
  // sky and atmosphere.

  fragColor = vec4(luminance, 1.0);
}
