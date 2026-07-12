// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.
//
// Atmospheric sky-apply pass.  Per-pixel single-ray scattering — math
// byte-equivalent to docs/developer/atmospheric-sky-preview.html so the
// in-Webots Mars sky matches the preview the user signed off on.
//
// We deliberately do NOT sample the sky-view LUT here.  The LUT is
// still baked for use by the procedural-sky-cubemap → PBR irradiance
// path (sky_to_cubemap.frag), but for the visible sky we run the full
// per-pixel ray-march so the result is pixel-identical to the
// reference HTML.
//
// Output is sRGB-encoded with alpha=0 so the downstream
// resources/wren/shaders/hdr_resolve.frag skips the second tonemap.
// The preview's filmic + sRGB IS the final tonemap; doing Webots'
// `1 - exp(-x)` curve on top of it would crush the result.

#version 330 core

precision highp float;
#define PI 3.14159265358979
#define METRES_PER_KM 1000.0

in vec3 texUv;  // world-space cube-position from sky_apply.vert

layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec3 fragNormal;

// Atmosphere parameters (mirrors preview HTML 1:1).
uniform float bottomRadiusKm;
uniform float topRadiusKm;
uniform vec3  rayleighScattering;
uniform float rayleighDensityExpScale;  // negative; preview uses 1/scaleHeight implicit
uniform vec3  mieScattering;
uniform vec3  mieAbsorption;
uniform float mieDensityExpScale;
uniform float miePhaseG;
uniform vec3  absorptionExtinction;

uniform vec3  sunDirection;       // TOWARD sun, world space
uniform vec3  sunIlluminance;     // matches preview's per-preset values
uniform float sunDiscCosRadius;
uniform float cameraHeightKm;

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

// Stable 3D hash → [0, 1).
float hash3(vec3 p) {
  return fract(sin(dot(p, vec3(127.1, 311.7, 74.7))) * 43758.5453);
}

// Trilinear value-noise on a 3D lattice — used for Milky Way dust
// lanes and moon-surface maria.
float vnoise3(vec3 p) {
  vec3 i = floor(p);
  vec3 f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  float n000 = hash3(i + vec3(0.0, 0.0, 0.0));
  float n100 = hash3(i + vec3(1.0, 0.0, 0.0));
  float n010 = hash3(i + vec3(0.0, 1.0, 0.0));
  float n110 = hash3(i + vec3(1.0, 1.0, 0.0));
  float n001 = hash3(i + vec3(0.0, 0.0, 1.0));
  float n101 = hash3(i + vec3(1.0, 0.0, 1.0));
  float n011 = hash3(i + vec3(0.0, 1.0, 1.0));
  float n111 = hash3(i + vec3(1.0, 1.0, 1.0));
  return mix(mix(mix(n000, n100, f.x), mix(n010, n110, f.x), f.y),
             mix(mix(n001, n101, f.x), mix(n011, n111, f.x), f.y), f.z);
}

// Three-octave fBm — enough texture to read as "structure" without
// the per-pixel cost spiralling.
float fbm3(vec3 p) {
  float v = 0.0, a = 0.5;
  for (int i = 0; i < 3; ++i) {
    v += a * vnoise3(p);
    p *= 2.13;
    a *= 0.5;
  }
  return v;
}

vec3 opticalDepth(vec3 origin, vec3 dir) {
  float tExit = raySphereExit(origin, dir, topRadiusKm);
  if (tExit < 0.0) return vec3(0.0);
  if (raySphereEnter(origin, dir, bottomRadiusKm) > 0.0) return vec3(1e10);
  const int STEPS = 8;
  float dt = tExit / float(STEPS);
  vec3 sum = vec3(0.0);
  for (int i = 0; i < STEPS; ++i) {
    vec3 p = origin + dir * (dt * (float(i) + 0.5));
    float h = max(0.0, length(p) - bottomRadiusKm);
    float rd = exp(h * rayleighDensityExpScale);
    float md = exp(h * mieDensityExpScale);
    vec3 ext = rayleighScattering * rd + (mieScattering + mieAbsorption) * md + absorptionExtinction * rd;
    sum += ext * dt * METRES_PER_KM;
  }
  return sum;
}

vec3 sampleSky(vec3 origin, vec3 viewDir, vec3 sun) {
  float tExit = raySphereExit(origin, viewDir, topRadiusKm);
  if (tExit <= 0.0) return vec3(0.0);
  float tGround = raySphereEnter(origin, viewDir, bottomRadiusKm);
  float tEnd = (tGround > 0.0) ? min(tExit, tGround) : tExit;

  const int STEPS = 32;
  float dt = tEnd / float(STEPS);
  vec3 rayleighSum = vec3(0.0);
  vec3 transmittance = vec3(1.0);

  float cosSunView = dot(viewDir, sun);
  float pR = (3.0 / (16.0 * PI)) * (1.0 + cosSunView * cosSunView);
  float g = miePhaseG;
  float g2 = g * g;
  float num = (1.0 - g2) * (1.0 + cosSunView * cosSunView);
  float denom = (2.0 + g2) * pow(max(1e-6, 1.0 + g2 - 2.0 * g * cosSunView), 1.5);
  float pM = (3.0 / (8.0 * PI)) * num / denom;

  for (int i = 0; i < STEPS; ++i) {
    vec3 p = origin + viewDir * (dt * (float(i) + 0.5));
    float h = max(0.0, length(p) - bottomRadiusKm);
    float rd = exp(h * rayleighDensityExpScale);
    float md = exp(h * mieDensityExpScale);
    vec3 ext = rayleighScattering * rd + (mieScattering + mieAbsorption) * md + absorptionExtinction * rd;
    vec3 segT = exp(-ext * dt * METRES_PER_KM);

    vec3 sunOD = opticalDepth(p, sun);
    vec3 sunT = exp(-sunOD);

    vec3 dRayleigh = rayleighScattering * rd * pR;
    vec3 dMie      = mieScattering      * md * pM;
    vec3 inscatter = (dRayleigh + dMie) * sunT * dt * METRES_PER_KM;

    rayleighSum += transmittance * inscatter;
    transmittance *= segT;
  }

  vec3 colour = rayleighSum * sunIlluminance;
  if (tGround < 0.0 && cosSunView > sunDiscCosRadius)
    colour += transmittance * sunIlluminance;
  return colour;
}

// Preview's filmic curve.  Hable-style ACES approximation.
vec3 filmic(vec3 x) {
  x *= 0.6;
  return (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);
}

void main() {
  // ENU -> shader Y-up swap.  Webots cube vertex worldPosition is in
  // world ENU coords (X=east, Y=north, Z=up).  The atmospheric model
  // assumes Y=up, -Z=forward (OpenGL standard).  Mapping:
  //   world.x (east)  -> shader.x (right)
  //   world.z (up)    -> shader.y (up)
  //   world.y (north) -> shader.-z (forward into scene)
  // Apply the same swap to the sun direction so view and sun stay
  // in matched frames inside the math.
  vec3 viewDir = normalize(vec3(texUv.x, texUv.z, -texUv.y));
  vec3 sun = normalize(vec3(sunDirection.x, sunDirection.z, -sunDirection.y));
  vec3 origin = vec3(0.0, bottomRadiusKm + cameraHeightKm, 0.0);

  vec3 sky = sampleSky(origin, viewDir, sun);

  // Night-time decorations: procedural stars + moon disc.  Fade in
  // as the sun drops below the horizon (sun.y, Y-up internal).  All
  // contributions are added in linear-HDR space so the downstream
  // filmic tonemap rolls them off the same way it rolls off the
  // procedural sky.
  float nightStrength = smoothstep(0.08, -0.05, sun.y);
  if (nightStrength > 0.001) {
    // ---- Milky Way galactic band ----
    // Subtle glow across a great circle in the sky.  Tuned to read
    // as a faint, structured band — visible only on direct look,
    // not a sky-dominating wash.  Real-life Milky Way at a dark
    // site is barely a few times brighter than ambient airglow.
    vec3 galacticPole = normalize(vec3(0.92, 0.15, 0.36));
    float bandDist = abs(dot(viewDir, galacticPole));    // 0 = on plane
    float bandCore = smoothstep(0.10, 0.0, bandDist);    // ~12° wide
    float bandWide = smoothstep(0.30, 0.05, bandDist);   // soft halo
    // Fade out the lower hemisphere band so it doesn't bleed below
    // the horizon (or below the silhouette of a flat ground).
    float bandAbove = smoothstep(-0.10, 0.15, viewDir.y);
    // fBm with higher frequency for visible dust-lane structure.
    float dust = fbm3(viewDir * 8.0);
    float lanes = smoothstep(0.35, 0.65, fbm3(viewDir * 18.0));
    vec3 mwTint = mix(vec3(0.65, 0.75, 1.00),    // blue-white core
                      vec3(0.92, 0.78, 0.90),    // pink-purple knots
                      smoothstep(0.4, 0.7, dust));
    sky += mwTint * (bandCore * 0.22 + bandWide * 0.06) * (0.30 + 0.70 * dust)
                  * (1.0 - 0.80 * lanes) * bandAbove * nightStrength;

    // ---- Stars ----
    // Stars exist on the upper hemisphere only — fade out as the
    // view direction drops below the horizon so no stars 'leak'
    // beneath the ground when the camera tilts low.
    float starHorizon = smoothstep(-0.02, 0.12, viewDir.y);

    // Lower density than v3 — closer to a real dark-sky count.
    // Spectral colour variation hashed from a secondary channel.
    vec3 cell = floor(viewDir * 600.0);
    float h = hash3(cell);
    float starOn = step(0.9970, h);
    float starMag = pow(fract(h * 31.13), 5.0);
    float hue = fract(h * 7.731);
    vec3 starColor = hue < 0.60
                   ? vec3(0.95, 0.97, 1.05)      // white-blue
                   : (hue < 0.88
                       ? vec3(1.00, 0.92, 0.78)  // yellow main-sequence
                       : vec3(1.05, 0.78, 0.60)); // red giants
    sky += starColor * 16.0 * starOn * (0.20 + 1.10 * starMag)
                     * starHorizon * nightStrength;

    // Sirius / Vega class — very rare, very bright, lower-frequency
    // grid so they're at coarse positions and don't overlap the
    // background pass.
    vec3 cellB = floor(viewDir * 220.0);
    float hb = hash3(cellB + vec3(13.7, 91.3, 47.1));
    float bigOn = step(0.99955, hb);
    sky += vec3(1.0, 0.95, 0.85) * 70.0 * bigOn * starHorizon * nightStrength;

    // ---- Moon ----
    // Bright disc opposite the sun, with clustered maria, fine-
    // grain craters, limb darkening, and a cream tint.  Halo is a
    // subtle blue-grey scatter around the disc.
    vec3 moonDir = -sun;
    if (moonDir.y > 0.0) {
      float cosMoon = dot(viewDir, moonDir);
      // Slightly larger disc and softer edge than v3 — reads more
      // moon-like and less laser-pointer-like.
      float moonDisc = smoothstep(0.99910, 0.99960, cosMoon);
      float moonHalo = smoothstep(0.985, 0.99910, cosMoon);

      // Tangent basis on the moon for stable surface coords.
      vec3 mUp    = normalize(cross(moonDir, vec3(0.0, 0.0, 1.0)));
      vec3 mRight = normalize(cross(moonDir, mUp));
      vec2 moonUv = vec2(dot(viewDir, mRight), dot(viewDir, mUp));

      // Maria: large-scale dark patches.  Threshold tightened so
      // dark areas form distinct blobs (like Mare Imbrium /
      // Tranquillitatis) instead of a uniform speckle.
      float maria = fbm3(vec3(moonUv * 160.0, 1.7));
      float mariaMask = smoothstep(0.48, 0.62, maria);

      // Craters: finer noise, additional small dark dots.
      float craters = smoothstep(0.55, 0.72, vnoise3(vec3(moonUv * 900.0, 7.3)));

      // Limb darkening: radial fall-off from disc centre toward edge.
      // angle ≈ acos(cosMoon) ≈ sqrt(2*(1-cosMoon)) for small angles.
      float r = sqrt(max(0.0, 1.0 - cosMoon) * 2000.0);
      float limb = clamp(1.0 - r * 0.35, 0.6, 1.0);

      vec3 highland = vec3(0.97, 0.94, 0.88);   // cream-grey
      vec3 mare     = vec3(0.45, 0.42, 0.40);   // dark basalt
      vec3 moonColor = mix(highland, mare, mariaMask);
      moonColor *= 1.0 - 0.30 * craters;
      moonColor *= limb;

      sky += moonColor * 18.0 * moonDisc * nightStrength;
      sky += vec3(0.78, 0.82, 0.92) * 0.5 * moonHalo * nightStrength;
    }
  }

  // Exposure is folded into sunIlluminance (Mars is dim by physics;
  // the C++ side picks the scale so 1x exposure looks right).
  vec3 tone = filmic(sky);
  vec3 srgb = pow(tone, vec3(1.0 / 2.2));

  fragNormal = vec3(0.0);
  // Alpha = 0 -> hdr_resolve.frag passes through unchanged.
  fragColor = vec4(srgb, 0.0);
}
