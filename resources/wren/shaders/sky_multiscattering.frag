// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.
//
// Hillaire 2020 — Multiple-Scattering LUT (§5.2).
//
// Two-axis 2D lookup table parameterised on (mu_sun, height):
//   - texUv.x : maps linearly to mu_sun in [-1, 1], the cosine of the
//               angle between the sun direction and the planet zenith
//               at the sample point.
//   - texUv.y : maps linearly to altitude in [0, atmosphereHeightKm].
//
// Each texel stores the per-channel multi-scattered luminance arriving
// at the sample point from a uniform-irradiance sun, normalised by
// sun illuminance.  Hillaire's trick: simulate one extra bounce of
// scattering with an isotropic phase function, accumulate the
// in-scatter coefficient f_ms as well, then close the geometric
// series  L_inf = L_2 + f_ms*L_2 + f_ms^2*L_2 + ... = L_2 / (1 - f_ms)
// to approximate infinite-bounce scattering at almost zero extra cost.
//
// Removing this LUT collapses the sky to single-scattering only.  On
// Earth that means a too-dark horizon at twilight; on Mars the dust
// Mie dominates so the difference is smaller but still visible at
// sunrise/sunset.
//
// Consumes the transmittance LUT via inputTextures[0] (Hillaire §5.1).

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

// Texture slot 0: transmittance LUT (per WREN convention).
uniform sampler2D inputTextures[1];
#define transmittanceLut inputTextures[0]

const int SPHERE_DIRECTION_COUNT = 64;
const int MARCH_STEPS = 32;
const float PI = 3.14159265358979;
const float METRES_PER_KM = 1000.0;
const float ISOTROPIC_PHASE = 1.0 / (4.0 * PI);

float raySphereExit(vec3 origin, vec3 dir, float radius) {
  float b = dot(origin, dir);
  float c = dot(origin, origin) - radius * radius;
  float d = b * b - c;
  if (d < 0.0)
    return -1.0;
  return -b + sqrt(d);
}
float raySphereEnter(vec3 origin, vec3 dir, float radius) {
  float b = dot(origin, dir);
  float c = dot(origin, origin) - radius * radius;
  float d = b * b - c;
  if (d < 0.0)
    return -1.0;
  float t = -b - sqrt(d);
  return (t < 0.0) ? -1.0 : t;
}

// Sample the transmittance LUT for the ray (origin, dir).  Same
// parameterisation as sky_transmittance.frag's main() — keep in sync.
vec3 tLut(vec3 origin, vec3 dir) {
  float r = length(origin);
  vec3 zenith = origin / r;
  float mu = dot(dir, zenith);
  float h = r - bottomRadiusKm;
  vec2 uv = vec2(mu * 0.5 + 0.5, h / (topRadiusKm - bottomRadiusKm));
  return texture(transmittanceLut, uv).rgb;
}

void scatteringValues(vec3 p, out vec3 rayleighSc, out vec3 mieSc, out vec3 extinction) {
  float alt = max(0.0, length(p) - bottomRadiusKm);
  float rayleighDensity = exp(alt * rayleighDensityExpScale);
  float mieDensity = exp(alt * mieDensityExpScale);
  rayleighSc = rayleighScattering * rayleighDensity;
  mieSc = mieScattering * mieDensity;
  extinction = rayleighSc + mieSc + mieAbsorption * mieDensity + absorptionExtinction * rayleighDensity;
}

// Uniformly distribute n points on a unit sphere via the Fibonacci
// spiral.  Standard low-discrepancy sampling — gives much smoother
// results than rejection-sampled random directions at the same count.
vec3 sphericalFibonacci(int i, int n) {
  float phi = (1.0 + sqrt(5.0)) * 0.5;
  float u = (float(i) + 0.5) / float(n);
  float v = fract(float(i) * (phi - 1.0));
  float cosTheta = 1.0 - 2.0 * u;
  float sinTheta = sqrt(max(0.0, 1.0 - cosTheta * cosTheta));
  float phiAngle = 2.0 * PI * v;
  return vec3(sinTheta * cos(phiAngle), cosTheta, sinTheta * sin(phiAngle));
}

void main() {
  float muSun = texUv.x * 2.0 - 1.0;
  float h = texUv.y * (topRadiusKm - bottomRadiusKm);
  vec3 origin = vec3(0.0, bottomRadiusKm + h, 0.0);
  vec3 sunDir = vec3(sqrt(max(0.0, 1.0 - muSun * muSun)), muSun, 0.0);

  vec3 L2Sum = vec3(0.0);
  vec3 fmsSum = vec3(0.0);

  for (int s = 0; s < SPHERE_DIRECTION_COUNT; ++s) {
    vec3 viewDir = sphericalFibonacci(s, SPHERE_DIRECTION_COUNT);

    // March from origin along viewDir to atmosphere exit or planet hit.
    float tExit = raySphereExit(origin, viewDir, topRadiusKm);
    float tGround = raySphereEnter(origin, viewDir, bottomRadiusKm);
    float tEnd = (tGround > 0.0) ? tGround : tExit;
    if (tEnd <= 0.0)
      continue;
    float dt = tEnd / float(MARCH_STEPS);

    vec3 transmittanceFromOrigin = vec3(1.0);
    vec3 L2 = vec3(0.0);
    vec3 fms = vec3(0.0);

    for (int j = 0; j < MARCH_STEPS; ++j) {
      vec3 p = origin + viewDir * (dt * (float(j) + 0.5));
      vec3 rSc, mSc, ext;
      scatteringValues(p, rSc, mSc, ext);
      vec3 stepT = exp(-ext * dt * METRES_PER_KM);

      // Sun transmittance at this sample point.  Zero if the planet
      // occludes the sun.
      vec3 sunT = tLut(p, sunDir);
      if (raySphereEnter(p, sunDir, bottomRadiusKm) > 0.0)
        sunT = vec3(0.0);

      // Single-scatter inscatter coefficient (sun -> here -> view).
      vec3 inScatter = (rSc + mSc) * sunT * ISOTROPIC_PHASE;
      // Analytic per-segment integral of T(s)*inScatter ds with T(s)
      // = exp(-ext*s).  See Hillaire 2020 §5.2.
      vec3 sIntegral = (inScatter - inScatter * stepT) / max(ext, vec3(1e-10));
      L2 += transmittanceFromOrigin * sIntegral;

      // In-scatter coefficient f_ms: same integral but with the
      // scattering coefficient itself (no sun term, no phase).
      vec3 fmsInScatter = (rSc + mSc) * ISOTROPIC_PHASE;
      vec3 fmsIntegral = (fmsInScatter - fmsInScatter * stepT) / max(ext, vec3(1e-10));
      fms += transmittanceFromOrigin * fmsIntegral;

      transmittanceFromOrigin *= stepT;
    }

    // Ground bounce: if the ray hit the planet surface, add an
    // isotropic albedo bounce of the sun light reaching that point.
    if (tGround > 0.0) {
      vec3 hitPos = origin + viewDir * tGround;
      vec3 hitZenith = normalize(hitPos);
      float cosSunAtHit = max(0.0, dot(hitZenith, sunDir));
      vec3 groundT = tLut(hitPos, sunDir);
      if (raySphereEnter(hitPos, sunDir, bottomRadiusKm) > 0.0)
        groundT = vec3(0.0);
      L2 += transmittanceFromOrigin * groundAlbedo * (1.0 / PI) * cosSunAtHit * groundT;
    }

    L2Sum += L2 * (1.0 / float(SPHERE_DIRECTION_COUNT));
    fmsSum += fms * (1.0 / float(SPHERE_DIRECTION_COUNT));
  }

  // Geometric series: assume the multi-scattered light is isotropic
  // and bounces with average coefficient f_ms.
  vec3 multiScatter = L2Sum / max(vec3(1.0) - fmsSum, vec3(1e-10));
  fragColor = vec4(multiScatter, 1.0);
}
