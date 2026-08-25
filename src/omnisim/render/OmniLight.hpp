// Copyright 2026 OmniLink
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// OmniLight — OmniSim's baked global illumination.
//
// THE DESIGN PRINCIPLE (the sky-LUT philosophy, scaled to full light transport): trace the
// light COMPLETELY and PHYSICALLY, but only when the light rig or the scene changes — then
// sampling the result costs one 3D-texture read per pixel. No per-frame tracing, no temporal
// accumulation, no denoiser: the runtime cost is a constant regardless of scene complexity.
//
// WHAT IT COMPUTES: an irradiance probe volume. Each probe path-traces the scene (a CPU BVH
// over the actual render triangles) with:
//   - real sky radiance on miss (the caller supplies a sampler over the same scattered
//     atmosphere the dome renders),
//   - transmitted-sun direct lighting at every hit (shadow ray to the sun),
//   - surface albedo from the render materials (baseColor x the texture's linear mean),
//   - emissive surfaces as light sources (the night scene's lanterns and strips bounce),
//   - two bounces (probe -> surface -> surface -> sky/sun).
// The result per probe is an L1 spherical-harmonic irradiance function (4 RGB coefficients),
// stored premultiplied by a validity weight — probes buried inside geometry (detected by
// backface-hit fraction) get weight 0 and trilinear interpolation renormalises around them.
//
// WHAT CONSUMES IT: the lit shader's ambient term. Direct sun stays real-time (shadow maps);
// specular stays SSR + analytic IBL. OmniLight replaces the flat hemisphere AMBIENT with the
// traced field — sky visibility, colour bleed, interior darkening, emissive bounce.
//
// The bake is deterministic (per-probe hashed QMC), multithreaded, and runs on a worker
// thread over a SNAPSHOT of world-space triangles (no live scene pointers — reload-safe).
// Hardware RT is deliberately not required: tracing once per light change puts the bake in
// seconds on a CPU, which meets the design; a GPU/RT bake is a drop-in upgrade later.

#ifndef OMNILIGHT_HPP
#define OMNILIGHT_HPP

#include <atomic>
#include <cstdint>
#include <functional>
#include <vector>

struct OmniLightTriangle {
  float v0[3], v1[3], v2[3];  // world space
  uint32_t material = 0;
};

struct OmniLightMaterial {
  float albedoLin[3] = {0.5f, 0.5f, 0.5f};    // linear diffuse albedo
  float emissiveLin[3] = {0.0f, 0.0f, 0.0f};  // linear emitted radiance
};

// A static local light (PointLight / SpotLight), baked INTO the traced field with real
// occlusion — the cure for the real-time extras' light-through-walls and shadowlessness.
// Layout mirrors the renderer's ExtraLight records (colour premultiplied by intensity, linear).
struct OmniLightLocal {
  float pos[3] = {0, 0, 0};
  float colorLin[3] = {1, 1, 1};
  float radius = 0.0f;              // <= 0: no cutoff
  float atten[3] = {1, 0, 0};       // constant, linear, quadratic
  int type = 1;                     // 1 = point, 2 = spot
  float spotDir[3] = {0, 0, -1};
  float cosCut = 0.0f;              // spot cutOffAngle cosine
  float cosBeam = 1.0f;             // spot beamWidth cosine
};

struct OmniLightParams {
  float sunDirTo[3] = {0.0f, 0.0f, 1.0f};   // TOWARD the sun, normalized
  float sunEnergy[3] = {2.5f, 2.4f, 2.1f};  // linear sun radiance (colour x intensity)
  int raysPerProbe = 256;
  int maxDims[3] = {40, 40, 16};            // probe-grid ceiling per axis
  float minSpacing = 0.45f;                 // metres between probes (floor)
  float boundsPad = 0.5f;                   // metres of margin around the scene AABB
  int threads = 0;                          // 0 = hardware_concurrency - 1
  float outputScale = 1.0f;                 // global multiplier on the baked irradiance (A/B knob)
  // Optional live progress (for a loading indicator): the bake stores probes-completed and the
  // probe total here as it runs. Plain relaxed stores; may be null.
  std::atomic<int> *progressDone = nullptr;
  std::atomic<int> *progressTotal = nullptr;
  // Static local lights to bake (occluded, bounced). The caller crossfades the real-time
  // unshadowed versions OUT as the volume fades in.
  std::vector<OmniLightLocal> locals;
  float localScale = 1.0f;          // brightness calibration vs the retired real-time term
  // Sky radiance sampler (linear RGB for a world-space direction). Called from WORKER threads —
  // must be pure/thread-safe (the caller passes a closure over a prebaked table).
  std::function<void(const float dir[3], float out[3])> skySample;
};

struct OmniLightVolume {
  int dims[3] = {0, 0, 0};
  float origin[3] = {0, 0, 0};   // world position of probe (0,0,0)
  float spacing[3] = {0, 0, 0};  // metres between probes per axis
  // RGBA16F texels for a 3D texture of size dims.x * dims.y * (dims.z * 4): four Z-slabs holding
  // the SH coefficients {L00.rgb, weight}, {L1x.rgb, 0}, {L1y.rgb, 0}, {L1z.rgb, 0}, each
  // premultiplied by the probe's validity weight.
  std::vector<uint16_t> texels;
  int probeCount = 0;
  int validProbes = 0;
  // Traced specular probe: one RGBA16F cubemap (3 mips: size, size/4, size/16) path-traced from
  // cubeCenter — real reflections of the actual scene for metals and glass, parallax-corrected
  // at runtime against the scene AABB. Mip layout: per mip, 6 faces of mipSize^2 texels.
  std::vector<uint16_t> cubeTexels;
  int cubeSize = 0;
  float cubeCenter[3] = {0, 0, 0};
  float aabbMin[3] = {0, 0, 0};
  float aabbMax[3] = {0, 0, 0};
  double bakeSeconds = 0.0;
  double bvhSeconds = 0.0;
  size_t triangleCount = 0;
  bool valid = false;
};

// Bake the volume. `tris`/`materials` are consumed read-only; everything is snapshotted or
// thread-local — safe to run on a worker thread with no live-scene access.
bool omniLightBake(const std::vector<OmniLightTriangle> &tris,
                   const std::vector<OmniLightMaterial> &materials, const OmniLightParams &p,
                   OmniLightVolume &out);

#endif  // OMNILIGHT_HPP
