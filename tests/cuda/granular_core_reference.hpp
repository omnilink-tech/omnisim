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

#ifndef GRANULAR_CORE_REFERENCE_HPP
#define GRANULAR_CORE_REFERENCE_HPP

//
// Description: a dependency-free, CPU-only reference of the granular contact
//   response that ships as the NVRTC kernel string `kPhysicsKernelSrc` in
//   src/omnisim/nodes/OmGranularGroup.cpp. It is the *physics* ground truth:
//   identical force law (penalty normal spring + normal damping + Coulomb-
//   capped tangential friction), identical semi-implicit-Euler integrate, and
//   identical floor/box-wall handling — just with a brute-force O(N^2)
//   neighbour scan instead of the kernel's uniform-grid broadphase.
//
//   The grid is only an acceleration structure: for a given configuration it
//   must surface the same overlapping pairs a brute-force scan does, so the
//   contact *response* is what this header pins down. That response had no
//   headless correctness gate before — only the GPU perf sweep
//   (tests/cuda/bench_results.md) and a "does the .wbt load" test
//   (tests/cuda/test_granular_group_load.py), both of which need a GPU + the
//   full engine. This header needs neither: it compiles with a stock C++17
//   compiler and runs on any box, so granular_core_selfcheck.cpp can assert
//   the pile settles, dissipates energy, and never tunnels the floor.
//
//   Keep this in lock-step with kPhysicsKernelSrc. If the kernel's force law
//   changes, change it here too and re-run the self-check — that is the point.
//   See docs/developer/granular-cuda-plan.md (L8 tracker).
//

#include <cmath>
#include <cstddef>
#include <vector>

namespace omnisim {
namespace granular {

// Per-group simulation constants. Defaults mirror the values the shipping
// kernel hard-codes in OmGranularGroup::DeviceState::stepPhysics (kSpring=300,
// kDamp=0.5, kFriction=0.05, frictionMu=0.15, mass=5 g, wallRestitution=0.45),
// so the reference reproduces the shipped behaviour rather than an idealized
// one. upAxis follows the world convention (0=X, 1=Y, 2=Z); ENU / Z-up is the
// OmniSim default.
struct Params {
  float radius = 0.02f;            // m, uniform per group
  float mass = 0.005f;             // kg, uniform per group
  float kSpring = 300.0f;          // N/m, normal penalty spring
  float kDamp = 0.5f;              // N*s/m, normal-direction damping
  float kFriction = 0.05f;         // N*s/m, tangential viscous coefficient
  float frictionMu = 0.15f;        // Coulomb cap on tangential force
  float wallRestitution = 0.45f;   // floor + box walls
  float gravity = -9.81f;          // m/s^2 along upAxis (signed)
  float boundsHalfWidth = 0.3f;    // m, box half-extent on the two side axes
  float floor = 0.0f;              // m, floor plane position along upAxis
  int upAxis = 2;                  // 0=X 1=Y 2=Z; ENU default is Z
  int substeps = 8;                // inner substeps per outer step (== K_SUBSTEPS)
};

// Particle state, structure-of-arrays to match the kernel's float4 layout
// (pos.xyz + radius, vel.xyz). Radius is uniform so we keep it in Params, not
// per-particle, but the contact math reads it the same way.
struct State {
  std::vector<float> px, py, pz;  // position components
  std::vector<float> vx, vy, vz;  // velocity components
  std::size_t size() const { return px.size(); }
  void resize(std::size_t n) {
    px.assign(n, 0.0f); py.assign(n, 0.0f); pz.assign(n, 0.0f);
    vx.assign(n, 0.0f); vy.assign(n, 0.0f); vz.assign(n, 0.0f);
  }
};

namespace detail {
  inline float comp(int axis, float x, float y, float z) {
    return axis == 0 ? x : (axis == 1 ? y : z);
  }
}  // namespace detail

// One inner substep on the whole system: accumulate per-particle force
// (gravity + every overlapping neighbour's normal/damping/friction), then
// integrate velocity + position with semi-implicit Euler and clamp to the
// floor and the two side walls. Brute force over pairs — O(N^2) — which is
// exactly what we want from a reference: no broadphase to be wrong about.
//
// `fx/fy/fz` is scratch the caller owns so we don't reallocate every substep.
inline void substep(State &s, const Params &p, float dt,
                    std::vector<float> &fx, std::vector<float> &fy, std::vector<float> &fz) {
  const std::size_t n = s.size();
  const float r = p.radius;
  const float minDist = 2.0f * r;
  const float invMass = 1.0f / p.mass;

  // Gravity-as-force on the up axis (force = mass * g; integrate divides it
  // back out — matches the kernel's fx = g / invMass convention).
  const float gForce = p.gravity * p.mass;
  for (std::size_t i = 0; i < n; ++i) {
    fx[i] = (p.upAxis == 0) ? gForce : 0.0f;
    fy[i] = (p.upAxis == 1) ? gForce : 0.0f;
    fz[i] = (p.upAxis == 2) ? gForce : 0.0f;
  }

  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      const float ddx = s.px[i] - s.px[j];
      const float ddy = s.py[i] - s.py[j];
      const float ddz = s.pz[i] - s.pz[j];
      const float distSq = ddx * ddx + ddy * ddy + ddz * ddz;
      if (distSq >= minDist * minDist || distSq <= 0.0f)
        continue;
      const float dist = std::sqrt(distSq) + 1e-6f;
      const float overlap = minDist - dist;
      const float nx = ddx / dist, ny = ddy / dist, nz = ddz / dist;

      // Normal penalty spring (pushes i away from j).
      const float normalSpring = p.kSpring * overlap;
      float ix = normalSpring * nx, iy = normalSpring * ny, iz = normalSpring * nz;

      // Normal damping opposes the approach velocity.
      const float relVx = s.vx[i] - s.vx[j];
      const float relVy = s.vy[i] - s.vy[j];
      const float relVz = s.vz[i] - s.vz[j];
      const float relVn = relVx * nx + relVy * ny + relVz * nz;
      ix -= p.kDamp * relVn * nx;
      iy -= p.kDamp * relVn * ny;
      iz -= p.kDamp * relVn * nz;

      // Tangential viscous friction with a Coulomb cap of mu * normalSpring.
      const float relVtx = relVx - relVn * nx;
      const float relVty = relVy - relVn * ny;
      const float relVtz = relVz - relVn * nz;
      const float relVtMagSq = relVtx * relVtx + relVty * relVty + relVtz * relVtz;
      if (relVtMagSq > 1e-10f) {
        const float relVtMag = std::sqrt(relVtMagSq);
        float ftMag = p.kFriction * relVtMag;
        const float ftCap = p.frictionMu * normalSpring;
        if (ftMag > ftCap) ftMag = ftCap;
        const float ftScale = ftMag / relVtMag;
        ix -= ftScale * relVtx;
        iy -= ftScale * relVty;
        iz -= ftScale * relVtz;
      }

      // Newton's third law: equal and opposite on j.
      fx[i] += ix; fy[i] += iy; fz[i] += iz;
      fx[j] -= ix; fy[j] -= iy; fz[j] -= iz;
    }
  }

  // Integrate + wall clamp.
  const int up = p.upAxis;
  const int sideA = (up + 1) % 3;
  const int sideB = (up + 2) % 3;
  const float halfBox = p.boundsHalfWidth;
  for (std::size_t i = 0; i < n; ++i) {
    s.vx[i] += fx[i] * invMass * dt;
    s.vy[i] += fy[i] * invMass * dt;
    s.vz[i] += fz[i] * invMass * dt;
    s.px[i] += s.vx[i] * dt;
    s.py[i] += s.vy[i] * dt;
    s.pz[i] += s.vz[i] * dt;

    float *pc[3] = {&s.px[i], &s.py[i], &s.pz[i]};
    float *vc[3] = {&s.vx[i], &s.vy[i], &s.vz[i]};

    // Floor: position component along upAxis >= floor + radius.
    if (*pc[up] < p.floor + r) {
      *pc[up] = p.floor + r;
      if (*vc[up] < 0.0f) *vc[up] = -*vc[up] * p.wallRestitution;
    }
    // Two side walls on each non-up axis.
    if (*pc[sideA] < -halfBox + r) { *pc[sideA] = -halfBox + r; if (*vc[sideA] < 0.0f) *vc[sideA] = -*vc[sideA] * p.wallRestitution; }
    if (*pc[sideA] >  halfBox - r) { *pc[sideA] =  halfBox - r; if (*vc[sideA] > 0.0f) *vc[sideA] = -*vc[sideA] * p.wallRestitution; }
    if (*pc[sideB] < -halfBox + r) { *pc[sideB] = -halfBox + r; if (*vc[sideB] < 0.0f) *vc[sideB] = -*vc[sideB] * p.wallRestitution; }
    if (*pc[sideB] >  halfBox - r) { *pc[sideB] =  halfBox - r; if (*vc[sideB] > 0.0f) *vc[sideB] = -*vc[sideB] * p.wallRestitution; }
  }
}

// One outer (physics) step = `substeps` inner substeps, mirroring the kernel's
// dt_inner = dt_outer / K split that keeps the explicit penalty springs stable.
inline void step(State &s, const Params &p, float dtOuter) {
  const float dtInner = dtOuter / static_cast<float>(p.substeps);
  std::vector<float> fx(s.size()), fy(s.size()), fz(s.size());
  for (int k = 0; k < p.substeps; ++k)
    substep(s, p, dtInner, fx, fy, fz);
}

// --- Diagnostics the self-check asserts on -------------------------------

// Largest pairwise penetration depth in the system (0 if no contacts). A
// settled pile keeps this well under one radius — a blown-up solve sends it
// to many radii or NaN.
inline float maxPenetration(const State &s, const Params &p) {
  const std::size_t n = s.size();
  const float minDist = 2.0f * p.radius;
  float worst = 0.0f;
  for (std::size_t i = 0; i < n; ++i)
    for (std::size_t j = i + 1; j < n; ++j) {
      const float ddx = s.px[i] - s.px[j];
      const float ddy = s.py[i] - s.py[j];
      const float ddz = s.pz[i] - s.pz[j];
      const float dist = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
      const float pen = minDist - dist;
      if (pen > worst) worst = pen;
    }
  return worst;
}

// Mean kinetic energy per particle (0.5 * m * v^2). Peaks at the bottom of
// free-fall, then decays toward ~0 as the pile settles.
inline double meanKineticEnergy(const State &s, const Params &p) {
  const std::size_t n = s.size();
  if (n == 0) return 0.0;
  double sum = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double v2 = static_cast<double>(s.vx[i]) * s.vx[i] +
                      static_cast<double>(s.vy[i]) * s.vy[i] +
                      static_cast<double>(s.vz[i]) * s.vz[i];
    sum += 0.5 * p.mass * v2;
  }
  return sum / static_cast<double>(n);
}

// Min / mean / max of the position component along the up axis — the pile
// profile the kernel's telemetry readback reports.
inline void upStats(const State &s, const Params &p, double &mn, double &mean, double &mx) {
  const std::size_t n = s.size();
  const std::vector<float> &up = (p.upAxis == 0) ? s.px : (p.upAxis == 1) ? s.py : s.pz;
  mn = up.empty() ? 0.0 : up[0];
  mx = mn;
  double sum = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double y = up[i];
    if (y < mn) mn = y;
    if (y > mx) mx = y;
    sum += y;
  }
  mean = n ? sum / static_cast<double>(n) : 0.0;
}

inline bool allFinite(const State &s) {
  const std::size_t n = s.size();
  for (std::size_t i = 0; i < n; ++i) {
    if (!std::isfinite(s.px[i]) || !std::isfinite(s.py[i]) || !std::isfinite(s.pz[i]) ||
        !std::isfinite(s.vx[i]) || !std::isfinite(s.vy[i]) || !std::isfinite(s.vz[i]))
      return false;
  }
  return true;
}

}  // namespace granular
}  // namespace omnisim

#endif
