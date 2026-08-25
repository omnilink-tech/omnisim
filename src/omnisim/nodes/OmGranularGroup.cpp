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

#include "OmGranularGroup.hpp"

#include "OmCudaContext.hpp"
#include "OmCudaError.hpp"
#include "OmLog.hpp"
#include "OmMFVector3.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSFNode.hpp"
#include "OmSimulationWorld.hpp"
#include "OmBoundingSphere.hpp"
#include "OmRobot.hpp"
#include "OmSolid.hpp"
#include "OmVector3.hpp"
#include "OmWorld.hpp"
#include "OmWorldInfo.hpp"

#include <QtCore/QObject>

#include <memory>

#ifdef OMNISIM_WITH_CUDA
#include "OmCudaBuffer.hpp"

// We use the CUDA Driver API + NVRTC for kernel compilation/dispatch instead
// of a .cu file because the rest of omnisim-bin is mingw-built; nvcc requires
// MSVC's cl.exe as the host compiler on Windows, and the resulting .obj has
// an incompatible ABI. NVRTC compiles the kernel source string at startup
// into PTX, which the driver loads at runtime — no MSVC dependency in the
// link line.
#include <cuda.h>
#include <nvrtc.h>

#include <algorithm>
#include <cstring>
#include <random>
#include <vector>
#endif

#ifdef OMNISIM_WITH_CUDA
namespace {
  // Combined gravity + sphere-sphere collision + box-wall kernel.
  //
  // Strategy: O(N²) brute force. Each thread reads its particle's pos+vel,
  // accumulates penalty-spring forces against every other particle, then
  // applies gravity and integrates with semi-implicit Euler. Walls (floor +
  // 4 sides of an axis-aligned box) are clamped at the end with velocity
  // reflection-and-damp.
  //
  // Why brute force: at N=1000 the inner loop is 10⁶ pairs/step, well under
  // 1 ms on a sm_86 GPU; uniform-grid broadphase pays off only for much
  // larger N. We can swap it in later behind the same interface.
  //
  // Layout:
  //   pos[i].xyz = position, pos[i].w = radius
  //   vel[i].xyz = velocity, vel[i].w = unused (mass is uniform per group)
  //
  // Numerical care:
  //   - read each particle's pos+vel into registers ONCE (avoids inner-loop
  //     re-read of pos[i] which would race against neighbour updates)
  //   - write back at the end; every kernel launch is a barrier
  //   - eps in the normal computation prevents divide-by-zero on coincident
  //     particles (rare but possible with random seeding)
  // Two-phase per-outer-step:
  //   Phase 1 (computeForces): each particle scans all others, accumulates
  //     gravity + penalty-spring + damping force. Pure read of pos/vel.
  //   Phase 2 (integrate): each particle reads its own force, advances
  //     velocity + position with semi-implicit Euler, clamps to walls.
  //
  // The split is necessary because a one-pass kernel can't see the latest
  // forces from neighbours that haven't been written yet — and a fused
  // kernel that reads pos[j] mid-loop reads stale data anyway. Splitting
  // makes both passes well-defined.
  //
  // We loop the (forces → integrate) pair K_SUBSTEPS times per outer step
  // because explicit-Euler stability for penalty springs needs
  //   dt < sqrt(m / k).
  // With k=300 N/m, m=0.005 kg → dt_max ≈ 13 ms. K=8 → dt_inner=2 ms is
  // safely under that bound and gives enough headroom for collision
  // damping.
  //
  // Broadphase: a uniform grid of side cellSize=2*radius, built once per
  // outer step as a per-cell linked list of particle indices. Each
  // computeForces inner loop walks the 27 neighbour cells of the
  // particle's own cell and tests only those — the work per particle is
  // O(N/numCells × 27) = O(particle density × 27). At our typical
  // density, ≈ 30-100 pair tests per particle vs N pair tests for brute
  // force. Replaces the O(N²) cliff of the prior implementation, lifting
  // the real-time ceiling from N≈2k to well into the tens of thousands.
  constexpr const char *kPhysicsKernelSrc = R"CUDA(
#define K_SUBSTEPS 8

// Build the per-cell head-of-linked-list. cellHead must be pre-zeroed to
// -1 (cudaMemset 0xFF). Each particle atomicExchanges itself onto its
// cell's head and stores the previous head as its `next`. Result: each
// cell points to a chain of every particle currently inside it.
extern "C" __global__
void buildGrid(const float4 *pos, int n,
               int *cellHead, int *cellNext,
               float gridMinX, float gridMinY, float gridMinZ,
               float cellSize,
               int gridX, int gridY, int gridZ) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const float4 p = pos[i];
  int cx = (int)floorf((p.x - gridMinX) / cellSize);
  int cy = (int)floorf((p.y - gridMinY) / cellSize);
  int cz = (int)floorf((p.z - gridMinZ) / cellSize);
  if (cx < 0) cx = 0; else if (cx >= gridX) cx = gridX - 1;
  if (cy < 0) cy = 0; else if (cy >= gridY) cy = gridY - 1;
  if (cz < 0) cz = 0; else if (cz >= gridZ) cz = gridZ - 1;
  const int cid = (cz * gridY + cy) * gridX + cx;
  cellNext[i] = atomicExch(&cellHead[cid], i);
}

extern "C" __global__
void computeForces(const float4 *pos, const float4 *vel, float4 *force, int n,
                   float gx, float gy, float gz, float invMass,
                   float kSpring, float kDamp,
                   float kFriction, float frictionMu,
                   const float4 *colliders, int numColliders,
                   float colliderKSpring, float colliderKDamp,
                   float4 *colliderForces,
                   const int *cellHead, const int *cellNext,
                   float gridMinX, float gridMinY, float gridMinZ,
                   float cellSize,
                   int gridX, int gridY, int gridZ) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;

  const float4 pi = pos[i];
  const float4 vi = vel[i];
  const float ri = pi.w;

  // Gravity-as-force (mass cancels later). The full 3-vector is taken from
  // OmWorldInfo so the kernel works in any coordinate system (ENU/NUE/EUN).
  float fx = gx / invMass;
  float fy = gy / invMass;
  float fz = gz / invMass;

  // Walk the 27 neighbour cells of this particle's cell. Collision tests
  // per particle = average particle density × 27 instead of N — the
  // broadphase that takes the demo from "1000 in real time" to "tens of
  // thousands in real time".
  int cx = (int)floorf((pi.x - gridMinX) / cellSize);
  int cy = (int)floorf((pi.y - gridMinY) / cellSize);
  int cz = (int)floorf((pi.z - gridMinZ) / cellSize);
  if (cx < 0) cx = 0; else if (cx >= gridX) cx = gridX - 1;
  if (cy < 0) cy = 0; else if (cy >= gridY) cy = gridY - 1;
  if (cz < 0) cz = 0; else if (cz >= gridZ) cz = gridZ - 1;

  for (int dz = -1; dz <= 1; ++dz) {
    const int ncz = cz + dz;
    if (ncz < 0 || ncz >= gridZ) continue;
    for (int dy = -1; dy <= 1; ++dy) {
      const int ncy = cy + dy;
      if (ncy < 0 || ncy >= gridY) continue;
      for (int dx = -1; dx <= 1; ++dx) {
        const int ncx = cx + dx;
        if (ncx < 0 || ncx >= gridX) continue;
        const int neighborCid = (ncz * gridY + ncy) * gridX + ncx;
        int j = cellHead[neighborCid];
        while (j >= 0) {
          if (j != i) {
            const float4 pj = pos[j];
            const float ddx = pi.x - pj.x;
            const float ddy = pi.y - pj.y;
            const float ddz = pi.z - pj.z;
            const float distSq = ddx * ddx + ddy * ddy + ddz * ddz;
            const float minDist = ri + pj.w;
            if (distSq < minDist * minDist) {
              const float dist = sqrtf(distSq) + 1e-6f;
              const float overlap = minDist - dist;
              const float nx = ddx / dist;
              const float ny = ddy / dist;
              const float nz = ddz / dist;

              // Normal: penalty spring + damping along the normal.
              const float normalSpring = kSpring * overlap;
              fx += normalSpring * nx;
              fy += normalSpring * ny;
              fz += normalSpring * nz;

              const float4 vj = vel[j];
              const float relVx = vi.x - vj.x;
              const float relVy = vi.y - vj.y;
              const float relVz = vi.z - vj.z;
              const float relVn = relVx * nx + relVy * ny + relVz * nz;
              fx -= kDamp * relVn * nx;
              fy -= kDamp * relVn * ny;
              fz -= kDamp * relVn * nz;

              // Tangential friction with Coulomb cap.
              const float relVtx = relVx - relVn * nx;
              const float relVty = relVy - relVn * ny;
              const float relVtz = relVz - relVn * nz;
              const float relVtMagSq = relVtx * relVtx + relVty * relVty + relVtz * relVtz;
              if (relVtMagSq > 1e-10f) {
                const float relVtMag = sqrtf(relVtMagSq);
                float ftMag = kFriction * relVtMag;
                const float ftCap = frictionMu * normalSpring;
                if (ftMag > ftCap) ftMag = ftCap;
                const float ftScale = ftMag / relVtMag;
                fx -= ftScale * relVtx;
                fy -= ftScale * relVty;
                fz -= ftScale * relVtz;
              }
            }
          }
          j = cellNext[j];
        }
      }
    }
  }

  // One-way coupling: external sphere colliders (typically the bounding
  // spheres of OmSolid robot links). Particles are pushed away when they
  // overlap; reverse force on the collider is discarded for now (full
  // two-way coupling is M3 work — wires accumulated forces back into ODE).
  // colliders[c].xyz = center, colliders[c].w = radius, in world space.
  for (int c = 0; c < numColliders; ++c) {
    const float4 cs = colliders[c];
    const float dx = pi.x - cs.x;
    const float dy = pi.y - cs.y;
    const float dz = pi.z - cs.z;
    const float distSq = dx * dx + dy * dy + dz * dz;
    const float minDist = ri + cs.w;
    if (distSq >= minDist * minDist) continue;

    const float dist = sqrtf(distSq) + 1e-6f;
    // Clamp overlap to one particle radius so a particle that starts
    // deep inside a collider (e.g. the Husky drives onto a particle) gets
    // a bounded push rather than ejection at orbital velocity.
    float overlap = minDist - dist;
    if (overlap > ri) overlap = ri;
    const float nx = dx / dist;
    const float ny = dy / dist;
    const float nz = dz / dist;

    const float fcx = colliderKSpring * overlap * nx;
    const float fcy = colliderKSpring * overlap * ny;
    const float fcz = colliderKSpring * overlap * nz;
    fx += fcx;
    fy += fcy;
    fz += fcz;

    // Damping along the normal: opposes the particle's velocity relative
    // to the (assumed zero-velocity) collider. M3 will replace this with
    // the collider's actual ODE velocity for a correct contact response.
    const float relVn = vi.x * nx + vi.y * ny + vi.z * nz;
    const float fdx = colliderKDamp * relVn * nx;
    const float fdy = colliderKDamp * relVn * ny;
    const float fdz = colliderKDamp * relVn * nz;
    fx -= fdx;
    fy -= fdy;
    fz -= fdz;

    // Two-way coupling: opposite force on the collider, accumulated
    // atomically across all particles touching this collider this
    // substep. Particle felt (fcx + fdx_neg, ...) so the collider feels
    // -(fcx - fdx) by Newton's third law. Summed and divided by K
    // substeps in C++ before being pushed to ODE via dBodyAddForceAtPos.
    if (colliderForces != nullptr) {
      atomicAdd(&colliderForces[c].x, -(fcx - fdx));
      atomicAdd(&colliderForces[c].y, -(fcy - fdy));
      atomicAdd(&colliderForces[c].z, -(fcz - fdz));
    }
  }
  force[i] = make_float4(fx, fy, fz, 0.0f);
}

extern "C" __global__
void integrate(float4 *pos, float4 *vel, const float4 *force, int n,
               float dt, float invMass,
               int upAxis, float floorOffset,
               int sideA, float halfBoxA,
               int sideB, float halfBoxB,
               float wallRestitution) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  float4 p = pos[i];
  float4 v = vel[i];
  const float ri = p.w;
  const float4 f = force[i];

  v.x += f.x * invMass * dt;
  v.y += f.y * invMass * dt;
  v.z += f.z * invMass * dt;
  p.x += v.x * dt;
  p.y += v.y * dt;
  p.z += v.z * dt;

  // Read/write a chosen axis component via a tiny helper so the same kernel
  // handles ENU (Z up), NUE (Y up), EUN (Y up) without specializing.
  float *pComp[3] = {&p.x, &p.y, &p.z};
  float *vComp[3] = {&v.x, &v.y, &v.z};

  // Floor: clamp position component along upAxis to >= floorOffset + ri.
  if (*pComp[upAxis] < floorOffset + ri) {
    *pComp[upAxis] = floorOffset + ri;
    if (*vComp[upAxis] < 0.0f) *vComp[upAxis] = -*vComp[upAxis] * wallRestitution;
  }
  // Side walls along the two non-up axes.
  if (*pComp[sideA] < -halfBoxA + ri) { *pComp[sideA] = -halfBoxA + ri; if (*vComp[sideA] < 0.0f) *vComp[sideA] = -*vComp[sideA] * wallRestitution; }
  if (*pComp[sideA] >  halfBoxA - ri) { *pComp[sideA] =  halfBoxA - ri; if (*vComp[sideA] > 0.0f) *vComp[sideA] = -*vComp[sideA] * wallRestitution; }
  if (*pComp[sideB] < -halfBoxB + ri) { *pComp[sideB] = -halfBoxB + ri; if (*vComp[sideB] < 0.0f) *vComp[sideB] = -*vComp[sideB] * wallRestitution; }
  if (*pComp[sideB] >  halfBoxB - ri) { *pComp[sideB] =  halfBoxB - ri; if (*vComp[sideB] > 0.0f) *vComp[sideB] = -*vComp[sideB] * wallRestitution; }

  pos[i] = p;
  vel[i] = v;
}
)CUDA";

  void logCudaDriverError(const char *what, CUresult err) {
    const char *name = nullptr;
    const char *desc = nullptr;
    cuGetErrorName(err, &name);
    cuGetErrorString(err, &desc);
    const QString msg = QObject::tr("CUDA driver: %1 failed (%2): %3")
                          .arg(what)
                          .arg(name ? name : "?")
                          .arg(desc ? desc : "?");
    OmLog::diagnostic("CUDA_KERNEL_LAUNCH_FAILED", msg);
    OmLog::error(msg);
  }

  void logNvrtcError(const char *what, nvrtcResult err) {
    const QString msg = QObject::tr("NVRTC: %1 failed: %2").arg(what).arg(nvrtcGetErrorString(err));
    OmLog::diagnostic("CUDA_KERNEL_LAUNCH_FAILED", msg);
    OmLog::error(msg);
  }
}  // namespace
#endif

// Per-particle GPU state in M2 = position+radius packed (float4) plus
// velocity (float4 to keep alignment). M3 will extend with sleep flags and
// per-cell hash buckets, but the buffer count is fixed at construction.
class OmGranularGroup::DeviceState {
public:
  explicit DeviceState(int particleCount, double radius, int upAxis,
                       double boundsHalfWidth, double mass) :
    mParticleCount(particleCount),
    mUpAxis(upAxis),
    mBoundsHalfWidth(boundsHalfWidth),
    mMass(mass),
    mRadius(radius) {
#ifdef OMNISIM_WITH_CUDA
    mModule = nullptr;
    mForcesKernel = nullptr;
    mIntegrateKernel = nullptr;
    mStepCount = 0;
    if (particleCount <= 0)
      return;

    // Grid layout for the broadphase. cellSize = 2*radius so any pair of
    // touching spheres has centres in the same or adjacent cell. Side
    // axes span [-halfBox, +halfBox], up axis spans [floor, floor +
    // headroom] where headroom covers the seed drop height + margin.
    mCellSize = static_cast<float>(2.0 * mRadius);
    const float kHeadroomUp = 8.0f;  // covers up to ~5 m drop + slack
    const float sideMin = -static_cast<float>(mBoundsHalfWidth) - mCellSize;
    const float sideSpan = 2.0f * static_cast<float>(mBoundsHalfWidth) + 2.0f * mCellSize;
    const int nSide = std::max(1, static_cast<int>(std::ceil(sideSpan / mCellSize)));
    const int nUp = std::max(1, static_cast<int>(std::ceil(kHeadroomUp / mCellSize)));
    // Map upAxis to the appropriate (x|y|z) min and gridDim slot.
    float gmin[3] = {sideMin, sideMin, sideMin};
    int gdim[3] = {nSide, nSide, nSide};
    gmin[mUpAxis] = 0.0f - mCellSize;  // floor at 0, one cell of margin below
    gdim[mUpAxis] = nUp;
    mGridMinX = gmin[0]; mGridMinY = gmin[1]; mGridMinZ = gmin[2];
    mGridX = gdim[0]; mGridY = gdim[1]; mGridZ = gdim[2];
    const std::size_t numCells = static_cast<std::size_t>(mGridX) *
                                 static_cast<std::size_t>(mGridY) *
                                 static_cast<std::size_t>(mGridZ);

    try {
      mPositions.reset(new OmCudaBuffer<float>(static_cast<std::size_t>(particleCount) * 4));
      mVelocities.reset(new OmCudaBuffer<float>(static_cast<std::size_t>(particleCount) * 4));
      mForces.reset(new OmCudaBuffer<float>(static_cast<std::size_t>(particleCount) * 4));
      // Pre-size the collider + collider-force buffers to a generous max.
      // 256 entries is ~4 KB each and covers any realistic robot link
      // count; per-step we copy only the active prefix.
      mColliders.reset(new OmCudaBuffer<float>(static_cast<std::size_t>(kMaxColliders) * 4));
      mColliderForces.reset(new OmCudaBuffer<float>(static_cast<std::size_t>(kMaxColliders) * 4));
      // Broadphase grid: one int per cell (head pointer, init -1) + one
      // int per particle (next pointer in that cell's chain).
      mCellHead.reset(new OmCudaBuffer<int>(numCells));
      mCellNext.reset(new OmCudaBuffer<int>(static_cast<std::size_t>(particleCount)));
    } catch (const OmCudaError &e) {
      OmLog::diagnostic(e.code(), e.qmessage());
      OmLog::error(e.qmessage());
      mPositions.reset();
      mVelocities.reset();
      mForces.reset();
      mColliders.reset();
      mColliderForces.reset();
      mCellHead.reset();
      mCellNext.reset();
      return;
    }

    seedInitialState(radius, upAxis);
    if (!compileKernel())
      return;
    mAllocated = true;
#endif
  }

  ~DeviceState() {
#ifdef OMNISIM_WITH_CUDA
    if (mModule != nullptr)
      cuModuleUnload(static_cast<CUmodule>(mModule));
#endif
  }

  bool isAllocated() const {
#ifdef OMNISIM_WITH_CUDA
    return mAllocated;
#else
    return false;
#endif
  }

  int particleCount() const { return mParticleCount; }

#ifdef OMNISIM_WITH_CUDA
  // One physics step = one kernel launch. Returns true on success. When
  // outKernelMs is non-null, the GPU-side launch time (without host sync
  // overhead) is written there — costs one cuEventSynchronize so only ask
  // for it on telemetry steps, not every step.
  bool stepPhysics(double dtMs, const float *hostColliders, int numColliders,
                   float *outKernelMs = nullptr,
                   float *outColliderForces = nullptr) {
    if (!mAllocated)
      return false;
    OmCudaContext *ctx = OmCudaContext::instance();
    if (ctx == nullptr)
      return false;

    // Cap and upload collider list. Empty / NULL is fine — kernel skips
    // the inner loop when numColliders == 0.
    if (numColliders > kMaxColliders)
      numColliders = kMaxColliders;
    if (numColliders > 0 && hostColliders != nullptr) {
      try {
        mColliders->copyFromHost(hostColliders, static_cast<std::size_t>(numColliders) * 4);
      } catch (const OmCudaError &e) {
        OmLog::diagnostic(e.code(), e.qmessage());
        OmLog::error(e.qmessage());
        return false;
      }
    }
    // Zero the per-collider reverse-force buffer before each outer step.
    // Substeps atomically accumulate into it; we read the sum back below.
    if (numColliders > 0 && outColliderForces != nullptr) {
      cudaError_t cerr = cudaMemsetAsync(mColliderForces->device(), 0,
                                         static_cast<std::size_t>(numColliders) * 4 * sizeof(float),
                                         ctx->stream());
      if (cerr != cudaSuccess) {
        const QString msg = QObject::tr("cudaMemsetAsync(colliderForces) failed: %1")
                              .arg(cudaGetErrorString(cerr));
        OmLog::diagnostic(OmCudaCodes::MEMCPY_FAILED, msg);
        OmLog::error(msg);
        return false;
      }
    }

    // kSubsteps is a class member now (so substeps() can be called from
    // the outside to scale the readback force). Must match K_SUBSTEPS in
    // the kernel.
    const float dtOuter = static_cast<float>(dtMs * 0.001);
    const float dtInner = dtOuter / static_cast<float>(kSubsteps);
    const float kSpring = 300.0f;       // N/m, stable at dt_inner=2 ms with m=5 g
    const float kDamp = 0.5f;           // N·s/m, normal-direction
    const float kFriction = 0.05f;      // N·s/m, low — particles roll/slide
    const float frictionMu = 0.15f;     // Coulomb cap on tangential force
    const float invMass = 1.0f / static_cast<float>(mMass);
    const float wallRestitution = 0.45f;  // bouncy walls + floor

    // Read gravity + coordinate system from WorldInfo so the kernel works
    // in any of ENU / NUE / EUN. ENU (Z-up) is the OmniSim default.
    float gx = 0.0f, gy = 0.0f, gz = -9.81f;
    int upAxis = 2;
    const OmWorldInfo *wi = OmWorld::instance() ? OmWorld::instance()->worldInfo() : NULL;
    if (wi != NULL) {
      const OmVector3 g = wi->gravityVector();
      gx = static_cast<float>(g.x());
      gy = static_cast<float>(g.y());
      gz = static_cast<float>(g.z());
      const OmVector3 up = wi->upVector();
      if (up.x() != 0.0)
        upAxis = 0;
      else if (up.y() != 0.0)
        upAxis = 1;
      else
        upAxis = 2;
    }
    const int sideA = (upAxis + 1) % 3;
    const int sideB = (upAxis + 2) % 3;
    const float floorOffset = 0.0f;
    // Box half-extents on the two non-up axes — read from the
    // GranularGroup's boundsHalfWidth field so each world can pick its
    // own arena size without rebuilding.
    const float halfBoxA = static_cast<float>(mBoundsHalfWidth);
    const float halfBoxB = static_cast<float>(mBoundsHalfWidth);

    const int n = mParticleCount;
    void *posPtr = mPositions->device();
    void *velPtr = mVelocities->device();
    void *forcePtr = mForces->device();
    void *colPtr = mColliders->device();
    void *colForcePtr = (outColliderForces != nullptr) ? mColliderForces->device() : nullptr;
    void *cellHeadPtr = mCellHead->device();
    void *cellNextPtr = mCellNext->device();
    // Explicit-Euler stability bound: dt < sqrt(m/k). At dt_inner=2 ms,
    // k_max = m/dt² = m/4e-6 — plenty of headroom even at high masses.
    //
    // Tuned against visual feel: at base k=100 N/m (canonical 5 g),
    // contact pushes a ball away gently rather than ejecting it. Scaling
    // linearly with mass means the same overlap gives the same
    // acceleration regardless of particle mass; what makes a heavier
    // pile resist the robot is the INERTIA of the balls, not stronger
    // springs. Damping ratio ≈ 0.7 of critical at every mass
    // (kDamp ≈ sqrt(k·m)·1.4) tames bounciness without sticking.
    const float colliderKSpring = 100.0f * static_cast<float>(mMass) / 0.005f;
    const float colliderKDamp = 1.0f * static_cast<float>(mMass) / 0.005f;
    void *forcesArgs[] = {&posPtr, &velPtr, &forcePtr, const_cast<int *>(&n),
                          const_cast<float *>(&gx), const_cast<float *>(&gy),
                          const_cast<float *>(&gz),
                          const_cast<float *>(&invMass),
                          const_cast<float *>(&kSpring), const_cast<float *>(&kDamp),
                          const_cast<float *>(&kFriction),
                          const_cast<float *>(&frictionMu),
                          &colPtr, const_cast<int *>(&numColliders),
                          const_cast<float *>(&colliderKSpring),
                          const_cast<float *>(&colliderKDamp),
                          &colForcePtr,
                          &cellHeadPtr, &cellNextPtr,
                          &mGridMinX, &mGridMinY, &mGridMinZ,
                          &mCellSize,
                          &mGridX, &mGridY, &mGridZ};
    void *buildGridArgs[] = {&posPtr, const_cast<int *>(&n),
                             &cellHeadPtr, &cellNextPtr,
                             &mGridMinX, &mGridMinY, &mGridMinZ,
                             &mCellSize,
                             &mGridX, &mGridY, &mGridZ};
    void *integrateArgs[] = {&posPtr, &velPtr, &forcePtr, const_cast<int *>(&n),
                             const_cast<float *>(&dtInner),
                             const_cast<float *>(&invMass),
                             const_cast<int *>(&upAxis),
                             const_cast<float *>(&floorOffset),
                             const_cast<int *>(&sideA), const_cast<float *>(&halfBoxA),
                             const_cast<int *>(&sideB), const_cast<float *>(&halfBoxB),
                             const_cast<float *>(&wallRestitution)};
    constexpr unsigned int kBlock = 256;
    const unsigned int grid = (static_cast<unsigned int>(n) + kBlock - 1) / kBlock;

    CUevent evStart = nullptr, evStop = nullptr;
    if (outKernelMs != nullptr) {
      cuEventCreate(&evStart, CU_EVENT_DEFAULT);
      cuEventCreate(&evStop, CU_EVENT_DEFAULT);
      cuEventRecord(evStart, 0);
    }

    CUresult err = CUDA_SUCCESS;

    // Build the broadphase grid ONCE per outer step. Particles barely
    // move within an outer step (≈ a few mm at typical velocities, vs
    // cell sizes of ~10 cm) so the grid stays valid across all substeps.
    // Saves K-1 grid rebuilds per outer step.
    {
      cudaError_t cerr = cudaMemsetAsync(
        mCellHead->device(), 0xFF,
        static_cast<std::size_t>(mGridX) * mGridY * mGridZ * sizeof(int),
        ctx->stream());
      if (cerr != cudaSuccess) {
        const QString msg = QObject::tr("cudaMemsetAsync(cellHead) failed: %1")
                              .arg(cudaGetErrorString(cerr));
        OmLog::diagnostic(OmCudaCodes::MEMCPY_FAILED, msg);
        OmLog::error(msg);
        if (evStart) { cuEventDestroy(evStart); cuEventDestroy(evStop); }
        return false;
      }
      err = cuLaunchKernel(static_cast<CUfunction>(mBuildGridKernel),
                           grid, 1, 1, kBlock, 1, 1,
                           0, nullptr, buildGridArgs, nullptr);
      if (err != CUDA_SUCCESS) {
        logCudaDriverError("cuLaunchKernel(buildGrid)", err);
        mAllocated = false;
        if (evStart) { cuEventDestroy(evStart); cuEventDestroy(evStop); }
        return false;
      }
    }

    for (int s = 0; s < kSubsteps; ++s) {
      err = cuLaunchKernel(static_cast<CUfunction>(mForcesKernel),
                           grid, 1, 1, kBlock, 1, 1,
                           0, nullptr, forcesArgs, nullptr);
      if (err != CUDA_SUCCESS) {
        logCudaDriverError("cuLaunchKernel(computeForces)", err);
        mAllocated = false;
        if (evStart) { cuEventDestroy(evStart); cuEventDestroy(evStop); }
        return false;
      }
      err = cuLaunchKernel(static_cast<CUfunction>(mIntegrateKernel),
                           grid, 1, 1, kBlock, 1, 1,
                           0, nullptr, integrateArgs, nullptr);
      if (err != CUDA_SUCCESS) {
        logCudaDriverError("cuLaunchKernel(integrate)", err);
        mAllocated = false;
        if (evStart) { cuEventDestroy(evStart); cuEventDestroy(evStop); }
        return false;
      }
    }

    if (outKernelMs != nullptr) {
      cuEventRecord(evStop, 0);
      cuEventSynchronize(evStop);
      cuEventElapsedTime(outKernelMs, evStart, evStop);
      cuEventDestroy(evStart);
      cuEventDestroy(evStop);
    }

    // Readback accumulated reverse forces. Sums over kSubsteps substeps;
    // caller divides by kSubsteps to convert to a single per-outer-step
    // force value before handing to ODE.
    if (numColliders > 0 && outColliderForces != nullptr) {
      try {
        mColliderForces->copyToHost(outColliderForces, static_cast<std::size_t>(numColliders) * 4);
      } catch (const OmCudaError &e) {
        OmLog::diagnostic(e.code(), e.qmessage());
        OmLog::error(e.qmessage());
        return false;
      }
    }

    ++mStepCount;
    return true;
  }

  static constexpr int substeps() { return kSubsteps; }

  // Copy raw positions back to host (count*4 floats). Returns false on
  // copy error. The buffer must be at least count*4 floats.
  bool copyPositionsToHost(float *out, std::size_t outCount) const {
    if (!mAllocated)
      return false;
    if (outCount < static_cast<std::size_t>(mParticleCount) * 4)
      return false;
    try {
      mPositions->copyToHost(out, static_cast<std::size_t>(mParticleCount) * 4);
      return true;
    } catch (const OmCudaError &e) {
      OmLog::diagnostic(e.code(), e.qmessage());
      OmLog::error(e.qmessage());
      return false;
    }
  }

  // Copy positions back to host. Returns min/mean/max along the world's
  // up-axis (chosen at construction). Cheap on device→host bandwidth since
  // the whole buffer fits in a few MB at typical counts.
  bool readbackUpStats(double *outMin, double *outMean, double *outMax) const {
    if (!mAllocated)
      return false;
    std::vector<float> host(static_cast<std::size_t>(mParticleCount) * 4, 0.0f);
    try {
      mPositions->copyToHost(host.data(), host.size());
    } catch (const OmCudaError &e) {
      OmLog::diagnostic(e.code(), e.qmessage());
      OmLog::error(e.qmessage());
      return false;
    }
    double mn = host[mUpAxis], mx = host[mUpAxis], sum = 0.0;
    for (int i = 0; i < mParticleCount; ++i) {
      const double y = host[i * 4 + mUpAxis];
      if (y < mn) mn = y;
      if (y > mx) mx = y;
      sum += y;
    }
    *outMin = mn;
    *outMean = sum / mParticleCount;
    *outMax = mx;
    return true;
  }

  unsigned long long stepCount() const { return mStepCount; }

private:
  // Seed positions in a cube above the floor; velocities zero. The cube
  // half-width is sized so that volume_per_particle ≥ (4r)³, leaving plenty
  // of room — random uniform sampling produces no initial overlaps. Drop
  // height set so free-fall takes ~1 s, letting the periodic readback log
  // show clear integration in flight before particles hit the floor.
  //
  // upAxis is the index (0=X, 1=Y, 2=Z) of the world's up direction —
  // matters because seed must place particles ABOVE the floor along that
  // axis, not always Y. ENU (Z-up) is the OmniSim default.
  void seedInitialState(double radius, int upAxis) {
    std::mt19937 rng(0xC0FFEEu);
    // half-width = max(0.5 m, cbrt(N · (4·r)³ / 8))
    const double targetVolPerParticle = std::pow(4.0 * radius, 3.0);
    const double cubeSide = std::cbrt(static_cast<double>(mParticleCount) * targetVolPerParticle);
    const float spread = static_cast<float>(std::max(0.5, cubeSide * 0.5));
    const float baseUp = 1.5f;  // drop height; ~0.55 s of free-fall
    std::uniform_real_distribution<float> uSide(-spread, spread);
    std::uniform_real_distribution<float> uUp(0.0f, spread);
    std::vector<float> hostPos(static_cast<std::size_t>(mParticleCount) * 4, 0.0f);
    std::vector<float> hostVel(static_cast<std::size_t>(mParticleCount) * 4, 0.0f);
    const int sideA = (upAxis + 1) % 3;
    const int sideB = (upAxis + 2) % 3;
    for (int i = 0; i < mParticleCount; ++i) {
      float xyz[3] = {0.0f, 0.0f, 0.0f};
      xyz[sideA] = uSide(rng);
      xyz[sideB] = uSide(rng);
      xyz[upAxis] = baseUp + uUp(rng);
      hostPos[i * 4 + 0] = xyz[0];
      hostPos[i * 4 + 1] = xyz[1];
      hostPos[i * 4 + 2] = xyz[2];
      hostPos[i * 4 + 3] = static_cast<float>(radius);
    }
    try {
      mPositions->copyFromHost(hostPos.data(), hostPos.size());
      mVelocities->copyFromHost(hostVel.data(), hostVel.size());
    } catch (const OmCudaError &e) {
      OmLog::diagnostic(e.code(), e.qmessage());
      OmLog::error(e.qmessage());
      mPositions.reset();
      mVelocities.reset();
    }
  }

  // NVRTC-compile the kernel, load PTX into a CUmodule, look up the entry.
  bool compileKernel() {
    nvrtcProgram prog = nullptr;
    nvrtcResult ne = nvrtcCreateProgram(&prog, kPhysicsKernelSrc, "granular_physics.cu", 0, nullptr, nullptr);
    if (ne != NVRTC_SUCCESS) {
      logNvrtcError("nvrtcCreateProgram", ne);
      return false;
    }
    // Target the SM list pinned by the plan: sm_75..sm_90. NVRTC accepts a
    // single --gpu-architecture; pick sm_75 as the lowest common denominator
    // — the driver JITs to the actual device on first launch.
    const char *opts[] = {"--gpu-architecture=compute_75"};
    ne = nvrtcCompileProgram(prog, 1, opts);
    if (ne != NVRTC_SUCCESS) {
      std::size_t logSize = 0;
      nvrtcGetProgramLogSize(prog, &logSize);
      std::vector<char> compileLog(logSize + 1, '\0');
      nvrtcGetProgramLog(prog, compileLog.data());
      OmLog::error(QString("NVRTC compile log:\n%1").arg(QString::fromUtf8(compileLog.data())));
      logNvrtcError("nvrtcCompileProgram", ne);
      nvrtcDestroyProgram(&prog);
      return false;
    }
    std::size_t ptxSize = 0;
    nvrtcGetPTXSize(prog, &ptxSize);
    std::vector<char> ptx(ptxSize, '\0');
    nvrtcGetPTX(prog, ptx.data());
    nvrtcDestroyProgram(&prog);

    // Initialize the driver API and ensure a primary context exists on the
    // device OmCudaContext picked. cuInit is idempotent.
    CUresult ce = cuInit(0);
    if (ce != CUDA_SUCCESS) {
      logCudaDriverError("cuInit", ce);
      return false;
    }
    CUdevice dev = 0;
    ce = cuDeviceGet(&dev, OmCudaContext::instance()->deviceId());
    if (ce != CUDA_SUCCESS) {
      logCudaDriverError("cuDeviceGet", ce);
      return false;
    }
    CUcontext ctx = nullptr;
    ce = cuDevicePrimaryCtxRetain(&ctx, dev);
    if (ce != CUDA_SUCCESS) {
      logCudaDriverError("cuDevicePrimaryCtxRetain", ce);
      return false;
    }
    cuCtxSetCurrent(ctx);

    CUmodule module = nullptr;
    ce = cuModuleLoadDataEx(&module, ptx.data(), 0, nullptr, nullptr);
    if (ce != CUDA_SUCCESS) {
      logCudaDriverError("cuModuleLoadDataEx", ce);
      return false;
    }
    CUfunction func = nullptr;
    ce = cuModuleGetFunction(&func, module, "computeForces");
    if (ce != CUDA_SUCCESS) {
      logCudaDriverError("cuModuleGetFunction(computeForces)", ce);
      cuModuleUnload(module);
      return false;
    }
    CUfunction integ = nullptr;
    ce = cuModuleGetFunction(&integ, module, "integrate");
    if (ce != CUDA_SUCCESS) {
      logCudaDriverError("cuModuleGetFunction(integrate)", ce);
      cuModuleUnload(module);
      return false;
    }
    CUfunction buildGridFn = nullptr;
    ce = cuModuleGetFunction(&buildGridFn, module, "buildGrid");
    if (ce != CUDA_SUCCESS) {
      logCudaDriverError("cuModuleGetFunction(buildGrid)", ce);
      cuModuleUnload(module);
      return false;
    }
    mModule = module;
    mForcesKernel = func;
    mIntegrateKernel = integ;
    mBuildGridKernel = buildGridFn;
    return true;
  }
#endif

  int mParticleCount;
  int mUpAxis;
  double mBoundsHalfWidth;
  double mMass;
  double mRadius;
#ifdef OMNISIM_WITH_CUDA
  std::unique_ptr<OmCudaBuffer<float>> mPositions;
  std::unique_ptr<OmCudaBuffer<float>> mVelocities;
  std::unique_ptr<OmCudaBuffer<float>> mForces;
  std::unique_ptr<OmCudaBuffer<float>> mColliders;
  std::unique_ptr<OmCudaBuffer<float>> mColliderForces;
  // Uniform-grid broadphase: cellHead[cid] -> index of first particle in
  // cell cid (or -1 if empty); cellNext[i] -> index of next particle in
  // i's cell. Built fresh once per outer step.
  std::unique_ptr<OmCudaBuffer<int>> mCellHead;
  std::unique_ptr<OmCudaBuffer<int>> mCellNext;
  int mGridX = 0, mGridY = 0, mGridZ = 0;
  float mGridMinX = 0.0f, mGridMinY = 0.0f, mGridMinZ = 0.0f;
  float mCellSize = 0.0f;
  void *mBuildGridKernel = nullptr;
  static constexpr int kMaxColliders = 256;
  static constexpr int kSubsteps = 8;
  void *mModule;            // CUmodule, opaque so we don't leak <cuda.h>
  void *mForcesKernel;      // CUfunction for computeForces
  void *mIntegrateKernel;   // CUfunction for integrate
  bool mAllocated = false;
  unsigned long long mStepCount;
#endif
};

void OmGranularGroup::init() {
  mRadius = findSFDouble("radius");
  mCount = findSFInt("count");
  mInitialPositions = findMFVector3("initialPositions");
  mContactMaterial = findSFNode("contactMaterial");
  mBoundsHalfWidth = findSFDouble("boundsHalfWidth");
  mMass = findSFDouble("mass");
  mDeviceState = NULL;
}

// THE REGISTRY (P9). File-static rather than a class member so nothing has to be
// constructed to ask "does this world have particles?" - anyGranularGroups() is an
// empty-test on an already-existing vector. Membership is maintained by the ctor/dtor
// pair, so it cannot drift from the set of live nodes the way a lazily-populated list
// would; a node that is parsed and destroyed without ever finalizing joins and leaves
// without anyone noticing, which is exactly the behaviour a renderer wants.
static std::vector<OmGranularGroup *> gLiveGranularGroups;

bool OmGranularGroup::anyGranularGroups() {
  return !gLiveGranularGroups.empty();
}

const std::vector<OmGranularGroup *> &OmGranularGroup::liveGroups() {
  return gLiveGranularGroups;
}

OmGranularGroup::OmGranularGroup(OmTokenizer *tokenizer) : OmBaseNode("GranularGroup", tokenizer) {
  init();
  gLiveGranularGroups.push_back(this);
}

OmGranularGroup::OmGranularGroup(const OmGranularGroup &other) : OmBaseNode(other) {
  init();
  gLiveGranularGroups.push_back(this);
}

OmGranularGroup::OmGranularGroup(const OmNode &other) : OmBaseNode(other) {
  init();
  gLiveGranularGroups.push_back(this);
}

OmGranularGroup::~OmGranularGroup() {
  for (std::size_t i = 0; i < gLiveGranularGroups.size(); ++i)
    if (gLiveGranularGroups[i] == this) {
      gLiveGranularGroups.erase(gLiveGranularGroups.begin() + i);
      break;
    }
  delete mDeviceState;
  mDeviceState = NULL;
}

bool OmGranularGroup::wgpuParticles(const float *&xyzrOut, int &countOut) const {
#ifdef OMNISIM_WITH_CUDA
  if (mDeviceState == NULL || !mDeviceState->isAllocated())
    return false;  // no live GPU state -> decline (see the header: NOT WREN's origin pile)
  const int n = mDeviceState->particleCount();
  if (n <= 0 || mHostPositionBuffer.size() < static_cast<std::size_t>(n) * 4)
    return false;
  xyzrOut = mHostPositionBuffer.data();
  countOut = n;
  return true;
#else
  (void)xyzrOut;
  (void)countOut;
  return false;  // CUDA-off build: the node is inert, so it draws nothing
#endif
}

double OmGranularGroup::radius() const {
  return mRadius ? mRadius->value() : 0.0;
}

int OmGranularGroup::count() const {
  return mCount ? mCount->value() : 0;
}

double OmGranularGroup::boundsHalfWidth() const {
  return mBoundsHalfWidth ? mBoundsHalfWidth->value() : 1.0;
}

double OmGranularGroup::mass() const {
  return mMass ? mMass->value() : 0.005;
}

void OmGranularGroup::preFinalize() {
  OmBaseNode::preFinalize();

  if (mRadius && mRadius->value() <= 0.0) {
    parsingWarn(tr("'radius' must be strictly positive; got %1.").arg(mRadius->value()));
    mRadius->setValue(0.05);
  }
  if (mCount && mCount->value() < 0) {
    parsingWarn(tr("'count' must be non-negative; got %1.").arg(mCount->value()));
    mCount->setValue(0);
  }
}

void OmGranularGroup::postFinalize() {
  OmBaseNode::postFinalize();
  allocateDeviceBufferIfPossible();

#ifdef OMNISIM_WITH_CUDA
  if (mDeviceState != NULL && mDeviceState->isAllocated()) {
    OmSimulationWorld *world = OmSimulationWorld::instance();
    if (world != NULL)
      connect(world, &OmSimulationWorld::physicsStepStarted, this,
              &OmGranularGroup::onPhysicsStepStarted);
  }
#endif
}

void OmGranularGroup::onPhysicsStepStarted() {
#ifdef OMNISIM_WITH_CUDA
  if (mDeviceState == NULL || !mDeviceState->isAllocated())
    return;
  const double dtMs = OmWorld::instance()->basicTimeStep();

  // Gather external sphere colliders for two-way coupling. Walks every
  // OmSolid in the world (recursively), skips self / ground / arena
  // walls / enormous bounding spheres / colliders without dynamic ODE
  // bodies, and packs (cx, cy, cz, r) into a host buffer the kernel
  // consumes; the parallel solids vector is used for reverse-force
  // application (Newton's 3rd law: particles push the robot back).
  std::vector<float> hostColliders;
  std::vector<OmSolid *> colliderSolids;
  collectColliders(hostColliders, colliderSolids);
  const int numColliders = static_cast<int>(hostColliders.size() / 4);

  // Log collider list once at startup so we can see what's being picked up.
  static bool loggedOnce = false;
  if (!loggedOnce && mDeviceState->stepCount() == 1) {
    loggedOnce = true;
    OmLog::info(tr("GranularGroup: found %1 colliders for one-way coupling.")
                  .arg(numColliders));
    for (int i = 0; i < numColliders && i < 12; ++i) {
      OmLog::info(tr("  collider[%1]: center=(%2, %3, %4) radius=%5")
                    .arg(i)
                    .arg(hostColliders[i * 4 + 0], 0, 'f', 3)
                    .arg(hostColliders[i * 4 + 1], 0, 'f', 3)
                    .arg(hostColliders[i * 4 + 2], 0, 'f', 3)
                    .arg(hostColliders[i * 4 + 3], 0, 'f', 3));
    }
  }

  // Periodic readback + kernel timing. Most steps just launch and return —
  // every kReadbackInterval steps we additionally time the launch (one
  // cuEventSynchronize per kernel) and copy positions back for telemetry.
  constexpr unsigned long long kReadbackInterval = 16;
  const bool isReadback = (mDeviceState->stepCount() + 1) % kReadbackInterval == 0;
  float kernelMs = 0.0f;
  std::vector<float> colliderForces(numColliders > 0 ? numColliders * 4 : 0, 0.0f);
  mDeviceState->stepPhysics(dtMs,
                            numColliders > 0 ? hostColliders.data() : nullptr,
                            numColliders,
                            isReadback ? &kernelMs : nullptr,
                            numColliders > 0 ? colliderForces.data() : nullptr);

  // Apply accumulated reverse forces back to ODE — this is the "particles
  // push the robot" half of two-way coupling. Forces summed across all
  // K substeps; divide by K to convert to a per-outer-step force value
  // that ODE's dWorldStep will integrate over the next dt.
  if (numColliders > 0) {
    const double invSubsteps = 1.0 / static_cast<double>(DeviceState::substeps());
    for (int c = 0; c < numColliders; ++c) {
      const double fx = static_cast<double>(colliderForces[c * 4 + 0]) * invSubsteps;
      const double fy = static_cast<double>(colliderForces[c * 4 + 1]) * invSubsteps;
      const double fz = static_cast<double>(colliderForces[c * 4 + 2]) * invSubsteps;
      if (fx == 0.0 && fy == 0.0 && fz == 0.0)
        continue;
      OmSolid *solid = colliderSolids[c];
      // Wake the body explicitly — ODE doesn't auto-wake on force apply.
      solid->awake();
      // Apply at the bounding-sphere centre (close to the contact point;
      // exact contact integration is a finer-grained M3 follow-up).
      const float *col = &hostColliders[c * 4];
      solid->addForceAtPosition(OmVector3(fx, fy, fz),
                                OmVector3(col[0], col[1], col[2]));
    }
  }

  if (isReadback) {
    double mn = 0.0, mean = 0.0, mx = 0.0;
    if (mDeviceState->readbackUpStats(&mn, &mean, &mx)) {
      OmLog::info(tr("GranularGroup step %1: up min=%2 mean=%3 max=%4 "
                     "(n=%5, kernel %6 ms)")
                    .arg(static_cast<qulonglong>(mDeviceState->stepCount()))
                    .arg(mn, 0, 'f', 4)
                    .arg(mean, 0, 'f', 4)
                    .arg(mx, 0, 'f', 4)
                    .arg(mDeviceState->particleCount())
                    .arg(static_cast<double>(kernelMs), 0, 'f', 3));
    }
  }

  // Per-step rendering: copy positions back into the host buffer the wgpu
  // collector reads. M2 host-readback path — replaced in M1 by interop.
  updateRenderingFromHost();
#endif
}

void OmGranularGroup::collectColliders(std::vector<float> &out,
                                       std::vector<OmSolid *> &outSolids) const {
  out.clear();
  outSolids.clear();
  OmWorld *world = OmWorld::instance();
  if (world == NULL)
    return;
  // Recursive walk gives us every nested Solid (so a URDFRobot's per-link
  // children — wheels, chassis, supports — each show up as separate
  // colliders rather than collapsing into one giant sphere).
  const QList<OmSolid *> solids = world->findSolids(false);

  // Quick reject: skip absurdly large bounding spheres. A perimeter wall
  // bounds a thin slab (a few cm thick × full arena length) → its
  // bounding sphere has radius ~ half the wall length, which is huge and
  // would push every particle inward every step. Robot links are
  // typically << half-arena-width; we cap at half the bounded box's
  // half-width, so anything wall-sized gets dropped.
  const double maxAcceptableRadius = boundsHalfWidth() * 0.5;

  // One-shot diagnostic so we can see what's being picked up vs rejected.
  static bool loggedRaw = false;
  if (!loggedRaw) {
    loggedRaw = true;
    OmLog::info(tr("collectColliders: findSolids returned %1 candidate(s); "
                   "max radius filter = %2 m.")
                  .arg(solids.size())
                  .arg(maxAcceptableRadius, 0, 'f', 3));
    int idx = 0;
    for (OmSolid *solid : solids) {
      if (idx++ >= 16) break;
      const QString name = solid ? solid->name() : QStringLiteral("(null)");
      OmBoundingSphere *bs = solid ? solid->boundingSphere() : NULL;
      if (bs == NULL) {
        OmLog::info(tr("  cand[%1] '%2': no boundingSphere").arg(idx - 1).arg(name));
        continue;
      }
      bs->recomputeIfNeeded();
      OmVector3 c;
      double r = 0.0;
      bs->computeSphereInGlobalCoordinates(c, r);
      OmLog::info(tr("  cand[%1] '%2': center=(%3, %4, %5) radius=%6 %7")
                    .arg(idx - 1)
                    .arg(name)
                    .arg(c.x(), 0, 'f', 3)
                    .arg(c.y(), 0, 'f', 3)
                    .arg(c.z(), 0, 'f', 3)
                    .arg(r, 0, 'f', 3)
                    .arg(r > maxAcceptableRadius ? "REJECTED (too big)" :
                         r <= 0.0 ? "REJECTED (zero/neg)" : "kept"));
    }
  }

  for (OmSolid *solid : solids) {
    if (solid == NULL)
      continue;
    OmBoundingSphere *bs = solid->boundingSphere();
    if (bs == NULL)
      continue;
    // computeSphereInGlobalCoordinates is `const` and skips the dirty-bit
    // recompute that scaledRadius()/center() do — so we have to flush
    // ourselves or every call returns stale (radius=0) data.
    bs->recomputeIfNeeded();
    OmVector3 center;
    double radius = 0.0;
    bs->computeSphereInGlobalCoordinates(center, radius);
    if (radius <= 0.0 || radius > maxAcceptableRadius)
      continue;
    // Skip colliders without a dynamic ODE body. Use body() not
    // bodyMerger() because the latter asserts on static solids that have
    // neither a kinematic flag nor a solid-merger (the RectangleArena
    // walls hit this — body() just returns NULL for them).
    if (solid->body() == NULL)
      continue;
    // Skip the OmRobot root: its bounding sphere encompasses the WHOLE
    // robot (chassis + every wheel), giving a "force field" effect that
    // pushes balls away before they reach any visible link. The robot's
    // child Solids (wheels, bumpers, top-chassis) are still in `solids`
    // and serve as fine-grained colliders.
    if (dynamic_cast<OmRobot *>(solid) != NULL)
      continue;
    // Shrink the bounding sphere a touch — most links are blocky (wheel
    // tyre, flat chassis) so their bounding sphere extends ~15-20%
    // beyond the visible surface. Pulling the contact in tightens the
    // visible "ball stops at the wheel surface" feel without exposing
    // the actual collision geometry to the kernel.
    const double effectiveRadius = radius * 0.80;
    out.push_back(static_cast<float>(center.x()));
    out.push_back(static_cast<float>(center.y()));
    out.push_back(static_cast<float>(center.z()));
    out.push_back(static_cast<float>(effectiveRadius));
    outSolids.push_back(solid);
  }
}

void OmGranularGroup::createWrenObjects() {
  OmBaseNode::createWrenObjects();

  const int n = count();
  if (n <= 0)
    return;

  // D1.4: the per-particle WREN renderable/transform tree is gone; the wgpu
  // renderer draws from wgpuParticles(). Pre-size the readback buffer so
  // per-step rendering doesn't allocate.
  mHostPositionBuffer.assign(static_cast<std::size_t>(n) * 4, 0.0f);
}

bool OmGranularGroup::refreshHostPositions() {
#ifdef OMNISIM_WITH_CUDA
  if (mDeviceState == NULL || !mDeviceState->isAllocated())
    return false;
  const std::size_t expected = static_cast<std::size_t>(mDeviceState->particleCount()) * 4;
  if (expected == 0)
    return false;
  if (mHostPositionBuffer.size() != expected)
    mHostPositionBuffer.assign(expected, 0.0f);
  return mDeviceState->copyPositionsToHost(mHostPositionBuffer.data(), expected);
#else
  return false;
#endif
}

void OmGranularGroup::updateRenderingFromHost() {
  // P9: the GPU->host copy is deliberately NOT gated on any renderer state --
  // a --no-rendering session still has a live wgpu Camera, and the host buffer
  // is the deliverable wgpuParticles() hands out. (The per-particle WREN
  // transform pushes that used to follow the copy are gone -- D1.4.)
  refreshHostPositions();
}

void OmGranularGroup::allocateDeviceBufferIfPossible() {
  if (mDeviceState != NULL)
    return;

  if (!OmCudaContext::available()) {
    static bool warnedOnce = false;
    if (!warnedOnce) {
      warnedOnce = true;
      const QString msg = tr("GranularGroup is inert: CUDA is not available on this build/box. "
                             "Particles will not simulate; the world remains loadable.");
      OmLog::diagnostic("CUDA_NOT_AVAILABLE", msg);
      OmLog::warning(msg);
    }
    return;
  }

  // Resolve the world's up-axis once at allocation time (ENU=Z, NUE/EUN=Y).
  int upAxis = 2;  // ENU default
  const OmWorldInfo *wi = OmWorld::instance() ? OmWorld::instance()->worldInfo() : NULL;
  if (wi != NULL) {
    const OmVector3 up = wi->upVector();
    if (up.x() != 0.0)
      upAxis = 0;
    else if (up.y() != 0.0)
      upAxis = 1;
    else
      upAxis = 2;
  }

  const int particleCount = count();
  mDeviceState = new DeviceState(particleCount, radius(), upAxis, boundsHalfWidth(), mass());
  if (mDeviceState->isAllocated()) {
    OmLog::info(tr("GranularGroup: reserved GPU buffers + compiled gravity kernel "
                   "for %1 particles (radius %2 m, upAxis=%3).")
                  .arg(particleCount)
                  .arg(radius())
                  .arg(upAxis));
#ifdef OMNISIM_WITH_CUDA
    double mn = 0.0, mean = 0.0, mx = 0.0;
    if (mDeviceState->readbackUpStats(&mn, &mean, &mx))
      OmLog::info(tr("GranularGroup seed: up min=%1 mean=%2 max=%3 (n=%4)")
                    .arg(mn, 0, 'f', 4)
                    .arg(mean, 0, 'f', 4)
                    .arg(mx, 0, 'f', 4)
                    .arg(particleCount));
#endif
  }
}
