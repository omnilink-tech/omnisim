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

#ifndef OM_GRANULAR_GROUP_HPP
#define OM_GRANULAR_GROUP_HPP

//
// Description: M2 skeleton of the GPU-resident granular particle group.
//
//   This first slice is intentionally inert: it parses, occupies a row in the
//   scene tree, holds the static configuration that M3's dynamic-state
//   serializer will extend, and (when CUDA is on) reserves a typed device
//   buffer sized to fit the configured particle count. No physics, no
//   rendering, no controller-visible API yet — that's later in M2 and M3.
//
//   The PROTO is portable: a `.wbt` containing a GranularGroup loads cleanly
//   on `OMNISIM_WITH_CUDA=OFF` builds (particles inert, one-time
//   CUDA_NOT_AVAILABLE warning), and the static fields serialize via the
//   standard Webots `.wbt` mechanism — strict extension point for M3.
//
//   See docs/developer/cuda-compute-infrastructure-plan.md (M2) and
//   docs/developer/granular-physics-plan.md (Tier 3-GPU).
//

#include "OmBaseNode.hpp"

#include <vector>

class OmSFDouble;
class OmSFInt;
class OmMFVector3;
class OmSFNode;

class OmGranularGroup : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmGranularGroup(OmTokenizer *tokenizer = NULL);
  OmGranularGroup(const OmGranularGroup &other);
  explicit OmGranularGroup(const OmNode &other);
  ~OmGranularGroup() override;

  int nodeType() const override { return WB_NODE_GRANULAR_GROUP; }
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;

  // Read-only accessors for future M2/M3 increments and for tests.
  double radius() const;
  int count() const;
  double boundsHalfWidth() const;
  double mass() const;

  // ---- THE wgpu RENDER SURFACE (P9 of the WREN-deletion runbook) -----------
  //
  // The wgpu renderer draws the particles from mHostPositionBuffer via
  // wgpuParticles(); both the main view and every sensor go through
  // OmWgpuSceneRenderer::collectDynamicDraws, which every render path calls.
  // (The old per-particle WREN renderable/transform tree is gone -- D1.4.)
  //
  // THE REGISTRY. OmGranularGroup derives from OmBaseNode, not OmSolid, so
  // collectWorldDraws ("walk the top Solids, then the root's non-Solid children")
  // cannot reach it - and a scene walk to discover that a world has no particles
  // would tax every world that has none. Same shape as the deformables' registry
  // (OmDeformableFrameListener): membership is maintained by the node itself,
  // and anyGranularGroups() is a static empty-test that constructs nothing.
  static bool anyGranularGroups();
  static const std::vector<OmGranularGroup *> &liveGroups();

  // World-space particle centres + radii, 4 floats per particle (x, y, z, r),
  // pointing straight at mHostPositionBuffer - no copy. Returns FALSE, and touches
  // neither output, when this group has no live GPU state.
  //
  // ⚠ THAT REFUSAL IS DELIBERATE AND IT DIFFERS FROM WREN. On a build or a box
  // without CUDA, mHostPositionBuffer is all zeros while the WREN path still draws
  // N spheres - so WREN renders `count` particles piled at the world origin, which
  // is not a scene anyone authored. The wgpu path declines to draw instead. A
  // GranularGroup with no CUDA is inert by design (the engine already says so, once,
  // via the CUDA_NOT_AVAILABLE diagnostic); rendering a phantom heap at the origin
  // is a bug to leave behind, not a behaviour to reproduce.
  bool wgpuParticles(const float *&xyzrOut, int &countOut) const;

  // Copy this step's particle state GPU->host into the buffer wgpuParticles() hands out.
  // PUBLIC because the wgpu collector calls it: OmGranularGroup::onPhysicsStepStarted already
  // does this every step, so in a running simulation it is belt-and-braces -- but at t=0 and on
  // a PAUSED simulation no step has ever fired and the buffer is still the zeros
  // createWrenObjects() assigned, i.e. every particle at the world origin. Renderer-agnostic on
  // purpose (it must NOT be gated on WREN objects existing: a --no-rendering or WREN-less
  // session still has a live wgpu Camera). Returns true iff the buffer now holds this step's
  // positions.
  bool refreshHostPositions();

private slots:
  // Per-physics-step entry: dispatches the gravity-integration kernel and
  // (every kReadbackInterval steps) copies positions back for telemetry.
  void onPhysicsStepStarted();

private:
  OmGranularGroup &operator=(const OmGranularGroup &);  // non copyable
  OmNode *clone() const override { return new OmGranularGroup(*this); }

  void init();
  // Allocates the device-side particle buffer if CUDA is available; logs a
  // one-shot CUDA_NOT_AVAILABLE warning otherwise. Idempotent.
  void allocateDeviceBufferIfPossible();

  // user-accessible fields
  OmSFDouble *mRadius;
  OmSFInt *mCount;
  OmMFVector3 *mInitialPositions;
  OmSFNode *mContactMaterial;
  OmSFDouble *mBoundsHalfWidth;
  OmSFDouble *mMass;

  // Opaque ownership of the device buffer when CUDA is on. Held by an
  // implementation-private helper to keep this header free of CUDA
  // dependencies — every translation unit including this can stay agnostic
  // to the build flavor.
  class DeviceState;
  DeviceState *mDeviceState;

  // M2 host-readback path — refreshes mHostPositionBuffer each physics step
  // for the wgpu draw. Replaced by GPU interop in M1 (planned).
  void updateRenderingFromHost();
  // Gathers world-space (cx, cy, cz, radius) for each OmSolid bounding
  // sphere that should push particles. Only includes Solids that have a
  // dynamic ODE body (so reverse-force application has a target).
  // Filters out self, the ground, arena walls, and anything with an
  // absurdly large bounding sphere. Output is packed as 4 floats per
  // collider, ready to copy to GPU. The parallel `outSolids` array stores
  // the OmSolid pointer for each entry so applyReverseForcesToOde can
  // look up the body to push back.
  void collectColliders(std::vector<float> &out, std::vector<class OmSolid *> &outSolids) const;

  std::vector<float> mHostPositionBuffer;
};

#endif
