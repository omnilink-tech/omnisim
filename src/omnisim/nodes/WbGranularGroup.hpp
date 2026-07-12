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

#ifndef WB_GRANULAR_GROUP_HPP
#define WB_GRANULAR_GROUP_HPP

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

#include "WbBaseNode.hpp"

#include <vector>

class WbSFDouble;
class WbSFInt;
class WbMFVector3;
class WbSFNode;

struct WrTransform;
struct WrStaticMesh;
struct WrMaterial;
struct WrRenderable;

class WbGranularGroup : public WbBaseNode {
  Q_OBJECT

public:
  explicit WbGranularGroup(WbTokenizer *tokenizer = NULL);
  WbGranularGroup(const WbGranularGroup &other);
  explicit WbGranularGroup(const WbNode &other);
  ~WbGranularGroup() override;

  int nodeType() const override { return WB_NODE_GRANULAR_GROUP; }
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;

  // Read-only accessors for future M2/M3 increments and for tests.
  double radius() const;
  int count() const;
  double boundsHalfWidth() const;
  double mass() const;

private slots:
  // Per-physics-step entry: dispatches the gravity-integration kernel and
  // (every kReadbackInterval steps) copies positions back for telemetry.
  void onPhysicsStepStarted();

private:
  WbGranularGroup &operator=(const WbGranularGroup &);  // non copyable
  WbNode *clone() const override { return new WbGranularGroup(*this); }

  void init();
  // Allocates the device-side particle buffer if CUDA is available; logs a
  // one-shot CUDA_NOT_AVAILABLE warning otherwise. Idempotent.
  void allocateDeviceBufferIfPossible();

  // user-accessible fields
  WbSFDouble *mRadius;
  WbSFInt *mCount;
  WbMFVector3 *mInitialPositions;
  WbSFNode *mContactMaterial;
  WbSFDouble *mBoundsHalfWidth;
  WbSFDouble *mMass;

  // Opaque ownership of the device buffer when CUDA is on. Held by an
  // implementation-private helper to keep this header free of CUDA
  // dependencies — every translation unit including this can stay agnostic
  // to the build flavor.
  class DeviceState;
  DeviceState *mDeviceState;

  // WREN rendering — M2 host-readback path. Replaced by GL/CUDA interop in
  // M1 (planned). One transform per particle, all attached under
  // mWrenParticleRoot which itself attaches to the scene root in
  // createWrenObjects(). The shared sphere mesh + material live for the
  // node's lifetime.
  void deleteWrenObjects();
  void updateRenderingFromHost();
  // Gathers world-space (cx, cy, cz, radius) for each WbSolid bounding
  // sphere that should push particles. Only includes Solids that have a
  // dynamic ODE body (so reverse-force application has a target).
  // Filters out self, the ground, arena walls, and anything with an
  // absurdly large bounding sphere. Output is packed as 4 floats per
  // collider, ready to copy to GPU. The parallel `outSolids` array stores
  // the WbSolid pointer for each entry so applyReverseForcesToOde can
  // look up the body to push back.
  void collectColliders(std::vector<float> &out, std::vector<class WbSolid *> &outSolids) const;

  WrTransform *mWrenParticleRoot;
  WrStaticMesh *mWrenSphereMesh;       // shared by every renderable
  WrMaterial *mWrenSphereMaterial;     // shared by every renderable
  std::vector<WrRenderable *> mWrenParticleRenderables;
  std::vector<WrTransform *> mWrenParticleTransforms;
  std::vector<float> mHostPositionBuffer;
};

#endif
