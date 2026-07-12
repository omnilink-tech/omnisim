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

#ifndef WB_WGPU_SCENE_RENDERER_HPP
#define WB_WGPU_SCENE_RENDERER_HPP

//
// WbWgpuSceneRenderer — R5c of engine-migration-plan.md §14.3.
//
// Shared off-screen wgpu scene-rendering helpers, factored so SENSOR device
// nodes (WbRangeFinder now; Lidar next) can render the world through the same
// pipeline WbCamera uses, without each node re-implementing the scene walk +
// view-projection math. Stateless apart from the caller-owned render-target +
// mesh-cache pointers handed to ensureTarget() by reference.
//
// Degrades gracefully on non-wgpu builds: ensureTarget() constructs a
// WbWgpuRenderTarget whose isUsable() is false when WB_WGPU_NATIVE_AVAILABLE
// is undefined, so it returns false and the caller falls through to WREN. No
// OMNISIM_WITH_VULKAN guard is needed here (mirrors WbCamera.cpp).
//
// NOTE: WbCamera still carries its own inline equivalents — the original,
// runtime-verified γ path. Migrating WbCamera onto this module is a tracked
// follow-up (engine-migration-plan.md §14.4); until then keep the two in sync.
//

#include <array>
#include <vector>

class WbRenderBackend;
class WbWgpuRenderTarget;
class WbWgpuMeshCache;
class WbMatrix4;
class WbSolid;
class WbGeometry;
class WbWgpuTextureCache;
class QObject;
struct WbWgpuSolidDraw;

namespace WbWgpuSceneRenderer {

  // Lazily (re)build a wgpu render target + mesh cache sized (w,h) for the
  // given backend. Operates on the caller's member pointers (passed by
  // reference) so each device node owns its own target. Returns true iff a
  // usable target now lives in `target`. False = not a Vulkan/wgpu backend, or
  // wgpu unavailable, or target construction failed → caller uses WREN.
  // Mirrors WbCamera::ensureWgpuTarget exactly.
  bool ensureTarget(WbRenderBackend *back, int w, int h,
                    WbWgpuRenderTarget *&target, WbWgpuMeshCache *&cache,
                    int &targetW, int &targetH);

  // Walk WbWorld::instance()'s top Solids → one WbWgpuSolidDraw per visible
  // Shape (geometry with a wrenMesh + a PBRAppearance baseColor). The draws'
  // modelMatrix16 pointers alias entries in `modelStorage`, so keep it alive
  // until the draw call completes. Same coverage as WbCamera's scene walk
  // (Box/Plane/Sphere/Cylinder primitives + Capsule + WREN-readback fallback).
  // outNodes (optional, R4 step-3c-A.1 picking): if non-null, filled parallel to
  // `out` with the top-level WbSolid each draw belongs to, so a picked draw index
  // maps back to a selectable node. Default null keeps existing callers unchanged.
  // texCache (optional, R4 material fidelity): when non-null, each draw's albedo
  // baseColorMap is uploaded through it and stored in WbWgpuSolidDraw::textureView.
  // Null (default) → flat baseColor, byte-identical to before.
  // Per-draw matrix-refresh record, 1:1 with `out` (via outRefresh): lets a caller re-derive the
  // model matrices WITHOUT re-walking the scene — the walk costs ~40 ms/frame on a 3.5k-draw city
  // (the main-view FPS killer), while matrices are the only per-frame-variant part of a
  // structurally-stable scene. Shape path: geom->matrix() (+localScale); CadShape path:
  // wr_transform_get_matrix(wrenT). `node` is the owning scene node — connect to its
  // QObject::destroyed to invalidate the cached list before any pointer can dangle.
  struct WbWgpuDrawRefresh {
    WbGeometry *geom = nullptr;  // Shape path (exclusive with wrenT)
    void *wrenT = nullptr;       // WrTransform* — CadShape path
    QObject *node = nullptr;     // owning node; destroyed() ⇒ rebuild the cached list
    float localScale[3] = {1.0f, 1.0f, 1.0f};
    bool hasLocalScale = false;
  };

  void collectWorldDraws(WbWgpuMeshCache &cache,
                         std::vector<WbWgpuSolidDraw> &out,
                         std::vector<std::array<float, 16>> &modelStorage,
                         std::vector<WbSolid *> *outNodes = nullptr,
                         WbWgpuTextureCache *texCache = nullptr,
                         std::vector<WbWgpuDrawRefresh> *outRefresh = nullptr);

  // Recompute every cached draw's model matrix in place from its refresh record (no scene walk, no
  // mesh/texture-cache work). A record that turns degenerate keeps its previous matrix. Returns
  // false on a size mismatch — the caller must rebuild via collectWorldDraws.
  bool refreshWorldDraws(std::vector<std::array<float, 16>> &modelStorage,
                         const std::vector<WbWgpuDrawRefresh> &refresh);

  // Column-major view-projection from a camera/sensor node's world matrix +
  // horizontal FOV (radians) + aspect + near/far. Encodes the Webots
  // (+X forward, +Z up) → wgpu (-Z forward, +Y up) basis swap. Matches
  // WbCamera's inline math (minus the OMNISIM_R34_IDENTITY_VP debug bypass,
  // which is Camera-diagnostic only).
  // reversedZ: emit clip z' = w - z, mapping near→1 / far→0. Paired with a FLOAT depth buffer and
  // a Greater depth test this gives near-uniform depth precision at every distance (the standard
  // reversed-Z trick) — fixes far-field z-fighting (road/grass decals) no near-plane tweak can.
  void buildViewProj(const WbMatrix4 &cameraWorld, double horizFov, double aspect,
                     double zNear, double zFar, float outViewProj16[16], bool reversedZ = false);

  // Column-major VIEW matrix (kBasisSwap * inverse(cameraWorld)) — the
  // camera-relative transform with the basis swap but no projection. The Lidar
  // radial-range path (clearAndDrawSceneRangeF32) needs it to recover
  // view-space position. buildViewProj is `perspective * buildView`.
  void buildView(const WbMatrix4 &cameraWorld, float outView16[16]);

  // T1.2 CSM sub-step 1: column-major ORTHOGRAPHIC light-space view-projection,
  // for rendering the scene from a directional light's POV into a depth map (the
  // first half of cascaded shadow maps). `lightWorld` is the light's world frame
  // (its local +X is the light/forward direction, matching the camera
  // convention buildView uses); `halfExtent` is the orthographic half-width/
  // height (the light frustum covers [-halfExtent, +halfExtent] in light X/Y);
  // zNear/zFar bound the depth range along the light axis. Uses the same
  // kBasisSwap + wgpu NDC depth→[0,1] convention as buildViewProj, so a depth
  // map rendered with this matrix is directly comparable to clip.w-style depth.
  // Pure CPU math; no GPU state — nothing calls it until the depth-render
  // sub-step lands, so it is inert/zero-break on its own.
  void buildOrthoLightViewProj(const WbMatrix4 &lightWorld, double halfExtent,
                               double zNear, double zFar, float outViewProj16[16]);

  // Maximum cascade count the CSM path supports. Caller-side buffer sizing +
  // (later) the fixed kSolidLitCsm shadow-array depth bind to this.
  constexpr int kMaxCascades = 4;

  // T1.2 CSM (multi-cascade) — the generalisation of buildOrthoLightViewProj from
  // ONE light frustum to N. Partitions the camera frustum [camNear,camFar] into
  // `numCascades` depth slices (PSSM practical split scheme, log↔uniform blended by
  // `splitLambda`∈[0,1]) and fits a TIGHT orthographic light view-projection to each
  // slice — invert the slice's camera viewProj for its 8 world corners, AABB them in
  // light-view space, square + extend-near so casters in front still cast. Writes
  // `numCascades*16` floats to `outLightViewProjs` (cascade-major) and the
  // `numCascades+1` boundary distances to `outSplits` (so the receiver shader can pick
  // a cascade by view depth). `numCascades` is clamped to [1, kMaxCascades]; size the
  // two output buffers for kMaxCascades. Same kBasisSwap + wgpu NDC depth→[0,1]
  // convention as buildViewProj, so a depth map rendered per cascade is directly
  // comparable to the camera path's depth. Pure CPU, no GPU state — INERT until the
  // multi-cascade depth pass + kSolidLitCsm pipeline reference it (mirrors how
  // buildOrthoLightViewProj landed: zero-break on its own).
  void buildCascadeLightViewProjs(const WbMatrix4 &cameraWorld, double horizFov,
                                  double aspect, double camNear, double camFar,
                                  const WbMatrix4 &lightWorld, int numCascades,
                                  double splitLambda, float *outLightViewProjs,
                                  float *outSplits);

  // T1.2 CSM (multi-cascade) headless self-test: build the GPU-proven prototype camera/light
  // frames + the N=3 cascade fit (buildCascadeLightViewProjs) and drive the render-layer
  // WbWgpuRenderTarget::selfTestCsm with those raw matrices. Lives here (nodes/) because it needs
  // WbMatrix4 + the cascade math — the render layer carries no up-dependency on maths/ or nodes/.
  // Fills the three RGB outputs (floor-under-caster shadowed / floor-to-side lit / strength-0
  // reference) + *cascadeSelected (the cascade the shadow point routes to, expected ≥1), and
  // returns the render verdict. Drives OMNISIM_PROBE_CSM. Expects a SQUARE render target
  // (the probe uses 256×256, so the prototype's aspect=1 projection lines up).
  bool csmSelfTest(WbWgpuRenderTarget &rt, unsigned char shadowedOut[3],
                   unsigned char litSideOut[3], unsigned char shadowOffOut[3], int *cascadeSelected);

  // T1.4 TAA — sub-pixel camera jitter (the INPUT side of TAA; the resolve/accumulator average the
  // jittered frames). haltonJitter writes the frame's jitter offset in PIXELS — a Halton(2,3)
  // low-discrepancy sample over an 8-frame sequence, in [-amplitudePx, +amplitudePx] on each axis —
  // to outOffsetPx2. jitterViewProj applies such a pixel offset to a column-major view-projection as
  // a DEPTH-INDEPENDENT clip-space shift (offsetPx in screen pixels: +x right, +y down), so the whole
  // rendered frame samples at a sub-pixel-shifted position. Pure CPU; inert until the main-view scene
  // render applies them per frame (L3 main-view wiring).
  void haltonJitter(int frameIndex, double amplitudePx, float outOffsetPx2[2]);
  void jitterViewProj(float viewProj16[16], const float offsetPx2[2], double width, double height);

}  // namespace WbWgpuSceneRenderer

#endif  // WB_WGPU_SCENE_RENDERER_HPP
