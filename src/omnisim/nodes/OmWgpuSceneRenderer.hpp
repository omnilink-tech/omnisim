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

#ifndef OM_WGPU_SCENE_RENDERER_HPP
#define OM_WGPU_SCENE_RENDERER_HPP

//
// OmWgpuSceneRenderer — R5c of engine-migration-plan.md §14.3.
//
// Shared off-screen wgpu scene-rendering helpers, factored so SENSOR device
// nodes (OmRangeFinder now; Lidar next) can render the world through the same
// pipeline OmCamera uses, without each node re-implementing the scene walk +
// view-projection math. Stateless apart from the caller-owned render-target +
// mesh-cache pointers handed to ensureTarget() by reference.
//
// Degrades gracefully on non-wgpu builds: ensureTarget() constructs a
// OmWgpuRenderTarget whose isUsable() is false when WB_WGPU_NATIVE_AVAILABLE
// is undefined, so it returns false and the caller falls through to WREN. No
// OMNISIM_WITH_VULKAN guard is needed here (mirrors OmCamera.cpp).
//
// NOTE: OmCamera still carries its own inline equivalents — the original,
// runtime-verified γ path. Migrating OmCamera onto this module is a tracked
// follow-up (engine-migration-plan.md §14.4); until then keep the two in sync.
//

#include <QtCore/QList>

#include <array>
#include <cstddef>
#include <vector>

class OmBaseNode;
class OmRenderBackend;
class OmWgpuRenderTarget;
class OmWgpuMeshCache;
class OmMatrix4;
class OmSolid;
class OmCadShape;
class OmGeometry;
class OmWgpuTextureCache;
class QObject;
struct OmWgpuSolidDraw;

namespace OmWgpuSceneRenderer {

  // Lazily (re)build a wgpu render target + mesh cache sized (w,h) for the
  // given backend. Operates on the caller's member pointers (passed by
  // reference) so each device node owns its own target. Returns true iff a
  // usable target now lives in `target`. False = not a Vulkan/wgpu backend, or
  // wgpu unavailable, or target construction failed → caller uses WREN.
  // Mirrors OmCamera::ensureWgpuTarget exactly.
  bool ensureTarget(OmRenderBackend *back, int w, int h,
                    OmWgpuRenderTarget *&target, OmWgpuMeshCache *&cache,
                    int &targetW, int &targetH);

  // Walk OmWorld::instance()'s top Solids → one OmWgpuSolidDraw per visible
  // Shape (a FINALIZED geometry + a PBRAppearance baseColor; W1c dropped the
  // former "geometry has a WREN mesh" precondition — see collectWorldDraws).
  // The draws'
  // modelMatrix16 pointers alias entries in `modelStorage`, so keep it alive
  // until the draw call completes. Same coverage as OmCamera's scene walk
  // (Box/Plane/Sphere/Cylinder primitives + Capsule + WREN-readback fallback).
  // outNodes (optional, R4 step-3c-A.1 picking): if non-null, filled parallel to
  // `out` with the top-level OmSolid each draw belongs to, so a picked draw index
  // maps back to a selectable node. Default null keeps existing callers unchanged.
  // texCache (optional, R4 material fidelity): when non-null, each draw's albedo
  // baseColorMap is uploaded through it and stored in OmWgpuSolidDraw::textureView.
  // Null (default) → flat baseColor, byte-identical to before.
  // Per-draw matrix-refresh record, 1:1 with `out` (via outRefresh): lets a caller re-derive the
  // model matrices WITHOUT re-walking the scene — the walk costs ~40 ms/frame on a 3.5k-draw city
  // (the main-view FPS killer), while matrices are the only per-frame-variant part of a
  // structurally-stable scene. Shape path: geom->matrix() (+localScale); CadShape path:
  // cad->wgpuWorldMatrix() (W1b, WREN-free), or wr_transform_get_matrix(wrenT) when the
  // OMNISIM_WGPU_NATIVE_CADSHAPE hatch is off. `node` is the owning scene node — connect to its
  // QObject::destroyed to invalidate the cached list before any pointer can dangle.
  // Exactly ONE of geom / cad / wrenT is set, and it is the same source the draw's VERTICES came
  // from — collectWorldDraws decides once per submesh and sets both together.
  struct OmWgpuDrawRefresh {
    OmGeometry *geom = nullptr;  // Shape path (exclusive with cad / wrenT)
    OmCadShape *cad = nullptr;   // CadShape path, native pose matrix (W1b)
    QObject *node = nullptr;     // owning node; destroyed() ⇒ rebuild the cached list
    float localScale[3] = {1.0f, 1.0f, 1.0f};
    bool hasLocalScale = false;
  };

  // ---- Per-VIEWER node visibility (wb_supervisor_node_set_visibility) ----
  //
  // WREN-retirement blocker. `wb_supervisor_node_set_visibility(node, from, visible)` is a
  // PUBLIC controller API and its ONLY implementation was a WREN scene-graph toggle
  // (wr_node_set_visible), so on the wgpu main view and on a wgpu-rendered Camera/RangeFinder/
  // Lidar the call SILENTLY did nothing. The state itself is NOT WREN-only — it is ordinary
  // engine-side state, one list per viewer:
  //
  //   * main view      -> OmViewpoint::mInvisibleNodes      (public getter getInvisibleNodes())
  //   * camera device  -> OmAbstractCamera::mInvisibleNodes (protected; an OmAbstractCamera
  //                       subclass passes `&mInvisibleNodes` from inside itself — no new
  //                       accessor is needed, and none was added)
  //
  // so this needed no parallel system, only a consumer on the wgpu side.
  //
  // ⚠ THE PER-CAMERA DIMENSION IS THE WHOLE POINT: the API hides a node from ONE viewer, not
  // globally, and WREN honours that by BRACKETING — OmAbstractCamera::computeValue() calls
  // viewpoint->enableNodeVisibility(true) to un-hide the main view's set before rendering a
  // camera, applies its OWN set, renders, then restores. A node hidden from the Viewpoint is
  // therefore still VISIBLE to every Camera. collectWorldDraws has no notion of "which viewer
  // am I collecting for", so the viewer's set is passed IN:
  //
  //   `hiddenNodes` == nullptr (the default)  -> nothing is filtered, byte-identical to the
  //                                              pre-change collect. This is deliberate: a
  //                                              default that silently applied the Viewpoint's
  //                                              set would leak main-view hides into every
  //                                              sensor render, which is a DIFFERENT and wrong
  //                                              behaviour.
  //   `hiddenNodes` != nullptr                -> that viewer's set is honoured.
  //
  // Consequently this is INERT until each call site names its viewer (main view:
  // `mainViewHiddenNodes()`; a camera/range-finder/lidar: `&mInvisibleNodes`).
  //
  // Semantics match WREN's: the list holds the nodes returned by
  // OmBaseNode::findClosestDescendantNodesWithDedicatedWrenNode() — an OmPose/OmSolid/OmRobot
  // (itself) or an OmGeometry (for a hidden Shape) — and hiding a WREN transform hides its
  // WHOLE SUBTREE, so a hidden node here prunes the entire subtree from the walk. A hidden
  // node is NOT counted in `outSkipped`: it is deliberately absent, not "not ready yet", and
  // counting it would make the caller re-collect for ever.
  //
  // ⚠ CACHED draw lists must be invalidated when visibility changes. OmViewpoint emits
  // nodeVisibilityChanged(node, visible) for exactly this; a camera device has no equivalent
  // signal but also does not cache (OmCamera re-collects every frame). Without that hook the
  // main view's cache hides/reveals the node only at its next scheduled rebuild.
  //
  // Hatch: OMNISIM_WGPU_NODE_VISIBILITY, VALUE-PARSED (unset or non-zero = ON, "0" = OFF).
  // OFF restores the ignore-everything behaviour even for a caller that passes a set.
  //
  // WIRING — one trailing argument per collect, nothing else:
  //   OmView3D.cpp / OmWgpuView.cpp (main view, all collects)
  //       ..., &wgpuCollectSkipped, OmWgpuSceneRenderer::mainViewHiddenNodes());
  //       + connect(viewpoint, &OmViewpoint::nodeVisibilityChanged, this,
  //                 [this]() { invalidateWgpuDrawList(); });   // the cached list is the catch
  //   OmCamera.cpp / OmRangeFinder.cpp / OmLidar.cpp (a viewer each, from inside the class)
  //       collectWorldDraws(*mWgpuMeshCache, draws, modelStorage,
  //                         nullptr, nullptr, nullptr, nullptr, &mInvisibleNodes);
  //       (mInvisibleNodes is OmAbstractCamera's protected member — reachable from the
  //        subclass, so no accessor is needed and none was added.)
  // hiddenNodes is the LAST parameter, so a call that skipped the four optionals before it must
  // spell them as nullptr — count them against the declaration below, do not copy an arity.
  // ⚠ A main-view collect that ALSO produces ground truth (OmView3D's mWgpuDrawSolids feeds the
  // synthetic-data instance dump) will drop viewpoint-hidden nodes from that dump too, which is
  // WREN's behaviour as well but is worth a deliberate decision rather than a surprise.
  bool nodeVisibilityEnabled();

  // The main view's hidden set: &OmWorld::instance()->viewpoint()->getInvisibleNodes(), or
  // nullptr when there is no world/viewpoint, when the set is empty, or when the hatch is off.
  // Call it per collect and pass the result straight through — never cache the pointer, it
  // belongs to the OmViewpoint and dies with the world.
  const QList<const OmBaseNode *> *mainViewHiddenNodes();

  // outSkipped (optional): incremented for every Shape that was DROPPED because it is
  // not READY yet — its geometry is not finalized, or (on the WREN-fallback branches
  // only) its WREN mesh does not exist yet. Subtrees for some nodes (measured: a
  // Robot's Pose-wrapped shapes) finalize AFTER the first render frame, so a draw list
  // collected on frame 0 silently misses them — and nothing re-collects until a
  // structural signal or the 600-frame fallback. A non-zero count tells the caller
  // this collect is INCOMPLETE and must not be cached as final (the patrol-rover
  // "renders in the engine, invisible in wgpu" bug).
  void collectWorldDraws(OmWgpuMeshCache &cache,
                         std::vector<OmWgpuSolidDraw> &out,
                         std::vector<std::array<float, 16>> &modelStorage,
                         std::vector<OmSolid *> *outNodes = nullptr,
                         OmWgpuTextureCache *texCache = nullptr,
                         std::vector<OmWgpuDrawRefresh> *outRefresh = nullptr,
                         int *outSkipped = nullptr,
                         const QList<const OmBaseNode *> *hiddenNodes = nullptr);

  // ---- DEFORMABLES (Cloth / SoftBody) on the wgpu main view -----------------
  //
  // THE DEFECT THIS CLOSES. collectWorldDraws walks Solids and dynamic_casts to
  // OmCadShape / OmShape / OmGroup / OmBasicJoint. A Cloth and a SoftBody are
  // NONE of those -- they derive straight from OmBaseNode, own a WrDynamicMesh
  // rather than a Geometry, and are parented to the scene root at identity because
  // their vertices come back from Newton in WORLD space. So when wgpu became the
  // default main-view renderer (2026-08-19) every deformable in the corpus stopped
  // rendering: newton_cloth_drape.omniworld reported draws=3 and an OmniLight BVH
  // of 24 tris -- the two static boxes -- for a world whose subject is a
  // 289-particle sheet.
  //
  // ⚠ AND THE MESH WAS ALSO FROZEN, which is the half that is easy to miss. Both
  // nodes' animateMesh() -- the per-frame particle readback -- is driven by a
  // wr_scene FRAME LISTENER, i.e. from inside wr_scene_render, which the wgpu main
  // view never calls. Collecting the geometry without also driving the readback
  // would have produced a cloth that renders correctly in one screenshot and never
  // moves. This function does both, in that order, so what it uploads is this
  // step's surface.
  //
  // SHAPE. Deformables are NOT part of the cached draw list: their vertices change
  // every step, so they are appended to the END of `out` (and `modelStorage`) on
  // every frame and the caller trims exactly `return value` entries off both before
  // the next frame's cache check. Appending last is what keeps every index into the
  // cached prefix (the selection tint, the draw->Solid map) valid.
  //
  // COST WHEN THERE IS NO DEFORMABLE: one static bool + one null pointer test. The
  // node list is the frame listener's existing subscription registry, so nothing
  // walks the scene to discover that a world has no cloth.
  //
  // UPLOAD STRATEGY. One cache entry per NODE, keyed on the node pointer inside a
  // reserved region of the key space, uploaded once and then re-written in place
  // through OmWgpuMeshCache::updateVertices() -- NOT released and re-acquired per
  // frame. The topology (index buffer) is uploaded once and never touched again.
  //
  // Returns the number of draws appended (0 when disabled, when no deformable is
  // subscribed, or when nothing was renderable yet).
  //
  // ⚠ THE PER-CACHE UPLOAD EPOCH, and the hatch that proves it. The re-upload decision is
  // "is THIS cache's copy of this mesh stale?" (OmWgpuMeshCache::vertexEpochIs), NOT the
  // process-global "did the clock advance since the last pump" edge that also drives
  // animateDeformables(). The two look identical while there is one collector and diverge the
  // moment there are two: each device owns its own mesh cache, whichever renderer reaches a
  // step first consumes the global edge, and every later one still takes the first-upload
  // branch once and then never updates -- a surface that animates on screen and is FROZEN in
  // the sensor image. OMNISIM_WGPU_DEFORMABLE_EPOCH=0 restores the global-edge rule so that
  // failure can be reproduced on demand; do not ship anything that depends on it.
  //
  // Hatch: OMNISIM_WGPU_DEFORMABLES, VALUE-PARSED -- unset or anything other than
  // "0"/"false"/"off"/"no" (case-insensitive) is ON. OFF restores the pre-fix
  // behaviour exactly: no deformable draws AND no wgpu-side animateMesh() pump.
  // Default ON: this is a regression fix, not an experiment.
  // ⚠ PREFER collectDynamicDraws() BELOW. This is one half of it, exposed only so a caller
  // that genuinely wants deformables and not particles can say so; every renderer should call
  // the combined entry point instead, for the reason spelled out there.
  //
  // `hiddenNodes` follows collectWorldDraws' convention exactly (null = filter nothing). A
  // Cloth/SoftBody is parented at the scene root with no subtree, so membership IS the test.
  size_t collectDeformableDraws(OmWgpuMeshCache &cache,
                                std::vector<OmWgpuSolidDraw> &out,
                                std::vector<std::array<float, 16>> &modelStorage,
                                OmWgpuTextureCache *texCache = nullptr,
                                const QList<const OmBaseNode *> *hiddenNodes = nullptr);

  // ---- GRANULAR PARTICLES (OmGranularGroup) — P9 ---------------------------
  //
  // One draw per particle: the cached unit UV sphere (the same mesh a `Sphere`
  // geometry gets) placed by a TRS model matrix built from the node's CPU-side
  // (x, y, z, radius) readback, with WREN's sand-yellow phong material and its
  // castShadows=false / receive-shadows=true pair. Deliberately NOT the instanced
  // pass — clearAndDrawInstanced has no lighting, materials or shadows — and NOT
  // the deformable vertex-stream path, since a particle's vertices never change,
  // only its transform.
  //
  // The node derives from OmBaseNode, not OmSolid, so collectWorldDraws cannot
  // reach it; membership comes from OmGranularGroup's own registry, whose
  // anyGranularGroups() is an empty-test that constructs nothing. Cost on a world
  // with no GranularGroup: one static bool + one vector empty-test.
  //
  // A group with no live CUDA state draws NOTHING (see OmGranularGroup::
  // wgpuParticles — WREN draws `count` spheres piled at the origin in that case,
  // which is a bug rather than a behaviour worth reproducing).
  //
  // Returns the number of draws appended. Hatch: OMNISIM_WGPU_GRANULAR,
  // VALUE-PARSED (unset or anything but "0"/"false"/"off"/"no" is ON).
  size_t collectGranularDraws(OmWgpuMeshCache &cache,
                              std::vector<OmWgpuSolidDraw> &out,
                              std::vector<std::array<float, 16>> &modelStorage,
                              const QList<const OmBaseNode *> *hiddenNodes = nullptr);

  // ---- TRACK belt elements + MUSCLE surfaces — P2 ---------------------------
  //
  // ⚠ THESE TWO ARE NOT THE SAME KIND OF THING, and porting them as one would get
  // both wrong.
  //
  // A `Track` is INSTANCED GEOMETRY. It derives from OmSolid, and its belt is N
  // copies of a real OmGeometry: animateMesh() generates no vertices at all, it
  // advances a belt-path position and writes wr_transform_set_position /
  // _set_orientation on N WrTransforms. So the port is N extra MODEL MATRICES
  // reusing a mesh the ordinary Shape acquisition already knows how to build --
  // the same shape P9's granular port used. (The Track's own body IS reached by
  // collectWorldDraws, since it is a Solid; only the belt is missing. And the
  // SOURCE geometry is not double-drawn: `animatedGeometry` is an SFNode field,
  // not a child, so the children walk never reaches it -- which is the wgpu
  // equivalent of WREN's wr_node_set_visible(geom, false).)
  //
  // A `Muscle` is PROCEDURAL and owns NO CPU-side geometry: it synthesises a
  // constant-volume spheroid straight into a WrDynamicMesh every step. Its port
  // is a vertex STREAM (OmMuscle::wgpuVertexStream) uploaded through the
  // per-(cache, mesh) epoch, like a Cloth -- and unlike a Cloth it is not
  // appearance-driven at all: hardcoded phong colours, a fixed texture asset, and
  // a status-driven recolour.
  //
  // ⚠ NEITHER IS COVERED BY anyDeformablesSubscribed(). The frame listener's track
  // and muscle lists are PENDING-REBUILD queues that frameStarted() CLEARS after
  // running them, so they are empty most of the time; the liveness registries are
  // OmTrack::anyTracks() / OmMuscle::anyMuscles(), both empty-tests on vectors that
  // already exist. Both collects also drive the wgpu-side animateMesh() pump these
  // node types would otherwise never receive (a wr_scene frame listener fires from
  // inside wr_scene_render, which no wgpu path calls) -- which for a Track also
  // DRAINS mAnimationStepSize, an accumulator that would otherwise grow for ever.
  //
  // Hatches: OMNISIM_WGPU_TRACK / OMNISIM_WGPU_MUSCLE, VALUE-PARSED (unset or
  // anything but "0"/"false"/"off"/"no" is ON), read once per process.
  // ⚠ PREFER collectDynamicDraws() BELOW; these are exposed only for symmetry.
  size_t collectTrackDraws(OmWgpuMeshCache &cache,
                           std::vector<OmWgpuSolidDraw> &out,
                           std::vector<std::array<float, 16>> &modelStorage,
                           OmWgpuTextureCache *texCache = nullptr,
                           const QList<const OmBaseNode *> *hiddenNodes = nullptr);

  size_t collectMuscleDraws(OmWgpuMeshCache &cache,
                            std::vector<OmWgpuSolidDraw> &out,
                            std::vector<std::array<float, 16>> &modelStorage,
                            OmWgpuTextureCache *texCache = nullptr,
                            const QList<const OmBaseNode *> *hiddenNodes = nullptr);

  // ---- THE ONE ENTRY POINT FOR NON-SOLID, PER-STEP-VARYING CONTENT ---------
  //
  // ⚠ CALL THIS, NOT THE TWO ABOVE, AND CALL IT FROM EVERY RENDER PATH.
  //
  // collectWorldDraws walks Solids. Cloth, SoftBody and GranularGroup are none of
  // them — they hang off OmBaseNode — so each needs its own collect, and each new
  // one is a fresh opportunity to wire the main view and forget the sensors. That
  // is not hypothetical: it is exactly what happened to the deformables. They were
  // fixed for the wgpu main view in `d5897ff7d`, the fix appended downstream of
  // collectWorldDraws in OmView3D, and collectDeformableDraws then had ONE caller
  // in the whole tree — so Cloth and SoftBody were invisible to every wgpu-rendered
  // Camera, RangeFinder and Lidar for as long as that shipped. Under WREN they were
  // visible (both renderables carry VM_REGULAR and every sensor mask intersects it),
  // so that was a REGRESSION AGAINST WREN, not a missing nicety.
  //
  // This function exists so the next such node type is wired once, here, and every
  // renderer gets it. Adding one means editing this function — not eight call sites.
  //
  // Returns the total number of draws appended (deformables first, then particles).
  // A caller with a CACHED draw list must trim exactly this many entries off the
  // tail of both `out` and `modelStorage` before its next cache check; a caller with
  // function-local vectors (every sensor device) rebuilds them per call and has
  // nothing to trim.
  //
  // outCounts (optional): the per-kind breakdown, for a caller that reports it. The
  // appended tail is [deformable][granular][track][muscle] in that order, which is
  // what lets OmView3D's `deformZ` telemetry keep measuring ONLY the deformables and
  // its `trackT` / `muscleR` telemetry measure only their own.
  struct OmWgpuDynamicCounts {
    size_t deformable = 0;
    size_t granular = 0;
    size_t track = 0;
    size_t muscle = 0;
  };
  size_t collectDynamicDraws(OmWgpuMeshCache &cache,
                             std::vector<OmWgpuSolidDraw> &out,
                             std::vector<std::array<float, 16>> &modelStorage,
                             OmWgpuTextureCache *texCache = nullptr,
                             const QList<const OmBaseNode *> *hiddenNodes = nullptr,
                             OmWgpuDynamicCounts *outCounts = nullptr);

  // The hatches above, for a caller that wants to report them.
  bool deformablesEnabled();
  bool granularEnabled();
  bool trackEnabled();
  bool muscleEnabled();
  // OMNISIM_WGPU_SENSOR_DYNAMIC — whether a SENSOR device collects dynamic content at all.
  // OFF reproduces the pre-P1 sensor image exactly (deformables and particles invisible to
  // Camera/RangeFinder/Lidar) while leaving the main view untouched, so a red/green A/B runs
  // on one binary. VALUE-PARSED, default ON. Consumed by OmAbstractCamera::collectWgpuDraws.
  bool sensorDynamicEnabled();

  // W1c GL-arm telemetry, cumulative for the process.
  //
  // collectWorldDraws used to run inside an unconditional
  // makeWrenCurrent()/doneWren() bracket owned by OmView3D, because ONE consumer
  // inside the walk needs a current GL context: WREN's mesh readback
  // (wr_static_mesh_read_data -> glGetBufferSubData), which is now reached only by
  // the `default:` branch's fallback and the CadShape hatch-off fallback. That
  // bracket is gone. The collect arms the context ITSELF — lazily (nothing happens
  // until a readback is actually about to run), at most once per collect, and via
  // RAII so every early return and every exception releases it.
  //
  // This counter is what turns "the bracket is dead" into a MEASURED number per
  // world rather than an assertion: with a warm mesh cache there are no misses,
  // hence no readback, hence no arm, and this value stops moving. OmView3D
  // publishes it (and its per-window delta) in the OMNISIM_WGPU_REPORT line as
  // glArms / glArmsWindow.
  unsigned long long glArmCount();

  // Recompute every cached draw's model matrix in place from its refresh record (no scene walk, no
  // mesh/texture-cache work). A record that turns degenerate keeps its previous matrix. Returns
  // false on a size mismatch — the caller must rebuild via collectWorldDraws.
  //
  // `outDraws` (optional, P2): the caller's cached draw list, 1:1 with `refresh`. When a Track in
  // the world declares a `textureAnimation`, each draw's UV affine is ALSO re-read from its
  // appearance -- the ConveyorBelt case, where the belt's motion IS a per-step TextureTransform
  // and a matrix-only refresh froze it on the main view while every sensor (which rebuilds its
  // list per frame) saw it move. Null, or a world with no such Track, does exactly what it did
  // before.
  bool refreshWorldDraws(std::vector<std::array<float, 16>> &modelStorage,
                         const std::vector<OmWgpuDrawRefresh> &refresh,
                         std::vector<OmWgpuSolidDraw> *outDraws = nullptr);

  // Column-major view-projection from a camera/sensor node's world matrix +
  // horizontal FOV (radians) + aspect + near/far. Encodes the Webots
  // (+X forward, +Z up) → wgpu (-Z forward, +Y up) basis swap. Matches
  // OmCamera's inline math (minus the OMNISIM_R34_IDENTITY_VP debug bypass,
  // which is Camera-diagnostic only).
  // reversedZ: emit clip z' = w - z, mapping near→1 / far→0. Paired with a FLOAT depth buffer and
  // a Greater depth test this gives near-uniform depth precision at every distance (the standard
  // reversed-Z trick) — fixes far-field z-fighting (road/grass decals) no near-plane tweak can.
  void buildViewProj(const OmMatrix4 &cameraWorld, double horizFov, double aspect,
                     double zNear, double zFar, float outViewProj16[16], bool reversedZ = false);

  // W3 defect 2 -- an authored `far` of 0 means "infinite", not "zero".
  //
  // Camera.wrl defaults `far 0`, and WREN has always substituted a finite plane for
  // it deep inside the renderer: wren::Camera::setFar is
  // `mFar = farDistance > 0.0f ? farDistance : 10000.0f` (src/wren/Camera.hpp:51-53),
  // so every WREN-rendered Camera has run with a 10 km far plane. The wgpu sensor
  // path had no such substitution and fed the raw 0 straight into perspective(),
  // which then emits m22 = zFar / (zNear - zFar) = 0 and m32 = 0 -- clip.z is
  // identically 0 for EVERY fragment, so the depth test degenerates and visibility
  // becomes draw-order dependent. It stayed invisible only because all nine existing
  // wgpu probe worlds declare an explicit `far 10.0`.
  //
  // Same predicate and same constant as WREN, so a sensor that opts into wgpu gets
  // the projection WREN would have given it. A positive authored value is returned
  // unchanged, so every world that declares a real `far` is bit-identical.
  constexpr double kInfiniteFarSubstitute = 10000.0;
  double sensorFarPlane(double authoredFar);

  // Column-major VIEW matrix (kBasisSwap * inverse(cameraWorld)) — the
  // camera-relative transform with the basis swap but no projection. The Lidar
  // radial-range path (clearAndDrawSceneRangeF32) needs it to recover
  // view-space position. buildViewProj is `perspective * buildView`.
  void buildView(const OmMatrix4 &cameraWorld, float outView16[16]);

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
  void buildOrthoLightViewProj(const OmMatrix4 &lightWorld, double halfExtent,
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
  void buildCascadeLightViewProjs(const OmMatrix4 &cameraWorld, double horizFov,
                                  double aspect, double camNear, double camFar,
                                  const OmMatrix4 &lightWorld, int numCascades,
                                  double splitLambda, float *outLightViewProjs,
                                  float *outSplits);

  // T1.2 CSM (multi-cascade) headless self-test: build the GPU-proven prototype camera/light
  // frames + the N=3 cascade fit (buildCascadeLightViewProjs) and drive the render-layer
  // OmWgpuRenderTarget::selfTestCsm with those raw matrices. Lives here (nodes/) because it needs
  // OmMatrix4 + the cascade math — the render layer carries no up-dependency on maths/ or nodes/.
  // Fills the three RGB outputs (floor-under-caster shadowed / floor-to-side lit / strength-0
  // reference) + *cascadeSelected (the cascade the shadow point routes to, expected ≥1), and
  // returns the render verdict. Drives OMNISIM_PROBE_CSM. Expects a SQUARE render target
  // (the probe uses 256×256, so the prototype's aspect=1 projection lines up).
  bool csmSelfTest(OmWgpuRenderTarget &rt, unsigned char shadowedOut[3],
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

}  // namespace OmWgpuSceneRenderer

#endif  // OM_WGPU_SCENE_RENDERER_HPP
