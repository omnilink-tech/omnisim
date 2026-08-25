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

#ifndef OM_NEWTON_BACKEND_HPP
#define OM_NEWTON_BACKEND_HPP

// Deliberately Qt-free (core-evolution-plan.md, Phase Q2): physics/ is future-core code.
// String-typed APIs use std::string; conversions to Qt happen at the caller/log boundary.
#include <string>
#include <vector>

//
// OmNewtonBackend — opt-in GPU-resident physics backend, wraps NVIDIA
// Newton (Apache 2.0, https://github.com/newton-physics/newton). Lives
// alongside OmOdeBackend; world authors opt a OmSolid into this
// backend via the (forthcoming) `physicsBackend "newton"` field.
//
// Build modes
// -----------
//
// OMNISIM_WITH_NEWTON=ON (default since the Stage 3 build-flag flip,
// 2026-06-06 -- see src/omnisim/Makefile + docs/developer/default-flip-plan.md):
//   - Newton library code is compiled in. isAvailable() actually brings up
//     the Newton runtime (warp + newton Python packages, embedded via the
//     stable CPython API) and returns true on success; on a box without the
//     runtime (it is NOT bundled) or without NVIDIA hardware it returns
//     false and the dispatcher falls back to OmOdeBackend -- safe, but not
//     actually Newton. The upstream PyPI package was renamed from
//     `newton-physics` to `newton` in late 2025; install with
//     `pip install "newton[examples]" warp-lang`.
//
// OMNISIM_WITH_NEWTON=OFF:
//   - Pure-ODE legacy build: OmNewtonBackend exists but isAvailable()
//     returns false, every Newton request falls back to OmOdeBackend at the
//     dispatcher layer, and zero Newton library code is linked into the
//     binary.
//
// This file is included on every build, regardless of
// OMNISIM_WITH_NEWTON, because the dispatcher needs to know that
// "newton" is a recognised backend kind even when the runtime is
// unavailable. The .cpp toggles the actual runtime hookup.
//

#include "OmPhysicsBackend.hpp"

// Opaque Python-state holder; defined in OmNewtonBackend.cpp where the
// embedded CPython headers can be safely included. Keeping the
// declaration opaque means no header in the rest of the tree pulls in
// <Python.h>, which would conflict with Qt's `slots`/`signals` macros.
struct OmNewtonRuntimeState;

// W4.1/W4.2: one native Newton rigid contact. bodyA/bodyB are Newton body indices (the same space as
// mNewtonBodyIndex / the pose readback / body_f writes; -1 = static world geometry). point + normal are world
// frame; depth is the penetration along the normal; forceMag is 0 under XPBD (positional solve).
struct OmNewtonContact {
  int bodyA;
  int bodyB;
  double point[3];
  double normal[3];
  double depth;
  double forceMag;
};

// One answered ray from the Newton raycast service (the ODE-ray replacement,
// ode-retirement-campaign.md kernel blocker #1). newtonBody is the hit Solid's
// Newton body index -- the same space as OmNewtonContact.bodyA and
// OmSolid::newtonBodyIndex(); -1 means static world geometry with no Solid
// (e.g. the implicit ground plane). dist = -1.0 on a miss within the ray's
// budget. normal is the hit surface normal when the bundled mujoco exposes it,
// else zeros (consumers must treat a zero normal as "absent", not "flat").
struct OmNewtonRayHit {
  double dist;
  int newtonBody;
  double normal[3];
};

// Which edges of a cloth patch are pinned, for OmNewtonBackend::addClothGrid's
// `fixFlags`. "Left"/"right" are the low/high ends of the patch's LOCAL X axis
// and "bottom"/"top" the low/high ends of its LOCAL Y axis -- the grid lies in
// local Z = 0, so none of these is a world-space up/down claim, and a patch
// rotated to hang vertically still pins along its own axes. Matches the
// fix_left / fix_right / fix_bottom / fix_top arguments of newton's
// ModelBuilder.add_cloth_grid.
enum OmNewtonClothFix {
  OmClothFixNone = 0,
  OmClothFixLeft = 1 << 0,
  OmClothFixRight = 1 << 1,
  OmClothFixBottom = 1 << 2,
  OmClothFixTop = 1 << 3
};

class OmNewtonBackend : public OmPhysicsBackend {
public:
  OmNewtonBackend();
  ~OmNewtonBackend() override;
  // Bring up CPython + import warp/newton/the OmniSim helper without creating
  // any world-specific state. Safe to call early (for example while a world is
  // being parsed) so the 1s+ cold import can overlap unrelated startup work.
  // Idempotent and process-lived; the constructor reuses the cached modules.
  static bool preloadRuntime();
  // Worker-thread form used by the registry. It releases CPython's GIL before
  // returning so the GUI thread can safely adopt the process-lived runtime.
  static bool preloadRuntimeAsyncWorker();
  static void adoptAsyncPreloadedRuntime();
  OmPhysicsBackendKind kind() const override { return OmPhysicsBackendKind::Newton; }
  const char *name() const override { return "newton"; }
  bool isAvailable() const override { return mAvailable; }

  // World-build-phase API (called once per OmWorld load, before sim
  // starts). Each returns 0 on success, -1 on failure; failures emit
  // a single OmLog::warning identifying the step. Calling these on a
  // backend with isAvailable()==false is a no-op success: the registry
  // never routes solids here in that state, so the methods exist only
  // to keep call-sites simple.

  // Resets any prior world state and starts a fresh ModelBuilder.
  // Idempotent: safe to call between world loads.
  int beginWorld();
  // Idempotent: opens a world (with a default ground plane) if one
  // isn't already open. Solids that opt into Newton can call this in
  // postFinalize without coordinating with siblings; the first call
  // wins and subsequent calls are no-ops. Returns 0 on a freshly-opened
  // world, 0 if already open, -1 on error.
  int ensureWorldOpen();
  // True iff a world has been begun and is in the build phase
  // (addBody/addShapeSphere accept). Becomes false once finalizeWorld()
  // succeeds (the world is then in the run phase).
  bool isWorldOpenForBuild() const;
  // True iff the world has been finalized (step/getBodyPosition accept).
  bool isWorldRunning() const;
  // Adds an infinite static ground plane to the model under construction.
  int addGroundPlane();
  // Adds a dynamic rigid body with the given mass at world position
  // (x, y, z) and orientation given by the (qx, qy, qz, qw) quaternion
  // (xyzw layout matching Newton's body_q convention). Returns the
  // body's index (>= 0) on success, -1 on error.
  // hasCom (default false): when false the link's center of mass defaults to
  // the link origin (legacy behavior, every existing Newton robot is validated
  // against it). When true the (cx, cy, cz) link-frame COM offset is passed
  // through to the runtime's add_body. Opt-in via OMNISIM_NEWTON_USE_LINK_COM.
  int addBody(double mass, double x, double y, double z,
              double qx, double qy, double qz, double qw,
              double ixx = 0.0, double iyy = 0.0, double izz = 0.0,
              double ixy = 0.0, double ixz = 0.0, double iyz = 0.0,
              bool hasCom = false, double cx = 0.0, double cy = 0.0, double cz = 0.0);
  // P8.1 statics-on-Newton: adds a STATIC rigid body (mass=0) at world
  // pose (x, y, z) + quaternion (qx, qy, qz, qw). Static bodies have
  // their xform pinned by the solver — `step()` won't move them, but
  // collision shapes attached via addShape* are tested against
  // dynamic-body shapes in the contact phase. Returns the body's
  // index (>= 0) on success, -1 on error. See
  // docs/developer/physics-p8-statics-design.md §4.
  int addStaticBody(double x, double y, double z,
                    double qx, double qy, double qz, double qw);
  // Kernel blocker #4 (_scratch/design_kinematic_inertia.md Part 1): adds a
  // KINEMATIC body -- a movable static obstacle. Build recipe identical to
  // addStaticBody (zero-mass link pinned to the world by a 0-DOF FIXED joint
  // at finalize), which is exactly the fixed-root shape SolverMuJoCo exports
  // as a MuJoCo MOCAP body: its geoms collide with dynamic bodies as a
  // kinematic obstacle (one-way coupling -- pushes dynamics, never pushed
  // back; no kinematic-vs-kinematic response), while its pose stays
  // ENGINE-owned and is pushed per change via setKinematicPose. Attach
  // collision shapes via addShape* after this, exactly like addStaticBody.
  // Returns the body's index (>= 0) or -1.
  int addKinematicBody(double x, double y, double z,
                       double qx, double qy, double qz, double qw);
  // RUN phase: write a kinematic (mocap) body's world pose straight into
  // mj_data.mocap_pos / mocap_quat -- effective on the NEXT step (MuJoCo's
  // kinematics copies mocap arrays into xpos/xquat at step start).
  // Quaternion is xyzw on the wire (Newton body_q convention); the runtime
  // reorders to MuJoCo's wxyz. Valid for any fixed-root body (kinematic or
  // static registration -- both are mocap-exported), so a supervisor-moved
  // static prop can be driven through it too. 0/-1.
  int setKinematicPose(int bodyIdx, double x, double y, double z,
                       double qx, double qy, double qz, double qw);
  // Attaches a sphere collision/visual shape to bodyIdx. Returns 0/-1.
  // (cx, cy, cz) is an offset in the body's local frame -- default
  // (0,0,0) places the sphere at the body origin.
  int addShapeSphere(int bodyIdx, double radius,
                     double cx = 0.0, double cy = 0.0, double cz = 0.0);
  // Attaches a box collision/visual shape to bodyIdx. (hx, hy, hz) are
  // half-extents along each local axis (Newton's convention -- OmBox
  // uses full size, so callers must divide by 2). (cx, cy, cz) is the
  // box centre offset in the body's local frame (default origin), and
  // (qx,qy,qz,qw) its orientation in that frame -- see the shape-xform
  // note below. Identity default, so an unrotated caller is unchanged.
  int addShapeBox(int bodyIdx, double hx, double hy, double hz,
                  double cx = 0.0, double cy = 0.0, double cz = 0.0,
                  double ke = -1.0,
                  double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0);
  // ── SHAPE XFORM CONVENTION ────────────────────────────────────────────────
  // Every addShape* below takes the collider's pose in the OWNING BODY's local
  // frame: (cx,cy,cz) translation + (qx,qy,qz,qw) orientation, quaternion in
  // XYZW order (newton/warp's `wp.transform` order -- NOT OmQuaternion's WXYZ,
  // so convert at the call site). OmSolid's boundingObject walkers compose that
  // pose from the authored Pose/Transform chain.
  //
  // ⚠ HISTORY -- this used to be translation-only, and the gap was papered over
  // rather than noticed. The walkers accumulated `translation()` and never read
  // `rotation()`, and addShapeCylinder then applied a FIXED -90 deg about X on
  // the stated premise that "a OmniSim Cylinder extends along its body-local Y".
  // That premise is false: an OmniSim Cylinder is Z-ALIGNED (OmCylinder.cpp
  // rescale(), docs/reference/cylinder.md), a URDF <cylinder> is Z-aligned, and
  // a newton capsule is Z-aligned -- nothing needed rotating. The two defects
  // cancelled on WHEELS (the URDF importer wraps every cylinder collision in a
  // Pose, and a wheel's is rpy 1.5708 about X; dropping +90 and inventing -90
  // describe the SAME capsule, which is symmetric about its centre) and
  // corrupted everything else -- OMNIARM6's seven arm cylinders are rpy "0 0 0",
  // so each was tipped Z->Y and lay crosswise through its own link. Both halves
  // are fixed here: the rotation is plumbed through and the -90 is gone.
  //
  // Cylinder + capsule: halfHeight is total length / 2 -- OmCylinder and
  // OmCapsule both expose `height()` as the full length, so callers must
  // divide. NOTE: native Newton cylinder narrow-phase locks wheels (probe 7),
  // so addShapeCylinder substitutes a same-radius/half-height capsule (line
  // contact, robust) -- both therefore land on newton's Z-aligned capsule and
  // need no axis correction of their own. Returns 0/-1.
  int addShapeCylinder(int bodyIdx, double radius, double halfHeight,
                       double cx = 0.0, double cy = 0.0, double cz = 0.0,
                       double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0);
  int addShapeCapsule(int bodyIdx, double radius, double halfHeight,
                      double cx = 0.0, double cy = 0.0, double cz = 0.0,
                      double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0);
  // Infinite static ground plane (newton-ode-replacement-plan.md W1.1) -- e.g. a Floor's Plane
  // boundingObject. Local normal +Z (the OmPlane convention); (cx,cy,cz) is the local offset, the body's
  // transform orients it. Returns 0/-1.
  //
  // ⚠ STILL TRANSLATION-ONLY (unlike box/cylinder/capsule/mesh above): a Plane authored inside a ROTATED
  // Pose keeps its +Z normal. Left alone deliberately -- this call is DEFERRED to finalize() and then
  // DROPPED entirely under the MuJoCo solver (the default), so the rotation has nowhere to land on the
  // path that actually runs. Same for addShapeHeightfield, which bakes the owning body's world transform
  // itself because a newton heightfield takes no body. Both are documented gaps, not oversights.
  int addShapePlane(int bodyIdx, double cx = 0.0, double cy = 0.0, double cz = 0.0);
  // Native triangle-mesh collision (newton-ode-replacement-plan.md W1) -- replaces the AABB-box
  // approximation. vertices = flat 3*nVertices doubles, indices = flat 3*nTriangles vertex indices;
  // (cx,cy,cz) is the mesh's local offset in the body frame. compute_inertia is OFF (the body already
  // carries the Solid's mass + inertia). Returns 0/-1.
  int addShapeMesh(int bodyIdx, const double *vertices, int nVertices,
                   const int *indices, int nTriangles,
                   double cx = 0.0, double cy = 0.0, double cz = 0.0,
                   double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0);
  // Native heightfield collision for an ElevationGrid boundingObject (terrain). heights = a flat
  // xDimension*yDimension array in the node's own row-major order (index = y*xDimension + x);
  // xSpacing/ySpacing are CELL sizes, so the field spans spacing*(dimension-1); (cx,cy,cz) is the
  // geometry's local offset in the body frame.
  //
  // ⚠ A newton heightfield is ALWAYS a static, world-attached shape -- newton's
  // add_shape_heightfield takes no body argument at all. bodyIdx is therefore used only to look the
  // owning Solid's world transform back out of the builder so the terrain lands where the .wbt puts
  // it; an ElevationGrid on a MOVING body will not follow that body. That is a newton constraint,
  // not an OmniSim choice, and terrain is static in every world in this tree.
  //
  // Before this existed an ElevationGrid fell off the end of OmSolid's geometry if/else chain and
  // registered NO collider: measured on the lane-4 probe, a 1 kg sphere reached z=-42.89 m and was
  // still accelerating at t=3 s. Returns 0/-1.
  int addShapeHeightfield(int bodyIdx, const double *heights, int xDimension, int yDimension,
                          double xSpacing, double ySpacing,
                          double cx = 0.0, double cy = 0.0, double cz = 0.0);

  // ---- Cloth / particles (OmCloth) --------------------------------------
  //
  // The FIRST non-rigid concept in this backend: every other add_* above
  // registers a body or a shape, and Newton keeps particles in a completely
  // separate set of Model arrays (particle_q / particle_qd / particle_flags)
  // with its own solver. Nothing here touches the rigid path, and a world
  // that never calls addClothGrid() is byte-identical to one built before
  // this API existed -- newton allocates no particle arrays when none are
  // added, and the runtime only constructs SolverVBD when a cloth exists.
  //
  // ⚠ REQUIRES the coupled solver, i.e. WorldInfo.newtonSolver "mujoco+vbd".
  // Under any other value the runtime has no SolverVBD to step the particles
  // with; it declines the registration and warns once (OmCloth turns that
  // into a per-node parse warning naming the field to set).

  // BUILD phase. Registers a rectangular cloth patch as a grid of particles
  // with FEM triangles + bending elements, mapping 1:1 onto newton's
  // ModelBuilder.add_cloth_grid.
  //
  //   pos  = world-space position of the grid's local origin (its (0,0) corner)
  //   quat = world-space orientation, (qx, qy, qz, qw) -- the house body_q
  //          layout, w LAST, same as addBody / addStaticBody
  //   dimX / dimY = the number of CELLS along the local X and Y axes. ⚠ NOT
  //          the vertex count: newton lays out (dimX + 1) * (dimY + 1)
  //          particles, row-major with Y outer and X inner, so particle
  //          (x, y) is at start + y * (dimX + 1) + x. OmCloth builds its
  //          triangle index list from exactly that rule, which is why the
  //          rendered mesh and the simulated particles cannot drift apart.
  //   cellX / cellY = cell size in metres (patch spans dimX*cellX by
  //          dimY*cellY in its local frame, lying in local Z = 0)
  //   mass = PER-PARTICLE mass in kg
  //   particleRadius = collision thickness of each particle; <= 0 leaves
  //          newton's own default
  //   triKe = triangle stretch stiffness, triKa = area preservation,
  //          triKd = damping, edgeKe = bending, edgeKd = bending damping.
  //          ⚠ NEGATIVE means "derive from the matching ke" on the runtime side
  //          (ka = ke, kd = 1e-2 * ke), which is the ratio newton's own coupled
  //          example uses. ZERO IS A LITERAL ZERO, not "unset" -- passing 0 for
  //          triKe really does give a sheet with no stretch resistance. This is
  //          the opposite convention from the WorldInfo newton* fields, where 0
  //          means unset, and it is the runtime's, not ours.
  //   fixFlags = bitwise OR of the OmNewtonClothFix constants above; a fixed
  //          edge's particles are pinned (mass 0, ACTIVE flag cleared) so the
  //          patch can hang from a rail instead of falling as a free sheet.
  //          Expanded into the runtime's four separate booleans at the call.
  //
  // On success returns the cloth's PARTICLE START INDEX (>= 0) and, when
  // `endOut` is non-null, writes the one-past-the-last index there -- that pair
  // is exactly the range snapshotParticlePositions() wants, so a world with
  // several sheets can stream each one independently. -1 on failure.
  int addClothGrid(const double pos[3], const double quat[4],
                   int dimX, int dimY, double cellX, double cellY,
                   double mass, double particleRadius,
                   double triKe, double triKa, double triKd,
                   double edgeKe, double edgeKd,
                   int fixFlags, int *endOut = nullptr);

  // BUILD phase. One cloth sheet from an ARBITRARY TRIANGLE MESH -- a GARMENT
  // rather than a rectangular patch. Maps onto newton's
  // ModelBuilder.add_cloth_mesh, which creates one FEM triangle per face and
  // derives the bending elements from mesh adjacency (so, unlike addClothGrid,
  // there is no edge list to pass -- 616 vertices of T-shirt yield 1780 bending
  // edges with nothing said about them).
  //
  //   vertices = flat 3*nVertices doubles in the mesh's OWN local frame
  //   indices  = flat 3*nTriangles vertex indices
  //   pos/quat = where that local frame lands in the world, (qx,qy,qz,qw), w LAST
  //   scale    = applied to the mesh coordinates BEFORE rot and pos (that is
  //              upstream's order, builder.py:9146-9151); 1.0 for a mesh already
  //              authored in metres
  //
  // ⚠ `density` IS NOT addClothGrid's `mass` AND THE UNIT IS DIFFERENT: kg per
  // SQUARE METRE, distributed by triangle area, not kg per particle. A garment
  // therefore weighs the same however finely it is tessellated, which is the
  // property that makes a decimated asset physically equivalent to the original.
  // Passing a grid's 0.001 here gives a T-shirt weighing under a gram.
  //
  // PINNING is `pinTopBand` (metres), not fixFlags: addClothGrid's
  // fix_left/right/top/bottom name the extremes of a rectangular lattice, which
  // a garment does not have, and upstream offers no mesh equivalent. The rule
  // is "pin every vertex within pinTopBand of the mesh's MAXIMUM LOCAL Z",
  // measured in the mesh's own frame before scale/rot/pos -- on a garment
  // authored as worn that is the shoulders, so a shirt hangs like it would on a
  // hanger. 0 disables it.
  //
  // ⚠ IT IS EFFECTIVELY REQUIRED TODAY. MEASURED: unpinned cloth resting on a
  // STATIC body sinks through it -- the 616-particle shirt passed through a
  // static box and kept falling, and so did a matched 196-particle GRID sheet
  // from the same height onto the same box, faster. That is the existing
  // cloth-vs-statics path and not mesh authoring, but it means "drape it over
  // something" is not yet a durable way to hold a garment up, and pinning is.
  //
  // ⚠ THE VERTEX ORDER THAT COMES BACK IS THE ORDER YOU SENT. The runtime welds
  // duplicates, drops degenerate/duplicate triangles and drops orphan vertices
  // before authoring (all three are silent, damaging defects in newton -- an
  // orphan vertex keeps mass 0 and becomes a KINEMATIC PIN), but it does so
  // ORDER-PRESERVINGLY and is an exact identity on clean input. OmCloth relies
  // on that: it keeps its own copy of the vertex list for the vertex buffer and
  // pairs it with the returned particle range, then asserts the counts match.
  // If they do not, the mesh needed cleaning and the caller must not render it.
  //
  // Returns the PARTICLE START INDEX (>= 0), writing one-past-the-last to
  // `endOut` when non-null. -1 on failure.
  int addClothMesh(const double pos[3], const double quat[4],
                   const double *vertices, int nVertices,
                   const int *indices, int nTriangles,
                   double density, double particleRadius,
                   double triKe, double triKa, double triKd,
                   double edgeKe, double edgeKd,
                   double scale, double pinTopBand, int *endOut = nullptr);

  // BUILD phase. One VOLUMETRIC (tetrahedral FEM) soft block -- the second
  // deformable primitive, and the same shape of call as addClothGrid above.
  //
  // A soft body is particles PLUS tetrahedra: the elasticity lives entirely in
  // the tets (kMu / kLambda are the Lame parameters in Pa, kDamp is viscous
  // damping), and the surface triangles newton generates are for collision and
  // rendering only. It therefore rides the SAME SolverVBD entry and the same
  // coupled path cloth does.
  //
  // ⚠ IT DOES NOT DISTURB THE INDEX-IDENTITY INVARIANT, and that is why this is
  // a small addition rather than a large one. A soft body adds PARTICLES ONLY,
  // so the coupled solver's "mjc" ModelView still owns every rigid body and the
  // raycast / weld / TouchSensor / pose-check readbacks that index by parent
  // body id stay valid (MEASURED at 1 and 3 bodies: view("mjc").body_count
  // equals the parent model's). A Cosserat ROD would not have this property --
  // it adds rigid capsule bodies and moves them out of "mjc".
  //
  // ⚠ `fixFlags` reuses OmNewtonClothFix, and its axis convention is newton's:
  // Left/Right pin local X = 0 / dimX, Top/Bottom pin local **Y** = dimY / 0.
  // There is NO pin on the Z faces, and "Top" is the local +Y face, NOT world
  // up -- pin the world-top face of a Z-up block by rotating +90 deg about X so
  // local +Y maps to world +Z and passing OmClothFixTop.
  //
  // Returns the block's PARTICLE START INDEX (>= 0), writing one-past-the-last
  // to `endOut` when non-null -- the range snapshotParticlePositions() wants.
  // -1 on failure.
  int addSoftGrid(const double pos[3], const double quat[4],
                  int dimX, int dimY, int dimZ,
                  double cellX, double cellY, double cellZ,
                  double density, double kMu, double kLambda, double kDamp,
                  double particleRadius, int fixFlags, int *endOut = nullptr);

  // BUILD or RUN phase. The render surface of soft block `gridIndex`: the open
  // faces of its tet mesh, as tightly-packed int32 triangle triples in
  // BLOCK-LOCAL indices (0 .. particleEnd-particleStart-1), so they pair
  // directly with snapshotParticlePositions() over the same block's range.
  //
  // ⚠ Unlike a cloth sheet -- whose winding OmCloth re-derives analytically
  // from dimX/dimY -- this CANNOT be recomputed later: newton's ModelBuilder is
  // consumed at finalize(). The runtime snapshots it at authoring time. Call
  // this ONCE and upload as GL_STATIC_DRAW; only the positions stream per tick.
  //
  // Writes at most `maxTriangles` triples into `indices` (3 ints each) and
  // returns the number written, or -1 on failure. Pass indices == nullptr to
  // query the count without copying.
  int softSurfaceTriangles(int gridIndex, int *indices, int maxTriangles) const;

  // RUN phase. ONE packed readback of every particle's world-space position
  // per frame, deliberately shaped like snapshotBodyTranslations above: the
  // runtime returns raw bytes and this memcpy's them straight into the
  // caller's buffer, so a 1089-vertex cloth costs ONE FFI crossing per frame
  // rather than one per particle.
  //
  // ⚠ The stride is 3 floats (12 bytes), NOT the 4 that snapshotBodyTranslations
  // uses. That is on purpose and is the whole point of the difference: this
  // buffer is handed to wr_dynamic_mesh_add_vertex / glm::vec3, both of which
  // are tightly packed xyz, so a padded stride would force a per-vertex
  // repack on the render path every frame.
  //
  // The window is a HALF-OPEN RANGE [particleStart, particleEnd), not a max
  // count: the runtime slices it before packing, so a caller drawing one sheet
  // of several gets only its own particles and needs no staging buffer to
  // discard the rest. Pass (-1, -1) for "every cloth particle in the world".
  // `xyz` must hold 3 * (particleEnd - particleStart) floats.
  //
  // Returns the number of particles written (xyz[3*i + 0..2] = particle i's
  // world x/y/z), or -1 when Newton is not running or the call failed.
  int snapshotParticlePositions(int particleStart, int particleEnd, float *xyz) const;

  // RUN or BUILD phase. Total particles owned by cloth in the current world,
  // summed over every sheet. 0 when there is none -- the ordinary case for
  // every existing world -- and -1 when the runtime is unavailable.
  int particleCount() const;
  // W3.1 external-wrench injection (newton-ode-replacement-plan.md): queue a per-tick WORLD-frame force
  // (fx,fy,fz) + torque (tx,ty,tz) on a Newton body (about the body's reference point). step() sums these
  // into state.body_f and clears them after the tick -- ODE addBodyForce semantics (re-applied each tick).
  // Valid during simulation (no openForBuild). Returns 0/-1.
  int addBodyForce(int bodyIdx, double fx, double fy, double fz,
                   double tx, double ty, double tz);
  // W3.2 mid-step velocity set: write a Newton body's linear (angular=0) or angular (angular=1) velocity
  // straight into body_qd ([vx,vy,vz, wx,wy,wz], world frame). Persistent state (not re-applied per tick).
  // Valid during simulation. Returns 0/-1.
  int setBodyVel(int bodyIdx, double x, double y, double z, int angular);
  // W4.1/W4.2 native contact readback: snapshot this step's rigid contacts (filled by model.collide every
  // substep) into `out`. Replaces the ODE collision bridge as the contact source for the damage tracker +
  // sensors. Returns the contact count (>=0) or -1 if unavailable. World-frame points/normals.
  int getContacts(std::vector<OmNewtonContact> &out) const;
  // The Newton raycast service. Casts n rays against the live mjModel -- the
  // same physics the world steps, so hits are consistent with contacts by
  // construction. raysIn = flat [px,py,pz, dx,dy,dz, maxLen] * n (world frame,
  // direction need not be unit). excludeBodies/nExclude: Newton body indices
  // the rays pass through (the caster's own robot, per odeNearCallback's
  // same-robot rule). Fills out[0..n-1]; returns n on success, -1 when the
  // runtime is unavailable -- the caller then keeps the ODE ray path (the
  // keepalive stays until W4.3). One FFI crossing per call: batch per sensor
  // refresh, never per ray.
  int raycastBatch(int n, const double *raysIn, OmNewtonRayHit *out,
                   const int *excludeBodies = nullptr, int nExclude = 0) const;
  // Adds a 1-DoF revolute (hinge) joint between parent and child bodies.
  //   - axis (ax, ay, az): rotation axis in local space (length 1).
  //   - parentAnchor (parent_x/y/z): hinge attachment in parent's frame.
  //   - childAnchor (child_x/y/z):   hinge attachment in child's frame.
  //   - targetKe / targetKd: PD gains for joint actuation. Pass 0 to
  //     get a free-spinning hinge (no motor); pass non-zero to enable
  //     velocity targets via setJointTargetVelocity. Defaults are 0.
  // Both bodies must already exist via addBody. Returns the joint index
  // (>= 0) on success, -1 on error.
  // childRot* = the child link's authored rotation relative to its joint
  // parent (quaternion of R_child^T * R_parent), baked into the joint's
  // child_xform so a revolute constraint preserves the child Solid's
  // off-axis `rotation` instead of projecting it away. Identity default
  // keeps axis-aligned children (URDF wheels) byte-unchanged.
  int addJointRevolute(int parentIdx, int childIdx,
                       double ax, double ay, double az,
                       double parentAnchorX, double parentAnchorY, double parentAnchorZ,
                       double childAnchorX, double childAnchorY, double childAnchorZ,
                       double targetKe = 0.0, double targetKd = 0.0,
                       double limitLower = 0.0, double limitUpper = 0.0,
                       double effortLimit = 0.0, double velocityLimit = 0.0,
                       double childRotX = 0.0, double childRotY = 0.0,
                       double childRotZ = 0.0, double childRotW = 1.0);
  // Prismatic (linear/slider) joint -- e.g. parallel-gripper fingers. Same
  // queue/topo-sort/gain path as addJointRevolute; the slot it returns is
  // used with setJointTarget{Position,Velocity} exactly like a revolute.
  int addJointPrismatic(int parentIdx, int childIdx,
                        double ax, double ay, double az,
                        double parentAnchorX, double parentAnchorY, double parentAnchorZ,
                        double childAnchorX, double childAnchorY, double childAnchorZ,
                        double targetKe = 0.0, double targetKd = 0.0,
                        double limitLower = 0.0, double limitUpper = 0.0,
                        double effortLimit = 0.0, double velocityLimit = 0.0);
  // Hinge2 / universal joint -- 2-DoF rotation about two axes sharing one anchor (a caster's steer + roll,
  // or a car front wheel). Built natively as a Newton d6 joint with two FREE angular DoF (passive); the
  // capability gate admits it alongside Hinge/Slider (newton-ode-replacement-plan.md W2).
  int addJointHinge2(int parentIdx, int childIdx,
                     double ax1, double ay1, double az1,
                     double ax2, double ay2, double az2,
                     double parentAnchorX, double parentAnchorY, double parentAnchorZ,
                     double childAnchorX, double childAnchorY, double childAnchorZ);
  // Ball / spherical joint -- 3-DoF rotation about a shared anchor, zero relative translation (common in
  // legged/soft rigs). Built natively as a Newton ball joint (JointType.BALL: a quaternion-based spherical
  // constraint, gimbal-free, not a d6 Euler triple). PASSIVE -- see addJointBallMotorized below for the
  // driven variant; the capability gate admits either alongside Hinge/Slider/Hinge2
  // (newton-ode-replacement-plan.md W2.2).
  int addJointBall(int parentIdx, int childIdx,
                   double parentAnchorX, double parentAnchorY, double parentAnchorZ,
                   double childAnchorX, double childAnchorY, double childAnchorZ);

  // ---- MOTORISED Hinge2 / Ball (OMNISIM_NEWTON_BALL_HINGE2, default OFF) ----
  // The two verbs above register a PASSIVE constraint. These register the same
  // constraint with per-axis actuation, limits and ceilings, so a OmniSim
  // Hinge2Joint's motor/motor2 and a BallJoint's up-to-three RotationalMotors
  // actually drive the solver and their position sensors read back.
  //
  // Arrays are per-axis and packed in OmniSim' own axis order:
  //   gains   = (ke, kd) pairs   -- kd == 0 && ke == 0 leaves that axis FREE
  //   limits  = (lower, upper) pairs, lower == upper meaning "no limit"
  //   efforts / velLimits = one scalar per axis, <= 0 meaning "no ceiling"
  // childRot* is the child link's authored rotation relative to its joint
  // parent, exactly as in addJointRevolute.
  //
  // Hinge2: 2 axes -> gains[4], limits[4], efforts[2], velLimits[2].
  int addJointHinge2Motorized(int parentIdx, int childIdx,
                              const double axis1[3], const double axis2[3],
                              const double parentAnchor[3], const double childAnchor[3],
                              const double childRot[4],
                              const double gains[4], const double limits[4],
                              const double efforts[2], const double velLimits[2]);
  // Ball: 3 axes -> gains[6], limits[6], efforts[3], velLimits[3].
  // ⚠ parentQuat / childQuat, NOT axis vectors: the MuJoCo converter's ball
  // actuators are gear vectors in the JOINT frame, so the OmniSim axis triad
  // (axis, axis2, axis3) has to BE that frame. Pass parentQuat = quaternion of
  // the orthonormal basis [axis | axis2 | axis3] and childQuat = childRot *
  // parentQuat. Per-axis limits are accepted but NOT enforced by MuJoCo (its
  // ball element is emitted `limited: False`) -- the caller warns.
  int addJointBallMotorized(int parentIdx, int childIdx,
                            const double parentAnchor[3], const double parentQuat[4],
                            const double childAnchor[3], const double childQuat[4],
                            const double gains[6], const double limits[6],
                            const double efforts[3], const double velLimits[3]);
  // FIXED (0-DOF) tree joint welding childIdx rigidly to parentIdx at their
  // CURRENT registered poses (relative transform derived helper-side from the
  // builder's body_q, so no anchors cross the FFI). newton's MuJoCo conversion
  // keeps a FIXED-joint child as a separate WELDED mjc body with its own geoms
  // and no joint element -- the shape the force-TouchSensor un-fold needs so
  // contacts attribute to the sensor body and cfrc_int reads its mount wrench
  // (_scratch/design_weld_touch.md T1). Same queue/topo-sort path as
  // addJointRevolute. Returns the joint slot (>= 0) or -1.
  int addJointFixed(int parentIdx, int childIdx);

  // ---- Weld-slot API (Connector / VacuumGripper) -------------------------
  // _scratch/design_weld_touch.md. Slots are pre-allocated as INACTIVE MuJoCo
  // equality-weld constraints at build time and toggled at runtime --
  // mjModel.eq_* arrays are compile-time sized, so a slot that was never
  // allocated can never be locked. Phase 1 is CPU SolverMuJoCo only (the
  // default); on the GPU "mujoco_warp" solver engage/release warn once and
  // return -1 (never a silent no-op).

  // BUILD phase. Reserves one weld slot anchored on bodyIdx (an inactive
  // weld-to-world placeholder). Returns the slot id (>= 0) or -1.
  int addWeldSlot(int bodyIdx);
  // RUN phase. Activate `slot` welding bodyA <-> bodyB at their CURRENT
  // relative pose (zero-snap; the helper computes anchor + relquat from the
  // live MuJoCo state). bodyB < 0 welds bodyA to the WORLD; when bodyA < 0
  // the pair is swapped so obj1 is the real body (mirror of ODE's
  // dJointAttach body1==0 swap -- the caller tracks the inversion for the
  // rupture-force sign, exactly like OmConnector::mIsJointInversed). 0/-1.
  int weldEngage(int slot, int bodyA, int bodyB);
  // RUN phase. Deactivate `slot`. 0/-1.
  int weldRelease(int slot);
  // RUN phase. World-frame constraint wrench of an ACTIVE slot from the last
  // completed tick's solved efc rows: out[0..2] = force ON bodyA/obj1 (the
  // ODE dJointFeedback f1 analogue), out[3..5] = torque. Zeros when inactive
  // or before the first step. 0/-1.
  int weldForce(int slot, double out[6]) const;

  // RUN phase. ODE-f1-compatible mount wrench of a welded (fixed-joint,
  // un-folded) child body: the NEGATED MuJoCo cfrc_int (interaction force
  // with the parent, world-aligned axes, computed by mj_rnePostConstraint) --
  // negated because ODE's f1 is the force on the PARENT (node[0] of
  // OmTouchSensor's mount joint) while cfrc_int is the force ON the body.
  // out[0..2] force, out[3..5] torque (reference point: body COM, not the
  // ODE joint anchor -- only the force rows are parity-exact). The first call
  // registers the body with the per-tick snapshot; rows arrive from the next
  // tick. 0/-1.
  int touchForce(int bodyIdx, double out[6]) const;
  // Sets the per-step velocity target (rad/s) for a previously-added
  // revolute joint. Only honored by SolverXPBD when the joint was
  // created with non-zero targetKe. Idempotent across ticks; reset
  // by calling with vel=0. Returns 0/-1.
  int setJointTargetVelocity(int jointIdx, double vel);
  // Change-detected wrappers (physics-step-cost-optimization-plan.md §3 item
  // 5). The Python-side target dicts persist across ticks, so pushing a value
  // identical to the last one pushed for that joint is a pure FFI round trip.
  // These skip it. Exact double compare -- a controller that re-sends the same
  // number wants the same behaviour, and NaN (which compares unequal) always
  // pushes. The cache is per solver world; clearJointTargetCache() drops it.
  int setJointTargetVelocityIfChanged(int jointIdx, double vel);
  int setJointTargetPositionIfChanged(int jointIdx, double pos);
  void clearJointTargetCache();
  // Sets the per-step position target (rad) for a revolute joint with
  // target_ke > 0 (position-spring + damping config). Drives the
  // joint toward this angle each tick. 0/-1.
  int setJointTargetPosition(int jointIdx, double pos);
  // Sets a per-step raw joint TORQUE (Nm) via control.joint_f (applied
  // generalized force). Additive over the PD; for pure torque control build
  // the joint in EFFORT mode (OMNISIM_NEWTON_TORQUE_MODE). Re-send every tick.
  int setJointForce(int jointIdx, double tau);
  // Read the current Newton-tracked angle (rad) for a revolute slot.
  // 0.0 if the model hasn't been finalised. Used by OmBasicJoint's
  // position-control bridge -- the ODE-side hinge->position() doesn't
  // reflect Newton state, so the bridge has to query Newton directly.
  double getJointAngle(int jointIdx) const;

  // Batched IK PREVIEW against the live model (internal parity plan, item W2.1,
  // World.solve_ik). NOTHING MOVES: the solver owns its buffers and never
  // writes state_a. `linkBodyIdx` is the end effector's Newton body index
  // (OmSolid::nearestNewtonBodyIndex()); `targets` is 3*nTargets world-frame
  // doubles; `rotations` is nullptr or 4*nTargets [qx,qy,qz,qw]; `slots` are
  // the 1-coordinate joint slots to solve for (the caller MUST restrict to
  // the robot's own Hinge/Slider slots -- unmasked, newton's optimiser
  // "reaches" targets by moving bodies the caller cannot command);
  // `toolOffset` is nullptr or a 3-double TCP offset in the effector frame.
  // Fills anglesOut (nTargets*slots.size(), problem-major, clamped to the
  // authored limits) and residualsOut (nTargets, METRES -- measured by
  // forward kinematics on exactly the write-back vector, so the caller can
  // reject a target instead of driving to it). ⚠ First call per (world,
  // shape) compiles a warp kernel: seconds cold, ~150 ms warm (measured).
  // ⚠ Verified on the CPU "mujoco" solver only; unverified on mujoco_warp.
  // 0 on success, -1 otherwise.
  int solveIk(int linkBodyIdx, int nTargets, const double *targets, const double *rotations,
              const std::vector<int> &jointSlots, const double *toolOffset, int iterations, std::vector<double> &anglesOut,
              std::vector<double> &residualsOut) const;

  // ---- Multi-DoF variants (OMNISIM_NEWTON_BALL_HINGE2) -------------------
  // `dof` selects one degree of freedom of a joint slot: for a Hinge2 (d6)
  // dof 0 is axis1 and dof 1 is axis2. dof 0 is equivalent to the plain
  // setters/getter above, which stay the 1-DoF contract everything else uses.
  int setJointTargetVelocityDof(int jointIdx, int dof, double vel);
  int setJointTargetPositionDof(int jointIdx, int dof, double pos);
  int setJointForceDof(int jointIdx, int dof, double tau);
  double getJointAngleDof(int jointIdx, int dof) const;
  // A BALL joint's relative rotation as an (x, y, z, w) quaternion in the joint
  // frame -- newton stores a spherical joint's position as a quaternion, not as
  // three angles, so the caller decomposes it. Writes identity and returns -1
  // when unavailable. 0 on success.
  int getJointBallQuat(int jointIdx, double out[4]) const;
  // Warp the Newton-side body pose to the given world transform and
  // zero its velocity. Called when a Supervisor write hits the
  // Solid's translation/rotation field -- without this, ODE moves
  // but Newton stays put and the two diverge over many resets.
  void resetBodyPose(int bodyIdx, double x, double y, double z,
                     double qx, double qy, double qz, double qw);
  // Reset every revolute joint angle + velocity to the builder's
  // initial values, then eval_fk to propagate the change into
  // body_q. Companion to resetBodyPose: a chassis reset isn't useful
  // if the legs still hold their previous angles via joint_q.
  void resetJointsToDefaults();
  // Diagnostic: dump joint_q from both state_a and model so we can
  // see whether the solver is actually writing joint angles.
  std::string diagDumpJointQ() const;
  // Finalises the model: builder -> Model -> Solver + state ping-pong.
  // After this call, addBody / addShapeSphere are no-ops returning -1
  // until the next beginWorld(). 0/-1.
  int finalizeWorld();

  // World-teardown hook: releases the embedded Python World and clears
  // the running/openForBuild flags so the NEXT world load starts from a
  // fresh beginWorld(). Without this, a WORLD RELOAD (Ctrl+Shift+R) left
  // the process-singleton's `running` flag true -> the reloaded world's
  // ensureWorldOpen() no-opped, every addBody/addShape registration
  // silently failed, and the reloaded robots ran on ODE (a Newton-tuned
  // humanoid explodes within ~3 s there) while the ORPHANED old Newton
  // world kept stepping invisibly. Called by ~OmSimulationWorld.
  void teardownWorld();

  // Sets the per-world Newton solver preference (WorldInfo.newtonSolver)
  // BEFORE finalizeWorld(): "mujoco" -> SolverMuJoCo (robust frictional
  // contact, required for pinch grasps); "" / "auto" / "xpbd" -> default GPU
  // XPBD. Must be called during the build phase. 0/-1.
  int setSolverPreference(const std::string &name);

  // Declares ONE rigid body's visibility to the cloth solver
  // (Solid.newtonClothCoupling). mode is +1 (couple) or -1 (do not couple);
  // 0 means "unset" and must not be sent -- the runtime's roster is keyed on
  // presence, and an empty roster is what keeps a world that does not use the
  // field bit-identical to one built before the field existed.
  //
  // Narrows ONLY the SolverCoupledProxy roster on the coupled
  // (newtonSolver "mujoco+vbd") path. Inert on every rigid solver, and inert
  // on "vbd", where one SolverVBD owns everything and there is no proxy.
  // Build phase, before finalizeWorld(). 0/-1.
  int setClothCoupling(int bodyIndex, int mode);

  // Folds the WorldInfo.newtonSubsteps choice into the runtime BEFORE the
  // first step (default-flip-plan.md §4.2 N3): N>1 splits each tick into N
  // sub-steps so high closing-speed XPBD contact stays convergent, without a
  // launch env var. The OMNISIM_NEWTON_SUBSTEPS env var still overrides.
  // n<=1 is the unchanged single-step default. 0/-1.
  int setNewtonSubsteps(int n);

  // Plumbs the per-world MuJoCo friction-cone type + impratio (WorldInfo
  // newtonCone / newtonImpratio) into the runtime BEFORE finalizeWorld()
  // constructs SolverMuJoCo. "" / 0 keep MuJoCo stock (pyramidal, impratio 1);
  // the OMNISIM_NEWTON_CONE / OMNISIM_NEWTON_IMPRATIO env vars still win.
  // Cached + re-asserted by finalizeWorld() (same multi-build race as the
  // solver preference). 0/-1.
  int setContactCone(const std::string &cone, double impratio);

  // Plumbs the per-world MuJoCo contact dimensionality (WorldInfo
  // newtonCondim) into the runtime. 0 = unset -> the model keeps whatever
  // newton built, which is condim 3 (sliding friction only) on every geom of
  // every OmniSim world measured to date, so an existing world is
  // byte-identical. 4 adds torsional friction, 6 adds rolling. The
  // OMNISIM_NEWTON_CONDIM env var still wins. Cached + re-asserted by
  // finalizeWorld() (same multi-build race as the solver preference). 0/-1.
  int setContactCondim(int condim);

  // Plumbs the per-world MuJoCo NOSLIP post-solve iteration count (WorldInfo
  // newtonNoslipIterations -> mjOption.noslip_iterations) into the runtime.
  // 0 = unset == MuJoCo's own stock 0, so every existing world is
  // byte-identical. It is a friction-only Gauss-Seidel pass run AFTER the main
  // solve, and the thing it removes is the tangential drift that makes a pinch
  // creep out of a grasp while its normal force is held exactly where it was
  // commanded. The OMNISIM_NEWTON_NOSLIP env var still wins and is
  // VALUE-parsed, so =0 is the exact-revert hatch for a world that declares
  // the field. ⚠ CPU path only: mujoco_warp has no noslip field and its
  // put_model raises on a non-zero one, so the runtime declines there and
  // warns once instead of failing the build. Cached + re-asserted by
  // finalizeWorld() (same multi-build race as the solver preference). 0/-1.
  int setNoslipIterations(int iters);

  // Plumbs whether cloth/soft PARTICLES collide with EACH OTHER (WorldInfo
  // newtonClothSelfContact -> SolverVBD particle_enable_self_contact plus the
  // self-contact radius/margin derived from the authored particleRadius) to
  // the Python world BEFORE finalize(), which is when the VBD kwargs are
  // built. -1 = unset, leaving the runtime default (ON); 0 = off; 1 = on.
  // ⚠ Neither value is "correct": draping needs it ON (newton's own default is
  // off, and a fold then passes through itself in silence), grasping needs it
  // OFF (a pinched fold tracks -22.11 mm on vs -0.92 mm off, 24x, because the
  // gathered fabric pushes itself out of the jaws). The OMNISIM_CLOTH_SELF_
  // CONTACT env var still wins and is VALUE-parsed, so =1 exact-reverts a
  // world declaring 0. Cached + re-asserted by finalizeWorld() (same
  // multi-build race as the solver preference). 0/-1.
  int setClothSelfContact(int mode);

  // Plumbs the per-world contact friction / compliance / solver iteration
  // counts (WorldInfo newtonGroundMu / newtonContactKe / newtonContactKd /
  // newtonIterations / newtonLsIterations) into the runtime BEFORE
  // finalizeWorld(). 0 on any of them = unset, so every existing world is
  // byte-identical, and the matching OMNISIM_NEWTON_* env var still wins.
  //
  // These five were reachable ONLY from the process environment until
  // 2026-08-02, which made a .wbt an incomplete description of its own
  // physics: a tuned friction grasp could not be handed to another machine,
  // or to our own grader, because the file carried the scene and the shell
  // carried the contact model. Cached + re-asserted by finalizeWorld(). 0/-1.
  int setContactSolverParams(double mu, double ke, double kd, int iters, int lsIters);
  // Apply the cached m*Pref contact/solver values to the CURRENT Python world.
  // Called by ensureWorldOpen() right after the world is constructed and
  // BEFORE addGroundPlane() -- newton copies cfg.mu/ke/kd into shapes at add
  // time, so any later application reaches nothing (the 55-degree-ramp
  // defect: declared mu 2.0, box slid at the default).
  int applyContactSolverParamsToWorld();

  // Caches WorldInfo.coordinateSystem ("ENU" / "NUE" / "EUN") so the NEXT
  // beginWorld() can hand it to the runtime before anything is added. Pass the
  // field verbatim; the runtime resolves the up axis from where the "U" sits,
  // the same rule OmWorldInfo::updateGravityBasis() uses for mUpVector.
  //
  // WHY IT IS CACHED RATHER THAN PUSHED. OmSolid calls this at the HEAD of
  // flushPendingNewtonRegistrations, where no Newton world exists yet (rc -1 is
  // expected and fine) -- and it MUST be called there, because the world is
  // opened lazily from inside that function's registration loop. 0/-1.
  int setCoordinateSystem(const std::string &cs);
  // Apply the cached mCoordSystemPref to the CURRENT Python world. Called by
  // ensureWorldOpen() immediately after beginWorld() and BEFORE
  // addGroundPlane(): newton bakes builder.up_vector into the implicit ground
  // plane's NORMAL at add time, so an application after that point cannot move
  // the plane -- and cannot fix gravity either, since setWorldGravity()
  // projects the world's gravity vector onto that same up vector.
  int applyCoordinateSystemToWorld();

  // Refuse to run on ODE when the world asked for Newton and the INSTALLED
  // runtime would not come up. Called once the world's own backend choice is
  // known -- not at construction, which happens before the world is parsed.
  // Returns true if it refused (having logged a FATAL, which exits).
  static bool refuseIfBrokenAndNewtonWanted(bool newtonWanted);

  // Cumulative seconds spent inside the embedded step call, for
  // OMNISIM_NEWTON_STEP_PROFILE. Untouched when profiling is off.
  double mNewtonProfCallSec = 0.0;

  // Plumbs the per-world MuJoCo constraint-row / contact buffer caps
  // (WorldInfo newtonNjmax / newtonNconmax) into the runtime BEFORE
  // finalizeWorld() constructs SolverMuJoCo. 0 / 0 keep the runtime's built-in
  // 256, so every existing world is byte-identical; a positive value raises a
  // cap (measured: 32 constraint rows per 4WD Husky resting on its 8 wheel
  // contacts, so 10 robots peak at 320 and overflow 256 -- the kernel then
  // per-tick-printfs "nefc overflow"
  // and silently truncates the constraint vector); a negative value asks
  // newton for its own auto-estimate. The OMNISIM_NEWTON_NJMAX /
  // OMNISIM_NEWTON_NCONMAX env vars still win. Cached + re-asserted by
  // finalizeWorld() (same multi-build race as the solver preference). 0/-1.
  int setConstraintBuffers(int njmax, int nconmax);

  // Plumbs WorldInfo.gravity into the runtime BEFORE finalizeWorld().
  // Without this every Newton world ran at the ModelBuilder default -9.81
  // regardless of the world file (gravity-0 scenes fell onto the implicit
  // ground plane). The builder gravity is a scalar along up_vector; the
  // runtime projects the world vector onto it. 0/-1.
  int setWorldGravity(double gx, double gy, double gz);

  // Runtime-phase API (called every physics tick after the world is
  // finalised). 0/-1.

  // Advances the Newton solver by dt seconds.
  int step(double dt);
  // Reads the world-space pose of bodyIdx into xform[7]:
  //   xform = [x, y, z, qx, qy, qz, qw]
  // matching Newton's native body_q layout. Quaternion is [xyz, w] with
  // w as the scalar component (newton/warp convention). Returns 0/-1.
  int getBodyXform(int bodyIdx, double xform[7]) const;
  // Body spatial velocity from Newton: vel = [vx,vy,vz, wx,wy,wz] in WORLD
  // frame (matches OmniSim getVelocity()). Returns 0/-1. Without this,
  // getVelocity() reads 0 for Newton-backed solids.
  int getBodyVelocity(int bodyIdx, double vel[6]) const;

  // R3.7b: bulk snapshot of every registered body's translation as
  // 4-float (xyzw, with w = 0 padding) records. Writes up to
  // maxBodies records into `xyzw`; returns the number of bodies
  // actually written (= min(maxBodies, currently registered)) on
  // success, -1 if Newton isn't running or the call into Python
  // failed. The output is laid out tightly: out[4*i+0..2] = body i's
  // (x, y, z), out[4*i+3] = 0.0. Layout matches
  // OmWgpuRenderTarget::clearAndDrawInstanced's `bodyOffsetsXyz0`
  // parameter so the future R3.4-step-4 Camera scene walk can pass
  // the buffer through unchanged.
  //
  // Single Python call + numpy slice — costs one GPU->CPU transfer
  // of body_q regardless of body count (and that transfer is
  // already cached per-step by the body_xform path).
  int snapshotBodyTranslations(int maxBodies, float *xyzw) const;

  // ---- OmPhysicsBackend dispatcher overrides ---------------------------
  //
  // Bridge between the abstract opaque OmBodyHandle interface and Newton's
  // integer-index API. Body handles for Newton-backed solids are packed as
  // (void*)(uintptr_t)(idx + 1) so a NULL handle is clearly invalid (idx=0
  // is a valid Newton body, so the +1 offset keeps 0 reserved). Callsites
  // that go through the registry and resolve to this backend get full
  // polymorphism for the read methods below; write paths stay on the
  // Newton-specific API for now because the abstract write methods don't
  // map 1:1 (Newton drives via joint targets, not direct body force/pose
  // writes outside of resetBodyPose).
  //
  // Newton functionally overrides 7 of OmPhysicsBackend's ~42 dispatcher
  // operation virtuals (the six body/joint reads below + reset); the
  // remaining ~35 -- every write-side virtual -- inherit the -1 default,
  // which callers treat as "this backend doesn't support that op" -- correct
  // for Newton's design (it drives writes via its own integer-index API).
  static OmBodyHandle handleFromIndex(int idx);
  static int indexFromHandle(OmBodyHandle h);

  int getBodyPosition(OmBodyHandle body, double pos[3]) const override;
  int getBodyQuaternion(OmBodyHandle body, double q[4]) const override;
  int getBodyLinearVel(OmBodyHandle body, double v[3]) const override;
  int getBodyAngularVel(OmBodyHandle body, double v[3]) const override;
  // Point velocity computed from body-origin state: v_point = v + omega x r,
  // with r = point - body_origin (both in world frame). Newton doesn't expose
  // a native point-velocity primitive, so this stages through getBodyXform +
  // getBodyVelocity. Two Python round-trips per call; sensors call this once
  // per tick, not per inner step.
  int getBodyPointVel(OmBodyHandle body, const double point[3], double v[3]) const override;
  // P1.6 hinge-angle override: routes the abstract joint-handle read to
  // the existing integer-index getJointAngle(idx). Joint handles share
  // the body-handle packing (idx+1 in a void*) so indexFromHandle does
  // the unpack. Rate intentionally inherits the -1 default -- Newton
  // doesn't expose joint angular rate today, and the post-physics-step
  // rate-based normalisation in OmHingeJoint is ODE-only anyway (Newton
  // reads angle from joint_q directly without renormalising).
  int getJointHingeAngle(OmJointHandle joint, double *angleOut) const override;

  // P7 reset hook: called by OmSimulationWorld::reset after the
  // per-Solid Newton-sync cascade has fired. Re-runs eval_fk so the
  // articulation body_q reflects the freshly-zeroed joint_q values
  // for any Solids whose syncNewtonPoseFromFields didn't get called
  // (e.g. fixed-base robots where the chassis didn't move, so no
  // positionChangedArtificially signal fires). Cheap, idempotent.
  int reset() override;

private:
  // Set true at construction iff:
  //   - OMNISIM_WITH_NEWTON is defined (build flag was ON), AND
  //   - the Newton/Warp runtime was successfully initialised, AND
  //   - the FFI smoke check + helper-module import succeeded.
  bool mAvailable;

  // Owns references to the process-cached helper module/class + the
  // `World` instance for the current world). Allocated when the
  // backend brings Newton up; freed when the backend is destroyed or
  // beginWorld() rolls a fresh one.
  OmNewtonRuntimeState *mRuntime;

  // Sticky copy of the last WorldInfo.newtonSolver request. The GUI
  // rebuilds the world several times per load and each rebuild rolls a
  // fresh Python `world` whose `_solver_pref` resets to None, so a build
  // that finalizes before the flush re-applies the preference would
  // silently fall back to XPBD. finalizeWorld() re-asserts this cached
  // value onto the world it is about to finalize, making the choice
  // build-order-independent.
  std::string mSolverPref;
  // Sticky per-world MuJoCo cone/impratio (WorldInfo newtonCone /
  // newtonImpratio), cached by setContactCone() and re-asserted by
  // finalizeWorld() for the same build-order-independence as mSolverPref.
  // "" / 0.0 = MuJoCo stock defaults.
  std::string mConePref;
  double mImpratioPref = 0.0;
  // Sticky per-world MuJoCo contact dimensionality (WorldInfo newtonCondim),
  // cached by setContactCondim() and re-asserted by finalizeWorld(). 0 = unset
  // (leave the model's own condim, which is 3 everywhere today).
  int mCondimPref = 0;
  // Sticky per-world MuJoCo noslip iteration count (WorldInfo
  // newtonNoslipIterations), cached by setNoslipIterations() and re-asserted
  // by finalizeWorld(). 0 = unset (MuJoCo's own stock 0: the pass is off).
  int mNoslipItersPref = 0;
  // Sticky per-world cloth particle self-contact mode (WorldInfo
  // newtonClothSelfContact), cached by setClothSelfContact() and re-asserted
  // by finalizeWorld(). -1 = unset (the runtime's own default, which is ON).
  int mClothSelfContactPref = -1;
  // Sticky per-world MuJoCo constraint-row / contact buffer caps (WorldInfo
  // newtonNjmax / newtonNconmax), cached by setConstraintBuffers() and
  // re-asserted by finalizeWorld() for the same build-order-independence as
  // mSolverPref. 0 / 0 = unset (the runtime's built-in 256).
  int mNjmaxPref = 0;
  int mNconmaxPref = 0;
  // Sticky per-world contact friction / compliance / iteration counts
  // (WorldInfo newtonGroundMu / newtonContactKe / newtonContactKd /
  // newtonIterations / newtonLsIterations), cached by
  // setContactSolverParams() and re-asserted by finalizeWorld(). 0 = unset.
  double mGroundMuPref = -1.0;  // negative = unset (0 means frictionless)
  double mContactKePref = 0.0;
  double mContactKdPref = 0.0;
  int mIterationsPref = 0;
  int mLsIterationsPref = 0;
  // Sticky WorldInfo.coordinateSystem ("ENU" / "NUE" / "EUN"), cached by
  // setCoordinateSystem() and applied by ensureWorldOpen() to the world it just
  // constructed -- BEFORE the implicit ground plane, which is the only moment
  // the up axis can still be chosen. Empty = never declared -> the runtime keeps
  // its constructed z-up default, which is what ENU resolves to anyway.
  //
  // Unlike the other sticky prefs there is NO finalizeWorld() re-assert: by
  // finalize the ground plane's normal is already baked, so a late assignment
  // would report an axis the geometry does not have. Correctness here comes from
  // ensureWorldOpen() being the only caller of beginWorld().
  std::string mCoordSystemPref;

  // internal parity plan, item W1.1. True iff the solver that ACTUALLY built for the
  // current world is the GPU "mujoco_warp" one (read from _solver_kind at
  // finalize, so it reflects the effective solver, not the requested one).
  //
  // newton's SolverMuJoCo::step() writes mj_data only on its `use_mujoco_cpu`
  // branch; the GPU branch steps mjw_data on the device and leaves mj_data
  // FROZEN AT THE BUILD POSE. mj_data still exists there -- put_data() seeds
  // the GPU copy from it -- so an unguarded reader does not fail, it answers
  // against the scene as authored at t=0. The runtime declines those reads
  // (World._gpu_readback_declined); this flag exists so the engine can SAY SO
  // in the log, which the runtime cannot: on Windows omnisim-bin.exe is a
  // GUI-subsystem binary, so the embedded interpreter's stdout is discarded.
  //
  // The two warn-once flags are per WORLD, not per process (finalizeWorld
  // clears them), because a harness session loads many worlds in one process
  // and a once-per-process warning is exactly the silent-second-time failure
  // this whole item is about.
  bool mSolverIsMuJoCoWarp = false;
  mutable bool mWarnedGpuRays = false;
  mutable bool mWarnedGpuContacts = false;
  void warnGpuReadbackOnWarp(bool rays) const;

  OmNewtonBackend(const OmNewtonBackend &) = delete;
  OmNewtonBackend &operator=(const OmNewtonBackend &) = delete;
};

#endif
