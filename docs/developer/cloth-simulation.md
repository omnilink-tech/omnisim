# Cloth simulation

*Deformable cloth on newton's `SolverVBD`, coupled to the rigid `SolverMuJoCo` world.*

**Status (2026-08-16): the drape path works, and cloth GRASPING is now demonstrated and
measured against a negative control.** A sheet parses, registers particles, falls,
self-collides, drapes over rigid geometry and renders. Since `8888649ec` a gripper also
picks fabric up and holds it, on two different assets:

| world | asset | tracking err | slip max | negative control |
|---|---|---|---|---|
| [`newton_vbd_cloth_grasp.omniworld`](../../projects/samples/demos/worlds/physics/newton_vbd_cloth_grasp.omniworld) | flat patch, 399 particles | **−0.92 mm** | 2.76 mm | −249.67 mm (jaws never close) |
| [`newton_vbd_tshirt_grasp.omniworld`](../../projects/samples/demos/worlds/physics/newton_vbd_tshirt_grasp.omniworld) | T-shirt, 616 particles | **−1.50 / −1.59 mm** (front panel) | 4.17 / 5.14 mm | −173.06 / −115.07 mm (jaws open) |

Tracking error is (rise of the gripped region) − (rise of the measured jaw midpoint). A
second, independent instrument — commanded-vs-measured jaw gap, read from jaw body poses
rather than particles — corroborates it and cannot be confounded by which particles are
labelled "gripped": 28.4 mm measured against 26.00 mm commanded on a hold, exactly 26.00 mm
on the miss. The "fabric is merely draped over the pad like a ledge" objection is ruled out
numerically: a ledge would sit ~30 mm above pad centre, and the measured offset is
+1.26–4.30 mm.

⚠️ **Three disclosures that must travel with any public grasp claim.**
1. The garment's shoulders / the patch's top edge are **PINNED**, so these are *tracking*
   numbers, not *load-bearing* ones — the fabric is not hanging from the jaws by its own
   weight. Pinned-set drift is 0.00e+00 and no value is non-finite.
2. The negative control is not contact-free — its open jaws plough through the garment and
   drag it 77–135 mm. It bounds the "did the jaws do anything" question, not "did the jaws
   touch anything".
3. **Self-contact must be OFF to grasp** (`newtonClothSelfContact FALSE`) and ON to drape.
   There is no correct default: leaving it on costs 24× on tracking error (−0.92 mm → −22.11 mm).

Not every grasp target holds. Closing on the **hem edge** misses (−47.20 / −57.14 mm, jaw gap
26.00 mm = closed on nothing), and that miss is reported as a miss. What is **not** yet
demonstrated is the composed **fold** — see §6 "The composed FOLD is not demonstrated" before
quoting `newton_tshirt_fold.omniworld` as working.

Read this alongside [`resources/nodes/Cloth.wrl`](../../resources/nodes/Cloth.wrl), which is
the authored field contract and carries several mechanisms in its comments that are not
re-derivable from the code.

---

## 0. The one thing that is load-bearing

```
WorldInfo { newtonSolver "mujoco+vbd" }
```

Declare it. It is what the field contract tells authors to write, it is the value in the
schema's enum ([`resources/nodes/WorldInfo.wrl:36`](../../resources/nodes/WorldInfo.wrl)), and it
states the world's intent to anyone reading the file.

⚠️ **But it is NOT what makes cloth simulate, and this section said otherwise until 2026-08-16.**
The real gate is **the presence of cloth particles**, not the declared string. The runtime
dispatches on `has_cloth()`
([`omnisim_newton_runtime.py:5185`](../../src/omnisim/physics/omnisim_newton_runtime.py#L5185)):

```python
if self.has_cloth():
    self._build_cloth_coupled_solver(_kw)   # SolverMuJoCo + SolverVBD over one Model
else:
    self.solver = newton.solvers.SolverMuJoCo(self.model, **_kw)
```

**Measured** (OmniBench lane 4, 2026-08-16): the lane's cloth probe world declares
`newtonSolver "mujoco"` and still logged `registered 441 particles` and finalised on
`MuJoCo (mujoco_warp, WorldInfo.newtonSolver) + VBD cloth via SolverCoupledProxy`. Only `"vbd"`
— the single-solver path — branches differently.

Why the old claim was believable: a `Cloth` that fails to register logs a warning
([`OmCloth.cpp:714`](../../src/omnisim/nodes/OmCloth.cpp#L714)) naming `"mujoco+vbd"` as *"the
usual cause"*. That text is a **heuristic guess at the cause, not a description of a gate** —
it fires on `mParticleStart < 0` for any reason. Treating it as the mechanism is how a
diagnostic hint became a documented rule. ⚠️ The warning text is itself now misleading and
should be reworded; it is unchanged as of this writing.

Still true and still load-bearing: a frozen sheet and a correctly-placed-but-unsimulated sheet
look identical in the viewport, so the programmatic check is what you rely on —
`OmCloth::isSimulated()` ([`OmCloth.hpp:95`](../../src/omnisim/nodes/OmCloth.hpp)), which is
just `mParticleStart >= 0`. A world with **no** `Cloth` node has no reason to set `"mujoco+vbd"`.

---

## 1. The `Cloth` node

A rectangular patch of fabric. Geometry is a grid of `dimX × dimY` **cells**, therefore
`(dimX + 1) × (dimY + 1)` **particles**, spanning `dimX*cellX` by `dimY*cellY` metres, lying
flat in its own local Z = 0 with its **(0,0) corner** — not its centre — at `translation`.

| field | type | default | meaning |
|---|---|---|---|
| `translation` | SFVec3f | `0 0 1` | world position of the patch's local (0,0) **corner** |
| `rotation` | SFRotation | `0 0 1 0` | world orientation (axis + angle in rad) |
| `dimX` | SFInt32 | `20` | cells along local X (≥ 1); particles along X = `dimX + 1` |
| `dimY` | SFInt32 | `20` | cells along local Y (≥ 1); particles along Y = `dimY + 1` |
| `cellX` | SFFloat | `0.05` | cell size along local X (m) |
| `cellY` | SFFloat | `0.05` | cell size along local Y (m) |
| `mass` | SFFloat | `0.001` | **PER-PARTICLE** mass (kg). Default 21×21 ⇒ ~0.44 kg total |
| `particleRadius` | SFFloat | `0` → `0.01` | per-particle collision thickness (m) |
| `triKe` | SFFloat | `0` → `1e5` | in-plane distortion (stretch **and** shear jointly) |
| `triKa` | SFFloat | `0` → `= triKe` | area / dilation ("resists ballooning") |
| `triKd` | SFFloat | `0` → `1e-2 * triKe` | triangle damping. Raise to kill jitter/ringing |
| `edgeKe` | SFFloat | `0` → `0.01` | bending across edges. Raise for canvas, lower for silk |
| `fixLeft` | SFBool | `FALSE` | pin the local **−X** edge |
| `fixRight` | SFBool | `FALSE` | pin the local **+X** edge |
| `fixBottom` | SFBool | `FALSE` | pin the local **−Y** edge |
| `fixTop` | SFBool | `TRUE` | pin the local **+Y** edge |
| `diffuseColor` | SFColor | `0.8 0.2 0.2` | engine-owned phong diffuse. **Used only when `appearance` is NULL** |
| `appearance` | SFNode | `NULL` | `Appearance` \| `PBRAppearance` — the same material node a `Shape` takes (§3.1) |
| `castShadows` | SFBool | `TRUE` | silhouette recomputed per light per frame |

### Field semantics worth stating explicitly

**Pinning is in the patch's LOCAL frame.** `fixLeft`/`fixRight` are the low/high ends of local
X, `fixBottom`/`fixTop` of local Y. **None of them is a world up/down claim** — a patch rotated
to hang vertically still pins along its own axes. A pinned edge's particles get **zero mass and
stop integrating**, so the patch hangs from them. `fixTop TRUE` is the default precisely so the
default patch hangs and droops instead of dropping flat.

**A vertical curtain is a rotated patch, not a different node.**

**`0` means "leave it to the runtime" on all five tunables** — the same convention the
`WorldInfo.newton*` fields use. ⚠ The consequence is that **a genuinely zero stiffness cannot
be authored**; set a tiny value instead.

⚠ **Two different "unset" conventions meet at the C++/Python seam, and the translation between
them is load-bearing** ([`OmCloth.cpp:237-254`](../../src/omnisim/nodes/OmCloth.cpp)):

- `.wrl` side: `0` = "runtime decides".
- runtime side: **negative** = "derive from the matching ke"; **zero is a literal zero
  stiffness** and would silently produce a sheet with no stretch resistance at all.

So an unset field is never forwarded as `0`. The two derivable ones (`triKa`, `triKd`) go out as
`-1`; the three with nothing to derive from (`triKe`, `edgeKe`, `particleRadius`) go out as the
runtime's own default literal. **Those three literals are duplicated in
[`OmCloth.cpp:249-254`](../../src/omnisim/nodes/OmCloth.cpp) and
[`omnisim_newton_runtime.py:983-984`](../../src/omnisim/physics/omnisim_newton_runtime.py) and
must be kept in step.**

**`edgeKd` is always derived and is not exposed as a field yet**
([`OmCloth.cpp:253`](../../src/omnisim/nodes/OmCloth.cpp)).

**What the stiffness terms actually mean**, per the VBD kernel, which is authoritative — an
upstream newton comment calling `tri_ka` "shear stiffness" is **wrong**
([`omnisim_newton_runtime.py:1009-1015`](../../src/omnisim/physics/omnisim_newton_runtime.py)).
`triKe` covers stretch and shear jointly; `triKa` is the area/dilation term.

**Parameters that are INERT under `SolverVBD`** and therefore deliberately not exposed:
`tri_drag`/`tri_lift` (aerodynamics), `particle_ke/kd/kf/mu`, `soft_contact_kf`,
`soft_contact_restitution`. VBD reads `model.soft_contact_ke/kd/mu` for **both** shape contact
and self-contact — there is no separate self-contact material — so those three are the whole
contact-material story ([runtime:1016-1023](../../src/omnisim/physics/omnisim_newton_runtime.py)).

### Pose is NOT inherited

Particle positions come back from Newton in **world space**, so the WREN transform is parented
to the scene root at identity — the same choice `OmGranularGroup` makes, for the same reason.
**A `Cloth` nested under a `Pose`/`Group` therefore ignores that ancestor**
([`OmCloth.hpp:44-49`](../../src/omnisim/nodes/OmCloth.hpp)). It is registered as
top-level-insertable for exactly this reason.

---

## 2. Architecture

```
                    ONE newton Model
   ┌──────────────────────────┴──────────────────────────┐
   │  entry "mjc"                      entry "vbd"       │
   │  SolverMuJoCo                     SolverVBD         │
   │  owns ALL bodies + ALL joints     owns ALL particles│
   └──────────────────────────┬──────────────────────────┘
                    SolverCoupledProxy
        (mjc → vbd proxy bodies, mode="lagged")
```

Built by
[`_build_cloth_coupled_solver`](../../src/omnisim/physics/omnisim_newton_runtime.py) at
`omnisim_newton_runtime.py:2735`.

**Why not put everything on VBD** (which does simulate rigid bodies)? Because OmniSim's working
grasp is built on MuJoCo joint features VBD does not implement: the `effortLimit * 10` position
PD servo every Newton joint is constructed with (see
[docs/guide/friction-grasp.md](../guide/friction-grasp.md)), armature, and the
`POSITION_VELOCITY` target mode. **Moving the robot to VBD would silently change every actuator
in the tree.** So the robot keeps the *exact* `SolverMuJoCo` it would have had — same
`impratio`, cone, iterations, `njmax`/`nconmax`.

**How the coupling works, because the failure modes follow from it:** the `mjc` entry owns every
body and joint; the `vbd` entry owns only particles; a Proxy mapping hands VBD virtual proxy
bodies mirroring MuJoCo's. Each step VBD solves the cloth against those proxies, and the contact
impulses it computes are fed back to MuJoCo as forces (`mode="lagged"`: begin-poses and
end-velocities are synced, then lagged feedback is prepared so it is not double-counted).
**The robot feels the cloth without MuJoCo ever knowing particles exist.**

Wiring follows upstream newton's `example_proxy_joint_gripper.py` and
`example_mujoco_vbd_coupled_solver.py`.

### Ownership is a trap in BOTH directions

From `SolverCoupled._build_owner_map` / `_build_entries`
([runtime:2966-2988](../../src/omnisim/physics/omnisim_newton_runtime.py)):

- An index owned by **no** entry is **FROZEN** in every view — inverse mass zeroed, shapes
  stripped of collide flags — **with no warning**. A body left out would simply stop moving.
- But if **no** entry declares **any** body, the disabling pass is skipped wholesale and every
  entry integrates every body: **silent double integration, i.e. gravity applied twice**.

Both are avoided by declaring the *full* body and joint sets on `mjc` and the *full* particle
set on `vbd`, and the runtime **asserts** exactly that rather than trusting the ranges — an
unowned particle would give the sheet an invisible dead patch.

There is also an **index-identity check** ([runtime:3068-3088](../../src/omnisim/physics/omnisim_newton_runtime.py)):
everything that reads `mj_model`/`mj_data` (raycast, welds, `TouchSensor`, the pose check, the
contact readback) indexes with **parent-model ids**, which is only sound while the `mjc`
`ModelView` is an identity compaction of the parent. A future entry taking bodies away from
`mjc` would make every one of those readbacks answer about the wrong body, silently. The runtime
logs a loud warning if the body counts ever disagree.

### Rigid contact ownership — the silent-contactless-world trap

⚠ **`use_mujoco_cpu=True` + `use_mujoco_contacts=False` is a world with NO CONTACTS AT ALL, and
nothing says so.** Source-verified in the vendored newton 1.5.0: `SolverMuJoCo.step()` injects
the newton `Contacts` object into MuJoCo on its **GPU branch only**; the CPU branch calls
`mj_step` and never looks at `contacts`. With MuJoCo's own detection off and newton's never
delivered, the robot falls through the floor. newton's constructor rejects only
`enable_sleeping and not use_mujoco_contacts` — **this combination it accepts silently.**

Three configurations, of which the third is refused
([runtime:2771-2840](../../src/omnisim/physics/omnisim_newton_runtime.py)):

| | config | status |
|---|---|---|
| **A** | MuJoCo keeps its own contact detection (default) | **DEFAULT, and the one measured to work** |
| **B** | `use_mujoco_contacts=False`, newton's pipeline is the single source | **OPT-IN** via `OMNISIM_CLOTH_NEWTON_CONTACTS=1`; **forces `mujoco_warp`** |
| **C** | `use_mujoco_contacts=False` while staying on CPU `mj_step` | **REFUSED — raises** |

(B) is upstream's own shape, but every upstream multiphysics example also runs the GPU branch
(`use_mujoco_cpu` defaults to `False` upstream and none of them overrides it). Selecting (B)
therefore forces `mujoco_warp` — and **AGENTS.md scopes OmniSim's bitwise determinism claim to
the CPU `mj_step` path and records it REFUTED on `mujoco_warp`** (0 of 24 same-config cold pairs).
So (B) buys upstream-parity contact handling at the cost of determinism, which is why it is not
the default. See [determinism-scope.md](../benchmarks/determinism-scope.md).

**Measured** (recorded at [runtime:2792-2799](../../src/omnisim/physics/omnisim_newton_runtime.py),
RTX 3060 laptop, newton 1.5.0) — the check that (A) does not double-resolve rigid contact: two
boxes placed 20 m from the cloth so it cannot touch them rest at 0.099892244 / 0.149892256
rigid-only vs 0.099892229 / 0.149892226 coupled — **a 1.5e-08 m difference, float noise.**
Meanwhile a box *under* the sheet is pressed from 0.099892 to 0.098761 by its weight, which is
the coupling doing its job.

⚠ **And never `disable_contacts=True`.** Upstream's `example_proxy_joint_gripper.py:118` passes
it; copying that here would be a mistake — it sets `mjDSBL_CONTACT` and `nconmax=0`, so MuJoCo
resolves no contact whatsoever. It is harmless in that example only because its scene's sole
contact *is* the soft block: no floor, no table. **Every OmniSim world has a floor.**

### Collision detection is a `CollisionPipeline`, and on a cloth world it IS the detection

OmniSim's rigid path lets `SolverMuJoCo` run its own collision. Particles have no such
shortcut: `SolverVBD` reads the `soft_contact_*` arrays a `CollisionPipeline` writes, and
nothing else produces them
([`_build_cloth_collision_pipeline`, runtime:2682](../../src/omnisim/physics/omnisim_newton_runtime.py)).

`enable_rigid_soft_full_surface_contact=True` (default on) is the water-tight rigid–soft pass:
it adds soft **edge** and **face** records on top of the per-vertex ones, so a rigid feature
passing *between* two cloth vertices is still caught. Upstream's
`example_vbd_gripper_soft_grid.py` exists to demonstrate that a grid whose only feature crossing
the jaws is an interior diagonal edge **is gripped with the flag and slips out without it** —
for a coarse sheet against a thin gripper this is the difference between a grasp and a drop, and
it is **not recoverable at runtime** because the flag *sizes* the soft-contact buffer at
construction.

⚠ It needs a **volume SDF** on every participating mesh/convex rigid shape. OmniSim's bounding
objects are mostly analytic primitives (which ignore it), but URDF collision meshes are not, and
**nothing in this runtime provisions SDFs today.** If construction refuses, the runtime falls
back to the per-vertex pass **and logs that it did** — a silently degraded grip that drops the
sheet is exactly the failure this is written against.

### Pure-cloth worlds

A world with **zero rigid bodies** runs `SolverVBD` alone, no coupling
([runtime:2946-2961](../../src/omnisim/physics/omnisim_newton_runtime.py)). The
`SolverCoupledProxy.Proxy` validation requires a mapping with at least one body/particle/joint,
and a coupled solver with nothing to couple is pure overhead. ⚠ On such a world the
**mj_model-backed features are unavailable**: raycast sensors, welds, `TouchSensor`, joint
readback. The runtime logs this.

### Device: cloth FORCES cuda

⚠ **A world carrying cloth finalizes on CUDA even though its robot still steps through CPU
`mj_step`** ([runtime:3500-3531](../../src/omnisim/physics/omnisim_newton_runtime.py)). This is a
deliberate inversion of the CPU-device pin that bought the engine 2.1–3.6× (see
physics-step-cost-optimization-plan.md (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md))), and it is the
only place in the runtime where the two solvers want different devices.

Reason: `SolverVBD` is a warp solver and its per-particle Gauss-Seidel sweeps are the whole cost.
**Measured at 289 particles: CPU 6.7 fps vs CUDA + CUDA-graph 164 fps** — a 24× swing that
dwarfs the readback saving the pin buys back, and 6.7 fps is not a simulation anyone can drive.

An explicit `OMNISIM_NEWTON_MODEL_DEVICE=cpu` still wins (that is what an override is for) — but
it is the slow configuration and the log says so.

### CUDA-graph capture is usually REFUSED on cloth worlds

([`_cloth_graph_ok`, runtime:5487](../../src/omnisim/physics/omnisim_newton_runtime.py))

Arming graph capture on a cloth world whose `mjc` entry is the CPU `mj_step` solver aborts with
`Warp CUDA error 906: operation would make the legacy stream depend on a capturing blocking
stream`. The reason is **structural, not a bug to chase**: `use_mujoco_cpu` means the solver
copies state GPU→host, runs `mj_step` on the CPU, and copies back **every substep**, and a
device-to-host memcpy cannot be recorded into a CUDA graph. The old failure handling caught it
and fell back, so runs were *correct* — but printed two CUDA errors per world and paid a
pointless capture attempt, and *"it errors and then works"* is exactly the shape that gets
mis-diagnosed later. It is now refused up front, once, with a reason.

So: **the 164 fps figure belongs to the `mujoco_warp` configuration.** A CPU-`mjc` cloth world
still gets the device pin's win (CUDA VBD vs CPU VBD) without the graph on top.
`OMNISIM_CLOTH_FORCE_GRAPH=1` re-arms it to re-measure.

---

## 3. Rendering

Cloth geometry changes every frame, so it does **not** go through `OmIndexedFaceSet`/`MFVec3f` —
that path rebuilds the whole WREN mesh on every per-element field change, i.e. one full rebuild
per vertex per frame. Instead it uses `WrDynamicMesh` directly, the mechanism `OmMuscle` and
`OmTrack` already use ([`OmCloth.hpp:35-42`](../../src/omnisim/nodes/OmCloth.hpp)):

- **indices uploaded ONCE** as `GL_STATIC_DRAW` (cloth topology is fixed);
- **coords + normals re-streamed per frame** as `GL_STREAM_DRAW`.

`OmCloth::animateMesh()` ([`OmCloth.cpp:393`](../../src/omnisim/nodes/OmCloth.cpp)) is the
per-frame entry point, called by `OmWrenVertexArrayFrameListener` exactly as
`OmMuscle::animateMesh` is. It reads the frame's particle positions back from Newton in **one
packed FFI crossing** (`OmNewtonBackend::snapshotParticlePositions`,
[`OmNewtonBackend.hpp:301`](../../src/omnisim/physics/OmNewtonBackend.hpp)) into a tightly-packed
xyz float buffer — no repack on either side — then recomputes per-vertex normals by
area-weighted accumulation and re-streams.

**Triangle winding mirrors newton's `add_cloth_grid` exactly** (two triangles per cell,
`v0/v1/v3` and `v1/v2/v3`, over a row-major Y-outer/X-inner vertex layout). That is what
guarantees the rendered surface and the simulated particles cannot drift out of correspondence
([`OmCloth.hpp:126-131`](../../src/omnisim/nodes/OmCloth.hpp)).

There is **no `Shape`/`IndexedFaceSet` child to author and none is accepted** — the *geometry* is
engine-owned, as for `GranularGroup`. The *material* is not: see §3.1.

### 3.1 `appearance` — the material is the same one every `Shape` uses

A Cloth takes an `Appearance` or `PBRAppearance` child in its `appearance` field and hands it
straight to `OmAbstractAppearance::modifyWrenMaterial()` — the identical call
`OmShape::applyMaterialToGeometry` makes. So a garment gets `baseColorMap`, `roughnessMap`,
`metalnessMap`, `normalMap`, `occlusionMap`, `emissiveColorMap`, `textureTransform` and the sky's
irradiance cubemap with no cloth-specific code at all. Leaving the field NULL keeps the original
engine-owned Phong material driven by `diffuseColor`, so **no existing world changes**.

⚠ **This is not a cosmetic upgrade, and the reason is the lighting recipe rather than the
material.** `OmniSimSun.proto` declares `ambientIntensity 0`, so a world's only fill light is
image-based — and image-based light reaches a surface **only through a PBR material**. A Phong
Cloth therefore renders at exactly `(0, 0, 0)` on every part of the fabric the sun does not hit
directly, which on a draped garment is most of it. MEASURED on
`newton_vbd_tshirt_grasp.omniworld` (machine `9722d23d12a3`), fraction of the frame at pure
black, close shot: **34.97 %** on the Phong material; **0.54 %** on an equivalent
`PBRAppearance` shot from the same camera in `newton_cloth_material.omniworld`.

⚠ **Do not read that as "PBR fixes the black" in general.** The same measurement says the black
is *engine-wide*, not a cloth defect: a plain `PBRAppearance` `Box` suspended in open sky
measures `(0, 0, 0)` on its underside too, and three identical plates at `IBLStrength` 1 / 5 / 25
stay black under the default `atmosphericSky "earth"` while the same three light up correctly
under `atmosphericSky ""`. The procedural sky's baked irradiance cubemap contributes ~nothing;
that is a `OmBackground`/`OmWrenAtmosphericSky` matter, not a `Cloth` one. `newton_cloth_material`
works around it with two dim, shadowless fill `DirectionalLight`s and says so in its header.

**Texture coordinates.** A grid patch is parameterised over its own `(dimX, dimY)` lattice, so a
texture lands square with no authoring. A **mesh** cloth uses the UV set the asset carries;
`OmCloth::loadMeshFromUrl` keeps `aiComponent_TEXCOORDS` (it used to strip it) and carries the uv
through the position weld, taking the first uv seen at each surviving position. Only a file with
no UVs at all falls back to the planar XY projection of the rest pose, which is a placeholder and
looks like one on anything with a sleeve. ⚠ Keeping UVs does **not** reintroduce the
unwelded-seam trap — the weld is on position alone and is unchanged — but it does mean a
UV-seam split collapses to one particle keeping one of its UVs, i.e. a texel of smear rather than
a tear. So the baked map has to be **continuous over the whole surface** — a cut-and-lay-flat
atlas cannot survive the load — and `scripts/dev/usd_to_cloth_obj.py` gets that from a *folded*
map, one that is 2-to-1 rather than cut. Mirroring is invisible on a tiling weave, and it is the
reason this is a fabric parameterisation rather than a place to put a chest print.

⚠ **The map is `--uv arap-panel` since 2026-08-15, and the old `mirrored-cylindrical`
(`u = |atan2(y, x)| / pi`) is kept but is no longer the default — it MEASURED as the cause of the
shirt's mottling.** The seam was never the problem: `|atan2|` already folds on the two sides, and
the fold statistics are the same before and after (85 fold vertices, 95 % of them on the
side/sleeve silhouette, none on the chest). The problem was the *metric between* the seams, and it
was concentrated in one place — the **sleeves**. They are tubes lying along X, so an angle about Z
and a bare `z` are both nearly constant along a sleeve and the whole sleeve collapses into a sliver
of UV space. Measured per region on `tshirt.obj`: the torso was fine (mip spread 0.52) and the
sleeves were not (1.75, anisotropy to 1805). ⚠ **The obvious fix — a front/back planar split,
`u` from x and `v` from z — was measured and is WORSE**: whole-mesh mip spread 1.58 vs 1.18, and
the torso goes from 0.2 % to 10.9 % of its area more than a mip level off, because this shirt's
torso is a nearly round tube (0.32 wide × 0.27 deep) that a planar projection crushes at the sides.
`arap-panel` instead relaxes the mirrored-cylindrical map toward isometry (local/global ARAP,
reflections allowed so the fold stays free) and is scored by `uv_distortion` in the same run:

| `tshirt_hi.obj` | mip spread | area >1 mip | aniso p50 | aniso p95 | aniso max | texels²/m² (ideal 36) |
|---|---|---|---|---|---|---|
| mirrored-cylindrical | 1.185 | 4.22 % | 1.613 | 4.423 | 800.6 | 52.1 |
| **arap-panel** | **0.187** | **0.10 %** | **1.061** | **1.317** | **41.1** | **35.5** |

Two consequences worth knowing. The `--uv-tiles-per-m` figure is now **exact** (the old map scaled
`u` by a bbox-fitted ellipse whose major axis was the *sleeve tip*, which over-scaled the torso by
1.6× — that is the whole of its aniso p50), so the shipped shirt's weave went from an effective
~7.3 repeats/m to the 6 it always declared, i.e. ~1.23× larger. And anisotropy matters twice here
because these worlds use a `normalMap`: the tangent frame is derived from the UVs, so where the map
is near-degenerate the perturbed normal points somewhere arbitrary — the same mottle by a second
route, and the reason the artefact moved with the surface rather than with the light.

**Back faces are shaded with the front face's normal, deliberately.** Face culling is off, and
WREN's textbook two-sided path (`wr_renderable_invert_front_face`, which flips the winding and
raises the shaders' `reverseNormals` uniform) was tried and **measured worse**: a horizontal
sheet pinned on all four edges went from **0.0 % to 39.9 %** pure-black pixels seen from below,
because flipping the normal toward the viewer turns it away from the light and there is no fill
to land in the shadow it creates. On the T-shirt it moved nothing (12.70 % → 13.06 %). Revisit
only once an unlit surface stops rendering at `(0, 0, 0)`.

### Registration is deferred on purpose

`OmCloth` registers with Newton from `onPhysicsStepStarted`, **not** `postFinalize`
([`OmCloth.cpp:196-224`](../../src/omnisim/nodes/OmCloth.cpp)). The Newton world is opened
lazily from `OmSolid::flushPendingNewtonRegistrations()`, which caches `WorldInfo`'s
`coordinateSystem` and its `newtonGroundMu`/`newtonContactKe`/`newtonContactKd` **before** the
world is constructed. newton bakes the up axis into the implicit ground plane's normal and
copies mu/ke/kd into every shape **at add time**, so a world opened ahead of that cache can never
be given them afterwards — the exact defect that made a declared `newtonGroundMu 2.0` slide at
the default on the 55-degree ramp comparator.

---

## 4. Environment variables

All parsed by `_cloth_env_float` / `_cloth_env_int` / `_cloth_env_flag`
([runtime:2656-2680](../../src/omnisim/physics/omnisim_newton_runtime.py)). Unset **or empty**
= default. A malformed float/int logs and falls back to the default rather than raising. Flags
are **value-parsed**: anything not in `{0, false, off, no}` (case-insensitive) is true — so
`=0` genuinely means off.

### Contact & material

| var | default | effect |
|---|---|---|
| `OMNISIM_CLOTH_CONTACT_MARGIN` | `0.01` | `CollisionPipeline` soft-contact margin (m) |
| `OMNISIM_CLOTH_FULL_SURFACE_CONTACT` | `True` | water-tight rigid–soft pass (edge + face records) |
| `OMNISIM_CLOTH_SOFT_KE` | `1.0e3` | `model.soft_contact_ke` |
| `OMNISIM_CLOTH_SOFT_KD` | `1.0e1` | `model.soft_contact_kd` |
| `OMNISIM_CLOTH_SOFT_MU` | **the world's `newtonGroundMu`** (else `1.0`) | `model.soft_contact_mu` — particle-side friction |
| `OMNISIM_CLOTH_NEWTON_CONTACTS` | `False` | config (B): newton owns contact; **forces `mujoco_warp`** |
| `OMNISIM_CLOTH_RIGID_CONTACT_HARD` | `False` | hard AL duals for proxy body–body contacts |

### VBD solver

| var | default | effect |
|---|---|---|
| `OMNISIM_CLOTH_VBD_ITERATIONS` | `10` | VBD iterations per step |
| `OMNISIM_CLOTH_FRICTION_EPSILON` | `0.01` | VBD friction regularisation |
| `OMNISIM_CLOTH_SELF_CONTACT` | `True` | ⚠ has a WORLD FIELD since 2026-08-15 — `WorldInfo.newtonClothSelfContact` (`-1` unset / `0` off / `1` on). Precedence is **env > field > default**, so this variable still wins and `=1` exact-reverts a world declaring `0`. Prefer the field: it is the only one of the two a `.omniworld` can carry, and this setting is worth **24×** on a pinch (see below). |
| `OMNISIM_CLOTH_SELF_CONTACT_RADIUS` | min authored `particleRadius` (else `0.01`) | self-contact radius |
| `OMNISIM_CLOTH_SELF_CONTACT_MARGIN` | `1.5 ×` the radius above | detection band |
| `OMNISIM_CLOTH_RIGID_PARTICLE_BUFFER` | `2048` | ⚠ **non-negotiable**, see below |

### Coupling

| var | default | effect |
|---|---|---|
| `OMNISIM_CLOTH_PROXY_JOINTS` | `False` | enable the robot's joints inside the VBD view |
| `OMNISIM_CLOTH_JOINT_LINEAR_KE` | `2.0e7` | only read when `PROXY_JOINTS` is on |
| `OMNISIM_CLOTH_JOINT_ANGULAR_KE` | `2.0e6` | only read when `PROXY_JOINTS` is on |
| `OMNISIM_CLOTH_PROXY_MASS_SCALE` | `1.0` | proxy body mass scale |
| `OMNISIM_CLOTH_COUPLING_MODE` | `"lagged"` | proxy coupling mode |
| `OMNISIM_CLOTH_PROXY_RELAXATION` | `1.0` | under-relaxation, see the NaN note below |
| `OMNISIM_CLOTH_PROXY_ITERATIONS` | `1` | coupling iterations per step |

### Instrumentation

| var | default | effect |
|---|---|---|
| `OMNISIM_CLOTH_TELEMETRY` | *(unset = off)* | JSONL output path |
| `OMNISIM_CLOTH_TELEMETRY_EVERY` | `25` | sample every N steps |
| `OMNISIM_CLOTH_TELEMETRY_FULL` | *(unset = off)* | also dump every particle position |
| `OMNISIM_CLOTH_OVERFLOW_INTERVAL` | `240` | contact-buffer overflow sample period; `0` disables |
| `OMNISIM_CLOTH_FORCE_GRAPH` | `False` | re-arm CUDA-graph capture on a CPU-`mjc` cloth world |

### The two marked non-negotiable

**`particle_enable_self_contact`.** newton's default is `False`. Without it **a fold passes
straight through itself** — the cloth is not solid against its own surface — and there is no
error, no warning and no contact record to notice it by. A draped or folded sheet is the *normal*
case for cloth, so newton's default is wrong for every world this runtime will ever author
([runtime:2897-2904](../../src/omnisim/physics/omnisim_newton_runtime.py)).

⚠ **THE DEFAULT IS RIGHT FOR DRAPING AND WRONG FOR GRASPING, AND THE DIFFERENCE IS 24×.**
"Non-negotiable" above means *as a default*, not *in every world*. Both shipped
deformable-grasp worlds run with `OMNISIM_CLOTH_SELF_CONTACT=0` and say so in their headers:
measured on the patch world, tracking error is **−22.11 mm with self-contact ON and −0.92 mm
with it OFF**, because fabric gathered inside the jaws self-collides and pushes itself back out
of the pinch. A draped sheet needs self-contact and a pinched one is ruined by it, so the flag
is a per-world choice that the world file **cannot express** — there is no `Cloth` or `WorldInfo`
field for it, only the environment variable. That is a real expressiveness gap.

⚠ **Do not repeat commit `1fb7f135f`'s summary of this.** It says
*"particle_enable_self_contact costs 0.99 ms/step (35%) and STAYS: it buys the 24x
grasp-tracking improvement"* — that reads the same measurement **backwards**. The 24× is what
turning it **OFF** buys, and only for a grasp. Keeping it on is justified by the *fold*
argument above, which is a separate and genuine reason; it does not help a grip.

The self-contact **radius/margin** default to the authored particle radius rather than newton's
own 0.2 m for both, which on a 5 cm cell would make every particle "self-contact" most of the
sheet. 0.2/0.2 also sits exactly on the boundary that passes newton's own validation while
violating its own advice that the margin be comfortably larger than the radius — the margin is
the band in which a contact is *detected*, so equal-to-radius means a fast-moving fold can cross
the whole band inside one substep and be missed. Hence margin = 1.5 × radius.

**`rigid_body_particle_contact_buffer_size`.** newton's default is 256 per body. **Measured: 266
soft contacts on a body with only 153 particles in play**, so 256 overflows on a scene far
smaller than any real one. The overflow's **only** signal is a `wp.printf` from inside a warp
kernel, which is invisible here twice over — `omnisim-bin` is a GUI-subsystem binary on Windows
(stdout discarded outright) and the embedded interpreter's stdout does not reach the host
reliably anywhere. So the symptom is *"the grip feels wrong"*, with no message, **exactly the
shape of the `njmax`/`nconmax` cliff AGENTS.md documents for `mujoco_warp`**. 2048 is 8× the
default at a per-body cost of a few ints
([runtime:2909-2921](../../src/omnisim/physics/omnisim_newton_runtime.py)).

That cliff is now instrumented: `_cloth_overflow_check`
([runtime:5431](../../src/omnisim/physics/omnisim_newton_runtime.py)) reads
`body_particle_contact_overflow_max` against the allocated capacity every
`OMNISIM_CLOTH_OVERFLOW_INTERVAL` steps and warns **once per world** naming the peak, the cap and
the step. It runs
outside the substep loop so it never lands inside a graph capture.

---

## 5. Telemetry — the only way to verify cloth

⚠ **Cloth has no supervisor accessor and no HTTP endpoint.** Particle positions are read back
only for the renderer. A controller can drive a gripper into a sheet and have **no way to tell a
grasp from a miss** — which is disqualifying under this tree's standing rule that a grasp must be
proven *geometrically*, since "it looked right in the viewport" is not evidence.

`OMNISIM_CLOTH_TELEMETRY=<path>` is the instrument
([`_cloth_telemetry`, runtime:5929](../../src/omnisim/physics/omnisim_newton_runtime.py)). It is
**off unless the path is named** — a measurement instrument, not a feature, costing exactly zero
in a run that did not ask for it (the gate is resolved once and cached, because an
`os.environ` lookup per tick is precisely the API-layer cost the step-cost campaign found
dominating the computation it was blamed on).

JSONL, one self-describing record per sample, appended and flushed per line so a killed run is
still readable:

```json
{"step": 0, "n": 289,
 "centroid":  [x, y, z],
 "bbox_min":  [x, y, z],
 "bbox_max":  [x, y, z],
 "soft_contacts": 412,
 "nonfinite": 3,
 "q": [[x,y,z], ...]}
```

| field | when | meaning |
|---|---|---|
| `step` | always | sample index (not the engine step) |
| `n` | always | particle count in the cloth range |
| `centroid` / `bbox_min` / `bbox_max` | always | 6-dp world-space aggregates over all cloth particles |
| `nonfinite` | **only when non-zero** | count of non-finite components |
| `soft_contacts` | when readable | `Contacts.soft_contact_count` |
| `q` | `TELEMETRY_FULL` only | every particle position |

**`nonfinite` exists because NaN is the failure mode this stack actually exhibits** — the coupled
proxy *diverges* rather than erroring — so it is reported as a count instead of being allowed to
poison the aggregates silently.

**`soft_contacts` is the first number to look at when cloth goes through something.** A sheet
that falls through a floor with `soft_contacts == 0` is a **contact-generation** failure (flags,
visibility, pipeline). A sheet that falls through with `soft_contacts > 0` is **too soft / too
few substeps / tunnelling**. Without this the two are indistinguishable from the trajectory
alone, **and they have opposite fixes.**

Telemetry never takes down a run it was only observing: on any error it logs once and disables
itself.

---

## 6. Known limits and traps

### The composed FOLD is not demonstrated

`newton_tshirt_fold.omniworld` and its 1120-line controller `vbd_tshirt_fold.py` are committed
as **the record of an investigation, not as a working demo**, and the newest commit on that
line (`fbe30b159`) says verification of the composed fold is still *pending*.

Do not cite `.build_tmp/tshirt_fold_pads.jsonl` as evidence: it is **7 rows ending at
t = 0.208 s**, aborted-run residue rather than a result, and its own metadata carries a
`geometry_warnings` entry recording that the fold falls 3.9 mm short of material.

The blocker is structural, not a tuning problem. **A friction-only carry is impossible on
VBD**: velocity-regularised friction creeps at `u* ~ W·ε/(2μN)`, which is non-zero for *every*
μ, so a grasped-by-friction sheet slides for as long as you hold it. Filed upstream as
newton#3943. The replacement mechanism — a kinematic **particle attach** — has landed and is
covered by [`tests/test_newton_cloth_attach.py`](../../tests/test_newton_cloth_attach.py), the
one real pytest in this area: it runs a genuine CPU-VBD solve (169 particles, ~80 steps, no
engine binary), tracks a force-driven anchor to **< 0.02 m** while free cloth moves < 20% of
that, restores masses on detach, and acks **0** for a far-point attach. The attach works; the
fold composed on top of it is what remains unproven.

Two neighbouring demos are in the same state and should not be cited either — the sponge
dishwash world (`newton_vbd_sponge_dishwash.omniworld`) quotes a working run in its header but
commits no run artifact, and the three OmniArm 6 cloth worlds were committed explicitly as an
investigation (`d2c7efe14`): their cloth half is sound, but the arm does not hold its commanded
poses (`_cloth_fold_result.json` records `stopped_after: "stage"`, TCP error 0.5551 m).

### Stiffness is meaningful only RELATIVE TO PARTICLE MASS

**Measured 2026-08-14.** `triKe 1e5` — the schema default — over a **0.4 g** particle made a
sheet **implode**: y extent collapsed **0.240 → 0.083 m in 400 ms**. Dropping to `1e3` was
stable on a table but far too soft for a **hanging** sheet, which **stretched to 2.6× its
length**.

There is no single good default. **Match stiffness to the configuration** (and remember `mass`
is *per particle*, so the default 21×21 patch is ~0.44 kg total). Treat the schema's `1e5` as a
starting point for a *supported* sheet, not a universal value.

### A resting cloth with a free overhang is UNSTABLE

**Measured 2026-08-14.** Cloth resting on a surface with a free overhang slides off: **42% and
25% overhangs both slid off**. Pinning the far edge held the sheet, but fabric **still crept over
the edge in a stick-slip cycle**.

**The stable configuration is a HANGING (pinned) cloth**, which is why `fixTop` defaults to
`TRUE` and why the shipped drape world pins along +Y. If you are authoring a cloth-on-a-table
scene, expect to fight this.

### Cloth friction used to ignore the world's declared friction

`model.soft_contact_mu` was **hardcoded to 1.0**, so a world declaring `newtonGroundMu 6` got
mu=1.0 on every particle contact and the fabric slid on surfaces the same world had declared
grippy. **Fixed** — it now defaults to the world's own `newtonGroundMu`
([runtime:2858-2877](../../src/omnisim/physics/omnisim_newton_runtime.py)).

This is the same declared-but-unread failure mode this tree has been bitten by before
(`WorldInfo.contactProperties` reaching nothing while the solver ran mu=1.0 — see AGENTS.md).

**Measured**, on the in-progress `omniarm6_cloth_fold` world with 25% of the sheet overhanging a
table edge: at the old hardcoded 1.0 the sheet poured over the edge and ended on the floor
(y 0.294–0.543); raising cloth friction alone kept it near the table (y 0.052–0.311). Friction
is applied and it is the governing parameter, so it must be the one the world states.

⚠ This is only the **particle side** of the pair — newton averages it with each shape's own
material (geometric mean for mu), so the effective value still differs per surface.
`OMNISIM_CLOTH_SOFT_MU` remains the override for tuning fabric independently of the rigid world.

### `castShadows TRUE` used to crash the engine (fixed) — WREN-era history

⚠️ **This subsection describes the WREN renderer, which was deleted on 2026-08-23 (`976b9449d`,
`src/wren` among others). It is kept as the record of why the default is what it is; the file it
names no longer exists in the tree.** The fix was `src/wren/DynamicMesh.cpp:497-531` — read it at
`git show 976b9449d^:src/wren/DynamicMesh.cpp` if you need the code. Frozen WREN reference images
survive at [`tests/rendering/wren_reference/`](../../tests/rendering/wren_reference/).

The field defaulted to `FALSE` for a while because `TRUE` crashed the engine with an access
violation (`0xC0000005`) **on the GL driver's own worker thread**. It was fixed there; default is
`TRUE` again.

The mechanism, recorded because the symptom pointed nowhere near the cause:

`WrDynamicMesh` keeps a **second** vertex array for the shadow pass (`mShadowCoords`, two
`vec4` per vertex). `addCoord()` only mirrors into it **once the mesh has a `ShadowVolumeCaster`
attached** — which happens in `Renderable`, i.e. only once the `Renderable` exists. **`OmCloth`
fills the mesh BEFORE it builds its `Renderable`**, so `mShadowCoords` stayed empty, its dirty
flag stayed `false`, and the shadow VBO was never given a data store at all. The silhouette pass
still computed real indices from the index buffer, so `glDrawElements` ran against a buffer with
no storage and the driver dereferenced `NULL + index * 16` — **measured on the drape world as a
faulting read at `0x23e0` == 574 × 16, index 574 of 578 = 2 × 289 particles**, in JIT'd driver
code (hence "no loaded module") with no OmniSim frame on the stack.

`updateGlShadow()` now **re-derives** `mShadowCoords` whenever it does not match `mCoords`,
restoring the `mShadowCoords.size() == 2 * mCoords.size()` invariant that the whole shadow path
assumes, at the one point every shadow draw passes through. **So the fill-then-attach ordering is
now supported for any node, not just this one.**

⚠ **For anyone reading an older comment: the material's STENCIL programs are NOT what crashed.**
`OmCloth` sets them (as `OmMuscle` does) so the patch takes part in the per-light stencil pass —
without them it renders through `Scene::renderStencilWithoutProgram`, unlit by the shadow-aware
path, but it **does not fault**.

### Cloth is a GPU workload — size your run durations accordingly

**Measured 2026-08-14: ~0.17× realtime for a small scene.** The shipped drape world is
deliberately small (17×17 = **289 particles**) so it stays usable either way; raise `dimX`/`dimY`
only on CUDA.

The two bracketing figures recorded in the code are **6.7 fps on CPU** and **164 fps on CUDA +
graph capture**, both at 289 particles on an RTX 3060 laptop. The ~0.17× figure sits between
them, which is consistent with what a CPU-`mjc` cloth world actually gets — **CUDA VBD without
graph capture** (§2) — though that attribution is an inference, not a separate measurement.

Practical consequence: **a 10 s headless run buys you under 2 s of sim time.** Budget
accordingly, and remember Newton's one-time `finalizeWorld()` (1.9–4.9 s) is *inside* the
physics bracket on top of that.

### A coupled cloth+robot run is NOT known to be deterministic on CUDA

`OMNISIM_CLOTH_PROXY_RELAXATION` defaults to `1.0`, newton's own value, which reproduces prior
behaviour bit-for-bit — so exposing the knob changes nothing for a run that does not set it. It
is exposed because a parallel standalone build of this exact architecture (`SolverMuJoCo` arm +
`SolverVBD` cloth) **measured 1.0 diverging to NaN on 3 of 15 IDENTICAL invocations**, while
**0.7 ran 12/12 clean at no measurable cost** — and, more alarmingly, found that **merely
changing the LOGGING CADENCE changed the outcome**, because each log line adds a host sync
([runtime:3045-3059](../../src/omnisim/physics/omnisim_newton_runtime.py)).

**Treat a coupled cloth+robot run as non-deterministic on CUDA until shown otherwise, and reach
for this knob before blaming the grasp.**

### Other limits

- **No robot grasp of cloth is demonstrated.** See the banner at the top.
- **No SDF provisioning**, so `enable_rigid_soft_full_surface_contact` may silently fall back to
  the per-vertex pass on URDF collision meshes (it logs when it does).
- **Proxy joints default OFF.** Upstream's gripper example has 3 joints; a URDF robot here has
  30+, and every enabled proxy joint becomes a penalty constraint VBD must satisfy each
  iteration. Turn on `OMNISIM_CLOTH_PROXY_JOINTS` only if a precise pinch proves to need it.
- **`edgeKd` is not an authorable field** — always derived.
- **A `Cloth` under a `Pose`/`Group` ignores that ancestor** (§1).
- **A zero stiffness cannot be authored** (`0` means "unset") — use a tiny value.

---

## 7. Running the demo

```bash
python -m omnisim run-headless \
  projects/samples/demos/worlds/physics/newton_cloth_drape.omniworld --duration 30
```

[`newton_cloth_drape.omniworld`](../../projects/samples/demos/worlds/physics/newton_cloth_drape.omniworld) is
the end-to-end check that the three layers meet: the `Cloth` node parses,
`OmNewtonBackend::addClothGrid` reaches the runtime's `add_cloth_grid`, `SolverVBD` steps the
particles, and the per-frame readback streams into a `WrDynamicMesh` so the sheet is actually
visible.

Its world contract: `basicTimeStep 8`, `coordinateSystem "ENU"`, `newtonSolver "mujoco+vbd"`,
`newtonStatics TRUE`, `newtonGroundMu 1`; a 6×6×0.1 m floor, a 0.5 m cube table with its top face
at z = 0.5, and a 17×17-particle sheet (`dimX 16 dimY 16`, `cellX/cellY 0.05`, `mass 0.001`,
`particleRadius 0.01`, `fixTop TRUE`) starting flat at z = 1.0.

**The proof is numeric, not visual**, as the world's own header says: the lowest particle must
end well below its start **and above the floor**, and the pinned edge must not move at all.
Get those numbers from the telemetry:

```bash
OMNISIM_CLOTH_TELEMETRY=$PWD/.build_tmp/cloth.jsonl \
OMNISIM_CLOTH_TELEMETRY_EVERY=25 \
python -m omnisim run-headless \
  projects/samples/demos/worlds/physics/newton_cloth_drape.omniworld --duration 60
```

then read `bbox_min[2]` (falling, must stay above the floor), `bbox_max[1]` (the pinned +Y edge,
must not move), `soft_contacts` (non-zero once it lands) and `nonfinite` (must be absent) across
the records.

**Verify Newton actually drove the run** the usual way — the `.newton.json` verdict sidecar
next to the log must show `finalised: true`, `degraded: false`, and on a cloth world its
`runtime.device` should read `cuda` (§2). A bare `run-headless` PASS is a *log* verdict and
cannot see a frozen sheet.

### The material demo

```bash
python -m omnisim run-headless \
  projects/samples/demos/worlds/physics/newton_cloth_material.omniworld --until-finalized
```

[`newton_cloth_material.omniworld`](../../projects/samples/demos/worlds/physics/newton_cloth_material.omniworld)
is the counterpart check for §3.1: one grid patch and one mesh garment wearing the same
`PBRAppearance` (knit base colour + normal + roughness, generated by
[`scripts/dev/make_cloth_textures.py`](../../scripts/dev/make_cloth_textures.py)), so both
parameterisations — lattice UVs and asset UVs — are exercised in one load. It is also the only
cloth world that adds fill lighting, for the measured reason in §3.1.

Two of its choices are measurements rather than taste, and both are worth carrying into any
garment world:

- It simulates **`tshirt_hi.obj` (2394 particles)**, not the 616-particle `tshirt.obj` the other
  worlds use. At 616 the garment's own surface is genuinely crumpled at ~4 cm and smooth
  per-vertex normals turn that into hard-edged shading shards. Measured through the harness,
  40 stepped frames each, machine `9722d23d12a3`: **616 → 6.75 ms/step, 2394 → 14.02 ms/step**,
  i.e. **2.08× the cost for 3.9× the particles**. A demo can afford it; a training world may not,
  which is why both meshes ship.
- It declares **`triKe 1000000`**. At the default the shirt *creeps*: a 0.199 kg garment on a
  pinned shoulder band keeps elongating until it reads as a floor-length dress, on the
  616-particle mesh as much as on the 2394-particle one. It is stretch, not resolution.

---

## 8. See also

- [`resources/nodes/Cloth.wrl`](../../resources/nodes/Cloth.wrl) — authored field contract
- [`resources/nodes/WorldInfo.wrl:36`](../../resources/nodes/WorldInfo.wrl) — `newtonSolver` enum
- [docs/guide/newton-physics-backend.md](../guide/newton-physics-backend.md) — the Newton backend
- [docs/guide/friction-grasp.md](../guide/friction-grasp.md) — why the robot stays on `SolverMuJoCo`
- [docs/benchmarks/determinism-scope.md](../benchmarks/determinism-scope.md) — what "deterministic" is scoped to
- physics-step-cost-optimization-plan.md (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)) — the CPU device pin cloth inverts

*If this doc and the code disagree, the code wins — update this doc in the same change.*
