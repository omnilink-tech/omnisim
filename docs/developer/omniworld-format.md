# The `.omniworld` world-file format

*OmniSim's world files are `.omniworld`. `.wbt` is read forever. This document says what
the format is, what it can express that `.wbt`/Webots cannot, what it refuses, and how a
world selects a physics solver.*

---

## 0. Status, provenance, and how to read this document

**Written 2026-08-15 against `HEAD = e525ef5bc` with a DIRTY working tree.** Two sibling
lanes are landing into this same checkout as this is written:

| lane | what it is doing | state as read today |
|---|---|---|
| **migration** | dual-read `.omniworld` / `.wbt` across the engine and tooling | `src/omnisim/core/OmWorldFileFormat.hpp` exists; ~20 engine call sites converted; **uncommitted** |
| **capability** | whole-world single-`SolverVBD`, per-body cloth visibility | `newtonSolver "vbd"` in the schema + runtime; `Solid.newtonClothCoupling` runtime-only; **uncommitted** |

Consequences you must carry into every reading of this file:

- **Line numbers under `src/omnisim/` and `resources/nodes/` shifted while this document
  was being written** — `omnisim_newton_runtime.py` alone moved by ~100 lines, and several
  `OmSolid.cpp` / `OmTokenizer.cpp` anchors moved by 1–57. Every anchor below was
  re-verified against the tree at the end of the write, but treat all of them as **± a few
  lines**: the **symbol name or quoted string is the real anchor**, the line number is a
  convenience. Anchors into `resources/nodes/*.wrl` and the vendored newton runtime were
  stable.
- Anything marked **PLANNED** was not reachable from a world file at the time of writing.
  Anything marked **IN FLIGHT** exists in the working tree but is not committed.
- **This document must be reconciled once both lanes land.** Section 9 is the checklist.

Epistemic markers used throughout, in the house style:

- **MEASURED** — a number produced by a command run for this document. Every measurement
  here is a static count over the tracked corpus (`git ls-files`, `git grep`); the
  simulator was **not run** (thermal budget, other lanes holding the engine), so there is
  **no runtime measurement in this document at all**.
- **READ** — a claim taken from source by reading it. Not executed.
- **INFERRED** — a conclusion drawn from a READ, flagged as such.

---

## 1. What `.omniworld` is

A `.omniworld` file is byte-for-byte the same grammar as a `.wbt` file: the VRML-derived
node syntax OmniSim inherited from Webots, with the OmniSim header line, `EXTERNPROTO`
declarations, and a node tree. **Renaming a `.wbt` to `.omniworld` changes nothing about
its contents and is a valid migration** (§7).

What changes is what the *name* asserts. `.wbt` literally means "Webots" and asserts a
compatibility that, for 830 of the 916 tracked world files in this tree, is false
(§2). `.omniworld` asserts the opposite: **this is the format permitted to contain what
upstream cannot load.**

### 1.1 The dual-read / single-write contract

Stated in the funnel header itself
([`src/omnisim/core/OmWorldFileFormat.hpp:18-40`](../../src/omnisim/core/OmWorldFileFormat.hpp)):

> OmniSim reads BOTH `.omniworld` and `.wbt`, indefinitely. A `.wbt` world keeps working
> forever: there are external users and forks, and nothing in this policy is allowed to
> break them.
>
> OmniSim WRITES only `.omniworld`.

Precisely:

| operation | `.omniworld` | `.wbt` |
|---|---|---|
| open / load / hot-reload | yes | **yes, indefinitely** |
| `#include` expansion, `URDFRobot` expansion, header check | yes | yes |
| Save / Save As / new-world wizard / project templates | **emits this** | never emitted |
| generators (`omniworld generate`, capture/harness sibling files) | **should emit this** — see §9 | — |
| file-dialog filters, directory scans, recent-files | both | both |

"Accepted" is enforced through **one funnel**, not through scattered `endsWith(".wbt")`
tests. `OmWorldFileFormat` (READ) exposes:

| helper | line | purpose |
|---|---|---|
| `writeExtension()` | `:50-53` | `".omniworld"` — the one extension OmniSim writes |
| `legacyExtension()` | `:56-59` | `".wbt"` |
| `readExtensions()` / `readSuffixes()` | `:62-72` | both, preferred first |
| `isWorldFile()` / `isWorldSuffix()` | `:75-90` | the extension test |
| `nameFilters()` | `:93-97` | `{"*.omniworld", "*.wbt"}` for `QDir::setNameFilters` |
| `fileDialogFilter()` | `:101-104` | `World Files (*.omniworld *.OMNIWORLD *.wbt *.WBT)` |
| `stripExtension()` / `replaceExtension()` | `:108-121` | sibling artefacts (thumbnails, `_net` dirs) |
| `toWriteExtension()` | `:125-131` | the single "what do we save as" funnel |
| `resolveExisting()` | `:138-154` | a stored `foo.wbt` still resolves after the tree is migrated to `foo.omniworld`, and vice versa |

`resolveExisting()` is the one that makes the migration survivable in practice: recent-files
lists, launcher manifests, `--world` arguments and demo catalogues hold **stored strings**,
and a rename would otherwise break every one of them.

The header carries its own rule, and it is the rule to enforce in review:

> EVERY extension test in the engine must go through this header. Do not hand-roll
> `endsWith(".wbt")` — that is how half a dual-read policy ships.

### 1.2 The in-file header is the precedent, not an analogy

This is the second time the format has renamed a Webots identifier while keeping the old
one readable. The first was the header line itself:

```
#VRML_SIM R2025a utf8      <- legacy upstream form, accepted forever
#OMNISIM  R2025a utf8      <- canonical
```

The tokenizer accepts either through **one non-capturing alternation**
([`OmTokenizer.cpp:150-156`](../../src/omnisim/vrml/OmTokenizer.cpp)):

```cpp
bool found = mFileVersion.fromString(header, "^(?:OMNISIM|VRML_SIM) ", " utf8$");
```

and the writer emits only the new form
([`OmWriter.cpp:139-142`](../../src/omnisim/vrml/OmWriter.cpp)):

```cpp
case VRML_SIM:
  *this << QString("#OMNISIM %1 utf8\n").arg(OmApplicationInfo::version().toString(false));
```

**MEASURED (2026-08-15, `git ls-files '*.wbt'` + `head -1`):**

| scope | worlds | `#OMNISIM` | `#VRML_SIM` |
|---|---:|---:|---:|
| whole tree | 916 | 700 | 216 |
| `projects/` | 354 | 351 | 3 |
| `tests/` | 549 | 336 | 213 |
| `distribution/` | 9 | 9 | 0 |

That is the shape a dual-read migration actually has, and it is worth internalising before
starting this one: the header flip is **years old**, the write path has emitted only
`#OMNISIM` for that whole time, and **216 files still carry the legacy form and load
fine** — concentrated in `tests/`, where nothing rewrites them. The extension migration
will end in the same place, and that is the success condition, not a failure.

⚠ **Sharpest illustration:** `projects/samples/demos/worlds/physics/newton_cloth_drape.omniworld`
— the newest and most OmniSim-specific world in the tree, unloadable by Webots four times
over — opens with `#VRML_SIM R2025a utf8`. A file's Webots-era name says nothing about its
contents. That is the whole argument for the rename in one line.

---

## 2. Why the extension changed — the capability argument

**This is not branding.** The claim is falsifiable and it was measured.

A world is unloadable by upstream Webots if it contains any of: the `URDFRobot` node, a
`Cloth` / `SoftBody` / `GranularGroup` node, the `omnisim://` URL scheme, any `newton*`
`WorldInfo` field, or the `physicsBackend` / `renderBackend` fields.

**MEASURED (2026-08-15, `git grep -l` over 916 tracked `.wbt` files):**

| construct | worlds containing it | where it is defined |
|---|---:|---|
| `omnisim://` URL scheme | **674** | [`OmUrl.cpp:145,203`](../../src/omnisim/vrml/OmUrl.cpp) |
| any `newton*` `WorldInfo` field | **401** | [`WorldInfo.wrl:36-53`](../../resources/nodes/WorldInfo.wrl) |
| `URDFRobot` | **284** | [`OmUrdfImporter.hpp:38-43`](../../src/omnisim/vrml/OmUrdfImporter.hpp) |
| `Solid.physicsBackend` | **207** | [`Solid.wrl:16`](../../resources/nodes/Solid.wrl) |
| `renderBackend` | 9 | [`Viewpoint.wrl:17`](../../resources/nodes/Viewpoint.wrl), [`Camera.wrl:40`](../../resources/nodes/Camera.wrl) |
| `GranularGroup` | 6 | [`GranularGroup.wrl`](../../resources/nodes/GranularGroup.wrl) |
| `Cloth` | 3 | [`Cloth.wrl`](../../resources/nodes/Cloth.wrl) |
| `SoftBody` | 1 | [`SoftBody.wrl`](../../resources/nodes/SoftBody.wrl) (untracked, IN FLIGHT) |
| `#include` directive | 2 | [`OmTokenizer.cpp:402-410`](../../src/omnisim/vrml/OmTokenizer.cpp) |
| **union of all of the above** | **830 of 916 (90.6%)** | |

**Nine in ten world files in this tree already cannot be opened by the simulator whose
extension they carry.** The extension was making a promise the corpus stopped keeping.

The reverse direction fails too, and by a smaller, sharper mechanism: **OmniSim does not
accept the `webots://` URL scheme at all.** `git grep webots:// src/ resources/nodes/`
returns nothing (MEASURED); the only scheme `OmUrl` rewrites is `omnisim://`
([`OmUrl.cpp:145`](../../src/omnisim/vrml/OmUrl.cpp)). A stock Webots world whose textures
and PROTOs are addressed `webots://…` will not resolve its assets here. The two formats
have diverged in **both** directions, which is what makes "same extension" actively
misleading rather than merely stale.

---

## 3. What `.omniworld` can express that `.wbt`/Webots cannot

Everything in this section is READ from the schemas in `resources/nodes/*.wrl` and the
engine source, not from summaries. `resources/nodes/*.wrl` is the authoritative field
contract; several of those files carry mechanism in their comments that is not
re-derivable from the code, so read them, not paraphrases of them.

### 3.1 `URDFRobot` — native URDF import as a source expansion

```
URDFRobot {
  url         "omnisim://projects/robots/clearpath/husky/urdf/husky.urdf"
  translation 0 0 0.2
  rotation    0 0 1 0
  name        "husky"
  controller  "husky_random"
}
```

**Not a node.** It is a **source-text expansion** performed before tokenization
([`OmTokenizer.cpp:414-417`](../../src/omnisim/vrml/OmTokenizer.cpp)):

```cpp
if (contents.contains("URDFRobot"))
  contents = OmUrdfImporter::expandUrdfRobotBlocks(contents, mFileName);
```

`OmUrdfImporter::expandUrdfRobotBlocks` loads the referenced URDF, converts it to a
`Robot { … }` VRML subtree, and merges the `translation` / `rotation` / `name` /
`controller` fields written inside the `URDFRobot` block into the generated node
([`OmUrdfImporter.hpp:38-43`](../../src/omnisim/vrml/OmUrdfImporter.hpp)).

⚠ **Because it is a `tokenizeFile` expansion, a `URDFRobot` cannot be spawned from a
string.** A supervisor import goes through `tokenizeString`, which never expands it, and
the parser then classifies it as an undeclared PROTO and refuses. A container world that
will receive spawned robots needs **one authored robot to clone**, not zero. (This is the
same rule the harness's `POST /scene/spawn` documents.)

### 3.2 `#include` — a world-file preprocessor

```
#include "omnisim://projects/samples/demos/worlds/_common/lighting.wbt"
```

Splices the referenced file's body inline, recursively, with cycle protection, **before**
URDF expansion so an included file can carry its own `URDFRobot` blocks
([`OmTokenizer.cpp:403-411`, `expandWorldIncludes` at `:454`](../../src/omnisim/vrml/OmTokenizer.cpp)).
Duplicate `#OMNISIM`/`#VRML_SIM` header lines produced by splicing are stripped
([`:487-489`](../../src/omnisim/vrml/OmTokenizer.cpp)).

⚠ **Gated on `mFileType == WORLD`**, which is decided purely by file extension
([`fileTypeFromFileName`, `OmTokenizer.cpp:752-768`](../../src/omnisim/vrml/OmTokenizer.cpp)).
Before the migration lane's change that test was a bare `endsWith(".wbt")`; it is now
`OmWorldFileFormat::isWorldFile(name)` (IN FLIGHT). Had it not been converted, an
`.omniworld` file would have tokenized as `UNKNOWN`, silently skipping **both** the
`#include` pass and the header check. This is the single sharpest illustration of why the
funnel rule in §1.1 exists.

### 3.3 `omnisim://` — the asset URL scheme

Resolves against the install root, with a documented remote-asset fallback for streaming
and for assets missing locally ([`OmUrl.cpp:145, 203, 286-312`](../../src/omnisim/vrml/OmUrl.cpp)).
Used by `EXTERNPROTO` declarations, `ImageTexture`, `Mesh`, `CadShape`, `URDFRobot.url`,
and the sound fields on `ContactProperties`. **674 tracked worlds use it (MEASURED)** —
the single largest source of Webots-incompatibility in the corpus, and the most boring one.

### 3.4 The `newton*` `WorldInfo` surface

Sixteen live fields, all defined in
[`resources/nodes/WorldInfo.wrl:36-51`](../../resources/nodes/WorldInfo.wrl). Each folds a
launch-time `OMNISIM_NEWTON_*` env var into the world file so a world is self-contained;
**the env var still overrides the field in every case.**

| field | type | unset | what it selects |
|---|---|---|---|
| `newtonSolver` | SFString enum | `""` | the solver profile — see §4 |
| `newtonSubsteps` | SFInt32 | `1` | sub-steps per physics tick; RL deploy runs 8; `mujoco_warp` and the VBD CUDA graph need an **even** value |
| `newtonCone` | SFString enum | `""` = pyramidal | `"elliptic"` removes inscribed-pyramid creep; pair with `newtonImpratio 10` |
| `newtonGroundMu` | SFFloat | **`-1`** | contact friction. `0` is a **legal** value meaning frictionless |
| `newtonContactKe` / `newtonContactKd` | SFFloat | `0` → 2500 / 100 | contact stiffness / damping (maps to the geom's `solref`) |
| `newtonIterations` / `newtonLsIterations` | SFInt32 | `0` | MuJoCo solver / line-search iterations |
| `newtonImpratio` | SFFloat | `0` → 1 | frictional-to-normal constraint impedance ratio |
| `newtonCondim` | SFInt32 | `0` → condim 3 everywhere | 1/3/4/6. **Global, not per-shape.** 4 adds torsional friction — what a two-finger pinch wants |
| `newtonNoslipIterations` | SFInt32 | `0` = off | MuJoCo's friction-only post-solve pass. **CPU only**; `mujoco_warp` raises on non-zero, so the engine warns and ignores |
| `newtonNjmax` / `newtonNconmax` | SFInt32 | `0` → 256 | constraint-row / contact buffer caps. `-1` requests newton's estimate and is **worse than 0** on fleet worlds |
| `newtonStatics` | SFBool | schema `FALSE`, **engine default ON** | register top-level static colliders as mass-0 bodies |
| `newtonRobotColliders` | SFBool | `FALSE` | give a robot chassis its own `boundingObject` as a collider |
| `newtonCompoundColliders` | SFBool | `FALSE` | register **every** collider in a compound `boundingObject`, not just the first child |

Two conventions that catch people:

- **`0` means "unset" on most of these**, so a genuinely-zero value cannot be authored.
  `newtonGroundMu` is the deliberate exception: it uses `-1` for unset **specifically so a
  frictionless world can state itself in its own file** (`WorldInfo.wrl:39`).
- **`newtonStatics`'s schema default is not the engine default.** The schema says `FALSE`;
  the engine runs statics ON. Leaving the field out gets you solid floors.

### 3.5 The deformable nodes

Three node types with **engine-owned rendering** — the simulated particle positions are
streamed into a WREN dynamic mesh every frame, so there is no `Shape` / `IndexedFaceSet`
child to author and none is accepted.

| node | what it is | schema |
|---|---|---|
| `Cloth` | rectangular triangle sheet on `SolverVBD`; stiffness in triangles (`triKe`/`triKa`/`triKd`) and bending edges (`edgeKe`) | [`Cloth.wrl`](../../resources/nodes/Cloth.wrl) |
| `SoftBody` | rectangular **tetrahedral FEM** block on `SolverVBD`; stiffness entirely in the tets (`kMu`/`kLambda` = Lamé parameters in Pa, `kDamp`) — its surface triangles carry no elastic force, which is why there is no `triKe` here | [`SoftBody.wrl`](../../resources/nodes/SoftBody.wrl) **(IN FLIGHT, untracked)** |
| `GranularGroup` | GPU-resident sphere-particle group; lives in a CUDA buffer when `OMNISIM_WITH_CUDA` is on, inert with a one-time `CUDA_NOT_AVAILABLE` warning otherwise | [`GranularGroup.wrl`](../../resources/nodes/GranularGroup.wrl) |

Shared conventions worth stating once, because they are not guessable and all three
schemas repeat them:

- **`translation` is a CORNER, not a centre.** A `Cloth`'s local (0,0) corner; a
  `SoftBody`'s **minimum** corner. This is newton's convention and it is the **opposite**
  of `Solid`/`Box`. A 4×4×4 `SoftBody` of 0.05 m cells authored at `(0,0,0.5)` spans
  z = 0.5 … 0.7 — its centre is 0.1 m above the number in the file.
- **Pinning is LOCAL, and there is no Z pin.** `fixLeft`/`fixRight` = local ∓X,
  `fixBottom`/`fixTop` = local ∓Y. "Top" is local +Y, **not world up**. To pin the
  world-top face of a `SoftBody` in a Z-up world, rotate +90° about X so local +Y maps to
  world +Z, then set `fixTop`.
- **`0` means "leave it to the runtime"** on every stiffness/material field, matching the
  `newton*` convention. A genuinely zero stiffness cannot be authored — use a tiny value.
- **An unsimulated `Cloth` still renders; an unsimulated `SoftBody` renders nothing.** A
  cloth sheet's triangle winding is derived analytically from `dimX`/`dimY`; a soft block's
  surface is the set of open faces of its tet mesh, known only to newton and snapshotted at
  authoring time (the `ModelBuilder` is consumed at finalize). An invisible block in a world
  that also logged a solver warning is **that**, not a missing geometry field.

Two limits recorded in `SoftBody.wrl` and repeated here because they change how you author:

- ⚠ **Soft bodies are a GPU-class workload.** READ from the schema: 31 ms/step for a 4³
  block alone on CPU, 70 ms/step coupled to a rigid scene; 21.3 → 6.5 ms/step at
  `newtonSubsteps 2`, because the CUDA-graph predicate requires `_n_substeps % 2 == 0` and
  OmniSim's default of 1 is odd, so the graph can never arm.
- ⚠ **Soft-on-rigid is stable; RIGID-ON-SOFT is not.** A rigid body supported only by
  particles gains energy and is ejected. READ: reproduces in pure newton with OmniSim
  absent, in both coupled and uncoupled solvers, is not a timestep artifact, is not
  self-contact. Newton-side, mechanism unidentified. **Do not stage a rigid body resting on
  a soft one.**

### 3.6 Backend-selection fields

| field | node | values | note |
|---|---|---|---|
| `physicsBackend` | `Solid` | `"auto"` / `"newton"` / `"ode"` | `"ode"` is a **retired selector** — §4.4 |
| `defaultPhysicsBackend` | `WorldInfo` | `""` / `"newton"` / `"ode"` | world-level default for Solids still on `"auto"` |
| `renderBackend` | `Viewpoint`, `Camera` | `"wren"` / `"vulkan"` | opts a node into the wgpu renderer |
| `defaultRenderBackend` | `WorldInfo` | `""` / `"wren"` / `"wgpu"` | inert today: node defaults are still `"wren"`, so nothing opts into the `"auto"` sentinel |

⚠ The world-level field is spelled **`defaultPhysicsBackend`**. Writing the per-Solid name
`physicsBackend` inside `WorldInfo` is an **ERROR** line (§4.1) that takes a headless run's
exit code to 1 and reads as a crash.

---

## 4. What was removed, and what parses but is not read

Two distinct failure modes, with two very different diagnostics. Getting them the wrong way
round costs real debugging time, so the distinction is the point of this section.

### 4.1 REMOVED → hard ERROR → headless exit code 1

`OmParser` reports both an unknown node and an unknown field through
`OmTokenizer::reportError`, which calls **`OmLog::error`**
([`OmTokenizer.cpp:731-738`](../../src/omnisim/vrml/OmTokenizer.cpp)) — not a warning:

| removed | symptom | anchor |
|---|---|---|
| `Fluid` node | `'<file>':L:C: error: Skipped unknown 'Fluid' node or PROTO.` | [`OmParser.cpp:341`](../../src/omnisim/vrml/OmParser.cpp) |
| `ImmersionProperties` node | same form | [`OmParser.cpp:341`](../../src/omnisim/vrml/OmParser.cpp) |
| `Solid.immersionProperties` field | `error: Skipped unknown 'immersionProperties' field in Solid node.` | [`OmParser.cpp:372`](../../src/omnisim/vrml/OmParser.cpp) |
| ODE physics-plugin API (`webots_physics_init` / `_collide` / `_step`) | no world-file surface; `WorldInfo.physics` still parses, see §4.2 | — |

**MEASURED:** `ls resources/nodes/` contains no `Fluid.wrl` and no `ImmersionProperties.wrl`;
`Solid.wrl` declares no `immersionProperties` field. Buoyancy and drag are **not simulated**.

⚠ **This is the class that does not degrade quietly.** Unlike everything in §4.2, a removed
node or field takes `run-headless` to a non-zero exit — so a legacy immersion world reads as
a crash, not as a silent behaviour change. That is the desired behaviour, and it is also why
`.wbt` can no longer honestly claim Webots compatibility: **a world Webots accepts, OmniSim
now rejects outright.**

Also broken-rather-than-removed, and therefore invisible to the parser — a world using these
loads clean and does nothing:

- motorised **`BallJoint`** and **`Hinge2Joint`** do not actuate (motors accepted and
  silently ignored; their position sensors read 0);
- **`TouchSensor` reads nothing in EITHER type** — bumper and force alike. One mechanism,
  not two: the sensor's own `boundingObject` never becomes a Newton collider, so the parent
  body takes the contact. **Contact detection works; the contact DEVICE does not.** Use
  `getContactPoints` / `GET /sim/contacts` instead, and prove a grasp geometrically;
- contact **sound** and the contact-points **GUI overlay** produce nothing.

### 4.2 RETIRED → parses, accepted, NOT READ

These are still **declared** in the schemas for exactly one reason: an *undeclared* field is
an ERROR (§4.1), and legacy worlds carry them. Declared-and-ignored is the lesser evil.
`WorldInfo.wrl:4-9` states the policy, and each field is marked `RETIRED` inline.

| field | anchor | what it did |
|---|---|---|
| `WorldInfo.CFM` / `ERP` | `WorldInfo.wrl:16-17` | ODE's global constraint-force-mixing / error-reduction. Newton equivalents: `newtonContactKe` / `newtonContactKd` / `newtonImpratio` |
| `WorldInfo.physics` | `:18` | named an ODE physics plugin. Any value but `"<none>"` is ignored with one parse-time warning |
| `WorldInfo.optimalThreadCount` | `:21` | ODE thread count. Newton's threading is the solver's own |
| `WorldInfo.physicsDisableTime` / `…LinearThreshold` / `…AngularThreshold` | `:22-24` | ODE auto-disable. **Newton has no body sleep**, so these gate nothing |
| `WorldInfo.defaultDamping` | `:25` | the ODE damping call it fed is an empty function; damping is not plumbed to Newton |
| `WorldInfo.dragForceScale` / `dragTorqueScale` | `:31-32` | **retired in effect**: the value still sizes the GUI arrow and the newton figure printed in the 3D view, but `OmSolid::addForceAtPosition` / `addTorque` are empty functions, so **no force is applied**. Use the supervisor's `add_force` |
| `WorldInfo.contactProperties` (whole `ContactProperties` node) | `:34` | per-material-pair contact settings for ODE |
| `WorldInfo.broadphase` | `:35` | ODE broadphase structure. Newton/MuJoCo does its own and exposes no choice |
| `Physics.damping` / `Damping` node | `Physics.wrl:10` | ODE-tuned semantics; not plumbed |

⚠ **`contactProperties` is the one that has cost the most.** A world declaring
`coulombFriction 5` runs at **µ = 1.0**. The engine warns once, naming the declared value
and the fix ([`OmSolid.cpp:4462-4468`](../../src/omnisim/nodes/OmSolid.cpp)):

> `WorldInfo.contactProperties declares coulombFriction 5, but this world runs on the Newton
> backend, which does NOT read that field: the effective friction is 1.0. Set
> WorldInfo.newtonGroundMu 5 …`

**There is no exact migration** for a world using several materials: `contactProperties` is
a per-material **list** and `newtonGroundMu` is **one global value**.

### 4.3 The distinction, stated once

- §4.1 **breaks the run** and names itself. A legacy world that used it fails loudly.
- §4.2 **changes the physics silently.** A legacy world that used it loads clean, prints
  `PASS`, and simulates something the author did not ask for.

A bare `run-headless` PASS is a **log** verdict. It cannot see §4.2 at all, and it cannot
see a body free-falling through a missing collider either. Add `--fail-on-warning` to catch
the §4.2 warnings, and `--fail-on-runaway` when the claim is about the physics.

### 4.4 `physicsBackend "ode"` — accepted, and worse than useless

Still in the `Solid` enum ([`Solid.wrl:16`](../../resources/nodes/Solid.wrl)) because 207
tracked worlds carry it (MEASURED). READ: `OmOdeBackend::isAvailable()` is `false`, every op
returns `-1`, and `OmSolid::flushPendingNewtonRegistrations` **skips** any Solid whose
effective backend resolves to `"ode"`. Such a Solid is registered with no solver — **no
gravity, no contact.**

The engine warns, but only for a node that declares collision or mass — a hologram robot's
30 ghost links would otherwise emit 30 lines each
([`OmSolid.cpp:3500-3508`](../../src/omnisim/nodes/OmSolid.cpp)):

> `This Solid asks for physicsBackend "ode", which no longer selects a physics engine: ODE
> was removed and Newton is the only backend. The node will have NO gravity and NO contact
> — it is a visual-only body.`

⚠ **The value is used deliberately** as a "do not simulate this" switch for ghost/hologram
robots. That is why it is still accepted. **Never write it into a new world and never
suggest it as a workaround** — leave `physicsBackend` off (the `"auto"` default resolves to
Newton) or write `"newton"` explicitly.

---

## 5. The solver model

An `.omniworld` world selects its physics through exactly one field:
**`WorldInfo.newtonSolver`** ([`WorldInfo.wrl:36`](../../resources/nodes/WorldInfo.wrl)).
There is one backend — Newton — and this field selects a **profile** within it.

Plumbing (READ): `OmSolid::flushPendingNewtonRegistrations` forwards the raw string to
`OmNewtonBackend::setSolverPreference`
([`OmSolid.cpp:4332`](../../src/omnisim/nodes/OmSolid.cpp)), which stores it in the runtime
via `World.set_solver_preference` (`omnisim_newton_runtime.py`, `set_solver_preference`,
:273). `finalize()` then branches on it.

### 5.1 The value table

| value | what newton builds | rigid owner | particle owner | determinism |
|---|---|---|---|---|
| `""` / `"auto"` / `"mujoco"` | `SolverMuJoCo`, CPU `mj_step` | MuJoCo | — | **bitwise** (CPU, verified at 336 contacts) |
| `"mujoco_warp"` | `SolverMuJoCo`, GPU `mujoco_warp` | MuJoCo | — | **refuted** — 0 bitwise of 24 same-config cold pairs |
| `"mujoco+vbd"` | `SolverCoupledProxy{ SolverMuJoCo, SolverVBD }` over one Model | MuJoCo | VBD | not established; see §5.5 |
| `"vbd"` **(IN FLIGHT)** | one `SolverVBD` | **VBD (AVBD)** | VBD | not established |
| `"xpbd"` | — | — | — | **removed 2026-08-07.** Parser invalid-value warning + the default |

### 5.2 `"mujoco"` / `"mujoco_warp"` — what they cannot do

- **`"mujoco_warp"` is a batching lever, not a speed lever.** At `nworld=1` it measured
  **9.06× slower** than CPU `mj_step`. Its case is thousands of parallel GPU envs.
- It needs a CUDA GPU **and an even `newtonSubsteps`** — an odd count cannot arm the CUDA
  graph and re-issues ~800 kernel launches per tick.
- It does **not** read the MJCF `<size njmax>` tag and its own estimate is far too small:
  declare `newtonNjmax` / `newtonNconmax` for fleets. Overflow **silently truncates the
  constraint vector**; its only native diagnostic is a `wp.printf` from inside a warp kernel,
  which is discarded on Windows (`omnisim-bin` is a GUI-subsystem binary).
- `newtonNoslipIterations` is **CPU-only** — `mujoco_warp` has no such option field and
  `put_model` raises on a non-zero value, so the engine warns and ignores.
- **Raycast-backed sensors see the scene frozen at its build-time pose** on
  `"mujoco_warp"`: the raycast service serves off `solver.mj_data`, which newton's
  `SolverMuJoCo.step()` writes only on its `use_mujoco_cpu` branch. The rule there is
  **static targets only**. (A moving sensor against a static wall tracks correctly; the
  decisive differential — a moving *target* under `mujoco_warp` — has **not** been run.)

### 5.3 `"mujoco+vbd"` — the coupled pair

`_build_cloth_coupled_solver` (runtime, :3491). READ, from its own docstring:

- The **robot keeps the exact `SolverMuJoCo` it would have had** — same `**mjc_kwargs`, same
  `impratio`, cone, iterations, `njmax`/`nconmax`. Cloth particles go on `SolverVBD`.
  `SolverCoupledProxy` exchanges them over **one shared `Model`**.
- Why not put everything on VBD: OmniSim's working grasp is built on MuJoCo joint features
  VBD does not implement — the `effortLimit * 10` position PD servo every Newton joint is
  constructed with, armature, and `POSITION_VELOCITY` target mode. Moving the robot to VBD
  would silently change **every actuator in the tree**.
- The `"mjc"` entry owns every body and joint; the `"vbd"` entry owns only the particles; a
  `Proxy` mapping hands VBD virtual proxy bodies mirroring MuJoCo's. Each step VBD solves
  the cloth against those proxies and feeds the contact impulses back as forces
  (`mode="lagged"`). **The robot feels the cloth without MuJoCo ever knowing particles
  exist.**

**What it structurally cannot do:** relax a rigid body and a cloth particle in the **same
sweep**. Across the proxy the cloth only ever sees a jaw whose pose was synced from the
*previous* rigid solve, so a pinch is always resolved against a **one-step-stale pad**. That
is the entire reason `"vbd"` exists.

### 5.4 `"vbd"` — one `SolverVBD` owns everything (IN FLIGHT)

`_vbd_world()` (:3118) and `_build_vbd_world_solver()` (:3423). One `SolverVBD` owns rigid
bodies, joints **and** particles; there is **no `mj_model` in the world at all**.
`OMNISIM_NEWTON_VBD_WORLD` is the value-parsed override (`=1` forces, `=0` refuses).

Construction order is `CollisionPipeline` → `Contacts` → `SolverVBD`, which is what
`solver_vbd.py:174-175` specifies for CUDA-graph capture. `newton.eval_fk` is called
immediately before construction because VBD requires `body_q` to agree with `joint_q`
(`solver_vbd.py:169-172`); `OMNISIM_NEWTON_VBD_EVAL_FK=0` is the hatch.

**The price, and it is large.**

*OmniSim-side, because there is no `mj_model`:* raycast-backed devices (`DistanceSensor`,
`Receiver`, `LightSensor`, `Radar`, Camera-recognition occlusion), `Connector` /
`VacuumGripper` welds, `TouchSensor`, and the MuJoCo contact readback (`getContactPoints`,
`GET /sim/contacts`, `/sim/grips`) are all gone. Rigid contact becomes AVBD
penalty / augmented-Lagrangian rather than MuJoCo's friction cone, so **every** MuJoCo
tuning field is unread: `newtonCone`, `newtonImpratio`, `newtonIterations`,
`newtonLsIterations`, `newtonNoslipIterations`, `newtonNjmax`, `newtonNconmax`.

*Newton-side.* `SolverVBD`'s own limits, quoted from its class docstring at
`newton/_src/solvers/vbd/solver_vbd.py:119-135` — a path **inside the vendored Newton runtime
bundle, which is gitignored and exists in no clone**; on Windows it stages to
`msys64/mingw64/bin/newton-runtime/site-packages/`, and on Linux it is wherever `pip install
newton` put it in the system `python3`. Vendored newton **1.5.0**:

> **Joint limitations:**
> - Supported joint types: BALL, FIXED, FREE, REVOLUTE, PRISMATIC, D6, CABLE.
>   **DISTANCE joints are not supported.**
> - `Model.joint_enabled` is supported for all joint types.
> - `Model.joint_target_ke`/`joint_target_kd` are supported for REVOLUTE, PRISMATIC, D6
>   (as drives), and CABLE… VBD interprets `kd` as absolute damping in physical units.
> - `Model.joint_limit_lower`/`joint_limit_upper` and `joint_limit_ke`/`joint_limit_kd` are
>   supported for REVOLUTE, PRISMATIC, and D6 joints.
> - `Control.joint_f` (feedforward forces) is supported.
> - **Not supported:** `Model.joint_armature`, `Model.joint_friction`,
>   `Model.joint_effort_limit`, `Model.joint_velocity_limit`, `Model.joint_target_mode`,
>   **equality constraints, mimic constraints.**

⚠ **That list is a DOCSTRING and nothing enforces it.** READ, verified by grepping the vbd
solver directory for those attribute names: the only hits are the three docstring lines
themselves. No warning, no validation — the arrays are simply never read by any VBD kernel.
So OmniSim converts the silence into noise itself: `_vbd_capability_report(model)` (:3326)
reads the **finalized** model and logs one
`newtonSolver "vbd": IGNORED -- …` line per feature **this** world actually uses, naming
where each came from (`joint_effort_limit` ← `Motor.maxTorque`, `joint_velocity_limit` ←
`Motor.maxVelocity`, `joint_target_mode` ← the `POSITION_VELOCITY` DoF count).

**What still works:** joint PD drives. `joint_target_ke`/`joint_target_kd` are honoured on
REVOLUTE, PRISMATIC and D6, so a motorised gantry or gripper is drivable — **but the gains
are baked at solver construction, so changing them afterwards has no effect.**

### 5.5 Per-body cloth visibility — `Solid.newtonClothCoupling` (PLANNED)

Only on the **coupled** path. Without it, the `Proxy` declares
`bodies=list(range(n_bodies))` — every rigid body in the world becomes a live proxy inside
the cloth's collision view: tables, walls, every link of every arm.

READ, `_resolve_coupled_bodies` (:3228): newton supports narrowing this natively.
`_apply_entry_shape_visibility` (`solver_coupled.py:805-838`) **clears**
`COLLIDE_SHAPES | COLLIDE_PARTICLES | HYDROELASTIC` on every shape whose body is not a
proxy, so a narrow roster is both cheaper and more controllable.

- ✅ **Safe for the rigid side, and not by assumption.** The clearing writes through
  `view._cow_array("shape_flags")` — copy-on-write, view-local — so it never mutates the
  parent model and MuJoCo's own rigid-rigid collision is untouched.
- ⚠ **NOT safe for the cloth.** A body left off the roster is **invisible to the sheet**:
  the fabric falls straight through it, silently. The one thing that survives narrowing is
  WORLD/STATIC shape (newton's body −1, including the implicit z=0 ground plane). **An
  authored floor `Solid` is a real body and is not that** — narrow the roster and forget
  your table, and you get a sheet on the ground plane, or through the world.

Resolution, in precedence order: `OMNISIM_CLOTH_COUPLED_BODIES` (`"all"`, or a
comma-separated index list) → **allowlist** if any Solid declared `+1` → **denylist**
otherwise (every body except those declaring `−1`). A world using none of the field takes
the denylist branch with an empty denylist, i.e. the full roster it had before —
bit-identical. Resolving to **zero** bodies falls back to the full roster with a loud log
line, because refusing would kill the world and honouring it would build a `Proxy` newton
rejects outright.

⚠ **PLANNED, and currently unreachable from a world file.** The runtime side is complete
(`set_cloth_coupling`, :258; `_resolve_coupled_bodies`, :3228) but as of this writing
**MEASURED:** `newtonClothCoupling` does not appear in
[`resources/nodes/Solid.wrl`](../../resources/nodes/Solid.wrl), and no C++ call site
forwards it. Today the roster is reachable **only** through
`OMNISIM_CLOTH_COUPLED_BODIES`. `Cloth.wrl` already refers to the field as if it exists.

### 5.6 ⚠ FINDING: `"mujoco+vbd"` is declared everywhere and read nowhere

**READ, not run-verified** (the simulator was deliberately not run for this document).

The schema, `Cloth.wrl`, `SoftBody.wrl`, `docs/developer/cloth-simulation.md` §0, and the
runtime warnings at [`OmCloth.cpp:264`](../../src/omnisim/nodes/OmCloth.cpp) and
[`OmSoftBody.cpp:375`](../../src/omnisim/nodes/OmSoftBody.cpp) all state that
`newtonSolver "mujoco+vbd"` is **the** condition under which a `Cloth` simulates. Tracing
the value through the code:

1. `set_solver_preference` stores the raw string in `self._solver_pref` (:273).
2. `git grep '_solver_pref'` over the runtime returns **four** sites: the store, and three
   comparisons — `== "mujoco_warp"` (the CUDA device pin), `in ("mujoco", "mujoco_warp")`
   (the provenance label written into the log and the `.newton.json` sidecar), and
   `== "vbd"` (`_vbd_world()`). **Nothing compares against `"mujoco+vbd"`.**
   `git grep -i vbd` over `src/omnisim/**/*.cpp,*.hpp` returns only comments and warning
   strings — no gate.
3. The coupled-solver build is gated on `has_cloth()` alone
   (`if self.has_cloth(): self._build_cloth_coupled_solver(_kw)`), i.e. on **any particle
   source having been authored** — and `has_cloth()` is just `cloth_grids or soft_grids`.
4. Neither `OmNewtonBackend::addClothGrid` nor `World.add_cloth_grid` consults the solver
   preference; `addClothGrid`'s five `return -1` paths are availability, null-args, a
   4,000,000-particle cap, a Python exception, and an empty range.
5. [`OmNewtonBackend.hpp:279`](../../src/omnisim/physics/OmNewtonBackend.hpp) states the
   actual rule correctly: *"the runtime only constructs SolverVBD when a cloth exists."*

**INFERRED from (1)–(5):** `"mujoco+vbd"` is behaviourally identical to
`""` / `"auto"` / `"mujoco"`, and a `Cloth` in a world declaring `newtonSolver "mujoco"`
should register particles and simulate normally. Its documented value is **declarative** —
it tells a reader the world intends a coupled solve — not causal.

This is exactly the failure mode §4.2 is about, inverted: a field that is documented as
load-bearing and is not read. **Do not resolve it from this document** — it belongs to the
capability lane, and the resolution is a choice between two defensible options: enforce the
declaration (refuse particle registration without it, matching every doc), or demote it to
an alias of `"mujoco"` and correct the four doc sites. See §9.

### 5.7 Cloth manipulation status — say exactly this and no more

**In-engine, cloth MANIPULATION by a robot gripper is NOT demonstrated.** A sheet parses,
registers particles, falls, self-collides, drapes over rigid geometry and renders — that
path works and is measured
([`docs/developer/cloth-simulation.md`](cloth-simulation.md), reference world
`projects/samples/demos/worlds/physics/newton_cloth_drape.omniworld`). No world in this tree shows
a gripper picking cloth up and holding it.

**It IS proven in a standalone newton prototype** — outside this tree, OmniSim absent:
5 approach angles, 0.18–0.37 mm tracking over a 250 mm lift, with a negative control failing
at zero pad–cloth contacts. That is what justifies building `newtonSolver "vbd"`, and it is
the extent of the claim. It is **not** an in-engine result and must never be reported as
one.

---

## 6. Choosing a solver — the decision table

| your world | value | why |
|---|---|---|
| ordinary rigid world, robots, arms, fleets | omit the field | `""` → CPU `mj_step`, deterministic, every device works |
| parallel RL training, thousands of envs | `"mujoco_warp"` | the only case where the GPU path pays. Even `newtonSubsteps`; declare `newtonNjmax`/`newtonNconmax` |
| cloth or a soft body **draping / falling / being pushed** | `"mujoco+vbd"` | keeps MuJoCo's joints, friction cone and every device; the fabric still sees the robot across the proxy |
| a rigid body must **GRIP** fabric | `"vbd"` **(IN FLIGHT)** | the only path where pad and particle are relaxed in one sweep. Read §5.4 first — you are giving up every `mj_model`-backed device |
| no cloth at all | never `"mujoco+vbd"` / `"vbd"` | you pay a second solver's setup for nothing |

---

## 7. Migration guidance

### 7.1 If you have existing `.wbt` worlds: do nothing

`.wbt` is read indefinitely. Not "deprecated", not "until the next major" — indefinitely,
for the same reason the `#VRML_SIM` header is still accepted years after the flip, and with
216 tracked files still exercising that path (§1.2).

### 7.2 If you want to migrate a world

```bash
git mv path/to/world.wbt path/to/world.omniworld
```

That is the entire content change: **zero edits to the file body.** But `git mv` is not the
whole job, because a world is referenced by name from several places that a rename does not
follow:

1. **Sibling artefacts.** A world's thumbnail and `<stem>_net` directory shadow it by stem.
   `OmWorldFileFormat::replaceExtension` handles this inside the engine; on disk, rename
   them alongside.
2. **Controllers, launchers, demo catalogues, docs.** `git grep -F 'world.wbt'` before you
   commit. `demos.json`, `tests/smoke/smoke_worlds.json`, benchmark work lists and
   `docs/**` all hold literal paths. `OmWorldFileFormat::resolveExisting` covers the engine
   side by falling back to the sibling extension, so a stale reference keeps working — but
   it will not fix a Python glob, and it will not fix prose.
3. **`.wbt`-only globs in tooling.** MEASURED: `omnisim/doctor.py:81` uses
   `DEMOS_WORLDS.rglob("*.wbt")`; `omnisim/cli.py:296` and
   `scripts/dev/batch_validate.py:474` document `--glob` with a `*.wbt` example. A migrated
   world **disappears from `omnisim doctor`'s world list** until the Python side gets its
   own funnel. See §9.

### 7.3 What breaks on migration

Nothing in the file. The failure modes are all reference-tracking (§7.2) and all visible as
"the world is not found" or "the world is not listed", never as changed physics.

### 7.4 What breaks regardless of extension — the real migration work

If you are porting a **Webots** world into OmniSim, the extension is the trivial part. In
rough order of how much time each has cost:

1. **`webots://` → `omnisim://`** on every `EXTERNPROTO`, texture and mesh URL. OmniSim does
   not accept `webots://` at all (§2) — assets silently fail to resolve.
2. **`Fluid` / `ImmersionProperties` / `Solid.immersionProperties`** must be deleted. They
   are hard ERRORs (§4.1) and take the exit code to 1.
3. **`WorldInfo.contactProperties` friction must be re-declared as `newtonGroundMu`** (§4.2).
   This is the one that changes physics silently. `--fail-on-warning` catches it;
   [`tests/benchmarks/omnibench/lane1/translation_audit.py`](../../tests/benchmarks/omnibench/lane1/translation_audit.py)
   catches it statically across a whole tree in milliseconds, and `--fix` declares the field
   from the world's own value.
4. **Loop-closing `SolidReference`** — the `endPoint` form reaching back to an existing
   `Solid` — **kills physics for the entire world** while `run-headless` prints PASS.
   MuJoCo is a tree-articulation solver; `SolverMuJoCo` construction dies and the world gets
   no Newton world at all. Only `--fail-on-warning` catches this class.
   (`solidName "<static environment>"` is fine — it is a tree edge, not a loop.)
5. **A `Solid` with no `boundingObject` collides with nothing** and is the usual cause of a
   body falling through a floor. Read the fields, not the pixels:
   `GET /scene/node/<def>` reports `boundingObject` and `physics` presence.

### 7.5 New worlds

Write `.omniworld`. Use the canonical lighting recipe
([`docs/WORLD_RECIPE.md`](../WORLD_RECIPE.md)) — `OmniSimSky {}`, `DEF SUN OmniSimSun {}`,
`DEF SUN_MARKER OmniSimSunMarker {}` — and bake the viewpoint with
`scripts/dev/set_viewpoint.py` rather than eyeballing it.

### 7.6 Verifying a migrated world

```bash
# load + finalize only, stops the moment the .newton.json sidecar exists
python -m omnisim run-headless path/to/world.omniworld --until-finalized --fail-on-warning

# many worlds through ONE engine (and it dodges the launch race)
python scripts/dev/batch_validate.py --glob 'projects/**/worlds/*.omniworld'
```

⚠ `--fail-on-warning` is not optional for a migration check. §4.2's entire failure mode is a
world that loads clean and simulates something else.

⚠ Verify Newton actually drove the run by reading the **sidecar**, not the log:
`<OMNISIM_LOG_PATH>.newton.json` carries `{backend, solver, finalised, degraded, runtime}`,
where `runtime` names the newton / warp / mujoco / mujoco_warp versions **and the model
device**. `OmLog` deletes any stale copy when it truncates the log at startup, so the file's
presence means "Newton drove *this* run". A missing sidecar after a too-short run proves
nothing.

---

## 8. A worked minimal `.omniworld`

Every construct below is unloadable by upstream Webots: the `#OMNISIM` header, the
`omnisim://` scheme, three `newton*` fields, and the `Cloth` node. This is the smallest
file that exercises the format's reason to exist.

```
#OMNISIM R2025a utf8

# A pinned sheet drapes over a table. Requires the coupled solver.

EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"

WorldInfo {
  gravity          9.81
  basicTimeStep    8              # cloth wants a small tick
  coordinateSystem "ENU"
  newtonSolver     "mujoco+vbd"   # SolverMuJoCo (rigid) + SolverVBD (particles),
                                  # coupled by SolverCoupledProxy over ONE Model
  newtonStatics    TRUE
  newtonGroundMu   1              # NOT WorldInfo.contactProperties -- that is not read
}
Viewpoint {
  position    -2.2 -2.4 1.9
  orientation 0.36 -0.30 -0.88 1.24
}
OmniSimSky {
}
DEF SUN OmniSimSun {
}
DEF SUN_MARKER OmniSimSunMarker {
}
DEF FLOOR Solid {
  translation 0 0 0
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.35 0.36 0.40 roughness 1 metalness 0 }
      geometry Box { size 6 6 0.1 }
    }
  ]
  boundingObject Box { size 6 6 0.1 }   # NO boundingObject => a hologram floor
}
DEF TABLE Solid {
  translation 0 0 0.25
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.55 0.42 0.30 roughness 1 metalness 0 }
      geometry Box { size 0.5 0.5 0.5 }
    }
  ]
  boundingObject Box { size 0.5 0.5 0.5 }
}
DEF SHEET Cloth {
  translation    -0.4 -0.4 1.0   # the sheet's local (0,0) CORNER, not its centre
  dimX           16              # 17 x 17 = 289 particles
  dimY           16
  cellX          0.05
  cellY          0.05
  mass           0.001           # PER-PARTICLE (kg) -- ~0.29 kg total here
  particleRadius 0.01
  fixTop         TRUE            # pin the local +Y edge; LOCAL, not world up
  diffuseColor   0.8 0.2 0.2
}
```

**What to expect, and how to check it.** The sheet starts flat at z = 1.0, hinged along its
+Y edge, and drapes over the table top at z = 0.5. **The proof is numeric, not visual:**

```bash
OMNISIM_CLOTH_TELEMETRY=$PWD/.build_tmp/cloth.jsonl \
OMNISIM_CLOTH_TELEMETRY_EVERY=25 \
python -m omnisim run-headless path/to/drape.omniworld --duration 60
```

Read `bbox_min[2]` (falling, must stay above the floor), `bbox_max[1]` (the pinned +Y edge,
must not move at all), `soft_contacts` (non-zero once it lands) and `nonfinite` (must be
absent).

⚠ **Remove `newtonSolver "mujoco+vbd"` and the world still loads and still renders the
sheet — at its authored rest pose, frozen.** A frozen sheet and a correctly-placed
unsimulated sheet are pixel-identical, so the one-shot warning naming the field is the only
signal separating "misconfigured" from "the physics is broken". (See §5.6: whether that
freeze actually happens on the current build is exactly the discrepancy this document could
not resolve without running the engine.)

---

## 9. Reconcile checklist — this document is not finished

**Do not treat this file as settled until every row is closed.** Owners are the two sibling
lanes, not this document.

| # | item | state today (MEASURED / READ) | owner |
|---|---|---|---|
| 1 | `"mujoco+vbd"` documented as load-bearing, never read (§5.6) | READ: four `_solver_pref` sites, none matching it; coupled build gated on `has_cloth()` | capability lane — **enforce or demote, then fix the four doc sites** |
| 2 | `Solid.newtonClothCoupling` | runtime complete; **absent from `Solid.wrl`**, no C++ plumbing. `Cloth.wrl` already cites it | capability lane |
| 3 | `newtonSolver "vbd"` | in the schema enum + runtime, **uncommitted**. Update §5.1/§5.4 from IN FLIGHT to shipped once committed | capability lane |
| 4 | `SoftBody` | `SoftBody.wrl`, `OmSoftBody.{cpp,hpp}`, factory registration all **untracked** | capability lane |
| 5 | `WORLD_WRONG_EXTENSION` diagnostic rule is stale | [`diagnostic_codes.py:120`](../../scripts/harness/diagnostic_codes.py) anchors `must be '\.wbt'\.$`; the new engine message is `must be '.omniworld' (or the legacy '.wbt').` — the rule can no longer match | migration lane; `tests/harness/test_diagnostics.py` should catch it |
| 6 | Python-side dual-read funnel | `omnisim/doctor.py:81` `rglob("*.wbt")`; `omnisim/cli.py:296` and `scripts/dev/batch_validate.py:474` document `*.wbt` globs. **There is no Python equivalent of `OmWorldFileFormat`** — the C++ funnel rule needs a twin | migration lane |
| 7 | Generators | `scripts/dev/omniworld.py` still documents `--out /tmp/w.wbt`; `headless_runner.py:441` writes the runaway sibling as `.omnisim_runaway_<stem>.wbt`. Per §1.1 everything that WRITES should emit `.omniworld` | migration lane |
| 8 | Cloth manipulation | in-engine: **not demonstrated**. Standalone prototype only (§5.7). Update the moment an in-engine world holds fabric | capability lane |

---

## 10. See also

- [`src/omnisim/core/OmWorldFileFormat.hpp`](../../src/omnisim/core/OmWorldFileFormat.hpp) — the extension funnel; the normative source for §1
- [`resources/nodes/WorldInfo.wrl`](../../resources/nodes/WorldInfo.wrl) — the authoritative `newton*` field contract
- [`resources/nodes/Cloth.wrl`](../../resources/nodes/Cloth.wrl) · [`SoftBody.wrl`](../../resources/nodes/SoftBody.wrl) · [`GranularGroup.wrl`](../../resources/nodes/GranularGroup.wrl) — deformable field contracts
- [docs/developer/cloth-simulation.md](cloth-simulation.md) — the coupled cloth path, its measurements and its limits
- [docs/guide/newton-physics-backend.md](../guide/newton-physics-backend.md) — the Newton backend end to end
- [docs/guide/friction-grasp.md](../guide/friction-grasp.md) — why the robot stays on `SolverMuJoCo` on the coupled path
- [docs/benchmarks/determinism-scope.md](../benchmarks/determinism-scope.md) — what "deterministic" is scoped to, per solver
- [docs/reference/worldinfo.md](../reference/worldinfo.md) · [docs/reference/solidreference.md](../reference/solidreference.md)
- [docs/WORLD_RECIPE.md](../WORLD_RECIPE.md) — the canonical lighting recipe every new world must use

*If this doc and the code disagree, the code wins — update this doc in the same change.*
