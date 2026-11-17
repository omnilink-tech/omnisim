# OmniQuad — provenance

**OmniQuad** is OmniSim's generic 12-DoF quadruped: a rectangular chassis,
four fore/aft hip drums, four thigh plates, four shins ending in a spherical
foot pad. It ships Apache-2.0 with the rest of the tree (© OmniLink).

This file exists because `scripts/release/publish_snapshot.sh` publishes a
**squashed single commit** — git history does not travel to the public
repository, so provenance has to live in a file next to the geometry.

---

## 1. What this package was, and what changed

This package was previously `projects/robots/boston_dynamics/spot/`, carrying
13 `.dae` visual meshes, 13 `.stl` collision meshes and one texture obtained
from Clearpath Robotics' `spot_description` under BSD-3.

**Those meshes were not Clearpath's to license.** Every `.dae` names its one
geometry object

```
02-042137-001-A00 TOP LEVEL DEFEATURED - NOT FOR PRODUCTION
```

— a Boston Dynamics CAD part number plus a CAD release annotation, carrying
Blender's *duplicate* suffixes (`.001` … `.020`), i.e. **one imported CAD
assembly split 13 ways**. Boston Dynamics distributes exactly this to
customers as "Defeatured Spot CAD Models", behind a login, under
reserved-rights terms. Clearpath's BSD-3 validly covers what Clearpath
authored — the URDF, the per-link split, the materials, the decimated
collision hulls — but a licensor can only license what it owns, and it did
not own the *shape*. The full evidence chain is in
`docs/developer/spot-provenance-research.md` (archived 2026-09-02, see [docs/ARCHIVE.md](../../../../docs/ARCHIVE.md)).

So on **2026-08-22** the geometry was replaced with OmniSim's own and the
package renamed to a generic robot:

| | before | after |
|---|---|---|
| directory | `projects/robots/boston_dynamics/spot/` | `projects/robots/omnisim/omniquad/` |
| display name | Spot | OmniQuad |
| id / URDF robot name | `spot` | `omniquad` |
| world DEF | `SPOT` | `OMNIQUAD` |
| visual geometry | 13 `.dae` (BD CAD) | inline URDF primitives |
| collision geometry | 9 `.stl` + 4 primitives | 9 primitives + the same 4 |
| texture | `spot_mat.png` | none |

**Deleted:** 13 `.dae`, 13 `.stl`, `spot_mat.png` — 27 files, ~20 MB. No
binary geometry asset remains in this package. Primitive geometry is
human-readable, diffable, and its origin is verifiable by reading the file,
which is not true of a mesh.

---

## 2. Geometry — entirely own-authored

**No mesh, texture, CAD or model file from any third party is used in this
package, and none of its geometry is imported from, traced from, or derived
from any manufacturer's product model.**

Every visual shell *and* every collision solid is a `<box>`, `<cylinder>` or
`<sphere>` declared inline in the URDF, emitted by

```bash
python scripts/dev/gen_omnisim_robot_visuals.py --robot omniquad          # rewrite
python scripts/dev/gen_omnisim_robot_visuals.py --all --check             # CI gate
```

`--check` fails if the URDFs on disk have drifted from the authored spec. The
spec (shell composition, palette, collision primitives) lives in
[`scripts/dev/gen_omnisim_robot_visuals.py`](../../../../scripts/dev/gen_omnisim_robot_visuals.py)
under `OMNIQUAD` / `OMNIQUAD_COLLISION`, and covers all four variants:
`omniquad.urdf`, `omniquad.classic.urdf`, `omniquad_bigfoot.urdf`,
`omniquad_ghost.urdf`.

### 2.1 The collision primitives were fitted, not eyeballed

Replacing a *visual* mesh is cosmetic. Replacing a *collision* mesh is not —
it is what the solver sees. Two facts made this tractable:

1. **The feet were already own-authored primitives** in every variant
   (`<sphere radius="0.035">`, or a box in `_bigfoot`). Foot–ground contact —
   the dominant contact in every locomotion policy here — was never
   mesh-based and is **untouched by this change**.
2. **MuJoCo convexifies every triangle-mesh collider** (`solver_mujoco.py`
   sends both `GeoType.MESH` and `GeoType.CONVEX_MESH` to `mjGEOM_MESH`), so
   the shape the solver actually used was never the authored mesh — it was
   `ConvexHull(mesh)`.

So each of the 9 remaining mesh colliders was replaced by the single convex
primitive minimising the **voxel symmetric difference** against that hull
(the volume in one shape and not the other), not by matching a bounding box
or a volume by eye:

| collider | replacement | hull vol (m³) | primitive vol (m³) | ratio | sym-diff |
|---|---|---|---|---|---|
| `body` | box `0.8200 0.2150 0.1880` @ `0.0064 0 -0.0039` | 0.033918 | 0.033142 | **0.977** | 20.2 % |
| `*_hip` ×4 | cylinder r `0.0570` l `0.1480`, X-axis | 0.001491–0.001494 | 0.001511 | **1.011–1.013** | ~21 % |
| `*_upper_leg` ×4 | box `0.1063 0.1175 0.3940` | 0.004858–0.004870 | 0.004921 | **1.011–1.013** | ~30 % |
| `*_lower_leg` ×4 | *(unchanged — already `<sphere radius="0.035">`)* | — | — | — | — |

Every substituted volume lands **within 2.3 % of the hull it replaces**, and
the largest distance any *contact face* moved is small: **6.5–6.8 mm** on the
hips, **21.9–22.0 mm** on the upper legs, **35.9 mm** on the body (the tapered
nose/tail, which is where a box can least follow a hull). The four foot pads
did not move at all — they are the same `<sphere radius="0.035">` at
`z = −0.32` they always were. The
residual symmetric difference is the irreducible cost of representing a
rounded, tapered hull with one convex primitive; no single primitive does
better, and a multi-primitive `Group` is not an option here because OmniSim's
`WorldInfo.newtonCompoundColliders` defaults FALSE (a `Group` boundingObject
then registers `children[0]` and silently drops the rest).

Reproduce the fit with
`_baseline/fit_primitives.py` (archived 2026-09-02, see [docs/ARCHIVE.md](../../../../docs/ARCHIVE.md))
as described in §7.2 of the provenance research.

### 2.2 Nothing else moved

The kinematic skeleton and the dynamics are **byte-identical** across the
change, verified by parsing old and new URDFs into structured data and
diffing:

```
JOINTS changed    : 0   (of 68 across 4 files)
INERTIALS changed : 0   (of 39 present)
COLLISIONS changed: 27  (= 9 mesh colliders x 3 physics variants; the 12 foot
                         primitives and the ghost's zero colliders untouched)
```

Joint origins, axes, `<limit>`s, `<dynamics>`, the OmniSim `<rest>` extension,
link masses, centres of gravity and inertia tensors are unchanged. Inertials
were always **declared** in the URDF rather than derived from the meshes, so a
collider swap cannot perturb them.

---

## 3. Behaviour preservation — measured, not argued

A/B on one engine binary, interleaved arms, machine `9722d23d12a3`
(RTX 3060 Laptop), Newton `mujoco_warp`, `.newton.json` sidecar asserted on
every run. The control arm loads the *previous* CAD-mesh colliders through the
identical world, controller, policy and env; only the URDF differs.

| scenario | CAD-mesh colliders | authored primitives | verdict |
|---|---|---|---|
| `gpu_omniquad_walk_vc_main` velocity-conditioned walk, ~178 s sim, n=2 each | no fall; x@100 s = 24.601 m | no fall; x@100 s = 24.601 m | **bitwise identical over 11 067–11 336 logged steps; max &#124;Δpos&#124; = 0.00000000 m** |
| bare trot model, no policy, ~140 s sim | no fall; x@100 s = −5.033 m | identical | **bitwise identical, 8 744 steps** |
| `gpu_omniquad_walk_main` champion (falls at 3.73 s on this machine — pre-existing) | FALL@3.73 s, x = +1.66 | FALL@3.73 s, x = +1.66 | identical to the step; first divergence at 3.744 s, i.e. the first step *after* the body hits the ground |
| joint-limit impact stress (`tests/engine/joint_limits/`) | PASS, 0 violations, worst q̇ 4.21 / 4.24 / 7.84 rad s⁻¹ | PASS, 0 violations, 4.19–4.20 / 4.24 / 7.84 | equivalent |
| **`gpu_omniquad_getup_main` get-up, n=2 each** | **fails: flips at t≈5 s, ends inverted at base_z 0.1154, tilt 179.98°** | **succeeds: recovers to base_z 0.5396 (ref 0.5429), tilt 1.91°** | ⚠️ **CHANGED** |

Same-arm replicates were themselves bitwise identical, so the instrument is
deterministic and the comparison is meaningful.

⚠️ **The get-up result is a real behaviour change and is recorded as such, not
as a success.** It is an improvement — the policy now completes a recovery it
previously failed — but it is a change. It is also the expected place for one:
get-up is the only scenario in the set whose contact set is the **body and
thigh colliders**, which are exactly the nine that were substituted, and the
substituted body box is squarer and slightly smaller than the CAD hull's
tapered form, which changes the roll-over dynamics. Everything that walks is
bitwise unchanged.

---

## 4. What is *not* OmniLink's own work

The **kinematic skeleton and the declared dynamics** — link/joint topology,
link and joint names, joint origins, axes, limits, and the inertial values —
descend from Clearpath Robotics' `spot_description` URDF and remain covered by
its BSD-3 grant. That grant is sound for exactly this part: it is what
Clearpath actually authored. [`LICENSE.upstream`](LICENSE.upstream) is
retained in this directory to satisfy BSD-3 clause 1, and must not be removed
while the skeleton is in use.

So the split is:

| part | origin | licence |
|---|---|---|
| all visual geometry | OmniLink (this repo) | Apache-2.0 |
| all collision geometry | OmniLink (this repo) | Apache-2.0 |
| link/joint topology, origins, axes, limits | Clearpath Robotics `spot_description` | BSD-3 (`LICENSE.upstream`) |
| masses / inertia tensors | OmniSim-authored analytic primitives (see the URDF comments) over the above topology | Apache-2.0 |
| trained policies, ghosts, gait libraries, worlds | OmniLink (this repo) | Apache-2.0 |

**Trademark.** Nothing here uses "Spot", "Boston Dynamics" or any Boston
Dynamics mark, and the robot no longer resembles their product. OmniQuad is a
generic quadruped.

---

## 5. Files

```
omniquad/
  LICENSE.upstream            BSD-3 (Clearpath / ORI) — covers the skeleton, see §4
  omnisim.yaml
  PROVENANCE.md               this file
  urdf/
    omniquad.urdf             canonical: widened hip ROM for self-righting, <rest> tags
    omniquad.classic.urdf     pre-widen ROM, preserved because the Newton residual-RL
                              runs were trained against it
    omniquad_bigfoot.urdf     omniquad.urdf with box feet instead of spheres (8 lines)
    omniquad_ghost.urdf       visual-only (no collision, no inertial) — the Shadowing ghost
    make_ghost_urdf.py        generates omniquad_ghost.urdf from omniquad.urdf
  worlds/
    omniquad.omniworld        the demo world; also the canonical lighting-recipe reference
                              cited by AGENTS.md, ARCHITECTURE.md and docs/WORLD_RECIPE.md
```

There are **no binary assets** in this package.
