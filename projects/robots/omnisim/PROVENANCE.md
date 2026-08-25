# OmniSim reference robots — provenance

All **geometry** under `projects/robots/omnisim/` is **original work of this
repository**, licensed Apache-2.0 like the rest of the tree (© OmniLink).

⚠️ **One carve-out:** `omniquad/` is a *restyled* package, not a from-scratch
one. Its geometry is entirely ours, but its **kinematic skeleton and declared
inertials** descend from Clearpath Robotics' BSD-3 `spot_description` URDF and
are still covered by that grant — `omniquad/LICENSE.upstream` is retained for
BSD-3 clause 1 and must not be removed. See
[`omniquad/PROVENANCE.md`](omniquad/PROVENANCE.md) §4.

This file exists because `scripts/release/publish_snapshot.sh` publishes a
**squashed single commit** — git history does not travel to the public
repository, so provenance has to live in a file next to the geometry.

## The packages

| Package | Robot | DoF | Notes |
|---|---|---|---|
| [`omniarm6/`](omniarm6/) | OmniArm 6 | 6 | Generic collaborative arm, 800 mm reach. Seven end-effector variants. |
| [`omniarm7/`](omniarm7/) | OmniArm 7 | 7 | Generic redundant-DoF collaborative arm, 1.515 m shoulder-to-flange. |
| [`omnitug500/`](omnitug500/) | OmniTug 500 | — | Generic four-wheeled warehouse tug (AGV). Visual-only prop. |
| [`omniquad/`](omniquad/) | OmniQuad | 12 | Generic quadruped. **Visual *and* collision** geometry authored here; skeleton is BSD-3, see [`omniquad/PROVENANCE.md`](omniquad/PROVENANCE.md). |

## Geometry

**No mesh, texture, CAD or model file from any third party is used in these
packages, and none of their geometry is imported from, traced from, or derived
from any manufacturer's product model.**

Two forms, both generated here and both text:

* **Primitive solids** — `<box>`, `<cylinder>`, `<sphere>` declared inline in the
  URDF (OmniArm 7, OmniTug 500, OmniQuad, the 140 mm gripper).
* **Surfaces of revolution** — OmniArm 6's visual shells, emitted as `.obj` by
  [`scripts/dev/gen_omniarm_meshes.py`](../../../scripts/dev/gen_omniarm_meshes.py)
  from authored `(r, z)` profiles, with analytic per-vertex normals. Bare
  cylinders cannot express a taper or a filleted rim, and the arm read as a toy
  without them.

**OBJ is a deliberate choice over a binary mesh format.** It is plain text, so a
reviewer can diff it, and `--check` proves byte-for-byte that what ships is what
the committed script emits. There is no `.stl`, `.dae`, `.glb` or texture file
anywhere in these packages, and no binary whose origin has to be taken on trust.

The arm and quadruped shells are emitted by
[`scripts/dev/gen_omnisim_robot_visuals.py`](../../../scripts/dev/gen_omnisim_robot_visuals.py),
which holds the shell specification and can regenerate them:

```bash
python scripts/dev/gen_omniarm_meshes.py                # OmniArm 6 shells (.obj)
python scripts/dev/gen_omnisim_robot_visuals.py --all   # the URDF visual blocks
# add --check to either for the CI drift gate
```

`--check` fails if the URDFs on disk have drifted from the authored spec.
The OmniTug 500 URDF is authored directly (one link) and is not generated.

## Physical parameters

Joint origins, axes, position/velocity/effort limits, link masses, centres of
gravity, inertia tensors and collision primitives describe generic hardware of
each robot's class and are **carried over unchanged from the models these
packages replace**. That is intentional: the restyle is cosmetic by design, so
existing worlds, controllers, bridges and trained policies (ghosts, ONNX
policies) remain valid without retraining or retuning.

⚠️ **`omniquad/` is the exception on the collision half**: nine of its
colliders *were* third-party meshes and had to be replaced too. They were
fitted to the convex hulls MuJoCo actually used (volumes within 2.3 %) and the
substitution was A/B-measured against the meshes it replaces — bitwise
identical for every walking scenario, with one recorded divergence in the
get-up policy. Numbers and method: [`omniquad/PROVENANCE.md`](omniquad/PROVENANCE.md) §2.1 and §3.

Consequently the visual shells are *not* derived from the collision or inertial
data and do not need to match them exactly — visuals do not participate in
physics. Collision geometry (primitive cylinders) and the inertials are what
the solver sees.

## Third-party geometry reachable from these packages

**None.**

> ⚠ **Corrected 2026-08-24.** This section used to read: *"One variant,
> `omniarm6/omniarm6_2f140_grip.urdf`, references gripper meshes in
> `projects/robots/robotiq/2f140/` — a separate package with its own
> provenance."* Both halves are now false. Re-measured: every `filename=` in
> every OmniArm 6 URDF resolves to that package's own `meshes/link*.obj` (16
> references, 16 files, no `package://`), and
> [`projects/robots/robotiq/`](../robotiq/PROVENANCE.md) holds **no binary
> geometry at all** — its 2F-140 is URDF primitive solids emitted by
> `gen_omnisim_robot_visuals.py --robot gripper140`. The reference was true when
> written and was left behind when the gripper meshes were replaced.
