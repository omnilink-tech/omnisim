# The Ghost Store

Every **ghost** — the achievable reference a Shadowing policy tracks — lives here, partitioned by
the robot it is for.

```
ghosts/
  g1/     ghost_*_lut.json          the flagship humanoid lineage (+ dance/ retargets)
  h1/     ghost_h1_lut.json
  go2/    go2_shadow_ghost_lut.json the quadruped Shadowing ghost
```

## Provenance and licence — read before adding a ghost

A ghost is **data**, and some of it has third-party ancestry. Every lut declares its own lineage in
its `source` field, and that field decides whether the file may be redistributed:

* **[`g1/README.md`](g1/README.md)** — the four lineage classes and which ones ship.
* **[`docs/developer/motion-data-provenance.md`](../../../docs/developer/motion-data-provenance.md)**
  — the full ruling, with licence text and URLs.

The short version: ghosts we synthesised are unrestricted; ghosts derived from the Ubisoft **LAFAN1**
mocap dataset are **CC BY-NC-ND 4.0** and cannot be redistributed *even as derivatives*, so they and
their descendants are excluded from the public snapshot. When you build a new ghost, **write the real
ancestry into `source` at build time** — a lut whose lineage cannot be reconstructed from the artifact
itself is unshippable by default.

## Why it moved here

It used to be `projects/policies/controllers/g1_ghost/` — a directory named for **one robot**,
inside a **controller**, that nonetheless held every ghost in the repo *including the H1's*. That
is not a filing quirk; it is the shape of the bug:

* The canonical loader (`skills/ghost_lut.py`) hardcoded that directory as the ghost store, so the
  library's own sweep (`skill_lib.py ghost --all`) globbed it and reported **"92 luts, 0 errors"**
  while never once looking at a **quadruped** ghost — those lived under `research/`, invisible to
  the tool that was supposed to be checking them.
* Because there was no per-robot partition and no `robot` key, a 12-wide *quadruped* lut and a
  12-wide *humanoid* lut were indistinguishable, and every tool's fallback guess was "G1,
  positionally".

## The contract (schema 2)

A lut **declares its own identity**. `skills/ghost_lut.py` requires:

| key | meaning |
|---|---|
| `robot` | which robot this ghost is FOR (`g1`, `h1`, `go2`, …) — must match its store partition |
| `joints` | its **real** leg-joint names, in column order. Never the positional fallback. |
| `leg_lut` | the phase-indexed reference, `nb` rows × `len(joints)` columns |
| `validator` | the **stamped** `ghost_validator.py` verdict (`--stamp`) — recommended |
| `provenance` | structured provenance — recommended (legacy luts carry free-text `source` only) |

The **real** joint names matter literally. The Go2 ghost used to carry *fake humanoid aliases*
(`FL_hip_roll_joint`) with its real names (`FL_hip_joint`) hidden in a side key — purely so that a
humanoid-name-keyed symmetry gate would engage on a quadruped. **A rule you have to lie to is the
wrong rule.** Joint roles now come from the shared registry
([`common/robot_registry.py`](../common/robot_registry.py)), which derives role and mirror from the
name across every naming family we ship (`left_/right_`, `FL_/FR_/RL_/RR_`, `front_left_/…`).

## Adding a ghost

```bash
# 1. validate it against the REAL robot model. An unregistered robot FAILS -- it does not skip.
python projects/policies/training/ghost_validator.py projects/policies/ghosts/<robot>/<lut>.json

# 2. stamp the verdict INTO the artifact (so no manifest can overclaim it later)
python projects/policies/training/ghost_validator.py projects/policies/ghosts/<robot>/<lut>.json --stamp

# 3. sweep the whole store (every robot -- there is no G1-only view any more)
python projects/policies/skills/skill_lib.py ghost --all
```

If your robot is not in the registry, **add it there** (`common/robot_registry.py`) rather than
working around the gate. The gate refusing to vouch for an unknown robot is the feature: it used to
skip every model-aware check and still print `VERDICT: PASS`.

Raw rollout recordings and the older research lineage (`.npz`, `.csv`) are **not** ghosts and stay
in [`../research/shadowing/ghosts/`](../research/shadowing/ghosts/) — a ghost is the phase-indexed
LUT a policy actually tracks.
