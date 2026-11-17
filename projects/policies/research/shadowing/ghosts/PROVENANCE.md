# Provenance — Shadowing reference motions (`ghosts/`)

**Status: RESOLVED. All 18 tracked files are original work of this repository**,
Apache-2.0 (© OmniLink). **None of them contains motion-capture data, and none
of them is derived from any third-party motion dataset.**

This file exists for two reasons. `scripts/release/publish_snapshot.sh` publishes
a squashed single commit, so git history does not travel to the public repository
and provenance has to live beside the data. And a "ghost" is a **reference
motion** — exactly the asset class where a mocap dataset would hide, and where
this repository already carries a live, documented licence blocker (see the
LAFAN1 section below). Silence here would be the wrong answer.

## What is here — measured 2026-08-24

| count | files | form |
|---:|---|---|
| 16 | `*.npz` | NumPy archives: float64/float32 arrays plus one small `<U` string array of joint names |
| 2 | `*_replay.csv` | plain-text pose tables (`x,z,rx,ry,rz,ra` + per-joint columns) |

The `.npz` files hold, per motion: `q` (full qpos incl. free base), `qvel`,
`ctrl` (the position targets actually applied), `base`, `com`, `feet`,
`init_qpos`, a scalar `dt`, and terrain scalars on the hill set
(`hill_grade_deg`, `hill_ramp_run`, `ride`, `vx`, …). **Every array is numeric.**
The only non-numeric member of any file is `joints`, a fixed-order list of the
robot's own URDF joint names (`FL_hip`/`FL_thigh`/`FL_calf`, `FL_hip_x`/`FL_hip_y`/
`FL_knee`, `left_hip_pitch_joint` …, `joint1`…`joint6`). There is no author
field, no tool tag, no source-file path, no timestamp and no vendor string in any
of the 16. Re-check with:

```bash
python - <<'EOF2'
import glob, numpy as np
for f in sorted(glob.glob('projects/policies/research/shadowing/ghosts/*.npz')):
    z = np.load(f, allow_pickle=True)
    print(f)
    for k in z.files:
        a = z[k]
        extra = ' -> %r' % a.ravel().tolist()[:4] if a.dtype.kind in 'USO' else ''
        print('   %-18s %s %s%s' % (k, a.dtype, a.shape, extra))
EOF2
```

> ⚠ The joint NAMES on the G1 and B2 sets follow those robots' own URDF
> conventions, which descend from Unitree's published models. Names are the
> interface, not the motion — no sample value comes from anywhere but the
> generators below — but the fact is recorded here rather than left for someone
> to notice.

## How each one was produced — every generator is committed

| file(s) | generator |
|---|---|
| `g1_sitstand_ghost.npz` | [`../generate_g1_sitstand.py`](../generate_g1_sitstand.py) — receding-horizon MPPI over the G1's own MJCF, given only a seated start, a standing goal and a balance cost |
| `g1_sitstand_ref.npz` | [`../ghost_to_trainer_ref.py`](../ghost_to_trainer_ref.py), from the row above |
| `b2_getup_ghost.npz` | [`../generate_b2_getup.py`](../generate_b2_getup.py) — MPPI discovering a push-up from a sprawled start |
| `omniquad_{crouch,getup,jump}_ghost.npz` | [`../../tools/generate_omniquad_crouch.py`](../../tools/generate_omniquad_crouch.py), `…_getup.py`, `…_jump.py` |
| `{b2,omniquad}_hill*_ghost.npz` | [`../generate_hill_walk.py`](../generate_hill_walk.py) — the analytic trot gait evaluated on a `hill_terrain.HillProfile` incline |
| `omniarm6_toss_{ghost,far}.npz` | [`../generate_omniarm6_toss.py`](../generate_omniarm6_toss.py) — a designed sagittal swing played through a torque- and velocity-limited OmniArm 6 model, plus exact projectile ballistics |
| `g1_{braceguard,standwave}_replay.csv` | [`../generate_g1_braceguard.py`](../generate_g1_braceguard.py) / [`../generate_g1_standwave.py`](../generate_g1_standwave.py) → [`../ghost_to_replay_csv.py`](../ghost_to_replay_csv.py) |

Every one of these is a **planner or an analytic gait model running against a
MuJoCo model of the robot inside this repository**. There is no recording step,
no retarget step, and no external input file: the inputs are the robot's MJCF,
a start state, a goal and a cost.

## ⚠ These are NOT the LAFAN1-lineage ghosts — do not confuse the two directories

A different directory, `projects/policies/ghosts/g1/`, holds ghosts that **are**
derived from Ubisoft La Forge's LAFAN1 motion-capture dataset (CC BY-NC-ND 4.0),
which cannot ship in an Apache-2.0 repository. Those are enumerated and excluded
in `scripts/release/publish_deny.txt`, with the full licence argument in
[`docs/developer/motion-data-provenance.md`](../../../../../docs/developer/motion-data-provenance.md).

**Nothing in this directory is in that lineage**, and nothing here is
deny-listed. The two are told apart by *how the motion was obtained*, not by
name: a LAFAN1 ghost begins with recorded human frames; a ghost here begins with
a cost function and a solver.

## Adding a ghost here

Commit its generator alongside it, and name that generator in the table above.
A ghost whose generator is not in the tree cannot be shown to be ours — which is
precisely the position `projects/policies/ghosts/g1/` had to be dug out of.
