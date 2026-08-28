# Metazoa — implementation contract (P1 + groundwork for P2/P3)

Read `PLAN.md` first (the why). This file is the binding *what* for four
parallel implementers A–D. Where code and this file disagree, fix one in the
same change. Every engine trap in `projects/alife/README.md` applies.

## Thermal protocol (binding on the one agent that runs the engine)

Before EVERY run: `nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader`.
Above 70 °C: wait (sleep 60) and re-check; never launch above 70. One engine
process at a time, `--duration ≤ 60`, ≤ 24 cells. After every run confirm
`Get-Process -Name omnisim-bin` (PowerShell) is empty; `pgrep` does not exist.

## Layout

```
projects/metazoa/
  PLAN.md  DESIGN.md  README.md (A starts it with P1 numbers)
  mz/cell.py        [A] cell VRML + geometry constants
  mz/worldgen.py    [A] cells + scene -> .omniworld (Pose-wrapped, crypt slab)
  probe_p1.py + controllers/metazoa_probe_p1/   [A]  the P1 gate
  mz/organism.py    [B] PURE: gait CPG, body plan, steering, flip sequence
  mz/ecology.py     [C] PURE: charge, light, recruit/divide/shed/death/recycle
  mz/scene.py       [D] reef VRML: floor, walls, light patches, edge, lighting
  mz/surface.py     [D] PURE: HTTP verb routing/validation (threading contract)
  metazoa.py        [D] epoch driver skeleton; watch_metazoa.py [D] launcher
  tests/            [B, C, D]
```

## The cell (exact VRML conventions, A authors, everyone else reads)

Frame: the **tail block** is the Robot root, centred at the origin, box
0.06×0.06×0.06 spanning x ∈ [−0.03, 0.03]. The **hinge** sits at the seam
x = +0.03, axis **local y**. The **nose block** is the hinge endPoint Solid
with its centre at x = +0.06 (spans 0.03..0.09). Folded flat = 0 rad;
positive angle lifts the nose toward +z (right-hand about +y... verify sign
in the probe and state it in README).

```
Pose { translation X Y Z rotation 0 0 1 YAW children [
DEF CELL_<i> Robot {
  name "cell_<i>"  controller "<none>"  customData ""
  children [
    Shape {...tail block, colour...}
    DEF CELL_<i>_RING Shape { appearance DEF CELL_<i>_RING_APP PBRAppearance { emissiveColor 0 1 0.6 ... } geometry Cylinder{ height 0.004 radius 0.022 } }  # charge ring on top of the tail block, visual only
    DEF CELL_<i>_F_TAIL Connector { name "f_tail" translation -0.03 0 0 rotation 0 0 1 3.14159 type "symmetric" isLocked FALSE autoLock FALSE unilateralLock TRUE unilateralUnlock TRUE distanceTolerance 0.03 axisTolerance 0.45 rotationTolerance 0.45 numberOfRotations 4 snap FALSE }
    DEF CELL_<i>_HINGE HingeJoint {
      jointParameters DEF CELL_<i>_HINGE_PARAMS HingeJointParameters { axis 0 1 0 anchor 0.03 0 0 position 0 minStop -2.8 maxStop 2.8 }
      device [ RotationalMotor { name "hinge" minPosition -2.6 maxPosition 2.6 maxTorque 0.6 maxVelocity 5 } PositionSensor { name "hinge_sensor" } ]
      endPoint DEF CELL_<i>_NOSE Solid {
        translation 0.06 0 0  name "cell_<i>_nose"
        children [
          Shape {...nose block...}
          DEF CELL_<i>_F_NOSE  Connector { name "f_nose"  translation  0.03 0 0 rotation 0 0 1 0        ...same fields... }
          DEF CELL_<i>_F_LEFT  Connector { name "f_left"  translation 0 0.03 0  rotation 0 0 1 1.5708   ...same fields... }
          DEF CELL_<i>_F_RIGHT Connector { name "f_right" translation 0 -0.03 0 rotation 0 0 1 -1.5708  ...same fields... }
        ]
        boundingObject Box { size 0.06 0.06 0.06 }
        physics Physics { density -1 mass 0.12 }
      }
    }
  ]
  boundingObject Box { size 0.06 0.06 0.06 }
  physics Physics { density -1 mass 0.12 centerOfMass [0 0 0] inertiaMatrix [7.2e-5 7.2e-5 7.2e-5, 0 0 0] }
} ] }
```

Connector frame: the connector's **+x axis is its docking normal** (Webots
convention); the rotations above point each face outward. A Connector must
be a child of the Solid whose body it welds. `newtonRobotColliders TRUE` in
WorldInfo (else the root block's collider becomes a 1 mm sphere). Physics
config as alife: dt 8, `newtonSolver "mujoco"`, `newtonGroundMu 1.0`,
ke 8000 / kd 200, elliptic, impratio 10, `newtonNjmax/Nconmax 2048`.

**Docking geometry.** Two cells chain when cell j's tail face meets cell i's
nose face: j's origin at i's origin + R_i·(0.12 + gap, 0, 0) with j yawed
like i, and optionally **rolled 90° about its own x** (`dock_rotation` ∈
{0, 1, 2, 3} × 90°) so consecutive hinge axes alternate pitch/yaw. Side
faces (±y of the nose block) take a branch cell whose tail face meets them:
branch origin at nose centre + R·(0, ±(0.03+0.03+gap), 0), yawed ±90°.

**Docking is a supervisor field write**: set the *active* side's
`isLocked` field TRUE (`Connector.isLocked` → `lock()`; the engine welds only
if a partner face is within tolerance). Unlock = FALSE. Both faces are
"symmetric", so either side may lock. Never call device APIs — cells have no
controller.

**Actuation**: `CELL_<i>_HINGE_PARAMS.position` batched field writes (as
alife). **Charge ring**: write `CELL_<i>_RING_APP.emissiveColor`.
**Limp**: write the motor's `maxTorque` field to 0 (and back).

## The organism (B, pure, `mz/organism.py`)

- `Genome` (dict): `{A, omega, dphi, bias_pitch, bias_yaw, branch_phase,
  branch_scale, steer_gain}` with ranges A 0.3–1.2, omega 2–8 rad/s, dphi
  0.6–2.4, biases ±0.6, branch_scale 0–1, steer_gain 0–0.6; `random_genome`,
  `mutate`, `validate`.
- `BodyPlan`: `{target_length 2–8, dock_rotation_pattern: list of 0..3
  cycled along the chain, branch_rule: "none" | {"at": k, "sides": ["L","R"]}}`.
- `axis_of(bodyplan, i)` → "pitch" | "yaw" for cell i (from the cumulative
  roll of the dock rotations; pitch = hinge axis horizontal-transverse, yaw =
  hinge axis vertical).
- `chain_targets(genome, bodyplan, n, t, steer=0.0)` → `[target_i]`: a
  travelling wave `bias_axis + A·sin(omega·t + i·dphi)` where pitch cells get
  `bias_pitch`, yaw cells get `bias_yaw + steer_gain·steer` (steer ∈ [−1, 1]
  = yaw bias asymmetry — this is the steering channel P1/P2 measure).
  Branch cells: `branch_scale·A·sin(omega·t + branch_phase ± …)`.
- `flip_sequence(t, period=2.0)` → hinge target for a lone cell: fast fold to
  +2.4 rad over 25 % of the period, hold 10 %, slow unfold to 0 over 65 %
  (the M-TRAN somersault; direction is the cell's +x). Lone cells cannot
  turn; **recruitment is done by the organism approaching the free cell**
  (its nose or an open side face), so also `approach_pose(cell_pose,
  free_face_pose, gap)` → the organism-head pose that mates with the free
  cell's face, and `heading_error(head_pose, target_xy)`.
- Tests: wave phase progression, axis pattern for rotations [0,1,0,1],
  steering sign, flip sequence shape, approach pose for a free cell at 90°.

## Ecology (C, pure, `mz/ecology.py`)

- Cell: `charge_wh` (cap 12, start 6), `alive`, `organism` id or None,
  `debris_since`. Drain/s = 0.05 + 0.4·A·omega/(2π) (gait work; lone flip =
  0.2), light +2.0 W within a patch radius 1.2 m (any cell of an organism in
  light charges the whole body's pool). Time scale ×20. Organisms equalise
  charge across members every tick (mean).
- Organism: members (ordered spine list + branches), genome, bodyplan,
  lineage. Rules: `seek_light` when mean charge < 40 %; `recruit` when
  > 60 % and len < target_length (target = nearest free cell); `divide` when
  len ≥ target_length and charge > 80 % → two organisms, spine split at the
  midpoint, both genomes `mutate`d, lineage = parent id; `shed` tail cell
  when charge < 10 %; a cell at 0 → limp, unlocked, `debris`; debris → after
  20 s teleported to the recycling edge as a fresh free cell at 50 % (the
  conserved population: **assert total cells constant every tick**).
- Light patches: 5, radius 1.2, drift at 0.05 m/s along a slow Lissajous;
  `dim` factor scales charge rate.
- `epoch_result()` per lineage: divisions·10 + cells recruited + light
  collected/10 + mean organism length; deterministic re-score contract as
  alife (never compare evolution-time scores across epochs).
- Tests: conservation under every transition, sharing equalises, division
  splits spine and mutates, shed order, recycle timer, light drift stays in
  the arena.

## Reef + surface + driver (D)

- `mz/scene.py`: `scene_lines(arena=18, n_patches=5)` starting at the
  Viewpoint (worldgen writes header/EXTERNPROTO/WorldInfo — same convention
  as `projects/alife/alife/scene.py`, copy its helpers): dark floor Box,
  0.25 m walls, `DEF PATCH_<k>` visual-only discs (radius 1.2, height 0.01,
  emissive warm white, NO physics) the supervisor moves by translation,
  `DEF EDGE` a subtle lighter ring 0.6 m inside the walls (visual), crypt
  slab far away, canonical lighting, hero Viewpoint, DIRECTOR supervisor
  `controller "metazoa_world"` `synchronization TRUE`.
- `mz/surface.py`: pure request router for `GET /capabilities`, `GET
  /census`, `POST /light {k,x,y}`, `POST /split {organism}`, `POST /recruit
  {organism, cell}`, `POST /dim {factor}`; validation, 404/400/409 shapes,
  measured-result envelopes; the sim-thread marshalling queue as a reusable
  class (copy the contract from `projects/alife/controllers/terrarium_life`).
- `metazoa.py` driver skeleton (`--epochs --cells 24 --organisms 6 --epoch-s
  --arena --seed --dry-run --resume`), `watch_metazoa.py` (lean render env
  from `projects/alife/watch.py`, `--build-only`, seeds fallback).
- Tests for surface routing and driver dry-run.

## P1 gate (A runs it; numbers into README.md)

World `probe_p1`: 12 cells on a 12 m floor.
1. **Chain of 4 (cells 0–3)** pre-placed nose-to-tail, gap 0.01, rotations
   [0,0,0,0]. At t = 1 s set each junction's active `isLocked` TRUE. Drive
   `chain_targets` with A 0.8, omega 5, dphi 1.57 for 20 s. Report: max
   junction separation (weld holds ⇔ ≤ 0.02 m), chain centroid speed (gate
   > 0.15 m/s), and whether the chain stays upright.
2. **Alternating chain of 4 (cells 4–7)**, rotations [0,1,0,1]: same drive
   with `bias_yaw` steer = +0.5 for 10 s then −0.5: report curvature sign
   and magnitude (the steering channel).
3. **Negative control (cells 8, 9)**: placed 0.10 m apart (outside
   tolerance); set `isLocked` TRUE; flip cell 8; cell 9 must NOT follow
   (report its displacement, gate < 0.01 m).
4. **Lone flip (cell 10)**: `flip_sequence` for 20 s; report net forward
   progress along its +x (gate > 0.3 m) and whether it ends upright.
5. **Cost**: a second world of 24 resting cells: engine ms/step (gate ≤ 8).
Write `_run/probe_p1.json`; every number also into `README.md` with the
machine id.
