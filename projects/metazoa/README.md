# Metazoa — robots that grow into robots

`PLAN.md` is the why, `DESIGN.md` the binding what. This file is what
happened when the design met the engine, phase by phase, numbers first,
failures included (the `projects/alife/README.md` model).

Everything below is measured on machine **`9722d23d12a3`** (RTX 3060 laptop,
Windows 11, AMD64 16 cores; `python projects/policies/common/env_fingerprint.py`),
engine binary sha256 `785786e4621110d4` (build `04c7b2c45`), libController
`98374ba6f6adb81a`, Newton 1.5.0 / warp 1.16.0 / MuJoCo 3.11.0, CPU `mj_step`,
`newtonSolver "mujoco"`, dt 8 ms. GPU stayed at 57–60 °C for every launch;
one engine at a time; no `omnisim-bin` left behind after any run.

Reproduce: `python projects/metazoa/probe_p1.py` (P1, cell v1, ~35 s wall)
and `python projects/metazoa/probe_p1.py --v2 --no-flip --tag b --A 0.9
--omega 4.5 --dphi 1.2` (P1b, cell v2) and the same with `--v3 --tag c`
(P1c, cell v3). Results `_run/probe_p1.json`, `_run/probe_p1_b.json`,
`_run/probe_p1_c.json`, logs + `.newton.json` sidecars next to them. Every
number here is from those JSONs, never from a log scrape. The probes are
deterministic: a repeat run reproduces every float bit-for-bit.

## P1 — the cell probe (cell v1, weld at MuJoCo's default stiffness)

World `worlds/probe_p1.omniworld`: 12 cells on a 12 m floor, one supervisor
(`controllers/metazoa_probe_p1/`), cells are `controller "<none>"`. Docking
is a supervisor **field write** (`CELL_<i>_F_NOSE.isLocked` → the engine's
`lock()`), actuation is batched `CELL_<i>_HINGE_PARAMS.position` writes,
measurement is `getPosition()` / `getOrientation()` only. Load evidence, every
run: `registered 24 dynamic + 6 static Newton bodies`, **12 `motorized`**
hinges (position-servo branch), `0 errors, 0 warnings`, sidecar
`finalised: true`, `device: cpu`.

### P1, P1b and P1c gate tables side by side

P1: cell v1, DESIGN gait A 0.8 / ω 5 / Δφ 1.57, maxTorque 0.6, substeps 1,
weld `eq_solref` at MuJoCo's default. P1b: cell v2 (belly rollers),
maxTorque 0.35, `newtonSubsteps 4`, runtime weld fix (`eq_solref =
[2·mj_timestep, 1]` = 4 ms), best gait of the sweep A 0.9 / ω 4.5 / Δφ 1.2.
P1c: cell v3 (belly + side rollers, Coulomb bearing brake 0.001 N·m),
otherwise as P1b, same best gait.

| gate | P1 (v1) | P1b (v2) | P1c (v3) |
|---|---|---|---|
| weld holds: max junction sep ≤ 0.02 m (pitch 4-chain) | 0.092 / 0.070 / 0.136 m — FAIL (elastic) | **0.0108 / 0.0102 / 0.0106**, rest 0.0100 — PASS | **0.0106 / 0.0101 / 0.0105**, rest 0.0100 — PASS |
| alternating-chain junctions | 0.081 / 0.050 / 0.153 — FAIL | 0.0112 / 0.0115 / 0.0118, rest 0.0107 — PASS | **0.0110 / 0.0106 / 0.0108**, rest 0.0100 — PASS |
| (a) pitch-only 4-chain speed (gate 0.15 m/s) | 0.044 m/s | **0.075 m/s** (1.43 m / 20 s) | 0.061 m/s (1.17 m / 20 s; best 5 s 0.071) — the brake costs 20 % |
| chain stays upright | cell 3 on its side — FAIL | 1.0 ×4 — PASS | 1.0 ×4 — PASS |
| (b) alternating [0,1,0,1] lateral undulation | 0.009 m/s | 0.012 m/s (0.004 / 0.026 per phase) — FAIL | **0.030 m/s** net over 20 s while steering (0.032 / 0.030 per phase; paths 0.28 / 0.26 m) — translates, 2.5× v2 |
| (c) steering ±0.5 yaw bias | not measurable | pivot in place ∓2.0 rad / 8.8 s, sign inconsistent on the − side across the sweep | **curvature −5.7 rad/m at +0.5, +7.7 rad/m at −0.5**; turn −1.59 / +2.05 rad; sign antisymmetric at **all 4** grid points |
| (e) negative control: cell 9 at 0.10 m | 2.9e-8 m — PASS | 2.9e-8 m — PASS | **5.8e-5 m** (sep min 0.0956, end 0.123) — PASS |
| lone flip > 0.3 m | −0.003 m — FAIL (attributed) | dropped by decision | dropped |
| bearing: witness coast after one 0.6 rad fold (gate < 0.5 m) | 0.003 m (block on floor) | **5.86 m** (frictionless bearing) | **0.037 m** — PASS |
| (d) 24 resting cells ≤ 8 ms/step | 1.06 ms (48 bodies) | 1.98 ms (96 bodies) | **3.40 ms** median, p90 4.08 (**144 bodies**, 120 motorized) — PASS |
| 12-cell probe world, actuated | 2.6–3.0 ms/step | 6.3 ms/step (48 bodies) | 8.7–9.0 ms/step (72 bodies, substeps 4) — over the 8 ms tick |

### Hinge sign convention (measured, cell 11; identical in v1 and v2)

Command `+0.6` rad on `HINGE_PARAMS.position` → the nose centre reads
`(+0.0548, 0, −0.0169)` in the tail frame, i.e. the nose rotated **−34° = −0.6 rad
about the seam, DOWN toward tail-frame −z**; the field reads back 0.599 (v1)
/ 0.580 (v2, the servo is softer at maxTorque 0.35). So the engine is the
plain right-hand rule about the authored axis `0 1 0`: **positive target =
nose toward −z (into the floor when the cell lies flat); negative = nose
lifts toward +z.** DESIGN.md's "positive lifts the nose" is the wrong half of
its own sentence — "right-hand about +y" is correct and it drops the nose.
B's `organism.py` must use this sign (a lift is negative). For a cell rolled
90° (a yaw hinge, local +y = world +z at yaw 0), positive bends the nose
toward local −z = the cell's **right** (−y of the head frame).

### Roller convention (cell v2, `mz/cell.py` `rollers=True`)

`CELL_<i>_ROLL_T` / `CELL_<i>_ROLL_N`: a passive `HingeJoint` (no device, no
limits) on the bottom of each block, `axis 0 1 0` = the block's local y,
anchor at (0, 0, −0.022), carrying `CELL_<i>_ROLL_{T,N}_SOLID`, a cylinder
r 0.012 × 0.04, 0.01 kg, turned onto local y with `Pose { rotation 1 0 0
1.5708 }` (the engine's Cylinder is Z-aligned; the runtime substitutes a
capsule of the same radius). It protrudes 4 mm: a v2 cell rests with its
block centres at **z 0.0339** (v1: 0.0299). The cell rolls freely along its
own x and resists y; a yaw-rolled cell's rollers point sideways and never
touch (intended). Motor maxTorque 0.35 (P1: upright at 0.2, tips at 0.6).

**A v2 cell has zero rolling resistance.** The sign witness's single 0.6 rad
fold-and-unfold sent it coasting **5.86 m along −x** to the west wall (v1:
0.003 m). In the first four P1b runs it was authored in the negative pair's
row and rolled into cell 9 from behind, pushing it 0.109 m to within 0.010 m
of cell 8's nose — which read as "the negative control failed" until the
trace showed cell 11 at cell 9's position. The layout now gives the witness
its own row and the control passes; the lesson for P2/P3 is that an
unlocked v2 cell keeps whatever x-velocity it was left with until it hits
something, and a resting free cell is a billiard ball for any organism that
brushes it.

### Roller convention, cell v3 (`rollers="v3"`, P1c)

Four passive rollers per cell, 6 bodies: `CELL_<i>_ROLL_{T,N}B` on the
bottom (−z) face of the tail / nose block, axle local **y**, at (0, 0, −0.022);
`CELL_<i>_ROLL_{T,N}S` on the **−y side** face, axle local **z**, at (0, −0.022,
0) — the engine's Cylinder is Z-aligned, so the side roller needs no
rotation and the bottom one is turned onto y with `Pose { rotation 1 0 0
1.5708 }`. A cell rolled +90° about x (dock rotation 1: local −y → world −z)
rides on its side pair, whose axle then runs along world y: it rolls along
the spine and resists the lateral push exactly as the bottom pair does for
a pitch cell. The unused pair points sideways / up and never touches. Dock
rotations are restricted to {0, 1} (roll 2 or 3 puts every roller in the
air). 4 mm protrusion on both pairs; rest z 0.0339, spawn 0.036.

**Bearing damping deviation.** `HingeJointParameters.dampingConstant 0.002`
is written as asked, but it is **UNIMPLEMENTED under Newton**
(`OmHingeJoint.cpp:210`: it was an ODE AMotor companion joint and nothing
replaced it), so it would have left the 5.86 m coast untouched. The bearing
is therefore a limitless `RotationalMotor { name "roller_<tb|ts|nb|ns>_brake"
maxTorque 0.001 maxVelocity 200 }` that is never commanded: the engine
builds it as a ke = 0 velocity wheel (kd 500) with target velocity 0, i.e.
−kd·ω clamped to 0.001 N·m — a Coulomb brake of 0.001 N·m per axle (0.083 N
at the 12 mm radius). Measured: the witness coasts **0.037 m** (v2: 5.86 m).
Cost of the deviation: 48 benign one-line `VELOCITY wheel: setPosition()
will be IGNORED` notices per 12-cell load (one per brake motor, at load, not
per tick) — the `warn/err 48` in the P1c runs is exactly these — and the
pitch inchworm loses ~20 % (0.075 → 0.061 m/s).

### P1c sweep (cell v3, alternating chain, steer ±0.5 in both halves; ≤ 4 runs)

| tag | A | ω | Δφ | pitch chain net m/s | alt. chain net m/s (phase +, −) | turn + / − [rad / 8.8 s] | curvature + / − [rad/m] | ms/step |
|---|---|---|---|---|---|---|---|---|
| c1 | 0.6 | 3 | 1.57 | 0.019 | 0.012 (0.030, 0.016) | −0.95 / +0.65 | −3.6 / +4.8 | 8.86 |
| c2 | 0.9 | 3 | 1.57 | 0.034 | 0.008 (0.018, 0.003) | −1.19 / +0.92 | −7.5 / (pivot) | 9.03 |
| **c3 = c** | **0.9** | **4.5** | **1.2** | 0.061 | **0.030 (0.032, 0.030)** | **−1.59 / +2.05** | **−5.7 / +7.7** | 8.85 |
| c4 | 0.6 | 4.5 | 1.2 | 0.045 | 0.016 (0.031, 0.005) | −1.42 / +0.95 | −5.3 / (pivot) | 8.70 |

Reading it: the side rollers did what the diagnosis said they would — the
yaw cells now sit on an axle that runs along the spine, the lateral wave has
something to push against, and the alternating chain **translates**
(0.030 m/s at c3, 2.5× v2, under a ±0.5 steering bias the whole time) with a
steering sign that is **the same at all four grid points**: +0.5 turns right
(negative heading), −0.5 turns left. Magnitudes are not symmetric (−5.7 vs
+7.7 rad/m at c3) and the − side stalls at the two low-amplitude/low-
frequency points (c2, c4 paths < 0.05 m), so the channel is monotone in sign
and roughly ±6–8 rad/m in size at the best gait, with a turn radius of
~0.15 m — a chain that can turn on a plate. Lateral undulation is still
half the speed of the pitch inchworm at the same gait.

### P1b sweep (alternating chain, steer ±0.5 in both halves; ≤ 4 runs)

| tag | A | ω | Δφ | pitch chain net m/s | alt. chain net m/s (phase +, −) | turn + / − [rad / 8.8 s] | ms/step |
|---|---|---|---|---|---|---|---|
| s1 | 0.6 | 3 | 1.57 | 0.015 | 0.008 (0.008, 0.021) | −1.08 / −1.17 | 7.05 |
| s2 | 0.9 | 3 | 1.57 | 0.039 | 0.003 (0.005, 0.010) | −1.49 / +1.10 | 6.80 |
| **s3 = b** | **0.9** | **4.5** | **1.2** | **0.075** | 0.012 (0.004, 0.026) | **−2.00 / +2.03** | 6.42 |
| s4 | 0.6 | 4.5 | 1.2 | 0.036 | 0.018 (0.018, 0.036) | −1.59 / −1.70 | 6.49 |

(s1–s4 carry the cell-11 layout artefact in their negative-control row
only; chains A/B and the witness were never touched by it — s3's chain
numbers reproduce bit-for-bit in run `b`.)

Reading it: (a) the **pitch inchworm doubled with rollers and a stiff weld**
(0.044 → 0.075 m/s) and prefers a high ω, short Δφ; (b) **lateral undulation
stays near zero** on this floor — the rollers give the cells an anisotropy
along their *own* x, but in a [0,1,0,1] chain the pitch cells' rollers ride
along the spine and the yaw cells' rollers point sideways and never touch,
so every ground contact of the lateral wave is a pitch cell's roller that
rolls freely in the direction the undulation pushes — there is no lateral
resistance for the wave to push against, which is the opposite of what a
snake's scales do; (c) the yaw bias turns the chain **in place** at ~0.23 rad/s
per 0.5 rad of bias, with the sign consistent for +0.5 (always right, i.e.
negative) but not for −0.5 (2 of 4 grid points turn right too). The
steering channel exists but is not yet monotone; a chain that does not
translate cannot show curvature.

### Finding 1 (P1): the Connector weld engages, but at the default `eq_solref` it is a mass-scaled spring

Three arms of the P1 (v1) gait, same world:

| arm | max junction sep (3 junctions) | end sep after 0.5 s rest | chain A displacement | upright |
|---|---|---|---|---|
| no `isLocked` write (control, `--tag nolock`) | 0.079 / 0.067 / 0.079 | 0.027 / 0.010 / **0.076** (cells drift apart) | **0.021 m** | yes |
| locked, maxTorque 0.6 (canonical) | 0.092 / 0.070 / 0.136 | 0.028 / 0.032 / 0.070 (still loaded, chain on its side) | **0.888 m** | no |
| locked, maxTorque 0.2 (`--tag tq02`) | **0.032 / 0.020 / 0.036** | **0.0101 / 0.0095 / 0.0095** (back to the authored 0.010) | 0.612 m | yes |
| locked, maxTorque 0.6, `newtonSubsteps 4` (`--tag ss4`) | 0.057 / 0.021 / 0.059 | 0.0099 / 0.0098 / 0.0098 | 0.932 m | yes |

The weld was real (the locked chain travels 0.89 m as one body where the
unlocked control travels 0.02 m, and after the load stops the junctions
return to the authored 0.010 m gap) but it stretched 3–13 cm under load,
scaling with the motor torque (0.6 → 0.2 N·m gave 3× less), i.e. elastic.
Cause: `weld_engage` (`omnisim_newton_runtime.py`) left MuJoCo's default
`eq_solref = [0.02 s, 1]`, and MuJoCo constraint stiffness is per unit of
effective mass (≈ m / timeconst²): ≈ 300 N/m for a **0.12 kg** block against
up to 20 N from a 0.6 N·m hinge on a 0.03 m lever. The RoboLife tow (46 kg
Husky, 1.5 mm stretch) saw the same constraint at 400× the mass. **Fixed in
the runtime on 2026-08-28** (`eq_solref = [2·mj_timestep, 1]`, env
`OMNISIM_NEWTON_WELD_TIMECONST=<s>` overrides): with `newtonSubsteps 4` the
time constant is 4 ms and P1b's junctions read **0.0102–0.0118 m maximum,
0.0100 / 0.0107 m at rest** under the same class of load — the gate passes.

Also measured: gait sweep `--tag sw1` (A 1.0, ω 8, Δφ 2.0; A·ω = 8 rad/s
exceeds maxVelocity 5) blew both chains apart at the soft weld — junction
separations of **38 m**, welds torn (end sep 2 m), every cell inverted. Keep
A·ω under the motor's velocity limit.

### Finding 2: a pitch-only 4-chain inchworms at 0.04 (v1) – 0.075 (v2) – 0.061 (v3) m/s, not 0.15

The direction is consistent (−x of the wave). The mechanism is
friction-asymmetric inchworming, and 0.15 m/s (0.19 m per cycle, 40 % of the
0.5 m body) is not reachable by it at these amplitudes. Lateral undulation
is stationary on both cells (see the sweep reading). So P2's CPG has
~0.075 m/s of pitch-chain propulsion, an in-place yaw channel of ~0.23 rad/s
per 0.5 bias, and no demonstrated lateral gait. Untried levers: a lifted
head (the alife lesson: bias that lifts the body off its belly), higher
`newtonGroundMu`, and rollers whose axis is along the *spine* for yaw cells.

### Finding 3 (physics result): a single symmetric cell cannot somersault

Cell 10 ran the DESIGN `flip_sequence` (fold to 2.4 rad in 0.5 s, hold 0.2 s,
unfold over 1.3 s, sign chosen so the nose folds toward the floor) for 10
cycles: net progress **−0.003 m**. The trace explains it: the tail block's
up-vector minimum is **0.3596 = cos 68.9°**, exactly half of the 2.4 rad fold
(2.4 / 2 = 1.2 rad = 68.75°, cos = 0.362), and the tail centre rises to
0.042 m. Both blocks tilt equally and the cell forms a symmetric **tent**
about the seam; the combined centre of mass never leaves the planted
block's footprint, so nothing tips. The M-TRAN somersault needs a 2:1
half-module (the folded half reaches past the planted half's edge) or a
ground anchor; two equal 6 cm cubes have neither, and by symmetry no hinge
trajectory whatsoever can net-translate them (the two blocks are mirror
images through the seam plane at every fold). **Accepted as a design
decision: lone cells are inert; organisms approach recruits.** (A v2 cell
does coast when kicked, but cannot kick itself.)

### Cost — resting PASS at every version; a moving v3 reef is over the tick

v1: 24 resting cells **1.06 ms/step** (48 bodies). v2: **1.98 ms** (96 bodies).
v3: **3.40 ms** median, p90 4.08 (144 bodies, 120 motorized incl. 96 brakes),
every cell resting at z 0.0339. Actuated 12-cell probe worlds: v1 2.6–3.0 ms
(substeps 1), v2 6.3 ms, **v3 8.7–9.0 ms** (72 bodies, substeps 4) — already
over the 8 ms realtime tick at half the reef's population, so a 24-cell v3
reef with everything welded and moving will run at roughly half realtime
at substeps 4. The dials, in order: `newtonSubsteps 2` (weld time constant
8 ms; unmeasured), fewer cells interactive (PLAN's 16-cell fallback), and
headless for the epochs.

### What P1/P1b/P1c settle for B, C, D

- The cell, its DEFs and the docking placement are as `mz/cell.py` states;
  `chain_placement` reads `dock_rotations[k]` as the roll of cell k
  **relative to the head** (so [0,1,0,1] = pitch, yaw, pitch, yaw — the P1
  wording), not cumulative. B's `axis_of` should be `pattern[k] % 2`.
- Docking by `isLocked` field write works, is unilateral, and refuses a face
  0.10 m away. Unlock is the same field FALSE. With the runtime fix and
  `newtonSubsteps 4` a weld holds to ~1 mm under gait loads.
- Hinge sign: positive = nose down / right; lifts are negative.
- Cell v3 (`rollers="v3"`) is the reef cell: maxTorque 0.35, rest z 0.0339,
  spawn z 0.036, 6 dynamic bodies per cell, dock rotations {0, 1} only, a
  0.001 N·m Coulomb bearing brake (a kicked free cell coasts ~4 cm).
- Propulsion available today: pitch chains at 0.061 m/s; alternating chains
  at 0.030 m/s with a sign-consistent steering channel of −5.7 / +7.7 rad/m
  per ±0.5 yaw bias. Lone cells are inert.
- Cost is not the risk at 24 resting cells; a 24-cell v3 reef in motion is
  ~2× over the realtime tick at substeps 4 (see Cost).

### Recommendation for the organism controller (A → B)

Default to the **pitch-only inchworm on a rotations-[0,0,…] chain at A 0.9 /
ω 4.5 / Δφ 1.2**, which is the fastest thing measured on the reef cell:
**0.061 m/s** (v3, 1.17 m in 20 s, junctions at 0.0106 m, all upright), and
switch to the **alternating [0,1,0,1] chain only when the organism must
steer**, at the same gait with the yaw-hinge bias as the channel: it moves at
**0.030 m/s** and turns with a consistent sign at ≈ −5.7 rad/m for +0.5 and
+7.7 rad/m for −0.5 (turn radius ~0.15 m) — the pitch chain has no steering
channel at all, so a body plan that needs to reach a light patch must carry
at least one yaw cell. Keep A·ω < 5 (the motor's velocity limit; the soft-
weld blow-up at A·ω = 8 is the cautionary datum), fade the amplitude in over
1 s, and never expect a lone cell to move: recruitment is the organism
driving its nose or an open side face to the recruit. All three speeds are
below the 0.15 m/s gate; that gate was written for a cell the physics does
not offer, and 0.06 m/s on an 18 m reef (5 min to cross) is what the ecology
should be sized to.

## Files

- `mz/cell.py` — geometry constants, `cell_vrml(..., rollers=False|True|"v3")`,
  `roller_lines` / `roller_set` / `bodies_per_cell`, `chain_placement`,
  `branch_placement`, `face_world`, `spawn_z`, DEF names.
- `mz/worldgen.py` — `write_world(cells, path, scene_lines=None, rollers=,
  substeps=4, ...)`, `_fallback_scene`, crypt parking.
- `probe_p1.py` — the P1/P1b driver (thermal gate, one engine at a time,
  log evidence, JSON merge). `--v2`, `--v3`, `--no-flip`, `--tag`,
  `--A/--omega/--dphi`, `--max-torque`, `--substeps`, `--no-lock`,
  `--only probe|cost`, `--env KEY=VAL`.
- `controllers/metazoa_probe_p1/` — the gate instrument.
- `_run/probe_p1.json` (P1 canonical), `_run/probe_p1_b.json` (P1b
  canonical), `_run/probe_p1_c.json` (P1c canonical = sweep run c3 + the
  `--tag c` cost lane), `_run/probe_p1_<tag>.json` for the A/B and sweep
  arms above; engine logs + sidecars alongside. `worlds/probe_p1*.omniworld`
  are the generated worlds (`_b` = v2, `_c3` / `_cost_c` = v3).


## P2 — wave direction and steering (probe_wave.py, one welded 4-chain, 15 s phases)

Index 0 = tail, 3 = head; cells placed along +x from index 0. `along_spine` is
the chain centroid's displacement projected on its own tail->head axis.

| chain pattern | phase | along spine | lateral | yaw change |
|---|---|---|---|---|
| `[0,0,0,0]` all pitch | +dphi | −0.868 m | +0.012 | +0.01 rad |
| | −dphi | +0.814 m | 0.000 | 0.00 |
| | +dphi steer ±1 | −0.95 / −0.90 | 0.00 | 0.00 |
| `[0,1,0,1]` alternating | +dphi | −0.097 | +0.055 | **+2.31** |
| | −dphi | +0.107 | +0.071 | −1.16 |
| **`[0,0,0,1]` head rudder** (yaw cell wave-free) | −dphi steer 0 | **+1.413** | −0.040 | −0.08 |
| | −dphi steer +1 | +1.020 | **+0.632** | **+0.97** |
| | −dphi steer −1 | +1.011 | **−0.610** | **−0.92** |
| `[1,0,0,0]` tail rudder | −dphi steer ±1 | +0.81 / +0.91 | ±0.05 | ±0.17 |

Three facts that fix the organism design: (1) the travelling wave propels a
chain toward its **low-index** end (run it with −dphi to travel head-first);
(2) the alternating pitch/yaw chain the contract assumed is a bad walker —
0.007 m/s and it spins 2.3 rad on its own; (3) a pitch inchworm with one
**wave-free yaw cell at the head** is a real steerable walker: 0.094 m/s
straight, ±0.95 rad per 15 s with steer ±1 while still moving at 0.068 m/s
(turn radius ≈ 1 m). A rudder at the tail is 5× weaker. Consequences: the
organism travels rudder-first, and grows by backing into free cells at its
tail (a straight reverse needs no steering).

Reef runs before this measurement (12 cells, 300 s): 6/6 seeded welds held
for the whole epoch, chains crawled at 0.07 m/s, but every recruiter backed
AWAY from its target because the wave was run tail-first and the head's 1 s
travel heading was wobble-dominated — the spine axis is now the heading.


## P2 — tail docking on the engine (probe_dock.py, 2026-08-29)

One 4-cell organism (head rudder, pitch spine), one free cell 0.9 m behind
its tail with its nose toward the tail, charge 70 % so the ecology recruits
at once. Trace (`_run/metazoa/world.log`): runway → ALIGNED (tail face
lateral 0.000, spine error 0.00 rad) → REVERSE (+dphi, no steer) at
0.04–0.07 m/s → capture assist at 0.220 m → **lock written at 9.7 mm /
0.000 rad** → verified → `recruit(at_tail=True)`; 4 welds held, the organism
crawled on as 5 cells at up to 0.14 m/s. First recruitment under the final
organism design, end to end, on the engine.

Two implementation facts: the engine discards a controller's stderr, so the
supervisor now writes uncaught tracebacks to `world.log` (the rework had
dropped a helper and read as a bare "exited with status 1"); seeded welds are
written at tick 25, not 0 (bodies are not registered at tick 0).


## P2 — off-axis docking (probe_dock.py --lateral 1.0, 2026-08-29)

Free cell 1.2 m behind and 1.0 m beside the organism's tail. Four measured
failures shaped the manoeuvre: (1) alignment at 0.5 rad let the body reverse
2.5 m and drift 0.35 m off-axis (×4 loops) → align at 0.12 rad / 0.10 m and
only near the bay; (2) a near/far runway pair 0.65 m apart could never
straighten a body with a ~1 m turn radius; (3) line-following INTO the bay
converges perfectly — and arrives head-first, the wrong way to back in
(spine_err 2.5 rad, ×4); (4) at full speed the body orbits a turn-in point
inside its turning circle forever (26 trace rows). Final: drive to a turn-in
point just outside the bay, follow the nose normal OUTWARD to straighten with
the tail toward the cell, reverse in; the wave amplitude scales down with
|steer| (turn radius = speed / yaw rate); rudder polarity pinned to the
measured convention (the wiggle read b = 0.0008 on a rudder body). Result:
ALIGNED at spine_err 0.12 → capture at 0.220 m → **lock 9.1 mm / 0.001 rad**
→ verified → recruited, 4 → 5 cells, in ~200 s of sim.


## P2 — six-cell bodies, one rudder or two (probe_wave.py, N_CELLS=6, 2026-08-29)

| pattern | straight (15 s) | steer +1 / −1 |
|---|---|---|
| `[1,0,0,0,0,1]` two rudders | 0.40 m (0.027 m/s) | **0.02 / 0.03 m, no turn** — the tail rudder anchors the body |
| `[0,0,0,0,0,1]` head rudder | **1.63 m (0.109 m/s)** | 1.25 / 1.14 m with yaw +0.62 / −0.43 rad |

A second rudder at the tail costs 4× the speed even when held flat and
brakes every turn, so bodies carry ONE rudder at the head. A division then
leaves the rear half rudderless; its new head performs a **dock-face
rotation** (90° about the spine, in place, junction released for one tick
and re-engaged) to become the rudder — the connector rotation real
reconfigurable modules have (Roombots, M-TRAN). Turn slowdown was cut from
0.6 to 0.25: a rudder only turns a moving body, and at 0.6 a full-lock turn
starved itself (6-cell reef bodies sat at 0.005 m/s with zero yaw rate).


## P3 — recruits inside full reef epochs (metazoa.py, 14 cells / 2 organisms / seed 4, 2026-08-29)

| epoch | sim s | recruits (attempts) | what stopped it |
|---|---|---|---|
| 1 | 420 | **2** (5, 1 failed verify at 0.0806 m) | both bodies ended pinned on the arena wall: one at 0.003 m/s in `line_out` for 250 s, the other driving AWAY from its target for 100 s with the rudder swinging ±1 every few seconds (target dead astern: the error wraps at ±π) |
| 2 | 480 | **3** (7, 0 failed) | turn commitment + wall aim: the pinned body still could not turn — nose in the wall at 4.71 m, a 0.06 m/s head wobble, no way on |
| 3 | 480 | **3** (7, 0 failed) | back-off added: it limit-cycled at 0.87 m off the wall (reverse until clear of the 0.9 m margin, forward straight back in) — now a latched rev → turn → clear state machine |

Every lock so far was written at **9.3–9.8 mm / 0.000 rad** and every recruit
holds (welds 8–9 of 8–9 at epoch end); cells are conserved in every epoch
(free + members = 14). Recruits cost ~150 s each, which is why a 4-cell seed
never reaches the 8-cell division threshold inside one epoch — the demo seeds
6-cell bodies. Engine cost with the supervisor: 8.1–9.0 ms/step median at
14 cells, dt 8 ms (the 3.4 ms figure above is the bare engine).

### Rudder angle sweep (probe_wave, 6 cells, `[0,0,0,0,0,1]`, 15 s phases)

| steer_gain (rad at full lock) | straight | steer +1 | steer −1 |
|---|---|---|---|
| 0.3 | 1.63 m | 1.43 m, +0.46 rad | 0.96 m, −0.21 rad |
| **0.5** | 1.63 m | **1.25 m, +0.62 rad** | **1.14 m, −0.43 rad** |
| 0.7 | 1.63 m | 0.80 m, +0.25 rad | 1.14 m, −0.49 rad |
| 1.0 | 1.63 m | 0.73 m, +0.25 rad | 0.79 m, −0.15 rad |

The reef genomes were seeded at 0.9–1.1 (an unmeasured guess) — that is why
epoch 4's bodies (16 cells, 6-cell seeds) sat at the wall at 0.005–0.05 m/s
with the rudder at full lock and recruited nothing. Seeds are now 0.45–0.55,
the mutation range 0–0.8, and the supervisor caps the rudder at 0.6 rad.
Turn radius at 0.5: 0.083 m/s ÷ 0.041 rad/s ≈ **2 m** — a body needs ~4 m of
lateral room to turn around, which sizes the arena and the wall back-off.

Amplitude does not tighten the turn (gain 0.5): A 0.6 → 0.13 rad/15 s over
0.60 m (radius 4.6 m), A 0.45 → no yaw at all. So `TURN_SLOWDOWN` is 0 — a
turn runs the full wave — and the wall back-off is 2.6 m, sized to the 2 m
radius. (The earlier "slowdown 0.6 starved the rudder" finding was measured
under the 1.0 rad over-steer and is superseded.)

### Rudder layout sweep (probe_wave, 6 cells, gain 0.5, 15 s phases)

| pattern | straight | steer +1 | steer −1 | turn radius |
|---|---|---|---|---|
| `[0,0,0,0,0,1]` single head rudder | 1.63 m | 1.25 m, +0.62 rad | 1.14 m, −0.43 rad | ~2 m |
| `[0,0,0,1,0,1]` spaced | 1.47 m | 1.28 m, +1.33 rad | 0.80 m, −1.22 rad | ~0.9 m |
| **`[0,0,0,0,1,1]` head PAIR** | 1.22 m | **0.61 m, +1.81 rad** | **0.64 m, −2.02 rad** | **~0.3 m** |

Reef bodies now carry a rudder pair (three times the yaw rate of one rudder
for a 25 % straight-line cost), the supervisor commands the steer on the two
head yaw cells, and a division re-rolls both the rear half's new head and the
cell behind it. Reef epochs 4–7 (6-cell seeds, single rudder) recruited
nothing in 540–600 s: with a 2 m turn radius every approach ended at a wall
(15 wall events in epoch 7). Both reef genomes replayed on the probe walk at
0.08–0.12 m/s, so the failure was the turn, not the gait.
