# Alife v2 — living ecosystem design contract

This file is the contract three parallel implementers build against. Where the
code and this file disagree, fix one of them in the same change.

## Goal

A flagship OmniSim simulation: organic-looking bilateral creatures that sense
food, steer toward it, eat, metabolise, reproduce (offspring inherit a mutated
brain) and die, in one continuously running world; between **epochs** the world
is regenerated with the surviving species' bodies mutated, so morphology
evolves too. Must run in **real time** (physics < 8 ms/step at dt 8) and look
good on the wgpu renderer.

## Hard engine facts (measured this project — do not re-derive)

- Runtime spawn/delete have NO physics. Every body is authored at load;
  birth/death are teleports of POOLED slots.
- One supervisor "director" drives all creatures via batched
  `HingeJointParameters.position` field writes (one round trip per tick).
  Never `setJointPosition` (blocking, ~1400× costlier).
- Creatures are `controller "<none>"` Robots (`""`/`"void"` spawn processes).
- Every creature root wrapped in `Pose { translation 0 0 0 children [Robot] }`
  (a top-level teleport perturbs every other creature's joints ~51%).
- Robot root: own `boundingObject`, `physics` with explicit `inertiaMatrix` +
  `centerOfMass`; `WorldInfo.newtonRobotColliders TRUE`.
- Every `RotationalMotor`: `minPosition/maxPosition` (else it is a velocity
  wheel and targets are ignored); keep |bias|+|amp| ≤ 1.55 rad inside ±1.8.
- Floor is a `Box`. No implicit ground plane: a body over nothing falls forever
  (zero contacts → nearly free — we USE this for parking).
- `Capsule` inside a `Pose` in a boundingObject is supported (offset +
  orientation carried, `OmSolid.cpp:2907`). Webots Capsule axis is **Y**.
  Never `Cone`. Never a closed kinematic loop.
- Contact-island density is the cost cliff, not body count. Resting bodies
  cost MORE than moving ones. Food must have NO physics (visual-only).
- MuJoCo's instability warning channel is read by nothing → director watchdog.
- Fitness/behaviour is deterministic and position-invariant, but depends on
  the physics config (dt/contacts) — keep dt 8, ke 8000/kd 200, elliptic,
  impratio 10, newtonGroundMu 1.5 everywhere.

## Files (owner in brackets)

- `alife/genome2.py` [A] — v2 genome: random, mutate_body, mutate_brain,
  validate, describe.
- `alife/worldgen2.py` [A] — genome list + scene spec → `.omniworld`.
- `alife/ecology.py` [B] — PURE python (no omnisim import): energy model,
  sensing, steering modulation, reproduction/death bookkeeping. Unit-tested.
- `controllers/terrarium_life/terrarium_life.py` [B] — the director:
  actuation + ecology + telemetry + HTTP bridge.
- `ecosystem.py` [C] — epoch driver; `watch_life.py` [C] — lean windowed
  launcher; `alife/scene.py` [C] — arena/walls/food-pool/lighting/viewpoint
  VRML helpers consumed by worldgen2.

## Genome v2 (JSON)

```json
{
  "id": "sp0_g0_00", "species": "sp0", "parent": null,
  "body": {
    "torso": {"length": 0.30, "radius": 0.06},
    "head":  {"radius": 0.055},
    "pairs": [
      {"x": 0.7, "z": 0.0,
       "segments": [{"length": 0.16, "radius": 0.022},
                    {"length": 0.14, "radius": 0.018}],
       "splay": 0.35}
    ],
    "hue": 0.62
  },
  "brain": {
    "freq": 1.6,
    "pairs": [
      {"hip":  {"amp": 0.6, "bias": 0.1, "phase": 0.0},
       "knee": {"amp": 0.4, "bias": -0.5, "phase": 1.2}}
    ],
    "mirror_phase": 3.14159,
    "steer_gain": 0.5,
    "heading_offset": 0.0,
    "sense_radius": 4.0,
    "wander": 0.4
  }
}
```

Ranges: torso length 0.18–0.42, radius 0.04–0.09; head radius 0.6–1.0 ×
torso radius; pairs 1–3 (bilateral: each pair = left + right limb); segments
1–2; segment length 0.08–0.24, radius 0.015–0.035; splay 0–0.7 rad;
freq 0.5–3.0; amp 0.1–1.0; |bias| ≤ 0.9; steer_gain 0–1; sense_radius 1.5–7;
wander 0–1. `mutate_body` (between epochs) and `mutate_brain` (at birth) are
separate; both are gaussian creep + rare structural change; `validate`
returns a list of problems (empty = ok) and MUST reject anything the engine
refuses.

## Body construction (worldgen2)

- Torso: Robot root; capsule along world **+x** at rest (Pose rotation
  `0 0 1 1.5708` around a Y-axis Capsule). Explicit inertia: cylinder approx
  `Ixx = 0.5 m R²`, `Iyy = Izz = m(3R² + L²)/12`, density 250 kg/m³.
- Head: VISUAL ONLY (Shape, no Solid) sphere at the +x nose, two small dark
  "eye" spheres. It is part of the torso body.
- Limb pair k at torso-frame `(x_k · L/2, ±R, z_k)`. Hip = `HingeJoint`,
  axis **`0 1 0`** (pitch: fore/aft swing = thrust) for both sides. Segment 1
  hangs down-and-outward: the endPoint Solid is rotated about **x** by
  `±splay` (outward), capsule along its local −z (Pose rotation
  `1 0 0 1.5708` on the Y-capsule), centre at `−length/2` along local z.
  Segment 2 (knee) = HingeJoint axis `0 1 0` at the end of segment 1, same
  construction. DEF names: `CREATURE_{i}`, joints `C{i}_P{k}_{L|R}_{H|K}`,
  parameters `..._PARAMS`, segment Solids `..._LINK`.
- Motors: `maxTorque 4 maxVelocity 8 minPosition -1.8 maxPosition 1.8`,
  `HingeJointParameters { minStop -2 maxStop 2 position 0 }`. Every joint
  has a PositionSensor (name = joint DEF lowercased).
- Colours: species hue → saturated HSV body, darker limbs, cream belly not
  required. Distinct per species.
- Spawn z = `legdrop + R + 0.05` where legdrop = Σ segment lengths · cos(splay).

## Scene (scene.py)

- Arena: `Box` floor `S×S×0.1` at z −0.05, dark ground colour; 4 static wall
  boxes 0.4 high on the perimeter (creatures must not fall off); S from the
  driver (default 14).
- Canonical lighting: `OmniSimSky {}`, `DEF SUN OmniSimSun {}`,
  `DEF SUN_MARKER OmniSimSunMarker {}`. Viewpoint: hero view of the arena.
- Food pool: `FOOD_{j}` = Solid with a small bright Sphere (r 0.09, fruit
  colour, mild emissive ≤ 0.3), **no boundingObject, no physics**. Parked
  items sit at z = −3 (under the floor, invisible). Pool size from driver.
- Parking pit for dead creatures: `(60 + 2i, 60, 5)` — nothing beneath, they
  free-fall (zero contacts). Revive = set translation + rotation +
  `resetPhysics()`; the director's watchdog must exempt parked slots.
- Director: `DEF DIRECTOR Robot { supervisor TRUE synchronization TRUE
  controller "terrarium_life" }`.
- WorldInfo exactly as listed in the hard facts; `basicTimeStep 8`;
  `newtonNjmax 2048 newtonNconmax 2048`.

## Director (terrarium_life.py)

Reads `_run/life/population.json` (list of genome v2 + `"slot"` index +
`"alive_at_start": bool`) and `_run/life/config.json`
(`{arena, food_pool, food_active_max, food_respawn_s:[lo,hi], epoch_s,
watch: bool}`). `DT = int(r.getBasicTimeStep())`.

Per tick:
1. **Sense**: for each alive creature, torso pos + yaw (from
   `getOrientation()`), nearest ACTIVE food within `sense_radius` → bearing.
2. **Steer** (ecology.steer): `err = wrap(bearing − (yaw + heading_offset))`,
   `turn = clamp(err / (π/2), −1, 1)`; no food → `turn = wander · slow
   noise`. Left amps × `(1 − steer_gain·turn)`, right × `(1 + steer_gain·turn)`,
   clamped ≥ 0.
3. **Actuate**: `target = bias + amp·sin(2πf·t + phase [+ mirror_phase on
   right])` for every joint → batched field writes.
4. **Metabolism** (ecology.energy_step): cost/s = `0.012·mass + 0.02·Σ(amp·f)`;
   `E ≤ 0` → die (park slot, count death).
5. **Eat**: dist(torso, food) < `torso.length/2 + 0.25` → `E += 45` (cap
   200), food parked, respawn timer in `[lo,hi]`.
6. **Reproduce**: `E > 140` and a free slot of the same species exists →
   `E −= 55`, child E = 55, brain = `mutate_brain(parent)`, revived 0.8 m
   beside parent with random yaw. Count birth, lineage.
7. **Food respawn**: keep `≤ food_active_max` active; timers.
8. **Watchdog**: non-finite or |pos| > 1e4 on an ALIVE creature → kill + count.
9. **Telemetry**: every 250 ticks write `_run/life/telemetry.json`
   (`tick, sim_s, population per species, food_active, births, deaths,
   eats, per-slot {species, alive, energy, age_s, eaten, offspring, pos,
   brain}`). At `epoch_s`: write `_run/life/epoch_result.json` (per species:
   births, deaths, eaten, peak_pop, mean_lifespan_s, best_brain = alive-or-
   dead creature with most offspring then eaten) and `simulationQuit(0)`
   unless `watch` (then run forever, resetting nothing).

HTTP bridge (port `LIFE_PORT`, default 8790), same threading contract as
`terrarium_bridge.py` (HTTP thread never touches the supervisor; commands
marshalled to the sim thread; 409 while busy; reads from a snapshot):
`GET /capabilities`, `GET /census` (= telemetry snapshot), `POST /feed
{x,y}` (place a parked food item), `POST /cull {creature}`, `POST /spawn
{species}` (revive a free slot with the species' best brain),
`POST /perturb {creature, steer_gain|freq}`. Results carry MEASURED state.

## Epoch driver (ecosystem.py)

`python projects/alife/ecosystem.py --epochs 6 --species 4 --slots 4
--alive 2 --epoch-s 120 --arena 14 --food-pool 24 --food-active 12`

Per epoch: build population (K species × M slots; `alive` per species
alive_at_start, brains = species best brain from the previous epoch,
mutated), write `population.json` + `config.json`, `worldgen2.write_world`,
run ONE `run-headless --duration epoch_s+60` (`OMNISIM_LOG_PATH` unique),
read `epoch_result.json`, score species = `births·10 + eaten +
mean_lifespan_s/10`, keep the top ⌈K/2⌉, refill with `mutate_body` of the
keepers (new species ids). Persist `_run/life/epoch_NN/` (population,
world, result). Print a per-epoch table. `--resume`.

`watch_life.py`: regenerate the LAST epoch's world with `watch: true` and
launch windowed with the lean render profile from `watch.py`
(`OMNISIM_WGPU_SSR=0 TAA=0 VOLUMETRIC=0 PCSS=0`). `--dry-run` supported.

## Verification gates

- A: probe world of 8 creatures (2 pairs × 2 segments) PASSes
  `run-headless --duration 30` with 0 errors; every hinge logs
  `[motorized: kd=2]`; torso rests at the geometric height; report engine
  ms/step (the budget is 8 ms — report, do not tune yet). Also verify
  revive-from-free-fall: park one creature, 10 s later revive it, it rests
  at the right height with |v| ≈ 0.
- B: `python -m pytest projects/alife/tests/test_ecology.py` green; steering
  math has a test where a creature facing +x with food at +y gets turn > 0.
- C: `ecosystem.py --dry-run` prints the plan; `watch_life.py --dry-run`
  prints env + command. No engine runs.
