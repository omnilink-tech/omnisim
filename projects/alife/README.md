# Artificial-life terrarium — feasibility probes

Karl Sims-style evolved virtual creatures on OmniSim: creatures with real jointed
bodies that collide, push against terrain, and are selected on measured
locomotion. This directory currently holds the **feasibility probes**, not the
terrarium. Everything below is measured on machine `9722d23d12a3`
(RTX 3060 laptop, Windows), Newton 1.5.0 / MuJoCo 3.11.0, CPU `mj_step`.

Reproduce: `python projects/alife/gen_terrarium.py && python -m omnisim
run-headless projects/alife/worlds/terrarium_probe_0.omniworld --duration 90`

## Verdict: the architecture is viable

| # | question | verdict | evidence |
|---|---|---|---|
| P1 | Can a supervisor actuate `controller "<none>"` creatures via batched field writes? | **YES** | 0.29 m displacement; 48/48 hinges register `[motorized: kd=2]` (position-servo branch, not the ke=0 velocity wheel) |
| P2 | Does a top-level teleport zero every joint velocity in the world? | **NO — but it perturbs** | ratio 0.487 vs control 0.986 (never 0.0, so no `resetJointsToDefaults`) |
| P3 | Does `Pose{}` wrapping suppress that perturbation? | **YES** | ratio 1.024 vs control 0.993 — indistinguishable from no event |
| P4 | What does it cost? | **21.3 ms/step @ 12 creatures** | see the contact tradeoff below |
| P5 | Is `setJointPosition` usable per-tick? | **NO** | 9.96 ms/call under load vs 0.223 ms for 32 batched field writes |
| P6 | Can this morphology locomote at all? | **YES** | best gait 0.875 m in 4 s (0.22 m/s); spread 0.875 m across 8 gaits |

**P1 is the load-bearing one.** The engine pushes motor targets to Newton every
step from a *global*, controller-independent registry
(`OmSimulationWorld.cpp:276` → `OmBasicJoint.cpp:828`), so whoever last wrote
`OmMotor::mTargetPosition` wins regardless of which robot owns the joint or
whether it has a controller at all. It is genuine solver torque through a MuJoCo
`POSITION_VELOCITY` actuator at `ke = maxTorque*10`. One process drives the
whole ecosystem, and creatures really do push against the ground.

## Hard-won specifics (do not relearn these)

- **`controller "<none>"` is the only free option.** `""` and `"void"` both spawn
  a `<generic>` controller process. 12 creatures would be 12 interpreters.
- **`setJointPosition` cost scales with engine step time** — it is a blocking
  IPC flush that waits on the step. Probe 0 measured 0.307 ms/call at ~12 ms/step
  and 9.96 ms/call at ~27 ms/step. Batched `HingeJointParameters.position` field
  writes are postponed and drained as one batch immediately before the motor
  push, costing 0.223 ms for 32 joints — **~1400× cheaper**. Never actuate with
  `setJointPosition`.
- **Wrap every creature root in `Pose { translation 0 0 0 children [ Robot ] }`.**
  Not cosmetic: a top-level Solid teleport (i.e. every birth) knocks ~51% off
  other creatures' joint velocities in one tick; a Pose-wrapped teleport does
  nothing measurable. The gate is `upperPose() == nullptr` (`OmSolid.cpp:1192`).
- **Motors need `minPosition`/`maxPosition` wider than the gait's `bias + amp`.**
  Without limits at all the joint is built as a ke=0 velocity wheel and position
  targets are silently ignored. With limits too narrow you get one WARNING per
  joint per tick — a run produced 7977 warnings and the log I/O alone slowed the
  sim enough that it never reached its own test points.
- **`WorldInfo.newtonRobotColliders TRUE` is required**, or the torso's
  `boundingObject` is discarded for a 1 mm placeholder sphere.
- **The Robot root needs explicit `inertiaMatrix` + `centerOfMass`.** The
  geometry-derived inertia path excludes `OmRobot` (`OmSolid.cpp:3842`) and falls
  back to a Husky-tuned preset. `inertiaMatrix` without `centerOfMass` is a parse
  WARNING and drops the node to mode INVALID.
- **The floor must be a `Box`, never a `Plane`.** A Plane is dropped by the
  MuJoCo converter and substituted by the implicit ground plane, which masks
  whether your collider is real.
- **Nothing reads MuJoCo's instability warning channel.** A creature that NaNs or
  explodes emits no engine log line and `run-headless` still prints PASS at
  exit 0. The director carries its own per-tick watchdog; keep it.
- **Perfectly symmetric gaits produce exactly 0.0000 displacement.** `bound`
  (`[0,0,π,π]`) and `pace` (`[0,π,0,π]`) both scored exact zero, twice. That is
  the physics behaving correctly, not a stuck body — symmetric forces cancel.

## The cost dial

12 creatures = 60 dynamic bodies, 48 motorised hinges, active ground contact:

| contacts | ke / kd / cone / impratio | ms/step | best locomotion |
|---|---|---|---|
| hard | 8000 / 200 / elliptic / 10 | 21.3 | 0.889 m |
| soft | 2000 / 80 / pyramidal / 1 | 7.69 | 0.289 m |

Softer contact is 2.8× faster and costs 3× the locomotion — limbs sink in and
lose push-off. This is a genuine tradeoff, not a free optimisation. The hard
values were inherited from `omniarm6_real_pick_place.omniworld`, a precision
*grasping* world, and are almost certainly not the right point for a many-body
ecosystem; a middle setting is unswept.

Note the cost is driven by **contact-island density, not body count**. Probe 0's
belly-dragging creatures (no limb bias) ran ~12 ms/step; adding the bias that
lifts them onto their limbs took the identical scene to ~27 ms/step.

## Known broken

- `gen_terrarium.py` places crypt creatures on a 1-D line, so any `TERRA_N > 12`
  puts them past the 8 m slab and they **fall into the void forever** (there is
  no implicit ground plane). Bodies in freefall have no contacts, so a naive
  scaling sweep reports *more* creatures as *cheaper* — `TERRA_N=24` measured
  10.43 ms/step against 12 creatures' 21.29 ms. That number is an artefact.
  **Population scaling is therefore UNMEASURED.** Fix the crypt to a grid before
  sizing anything.
- `TERRA_N=4` produces no result file; uninvestigated.
- The director hardcodes `ACTUATED = range(8)`, so it does not actuate creatures
  outside slots 0–7 regardless of `TERRA_N`.

## Evolution: what works, and one unresolved problem

The pipeline runs end to end. `evolve.py` seeds a random population, and each
generation writes a fresh world, runs ONE headless engine, scores every creature
by net displacement, and truncation-selects with mutation. **Morphology evolves
because the world is regenerated every generation** -- that sidesteps the
spawn-has-no-physics blocker entirely, since a body plan never has to be created
mid-run.

Measured cost: ~15 ms/step for 12 creatures (60 dynamic bodies), ~20 s per
generation of 700 ticks. `simulationQuit(0)` in the director is load-bearing:
`run-headless --duration N` is a wall-clock SLEEP, and without it a generation
costs 61 s to do 3 s of simulation.

### The measurement is sound

Three properties verified, and they are stronger than expected:

- **Determinism holds.** The same world run twice gave 12/12 bitwise-identical
  fitness values.
- **Fitness is position-invariant.** 12 identical clones of one champion at 12
  different grid positions scored *exactly* the same, spread **0.0000 m**.
- **Neighbours do not interact** at 20 m spacing (alone / with clones / with its
  original cohort all agreed to 4 decimals).

### ...but evolution-time fitness does not reproduce

**Only 1 of 21 champions reproduces its evolution-time score.** The gen-7
champion recorded **8.593 m** during the run and re-scores **0.392 m** in the
21-champion rescore world and **3.1475 m** in the attribution worlds. One
genome, three evaluation worlds, three different numbers -- despite each
individual world being perfectly deterministic and position-invariant.

The consequence: **scores are not comparable across generations**, because every
generation changes the population and therefore the world. Selection was partly
scoring world-context, which is why the run peaked at gen 7 and decayed to ~1 m
by gen 23 even though elitism carried the champion forward intact.

**This is UNRESOLVED.** Determinism, position, neighbour interaction and
collision are all individually ruled out by measurement, so the cause is not yet
attributed. Do not quote evolution-time fitness. `rescore.py` measures every
champion simultaneously in one world, which IS internally consistent, and that
is the only ranking used downstream.

### The honest result

Under the one-world rescore: **first champion 0.631 m -> best champion 1.904 m,
a 3.0x improvement**, and the best body is from the *last* generation (gen 22,
a 6-limb mixed pitch/yaw plan) -- so the run was still improving, contrary to
what the evolution-time trace suggested.

## Demo

```bash
python projects/alife/evolve.py --generations 24 --pop 12 --ticks 700   # evolve
python projects/alife/rescore.py                                        # honest ranking
python projects/alife/showcase.py --count 5 --spacing 3.5               # build lineup
python projects/alife/watch.py                                          # watch it (lean)
```

### Real-time performance (measured, machine `9722d23d12a3`)

The 5-champion showcase is **realtime-capable with ~2x headroom**: physics
costs **4.16 ms/step against the 8 ms tick** (`bench_realtime.py`), so a
windowed engine in its default realtime mode idles roughly half of every step.
The earlier 11.3 ms/step read through the harness was harness round-trip
overhead, not engine cost.

Two cheaper configs were benchmarked and REJECTED because both degrade the
gaits the champions evolved under (fitness is physics-config-dependent):

| config | ms/step | budget | champion displacement |
|---|---|---|---|
| **dt8 + hard contacts (kept)** | 4.16 | 8 ms | **1.854 m** |
| dt16 + hard | 4.08 | 16 ms | 0.779 m |
| dt8 + mid contacts | 4.06 | 8 ms | 0.519 m |
| dt16 + mid | 4.00 | 16 ms | 0.370 m |

GPU cost is entirely the per-frame wgpu realism stack (screen-space passes;
cost tracks window resolution, not scene contents -- physics is CPU `mj_step`
and never touches the GPU). `watch.py` disables the passes a flat arena cannot
benefit from -- SSR, TAA, volumetrics, PCSS (plain CSM shadows remain). All
four env knobs are value-parsed (`=0` disables), verified in
`OmWgpuRenderTarget.cpp`. Measured A/B, single engine, same window:

| arm | GPU util avg | max | power |
|---|---|---|---|
| idle desktop | 0% | -- | 14.7 W |
| full stack | 36.4% | 53% | 31.7 W |
| `watch.py` lean | 29.3% | 45% | 28.6 W |

Two rules that matter more than any knob: **never pass `--mode=fast` to a
windowed session** (it uncaps stepping and burns everything), and **never leave
a second engine running** (check with PowerShell `Get-Process omnisim-bin`;
`pgrep` does not exist in this Git Bash and its failure reads as an all-clear).
`OMNILIGHT=0` does NOT save GPU -- the GI bake is async CPU work (~0.4 s once)
with unchanged frame cost.

Champions line up left-to-right slowest-to-fastest on coloured start pads, so
how far each has travelled is readable straight off the frame.

`controllers/terrarium_bridge/` serves the same cast over HTTP for an agent to
operate (`/census`, `/perturb`, `/cull`, `/revive`, `/reset`). Reads come from an
immutable snapshot the sim thread republishes each tick; writes are marshalled
onto the sim thread and rejected with 409 while one is in flight. Every result
carries measured state, never the argument echoed back.

## Files

- `gen_terrarium.py` — world generator. Env: `TERRA_N`, `TERRA_KE`, `TERRA_KD`,
  `TERRA_CONE`, `TERRA_IMPRATIO`, `TERRA_OUT`. Never hand-edit the world.
- `controllers/terrarium_director/` — the probe instrument (P1–P6). Env:
  `PROBE_TICKS`. Writes `_probe_result.json`.
- `sweep.sh` — serial contact/scale sweep. One engine process at a time.

---

# v2 — the living ecosystem (flagship)

`DESIGN_v2.md` is the contract; this section is what happened when it met the
engine. Everything measured on machine `9722d23d12a3`, CPU `mj_step`, dt 8.

## What it is

Bilateral quadrupeds with capsule bodies, a head, hip + knee per leg. One
supervisor (`controllers/terrarium_life/`) senses for every creature, steers
it toward the nearest food, drives every hinge through batched field writes,
burns its energy, feeds it, breeds it (a child inherits a mutated brain into a
pooled slot of the same species) and buries it. Between **epochs** the world
is regenerated with the surviving species' bodies mutated, so morphology
evolves too. Food is visual-only (zero physics cost). An HTTP bridge on
`:8790` (`/census`, `/feed`, `/cull`, `/spawn`, `/perturb`) lets an agent run
the ecosystem.

```bash
python projects/alife/ecosystem.py --epochs 8 --species 3 --slots 2 --alive 1 --epoch-s 90 --arena 18
python projects/alife/watch_life.py          # windowed, lean render profile
```

## The path to creatures that actually walk (each step measured)

Seven things had to be found by measurement before a single creature foraged.
In order:

1. **Random CPG gaits on random bodies never steer.** Seven epochs of the
   first design stayed at 0-4 eats with food always in range. A steering sweep
   on the best forager (`probe_steer.py`) found no monotone channel: turn -1 /
   -0.5 / -0.25 / 0 / +0.25 / +0.5 / +1 gave curvatures +1.70 / +0.70 / +0.49
   / -0.77 / -1.23 / +0.29 / -1.33 rad/m. Evolution cannot select a behaviour
   the physics never offers.
2. **Chiral gaits corkscrew.** Pairs at arbitrary phase offsets spin the body
   at 1.6 rad/s against a steering authority of 0.49 rad/s. Real gaits are
   symmetric; pair phases are now locked to trot (`genome2.lock_gait`).
3. **Wide hip swings put every body on its belly** (torso z = radius,
   `_life_t12.png`); **narrow stances flip every body onto its back within
   3 s** (`_life_t6.png`). Fixed by per-joint gait envelopes and a lateral
   stability margin (stance half-width / rest height >= 0.75, i.e. a sprawler)
   enforced in `validate()`, plus a righting reflex in the director.
4. **`setVelocity([0]*6)` after a teleport freezes the body for ~2 s**
   (`terrarium_probe_tp`: the control moved at once, the reset body sat for
   240 ticks). `resetPhysics()` does not zero velocity either (engine:
   UNIMPLEMENTED). So dead slots park on a far static slab, at rest, and revive
   is a plain teleport.
5. **The launch transient flips a newborn.** Knees snapping from 0 to their
   bias in one tick threw the archetype 10 cm up and onto its side (torso z
   0.215 -> 0.319 -> 0.050). A 1.5 s gait fade-in plus gentle limb motors
   (maxTorque 1.5, maxVelocity 4) turned the SAME body from a non-walker into
   a **0.73 m/s straight walker with zero flips** (`probe_steer2.py`).
6. **Steering works on the designed walker, at small gains only.** Amplitude
   asymmetry 0.3: -0.30 rad/m, still 0.79 m/s, no flip. 0.6: collapse.
   Stride-bias asymmetry: flips. Gain capped at 0.32; proportional law with a
   0.12 rad deadband; turn radius ~3 m, so wall avoidance starts 3 m out.
7. **Steering polarity is body-specific and unpredictable**, so each newborn
   wiggles (1 s left, 1 s right, after the fade-in): the difference of the two
   yaw responses is its steering sign, the sum its drift.

The walker archetype (torso 0.30 x r0.05, two pairs at x +-0.7, two 0.12 m
segments, splay 0.6, trot at 1.2 Hz, hip amp 0.35, knee amp 0.35 with bias
-amp and +pi/2 lag) is the centre of the genome envelope; evolution explores
proportions, gait amplitudes/frequency, sense radius, steering gain and colour
within it.

## Results

8 epochs, 3 species x 2 slots, 18 m arena, 12 food active, 90 s each (one
engine per epoch, 832 s wall total):

| epoch | births | deaths | eats | best species (eats) | rightings | ms/step |
|---|---|---|---|---|---|---|
| 0 | 2 | 2 | 13 | sp1 (9) | 40 | 8.2 |
| 1 | 2 | 0 | 13 | sp1 (9) | 17 | 7.4 |
| 2 | 1 | 1 | 6 | sp1 (5) | 25 | 4.9 |
| 3 | 4 | 1 | 15 | sp3 (6) | 34 | 8.3 |
| 4 | 3 | 0 | 22 | sp6 (12) | 14 | 7.2 |
| 5 | 3 | 0 | 24 | sp1 (14) | 9 | 8.3 |
| 6 | 3 | 0 | 28 | sp8 (20) | 13 | 8.3 |
| 7 | 2 | 0 | 25 | sp8 (17) | 13 | 6.9 |

Foraging doubled across the run (13 -> 25-28 eats per epoch; best species
9 -> 20 eats in 90 s) while falls dropped by two-thirds (40 -> 13 rightings):
selection is acting on both the brains (within an epoch) and the bodies (sp1
-> sp6 -> sp8 is one lineage of body mutations). Compare the first design's
seven epochs at 0-4 eats with no trend. Engine cost 5-8 ms/step against the
8 ms realtime tick at the typical population.

## Honest limits

- Realtime holds at the typical population (~4 alive, ~6-8 ms/step) and is
  exceeded at the 6-slot peak (~11 ms/step). Sized for a laptop.
- Aim precision is modest: creatures find food by walking a lot (40-95 m per
  life) with a 3 m turn radius, not by beelining. Tighter turning needs a
  steering mechanism this gait does not have (in-place pivot).
- Creature-creature collisions still knock bodies over; the righting reflex
  recovers them at an energy cost.
- Bodies are two-pair only (three pairs = 13 bodies, over the realtime budget).
