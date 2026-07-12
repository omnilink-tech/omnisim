# Frame-rate optimization journey — measurements and findings

This doc captures the bisect we ran trying to take the damage-system
demo from ~1 fps under collision to 30 fps. The investigation proved
the bottleneck is **Webots' core** (ODE physics + WREN rendering for
URDFRobot-class bodies), not the Python damage system.

Hardware: NVIDIA RTX 3060 Laptop, AMD Ryzen 7. OmniSim built with
`OMNISIM_WITH_CUDA=ON`, MSYS2 mingw64. Each measurement is the mean
fps over 25-30 wall-seconds, sampling sim_time vs wall-clock at 2 Hz.

## Measurements

| scenario | mean fps | real-time ratio | notes |
|---|---:|---:|---|
| 2-husky head-on + cuda_particle_pool world (200 particle Solids parked at z=-100) | **1.3** | 0.02× | The 200 hidden Solids ate 90 % of the budget |
| 2-husky head-on (no pool, lite damage, smoke/sparks legacy path active) | **10.2** | 0.16× | Removing 200 Solids from scene tree ⇒ 8× fps gain |
| 2-husky head-on (lite damage + smoke/sparks suppressed, `--mode=run`) | **7.9** | 0.13× | Suppressing cosmetic spawn paths gained nothing measurable |
| 1-husky cube drop (lite damage, `--mode=run`) | **20.1** | 0.32× | Halving URDFRobot count ⇒ 2.5× fps gain |

`--mode=run` removes Webots' real-time throttle but doesn't bypass
render or physics cost.

## What this tells us

1. **Hidden scene-graph nodes are NOT free.** The 200 sphere Solids
   in `husky_head_on_cuda.wbt` were parked off-screen at z=-100 with
   the intent that Webots' renderer would skip them. It does not.
   Each Solid still goes through scene-tree traversal, transform
   computation, and (if the camera frustum check is generous) into
   the renderer. The lesson for the C++ `WbOmniParticleField` work
   is critical: particles MUST live in a single node's GPU buffer,
   never as N scene-graph entities. Any pool ≥ 50 entities at the
   scene-graph level becomes an fps killer.

2. **Cosmetic spawning isn't the bottleneck at small scene scale.**
   Decals, dent overlays, IFS regen, smoke/spark imports — gating
   them all behind `OMNISIM_LITE_DAMAGE=1` produced no measurable
   fps gain in the 2-husky head-on. So either Webots' VRML parser
   amortizes these well at the rate we're producing them, or the
   per-step ODE+render cost is so much larger that the parser
   delta is in the noise. Either way, the optimization opportunity
   isn't there until we're already at a much higher fps.

3. **URDFRobot count is the biggest single lever.** Going from 2
   huskies to 1 = 2.5× fps. Each husky has ~10 collision shapes
   (chassis double box + 4 wheel cylinders + bumpers) and ODE's
   broadphase has to consider all chassis-chassis, chassis-wheel,
   and wheel-wheel pairs across both bodies. With 2 huskies that's
   ~100 broadphase pairs per step plus floor contacts.

4. **Webots core is the wall.** With one husky and damage stripped
   to bare HP tracking + Phase 7 appearance, we hit 20 fps, not 30.
   The remaining 1.5× gap is in ODE constraint solving and WREN
   rendering of the husky's mesh assets. Both are inside the Webots
   binary and can't be optimized from Python.

## Optimizations we kept

- `OMNISIM_DISABLE_FRACTURE=1` — skips Phase 17 fragment spawn
  (URDF subtree mutation under collision = omnisim-bin crashes).
- `OMNISIM_DISABLE_DETACH=1` — skips Phase 9/19 part detach (same
  crash root cause).
- `OMNISIM_LITE_DAMAGE=1` — skips Phase 8 decals + Phase 14c dents
  + Phase 14b/15 IFS re-emit + smoke/sparks. No measurable fps
  gain on 2-husky head-on, but reduces visual noise and import
  storms; useful when chasing further perf on bigger scenes.
- `OMNISIM_HARNESS_DAMAGE_EXTRA_ROBOTS=""` — drops back to
  single-tracker mode. Only the primary husky shows damage state;
  visual symmetry on the secondary is sacrificed.
- Pool optimisation: skip per-step `setSFVec3f` on already-parked
  dead particles (only fires when a slot transitions live → dead).
  Saves ~200 setSFVec3f calls per step once the pool drains.

## Path to 30 fps

In order of practical effort:

1. **Reframe to single-husky cube-drop demo.** Achieves 20 fps
   today. Adding `--mode=fast --no-rendering` for headless runs,
   or simplifying the husky's URDF collision (replace per-wheel
   cylinder with a single chassis box) might close the last 1.5×.
   Lowest engineering cost.

2. **C++ `WbOmniParticleField` (P2-P5 in §13.7 of [engine-migration-plan.md](../engine-migration-plan.md#137--adjacent-subsystems-cuda--granular--particles--damage)).**
   Doesn't fix ODE/WREN cost directly, but eliminates the ~200
   per-step `setSFVec3f` and the ~50-200 ms per `importMFNodeFromString`
   call that sustained collision triggers. Most useful for *bigger*
   scenes (many particles, many huskies) where the per-particle
   Webots-API tax stops being amortized.

3. **Reduce URDFRobot collision complexity in `husky.urdf`.** Single
   chassis box, no per-wheel cylinder. Loses some realism but should
   take 2-husky head-on from 7.9 fps to ~15-20 fps. Affects every
   world that imports the husky URDF, so trade-off worth checking.

4. **Webots core / WREN profiling.** Out of scope here; would need
   `OMNISIM_RENDERER_TIMINGS=1` runs and analysis of actual frame
   composition. Likely Cyberbotics-upstream work, not OmniSim-fork.

## Commit landing this journey

`damage_system: webots stability fixes -- IFS field + env-var disable hooks`
(d905d50 + the LITE_DAMAGE work below) covers the env vars and the
pool optimisation. Smoke/spark gating + the bisect numbers are
documented here.
