# omnibench / step_cost — what does one engine tick cost, per backend?

One question, one instrument, both backends: **how many milliseconds does a
single basic timestep cost the engine, on ODE and on Newton, for the same
scene on the same machine?**

```bash
python tests/benchmarks/omnibench/step_cost/run_step_cost.py                    # default sweep
python tests/benchmarks/omnibench/step_cost/run_step_cost.py --bodies 5 --repeats 5
python tests/benchmarks/omnibench/step_cost/run_step_cost.py --sweep 1,20,200 --json out.json
python tests/benchmarks/omnibench/step_cost/run_step_cost.py --airborne         # contacts off
```

## Why it exists

Three consecutive optimisation commits (`899eb425`, `7b3762f7`, and the
`a6aa9e54` revert) were steered by the figure *"Newton ~0.74 ms/step against
ODE's 0.12 ms/step on the same scene"*. **No script in the tree produced
either number.** They were ad hoc — which is exactly the "unreproducible
headline" failure [SPEC.md](../SPEC.md) exists to prevent, and it left the
optimisation target unfalsifiable.

It also answers the question that decides whether the target is reachable:
Newton's per-tick cost is dominated by a **fixed** conversion overhead that
does not scale with the scene (`mj_step` measures ~0.034 ms on a 1-body world
*and* on a 3-link pendulum), while ODE's cost scales with bodies and contacts.
Two curves that cross. `--sweep` finds where.

## The measurement

`--log-performance=<csv>,<N>` makes the engine log its own phase timers for
exactly the first `N` basic timesteps and then stop. Run the **same world**
twice with two different `N` and difference the totals:

```
per_step = (TOT_long - TOT_short) / (N_long - N_short)
one_time =  TOT_short - N_short * per_step
```

Differencing two nested windows **cancels every one-off cost exactly**,
without having to know what it is or where it lands. That matters more here
than it usually would: Newton's `finalizeWorld()` runs *inside* the physics
bracket on the first tick and measures **~2.0 s** on a 5-body world, so a
single-window average over 1200 steps reports ~2.7 ms/step for a tick that
actually costs ~1.0. **Every short-run Newton figure in this repo is inflated
by that term**, including — very likely — the 0.74 ms it has been optimised
against.

Both runs execute the identical number of steps (the controller is told
`N_long + margin` either way); only the logging window differs. So the two
processes do the same work and differ in nothing but what they recorded.

## What is and is not attributed

| bucket | contains |
|---|---|
| `physics` | [`OmSimulationWorld.cpp:243-301`](../../../../src/omnisim/engine/OmSimulationWorld.cpp#L243) — the Newton registration flush, the motor-target push, `newton->step()`, **and `mCluster->step()`** (the ODE pass, which runs unconditionally even in a fully-Newton world) |
| `postPhysics` | the per-Solid pose readback (`getBodyXform` / `getBodyVelocity`) — where Newton's per-body FFI lands |
| `total` | `prePhysics + physics + postPhysics` — the honest "what one tick costs the engine" number |

The **controller** column is reported but deliberately **not** included. It is
the controller IPC round trip, identical in shape on both backends, and
folding it in is what makes an ODE tick look like 0.12 ms when its physics
phase costs 0.025.

The step-driver controller ([`step_cost_runner`](controllers/step_cost_runner/))
reads no poses, no sensors, no contacts — every such read is an FFI round trip
that would land in `postPhysics` and contaminate the number being reported.

## The scene

`N` unit boxes (0.1 m, 1 kg) on a 40×40 static floor, laid out on a grid with
0.5 m spacing so **no box touches another** — the only contacts are box↔floor,
so contact count scales linearly and cleanly with `N`.

The floor's top face is at `z = 0.5`, deliberately **above** Newton's implicit
`z = 0` ground plane, so both backends rest the boxes on the same authored
geometry and the implicit plane is never the thing being measured. The world
sets `newtonStatics TRUE` (without it the floor is intangible under Newton and
the boxes would fall to the phantom plane instead), `physicsDisableTime 0` (ODE
auto-sleep off — a sleeping body generates no contacts and would flatter ODE),
and `newtonSolver "mujoco"` (CPU `mj_step`).

`--airborne` lifts the boxes clear of the floor instead, which isolates the
body-count cost from the contact-solving cost.

Worlds are generated deterministically at run time and deleted afterwards
(`--keep-worlds` to inspect them).

## Honesty rules this obeys

- **Backend is proven, never assumed.** Newton rows require a present,
  non-degraded, finalised `.newton.json` verdict sidecar; ODE rows require its
  absence. A row whose backend could not be proven is **dropped**, not guessed.
- **Every result names a machine.** The `machine` field is
  `env_fingerprint`'s identity (GPU model + CPU + *hashed* hostname). A raw
  hostname is a personal identifier and one reached the committed 2026-07-24
  results and had to be scrubbed — do not "simplify" this to `COMPUTERNAME`.
- **Repeats and spread are reported**, not just a median. Lane 1 has
  demonstrated ~28% run-to-run spread on identical configs against n=1 cells.
- **No cross-instrument comparison.** Both backends go through the same
  binary, the same launch shape, the same controller and the same CSV.
