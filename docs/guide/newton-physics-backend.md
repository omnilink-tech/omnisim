# Physics backends — Newton (default for new work) and ODE (legacy fallback)

OmniSim's solver is **Newton** — NVIDIA's GPU-accelerated rigid-body solver, built on Warp. Newton is what we recommend for any world authored from now on and what new templates, demos, and benchmarks default to. It's the simulator's direction.

**ODE** remains supported as a legacy fallback for:

- Machines without an NVIDIA GPU (CI runners, contributors without CUDA).
- Worlds inherited from Webots that depend on ODE-specific semantics (deterministic contact ordering, exact bit-reproducibility, certain joint types).
- RL workflows whose checkpoints / regression baselines were captured against ODE and would need re-baselining if flipped to Newton.

The `physicsBackend` field selects between them per Solid (or per Robot — the field cascades). The schema default is `"auto"` and, as of the Stage 3 build-default flip, Newton is also compiled in by default. An `"auto"` world picks Newton when the runtime is present **and** the articulation uses only Newton-faithful features (the capability gate — see ["How `auto` stays safe"](#how-auto-stays-safe--the-capability-gate)), and falls back to ODE otherwise. Set `physicsBackend "ode"` to pin a Solid to the legacy solver. See [engine-migration-plan.md §13.4](../developer/engine-migration-plan.md) and [default-flip-plan.md](../developer/default-flip-plan.md) for the rollout.

## When to opt out (use ODE explicitly)

The default direction is Newton. Use ODE explicitly when:

- You're authoring a deterministic regression test where solver-noise variance would obscure the signal.
- You're working on a machine without NVIDIA hardware and your world has a few small robots that ODE handles fine anyway.
- You're updating a Webots-era world whose behavior is calibrated against ODE's specific contact / joint semantics.

For everything else — many-robot scenes, policy training where throughput matters, new demos and benchmarks — stay on the Newton default.

## Selecting a backend explicitly

Newton selection is per-Solid (or per-Robot) via a single SFString field:

```vrml
URDFRobot {
  url "path/to/your_robot.urdf"
  physicsBackend "newton"     # default is "auto" (Newton if available, else ODE)
}
```

The field cascades down the scene tree: setting `physicsBackend "newton"` on an outer `URDFRobot` opts every Solid in that robot's subtree into Newton (chassis, wheels, sensors, arms — all of them). You don't need to mark each link individually.

For a standalone `Solid` (not inside a URDFRobot):

```vrml
Solid {
  translation 0 0 1
  children [
    Shape { ... }
  ]
  physics Physics { mass 1 }
  boundingObject Sphere { radius 0.5 }
  physicsBackend "newton"
}
```

If the field is omitted, it defaults to `"auto"` (Phase D, 2026-05-28): the world picks Newton automatically on Newton-capable machines and falls back to ODE on hardware without it. Existing Webots-era worlds that need ODE's exact semantics should set `physicsBackend "ode"` explicitly. See [engine-migration-plan.md §13.4](../developer/engine-migration-plan.md).

## What works on Newton today

- **Articulated rigid-body dynamics** — chassis + revolute joints, the bread-and-butter for vehicle robots.
- **URDF importer routing** — the URDF loader recognizes `physicsBackend "newton"` and builds the robot as a Newton articulation instead of an ODE body tree.
- **drive_forward** and most velocity-controlled motor controllers — they work unchanged because the public motor API is backend-agnostic.
- **Sensor reads** (Accelerometer, GPS, Gyro) — these read body state polymorphically through `WbPhysicsBackend`. Newton-backed sensors return real Newton dynamics, not bridge-proxy stale values.
- **Position-controlled motors** via XPBD joint targets — joints created with non-zero `targetKe` gains accept `setJointTargetPosition` / `setJointTargetVelocity` calls each tick.

## How `auto` stays safe — the capability gate

A world without an explicit `physicsBackend` resolves to `"auto"`, which picks Newton **only when both** (a) the Newton runtime is present and (b) the Solid's whole articulation uses only Newton-faithful features — otherwise it resolves to ODE, per articulation, logged once. The gate (`WbSolid::articulationNewtonCapable`) is what makes a default flip safe: a world can never be *silently* degraded onto a solver that can't run it faithfully.

- **Mesh collision geometry** → ODE. Newton only AABB-approximates a mesh boundingObject, so an articulation with mesh collision stays on ODE for exact contact (N2). Primitive boundingObjects (box / sphere / cylinder / capsule) run on Newton.
- **Non-Hinge/Slider joints** → ODE. Newton registers HingeJoint + SliderJoint; ball / hinge2 / AMotor / LMotor are not yet registered, so an articulation containing one stays *wholly* on ODE (never a mixed-solver articulation). Hinge + slider — including position- and velocity-controlled motors — run on Newton.

Bypass for A/B testing: `OMNISIM_AUTO_NO_CAPABILITY_GATE=1` forces `auto`→Newton even for unsupported features (may degrade — your choice). Explicit `physicsBackend "newton"` is also an unconditional opt-in.

## Known limitations

- **External body forces** — `wb_supervisor_node_add_force()` / `add_force_with_offset()` / `add_torque()`, `WbPropeller` thrust, and interactive mouse-drag apply force through the ODE body path. Newton drives via joint targets, not arbitrary per-body force writes, so these don't act on a Newton-backed Solid. The supervisor APIs now **warn** ("…is not supported on the '<backend>' physics backend; drive the joint via target velocity/position instead") rather than silently dropping the call. For genuinely force-driven worlds (propeller drones, scripted pushes), set `physicsBackend "ode"` (N4).
- **High closing-speed contact** — XPBD's positional solve can diverge to NaN when a body moves a large fraction of a contact's size in one tick (e.g. a head-on at full drive speed). Raise `newtonSubsteps` (see Tuning); the base-divergence guard additionally freezes the articulation at its last good pose instead of exposing a NaN to controllers.
- **Legged locomotion** — single-robot legged-deploy worlds select `WorldInfo.newtonSolver "mujoco"` (robust frictional contact). Quadruped walking (Spot / Go2 / B2) is trained and deployed on this path. Humanoid (G1) walking demos run on a **weight-bearing balance harness**, not free-standing; a durable unassisted humanoid walk is an open problem. Do not quote any legged result without reading the canonical status first: [rl-current-state.md](../developer/rl-current-state.md).
- **Damage system parity** — validated to *practical* parity (within the `damage_events_diff` 10× tolerance): a 50/30 head-on gives 58 Newton vs 39 ODE events with `newtonSubsteps 4` + `OMNISIM_DAMAGE_VEL_SMOOTH=5`. Exact backend-symmetric parity awaits an engine-side contact-impulse/depth API (designed, not yet landed).

## Tuning knobs (`WorldInfo`, no env vars)

Two `WorldInfo` fields fold the former launch env vars into the world file. Defaults preserve today's exact behavior (every existing world, husky, and trained RL policy is byte-unchanged); the env vars still override.

- `newtonSubsteps N` (default `1`) — split each physics tick into N XPBD sub-steps. Use N≥2–4 for high closing-speed contact. (Env override: `OMNISIM_NEWTON_SUBSTEPS`.)
- `newtonStatics TRUE` (default `FALSE`) — register top-level static colliders (floors, walls) as mass-0 Newton bodies. A default ground plane is always present on the XPBD path, so wheeled robots drive without this; the forced-`SolverMuJoCo` legged-deploy worlds need it for a ground. (Env override: `OMNISIM_NEWTON_STATICS`.)

The **base-divergence guard is on by default**: if Newton's base state goes non-finite or beyond `OMNISIM_NEWTON_BASE_GUARD_MAX` metres (1000), the articulation freezes at its last good pose (logged once) rather than feeding garbage to controllers. It is a strict no-op for any physically-valid state. Disable for legacy studies with `OMNISIM_NEWTON_BASE_GUARD=0`.

## How the two backends coexist

OmniSim's `WbPhysicsBackend` is a per-Solid dispatcher. Each `Solid` resolves to either the ODE or Newton backend based on its `physicsBackend` field. In a mixed scene:

- ODE-backed Solids step through ODE as usual.
- Newton-backed Solids step through Newton.
- **Cross-backend contacts** (e.g. a Newton-backed husky's wheel touching an ODE-backed floor) are bridged each step: the Newton body's AABB is registered as a kinematic proxy in ODE, ODE detects contacts and computes impulses, and the impulses are fed back to Newton as external forces.

The bridge costs about 1 ms/step for ~10 GPU bodies — acceptable during the migration period. Eliminated entirely if every dynamic body in the scene is on Newton (P8 of the migration plan).

## Build requirements

Newton is compiled in **by default** as of the Stage 3 default-flip (`OMNISIM_WITH_NEWTON ?= ON` in the Makefile). A plain build gives you Newton:

```bash
make -C src/omnisim release
```

For a **pure-ODE legacy build** (zero Newton code linked in — e.g. a machine without the toolchain to embed CPython, or a deterministic-regression box), pass the flag OFF. This is permanent and one flag away:

```bash
make -C src/omnisim OMNISIM_WITH_NEWTON=OFF release
```

### Runtime requirement — install it, or use a release build that bundles it

Newton runs on a Python runtime (`warp-lang` + `newton`). A build *links* the Newton backend but uses Newton at world load only when that runtime is importable by the binary's embedded interpreter. There are two ways it reaches you:

- **A packaged release bundles it.** Release builds vendor a self-contained CPython + `warp` / `newton` / `mujoco-warp` / `usd` next to the binary (`make bundle-newton-runtime`, ~600 MB — `warp` carries its own slim CUDA subset, so there is no separate multi-GB toolkit), so a stock install runs Newton with **no** manual pip/PATH step. The embedded interpreter resolves the bundle via a `python3XX._pth` redirect — see [default-flip-plan.md §4.3.1](../developer/default-flip-plan.md).
- **A self-built / dev binary needs the runtime installed.** A from-source build does not stage the runtime unless you run the bundler. Install it into the Python the binary embeds, and have an NVIDIA GPU + recent driver (Newton/Warp is GPU-accelerated; a CPU MuJoCo solver path also works for single-robot deploy):
  ```
  pip install "newton[examples]" warp-lang
  ```
  (The upstream PyPI package was renamed from `newton-physics` to `newton` in late 2025.)

If the runtime is absent, `WbNewtonBackend::isAvailable()` is false and every `auto`/`newton` Solid **falls back to ODE** — safe, and logged once per world — so the simulator always runs, but such a machine is effectively still on ODE.

## Runtime fall-through

Even with `OMNISIM_WITH_NEWTON=ON`, the Newton runtime can fail to initialize (missing GPU, driver mismatch, Python package not installed). In that case:

1. `WbNewtonBackend::isAvailable()` returns false.
2. `WbPhysicsBackendRegistry::resolve(WbPhysicsBackendKind::Newton)` silently returns the ODE backend.
3. The world still loads and runs, with all `physicsBackend "newton"` Solids handled by ODE.
4. One `WbLog::warning` per world identifies the fall-back so the user knows.

Worlds that depend on Newton-specific behavior (XPBD position control, GPU contact resolution) will still load and run, but won't get the Newton characteristics.

## Sample worlds

- `projects/samples/demos/worlds/physics/newton_smoke_test.wbt` — single Newton sphere falling on an ODE floor. The minimal validation that the Newton runtime came up.
- `projects/samples/demos/worlds/physics/newton_husky_smoke_test.wbt` — single Newton-backed Husky drives forward.
- `projects/robot_combat/worlds/tests/newton_husky_head_on.wbt` — two teams of four Newton Huskies in a head-on collision arena.
- `projects/robot_combat/worlds/demos/newton_husky_combat_2.wbt` — combat-style multi-husky arena.

## Where to learn more

- [docs/developer/engine-migration-plan.md](../developer/engine-migration-plan.md) — the master physics plan: solver decision, architecture, current phase status, measured perf, and remaining work.
- [docs/developer/default-flip-plan.md](../developer/default-flip-plan.md) — how Newton became the compiled-in default, and the runtime bundling.
- Newton documentation: https://github.com/newton-physics/newton
- Warp documentation: https://nvidia.github.io/warp/
