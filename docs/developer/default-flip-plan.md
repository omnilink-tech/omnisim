# Default-Flip Plan — ODE + WREN → Newton + wgpu (legacy kept as fallback & oracle)

> ## ⚠️ SUPERSEDED 2026-08-08 FOR THE PHYSICS ARM — THE FLIP HAPPENED AND THEN THE FALLBACK WAS DELETED
>
> **Read [ode-retirement-campaign.md](ode-retirement-campaign.md) for the physics arm.**
> This document's subtitle — *"legacy kept as fallback & oracle"* — is the part that no
> longer holds. `bdc02139` deleted the ODE backend, so on the physics side there is **no
> fallback and no oracle**: `OMNISIM_LEGACY`, `OMNISIM_FORCE_ODE` and
> `OMNISIM_ALLOW_ODE_FALLBACK` select nothing, and the safety harness this document
> designs (interleaved A/B against a legacy arm, byte-identity gates, one-switch revert)
> **cannot be run for physics any more**.
>
> Two clarifications so this is not read as a repudiation:
>
> - **The rendering arm is still live and unaffected.** WREN remains canonical, wgpu is
>   opt-in (`renderBackend "wgpu"`), the flip is human-gated, and every render-side
>   mechanism described below still works. Read the render half as current.
> - **The physics half succeeded on its own terms first.** The default flipped, the
>   silent-fallback init bug was fixed (`6a459f84`), the solver default moved to
>   `SolverMuJoCo` (`7b431e81`), and XPBD was then removed (`94f04222`). Deletion came
>   after, as a separate owner decision — not because this plan failed.
>
> ⚠ One methodological loss worth naming: the "silent ODE fallback" failure mode that this
> document spends considerable effort detecting is **gone as a failure mode** (a broken
> Newton runtime is now a hard failure), but the *detection discipline* it established is
> still the right one — assert the `.newton.json` sidecar, never the exit code, never a
> tail-scraped log line.

> **Update 2026-06-23 — the physics default is now *real* for a stock release.**
> Three changes closed the gap between "build flag ON" and "an end user actually
> gets Newton":
> 1. **Silent-ODE-fallback INIT BUG FIXED (`6a459f84`).** Newton was silently
>    falling back to ODE on headless runs: warp prints a banner on import, but
>    under headless `stdout=DEVNULL` the C++ side had set `sys.stdout`/`stderr` to
>    `None`, so the warp import (and the FFI smoke that follows) raised and was
>    swallowed → ODE, then collapse. Fixed by a **writable-stdio guard before the
>    warp import** in `OmNewtonBackend.cpp`. Newton now reliably *engages* wherever
>    the runtime is present. **The real "Newton is active" signal is the
>    `[OmNewtonBackend] world finalised (solver=...)` line — NOT an earlier
>    `imports OK`** (imports can succeed and the solver still not bind).
> 2. **`OMNISIM_REQUIRE_NEWTON` guard added (`cfb11d06`).** Opt-in (default off):
>    when set, a Newton-init failure becomes `OmLog::fatal` (loud + non-zero exit)
>    instead of a silent ODE fallback — the guard to wire into deploys/CI so a
>    regression can never quietly downgrade to ODE again. `OMNISIM_FORCE_ODE` /
>    `OMNISIM_LEGACY` still win over it.
> 3. **`make release` now bundles the Newton runtime BY DEFAULT and idempotently**
>    (`577ff609` wired it via `BUNDLE_NEWTON ?= 1`, opt-out `=0`; `3de05aa3` made
>    it idempotent — skips if `$(TARGET_PATH)/newton-runtime` is already staged, so
>    dev rebuilds via `build_omni.bat` don't re-copy ~600 MB or fail). This
>    **reverses** the §4.3 "do NOT bundle" decision for releases. The build flag is
>    `OMNISIM_WITH_NEWTON ?= ON`, the schema default is `physicsBackend "auto"` →
>    Newton when the runtime is present. See [newton-runtime-bundle.md](newton-runtime-bundle.md).
>
> **Honest scope:** ODE is **not** deprecated — it remains the permanent fallback.
> The bundle closes the "stock *release* silently runs ODE" gap; a from-source
> clone (or `make debug`) **without** the runtime still falls back to ODE by
> design. So the precise statement is: build ON · schema `auto`→Newton · release
> bundles the runtime · init now reliable ⇒ the user-visible runtime default for a
> **stock release on a supported host is Newton-where-supported**, but a no-runtime
> source clone is ODE. The fidelity tail also remains (~35–40% corpus-faithful,
> 5/8 robot worlds; `ur_arms`/`mavic`/Atlas gaps). See §4.3.1 and §9.
>
> **Update 2026-06-08 — the §1 table below (dated 2026-06-06) is overtaken on
> four points; canonical status is
> [engine-migration-plan.md §8.1 (2026-06-08)](engine-migration-plan.md):**
> 1. **Render build flag flipped ON** — `OMNISIM_WITH_VULKAN ?= ON`
>    ([Makefile:251](../../src/omnisim/Makefile), C1 flip 2026-06-07), symmetric
>    with Newton. (Layer A render now ON, not OFF.) The wgpu code is still
>    double-guarded by `WB_WGPU_NATIVE_AVAILABLE`, so a plain `make` without
>    `WGPU_NATIVE_HOME` is WREN-equivalent.
> 2. **The render fallback lever now exists** — `OMNISIM_FORCE_WREN` (+ the
>    umbrella `OMNISIM_LEGACY=1` reverting both arms), so "no render equivalent"
>    of `OMNISIM_FORCE_ODE` (§0.2 / Layer C) is resolved (`459a2a21`).
> 3. **The wgpu main view is un-gated** for any `renderBackend "wgpu"` Viewpoint
>    (offscreen→GL-blit `renderMainFrameViaWgpu`, 2026-06-07, ~75% parity).
>    Remaining render work = the *default* flip + ~100% parity + cross-platform
>    surfaces, not the un-gate itself.
> 4. **Newton gaps that have since closed** (Layer D physics): mesh collision
>    (W1.3), Hinge2 (W2, a non-revolute joint), external force/torque injection
>    (W3.1), and native contacts (W4). Remaining real gaps: the PAL *write*
>    surface, ball/fixed joints, legged standing, contact-impulse under XPBD, RL
>    transfer. Measured Newton coverage is ~35–40% corpus-*faithful* (5/8 robot
>    worlds), 100% gate-eligible — see
>    [newton-ode-replacement-plan.md](newton-ode-replacement-plan.md) §2.

**Status:** living plan · drafted 2026-06-06
**Scope:** how OmniSim switches its *default* physics engine (ODE → Newton) and *default*
renderer (WREN → wgpu) **without breaking the simulator**, while keeping the legacy stack
permanently available as (a) a one-switch fallback and (b) a side-by-side **oracle** for
debugging/verifying the new stack.

**Companion docs (the per-arm detail this plan sits on top of):**
- `engine-migration-plan.md` — the physics ODE→Newton arm (phases P0–P9, A–D) and the §14 render arm.
- `r4-completion-checklist.md` — the rendering R4/3c-A/3c-B state + the wgpu un-gate blocker.
- `rl-current-state.md` — the canonical Newton robot/RL status (what actually works under Newton).

This document is **cross-cutting**: it does not re-plan either arm; it defines the *safety harness*
and the *sequencing* that turn "the new backend is opt-in and works for some worlds" into "the new
backend is the default and legacy is one switch away."

---

## 0. Guiding principles (the "doesn't break the simulator" contract)

These are invariants. Every step below is checked against them.

1. **Legacy is never deleted.** ODE and WREN stay compiled, wired, and reachable forever. The
   migration constraint "no file renamed in existing build" (see `project_engine_migration_plan`)
   still holds — this plan adds code, it does not rename or remove.
2. **The default is reversible with one lever.** At every milestone there is a single, documented
   switch that returns the whole process to pure ODE+WREN. Today that lever is half-built
   (`OMNISIM_FORCE_ODE` exists; there is no render equivalent). §3.1 finishes it.
3. **The legacy path stays byte-identical when selected.** Selecting `"ode"`/`"wren"` (explicitly,
   by world default, or by the global lever) must produce the exact pre-migration behavior. This is
   gated by the determinism smoke (physics) and the golden-image gate (render).
4. **A flip never silently degrades a world.** If the new backend cannot faithfully run a world's
   features (e.g. Newton + mesh collisions, or wgpu on a host without Vulkan), resolution falls back
   to legacy **for that world**, loudly logged — not silently wrong.
5. **No flip ships without its gate green.** Each default flip is gated on an automated check
   (determinism / golden image / combined-build smoke) plus, for the render main-view flip, a human
   sign-off. Gates run in CI before the default changes, not after.

---

## 1. Where we are today (honest state, 2026-06-06)

There are **four independent layers** that together decide what a user actually gets. Conflating
them is the main source of confusion, so the plan tracks them separately:

| Layer | Physics arm | Render arm |
|---|---|---|
| **A. Build flag** (is the new backend even compiled in?) | `OMNISIM_WITH_NEWTON` — **defaults ON** (`Makefile`, Stage 3 flip 2026-06-06; `=OFF` for pure-ODE) | `OMNISIM_WITH_VULKAN` — **defaults OFF** (`Makefile`) |
| **B. Schema default** (what an unspecified node resolves to) | `physicsBackend "auto"` — **already flipped** (Robot.wrl:13, Solid.wrl:16, Phase D) | `renderBackend "wren"` — **not flipped** (Viewpoint.wrl:17, Camera.wrl:41) |
| **C. Runtime resolve** (the dispatcher + fallback) | `OmPhysicsBackendRegistry::resolve()` — Newton-when-available, else ODE; honors `OMNISIM_FORCE_ODE` | `OmRenderBackendRegistry::resolve()` — Vulkan-when-available, else WREN; **no global force** |
| **D. New-backend completeness** (is it safe as a default?) | **~35–40% general** (70% wheeled): gaps in mesh collision, non-revolute joints, force writes, legged standing, RL transfer | **main-view OOM FIXED** (`a4fec74b`, was an app texture-cache key bug, not wgpu-native; soaks 75 s+), parity 75%, **Win32-only**; sensors (R5) + 3c-A **done & verified**; remaining: un-gate flip + cross-platform |

**Net effect (updated 2026-06-06, after the physics Stage 3 flip):** a default build (`make` with no
flags) is now **Newton + WREN** — Layer A is ON for physics, still OFF for rendering. Default worlds
route to Newton where the runtime is present and the capability gate admits the articulation, else ODE
(Layer B physics is `auto`). Rendering is unchanged: Layer A OFF, Layer B still `wren`, so a node must
opt in for wgpu. Pure ODE+WREN is one flag away: `make OMNISIM_WITH_NEWTON=OFF` (build) or
`OMNISIM_LEGACY=1` (runtime).

**The asymmetry to internalize:**
- **Physics** is *infrastructure-complete but capability-incomplete*. The flip mechanism is done; the
  blocker is Newton not yet faithfully simulating enough world features.
- **Rendering** is *capability-far-along but flip-incomplete*. Sensors and parity-on-static-scenes
  work; the blockers are the main-view un-gate, the last 25% of lighting parity, cross-platform
  surfaces, and the missing render-side fallback lever.

---

## 2. The end state ("switched")

When this plan is done, with **no** flags or env vars set on a supported host:

- A new build compiles **both** new backends in (`OMNISIM_WITH_NEWTON=ON`, `OMNISIM_WITH_VULKAN=ON`
  become the build defaults) and ships their runtime deps (`wgpu_native.dll`; Newton/Warp Python).
- An unspecified Solid resolves to **Newton** (when the world's features are Newton-supported, else
  ODE — see §4.1 capability gate). An unspecified Viewpoint/Camera resolves to **wgpu** (when the
  host has a working surface, else WREN).
- **`OMNISIM_LEGACY=1` returns the entire process to ODE + WREN**, byte-for-byte. So do the
  finer-grained `OMNISIM_FORCE_ODE` / `OMNISIM_FORCE_WREN`, and per-node `physicsBackend "ode"` /
  `renderBackend "wren"`.
- The **dual-backend oracle** (§3.3) can run any world on legacy and new simultaneously and diff
  them — the debug/verify path the user asked for.

Legacy is not a deprecated branch; it is a **first-class, supported, tested fallback and reference
implementation**.

---

## 3. The safety harness — BUILD THIS FIRST

This is the foundation that makes the whole migration safe and reversible. None of it flips a
default; all of it is additive. **Until §3 is green, no default flips (§4.5, §5.5).**

### 3.1 Symmetric, unified legacy escape hatch  — ✅ DONE (commit 459a2a21)
Today only physics has a global override. Finish the symmetry:

- **Keep** `OMNISIM_FORCE_ODE` (read in `OmPhysicsBackend.cpp:272`, short-circuits `resolve()`).
- **Add** `OMNISIM_FORCE_WREN` — mirror it in `OmRenderBackendRegistry::resolve()`
  (`OmRenderBackend.cpp:108`): when set, return `wrenBackend()` for every kind, ignoring the field.
  This is the render-side lever that does not exist yet (confirmed by grep).
- **Add** `OMNISIM_LEGACY=1` — a single umbrella that implies *both* `OMNISIM_FORCE_ODE` and
  `OMNISIM_FORCE_WREN`. One lever, whole process back to legacy. Read once, cached (same one-time
  static pattern as the existing `forceOde()` lambda).

Acceptance: with `OMNISIM_LEGACY=1` on a both-backends-ON build, the determinism smoke and the
golden-image gate are byte-identical to a pure ODE+WREN build.

### 3.2 World-level default override (no per-node editing)  — ✅ DONE (commits baebd7c7 physics, 6592b4b9 render)
A world author (or a bug repro) must be able to pin legacy for an *entire world* without editing
every Solid/Viewpoint. Add to `OmWorldInfo`:

- `defaultPhysicsBackend ""` — when non-empty, supplies the value for any Solid whose field is
  `"auto"`/empty (consulted inside `effectivePhysicsBackendName()` before the `auto`→Newton step).
- `defaultRenderBackend ""` — same for Viewpoint/Camera resolution.

This gives three nested scopes (process env > world default > per-node), all resolving through the
existing registries. It is the clean way to keep an old world on legacy after the schema default
flips, and to stage worlds onto the new stack one at a time.

### 3.3 The dual-backend **oracle** harness (legacy as the verifier)  — 🟢 physics half DONE (commit 997c47d9); render half exists via wgpu_sensor_regression.py
This is the user's explicit "use legacy to debug/verify the new architecture" requirement, made a
first-class tool. Two modes:

- **Physics oracle** — ✅ BUILT: `scripts/dev/physics_oracle.py` runs the same world twice (ODE via
  `OMNISIM_LEGACY=1` vs Newton via the migration default) through the `oracle_dumper` supervisor
  (`tests/physics/controllers/oracle_dumper/`) on `tests/physics/worlds/oracle_drop.omniworld`, then reports
  per-body max |Δpos| + first-divergence step. Verified: free-fall matches, contact diverges at the
  exact landing step (the static FLOOR shows 0.000000 m as a sanity check). Extend later with joint
  angles + more worlds.
- **Render oracle** — render the same Viewpoint under WREN and wgpu and image-diff them. This is the
  golden-image gate (§3.4) generalized: WREN is the golden, wgpu is the candidate, the existing
  parity decomposition (brightness vs hue, masked by the pick pass — already built in
  `OmWgpuView`'s self-check / `grabWindowBufferNow`) is the metric. **Valid only on a static scene
  (`--mode=pause`)** — motion decorrelates the two backends' frame timing (a known trap).

The oracle is both the **debugging tool** (where does Newton diverge from ODE on *this* robot? where
does wgpu differ from WREN on *this* material?) and the **regression gate** (§3.4). Build it once,
use it for both.

### 3.4 The automated gates
- **Physics determinism gate** — exists: `tests/smoke/run_smoke.py` + `smoke_worlds.json` (empty,
  accelerometer, contact_points, template_deterministic), asserted two-run byte-identical with
  `OMNISIM_FORCE_ODE=1` (the CPU reference baseline — *reference*, not *deprecated*:
  `omnisim-ode` is the best-scoring integration in
  [OmniBench](../benchmarks/omnibench-2026-07-24.md) lane 1 — an integration result, not a
  solver one ([correctness-scope.md](../benchmarks/correctness-scope.md)) — and lane 3a
  grades it bitwise cold-cold **and** cold-warm on both machines). ⚠️ **This gate is
  ODE-only by design and cannot be generalised to Newton** — lane 3a's Newton grade is the
  XPBD solver on one light-contact world, and the GPU `mujoco_warp` path is *not* run-to-run
  reproducible at all (0 bitwise of 24 pairs;
  [determinism-scope.md](../benchmarks/determinism-scope.md)). A two-run byte-identical
  assertion is only a valid gate on ODE or on `newtonSolver "mujoco"`.
- **Newton pass/fail gate** — ✅ BUILT (commit fdb10589, **Option C: physical invariants**). The design
  question here was real: you can't assert Newton == ODE (different solvers legitimately differ), and a
  Newton trajectory golden isn't portable (Warp/GPU output varies by hardware). Resolved by asserting
  invariants any sane rigid-body result must satisfy — `physics_oracle.py --gate`: INV1 finite+bounded
  (no NaN/explosion), INV2 settles by end, INV3 reproducible run-to-run (<1cm). Portable, exit 1 on
  violation. Verified both ways: PASS on the settled drop world (INV3 max-diff 0 m on *that*
  world — ⚠️ the original note here read "⇒ Newton is bit-deterministic on this box" and that
  inference is **struck**: a settled single drop world is the easiest possible case, and on
  contact-rich scenes the GPU `mujoco_warp` solver diverges up to 9.152 m run-to-run
  ([determinism-scope.md](../benchmarks/determinism-scope.md)). INV3's <1 cm tolerance is the
  real contract; the 0 m observation is not a determinism result); FAIL (exit 1) when bodies
  are still mid-fall, so it's not a rubber stamp. Extend later with more worlds (per-arm
  completeness, Stages 1–2).
- **Render golden-image gate** — does **not** exist as an automated harness yet (only the one-shot
  `OMNISIM_VIEW3D_WGPU_SELFCHECK` PNG dump). **Build it** on the §3.3 render oracle: a fixed set of
  worlds, render WREN-golden vs wgpu-candidate in `--mode=pause`, fail if the parity metric regresses
  past a threshold. This is the gate that protects "wgpu renders the scene faithfully."
- **Combined-build smoke** — 🟢 **first validation done (2026-06-06):** a single
  `OMNISIM_WITH_NEWTON=ON OMNISIM_WITH_VULKAN=ON` binary runs Newton physics (the physics oracle above)
  AND the wgpu sensor pipeline (`wgpu_sensor_regression.py`: every camera/depth/CSM/rangefinder/lidar
  case passes on the same binary) — the two new backends coexist and function together. **Caveat
  found:** back-to-back headless launches are flaky — the first omnisim-bin launch intermittently loses
  the controller-port race and yields "no result" (the failing *set* shifts run-to-run, confirming
  flake not regression). The oracle now retries; the sensor smoke already retries. **A CI-grade gate
  needs the launch race fixed at the source** (unique controller port per launch) — tracked in §3.5.

### 3.5 CI
**Status (2026-06-06): local pre-push IS the physics gate** (cloud CI dropped). Newton is a GPU solver
driven by embedded CPython on top of an MSYS2/Qt build — none of which exist on GitHub-hosted runners —
and this project's pushes come from the GPU dev box, so a hosted-runner workflow couldn't run the gate
anyway. The gate therefore lives in the git **pre-push hook** (`.githooks/pre-push`, wired via
`core.hooksPath`): every non-scratch `git push` runs `physics_oracle.py --gate --require-newton` — which
asserts the physical invariants AND that Newton was *actually* active (not a silent ODE fallback) — plus
the legacy smoke. Bypass: `OMNISIM_SKIP_PUSH_CHECK=1 git push`.
**Launch race largely fixed:** the oracle now launches with `--no-rendering` (a physics gate reads body
positions, never pixels), which removed the renderer cold-start race that caused the intermittent
"no/short data"; a retry backstop covers the residual. Verified reliable 3×-back-to-back. (A self-hosted
GPU-runner cloud CI can be added later if external contributors ever need it.)

Originally scoped (the full target set, per push to `main`):
1. Pure legacy build + determinism smoke (the byte-identical baseline).
2. `NEWTON=ON` build + physics determinism (forced-ODE) + Newton-golden diff.
3. `VULKAN=ON` build + render golden-image gate.
4. Combined `NEWTON=ON VULKAN=ON` build + boot smoke.

Windows runner is mandatory (the only surface backend today is Win32-HWND). Mac/Linux runners get
added when those surfaces land (§5.3).

**Reliable CI — the launch flake, root-caused (2026-06-06).** Headless runs intermittently yielded
"no result". Investigation (in `scripts/dev/physics_oracle.py`) found the actual cause and fix:
- **NOT a port race.** Passing a unique `--port` per launch made it *worse* — every run failed —
  because `--port` rebinds only the extern/streaming server; a locally-spawned controller does **not**
  inherit it, so it then can't connect at all. Ruled out + reverted.
- **It's a Windows std-handle issue.** `subprocess.run(..., stdout=DEVNULL)` can hand the spawned
  controller invalid std handles → it fails to connect → no CSV. **Fix: redirect child output to a real
  `.log` file** (valid inheritable handles, like a `Start-Process` launch, which never flaked). This
  removed the *systematic* first-run failure in the oracle; a retry safety net covers residual flake.
- **Applied to both launchers** (oracle + `wgpu_sensor_regression.py`). It made the oracle (2 launches)
  reliable. The sensor regression (19 cases × retries = many rapid sequential launches) still shows a
  **residual, shifting** flake — a *different* ~3–4 cases fail each run, which proves it's flake not
  regression (no case fails consistently; every case passes on some run, so the combined build is
  genuinely functional). This residual is a **deeper harness race in rapid sequential local-controller
  startup**, beyond the std-handle surface fix. **Before a non-retry CI gate**, that race needs a real
  harness fix (serialize/settle controller startup, or make local-controller connection robust to
  contention) — tracked as the remaining §3.5 reliability item.

- **RESOLVED at the source (2026-07-05) — per-launch IPC pipe nonce.** The residual race was the
  Windows named-pipe name `webots-<tmpId>-<robot>` being salted **only** by the reused TCP port
  (`<tmpId>`, default 1234). Back-to-back launches that both grab port 1234 built an *identical*
  pipe name; because Windows allows multiple server instances of one name, a fresh child could
  `CreateFile(OPEN_EXISTING)` onto the **previous** launch's lingering pipe instance and cross the
  pairing → "no result". Fix: fold the **simulator PID** into the *intern* pipe name
  (`webots-<tmpId>-<pid>-<robot>`), passed to the child via `OMNISIM_IPC_NONCE` and appended on
  both sides (`OmController::start` + libController `robot.c compute_socket_filename`); extern names
  stay unsalted. Two back-to-back launches on the same port now derive **different** pipe names, so
  the cross-connection is impossible by construction. Verified: the nonce shows in the live pipe
  (`\\.\pipe\webots-1234-<pid>-oracle_dumper`) and **105/105** instrumented back-to-back launches
  connect first-try (was ~90% under the race). Two *other* flake sources were separated out in the
  same pass and are NOT this race: (a) a shared `omnisim_log.txt` on rapid relaunch — fixed by a
  per-launch `OMNISIM_LOG_PATH` in the retry launchers (AGENTS.md §3e); (b) a genuine intermittent
  multi-robot Newton cold-load crash ("Qt teardown race, exit 1") — still real, still absorbed by
  the retry loops, NOT claimed fixed.

---

## 4. Physics flip track (ODE → Newton)

The schema default is already `"auto"` (Phase D). The remaining work is **completeness** and
**making `auto` conservative**, then flipping the *build* default.

### 4.1 Make `"auto"` capability-gated (the core safety change)  — ✅ DONE + VERIFIED (d29fdd44, fixed 807419f8); BOTH tiers
**Status (2026-06-06):** `OmSolid::articulationNewtonCapable()` (root-resolved, recomputed per query,
consulted in `effectivePhysicsBackendName()`) ships and is verified on BOTH tiers:
- **Tier B (mesh):** verified — oracle_mesh.omniworld: mesh faller → ODE, box faller → Newton.
- **Tier A (joints):** verified — oracle_mixedjoint.omniworld (ROOT -hinge-> MID -ball-> TIP): the gate fires
  and the whole articulation stays on ODE (newton-steps=0), no solver-mix.
- **No false-trigger:** oracle_drop (all boxes) → Newton (newton-steps>0), no gate.
Three real bugs were found + fixed to make Tier A actually work (807419f8): (1) `OmBallJoint` inherits
`OmHingeJoint`, so `dynamic_cast<OmHingeJoint*>` wrongly accepted ball joints — now checks exact
`nodeType()`; (2) the walk must recurse joint ENDPOINTS (`solidEndPoint()`), not just `solidChildren()`,
to reach deeper joints; (3) the result must NOT be cached on first query (can fire pre-subtree-collection)
— recomputed each call (finalization-only, not per-frame). Bypass: `OMNISIM_AUTO_NO_CAPABILITY_GATE`.
Original design rationale below.

Today `auto` → Newton whenever Newton is *available* (`resolve()` only checks `isAvailable()`). That
is unsafe as a default because Newton silently degrades several features (mesh collision → AABB box;
non-revolute joints, body force/torque writes → inherit `-1` and stay on ODE for *that op*, which
mixes solvers within one world). Change `auto` to mean **"Newton only if every feature this
world/Solid uses is Newton-supported, else ODE"**:

- Enumerate the Newton-unsupported features (from the research): mesh collision geometry; prismatic /
  ball / hinge2 / AMotor / LMotor joints; direct `addForceAtPos`/`addTorque` callers; connector
  constraint coupling.
- During world finalize, if a Solid's articulation touches any unsupported feature, its effective
  `auto` resolves to **ODE** (logged once). Explicit `physicsBackend "newton"` still forces Newton
  (power-user override, may degrade — their choice).

This makes the *already-flipped* schema default safe: unspecified worlds get Newton only where Newton
is faithful, ODE everywhere else, never a silent mix.

**Implementation spec (scoped 2026-06-06, ready to build — needs one policy call).** Investigation of
the codebase refined the design into TWO tiers, because the features differ in *kind*:
- **Tier A — correctness (gate unconditionally):** non-Hinge/non-Slider joints. The Newton joint
  registration only handles `OmHingeJoint`/`OmSliderJoint` (OmBasicJoint.cpp:181-182); a ball/hinge2
  joint's bodies still go to Newton but the joint is never registered, so the articulation is
  *mixed/broken*. Gating the whole articulation to ODE is a genuine correctness fix.
- **Tier B — fidelity (policy call):** mesh collision. It is NOT unsupported — the code already
  AABB-fits mesh boundingObjects for Newton (`computeBoundingObjectMeshAabb`, OmSolid.cpp:2390,
  "enough for ground contact and joint-actuator stability"). So gating mesh → ODE is choosing ODE's
  *exact* mesh collision over Newton's *coarse AABB* — a fidelity preference, not a correctness need.
  **THE POLICY CALL:** does `auto` accept Newton's AABB approximation (more worlds on Newton) or prefer
  exact ODE for mesh worlds (safer fidelity)? Conservative default = prefer ODE; aggressive = accept AABB.
- **Detection + caching:** correctness needs *articulation-level* analysis (walk from the articulation
  root over the subtree's joints + boundingObjects) to avoid the per-link mixing that caused the OmniQuad
  frozen-sensor bug — so compute the decision ONCE at finalize and cache it on the Solid, not per-call
  inside the hot `effectivePhysicsBackendName()`. Reuse the existing mesh walker; add a joint-type walk.
- **Verify:** a robot-with-ball-joint world (Tier A → must route to ODE, confirm via the oracle it
  doesn't fly apart) and a mesh-boundingObject world (Tier B → routes per the chosen policy).
This is implementable as soon as the Tier-B policy is chosen; not rushed onto the sensitive path blind.

### 4.2 Completeness milestones (each gated by the §3.3 physics oracle)

**Status (2026-06-06 — resolved for the "safe default" bar; commits 29e8b1d2 N3, 678a59f7 flip):**

> ⚠ **THIS BLOCK IS FROZEN AT 2026-06-06 AND THREE OF ITS FIVE ROWS WERE OVERTAKEN WITHIN DAYS.** Read the corrections inline below before acting on any of it.
> - **N1** — ball and hinge2 are **registered on Newton now** (W2.2 / W2.3); the Tier-A allow-list in [`OmSolid.cpp:2554`](../../src/omnisim/nodes/OmSolid.cpp#L2554) includes `WB_NODE_BALL_JOINT` and `WB_NODE_HINGE_2_JOINT`. Still unregistered: explicit fixed joint (W2.1), AMotor / LMotor (W2.4).
> - **N2** — mesh runs a **real narrow phase on Newton by default** (W1.3), not an AABB approximation; revert lever `OMNISIM_NEWTON_MESH_TO_ODE`.
> - **N4** — the Newton **`body_f` path IS implemented** (W3.1, verified: +20 N lifts a 1 kg Newton box 0.50 → 5.44 m). What remains unimplemented is *joint-space* force (`control.joint_f` is inert under XPBD).
- **N1 (slider/prismatic) — ✅ DONE.** `OmNewtonBackend::addJointPrismatic` registers sliders on Newton
  (verified: the cobot arm's friction grasp + gripper fingers). Ball / hinge2 / AMotor / LMotor remain unregistered
  and are **capability-gated to ODE** (§4.1 Tier A) — a documented limitation, not a silent failure.
- **N2 (mesh) — DOCUMENTED LIMITATION.** Newton AABB-approximates mesh boundingObjects; the capability
  gate (§4.1 Tier B) routes mesh articulations to ODE for exact contact. Real narrow-phase is future work;
  the gate keeps `auto` safe meanwhile.
- **N3 (stand without env babysitting) — ✅ DONE.** `OMNISIM_NEWTON_SUBSTEPS`/`STATICS` folded into
  per-world `WorldInfo.newtonSubsteps`/`newtonStatics` (defaults preserve exact behavior); the
  base-divergence guard is now default-ON (strict no-op for valid states, freezes on divergence instead of
  exposing a NaN). **Update 2026-06-23:** a class of apparent "biped collapse" on headless runs turned out
  to be the silent-ODE-fallback init bug (`6a459f84`, top-of-file banner) — under headless `stdout=DEVNULL`
  the warp import failed and the engine quietly ran ODE. With Newton now reliably engaging, confirm the
  real `[OmNewtonBackend] world finalised (solver=...)` line before attributing a fall to physics, and use
  `OMNISIM_REQUIRE_NEWTON=1` (`cfb11d06`) to make a silent downgrade impossible in standing CI.
- **N4 (body force/torque writes) — DOCUMENTED LIMITATION.** Supervisor `add_force`/`add_torque` already
  warn backend-aware on Newton bodies (no silent corruption); the Newton `body_f` path is not implemented
  (low value vs frame-convention risk). Force-driven worlds use `physicsBackend "ode"`.
- **N5 (RL transfer parity) — DOCUMENTED LIMITATION.** OmniQuad walks (model+residual); G1 deploy loses balance
  at t≈1.55s (largely inherent biped instability); tracked in `rl-current-state.md`. Not a flip blocker.

Original milestone descriptions (in rough priority for "safe default"):
- **N1 — non-revolute joints** (prismatic at least): widen `OmNewtonBackend` past the `-1` inherit so
  sliders run on Newton. Unblocks a large class of worlds the capability gate currently routes to ODE.
- **N2 — mesh collision** narrow-phase (or an explicit, faithful convex-hull path) so mesh worlds
  stop falling back to AABB boxes.
- **N3 — legged standing without env babysitting**: fold `OMNISIM_NEWTON_STATICS` / `SUBSTEPS`
  defaults into the solver config so G1/Atlas don't need manual env vars; close the G1 t≈1.55s fall.
- **N4 — body force/torque write path** for controllers that inject forces.
- **N5 — RL transfer parity** (the deploy-vs-sim gap; tracked in `rl-current-state.md`, partly
  inherent biped instability — may stay a known limitation rather than a blocker).

Each milestone: implement → oracle-diff vs ODE on the relevant world(s) → add that world to the
Newton-golden smoke. The capability gate (§4.1) shrinks as milestones land.

### 4.3 Build-default flip criterion (Layer A: `OMNISIM_WITH_NEWTON ?= ON`)

**✅ DONE 2026-06-06 (commit 678a59f7).** `OMNISIM_WITH_NEWTON ?= ON` in the Makefile. How each
criterion below was satisfied: §4.1 capability gate verified (mesh + non-Hinge/Slider → ODE); N1–N3
landed, N4/N5 documented as capability-gated/warned limitations (§4.2); **runtime shipping decision =
document, do NOT bundle** — vendoring Warp + its CUDA redistributables (~GB) was rejected; a default
build uses Newton when `pip install newton warp-lang` is present, else falls back to ODE (safe, logged).
Verified: no-flags build compiles+links+runs Newton (invariant gate PASS); ODE path intact under
`OMNISIM_FORCE_ODE`; pure-legacy `make OMNISIM_WITH_NEWTON=OFF` links (also fixed a pre-existing
`snapshotBodyTranslations` missing-stub that had silently broken that build). §3 harness is green except
the §3.5 CI-runner item (Stage F — GPU-runner needed for the Newton gates; the local gates pass).

Original criterion (flip only when **all** hold):
- §3 harness green (escape hatch, oracle, gates, combined build, CI).
- §4.1 capability gate in and verified (no silent solver-mixing).
- N1–N3 landed (the features common worlds actually use), N4/N5 either landed or explicitly
  documented as "explicit-opt-in only, capability-gated out of `auto`."
- Newton/Warp runtime **shipping story** solved (today it requires a user Python+Warp install; a
  default-ON build must bundle or vendor it, else `isAvailable()` is false on end-user machines and
  `auto` silently stays ODE — acceptable as a *fallback* but means the "default" isn't really Newton
  for them). Decide: bundle Warp, or accept "Newton default only where the runtime is present."

### 4.3.1 Newton runtime bundling (L6) — reverses the §4.3 "do NOT bundle" decision

> **Update 2026-06-23 — bundling is now DEFAULT in `make release`, not opt-in,
> and idempotent.** `make release` runs the bundle automatically via
> `BUNDLE_NEWTON ?= 1` (`577ff609`); opt out with `make release BUNDLE_NEWTON=0`.
> It is **idempotent** (`3de05aa3`): it skips if `$(TARGET_PATH)/newton-runtime`
> is already staged, so dev rebuilds through `build_omni.bat` neither re-copy the
> ~600 MB nor fail. Combined with the init-bug fix (`6a459f84`, top-of-file
> banner) and the `OMNISIM_REQUIRE_NEWTON` guard (`cfb11d06`), a stock release on
> a supported host now reliably *runs* Newton — the missing piece that made the
> Layer-A/B "default" real rather than nominal. A from-source clone or `make
> debug` without the runtime still falls back to ODE (by design). Full procedure
> + caveats: [newton-runtime-bundle.md](newton-runtime-bundle.md).
>
> **Update 2026-06-09 (lane L6, branch `lane-l6`).** §4.3 chose "document, do
> NOT bundle" on the premise that vendoring Warp + CUDA redistributables is
> "~GB". **That number was an overestimate and the decision is now reversed:**
> the runtime *is* being bundled, because the lane mandate
> ([migration-parallel-lanes.md](migration-parallel-lanes.md) L6) and the §8.1
> default-flip blocker both require "a stock install runs Newton with no manual
> pip/PATH steps."

**Measured footprint (real, on the dev box) — ~450 MB, not GB:** warp-lang 1.13.0
**314 MB** (warp vendors its own slim CUDA subset — there is no separate
multi-GB CUDA toolkit to ship), usd-core 26.5 (`pxr`) 48 MB, newton 1.2.0 33 MB,
mujoco_warp 3.8.0.3 9.5 MB (the `SolverMuJoCo` path the frictional pinch grasp
needs), plus
a CPython runtime. Bundle-able — the same order as the Qt/msys64 payload already
shipped.

**Mechanism — packaging-only, no engine/C++ change.** `OmNewtonBackend` brings
the interpreter up with a bare `Py_InitializeEx(0)` (no `Py_SetPythonHome`, no
`PyConfig`, no `sys.path` edits — verified in `OmNewtonBackend.cpp`), then
`import warp`/`import newton`. So *which* runtime is found is purely a function
of the `python3XX.dll` the process loads + that interpreter's `sys.path`. The
bundler stages a self-contained CPython beside `omnisim-bin.exe` and drops a
`python3XX._pth` next to the loaded DLL → CPython's isolated path config builds
`sys.path` strictly from that file (registry / `PYTHONHOME` / per-user site all
ignored), deterministically resolving the bundled `site-packages`. This is the
standard "Windows embeddable package" redistribution path; it stays entirely in
the L6-owned packaging layer (no L1 file touched).

**The version-match trap (handled).** The currently-bundled `omnisim-bin.exe`
imports `python312.dll`; a binary rebuilt under the current Makefile (PYTHON_HOME
→ Python314) imports `python314.dll`. A python-version mismatch is the #1 silent
break, so the bundler **autodetects** the version from the binary's PE import
table (pure-Python parse, no objdump dep) rather than hardcoding it.

**Delivered:**
- `scripts/packaging/bundle_newton_runtime.py` — stages CPython + `._pth` +
  `site-packages`; `vendor` mode (offline, ~450 MB) and `bootstrap` mode (slim,
  first-run install); `--verify` proves imports under a scrubbed env.
- `make bundle-newton-runtime` — the bundler target, also invoked **by default
  from `make release`** as of 2026-06-23 (`577ff609`, `BUNDLE_NEWTON ?= 1`,
  opt-out `=0`) and run **idempotently** (`3de05aa3` — skipped when
  `newton-runtime` is already staged, so `build_omni.bat` rebuilds neither
  re-copy ~600 MB nor fail). Run standalone any time for a one-off vendoring.
- `windows_distro.py` package-time guard — warns (or fails under
  `OMNISIM_REQUIRE_NEWTON_BUNDLE=1`) if the runtime is absent, so the installer
  can never *silently* ship an ODE-only "stock" build (principle #4). The
  recursive `msys64/` copy ships a staged bundle automatically.

**Verified on this box / pending the release box.** The redirect+import
mechanism is proven: under `-E -S` isolation (= the embedded `Py_InitializeEx(0)`
+ `._pth` condition) with only bundle paths on `sys.path`, warp 1.13.0 / newton
1.2.0 / mujoco_warp / pxr all import cleanly. wgpu is already bundled
(`wgpu_native.dll` is copied next to the binary by the Makefile and ships in the
recursive `msys64/` tree). **Still pending** (needs the release/build box, not
doable in this session): produce the actual vendored installer (run
`make bundle-newton-runtime` against a freshly-built binary) and confirm
`omnisim-bin.exe` brings Newton up from the bundle on a PATH-stripped / clean
box — the script's own `--verify` / `--verify-binary` is that gate.

**Release procedure:** [newton-runtime-bundle.md](newton-runtime-bundle.md).

### 4.4 Rollback
`OMNISIM_FORCE_ODE=1` (or `OMNISIM_LEGACY=1`) returns every world to ODE on a Newton-ON build,
byte-identical to legacy (already used by the smoke runner). Per-world: `defaultPhysicsBackend "ode"`
(§3.2). Per-node: `physicsBackend "ode"`.

---

## 5. Rendering flip track (WREN → wgpu)

Sensors (R5) and 3c-A are done and verified; the flip is blocked on the main-view un-gate, parity,
cross-platform, and the missing fallback lever (§3.1 supplies the lever).

### 5.1 Un-gate the main view (the headline blocker)

> 🟢 **RESOLVED (2026-06-06) — the entire "wgpu-native bug / Option C / Rust source patch"
> conclusion BELOW is RETRACTED.** The sustained-use VRAM OOM was **app-level**, not a wgpu-native
> per-submit leak. Root cause: `OmWgpuSceneRenderer::collectShapeDraws` keyed the wgpu texture
> cache on the `OmImageTexture*` **pointer**, but a scene PROTO instantiates a separate
> `OmImageTexture` node per use — so dozens of shapes sharing one texture FILE (one `Plaster.jpg`
> across many factory walls) had distinct pointers and the cache **re-uploaded the same file once
> per instance**. panda.wbt's ~63 unique files became **507 GPU uploads** (mostly 2048²/1024²,
> multi-GB) → VRAM OOM at ~30 s. **Fix (`a4fec74b`): key the cache on the source file path
> (`stableTexId`)** → shared files collapse to one upload; path-less textures keep the pointer.
> **Verified:** texture creations 507 → 63, zero wgpu errors, main view renders 200+ frames
> (`cdOk=1`) at 1960×1122, soaks 75 s+ (was a ~30 s crash). How it was found (harness `f2dfb949`,
> all env-gated): `OMNISIM_PROBE_SOAK` proved the bare submit+readback loop is leak-free (6000
> frames @1024², flat `wgpuGenerateReport` registry) → isolated the leak to the app, not the
> device; `OMNISIM_WGPU_MAINVIEW_FORCE` drove the real main view on panda.wbt; `OMNISIM_WGPU_TEXLOG`
> + a stack capture pinned the flood to `collectShapeDraws`. **No Rust / wgpu-native work was needed
> — every "Option A/C, surface-crashes-too, v29-is-latest, source-patch" claim below was chasing a
> misdiagnosis.** Remaining for the field flip: §5.2 parity, §5.3 cross-platform, §5.5 criteria,
> the OOM no longer blocks. The §3.1 fallback lever still applies. **Multi-world soak DONE
> (2026-06-07):** 6 varied worlds — panda / construction_site_dev / forest (heavy, 3 different PROTO
> families) + camera / lidar / range_finder (sparse sensor) — all survive 72 s+ with **0 wgpu errors**
> and the texture-creation count **plateaus** at each scene's unique-file count (63 / 87 / 29 / 6 / 6 / 6)
> then stays flat; `OmniSimSky` (procedural, the path-less fallback) never churns. The un-gate is
> de-risked → the remaining flip criteria are §5.2 parity, §5.3 cross-platform, and §5.5 sign-off.
>
> ⚠️ Historical (pre-fix) analysis retained below for the record; treat as superseded.

The 3c-B seam (`OmView3D::renderNow` → `renderMainFrameViaWgpu`, gated by `OMNISIM_WGPU_MAINVIEW`)
renders the main view through wgpu but crashes under sustained use: a per-frame **offscreen
readback** VRAM-OOM inside wgpu-native. Four app-level fixes were ruled out by measurement; the
**readback itself** is the culprit (the live surface pane, which presents without readback, never
crashes). Interop (zero-copy wgpu→GL, "Option B") was checked and is **infeasible on the pinned
wgpu-native** (no external-memory/shared-texture export API). So the realistic paths are:
- **Option A — window-swap: ❌ RULED OUT (2026-06-06).** The premise was "the surface path has no
  readback, so it won't crash." **Tested and FALSE:** running with `OMNISIM_VIEW3D_WGPU` (the live
  `OmWgpuView` pane, whose `renderTick`→`drawWorld(nullptr)`→`presentScene(rgbaOut=nullptr)` is pure
  surface-present, no readback) **crashed at 26 s with the same `0xC0000409`**. So sustained per-frame
  wgpu rendering OOMs in BOTH the offscreen-readback path AND the surface-present path — the
  accumulation is a shared, device-level wgpu-native per-submit leak, not the readback. A window-swap to
  the surface would crash identically, so it cannot un-gate. (Original window-swap scope notes kept below
  for the record, but the approach is dead.) **The crash also means sustained continuous wgpu rendering
  is broken in general** — the sensor pipeline only survives because each sensor read is a bounded burst,
  not a continuous loop.
  Former scope notes:
  `OmSimulationView` has **84** `OmView3D`/`mView3D` references and calls many `OmView3D`-specific
  methods/signals (`grabWindowBufferNow`, `showBlackRenderingOverlay`, `resetScreenshotRequest`,
  `contextMenuRequested`, `unleashAndClean`, `applicationActionsUpdateRequested`, …), while
  `OmWgpuView : QWindow` is NOT a `OmView3D` subclass (no shared interface). So Option A requires either
  giving `OmWgpuView` the full `OmView3D` interface (methods + signals + the editor interactions) or
  threading dual-path handling through all 84 sites — a multi-day focused effort. **Touches the shared
  main-view path → human sign-off + golden-image gate mandatory.**
- **Option C — wgpu-native upgrade/fork:** move to a wgpu-native that either fixes the readback
  lifecycle or exposes interop, then the committed offscreen-blit seam un-gates as-is (keeps full
  interaction immediately). Cost: dependency upgrade risk; revalidate the whole render stack.

**Decision resolved (2026-06-06): Option C ONLY.** Option A is ruled out (above — the surface path
crashes identically under sustained rendering). So the un-gate **requires a wgpu-native-level fix**: the
device leaks per-submit resources under sustained per-frame submit until VRAM OOMs (~26-34 s / ~2000
frames), regardless of readback-vs-present, not reclaimed by `wgpuDevicePoll`, not target-tied. This is
a bug in (or a usage constraint of) the pinned wgpu-native build. Path forward: upgrade/patch
wgpu-native (or find the missing per-frame reclaim call its API requires) on a box where that can be
iterated. Until then the seam stays gated — safe (WREN is the default; the gate degrades to WREN), and
the wgpu sensor pipeline stays fine (bounded bursts, not a continuous loop).

**Deeper root-cause confirmation (2026-06-06), so no one re-treads it.** A fresh code audit ruled out
the remaining app-level hypotheses with direct evidence — the OOM is **definitively wgpu-native-internal**,
not an app leak:
- **Texture IDs are stable** — `OmWgpuSceneRenderer::collectShapeDraws` keys the texture cache on the
  `OmImageTexture*` (`reinterpret_cast<uint64_t>(map)`), which is a stable scene-graph pointer across
  frames, so material textures cache-HIT (no per-frame creation). The earlier LRU cap addressed
  unbounded *growth*; this confirms there isn't even per-frame *churn* to grow.
- **Per-frame draw resources ARE released** — the per-draw bind groups and uniform buffers in
  `OmWgpuRenderTarget`'s scene/textured draw paths are `wgpuBindGroupRelease`/`wgpuBufferRelease`'d each
  frame (lines ~1028-1071, ~1260-1303); the render target color/depth are cached; `ensureScenePipeline`
  is guarded. So no app object accumulates.
⇒ The accumulation is wgpu-native's own deferred-free / submitted-work tracking under sustained
per-frame `submit`+`copyTextureToBuffer`+`mapAsync`, which a single post-readback `wgpuDevicePoll` did
NOT reclaim (tried, ruled out). Recreating the render target won't help (accumulation is device-level,
not target-level); recreating the device is too heavy. **So A or C remain the only paths — this is not
fixable by app-level resource hygiene (now exhaustively verified).**
**Attempt-1 (periodic target recreation) TESTED + RULED OUT (2026-06-06):** recreated the render target
every 500 frames (well under the ~2000-frame fault). The fault still hit at **33 s, unchanged** — proving
the accumulation is device-level, NOT tied to the target object (recreating it flushed nothing).

**Attempt-2 (Option A, surface path) TESTED + RULED OUT:** ran the live `OmWgpuView` surface pane
(`OMNISIM_VIEW3D_WGPU`, pure `presentScene`, no readback) — **crashed at 26 s, same `0xC0000409`.** So
both the offscreen-readback and the surface-present paths OOM under sustained rendering; the window-swap
cannot help.

**Attempt-3 (missing per-frame device poll) TESTED + RULED OUT:** the surface present path had NO
`wgpuDevicePoll`; added one after `wgpuSurfacePresent`, tried BOTH non-blocking (`wait=false` → still
crashed ~27 s) and blocking (`wait=true` → still crashed ~23 s). So the leak is NOT reclaimable by a
poll/maintain in any mode.

**Option C (upgrade) is NOT available (2026-06-06):** the pinned build is `v29.0.0.0` and the GitHub
releases API confirms **that IS the latest wgpu-native release** (next newest is v27.x). So the leak is
present in the LATEST binary release — "upgrade" can't fix it. The remaining paths are a wgpu-native
**source-level patch** (fork the Rust, locate the per-submit reclaim leak in wgpu-core, build from
source — a multi-day Rust effort), or a deeper usage fix beyond the 7 already-tested app approaches. Both
are substantial; the seam stays gated (WREN default = safe) until one is undertaken.
**Source-patch feasibility CHECKED (2026-06-06): no Rust toolchain on this machine** (`cargo`/`rustc`
absent; the v29.0.0.0 source tag IS reachable on GitHub). So the source patch additionally requires
installing a Rust toolchain (a system change) before the multi-day wgpu-core dive. ⇒ The un-gate is not
completable in this environment by any path: app fixes exhausted (7), Option A dead (surface crashes),
Option C upgrade unavailable (v29 is latest), source patch needs a toolchain install + deep Rust work.
It is fully scoped and handed off — pick it up on a box with a Rust toolchain and time for wgpu-core.

**DEFINITIVE CONCLUSION (un-gate):** sustained per-frame wgpu rendering leaks device VRAM
(`wgpuDeviceCreateTexture 'Not enough memory left'`) in wgpu-native itself — across offscreen AND surface
paths, not target-tied, not poll-reclaimable, with all app resources released and caches stable
(exhaustively verified over 7 ruled-out fixes). **This is a wgpu-native bug; the un-gate REQUIRES
Option C (upgrade/patch wgpu-native) — there is no app-level or architectural workaround on the pinned
build.** It also means sustained *continuous* wgpu rendering is broken in general (the sensor pipeline
only survives because each read is a bounded burst). The seam stays gated (safe: WREN default).

### 5.2 Lighting parity to target
Parity is at 75% (CSM + AgX tuned; residual is sun-shaft edge placement + lit-area brightness). Every
*parameter* lever is exhausted; closing the residual is shadow-placement work. Set the **gate
threshold** (e.g. parity ≥ 90% on the golden set) as the render flip criterion rather than chasing
100% — and make the threshold the §3.4 gate, so "good enough to default" is an automated decision,
not a vibe.

### 5.3 Cross-platform surfaces
Only Win32-HWND surface exists (`OmWgpuSurface`). Metal (macOS) and Vulkan-surface (Linux) are
unimplemented. Two ways to sequence:
- **Windows-first ship:** flip the default to wgpu on Windows only; Mac/Linux keep WREN via the
  resolve() fallback (already silent-safe) until their surfaces land. Honest and shippable.
- **All-platform:** implement Metal + Linux surfaces before any default flip. Cleaner story, much
  later. Recommend Windows-first, with the resolve() fallback making non-Windows automatically WREN.

### 5.4 R5 sensors — already done
Camera/depth/range-finder/lidar under wgpu are implemented and regression-verified against the WREN
oracle (`scripts/dev/wgpu_sensor_regression.py`, EXIT=0). This is a large chunk of "wgpu truly
replaces WREN" already banked; the flip does not wait on it. Keep it in the golden gate so it stays
green.

### 5.5 Field-default flip criterion (Layer B: Viewpoint/Camera `"wren"` → `"wgpu"`)
Flip the schema defaults (Viewpoint.wrl:17, Camera.wrl:41) and the `OmWrenWindow`/`OmView3D` dispatch
default only when **all** hold:
- §3 harness green (esp. `OMNISIM_FORCE_WREN` / `OMNISIM_LEGACY` and the golden-image gate).
- §5.1 main view un-gated (Option A or C) and stable under sustained use.
- §5.2 parity ≥ threshold on the golden set.
- §5.3 host has a working surface; otherwise resolve() falls back to WREN (so the flip is safe even
  on unsupported hosts — they transparently stay WREN).
- Human sign-off on a realistic world (panda.wbt is the standing parity world).

### 5.6 Rollback
`OMNISIM_FORCE_WREN=1` (built in §3.1) or `OMNISIM_LEGACY=1` returns the whole process to WREN.
Per-world: `defaultRenderBackend "wren"` (§3.2). Per-node: `renderBackend "wren"`. The seam already
degrades to WREN whenever wgpu is unavailable or unselected.

---

## 6. The flip itself, and how it stays reversible

The actual "switch" is deliberately small — the heavy lifting is §3–§5. When the gates are green:

1. **Build layer (A):** `OMNISIM_WITH_NEWTON ?= ON`, `OMNISIM_WITH_VULKAN ?= ON` in `Makefile`, and
   wire the runtime deps to ship (wgpu_native.dll already auto-copies; add the Newton/Warp bundle or
   document the runtime requirement). A pure-legacy build remains one flag away:
   `make OMNISIM_WITH_NEWTON=OFF OMNISIM_WITH_VULKAN=OFF`.
2. **Schema layer (B):** physics is already `auto`; flip Viewpoint/Camera `renderBackend` to `wgpu`.
3. **Runtime layer (C):** unchanged — `resolve()` already does the available-else-legacy fallback and
   (after §3.1) honors the global legacy levers.
4. **Completeness layer (D):** the capability gate (§4.1) + the parity/un-gate criteria ensure D is
   actually ready before B/A flip.

**Reversibility is structural, not a rollback procedure:** because legacy is never removed and the
resolve() fallback + global levers are permanent, "undo the default" is `OMNISIM_LEGACY=1` for a
user, or reverting the two-line Makefile/schema defaults for a build — no data migration, no
re-render, no re-sim.

---

## 7. Sequencing (ordered; each gate must pass before the next default flip)

**Stage 0 — Safety harness (§3).** No default changes. Deliver: `OMNISIM_FORCE_WREN` +
`OMNISIM_LEGACY`; WorldInfo default fields; the dual-backend oracle (physics + render); the
Newton-golden smoke + the render golden-image gate; the combined-build smoke; CI running all of it.
*This is the prerequisite for everything and the most valuable standalone deliverable — it's also the
debug/verify tooling the user asked for.*
**Progress:** §3.1 escape hatch ✅ (459a2a21); §3.2 WorldInfo default fields ✅ (baebd7c7 physics +
6592b4b9 render, both verified); §3.3 physics oracle ✅ (997c47d9), render oracle already exists
(`wgpu_sensor_regression.py`); §3.4 combined NEWTON+VULKAN build ✅ validated; §3.5 launch flake
root-caused + first-launch fixed in both launchers; §3.4 gates ✅ — physics invariant gate built
(fdb10589, Option C) + verified both ways, render golden-pixel gate already exists
(`wgpu_sensor_regression.py`). **Remaining in Stage 0:** §3.5 only — the deeper rapid-launch race + actual
CI wiring (the gates exist; CI needs them green-or-red without leaning on retries). Then Stage 0 is
complete and the per-arm completeness work (Stages 1–6) can begin.

**Stage 1 — Physics capability gate (§4.1). ✅ DONE.** `auto` is conservative: faithful where Newton is
faithful (hinge/slider + primitive collision), ODE elsewhere (mesh, ball/hinge2/AMotor/LMotor), never
mixed within an articulation.

**Stage 2 — Physics completeness (§4.2). ✅ DONE to the "safe default" bar.** N1 (slider) + N3 (stand
without env babysitting: WorldInfo `newtonSubsteps`/`newtonStatics` + default-on base guard) landed;
N2 (mesh narrow-phase), the remaining joint types, N4 (force writes), and N5 (RL transfer) are
documented as capability-gated / warned known limitations rather than blockers. The gate shrinks as
those land later.

**Stage 3 — Physics build-default flip (§4.3). ✅ DONE 2026-06-06 (678a59f7).**
`OMNISIM_WITH_NEWTON ?= ON`. Runtime-shipping decision: document the `newton`/`warp` install
requirement, don't bundle. Legacy one lever away (`OMNISIM_WITH_NEWTON=OFF` / `OMNISIM_LEGACY=1`).
**The physics arm is complete through the build-default flip.** Remaining cross-cutting item: §3.5 CI
(Stage F), which needs a self-hosted GPU runner for the Newton gates.

**Stage 4 — Render un-gate (§5.1).** Decide A vs C; land it; main view stable under sustained use.
Human sign-off. This is the render headline.

**Stage 5 — Render parity + surfaces (§5.2, §5.3).** Parity ≥ threshold on the golden set;
Windows-first surface decision. Non-Windows stays WREN via fallback.

**Stage 6 — Render default flip (§5.5).** Viewpoint/Camera `"wren"` → `"wgpu"` +
`OMNISIM_WITH_VULKAN ?= ON`. Legacy one lever away.

**Stage 7 — Combined default + soak.** Both defaults on; run the full oracle/golden/determinism suite
on the combined build across the world corpus; soak the main view for the readback-crash duration ×
margin. Document the supported matrix and the legacy levers in user-facing docs.

Stages 1–3 (physics) and 4–6 (render) are **independent** and can proceed in parallel after Stage 0;
they only rejoin at Stage 7.

---

## 8. Risk register

| Risk | Arm | Mitigation |
|---|---|---|
| Silent solver-mixing within one world (Newton body + ODE joint) | Physics | §4.1 capability gate resolves the *whole articulation* to one backend; oracle catches drift |
| Newton runtime absent on end-user machines → "default" silently ODE | Physics | §4.3 shipping decision (bundle Warp) or document honestly; fallback is safe but not "Newton" |
| Main-view readback crash reaches users | Render | Stays gated until §5.1; default is WREN; flip blocked on un-gate + soak |
| wgpu parity regressions ship unnoticed | Render | §3.4 golden-image gate in CI; WREN is the oracle |
| Non-Windows hosts have no wgpu surface | Render | resolve() fallback → WREN automatically; Windows-first flip (§5.3) |
| Combined Newton+wgpu build never validated together | Both | §3.4 combined-build smoke before any flip |
| A flip is hard to undo under pressure | Both | §6 structural reversibility: `OMNISIM_LEGACY=1`; two-line build revert |
| RL policies trained on ODE deploy worse on Newton | Physics | Documented limitation (`rl-current-state.md`); `physicsBackend "ode"` per-robot; not a default-flip blocker |
| Determinism worlds move onto Newton and "regress" | Physics | Already handled: smoke runs `OMNISIM_FORCE_ODE=1`; Newton gets its own golden (§3.4) |

---

## 9. Definition of done

> **Status 2026-06-23 — physics arm: the runtime default is now real, with a
> bounded honesty caveat.**
> **Now done (physics):** build flag ON (§4.3); schema `auto`→Newton (Phase D);
> capability gate (§4.1); the init bug that made headless runs silently fall back
> to ODE is FIXED (`6a459f84`) so Newton **reliably engages** where the runtime is
> present; `OMNISIM_REQUIRE_NEWTON` (`cfb11d06`) gives deploys/CI a fail-loud
> guard; and **`make release` bundles the runtime by default + idempotently**
> (`577ff609`/`3de05aa3`) so a stock release on a supported host *runs* Newton with
> no manual pip/PATH step.
> **Still remaining (physics):** (1) **wire `OMNISIM_REQUIRE_NEWTON=1` into the
> deploy path and the standing-CI gate** so a regression can't quietly downgrade;
> (2) the **fidelity tail** — ~35–40% corpus-faithful, 5/8 robot worlds, with
> `ur_arms`/`mavic`/Atlas gaps still open (tracked in
> [newton-ode-replacement-plan.md](newton-ode-replacement-plan.md) and
> `rl-current-state.md`); (3) **Phase-E "ODE as an *explicit* opt-out"** — ODE is
> still the silent fallback for no-runtime source clones, not yet a documented,
> intentional-only mode. ODE is **not** deprecated and never will be: it is the
> permanent fallback and the verification oracle.
> The render arm is unchanged by this update (still blocked on the §5.1 un-gate).

- A no-flags build on a supported (Windows + GPU + Newton-runtime) host runs **Newton + wgpu** by
  default, with ODE + WREN reachable via `OMNISIM_LEGACY=1` / `OMNISIM_FORCE_ODE` /
  `OMNISIM_FORCE_WREN` / per-world / per-node — all byte-faithful to the legacy stack.
- The **dual-backend oracle** can diff legacy vs new for any world (physics state + rendered image)
  on demand — the standing debug/verify tool.
- CI runs, on every push: legacy determinism baseline, Newton-golden smoke, render golden-image gate,
  and the combined-build boot smoke — all green.
- The capability gate guarantees no world is ever silently degraded: it gets the new backend only
  where the new backend is faithful, legacy otherwise, loudly logged.
- Legacy remains fully compiled, tested, and documented as the supported fallback and reference — not
  a deprecated path.

**One-line summary:** the *switch* is two lines of build/schema default once the *harness* (global
legacy lever, world-level overrides, dual-backend oracle, golden + determinism gates, CI, combined
build) and the *completeness gates* (Newton capability-gated `auto`; wgpu main-view un-gate + parity
+ surface) are green — and because legacy is never removed and the fallback is structural, the switch
is reversible by a single environment variable at any time.
