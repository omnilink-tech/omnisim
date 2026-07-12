# Newton robot_combat completion plan

**Status:** Complete (optional polish open) — 2026-07-09 · created 2026-06-20. The wheel-lock fix
(`newtonSolver "mujoco"` on every wheeled combat world) landed and was verified — see "Fix (landed)"
below; only the "Remaining polish (optional)" items stay open.
**Scope:** Finish-and-validate the robot_combat demos on the Newton physics backend.
**Owner doc:** this file. Canonical cross-references: [newton-ode-replacement-plan.md](newton-ode-replacement-plan.md),
[engine-migration-plan.md](engine-migration-plan.md) §13.3, [physics-contact-impulse-api.md](physics-contact-impulse-api.md),
[p6-captures/](p6-captures/), [guide/newton-physics-backend.md](../guide/newton-physics-backend.md).

---

## TL;DR — the premise was wrong

We did **not** revert robot_combat to ODE. The demos were migrated to Newton on **2026-06-11**
(commit `57860c58`, "feat(physics): migrate robot-combat demos to Newton") — 24 `physicsBackend` pins across
7 worlds (4 BattleBots + 3 ORC) flipped `ode` → `newton`, and `BattleBot.proto` defaults the field to `"newton"`.
The only surviving `ode` pin is `projects/robot_combat/worlds/tests/husky_head_on_ode.wbt`, kept **deliberately**
as the ODE-golden parity reference.

The Newton bugs previously believed open are **fixed in source**. This is therefore **not a migration job — it is a
finish-and-validate job**: the C++ is done; what is missing is per-world configuration, behavioral validation, and
confirmation that the prior "verified on Newton" sign-off was not a silent ODE fallback.

## What is already fixed (do not re-solve)

| Symptom | Status | Commit / mechanism |
|---|---|---|
| Child-rotation drop (wheels/weapon spawn ~90° off, then tumble) | **FIXED** | `98409b28` (2026-06-14) — `WbBasicJoint.cpp:377` bakes `R_child^T·R_parent` into the revolute `child_xform`; `WbNewtonBackend.cpp:1008` |
| Chassis-freeze (Newton 0 damage events vs ODE 149) | **FIXED** | `d56cbf58` (2026-05-29) — static furniture was wrongly registered as dynamic Newton bodies firing `resetJointsToDefaults()` per tick; `subtreeHasPhysics()` predicate |
| Cylinder wheel narrow-phase lock | **WORKED AROUND** | `dbbef07e` (2026-06-08, W1.2) — cylinder boundingObjects handed to Newton as same-radius capsules rotated −90° about X; revert lever `OMNISIM_NEWTON_CYLINDER_AS_SPHERE=1` |
| High closing-speed XPBD NaN | **MITIGATED** | `3ba5e079` — `WorldInfo.newtonSubsteps` / `OMNISIM_NEWTON_SUBSTEPS` (N≥4) + base-divergence freeze guard |
| Damage-event inflation (57,161 vs 149) | **MITIGATED** | `dadd50ae` — `OMNISIM_DAMAGE_VEL_SMOOTH` EMA low-pass → 58 vs 39 (within the 10× `damage_events_diff` tolerance) |

Stale prior notes corrected by this research: the Newton child-rotation-drop fix ("proposed, not implemented")
and the Newton wheel-lock issue ("UNFIXED", `drive_test.wbt`/`drive_demo` — neither exists in the tree).

## Open work, ranked

Ranking reflects the adversarial critique (the synthesis under-rated #1 and entirely missed #2).

1. **🔴 Confirm Newton is actually running.** `_scratch/traj_battlebox_duel.wbt_{newton,ode}.txt` are **byte-for-byte
   identical** across all 756 lines — genuinely different backends diverge. With the known "launching from MSYS bash
   makes the embedded interpreter miss `warp` → silent ODE fallback" trap (`WbPhysicsBackend.cpp:325`), the entire prior
   sign-off may have run on ODE. If so, none of the four "fixed" behaviors has ever been exercised on the Newton solver
   in combat. **Gates the validity of all other evidence.**
2. **🔴 Part-detachment under Newton is unvalidated.** The "win" mechanic detaches broken wheels/weapons via
   `exportString` / `importMFNodeFromString` / `parent.remove()` / `setVelocity()`. Unconfirmed that a Newton-registered
   Solid can be removed mid-sim and a re-imported free Solid re-registers as a dynamic body without freezing or NaN-ing
   the shared articulation — the exact dynamic-graph fragility behind the chassis-freeze bug.
3. **🟠 No combat world sets the Newton runtime flags** (`grep` confirms zero set `newtonRobotColliders` /
   `newtonStatics` / `newtonSubsteps`):
   - **Chassis = 1mm placeholder collider** → ramming produces no chassis contacts → the core mechanic deals **zero
     damage**. Every Robot wrapper (inlined BattleBots *and* URDF huskies — both are `WbRobot`, both get the placeholder)
     needs `newtonRobotColliders TRUE`. Geometrically safe for BattleBots (chassis box bottom clears the wheel-contact
     line by ~0.05m); **per-family wheel-load check required** for URDF huskies/ORC (the envelope box risks pinning the
     body and starving the wheels — `P3.10i/j`).
   - **Arena walls intangible** → bots drive out of the BattleBox. Needs `newtonStatics TRUE` on **walled** worlds only
     (the implicit z=0 ground plane holds the floor; vertical walls do not). ORC open-field worlds have no perimeter
     walls — do **not** enable statics there.
   - **1 substep** → high-speed ram NaN risk once colliders are live. Needs `newtonSubsteps 4`.
4. **🟠 Stale `_scratch/` output paths** in both damage directors, both brains, and world `customData`
   (repo is on `o:/`) → headless director runs write results to a missing drive → **no headless oracle**. Fix first.
5. **🟡 Damage knobs are env-only.** `OMNISIM_DAMAGE_VEL_SMOOTH` / `OMNISIM_DAMAGE_USE_DEPTH` have **no WorldInfo field**,
   and under XPBD `cp.depth`/`forceMag` are hardcoded 0, so depth-scoring is inert. Backend-symmetric damage is
   impossible under XPBD without a solver switch or the deferred native depth API; the knobs must be baked into the
   **launch scripts**, not the worlds.
6. **🟡 Director perf** — `O(F²×C²)` contact loop (`battlebot_damage_director.py:660`) will dominate `royal_rumble` /
   ORC melees (structural twin of the ~130× heavy-tracker collapse). Needs broad-phase pair prune + contact
   sub-sampling, applied to **both forked copies** (battlebots/ + orc/).
7. **⚪ Deferred (needs native rebuild):** clean contact-impulse/depth API (retire the velocity proxy); true cylinder
   narrow-phase fix; new WorldInfo fields for the damage knobs; patch the latent compound-collider regression
   (`WbSolid.cpp` `addNewtonPrimitive` still emits `addShapeSphere("was cylinder")`).

## Phased plan

Phases 0–5 require **zero native rebuilds** — `.wbt` flags, Python output-path/perf fixes, and validation.

### P0 — Prove Newton is real *(do before any edits)*
- From **PowerShell** (not bash), launch `battlebox_duel.wbt` with the 2026-06-18 `omnisim-bin.exe`.
- Assert: (a) `[WbNewtonBackend]` "world opened" + per-hinge registration + "registered N dynamic + M static bodies"
  census lines; (b) newton vs ode trajectories **diverge**; (c) a single bot's chassis translates several meters under
  sustained drive (not spin-in-place).
- **Exit:** Newton confirmed live + bots drive. If silently ODE, stop and re-scope.

### P1 — Fix the headless oracle
- Repoint director/brain/`customData` output `d:/` → `$OMNISIM_HOME/_scratch` (or relative `_scratch/`).
- Confirm a director writes a parseable scorecard headlessly.
- **Exit:** a headless match produces a readable result file.

### P2 — Validate the most-fragile dynamics
- Part-detachment smoke under Newton: detach a wheel mid-match via the director path → freed Solid re-registers as
  dynamic, parent keeps driving, no freeze/NaN.
- **Exit:** a match runs start-to-finish with ≥1 detachment, cleanly.

### P3 — Make collisions real (per-world flags)
- `newtonRobotColliders TRUE` on all combat worlds; `newtonStatics TRUE` on walled (battlebox) worlds only;
  `newtonSubsteps 4` everywhere. Per-family wheel-load check on URDF huskies.
- **Exit:** ramming produces chassis contacts + HP loss; bots stay in the arena; fast impacts stay finite.

### P4 — Damage parity + delivery
- Bake `OMNISIM_NEWTON_SUBSTEPS` + `OMNISIM_DAMAGE_VEL_SMOOTH` into `scripts/battlebox_tournament.sh` /
  `scripts/combat_run.sh`. Re-capture the ODE-golden baseline (`husky_head_on_ode.wbt`) **after** edits. Re-tune
  per-world HP/threshold `customData` if needed.
- **Exit:** Newton damage within 10× of the re-captured ODE golden.

### P5 — Perf for melees
- Broad-phase pair prune (skip fighter pairs beyond `contact_gate_m`) + contact sub-sampling in **both** director copies.
- **Exit:** `royal_rumble` / ORC run at watchable speed.

### P6 — Deferred native work
- Contact-impulse/depth API; true cylinder narrow-phase fix; WorldInfo damage-knob fields; compound-collider regression.

## Risks & unknowns

- **Biggest unknown:** whether the prior migration's "verified on Newton" evidence is real or a silent ODE fallback
  (P0 settles this).
- **Per-family collider risk:** the same `newtonRobotColliders TRUE` flag is safe for BattleBots but can drive-lock URDF
  huskies — validate wheel load separately per family.
- **Forked controllers:** `battlebot_damage_director.py` and `battlebot_brain.py` each exist twice (battlebots/ + orc/)
  and have diverged — every P1–P5 edit must touch both or refactor to a shared module.
- **No WorldInfo field for damage knobs** → demos are not env-var-free for external agents without P6 (or launch-script baking).

## Verification recipe (traps)

- Run from **PowerShell**, not MSYS bash (bash → interpreter misses `warp` → silent ODE fallback).
- `python -m omnisim run-headless <world> --duration N --cold` (the warmup reload deterministically crashes Newton → always `--cold`).
- Force-killing an instance leaves port 1234 in `TIME_WAIT` → next launch exits 2; let instances exit cleanly.
- Multi-robot Newton cold loads crash intermittently (Qt teardown race, exit 1) → retry.
- `omnisim-bin.exe` is windows-subsystem (no piped controller stderr) — verify via census/log lines and result files, not stdout.

## Key files

- Worlds: `projects/robot_combat/battlebots/worlds/*.wbt`, `projects/robot_combat/orc/worlds/*.wbt`,
  `projects/robot_combat/worlds/{demos,tests}/*.wbt`
- Protos: `projects/robot_combat/battlebots/protos/{BattleBot,BattleBox}.proto`
- Controllers: `projects/robot_combat/{battlebots,orc}/controllers/battlebot_damage_director/battlebot_damage_director.py`,
  `.../battlebot_brain/battlebot_brain.py`
- Engine: `src/omnisim/nodes/WbSolid.cpp` (collider decision 3105-3146, statics harvester 2984-3001),
  `src/omnisim/nodes/WbWorldInfo.cpp` (flag parsing 48-51),
  `src/omnisim/physics/WbNewtonBackend.cpp`, `src/omnisim/nodes/WbBasicJoint.cpp`
- Launch scripts: `scripts/battlebox_tournament.sh`, `scripts/combat_run.sh`
- Reference world (template for flags): `projects/samples/demos/worlds/showcase/warehouse_husky.wbt`
  (sets `newtonRobotColliders TRUE` + `newtonStatics TRUE` + `newtonSubsteps 6`)

---

## Execution outcomes (2026-06-20)

All five phases executed and committed to `main`. Headless verification ran from PowerShell via
three new harnesses: `scripts/dev/detach_smoke.py`, `scripts/dev/combat_smoke.py`,
`scripts/dev/combat_match.py`.

**P0 — Newton is real (commit a41531dc).** The prior "verified on Newton" sign-off was a *silent
ODE fallback*: `_scratch/traj_battlebox_duel.wbt_{newton,ode}.txt` were byte-identical. Re-run from
PowerShell, Newton diverges from ODE (0.14 m early / 1.19 m full on the articulated bot; the static
arena stays identical) and bots translate (red +0.74 m over a 1 s window). So the four "fixed"
blockers had never actually been exercised on the Newton solver in combat until now.

**P1 — headless oracle (a41531dc).** Match scorecards / traces / brain logs hardcoded a stale
`_scratch` drive. Added a repo-root-anchored resolver to both director forks, both
brains, and `duel_tracker`. A unit test caught a real bug pre-commit: the fallback must NOT use
`WEBOTS_HOME` (it points at the *system* Webots install `C:\Program Files\Webots` on dev machines)
— it derives the repo root from `__file__`.

**P2 — part-detachment (a41531dc).** The "win" mechanic (export → import → `parent.remove`) works
under Newton: the freed wheel re-registers as a dynamic body and falls under gravity (z 0.147 →
0.025 m, not frozen), the parent bot keeps driving, nothing NaNs. Added a default-off
`OMNISIM_DAMAGE_FORCE_DETACH` validation hook (both forks).

**P3 — collisions real (commit 51322652).** Per-world flags baked in (the 2026-06-11 migration
predated `newtonRobotColliders`). Confirmed at `WbSolid.cpp:3105-3146` that *every* Robot wrapper —
inlined `Robot{}` included — otherwise gets a 1 mm placeholder chassis collider. Per-world config:

| World(s) | colliders | statics | substeps | note |
|---|---|---|---|---|
| battlebox_duel, royal_rumble, gravedigger_vs_biteforce, huge_vs_bitebot, hydra_vs_gravedigger | yes | yes | 4 | walled BattleBox, inlined bots, chassis clears wheels ~0.05 m |
| orc_open_field | yes | yes | 4 | box flanking walls |
| orc_forest_war | yes | no | 4 | UnevenTerrain mesh — statics risks cost/instability |
| orc_queen_defense | yes | no | 4 | soft-fence/OOTA, no physical walls |
| newton_husky_combat_2 | yes | yes | 4 | walled RectangleArena; A/B confirms husky.urdf chassis does NOT pin the wheels |

`combat_smoke.py` validated every world: finite, bots move (no wheel pinning), contained.

**P4 — damage parity + delivery (commit pending).** Added `OMNISIM_DAMAGE_TIMER_S` to bound matches
for headless testing (both forks). **Key finding:** in a 60 s match the bots **never engage** —
minimum chassis gap was 1.55 m, just outside the 1.3 m damage gate → a draw with zero damage (a
full **240 s** Newton match is *also* a scoreless draw — both bots keep all 4 wheels + weapon). An
ODE A/B is **identical** (draw, no damage), so the weak engagement is a *pre-existing
battlebot_brain behavior* (`SEEK→CHARGE` needs tight yaw alignment within `charge_range_m`), **not**
a Newton regression. Consequences:
- `OMNISIM_DAMAGE_VEL_SMOOTH` is consumed only by the harness `damage_tracker.py`, not the
  battlebot director — it applies to the husky/harness worlds, not the BattleBots/ORC duels.
- The bash launch scripts (`battlebox_tournament.sh`, `combat_run.sh`) silently run ODE from bash
  (warp unavailable); deprecation notes added pointing to `combat_match.py` as the Newton-correct
  runner. The scorecard chain itself is validated end-to-end (lands in `_scratch`, parseable).

**P5 — melee perf (commit pending).** Added a broad-phase prune to the directors' `O(F²×C²)`
bot-on-bot contact cross-reference (both forks): each bot's contact list includes its own
wheel-ground contacts, so far-apart pairs are skipped (`chassis gap > contact_gate_m + 2 m`). Most
impactful in the 20–40 m ORC arenas; conservative by construction (two ≤0.8 m bots can't share a
10 cm contact match beyond ~1 m, pruned at ~3.3 m).

### Open follow-ups (outside "migrate to Newton" scope)
- **Bot engagement** is the biggest demo-quality lever: bots wander and rarely clash under *both*
  backends. Tuning `battlebot_brain` (`charge_range_m`, `align_tight`, aggression) would make the
  duels actually fight — a controller task, pre-existing, backend-agnostic.
- **Damage-knob delivery:** `OMNISIM_DAMAGE_VEL_SMOOTH`/`USE_DEPTH` still have no WorldInfo field
  (native, deferred); the battlebot director also has no vel-smoothing of its own proxy.
- **Native depth/impulse API** for exact (vs within-10×) damage parity — deferred native work.

---

## Wheel-motion fix: MuJoCo solver required (2026-06-20, follow-up)

When the demos were watched in the GUI, the bot **wheels "flapped like a bird"** and the bots barely
moved. Investigation (a deep research workflow + extensive pure-Python bisection via
`scripts/dev/wheel_model_diff.py` against the runtime mirror `projects/policies/research/backends/g1_deploy_runtime.py`)
found **two separate problems**:

1. **Flap = wheel collider orientation.** Each BattleBot wheel Solid carries `rotation 1 0 0 1.5708`
   on its **body frame** with a bare `boundingObject Cylinder`. Newton's cylinder→capsule code
   (`WbNewtonBackend.cpp add_shape_cylinder`) rotates the capsule −90° about X assuming a body-Y
   cylinder, so combined with the body's +90° the collision capsule ends up **vertical** while the
   wheel spins about the lateral axle. The husky way (identity wheel body, rotation in a `Pose`
   around the geometry + boundingObject) avoids it.
2. **Lock = XPBD cannot drive a multi-wheel rover.** Newton's default **XPBD** solver locks
   **laterally-paired (left/right) wheels** — proven in pure Python: a 1-wheel rover drives 6 m, a
   lateral wheel pair locks (~0.2 m), a diagonal pair drives, 4 wheels lock. Friction, contact
   stiffness, substeps, effort, the chassis collider, and breaking exact coaxiality all fail to
   unlock it. **MuJoCo and ODE drive every case.** This hits *every* 4-wheel robot under XPBD.

**Refuted along the way** (each chased and ruled out by direct testing): an XPBD swing-twist
singularity at 180°; an omnisim embedded-runtime bug; wrong motor gains. A notable trap: reading a
continuous wheel's `joint_q` via `eval_ik` **wraps to [−π, π]**, so a fast-spinning wheel reads back
near zero and looks locked — chassis displacement is the only honest measure.

**Fix (landed):** `newtonSolver "mujoco"` on every wheeled combat world (`battlebox_*`, `*_vs_*`,
`orc_*`, `newton_husky_combat_2`). Verified: `drive_test.wbt` rolls 11 m straight; `battlebox_duel`
runs a full match (damage layer functional); `royal_rumble` (5 bots) and `orc_open_field` (4 bots,
6–7 m drive) all roll. MuJoCo on CPU is ~0.2–0.5× realtime for 4–5 bots — slower than XPBD but fully
usable for the demos. The lock is a **Newton-solver limitation, not an omnisim code bug**, so an
engine code change cannot fix it — MuJoCo is the right path (and is already how OmniSim handles
cases XPBD structurally can't, e.g. pinch grasps).

**Regression + tooling:** `projects/robot_combat/worlds/tests/drive_test.wbt` (two rovers: current
"flapper" authoring vs husky "roller") + `projects/robot_combat/controllers/drive_demo`, run via
`scripts/dev/drive_test.py {newton|ode}`. Pure-Python repros: `scripts/dev/wheel_lock_repro.py`,
`scripts/dev/wheel_model_diff.py`.

### Remaining polish (optional)
- **Husky wheel-authoring** for the battlebot wheels (move the `rotation` into a `Pose`): under
  MuJoCo the current wheels drive but with a mild ~5° wobble / ~5% lateral drift from the vertical
  capsule; the husky-authored "roller" drives arrow-straight (tilt 1°, 0 drift). Cosmetic — the
  lock/flap is already fixed. ~50 wheel edits across worlds + `BattleBot.proto`.
- **Bot engagement** (unchanged): with the wheels now driving, the bots *can* move but the brain
  still doesn't reliably close in to ram (a draw over a 60 s MuJoCo match) — confirmed pre-existing
  and backend-agnostic. Tuning `battlebot_brain` is the lever.
