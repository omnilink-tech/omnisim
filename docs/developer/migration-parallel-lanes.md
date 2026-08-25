# Engine-migration parallel lanes — multi-session work split

**Purpose:** split the remaining engine-migration work (the §8 "architecturally
complete" tail, after the architectural-baseline milestone — documented in
[architectural-baseline.md](architectural-baseline.md), not git-tagged) into the maximum
number of lanes that **multiple Claude Code sessions can run at the same time
without colliding**.

**Operating model: all sessions commit directly to `main`.** That is the binding
constraint of this plan: two sessions editing the same file at the same time =
live conflict. So **every lane owns a strictly disjoint set of files, and never
edits another lane's files.** The lane count is therefore capped by a few
"spine" files that a whole arm funnels through (`OmNewtonBackend.cpp` on physics;
`OmWgpuShaders.cpp` / `OmWgpuRenderTarget.cpp` / `OmWgpuSceneRenderer.cpp` on
render) — those cannot be split further without per-session branches, which this
plan deliberately does not use.

**Working tree — shared by default, isolated worktree when a lane needs it.**
By default all sessions run in the one local checkout. Disjoint *files* keep
edits from clashing, but git's **index and HEAD are shared global state**, so two
mechanical rules matter: (a) commit with an atomic, path-scoped
`git commit -m "…" -- <your files>` — **never** `git add` then a bare `git commit`,
because a bare commit (or another session's `git add -A`) sweeps whatever is staged,
including another lane's files (observed 2026-06-09: an L6 commit swept L2's staged
files; a later `git reset HEAD~1` would have dropped any commit stacked on top); and
(b) `git pull --rebase` refuses while another lane has uncommitted edits — that is
expected, so push your own *committed* files anyway (push acts on commits, not the
working tree, so it ignores their unstaged changes) and never `stash`/`checkout`/
`reset` their files.

A lane **may** instead run in its **own `git worktree`** — `git worktree add
../omnisim-<lane> main` — when it needs isolation the shared tree can't give. The two
real triggers: **(1) a build/relink** (in the shared tree this would compile other
lanes' half-finished WIP and fail for reasons unrelated to your change), and **(2) any
global git op** (reset, bisect, interactive rebase) that would move HEAD/index under
the other sessions. A worktree has its own index + HEAD + working files but shares the
same `.git` object store and the `main` branch, so its commits and pushes land on the
same `main` as everyone else — no per-session branch, the plan's invariant holds. Cost:
a separate folder to manage and you lose the other lanes' in-flight edits from view.
Treat it as an opt-in escape hatch, not the default — most file-scoped doc/source edits
are fine in the shared tree with the two rules above.

**Canonical status** (read for context, do not edit during parallel work):
[engine-migration-plan.md §8.1 "Status refresh"](engine-migration-plan.md).
**What "done" means per arm** lives in the per-lane tracker docs named below.

---

## The lanes at a glance

| # | Lane | Owns (exclusive) | Tracker doc | Independent? |
|---|------|------------------|-------------|--------------|
| **L1** | Newton physics core | `physics/OmNewtonBackend.{cpp,hpp}`, `nodes/OmSolid.{cpp,hpp}`, `nodes/OmBasicJoint.{cpp,hpp}` | newton-ode-replacement-plan.md | ✅ fully |
| **L2** | wgpu render fidelity | `render/OmWgpuShaders.{cpp,hpp}`, `render/OmWgpuRenderTarget.{cpp,hpp}`, `nodes/OmWgpuSceneRenderer.{cpp,hpp}` | rendering-arm-checklist.md | ⚠️ interface-seam with L3 |
| **L3** | wgpu viewport + surface | `gui/OmView3D.{cpp,hpp}` (wgpu path only), `gui/OmWgpuView.{cpp,hpp}`, `gui/OmSimulationView.{cpp,hpp}`, `render/OmWgpuSurface.{cpp,hpp}` | r4-completion-checklist.md, r4-step3c-plan.md | ⚠️ interface-seam with L2 |
| **L4** | Damage / contact consumers | `projects/default/controllers/harness_supervisor/damage_tracker.py` + the other contact-API Python consumers | physics-contact-impulse-api.md | ✅ (soft dep on L1) |
| **L5** | RL / legged deploy | `projects/policies/**` (trainers, deploy controllers) | rl-current-state.md, humanoid-balance-gap.md | ✅ (boundary with L1) |
| **L6** | Packaging / runtime bundling | `Makefile`, `scripts/packaging/**` | default-flip-plan.md (build layer) | ✅ fully |
| **L7** | Verification / CI | `scripts/dev/*oracle*.py`, `faithful_check.py`, `newton_coverage.py`, `.githooks/pre-push`, `.github/workflows*`, `docs/developer/wgpu-golden/` | ci-and-fast-feedback.md | ✅ (golden-refresh seam) |
| **L8** | Granular / CUDA *(optional, research)* | NEW files only (`GranularGroup` PROTO, CUDA solver TUs) | engine-migration-plan.md §13.7 | ⚠️ small L1 integration points |

**Not live lanes** (do not start as parallel work):
- **Cross-platform surfaces (Metal/Linux)** — **hardware-blocked**: no macOS/Linux box here. When hardware exists it owns *new* per-platform files (`WbWgpuSurfaceMetal.mm`, …) and L3 freezes the dispatch seam. Until then, deferred.
- **The architectural lever (R3.7 GPU interop)** — Newton GPU body buffer read directly by the wgpu vertex shader. Touches L1's `OmNewtonBackend` *and* L2's `OmWgpuRenderTarget`, so it **cannot** be disjoint. It runs **last, as a convergence step**, after L1 + L2 reach their checkpoints — not in parallel.

So: **L1–L7 run concurrently today** (L8 optional). That is the realistic maximum for "all on main."

---

## Coordination protocol (read before starting any lane)

These rules are what make "all on main" safe. Breaking #1 or #2 causes lost work.

1. **Edit only your lane's owned files.** If you believe you need to touch another
   lane's file, STOP and leave a note in your tracker doc + ping the user — do not
   edit it. (The one structured exception is the interface seams in #4.)

2. **Frozen — nobody edits these during parallel work:**
   - `physics/OmPhysicsBackend.{cpp,hpp}` — dispatcher surface, signed off as final
     ([dispatcher-surface-signoff.md](dispatcher-surface-signoff.md)). No new virtuals.
   - `physics/OmOdeBackend.*`, `render/OmWrenBackend.*` — legacy, must stay byte-identical (migration non-negotiables #1/#2).
   - `render/OmRenderBackend.*`, `render/OmVulkanBackend.*`, the render adapters/caches (`WbWgpuMesh*`, `WbWgpuTexture*`, `OmWgpuImageAdapter`, `OmWgpuGlBlit`) — stable plumbing.
   - The WREN code path inside `gui/OmView3D.cpp` (L3 edits the **wgpu** region only; the WREN render path stays byte-identical).
   - `engine-migration-plan.md` (§8.1 canonical status) — see #5.

3. **Commit small, often, path-scoped, and atomic.** Keep each commit file-scoped to
   your lane and push promptly — a lane sitting on a large uncommitted diff is the
   thing most likely to eat a conflict. In the **shared tree** commit with
   `git commit -m "…" -- <your files>` (atomic, path-limited); do **not** `git add`
   then a bare `git commit` — that staging window lets a bare commit (yours or another
   session's `git add -A`) sweep up other lanes' files. `git pull --rebase` is blocked
   while another lane has uncommitted edits — that is expected: push your *committed*
   files anyway (push ignores their unstaged changes); never `stash`/`checkout`/`reset`
   theirs. If you need a **build/relink or a global git op** (which would entangle other
   lanes' WIP), use an **isolated `git worktree`** instead — see the operating-model
   "Working tree" note above. Belt-and-braces: before a risky window, snapshot your
   in-flight work with `git diff HEAD -- <files> > ../omnisim-<lane>.patch`.

4. **The two interface seams** (the only sanctioned cross-lane coupling):
   - **L2 ↔ L3:** L3 *calls* `OmWgpuRenderTarget` + `OmWgpuSceneRenderer` public
     methods that L2 owns. **Freeze those public signatures** (`.hpp`). L2 changes
     internals + adds shaders/pipelines and self-verifies on the **offscreen golden
     path** (no GUI). If a lane needs a *new* public signature, it's a coordination
     event: agree it, L2 lands it first, L3 pulls, then proceeds.
   - **L1 ↔ L2/L3:** `OmWgpuSceneRenderer` (L2) reads `OmSolid`'s (L1) public
     geometry/pose/appearance accessors. **L1 must not change that public API.**
     (These are stable Webots accessors; unlikely to move.)

5. **Status docs:** during parallel work, each lane updates **only its own tracker
   doc** (column above). **Do not edit `engine-migration-plan.md` §8.1** — a single
   integrator (or the user's main session) reconciles the canonical snapshot
   periodically from the per-lane trackers. This prevents N lanes colliding on one file.

6. **Build flags:** only **L6** edits `Makefile`. A `Makefile` change forces a full
   rebuild for everyone — L6 announces it (tracker note) before landing.

7. **The pre-push gate runs on every non-scratch push** (`.githooks/pre-push`:
   Newton physics gate + render gate on render-touching pushes + smoke). If your
   lane's change legitimately shifts a gate expectation (e.g. L1 changes physics
   divergence, or L2/L3 change the rendered image), coordinate the golden/tolerance
   refresh with **L7** — do not blindly `--update-golden` past a real regression.

8. **Each lane self-verifies headlessly** with the command in its section, so a
   session never needs another lane's output to know it's green.

---

## L1 · Newton physics core

**Owns:** `src/omnisim/physics/OmNewtonBackend.{cpp,hpp}` (incl. the embedded
Python runtime source), `src/omnisim/nodes/OmSolid.{cpp,hpp}`,
`src/omnisim/nodes/OmBasicJoint.{cpp,hpp}`.
**Tracker:** [newton-ode-replacement-plan.md](newton-ode-replacement-plan.md) (W-series).

**Scope — the Newton capability climb (the single biggest physics spine):**
- **PAL write surface** — implement the write-side dispatcher virtuals Newton
  currently inherits as `-1` (set body pose/vel, apply force/torque already exist
  via the index API; extend coverage), brakes, connectors, joint-feedback.
- **Joint families (W2):** explicit fixed joint, ball (3-DoF), AMotor/LMotor,
  motorised Hinge2.
- **Control ops (W3):** joint-space force via the body-torque fallback
  (`control.joint_f` is dead under XPBD — see W3.1), joint-param sets (FMax / stops
  / CFM-ERP divergence doc), body damping / auto-disable long tail.
- **Shapes (W1.4):** heightfield (terrain worlds).
- **Contacts (W4):** real penetration depth + contact-impulse magnitude under the
  default XPBD solver (today `forceMag`/`depth` are 0 under XPBD; only MuJoCo
  populates force) → then **W4.3 drop the ODE keepalive** for pure-Newton worlds.
  ✅ **W4.3 is DONE — by deletion** (`bdc02139` removed the vendored ODE library outright), and
  the XPBD half is moot: `94f04222` removed XPBD, `7b431e81` made `SolverMuJoCo` the default and
  only solver, and native Newton contact readback is default-ON.
- **Solver robustness (W5.1/W5.2):** auto solver-select (XPBD vs MuJoCo) + substep auto-tune.
- **Engine-side legged dynamic fidelity** (the OmniQuad/Newton roll-instability root cause,
  if it lands in `OmNewtonBackend`) — see boundary with L5.

**Verify (headless, no GUI):** `python scripts/dev/physics_oracle.py --gate --require-newton`,
`scripts/dev/faithful_check.py`, `scripts/dev/newton_coverage.py` (the coverage meter is your headline number).
**Definition of done:** ⚠ **restated 2026-08-08** — the old DoD was "coverage meter ≥ ~99%
faithful on the corpus **with ODE as the documented residual**". `bdc02139` deleted ODE, so
**there is no residual arm to document**: anything Newton cannot do is not "handled by the other
backend", it is unsupported. The DoD is therefore **100% of the corpus faithful on Newton, or an
explicit, enumerated, checked-in list of worlds/features declared UNSUPPORTED** — with the
coverage meter reporting that list rather than absorbing it into a residual percentage.

---

## L2 · wgpu render fidelity (offscreen pipelines + shaders)

**Owns:** `src/omnisim/render/OmWgpuShaders.{cpp,hpp}`,
`src/omnisim/render/OmWgpuRenderTarget.{cpp,hpp}`,
`src/omnisim/nodes/OmWgpuSceneRenderer.{cpp,hpp}` (draw collection + material harvest).
**Tracker:** [rendering-arm-checklist.md](rendering-arm-checklist.md).

**Scope — the fidelity ladder + lighting parity (all verifiable offscreen):**
- **Lighting/shadow parity 75% → ~100%** — full **CSM cascades** (today single-cascade), ambient/fill model.
- **T1.4 TAA** (jitter + history + neighborhood clamp — 0% in-engine today).
- **T1.3 volumetric fog** on wgpu.
- **T2** — GPU-instanced draw at scale, `wgpu::QuerySet` timers, culling.
- **T3–T5** — decals, parallax-occlusion, SSR/GI, the high-end passes.

**Verify (headless):** `python scripts/dev/render_oracle.py --world <w> --golden <g>` +
the `OMNISIM_PROBE_*` self-tests (pick/readback/line/tex). **Self-verify on the
offscreen golden path — do not depend on L3's live viewport.**
**Seam:** keep `OmWgpuRenderTarget.hpp` + `OmWgpuSceneRenderer.hpp` public signatures
stable (L3 calls them). New signatures = coordinate with L3 (protocol #4).
**Definition of done:** every Tier-1 feature in-engine; parity gate ≥ threshold on the shipping-demo goldens.

---

## L3 · wgpu viewport + surface (on-screen + interaction)

**Owns:** `src/omnisim/gui/OmView3D.{cpp,hpp}` (**wgpu region only** —
`renderMainFrameViaWgpu` and below; the WREN path is frozen),
`src/omnisim/gui/OmWgpuView.{cpp,hpp}`, `src/omnisim/gui/OmSimulationView.{cpp,hpp}`,
`src/omnisim/render/OmWgpuSurface.{cpp,hpp}`.
**Tracker:** [r4-completion-checklist.md](r4-completion-checklist.md),
[r4-step3c-plan.md](r4-step3c-plan.md).

**Scope — make wgpu a first-class, then default, main viewport:**
- **Harden the on-screen surface** (`OmWgpuSurface`) — resize, multi-monitor, context-loss, robustness — so the wgpu pane can be *the* main view.
- **3c-C** — the remaining parity to replace WREN as THE viewport: video recorder / screenshots, status overlays, any remaining picking/manipulator polish on the surface path (the additive 3c-A picking/selection/handles already landed).
- **Main-view-default prep** — everything needed so the Phase-ζ `renderBackend "wren"→"wgpu"` flip is a one-line, reversible change (the flip itself is human-gated, not in this lane).

**Verify:** `OMNISIM_VIEW3D_WGPU_SELFCHECK=<file>` (numerical pixel stats, GUI-free) +
`OMNISIM_WGPU_MAINVIEW_DUMP` (screenshot). **Calls** L2's render-target/scene-renderer
APIs — does not edit them (protocol #4).
**Definition of done:** wgpu pane is feature-complete vs the WREN view; the default flip is a one-liner.

---

## L4 · Damage / contact consumers

**Owns:** `projects/default/controllers/harness_supervisor/damage_tracker.py` and the
other Python contact-API consumers (`event_bus.py`, the `battlebot_damage_director.py` files).
**Tracker:** [physics-contact-impulse-api.md](physics-contact-impulse-api.md).

**Scope:** migrate damage scoring off the synthetic `mass·|Δv|` proxy onto real
per-contact data — ⚠ **the ODE `cp.depth` wire is DELETED (`bdc02139`)**, so the **native
Newton contact source is the only source left** (and it is **default-ON** since 2026-08-07;
`OMNISIM_NEWTON_NATIVE_CONTACTS=0` reverts) — and retire
`OMNISIM_DAMAGE_VEL_SMOOTH`. Re-derive the per-part threshold tables (penetration
metres vs the Joule proxy). Do it opt-in first (`OMNISIM_DAMAGE_USE_DEPTH`).

**Dependency:** soft on **L1** — the depth + native-contact APIs already exist, so
this can start now; richer contact-*impulse* under XPBD arrives from L1 later and
upgrades the magnitude. Pure-Python; **no engine files touched** → fully disjoint.
**Verify:** head-on damage capture. ⚠ **"ODE vs Newton event-count parity" is unrunnable**
(`bdc02139` — no ODE arm). Note `husky_head_on_ode.wbt` does **not** fail loudly: the explicit
`"ode"` pin still wins and resolves to an inert stub, so a capture against it still succeeds and
still writes a `.jsonl` of a world where nothing moved. This lane needs a new
verification step: a **dated, checked-in Newton reference capture** and a Newton-vs-Newton
regression check against it. See [p6-captures/README.md](p6-captures/README.md) for the
preserved historical ODE numbers and why they cannot serve as the target.

---

## L5 · RL / legged deploy (research)

**Owns:** `projects/policies/**` (trainers, envs, deploy controllers, specs).
**Tracker:** [rl-current-state.md](rl-current-state.md) (canonical RL status),
[humanoid-balance-gap.md](humanoid-balance-gap.md).

**Scope:** build the **train-in-the-deploy-solver** trainer (foundation exists in
`projects/policies/research/training/build_g1_native.py`; the trainer itself is unwritten) — the
documented path to G1 standing past t≈1.55 s; Atlas; the controller side of the
OmniQuad/Newton deploy.

**Boundary with L1 (important):** the *engine-side* dynamic-fidelity fix (e.g. the
OmniQuad roll-instability if it turns out to be a `OmNewtonBackend` control-bridge bug)
belongs to **L1**. L5 owns the **RL + controller** side (`projects/policies/`). If your
investigation points at an engine fix, hand it to L1 via the tracker — don't edit
`OmNewtonBackend`. Pure-Python/controller work → disjoint from the engine.
**Verify:** deploy survival time on the deploy worlds.

---

## L6 · Packaging / runtime bundling

**Owns:** `Makefile`, `scripts/packaging/**`, build/runtime setup docs.
**Tracker:** [default-flip-plan.md](default-flip-plan.md) (the build/runtime layer).

**Scope:** ✅ **both halves of this sentence are stale (2026-08-08).** The Newton runtime
**is** bundled now — a stock `make release` ships it (`BUNDLE_NEWTON ?= 1`, opt out
`BUNDLE_NEWTON=0`) — and there is **no ODE to fall back to** (`bdc02139`). ⚠ But the silence did
not go away: an *installed-but-broken* runtime FATALs, while an **absent** one still degrades
quietly, now onto an inert stub that simulates **nothing**. So L6's clean-box goal is now
load-bearing for correctness, not just convenience, and any verification of it must set
`OMNISIM_REQUIRE_NEWTON=1`. What remains in scope: **ship `wgpu_native`** in the
release build matrix across platforms. This
is what turns "compiled in" into "actually runs the new backend on a clean box."

**Verify:** clean-box install runs Newton physics + has wgpu available. Owns
`Makefile` exclusively (protocol #6) — announce flag changes (they force full rebuilds).
**Definition of done:** a downloaded release runs Newton (where capable) with no manual pip/PATH steps.

---

## L7 · Verification / CI

**Owns:** `scripts/dev/{physics_oracle,render_oracle,dual_backend_oracle,faithful_check,newton_coverage}.py`,
`.githooks/pre-push`, `.github/workflows*`, `docs/developer/wgpu-golden/`.
**Tracker:** [ci-and-fast-feedback.md](ci-and-fast-feedback.md), [architectural-baseline.md](architectural-baseline.md) B-track.

**Scope:** golden-image parity coverage across all shipping demos (today: one panda
golden); standing CI (the self-hosted GPU-runner question for GitHub Actions, which
are still inert in `workflows.disabled/`); a perf-regression gate. This is the
durability layer that keeps every other lane's work from silently regressing.

**Seam:** when L1/L2/L3 land an intended behavior change, they coordinate the
golden/tolerance refresh **through L7** (protocol #7) rather than rewriting goldens themselves.
**Verify:** the gates fail on a deliberately-injected regression and pass clean.

---

## L8 · Granular / CUDA *(optional — research, lower priority)*

**Owns:** NEW files only — a `GranularGroup` PROTO, CUDA solver translation units.
**Scope:** granular Tier 1–3 (the 50k-pebble target), CUDA M2. Mostly green-field.
**Caveat:** has a few integration points with L1 (`OmSolid`/scene throttling for
`granularMode`) — keep those minimal and coordinate via the tracker; if it needs
real `OmNewtonBackend`/`OmSolid` edits, sequence behind L1 rather than racing it.
Start this only when there's a spare session and L1 is stable.

---

## After the lanes converge

- **Cross-platform surfaces (Metal/Linux):** unblock when macOS/Linux hardware is available; owns new per-platform files, L3 freezes the dispatch seam.
- **The architectural lever (R3.7):** once L1 exposes the live body-buffer snapshot and L2 has the instanced draw + CUDA/wgpu interop, wire Newton's GPU buffer straight into the wgpu vertex shader (zero CPU touch). One focused convergence session, not parallel.
- ~~**The two default flips**~~ → **one flip left.** `physicsBackend`→newton is **complete and
  irreversible**: `bdc02139` deleted the vendored ODE library, so there is nothing to flip back
  to. Remaining: `renderBackend`→wgpu once L2+L3+L7 hit parity — human-gated, done by the user /
  integrator, not inside a lane.

---

## Appendix — per-lane kickoff prompts (copy-paste, one per session)

> Each prompt assumes the session starts at repo root on `main`. Every prompt ends
> with the same coordination contract; only the lane scope differs.

**Shared contract appended to every lane (the session must obey):**
> You are working ONE lane of a multi-session, all-on-`main` split defined in
> `docs/developer/migration-parallel-lanes.md`. Read that doc + your lane section +
> the canonical status in `engine-migration-plan.md` §8.1 + your lane's tracker doc
> BEFORE editing. **Edit only your lane's owned files** (listed in the doc); never
> touch frozen files or another lane's files. Commit small, file-scoped, and often
> with an atomic `git commit -m "…" -- <your files>` (NEVER `git add` then a bare
> commit — it sweeps other lanes' staged files); push your committed files even when
> `git pull --rebase` is blocked by another lane's uncommitted edits (push ignores
> their unstaged changes), and never stash/checkout/reset theirs. If you must build or
> run a global git op, use your own `git worktree` (see the doc's "Working tree" note).
> Update ONLY your lane's tracker doc, never `engine-migration-plan.md`. Self-verify
> with your lane's headless command. If you think you need another lane's file, STOP
> and report instead.

- **L1 (Newton core):** "Take lane L1 (Newton physics core) from `migration-parallel-lanes.md`. Owned files: `physics/OmNewtonBackend.{cpp,hpp}`, `nodes/OmSolid.{cpp,hpp}`, `nodes/OmBasicJoint.{cpp,hpp}`. Work the W-series in `newton-ode-replacement-plan.md` (PAL writes, joints, control ops, contact impulse under XPBD, W4.3). Verify with `physics_oracle.py --gate --require-newton` + `newton_coverage.py`. [+ shared contract]"
- **L2 (Render fidelity):** "Take lane L2 (wgpu render fidelity). Owned files: `render/OmWgpuShaders.{cpp,hpp}`, `render/OmWgpuRenderTarget.{cpp,hpp}`, `nodes/OmWgpuSceneRenderer.{cpp,hpp}`. Work the fidelity ladder in `rendering-arm-checklist.md` (full CSM, TAA, fog, T2–T5, 75→100% parity). Keep public `.hpp` signatures stable for L3. Verify on the offscreen golden path (`render_oracle.py` + `OMNISIM_PROBE_*`). [+ shared contract]"
- **L3 (Render viewport):** "Take lane L3 (wgpu viewport + surface). Owned files: `gui/OmView3D.{cpp,hpp}` (wgpu region only), `gui/OmWgpuView.{cpp,hpp}`, `gui/OmSimulationView.{cpp,hpp}`, `render/OmWgpuSurface.{cpp,hpp}`. Work `r4-completion-checklist.md` / `r4-step3c-plan.md` (harden surface, 3c-C, default-flip prep). Call L2's render APIs, don't edit them. Verify with `OMNISIM_VIEW3D_WGPU_SELFCHECK`. [+ shared contract]"
- **L4 (Damage):** "Take lane L4 (damage/contact consumers). Owned files: `projects/default/controllers/harness_supervisor/damage_tracker.py` + the other Python contact-API consumers. Migrate damage scoring onto `cp.depth` + native Newton contacts, opt-in via `OMNISIM_DAMAGE_USE_DEPTH`; retire `OMNISIM_DAMAGE_VEL_SMOOTH`. Tracker: `physics-contact-impulse-api.md`. Verify with a head-on damage capture. [+ shared contract]"
- **L5 (RL legged):** "Take lane L5 (RL/legged deploy). Owned files: `projects/policies/**` only. Build the train-in-the-deploy-solver trainer (foundation in `build_g1_native.py`) toward G1 standing past 1.55 s; OmniQuad/Atlas controller side. If a fix needs `OmNewtonBackend`, hand it to L1 — don't edit the engine. Tracker: `rl-current-state.md`. [+ shared contract]"
- **L6 (Packaging):** "Take lane L6 (packaging/runtime). Owned files: `Makefile`, `scripts/packaging/**`. Bundle the warp/newton runtime + ship `wgpu_native` in the release matrix so a stock install runs Newton + has wgpu. Announce any `Makefile` flag change (forces full rebuilds). Tracker: `default-flip-plan.md` build layer. [+ shared contract]"
- **L7 (Verification/CI):** "Take lane L7 (verification/CI). Owned files: `scripts/dev/*oracle*.py`, `faithful_check.py`, `newton_coverage.py`, `.githooks/pre-push`, `.github/workflows*`, `docs/developer/wgpu-golden/`. Expand golden coverage across the shipping demos, stand up CI, add a perf-regression gate. Other lanes refresh goldens THROUGH you. Tracker: `ci-and-fast-feedback.md`. [+ shared contract]"
- **L8 (Granular/CUDA, optional):** "Take lane L8 (granular/CUDA, research). Create NEW files only (`GranularGroup` PROTO, CUDA solver TUs). Granular Tier 1–3 + CUDA M2 per `engine-migration-plan.md` §13.7. Keep `OmSolid`/`OmNewtonBackend` integration minimal and sequence it behind L1. [+ shared contract]"
