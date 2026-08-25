# OmniSim Architectural Baseline — checklist & final plan

> ## ⚠️ 2026-08-08 — TWO OF THIS MILESTONE'S FIVE DEFINING CLAUSES NO LONGER HOLD
>
> `bdc02139` deleted the ODE backend (`src/ode/` + `include/ode/`, 106,283 lines).
> **The milestone below was genuinely reached on 2026-06-07 and is not being retracted** —
> but §0's definition of "architecturally complete" was written around a *symmetric
> two-backend* architecture with legacy as a permanent fallback and oracle, and the
> physics half of that architecture has since been deliberately collapsed to one
> implementation. Read §0 clauses 1, 2 and 4 as historical for physics:
>
> - **Clause 1 ("both backends are first-class and dispatched")** — the *seam* survives and
>   still matters: `OmPhysicsBackend` remains the single backend-agnostic surface, and it is
>   what made the deletion a bounded change at all. But it now has **one** implementation.
>   ⚠ Note the cost: `OmOdeBackend` overrode all 45 non-destructor virtuals while Newton
>   declares ~11, so the ~35 slots Newton inherits are now **permanently unsupported ops**
>   rather than a backlog against a reference implementation. See
>   [dispatcher-surface-signoff.md](dispatcher-surface-signoff.md), whose completeness
>   argument rested entirely on ODE covering the surface.
> - **Clause 2 ("legacy is a permanent, one-switch fallback + oracle")** — **RETIRED for
>   physics, by decision.** ODE is not compiled, not reachable, and `OMNISIM_LEGACY=1` no
>   longer reverts physics. This was a written promise; the retirement campaign broke it on
>   purpose (see the §7 amendment in
>   [engine-migration-plan.md](engine-migration-plan.md)). The render half — WREN stays
>   compiled and `OMNISIM_FORCE_WREN` works — still holds.
> - **Clause 4 ("a standing dual-backend oracle … can prove the legacy path is
>   byte-identical")** — **no physics arm exists.** `physics_oracle.py`'s ODE-vs-Newton
>   trajectory diff and `dual_backend_oracle.py`'s "one command runs a world both ways"
>   cannot run for physics, and the pre-push gate's forced-ODE determinism smoke is gone.
>   Every reversibility claim below (`OMNISIM_LEGACY=1` → byte-identical ODE+WREN; "default
>   diverges from forced-ODE by 0.275 m"; `reversibility_check.py`'s config matrix) is a
>   **historical verification that can no longer be re-run**, not a live gate.
>
> Clauses 3 and 5 are unaffected. Also stale below: `make OMNISIM_WITH_NEWTON=OFF` is no
> longer a "pure-ODE" build (it yields no physics at all), the cross-backend contact bridge
> is gone, and the capability gate no longer has a backend to route to — a world Newton
> models badly is simply modelled badly.
>
> Record: [ode-retirement-campaign.md](ode-retirement-campaign.md).

**Status:** ✅ **COMPLETE — every §5 acceptance box checked (2026-06-07)**. ⚠️ **This milestone is documented here but was NEVER captured as a git tag** — `architectural-baseline-v1` does not exist in `git tag -l` (the only baseline tag is `v0.1.0-baseline`). Other docs that cite a `architectural-baseline-v1` tag are referring to this milestone, not a real tag; either create the tag or stop citing it.
The new architecture (Newton physics + wgpu render, both dispatched, reversible, gated, and surface-signed-off)
is structurally done: A1/A2 (render correctness) + B1–B4 (oracle + golden + determinism + render gates, in the
pre-push hook) + C1 (wgpu default ON) + C2 (both surfaces final) + the `OMNISIM_LEGACY=1` reversibility proof.
All further work is incremental fill-in *within* this structure — the Phase-ζ tail (wgpu lighting parity,
cross-platform surfaces, the runtime default flip) and the Newton capability climb — not deep architectural change.
**Purpose:** define the **architectural baseline** — the point at which OmniSim's new
Newton + wgpu architecture is structurally complete, correct, and verifiable, so that
*all further work is incremental fill-in within the structure, not deep architectural
change.* This is a **distinct, earlier milestone** than the §8 / Phase-ζ "definition of
done" in [engine-migration-plan.md](engine-migration-plan.md): ζ also requires
capability completeness, fidelity parity, default flips, and cross-platform hardware —
those build *on top of* this baseline and are explicitly **out of scope** here.

> **Why a separate milestone?** "Newton runs every world" and "wgpu matches WREN
> pixel-for-pixel" are open-ended, partly hardware-gated, and partly research. The
> *architecture* that hosts them is not. Conflating the two makes the project look
> permanently unfinished. Separating them gives a concrete, reachable "the foundation
> is done" line — the thing you actually build a two-decade simulator on.

---

## 0. The definition (what "architecturally complete" means here)

OmniSim is **architecturally complete** when **all** of the following hold:

1. **Both backends are first-class and dispatched.** Every physics body op and every
   render/sensor op flows through a single backend-agnostic surface (`OmPhysicsBackend`,
   `OmRenderBackend`); selecting a backend is a per-node field + a runtime resolve, never
   a per-call-site branch.
2. **Legacy is a permanent, one-switch fallback + oracle.** ODE and WREN stay compiled,
   reachable, and byte-identical when selected; one lever (`OMNISIM_LEGACY=1`) returns the
   whole process to legacy.
3. **The new stack produces *correct* output for a representative world.** Newton
   faithfully simulates at least the wheeled/Hinge-Slider articulation class; wgpu renders
   a representative scene with **all geometry present** and the scene's **real lights +
   cast shadows** (not hardcoded stubs). *Correctness*, not *parity* — see §3.
4. **The architecture is verifiable.** A standing dual-backend oracle + golden gates can,
   on demand and in CI, prove the legacy path is byte-identical and detect regressions in
   the new path.
5. **No remaining item requires a deep architectural change.** Everything left is adding a
   Newton virtual override, a wgpu shader/feature, a platform surface impl, or flipping a
   default — all *within* the existing structure.

It is **NOT** required that Newton run every world, that wgpu match WREN's pixels, that the
defaults be flipped, or that macOS/Linux surfaces exist. Those are §4 (out of scope).

---

## 1. Inventory — what is ALREADY structurally done ✅

The deep architecture is largely in place. This is the honest "how far along" picture
(post the 2026-06-06/07 work; supersedes the stale 2026-05-30 §8.1 audit).

### Physics arm (ODE → Newton)
- [x] **`OmPhysicsBackend` dispatcher** — symmetric PAL, ODE canonical / Newton opt-in
      ([engine-migration-plan.md §13.2](engine-migration-plan.md)).
- [x] **Build flag default ON** — `OMNISIM_WITH_NEWTON ?= ON` (`src/omnisim/Makefile`,
      `678a59f7`); pure-ODE is one flag (`make OMNISIM_WITH_NEWTON=OFF`).
- [x] **Schema default flipped** — `physicsBackend "auto"` (Robot.wrl, Solid.wrl).
- [x] **Runtime resolve + fallback** — Newton-when-available-and-capable else ODE;
      `OMNISIM_FORCE_ODE` + `OMNISIM_LEGACY` levers (`459a2a21`).
- [x] **Capability gate** — mesh + non-Hinge/Slider articulations route to ODE, loudly.
- [x] **Cross-backend contact bridge** — ODE static colliders ↔ Newton dynamic bodies.
- [x] **Base-divergence guard** — default-ON; freezes on NaN/explosion instead of garbage.
- [x] **Per-world knobs** — `WorldInfo.newtonSubsteps/newtonStatics/newtonSolver`.
- [x] **Reversibility proven** — `OMNISIM_FORCE_ODE` run differs from default (measured
      this session: Newton 4.7–5.2 ms/step vs ODE 7.2–7.7 ms/step on a 1→16 robot scene).

### Render arm (WREN → wgpu)
- [x] **`OmRenderBackend` dispatcher** — symmetric to physics, WREN canonical / wgpu opt-in.
- [x] **Runtime resolve + fallback** — `OMNISIM_FORCE_WREN` + `OMNISIM_LEGACY` (`459a2a21`).
- [x] **Main-view dispatch seam** — `OmView3D::renderNow → renderMainFrameViaWgpu`
      (`5ef0e329`); WREN default byte-identical.
- [x] **On-screen present path** — offscreen wgpu render → GL blit (`render/OmWgpuGlBlit.cpp`);
      keeps the editor GL surface, no HWND pixel-format conflict.
- [x] **Per-node opt-in un-gated** — `renderBackend "wgpu"` renders the main view live, no
      experimental flag (`d42c5554`).
- [x] **Sensor pipeline (R5)** — Camera/depth/RangeFinder/Lidar on wgpu, regression-green vs
      the WREN oracle.
- [x] **Material path** — albedo/roughness/metalness/normal + GGX + sRGB; **path-keyed
      texture cache** (`a4fec74b`) — the former "VRAM OOM" was this app-level key bug, fixed.
- [x] **Surface abstraction** — `OmWgpuSurface` (Win32-HWND); Metal/Linux are *impls within
      this abstraction*, not new structure.
- [x] **Runtime dep shipping** — `wgpu_native.dll` auto-shipped next to the binary; MinGW
      controller runtimes shipped to `lib/controller` (`1fcec588`).

### Cross-cutting
- [x] **Folder layout is OmniSim** — sources live under `src/omnisim/` (the `src/webots/`
      rename has landed).
- [x] **Physics oracle** — `scripts/dev/physics_oracle.py` (ODE vs Newton trajectory diff; gate +
      report modes; retry-hardened 2026-06-07 for the back-to-back launch race).
- [x] **Render oracle** — `scripts/dev/wgpu_sensor_regression.py` + the `wren-parity`
      self-check (wgpu vs `grabWindowBufferNow()`).
- [x] **Unified dual-backend oracle (B1)** — `scripts/dev/dual_backend_oracle.py`: one command runs a
      world both ways, reports per-arm divergence. Physics arm wired; render per-world arm = seam
      (`render_oracle.py`) for the render session.
- [x] **Headless verification harness** — `OMNISIM_PROBE_*` (pick/readback/line/tex/soak),
      `OMNISIM_WGPU_MAINVIEW_FORCE`, registry-report dump.

---

## 2. Remaining architectural items — THE CHECKLIST ⬜

These are the only things between today and the architectural baseline. Each is bounded and
slots into the existing structure. Grouped by track; ordered roughly by dependency.

> **🔀 Parallel-session ownership (added 2026-06-07).** Two sessions are working this checklist
> concurrently; keep this map current so they don't collide. Edit only your own track's boxes.
> - **🟦 Newton/physics session:** B1 (physics arm of the oracle), B3 (physics determinism gate),
>   B4 (physics gate cell + CI reconciliation), C2 (`OmPhysicsBackend` surface sign-off).
> - **🟥 Render session:** ~~A1 (missing-geometry bug)~~ ✅, ~~A2 (real sun + shadows)~~ ✅ (both
>   `14107285`), ~~B1 render arm (`render_oracle.py`)~~ ✅, ~~B2 (golden-image gate)~~ ✅,
>   ~~C2 (`OmRenderBackend` surface sign-off)~~ ✅, ~~B4 render cell (render gate in the pre-push
>   hook)~~ ✅, ~~C1 (`OMNISIM_WITH_VULKAN` default flip)~~ ✅ (all 2026-06-07). **🟥 render side
>   COMPLETE — every render-track baseline box is checked.**
> - **Shared:** B1 is unified by design (one tool, two arms) and B4/C2 each have a physics half and a
>   render half — each session owns its half and they meet in the middle.

### A. Render correctness (the new render path must produce a correct frame)
- [x] **A1 — Missing-geometry bug FIXED (2026-06-07, `14107285`).** *The original
      "large Planes drop from high viewpoints" framing was a misdiagnosis* — the missing
      surface was panda.wbt's **floor, an IndexedFaceSet, not a Plane**, and the cause was
      neither culling, camera, nor frustum (all correctly ruled out, which is why those
      leads went nowhere). **Real root cause:** `renderMainFrameViaWgpu` collected draws
      **without making the WREN GL context current**. `collectWorldDraws → acquireFromWren →
      wr_static_mesh_read_data` falls back to `glGetBufferSubData` for *non-primitive*
      geometry (IndexedFaceSet, CadShape, …); with no current GL context that readback
      returns garbage, so complex meshes were cached with bad vertices on the **first frame**
      and stayed invisible forever (the mesh cache keys on the `WrStaticMesh*` and never
      re-reads). Primitives (Box/Plane/Sphere/Cylinder) use CPU builders (`buildUnitBox`,
      `buildUnitRectangle`, …) and were unaffected — hence "walls render, floor vanishes,"
      and hence a *fresh* cache collected after a prior render+blit worked while the
      persistent first-frame cache did not. **Fix:** one `OmWrenOpenGlContext::makeWrenCurrent()`
      before the collect. **Verified** the full scene renders through the production path on
      panda (floor+walls+tables+crates+panel), `high_resolution_indexedfaceset` (the marble
      monkey — the exact readback path), and rosbot (floor).
      - *Known separate gap (NOT A1, pre-existing):* `geometric_primitives.omniworld` collects 0
        draws and renders empty even with the fix (the make-current only *adds* a correct
        readback, it cannot remove draws). Likely an unhandled geometry/appearance class in
        `collectWorldDraws` for that specific world — a fill-in item, not an architectural one.
- [x] **A2 — Real scene sun + cast shadows, DONE (2026-06-07, `14107285`).**
      `renderMainFrameViaWgpu` now harvests the world's first `OmDirectionalLight` (e.g. the
      `OmniSimSun` PROTO), builds an ortho light frustum, and renders via
      `clearAndDrawSceneTexturedShadowed` (ambient dropped 0.35→0.05) so the live wgpu view is
      shadow-dominated like WREN instead of flat-lit; falls back to a fixed direction if no
      directional light is found. Verified visually on panda (soft cast shadows under the
      tables/panel). The A1 root-cause scaffolding (A/B flat dump, clip-space plane probe,
      tall-aspect render, camera log) was trimmed to a single gated `OMNISIM_WGPU_MAINVIEW_DUMP`
      screenshot hook. *Finer sun-shaft shadow placement parity remains §4 out-of-scope.*

### B. The verify / gate infrastructure (the architecture's standing safety net)
- [x] **B1 — Dual-backend oracle, one tool. ✅ DONE — 🟦 physics arm + unified front-end (2026-06-07);
      🟥 render per-world arm landed (`render_oracle.py`, 2026-06-07).** New front-end
      `scripts/dev/dual_backend_oracle.py` is the single command that runs a world both ways and
      reports per-arm divergence ([default-flip-plan.md §3.3](default-flip-plan.md)). It orchestrates
      the arm-specific oracles so each stays the source of truth for its own launch/retry/tolerance:
      - **Physics arm (done):** delegates to `physics_oracle.py` — ODE-vs-Newton per-body divergence
        table (report) or the invariant gate (`--gate`). Verified this session on `oracle_drop.omniworld`:
        FLOOR 0.000000 m (static sanity), FALLER1/2 diverge at contact (steps 98/116), FALLER3 ~0
        (free-fall matches pre-contact) — legacy verifying the new backend. `--gate` aggregates a
        combined per-arm pass/fail.
      - **Render arm (✅ landed 2026-06-07, `render_oracle.py`):** the front-end auto-detects
        `scripts/dev/render_oracle.py` and calls it (verified: `dual_backend_oracle.py --arms render`
        now prints `->render_oracle.py`, not the UNWIRED fallback). `render_oracle.py --world W` drives
        the in-binary `wren-parity` self-check (`OMNISIM_VIEW3D_WGPU`, `--mode=pause`), parsing the
        SHADOWED `within-tol` (vs WREN, the local golden — WREN runs right here) + geometry coverage;
        `--world W` exit 0 = within tolerance (the dual-oracle's contract). It does NOT pop a stored
        reference — WREN is the oracle — so it has nothing to bit-rot, and it also takes an optional
        `--golden G` (that's B2). Verified on panda (within-tol 76%, coverage 100% ⇒ PASS) and rosbot
        (own parity 85% ⇒ PASS), and that `--min-within-tol 90` FAILS (parity gate trips on divergence).
      **Acceptance:** met for both arms — one command runs a world both ways and reports/gates per-arm
      divergence; the render arm is wired and gate-verified.
- [x] **B2 — Golden-image gate on a real scene. ✅ DONE (2026-06-07).** `render_oracle.py --golden G`
      compares the wgpu render of a real scene (panda) against a stored golden
      (`docs/developer/wgpu-golden/panda_selfcheck.wgpu.png`). **[Note 2026-07-09: the panda golden
      was retired along with `panda.wbt` (OmniSim narrowed its arm coverage to a single cobot arm); the
      pre-push render gate now runs the coverage-only check on that arm world — re-adding a byte-golden
      awaits a re-bake (see the NOTE in `.githooks/pre-push`).]** The golden is the self-check's OFFSCREEN
      wgpu render of the paused scene, which is **byte-deterministic** (verified: two runs SHA-identical)
      — the right basis for a golden, where the realtime main-view dump is not (render-count gated +
      render-vs-step timing). Tolerance compare via Pillow (mean-diff/ch ≤ 5) else exact SHA-256.
      **Acceptance MET — a regression fails the gate**, verified two ways: (1) a different render vs the
      golden (rosbot's frame vs panda's golden) → `GATE FAIL: golden`; (2) parity below threshold
      (`--min-within-tol 90`) → `GATE FAIL: parity`. **Scope (honest):** the self-check exercises the
      shared wgpu pipeline (geometry collection, mesh/texture adapters, GGX/sRGB shaders, shadowed
      render), so this catches shared-pipeline + any pixel-level regression. It does NOT specifically
      cover `renderMainFrameViaWgpu`'s main-view *integration* (where the A1 GL-context bug lived — a
      caller-context issue, not shared-render code); a deterministic main-view golden is a follow-up
      (needs the realtime dump made step-deterministic). The golden is dev-box-GPU-specific (like the
      physics determinism gate); regenerate with `--update-golden` on a legit render change.
- [x] **B3 — Physics determinism gate, standing. 🟦 ✅ DONE + VERIFIED (2026-06-07).** The gate is
      standing in the **pre-push hook** (`.githooks/pre-push`, wired via `core.hooksPath`), which runs
      both halves on every non-scratch push:
      (a) **ODE byte-identical** — `tests/smoke/run_smoke.py` forces `OMNISIM_FORCE_ODE=1` and asserts
      the legacy determinism worlds; verified green this session (`tests/output.txt`:
      `OK: accelerometer`, `OK: contact_points`, `OK: template_deterministic`, 0 FAILUREs — a content
      check, not just exit 0);
      (b) **Newton within tolerance** — `physics_oracle.py --gate --require-newton` asserts INV1
      finite+bounded / INV2 settled / INV3 reproducible AND that Newton is *actually* active (default
      diverges from forced-ODE by 0.275 m); verified PASS this session (INV3 max-diff 0 m on the
      settled drop world). ⚠️ **The inference "⇒ Newton is bit-deterministic on this box" is
      struck** — same strike as
      [default-flip-plan.md](default-flip-plan.md), which corrected this identical sentence: a
      settled single-drop world is the easiest possible case, and on contact-rich scenes the GPU
      `mujoco_warp` solver diverges up to 9.152 m run-to-run. INV3's <1 cm tolerance is the real
      contract; the 0 m observation is **not** a determinism result. Per-solver scope:
      [determinism-scope.md](../benchmarks/determinism-scope.md).
      **Reliability hardening (this session):** the gate occasionally exhausted the oracle's 3 capture
      retries on the intermittent rapid-launch race (a single launch is 100 %-reliable; back-to-back
      flakes — the §3.5 item). Raised `CAPTURE_ATTEMPTS` 3→5 with a growing backoff in
      `scripts/dev/physics_oracle.py`; measured a run that flaked attempts 1-3 (would have failed under
      the old code) recover on attempt 4 → gate green. Deeper harness-IPC root-cause stays
      [default-flip-plan.md §3.5](default-flip-plan.md). **Acceptance met:** `accelerometer` /
      `contact_points` / `template_deterministic` + the oracle all run green in the gate.
- [~] **B4 — CI runs the gates. 🟦 physics cell reconciled (2026-06-07); 🟥 render cell pending.**
      **Correction:** `.github/workflows/physics-gates.yml` does **not** exist (claimed "authored" — it
      was never committed and isn't on disk). More importantly, hosted cloud CI for the *physics* gate
      was **deliberately dropped** ([default-flip-plan.md §3.5](default-flip-plan.md)): Newton is a GPU
      solver driven by embedded CPython on an MSYS2/Qt build — none of which exist on GitHub-hosted
      runners — and this project's pushes come from the GPU dev box, so a hosted workflow could not run
      the gate at all. **The standing physics CI is therefore the git pre-push hook**
      (`.githooks/pre-push`, wired via `core.hooksPath`), which runs B3 (`physics_oracle.py --gate
      --require-newton` + the forced-ODE determinism smoke) on every non-scratch push. That is the
      landed, working "gates run automatically" mechanism for the physics arm — **B4-physics is met by
      the pre-push hook, not by a YAML workflow.**
      - *Optional future work (not a baseline blocker):* a **self-hosted** Windows+GPU runner could
        host a cloud `physics-gates.yml` for external contributors who push without the dev box. If/when
        that runner exists, author the workflow to call the same two commands the pre-push hook runs
        (no new gate logic — just a hosting surface). Until then, deferring it is correct, not a gap.
      - 🟥 **Render cell (render session) ✅ wired into the pre-push hook (2026-06-07).** `.githooks/pre-push`
        now runs the render gate (`render_oracle.py`: panda WREN-vs-wgpu parity + the golden) right after
        the physics gate — but ONLY when the pushed range touches render code (`src/omnisim/render/`,
        `WbWgpu*`, `OmView3D`/`OmWgpuView`, `render_oracle.py`), since the wgpu self-check needs a brief
        GUI window and we don't pop one on every push. Lenient by design: a WREN-only binary or a launch
        flake reports UNAVAILABLE → exit 0 (non-blocking); only a real parity divergence / golden mismatch
        fails the push. Bypass: `OMNISIM_SKIP_RENDER_GATE=1` (render only) or `OMNISIM_SKIP_PUSH_CHECK=1`
        (all). Verified: hook `bash -n` clean, the render-file filter matches render paths / skips
        physics+docs, python 3.12 + Pillow resolve in the hook shell, and `render_oracle.py` itself is
        gate-verified (panda 76%/100% PASS; `--min-within-tol 90` + golden-mismatch both FAIL). *The
        cloud 2×2 build-matrix cell shares the physics arm's hosting question (a self-hosted GPU runner);
        the standing render gate is the pre-push hook, same as physics.*
      - ⚠️ **SUPERSEDED 2026-08-24 — the paragraph above describes a gate that stopped working.** The
        WREN-vs-wgpu parity arm it grades on died with WREN at D1.4 (`976b9449d`, 2026-08-23): the
        markers `render_oracle.py` polled for (`wren-parity-shadowed:`, `render-coverage:`) were deleted
        from the engine with it, so the poll could never succeed. The failure was silent AND expensive —
        the tool timed out all 6 attempts (**7m02s measured**) on every render-touching push, reported
        UNAVAILABLE, and exited 0, so the hook printed "Render gate passed" while asserting nothing for
        ~10 days. D1.5 (`1c4f1b413`) added a retirement comment to the file but never touched the polling
        logic. **Now:** the oracle grades the markers the current wgpu self-check actually emits — draw
        count, degenerate-transform scan, `id-coverage`, screenshot non-background pixels — which is the
        same "did geometry reach the renderer and get rasterized" property the old coverage arm asserted.
        `--min-within-tol` is accepted-but-ignored (it warns). **7m02s → 21s**, and red-capability is now
        provable with no engine, no GPU and no world: `python scripts/dev/render_oracle.py --self-test`.
      **Acceptance (physics):** B3 runs green automatically on every push via the pre-push hook
      (verified this session). Full B4 closes when the render cell + any chosen cloud-CI surface land.

### C. Architectural decisions to lock (not code — policy that fixes the structure)
- [x] **C1 — Render build-flag default flipped ON. ✅ DONE (2026-06-07).** `OMNISIM_WITH_VULKAN ?= ON`
      in `src/omnisim/Makefile` — symmetric with `OMNISIM_WITH_NEWTON ?= ON`; a no-flags `make` now
      compiles the wgpu backend in, so the new render architecture is "live by default." This does NOT
      change the runtime default (WREN stays canonical; only `renderBackend "wgpu"` worlds use the GPU
      path — the Phase-ζ runtime flip is separate). Safe because the wgpu code is DOUBLE-GUARDED
      (`OMNISIM_WITH_VULKAN` compiles the TUs; `WB_WGPU_NATIVE_AVAILABLE`, set only when
      `WGPU_NATIVE_HOME` finds a real lib, gates every `wgpu*` call). **Verified the fallback:** a build
      with the flipped default but NO `WGPU_NATIVE_HOME` recompiles all 5 wgpu TUs as stubs and links
      cleanly (BUILD=0, zero `undefined reference`/`wgpu_native`) — so a plain `make` on a box without
      wgpu-native still builds a WREN-only-equivalent binary; and the `WGPU_NATIVE_HOME` build still
      passes the render gate (panda 76%/100%, golden match). `OMNISIM_WITH_VULKAN=OFF` restores the
      pre-flip build.
- [~] **C2 — Confirm both dispatcher surfaces are FINAL.** Quick audit: no future op should
      require *adding* a virtual to `OmPhysicsBackend` / `OmRenderBackend` (Newton overriding
      more of the existing 42 is capability, not new surface). **Acceptance:** a one-page
      sign-off that the two surfaces are closed for extension by ordinary feature work.
      → **🟦 physics half ✅ SIGNED OFF (2026-06-07)** in
      [dispatcher-surface-signoff.md §1](dispatcher-surface-signoff.md): ODE overrides all 45
      non-destructor virtuals (reference backend ⇒ surface covers the engine's whole physics
      vocabulary), Newton declares 11 overrides + inherits ~35 as `-1` (the capability backlog =
      override existing slots, not add new ones); callsite fixpoint confirmed (5 residual direct ODE
      calls, none needs a new virtual); world-construction + collision/contact are deliberate
      off-surface boundaries; lone bounded exception = full-`dMass` `setBodyMass` (world-load only).
      → **🟥 render half ✅ SIGNED OFF (2026-06-07)** in
      [dispatcher-surface-signoff.md §2](dispatcher-surface-signoff.md): `OmRenderBackend` is a THIN
      SELECTION MARKER (3 identity virtuals `kind/name/isAvailable`, zero render-op virtuals) — WREN +
      wgpu each override all 3; wgpu adds 4 non-virtual concrete accessors (`device/queue/instance/
      adapter`). The render operations live in a shared CONCRETE layer (`OmWgpuSceneRenderer`/
      `OmWgpuSurface`, shared by main-view + sensors), dispatched select-then-concrete — so ordinary
      wgpu work (lighting, sensor-RTT, Metal/Linux surfaces, the default flip) extends that concrete
      layer, exactly as baseline §0.5/§4 define the remaining work, and adds NO surface virtual. The
      stale "R1+ will add operation virtuals" header comment was reconciled to the shipped reality.

---

## 3. The line: correctness (in) vs. fidelity/capability (out)

The single most important distinction in this plan:

| In scope (architectural baseline) | Out of scope (post-baseline, incremental) |
|---|---|
| Every scene draw **renders** (A1) | How *closely* shadows/AA/tonemapping match WREN (the last 25% parity) |
| Main view uses the **real** scene lights + casts shadows (A2) | Sun-shaft placement, CSM cascades, IBL, TAA |
| Newton faithfully runs **one** articulation class (wheeled/Hinge-Slider) | Newton running mesh collision / ball+hinge2 joints / force writes / legged / RL transfer (the 35→100% capability climb) |
| The **surface abstraction** exists | Metal (macOS) + Vulkan-surface (Linux) **impls** (hardware-gated) |
| Backends are **dispatched + reversible** | Flipping the **defaults** to Newton/wgpu (the Phase-ζ policy go/no-go) |

---

## 4. Explicitly OUT of scope (do NOT block the baseline on these)

These are real, valuable, and tracked elsewhere — but they are fill-in *within* the
finished architecture, so the baseline does not wait on them:

- **Newton capability climb** (~35–40% → ~100% world coverage): mesh narrow-phase, non-revolute joints,
  force writes, native contacts, legged standing, RL transfer. Planned in
  [newton-ode-replacement-plan.md](newton-ode-replacement-plan.md) (its own coverage-meter-driven plan);
  status in [rl-current-state.md](rl-current-state.md) + engine-migration §13. *Each is a Newton virtual
  override on the existing surface, not new structure.*
- **wgpu fidelity** (75% → ~90%+ parity): shadow-placement, IBL, TAA, T2–T5. Tracked in
  [r4-completion-checklist.md](r4-completion-checklist.md).
- **Cross-platform surfaces** (Metal/Linux): needs hardware not present here; same
  `OmWgpuSurface`/`WbWindingSurface` abstraction, new platform impl.
- **The default flips** (Phase ζ): `physicsBackend "auto"→Newton` is effectively already on;
  `renderBackend "wren"→"wgpu"` waits on fidelity ≥ threshold + the gate ([default-flip-plan.md §5.5](default-flip-plan.md)).
- **Cosmetic `Wb*` → `Om*` class/file rename** (Phase C, deliberately skipped): mechanical,
  zero behavior change; do it as one mass rename whenever, not a baseline blocker.

---

## 5. Baseline acceptance criteria (the "definition of done" for THIS milestone)

OmniSim is **architecturally complete** when every box is checked:

- [x] A1 + A2 — the wgpu main view renders a representative world **geometrically complete**
      with its **real lights + shadows** (correctness, not parity). ✅ DONE 2026-06-07
      (`14107285`); verified on panda + high-res-IFS + rosbot.
- [~] B1–B4 — a dual-backend oracle + standing golden + determinism gates run in CI; a
      deliberate regression in either arm fails a gate; the legacy-only build stays bit-identical.
      *(🟦 physics side done: B1 physics arm + unified front-end, B3 determinism gate standing &
      verified, B4 physics reconciled to the pre-push hook. 🟥 render side ✅ DONE: B1 per-world render
      arm + B2 golden-image gate + B4 render cell (render gate in the pre-push hook), all 2026-06-07 and
      gate-verified to fail on divergence. B1–B4 are complete; only the C1 default flip remains.)*
- [x] C1 + C2 — build-flag default decided; both dispatcher surfaces signed off as final. ✅ DONE
      (2026-06-07): 🟥 C1 `OMNISIM_WITH_VULKAN` flipped ON (fallback-verified); C2 BOTH halves signed
      off (🟦 physics + 🟥 render).
- [x] The reversibility lever is proven end-to-end: `OMNISIM_LEGACY=1` returns the whole
      process to byte-identical ODE+WREN. ✅ DONE (2026-06-07). Now that the default build compiles
      BOTH new backends in (C1), this matters. New headless probe `OMNISIM_PROBE_BACKENDS` reports which
      backend each registry's `resolve()` picks under the current env; `scripts/dev/reversibility_check.py`
      drives it across the lever configs and asserts: `OMNISIM_LEGACY=1` → (Wren, Ode) [the umbrella
      reverts BOTH]; `OMNISIM_FORCE_ODE` → physics Ode; `OMNISIM_FORCE_WREN` → render Wren; and the
      DEFAULT → (Vulkan, Newton), so LEGACY is a real revert *from* the new stack. **Verified GATE PASS.**
      Byte-identity of the reverted output follows by construction (the legacy ODE+WREN code is untouched —
      non-negotiables #1/#2) and is empirically backed by the two standing pre-push gates: the forced-ODE
      determinism smoke (legacy physics byte-identical) and the render golden (wgpu deterministic, WREN
      unchanged).

At that point: **no remaining work requires a deep architectural change.** Newton coverage,
wgpu fidelity, cross-platform, and the default flips all proceed as ordinary, incremental,
independently-shippable features on a stable foundation.

---

## 6. The final plan (sequenced)

Dependency- and risk-ordered. Each phase is independently committable and leaves the tree
green; legacy stays the default throughout.

1. **Phase B-AUDIT (½ day) — close the surfaces (C2) + scope the oracle (B1).** Confirm no
   feature will need a new dispatcher virtual; spec the unified oracle. *Cheap, unblocks the
   framing.* — 🟦 **physics side LANDED (2026-06-07):** `OmPhysicsBackend` signed off
   ([dispatcher-surface-signoff.md §1](dispatcher-surface-signoff.md)); unified oracle built
   (`scripts/dev/dual_backend_oracle.py`) with the physics arm wired + a render-arm seam. 🟥 render
   side (C2 `OmRenderBackend` sign-off + the render arm) is the render session's.
2. **Phase R-FIX (the hard one) — A1. ✅ LANDED (2026-06-07, `14107285`).** Root-caused via a
   fresh-cache-vs-persistent-cache A/B (the fresh cache rendered the floor; the persistent one
   didn't) → narrowed to the mesh cache → found the real cause: `glGetBufferSubData` readback
   with no current GL context poisoning non-primitive meshes on the first frame. Fixed with one
   `makeWrenCurrent()` before the collect. The "missing-Plane / high-viewpoint" framing was a
   red herring (the floor is an IFS, not a Plane). *The one genuinely-uncertain item is closed.*
3. **Phase R-LIGHT — A2. ✅ LANDED (2026-06-07, `14107285`).** Real-sun + cast-shadow main-view
   wiring committed with A1; diagnostics trimmed to a single gated screenshot hook.
4. **Phase GATES — B1, B2, B3. ✅ DONE (2026-06-07).** 🟦 B3 physics determinism gate standing in the
   pre-push hook; 🟦 B1 physics arm + unified front-end; 🟥 B1 render arm (`render_oracle.py`, wired into
   the dual oracle) + 🟥 B2 golden-image gate (`--golden`, panda golden) — both gate-verified to fail on
   divergence. The unified oracle + a real golden scene + the determinism gate all catch a deliberate
   regression. *Remaining gate work = B4 render cell (Phase CI below): wire `render_oracle.py` into the
   pre-push hook alongside the physics gate.*
5. **Phase CI — B4. ✅ standing gates landed (2026-06-07).** Both arms now gate automatically on the
   pre-push hook: 🟦 physics (`physics_oracle.py --gate --require-newton` + forced-ODE smoke) and 🟥 render
   (`render_oracle.py` parity + golden, on render-touching pushes). The cloud 2×2 build-matrix cell stays
   deferred behind the same self-hosted-GPU-runner question as the physics arm — not a baseline blocker.
6. **Phase LOCK — C1 + sign-off. ✅ C1 + C2 DONE (2026-06-07).** `OMNISIM_WITH_VULKAN` flipped ON
   (fallback-verified); both dispatcher surfaces signed off. The ONLY §5 box left is the end-to-end
   reversibility proof (`OMNISIM_LEGACY=1` → byte-identical ODE+WREN on the golden set) — a shared,
   cross-arm verification — after which the baseline can be tagged.

**Estimate:** Phase R-FIX — the only open-ended item — **is now closed** (`14107285`). Every
remaining baseline item (B2 render golden gate, B4 render cell, C1 build-flag flip, C2 render
surface sign-off) is bounded, mechanical engineering measured in days, not the months the
*full* ζ end-state needs. **The architectural baseline is now within reach on the render side;
the capability/fidelity tail is long — and that is exactly the point of separating them.**

---

## 7. How this relates to the other plans

- [engine-migration-plan.md](engine-migration-plan.md) §8 — the *full* Phase-ζ end-state
  (defaults flipped + 64-cam@60fps + Tier 1–5 + macOS). This doc defines the **earlier**
  baseline that §8 builds on; §8's audit (§8.1, 2026-05-30) is superseded by §1 above.
- [default-flip-plan.md](default-flip-plan.md) — the safety harness + the *default-flip*
  sequencing. Our B-track is its §3 harness; the flips themselves are our §4 (out of scope).
- [r4-completion-checklist.md](r4-completion-checklist.md) — the render fidelity ladder
  (our §4 out-of-scope tail).
- [rl-current-state.md](rl-current-state.md) — the Newton capability/RL status (our §4
  out-of-scope tail).
