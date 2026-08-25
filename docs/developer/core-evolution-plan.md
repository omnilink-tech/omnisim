# Core Evolution Plan — five verified axes toward a smaller, more independent core

**Status: ACTIVE (baseline measured 2026-07-18). Phase I (I1–I3) SHIPPED and verified the same day (`4a853f3c`); Q0 closed by audit (premise wrong, already minimal); Q2's ratchet is live in CI-runnable tests (`21d7d382`); Q1 audited with file:line seams (below), implementation not started; axes 1/4/5 are direction-setting only.**

This document came out of the question *"should we rewrite the core engine in Rust for performance and identity?"* The answer we settled on: **no big-bang rewrite** — Rust-vs-C++ buys no meaningful performance at this layer (the real wins are architectural: GPU physics, wgpu, multi-instance), and a ~185k-line port would freeze the product for years while re-earning upstream's accumulated bug fixes with the old engine as the only oracle. The path to the same destination — own physics, own renderer, own agent surface, shrinking C++ inheritance — is to keep **strangling subsystems one verified seam at a time**, exactly how Newton and wgpu landed.

That answer is only honest if we also name what is *not* ideal today and fix it. This plan verifies five weaknesses with data from this tree and turns each into a workstream. **Axes 2 (Qt) and 3 (controller IPC) are the active phases** — they are the two where "better" is measurable within weeks, not quarters.

Relationship to other docs:

- [engine-migration-plan.md](engine-migration-plan.md) remains the master plan for the physics (Newton) and rendering (wgpu) arms. Axis 1 here only adds a *shrink ledger* on top of it; nothing below overrides its sequencing, and nothing below touches the wgpu default-flip bar (which stays human-gated).
- [architecture.md](architecture.md) describes the current architecture; this doc describes how it evolves.
- The controller IPC work extends the nonce fix from [default-flip-plan.md](default-flip-plan.md) §3.5 and the `omnisim doctor` ABI gate. [controller-protocol.md](controller-protocol.md) documents the current packet format the Phase-I handshake extends, and [controller-ipc-and-step-loop.md](controller-ipc-and-step-loop.md) the step-loop coupling it must not disturb.
- [module-dependency-map.md](module-dependency-map.md) describes where the layer boundaries leak today; the Axis-1/Q2 ratchets are the enforcement mechanism for holding them.

---

## 0. Measured baseline (2026-07-18)

All counts from this clone at `31870af6`, excluding generated artifacts (`src/omnisim/build/` is gitignored moc output and is excluded everywhere below). LOC = physical lines in `*.cpp/*.hpp/*.c/*.h`. Repro commands are inlined per section so the ledger can be re-measured after each phase.

| Quantity | Value |
|---|---|
| Engine core `src/omnisim` (non-generated) | **184,732 LOC / 775 files** |
| — `nodes/` (base node set) | 74,728 |
| — `gui/` | 25,852 |
| — `vrml/` (parser/tokenizer/PROTO) | 16,452 |
| — `render/` (wgpu bridge) | 14,875 |
| — `wren/` (WREN bridge) | 10,022 |
| — `scene_tree/` | 9,484 |
| — `core/` | 6,753 |
| — `physics/` (backend shims) | 6,276 |
| ~~Vendored ODE (`src/ode`)~~ — **DELETED 2026-08-08 (`bdc02139`)**; ledger measured 2026-07-18, before the deletion | ~~89,174~~ → **0** |
| WREN library (`src/wren`) | 18,698 |
| libController (`src/controller`) | 25,255 |
| ~~Files touching the ODE API directly~~ — **ratchet reached ~0 (`bdc02139`)**; figure is the 2026-07-18 pre-deletion baseline | ~~97 files / 39,681 LOC~~ |
| Files touching the WREN API directly | 110 files / 55,089 LOC |
| Files including Qt directly | **414 of 775 (53%)** |
| Qt6 DLLs in the local dev bundle | 90.1 MB (pacman residue, untracked — **never ships**) |
| Qt6 DLLs the installer actually ships | **~33.5 MB** (11 DLLs + plugins, `files_msys64.txt`); binary links 9 |
| Rust already in the product (wgpu_native.dll) | 13.2 MB |
| Bundled Newton Python runtime | 569 MB |
| Python product surface (CLI/harness/scripts + policies) | 32,912 + 120,281 LOC |
| PROTO files / `.wbt` worlds in tree | 261 / 700 |

```bash
# core size per subdir (excludes generated build/)
find src/omnisim -name '*.cpp' -o -name '*.hpp' -o -name '*.c' -o -name '*.h' \
  | grep -v '/build/' | xargs wc -l \
  | awk '$2 ~ /^src/ {split($2,p,"/"); s[p[3]]+=$1} END {for (k in s) print k, s[k]}' | sort -k2 -rn
# subsystem coupling
grep -rlE 'dBodyID|dWorldID|dJointID|dGeomID|#include <ode/' src/omnisim --include='*.cpp' --include='*.hpp' | grep -v /build/ | wc -l
grep -rlE '#include <wren/|wr_[a-z_]+\(' src/omnisim --include='*.cpp' --include='*.hpp' | grep -v /build/ | wc -l
# Qt footprint
grep -rlE '#include <Qt?[A-Z]' src/omnisim --include='*.cpp' --include='*.hpp' | grep -v /build/ | wc -l
objdump -p msys64/mingw64/bin/omnisim-bin.exe | grep 'DLL Name' | grep -i qt
```

---

## Axis 1 — The C++ core is oversized for what it will eventually do

**Evidence** *(measured 2026-07-18; ⚠ **the ODE half was resolved 2026-08-08 by `bdc02139`** — 89,174 vendored LOC and the 39,681 LOC of ODE-coupled engine code are gone, so Axis 1's headline "~95k LOC coupled to superseded subsystems on top of ~108k of vendored predecessors" is now roughly the WREN half alone. The WREN figures stand.)*: the engine still carries both generations of two subsystems at once: 89,174 LOC of vendored ODE plus 39,681 LOC of ODE-coupled engine code, alongside the Newton path whose C++ footprint is a 4,492-line shim ([`OmNewtonBackend.cpp`](../../src/omnisim/physics/OmNewtonBackend.cpp)) driving a Python/Warp GPU runtime; and 18,698 LOC of WREN plus 55,089 LOC of WREN-coupled code alongside the 14,875-line wgpu bridge. That is roughly **95k LOC of engine code coupled to subsystems whose successors are already in-tree**, sitting on top of ~108k of vendored predecessor libraries.

**Why it matters.** Every one of those lines is surface a contributor can break, a compile-time cost, and — for the rewrite question — the reason a port looks like "years". The end-state core is a scene-graph + device model + orchestration shell, likely well under half today's size.

**Plan (direction, no new phase).** The shrink happens as a *consequence* of the engine-migration plan, not as its own project:

1. ODE and WREN remain forever-fallbacks per the master plan — we do **not** delete them now. But new code must not widen the coupled set: treat the two grep counts above (97 ODE-coupled files, 110 WREN-coupled files) as ratchets that may only go down. Re-measure and record them in this file at each release.

   > ⚠️ **2026-08-08 — THE ODE HALF OF THIS ITEM IS DIRECTLY CONTRADICTED BY EVENTS.**
   > `bdc02139` deleted `src/ode/` and `include/ode/`, and it happened **out of the
   > sequence step 3 below specifies** — before the wgpu default flip and without waiting
   > a full release cycle of Newton-default stability. So *"we do not delete them now"* is
   > superseded for ODE by a later owner decision (see
   > [ode-retirement-campaign.md](ode-retirement-campaign.md)); **the WREN half stands
   > unchanged** and WREN is still canonical.
   >
   > For the ratchet specifically: the **ODE-coupled-file count is effectively zero** — the
   > grep at the top of this section (`dBodyID|dWorldID|dJointID|dGeomID|#include <ode/`)
   > no longer matches ODE code. ⚠ Do not read that as a finished cleanup: ~23
   > `#ifdef OMNISIM_WITH_ODE` sites, a fully stubbed `src/omnisim/ode/` module and
   > `OmOdeBackend.{cpp,hpp}` still compile (their `#else` branches), and
   > `OmOdeTypes.hpp` survives deliberately as vestigial typedefs carried through ~44
   > headers by the `OmBodyHandle`/`OmJointHandle` pattern. Retiring those is real
   > remaining work. **Re-measure both ratchets and the LOC ledger before quoting either.**
2. When a coupled file is touched for any other reason, prefer moving its backend-specific logic behind the existing `OmPhysicsBackend` / render-bridge seams rather than patching in place.
3. Revisit actual deletion only after the wgpu default flip (human-gated, see [wgpu-renderer-status.md](wgpu-renderer-status.md)) and a full release cycle of Newton-default stability.

**Acceptance.** The two ratchet numbers decline release-over-release; no new file enters either coupled set without a recorded reason.

---

## Axis 2 — Qt is load-bearing far beyond the GUI  ⭐ ACTIVE (Phase Q)

**Evidence.**

- **414 of 775 engine files (53%) include Qt directly.** By module: 2,209 `QtCore` includes vs 298 `QtWidgets`, 101 `QtGui`, 21 `QtNetwork`, 7 `QtOpenGL`, 4 `QtWebSockets`, 1 `QtQml`. Qt is not "the GUI layer" — it is the engine's de-facto standard library.
- **The bleed reaches compute code:** `nodes/` 92 files, `core/` 49, `vrml/` 47, `scene_tree/` 41, `maths/` 9, `physics/` 2 include Qt directly.
- **Headless is not headless.** [`main.cpp:1322`](../../src/omnisim/gui/main.cpp#L1322) constructs `OmGuiApplication`, which is `public QApplication` ([`OmGuiApplication.hpp:34`](../../src/omnisim/gui/OmGuiApplication.hpp#L34)) — the full QtWidgets stack — even under `--batch --mode=fast --no-rendering`. This is why Linux headless runs need Xvfb, why every one of K parallel instances (§3e of AGENTS.md) pays QApplication startup + RSS, and it is implicated in the true-headless crash class that forces some demos to run windowed.
- **The local dev bundle carries 90.1 MB of Qt6 DLLs but the binary links 9 of them** (Core, Gui, Widgets, Network, OpenGL, OpenGLWidgets, Qml, WebSockets, Xml ≈ 33 MB). Designer, the Quick/QML-UI family, Labs, Multimedia, ShaderTools etc. are pacman residue in the (untracked) local msys64 — **the installer never ships them** (see Q0 verdict below).
- **Two non-obvious load-bearing uses**, so nobody "just removes Qt": `QtQml` is the **JavaScript engine for procedural PROTO templates** ([`OmTemplateEngine.cpp`](../../src/omnisim/core/OmTemplateEngine.cpp)), and `QtWebSockets` backs the streaming servers. The controller IPC server is `QLocalServer` (`QtNetwork`) — see Axis 3.

**Why it matters.** A harness-first, agent-driven simulator whose canonical run mode is increasingly headless (RL lanes, smoke farms, cloud pods) should not require a widget toolkit — with a display platform plugin — to step physics. And 2,209 QtCore includes are the single largest obstacle to ever making the core small or portable.

### Phase Q plan

**Q0 — Trim the shipped Qt set. ✅ CLOSED 2026-07-18: audited, already minimal — the premise was wrong.**
The audit found the installer manifest ([`scripts/packaging/files_msys64.txt`](../../scripts/packaging/files_msys64.txt)) already ships a near-minimal closure: 11 Qt DLLs (~33.5 MB — the binary's 9 plus defensive PrintSupport/Concurrent) + the platform/imageformat/tls plugins + translations. The 90.1 MB measured in the local `msys64/mingw64/bin` is **untracked pacman residue that never ships** — `git ls-files msys64/mingw64/bin` is empty, and the installer's pacman closure (make/coreutils/gcc) pulls no Qt. Cross-checks: `qt_utils.dll` links only Core/Gui/Widgets (covered); `webots.exe`/`webotsw.exe` link no Qt; the engine link line requests `Qt6PrintSupport` but the linker drops it (not in the binary's import table — the manifest entry is defensive and stays). No change shipped; the size win Q0 promised does not exist. The remaining Q0-adjacent nit — the never-imported `-lQt6PrintSupport` on the link line — is cosmetic and left alone.

**Q1 — Tiered application object (the structural fix; weeks). AUDITED 2026-07-18; ⭐ FIRST SLICE SHIPPED 2026-07-19 (`9ae34047`): no-window mode.**
**⭐ SECOND SLICE SHIPPED 2026-07-19 (`e285bc4b`): Tier C compute-only mode.** `OMNISIM_NO_GL=1` runs physics/controllers/IPC with **no window, no GL context, no WREN at all**. The lever: `OmBaseNode::finalize` skips `createWrenObjects` when no context exists, and the node tree's own `areWrenObjectsInitialized()` convention (138 guard sites) short-circuits the entire render surface from that one flag; only three call sites lived outside the convention (PBR BRDF bake, `OmBackground::activate`, `OmAbstractCamera::setup`) and are now guarded. Enabling a vision device in this mode is a **fatal, attributed error** (vision worlds use `OMNISIM_NO_WINDOW`, which still renders). `run-headless` grows `--no-window` / `--no-gl`. Measured K=4 fleet (controller worlds, RTX 3060 laptop box): **1593 MB → 1236 MB (no-window, −22%) → 1041 MB (no-gl, −35%)**; single instance 259 MB compute-only. Smoke set green in all three modes. The full Tier C (QCoreApplication, no Qt-Gui at all) remains open — this spike proves its hardest premise: **the world loads, finalizes and steps with WREN structurally absent.** Follow-up (`87453242`): compute-only mode now defaults to the Qt **`minimal` platform** (no window-system connection, decided in `main()` before the app constructs; `qminimal.dll` vendored + in the packaging manifest) — on Linux this should remove the X/Wayland/**Xvfb requirement** for compute-only runs entirely (verified on Windows, Linux validation pending). Compute floor on this box: **~259 MB/instance** regardless of physics backend — the remainder is Qt base + engine, which is what the QCoreApplication step targets next.

`OMNISIM_NO_WINDOW=1` (opt-in; the default path is untouched) runs the world with **zero GUI construction** — no `OmMainWindow`/`OmSimulationView`/`OmView3D`, no splash/dialogs/actions — on a bare, never-shown `OmWrenWindow` whose GL context targets a `QOffscreenSurface`. Verified A/B on identical worlds (RTX 3060 laptop box): controller steps and camera images **byte-identical** to the default path, full smoke set green in both modes; **RSS 387→304 MB (controller world) and 388→322 MB (camera world), ≈−20% per instance**, phantom main-view framebuffers held to 160×120 (a 1280×720 chain measured ~200 MB). Three hard-won findings recorded in code comments: `OmView3D` is not embeddable headless (wires ~60 GUI actions, crashes without the widget tree); the `gl:` shader search path lives in the `OmView3D` ctor and its absence makes every WREN shader fail at link with an empty log (WREN now surfaces the driver's link log, and `makeWrenCurrent` warns once on failure instead of silently no-oping all GL); `makeCurrent` on a hidden native window is unreliable — `QOffscreenSurface` is the supported target. This mode is also the natural seam for the remaining tiers: Tier B (offscreen QPA) and Tier C (`QCoreApplication`) both build on this window-free run path.
A file:line audit of the tier split found: `OmMainWindow` is constructed **unconditionally** for every world-running invocation ([`OmGuiApplication.cpp:358/:408`](../../src/omnisim/gui/OmGuiApplication.cpp)) — no batch/no-rendering branch skips it; flags are parsed only *after* the `QApplication` base exists (`parseArguments`, `:133`), so tier selection needs a pre-construction argv scan in `main.cpp`. Under `--no-rendering`, WREN + a real GL surface still initialize (`OmWrenWindow` is a `QWindow` + `QOpenGLContext`, [`OmWrenOpenGlContext.hpp:27`](../../src/omnisim/render/OmWrenOpenGlContext.hpp) — the file moved from `src/omnisim/wren/` to `src/omnisim/render/` when WREN was deleted (`976b9449d`); the class survives as the GL-context shim the wgpu blit path uses); the flag only suppresses the main-view refresh ([`OmView3D.cpp:374`](../../src/omnisim/gui/OmView3D.cpp#L374)). The sim loop is a `QTimer` + `QLocalServer` on the event loop ([`OmSimulationWorld.cpp:63/:144`](../../src/omnisim/engine/OmSimulationWorld.cpp)) — **both work under a bare `QCoreApplication::exec()`**, so Tier C keeps its event loop. The R4 wgpu probes at the top of `main.cpp` (`:203–1256`) already run a compute+render path with zero GUI pre-app — the natural Tier-C seam. Tier C's five change areas: (1) pre-app argv scan + tier-selected app class in `main.cpp`; (2) a window-free driver parallel to `OmGuiApplication::setup()`; (3) decouple world handoff from `OmSimulationView` (`OmMainWindow.cpp:1252`); (4) guard `robot->renderCameras()` in the step loop (`OmSimulationWorld.cpp:304-309` — needs live GL; compute-only mode must skip or reject camera worlds); (5) headless equivalents for streaming/screenshot wiring (`OmMainWindow.cpp:380/:384`, `OmSimulationView.cpp:160-161`). Top risks: camera sensors need GL (biggest functional limit of Tier C); pervasive unguarded widget-tree coupling (null-deref surface); late app-class selection sequencing with the TCP/streaming servers.
Split startup into three tiers selected before the application object is constructed:

| Tier | App object | When | What it buys |
|---|---|---|---|
| A — desktop | `QApplication` (today's path) | default windowed | unchanged |
| B — render-headless | `QGuiApplication` + `offscreen` QPA | `--batch` with rendering (screenshots, capture, harness) | **no Xvfb on Linux**, no widget stack, fewer headless crash modes |
| C — compute-headless | `QCoreApplication` | `--no-rendering` | no display dependency at all; cheapest startup/RSS for K-instance farms and RL lanes |

Audit list for the tier split (each is a known QApplication assumption): `qInstallMessageHandler` setup, `OmMainWindow`/`OmView3D` non-null assumptions in supervisor and streaming paths, GL context ownership when WREN initializes offscreen, event-loop use inside the sim loop, and the R4 wgpu probes at the top of `main.cpp` (which already run pre-window — good precedent).
*Acceptance:* `run-headless --no-rendering` boots Tier C on Linux **without Xvfb** and passes the smoke set; Tier B produces byte-plausible screenshots through the harness; measure and record startup-time and RSS deltas per tier, per machine fingerprint (`env_fingerprint.py`), for K=4 parallel instances.

**Q2 — Stop the QtCore bleed with a ratchet, then drain leaf dirs (ongoing). Ratchet ✅ LIVE 2026-07-18 (`21d7d382`); ⭐ FIRST CLICKS 2026-07-19 (`cf96d6eb`): `physics/` is at ZERO Qt includes** (OmNewtonBackend drained — std::string APIs, std::ofstream sidecar with the byte-compatible schema, verified live on a Newton world), **`maths/` 9→7** (OmMathsUtilities + OmPolygon off Qt containers). Remaining in `maths/`: OmPrecision + the vector/quaternion `toString` family — **161 call sites**, deferred to a coordinated pass so it doesn't churn files parallel lanes are editing. **Q3 update, closed by measurement (same session): the full QCoreApplication swap is NOT worth pursuing for footprint** — bench: bare `QApplication` on the `minimal` platform 16.9 MB vs `QCoreApplication` 9.9 MB, i.e. ~7 MB of a 259 MB compute-only floor; re-litigate only if dropping the QtWidgets/QtGui *link dependency* ever becomes a goal in itself.
Add a conformance check (same pattern as the G1 spec-conformance test) that records the per-directory Qt-include counts above as a **high-water mark** — a PR may lower a count, never raise it. Then migrate leaf-first, where counts are small and types are simple: `maths/` (9 files) → `physics/` (2) → `util/`, replacing `QString`/`QVector`/`QMap` with `std::` equivalents at the boundaries. `nodes/`/`vrml/` (139 files) are *not* targeted until the ratchet has held for a while — they migrate opportunistically under the Axis-1 rule.
*Acceptance:* ratchet in CI; `maths/` and `physics/` at zero Qt includes.

**Q3 — Decisions deliberately deferred.** Whether the desktop shell stays Qt long-term (probably yes — it is fine at its actual job), and what replaces `QtQml` as the procedural-PROTO JS engine (candidates: QuickJS, or reusing the harness's Python path). Neither blocks Q0–Q2.

---

## Axis 3 — The controller IPC layer fails silently  ⭐ ACTIVE (Phase I)

**Evidence.**

- **Transport:** engine-side `QLocalServer` ([`OmController.cpp`](../../src/omnisim/control/OmController.cpp)); controller-side raw named-pipe open in C ([`robot.c`](../../src/controller/c/robot.c), `compute_socket_filename`). Pipe name `webots-<tmpId>-<nonce>-<robot>`, where the per-launch nonce (engine PID, exported as `OMNISIM_IPC_NONCE` at [`OmController.cpp:586`](../../src/omnisim/control/OmController.cpp#L586)) exists precisely because Windows allows multiple server instances of one pipe name and a fresh child could attach to a *previous* launch's lingering pipe (the launch-flake race, [default-flip-plan.md](default-flip-plan.md) §3.5).
- **No in-band coherence check.** The only version comparison is a **stderr warning after a successful connect and configure**, comparing 6 characters ([`robot.c:451-456`](../../src/controller/c/robot.c#L451)). The stale-libController failure happens *before* that point, so the observed symptom of an engine↔lib ABI split is: controller finalizes, arms, ticks zero times, exits 0 — **and a headless run still prints PASS**. This cost real sessions before commit `6eea9d76`.
- **Enforcement is entirely out-of-band:** `omnisim doctor` scans both **binaries for the nonce token** (a symbol probe — [`omnisim/doctor.py`](../../omnisim/doctor.py)) plus an mtime heuristic, and libController appends a `connect_error` sidecar when it cannot open the pipe. Both are preflights/postmortems; nothing stops the run itself from hanging silently.
- **Extern controllers are second-class:** without the env var they fall back to the legacy nonce-less pipe name ([`robot.c:1240-1243`](../../src/controller/c/robot.c#L1240)), so the stale-pipe race the nonce fixed still exists for externs.

**Why it matters.** This is the harness through which *every* robot behavior runs. A transport whose failure mode is "indistinguishable from a passing run" taxes every debugging session with "is this a real bug or the IPC layer?" — the exact tax `doctor` was built to refund. The fix is to make the transport unable to fail silently, so `doctor` becomes a convenience instead of the last line of defense.

### Phase I plan

**I1 — Fail-fast versioned handshake (the core fix). ✅ SHIPPED 2026-07-18 (`4a853f3c`).**
Implemented exactly as designed below (frame layout in [`messages.h`](../../src/controller/c/messages.h); engine side [`OmController::performIpcHandshake`](../../src/omnisim/control/OmController.cpp), lib side `scheduler_validate_and_echo_hello` in [`scheduler.c`](../../src/controller/c/scheduler.c)), on all three transports. The kill test lives at [`tests/ipc/test_handshake_failfast.py`](../../tests/ipc/test_handshake_failfast.py) — it launches a private engine and impersonates a pre-handshake libController (no shared-state mutation, safe alongside live sessions). Verified on the RTX 3060 laptop box: hello v1 sent on accept, mismatched client dropped in **5.0 s** with the attributed ERROR in the log; positive path — a Python controller completed 50 steps under the new pair, and the full smoke set passes. (A first kill-test design that binary-patched the DLL's magic string was discarded: the compiler inlines the 4-byte magic as an immediate, so patching the string literal is dead data — and it mutated the shared DLL.)
Define a fixed hello frame exchanged immediately on pipe/TCP connect, before any request traffic: magic `OMNI`, protocol version (u16), and the launch nonce. Engine sends on accept; libController must echo within a timeout (5 s) or the engine closes and reports; libController likewise verifies the frame or exits loudly with the same message the `connect_error` sidecar uses today. Version mismatch ⇒ **both** sides produce an attributed, fatal error naming the two builds. Engine and lib ship from one tree, so the version bumps in lockstep — no compatibility window needed; an *old* nonce-less lib can't complete the hello and gets the engine-side report instead of a hang. Applies to intern pipe, extern pipe, and TCP (`scheduler_is_tcp`) equally.
*Acceptance — the kill test:* deliberately pair the previous release's libController with a fresh engine; the run must fail **loudly in under 5 seconds** with the ABI message on both the engine log and the sidecar, non-zero controller exit — instead of today's zero-tick PASS. Add this as a scripted regression (build matrix keeps one stale lib artifact around for it).

**I2 — Zero-tick watchdog + verdict hardening. ✅ SHIPPED 2026-07-18 (`4a853f3c`).**
Engine: `OmController::armStartWatchdog` warns (never kills) when an intern controller hasn't paired within `OMNISIM_CONTROLLER_START_TIMEOUT_S` (default 60) — verified firing at a configured 8 s with the attributed line. Runner: handshake/pairing diagnostics are always-fatal patterns, and the runner now also reads **this run's** entries from the `connect_error` sidecar (matched by the log header's pid) — pipe-open failures happen before controller stderr exists and were previously invisible to verdicts; that gap was found live when a broken pair still produced `0 errors, 0 warnings, PASS`. Unit tests: [`tests/harness/test_headless_verdicts.py`](../../tests/harness/test_headless_verdicts.py).
Engine side: if a spawned controller has not completed configure + first `step` within T seconds, emit a `controller.hang` event on `/sim/events` and mark the run degraded. Harness/`run-headless` side: a run in which any robot's controller never reached its first step **cannot** report PASS (this generalizes the existing rule that headless PASS verdicts need content checks, and turns it from reviewer folklore into code).
*Acceptance:* a controller stub that connects and stalls forever produces a failed run with `controller.hang` attributed to the robot, under both the harness and `run-headless`.

**I3 — Nonce rendezvous for extern controllers. ✅ SHIPPED 2026-07-18 (`4a853f3c`).**
Extern pipe names now carry the launch nonce (`webots-<tmpId>-<enginePid>-<robot>` — observed live), the engine writes `ipc-nonce` into its port-salted tmp dir at listen time, and extern libController reads it (`read_instance_ipc_nonce` in [`robot.c`](../../src/controller/c/robot.c)), falling back to the legacy name when the file is absent (older engine). The read nonce also arms the handshake's in-band nonce cross-check via `scheduler_set_expected_ipc_nonce`. The kill test asserts the nonce-protected extern naming.
The engine already owns a port-salted tmp dir; write the launch nonce to a rendezvous file there at startup. Extern libController reads it when `OMNISIM_IPC_NONCE` is absent and uses the uniquified pipe name, closing the legacy stale-pipe race for externs. Fallback to the legacy name only when no rendezvous file exists (older engine).
*Acceptance:* back-to-back extern launches on a reused TCP port cannot cross onto a stale pipe (scripted: kill an engine mid-run, relaunch, assert the extern child attaches to the new instance).

**I4 — Transport convergence (later, joins Phase Q).**
Replace `QLocalServer` with a small native pipe/socket server sharing one versioned framing across intern/extern/TCP. This removes `QtNetwork` from `control/` (helping Q2's ratchet) and leaves exactly one place where transport framing is defined. Not started until I1–I3 have soaked; the HTTP wire protocol ([PROTOCOL.md](../../PROTOCOL.md)) is untouched throughout.

---

## Axis 4 — Two other languages already own the future (keep it that way)

**Evidence.** The physics arm's C++ contribution is a 4,492-line shim; the actual solver is the 569 MB Python/Warp/CUDA Newton runtime. The renderer seam ships 13.2 MB of compiled Rust (`wgpu_native.dll`) behind a 14,875-line C++ bridge. The agent-facing product surface — CLI, doctor, harness, capture, cinema, omniworld, the entire policies/RL tree — is 153k+ lines of Python. The C++ share of *code that changes per release* is already a minority and falling.

**Why it matters.** This is the quantitative answer to "should we rewrite for identity": the identity-bearing layers are already not C++. The risk is not too little new-language adoption but *accidental* re-growth of the C++ core.

**Plan (guardrails, no phase).** New subsystems default to the seam pattern: thin C++ bridge, implementation in Python (compute/orchestration) or Rust-behind-C-ABI (performance-critical native, as wgpu already is). A hot, well-isolated module may be ported to Rust behind its existing interface **only** with a benchmark against the incumbent — the same discipline as the wgpu switch bar. No language ports of working code without a measured reason.

---

## Axis 5 — The `.wbt`/VRML/PROTO format is the deepest inheritance (long-term identity frontier)

**Evidence.** The inherited format and node model are the single largest part of the core: `vrml/` (16,452) + `nodes/` (74,728) ≈ **91k LOC, half the engine**, serving 261 PROTO files and 700 worlds in-tree — plus every user's existing Webots asset, which is exactly why AGENTS.md can promise "your Webots knowledge applies directly".

**Why it matters.** If full identity detachment is ever the goal, a native OmniSim scene format (with `.wbt` as a permanent import path) is the real project — far more identity-defining than the implementation language. It is also the riskiest: it touches every world, every PROTO, omniworld's emitter, and the ecosystem on-ramp.

**Plan (explicitly NOT started).** No design work until Phases Q and I have shipped and Axis 1's ratchets have held for at least one release. When opened, it starts as a written design exploration (format survey: keep-VRML-forever vs JSON-native vs USD-adjacent; migration economics; what omniworld already proves about generation) — not as code. Parking it here is the decision.

---

## Sequencing and the "is it actually better" gate

Order: **Q0 → I1 → I2 → I3 → Q1 → Q2 (ratchet immediately, drain gradually)**, with I4/Q3 following, all interleavable with normal release work. Q0 and I1–I3 are individually shippable in days and carry their own regression tests.

The user-visible improvements we commit to measuring (per machine fingerprint, recorded in this file as they land):

| Metric | Baseline (2026-07-18) | Target after Q/I |
|---|---|---|
| Silent zero-tick PASS possible? | **Yes** (stale-lib class) | ✅ **No** — handshake + watchdog + sidecar verdicts; scripted kill test passes |
| Time-to-diagnosis for an ABI split | one debugging session (historical) | ✅ **5.0 s measured**, attributed ERROR in the log |
| Extern stale-pipe race | present (legacy name) | ✅ closed (nonce-protected name + rendezvous file) |
| Linux headless requires Xvfb | yes (QApplication) | no for Tier B/C |
| Shipped Qt payload | ~33.5 MB | ✅ already minimal (Q0 audit; no action) |
| Headless startup / RSS, K=4 instances | single-instance baseline (RTX 3060 laptop box, 2026-07-19): `empty.wbt --no-rendering` settles at **~400 MB RSS / 46 threads** — the full QApplication+OmMainWindow widget tree, a live GL surface, and the Newton import all exist despite nothing rendering; the tier split is what will attribute and shrink that | ✅ two slices shipped: `OMNISIM_NO_WINDOW=1` (`9ae34047`) **−20%** at full render parity; `OMNISIM_NO_GL=1` (`e285bc4b`) **−35%** compute-only (259 MB/instance, vision devices fatal). K=4 fleet measured: **1593 → 1236 → 1041 MB** |
| Qt includes in `maths/`+`physics/` | 11 files | ✅ **physics/ at 0**; maths/ at 7 (`cf96d6eb`; toString family = the remaining 161-site coordinated pass) |
| ODE-/WREN-coupled file counts | 97 / 110 | ratchet, monotonically ↓ — ✅ **ODE half reached ~0 via `bdc02139`** (by deletion, not by decoupling); WREN half still live. Re-measure before quoting. |

What we will **not** do, restated so this plan can't be quoted into it later: no big-bang language rewrite; no wgpu default flip through this plan (separate, human-gated bar); no ROS 2 dependency **in the engine** (the ROS 2 sidecar at `packages/omnisim-ros2/` is out of scope for this plan, and does not change the engine); no scene-format work before its gate above.
