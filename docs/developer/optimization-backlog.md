# Optimization Backlog

Tracks code-optimization work from the 2026-06-27 full-codebase audit: what
landed (tested), and what is **build-gated** — verified findings that need a
C++ build environment to implement and test (the audit box had no compiler).

## Done — tested and committed

| Area | Change | Result | Commit |
|------|--------|--------|--------|
| Imports | `from controller` → `from omnisim` (164 first-party files) | canonical; all compile, 0 legacy left | `36e1dbde` |
| Repo | untrack heavy regenerable `mars_{big,max,small}.wbt` | −41 MB tracked | `36e1dbde` / `a7d8cd8f` |
| Agents | **all 7 roll-your-own runners → shared `OmniLinkAgentRunner`** | **~1765 LOC removed**, 8/8 pass live-HTTP smoke | `852eaeba` `807a4404` `74193867` `774ae6b3` |
| Tests | live-HTTP smoke pytest (auto-discovers shared-runner agents) | regression guard, 8 agents green | `54cc2174` |
| Harness | `compute_render_stats` → Pillow C ops (no `getdata()`/per-pixel loop) | **17× faster** (314→18 ms @1080p), test-verified, deprecation gone | `4799549d` |

Run the agent guard: `pytest tests/agents/test_agent_runner_smoke.py`.

## Build-gated — verified findings, need a compiler to land + test

The audit's highest-value *engine* wins are C++; implement on a box with the
MSYS2/MinGW toolchain, then validate with `make -C src/omnisim sim-gui` +
`python -m omnisim run-headless <world>` / `test-smoke`.

1. **Controller IPC: O(n) linked-list lookups on the per-tick path.**
   `src/controller/c/supervisor.c:198,210` literally carry `// TODO: Hash map
   needed`. `find_field_by_name`, `find_node_by_def`, and `is_node_ref_valid`
   linear-scan the field/node lists; `is_node_ref_valid` runs before *every*
   node getter. Fix: hash map keyed by (node_id, name) + a validity bit →
   O(1). Highest impact for supervisor-heavy worlds.

2. **WREN `glFinish()` readback — MEASURED, NOT the bottleneck. De-prioritised.**
   `src/wren/Scene::getMainBuffer` (`Scene.cpp:111`) does `glFinish()` +
   `glReadPixels` synchronously, and the PBO async path already exists
   (`Scene::initFrameCapture`/`bindPixelBuffer`/`mapPixelBuffer`), so the fix
   looked cheap. **But a 2026-06-27 measurement on the running binary refutes
   the premise:** at a *fixed* 1896×1113 (so identical readback bytes), an empty
   world renders+reads-back in **79 ms** while `warehouse_husky` takes
   **1120 ms** — a **14× gap that is pure draw cost**, since `glReadPixels`
   moves the same 6 MB in both. Readback is a small *fixed* component (≤79 ms,
   most of it harness IPC + the stats call); a PBO swap would save <2% on a real
   scene. **The render bottleneck is the DRAW / scene-graph processing, not the
   readback.** (Caveat: absolutes are likely inflated by software GL and/or the
   harness re-rendering cold per call — the *ratio* is the robust signal.)
   Real next step for render perf: run `OMNISIM_RENDERER_TIMINGS=1` on a
   hardware-GL GUI with a heavy world to see which draw stage dominates, then
   attack **frustum/hidden-node culling** first (the wgpu path has none, and the
   archived fps journey already found off-screen Solids are drawn, not culled).

3. **Core loop O(n²) per tick.** `src/omnisim/engine/WbSimulationWorld.cpp:227`
   calls `robots().contains(robot)` *inside* the `foreach(robots())` loop. Fix:
   iterate a snapshot/mark-set instead of per-item `contains()`.

4. **Newton contact extraction walks the whole scene.**
   `src/omnisim/nodes/WbSolid.cpp:3881` calls `WbWorld::findSolids()` (full
   O(n) traversal) per contact-extraction. Fix: cache a
   `newtonBodyIndex → WbSolid*` map at finalize, maintained on (un)registration.

5. **Scene-tree full relayout on every cell edit.**
   `src/omnisim/scene_tree/WbSceneTreeModel.cpp:251` emits `layoutChanged()`
   (whole-view relayout) for a single-cell change (an MFVector2 workaround).
   Fix the delegate sizing and emit scoped `dataChanged(idx, idx)` instead.

6. **wgpu renderer** (`src/omnisim/render/WbWgpuRenderTarget.cpp`, 8.4k lines):
   per-draw bind groups allocated in the inner loop (cache by material key);
   uniform buffers released+recreated when draw count crosses a boundary (use
   exponential growth); no frustum culling. Also a decomposition candidate.

## Deferred — owned elsewhere or needs a test first

- **RL trainer base-class consolidation** (16 `gpu_*_trainer.py`): already being
  done by an active session via `BatchedG1NewtonEnv`. Do **not** fork it.
- **`g1_deploy_runtime.py` ~21 silent `except: pass`**: deploy hot path in the
  RL subsystem the active session is working — coordinate before touching.
- **harness ⇄ capture SupervisorClient dedup**: the socket framing
  (`_recv_exact`/`_supervisor_call`/sibling injection) is duplicated and has
  already drifted (an error string differs). Worth extracting to one module —
  but add a fake-socket unit test for the framing first; capture's socket path
  isn't covered today.

## Pre-existing test issues found (not regressions)

- `tests/sources/test_line_ending.py`: 3651 files have CRLF (repo is
  Windows-developed, no `.gitattributes`) — repo-wide, predates this work.
- `tests/sources/test_naming.py`: `wb_class_use` baseline drift (+808k) —
  environmental; re-snapshot the baseline if intentional.
- `tests/capture/test_capture_service.py`: `supervisor_stanza` no longer emits
  `fieldOfView`/camera-resolution the 2 tests expect (37/39 pass) — code/test
  drift in the capture stanza, unrelated to this work.
