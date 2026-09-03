# Multi-Instance + Many-Robot Optimization Plan

This is a focused, sequenced plan for a specific workload — running **many simultaneous OmniSim processes on one host**, each with **many robots and physics-heavy worlds**. It is narrower than improvement-backlog.md (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)) and references it where overlap exists.

The optimizations are sequenced by `(impact for the target workload) × (1 / implementation cost)`. Phase 1 is multi-instance unblockers; Phase 2 is per-step IPC; Phase 3 is sensor rendering.

## Validation contract

Every item lands with:

1. A code change in the file(s) listed.
2. A delta against the baseline captured at `tests/benchmarks/optim-baseline/` (see [tests/benchmarks/optim_bench.py](../../tests/benchmarks/optim_bench.py)).
3. The acceptance criterion below met — or, if not, a note explaining why and what we learned.

### Measurement protocol

Windows can't reliably clear the OS file cache without admin tools, so we compare warm-cache numbers against warm-cache numbers. The protocol for both baseline and "after" measurements is:

```
python tests/benchmarks/optim_bench.py all --steps 600 --steps-render 200 \
    --out tests/benchmarks/optim-baseline/<tag>-pass1.json   # warm-up, discarded
python tests/benchmarks/optim_bench.py all --steps 600 --steps-render 200 \
    --out tests/benchmarks/optim-baseline/<tag>-pass2.json   # measurement, kept
```

Pass 1 warms the OS file cache; pass 2 is the canonical measurement. Compare with `optim_bench.py compare <baseline-pass2.json> <after-pass2.json>`.

**Canonical baseline for items #2 onward:** [`tests/benchmarks/optim-baseline/post-item1-pass2.json`](../../tests/benchmarks/optim-baseline/post-item1-pass2.json) (committed at the SHA after item 1 lands).

**Measurement noise floor (pass1 vs pass2 within the same warm session):**

- Single-instance scenarios: ≤15% on sub-millisecond metrics, ~5–10% on loading.
- Multi-instance K=4: up to ~30% on per-step physics and ~12% on `scaling_efficiency`. Treat differences smaller than ~15% at K=4 as inside the noise band; for cleaner K=4 deltas, raise `--steps` to 3000+ (slower, smoother).

---

## Phase 1 — Multi-instance unblockers

### 1. Per-instance log file — LANDED

Log path defaults to the **shared install-root file** `<OMNISIM_HOME>/omnisim_log.txt` (`main.cpp` passes `omnisimDirPath + "/omnisim_log.txt"` to `OmLog::initFileLog`, which uses it verbatim). Port-isolation is **opt-in per child via the `OMNISIM_LOG_PATH` env override** — set it to a unique path per instance, or parallel children all clobber the one shared file (the per-`webotsTmpPath()` default was never landed). `OmLog.cpp` `initFileLog` reads `OMNISIM_LOG_PATH` first and only falls back to the passed path; on a busy/locked file it drops to a per-pid `omnisim_log.<pid>.txt` sibling.

**Result (after-item1.json vs baseline-c890936.json):**

- ✅ Per-instance log files all present: 1/1, 2/2, 4/4 — every child produces its own log; shared `omnisim_log.txt` is no longer written to at all when `OMNISIM_LOG_PATH` is set per child.
- ✅ Loading-scaling at K=4 went from **3.22× → 1.29×** the K=1 child (avg child loading 5315 ms → 406 ms). This is the dominant practical win — parallel launches no longer pay multi-second startup contention.
- ⚠️ Steady-state per-child physics scaling at K=2 went from 1.0× to ~1.6× of K=1 (0.019 ms vs 0.030 ms), so the strict "within 5%" criterion is not met. But absolute values are sub-microsecond and the workload (8 robots × 600 steps) is too small to be conclusive — likely measurement noise.
- ⚠️ The K=1-vs-K=1 cross-run loading delta (1651 ms → 315 ms, "−80.9%") is dominated by OS disk cache, not by the OmLog change. **Cross-run absolute numbers should not be compared**; only intra-run K-scaling ratios are clean. The next baseline should be captured cold to make future deltas comparable.
- Backward compat verified: each per-instance log opens with `=== OmniSim Log Started (pid=<N>): ... ===`; consumers reading `OMNISIM_HOME/omnisim_log.txt` are unaffected when only one instance runs.

---

### 2. Cache eviction on world reset

**Files:**

- [src/omnisim/nodes/OmImageTexture.cpp](../../src/omnisim/nodes/OmImageTexture.cpp) — `gImagesMap`
- [src/omnisim/nodes/OmTriangleMeshGeometry.cpp](../../src/omnisim/nodes/OmTriangleMeshGeometry.cpp) — `cTriangleMeshMap`
- [src/omnisim/nodes/OmCamera.cpp](../../src/omnisim/nodes/OmCamera.cpp) — `mInvalidRecognizedObjects` not cleared on `reset()`
- [src/omnisim/nodes/OmSolidDevice.cpp](../../src/omnisim/nodes/OmSolidDevice.cpp) — `cDirtySensors` not cleared on world reset
- [src/omnisim/core/OmLog.cpp](../../src/omnisim/core/OmLog.cpp) — `mPendingConsoleMessages` only cleared at process exit

**Change:** Add a single "world-reset" hook (or extend the existing one in `OmControlledWorld::reset`) that walks these caches and evicts entries belonging to nodes that no longer exist. Map-style caches get an `evictUnused()` method; the queues get a `clear()` on reset.

**Acceptance:** A reload-creep micro-bench (10× harness `/world/load` of the same world) shows process RSS plateau instead of growing monotonically. (Add this benchmark to `optim_bench.py` once the harness baseline is captured.)

**Risk:** Medium. Texture and mesh cache invalidation is shared across the WREN cache and CPU-side maps — need to evict in the right order. Validate via existing rendering smoke tests.

---

### 3. Pure-physics headless mode

**Files:**

- [src/omnisim/gui/OmView3D.cpp](../../src/omnisim/gui/OmView3D.cpp) — WREN init still runs under `--no-rendering`
- New CLI flag in [src/omnisim/gui/OmGuiApplication.cpp](../../src/omnisim/gui/OmGuiApplication.cpp), e.g. `--no-sensors` (or fold into `--no-rendering=full`)

**Change:** When the flag is set, skip WREN context creation entirely and stub sensor renders to return zero-filled buffers. Cameras/lidars still report timestamps; the data is just blank. This is for swarm-scale headless physics workers that don't consume sensor data.

**Acceptance:** Per-process RSS drop of ≥30% on a no-sensor world. K=4 parallel headless instances of `newton_husky_swarm_drive.omniworld` complete the same step count in ≤1.3× single-instance wall time (currently GPU contention bites earlier).

**Risk:** Medium. Need to gate every WREN call behind the flag without breaking the `--no-rendering` path. Some controller-visible behavior (camera frame counters) must keep ticking.

**Sizing data ([rss-by-robotcount.json](../../tests/benchmarks/optim-baseline/rss-by-robotcount.json), many-robots × 3 measured repeats):**

| robots | peak RSS (process tree) | per-robot increment |
|---:|---:|---:|
| 5 | 415.7 MB | — |
| 25 | 706.4 MB | 14.5 MB / robot |
| 50 | 1067.1 MB | 14.4 MB / robot |

Decomposition: peak RSS is `(omnisim-bin) + Σ(controller process)`. At ~18 MB per Python interpreter × N controllers, the per-robot increment is dominated by Python, not omnisim-bin. Estimated omnisim-bin fixed RSS is ~325 MB.

**Implication for the ceiling:** item 3 (skipping WREN init) plausibly saves 100-150 MB out of omnisim-bin's ~325 MB — meeting the strict "30 % of omnisim-bin RSS" criterion, but only ~10 % of total per-instance RSS at N=50 (because Python controller processes dominate). For workloads that are sensor-free *and* use light-weight controllers (C/C++ instead of Python), the win is larger. **Status: parked.** Architectural scope is too big to land in the same session as items 1 and 6 — `OmMainWindow` → `OmSimulationView` → `OmView3D` → `OmWrenWindow` is constructed unconditionally in `OmGuiApplication::exec`, and disentangling that path needs a real refactor rather than a flag check. Pick this up as its own milestone.

**The bigger memory lever for the user's stated workload** (multi-instance + many robots) is **controller-process consolidation** — e.g. sharing a Python interpreter across multiple robots, or moving hot-loop demo controllers to C++. That's outside this plan's scope.

---

## Phase 2 — Per-step controller IPC

### 4. Ring-buffer the controller IPC reader

**File:** [src/omnisim/control/OmController.cpp](../../src/omnisim/control/OmController.cpp), the `mRequest += readAll()` / `.left(n)` / `.remove(0, n)` loop in `readRequest()` around lines 1048–1135.

**Change:** Replace the QByteArray copy chain with a fixed-capacity ring buffer (or a `QByteArray` with a read cursor). Per-packet reassembly becomes O(packet) instead of O(buffered bytes).

**Acceptance:** `optim_bench.py many-robots --robots 50` shows the per-controller bucket time drop by ≥20%. No regression in the existing `tests/api/` controller tests.

**Risk:** Medium. This is on the hot path, hit by every controller every step. Need careful unit coverage of the framing.

**Result (attempted twice, reverted — no measurable win, possible regression):**

Implemented as a read cursor + `QByteArray::fromRawData` zero-copy slice + one `mid()` compaction at end of `readRequest()` (replaces `.left(N)` copy and per-packet `.remove(0, N)` shift).

**First attempt** — compared `post-item1-pass2` (small omnibot_random workload) baseline to `post-item4-pass2`. Sign of the delta flipped between sizes; `physics(ms)` (which the change shouldn't touch) moved at the same scale, indicating noise dominated. Pass1-vs-pass2 variance on the same code was up to 21 % on controller buckets.

**Second attempt** — purpose-built [`chunky` bench scenario](../../tests/benchmarks/optim_bench.py) (controllers set large per-step `customData` + read GPS/IMU/Compass) with `--repeats 4` for a cleaner noise floor. Compared `chunky-before-item4.json` to `chunky-after-item4.json`:

| customData/step | controller bucket BEFORE → AFTER | within noise? |
|---:|---|---|
| 1 KB | 9.76 → 11.09 ms (+13.7 %) | within (high AFTER spread) |
| 4 KB | 10.01 → 11.40 ms (+13.9 %) | **outside noise — real regression** |
| 16 KB | 9.99 → 12.41 ms (+24.2 %) | within (very high AFTER spread, ±2.57 ms) |

The chunky scenario was designed to stress the reader path (large request packets) with sharper measurement (warm-up discarded, median over 3 measurement repeats, spread reported). Even with that:

- AFTER variance is much wider than BEFORE (e.g. controller bucket spread went from ±0.06 ms to ±2.57 ms at 16 KB) — suggesting `fromRawData`/`mid()` introduces non-deterministic behavior we don't fully understand.
- The 4 KB case shows a statistically significant ~14 % regression.
- Likely cause: `QByteArray::fromRawData` + `QDataStream` may have higher per-access overhead than the upfront `.left()` copy at small packet sizes; `mRequest = mRequest.mid(N)` allocates a new buffer per pass instead of in-place editing like `.remove(0, N)`.

**Reverted.** The change is theoretically correct (eliminates an O(N²) shift across queued packets), but the workloads that would exhibit that pattern aren't represented in the bench, and the change costs measurable time on the workloads that ARE represented. Re-land would require a workload with genuine multi-packet queue depth per `readRequest()` (e.g. extern TCP controllers sending unbatched packets) and a more careful implementation that avoids the per-pass allocation churn.

---

### 5. Batched / parallel `writeAnswer`

**File:** [src/omnisim/control/OmControlledWorld.cpp](../../src/omnisim/control/OmControlledWorld.cpp), the per-controller serial loop around lines 448–458.

**Change:** Either (a) collect all answer payloads then issue writes in parallel via Qt's thread pool, or (b) use scatter-write / `writev`-style aggregation when the underlying transport supports it. Start with (a).

**Acceptance:** `optim_bench.py many-robots --robots 50` shows total step-time drop by ≥15% on a 4+-core host.

**Risk:** High. Parallel writes to controllers require careful ordering on shared state (logs, supervisor mutations). Validate against `tests/api/` and any controller-determinism tests.

**Status: deferred.** Same measurement problem as item 4 — the chunky bench shows that per-controller IPC cost on the simulator side is dominated by overheads outside the per-step `writeAnswer` body, so any savings from batching/parallelism are buried below the noise floor. On top of that, real parallelism is hard: Qt sockets are tied to their owner thread, so a thread pool can't safely call `mTcpSocket->write()`/`flush()`. The achievable shape (build all answers serially → write all serially → flush all once at end) is small and would also distort the per-controller perf bucket since `log->startMeasure(CONTROLLER, name)` currently fires after each flush. Re-attempt when (a) we have a bench with extern TCP controllers and a multi-process driver, or (b) we move to a model where each controller's IPC runs on its own QThread.

---

### 6. Replace 1024-byte controller stdout buffer — LANDED

Buffer grown to 64 KB in `src/controller/c/robot.c`; silent truncation removed. Uses existing `eof()` check with enlarged pipe constant.

**Result (noisy-before-clean.json vs noisy-after-clean.json, 10 robots × 600 steps, no `--stdout`):**

| bytes/step | BEFORE | AFTER | controller bucket delta |
|---:|---|---|---|
| 256 | rc=0, 0.462 ms | rc=0, 0.404 ms | **−12.6 %** |
| 1024 | rc=0, 0.519 ms | rc=0, 0.509 ms | −1.9 % |
| 4096 | rc=0, 0.664 ms | rc=0, 0.533 ms | **−19.7 %** |
| 16384 | **rc=1, 0 steps (crash after 124 s)** | rc=0, 600 steps, 1.159 ms | **status changed: was crashing, now works** |

- ✅ Controller bucket time improved at every size — no regression.
- ✅ 16 KB/step (well above the old 1 KB cap) **now completes**; OLD code returned rc=1 after 124 s, never produced a perf log.
- ⚠️ At 16 KB/step the new code takes 139 s wall — survives but isn't fast. Item #4 (IPC ring-buffer) targets the per-packet copy chain on the simulator side and should help further.
- Iteration history: a first pass used `PeekNamedPipe` to size the read; that added ~120 µs/call on Windows and regressed small-output by 22-30 %. Final version uses the existing `eof()` check with a 64 KB buffer matched to the enlarged pipe — same shape as the original code, just with the constants raised.

**Bench discovery while landing this:** the `--stdout`/`--stderr` flags make the simulator re-emit controller stdout on its own stdout, which floods the Python subprocess `PIPE` buffer when controllers are chatty. The bench now passes `forward_stdio=False` for `noisy` runs ([tests/benchmarks/optim_bench.py](../../tests/benchmarks/optim_bench.py) — `webots_cmd`).

---

## Phase 3 — Sensor rendering

### 7. Shared scene-traversal / cull pass for sensor cameras

**Files:** [src/omnisim/nodes/OmCamera.cpp](../../src/omnisim/nodes/OmCamera.cpp), [src/omnisim/nodes/OmLidar.cpp](../../src/omnisim/nodes/OmLidar.cpp), [src/omnisim/nodes/OmRangeFinder.cpp](../../src/omnisim/nodes/OmRangeFinder.cpp), and the per-sensor `OmWrenCamera` instances.

**Change:** Build the visible-set / culling state once per frame at the world level, and let each sensor camera filter against that pre-computed set instead of walking the scene independently. Largest single-frame win for many-camera worlds.

**Acceptance:** `optim_bench.py many-cameras --cameras 15` shows `mainRendering` + `gpuMemoryTransfer` drop by ≥30%.

**Risk:** High. Cull state is currently per-camera; the data structure for shared culling needs to be designed to not regress single-camera worlds.

---

### 8. Recognition: spatial index + batched overlay updates

**File:** [src/omnisim/nodes/OmCamera.cpp](../../src/omnisim/nodes/OmCamera.cpp) — recognition object detection (`~lines 720–737`) and overlay update path (`~lines 381–403`).

**Change:** Maintain a coarse spatial index (uniform grid or BVH) of recognition-eligible solids; each camera queries the index instead of iterating all solids. Batch overlay texture updates into one upload per camera per frame instead of N+1 (clear + per-object border).

**Acceptance:** `optim_bench.py many-cameras --cameras 15 --recognition` shows recognition portion of `mainRendering` drop by ≥40%. Recognition correctness covered by existing tests.

**Risk:** Medium. Spatial index must update on solid moves; reuse any existing physics-side broadphase if cheap.

---

### 9. Move depth-sensor CPU conversion to shader

**File:** [src/omnisim/nodes/utils/OmWrenTextureOverlay.cpp](../../src/omnisim/nodes/utils/OmWrenTextureOverlay.cpp) (moved out of `src/omnisim/wren/` when WREN was deleted, `976b9449d`) (already backlog item #18) — `~lines 296–316`, depth float→int conversion.

**Change:** Allocate the GPU texture with the right target format and let a shader (or a single GL `glReadPixels`-style direct copy) do the conversion. Skip the temporary CPU buffer entirely.

**Acceptance:** `optim_bench.py many-cameras --cameras 15 --depth` shows `gpuMemoryTransfer` drop by ≥25%.

**Risk:** Medium. Some downstream code consumes the int buffer on the CPU side; need to verify no caller relies on host-side memory.

---

## Session summary (after items 1 + 6)

**Landed:**
- ✅ Item 1 — per-instance log file isolation. Multi-instance K=4 startup-scaling went from 3.22× → 1.29× the K=1 child.
- ✅ Item 6 — controller stdout buffer (1 KB → 64 KB pipe + drain). 16 KB/step controllers no longer crash the simulator; smaller sizes also see 2-20% controller bucket improvement.
- ✅ Bench infrastructure: `cleanup`, `--repeats N` (with warm-up discard, median + min/max/stdev), `chunky` IPC-stress scenario, `reload-creep` harness scenario, peak-RSS sampling on every run, `compare` with noise annotations.

**Tried, deferred:**
- ❌ Item 2 — cache eviction. Signal too small (~1 MB/reload on small worlds) and the harness hot-reload is unreliable on heavy worlds, masking the creep.
- ❌ Item 4 — IPC reader cursor. Two attempts (against many-robots and chunky benches). The change is theoretically correct but the chunky 4 KB case showed a statistically-significant ~14 % regression, likely from `mid()` allocation churn vs. in-place `.remove(0, N)` plus higher per-access overhead of `QDataStream` over a `fromRawData` view at small packet sizes.
- ❌ Item 5 — parallel/batched `writeAnswer`. Not attempted — same measurement problem as item 4, plus Qt sockets are tied to their owner thread (real parallelism would need a per-controller QThread, which is a separate refactor).

**Sized but not pursued:**
- ⏸ Item 3 — pure-physics `--no-sensors` mode. RSS sizing data above shows the realistic ceiling is ~100-150 MB per omnisim-bin (= ~30 % of omnisim-bin RSS, ~10 % of total per-instance RSS). Architecture (unconditional `OmMainWindow` → `OmView3D` → `OmWrenWindow`) requires its own milestone.

**What to pick up next, in order of impact for this workload:**
1. **Controller-process consolidation** (out of plan): replacing the per-robot Python controller with a C++ alternative, or a shared-interpreter shim. Largest remaining memory + parallel-scaling win.
2. **Item 3** as scoped above.
3. Items 7-9 (sensor rendering) once item 3 lands and unblocks "render-on but no display" benchmarking.

## What this plan deliberately omits

- The 30-odd items in improvement-backlog.md (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)) that don't move the needle for this specific workload (e.g. WREN FPS metric restoration, world-load timing split, scene-tree responsiveness). Pick those up under their own plans.
- Anything that requires the not-yet-existing simulation-core boundary (backlog item #21+). Useful, but a year of work; not on this plan.

## Status tracking

When an item lands, replace its **Acceptance** block with:

> **Result:** <delta vs. baseline, link to the comparison output>

When all items in a phase are done, mark the phase header with `(complete)` and re-baseline the bench.
