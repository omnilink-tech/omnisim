# Granular / CUDA — lane L8 tracker

**Lane:** L8 (granular / CUDA) of the [parallel-lanes split](migration-parallel-lanes.md).
**Owns:** the granular subsystem — `src/omnisim/nodes/WbGranularGroup.{cpp,hpp}`,
the CUDA compute layer `src/omnisim/compute/cuda/**`, the `GranularGroup` PROTO,
the granular worlds + tests under `tests/cuda/` and `tests/granular-spike/`.
**Status (code-verified 2026-06-09):** **CUDA M2 is complete and demoed.** The
headline target — interactive granular at the 50 000-pebble scale — is *met and
exceeded*. Open work is M3 (engine coupling under Newton + zero-copy render +
dynamic-state serialization), not the solver itself.

> **Why this doc exists.** `engine-migration-plan.md` §13.7 still lists granular
> as "🟡 NOT STARTED … depends on CUDA M2 … ~50 000 pebbles target," and
> `WbGranularGroup.hpp` cites two plan docs that were never written
> (`granular-physics-plan.md`, `cuda-compute-infrastructure-plan.md`). Both are
> stale: the work shipped (see the commit trail below) and the videos
> `docs/media/videos/cuda_showcase.*` demo it. This
> is the canonical L8 tracker; §13.7 should be reconciled to point here by the
> integrator (per parallel-lane protocol #5, lanes don't edit §8.x/§13 directly).

---

## What is actually built (M2 — done)

The GPU granular solver lives as an NVRTC-compiled kernel string
(`kPhysicsKernelSrc`) inside `WbGranularGroup.cpp`. NVRTC (not a `.cu` file) is a
deliberate choice: the rest of `omnisim-bin` is mingw-built, and nvcc needs
MSVC's `cl.exe` as host compiler on Windows → ABI-incompatible `.obj`. NVRTC
compiles the kernel to PTX at startup and the driver API loads it — no MSVC in
the link line.

Landed increments (commit trail, oldest first):

| Commit | Increment |
|---|---|
| `aae2ab36` | `GranularGroup` PROTO skeleton — parses, occupies scene tree, reserves the device buffer |
| `32ffa8dd` | real gravity-integration kernel via NVRTC + Driver API |
| `b13f7b79` | brute-force O(N²) collisions — 320× over CPU at N=400 |
| `ec5566ce` | Coulomb-capped tangential friction in the contact response |
| `bb4449c0` | WREN host-readback rendering — particles visible |
| `dc161ba0` | ENU-aware kernel (works in ENU/NUE/EUN) + scatter demo |
| `196ce720` | `boundsHalfWidth` PROTO field + Husky-meets-spheres world |
| `6fc93bd3` | one-way coupling — Husky pushes spheres |
| `21020e3b` | two-way coupling — robot pushes balls **and** balls push robot |
| `44488615` | uniform-grid broadphase — 100× more particles, real-time |

**Measured** (`tests/cuda/bench_results.md`, RTX 3060 Laptop, sm_86, with the
uniform-grid broadphase): 50 000 particles at **2.48 ms/step**, 100 000 at
**4.50 ms/step** — at the 16 ms basic-step budget that is **3.5× real-time at
100k** with headroom. The §13.7 "50 000 target / depends on M2 / 4–6 wk" row is
satisfied.

**Force law** (shared with the CPU reference, below): per inner substep, for
each overlapping pair — normal penalty spring `k·overlap`, normal damping
`−k_d·v_n`, tangential viscous friction capped at `μ·k·overlap`
(Coulomb); then semi-implicit-Euler integrate with floor + box-wall clamp and
restitution. 8 substeps per outer step keep the explicit springs stable
(`dt_inner = 2 ms < sqrt(m/k)`). The broadphase is a uniform grid of
`cellSize = 2·radius`, rebuilt once per outer step (particles move ≪ a cell per
step), walked over the 27 neighbour cells.

---

## Open work (M3 and adjacent)

Ordered by how much it blocks the engine migration:

1. **Newton coupling — the migration-relevant gap (L1 seam, do NOT fix in L8).**
   Two-way coupling is **ODE-only**: `WbGranularGroup::collectColliders` filters
   on `solid->body()` (an ODE `dBody`) and `onPhysicsStepStarted` pushes reverse
   forces via `WbSolid::addForceAtPosition` → ODE. In a `physicsBackend "newton"`
   world a robot link has no ODE body, so **particles won't couple to it**. The
   fix belongs to L1 (it needs a backend-neutral "apply force at world point" +
   a body-presence query that both backends answer). **L8 must not edit
   `WbSolid`/`WbNewtonBackend`** — flag it here and hand to L1 via the tracker
   (parallel-lane protocol #3/#4). Until then granular demos should pin
   `physicsBackend "ode"`.

2. **Zero-copy render (the old "M1").** Today rendering is a per-step
   device→host position readback feeding one WREN transform per particle. The
   plan deferred GL/CUDA interop until the renderer reached wgpu — Phase δ has
   now landed (`6a22f9d6`, Newton→wgpu interop demonstrated), so the successor
   is **wgpu/CUDA interop**: keep particle positions on the GPU and feed them
   straight into an instanced wgpu draw. Touches L2/L3 render surfaces → a
   convergence step, not a solo L8 edit.

3. **M3 dynamic-state serialization.** `WbGranularGroup.hpp` notes the static
   fields serialize via the standard `.wbt` path but the *dynamic* particle
   state does not — needed for save/restore and deterministic replay.

4. **Solver fidelity tail.** Exact contact-point force application (currently the
   bounding-sphere centre); the collider's real velocity in the damping term
   (currently assumed zero); per-particle sleep/auto-disable.

Not started: CUDA M1-proper for non-render compute interop, granular Tier
beyond the pile demo (cohesion/SPH-style), multi-`GranularGroup` scenes.

---

## Verification

Granular had **only** a GPU perf sweep (`bench_granular_group.py`) and a
"does the `.wbt` load" check (`test_granular_group_load.py`) — both need a GPU
and the full engine, and neither asserts the contact response is *physically
correct*. That gap is now closed headlessly:

- **`tests/cuda/granular_core_reference.hpp`** — a dependency-free CPU mirror of
  the kernel's force law (brute-force neighbour scan; the grid is only an
  accelerator, so the physics is what it pins down).
- **`tests/cuda/granular_core_selfcheck.cpp`** — drops 256 particles into a box
  and asserts: state stays finite, nothing escapes the floor/walls, deepest
  penetration < 0.5·radius, energy peaks during free-fall then dissipates to
  <5% of peak, the pile rests on the floor and reaches a stable height.

Run (no GPU, no engine build — stock C++17; on Windows use the MSYS2 mingw
toolchain at `C:\msys64\mingw64`):

```sh
g++ -std=c++17 -O2 tests/cuda/granular_core_selfcheck.cpp -o granular_selfcheck
./granular_selfcheck      # exit 0 = PASS
```

Verified PASS 2026-06-09 (8/8 checks): settled `up min=0.0200 max=0.0526`,
KE peak `0.0355` → end `1.2e-7`, max penetration `0.00025 m` (0.012·radius).
Keep the reference in lock-step with `kPhysicsKernelSrc`: if the kernel's force
law changes, change the header too and re-run this gate.

GPU-side reproduction (needs `OMNISIM_WITH_CUDA=ON` + an NVIDIA GPU):

```sh
make -C src/omnisim OMNISIM_WITH_CUDA=ON release
python tests/cuda/bench_granular_group.py
```

---

## Lane contract (L8)

Per [migration-parallel-lanes.md](migration-parallel-lanes.md): edit only L8's
owned files; **never** touch `WbSolid`/`WbNewtonBackend` (L1) or the render
surfaces (L2/L3) — coordinate the Newton-coupling and wgpu-interop seams through
the owning lane. Only L6 edits the `Makefile`. Commit small, path-scoped, and
atomic. Self-verify with the headless self-check above.
