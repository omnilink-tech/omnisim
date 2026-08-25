# Bundling the Newton runtime into a release (Newton-capable stock install)

**Audience:** whoever produces an OmniSim release build. **Goal:** a downloaded
install runs the Newton physics backend with **no** manual `pip`/PATH step —
closing the last gap before the `physicsBackend "auto"` default actually means
Newton on an end-user box. This is the L6 build/runtime layer of
[default-flip-plan.md §4.3.1](default-flip-plan.md).

> ⚠ **2026-08-08 — THE PREMISE OF THIS DOC CHANGED: there is no ODE to fall back
> to.** `bdc02139` deleted `src/ode` + `include/ode` (106,283 lines), and Newton
> with `SolverMuJoCo` is now the **only** physics backend, in a CPU (`mj_step`) and
> a batched-GPU (`mujoco_warp`) profile. Wherever this doc said "without the bundle
> you silently get ODE", read: **without the runtime there is no physics backend at
> all** — a missing or broken Newton runtime is a hard failure, not a silent
> downgrade. Every packaging and platform fact below still holds unchanged (the
> bundler is Windows-only, Linux resolves the system `python3`, venvs are invisible
> to the embedded interpreter, `BUNDLE_NEWTON ?= 1`); only the *consequence of
> absence* changed. Campaign record:
> [ode-retirement-campaign.md](ode-retirement-campaign.md).

> **Update 2026-06-23 — `make release` now bundles the runtime BY DEFAULT.**
> Bundling is no longer a separate opt-in step: `make release` invokes the
> bundler automatically (`BUNDLE_NEWTON ?= 1`, `577ff609`) and **idempotently**
> (`3de05aa3` — it skips when `newton-runtime` is already staged, so repeated
> `build_omni.bat` rebuilds neither re-copy the ~600 MB nor fail). Opt out with
> `make release BUNDLE_NEWTON=0`. So a stock **release** no longer silently runs
> ODE for lack of the runtime. **Caveat (still true):** a from-source clone or a
> `make debug` that *doesn't* stage the bundle has no Newton runtime — which in
> 2026-06 meant a silent ODE fallback and, since `bdc02139`, means **no physics
> backend at all** — the bundle ships with releases, not with an arbitrary source
> build. Pair with
> `OMNISIM_REQUIRE_NEWTON=1` (`cfb11d06`) to make a missing/failed runtime a loud
> fatal instead of a silent downgrade, and confirm the real
> `[OmNewtonBackend] world finalised (solver=...)` line (the silent-fallback init
> bug is fixed in `6a459f84`).

## Why a bundle is needed (and why it's packaging-only)

`OMNISIM_WITH_NEWTON` is ON by default, so a build *links* the Newton backend.
But Newton runs through an **embedded CPython** (`OmNewtonBackend` calls a bare
`Py_InitializeEx(0)` — no `Py_SetPythonHome`, no `PyConfig`) that then does
`import warp` / `import newton`. So whether Newton is available is purely a
function of which `python3XX.dll` the process loads and what is on that
interpreter's `sys.path`. On a dev box that resolves to the developer's CPython
(whose user-site has `warp`); on a clean box it resolves to a python without
`warp`, or none → no physics backend (a hard failure since `bdc02139`; before the
ODE deletion this was a silent ODE fallback).

Because the engine never sets the Python home, the fix lives **entirely in the
packaging layer** (no C++/link change): stage a self-contained CPython beside
`omnisim-bin.exe` and drop a `python3XX._pth` next to the loaded DLL. CPython's
isolated path config then builds `sys.path` strictly from that file — the
registry, `PYTHONHOME`, and per-user site are all ignored — so the embedded
interpreter deterministically resolves the bundled `site-packages`. This is the
standard "Windows embeddable package" redistribution mechanism.

## Producing a Newton-capable release

```bash
# Build + bundle in one step: `release` runs the bundler by default
# (BUNDLE_NEWTON ?= 1), idempotently — Newton + wgpu are ON by default.
make -C src/omnisim release

# Then package as usual — windows_distro.py ships the whole msys64/ tree
# recursively, so the staged bundle is included automatically, and it
# asserts the bundle is present (see "Package-time guard" below).
```

As of 2026-06-23, `make release` runs `bundle-newton-runtime` **by default**
(`BUNDLE_NEWTON ?= 1`, `577ff609`) and **idempotently** (`3de05aa3`): the step
is skipped when `$(TARGET_PATH)/newton-runtime` is already staged, so a developer
who runs `build_omni.bat` repeatedly never re-copies the ~600 MB and the build
never fails on an already-bundled tree. Run it standalone for a one-off
vendoring, or opt out for a slim build:

```bash
make -C src/omnisim bundle-newton-runtime    # standalone, same idempotent staging
make -C src/omnisim release BUNDLE_NEWTON=0   # skip the bundle (slim; NO physics runtime staged)
```

`make debug`/`profile` do **not** bundle, and a from-source clone without the
runtime has **no physics backend at all** — the bundle is a release artifact.
Knobs:

| Variable | Default | Effect |
|---|---|---|
| `BUNDLE_NEWTON` | `1` (in `release`) | `1` = run the bundler as part of `make release` (idempotent). `0` = skip it (slim build; if no runtime is present at runtime the engine has no physics backend and fails hard). |
| `BUNDLE_MODE` | `vendor` | `vendor` = pip-install warp/newton into the bundle now (offline installer). `bootstrap` = stage only CPython + a first-run installer (slim installer, network once). |
| `PYTHON_BUNDLER` | `python` | The python used to *run* the bundler script (any python3; unrelated to the staged runtime). |

You can also run the bundler directly for more control:

```bash
python scripts/packaging/bundle_newton_runtime.py \
    --target msys64/mingw64/bin --mode vendor --verify
python scripts/packaging/bundle_newton_runtime.py --inspect   # report only
```

## What gets staged

Beside `omnisim-bin.exe` (`msys64/mingw64/bin/`):

```
python3XX.dll                     # loader DLL, matches the binary's import
python3XX._pth                    # isolated path config -> the bundle below
newton-runtime/
  python.exe  Lib/  DLLs/         # self-contained CPython
  site-packages/                  # vendor mode: warp, newton, mujoco_warp, pxr (usd),
                                  #   newton_usd_schemas
  FIRST_RUN_INSTALL.txt           # bootstrap mode instead of site-packages
```

Footprint (vendor mode, measured): warp-lang ~314 MB (carries its own slim CUDA
subset — there is no separate multi-GB CUDA toolkit to ship), usd-core (`pxr`)
~48 MB, newton ~33 MB, mujoco_warp ~9.5 MB (the `SolverMuJoCo` path the frictional
pinch grasp uses), plus the CPython runtime → **~600 MB measured at warp 1.14**.
Also vendored (added 2026-08-19): `newton-usd-schemas` 0.5.0 (~115 KB, Apache-2.0,
zero deps) — newton's codeless USD schema plugin, without which `add_usd`
hard-fails (`require_newton_usd_schemas` raises), so USD import needs it bundled.

wgpu is already handled: the Makefile copies `wgpu_native.dll` next to the binary
when built with `WGPU_NATIVE_HOME`, and it ships in the same recursive `msys64/`
copy. No separate step.

## The version-match trap

The staged python **must** match the version the binary links. A binary built
today imports `python312.dll`; rebuilt under the current Makefile (PYTHON_HOME →
Python314) it imports `python314.dll`. A mismatch is the #1 silent break, so the
bundler **autodetects** the version from the binary's PE import table (pure
Python, no objdump dependency) rather than hardcoding it. `--inspect` prints what
the binary needs vs what is staged.

## Verification

- `--verify` (also run by `make bundle-newton-runtime`) launches the **staged**
  interpreter under a scrubbed environment (no `PYTHONPATH`/`PYTHONHOME`, reduced
  `PATH`, `PYTHONNOUSERSITE=1`) and asserts `import warp, newton` succeed — i.e.
  it proves the clean-box story on the build box.
- `--verify-binary` additionally runs `omnisim-bin.exe` on the Newton smoke world
  and checks for the `[OmNewtonBackend]` runtime-up line (the same signal the
  pre-push gate's `--require-newton` uses).

## Package-time guard

`windows_distro.py` checks for `newton-runtime/site-packages/warp` + a
`python3XX._pth` after staging the tree. If the bundle is absent it prints a
prominent warning (the installer would otherwise ship with **no working physics
backend at all** — the "never silently degrade" default-flip-plan principle #4;
before `bdc02139` the same gap made the installer *silently* ODE-only). Set
`OMNISIM_REQUIRE_NEWTON_BUNDLE=1` to make a missing bundle a hard packaging
error for the Newton-capable release matrix.

## Troubleshooting "stock install has no physics backend"

1. Is the bundle staged? `make release` stages it by default (idempotently);
   check for `msys64/mingw64/bin/newton-runtime/` and `python3XX._pth`. A
   `make debug`/`profile` or a `release BUNDLE_NEWTON=0` build, or a bare source
   clone, intentionally has no bundle — and therefore no physics backend.
2. Does the `._pth` version match the binary? Run `--inspect`.
3. Is `warp` actually under `newton-runtime/site-packages/` (vendor mode), or did
   it stay in `bootstrap` mode (first-run marker only)?
4. Launch and read the log — and read the **right** signal. The authoritative
   "Newton is active" line is `[OmNewtonBackend] world finalised (solver=...)`,
   **not** an earlier `imports OK` (imports can succeed while the solver still
   fails to bind). `[OmNewtonBackend] import warp failed …` means the bundle isn't
   on the embedded interpreter's path (DLL/`._pth` not beside the binary).
   ⚠ 2026-08-08: the once-per-world `isAvailable() false → ODE` line described a
   downgrade path that no longer exists — since `bdc02139` there is nothing to
   degrade *to*, so a runtime that will not come up is a hard failure. The
   historical silent-fallback init bug — warp's import banner crashing under
   headless `stdout=DEVNULL` → swallowed exception → ODE — was **fixed in
   `6a459f84`**; `OMNISIM_REQUIRE_NEWTON=1` (`cfb11d06`) is still honoured as an
   explicit belt-and-braces assertion.

## Platform status

- **Windows:** supported (proven Newton + wgpu binary and a Win32 wgpu surface).
  The `._pth` mechanism above is Windows/embeddable — **the bundle is a
  Windows-only packaging mechanism.**
- **Linux:** **Newton is PROVEN working (2026-07-12, WSL2 Ubuntu 26.04,
  RTX 5070 Ti, public repo v5.0.0)** — verdict sidecar verbatim:

  ```json
  {"backend":"newton","degraded":false,"finalised":true,"solver":"MuJoCo (mujoco_warp, WorldInfo.newtonSolver)"}
  ```

  There is no Windows gate anywhere in `src/omnisim/physics/`. **The bundler is
  *irrelevant* on Linux — not "pending".** The engine's bare `Py_InitializeEx`
  resolves the **system** `python3`'s `sys.path`, so the whole setup is:

  ```bash
  pip install torch warp-lang newton mujoco mujoco-warp   # into the SYSTEM python3
  ```

  **Gotcha: NOT into a venv** — the embedded interpreter ignores virtualenvs, so
  wheels installed in a venv are invisible to the engine, which then comes up with
  **no physics backend** and fails hard (before `bdc02139` it silently ran ODE).
  Install into the system interpreter (or the one whose `libpython` the
  binary links). Wheel-target safety: Ubuntu 22.04/24.04 (py3.10/3.12) are the
  safest; Ubuntu 26.04/py3.14 works but is wheel-fragile. An NVIDIA/CUDA GPU is
  required only for the batched-GPU `mujoco_warp` profile — without one the
  default `SolverMuJoCo` still runs on the CPU (`mj_step`). Full setup: the
  [quickstart's Linux section](quickstart.md#linux-quickstart-ubuntu) /
  `scripts/install/linux_bootstrap.sh` (v5.1).
- **macOS:** **untested** — no claims made (and `warp`'s Apple support story
  differs from its CUDA one).

## Status (2026-06-09, lane L6)

Mechanism and the bundler's staging output are **verified** on the dev box: the
staged `python.exe` imports warp 1.13.0 / newton 1.2.0 / mujoco_warp / pxr under
`-E -S` isolation with only bundle paths on `sys.path`. **Pending the release
box:** producing the actual vendored installer against a freshly-built binary and
confirming a PATH-stripped binary brings Newton up from the bundle (the script's
`--verify`/`--verify-binary` is that gate).
