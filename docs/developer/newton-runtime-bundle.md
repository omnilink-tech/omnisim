# Bundling the Newton runtime into a release (Newton-capable stock install)

**Audience:** whoever produces an OmniSim release build. **Goal:** a downloaded
install runs the Newton physics backend with **no** manual `pip`/PATH step —
closing the last gap before a stock install actually runs Newton on an end-user
box. (`physicsBackend` parses but selects nothing: omit it or write `"newton"`;
`"ode"` means NO physics, and the field inside `WorldInfo` is a load ERROR.) This is the L6 build/runtime layer of
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
                                  #   newton_usd_schemas, onnxruntime
  FIRST_RUN_INSTALL.txt           # bootstrap mode instead of site-packages
```

Footprint (vendor mode, measured): warp-lang ~314 MB (carries its own slim CUDA
subset — there is no separate multi-GB CUDA toolkit to ship), usd-core (`pxr`)
~48 MB, newton ~33 MB, mujoco_warp ~9.5 MB (the `SolverMuJoCo` path the frictional
pinch grasp uses), plus the CPython runtime → **~600 MB measured at warp 1.14**.
Also vendored (added 2026-08-19): `newton-usd-schemas` 0.5.0 (~115 KB, Apache-2.0,
zero deps) — newton's codeless USD schema plugin, without which `add_usd`
hard-fails (`require_newton_usd_schemas` raises), so USD import needs it bundled.

## The bundle is also the CONTROLLER interpreter

⚠ This is not only the physics runtime. The engine embeds CPython for Newton
(it loads `python3XX.dll` from its own directory), but it **spawns a separate
`python` per Python controller**, resolved from `PATH`
(`OmLanguageTools::pythonCommand`). Two of the launch paths put this bundle
first on that `PATH` — `scripts/dev/headless_runner.py` (so every
`python -m omnisim run-headless`) and `scripts/dev/omnisim_run_agent.py` (so
every `run-agent`), both of which also point `PYTHONPATH` at
`newton-runtime/site-packages`. `launch.bat` and the installer's shortcut do
not (`launcher.c` adds only the engine dir, `cpp/` and msys `usr/bin`), so
those give the controller the user's system `python`.

That split is why a gap here hides: a developer's own `python` can have a
package the shipped bundle lacks, and nothing in `python -m omnisim` ever
touches the bundle's copy. Measured 2026-09-11 on a probe controller launched
by the engine — `sys.executable =
msys64\mingw64\bin\newton-runtime\python.exe`, `numpy` 2.5.3, `onnxruntime`
**ModuleNotFoundError** — while the same clone's `python` had onnxruntime
1.26.0. 26 shipped controllers `import onnxruntime` (every `*_deploy` /
`*_mimic` under `projects/policies/research/controllers`, plus the
`anypick_cam` and `omniarm6_bin_picking` demos), so on a clean clone every one
of them refused to run its policy.

Fixed by vendoring `onnxruntime` 1.26.0 (42 MB installed against a 694 MB
bundle, +6%; MIT; a platform wheel like warp/mujoco/numpy already are). It is
in `DEPLOY_STACK` in `newton_runtime_pins.py` and in the bundler's
`VERIFY_IMPORTS`, and `python -m omnisim doctor` reports it on the `policies`
row.

### `omnisim_bridges` is source-shipped, NOT vendored (resolved 2026-09-11)

`omnisim_bridges` (`packages/omnisim-bridges/`) had the same failure signature
and the **opposite** remedy. Probe controller, launched by the engine under
`run-headless`, before the fix:

```
executable                  O:\omnisim\msys64\mingw64\bin\newton-runtime\python.exe
PYTHONPATH                  ...\lib\controller\python;...\newton-runtime\site-packages
import omnisim_bridges      ModuleNotFoundError
after the _omnilink_relay bootstrap
import omnisim_bridges      OK -> O:\omnisim\packages\omnisim-bridges\src\...
```

That second line is the whole story: the package was **already reachable** from
the bundled interpreter. `_omnilink_relay/__init__.py` puts
`packages/omnisim-bridges/src` on `sys.path` — but the two bridges imported
`omnisim_bridges` *above* that import, so the bare `except Exception` swallowed
a `ModuleNotFoundError` and installed stubs. `omnilink_arm_bridge` showed the
mechanism cleanly by failing **only half-way**: `intent_router` (imported before
the relay) degraded while `intents` (imported after it) worked.

So it is **not vendored**, deliberately:

- it is an *editable* install precisely so edits to the package take effect
  immediately — a wheel in the bundle would shadow the tree with a stale copy,
  which is worse than the bug it fixes;
- it is pure Python with no platform wheel, unlike warp/mujoco/numpy/onnxruntime;
- its source already ships **in the installer**
  (`scripts/packaging/files_core.txt`: `packages/omnisim-bridges [recurse]`),
  so a tree-relative path resolves on an installed box too.

Both halves of the fix are therefore path-based:

1. `headless_runner.py` and `omnisim_run_agent.py` add
   `packages/omnisim-bridges/src` to the controller `PYTHONPATH` beside the
   bundle's `site-packages`. (`omnisim_run_agent.py` prepended the bundled
   python while setting **no** `PYTHONPATH` at all — so `run-agent` handed
   controllers an interpreter that could not import numpy either. Fixed in the
   same change.)
2. Every bridge controller bootstraps the same path itself, above its first
   `omnisim_bridges` import, so launch paths we do not control (`launch.bat`,
   the installer shortcut) work on a clean clone with no editable install.

And the silence is closed at both ends: the `except Exception` is narrowed to
`ImportError` (a genuine error *inside* the package is no longer mistaken for
the package being absent), a stub prints an unmissable banner naming
`sys.executable`, the path tried and the fix, and `doctor` grew a `bridges`
row. The banner is for the GUI console: controller stdout **and** stderr reach
neither `omnisim_log.txt` nor the `run-headless` capture (measured), which is
why `doctor` is the headless channel.

Anything added to this bundle should be checked against what a *controller*
needs, not only what physics needs — and checked against whether it should be
in the bundle at all, or source-shipped like this one.

### One controller interpreter on Windows (resolved 2026-09-22)

`run-world`, `test-world`, `run-headless`, `run-agent`, `launch.bat` and the
native desktop launchers now prefer `newton-runtime/python.exe`. The bundled
public SDK and transport dependencies are pinned in
[`requirements-omnilink.txt`](../../scripts/packaging/requirements-omnilink.txt).
Linux bootstrap installs the same pins into the Python selected for controllers.
The source-shipped `omnisim_bridges` package remains tree-relative.

The engine reads `python3XX._pth` beside its DLL. Standalone controllers instead
read `Lib/site-packages/omnisim_runtime.pth`, whose relative path exposes the
same vendored wheels. This works after moving the installation and does not
require a developer's `PYTHONPATH` or a private OmniLink checkout.

Check the actual controller interpreter with `omnisim.bat doctor --omnilink`.
It fails when SDK, transport or capture imports are missing or incompatible.
It does not authenticate an account or call a model. The public SDK transport
can also be checked without an account:

```powershell
.\msys64\mingw64\bin\newton-runtime\python.exe -I scripts/dev/verify_omnilink_sdk.py
```

This uses scripted loopback HTTP responses and the real installed SDK. It is
an integration check, not a model-performance test.

To repair an existing Windows bundle without replacing physics dependencies,
close running OmniSim instances, then use a full CPython installation matching
the engine ABI (the tool checks the version):

```powershell
python scripts/packaging/bundle_newton_runtime.py --controller-deps-only --cpython-home C:/path/to/Python312 --verify
```

A full vendor build includes these dependencies automatically. Adding packages
to a developer's system Python alone does not install them for controllers.
A rebuilt installer still needs installation testing before release.

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

## ⛔ Re-vendoring while an engine is running CORRUPTS the bundle

`pip install --upgrade --target` **rmtrees each existing package directory**
before reinstalling it. A running `omnisim-bin.exe` holds the bundle's compiled
extensions open, so on Windows that delete fails *part way through* and leaves a
package that imports but does not compute. Measured 2026-09-11: **341 files
removed from the bundle's numpy** before pip hit
`_umath_linalg.cp312-win_amd64.pyd` and died; the bundle had to be hand-repaired
from the wheel. The corruption then surfaces later as a confusing `ImportError`
somewhere unrelated.

This matters because AGENTS.md §0 and §2 tell agents to run
`make -C src/omnisim bundle-newton-runtime` as a bootstrap step. So the bundler
now **refuses by default**:

```
$ python scripts/packaging/bundle_newton_runtime.py
REFUSING to re-vendor the bundle: omnisim-bin is running.
    omnisim-bin.exe pid 8260
    omnisim-bin.exe pid 15328
...
Override only if you are certain: --allow-running-engines
```

The check runs before anything writes to the bundle. If processes cannot be
enumerated at all it warns loudly rather than silently proceeding. Stop the
engines and re-run — do **not** kill an `omnisim-bin` you did not spawn.

`pip_install(..., upgrade=False)` is the surgical path (`--no-deps`, no rmtree)
for adding one package to an otherwise-good bundle.

## Verification

- `--verify` (also run by `make bundle-newton-runtime`) launches the **staged**
  interpreter under a scrubbed environment (no `PYTHONPATH`/`PYTHONHOME`, reduced
  `PATH`, `PYTHONNOUSERSITE=1`) and asserts `VERIFY_IMPORTS` succeed — i.e.
  it proves the clean-box story on the build box.
- It then runs `INTEGRITY_PROBES`, which **exercise the compiled extensions**
  rather than only importing names: `numpy.linalg.det` and `.inv` (both route
  through `_umath_linalg`, the exact `.pyd` that was locked in the incident
  above) and `numpy.fft` (`_pocketfft_umath`). A half-deleted package imports
  fine and computes nothing, so name-only verification cannot see it — and
  `numpy` was not even in `VERIFY_IMPORTS` until 2026-09-11. On failure the
  bundler says the bundle is **DAMAGED** and how to repair it, instead of
  emitting a generic import error that sends the next person hunting their own
  environment.
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

The tooling (bundler + `make` target + `windows_distro` guard) **landed on
`main` 2026-06-09**. Mechanism and the bundler's staging output are **verified**
on the dev box: the staged `python.exe` imports warp 1.13.0 / newton 1.2.0 /
mujoco_warp / pxr under `-E -S` isolation with only bundle paths on `sys.path`.
**Pending the release box:** producing the actual vendored installer against a
freshly-built binary and confirming a PATH-stripped binary brings Newton up from
the bundle (the script's `--verify`/`--verify-binary` is that gate).
