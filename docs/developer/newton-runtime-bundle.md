# Bundling the Newton runtime into a release (Newton-capable stock install)

**Audience:** whoever produces an OmniSim release build. **Goal:** a downloaded
install runs the Newton physics backend with **no** manual `pip`/PATH step —
closing the last gap before the `physicsBackend "auto"` default actually means
Newton on an end-user box. This is the L6 build/runtime layer of
[default-flip-plan.md §4.3.1](default-flip-plan.md).

> **Update 2026-06-23 — `make release` now bundles the runtime BY DEFAULT.**
> Bundling is no longer a separate opt-in step: `make release` invokes the
> bundler automatically (`BUNDLE_NEWTON ?= 1`, `577ff609`) and **idempotently**
> (`3de05aa3` — it skips when `newton-runtime` is already staged, so repeated
> `build_omni.bat` rebuilds neither re-copy the ~600 MB nor fail). Opt out with
> `make release BUNDLE_NEWTON=0`. So a stock **release** no longer silently runs
> ODE for lack of the runtime. **Caveat (still true):** a from-source clone or a
> `make debug` that *doesn't* stage the bundle still falls back to ODE — the
> bundle ships with releases, not with an arbitrary source build. Pair with
> `OMNISIM_REQUIRE_NEWTON=1` (`cfb11d06`) to make a missing/failed runtime a loud
> fatal instead of a silent downgrade, and confirm the real
> `[WbNewtonBackend] world finalised (solver=...)` line (the silent-fallback init
> bug is fixed in `6a459f84`).

## Why a bundle is needed (and why it's packaging-only)

`OMNISIM_WITH_NEWTON` is ON by default, so a build *links* the Newton backend.
But Newton runs through an **embedded CPython** (`WbNewtonBackend` calls a bare
`Py_InitializeEx(0)` — no `Py_SetPythonHome`, no `PyConfig`) that then does
`import warp` / `import newton`. So whether Newton is available is purely a
function of which `python3XX.dll` the process loads and what is on that
interpreter's `sys.path`. On a dev box that resolves to the developer's CPython
(whose user-site has `warp`); on a clean box it resolves to a python without
`warp`, or none → silent ODE.

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
make -C src/omnisim release BUNDLE_NEWTON=0   # skip the bundle (slim, ODE-fallback)
```

`make debug`/`profile` do **not** bundle, and a from-source clone without the
runtime still falls back to ODE — the bundle is a release artifact. Knobs:

| Variable | Default | Effect |
|---|---|---|
| `BUNDLE_NEWTON` | `1` (in `release`) | `1` = run the bundler as part of `make release` (idempotent). `0` = skip it (slim build, ODE fallback if no runtime is present at runtime). |
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
  site-packages/                  # vendor mode: warp, newton, mujoco_warp, pxr (usd)
  FIRST_RUN_INSTALL.txt           # bootstrap mode instead of site-packages
```

Footprint (vendor mode, measured): warp-lang ~314 MB (carries its own slim CUDA
subset — there is no separate multi-GB CUDA toolkit to ship), usd-core (`pxr`)
~48 MB, newton ~33 MB, mujoco_warp ~9.5 MB (the `SolverMuJoCo` path the frictional
pinch grasp uses), plus the CPython runtime → **~600 MB measured at warp 1.14**.

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
  and checks for the `[WbNewtonBackend]` runtime-up line (the same signal the
  pre-push gate's `--require-newton` uses).

## Package-time guard

`windows_distro.py` checks for `newton-runtime/site-packages/warp` + a
`python3XX._pth` after staging the tree. If the bundle is absent it prints a
prominent warning (the installer would otherwise *silently* be ODE-only — the
"never silently degrade" default-flip-plan principle #4). Set
`OMNISIM_REQUIRE_NEWTON_BUNDLE=1` to make a missing bundle a hard packaging
error for the Newton-capable release matrix.

## Troubleshooting "stock install still runs ODE"

1. Is the bundle staged? `make release` stages it by default (idempotently);
   check for `msys64/mingw64/bin/newton-runtime/` and `python3XX._pth`. A
   `make debug`/`profile` or a `release BUNDLE_NEWTON=0` build, or a bare source
   clone, intentionally has no bundle and falls back to ODE.
2. Does the `._pth` version match the binary? Run `--inspect`.
3. Is `warp` actually under `newton-runtime/site-packages/` (vendor mode), or did
   it stay in `bootstrap` mode (first-run marker only)?
4. Launch and read the log — and read the **right** signal. The authoritative
   "Newton is active" line is `[WbNewtonBackend] world finalised (solver=...)`,
   **not** an earlier `imports OK` (imports can succeed while the solver still
   fails to bind). `[WbNewtonBackend] import warp failed …` means the bundle isn't
   on the embedded interpreter's path (DLL/`._pth` not beside the binary);
   `isAvailable() false → ODE` is logged once per world. Note the silent-fallback
   init bug — warp's import banner crashing under headless `stdout=DEVNULL` → swallowed
   exception → ODE — is **fixed in `6a459f84`**; for a hard guard, run with
   `OMNISIM_REQUIRE_NEWTON=1` (`cfb11d06`) so a Newton-init failure is a loud fatal
   (non-zero exit) instead of a silent downgrade.

## Platform status

- **Windows:** supported (the only platform with a proven Newton + wgpu binary
  and a Win32 wgpu surface). The `._pth` mechanism above is Windows/embeddable.
- **Linux/macOS:** **pending.** Newton/warp run on Linux, but the embedded
  interpreter there resolves its home differently (`._pth` is Windows-only — use
  `PYTHONHOME`/rpath + a relocatable layout), and the OmniSim Newton/wgpu binary
  on those platforms is not yet proven. Sequenced behind a working non-Windows
  binary; the bundler no-ops with a message off Windows for now.

## Status (2026-06-09, lane L6)

Mechanism and the bundler's staging output are **verified** on the dev box: the
staged `python.exe` imports warp 1.13.0 / newton 1.2.0 / mujoco_warp / pxr under
`-E -S` isolation with only bundle paths on `sys.path`. **Pending the release
box:** producing the actual vendored installer against a freshly-built binary and
confirming a PATH-stripped binary brings Newton up from the bundle (the script's
`--verify`/`--verify-binary` is that gate).
