# OmniSim Validation Playbook

This guide maps common code changes to the smallest validation lane that still gives useful signal.

## Validation Lanes

### Lane 1: Build-only sanity

Use this to catch compile and link errors quickly.

Examples:

```bash
python -m omnisim build gui
python -m omnisim build renderer
python -m omnisim build controller-libs
```

### Lane 2: Fast smoke

Use this as the default post-build check for most simulator changes.

```bash
python -m omnisim test-smoke
python -m omnisim test-smoke --nomake
```

The smoke set currently includes:

- `resources/projects/worlds/empty.omniworld`
- `tests/api/worlds/accelerometer.omniworld`
- `tests/physics/worlds/contact_points.omniworld`
- `tests/rendering/worlds/normals.omniworld` (currently `"skip": true` in `tests/smoke/smoke_worlds.json` — does not run; see its `skip_reason` for the offscreen-Camera framebuffer bug)
- `tests/protos/worlds/template_deterministic.omniworld`

### Lane 3: One-world targeted run

Use this when the change is localized and you already know the relevant world.

```bash
python -m omnisim test-world tests/api/worlds/accelerometer.omniworld --nomake
python -m omnisim run-headless tests/api/worlds/accelerometer.omniworld
```

### Lane 4: One existing test group

Use this when the change clearly belongs to one subsystem.

```bash
python -m omnisim test-group api --nomake
python -m omnisim test-group physics --nomake
python -m omnisim test-group rendering --nomake
python -m omnisim test-group protos --nomake
```

### Lane 5: Benchmark and performance comparison

Use this when the change affects load time, physics throughput, rendering cost, or sensor cost.

```bash
python -m omnisim profile-world tests/rendering/worlds/normals.omniworld
python -m omnisim benchmarks --nomake
```

### Lane 6: Full regression

Use the legacy full harness when a change is broad or risky.

```bash
python tests/test_suite.py
python tests/test_suite.py --group physics
python tests/test_suite.py tests/protos/worlds/template_deterministic.omniworld
```

## Headless Contract

The developer CLI (`run-headless`) uses:

```text
--minimize --batch --no-rendering --mode=fast --stdout --stderr
```

`--minimize` (not `--no-window`) is the safe headless default: it keeps the main window in a normal Qt event loop while hiding it via the OS taskbar, with `--batch --no-rendering` keeping the run cheap. `--no-window` is deliberately avoided because Newton's embedded CPython FFI deadlocks at the first `add_joint_revolute` calls under that mode.

The harness (`scripts/harness/omnisim_harness.py`) uses `--batch --mode=fast --minimize --stdout --stderr` (without `--no-window`) because it needs rendering enabled when a supervisor is in play for screenshots.

Use headless validation when:

- the change is runtime- or parser-oriented
- GUI behavior is not the feature under test
- you need a stable automation path for coding agents or CI

## What To Run For Common Changes

### Rendering changes

Use:

```bash
python -m omnisim build renderer
python -m omnisim build gui
python -m omnisim test-world tests/rendering/worlds/normals.omniworld --nomake
python -m omnisim benchmarks --nomake
```

### Runtime, node, or controller-orchestration changes

Use:

```bash
python -m omnisim build core
python -m omnisim test-smoke --nomake
python -m omnisim test-group physics --nomake
```

### Parser, PROTO, or template-regeneration changes

Use:

```bash
python -m omnisim build gui
python -m omnisim test-world tests/protos/worlds/template_deterministic.omniworld --nomake
python -m omnisim test-group protos --nomake
```

If you need parser coverage specifically, run it on a platform where that group is supported.

### GUI, editor, or scene-tree changes

Use:

```bash
python -m omnisim build gui
python -m omnisim run-world resources/projects/worlds/empty.omniworld
```

If the GUI change also affects world loading or rendering setup, add `test-smoke --nomake`.

### Controller API changes

Use:

```bash
python -m omnisim build controller-libs
python -m omnisim test-group api --nomake
```

## Known Blind Spots

Current gaps you should account for manually:

- the `parser` group is disabled on Windows in `tests/test_suite.py`
- speaker tests do not work in some headless GitHub Actions environments
- robot window tests depend on browser availability
- billboard tests have platform-specific CI issues
- `tests/README.md` still lists missing coverage for gyro, propeller, LED, position sensors, charger, friction, damping, and joints

## World Quality Rules

For smoke and benchmark worlds:

- prefer small deterministic worlds
- avoid network asset dependencies
- avoid worlds that intentionally produce warnings unless the warning is the subject of the test
- keep one clear reason per world for why it belongs in smoke or benchmark coverage

## Warning Hygiene

`tests/test_worlds.py` treats world warnings and uncached assets as first-class signals. When a change introduces new warnings, treat that as a regression unless the warning is clearly intentional and documented.

## When To Use `--nomake`

Use `--nomake` when:

- you already rebuilt the tree
- you are comparing repeated validation runs
- you are profiling a world and do not want the build step mixed into the loop

Do not use `--nomake` when:

- you are not sure the right target was rebuilt
- you changed code in more than one subsystem and want a safer path

## Adding New Fast Checks

To add a smoke world:

- edit `tests/smoke/smoke_worlds.json`
- keep the world small and deterministic
- prefer locally available assets

To add a benchmark world:

- edit `tests/benchmarks/benchmark_worlds.json`
- assign it to one category
- keep it stable enough for before/after comparisons
