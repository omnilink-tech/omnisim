# OmniSim tests

The suite is split into **lanes** by what they need. Pick the lane by cost and by
whether the simulator binary may be launched on this box at all (another session may
own the engine, or it may be mid-rebuild).

| Lane | Command | Needs | Cost (machine `9722d23d12a3`, 2026-09-02) |
|---|---|---|---|
| **Unit (engine-free)** | `make tests-unit` | Python + pytest + Pillow. No engine, no GPU, no network. | 50 s of pytest, 1,255 tests (measured 2026-09-02; it was 7 min 10 s on 2026-09-02 morning before the omniworld and pin lanes were sped up) |
| **Smoke (engine)** | `make tests-smoke` | A built engine + the Newton runtime (`python -m omnisim doctor` first). | 1–3 min warm; the FIRST run also builds the missing test controllers |
| **Docs** | `make tests-docs` | Python + pytest + Pillow. | ~2 s |
| **Sources (lint)** | `make tests-sources` | Python + pytest. | ~8.5 min (tree walks; `test_pep8` / `test_line_ending` are CI-only) |
| **Engine test groups** | `python -m omnisim test-group api` (also `parser`, `physics`, `rendering`, `cache`, `protos`, `other_api`) | A built engine. | minutes each |
| **One world** | `python -m omnisim test-world path/to/world.omniworld --nomake` | A built engine. | seconds |

`make` here is GNU make (MSYS on Windows: `/c/msys64/usr/bin/make`). The Makefile probes
for a `python3` / `python` / `py` that can import pytest and Pillow; override with
`PYTHON=<interpreter>` if it picks the wrong one.

## How the unit lane stays engine-free

No test file declares a marker by hand. [`tests/conftest.py`](conftest.py) marks a module
`engine` **at collection** when its source contains a launch-shaped pattern (`--batch`,
a `run_headless(` call, the `skipif(_binary() is None)` idiom, a real
`resolve_omnisim_binary()` call, OmniBench's `engine_launch`, `omnisim_env(` /
`omnisim.dev.runner`, a live `OMNISIM_HARNESS_URL`, a bring-up skip reason, or a
`subprocess.*(` call next to any mention of `omnisim`). The list is deliberately biased
towards marking: a pure test wrongly left out of the unit lane costs nothing, a launching
test wrongly left in it spawns `omnisim-bin` on a box whose engine is being rebuilt.

```bash
python tests/conftest.py --list-engine tests          # what is marked, and why
python tests/conftest.py --list-engine-free tests     # what `make tests-unit` runs at root level
pytest -m "not engine" tests/harness                  # the marker works with any path
pytest -m engine tests                                # only the engine tests
```

Known false positives (pure tests that are marked anyway): `tests/packaging/test_onboarding_payload.py`
and `tests/python/omniworld/test_catalog.py` (their subprocess call is `git` / the omniworld
CLI) and `tests/test_doctor_strict.py` (one of its tests reads the real binary). Run them
by path when you want them.

## The bring-up skip cap

Engine tests skip themselves when the engine does not come up (`"bring-up flake"`,
`"did not come up"`, `"Newton did not ..."`). Each skip is legitimate on its own; a run
where the engine **never** came up is not a pass, yet it used to read green. The same
conftest counts those skips and fails the session when there are more than
`OMNISIM_BRINGUP_SKIP_CAP` (default `2`; `-1` disables). The terminal summary lists every
one under `engine bring-up skips`. When it fires: `python -m omnisim doctor`, then one
engine test by hand.

## Running a single engine test under the thermal guard

This laptop throttles at 75 °C and the thing to constrain is **concurrent engine runs**.
Run one engine test at a time, under the guard:

```bash
python scripts/dev/thermal_guard.py run --ceiling 75 -- pytest tests/test_newton_static_floor_collides.py
```

Rules that are not guessable: never kill an `omnisim-bin` you did not spawn (a live engine
of another session is not an orphan — `taskkill /F` gives it exit code 1 and an empty log
that reads exactly like a crash); set `OMNISIM_LOG_PATH` per child when you run more than
one; and a bare headless `PASS` is a log verdict, not a physics verdict (`--fail-on-runaway`
for the latter). `AGENTS.md` §3b/§3e/§8 has the long form.

## Destructive tests, deliberately excluded from a bare `pytest`

`pytest.ini` ignores three standalone runners that abort collection or rewrite the tree:

- `tests/test_suite.py` — the upstream Webots suite runner (`python tests/test_suite.py`);
- `tests/test_sources.py` — runs `unittest discover` at import; use `make tests-sources`;
- `tests/test_worlds.py` — loads **and saves** every released world through the engine
  (a bare run rewrote 163 tracked worlds in place). Run it on purpose:
  `python -m omnisim test-world <world>` for one, `python -m pytest tests/test_worlds.py`
  for all, expecting rewrites.

`tests/benchmarks/` (OmniBench, AgentBench, the OmniLink task suites) are not pytest lanes;
each has its own entry point and README.
