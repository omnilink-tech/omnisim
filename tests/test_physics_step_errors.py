# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A physics step that RAISES is reported, not counted as executed (2026-11-04).

THE DEFECT. `OmNewtonBackend::step()` logged a raised `world.step()` as one
WARNING per tick and returned. The engine clock and every controller still
advanced, so the run looked healthy from outside: on a SparkRover assembly
scene (a ~2,700-box moving unit among ~6,000 static shapes) MuJoCo raised
"mj_stackAlloc: out of memory, stack overflow at mj_narrowphase" on every step,
the unit never moved and had no contacts, and the harness's `/sim/step`
answered `steps_executed: 250`. Raising newtonNjmax / newtonNconmax did not
help: on the CPU solver they size nothing (they are mujoco_warp buffers).

THE FIX, pinned here:
  * the engine counts raised steps into the `.newton.json` sidecar
    (`step_errors`, `step_errors_streak`, `step_first_error[_at]`,
    `step_last_error[_at]`), logs the first at ERROR (so run-headless FAILs)
    and later ones at WARNING on the 2nd, 4th, 8th ...;
  * the harness reads the sidecar around every `/sim/step` (`physics_steps`,
    `physics_ok`, a NEWTON_STEP_FAILED diagnostic) and into `/sim/state`;
  * `run-headless` FAILs a run with any step_errors and names the exception;
  * the MuJoCo ARENA is a lever: `WorldInfo.newtonArenaMB` /
    `OMNISIM_NEWTON_ARENA_MB` set it on the MjSpec newton compiles.

This file is engine-free; tests/test_physics_step_errors_engine.py is the engine pin.

    python -m pytest tests/test_physics_step_errors.py -v
"""
from __future__ import annotations

import importlib.util
import json
import os
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts" / "harness"))
sys.path.insert(0, str(REPO / "scripts" / "dev"))

import diagnostic_codes as dc  # noqa: E402
import omnisim_harness as oh  # noqa: E402
from headless_runner import step_error_failure  # noqa: E402


# --- the harness reads the engine's count ----------------------------------

def _sidecar(tmp_path: Path, data) -> Path:
    log = tmp_path / "engine.log"
    text = data if isinstance(data, str) else json.dumps(data)
    Path(str(log) + ".newton.json").write_text(text, encoding="utf-8")
    return log


def test_no_sidecar_is_unknown_not_zero(tmp_path):
    out = oh.read_physics_steps(tmp_path / "engine.log")
    assert out["step_errors"] is None and out["source"] == "sidecar_absent"


def test_an_older_engine_is_unknown_not_zero(tmp_path):
    log = _sidecar(tmp_path, {"backend": "newton", "finalised": True, "solver": "mujoco"})
    out = oh.read_physics_steps(log)
    assert out["step_errors"] is None and out["source"] == "not_reported_by_this_engine"


def test_a_healthy_world_reads_zero(tmp_path):
    log = _sidecar(tmp_path, {"backend": "newton", "finalised": True, "step_errors": 0,
                              "step_errors_streak": 0})
    out = oh.read_physics_steps(log)
    assert out["step_errors"] == 0 and out["source"] == "sidecar" and out["last_error"] is None


def test_a_failing_world_reads_its_count_and_exception(tmp_path):
    log = _sidecar(tmp_path, {
        "backend": "newton", "finalised": True, "step_errors": 249, "step_errors_streak": 249,
        "step_first_error": "mj_stackAlloc: out of memory, stack overflow\n  max = 1",
        "step_first_error_at": 2, "step_last_error": "mj_stackAlloc: out of memory",
        "step_last_error_at": 250})
    out = oh.read_physics_steps(log)
    assert out["step_errors"] == 249 and out["step_errors_streak"] == 249
    assert out["first_error_at"] == 2 and out["last_error_at"] == 250
    assert out["first_error"].startswith("mj_stackAlloc")


def test_a_torn_sidecar_is_unreadable_not_zero(tmp_path):
    log = _sidecar(tmp_path, '{"backend":"newton","step_err')
    out = oh.read_physics_steps(log)
    assert out["step_errors"] is None and out["source"] == "sidecar_unreadable"


def test_delta_counts_only_the_failures_between_the_reads():
    before = {"step_errors": 10}
    after = {"step_errors": 260, "step_errors_streak": 250}
    d = oh.physics_steps_delta(before, after)
    assert d["step_errors_during"] == 250 and d["step_errors_total"] == 260


def test_delta_after_a_reload_counts_the_new_world_only():
    # the engine resets the counters per world, so a total that went DOWN is a reload
    assert oh.physics_steps_delta({"step_errors": 500}, {"step_errors": 3})["step_errors_during"] == 3


def test_delta_is_unknown_when_the_engine_cannot_say():
    assert oh.physics_steps_delta({"step_errors": 0}, {"step_errors": None})["step_errors_during"] is None
    assert oh.physics_steps_delta({"step_errors": None}, {"step_errors": 4})["step_errors_during"] == 4


# --- the engine's lines are classified, with a hint ------------------------

def test_the_first_failure_line_is_newton_step_failed():
    d = dc.classify_line(
        "ERROR: [OmNewtonBackend] physics step 7 RAISED and was SKIPPED: nothing in this world "
        "moved ... The Python exception was: mj_stackAlloc: out of memory, stack overflow")
    assert d["code"] == "NEWTON_STEP_FAILED" and d["severity"] == "error"
    assert d["detail"].startswith("mj_stackAlloc")
    assert "newtonArenaMB" in dc.hint_for("NEWTON_STEP_FAILED")


def test_the_repeat_line_is_newton_step_failed_not_runtime_broken():
    # the old per-tick "step raised:" warning was filed as NEWTON_RUNTIME_BROKEN,
    # whose hint sends the reader to reinstall a runtime that is fine
    d = dc.classify_line("WARNING: [OmNewtonBackend] step raised again: 8 of 9 physics steps "
                         "in this world SKIPPED (8 in a row); latest: mj_stackAlloc")
    assert d["code"] == "NEWTON_STEP_FAILED"


# --- run-headless FAILs on the count ---------------------------------------

def test_run_headless_passes_a_clean_or_older_sidecar():
    assert step_error_failure({}) == []
    assert step_error_failure({"finalised": True}) == []
    assert step_error_failure({"finalised": True, "step_errors": 0}) == []


def test_run_headless_fails_any_raised_step_and_names_it():
    lines = step_error_failure({"step_errors": 3, "step_errors_streak": 0, "step_first_error_at": 41,
                                "step_first_error": "mj_stackAlloc: out of memory\n  detail"})
    assert lines and lines[0].startswith("[headless] FAIL: 3 physics step(s) RAISED")
    assert "step 41" in lines[0] and lines[1].endswith("mj_stackAlloc: out of memory")


# --- the arena lever (runtime, no engine) -----------------------------------

_RUNTIME = REPO / "src" / "omnisim" / "physics" / "omnisim_newton_runtime.py"
_BUNDLE = REPO / "msys64" / "mingw64" / "bin" / "newton-runtime" / "site-packages"


@pytest.fixture(scope="module")
def runtime():
    if _BUNDLE.is_dir() and str(_BUNDLE) not in sys.path:
        sys.path.insert(0, str(_BUNDLE))
    try:
        spec = importlib.util.spec_from_file_location("_omnisim_newton_runtime_arena", _RUNTIME)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        import mujoco  # noqa: F401
    except Exception as exc:  # noqa: BLE001
        pytest.skip("the Newton runtime does not import here: %r" % (exc,))
    return mod


class _W:
    """Just the attributes _mj_arena_bytes / _mj_arena_scope touch."""

    def __init__(self, rt, world_mb=0.0):
        self._arena_mb_world = world_mb
        self._newton_log = lambda msg: None
        self._mj_arena_bytes = rt.World._mj_arena_bytes.__get__(self)
        self._mj_arena_scope = rt.World._mj_arena_scope.__get__(self)


def _narena_under(scope) -> int:
    import mujoco
    xml = '<mujoco><worldbody><body><freejoint/><geom size=".1"/></body></worldbody></mujoco>'
    with scope:
        return mujoco.MjSpec.from_string(xml).compile().narena


def test_arena_precedence(runtime, monkeypatch):
    monkeypatch.delenv("OMNISIM_NEWTON_ARENA_MB", raising=False)
    assert _W(runtime)._mj_arena_bytes() == 0
    assert _W(runtime, 64)._mj_arena_bytes() == 64 << 20
    monkeypatch.setenv("OMNISIM_NEWTON_ARENA_MB", "128")
    assert _W(runtime, 64)._mj_arena_bytes() == 128 << 20      # the env var wins
    monkeypatch.setenv("OMNISIM_NEWTON_ARENA_MB", "0")
    assert _W(runtime, 64)._mj_arena_bytes() == 0              # value-parsed: 0 = MuJoCo's own
    monkeypatch.setenv("OMNISIM_NEWTON_ARENA_MB", "lots")
    assert _W(runtime, 64)._mj_arena_bytes() == 0


def test_arena_scope_sets_and_restores_the_compiled_arena(runtime, monkeypatch):
    import mujoco
    monkeypatch.delenv("OMNISIM_NEWTON_ARENA_MB", raising=False)
    stock = _narena_under(_W(runtime)._mj_arena_scope())
    assert _narena_under(_W(runtime, 96)._mj_arena_scope()) == 96 << 20
    assert mujoco.MjSpec.from_string("<mujoco/>").compile().narena < 96 << 20, \
        "the MjSpec.compile hook leaked out of its scope"
    assert stock < 96 << 20
