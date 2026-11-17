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

"""A physics step that RUNS but goes bad is a FAULT, not a pass (2026-11-06).

THE DEFECTS (found by the Newton stress campaign, both under a PASS verdict with
step_errors 0, because neither raises):
  * CPU MuJoCo: a NaN/Inf/huge qacc (or qvel/qpos) makes mj_step call
    mj_resetData() -- the WHOLE world jumps back to its authored pose with zero
    velocity -- and bump `mjData.warning[mjWARN_BADQACC]`, which neither newton
    nor the runtime read. A 10-link passive chain at 4 ms looped back to t=0
    every 1.92 s, forever; the only trace was one line in <log>.stderr.
  * the base-divergence guard: on mujoco_warp a blow-up (or an njmax overflow)
    turns body_q NaN; the guard restored the last good pose every step while the
    solver state stayed NaN -- a permanent freeze -- reported by one print().

THE FIX, pinned here:
  * the runtime counts both into `World._physics_fault_count` (+ the latest
    `_physics_fault_msg`); the reset detector runs every step and zeroes the
    counters it reports, because mj_resetData clears them itself;
  * C++ step() polls the count every step, logs the first fault at ERROR
    ("PHYSICS FAULT at step N"), and writes `physics_faults` into the sidecar;
  * the harness reads it into `physics_steps` and `physics_ok`, with a
    NEWTON_PHYSICS_FAULT diagnostic; run-headless FAILs on it;
  * mujoco_warp's STICKY `Data.overflow` bitmask latches a transient overflow the
    every-30-ticks current-count sampler missed.

Engine-free. The engine pin is the stress-campaign chain world
(_scratch/stress, `python run.py chain mujoco n=10`): it must FAIL now.

    python -m pytest tests/test_newton_physics_faults.py -v
"""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts" / "harness"))
sys.path.insert(0, str(REPO / "scripts" / "dev"))

import diagnostic_codes as dc  # noqa: E402
import omnisim_harness as oh  # noqa: E402
from headless_runner import physics_fault_failure  # noqa: E402


def _sidecar(tmp_path: Path, data) -> Path:
    log = tmp_path / "engine.log"
    Path(str(log) + ".newton.json").write_text(json.dumps(data), encoding="utf-8")
    return log


# --- the harness reads the count --------------------------------------------

def test_an_engine_without_the_counter_is_unknown_not_zero(tmp_path):
    out = oh.read_physics_steps(_sidecar(tmp_path, {"finalised": True, "step_errors": 0}))
    assert out["step_errors"] == 0 and out["physics_faults"] is None


def test_a_faulting_world_reads_its_count_and_first_fault(tmp_path):
    out = oh.read_physics_steps(_sidecar(tmp_path, {
        "finalised": True, "step_errors": 0, "physics_faults": 3,
        "physics_first_fault": "MuJoCo found a NaN ... RESET THE WHOLE WORLD", "physics_first_fault_at": 481,
        "physics_last_fault": "MuJoCo found a NaN ... (again)"}))
    assert out["physics_faults"] == 3 and out["first_fault_at"] == 481
    assert "RESET THE WHOLE WORLD" in out["first_fault"]


def test_the_delta_counts_faults_during_the_request():
    d = oh.physics_steps_delta({"step_errors": 0, "physics_faults": 1},
                               {"step_errors": 0, "physics_faults": 5})
    assert d["physics_faults_during"] == 4 and d["physics_faults_total"] == 5
    # a reload (counters went down) counts the new world only
    d = oh.physics_steps_delta({"step_errors": 0, "physics_faults": 9},
                               {"step_errors": 0, "physics_faults": 2})
    assert d["physics_faults_during"] == 2
    # an older engine: unknown, not zero
    assert oh.physics_steps_delta({"step_errors": 0}, {"step_errors": 0})["physics_faults_during"] is None


# --- run-headless FAILs on it -------------------------------------------------

def test_run_headless_passes_a_clean_or_older_sidecar():
    assert physics_fault_failure({}) == []
    assert physics_fault_failure({"physics_faults": 0}) == []


def test_run_headless_fails_a_fault_and_names_it():
    lines = physics_fault_failure({"physics_faults": 312, "physics_first_fault_at": 481,
                                   "physics_first_fault": "MuJoCo found a NaN ... RESET\nmore"})
    assert lines and lines[0].startswith("[headless] FAIL: 312 PHYSICS FAULT(S)")
    assert "step 481" in lines[0] and lines[1].endswith("RESET")


# --- the engine's lines are classified, with a hint -------------------------

def test_the_fault_lines_are_newton_physics_fault():
    d = dc.classify_line("ERROR: [OmNewtonBackend] PHYSICS FAULT at step 481: the step ran but did not "
                         "produce valid physics. MuJoCo found a NaN ...")
    assert d["code"] == "NEWTON_PHYSICS_FAULT" and d["severity"] == "error"
    assert "newtonSubsteps" in dc.hint_for("NEWTON_PHYSICS_FAULT")
    d = dc.classify_line("WARNING: [OmNewtonBackend] physics fault again: 8 fault(s) in this world so far; "
                         "latest: MuJoCo found ...")
    assert d["code"] == "NEWTON_PHYSICS_FAULT"


# --- the runtime's reset detector, on a REAL MuJoCo reset --------------------

_RUNTIME = REPO / "src" / "omnisim" / "physics" / "omnisim_newton_runtime.py"
_BUNDLE = REPO / "msys64" / "mingw64" / "bin" / "newton-runtime" / "site-packages"


@pytest.fixture(scope="module")
def runtime():
    if _BUNDLE.is_dir() and str(_BUNDLE) not in sys.path:
        sys.path.insert(0, str(_BUNDLE))
    try:
        spec = importlib.util.spec_from_file_location("_omnisim_newton_runtime_faults", _RUNTIME)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        import mujoco
    except Exception as exc:  # noqa: BLE001
        pytest.skip("the Newton runtime does not import here: %r" % (exc,))
    if not hasattr(mujoco, "MjSpec"):
        pytest.skip("no real MuJoCo here")
    return mod


class _Solver:
    use_mujoco_cpu = True
    mjc_body_to_newton = None

    def __init__(self, m, d):
        self.mj_model, self.mj_data = m, d


class _W:
    """Just what _sample_solver_resets / _physics_fault touch."""

    def __init__(self, rt, solver):
        self._solver = solver
        self.body_indices = [0]
        self._physics_fault_count = 0
        self._physics_fault_msg = ""
        self._newton_log = lambda msg: None
        self._mjc_solver = lambda: self._solver
        self._MJ_RESET_WARNINGS = rt.World._MJ_RESET_WARNINGS
        for name in ("_physics_fault", "_runtime_body_for_newton", "_sample_solver_resets"):
            setattr(self, name, getattr(rt.World, name).__get__(self))


def _blow_up(m, d):
    import mujoco
    d.qfrc_applied[2] = 1e30          # |qacc| >> mjMAXVAL: mj_checkAcc resets the world
    mujoco.mj_step(m, d)
    d.qfrc_applied[:] = 0.0


def test_a_mujoco_reset_is_a_fault_and_the_second_one_is_seen_too(runtime):
    import mujoco
    m = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="ball" pos="0 0 2"><joint name="free_ball" type="free"/>'
        '<geom size=".1"/></body></worldbody></mujoco>')
    d = mujoco.MjData(m)
    w = _W(runtime, _Solver(m, d))
    for _ in range(50):
        mujoco.mj_step(m, d)
    w._sample_solver_resets()
    assert w._physics_fault_count == 0, "a healthy step is not a fault"
    _blow_up(m, d)
    # premise: MuJoCo reset the ball to its authored height (then integrated one
    # gravity step from there: 9.81 * 0.002^2 = 3.9e-5 m), after 50 steps of fall
    assert abs(d.qpos[2] - 2.0) < 1e-4, "premise: MuJoCo reset the ball to its authored height"
    w._sample_solver_resets()
    assert w._physics_fault_count == 1
    assert "RESET THE WHOLE WORLD" in w._physics_fault_msg and "free_ball" in w._physics_fault_msg
    assert "newtonSubsteps" in w._physics_fault_msg
    # mj_resetData clears the counters itself, so a second reset reads 1 again:
    # the detector must zero what it reported to see it
    _blow_up(m, d)
    w._sample_solver_resets()
    assert w._physics_fault_count == 2
    w._sample_solver_resets()
    assert w._physics_fault_count == 2, "one reset is one fault"


def test_the_gpu_path_is_not_read_as_cpu(runtime):
    import mujoco
    m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><freejoint/><geom size=".1"/></body></worldbody></mujoco>')
    d = mujoco.MjData(m)
    s = _Solver(m, d)
    s.use_mujoco_cpu = False
    w = _W(runtime, s)
    _blow_up(m, d)
    w._sample_solver_resets()
    assert w._physics_fault_count == 0


def test_the_divergence_message_names_the_body_and_the_overflow(runtime):
    import numpy as np

    class _Data:
        overflow = type("A", (), {"numpy": lambda self: np.array([1], dtype=np.int32)})()

    w = type("W", (), {})()
    w.body_indices = [7, 8, 9]
    w._base_guard_max = 1000.0
    w._last_good_body_q = np.zeros((3, 7))
    w.solver = type("S", (), {"mjw_data": _Data()})()
    w._runtime_body_for_newton = runtime.World._runtime_body_for_newton.__get__(w)
    msg = runtime.World._divergence_fault_msg(w, np, np.array([8]))
    assert "runtime body 1" in msg and "frozen" in msg and "newtonNjmax" in msg and "njmax" in msg


# --- the C++ half is wired (source pins; the engine pin is the stress world) --

def test_the_backend_polls_the_count_and_writes_the_sidecar():
    src = (REPO / "src" / "omnisim" / "physics" / "OmNewtonBackend.cpp").read_text(encoding="utf-8")
    assert '"_physics_fault_count"' in src and '"_physics_fault_msg"' in src
    assert '\\"physics_faults\\":' in src
    assert "PHYSICS FAULT at step " in src
