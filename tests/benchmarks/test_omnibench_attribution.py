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

"""Gate: no OmniBench row may be attributed to a physics engine that was deleted.

src/ode was DELETED in commit bdc02139 and Newton is the only physics backend.
Every place that could not PROVE which backend drove a run used to answer "ODE",
which published measurements under the name of an engine that does not exist --
and, worse, made an unprovable row indistinguishable from a real ODE one.

Pinned here (all cheap, no engine, no GPU, no network):
  * lane 3c's fallback label is `omnisim-unverified` and it explains itself;
  * the label is a value the results schema knows about;
  * run_all.py's summary raises an `[attribution]` finding for such rows rather
    than printing them as ordinary results.

The sibling rule for the machine fingerprint embedded in every row lives in
tests/test_env_fingerprint_label.py.
"""

from __future__ import annotations

import ast
import importlib.util
import json
import sys
from pathlib import Path

import pytest

_HERE = Path(__file__).resolve().parent          # tests/benchmarks
_OMNIBENCH = _HERE / "omnibench"


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def drv():
    return _load(_OMNIBENCH / "lane3" / "driveability.py", "ob_drv_attr")


@pytest.fixture(scope="module")
def run_all():
    return _load(_OMNIBENCH / "run_all.py", "ob_run_all_attr")


@pytest.fixture(scope="module")
def results_mod():
    return _load(_OMNIBENCH / "common" / "results.py", "ob_results_attr")


@pytest.fixture(scope="module")
def launch_mod():
    return _load(_OMNIBENCH / "common" / "engine_launch.py", "ob_launch_attr")


# ---------------------------------------------------------------------------

def test_clean_sidecar_attributes_to_newton(tmp_path, drv):
    log = tmp_path / "engine.log"
    (tmp_path / "engine.log.newton.json").write_text(json.dumps(
        {"backend": "newton", "solver": "MuJoCo (cpu/mj_step, default)",
         "finalised": True, "degraded": False}), encoding="utf-8")
    engine, why = drv.engine_attribution(str(log))
    assert engine == "omnisim-newton"
    assert why is None


def test_missing_sidecar_is_unverified_not_ode(tmp_path, drv):
    """THE DEFECT: this returned "omnisim-ode"."""
    engine, why = drv.engine_attribution(str(tmp_path / "engine.log"))
    assert engine == "omnisim-unverified"
    assert "ode" not in engine
    assert why and why.startswith("engine=omnisim-unverified")
    assert "bdc02139" in why, "the row must say WHY, citing the deletion"
    assert "world-finalize" in why, "the row must name the LIKELY cause"


def test_degraded_sidecar_is_unverified_not_ode(tmp_path, drv):
    log = tmp_path / "engine.log"
    (tmp_path / "engine.log.newton.json").write_text(json.dumps(
        {"backend": "newton", "solver": "FAILED", "finalised": True,
         "degraded": True}), encoding="utf-8")
    engine, why = drv.engine_attribution(str(log))
    assert engine == "omnisim-unverified"
    assert why and "degraded=true" in why


def test_unparseable_sidecar_is_unverified(tmp_path, drv):
    log = tmp_path / "engine.log"
    (tmp_path / "engine.log.newton.json").write_text("{not json", encoding="utf-8")
    engine, why = drv.engine_attribution(str(log))
    assert engine == "omnisim-unverified"
    assert why


def test_schema_knows_the_label(results_mod):
    assert "omnisim-unverified" in results_mod.ENGINES
    # omnisim-ode stays READABLE (recorded results*/ rows carry it) ...
    assert "omnisim-ode" in results_mod.ENGINES
    # ... and the SPEC must say plainly that it is no longer writable.
    spec = (_OMNIBENCH / "SPEC.md").read_text(encoding="utf-8")
    assert "omnisim-unverified" in spec
    assert "bdc02139" in spec


def test_summary_raises_an_attribution_finding(run_all):
    rows = [
        {"test": "driveability_load_valid_world", "engine": "omnisim-unverified",
         "deviations": ["engine=omnisim-unverified: no readable backend-verdict "
                        "sidecar at engine.log.newton.json (FileNotFoundError)."],
         "metrics": {}},
        {"test": "driveability_summary", "engine": "omnisim-newton",
         "deviations": [], "metrics": {}},
    ]
    findings = run_all._findings_attribution(rows)
    assert len(findings) == 1, findings
    assert findings[0].startswith("[attribution]")
    assert "UNVERIFIED" in findings[0]
    assert "sidecar" in findings[0], "the finding must carry the recorded reason"
    # A fully attributed set produces no noise.
    assert run_all._findings_attribution([rows[1]]) == []


# --- the rule is SHARED, and lane 1 obeys it too --------------------------
# Added when lane 1 was found still publishing `"omnisim-%s" % backend` --
# i.e. "omnisim-newton" unconditionally, with no sidecar consulted -- months
# after lane 3 was fixed. Lane 3's fix was a lane's fix, not the suite's, so
# the rule moved to common/engine_launch.py and both lanes now call it.

@pytest.mark.parametrize("verdict,expect_label", [
    ({"present": False}, "omnisim-unverified"),
    ({"present": True, "error": "Expecting value: line 1"}, "omnisim-unverified"),
    ({"present": True, "degraded": True, "solver": "mujoco"}, "omnisim-unverified"),
    ({"present": True, "degraded": False, "finalised": False,
      "solver": "mujoco"}, "omnisim-unverified"),
    ({"present": True, "degraded": False, "finalised": True,
      "solver": "mujoco"}, "omnisim-newton"),
])
def test_common_rule_never_names_a_deleted_engine(launch_mod, verdict,
                                                  expect_label):
    label, why = launch_mod.engine_attribution(verdict)
    assert label == expect_label
    assert "ode" not in label
    if label == "omnisim-unverified":
        assert why and "bdc02139" in why, "an unverified row must explain itself"
        assert "NOT" in why, "it must deny ODE explicitly, not merely omit it"
    else:
        assert why is None


def test_lane3_delegates_to_the_shared_rule(drv, launch_mod):
    """One rule, two callers -- two copies is how they diverged the first time."""
    assert drv.engine_attribution.__module__ != launch_mod.__name__
    src = (_OMNIBENCH / "lane3" / "driveability.py").read_text(encoding="utf-8")
    assert "_launch.engine_attribution" in src, \
        "lane 3 must call the shared rule, not re-implement it"


def test_lane1_publishes_the_attributed_label_not_its_argument():
    """THE DEFECT: lane 1's row said `"engine": "omnisim-%s" % backend`.

    That is the --backend ARGUMENT echoed back -- a value the runner was told,
    never one it verified -- so a run whose Newton never finalised was still
    published as `omnisim-newton`. The same shape as the tool-honesty rule in
    AGENTS.md: never return the commanded value under a measured key.
    """
    src = (_OMNIBENCH / "lane1" / "run_omnisim.py").read_text(encoding="utf-8")
    assert '"engine": engine_label' in src, \
        "lane 1 must publish the sidecar-derived label"
    assert "engine_launch.engine_attribution" in src, \
        "lane 1 must use the shared rule"
    # Value-position only: the comments in that file legitimately QUOTE the old
    # strings to explain the defect, so a bare substring check would fail on
    # its own documentation.
    for lit in _string_constants(_OMNIBENCH / "lane1" / "run_omnisim.py"):
        assert "omnisim-%s" not in lit, \
            "lane 1 must not format a row label from its --backend argument"
        assert "ode (no sidecar)" not in lit, \
            "the console must not name a deleted engine either"
    assert "UNVERIFIED (no sidecar)" in src, \
        "the console must say UNVERIFIED when the sidecar is missing"


def test_no_lane_runner_defaults_a_row_to_ode():
    """A tree-wide floor: no runner may PRODUCE an ODE engine label.

    Deliberately not a substring grep. `"omnisim-ode"` must stay both readable
    and discussable: `common/results.py`'s ENGINES tuple still accepts it so the
    recorded `results*/` rows measured while ODE shipped can be parsed, and
    several docstrings quote it to explain the retirement. What may not exist is
    a live code path that ASSIGNS or RETURNS it as this run's engine.
    """
    offenders = []
    for py in sorted(_OMNIBENCH.rglob("*.py")):
        if "__pycache__" in py.parts:
            continue
        tree = ast.parse(py.read_text(encoding="utf-8", errors="replace"),
                         filename=str(py))
        for node in ast.walk(tree):
            # `return "omnisim-ode"` / `return "omnisim-ode", why`
            if isinstance(node, ast.Return):
                for c in ast.walk(node):
                    if isinstance(c, ast.Constant) and c.value == "omnisim-ode":
                        offenders.append("%s:%d: returns it" % (py.name, node.lineno))
            # `{"engine": "omnisim-ode"}` -- a published row field
            if isinstance(node, ast.Dict):
                for k, v in zip(node.keys, node.values):
                    if (isinstance(k, ast.Constant) and k.value == "engine"
                            and isinstance(v, ast.Constant)
                            and isinstance(v.value, str) and "ode" in v.value):
                        offenders.append("%s:%d: publishes engine=%r"
                                         % (py.name, node.lineno, v.value))
            # `engine = "omnisim-ode"` (ENGINES = (...) is a tuple, not a str)
            if isinstance(node, ast.Assign):
                if (isinstance(node.value, ast.Constant)
                        and node.value.value == "omnisim-ode"):
                    offenders.append("%s:%d: assigns it" % (py.name, node.lineno))
    assert not offenders, "a row may not be stamped with a deleted engine:\n" + \
        "\n".join(offenders)


def _string_constants(path: Path):
    """Every string literal in a module -- the values it can actually emit."""
    tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"),
                     filename=str(path))
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef,
                             ast.ClassDef)):
            d = ast.get_docstring(node, clean=False)
            if d is not None:
                docstrings.add(d)
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            if node.value not in docstrings:
                yield node.value
