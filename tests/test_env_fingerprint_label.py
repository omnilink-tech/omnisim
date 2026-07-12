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

"""The viewport physics label must not claim a verdict before the engine has one.

The engine finalizes the physics backend on the FIRST simulation tick
(WbSimulationWorld::step -> WbNewtonBackend::finalizeWorld), which is also when
the ``world finalised`` log line and the ``<log>.newton.json`` sidecar are
written. Deploy controllers fingerprint BEFORE their first robot.step(), so a
plain report() there always raced finalize and painted a permanent orange
"finalise UNCONFIRMED (SUSPECT)" label over Newton-driven demos (the
"viewport label never turns green" bug, 2026-07-12). These tests pin the
two-phase fix: pre_step=True -> neutral grey "verifying" label + pending flag;
a post-step re-report -> the real (green) verdict, and the warning sub-label
is cleared.
"""

import importlib.util
import json
import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[1]
_MOD_PATH = _REPO / "projects" / "policies" / "common" / "env_fingerprint.py"

_spec = importlib.util.spec_from_file_location("env_fingerprint", _MOD_PATH)
envfp = importlib.util.module_from_spec(_spec)
sys.modules["env_fingerprint"] = envfp
_spec.loader.exec_module(envfp)

GREEN = 0x44FF66
ORANGE = 0xFFAA00

IMPORTS_OK = ("INFO: [WbNewtonBackend] warp + newton imports OK; FFI smoke OK "
              "(newton.ModelBuilder()); helper module loaded\n")
FINALISED = ("INFO: [WbNewtonBackend] world finalised "
             "(solver=MuJoCo (mujoco_warp, WorldInfo.newtonSolver))\n")
ODE_FALLBACK = "WARNING: [WbNewtonBackend] runtime missing -- falling back to ODE\n"


class FakeSupervisor:
    """Records setLabel calls: {id: (text, color)}."""

    def __init__(self):
        self.labels = {}

    def setLabel(self, lid, text, x, y, size, color, transparency):
        self.labels[lid] = (text, color)


@pytest.fixture(autouse=True)
def _fast_probes(monkeypatch):
    """Skip the nvidia-smi / git subprocess probes -- irrelevant and slow here."""
    monkeypatch.setattr(envfp, "_nvidia_gpu", lambda: "NVIDIA Test RTX 0000 (driver 0.0)")
    monkeypatch.setattr(envfp, "_git_commit", lambda root: "deadbeef")


def _run_report(log_text, tmp_path, *, sidecar=None, pre_step=False):
    log = tmp_path / "omnisim_log.txt"
    log.write_text(log_text, encoding="utf-8")
    if sidecar is not None:
        (tmp_path / "omnisim_log.txt.newton.json").write_text(
            json.dumps(sidecar), encoding="utf-8")
    robot = FakeSupervisor()
    said = []
    fp = envfp.report(robot, said.append, engine_log_path=str(log),
                      repo_root=_REPO, pre_step=pre_step)
    return fp, robot, "".join(said)


def test_pre_step_defers_instead_of_false_alarm(tmp_path):
    """Pre-first-tick read (imports OK, no finalise, no sidecar) must NOT paint
    the orange UNCONFIRMED verdict -- that state is a startup race, not truth."""
    fp, robot, said = _run_report(IMPORTS_OK, tmp_path, pre_step=True)
    assert fp["pending"] is True
    assert not fp["ok"]
    text, color = robot.labels[90]
    assert "verifying" in text
    assert color not in (GREEN, ORANGE)
    assert "PENDING" in said


def test_post_step_resolves_green(tmp_path):
    """After the first tick the sidecar + finalise line exist: the re-report
    must go green and clear the warning sub-label left by any earlier pass."""
    sidecar = {"backend": "newton", "solver": "MuJoCo (mujoco_warp, WorldInfo.newtonSolver)",
               "finalised": True, "degraded": False}
    fp, robot, _ = _run_report(IMPORTS_OK + FINALISED, tmp_path, sidecar=sidecar)
    assert fp["ok"] is True
    assert not fp.get("pending")
    assert "mujoco_warp" in fp["physics"]
    text, color = robot.labels[90]
    assert color == GREEN
    assert "physics=newton/mujoco_warp" in text
    assert robot.labels[91][0] == ""  # stale warning line cleared


def test_pre_step_with_existing_verdict_is_immediate(tmp_path):
    """A controller (re)started mid-run finds the sidecar already there --
    no pointless grey phase, green right away."""
    sidecar = {"backend": "newton", "solver": "MuJoCo (mujoco_warp)",
               "finalised": True, "degraded": False}
    fp, robot, _ = _run_report(IMPORTS_OK, tmp_path, sidecar=sidecar, pre_step=True)
    assert fp["ok"] is True
    assert not fp.get("pending")
    assert robot.labels[90][1] == GREEN


def test_pre_step_ode_fallback_is_not_deferred(tmp_path):
    """An explicit Newton->ODE fallback is a real verdict, not a race: report
    it orange immediately even pre-step."""
    fp, robot, _ = _run_report(IMPORTS_OK + ODE_FALLBACK, tmp_path, pre_step=True)
    assert not fp.get("pending")
    assert not fp["ok"]
    assert "ODE" in fp["physics"]
    assert robot.labels[90][1] == ORANGE


def test_post_hoc_read_unchanged(tmp_path):
    """Without pre_step (doctor / CLI on a finished run's log), an imports-only
    log must keep the honest UNCONFIRMED orange verdict."""
    fp, robot, _ = _run_report(IMPORTS_OK, tmp_path)
    assert not fp.get("pending")
    assert not fp["ok"]
    assert "UNCONFIRMED" in fp["physics"]
    assert robot.labels[90][1] == ORANGE
