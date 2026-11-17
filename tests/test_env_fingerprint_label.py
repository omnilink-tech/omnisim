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
(OmSimulationWorld::step -> OmNewtonBackend::finalizeWorld), which is also when
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

# The bracketed tag is named after the emitting C++ class, and those classes are
# being renamed Wb* -> Om*, so these lines will read "[OmNewtonBackend] ..."
# afterwards. env_fingerprint's matchers accept BOTH prefixes permanently, so
# these shapes are built per-prefix and the contract is parametrized below.
_TAG_PREFIXES = ("Wb", "Om")


def imports_ok(tag="Wb"):
    return (f"INFO: [{tag}NewtonBackend] warp + newton imports OK; FFI smoke OK "
            "(newton.ModelBuilder()); helper module loaded\n")


def finalised(tag="Wb"):
    return (f"INFO: [{tag}NewtonBackend] world finalised "
            "(solver=MuJoCo (mujoco_warp, WorldInfo.newtonSolver))\n")


def ode_fallback(tag="Wb"):
    return f"WARNING: [{tag}NewtonBackend] runtime missing -- falling back to ODE\n"


# Current-engine (Wb) shapes, kept as the constants the existing tests use.
IMPORTS_OK = imports_ok("Wb")
FINALISED = finalised("Wb")
ODE_FALLBACK = ode_fallback("Wb")


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
    """A logged 'falling back to ODE' is a real verdict, not a race: report it
    orange immediately even pre-step.

    It must NOT be reported as an ODE run. The engine still logs that wording
    (stale text in OmNewtonBackend.cpp) but src/ode was DELETED (bdc02139), so
    the line means the Newton runtime did not come up -- the run has no verified
    physics at all.
    """
    fp, robot, _ = _run_report(IMPORTS_OK + ODE_FALLBACK, tmp_path, pre_step=True)
    assert not fp.get("pending")
    assert not fp["ok"]
    assert fp["physics"].startswith("UNVERIFIED")
    assert "ODE" not in fp["physics"], (
        "the verdict names a DELETED backend as the engine that ran: %r"
        % fp["physics"])
    assert robot.labels[90][1] == ORANGE


def test_post_hoc_read_unchanged(tmp_path):
    """Without pre_step (doctor / CLI on a finished run's log), an imports-only
    log must keep the honest UNCONFIRMED orange verdict."""
    fp, robot, _ = _run_report(IMPORTS_OK, tmp_path)
    assert not fp.get("pending")
    assert not fp["ok"]
    assert "UNCONFIRMED" in fp["physics"]
    assert robot.labels[90][1] == ORANGE


# ── the Wb -> Om dual-accept contract, under test ────────────────────────────
# The engine's C++ classes are being renamed Wb* -> Om* and the bracketed log tag
# follows the class name. Every verdict below must be IDENTICAL for both
# prefixes; if someone narrows _RE_IMPORTS_OK / _RE_ODE_FALLBACK back to one
# prefix, the "Om" half of each parametrization goes red.


@pytest.mark.parametrize("tag", _TAG_PREFIXES)
def test_imports_ok_recognised_for_both_tags(tmp_path, tag):
    """`imports OK` must defer (grey PENDING) pre-step under either prefix."""
    fp, robot, said = _run_report(imports_ok(tag), tmp_path, pre_step=True)
    assert fp["pending"] is True, f"{tag}: imports-OK line not recognised"
    assert not fp["ok"]
    assert "verifying" in robot.labels[90][0]
    assert robot.labels[90][1] not in (GREEN, ORANGE)
    assert "PENDING" in said


@pytest.mark.parametrize("tag", _TAG_PREFIXES)
def test_ode_fallback_recognised_for_both_tags(tmp_path, tag):
    """An explicit Newton->ODE fallback is a real verdict under either prefix:
    orange immediately, never deferred."""
    fp, robot, _ = _run_report(imports_ok(tag) + ode_fallback(tag), tmp_path,
                               pre_step=True)
    assert not fp.get("pending"), f"{tag}: ODE-fallback line not recognised"
    assert not fp["ok"]
    # Both this branch and the no-evidence branch now report UNVERIFIED (neither
    # may name ODE -- it is deleted), so the scan flag is the discriminator that
    # proves the LINE itself was matched.
    assert fp["scan"]["ode_fallback"] is True, (
        f"{tag}: _RE_ODE_FALLBACK did not match -- got {fp['physics']!r}")
    assert fp["physics"] == "UNVERIFIED (Newton runtime absent/broken)"
    assert robot.labels[90][1] == ORANGE


@pytest.mark.parametrize("tag", _TAG_PREFIXES)
def test_post_hoc_unconfirmed_for_both_tags(tmp_path, tag):
    """Post-hoc, an imports-only log stays honestly UNCONFIRMED/orange under
    either prefix -- 'imports OK' is never proof Newton drove the world."""
    fp, robot, _ = _run_report(imports_ok(tag), tmp_path)
    assert not fp.get("pending")
    assert not fp["ok"]
    assert "UNCONFIRMED" in fp["physics"], f"{tag}: imports-OK line not recognised"
    assert robot.labels[90][1] == ORANGE


@pytest.mark.parametrize("tag", _TAG_PREFIXES)
def test_finalised_resolves_green_for_both_tags(tmp_path, tag):
    """The finalise line + sidecar go green under either prefix. (The finalise
    regex itself is tag-agnostic; this pins that the surrounding scan is too.)"""
    sidecar = {"backend": "newton",
               "solver": "MuJoCo (mujoco_warp, WorldInfo.newtonSolver)",
               "finalised": True, "degraded": False}
    fp, robot, _ = _run_report(imports_ok(tag) + finalised(tag), tmp_path,
                               sidecar=sidecar)
    assert fp["ok"] is True
    assert "mujoco_warp" in fp["physics"]
    assert robot.labels[90][1] == GREEN


def test_mixed_prefix_log_parses(tmp_path):
    """A log concatenated across the rename (Wb imports line, Om finalise line)
    must still resolve -- users keep and merge old captures."""
    sidecar = {"backend": "newton", "solver": "MuJoCo (mujoco_warp)",
               "finalised": True, "degraded": False}
    fp, robot, _ = _run_report(imports_ok("Wb") + finalised("Om"), tmp_path,
                               sidecar=sidecar)
    assert fp["ok"] is True
    assert robot.labels[90][1] == GREEN


@pytest.mark.parametrize("log_text,label", [
    ("", "empty log"),
    ("INFO: nothing to see here\n", "log with no Newton markers"),
    (IMPORTS_OK, "imports OK only"),
    (IMPORTS_OK + ODE_FALLBACK, "logged 'falling back to ODE'"),
])
def test_no_verdict_ever_names_ode_as_the_engine_that_ran(tmp_path, log_text, label):
    """NAMING RULE: not one failure mode may attribute a run to ODE.

    src/ode was DELETED (commit bdc02139) and Newton is the only physics
    backend, so "ODE ran instead" is not a possible reading of ANY log -- yet
    _verdict() returned "ODE or unknown (no Newton finalise)" and "ODE (Newton
    fell back)" until 2026-08-08. That string is copied verbatim into every
    OmniBench result row by tests/benchmarks/omnibench/common/results.py, so it
    published measurements under the name of an engine that does not exist.
    The honest reading of every one of these logs is UNVERIFIED.
    """
    fp, _, _ = _run_report(log_text, tmp_path)
    assert not fp["ok"], label
    assert "ODE" not in fp["physics"], (
        "%s: verdict %r names ODE as the engine that ran" % (label, fp["physics"]))
    assert "ode" not in fp["physics"].lower(), (
        "%s: verdict %r names ode as the engine that ran" % (label, fp["physics"]))
    # The reason must still travel with the row, not just be dropped.
    assert fp["warnings"], "%s: no warning explains the non-verdict" % label


@pytest.mark.parametrize("scan,expected", [
    ({"finalised": True, "solver": "MuJoCo (cpu/mj_step)", "log_found": True}, "newton"),
    ({"ode_fallback": True, "log_found": True}, "unverified"),
    ({"log_found": True}, "unverified"),
    ({}, "unknown"),
])
def test_conformance_resolved_backend_never_says_ode(scan, expected):
    """Same rule, second consumer: omnisim/conformance/fingerprint.py.

    `resolved_backend()` collapses this module's log scan for the install
    conformance gate, and it returned "ode" on BOTH not-Newton branches. That
    value keys the tolerance-band lookup in conformance/compare.py and is hashed
    into `fingerprint_id()`, so the only check named after backend resolution
    selected an `{"ode": {"skip": true}}` band and reported PASS for a run where
    Newton demonstrably never drove the world.
    """
    from omnisim.conformance import fingerprint as cfp
    got = cfp.resolved_backend({"scan": scan})
    assert got == expected, (scan, got)
    assert got != "ode"


def test_unknown_prefix_is_not_accepted(tmp_path):
    """NEGATIVE CONTROL: the matchers dual-accept Wb|Om, not ANY prefix. A third
    prefix must go unrecognised, or the patterns are dangerously loose."""
    fp, _, _ = _run_report(imports_ok("Xx") + ode_fallback("Xx"), tmp_path,
                           pre_step=True)
    assert fp["scan"]["ode_fallback"] is False, (
        "an [XxNewtonBackend] line was accepted -- the tag pattern is too loose")
    # It degrades to the honest "cannot tell" verdict, not to a confident one --
    # and specifically not to a verdict naming ODE, which no longer exists.
    assert fp["physics"] == "UNVERIFIED (no Newton evidence in the log)"
