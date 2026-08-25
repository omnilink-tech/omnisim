# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
"""CI guards for the deterministic binary-level parity probe.

These are the PURE-PYTHON (numpy/stdlib only) parts -- they run in bare CI:
  * the single-sourced probe target generators are deterministic, limit-safe,
    and have the documented hold/ramp semantics;
  * the g1_parity_compare diff math is correct (PASS on identical traces, DIFF on
    a perturbed one, and it catches a commanded-target mismatch).

The trajectory generation (trainer harness) needs warp+newton+CUDA and the
binary-level comparison needs omnisim-bin, so those run as a LOCAL/GPU lane via
``scripts/dev/run_g1_parity_probe.ps1`` -- documented in
docs/developer/binary-parity-probe.md (this is the Tier-2 TODO that the
g1-spec-conformance test could never run on a CPU CI runner).
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

SPEC = pytest.importorskip("projects.policies.research.backends.g1_physics_spec")


# --------------------------------------------------------------------------- #
# Probe target generators (single source both sides import)
# --------------------------------------------------------------------------- #
def test_probe_targets_deterministic_and_sized():
    for k in (0, 1, 37, 199):
        a = SPEC.probe_targets(k)
        b = SPEC.probe_targets(k)
        assert a == b, "probe_targets is not deterministic"
        assert len(a) == 13


def test_probe_hold_is_the_pose():
    assert tuple(SPEC.probe_targets(5, sequence="hold")) == SPEC.PROBE_POSE_LEGS
    assert tuple(SPEC.probe_targets(5, amp=0.0)) == SPEC.PROBE_POSE_LEGS


def test_probe_sinusoid_within_urdf_limits():
    lo, hi, _vel, _eff = SPEC.leg_limits()
    for k in range(0, SPEC.PROBE_DURATION_TICKS + 1):
        tg = SPEC.probe_targets(k, sequence="sinusoid")
        for j in range(13):
            if lo[j] == 0.0 and hi[j] == 0.0:
                continue  # (0,0) sentinel == no position limit
            assert lo[j] - 1e-6 <= tg[j] <= hi[j] + 1e-6, \
                f"probe target out of limit at tick {k} joint {j}"


def test_probe_settle_ramps_from_straight_to_pose():
    st = SPEC.PROBE_SETTLE_TICKS
    first = SPEC.probe_settle_target(0, st)
    # starts near straight (all-zero), well inside the pose magnitudes
    assert max(abs(x) for x in first) < 0.05
    # ends exactly at the hold pose (ramp completes by 60% of the window)
    assert SPEC.probe_settle_target(st - 1, st) == list(SPEC.PROBE_POSE_LEGS)
    assert SPEC.probe_settle_target(int(0.6 * st), st) == list(SPEC.PROBE_POSE_LEGS)


def test_probe_gains_are_the_stand_gains():
    # the deterministic STAND gains (stiff position-PD), single-sourced.
    assert SPEC.PROBE_KE == 400.0
    assert SPEC.PROBE_KD == 60.0


# --------------------------------------------------------------------------- #
# Comparison harness math (synthetic traces)
# --------------------------------------------------------------------------- #
def _trace(side, qfn, n=40, settle=8):
    order = list(SPEC.LEGS_JOINTS)
    ticks = []
    for i in range(n):
        k = i - settle if i < settle else i - settle
        phase = "settle" if i < settle else "probe"
        tgt = SPEC.probe_targets(max(0, i - settle), sequence="hold")
        ticks.append({"k": k, "phase": phase, "target": [float(x) for x in tgt],
                      "q": [float(qfn(i, j)) for j in range(13)],
                      "base_pos": [0.0, 0.0, 0.78],
                      "base_rot": [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]})
    return {"schema": 1, "side": side,
            "meta": {"joint_order": order, "construction": side,
                     "sequence": "hold", "static_base": False,
                     "use_link_com": "1"},
            "ticks": ticks}


def _run_compare(tmp_path, a, b, *args):
    pa, pb = tmp_path / "a.json", tmp_path / "b.json"
    pa.write_text(json.dumps(a)); pb.write_text(json.dumps(b))
    r = subprocess.run(
        [sys.executable, str(REPO / "projects/policies/research/training/g1_parity_compare.py"),
         str(pa), str(pb), *args],
        capture_output=True, text=True)
    return r


def test_compare_pass_on_identical(tmp_path):
    import math
    q = lambda i, j: SPEC.PROBE_POSE_LEGS[j] + 0.01 * math.sin(i + j)
    a = _trace("trainer", q)
    b = _trace("deploy", q)
    r = _run_compare(tmp_path, a, b)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "PASS" in r.stdout


def test_compare_diff_on_perturbed(tmp_path):
    q = lambda i, j: SPEC.PROBE_POSE_LEGS[j]
    a = _trace("trainer", q)
    b = _trace("deploy", lambda i, j: SPEC.PROBE_POSE_LEGS[j] + (0.5 if j == 0 else 0.0))
    r = _run_compare(tmp_path, a, b)
    assert r.returncode == 1
    assert "DIFF" in r.stdout


def test_compare_catches_command_mismatch(tmp_path):
    # If the two sides ran different scripted motions, the diff is meaningless --
    # the harness must FAIL loudly rather than report a bogus parity number.
    q = lambda i, j: SPEC.PROBE_POSE_LEGS[j]
    a = _trace("trainer", q)
    b = _trace("deploy", q)
    b["ticks"][3]["target"][0] += 0.3   # corrupt one commanded target
    r = _run_compare(tmp_path, a, b)
    assert r.returncode == 3
    assert "commanded targets differ" in r.stdout
