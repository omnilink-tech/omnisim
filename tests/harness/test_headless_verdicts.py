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

"""Unit tests for headless_runner's always-fatal verdict patterns (Phases I1/I2).

The IPC-handshake and pairing diagnostics must be treated as controller-start
failures (always fatal), never as ordinary --fail-on-warning-gated warnings: each
one proves a robot executed zero simulation steps.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts" / "dev"))

from headless_runner import (  # noqa: E402
    RUNAWAY_WINDOW,
    connect_sidecar_failures,
    controller_start_failures,
    early_exit_cause_lines,
    looks_like_startup_race,
    platform_plugin_hint,
    read_runaway_samples,
    runaway_coverage_gap,
    runaway_verdict,
    write_runaway_sibling,
)


def test_handshake_timeout_is_fatal():
    line = ("ERROR: 'g1_walk' controller: libController did not complete the OmniSim IPC "
            "handshake within 5 seconds: it predates the handshake protocol")
    assert controller_start_failures(line) == [line]


def test_version_mismatch_is_fatal():
    line = ("Error: OmniSim IPC protocol version mismatch: the simulator speaks version 2 "
            "but this libController speaks version 1 (a build mismatch).")
    assert controller_start_failures(line) == [line]


def test_unrecognized_bytes_is_fatal():
    line = ("Error: the simulator sent unrecognized bytes where the OmniSim IPC handshake "
            "was expected.")
    assert controller_start_failures(line) == [line]


def test_watchdog_never_paired_is_fatal():
    line = ("WARNING: Controller 'bench_quit': started 60 seconds ago but never paired "
            "with the simulator (no IPC connection).")
    assert controller_start_failures(line) == [line]


def test_stale_instance_crossing_is_fatal():
    line = ("Error: this controller connected to a DIFFERENT simulator instance "
            "(launch nonce 12345, expected 678).")
    assert controller_start_failures(line) == [line]


def test_sidecar_entries_for_this_run_are_fatal(tmp_path):
    log = tmp_path / "run.log"
    log.write_text("=== OmniSim Log Started (pid=4242): 2026-07-18 ===\n")
    mine = ("run=4242: the OmniSim IPC handshake failed: engine and libController are "
            "different builds (pipe-x)")
    stale = "run=99: the simulator launched this controller but ... (pipe-y)"
    (tmp_path / "run.log.connect_error.txt").write_text(f"{stale}\n{mine}\n")
    assert connect_sidecar_failures(log.read_text(), log) == [mine]


def test_sidecar_absent_or_stale_only_is_clean(tmp_path):
    log = tmp_path / "run.log"
    log.write_text("=== OmniSim Log Started (pid=4242): 2026-07-18 ===\n")
    assert connect_sidecar_failures(log.read_text(), log) == []  # no sidecar file
    (tmp_path / "run.log.connect_error.txt").write_text("run=7: old failure (pipe)\n")
    assert connect_sidecar_failures(log.read_text(), log) == []  # stale entries only


def test_benign_lines_do_not_trip_the_patterns():
    benign = "\n".join([
        "INFO: Starting controller: python g1_walk.py",
        "INFO: 'g1_walk' controller exited successfully.",
        # Both the current tag and the post-rename one (the engine's C++ classes
        # are being renamed Wb* -> Om* and the bracketed tag follows the class
        # name). Neither may be mistaken for a controller-launch failure.
        "[OmNewtonBackend] world finalised (solver=mujoco)",
        "[OmNewtonBackend] world finalised (solver=mujoco)",
        "WARNING: something unrelated about textures",
    ])
    assert controller_start_failures(benign) == []


# ── the STARTUP RACE attribution ─────────────────────────────────────────────


def test_startup_race_is_recognised_when_nothing_else_failed():
    log = "\n".join([
        "=== OmniSim Log Started (pid=1): 2026-07-26 ===",
        "Qt Warning: QWaitCondition: Destroyed while threads are still waiting",
        "Qt Warning: QThreadStorage: entry 3 destroyed before end of thread",
    ])
    assert looks_like_startup_race(log) is True


def test_startup_race_never_excuses_a_world_that_errored():
    log = "\n".join([
        "Qt Warning: QWaitCondition: Destroyed while threads are still waiting",
        "ERROR: 'fall_through.wbt': expected '}' but found EOF.",
    ])
    assert looks_like_startup_race(log) is False


def test_clean_log_is_not_the_startup_race():
    assert looks_like_startup_race("=== OmniSim Log Started (pid=1) ===\n") is False


# ── the RUNAWAY (physically impossible end state) verdict ────────────────────
#
# The C2 numbers below are measured, not invented: the broken world's crate
# reaches z = -69,514 m at vz = -1167 m/s in an 8 s wall-clock run, the fixed
# one settles at z = 0.249.


def _series(zs, dt=0.128, key="CRATE_BOT"):
    return [(i * dt, {key: [0.0, 0.0, z]}) for i, z in enumerate(zs)]


def test_free_fall_through_a_missing_floor_fails():
    header = {"static_floor_z": -0.05}
    # z = 1 - g t^2 / 2, sampled every 0.128 s: never lands, always accelerating.
    zs = [1.0 - 4.905 * (i * 0.128) ** 2 for i in range(40)]
    offenders = runaway_verdict(header, _series(zs))
    assert [o["body"] for o in offenders] == ["CRATE_BOT"]
    assert offenders[0]["z_final"] < -100.0
    assert offenders[0]["vz_final"] < -25.0


def test_a_body_that_lands_and_rests_passes():
    header = {"static_floor_z": -0.05}
    zs = [1.0, 0.92, 0.75, 0.5, 0.3, 0.25, 0.249, 0.249, 0.249, 0.249,
          0.249, 0.249, 0.249, 0.249]
    assert runaway_verdict(header, _series(zs)) == []


def test_a_body_still_falling_ABOVE_the_floor_passes():
    """Mid-air at exit is legal: a run can simply end during a legitimate drop."""
    header = {"static_floor_z": -0.05}
    zs = [40.0 - 4.905 * (i * 0.128) ** 2 for i in range(20)]   # from 40 m, still >0
    assert min(z for _t, row in _series(zs) for z in [row["CRATE_BOT"][2]]) > 0.0
    assert runaway_verdict(header, _series(zs)) == []


def test_a_body_descending_at_constant_speed_passes():
    """A lift / a drone holding a descent rate is fast but NOT accelerating."""
    header = {"static_floor_z": 0.0}
    zs = [-1.0 * i for i in range(30)]      # -7.8 m/s, perfectly steady
    assert runaway_verdict(header, _series(zs)) == []


def test_the_absolute_z_bound_fires_without_a_floor_reference():
    header = {"static_floor_z": None}
    zs = [-1000.0 * i for i in range(1, 10)]
    offenders = runaway_verdict(header, _series(zs), z_limit=1000.0)
    assert offenders and "exceeds the 1000 m bound" in " ".join(offenders[0]["reasons"])


def test_too_few_samples_says_nothing():
    """A guess is worse than silence: no velocity, no verdict."""
    assert runaway_verdict({"static_floor_z": 0.0}, _series([1.0, 0.5])) == []


def test_only_the_offending_body_is_named():
    header = {"static_floor_z": -0.05}
    rows = []
    for i in range(40):
        t = i * 0.128
        rows.append((t, {"GOOD": [0.0, 0.0, 0.25],
                         "GONE": [0.0, 0.0, 1.0 - 4.905 * t * t]}))
    assert [o["body"] for o in runaway_verdict(header, rows)] == ["GONE"]


def test_samples_reader_tolerates_a_truncated_final_line(tmp_path):
    p = tmp_path / "run.log.runaway.jsonl"
    p.write_text(
        '{"kind":"header","dt_ms":16,"bodies":[{"key":"A"}]}\n'
        '{"kind":"floor","static_floor_z":-0.05,"static_bodies":1}\n'
        '{"kind":"s","t":0.128,"z":{"A":[0,0,1.0]}}\n'
        '{"kind":"s","t":0.256,"z":{"A":[0,0,0.9\n')          # killed mid-write
    data = read_runaway_samples(p)
    assert data["present"] and data["complete"] is False
    assert data["header"]["static_floor_z"] == -0.05      # the floor record merged
    assert len(data["rows"]) == 1                         # the half line is dropped


def test_samples_reader_reports_a_missing_file(tmp_path):
    data = read_runaway_samples(tmp_path / "nope.jsonl")
    assert data == {"present": False, "header": {}, "rows": [], "complete": False}


# ── the COVERAGE GATE: "no verdict" must never be read as a PASS ─────────────
#
# `runaway_verdict` staying silent when it cannot compute a velocity is CORRECT
# (test_too_few_samples_says_nothing, above, pins that). The bug was on the
# CALLER side: main() failed only `if not rows`, so an empty verdict from an
# unmeasurable run reached the same `[headless] PASS` as a genuinely clean one.
# `runaway_coverage_gap` is the missing question -- "could this evidence have
# produced a verdict at all?" -- and main() FAILs on it.


def test_no_tracked_bodies_is_a_coverage_gap_not_a_pass():
    """The measured shape: a world whose dynamic bodies sit under a Group (or
    inside a PROTO) reports `bodies: []`, so every sample row is an empty `z`
    map. `rows` is NON-empty, so the runner's `if not rows` guard misses, and
    the verdict is [] no matter what the physics did."""
    header = {"bodies": [], "static_floor_z": -0.05}
    rows = [(i * 0.128, {}) for i in range(40)]
    assert rows                                     # the old guard sees content
    assert runaway_verdict(header, rows) == []      # ...and the verdict is silent
    gap = runaway_coverage_gap(header, rows)
    assert gap is not None
    assert "ZERO dynamic bodies" in gap


def test_too_few_samples_is_a_coverage_gap_not_a_pass():
    """Same runaway, two sample counts. Measured on the C2 fall-through tail:
    a body 59 km below the floor and still accelerating scores [] at 4 samples
    and FAILs at 5 -- so the 4-sample run must not be allowed to PASS."""
    header = {"bodies": [{"key": "CRATE_BOT"}], "static_floor_z": -0.05}
    zs = [1.0 - 4.905 * t * t for t in (110.0, 110.128, 110.256, 110.384, 110.512)]
    rows = [(110.0 + i * 0.128, {"CRATE_BOT": [0.0, 0.0, z]})
            for i, z in enumerate(zs)]
    short = rows[:RUNAWAY_WINDOW + 1]
    assert runaway_verdict(header, short) == []                 # silent
    assert runaway_coverage_gap(header, short) is not None      # ...and now loud
    assert [o["body"] for o in runaway_verdict(header, rows)] == ["CRATE_BOT"]
    assert runaway_coverage_gap(header, rows) is None


def test_a_body_in_the_header_that_never_sampled_is_a_coverage_gap():
    """A roster is not evidence: a body listed in the header whose pose never
    came back (a failed getPosition every tick) leaves nothing to judge."""
    header = {"bodies": [{"key": "GHOST"}], "static_floor_z": 0.0}
    rows = [(i * 0.128, {}) for i in range(40)]
    assert runaway_coverage_gap(header, rows) is not None


def test_a_healthy_measured_run_has_no_coverage_gap():
    """The gate must stay out of the way of the case it is not about: a real
    roster, enough samples, body resting on the floor -> silence means PASS."""
    header = {"bodies": [{"key": "CRATE_BOT"}], "static_floor_z": -0.05}
    rows = _series([1.0, 0.92, 0.75, 0.5, 0.3, 0.25, 0.249, 0.249, 0.249, 0.249])
    assert runaway_coverage_gap(header, rows) is None
    assert runaway_verdict(header, rows) == []


def test_sibling_world_is_a_sibling_and_carries_the_watchdog(tmp_path):
    world = tmp_path / "fall_through.wbt"
    world.write_text("#VRML_SIM R2025a utf8\nWorldInfo {\n}\n")
    out = tmp_path / "logs" / "run.log.runaway.jsonl"
    sibling = write_runaway_sibling(world, out, period_steps=4)
    assert sibling.parent == world.parent          # URDF / PROTO paths resolve
    assert sibling.name == ".omnisim_runaway_fall_through.wbt"
    text = sibling.read_text()
    assert text.startswith(world.read_text())     # original world untouched
    assert 'controller "runaway_watchdog"' in text
    assert "synchronization FALSE" in text         # must never stall the world
    assert "--period-steps=4" in text
    assert "\\" not in text.split("--out=")[1].split('"')[0]   # posix path in .wbt


# ---- Early-exit cause (public issue #6) -------------------------------------
# The engine's Qt platform-plugin abort is recorded ONLY in the log, as a
# "Qt Fatal:" line with a header-only log around it; the runner used to print
# just "simulator exited early with code 3". Measured 2026-08-29 on Windows
# with QT_QPA_PLATFORM=xcb -- this is that log, verbatim.

_XCB_ABORT_LOG = (
    "=== OmniSim Log Started (pid=1560): 2026-08-29 13:10:49 ===\n"
    "Qt Warning: Could not find the Qt platform plugin \"xcb\" in \"\"\n"
    "Qt Fatal: This application failed to start because no Qt platform plugin "
    "could be initialized. Reinstalling the application may fix this problem.\n"
    "\n"
    "Available platform plugins are: minimal, windows.\n"
)


def test_early_exit_cause_surfaces_the_qt_fatal_line():
    causes = early_exit_cause_lines(_XCB_ABORT_LOG)
    assert len(causes) == 1
    assert causes[0].startswith("Qt Fatal: This application failed to start")


def test_early_exit_cause_keeps_log_order_and_caps_a_flood():
    text = "\n".join(["INFO: noise"] + [f"ERROR: e{i}" for i in range(40)] + ["FATAL: last"])
    causes = early_exit_cause_lines(text, limit=5)
    assert causes == [f"ERROR: e{i}" for i in range(5)]


def test_early_exit_cause_is_empty_for_a_clean_log():
    assert early_exit_cause_lines("=== OmniSim Log Started ===\nINFO: fine\nWARNING: meh\n") == []


def test_platform_plugin_hint_fires_only_on_the_signature():
    hint = platform_plugin_hint(_XCB_ABORT_LOG)
    assert hint and hint[0].startswith("QT PLATFORM PLUGIN FAILED")
    assert any("BEFORE the world was opened" in ln for ln in hint)
    assert platform_plugin_hint("ERROR: 'x.omniworld': Failed to load due to syntax error(s).") == []


def test_platform_plugin_abort_is_not_the_startup_race():
    # A header-only log with a Qt Fatal line must never be retried as "the flake".
    assert looks_like_startup_race(_XCB_ABORT_LOG) is False
