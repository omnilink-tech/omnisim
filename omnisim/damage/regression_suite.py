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

"""Damage-system numerical regression suite.

Each test runs a deterministic scenario headless, queries the
supervisor's `damage_geometry_stats` and `damage_state`, and asserts
that key metrics fall in expected ranges. Catches regressions in the
deformation pipeline as Phases 16b/16c/16d (spring coupling, real
normals, relaxation) get layered in.

Used by:
    python -m omnisim damage-regression [--filter SUBSTRING]

Exits non-zero if any test fails. Prints per-test stat-vs-range table
on success and on failure so a maintainer can see drift.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

from ..paths import REPO_ROOT
from .headless_test import DEFAULT_WORLD, HeadlessOmniSim, SupervisorClient


# ---------------------------------------------------------------------------
# Range / assertion machinery
# ---------------------------------------------------------------------------


@dataclass
class Range:
    """Closed-interval range for a numeric stat. Both bounds inclusive.
    Use None for "unbounded on this side".
    """
    low: float | None = None
    high: float | None = None

    def contains(self, x: float) -> bool:
        if self.low is not None and x < self.low:
            return False
        if self.high is not None and x > self.high:
            return False
        return True

    def __repr__(self) -> str:
        lo = "-inf" if self.low is None else f"{self.low}"
        hi = "+inf" if self.high is None else f"{self.high}"
        return f"[{lo}, {hi}]"


@dataclass
class TestResult:
    name: str
    ok: bool
    duration_s: float
    failures: list[tuple[str, str]] = field(default_factory=list)
    observed: dict[str, float] = field(default_factory=dict)


def expect(failures: list, name: str, value: float, rng: Range) -> None:
    """Helper: append a failure tuple if value falls outside rng."""
    if not rng.contains(value):
        failures.append((name, f"{value!r} not in {rng}"))


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def _connect(client: SupervisorClient) -> None:
    client.connect()


def _run_with_supervisor(test_name: str, body: callable) -> TestResult:
    """Spawn headless OmniSim, connect, run `body(client) -> failures, observed`,
    tear down.
    """
    t0 = time.monotonic()
    failures: list = []
    observed: dict = {}
    try:
        with HeadlessOmniSim(DEFAULT_WORLD,
                            log_path=REPO_ROOT / f"tmp_regression_{test_name}.log"):
            client = SupervisorClient(connect_timeout_s=30.0)
            _connect(client)
            try:
                failures, observed = body(client)
            finally:
                client.close()
    except Exception as exc:  # noqa: BLE001
        failures.append(("infra", f"exception: {exc}"))
    return TestResult(
        name=test_name,
        ok=not failures,
        duration_s=time.monotonic() - t0,
        failures=failures,
        observed=observed,
    )


def test_idle_baseline(client: SupervisorClient):
    """Stationary chassis, no impacts. Vertex buffer should be at most
    barely-touched from box landings on first sim step. Failure here
    means deformation is firing without impacts.
    """
    failures: list = []
    observed: dict = {}
    # Single sim step to warm up the supervisor's main loop after
    # connect; don't do a long run.
    client.call("damage_reset")
    client.call("step", {"steps": 1})
    stats = client.call("damage_geometry_stats").get("chassis")
    if stats is None:
        # No vertex buffer initialized yet — that's the truly-pristine
        # state. Pass.
        observed["state"] = "no_buffer"
        return failures, observed
    observed["displaced"] = stats["displaced_count"]
    observed["max_displacement_cm"] = stats["max_displacement_m"] * 100
    expect(failures, "displaced_count_low",
           stats["displaced_count"], Range(low=0, high=20))
    expect(failures, "max_displacement_cm",
           stats["max_displacement_m"] * 100, Range(low=0.0, high=2.0))
    return failures, observed


def test_broken_inject(client: SupervisorClient):
    """Inject chassis: broken. Phase 14b's additive uniform crumple
    should push current_crumple to 0.30 and displace nearly all vertices.
    """
    failures: list = []
    observed: dict = {}
    client.call("damage_reset")
    client.call("damage_inject", {"part": "chassis", "state": "broken"})
    client.call("step", {"steps": 1})
    stats = client.call("damage_geometry_stats")["chassis"]
    state = client.call("damage_state")["damage"]["chassis"]
    observed["chassis_state"] = state["state"]
    observed["current_crumple"] = stats["current_crumple"]
    observed["displaced"] = stats["displaced_count"]
    observed["max_cm"] = stats["max_displacement_m"] * 100
    observed["rms_cm"] = stats["rms_displacement_m"] * 100
    expect(failures, "current_crumple",
           stats["current_crumple"], Range(low=0.299, high=0.301))
    expect(failures, "displaced_count",
           stats["displaced_count"], Range(low=130, high=150))
    expect(failures, "max_displacement_cm",
           stats["max_displacement_m"] * 100, Range(low=2.0, high=12.0))
    expect(failures, "rms_displacement_cm",
           stats["rms_displacement_m"] * 100, Range(low=1.0, high=4.0))
    expect(failures, "chassis_state",
           1.0 if state["state"] == "broken" else 0.0, Range(low=1.0, high=1.0))
    return failures, observed


def test_box_drop_30s(client: SupervisorClient):
    """30s of the standard box-drop schedule. Verifies the synthetic-
    chassis impact path drives state transitions, dent accumulation,
    and overall mesh deformation.
    """
    failures: list = []
    observed: dict = {}
    client.call("damage_reset")
    sim_step_ms = client.call("sim_state")["basic_time_step_ms"]
    target_steps = int(30000 / sim_step_ms)
    client.call("step", {"steps": target_steps})
    stats = client.call("damage_geometry_stats")["chassis"]
    state = client.call("damage_state")
    observed["chassis_state"] = state["damage"]["chassis"]["state"]
    observed["events_total"] = state["events_total"]
    observed["displaced"] = stats["displaced_count"]
    observed["max_cm"] = stats["max_displacement_m"] * 100
    observed["rms_cm"] = stats["rms_displacement_m"] * 100
    observed["current_crumple"] = stats["current_crumple"]
    expect(failures, "events_total",
           state["events_total"], Range(low=10000, high=200000))
    expect(failures, "current_crumple",
           stats["current_crumple"], Range(low=0.05, high=0.301))
    expect(failures, "displaced_count",
           stats["displaced_count"], Range(low=80, high=150))
    expect(failures, "rms_displacement_cm",
           stats["rms_displacement_m"] * 100, Range(low=1.0, high=15.0))
    chassis_state = state["damage"]["chassis"]["state"]
    expect(failures, "chassis_state_progressed",
           1.0 if chassis_state in ("scuffed", "damaged", "broken") else 0.0,
           Range(low=1.0, high=1.0))
    return failures, observed


def test_box_drop_extreme_fracture(client: SupervisorClient):
    """Phase 17 fracture regression. 60s of the standard box-drop
    schedule. Empirically a 30s run produces ~5cm peak strain and
    no fragments; a 60s run accumulates enough sustained yielding
    that one or more fracture islands form and spawn fragments.

    Asserts:
      - chassis state == "broken" (took the full damage progression)
      - max_strain_m >= FRACTURE_STRAIN_M (some vertex past plastic
        yield)
      - fragments_spawned_total >= 1 (at least one chunk torn off)
      - any spawned fragment is a real Solid in the scene tree
    """
    failures: list = []
    observed: dict = {}
    client.call("damage_reset")
    sim_step_ms = client.call("sim_state")["basic_time_step_ms"]
    target_steps = int(60000 / sim_step_ms)
    client.call("step", {"steps": target_steps})

    stats = client.call("damage_geometry_stats")
    chassis = stats["chassis"]
    state = client.call("damage_state")
    fragments_total = stats.get("__fragments_spawned_total", 0)
    observed["chassis_state"] = state["damage"]["chassis"]["state"]
    observed["events_total"] = state["events_total"]
    observed["max_strain_cm"] = chassis["max_strain_m"] * 100
    observed["strained_vertex_count"] = chassis["strained_vertex_count"]
    observed["fragments_spawned_total"] = fragments_total
    observed["fragments_alive"] = stats.get("__fragments_alive", 0)

    expect(failures, "chassis_state_broken",
           1.0 if state["damage"]["chassis"]["state"] == "broken" else 0.0,
           Range(low=1.0, high=1.0))
    expect(failures, "max_strain_cm",
           chassis["max_strain_m"] * 100, Range(low=4.0, high=200.0))
    expect(failures, "fragments_spawned_total",
           fragments_total, Range(low=1, high=50))
    expect(failures, "fragments_alive",
           stats.get("__fragments_alive", 0), Range(low=1, high=50))

    # Verify at least one fragment is actually present in the scene
    # (catches a spawn that succeeded but somehow got removed).
    if fragments_total > 0:
        tree = client.call("scene_tree")
        live_fragments = sum(
            1 for n in tree.get("nodes", [])
            if (n.get("def") or "").startswith("damage_fragment_")
        )
        observed["scene_tree_fragments"] = live_fragments
        expect(failures, "scene_tree_fragments",
               live_fragments, Range(low=1, high=50))

    return failures, observed


def test_repair_round_trip(client: SupervisorClient):
    """Phase 18 repair regression. Damage chassis to broken via inject,
    call damage_heal_to_pristine (instant repair), assert chassis is
    pristine and vertex buffer is at baseline.

    Validates Phase 18d wire-protocol heal commands AND the heal-side
    state-transition event emission. Uses instant heal rather than
    time-based regen to avoid fighting continuous box impacts (regen
    rate is bounded; impacts arrive faster than any safe heal rate).
    Continuous regen is exercised in test_repair_regen_dt.
    """
    failures: list = []
    observed: dict = {}
    client.call("damage_reset")
    # Damage to broken
    client.call("damage_inject", {"part": "chassis", "state": "broken"})
    client.call("step", {"steps": 1})
    pre = client.call("damage_state")
    pre_geom = client.call("damage_geometry_stats")
    observed["pre_chassis_state"] = pre["damage"]["chassis"]["state"]
    observed["pre_displaced"] = pre_geom["chassis"]["displaced_count"]

    # Instant heal back to pristine
    client.call("damage_heal_to_pristine")
    client.call("step", {"steps": 1})

    state = client.call("damage_state")
    stats = client.call("damage_geometry_stats")
    chassis_state = state["damage"]["chassis"]
    chassis_geom = stats["chassis"]
    observed["post_chassis_state"] = chassis_state["state"]
    observed["post_chassis_hp"] = chassis_state["hp"]
    observed["post_displaced"] = chassis_geom["displaced_count"]
    observed["post_max_cm"] = chassis_geom["max_displacement_m"] * 100
    observed["post_current_crumple"] = chassis_geom["current_crumple"]

    expect(failures, "pre_was_broken",
           1.0 if pre["damage"]["chassis"]["state"] == "broken" else 0.0,
           Range(low=1.0, high=1.0))
    expect(failures, "post_chassis_state",
           1.0 if chassis_state["state"] == "pristine" else 0.0,
           Range(low=1.0, high=1.0))
    # heal_to_pristine sets HP to hp_max instantly; one step of
    # ongoing box impacts can knock 5-15% off before we sample. So the
    # tolerance accepts >= 80% — well past any "still broken" signature
    # but tolerant of one-step damage.
    expect(failures, "post_chassis_hp_full",
           chassis_state["hp"] / chassis_state["hp_max"],
           Range(low=0.80, high=1.001))
    # After heal_to_pristine, the vertex buffer is wiped to baseline.
    # The post-heal step can apply tiny fresh dents from boxes still
    # resting on the chassis, so allow up to 50 displaced and 5cm max.
    expect(failures, "post_displaced",
           chassis_geom["displaced_count"], Range(low=0, high=50))
    expect(failures, "post_max_cm",
           chassis_geom["max_displacement_m"] * 100, Range(low=0.0, high=5.0))
    expect(failures, "post_current_crumple",
           chassis_geom["current_crumple"], Range(low=0.0, high=0.001))

    # Verify a state_transition event was emitted on heal (Phase 18a)
    events = client.call("damage_events", {"limit": 100})
    transitions = [e for e in events.get("events", [])
                    if e.get("type") == "state_transition"
                    and e.get("part") == "chassis"]
    heal_transitions = [t for t in transitions if t.get("to_state") == "pristine"]
    observed["heal_transitions_to_pristine"] = len(heal_transitions)
    expect(failures, "heal_transition_emitted",
           len(heal_transitions), Range(low=1, high=99))

    return failures, observed


def test_repair_regen_dt(client: SupervisorClient):
    """Phase 18a/b time-based regen. Damage to broken via inject, set
    a high heal rate so it overcomes any incidental incoming damage,
    run 5s sim, assert HP and vertex buffer are at least partially
    restored.

    Doesn't require full pristine — just demonstrates regen is
    actively pulling state back up over time.
    """
    failures: list = []
    observed: dict = {}
    client.call("damage_reset")
    client.call("damage_inject", {"part": "chassis", "state": "broken"})
    client.call("step", {"steps": 1})

    # Heal rate must overwhelm continuous box-impact damage. Each
    # synthetic chassis impact drops ~30-60 HP; with multiple impacts
    # per step (16 ms), the effective damage rate is on the order of
    # 5000+ HP/s. We pick a heal rate that decisively wins so the
    # test exercises the regen path rather than the steady-state
    # damage-vs-heal equilibrium (which is real but a different test).
    client.call("damage_set_heal_rate", {
        "rate_hp": 8000.0, "rate_mesh": 1.0,
        "parts": ["chassis"],
    })
    sim_step_ms = client.call("sim_state")["basic_time_step_ms"]
    target_steps = int(5000 / sim_step_ms)
    client.call("step", {"steps": target_steps})

    state = client.call("damage_state")
    stats = client.call("damage_geometry_stats")
    chassis_state = state["damage"]["chassis"]
    chassis_geom = stats["chassis"]
    observed["chassis_state"] = chassis_state["state"]
    observed["chassis_hp_frac"] = chassis_state["hp"] / chassis_state["hp_max"]
    observed["displaced"] = chassis_geom["displaced_count"]
    observed["max_displacement_cm"] = chassis_geom["max_displacement_m"] * 100

    # HP should be at least 80% (well past scuffed band).
    expect(failures, "hp_recovered",
           chassis_state["hp"] / chassis_state["hp_max"],
           Range(low=0.80, high=1.001))
    # State transitioned upward (no longer broken).
    expect(failures, "state_recovered",
           1.0 if chassis_state["state"] != "broken" else 0.0,
           Range(low=1.0, high=1.0))
    return failures, observed


def test_reset_round_trip(client: SupervisorClient):
    """Damage chassis to broken, reset, verify vertex buffer returns
    to (near-)baseline. Tolerance covers 1-2 step's worth of fresh
    contact dent from boxes still resting on the chassis.
    """
    failures: list = []
    observed: dict = {}
    client.call("damage_reset")
    client.call("damage_inject", {"part": "chassis", "state": "broken"})
    client.call("step", {"steps": 1})
    pre = client.call("damage_geometry_stats")["chassis"]
    client.call("damage_reset")
    client.call("step", {"steps": 1})
    post = client.call("damage_geometry_stats")
    observed["pre_displaced"] = pre["displaced_count"]
    observed["pre_max_cm"] = pre["max_displacement_m"] * 100
    if "chassis" in post:
        observed["post_displaced"] = post["chassis"]["displaced_count"]
        observed["post_max_cm"] = post["chassis"]["max_displacement_m"] * 100
        # After reset the buffer is wiped; one extra step can re-init
        # and apply fresh dents from boxes still resting on the chassis
        # transmitting impulse. Phase 16b spring coupling and Phase 16c
        # real normals both make a single dent touch slightly more
        # vertices than the original Phase 15 radial-only path, so the
        # tolerance allows up to 50 displaced and 3cm max — well below
        # any state-transition signature (>=130 displaced).
        expect(failures, "post_displaced",
               post["chassis"]["displaced_count"], Range(low=0, high=50))
        # max post-reset displacement is bounded by DEFORM_MAG_CAP
        # (10cm) per single-step impact. Typical fresh dent from one
        # resting-box impulse is 1-4cm; tolerance set to 5cm so a
        # single beefy impact in the post-reset step doesn't false-fail.
        expect(failures, "post_max_displacement_cm",
               post["chassis"]["max_displacement_m"] * 100,
               Range(low=0.0, high=5.0))
        expect(failures, "post_current_crumple",
               post["chassis"]["current_crumple"], Range(low=0.0, high=0.001))
    else:
        # Buffer fully cleared, no re-init yet — strongest pass case.
        observed["post_displaced"] = 0
        observed["post_max_cm"] = 0.0
    # Pre-reset state must have been heavily damaged (sanity: did inject work?)
    expect(failures, "pre_displaced",
           pre["displaced_count"], Range(low=130, high=150))
    return failures, observed


TESTS = [
    ("idle_baseline",         test_idle_baseline),
    ("broken_inject",         test_broken_inject),
    ("box_drop_30s",          test_box_drop_30s),
    ("box_drop_extreme",      test_box_drop_extreme_fracture),
    ("repair_round_trip",     test_repair_round_trip),
    ("repair_regen_dt",       test_repair_regen_dt),
    ("reset_round_trip",      test_reset_round_trip),
]


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="omnisim damage-regression",
        description="Damage-system numerical regression suite",
    )
    parser.add_argument("--filter",
                        help="run only tests whose name contains this substring")
    args = parser.parse_args(argv)

    selected = [(n, fn) for n, fn in TESTS
                if args.filter is None or args.filter in n]
    if not selected:
        print(f"no tests match filter {args.filter!r}", file=sys.stderr)
        return 1

    results: list[TestResult] = []
    for name, fn in selected:
        print(f"=== {name} ===")
        result = _run_with_supervisor(name, fn)
        results.append(result)
        status = "PASS" if result.ok else "FAIL"
        print(f"  {status} in {result.duration_s:.1f}s")
        for k, v in result.observed.items():
            print(f"    {k:>26}: {v}")
        for k, msg in result.failures:
            print(f"    {k}: {msg}")
        print()

    n = len(results)
    n_pass = sum(1 for r in results if r.ok)
    total_s = sum(r.duration_s for r in results)
    print(f"=== {n_pass}/{n} passed in {total_s:.1f}s ===")
    return 0 if n_pass == n else 1


if __name__ == "__main__":
    sys.exit(main())
