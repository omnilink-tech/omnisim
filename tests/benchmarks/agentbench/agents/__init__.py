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

"""Phase-0 scripted agents (SPEC 7).

``agent/`` (singular) is reserved for the Phase-1 sim-agnostic LLM loop.
``agents/`` (this package) holds the no-LLM oracle and negative controls that
must be green before a token is spent.

Each entry is ``(task_id, agent_name) -> callable(ctx) -> AgentResult``, plus
``expect_failures`` -- the assertions a negative control is DESIGNED to fail.
The runner checks that expectation, so "the null failed" is never accepted
without "...and it failed for the reason we claimed".
"""

from __future__ import annotations

from agentbench.agents import (
    a1_fixtures_extra, b1_fixtures, b2_fixtures, c1_fixtures, external, llm,
    null, oracle_a1, oracle_b1, oracle_b3, oracle_c2)

# The API-driven LLM runner (agents/llm.py + runner/). Registered for every
# task: its outcome is UNKNOWN by construction, so expect_pass is None -- the
# same convention `external` uses. ``llm`` reads AGENTBENCH_CONDITION from the
# environment; ``llm_shell`` / ``llm_tools`` pin the condition, so an A/B pair
# cannot be produced by forgetting to set it.
_LLM_AGENTS = {"llm": llm.run, "llm_shell": llm.run_shell_only,
               "llm_tools": llm.run_full_surface}

REGISTRY = {
    ("A1_husky_swarm_10", "oracle"): {
        "fn": oracle_a1.run, "expect_pass": True, "expect_failures": None},
    # Phase 1: score an artifact produced OUTSIDE this package (an LLM
    # run, a human, a competitor). No pass/fail expectation -- the whole
    # point is that the outcome is unknown in advance.
    ("A1_husky_swarm_10", "external"): {
        "fn": external.run, "expect_pass": None, "expect_failures": None},
    ("A1_husky_swarm_10", "null"): {
        "fn": null.run, "expect_pass": False, "expect_failures": None},
    # Predicted {A1.4, A1.5, A1.6}; MEASURED {A1.4, A1.5, A1.6, A1.8}. A1.8
    # is in the set because a robot that never moved has no bearing, which
    # the grader now says explicitly instead of scoring atan2(0, 0).
    ("A1_husky_swarm_10", "wrong"): {
        "fn": null.run_a1_static, "expect_pass": False,
        "expect_failures": {"A1.4", "A1.5", "A1.6", "A1.8"}},
    ("A1_husky_swarm_10", "parade"): {
        "fn": null.run_a1_parade, "expect_pass": False,
        "expect_failures": {"A1.8"}},
    # Targeted negatives for the six A1 assertions that had none (the 5.5
    # red-evidence rule; fixtures + rationale in agents/a1_fixtures_extra.py,
    # committed verdicts in phase0_validation/). Registered via the module's
    # own REGISTRY-shaped export below; where a fixture's MEASURED red set
    # differs from its pre-registered prediction, the override here carries
    # the measured set and the prediction stays visible in
    # a1_fixtures_extra.PREDICTED_FAILURES.

    ("B3_measure_and_report", "oracle"): {
        "fn": oracle_b3.run, "expect_pass": True, "expect_failures": None},
    ("B3_measure_and_report", "null"): {
        "fn": null.run, "expect_pass": False, "expect_failures": None},
    ("B3_measure_and_report", "wrong"): {
        "fn": null.run_b3_wrong, "expect_pass": False,
        "expect_failures": {"B3.2", "B3.4"}},

    ("C2_fall_through_floor", "oracle"): {
        "fn": oracle_c2.run, "expect_pass": True, "expect_failures": None},
    # Phase W Claude Code lane: grade an artifact a headless Claude Code
    # session produced in a staged product workspace (cc_lane/run_cc_cell.py).
    # Same unknown-outcome convention as A1's external entry.
    ("C2_fall_through_floor", "external"): {
        "fn": external.run, "expect_pass": None, "expect_failures": None},
    ("C2_fall_through_floor", "null"): {
        "fn": null.run, "expect_pass": False, "expect_failures": None},
    # Measured: the freeze cheat passes C2.1 and C2.2 -- the run is clean and
    # the robot "stays up" -- and fails exactly the three assertions that
    # encode "you froze it 0.75 m above the floor".
    ("C2_fall_through_floor", "wrong"): {
        "fn": null.run_c2_freeze, "expect_pass": False,
        "expect_failures": {"C2.3", "C2.4", "C2.5"}},
}

# The a1_fixtures_extra module exports its entries REGISTRY-shaped (the same
# convention the b1/b2/c1 fixture modules use for the orchestrator's merge).
REGISTRY.update(a1_fixtures_extra.REGISTRY)

# Predicted {A1.2}; MEASURED {A1.2, A1.5, A1.6, A1.8}. The collateral is a
# real engine behaviour the fixture surfaced, not fixture sloppiness: OmniSim
# keys a controller's IPC pipe by the robot's name (WbController,
# tmp/ipc/<encodedName>), so the twin named husky_0 gets a colliding pipe --
# both controller processes START (A1.4 stays green at 10 starts) but one
# never pairs, its robot never moves (net displacement 0.0 m -> A1.5, A1.6),
# and an unmoved robot has no bearing (-> A1.8). Committed evidence:
# phase0_validation/A1_husky_swarm_10.dupname.verdict.json.
REGISTRY[("A1_husky_swarm_10", "dupname")]["expect_failures"] = {
    "A1.2", "A1.5", "A1.6", "A1.8"}

# The B1/B2/C1 lanes export their fixtures REGISTRY-shaped (same convention
# as a1_fixtures_extra above); merged here, BEFORE the LLM fan-out loop, so
# the three new tasks inherit the llm/llm_shell/llm_tools agents too.
# B1 ships no oracle of its own (its module docstring says the orchestrator
# owns it): oracle_b1 measures the scene and answers honestly, registered
# here so the SPEC 7.1 oracle-PASSes-every-task gate covers B1.
REGISTRY.update(b1_fixtures.REGISTRY)
REGISTRY[("B1_overlap_audit", "oracle")] = {
    "fn": oracle_b1.run, "expect_pass": True, "expect_failures": None}
REGISTRY.update(b2_fixtures.REGISTRY)
REGISTRY.update(c1_fixtures.REGISTRY)

# Phase W Claude Code lane, freeze v2 Amendment 2: the remaining Lane B
# product-lane registrations (B1/B2/B3/C1), mirroring Amendment 1's C2 entry
# -- the same unknown-outcome convention as A1's external entry
# (expect_pass=None declares no expectation and validates nothing; it only
# lets the real grading pipeline score an artifact produced outside the
# runner). Artifact conventions per task live in the deliberately unfrozen
# agents/external.py: B1/B3 grade an ANSWER file against the staged world's
# recorder evidence; B2 grades the modified world (+ the answer channel for
# its committed proof); C1 grades the repaired world.
# R1 (robotics tier) is an AUTHORING task: the agent builds the world, the
# robot and the controller from an empty project plus the frozen obstacle
# spec, so its deliverable is a world like A1's.
# R4 (robotics tier) is an AUTHORING task like R1, and its absence here is the
# SAME DEFECT the null-agent comment below already records, recurring: the
# first R4/omnisim cell spent its whole 2700 s budget, delivered a world and a
# controller, and died at grading with "skip: no 'external' agent for
# R4_mobile_manipulation" (2026-08-11). It bit the OmniSim arm and not the
# upstream one because only the OmniSim path grades through run_agentbench --
# cc_lane's webots arm calls the grader in process and never asks this
# registry -- so R4 looked expressible on both and was expressible on one.
# ⚠ THIS FILE IS A FROZEN PATH. The line above is part of R4's outstanding
# freeze amendment (tasks/R4_mobile_manipulation/meta.json "freeze"), not a
# separate change, and the manifest must be regenerated with it.
for _external_task in ("B1_overlap_audit", "B2_subject_in_frame",
                       "B3_measure_and_report", "C1_parse_error_fix",
                       "R1_lidar_nav", "R2_arm_reach",
                       "R3_pick_and_place", "R4_mobile_manipulation"):
    REGISTRY[(_external_task, "external")] = {
        "fn": external.run, "expect_pass": None, "expect_failures": None}

# The null agent must exist for EVERY task: SPEC 7.1's rule is that no task is
# passable by doing nothing, and a task with no null entry is a task where
# nobody ever checked. R1's absence here is what made its first run die at
# grading with "no 'external' agent for R1_lidar_nav".
for _null_task in ("R1_lidar_nav", "R2_arm_reach", "R3_pick_and_place",
                   "R4_mobile_manipulation"):
    REGISTRY[(_null_task, "null")] = {
        "fn": null.run, "expect_pass": False, "expect_failures": None}

# Every task gets the LLM agents. Done as a loop rather than 3x3 hand-written
# entries so adding a task cannot silently forget the measured condition.
for _task_id in sorted({t for t, _ in list(REGISTRY)}):
    for _agent_name, _agent_fn in _LLM_AGENTS.items():
        REGISTRY[(_task_id, _agent_name)] = {
            "fn": _agent_fn, "expect_pass": None, "expect_failures": None}

AGENT_NAMES = sorted({a for _, a in REGISTRY})


def get(task_id, agent_name):
    try:
        return REGISTRY[(task_id, agent_name)]
    except KeyError:
        raise KeyError("no %r agent for task %r (have: %s)"
                       % (agent_name, task_id,
                          ", ".join(sorted(a for t, a in REGISTRY
                                           if t == task_id)))) from None


def agents_for(task_id):
    """Agents ``--agent all`` should run: the free, deterministic Phase-0 set.

    The LLM agents are registered (so ``--agent llm`` works) but excluded here.
    Two reasons, both about not surprising the operator: they cost money, and
    without a credential every one of them would add a guaranteed ``FAIL`` row
    to a suite whose entire point is that oracle/null expectations are green.
    Ask for them by name.
    """
    return sorted(a for t, a in REGISTRY
                  if t == task_id and a not in _LLM_AGENTS)
