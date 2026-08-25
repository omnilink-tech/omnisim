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

"""B1 ``overlap_audit`` -- the scripted NEGATIVE fixtures (red evidence).

Plan 5.5, the red-evidence rule: *no assertion may enter a scored campaign
until it has been observed FAILING on a deliberately wrong artifact, with
that negative fixture named in the assertion's record.* B1's grader is born
with these six, each aimed at specific assertions:

=============  ======================================  ====================
fixture        what it does wrong                      assertions driven red
=============  ======================================  ====================
``wrong_pair``  says yes, names the FARTHEST-apart      B1.4
                pair -- confident, well-formed, wrong
``shotgun``     says yes, then lists EVERY pair (the    B1.3, B1.4
                B3 shotgun replayed against pair
                naming, commit 1fb331a7)
``yes_only``    "yes" with no pair named -- half the    B1.3, B1.4
                deliverable withheld
``phantom``     EDITS the world so nothing overlaps,    B1.2, B1.4
                then claims a pair does (the artifact
                variant the task brief requires)
``missed``      answers "no" on the shipped world,      B1.2, B1.4
                where one pair measurably overlaps
``waffle``      commits to no verdict at all --         B1.1, B1.2, B1.4
                targets B1.1, whose only other red
                would be the null agent (which
                validates nothing)
=============  ======================================  ====================

``EXPECTED_FAILURES`` is that table as a structured constant, and
``graders/test_b1_core.py`` runs every fixture ANSWER through the core and
asserts the measured red set equals it -- so the table above is checked, not
believed. The answer builders are module-level functions shared by the agents
and the tests: the string the test grades is byte-identical to the string the
scripted agent will emit in a Phase-0 run.

**Registration.** This module does NOT edit ``agents/__init__.py`` (owned by
a parallel lane). It exports ``REGISTRY`` in the same shape; the orchestrator
merges it with one line::

    from agentbench.agents import b1_fixtures
    REGISTRY.update(b1_fixtures.REGISTRY)

placed BEFORE the ``_LLM_AGENTS`` fan-out loop so B1 also inherits the LLM
agents. Note there is no ``oracle`` here yet: a B1 oracle (measure the scene,
answer honestly) is Phase-0 work the orchestrator still owns, and without it
the oracle-PASSes-every-task gate (SPEC 7) does not cover B1.
"""

from __future__ import annotations

import re
import sys
from itertools import combinations
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench.agents import null  # noqa: E402
from agentbench.agents.base import AgentResult  # noqa: E402
from agentbench.common import worldtext  # noqa: E402

TASK_ID = "B1_overlap_audit"

# fixture name -> the assertions it must drive red (checked by
# graders/test_b1_core.py, consumed by the plan-5.5 coverage table).
EXPECTED_FAILURES = {
    "wrong_pair": {"B1.4"},
    "shotgun": {"B1.3", "B1.4"},
    "yes_only": {"B1.3", "B1.4"},
    "phantom": {"B1.2", "B1.4"},
    "missed": {"B1.2", "B1.4"},
    "waffle": {"B1.1", "B1.2", "B1.4"},
}


# --- answer builders (shared with the tests -- single source of truth) ------


def wrong_pair_answer(a, b):
    """Confident, well-formed, and aimed at a pair that is metres clear."""
    return ("Yes. %s and %s are overlapping; every other robot in the scene "
            "is clear." % (a, b))


def shotgun_answer(names):
    """Blankets every pair instead of committing to one.

    The B3 lesson replayed against pair naming: a grader that scores the
    best of many offered answers hands a pass to an agent that measured
    nothing. This answer must yield NO committed pair.
    """
    pairs = ", ".join("%s and %s overlap" % (a, b)
                      for a, b in combinations(sorted(names), 2))
    return ("Yes, at least one pair is overlapping. It is one of these: %s."
            % pairs)


def yes_only_answer():
    return "Yes, two of the robots in this scene are overlapping each other."


def phantom_answer(a, b):
    """The claim the phantom fixture makes AFTER removing the overlap."""
    return "Yes. %s and %s are overlapping." % (a, b)


def missed_answer():
    return "No. There is nothing touching in this scene; every robot sits clear."


def waffle_answer():
    """No verdict at all. Avoids every yes/no/overlap cue on purpose."""
    return ("I could not complete the audit; the scene query timed out and I "
            "have nothing reliable to report about the robots' spacing.")


# --- reading the scratch world (fixture-side helpers, not grader code) ------

_ROBOT_BLOCK = re.compile(r"URDFRobot\s*\{[^{}]*\}", re.S)
_NAME = re.compile(r'name\s+"([^"]+)"')
_TRANSLATION = re.compile(
    r"translation\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)")


def _robots_in(text):
    """[(name, (x, y, z), block_span)] for every URDFRobot block."""
    out = []
    for m in _ROBOT_BLOCK.finditer(text):
        block = m.group(0)
        name = _NAME.search(block)
        tr = _TRANSLATION.search(block)
        if name and tr:
            out.append((name.group(1),
                        tuple(float(g) for g in tr.groups()), m.span()))
    return out


def _extreme_pair(robots, farthest=True):
    """The (nameA, nameB) whose spawn points are farthest (or closest) apart."""
    best = None
    for (na, pa, _sa), (nb, pb, _sb) in combinations(robots, 2):
        d2 = sum((x - y) ** 2 for x, y in zip(pa, pb))
        if best is None or (d2 > best[0]) == farthest:
            best = (d2, na, nb)
    return (best[1], best[2]) if best else (None, None)


def _pick_world(ctx, res):
    world = worldtext.pick_artifact(ctx.scratch_dir)
    if world is None:
        res.final_message = "no world found in the scratch directory"
        ctx.trace.final(res.final_message, False)
        return None
    res.artifacts["world"] = str(world)
    return world


def _finish(ctx, res, msg, claims_verified=False):
    ctx.trace.final(msg, claims_verified)
    res.final_message = msg
    res.turns = ctx.trace.turns
    res.tool_calls = ctx.trace.tool_calls
    return res


# --- the fixture agents ------------------------------------------------------


def run_wrong_pair(ctx):
    """Names the pair with the LARGEST spawn separation -- necessarily not
    the overlapping one on any world this task ships."""
    res = AgentResult()
    world = _pick_world(ctx, res)
    if world is None:
        return res
    robots = _robots_in(world.read_text(encoding="utf-8"))
    a, b = _extreme_pair(robots, farthest=True)
    ctx.trace.turn("Two of these look close together to me.")
    return _finish(ctx, res, wrong_pair_answer(a or "robot_a", b or "robot_b"))


def run_shotgun(ctx):
    """Lists every pair. Measured nothing; commits to nothing."""
    res = AgentResult()
    world = _pick_world(ctx, res)
    if world is None:
        return res
    robots = _robots_in(world.read_text(encoding="utf-8"))
    names = [n for n, _p, _s in robots]
    ctx.trace.turn("Rather than measure, I will enumerate the possibilities.")
    return _finish(ctx, res, shotgun_answer(names))


def run_yes_only(ctx):
    """Right verdict, no pair -- half the deliverable withheld."""
    res = AgentResult()
    ctx.trace.turn("It looks crowded in there.")
    return _finish(ctx, res, yes_only_answer())


def run_phantom(ctx):
    """Separates the overlapping pair in the artifact, then claims overlap.

    This is the task brief's fixture (d): the artifact world has NO overlap
    but the answer claims one. It edits the scratch copy only -- the closest
    pair's second robot is moved to (12.5, -4), metres clear of everything --
    and then reports the pair it just separated as overlapping. The grader
    re-derives ground truth from the edited world's measured geometry, so the
    claim must fail on B1.2 (verdict) and B1.4 (pair), never on parsing.
    """
    res = AgentResult()
    world = _pick_world(ctx, res)
    if world is None:
        return res
    text = world.read_text(encoding="utf-8")
    robots = _robots_in(text)
    a, b = _extreme_pair(robots, farthest=False)      # the closest pair
    moved = False
    for name, _pos, (s, e) in robots:
        if name == b:
            block = text[s:e]
            new_block, n = _TRANSLATION.subn("translation 12.5 -4 0.2",
                                             block, count=1)
            if n:
                text = text[:s] + new_block + text[e:]
                moved = True
            break
    ctx.trace.turn("I will tidy the robots apart first, then report.")
    ctx.trace.tool("edit_file", {"path": str(world),
                                 "edit": "move %r to (12.5, -4)" % b,
                                 "applied": moved})
    world.write_text(text, encoding="utf-8")
    return _finish(ctx, res,
                   phantom_answer(a or "robot_a", b or "robot_b"),
                   claims_verified=True)


def run_missed(ctx):
    """Answers "no" on the shipped world, where one pair measurably overlaps."""
    res = AgentResult()
    ctx.trace.turn("A quick glance says they are all spread out.")
    return _finish(ctx, res, missed_answer())


def run_waffle(ctx):
    """Commits to no verdict at all -- the targeted red for B1.1."""
    res = AgentResult()
    ctx.trace.turn("I was unable to inspect the scene.")
    return _finish(ctx, res, waffle_answer())


_FIXTURE_FNS = {
    "wrong_pair": run_wrong_pair,
    "shotgun": run_shotgun,
    "yes_only": run_yes_only,
    "phantom": run_phantom,
    "missed": run_missed,
    "waffle": run_waffle,
}

# REGISTRY-shaped, for the orchestrator to merge into agents/__init__.py's
# REGISTRY (see the module docstring). Includes the generic null control --
# required so "no task may be passable by doing nothing" covers B1 -- whose
# expect_failures is None by the standing convention (a null red validates
# nothing, plan 5.5).
REGISTRY = {
    (TASK_ID, "null"): {
        "fn": null.run, "expect_pass": False, "expect_failures": None},
}
for _name, _fn in _FIXTURE_FNS.items():
    REGISTRY[(TASK_ID, _name)] = {
        "fn": _fn, "expect_pass": False,
        "expect_failures": set(EXPECTED_FAILURES[_name])}
