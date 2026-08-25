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

"""Scripted fixtures for C1 ``parse_error_fix`` (Phase-0, no LLM).

The task's initial world carries exactly two defects: an unbalanced brace
(the ``DEF PALLET_A Solid`` block never closes) and an undefined node type
(``DEF PALLET_B Soild`` -- a typo of ``Solid`` with no declaration anywhere).
Both must be repaired, and the scene must stay whole.

This module is **REGISTRY-shaped but not registered**: per the C1 grader
lane's file-scope rule it does not touch ``agents/__init__.py``. To wire C1
into the orchestrator, merge :data:`REGISTRY` below into
``agentbench.agents.REGISTRY`` (one ``REGISTRY.update(c1_fixtures.REGISTRY)``
after the existing literal, plus the import).

The red map -- §5.5's red-evidence rule, satisfied at birth
----------------------------------------------------------

Every C1 assertion has a *targeted* negative fixture observed failing it
(:data:`RED_MAP`), not just the null. The per-fixture behaviour of the engine
was **measured**, not assumed (2026-07-31, `python -m omnisim run-headless
... --duration 5` on this tree's binary):

* the untouched world FAILs to load: ``error: Expected field name or '}',
  found 'DEF'`` + ``Failed to load due to syntax error(s)`` (2 error lines);
* the brace-fixed, typo-left world logs ``ERROR: Missing declaration for
  'Soild', unknown node.`` + ``Skipped unknown 'Soild' node or PROTO.`` --
  and then **loads the rest of the world and steps it to completion**. An
  undefined node type is a *skip*, not an abort, so the half-fix reds C1.1
  (error lines) and C1.2 (``pallet_b`` was skipped out of the inventory)
  while C1.3 stays green;
* the typo-fixed, brace-left world FAILs to load exactly like the untouched
  one (the syntax error aborts the parse);
* the fully fixed world runs clean: ``0 errors``.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench.agents import null  # noqa: E402
from agentbench.agents.base import AgentResult  # noqa: E402
from agentbench.common import worldtext  # noqa: E402

TASK_ID = "C1_parse_error_fix"

# --- the two defects, as text operations ------------------------------------

# The PALLET_A Solid block ends at its children ']' and never closes; the
# blank line + PALLET_B header make the anchor unique in the shipped world.
BRACE_ANCHOR = "  ]\n\nDEF PALLET_B"
BRACE_REPAIR = "  ]\n}\n\nDEF PALLET_B"

UNDEFINED_TYPE = "Soild"
DEFINED_TYPE = "Solid"

# From 'DEF PALLET_A Solid {' up to and including the first column-0 '}' --
# which, because PALLET_A never closes and every intervening brace is
# indented, is PALLET_B's closer: one match removes BOTH pallet subtrees.
_PALLET_SPAN = re.compile(r"(?ms)^DEF PALLET_A Solid \{.*?^\}\n")


def fix_brace(text):
    """Close the PALLET_A Solid block. Returns (text, applied)."""
    if BRACE_ANCHOR not in text:
        return text, False
    return text.replace(BRACE_ANCHOR, BRACE_REPAIR, 1), True


def fix_node_type(text):
    """Repair the undefined node type. Returns (text, applied)."""
    if UNDEFINED_TYPE not in text:
        return text, False
    return text.replace(UNDEFINED_TYPE, DEFINED_TYPE), True


def apply_full_fix(text):
    """Both repairs -- the oracle's edit. Returns (text, applied)."""
    t, a = fix_brace(text)
    t, b = fix_node_type(t)
    return t, a and b


def amputate(text):
    """Delete both pallet subtrees instead of repairing them.

    The lazy 'fix': the result parses (nothing broken is left) and steps, but
    ``pallet_a`` and ``pallet_b`` are gone from the inventory. Must FAIL C1.2
    and nothing else.
    """
    t, n = _PALLET_SPAN.subn("", text)
    return t, bool(n)


# A trivial, VALID world: loads clean, steps clean, and shares only the floor
# name with the task's scene. Pins ODE so the run stays attributable (without
# the pin this fixture would probe the A1.10 INVALID rule instead of C1.2).
WHOLESALE_WORLD = """#VRML_SIM R2025a utf8

WorldInfo {
  basicTimeStep 16
  title "a trivial replacement world"
  defaultPhysicsBackend "ode"
}
Viewpoint {
  orientation -0.35740674 0.35740674 0.86285621 1.7177716
  position 4.000 -4.000 3.000
}
DEF GROUND Solid {
  translation 0 0 0
  name "floor"
  children [
    Shape {
      geometry Box {
        size 10 10 0.1
      }
    }
  ]
  boundingObject Box {
    size 10 10 0.1
  }
}
DEF BOT Robot {
  translation 0 0 0.4
  name "bot"
  controller "<none>"
  children [
    Shape {
      geometry Box {
        size 0.3 0.3 0.3
      }
    }
  ]
  boundingObject Box {
    size 0.3 0.3 0.3
  }
  physics Physics {
    density -1
    mass 2
  }
}
"""


# --- the scripted agents ----------------------------------------------------


def _edit_world(ctx, editor, turn_msg, tool_edit, final_msg):
    """Shared shape of every editing fixture: find, edit, write, claim."""
    res = AgentResult()
    world = worldtext.pick_artifact(ctx.scratch_dir)
    if world is None:
        res.final_message = "no world found in the scratch directory"
        ctx.trace.final(res.final_message, False)
        return res, None
    res.artifacts["world"] = str(world)
    ctx.trace.turn(turn_msg)
    text = world.read_text(encoding="utf-8")
    fixed, applied = editor(text)
    ctx.trace.tool("edit_file", {"path": str(world), "edit": tool_edit,
                                 "applied": bool(applied)})
    if not applied:
        res.final_message = "could not locate the text to edit"
        ctx.trace.final(res.final_message, False)
        return res, None
    world.write_text(fixed, encoding="utf-8")
    ctx.trace.final(final_msg, False)
    res.final_message = final_msg
    res.turns = ctx.trace.turns
    res.tool_calls = ctx.trace.tool_calls
    return res, world


def run_c1_oracle(ctx):
    """Repair BOTH defects, then re-run and check before claiming success."""
    res = AgentResult()
    world = worldtext.pick_artifact(ctx.scratch_dir)
    if world is None:
        res.final_message = "no world found in the scratch directory"
        ctx.trace.final(res.final_message, False)
        return res
    res.artifacts["world"] = str(world)

    ctx.trace.turn("Two independent defects: the PALLET_A Solid block is "
                   "never closed (the parser dies at the next DEF), and "
                   "PALLET_B is typed 'Soild', which no node or declaration "
                   "defines. I will close the brace, correct the type, and "
                   "re-run to prove the load is clean.")
    text = world.read_text(encoding="utf-8")
    fixed, applied = apply_full_fix(text)
    ctx.trace.tool("edit_file", {"path": str(world),
                                 "edit": "close the PALLET_A block; rename "
                                         "the undefined 'Soild' type to "
                                         "'Solid'",
                                 "applied": bool(applied)})
    if not applied:
        res.final_message = "could not locate both defects to fix"
        ctx.trace.final(res.final_message, False)
        return res
    world.write_text(fixed, encoding="utf-8")

    if getattr(ctx, "fake_sim", False):
        res.final_message = ("applied both fixes; verification skipped "
                             "(--fake-sim)")
        ctx.trace.final(res.final_message, False)
        return res

    from agentbench.adapters.omnisim import headless
    ctx.trace.tool("run_headless", {"world": str(world), "duration_s": 5.0})
    r = headless.run_standalone(world, ctx.run_dir / "agent_self_check",
                                duration=5.0, settle=0.0, phase_a=False,
                                tag="selfcheck", timeout_s=300)
    if r.xyz is None or r.n_robots == 0:
        res.final_message = ("applied both fixes but the verification run "
                             "produced no samples: %s" % (r.error,))
        ctx.trace.final(res.final_message, False)
        return res
    res.self_verified = True
    msg = ("Fixed both defects: the PALLET_A Solid block was missing its "
           "closing brace, and PALLET_B was declared with the undefined "
           "type 'Soild' (a typo of Solid). Proof over a 5 s re-run: the "
           "world loads with no errors and steps to completion with %d "
           "robot(s) tracked." % r.n_robots)
    ctx.trace.final(msg, True)
    res.final_message = msg
    res.turns = ctx.trace.turns
    res.tool_calls = ctx.trace.tool_calls
    return res


def run_c1_amputate(ctx):
    """'Fix' the load by deleting everything the parser complained about."""
    res, _ = _edit_world(
        ctx, amputate,
        "The parser chokes on the pallet blocks, so I will remove them.",
        "delete the PALLET_A and PALLET_B subtrees",
        "Fixed: removed the malformed pallet blocks. The world now loads "
        "cleanly.")
    return res


def run_c1_wholesale(ctx):
    """Replace the world wholesale with a trivial valid file."""
    res = AgentResult()
    world = worldtext.pick_artifact(ctx.scratch_dir)
    if world is None:
        res.final_message = "no world found in the scratch directory"
        ctx.trace.final(res.final_message, False)
        return res
    res.artifacts["world"] = str(world)
    ctx.trace.turn("The file is badly broken; rewriting it from scratch is "
                   "faster than debugging it.")
    ctx.trace.tool("write_file", {"path": str(world),
                                  "edit": "replace the whole file with a "
                                          "minimal valid world"})
    world.write_text(WHOLESALE_WORLD, encoding="utf-8")
    msg = "Rewrote the world from scratch. It loads cleanly now."
    ctx.trace.final(msg, False)
    res.final_message = msg
    res.turns = ctx.trace.turns
    res.tool_calls = ctx.trace.tool_calls
    return res


def run_c1_half_fix(ctx):
    """Fix the brace, miss the undefined node type."""
    res, _ = _edit_world(
        ctx, fix_brace,
        "The syntax error says a brace is missing before PALLET_B; closing "
        "the PALLET_A block.",
        "insert the missing '}' closing the PALLET_A block",
        "Fixed the unbalanced brace. The syntax error is gone.")
    return res


def run_c1_half_fix_type(ctx):
    """Fix the undefined node type, miss the brace."""
    res, _ = _edit_world(
        ctx, fix_node_type,
        "'Soild' is not a node type; correcting the typo to 'Solid'.",
        "rename the undefined 'Soild' type to 'Solid'",
        "Fixed the undefined node type. PALLET_B is a Solid now.")
    return res


# --- the fixture -> assertion red map ---------------------------------------
#
# MEASURED, per the module docstring: each entry is the exact set of C1
# assertions the fixture turns red, observed via the engine runs above and
# reproduced by the synthetic bundles in graders/test_c1_core.py. Per §5.5, a
# fixture that reds everything (the null) validates no individual assertion;
# the targeted evidence is:
#
#   C1.1  <- half_fix       (loads-with-error-lines: the skip case)
#         <- half_fix_type  (hard parse abort)
#   C1.2  <- amputate       (reds C1.2 ALONE -- the sharpest fixture here)
#         <- wholesale      (reds C1.2 alone from the other direction)
#         <- half_fix       (the skipped node leaves the inventory short)
#   C1.3  <- half_fix_type / null (no samples at all; C1.3 additionally has
#            in-core negatives for NaN, runaway and short-window trajectories
#            in test_c1_core.py, since no text edit of THIS world can produce
#            a loaded-but-non-finite run against a healthy floor)

RED_MAP = {
    "null": {"C1.1", "C1.2", "C1.3"},
    "amputate": {"C1.2"},
    "wholesale": {"C1.2"},
    "half_fix": {"C1.1", "C1.2"},
    "half_fix_type": {"C1.1", "C1.2", "C1.3"},
}

# REGISTRY-shaped, matching agents/__init__.REGISTRY entry-for-entry. The
# null keeps expect_failures=None by the package's own convention (a fixture
# that reds everything is evidence for nothing in particular).
REGISTRY = {
    (TASK_ID, "oracle"): {
        "fn": run_c1_oracle, "expect_pass": True, "expect_failures": None},
    (TASK_ID, "null"): {
        "fn": null.run, "expect_pass": False, "expect_failures": None},
    (TASK_ID, "amputate"): {
        "fn": run_c1_amputate, "expect_pass": False,
        "expect_failures": RED_MAP["amputate"]},
    (TASK_ID, "wholesale"): {
        "fn": run_c1_wholesale, "expect_pass": False,
        "expect_failures": RED_MAP["wholesale"]},
    (TASK_ID, "half_fix"): {
        "fn": run_c1_half_fix, "expect_pass": False,
        "expect_failures": RED_MAP["half_fix"]},
    (TASK_ID, "half_fix_type"): {
        "fn": run_c1_half_fix_type, "expect_pass": False,
        "expect_failures": RED_MAP["half_fix_type"]},
}
