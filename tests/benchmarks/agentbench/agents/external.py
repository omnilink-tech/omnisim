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

"""Grade an artifact an OUTSIDE agent produced.

Phase 1 needs to score work done by something that is not a Python class in
this package -- an LLM loop, a human, a competitor simulator's agent. Those
cannot implement ``agents/base.py`` and must never read this package anyway,
because it contains the oracle solution and the grader thresholds.

So the contract is a file on disk. Point ``AGENTBENCH_EXTERNAL_ARTIFACT`` at
the world the outside agent authored; this adapter copies it into the run's
scratch dir under the name the task expects, then hands off to the SAME
phase-B recording and the SAME grader every other agent gets. No credit is
given for the copy: if the artifact is missing the run fails like any other
agent that produced nothing.

    AGENTBENCH_EXTERNAL_ARTIFACT=/path/to/world.omniworld \
      python run_agentbench.py --tasks A1 --agent external

Optional provenance, recorded into the row so a result can be traced back to
the condition that produced it (SPEC 3: every number carries its conditions):
    AGENTBENCH_EXTERNAL_LABEL      e.g. "shell_only" | "full_surface". This
                                   becomes the row's ``condition`` column, so
                                   an A/B is queryable straight out of
                                   rows.jsonl.
    AGENTBENCH_EXTERNAL_NOTES      free text: model, tools allowed, turns
    AGENTBENCH_EXTERNAL_ANSWER     optional, world-deliverable tasks only:
                                   path to a text file holding the outside
                                   agent's final message, for graders that
                                   read the answer channel too (B2). For the
                                   answer-deliverable tasks (ANSWER_TASKS)
                                   the ARTIFACT itself is the answer file.
    AGENTBENCH_SIM                 which simulator's deliverable convention
                                   applies (default ``omnisim``). The same
                                   variable ``adapters.resolve`` reads, so the
                                   file is staged under the name the adapter
                                   that grades it looks for -- a ``.wbt`` world
                                   on the OmniSim/Webots arms, an MJCF ``.xml``
                                   plus its driver on MuJoCo.
"""

from __future__ import annotations

import os
import shutil
from pathlib import Path

from agentbench.agents.base import AgentResult
from agentbench.common import worldtext

# The filename each task's grader looks for in the scratch dir.
ARTIFACT_NAME = {
    "A1_husky_swarm_10": "husky_swarm_10.wbt",
    # Phase W Claude Code lane (cc_lane/): the artifact is the task's own
    # world, fixed by the outside agent -- same name the initial/ dir ships,
    # so the copy lands where pick_artifact and the grader expect it.
    "C2_fall_through_floor": "fall_through.wbt",
    # B2: the deliverable is the final camera pose, "however the agent set
    # it" -- for an external run that is the MODIFIED WORLD (same convention
    # as C2; the copy overwrites the staged initial, and the B2 view-evidence
    # adapter reads initial pose from the task's shipped world, final pose
    # from this artifact). The committed-proof half of B2 rides in the
    # answer channel below.
    "B2_subject_in_frame": "frame_the_cylinder.wbt",
    # C1: the deliverable is the repaired world.
    "C1_parse_error_fix": "parse_error.wbt",
    # R1 (robotics tier): an AUTHORING task -- the agent builds the world, the
    # robot and the controller from an empty project plus the frozen obstacle
    # spec, so the deliverable is whatever world it wrote.
    "R1_lidar_nav": "lidar_nav.wbt",
    # R2/R3 (robotics tier): both are authoring tasks -- the agent builds or
    # imports the arm, the world and the controller, so the deliverable is the
    # world it wrote.
    "R2_arm_reach": "arm_reach.wbt",
    "R3_pick_and_place": "pick_and_place.wbt",
    # R4 (robotics tier): an authoring task like R1 -- the agent builds the
    # arena, the mobile manipulator, the gripper and the controller from an
    # empty project plus the frozen obstacle + scene specs.
    "R4_mobile_manipulation": "mobile_manipulation.wbt",
}

#: Per-SIM overrides of the deliverable filename.
#:
#: A deliverable's name is a property of the SIMULATOR as much as of the task.
#: OmniSim and upstream Webots both take a ``.wbt`` world -- same file format,
#: which is what makes them comparable -- so ``ARTIFACT_NAME`` above is their
#: registry and this map only carries arms whose format differs.
#:
#: MuJoCo's deliverable is a **PAIR**: the MJCF ``.xml`` (the graded artifact)
#: and the Python program that steps it, collected beside it as the same STEM
#: with a ``.py`` suffix. MJCF is inert data -- no ``controller`` field, no
#: engine that would start a process, so the model alone cannot move -- and
#: collecting only the ``.xml`` would reproduce exactly the bug that zeroed
#: OmniSim's R1 (a world collected without the thing that drives it; see
#: ``cc_lane/run_cc_cell.collect_controllers``). ``adapters/mujoco/BRINGUP.md``
#: §6 is the arm's own statement of this convention, and
#: :func:`agentbench.adapters.mujoco.launcher.find_driver` is the SINGLE
#: authority on finding the driver -- it prefers the matching stem, accepts the
#: only ``.py`` beside the model, and REFUSES to guess between several, because
#: a wrong pick reads as an agent whose robot did nothing.
#:
#: Only the tasks ``sims.SIMS["mujoco"].tasks`` says that arm can express are
#: listed. A task missing from an arm's map has **no convention on that arm**
#: and resolves to ``None`` -- it must never silently inherit the ``.wbt``
#: name, which would stage a file that arm cannot even load.
ARTIFACT_NAME_BY_SIM = {
    "mujoco": {
        "A1_husky_swarm_10": "husky_swarm_10.xml",
        "R1_lidar_nav": "lidar_nav.xml",
        "R2_arm_reach": "arm_reach.xml",
    },
}

#: The simulator assumed when a caller names none (the system under test).
DEFAULT_SIM = "omnisim"


def artifact_name(task_id, sim=None):
    """The filename this task's deliverable must be collected under on ``sim``.

    ``None`` means "this task has no world-deliverable convention here" -- an
    ANSWER task, a task nobody registered, or a task that arm cannot express.
    Callers must treat that as a refusal rather than inventing a name: a
    deliverable staged under a name the grader does not look for is graded as
    if the agent had produced nothing.
    """
    by_sim = ARTIFACT_NAME_BY_SIM.get(sim)
    if by_sim is not None:
        return by_sim.get(task_id)
    return ARTIFACT_NAME.get(task_id)


def artifact_suffixes(sim=None):
    """The file suffixes a deliverable may carry on ``sim``, lowercased.

    Derived from the registry above rather than declared a second time, so a
    new arm cannot end up with a filename convention and a discovery rule that
    disagree.
    """
    names = ARTIFACT_NAME_BY_SIM.get(sim)
    if names is None:
        names = ARTIFACT_NAME
    out = tuple(sorted({Path(n).suffix.lower() for n in names.values()}))
    return out or (".wbt",)


def find_driver(model):
    """``(driver_path_or_None, rule)`` for a model that cannot move alone.

    Delegates to the MuJoCo adapter, which owns the rule. This wrapper exists
    only so a caller does not have to import that adapter (and so a tree
    without it degrades to a recorded reason instead of an ImportError).
    """
    try:
        from agentbench.adapters.mujoco import launcher as mj_launcher
    except ImportError as exc:                                # pragma: no cover
        return None, ("the MuJoCo launcher could not be imported (%r), so no "
                      "driver was collected; the model alone cannot move"
                      % (exc,))
    return mj_launcher.find_driver(model)

# Tasks whose deliverable is the agent's ANSWER, not a world (their meta.json
# says "the agent's final message (there is no artifact)"). For these,
# AGENTBENCH_EXTERNAL_ARTIFACT points at a text file (answer.txt by
# convention) holding the outside agent's final message; this adapter reads
# it into ``final_message`` -- the exact channel run_agentbench passes to the
# grader as ``answer=`` -- and stages NO world, so the grader measures ground
# truth from the task's own staged initial world, unmodified.
ANSWER_TASKS = frozenset({"B1_overlap_audit", "B3_measure_and_report"})

# Optional second channel for world-deliverable tasks whose grader ALSO reads
# the final message (B2's committed proof): a text file whose content becomes
# ``final_message``. Ignored for ANSWER_TASKS (their artifact IS the answer).
ANSWER_ENV = "AGENTBENCH_EXTERNAL_ANSWER"


def run(ctx):
    res = AgentResult()
    label = os.environ.get("AGENTBENCH_EXTERNAL_LABEL", "external")
    notes = os.environ.get("AGENTBENCH_EXTERNAL_NOTES", "")
    src = os.environ.get("AGENTBENCH_EXTERNAL_ARTIFACT", "")

    res.artifacts["external_label"] = label
    # The label IS the condition for an external run. Without this the row
    # falls back to run_agentbench.CONDITION ("scripted") and a shell_only vs
    # full_surface A/B produces rows that are byte-identical in their condition
    # columns -- unqueryable, which is the defect commit 2a234a7f set out to
    # fix and only fixed for the in-package LLM agent (agents/llm.py sets
    # artifacts["condition"]; this adapter did not).
    res.artifacts["condition"] = label
    if notes:
        res.artifacts["external_notes"] = notes

    ctx.trace.turn("External artifact: %s (label=%s)" % (src or "<unset>", label))

    if not src:
        msg = ("AGENTBENCH_EXTERNAL_ARTIFACT is unset -- nothing to grade. "
               "This is a FAILURE, not a skip: an agent that produced no "
               "artifact produced no artifact.")
        ctx.trace.final(msg, False)
        res.final_message = msg
        return res

    srcp = Path(src)
    if not srcp.is_file():
        msg = "external artifact does not exist: %s" % srcp
        ctx.trace.final(msg, False)
        res.final_message = msg
        return res

    if ctx.task_id in ANSWER_TASKS:
        # The artifact IS the answer: its text becomes final_message, which
        # run_agentbench hands to the grader as ``answer=``. No world is
        # staged -- the grader's recorder measures the task's own initial
        # world exactly as materialised, and the answer is checked against
        # that measurement.
        answer = srcp.read_text(encoding="utf-8", errors="replace").strip()
        ctx.trace.tool("read_external_answer",
                       {"from": str(srcp), "chars": len(answer),
                        "label": label})
        res.artifacts["external_source"] = str(srcp)
        res.artifacts["external_bytes"] = srcp.stat().st_size
        res.artifacts["answer_file"] = str(srcp)
        if not answer:
            answer = ("(the external answer file %s is empty -- the outside "
                      "agent committed to nothing)" % srcp.name)
        # Deliberately NOT self-verified (same rule as the world path below).
        ctx.trace.final(answer, False)
        res.final_message = answer
        res.turns = ctx.trace.turns
        res.tool_calls = ctx.trace.tool_calls
        return res

    # Which simulator's convention applies. ``$AGENTBENCH_SIM`` is the same
    # variable ``adapters.resolve`` reads, so the deliverable is staged under
    # the name the adapter that will grade it looks for.
    sim = (os.environ.get("AGENTBENCH_SIM") or getattr(ctx, "sim", None)
           or DEFAULT_SIM)
    res.artifacts["external_sim"] = sim
    name = artifact_name(ctx.task_id, sim)
    if name is None:
        msg = ("no external artifact convention for task %s on sim %s -- add "
               "it to ARTIFACT_NAME (or ARTIFACT_NAME_BY_SIM[%r]) before "
               "scoring this task externally" % (ctx.task_id, sim, sim))
        ctx.trace.final(msg, False)
        res.final_message = msg
        return res

    dest = ctx.scratch_dir / name
    ctx.trace.tool("copy_external_artifact",
                   {"from": str(srcp), "to": str(dest),
                    "bytes": srcp.stat().st_size, "label": label})
    # Relocating a world breaks every RELATIVE asset url in it, because the
    # engine resolves URDFRobot urls against the world file. Re-base them
    # against the directory the agent authored in, so the copy means what the
    # original meant. Only urls that actually resolve there are rewritten, so
    # a reference that was already broken stays broken and still fails.
    rebased = []
    if srcp.suffix.lower() in (".wbt", ".omniworld"):
        try:
            text = srcp.read_text(encoding="utf-8", errors="replace")
            new_text, rebased = worldtext.rebase_relative_urls(
                text, srcp.parent)
        except OSError:
            new_text, rebased = None, []
        if rebased:
            dest.write_text(new_text, encoding="utf-8")
            ctx.trace.tool("rebase_relative_urls",
                           {"world": str(dest), "rewritten": rebased,
                            "why": "the grader relocates the deliverable; "
                                   "relative asset urls resolve against the "
                                   "world file and would break"})
        else:
            shutil.copyfile(srcp, dest)
    else:
        shutil.copyfile(srcp, dest)
    res.artifacts["world"] = str(dest)
    res.artifacts["external_source"] = str(srcp)
    res.artifacts["external_bytes"] = srcp.stat().st_size

    # An MJCF model is INERT: it declares no controller and starts no process,
    # so staging it alone stages a scene that cannot move -- which grades as an
    # agent whose robot did nothing. The driver travels with it, under the
    # model's own stem so the adapter's first discovery rule finds it again.
    if dest.suffix.lower() == ".xml":
        drv, drv_rule = find_driver(srcp)
        res.artifacts["driver_rule"] = drv_rule
        if drv is None:
            # NOT guessed. find_driver refuses to choose between candidates,
            # and a wrong pick would be the grader deciding which program the
            # agent meant. The reason is recorded and the run proceeds; the
            # adapter reports the missing driver as what it is.
            res.artifacts["driver"] = None
            ctx.trace.tool("collect_external_driver",
                           {"model": str(srcp), "driver": None,
                            "rule": drv_rule})
        else:
            ddest = dest.with_suffix(".py")
            shutil.copyfile(drv, ddest)
            res.artifacts["driver"] = str(ddest)
            ctx.trace.tool("collect_external_driver",
                           {"model": str(srcp), "from": str(drv),
                            "to": str(ddest), "rule": drv_rule,
                            "why": "MJCF is inert data; the deliverable is "
                                   "the model AND the program that steps it"})

    # Deliberately NOT self-verified: the outside agent may or may not have
    # checked its own work, and this adapter has no way to know. Leaving it
    # False keeps `self_verified` honest rather than flattering.
    msg = ("Graded an externally produced artifact (%s, %d bytes, label=%s). "
           "The grader measures it independently. %s"
           % (srcp.name, srcp.stat().st_size, label, notes))

    # A grader that ALSO reads the final message (B2's committed proof) gets
    # the outside agent's own words via the optional answer channel; without
    # it the fallback description above stands (and commits to nothing,
    # which is honest for an answer nobody captured).
    ans_src = os.environ.get(ANSWER_ENV, "")
    if ans_src:
        ansp = Path(ans_src)
        if ansp.is_file():
            text = ansp.read_text(encoding="utf-8", errors="replace").strip()
            ctx.trace.tool("read_external_answer",
                           {"from": str(ansp), "chars": len(text),
                            "label": label})
            res.artifacts["answer_file"] = str(ansp)
            if text:
                msg = text
        else:
            res.artifacts["answer_file_missing"] = str(ansp)

    ctx.trace.final(msg, False)
    res.final_message = msg
    res.turns = ctx.trace.turns
    res.tool_calls = ctx.trace.tool_calls
    return res
