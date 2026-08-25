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

"""One graded Claude Code cell, end to end (Phase W ``claude_code`` lane).

    python tests/benchmarks/agentbench/cc_lane/run_cc_cell.py \\
        --sim omnisim --task C2_fall_through_floor

Per cell, strictly SEQUENTIALLY (no two engines ever run concurrently; the
grading launches start only after the Claude Code child has exited):

1. **Preflight** -- record ``claude --version``; verify auth with a
   MULTI-LINE ``claude -p <probe> --output-format json`` in a neutral dir and
   read the default model id from the result's ``modelUsage``; pin that id
   (or ``--model``) for the task run. The probe is multi-line on purpose:
   see :data:`PREFLIGHT_PROMPT`.
2. **Workspace** -- instantiate a FRESH per-cell copy of the staged product
   template (``stage_workspaces.py``; cells never share state), stage the
   task's initial world into it, and pass the task's ``prompt.txt`` text --
   byte-identical on both arms -- as the ``-p`` argument. The workspace is
   MIRRORED into ``<cell>/workspace/`` while the session runs and copied
   authoritatively when it exits: it is the artifact of record and it is
   preserved on every outcome, including blocked and crashed.
3. **Run** -- ``claude -p <prompt> --output-format stream-json --verbose
   --model <pinned> --dangerously-skip-permissions`` with ``cwd`` = the
   workspace instance and a scrubbed child env (no nesting vars, no
   ``ANTHROPIC_API_KEY``, no ``AGENTBENCH_*``), through the REAL
   ``claude.exe`` and never npm's ``.cmd`` shim (see
   :func:`_resolve_cli_launcher`). NDJSON events land in
   ``<cell>/cc_stream.jsonl`` as they happen, so a running cell is
   inspectable with ``--status``. If the permissions flag is refused before
   Claude starts, fall back to ``--permission-mode acceptEdits`` + a broad
   ``--allowedTools`` list and record which mode ran.
4. **Metrics** -- cost/turns/duration/tokens from the stream's ``result``
   event; ``tool_calls`` counted from the session transcript
   (``~/.claude/projects/**/<session_id>.jsonl``, assistant ``tool_use``
   blocks) and from the stream when the transcript is unavailable. Unmeasured
   is ``null`` with the reason recorded -- never a number that was not
   measured. ``metrics_source: "claude_code_headless_json"``.
4a. **Liveness** -- did a session actually run, at the protocol's budget
   (:func:`assess_liveness`)? The session's stdout is METADATA; the workspace
   is the verdict. A cell whose agent left a deliverable is graded even if the
   session emitted prose, crashed or returned nothing; a cell whose session
   never started, or which our own scheduler starved of budget, BLOCKS rather
   than publishing a row that would read as an agent failure.
4b. **Place** (R1 only) -- draw the graded obstacle layout from the cell's
   seed, move the obstacles inside the COLLECTED deliverable to it, and
   declare it in an ``r1_graded_layout.json`` sidecar
   (``common/r1_placement.py``). This is R1's anti-hardcode mechanism and it
   runs HERE, after the session has ended and its workspace has been swept and
   copied out of, precisely so no session can see the layout it will be graded
   on. A cell whose placement fails is BLOCKED, never graded: grading it would
   score the layout the task publishes, which a memorising controller passes
   6/6 (measured).
5. **Grade** -- through the REAL pipelines only: the OmniSim arm goes through
   ``run_agentbench.py --agent external`` (``AGENTBENCH_EXTERNAL_ARTIFACT``
   env contract, label ``claude_code`` -> the row's condition); the Webots arm
   through the Phase W launcher + AABB prober + the sim-neutral grader core,
   exactly as ``preregister/run_oracles.py`` grades its webots cells.
6. **Row** -- the grader row with the Claude Code metrics merged in, appended
   to ``rows.jsonl`` in the cell's run dir, plus a human-readable
   ``cell_report.md``.

Per-task deliverable conventions (agents/external.py is the authority):

* ``C2`` / ``C1`` / ``B2`` / ``A1`` -- the deliverable is a WORLD;
  ``discover_artifact`` applies (the staged task world when the session
  modified it, outranking newer verification scratch). B2 additionally
  captures the session's final message into ``answer.txt`` (the committed
  proof the B2 grader reads) and passes it via ``AGENTBENCH_EXTERNAL_ANSWER``.
A world-deliverable is collected as a PROJECT, not a file: its relative asset
urls are re-based against the directory it was authored in, and every
``controller "<name>"`` it declares is copied into ``<cell>/artifact/
controllers/<name>/`` (evidence) and ``<cell>/grade/controllers/<name>/`` (what
the engine reads). Both grading paths launch their world from under
``<cell>/grade/worlds/``, so the engine's own project-root walk resolves
``<cell>/grade`` and finds them. Measured need: R1/omnisim shipped only the
``.wbt``, the robot got no controller and never moved (path length 0.0 m).

A deliverable's FILE FORMAT is the arm's property, not the task's, and the
registry is per-sim (``agents/external.ARTIFACT_NAME_BY_SIM`` /
``artifact_name(task_id, sim)``). On OmniSim and upstream Webots it is a
``.wbt`` world; on **MuJoCo it is a PAIR** -- an MJCF ``.xml`` (the graded
artifact) and the Python program that steps it, collected beside it under the
same stem. MJCF declares no controller and starts no process, so a model
collected alone is a scene that cannot move: the same defect as a ``.wbt``
collected without its ``controllers/`` directory, which is why
``collect_driver`` sits beside ``collect_controllers`` here.
``adapters.mujoco.launcher.find_driver`` is the ONE rule for finding it -- the
matching stem first, then the only ``.py`` beside the model, and a recorded
refusal rather than a guess when several are plausible.

* ``B1`` / ``B3`` -- the deliverable is the agent's ANSWER; the session's
  final message text (the headless JSON ``result`` field) is collected into
  ``answer.txt``, which IS the artifact. The grader measures ground truth
  from the task's own PRISTINE staged world -- never a world the session may
  have touched, so an agent cannot move the scene to match a wrong answer.

Concurrency protocol (plan §2.7; mechanics in ``concurrency.py``): a
per-task exclusive lock is held for the whole cell (same-task cells never
overlap -- plan §5.3's shared-global evidence risk); a global N-slot engine
semaphore (default 2, ``--engine-slots``) is held around engine-heavy phases
-- the WHOLE session for omnisim cells (the agent may launch the engine at
any point) and the grading/recorder pass always. Rows produced while another
lane was active are flagged ``measured_under_concurrency: true``. A
``claude -p`` refusal that is a usage/rate limit is recorded as a DEFERRED
attempt (not a failed run, no quota burned), waits ``--rate-limit-backoff-s``
(default 15 min) and retries the same cell with a fresh workspace.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
AGENTBENCH = HERE.parent
BENCHMARKS = AGENTBENCH.parent
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench.cc_lane import concurrency                  # noqa: E402
from agentbench.cc_lane import containment_guard            # noqa: E402
from agentbench.cc_lane import evidence                     # noqa: E402
from agentbench.cc_lane import stage_workspaces as staging  # noqa: E402
from agentbench.agents import external as external_agent    # noqa: E402
from agentbench.common.paths import REPO, ob_results        # noqa: E402
from agentbench import tasks as tasks_mod                   # noqa: E402
from agentbench import sims                                 # noqa: E402
from agentbench.common import r1_placement                  # noqa: E402
from agentbench.common import worldtext                     # noqa: E402

CONDITION = "claude_code"
METRICS_SOURCE = "claude_code_headless_json"
SUITE = "agenticsimbench/v0.3"

#: The benchmark model, pinned (SPEC 4.4). Every cell of a comparison runs
#: this exact id; a campaign that mixes models is not a comparison. Changing
#: it forces a suite version bump -- the Fable-scored ``agentbench/v0`` grid
#: is preserved under its own suite id and is never merged with these rows.
DEFAULT_MODEL = "claude-opus-5"

#: Fallback only. The real budget is PER TASK (``Task.timeout_s``, itself
#: clamped to ``tasks.TASK_HARD_CEILING_S``); ``run_cell`` gates the child on
#: ``min(task, this)`` so a CLI value can tighten a task but never loosen it.
DEFAULT_TIMEOUT_S = float(tasks_mod.TASK_HARD_CEILING_S)
DEFAULT_RATE_LIMIT_BACKOFF_S = 15 * 60.0
DEFAULT_MAX_RATE_LIMIT_RETRIES = 12

#: A cell may not occupy the machine for longer than its task budget PLUS this
#: fixed allowance, however many deferrals it takes.
#:
#: WHY A CEILING EXISTS AT ALL. The per-session cap bounded ONE session. The
#: deferral loop then handed each retry a fresh full-length session plus a
#: backoff, so a cell's real worst case was ``retries x (budget + backoff)`` --
#: 12 x (30 + 15) min, about NINE HOURS for one cell, while tasks/__init__
#: claimed "cells x ceiling is the worst case the campaign can bill". That
#: claim was simply false. Measured 2026-08-09: a webots R1 cell ran 58
#: minutes across at least two sessions before the owner noticed.
#:
#: WHY IT IS ADDITIVE AND NO LONGER A MULTIPLIER (2026-08-10). It was
#: ``budget x 2.0``, sized when repeats were the protocol and a doubled budget
#: was the room a rate-limited cell needed to get a second full-length try.
#: A multiplier scales the allowance with the AGENT's budget, and the things
#: it is paying for do not: staging a workspace, an auth preflight, the port
#: and process sweeps, R1's grade-time placement, the real engine grading
#: launch and teardown cost what they cost whether the agent had 12 minutes or
#: 45. Carried forward unchanged, the 45-minute ceiling would have bought a
#: 90-minute cell bound -- far looser than the 2.0 was ever meant to license.
#:
#: WHAT THE ALLOWANCE IS FOR, in two parts:
#:
#: * INSIDE the enforced bracket (the deadline is armed before preflight and
#:   caps the whole session/deferral loop): preflight ``claude --version`` +
#:   the authenticated probe, template build + workspace instantiation, the
#:   pre-session port/repo sweeps -- and enough slack that ONE rate-limit
#:   deferral (``DEFAULT_RATE_LIMIT_BACKOFF_S``, 900 s) still leaves the retry
#:   its full budget rather than silently shortening it.
#: * OUTSIDE it, and honestly labelled as such: the post-session tail the
#:   deadline cannot enforce -- sweeps, R1 placement, the grading engine
#:   launch, teardown. It is counted here anyway so that
#:   ``cells x (budget + allowance)`` is a TRUE campaign cost bound rather
#:   than one that quietly excludes grading.
#:
#: SIZED FROM MEASUREMENT, not taste. Over the 90 committed cell reports in
#: ``results/`` (phasew_cc_v1 + the v0.3 pilots, machine 9722d23d12a3), the
#: non-session part of a cell -- ``(utc_end - utc_start) - cc_meta.wall_s`` --
#: ran median **28 s**, p90 **225 s**, max **838 s**. 900 s covers the
#: observed worst case with margin and equals one backoff, so the two things
#: the allowance pays for both fit inside it.
CELL_WALL_ALLOWANCE_S = 900.0


def cell_wall_bound_s(task_budget_s):
    """The whole cell's wall-clock bound for a task budget of ``task_budget_s``.

    One function so the runner, ``readiness.py`` and the budget regression test
    cannot disagree about what the tree declares. Under the current ceiling
    (2700 s) the bounds are A1 3060 s, C2/R1/R2/R3 2700 s, C1 2340 s, B1
    1800 s, B2/B3 1620 s.
    """
    return float(task_budget_s) + CELL_WALL_ALLOWANCE_S


#: The measurement protocol these rows were produced under, recorded on every
#: row. Owner's decision 2026-08-10: **one run per (task, arm) under a
#: wall-clock ceiling**, replacing n repeats -- repeats were the dominant token
#: cost and the suite could not afford them.
#:
#: It is an id, not a description, because it is the FENCE. Rows carrying a
#: different id -- or, for everything scored before this date, no ``protocol``
#: block at all -- ran under a different budget and a different repeat count
#: and may never be pooled with these (``tasks.TASK_HARD_CEILING_S`` is global,
#: so a ceiling change moves every task at once).
PROTOCOL_ID = "single-run-under-ceiling/2026-08-10"

#: Runs the protocol schedules per (task, arm). ONE. ``run_cell`` takes it as
#: an argument so a deliberate, recorded multi-run experiment can say so on its
#: own rows instead of masquerading as the protocol.
PROTOCOL_RUNS_PER_CELL = 1

# Deliverable conventions -- agents/external.py is the single authority.
ANSWER_TASKS = external_agent.ANSWER_TASKS            # B1, B3
ARTIFACT_NAME = external_agent.ARTIFACT_NAME          # world-task filenames
#: ``(task_id, sim) -> filename`` and ``sim -> ('.wbt',) | ('.xml',)``. Every
#: name this module stages goes through these, because the deliverable's file
#: format is the ARM's property, not the task's: OmniSim and Webots take a
#: ``.wbt`` world, MuJoCo takes an MJCF ``.xml`` plus the program that steps
#: it. A ``.get(task_id)`` on the flat registry silently stages a ``.wbt`` for
#: a simulator that cannot load one.
artifact_name = external_agent.artifact_name
artifact_suffixes = external_agent.artifact_suffixes
#: World tasks whose grader also reads the final message (committed proof).
WORLD_PLUS_ANSWER_TASKS = frozenset({"B2_subject_in_frame"})

# Webots-arm grading support (mirrors preregister/run_oracles.py exactly).
NEEDS_AABB = frozenset({"B1_overlap_audit", "B2_subject_in_frame",
                        "B3_measure_and_report", "C2_fall_through_floor"})
#: Graders that take no ``answer=`` argument.
NO_ANSWER_GRADERS = frozenset({"C1_parse_error_fix"})

# Env prefixes/keys never passed to the Claude Code child. CLAUDE_CONFIG_DIR
# is deliberately KEPT (it locates the subscription auth; scrubbing it would
# break login, not isolation).
SCRUB_PREFIXES = ("CLAUDE_CODE_", "AGENTBENCH_", "ANTHROPIC_", "OMNISIM_")
SCRUB_KEYS = ("CLAUDECODE", "CLAUDE_AGENT_ID", "CLAUDE_AGENT_NAME",
              "WEBOTS_HOME", "WEBOTS_PATH")

# --- headless enforcement, for BOTH arms ------------------------------------
#
# MEASURED 2026-08-09 (webots arm): the agent wrote its own
# ``scripts/run_sim.sh`` calling ``webots --batch --mode=fast`` with NO
# ``xvfb-run``. Our own launcher wraps every GRADING launch in ``xvfb-run``;
# nothing the AGENT launches inherits that, and WSLg supplies ``DISPLAY=:0``,
# so upstream opened real GUI windows on the operator's Windows desktop. That
# is not only a nuisance: GUI rendering changes the wall clock we measure, and
# median wall clock is the leaderboard's SECOND ordering key.
#
# THERE IS NO SINGLE VARIABLE BOTH SIMULATORS HONOUR -- checked in the source,
# not assumed. Every windowless knob in this tree is OURS (``OMNISIM_NO_WINDOW``
# 9ae34047, ``OMNISIM_NO_GL`` e285bc4b; ``grep qEnvironmentVariable`` over
# src/omnisim/gui + src/omnisim/app returns only ``OMNISIM_*`` names), upstream
# R2025a has no equivalent, and the one Qt-level knob that would cover both
# (``QT_QPA_PLATFORM``) CANNOT be set globally: the bundled Windows Qt6 ships
# ``qwindows.dll`` and ``qminimal.dll`` but no ``qoffscreen.dll``
# (msys64/mingw64/bin/platforms/), so the value that suits upstream aborts
# omnisim-bin before it starts, and ``minimal`` has no GL context at all.
#
# So what is symmetric is the POLICY, not the spelling: every cell's child env
# carries the windowless directive for ITS OWN engine, each directive is inert
# on the other arm, and a cell whose directive could NOT be established is
# FLAGGED rather than silently measured (same idiom as
# ``measured_under_concurrency``: the row says so and its wall clock is not
# comparable). Neither arm is handicapped and neither is favoured.
#
#   OMNISIM_NO_WINDOW=1  -- omnisim arm. ``OmGuiApplication`` takes the
#       ``setupNoWindow()`` path (src/omnisim/gui/OmGuiApplication.cpp:369) and
#       ``mMainWindow`` stays NULL: no window is ever created. Rendering is
#       UNTOUCHED (that is the difference from OMNISIM_NO_GL), so Camera,
#       Lidar and screenshots still work and the agent loses no capability.
#       Upstream reads no ``OMNISIM_*`` variable, so it is inert on the webots
#       arm.
#   DISPLAY=:99 (+ WSLENV) -- webots arm. Windows Qt never reads ``DISPLAY``,
#       so it is inert for omnisim-bin; ``WSLENV`` is what carries it across
#       the Win->WSL boundary (nothing else in the Windows env reaches WSL), so
#       every ``wsl.exe`` the agent spawns -- with or without ``xvfb-run`` --
#       lands on the lane's virtual display instead of WSLg's ``:0``.
#       ``xvfb-run -a`` (our grading launcher) allocates its OWN display and is
#       unaffected either way.
WEBOTS_HEADLESS_DISPLAY = ":99"
#: Same geometry Debian's ``xvfb-run`` defaults to, deliberately: our grading
#: launcher already runs upstream Webots under ``xvfb-run -a``, so a server
#: started with those arguments is the environment upstream is known to work
#: in here -- the closest thing to evidence available without a launch.
XVFB_SCREEN = "1280x1024x24"

HEADLESS_ENV = {
    "OMNISIM_NO_WINDOW": "1",
    "DISPLAY": WEBOTS_HEADLESS_DISPLAY,
    "WSLENV": "DISPLAY/u",
}

# Fallback when --dangerously-skip-permissions is refused in this
# environment (recorded into the row either way).
FALLBACK_ALLOWED_TOOLS = (
    "Bash", "Edit", "Write", "Read", "Glob", "Grep", "NotebookEdit",
    "WebFetch", "WebSearch", "TodoWrite", "Task")


# --- pure helpers (unit-tested without Claude Code) ---------------------------


def scrub_env(base=None, *, headless=True):
    """(child_env, removed_keys): the nesting/benchmark/credential vars gone.

    ``ANTHROPIC_API_KEY`` stays ABSENT by design: the cell runs on the
    operator's Claude subscription login, and the key (if any is exported for
    the grader backends) must not leak into the product session.

    This is also where the headless directives go in, for BOTH arms at once
    (see ``HEADLESS_ENV`` above for why the policy is symmetric even though
    the variables cannot be). Applied AFTER the scrub, because
    ``OMNISIM_NO_WINDOW`` starts with a scrubbed prefix and would otherwise
    remove itself.
    """
    base = dict(os.environ if base is None else base)
    removed = []
    for k in list(base):
        if k in SCRUB_KEYS or k.startswith(SCRUB_PREFIXES):
            removed.append(k)
            base.pop(k)
    if headless:
        for k, v in HEADLESS_ENV.items():
            if k == "WSLENV":
                base[k] = _merge_wslenv(base.get("WSLENV"), v)
            else:
                base[k] = v
    return base, sorted(removed)


def _merge_wslenv(existing, entry):
    """Add one ``NAME/flags`` entry to a ``WSLENV`` list without clobbering
    whatever the operator already shares across the WSL boundary."""
    parts = [p for p in (existing or "").split(":") if p]
    name = entry.split("/")[0]
    parts = [p for p in parts if p.split("/")[0] != name]
    parts.append(entry)
    return ":".join(parts)


def drop_webots_headless(env):
    """Undo the webots-arm headless directive in a child env.

    Used when the lane's virtual display could not be brought up: forwarding a
    ``DISPLAY`` that answers nothing would leave the webots agent unable to
    start the simulator at all, which is a far worse bias than a visible
    window. The cell is flagged instead.
    """
    env.pop("DISPLAY", None)
    parts = [p for p in (env.get("WSLENV") or "").split(":") if p
             and p.split("/")[0] != "DISPLAY"]
    if parts:
        env["WSLENV"] = ":".join(parts)
    else:
        env.pop("WSLENV", None)
    return env


def ensure_virtual_display(display=WEBOTS_HEADLESS_DISPLAY, *, distro=None,
                           screen=XVFB_SCREEN, timeout_s=120.0, runner=None):
    """Bring up (or reuse) the lane's Xvfb inside the Webots distro.

    Returns ``{"ok", "display", "state", "detail"}`` and NEVER raises -- a
    machine with no WSL, no distro or no Xvfb is a machine where the webots
    arm's headless directive simply cannot be enforced, and that has to be
    recorded rather than crash a cell.

    Idempotent by design: cells run back to back and the server is reused, so
    this costs one ``wsl.exe`` round trip after the first cell. It is started
    detached and deliberately NOT torn down between cells; ``pkill -f 'Xvfb
    :99'`` inside the distro is the manual stop.
    """
    distro = distro or staging.WEBOTS_DISTRO
    n = str(display).lstrip(":").split(".")[0]
    script = "\n".join([
        "set -u",
        "N=%s" % n,
        # A live server is reused. pgrep first (authoritative); the socket is
        # the fallback for a distro without procps.
        'if command -v pgrep >/dev/null 2>&1; then',
        '  pgrep -f "Xvfb :$N" >/dev/null 2>&1 && { echo REUSED; exit 0; }',
        'elif [ -e "/tmp/.X11-unix/X$N" ]; then echo REUSED; exit 0; fi',
        'command -v Xvfb >/dev/null 2>&1 || { echo NOXVFB; exit 3; }',
        'nohup Xvfb ":$N" -screen 0 %s -nolisten tcp '
        '>"/tmp/agentbench_xvfb_$N.log" 2>&1 &' % screen,
        'for _ in $(seq 1 40); do',
        '  [ -e "/tmp/.X11-unix/X$N" ] && { echo STARTED; exit 0; }',
        '  sleep 0.25',
        'done',
        'echo TIMEOUT; cat "/tmp/agentbench_xvfb_$N.log" 2>/dev/null | tail -5',
        'exit 4',
    ]) + "\n"
    run = runner or (lambda cmd: subprocess.run(
        cmd, capture_output=True, text=True, timeout=timeout_s,
        encoding="utf-8", errors="replace"))
    try:
        proc = run(["wsl", "-d", distro, "--", "bash", "-lc", script])
    except (OSError, subprocess.SubprocessError) as exc:
        return {"ok": False, "display": display, "state": "wsl_unavailable",
                "detail": repr(exc)}
    out = ((proc.stdout or "") + (proc.stderr or "")).strip()
    for marker in ("REUSED", "STARTED"):
        if marker in out:
            return {"ok": True, "display": display, "state": marker.lower(),
                    "detail": out[-400:]}
    state = ("xvfb_missing" if "NOXVFB" in out
             else "start_timeout" if "TIMEOUT" in out else "failed")
    return {"ok": False, "display": display, "state": state,
            "detail": "rc=%s %s" % (proc.returncode, out[-400:])}


def enforce_headless(env, sim, *, ensure=None):
    """Tailor the shared headless directives to the arm this cell runs on.

    Returns the provenance dict recorded into the cell report AND the row --
    ``enforced`` is what a reader needs to know before comparing this cell's
    wall clock with another's.
    """
    if sim == "mujoco":
        # Nothing to suppress and nothing suppressed. MuJoCo opens no window
        # of its own -- it is a library, and only an agent's own program can
        # open the interactive viewer. ``MUJOCO_GL`` is deliberately NOT
        # pinned (adapters/mujoco/launcher.child_env): forcing a software GL
        # backend would be the grader changing what the agent's program does,
        # and a driver that opens a viewer failing on a headless box is the
        # honest outcome of a benchmark that is headless for every arm.
        #
        # So this is ``enforced: True`` in the only sense the flag carries --
        # the cell's wall clock IS comparable, no GUI can appear behind our
        # back -- and the mechanism says plainly that we did nothing to earn
        # it. The omnisim directive comes back out because it is inert here.
        drop_webots_headless(env)
        env.pop("OMNISIM_NO_WINDOW", None)
        return {"arm": sim, "enforced": True,
                "mechanism": "structural (no GUI to suppress)",
                "vars": {},
                "detail": ("MuJoCo is a library and starts no window; only "
                           "the agent's own driver could open the viewer. "
                           "MUJOCO_GL is deliberately not pinned -- that "
                           "would change what the agent's program does -- so "
                           "a driver that needs a display fails as the "
                           "agent's error, which is the honest outcome")}
    if sim != "webots":
        # The DISPLAY pair is for upstream inside WSL. It is inert on Windows,
        # but forwarding it from an omnisim cell could only confuse a WSL
        # command the agent had some other reason to run, so it comes back out.
        drop_webots_headless(env)
        return {"arm": sim, "enforced": True,
                "mechanism": "OMNISIM_NO_WINDOW=1",
                "vars": {"OMNISIM_NO_WINDOW": env.get("OMNISIM_NO_WINDOW")},
                "detail": ("OmGuiApplication takes setupNoWindow(); no main "
                           "window is created and rendering is untouched")}
    probe = (ensure or ensure_virtual_display)()
    if not probe.get("ok"):
        drop_webots_headless(env)
        return {"arm": sim, "enforced": False,
                "mechanism": "DISPLAY=%s via WSLENV" % WEBOTS_HEADLESS_DISPLAY,
                "vars": {}, "virtual_display": probe,
                "detail": ("the lane's virtual display could not be brought "
                           "up (%s), so DISPLAY was NOT forwarded -- a dead "
                           "DISPLAY would stop the agent launching the "
                           "simulator at all. This cell's engine launches may "
                           "have rendered to the operator's desktop and its "
                           "wall clock is not comparable"
                           % probe.get("state"))}
    return {"arm": sim, "enforced": True,
            "mechanism": "DISPLAY=%s via WSLENV" % WEBOTS_HEADLESS_DISPLAY,
            "vars": {"DISPLAY": env.get("DISPLAY"),
                     "WSLENV": env.get("WSLENV")},
            "virtual_display": probe,
            "detail": ("every wsl.exe the agent spawns inherits DISPLAY, so "
                       "an unwrapped `webots` renders to the lane's Xvfb "
                       "instead of WSLg's :0")}


def count_tool_calls(transcript_path):
    """Assistant ``tool_use`` blocks in a Claude Code session .jsonl.

    Returns ``(count, None)`` or ``(None, reason)`` -- never a guess.
    """
    p = Path(transcript_path)
    try:
        text = p.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        return None, "transcript unreadable: %r" % (exc,)
    n = 0
    parsed_any = False
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        parsed_any = True
        msg = rec.get("message")
        if not isinstance(msg, dict) or msg.get("role") != "assistant":
            continue
        content = msg.get("content")
        if isinstance(content, list):
            n += sum(1 for b in content
                     if isinstance(b, dict) and b.get("type") == "tool_use")
    if not parsed_any:
        return None, "transcript had no parseable JSONL lines"
    return n, None


def find_transcript(session_id, claude_home=None):
    """The session transcript ``<session_id>.jsonl`` under
    ``~/.claude/projects/``, or None."""
    root = Path(claude_home or (Path.home() / ".claude")) / "projects"
    if not session_id or not root.is_dir():
        return None
    hits = sorted(root.glob("*/%s.jsonl" % session_id),
                  key=lambda p: p.stat().st_mtime, reverse=True)
    return hits[0] if hits else None


#: Files under the session's own project slug that the transcript copy already
#: covers, so they are not duplicated into the evidence dir.
_SESSION_HOME_SKIP_SUFFIXES = (".jsonl",)

#: Ceiling on the collected session home. Memory files and settings are
#: kilobytes; anything larger is recorded by name, never silently copied.
SESSION_HOME_MAX_BYTES = 8 * 1024 * 1024


def session_home_slug(workspace, claude_home=None):
    """The ``~/.claude/projects/<slug>/`` directory for a session whose cwd was
    ``workspace``, or None.

    Claude Code slugifies the cwd (drive colon and separators -> ``-``), so the
    slug is derivable rather than searched for. Falls back to a case-insensitive
    scan when the exact spelling does not match.
    """
    root = Path(claude_home or (Path.home() / ".claude")) / "projects"
    if not root.is_dir() or not workspace:
        return None
    raw = str(workspace).replace("\\", "/").rstrip("/")
    slug = raw.replace(":", "-").replace("/", "-").replace("_", "-")
    exact = root / slug
    if exact.is_dir():
        return exact
    want = slug.lower()
    for p in root.iterdir():
        if p.is_dir() and p.name.lower() == want:
            return p
    return None


def collect_session_home(run_dir, *, workspace, session_id=None,
                         claude_home=None, max_bytes=SESSION_HOME_MAX_BYTES):
    """Copy the session's own ``~/.claude/projects/<slug>/`` state into
    ``<cell>/claude_home/`` and record what was found.

    WHY. A cell session writes persistent memory (``memory/MEMORY.md`` and
    friends) into the OPERATOR's real ``~/.claude`` tree -- outside the
    sandbox and outside the preserved evidence. Measured 2026-08-12: five
    agentbench instance slugs on this machine carry a ``memory/`` directory
    that no cell report mentions. That is state the agent produced, so it is
    evidence; leaving it only in the operator's home means a reader cannot see
    what the session decided to remember, and a reader of the operator's home
    cannot tell which cell wrote it.

    Scoped to THIS session's slug, which is derived from the cell's own
    workspace path and is therefore unique per cell: the operator's own
    project slugs (``o--omnisim`` and friends) are never read and never
    touched. Nothing is deleted -- collection only.
    """
    run_dir = Path(run_dir)
    rec = {"claude_home": str(claude_home or (Path.home() / ".claude")),
           "slug_dir": None, "dest": None, "files": 0, "bytes": 0,
           "entries": [], "reason": None, "truncated": False}
    slug = session_home_slug(workspace, claude_home)
    if slug is None:
        rec["reason"] = ("no ~/.claude/projects slug matches this cell's "
                         "workspace %r; nothing collected (a session that "
                         "never started writes none)" % str(workspace))
        return rec
    rec["slug_dir"] = str(slug)
    dest = run_dir / "claude_home"
    rec["dest"] = str(dest)
    total = 0
    for src in sorted(slug.rglob("*")):
        if not src.is_file():
            continue
        rel = str(src.relative_to(slug)).replace("\\", "/")
        if src.name.lower().endswith(_SESSION_HOME_SKIP_SUFFIXES):
            rec["entries"].append({"path": rel, "action": "skipped_transcript"})
            continue
        try:
            size = src.stat().st_size
        except OSError as exc:
            rec["entries"].append({"path": rel, "action": "unreadable",
                                   "detail": repr(exc)})
            continue
        if total + size > max_bytes:
            rec["truncated"] = True
            rec["entries"].append({"path": rel, "bytes": size,
                                   "action": "skipped_over_budget"})
            continue
        out = dest / rel
        try:
            out.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, out)
        except OSError as exc:
            rec["entries"].append({"path": rel, "action": "copy_failed",
                                   "detail": repr(exc)})
            continue
        total += size
        rec["files"] += 1
        rec["bytes"] += size
        rec["entries"].append({"path": rel, "bytes": size,
                               "action": "collected"})
    if rec["files"] == 0 and not rec["reason"]:
        rec["reason"] = "the slug exists but holds no collectable files"
    return rec


# --- CONTAINMENT: did the session read the benchmark's own internals? --------
#
# MEASURED 2026-08-11, the first R4/omnisim cell. The workspace we hand an
# agent is a FILTERED copy of the product: `stage_workspaces` removes the
# whole `tests/benchmarks/agentbench/` tree precisely because it holds the
# graders, their thresholds, the oracle fixtures, the task metas and the
# recorded verdicts. The filter is real and it works -- and it is enforced on
# the COPY only. The session ran `ls -la /o/omnisim/`, found the source tree
# at its real path, and read `graders/r4_core.py` (twice -- the second read
# offset onto the assertion bodies), `graders/r4.py`, the task's `meta.json`
# and the adapters' recorder internals. 22 tool calls reached the tree.
#
# That is an ANSWER KEY, so the score of such a cell is not a measurement of
# an agent and must never enter a grid. Nothing about it is the agent's fault:
# the lane cannot path-confine `run_shell` on a bare host (the same limit
# `adapters/omnisim/headless.controller_shadow_check` states for its own
# tamper guard), and the product docs we deliberately DO stage name the
# checkout. Preventing it needs a sandbox and is a SPEC-level change.
#
# What is fixable today is that it happened SILENTLY. A contaminated cell and
# a clean one produced identical reports, so the only way to find this was to
# read 1.4 MB of NDJSON by hand. So: audit every session against the lane's
# OWN exclusion list -- one list, two enforcements (staging removes them, this
# catches a session that reached them another way) -- record the verdict on
# every cell including the clean ones, and BLOCK rather than score, exactly as
# the liveness gate does. The evidence is preserved either way.

def _containment_needles():
    """``(label, compiled pattern)`` for every path a session must not read.

    Derived from ``stage_workspaces``' own exclusion lists so there is ONE
    declaration of "the agent must not see this", not two that can drift.

    Matched as PATHS, not as words, because the haystack is a shell command
    line as often as a file path:

    * a multi-component prefix (``tests/benchmarks/agentbench``) is matched
      without its trailing slash and with a boundary after it, so both
      ``.../agentbench/graders/r4_core.py`` and ``find ... /agentbench -name``
      are caught while ``agentbench_notes`` is not. The measured cell used
      BOTH spellings, so requiring the trailing slash would have missed real
      hits;
    * a single-component prefix (``cloud/``, ``social/``) keeps its slash and
      demands a boundary before it, so ``cd cloud/x`` matches and the English
      word in ``grep -r "cloud" docs/`` does not. A needle that fires on prose
      trains the next reader to ignore the gate.
    """
    out = []
    for p in staging.OMNISIM_EXCLUDE_PREFIXES:
        s = str(p).replace("\\", "/").strip("/").lower()
        if not s:
            continue
        core = re.escape(s)
        pat = (core + r"(?![a-z0-9_.\-])" if "/" in s
               else r"(?<![a-z0-9_.\-])" + core + "/")
        out.append((s + "/", re.compile(pat)))
    for f in staging.OMNISIM_EXCLUDE_FILES:
        s = str(f).replace("\\", "/").strip("/").lower()
        if s:
            out.append((s, re.compile(re.escape(s))))
    return tuple(out)


def _tool_use_inputs(path):
    """Every ``tool_use`` input in a Claude Code NDJSON file, as JSON text.

    Works on both shapes the lane keeps: the ``--output-format stream-json``
    events written to ``<cell>/cc_stream.jsonl`` while the session runs, and
    the richer session transcript under ``~/.claude/projects/``. Both nest the
    blocks under ``message.content``; neither is trusted to parse, so an
    unreadable or malformed line is skipped rather than raising.
    """
    out = []
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return out
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        msg = rec.get("message")
        if not isinstance(msg, dict):
            continue
        content = msg.get("content")
        if not isinstance(content, list):
            continue
        for b in content:
            if isinstance(b, dict) and b.get("type") == "tool_use":
                try:
                    out.append((b.get("name"),
                                json.dumps(b.get("input") or {},
                                           default=str)))
                except (TypeError, ValueError):
                    continue
    return out


def audit_containment(sources, *, tool_calls=None, hit_cap=60):
    """Did the session reach anything the workspace filter removed?

    ``sources`` are NDJSON paths (the cell's stream and, when it exists, the
    session transcript). ``tool_calls`` is the count measured independently
    from the transcript, and it is what makes an empty scan interpretable.
    Returns a record; ``clean`` is:

      * ``True``  -- tool calls were read and none named an excluded path,
      * ``False`` -- at least one did,
      * ``None``  -- no tool call could be read, so the question is OPEN.

    ``None`` is not ``True``, and the difference is the point: a cell whose
    stream vanished has not been shown clean. But ``None`` is not
    automatically a blocker either, because "no tool call was recorded" and
    "the session made tool calls we cannot see" are different situations and
    only the second is dangerous. :func:`containment_blocks` draws that line
    with ``tool_calls``; this function only measures.

    The match is over the tool-call INPUT text (paths, shell command lines,
    grep roots), lowercased and slash-normalised, so a Windows path, a WSL
    path and a `cd` into the tree all read the same. It cannot see a path the
    agent built at run time inside a script -- stated rather than implied.
    """
    needles = _containment_needles()
    calls, readable = [], 0
    for src in sources:
        if src is None:
            continue
        got = _tool_use_inputs(src)
        if got:
            readable += 1
        calls.extend(got)
    if not calls:
        return {"clean": None, "sources_read": readable,
                "tool_calls_scanned": 0, "session_tool_calls": tool_calls,
                "reason": "no parseable tool call in any source, so "
                          "containment was NOT established from the "
                          "session's own record",
                "needles": len(needles), "hit_count": 0, "hits": []}
    hits, seen = [], set()
    for name, blob in calls:
        low = blob.replace("\\\\", "/").replace("\\", "/").lower()
        for label, pat in needles:
            if pat.search(low):
                key = (name, blob)
                if key in seen:
                    continue
                seen.add(key)
                hits.append({"tool": name, "matched": label,
                             "input": blob[:400]})
                break
    return {"clean": not hits, "sources_read": readable,
            "tool_calls_scanned": len(calls), "session_tool_calls": tool_calls,
            "needles": len(needles),
            "hit_count": len(hits), "hits": hits[:hit_cap],
            "hits_truncated": len(hits) > hit_cap}


def containment_blocks(audit):
    """Must this cell be abandoned rather than scored? ``(bool, reason)``.

    Two blocking cases, and the second is deliberately narrow:

    1. **Measured contamination.** A tool call named an excluded path. The
       session had the graders, their thresholds or the task meta, so its
       score is not a measurement of an agent.
    2. **A blind spot over a session that DID use tools.** Nothing was
       readable while the transcript counted tool calls, so the session acted
       and we cannot see what it touched. Unestablished, on evidence that
       exists somewhere -- not scored.

    A session with no recorded tool calls is NOT blocked: it could not have
    read anything, and blocking it would turn a legitimately quiet run (or a
    synthetic harness) into an instrument failure.
    """
    if audit.get("clean") is False:
        return True, describe_containment(audit)
    if audit.get("clean") is None and (audit.get("session_tool_calls") or 0) > 0:
        return True, describe_containment(audit)
    return False, None


def describe_containment(audit):
    """The blocker text for a contaminated (or unestablished) cell."""
    if audit.get("clean") is None:
        return ("containment could NOT be established: %s -- while the "
                "session's own transcript counted %s tool call(s). A session "
                "that acted where we cannot see what it touched is not "
                "scored: 'no evidence' and 'checked and clean' are different "
                "claims."
                % (audit.get("reason", "no reason recorded"),
                   audit.get("session_tool_calls")))
    tops = ", ".join(sorted({h["matched"] for h in audit.get("hits", [])}))
    return ("CONTAMINATED: %d tool call(s) reached paths the workspace filter "
            "removes (%s). Those trees hold the graders, their thresholds, "
            "the oracle fixtures and the task metas, so this session had the "
            "answer key and its score is not a measurement of an agent. The "
            "cell is abandoned rather than scored; the workspace and the "
            "collected deliverable are preserved for inspection."
            % (audit.get("hit_count", 0), tops or "unknown"))


# --- THE ROUND'S ENGINE: one binary, or the cells are not comparable ---------
#
# A diagnostic round compares what several agents hit on the SAME simulator.
# The engine binary is a build artifact in a working tree that is rebuilt
# routinely, and a rebuild between cell 1 and cell 3 makes a defect that
# appears in 1/3 unreadable: it could be a systematic defect that got fixed, or
# one agent's bad day, and nothing in the rows distinguishes them.
#
# So the round FREEZES a copy of the binary under a round-specific name, and
# every cell records the identity of the binary it actually ran and refuses to
# run against a different one. The copy is the archive (it survives a rebuild);
# the sha256 assertion is the pin.
#
# libController rides along because the engine<->libController ABI split hangs
# every controller at zero ticks while a headless run still prints PASS
# (`6eea9d76`), which would look exactly like an agent whose robot did nothing.

ENGINE_BIN_CANDIDATES = (
    "msys64/mingw64/bin/omnisim-bin.exe",           # Windows
    "bin/omnisim-bin",                              # Linux
)
LIBCONTROLLER_CANDIDATES = (
    "lib/controller/Controller.dll",
    "lib/controller/libController.so",
)


def _file_identity(path):
    try:
        p = Path(path)
        data = p.read_bytes()
        st = p.stat()
    except OSError as exc:
        return {"path": str(path), "present": False, "error": str(exc)}
    return {"path": str(path), "present": True, "bytes": st.st_size,
            "sha256": hashlib.sha256(data).hexdigest(),
            "mtime_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                       time.gmtime(st.st_mtime))}


def engine_identity(home=None):
    """Which engine binary (and libController) is on disk right now."""
    home = Path(home or staging.REPO)
    out = {"home": str(home), "engine": None, "libcontroller": None}
    for rel in ENGINE_BIN_CANDIDATES:
        if (home / rel).exists():
            out["engine"] = _file_identity(home / rel)
            break
    for rel in LIBCONTROLLER_CANDIDATES:
        if (home / rel).exists():
            out["libcontroller"] = _file_identity(home / rel)
            break
    return out


def engine_pin_blocks(identity, pin):
    """``(bool, reason)`` -- did the engine change under this round?"""
    if not pin:
        return False, None
    got = ((identity or {}).get("engine") or {}).get("sha256")
    if got == pin:
        return False, None
    return True, (
        "ENGINE PIN MISMATCH: this round is pinned to omnisim-bin sha256 %s "
        "and the binary on disk is %s. Every cell in a round must run the "
        "identical engine or its observations are not comparable to the other "
        "cells or to a later round. The cell is abandoned rather than scored."
        % (pin, got or "(no engine binary found)"))


# --- CONTAINMENT, part 2: PREVENTION -----------------------------------------
#
# `audit_containment` above DETECTS a session that read the answer key. That is
# the right last resort and the wrong first one: it costs a whole cell, and the
# operator's quota, to learn something we could have refused in 2 ms.
#
# `containment_guard` is the first resort -- a PreToolUse hook, installed per
# cell through `claude --settings`, that sees every tool call's input before the
# tool runs and denies the ones that reach outside the cell. Read that module's
# docstring for the RULE and its rationale (what the sandbox admits, why engine
# source under src/ is admitted, and what a hook structurally cannot see).
#
# VERIFIED 2026-08-12 on claude-code 2.1.179, with `--dangerously-skip-
# permissions` set exactly as a real cell sets it: `ls -la /o/omnisim/` and
# `Read O:\omnisim\tests\benchmarks\agentbench\SPEC.md` were both DENIED before
# execution; a read inside the workspace was allowed. Hooks are not part of the
# permission system, so bypassing permissions does not bypass them.
#
# The settings file is kept MINIMAL and exactly the shape that was verified.
# `claude -p` states that "settings files that fail validation are silently
# ignored" -- so an extra, unvalidated key in this file would not raise, it
# would turn the guard off without saying so. Nothing goes in it that has not
# been observed to work.

GUARD_DIRNAME = "guard"


def install_guard(run_dir, workspace, *, repo=None, scratch_root=None,
                  python=None):
    """Write the per-cell guard (hook script + config + settings file).

    Lives in ``<run_dir>/guard/``, OUTSIDE the workspace, and the run dir is
    itself on the guard's protected list so the session cannot read or rewrite
    the thing refusing it. Returns the record that goes in the report.
    """
    gdir = Path(run_dir) / GUARD_DIRNAME
    gdir.mkdir(parents=True, exist_ok=True)
    script = gdir / "containment_guard.py"
    shutil.copy2(Path(containment_guard.__file__), script)
    log = gdir / "guard_events.jsonl"
    cfg = {
        "workspace": str(workspace),
        "repo": str(repo or staging.REPO),
        "scratch_root": str(scratch_root) if scratch_root else None,
        # From staging's own declarations, never re-typed here: one list of
        # "the agent must not see this", now THREE enforcements (the staging
        # filter, this guard, and the post-hoc audit).
        "junction_dirs": list(staging.OMNISIM_JUNCTION_DIRS),
        "exclude_prefixes": list(staging.OMNISIM_EXCLUDE_PREFIXES),
        "exclude_files": list(staging.OMNISIM_EXCLUDE_FILES),
        "protect": [str(run_dir)],
        "log": str(log),
    }
    (gdir / containment_guard.CONFIG_NAME).write_text(
        json.dumps(cfg, indent=2), encoding="utf-8")
    settings = {"hooks": {"PreToolUse": [{"matcher": "*", "hooks": [
        {"type": "command",
         "command": '"%s" "%s"' % (python or sys.executable, script),
         "timeout": 20}]}]}}
    spath = gdir / "settings.json"
    spath.write_text(json.dumps(settings, indent=2), encoding="utf-8")
    return {"installed": True, "settings": str(spath), "script": str(script),
            "config": str(gdir / containment_guard.CONFIG_NAME),
            "log": str(log), "python": str(python or sys.executable),
            "rule": ("the cell's staged workspace only; the real repo "
                     "checkout is denied except its junctioned runtime dirs "
                     "(%s); the staging exclusion list is denied by name; "
                     "other cells, the staging templates and this cell's run "
                     "dir are denied"
                     % ", ".join(staging.OMNISIM_JUNCTION_DIRS))}


def read_guard_log(rec, *, keep=40):
    """Summarise what the guard actually did. Mutates and returns ``rec``.

    ``enforced`` is the instrument check that matters: the hook wrote a line
    for every tool call it saw, so an EMPTY log after a session that made tool
    calls means the settings file was not loaded and nothing was guarded --
    which `claude -p` does silently. That reads as a clean cell and is not one.
    """
    path = rec.get("log")
    seen, denied, denials = 0, 0, []
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except (OSError, TypeError):
        text = ""
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            evt = json.loads(line)
        except ValueError:
            continue
        seen += 1
        if not evt.get("allow"):
            denied += 1
            if len(denials) < keep:
                denials.append(evt)
    rec.update(calls_seen=seen, denied=denied, denials=denials,
               enforced=seen > 0)
    return rec


def guard_blocks(rec, *, tool_calls=None):
    """``(bool, reason)`` -- was the guard actually in force?

    Blocks only on the instrument failure: the guard was installed, the
    session made tool calls, and the guard saw NONE of them. That is a cell
    run with no containment at all, and it must not be scored as though it had
    some. A session with no tool calls is not blocked (nothing to guard), and
    neither is a cell where the guard was deliberately not installed -- that
    is recorded in the report and is the caller's declared choice.
    """
    if not rec or not rec.get("installed"):
        return False, None
    if rec.get("enforced"):
        return False, None
    if (tool_calls or 0) <= 0:
        return False, None
    return True, (
        "the containment guard was installed but saw NONE of the session's "
        "%s tool call(s): the hook never ran, so this cell was NOT contained. "
        "`claude -p` ignores a settings file that fails validation without "
        "saying so, which is exactly what this check exists to catch. The "
        "cell is abandoned rather than scored; evidence is preserved."
        % tool_calls)


#: What a recursive search from the workspace root is expected to find. One
#: entry per (label, glob) the diagnostic round measured going missing.
SEARCH_PROBES = (
    ("controller_dirs", "**/controllers/*/"),
    ("worlds", "**/worlds/*.wbt"),
)


def search_visibility(workspace, *, junction_dirs=None, cap=20000):
    """How much of the workspace the AGENT's own search tools cannot see.

    THE INSTRUMENT ARTEFACT, made visible instead of left to distort silently.
    MEASURED 2026-08-12 on 3/3 cells: ``Glob **/controllers/drive_forward/*.py``
    at the workspace root returns *No files found* while the identical Glob at
    the real checkout finds it; one cell's ``find . -type d -name controllers``
    returned ``scripts/`` and ``tests/`` and **none of the 47 shipped
    controller directories**. The cause is mechanical -- the runtime dirs are
    NTFS junctions, and ripgrep (which Glob/Grep are built on), ``find`` and
    ``grep -r`` all refuse to traverse a reparse point (re-verified here:
    ``rg --files`` under a junction returns 0 files, and it skips symlinked
    FILES too, so a per-file symlink mirror would not help either).

    The consequence is that an agent in this workspace concludes the product
    ships no examples -- which is a statement about our staging, not about
    OmniSim. Until the staging change that fixes it (a real ``projects/`` tree;
    costed in the round report), every cell records this block so no reader can
    mistake the artefact for a product property.

    Returns counts for the link-refusing walk (what the agent sees) and the
    link-following walk (what is really there), never raising.
    """
    ws = Path(workspace)
    dirs = tuple(junction_dirs or staging.OMNISIM_JUNCTION_DIRS)
    out = {"workspace": str(ws), "junction_dirs": list(dirs),
           "probes": {}, "biased": False,
           "note": ("counts from a link-REFUSING walk (what ripgrep/find/Glob "
                    "see) against a link-FOLLOWING walk (what is on disk)")}

    def _walk(follow):
        ctrl, worlds, seen = 0, 0, set()
        n = 0
        for dirpath, dirnames, filenames in os.walk(ws, followlinks=follow):
            n += 1
            if n > cap:
                break
            if follow:
                dirnames[:] = [d for d in dirnames
                               if d not in ("msys64", ".git", "__pycache__")]
                try:
                    key = os.path.realpath(dirpath)
                except OSError:
                    key = dirpath
                if key in seen:
                    dirnames[:] = []
                    continue
                seen.add(key)
            else:
                dirnames[:] = [d for d in dirnames
                               if not staging.is_link(Path(dirpath) / d)]
            if Path(dirpath).name == "controllers":
                ctrl += len(dirnames)
            worlds += sum(1 for f in filenames if f.lower().endswith((".wbt", ".omniworld")))
        return {"controller_dirs": ctrl, "worlds": worlds}

    try:
        visible = _walk(False)
        present = _walk(True)
    except OSError as exc:
        out["error"] = repr(exc)
        return out
    for key in ("controller_dirs", "worlds"):
        out["probes"][key] = {"visible_to_search": visible[key],
                              "present_on_disk": present[key],
                              "hidden_behind_links": max(
                                  0, present[key] - visible[key])}
        if present[key] > visible[key]:
            out["biased"] = True
    return out


def _authoring_dir(artifact, run_dir):
    """The directory the agent's world was WRITTEN in, not where it now sits.

    Relative asset urls only mean anything against their authoring directory,
    and by collection time the deliverable may already have been moved once:
    the junction-hygiene sweep preserves anything the session wrote through
    the ``projects/`` junction into ``<run_dir>/repo_artifacts/<same relative
    path>`` and DELETES the repo copy, so the original directory is gone.

    Because the sweep mirrors the repo-relative path exactly, the original
    location is recoverable: ``REPO / <path relative to repo_artifacts>``.
    Measured on the first v0.3 pilot (A1/omnisim): the agent saved to
    ``projects/samples/demos/worlds/showcase/`` -- where its
    ``../../../../robots/clearpath/.../husky.urdf`` resolves correctly -- and
    the preserved mirror has no ``robots/`` sibling, so re-basing against the
    mirror would have rewritten nothing and Phase B would still have loaded
    zero robots.
    """
    artifact = Path(artifact)
    mirror_root = Path(run_dir) / "repo_artifacts"
    try:
        rel = artifact.relative_to(mirror_root)
    except ValueError:
        return artifact.parent          # collected straight from the workspace
    return (REPO / rel).parent


# --- the deliverable is a PROJECT, not a file --------------------------------
#
# MEASURED 2026-08-09 (R1/omnisim): the agent wrote its controller to
# ``projects/samples/demos/controllers/avoid_obstacles/avoid_obstacles.py`` and
# a world declaring ``controller "avoid_obstacles"``. Collection took ONLY the
# ``.wbt``. The collected world therefore had no ``controllers/`` sibling, the
# engine's search (``OmRobot::updateControllerDir``) found no directory, the
# robot got no controller and never moved: path length 0.0 m, two assertions
# failed -- on OUR bug, not the agent's work.
#
# It is the same defect class ``rebase_relative_urls`` fixed for URDF urls
# (A1/omnisim): an authoring agent's deliverable is a PROJECT, and lifting one
# file out of it breaks the references the project supplied.

#: ``controller`` values that name no directory: the engine's own sentinels
#: (``OmRobot::updateControllerDir``, src/omnisim/nodes/OmRobot.cpp:525).
CONTROLLER_SENTINELS = frozenset({"<none>", "<extern>", "<generic>"})

#: Controllers the GRADER owns. ``adapters/omnisim/headless.py``
#: (GRADER_CONTROLLERS) is the authority; named here too so this module stays
#: importable without numpy. Collecting one is not blocked -- the shipped
#: tamper check refuses the run and says why, which is the honest outcome --
#: but it IS flagged, so an INVALID verdict is explainable from the row.
GRADER_OWNED_CONTROLLERS = frozenset({"agentbench_recorder",
                                      "harness_supervisor"})

_CONTROLLER_IGNORE = shutil.ignore_patterns("__pycache__", "*.pyc", ".git")


def project_root_for_world(world):
    """The project root the ENGINE resolves for ``world``.

    ``OmProject::projectPathFromWorldFile`` (src/omnisim/core/OmProject.cpp):
    walk UP for the nearest ancestor literally named ``worlds`` and return its
    parent; with none, the world file's parent's parent. Transcribed in
    ``adapters/omnisim/headless.py`` and ``adapters/webots/launcher.py`` too --
    all three agree; this copy exists so the cell runner does not have to
    import numpy (headless.py) to place a directory.
    """
    world_dir = Path(world).resolve().parent
    cur = world_dir
    while True:
        if cur.name == "worlds":
            return cur.parent
        if cur.parent == cur:
            break
        cur = cur.parent
    return world_dir.parent


def controller_names(world_path):
    """Every ``controller "<name>"`` the world text names, in order, deduped.

    Reuses ``common/worldtext``'s own SFString field regex family rather than
    adding a second ``.wbt`` parser, and strips comments first so a
    commented-out Robot never drags a directory along. The sentinels are
    dropped: they name behaviour, not a directory.
    """
    try:
        text = worldtext.strip_comments(
            Path(world_path).read_text(encoding="utf-8", errors="replace"))
    except OSError:
        return []
    out = []
    for name in worldtext._SF_STRING["controller"].findall(text):
        name = name.strip()
        if not name or name in CONTROLLER_SENTINELS or name in out:
            continue
        # A controller field is a NAME, not a path; anything path-shaped is
        # not something the engine would resolve to <project>/controllers/<x>.
        if "/" in name or "\\" in name or name in (".", ".."):
            continue
        out.append(name)
    return out


def candidate_project_roots(artifact, authored_world=None):
    """Every plausible AUTHORING project root for ``artifact``, best first.

    ``project_root_for_world`` implements the ENGINE's rule (the parent of the
    nearest ``worlds`` ancestor, else parent-of-parent). That is right for
    resolving a controller at launch, and too narrow for FINDING one at
    collection: the workspace we hand an agent is deliberately near-empty, so
    it invents its own layout. A world written at the workspace ROOT -- the
    natural place, since the task's own assets are staged there -- has no
    ``worlds`` ancestor, so the engine rule lands on the workspace's PARENT
    and the sibling ``controllers/`` directory is never searched.

    Measured 2026-08-09, webots R1: the agent authored a world naming
    ``lidar_avoider`` and ``monitor``, and collection came back with NOTHING.
    A cell whose controllers are not collected grades a robot that cannot
    move -- the same defect that was zeroing the OmniSim arm this morning, so
    leaving it would have understated webots for an instrument reason and
    flattered us in the comparison.

    So: offer the engine's answer AND the world's own directory, for the
    artifact and for wherever it was authored. ``collect_controllers`` picks
    whichever root actually contains ``controllers/<name>``, so a wrong guess
    costs nothing and a missing one costs a measurement.
    """
    out, seen = [], set()
    for w in (artifact, authored_world):
        if w is None:
            continue
        w = Path(w)
        for root in (project_root_for_world(w), w.parent):
            if root is None:
                continue
            key = str(Path(root))
            if key not in seen:
                seen.add(key)
                out.append(Path(root))
    return tuple(out)


def collect_controllers(artifact, dest_project, *, search_roots):
    """Copy the controller dirs ``artifact`` references into ``dest_project``.

    LAYOUT, and why it resolves at phase B: the engine resolves
    ``controller "<name>"`` to ``<project_root>/controllers/<name>/``, and
    ``<project_root>`` is the parent of the nearest ``worlds`` ancestor of the
    world file (:func:`project_root_for_world`). Both grading paths put the
    world they launch under ``<dest_project>/worlds/...`` -- the omnisim arm by
    handing ``run_agentbench.py`` an ``--out`` inside ``<dest_project>/worlds``
    (its per-cell ``<cell>/scratch/`` dir then sits under that ``worlds``
    ancestor), the webots arm by staging into ``<dest_project>/worlds``
    directly -- so both resolve the SAME ``<dest_project>/controllers/<name>``
    written here. On the webots arm the launcher additionally copies
    ``<project_root>/controllers/.`` into its WSL workdir
    (``adapters/webots/launcher.build_run_script``), so the directory travels
    across the WSL boundary with the world.

    ``search_roots`` are candidate AUTHORING project roots, in order: where
    the world sits now (the workspace, or the junction-hygiene mirror, which
    preserves the repo-relative path and therefore the project shape) and
    where it was originally written. Only the agent's OWN project is searched:
    a controller that ships with the product resolves at phase B from the
    install exactly as it did while the agent worked, and copying it would
    shadow the real one with a stale snapshot.

    Returns ``(collected, not_in_project)``; never raises.
    """
    names = controller_names(artifact)
    if not names:
        return [], []
    roots, seen = [], set()
    for r in search_roots:
        if r is None:
            continue
        r = Path(r)
        if str(r) in seen:
            continue
        seen.add(str(r))
        roots.append(r)
    dest_dir = Path(dest_project) / "controllers"
    collected, missing = [], []
    for name in names:
        src = next((r / "controllers" / name for r in roots
                    if (r / "controllers" / name).is_dir()), None)
        if src is None:
            missing.append(name)
            continue
        dst = dest_dir / name
        try:
            shutil.copytree(src, dst, ignore=_CONTROLLER_IGNORE,
                            dirs_exist_ok=True)
            files = sorted(p.name for p in dst.rglob("*") if p.is_file())
        except OSError as exc:
            missing.append(name)
            collected.append({"name": name, "from": str(src), "to": None,
                              "error": repr(exc)})
            continue
        collected.append({
            "name": name, "from": str(src), "to": str(dst), "files": files,
            "grader_owned": name in GRADER_OWNED_CONTROLLERS})
    return collected, missing


def collect_driver(artifact, dest_model):
    """Copy the program that steps ``artifact`` beside the collected model.

    The MuJoCo counterpart of :func:`collect_controllers`, and it exists for
    the identical reason: an MJCF scene declares no behaviour and starts no
    process, so a model collected on its own is a scene that cannot move, and
    grading it measures OUR collection rather than the agent's work.

    ``adapters.mujoco.launcher.find_driver`` is the authority on WHICH file
    that is (via ``agents/external.find_driver``, so there is one rule, not
    two): the model's own stem first, then the only ``.py`` beside it, and a
    refusal -- never a guess -- when several are plausible. The copy keeps the
    collected model's stem so the same rule finds it again at grading time.

    Returns a record; ``found`` is ``None`` when the rule refused, with
    ``rule`` carrying the reason verbatim.
    """
    drv, rule = external_agent.find_driver(artifact)
    rec = {"found": (str(drv) if drv is not None else None), "to": None,
           "rule": rule}
    if drv is None:
        return rec
    dest = Path(dest_model).with_suffix(".py")
    try:
        shutil.copy2(drv, dest)
    except OSError as exc:                                    # noqa: BLE001
        rec["error"] = repr(exc)
        return rec
    rec["to"] = str(dest)
    return rec


def stage_controllers(art_dir, grade_project, report=None):
    """Mirror the collected ``controllers/`` into the grading project root.

    Two copies on purpose. The one under ``<run_dir>/artifact/`` is EVIDENCE:
    a reader opening the cell dir sees the deliverable whole (the R1 forensic
    pass had to reconstruct it from ``repo_artifacts/``). This one is the copy
    the ENGINE reads, at the project root both arms resolve. A controller
    directory is a script, so the duplication costs nothing.
    """
    src = Path(art_dir) / "controllers"
    if not src.is_dir():
        return []
    dst = Path(grade_project) / "controllers"
    staged = []
    try:
        shutil.copytree(src, dst, ignore=_CONTROLLER_IGNORE,
                        dirs_exist_ok=True)
        staged = sorted(p.name for p in dst.iterdir() if p.is_dir())
    except OSError as exc:                                    # noqa: BLE001
        if report is not None:
            report["controller_staging_error"] = repr(exc)
    if report is not None:
        report["staged_controllers"] = {"to": str(dst), "names": staged}
    return staged


#: Path components inside the graded project that belong to the DELIVERABLE
#: and must never be written by asset re-materialisation: ``controllers/`` is
#: the agent's collected code and ``worlds/`` is where each arm stages the
#: world it launches. A published asset landing in either would shadow the
#: thing being measured.
_ASSET_RESERVED_ROOTS = frozenset({"controllers", "worlds"})


def stage_task_assets(task_id, sim, grade_project, report=None):
    """Re-materialise the LANE'S OWN published task files in the graded project.

    THE DEFECT THIS FIXES (measured 2026-08-11, R4's first webots cell). The
    deliverable of an authoring task is the world plus the controller
    directories it names, and nothing else. The agent, meanwhile, works in a
    workspace where this lane has planted the task's published assets at the
    workspace ROOT -- ``benchmark_assets/scene.json`` for R3 and R4,
    ``benchmark_assets/obstacles.json`` for R1. A controller that opens one of
    those at run time by the only relative path that works from where it sits
    (``<controller dir>/../../benchmark_assets/...``) therefore works
    perfectly in the agent's workspace and dies at startup in the graded copy:
    the R4 cell's ``warehouse_controller`` raised ``FileNotFoundError`` in
    ``__init__``, exited 1, and its robot never moved -- base path length
    **0.00 m over 150 s** -- while the world it delivered was otherwise exact
    (all five obstacles, the table and the pad matched to 0.000 m). That is
    OUR packaging, scored as the agent's failure, and it applies to
    R1/R2/R3/R4 identically.

    THE FIX, AND WHY THE ARITHMETIC IS EXACT. The graded project root plays
    the part the workspace root played: both arms launch their world from
    under ``<grade_project>/worlds/`` (so the engine's project-root walk
    resolves ``<grade_project>``) and both read controllers from
    ``<grade_project>/controllers/<name>/``. The offset from a controller
    directory up to the project root is therefore the SAME two levels it was
    in the workspace, so planting the published files at ``<grade_project>/
    <same relative path>`` makes every relative reference that resolved for
    the agent resolve again -- with no path rewriting and no guessing.

    THE SAFETY RULE, stated plainly, because this is the half that could go
    wrong: **the bytes are copied from the frozen task tree in the repo
    (``tasks/<id>/initial{,_webots}/``), never from the agent's workspace and
    never from the collected artifact.** ``stage_workspaces.published_task_files``
    is the single authority on that set -- the same function that planted them
    in the workspace in the first place -- so what the graded project receives
    is, byte for byte, what the lane itself published and what the agent
    already had. An agent cannot smuggle anything through this channel: a
    tampered ``benchmark_assets/scene.json`` in its workspace is simply not
    read, and a file the agent authored has no path into the graded project
    except the deliverable (its world and its controller directories), which
    is exactly what we are trying to measure. Grade-time obstacle placement is
    unaffected for the same reason it always was: R1's drawn layout is written
    into the COLLECTED world after the session is over, and the published
    ``obstacles.json`` re-materialised here is the reference layout the agent
    was handed, not the one it is scored against.

    Two further guards. Anything whose first path component is ``controllers``
    or ``worlds`` is REFUSED and recorded (:data:`_ASSET_RESERVED_ROOTS`) --
    those namespaces belong to the deliverable and a published file must never
    shadow it. And nothing is copied into ``<cell>/artifact/``: that directory
    is evidence of what the AGENT delivered, and mixing our files into it
    would make the deliverable unreadable as a record.

    Returns the record it also writes into ``report["staged_task_assets"]``.
    """
    dest = Path(grade_project)
    rec = {"task": task_id, "sim": sim, "to": str(dest),
           "source": None, "files": [], "refused": [], "errors": []}
    try:
        files = staging.published_task_files(task_id, sim)
        rec["source"] = str(staging.task_initial_dir(task_id, sim))
    except Exception as exc:                                  # noqa: BLE001
        rec["errors"].append("could not enumerate published files: %r"
                             % (exc,))
        files = []
    for rel, src in files:
        first = rel.split("/", 1)[0]
        if first in _ASSET_RESERVED_ROOTS:
            rec["refused"].append(
                {"path": rel, "why": "'%s/' belongs to the deliverable"
                                     % first})
            continue
        dst = dest / rel
        try:
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            rec["files"].append(
                {"path": rel, "from": str(src),
                 "sha256": hashlib.sha256(
                     Path(src).read_bytes()).hexdigest()[:16]})
        except OSError as exc:                                # noqa: BLE001
            rec["errors"].append({"path": rel, "error": repr(exc)})
    if report is not None:
        report["staged_task_assets"] = rec
    return rec


def no_artifact_row(sim, task_id, rule):
    """The row for a cell whose session produced no deliverable at all.

    SPEC 3.2 progress level 0 (``no_artifact``) is a FAIL, and a FAIL is a
    row -- blocking instead would drop the cell out of the SCORED SET (its
    task's denominator, whatever that denominator is later called: at one run
    per (task, arm) it is a single cell, and losing it deletes the result
    rather than diluting it), and a cell that produced nothing is exactly a
    cell the agent lost.

    Nothing is measured here and nothing is invented: with no artifact there
    is no state to assert over, so ``assertions`` is empty, every metric the
    grader would have filled stays absent, and the outcome is FAIL by
    construction. ``measurements`` carries the reason, not a number.
    """
    return {
        "task": task_id, "sim": sim, "condition": CONDITION,
        "outcome": "FAIL", "progress": 0,
        "assertions": {}, "failed_assertion": "no_artifact",
        "measurements": {"no_artifact": True, "artifact_rule": rule},
        "agent": {}, "agent_artifacts": {}, "metrics": {},
        "notes": ("the session produced no gradeable deliverable; scored "
                  "FAIL at progress 0 (SPEC 3.2), never dropped"),
    }


def _has_suffix(path, suffixes):
    return str(path).lower().endswith(tuple(suffixes))


#: Sibling scratch written NEXT TO a world by our own tooling, never by an
#: agent. Every one of these is a copy of somebody else's world with a
#: supervisor spliced in, and collecting one grades the cell on a file it did
#: not write.
#:
#: MEASURED 2026-08-12, `20260812_round1_a1_omnisim_c1`: `artifact.found`
#: resolved to `repo_artifacts/.../.capture_newton_husky_swarm_drive.wbt` --
#: the shipped 8-Husky demo plus the capture service's supervisor -- because
#: it was the NEWEST candidate. The cell FAILED every assertion although its
#: ten robots demonstrably drove 5.14-30.27 m and it passed
#: `--fail-on-runaway`. `common.worldtext._HARNESS_SIBLING` knows `.harness_*`
#: and not `.capture_*`; rather than enumerate tools for ever, the rule here is
#: the CONVENTION they all follow -- a leading dot. No agent writes
#: `.something.wbt` as a deliverable.
SCRATCH_WORLD_PREFIXES = (".harness_", "..harness_", ".capture_",
                          ".omnisim_", worldtext.INJECTED_PREFIX)

#: Directory names that hold an agent's own VERIFICATION scratch rather than
#: its deliverable. Ranked BELOW a world written anywhere else, whatever the
#: mtimes say -- the first webots smoke cell wrote a labelled-broken CONTROL
#: copy last, and a newest-first rule collected it.
SCRATCH_DIR_NAMES = ("verify", "verification", "scratch", "tmp", "temp",
                     "_control", "control")


def is_scratch_world(name):
    """Tool-written sibling scratch, by name. Case-insensitive."""
    n = (name or "").lower()
    return n.startswith(".") or n.startswith(worldtext.INJECTED_PREFIX)


def _artifact_rank(path, root=None):
    """Rank key, best HIGHEST (reverse-sorted with mtime as the tiebreak).

    A world sitting in a verification directory can never outrank one that is
    not, however recently it was touched. Computed on the path RELATIVE to the
    search root: the whole lane lives under ``%TEMP%``, and an absolute-path
    rule would score every candidate as scratch (measured while writing this).
    """
    p = Path(path)
    if root is not None:
        try:
            p = p.relative_to(Path(root))
        except ValueError:
            pass
    parts = {q.lower() for q in p.parts[:-1]}
    return (0 if parts & set(SCRATCH_DIR_NAMES) else 1,)


def discover_artifact(workspace, staged_paths, start_ts, extra_roots=(),
                      suffixes=(".wbt",)):
    """The world (or model) the Claude Code session produced.

    ``suffixes`` is the arm's deliverable file format -- ``(".wbt",)`` for
    OmniSim and upstream Webots, ``(".xml",)`` for MuJoCo, and the default
    keeps every existing caller byte-identical. It is a per-SIM fact, not a
    per-task one, and ``agents/external.artifact_suffixes`` derives it from the
    same registry that names the collected file, so the discovery rule and the
    filename convention cannot drift apart. **Nothing else in this function is
    arm-specific**: the mtime gate, the staged-world precedence and the
    junction fallbacks all apply unchanged.

    Rule, in order:

    1. **The staged task world, when the session modified it.** The prompt is
       "fix it" -- the task's own world is the deliverable. This outranks
       "newest" because a session working in a full product workspace
       legitimately writes *verification* scratch worlds (measured in the
       first webots smoke cell: a ``verify/worlds/broken.wbt`` CONTROL copy
       was the newest ``.wbt`` and a newest-first rule collected the file the
       agent itself labelled broken).
    2. Otherwise the most recently modified non-injected ``.wbt`` under the
       workspace with mtime AFTER session start (instantiation preserves
       source mtimes via ``copy2``, so only session writes qualify), links
       never traversed.
    2b. Otherwise the same mtime-gated scan over ``extra_roots`` -- the
        cell's own preserved copies from the junction-artifact hygiene sweep
        (``repo_artifacts/``). The post-session sweep DELETES the repo copy
        of anything the session wrote through the ``projects/`` junction
        (so the next same-task cell cannot see it -- the r1/r3/r6
        contamination), and ``copy2`` preserved the session mtimes, so the
        gate still holds.
    2c. Otherwise the same mtime-gated scan THROUGH the workspace's junction
        dirs. The template junctions ``projects/`` (and siblings) into the
        real repo, and an agent asked to author a world naturally saves it
        under ``projects/.../worlds/`` -- through the junction, so the
        link-refusing walk above is blind to it (measured 2026-08-01:
        A1:omnisim r0-r2 each wrote a valid ``husky_random*.wbt`` there and
        all three cells blocked with "no .wbt found"). Junction targets are
        SHARED (the real repo), so this pass is a fallback only, is
        mtime-gated the same way, and its rule note names the risk (a
        concurrent writer to the repo could in principle race the mtime
        window).
    3. Otherwise the staged task world unchanged -- the grader then fails it
       on its own terms.

    Returns ``(path_or_None, rule_note)``.
    """
    ws = Path(workspace)
    suffixes = tuple(str(s).lower() for s in suffixes) or (".wbt",)
    kinds = "/".join(suffixes)
    skipped = []            # tool scratch we refused, reported on the rule

    def _rank_sort(cands, root=None):
        """Best FIRST. Rank beats mtime; mtime breaks ties within a rank."""
        cands.sort(key=lambda t: (_artifact_rank(t[1], root), t[0]),
                   reverse=True)
        return cands

    def _note(base):
        if not skipped:
            return base
        return ("%s; skipped %d tool-written sibling/scratch file(s) (%s)"
                % (base, len(skipped),
                   ", ".join(sorted({Path(p).name for p in skipped})[:4])))

    for s in staged_paths:
        p = Path(s)
        if _has_suffix(s, suffixes) and p.is_file() \
                and p.stat().st_mtime >= start_ts:
            return p, ("staged task world modified during the session "
                       "(the deliverable outranks newer verification "
                       "scratch)")

    def scan(follow_links):
        cands = []
        seen = set()
        # NB: os.walk descends Windows JUNCTIONS even with
        # followlinks=False (they are not islink()); the non-follow pass
        # prunes them explicitly below, and followlinks=True additionally
        # covers true directory symlinks.
        for dirpath, dirnames, filenames in os.walk(ws,
                                                    followlinks=follow_links):
            if follow_links:
                # never loop (a junction cycle) and never descend into the
                # binary runtime tree (huge, holds no authored worlds)
                dirnames[:] = [d for d in dirnames if d != "msys64"
                               and d != ".git"]
                try:
                    key = os.path.realpath(dirpath)
                except OSError:
                    key = dirpath
                if key in seen:
                    dirnames[:] = []
                    continue
                seen.add(key)
            else:
                dirnames[:] = [d for d in dirnames
                               if not staging.is_link(Path(dirpath) / d)]
            for f in filenames:
                if not _has_suffix(f, suffixes):
                    continue
                p = Path(dirpath) / f
                if is_scratch_world(f):
                    skipped.append(str(p))
                    continue
                try:
                    mt = p.stat().st_mtime
                except OSError:
                    continue
                if mt >= start_ts:
                    cands.append((mt, p))
        return cands

    cands = scan(follow_links=False)
    if cands:
        _rank_sort(cands, ws)
        return cands[0][1], _note("best-ranked %s modified after session "
                                  "start (%d candidate(s); a world in a "
                                  "verification dir never outranks one that "
                                  "is not)" % (kinds, len(cands)))
    cands = []
    for root in extra_roots:
        root = Path(root)
        if not root.is_dir():
            continue
        for p in sorted(q for sfx in suffixes for q in root.rglob("*" + sfx)):
            if not p.is_file():
                continue
            if is_scratch_world(p.name):
                skipped.append(str(p))
                continue
            try:
                mt = p.stat().st_mtime
            except OSError:
                continue
            if mt >= start_ts:
                cands.append((mt, p))
    if cands:
        _rank_sort(cands, None)
        return cands[0][1], _note(
            "best-ranked %s modified after session start, PRESERVED by the "
            "junction-artifact hygiene sweep (%d candidate(s)) -- the "
            "session wrote its deliverable through the projects/ junction "
            "into the real repo; the sweep copied it (mtimes kept) into the "
            "cell dir and deleted the repo copy" % (kinds, len(cands)))
    cands = scan(follow_links=True)
    if cands:
        _rank_sort(cands, ws)
        return cands[0][1], _note(
            "best-ranked %s modified after session start, found THROUGH a "
            "workspace junction (%d candidate(s)) -- the session wrote its "
            "deliverable into a junctioned dir whose target is shared with "
            "the real repo; the file was collected mtime-gated, but a "
            "concurrent repo writer could race this window"
            % (kinds, len(cands)))
    for s in staged_paths:
        if _has_suffix(s, suffixes) and Path(s).is_file():
            return Path(s), ("no %s modified during the session; falling "
                             "back to the staged task world unchanged" % kinds)
    return None, "no %s found in the workspace at all" % kinds


def merge_cc_metrics(row, cc):
    """The grader's row + the Claude Code cell's own measurements.

    The verdict, assertions and measurements are the grader's alone and are
    never touched. Only the agent-side metrics and provenance are filled in;
    anything ``cc`` does not carry stays exactly as the grader wrote it
    (unmeasured is never invented).
    """
    row = copy.deepcopy(row)
    m = row.setdefault("metrics", {})
    # These are AGENT metrics, and the Claude Code cell is their only honest
    # source. They are overwritten even when the CC value is None: leaving
    # the grading stub's own numbers (the external adapter's one copy call)
    # in an agent column would be a fabricated measurement. Unmeasured is
    # null, with the reason in agent_artifacts.claude_code.
    for row_key, cc_key in (("turns", "num_turns"),
                            ("tool_calls", "tool_calls"),
                            ("usd", "total_cost_usd"),
                            ("t_agent_s", "t_agent_s"),
                            ("tokens_in", "tokens_in"),
                            ("tokens_out", "tokens_out"),
                            ("tokens_cache_read", "tokens_cache_read"),
                            ("tokens_cache_write", "tokens_cache_write")):
        m[row_key] = cc.get(cc_key)
    agent = row.setdefault("agent", {})
    if cc.get("model"):
        agent["model"] = cc["model"]
    agent["kind"] = "claude_code"
    art = row.setdefault("agent_artifacts", {})
    art["condition"] = CONDITION
    art["metrics_source"] = METRICS_SOURCE
    art["claude_code"] = {k: cc.get(k) for k in (
        "claude_code_version", "model", "model_usage", "permission_mode",
        "session_id", "transcript", "tool_calls_reason", "cell_wall_s",
        "num_turns", "duration_ms", "duration_api_ms", "is_error", "subtype",
        "cli_command")}
    art["claude_code"]["model_attribution"] = model_attribution(
        cc.get("model_usage"), DEFAULT_MODEL)
    return row


#: A cell is "on the benchmark model" when that model did essentially all the
#: generating. Claude Code bills small auxiliary calls to a cheaper model, so
#: demanding 100 % would fail every honest cell; this is the share below which
#: an off-model contribution cannot plausibly have done the task's thinking.
OFF_MODEL_TOKEN_TOLERANCE = 0.02          # 2 % of output tokens


def model_attribution(model_usage, expected):
    """Did the PINNED model actually do the work? Evidence, not assertion.

    Returns a dict carrying the output-token share of ``expected``, every
    other model that billed anything, and an ``ok`` flag. ``ok`` is None when
    there is nothing to judge (no usage reported) -- never True by default,
    because "we could not tell" and "we checked and it was fine" are different
    claims and only one of them supports a published number.
    """
    if not isinstance(model_usage, dict) or not model_usage:
        return {"expected": expected, "ok": None,
                "reason": "no modelUsage reported by the CLI"}
    totals = {k: (v or 0) for k, v in model_usage.items()}
    total = sum(totals.values())
    mine = totals.get(expected, 0)
    others = {k: v for k, v in totals.items() if k != expected and v}
    share = (mine / total) if total else None
    return {
        "expected": expected,
        "expected_output_tokens": mine,
        "other_models": others or None,
        "expected_share": (round(share, 6) if share is not None else None),
        "tolerance": OFF_MODEL_TOKEN_TOLERANCE,
        "ok": (None if share is None
               else bool(share >= 1.0 - OFF_MODEL_TOKEN_TOLERANCE)),
    }


# --- liveness: did a session actually RUN? -------------------------------------
#
# WHY A ROW NEEDS THIS. `r1_3arm_20260810` published
# `FAIL / no_artifact / progress 0` for a webots cell whose session was killed
# 156.8 s into a 1800 s task budget with a null session id. In the grid that
# row is INDISTINGUISHABLE from "the agent worked for half an hour and built
# nothing" -- and it is a completely different claim. One is a measurement of
# the agent; the other is a measurement of our scheduler.
#
# So: a row is only published when a session demonstrably ran, and the
# criteria are recorded on the row so a reader can re-check them rather than
# trust the verdict.

#: A session must be granted at least this fraction of its task's declared
#: budget. SIZED FROM THE FAILURE: the webots cell was granted 156.7 s of
#: 1800 s -- 8.7% -- because the cell's wall deadline was armed before it
#: queued behind a same-task lock, and the queue ate 42 of its 45 minutes.
#: Below half the declared budget the cell is not measuring the agent at the
#: protocol's budget, and comparing it to a full-budget cell on another arm is
#: not a comparison. Enforced BEFORE the session starts (nothing is spent on a
#: doomed run) and re-asserted after.
MIN_SESSION_BUDGET_FRACTION = 0.5

#: Below this, with no tool call, a session did not attempt the task. The
#: authenticated 1-turn preflight probe measures 2.2-4.6 s on this machine and
#: the shortest real R1 session in the corpus ran 908 s, so 30 s sits an order
#: of magnitude above "answered instantly" and two below "tried".
MIN_LIVE_WALL_S = 30.0


#: Checks recorded but never allowed to veto publication on their own. Each
#: is a METADATA gap or a contamination fact, not evidence that the agent
#: never worked -- and blocking on one would drop a real agent result out of
#: the denominator, which is the mirror-image mistake.
LIVENESS_ADVISORY_CHECKS = ("attributable", "transcript_present",
                            "model_pin_applied")


def assess_liveness(*, meta, stream, transcript, session_budget_s,
                    task_budget_s, pinned_model, result_json=None):
    """Did a session run, and did it run at the protocol's budget?

    Returns ``{ok, verdict, checks, failed, vetoing}``. ``ok=False`` means the
    row would describe the INSTRUMENT rather than the agent.

    THE ASYMMETRY IS THE POINT. Two checks veto, and both are OUR failures:
    a session that never started, and a session we starved of budget. An
    agent that ran for its full budget and built nothing FAILS -- honestly,
    with a row, in the denominator. Anything else and a simulator would score
    better the more often our harness broke on it.
    """
    stream = stream or {}
    result_json = result_json or None
    wall = meta.get("wall_s")
    turns = stream.get("assistant_turns") or 0
    tools = stream.get("tool_calls") or 0
    sid = stream.get("session_id") or (result_json or {}).get("session_id")
    # Every independent trace that a session existed. A veto needs ALL of
    # them to be absent -- "plainly never ran" is a conjunction, not a
    # disjunction, or an unlucky metadata gap starts eating real results.
    ran = {
        "result_event": bool(result_json),
        "stream_events": stream.get("events") or 0,
        "assistant_turns": turns,
        "tool_calls": tools,
        "transcript_on_disk": transcript is not None,
        "occupied_full_budget": bool(meta.get("timed_out")),
        "wall_s": wall,
        "wall_over_min": bool((wall or 0) >= MIN_LIVE_WALL_S),
    }
    started = bool(ran["result_event"] or ran["stream_events"]
                   or ran["transcript_on_disk"] or ran["occupied_full_budget"]
                   or ran["tool_calls"] or ran["wall_over_min"])
    checks = {
        # VETO 1. Did anything at all happen? A session killed at its budget
        # counts: it held the machine for the whole budget, and SPEC 2.4 says
        # budget exhaustion is the AGENT's result.
        "session_started": {
            "ok": started, "value": ran,
            "why": "a session leaves at least one trace -- a result event, a "
                   "stream event, a transcript, a tool call, %.0f s of wall "
                   "clock, or a kill at its own budget. None of them means "
                   "the CLI exited before Claude ever ran."
                   % MIN_LIVE_WALL_S},
        # VETO 2. THE ONE THAT WOULD HAVE CAUGHT THE WEBOTS CELL.
        "budget_honoured": {
            "ok": (session_budget_s is None or not task_budget_s
                   or session_budget_s >= MIN_SESSION_BUDGET_FRACTION
                   * task_budget_s),
            "value": {"session_budget_s": session_budget_s,
                      "task_budget_s": task_budget_s,
                      "fraction": (None if not task_budget_s else
                                   round((session_budget_s or 0.0)
                                         / task_budget_s, 4)),
                      "floor_fraction": MIN_SESSION_BUDGET_FRACTION},
            "why": "a session granted less than half its task's declared "
                   "budget was starved by the instrument; its outcome is not "
                   "comparable with a full-budget cell on another arm"},
        # ADVISORY. Recorded so a reader can see the gap; never a veto.
        "attributable": {
            "ok": bool(sid), "value": sid,
            "why": "the session id ties this row to a transcript. Null here "
                   "was the unread tell in r1_3arm_20260810; with the stream "
                   "it arrives in the first event, so a null now means the "
                   "CLI never got as far as system/init"},
        "transcript_present": {
            "ok": transcript is not None, "value": str(transcript or ""),
            "why": "with the transcript on disk the row's turn and tool "
                   "counts can be re-derived by anyone; without it they rest "
                   "on the stream alone"},
        "model_pin_applied": {
            "ok": (not pinned_model or not stream.get("model")
                   or str(stream.get("model")).startswith(str(pinned_model))),
            "value": {"pinned": pinned_model,
                      "stream_model": stream.get("model")},
            "why": "the stream's init/assistant events name the model that "
                   "actually ran; a mismatch means --model did not reach the "
                   "CLI (r1_3arm_20260810 ran 82/82 turns on the CLI default "
                   "while the report claimed the pin)"},
    }
    failed = [k for k, v in checks.items() if not v["ok"]]
    vetoing = [k for k in failed if k not in LIVENESS_ADVISORY_CHECKS]
    if not vetoing:
        verdict = ("session ran" if not failed
                   else "session ran (advisory gaps: %s)" % ", ".join(failed))
    elif "budget_honoured" in vetoing:
        verdict = ("session STARVED by the instrument: granted %s s of the "
                   "task's %s s budget" % (session_budget_s, task_budget_s))
    else:
        verdict = "session NEVER RAN (no trace of a started session)"
    return {"ok": not vetoing, "verdict": verdict, "checks": checks,
            "failed": failed, "vetoing": vetoing,
            "advisory": list(LIVENESS_ADVISORY_CHECKS),
            "min_session_budget_fraction": MIN_SESSION_BUDGET_FRACTION,
            "min_live_wall_s": MIN_LIVE_WALL_S}


def cc_metrics_from_result(result_json, *, tool_calls, tool_calls_reason,
                           version, permission_mode, transcript,
                           cell_wall_s, cli_command):
    """Flatten the ``claude -p --output-format json`` result + transcript
    count into the dict :func:`merge_cc_metrics` consumes. Missing fields
    stay None."""
    r = result_json or {}
    usage = r.get("usage") or {}

    def _num(v):
        return v if isinstance(v, (int, float)) else None

    model = None
    mu = r.get("modelUsage")
    model_usage = None
    if isinstance(mu, dict) and mu:
        # the model that did the work = most output tokens
        model = max(mu, key=lambda k: (mu[k] or {}).get("outputTokens", 0))
        # ...but record EVERY model the session billed, not just the winner.
        # Claude Code makes small auxiliary calls on a cheaper model (measured:
        # 15 Haiku output tokens beside 11,175 Opus ones), so a cell is never
        # literally single-model. Reporting only the argmax would leave a
        # second model id sitting in the logs for a sceptic to find later and
        # read as a contaminated grid. Publishing the split makes the claim
        # "the work ran on <pinned>" checkable instead of asserted.
        model_usage = {k: (v or {}).get("outputTokens")
                       for k, v in sorted(mu.items())}
    return {
        "model_usage": model_usage,
        "num_turns": _num(r.get("num_turns")),
        "tool_calls": tool_calls,
        "tool_calls_reason": tool_calls_reason,
        "total_cost_usd": _num(r.get("total_cost_usd")),
        "t_agent_s": (round(r["duration_ms"] / 1000.0, 3)
                      if _num(r.get("duration_ms")) is not None else None),
        "duration_ms": _num(r.get("duration_ms")),
        "duration_api_ms": _num(r.get("duration_api_ms")),
        "tokens_in": _num(usage.get("input_tokens")),
        "tokens_out": _num(usage.get("output_tokens")),
        "tokens_cache_read": _num(usage.get("cache_read_input_tokens")),
        "tokens_cache_write": _num(usage.get("cache_creation_input_tokens")),
        "model": model,
        "session_id": r.get("session_id"),
        "is_error": r.get("is_error"),
        "subtype": r.get("subtype"),
        "result_text": r.get("result"),
        "claude_code_version": version,
        "permission_mode": permission_mode,
        "transcript": str(transcript) if transcript else None,
        "cell_wall_s": cell_wall_s,
        "cli_command": cli_command,
    }


# --- the Claude Code child -----------------------------------------------------


#: Where npm's Windows shim forwards to, relative to the shim's own directory.
_NPM_CLI_REL = Path("node_modules") / "@anthropic-ai" / "claude-code" / "bin"


def _resolve_cli_launcher(exe):
    """The REAL Claude Code executable behind a launcher, + how it resolved.

    THE BUG THIS EXISTS FOR (measured 2026-08-11, cost: a whole 3-arm
    campaign). ``shutil.which("claude")`` on Windows returns npm's
    ``claude.CMD`` shim, not an executable. ``subprocess.Popen`` hands a
    ``.cmd`` to ``CreateProcess``, which runs it **through cmd.exe** -- and
    cmd.exe TERMINATES THE COMMAND LINE AT THE FIRST NEWLINE of any argument.

    Every task prompt in this suite is multi-line. So the child received the
    prompt's FIRST LINE ONLY, and every flag positioned after it was silently
    discarded: ``--output-format json`` (hence prose on stdout, hence "no JSON
    on stdout" and two blocked cells), ``--model claude-opus-5`` (hence the
    surviving transcript showing 82/82 assistant messages on the CLI DEFAULT
    ``claude-opus-4-8`` -- the pinned model never ran) and
    ``--dangerously-skip-permissions``. Exit code 0 throughout.

    It survived because the PREFLIGHT probe's prompt was ``"Say OK"`` -- one
    line, nothing to truncate -- so the instrument's own self-check passed on
    the one input shape that cannot fail. ``preflight()`` now probes with a
    multi-line prompt for exactly this reason.

    Reproduced both ways on this machine: same argv, same env, same cwd --
    ``claude.CMD`` -> ``PONG`` (text, default model), the resolved
    ``claude.exe`` -> the JSON envelope with ``modelUsage`` naming the pin.
    """
    p = Path(exe)
    if p.suffix.lower() not in (".cmd", ".bat"):
        return p, "direct (%s)" % (p.suffix or "no suffix")
    cand = p.parent / _NPM_CLI_REL / ("claude" + (".exe" if os.name == "nt"
                                                  else ""))
    if cand.is_file():
        return cand, "npm %s shim -> %s" % (p.name, cand)
    # Fall back to the path the shim itself names (npm's template quotes it).
    try:
        for tok in p.read_text(encoding="utf-8", errors="replace").split('"'):
            if tok.lower().endswith(".exe"):
                real = Path(tok.replace("%dp0%", str(p.parent)))
                if real.is_file():
                    return real, "parsed out of %s" % p.name
    except OSError:
        pass
    return p, ("UNRESOLVED %s shim: multi-line arguments will be TRUNCATED by "
               "cmd.exe" % p.suffix)


def _claude_exe():
    exe = shutil.which("claude")
    if exe is None:
        raise FileNotFoundError("claude CLI not found on PATH")
    real, _how = _resolve_cli_launcher(exe)
    return str(real)


def claude_launcher_report():
    """``{which, resolved, how, safe_for_multiline}`` -- recorded on every
    cell so a future reader can tell WHICH binary produced the row."""
    which = shutil.which("claude")
    if which is None:
        return {"which": None, "resolved": None, "how": "not found",
                "safe_for_multiline": False}
    real, how = _resolve_cli_launcher(which)
    return {"which": which, "resolved": str(real), "how": how,
            "safe_for_multiline":
                Path(real).suffix.lower() not in (".cmd", ".bat")}


def _kill_tree(pid):
    """Kill a process AND its descendants. Returns what it did, for the row.

    Killing only the direct child is not enough here: the session spawns
    engines, and on the webots arm a whole ``wsl.exe`` -> webots subtree. Those
    survive their parent, keep running (and keep opening windows), and -- the
    reason this function exists -- keep the inherited stdio handles open.
    """
    if os.name == "nt":
        r = subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"],
                           capture_output=True, text=True, timeout=60)
        return {"how": "taskkill /T /F", "rc": r.returncode,
                "detail": (r.stdout or r.stderr or "").strip()[:200]}
    try:
        os.killpg(os.getpgid(pid), signal.SIGKILL)
        return {"how": "killpg SIGKILL", "rc": 0, "detail": ""}
    except Exception as exc:                                  # noqa: BLE001
        return {"how": "killpg SIGKILL", "rc": -1, "detail": repr(exc)}


def _run_claude(args, *, cwd, env, timeout_s, stdout_path=None,
                stderr_path=None):
    """Run one Claude Code session under a HARD wall-clock cap.

    Deliberately NOT ``subprocess.run(capture_output=True, timeout=...)``.
    That reaps a timed-out child with ``communicate()`` and **no timeout of
    its own** on Windows, so any grandchild still holding the inherited pipe
    blocks the reap for ever -- the TimeoutExpired never propagates and the
    cell outlives its budget silently.

    Measured 2026-08-09: a webots A1 cell reached **28.5 minutes against a
    15-minute cap** and was still spawning engines, because the agent had
    launched ``wsl.exe`` -> webots and those grandchildren held the pipes.
    The cost ceiling was not a ceiling at all.

    So: stdio goes to FILES (nothing to block on), the wait is ours, and on
    expiry the whole process TREE dies before we re-raise.
    """
    exe = _claude_exe()
    # THE FENCE over the campaign-killing defect in _resolve_cli_launcher: a
    # .cmd/.bat launcher goes through cmd.exe, which truncates the command
    # line at the first newline of any argument -- silently, with rc=0. Refuse
    # loudly rather than run a session whose prompt and flags were eaten.
    if (Path(exe).suffix.lower() in (".cmd", ".bat")
            and any(("\n" in a or "\r" in a) for a in args)):
        raise RuntimeError(
            "refusing to launch Claude Code through %r: it is a cmd/bat "
            "launcher and one of the arguments is multi-line. cmd.exe ends "
            "the command line at the first newline, so the prompt would be "
            "truncated and every flag after it (--output-format, --model, "
            "--dangerously-skip-permissions) silently dropped -- with exit "
            "code 0. This is the r1_3arm_20260810 failure; see "
            "_resolve_cli_launcher." % exe)
    cmd = [exe] + list(args)
    out_p = Path(stdout_path) if stdout_path else None
    err_p = Path(stderr_path) if stderr_path else None
    tmp_out = out_p or (Path(cwd) / "_cc_stdout.tmp")
    tmp_err = err_p or (Path(cwd) / "_cc_stderr.tmp")
    kwargs = {}
    if os.name != "nt":
        kwargs["start_new_session"] = True        # own group, so killpg works

    with open(tmp_out, "w", encoding="utf-8") as fo, \
            open(tmp_err, "w", encoding="utf-8") as fe:
        # stdin is CLOSED, not inherited: with a TTY-less inherited stdin the
        # CLI waits 3 s for piped input on every launch and warns about it,
        # and an inherited console handle is one more thing a grandchild can
        # hold open past the kill.
        proc = subprocess.Popen(cmd, cwd=str(cwd), env=env,
                                stdout=fo, stderr=fe,
                                stdin=subprocess.DEVNULL, **kwargs)
        try:
            rc = proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            killed = _kill_tree(proc.pid)
            try:
                proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                pass
            exc = subprocess.TimeoutExpired(cmd, timeout_s)
            exc.killed_tree = killed              # recorded into the row
            raise exc

    def _read(p):
        try:
            return p.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return ""

    return subprocess.CompletedProcess(cmd, rc, _read(tmp_out),
                                       _read(tmp_err))


#: The preflight probe prompt. MULTI-LINE ON PURPOSE, and that is the whole
#: point of it. The old probe was the single line ``"Say OK"``, and a
#: single-line prompt is the one input shape the cmd.exe truncation defect
#: cannot corrupt -- so the instrument's own self-check passed green through
#: an entire campaign in which every real cell had its prompt cut to one line
#: and its flags discarded. A preflight that cannot exercise the failure mode
#: of the thing it precedes is decoration. This one round-trips a newline.
PREFLIGHT_PROMPT = (
    "Reply with exactly: OK\n"
    "\n"
    "This probe is deliberately multi-line: it exercises the same argv path a\n"
    "real cell uses, so a launcher that truncates at a newline fails HERE.\n")


def preflight(run_dir, env):
    """``claude --version`` + an authenticated MULTI-LINE call. Returns
    ``{"version", "default_model", "ok", "detail", "launcher",
    "argv_integrity"}``."""
    out = {"version": None, "default_model": None, "ok": False, "detail": "",
           "launcher": claude_launcher_report(), "argv_integrity": None}
    try:
        v = subprocess.run([_claude_exe(), "--version"], capture_output=True,
                           text=True, timeout=60)
        out["version"] = (v.stdout or "").strip() or None
    except (OSError, subprocess.TimeoutExpired) as exc:
        out["detail"] = "claude --version failed: %r" % (exc,)
        return out
    neutral = Path(run_dir) / "preflight_cwd"
    neutral.mkdir(parents=True, exist_ok=True)
    # The probe runs the SESSION's exact output flags, so preflight exercises
    # the path it is vouching for rather than a simpler one.
    stream_path = Path(run_dir) / "preflight_stream.jsonl"
    try:
        proc = _run_claude(["-p", PREFLIGHT_PROMPT] + SESSION_OUTPUT_ARGS,
                           cwd=neutral, env=env, timeout_s=300,
                           stdout_path=stream_path,
                           stderr_path=Path(run_dir) / "preflight_stderr.log")
    except subprocess.TimeoutExpired:
        out["detail"] = "preflight call timed out"
        return out
    except RuntimeError as exc:               # the multi-line launcher fence
        out["argv_integrity"] = "refused"
        out["detail"] = str(exc)
        return out
    stream = evidence.read_stream(stream_path)
    doc = stream.get("result")
    if doc is None:
        # PROSE (or nothing) here now means precisely one thing, and it is not
        # "the CLI is broken": the flags after the multi-line prompt did not
        # reach it.
        out["argv_integrity"] = "flags_lost_after_multiline_prompt"
        out["detail"] = (
            "preflight produced no result event (rc=%s, %d stream events) "
            "from a MULTI-LINE prompt, so the output-format flags did not "
            "reach the CLI: the launcher truncated argv at the first newline. "
            "Launcher: %s. stdout head: %r; stderr: %s"
            % (proc.returncode, stream.get("events", 0), out["launcher"],
               (proc.stdout or "")[:200], (proc.stderr or "")[-300:]))
        return out
    out["argv_integrity"] = "ok"
    # THE CLI'S OWN CHOICE, from the init event -- not the argmax over
    # ``modelUsage`` output tokens, which is a heuristic that answers a
    # different question and gets it wrong on a short probe. Measured
    # 2026-08-11 on the verification cell: the probe's 2-token "OK" from Opus
    # lost the argmax to 20 auxiliary Haiku tokens and the report recorded
    # ``cli_default_model: claude-haiku-4-5``, which is not the CLI default at
    # all. `system/init.model` is what the CLI selected, stated by the CLI.
    out["default_model"] = stream.get("model") or None
    out["model_usage"] = {k: (v or {}).get("outputTokens") for k, v in
                          sorted((doc.get("modelUsage") or {}).items())}
    out["ok"] = not doc.get("is_error", False)
    out["detail"] = doc.get("result", "")
    # Keep the historical filename working for anything that reads it.
    (Path(run_dir) / "preflight_stdout.json").write_text(
        json.dumps(doc, indent=2), encoding="utf-8")
    return out


#: The session's stdout format. ``stream-json`` (which the CLI accepts only
#: WITH ``--verbose``) emits one NDJSON event per turn and per tool call as it
#: happens, instead of a single blob at the very end.
#:
#: Two reasons, and the second is the load-bearing one:
#:
#: * LIVE VISIBILITY. A cell used to be a black box for up to 45 minutes.
#:   With the stream on disk, ``--status`` answers "what is it doing right
#:   now" from a file, without touching the session.
#: * PARTIAL RESULTS SURVIVE. The single blob is all-or-nothing: a session
#:   that is killed, crashes, or has its flags eaten prints nothing parseable
#:   and the cell loses EVERYTHING -- session id, turns, tool calls, the lot.
#:   That is how ``r1_3arm_20260810`` recorded ``session_id: null`` for a
#:   session whose transcript was sitting in ``~/.claude/projects`` the whole
#:   time. Every event that arrived before the end is now banked.
SESSION_OUTPUT_ARGS = ["--output-format", "stream-json", "--verbose"]


def run_claude_cell(prompt, workspace, env, run_dir, *, model,
                    timeout_s=DEFAULT_TIMEOUT_S, settings_path=None):
    """The task session. Returns ``(result_json_or_None, meta)``.

    ``meta["stream"]`` carries what the stream banked whether or not the
    session reached its ``result`` event, so a killed session still yields a
    session id, a turn count and a tool-call log.
    """
    run_dir = Path(run_dir)
    stream_path = run_dir / "cc_stream.jsonl"
    base = ["-p", prompt] + SESSION_OUTPUT_ARGS
    if model:
        base += ["--model", model]
    if settings_path:
        # The containment guard (a PreToolUse hook). It rides on BOTH attempts
        # -- a cell that fell back to `acceptEdits` must be contained exactly
        # as much as one that did not.
        base += ["--settings", str(settings_path)]
    attempts = [
        ("dangerously-skip-permissions",
         base + ["--dangerously-skip-permissions"]),
        ("acceptEdits+allowedTools",
         base + ["--permission-mode", "acceptEdits",
                 "--allowedTools", " ".join(FALLBACK_ALLOWED_TOOLS)]),
    ]
    meta = {"permission_mode": None, "cli_command": None, "rc": None,
            "timed_out": False, "wall_s": None, "launch_error": None,
            # THE CHILD'S OWN stderr, verbatim and separate from the prose we
            # write into `launch_error`. `deferral_reason` reads THIS. Mixing
            # the two is defect 4: our summary quotes `rc=4294967295`, whose
            # digits contain "429", and a killed cell was deferred as
            # rate-limited (see concurrency._HTTP_429).
            "stderr_tail": "",
            "stream": None, "stream_path": str(stream_path),
            "launcher": claude_launcher_report()}

    def _bank(mode, args, *, rc=None, timed_out=False, wall=None,
              killed=None):
        """Read the stream WHATEVER happened, and mirror the result event to
        cc_stdout.json for anything still reading the old filename."""
        stream = evidence.read_stream(stream_path)
        try:
            err_tail = (run_dir / "cc_stderr.log").read_text(
                encoding="utf-8", errors="replace")[-4000:]
        except OSError:
            err_tail = ""
        meta.update(permission_mode=mode, rc=rc, timed_out=timed_out,
                    wall_s=wall, killed_tree=killed, stderr_tail=err_tail,
                    stream=evidence.stream_summary_for_report(stream),
                    cli_command="claude " + " ".join(
                        args[:1] + ["<prompt>"] + args[2:]))
        if stream.get("result") is not None:
            (run_dir / "cc_stdout.json").write_text(
                json.dumps(stream["result"], indent=2), encoding="utf-8")
        return stream

    for mode, args in attempts:
        t0 = time.monotonic()
        try:
            proc = _run_claude(
                args, cwd=workspace, env=env, timeout_s=timeout_s,
                stdout_path=stream_path,
                stderr_path=run_dir / "cc_stderr.log")
        except subprocess.TimeoutExpired as exc:
            _bank(mode, args, timed_out=True,
                  wall=round(time.monotonic() - t0, 1),
                  killed=getattr(exc, "killed_tree", None))
            return None, meta
        except RuntimeError as exc:            # the multi-line launcher fence
            meta.update(permission_mode=mode, wall_s=round(
                time.monotonic() - t0, 1), launch_error=str(exc),
                cli_command="claude " + " ".join(
                    args[:1] + ["<prompt>"] + args[2:]))
            return None, meta
        wall = round(time.monotonic() - t0, 1)
        stream = _bank(mode, args, rc=proc.returncode, wall=wall)
        meta["stderr_tail"] = (proc.stderr or "")[-4000:]
        if stream.get("result") is not None:
            return stream["result"], meta
        stderr = (proc.stderr or "")
        # Fall through to the fallback ONLY when the launch itself was
        # refused before Claude started (the documented case: the
        # permissions flag rejected in this environment).
        if ("--dangerously-skip-permissions" in stderr
                and mode == "dangerously-skip-permissions"
                and proc.returncode != 0 and wall < 60):
            meta["launch_error"] = stderr.strip()[-400:]
            continue
        meta["launch_error"] = (
            "no result event in the session stream (rc=%s, %d events, %d "
            "assistant turns, %d tool calls): %s"
            % (proc.returncode, stream.get("events", 0),
               stream.get("assistant_turns", 0), stream.get("tool_calls", 0),
               stderr.strip()[-400:]))
        return None, meta
    return None, meta


def deferral_reason(result, meta):
    """The rate/usage-limit marker for THIS attempt, or None.

    THE ONE RULE: the evidence is the CHILD's -- its headless JSON result and
    its own stderr -- never the prose ``run_claude_cell`` writes into
    ``launch_error``. That prose quotes the child's exit code, and on Windows a
    KILLED process exits ``4294967295``, whose digits contain ``429``.

    MEASURED 2026-08-12, `20260812_round1_a1_omnisim_c2`: an operator killed
    the Claude child at plateau (the sanctioned early stop). The cell recorded
    `marker: "429"`, deferred the attempt, tore down the workspace it had just
    preserved, and slept 900 s **holding the task lock** -- no artifact, no
    grade, no repo sweep, no row, and the whole early-stop method dead with it.
    `concurrency._HTTP_429` fixed the substring; this function fixed the
    channel, and either alone would have left the other latent.
    """
    meta = meta or {}
    return concurrency.rate_limit_reason(result, meta.get("stderr_tail") or "")


# --- grading ---------------------------------------------------------------------


def grade_omnisim(task_id, artifact, grade_dir, notes, *, answer_path=None,
                  layout_dir=None):
    """Through the shipped runner: ``--agent external`` + the env contract
    from ``agents/external.py``. Returns the grader's row (dict).

    ``artifact`` is the task's deliverable -- a world for the world tasks,
    the answer text file for the ANSWER_TASKS (external.py reads it into the
    grader's answer channel and stages no world, so ground truth comes from
    the pristine scratch the runner materialises). ``answer_path`` threads a
    world task's final-message file (B2's committed proof) through
    ``AGENTBENCH_EXTERNAL_ANSWER``.

    ``layout_dir`` is where R1's placement step declared the drawn obstacle
    layout. This arm grades in a SUBPROCESS that builds its own per-cell run
    directory -- deleting any existing one first -- so the sidecar cannot be
    pre-placed where the grader's own search would reach it; the directory is
    named in the child's environment instead. It is set AFTER the
    ``AGENTBENCH_*`` scrub below, whose job is to stop a STALE OUTER value
    reaching the child, not to stop this one.
    """
    Path(grade_dir).mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    for k in list(env):
        if k.startswith("AGENTBENCH_"):
            env.pop(k)
    if layout_dir is not None:
        env[r1_placement.LAYOUT_DIR_ENV] = str(layout_dir)
    env["AGENTBENCH_EXTERNAL_ARTIFACT"] = str(artifact)
    env["AGENTBENCH_EXTERNAL_LABEL"] = CONDITION
    env["AGENTBENCH_EXTERNAL_NOTES"] = notes
    if answer_path is not None:
        env[external_agent.ANSWER_ENV] = str(answer_path)
    cmd = [sys.executable, str(AGENTBENCH / "run_agentbench.py"),
           "--tasks", task_id, "--agent", "external",
           "--repeats", "1", "--out", str(grade_dir)]
    proc = subprocess.run(cmd, env=env, capture_output=True, text=True,
                          encoding="utf-8", errors="replace",
                          cwd=str(REPO), timeout=3600)
    (Path(grade_dir) / "runner_stdout.log").write_text(
        proc.stdout or "", encoding="utf-8")
    (Path(grade_dir) / "runner_stderr.log").write_text(
        proc.stderr or "", encoding="utf-8")
    rows_path = Path(grade_dir) / "rows.jsonl"
    if not rows_path.is_file():
        raise RuntimeError("run_agentbench produced no rows.jsonl (rc=%s); "
                           "stdout tail: %s"
                           % (proc.returncode, (proc.stdout or "")[-800:]))
    lines = [ln for ln in rows_path.read_text(encoding="utf-8").splitlines()
             if ln.strip()]
    row = json.loads(lines[-1])
    # run_agentbench.py appends a generic "contains no trackable robot --
    # that is a FAIL of the artifact" note to ANY world whose recorder found
    # zero robots. B2's world is DELIBERATELY robot-free (its standalone
    # phase exists only to freeze the t=0 solids inventory; task meta says
    # so), so for B2 that note describes the EXPECTED shape, not a defect --
    # and it misled a live forensic pass (2026-08-01). run_agentbench.py is
    # frozen for the campaign, so the clarification is appended here, never
    # edited into the grader's own words.
    _ROBOT_FREE_NOTE = "contains no trackable robot"
    if task_id == "B2_subject_in_frame" and any(
            _ROBOT_FREE_NOTE in str(n) for n in (row.get("notes") or [])):
        row.setdefault("notes", []).append(
            "cc_lane clarification: B2's world is deliberately robot-free "
            "(the standalone run exists only to freeze the t=0 solids "
            "inventory), so the preceding 'no trackable robot' note is the "
            "expected shape of this task, not evidence of a broken "
            "artifact; the verdict rests on the B2.* camera assertions.")
    return row


def grade_webots(task_id, artifact, run_dir, answer, project_assets=()):
    """Through the Phase W control-arm pipeline, exactly as
    ``preregister/run_oracles.py`` grades its webots cells: launcher run +
    AABB prober (tasks that consume AABBs) + the sim-neutral grader core,
    plus B2's shipped-world view-evidence pass. Returns a
    run_agentbench-shaped row (dict).

    For the ANSWER_TASKS ``artifact`` is None: the graded world is the
    task's own PRISTINE ``initial_webots`` set (staged fresh from tasks/),
    because the answer is checked against ground truth the agent must not be
    able to move.

    ``project_assets`` are project-root-relative paths (the ones
    :func:`stage_task_assets` planted) that must travel with the project into
    the WSL workdir -- only ``worlds/`` and ``controllers/`` do so on their
    own, which is why a controller reading ``../../benchmark_assets/...``
    crashed at startup on R4's first cell.
    """
    from agentbench import tasks as task_registry
    from agentbench.adapters.webots import launcher, task_support
    from agentbench.common import worldtext

    task = task_registry.get(task_id)
    run_dir = Path(run_dir)
    # ``worlds``, not ``scratch``: ``run_dir`` is the grading PROJECT root, and
    # a world staged under a ``worlds`` component makes the engine's own root
    # walk (project_root_for_world) resolve THIS directory -- so the agent's
    # collected ``<run_dir>/controllers/<name>/`` is what
    # ``controller "<name>"`` finds, and the launcher copies that same dir into
    # its WSL workdir. The old ``scratch`` name reached the same root only by
    # the fallback branch (world's parent's parent), which any ancestor
    # literally named ``worlds`` above the results tree would have overridden.
    scratch = run_dir / "worlds"
    scratch.mkdir(parents=True, exist_ok=True)
    if task_id in ANSWER_TASKS or artifact is None:
        src_dir = task.dir / "initial_webots"
        for p in sorted(src_dir.rglob("*")):
            if p.is_dir() or p.name == ".gitkeep":
                continue
            dst = scratch / p.relative_to(src_dir)
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(p, dst)
    else:
        shutil.copy2(artifact, scratch / Path(artifact).name)
    graded = worldtext.pick_artifact(scratch)

    sa = task.standalone
    _dir, facts = launcher.launch(
        graded, run_dir / "webots",
        duration=float(sa.get("duration_s", 10.0)),
        contact_steps=int(sa.get("contact_steps", 1)),
        # Forward the link/solid track requests, or the WEBOTS arm stays blind
        # to arm links and named objects while OmniSim can see them -- an
        # instrument asymmetry that would read as a capability difference.
        links=int(sa.get("links", 0)),
        solids=tuple(sa.get("solids", ())),
        extra_project_paths=tuple(project_assets))
    aabbs_doc, pfacts = None, {}
    if task_id in NEEDS_AABB:
        aabbs_doc, pfacts = task_support.probe_world(graded, run_dir / "aabb")
    run2, merged = task_support.augment_run(run_dir / "webots", aabbs_doc)

    kw = {"artifact": graded, "scratch_dir": scratch, "sim": "webots",
          "run": run2, "self_verified": False}
    if task_id == "B2_subject_in_frame":
        shipped = task.dir / "initial_webots" / "frame_the_cylinder.wbt"
        init_doc, _f = task_support.probe_world(shipped,
                                                run_dir / "aabb_initial")
        kw["view"] = task_support.view_evidence(init_doc, aabbs_doc,
                                                artifact=graded)
    if task_id not in NO_ANSWER_GRADERS:
        kw["answer"] = answer
    grader = task.grader()
    v = grader.grade(run_dir, **kw)
    vd = v.as_dict()
    (run_dir / "verdict.json").write_text(
        json.dumps(vd, indent=2, default=str), encoding="utf-8")

    machine = ob_results.machine_fingerprint()
    row = {
        "suite": SUITE,
        "task": task.id, "tier": task.tier,
        "sim": "webots",
        "condition": CONDITION,
        "read_deny": [],
        "repeat": 0,
        "agent": {"model": None, "temperature": None,
                  "scaffold_sha": "cc_lane", "system_prompt_sha": None,
                  "kind": "claude_code", "backend": None,
                  "credential_source": None},
        "tool_set": {"name": "claude_code", "tools_sha256": None,
                     "manifest_sha256": None},
        "stop_reason": None,
        "agent_artifacts": {
            "condition": CONDITION,
            "external_label": CONDITION,
            "aabb_probe": ({"ok": aabbs_doc is not None,
                            "exit_code": pfacts.get("exit_code")}
                           if task_id in NEEDS_AABB else None),
            "aabb_merged_bodies": merged,
            "launch": {k: facts.get(k) for k in
                       ("exit_code", "timed_out", "wall_s",
                        "attempts_used")},
        },
        "outcome": vd["outcome"], "progress": vd["progress"],
        "progress_name": vd["progress_name"],
        "self_verified": vd["self_verified"],
        "assertions": {k: a["ok"] for k, a in vd["assertions"].items()},
        "assertion_detail": vd["assertions"],
        "failed_assertion": vd["failed_assertion"],
        "expectation": {"expect_pass": None, "expect_failures": None,
                        "got_pass": vd["outcome"] == "PASS",
                        "failed_assertions": v.failed,
                        "pass_matches": None, "failures_match": None,
                        "ok": None},
        "metrics": {"t_agent_s": None, "t_total_s": None, "turns": None,
                    "tool_calls": None, "tokens_in": None,
                    "tokens_out": None, "tokens_cache_read": None,
                    "tokens_cache_write": None, "usd": None},
        "measurements": vd["measurements"],
        "par_s": task.par_s, "timeout_s": task.timeout_s,
        "interventions": 0,
        "sim_version": {"name": "webots",
                        "version": facts.get("webots_version"),
                        "install": launcher.WEBOTS_HOME,
                        "distro": launcher.DISTRO},
        "image_digest": None,
        "artifacts": vd["artifacts"],
        "deviations": [],
        "notes": vd["notes"],
        "machine": machine,
        "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    return row


def grade_mujoco(task_id, artifact, run_dir, answer):
    """Through the MuJoCo arm: run the model + its driver under the grader's
    own recorder, then the SAME sim-neutral grader core the other arms use.

    ``artifact`` is the collected MJCF model; its driver sits beside it under
    the same stem (:func:`collect_driver`), which is
    ``launcher.find_driver``'s first rule -- so the pair travels together and
    the launcher re-derives the same answer here that collection did.

    Two differences from the other arms, both structural rather than chosen:

    * **The recording window is the GRADER's clock.** MJCF has no
      ``--duration`` and MuJoCo starts no processes, so the adapter's recorder
      owns termination (``adapters/mujoco/BRINGUP.md`` §2). The task's own
      ``standalone.duration_s`` is what it is given -- the identical number the
      webots arm passes to ``launcher.launch``.
    * **No AABB prober and no separate probe pass.** The t=0 scan bounds every
      body and world geom in the same run, so there is no second launch to
      merge (BRINGUP §3.2). Nothing is lost: what the webots arm gets from a
      probe, this arm gets from the run it already did.

    An arm that cannot run at all (no ``mujoco`` in the cell's interpreter)
    reports a MISSING DEPENDENCY through ``process.json`` -- ``launcher.launch``
    never raises -- so it reads as a broken instrument in the row rather than
    as a simulator that failed to simulate.
    """
    from agentbench import tasks as task_registry
    from agentbench.adapters.mujoco import launcher as mj_launcher
    from agentbench.adapters.mujoco import recording as mj_recording

    task = task_registry.get(task_id)
    run_dir = Path(run_dir)
    scratch = run_dir / "worlds"
    scratch.mkdir(parents=True, exist_ok=True)
    model = scratch / Path(artifact).name
    shutil.copy2(artifact, model)
    # The driver travels with the model, under the same stem -- exactly as it
    # was collected. A model staged without it is a scene that cannot move.
    src_driver = Path(artifact).with_suffix(".py")
    staged_driver = None
    if src_driver.is_file():
        staged_driver = model.with_suffix(".py")
        shutil.copy2(src_driver, staged_driver)

    sa = task.standalone
    mj_dir = run_dir / "mujoco"
    _dir, facts = mj_launcher.launch(
        model, mj_dir,
        duration=float(sa.get("duration_s", 10.0)))
    run = mj_recording.read_run(mj_dir)

    # ``mujoco_dir``, never ``run_dir``: the grader passes its own positional
    # ``run_dir`` through to the adapter, and the adapter looks for this arm's
    # artifact set under ``mujoco_dir`` (or a ``mujoco/`` child of run_dir).
    # ``run=`` is supplied outright, so the search never has to happen.
    kw = {"artifact": model, "scratch_dir": scratch, "sim": "mujoco",
          "run": run, "mujoco_dir": mj_dir, "self_verified": False}
    if task_id not in NO_ANSWER_GRADERS:
        kw["answer"] = answer
    grader = task.grader()
    v = grader.grade(run_dir, **kw)
    vd = v.as_dict()
    (run_dir / "verdict.json").write_text(
        json.dumps(vd, indent=2, default=str), encoding="utf-8")

    machine = ob_results.machine_fingerprint()
    row = {
        "suite": SUITE,
        "task": task.id, "tier": task.tier,
        "sim": "mujoco",
        "condition": CONDITION,
        "read_deny": [],
        "repeat": 0,
        "agent": {"model": None, "temperature": None,
                  "scaffold_sha": "cc_lane", "system_prompt_sha": None,
                  "kind": "claude_code", "backend": None,
                  "credential_source": None},
        "tool_set": {"name": "claude_code", "tools_sha256": None,
                     "manifest_sha256": None},
        "stop_reason": None,
        "agent_artifacts": {
            "condition": CONDITION,
            "external_label": CONDITION,
            # The pair, named on the row: a MuJoCo cell graded without a
            # driver measured a scene nobody stepped, and the row has to say
            # so rather than leaving a reader to infer it from zero motion.
            "mujoco_driver": (str(staged_driver) if staged_driver else None),
            "mujoco_driver_rule": facts.get("driver_rule"),
            "launch": {k: facts.get(k) for k in
                       ("exit_code", "timed_out", "wall_s", "attempts_used",
                        "python", "interpreter_probe", "error")},
        },
        "outcome": vd["outcome"], "progress": vd["progress"],
        "progress_name": vd["progress_name"],
        "self_verified": vd["self_verified"],
        "assertions": {k: a["ok"] for k, a in vd["assertions"].items()},
        "assertion_detail": vd["assertions"],
        "failed_assertion": vd["failed_assertion"],
        "expectation": {"expect_pass": None, "expect_failures": None,
                        "got_pass": vd["outcome"] == "PASS",
                        "failed_assertions": v.failed,
                        "pass_matches": None, "failures_match": None,
                        "ok": None},
        "metrics": {"t_agent_s": None, "t_total_s": None, "turns": None,
                    "tool_calls": None, "tokens_in": None,
                    "tokens_out": None, "tokens_cache_read": None,
                    "tokens_cache_write": None, "usd": None},
        "measurements": vd["measurements"],
        "par_s": task.par_s, "timeout_s": task.timeout_s,
        "interventions": 0,
        # READ from the interpreter that actually ran the cell, never
        # asserted: the launcher probes it once per launch (`probe_python`)
        # and publishes what it found, so a missing library reads as a missing
        # library rather than as a simulator that failed to simulate.
        "sim_version": {"name": "mujoco",
                        "version": facts.get("mujoco_version"),
                        "install": facts.get("python"),
                        "distro": None},
        "image_digest": None,
        "artifacts": vd["artifacts"],
        "deviations": [],
        "notes": vd["notes"],
        "machine": machine,
        "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    return row


# --- the cell ---------------------------------------------------------------------


def _utc():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _sweep_processes(ws, sim, report, when):
    """Kill stragglers whose command line references the instance (they are
    ours by construction -- the session launched them inside its workspace
    and exited without reaping). Logged into the cell report; never raises.
    Measured need (2026-08-01): a lingering omnisim-bin after the B1 session
    held omnisim_log.txt open and a plain rmtree crashed the whole cell."""
    if ws is None:
        return []
    try:
        kills = staging.sweep_workspace_processes(
            ws, wsl_distro=(staging.WEBOTS_DISTRO if sim == "webots"
                            else None))
    except Exception as exc:                              # noqa: BLE001
        kills = [{"where": "host", "pid": None, "name": None,
                  "cmdline": None, "action": "sweep_failed",
                  "detail": repr(exc)}]
    report.setdefault("process_sweeps", []).append(
        {"when": when, "utc": _utc(), "kills": kills})
    for k in kills:
        if k.get("pid") is not None:
            print("process sweep (%s): %s pid=%s (%s)"
                  % (when, k.get("action"), k.get("pid"),
                     (k.get("name") or "?")))
    return kills


def _sweep_ports(report, when, *, mine=None, others=()):
    """Reap anything listening on the well-known harness/capture ports.

    Port-scoped contamination guard (measured 2026-08-01, phasew_cc_v1
    B2:omnisim r0-r2): a harness leaked by a PREVIOUS cell's agent -- whose
    cmdline referenced the real repo, so the workspace sweep could not see it
    -- sat on port 6789 serving a foreign world, and every later agent that
    probed the canonical port mistook it for its task world. Between cells
    the lane owns these ports; any listener is a leak. Logged into the cell
    report (an empty kill list is the clean-ports witness); never raises."""
    try:
        kills = staging.reap_port_listeners(mine=mine, others=others)
    except Exception as exc:                              # noqa: BLE001
        kills = [{"where": "port", "pid": None, "name": None,
                  "cmdline": None, "action": "sweep_failed",
                  "detail": repr(exc)}]
    report.setdefault("process_sweeps", []).append(
        {"when": when, "kind": "port", "utc": _utc(), "kills": kills,
         # WHO ELSE WAS LIVE, recorded next to what we killed. Without it a
         # reader cannot tell "nothing to spare" from "we did not look", and
         # the r4_c1 incident is exactly the difference.
         "other_live_cells": [{k2: c.get(k2) for k2 in
                               ("pid", "lane", "sim", "task", "workspace")}
                              for c in (others or ())]})
    for k in kills:
        if k.get("pid") is not None:
            print("port sweep (%s): %s pid=%s on %s (%s)"
                  % (when, k.get("action"), k.get("pid"), k.get("where"),
                     (k.get("name") or "?")))
    return kills


def _quarantine_dir(run_dir):
    """The campaign's quarantine dir for pre-session junction leftovers.

    A campaign cell runs at ``<campaign>/cells/<key>``, so quarantine lands
    at ``<campaign>/quarantine/``; a standalone cell quarantines beside
    itself under ``results/cc_lane/quarantine/``. Findings there are
    attributed to NO cell -- they are leftovers whose author is unknown
    (a crashed or pre-fix prior cell)."""
    run_dir = Path(run_dir)
    parent = run_dir.parent
    if parent.name in ("cells", "cells_superseded"):
        return parent.parent / "quarantine"
    return parent / "quarantine"


def _sweep_repo_artifacts(report, when, dest, *, window=None,
                          protect_after_ts=None):
    """Junction-artifact hygiene (measured 2026-08-01, A1 r1/r3/r6: session
    writes through the projects/ junction landed in the REAL repo and later
    same-task cells saw them as their own). Preserve-then-delete untracked
    session-shaped files under the repo's projects/ tree; full rails in
    ``stage_workspaces.sweep_repo_junction_artifacts``. Logged into the cell
    report (an empty record list is the clean-tree witness); never raises."""
    try:
        records = staging.sweep_repo_junction_artifacts(
            dest, window=window, protect_after_ts=protect_after_ts)
    except Exception as exc:                              # noqa: BLE001
        records = [{"rel": None, "action": "sweep_failed",
                    "detail": repr(exc)}]
    report.setdefault("repo_artifact_sweeps", []).append(
        {"when": when, "utc": _utc(), "dest": str(dest),
         "window": list(window) if window else None,
         "protect_after_ts": protect_after_ts, "records": records})
    for r in records:
        if r.get("rel"):
            print("repo artifact sweep (%s): %s %s"
                  % (when, r.get("action"), r.get("rel")))
    return records


#: How often the live mirror re-syncs the agent's workspace into the cell dir
#: while the session runs. 20 s is a compromise: often enough that `--status`
#: shows work appearing, rare enough that an incremental walk of a 4,800-file
#: omnisim workspace is noise against a 30-minute session.
WORKSPACE_MIRROR_INTERVAL_S = 20.0


def _preserve_ws(ws, run_dir, report, *, newer_than=None, link_dirs=None):
    """Copy the agent's workspace into ``<cell>/workspace/``. Idempotent.

    THE RULE, and it has no exceptions: the workspace is the ARTIFACT OF
    RECORD, so it is preserved on every outcome -- PASS, FAIL, blocked,
    timeout, crash -- before anything can delete it. `r1_3arm_20260810` blocked
    two cells on a stdout-parsing detail and then tore down both workspaces;
    the only surviving trace of a mujoco run claiming a complete working
    solution is the agent's own prose summary, which nothing on this machine
    can now confirm or refute. A verdict that rests on a self-report is not a
    measurement.

    Never fatal: an unpreservable workspace is recorded and the cell carries
    on (losing the row as well as the evidence helps nobody).
    """
    if ws is None:
        return None
    dest = Path(run_dir) / "workspace"
    # The session window, so writes made THROUGH the projects/ junction land in
    # the record too (defect 5). Read off the report when the caller did not
    # pass it, so the teardown backstop preserves the same set as the mirror.
    if newer_than is None:
        newer_than = report.get("session_start_ts")
    if link_dirs is None:
        link_dirs = (evidence.DEFAULT_LINK_WINDOW_DIRS
                     if newer_than is not None else ())
    try:
        man = evidence.preserve_workspace(ws, dest, newer_than=newer_than,
                                          link_dirs=link_dirs)
    except Exception as exc:                       # noqa: BLE001
        man = {"error": repr(exc), "source": str(ws), "dest": str(dest)}
    prev = report.get("workspace_preserved")
    if isinstance(prev, dict) and prev.get("passes"):
        man["passes"] = prev["passes"] + 1
    else:
        man["passes"] = 1
    report["workspace_preserved"] = man
    _write_report(run_dir, report)
    return man


def _teardown_ws(ws, sim, report, run_dir):
    """Best-effort workspace teardown POSTSCRIPT: sweep stragglers, then the
    resilient teardown (retry -> *.pending_delete). Updates and re-writes the
    cell report; NEVER raises for a locked file -- by the time this runs the
    row/report must already be on disk, and a locked temp file must never
    kill a campaign."""
    if ws is None:
        return None
    # LAST CHANCE. Every teardown path goes through here, so this is the one
    # place that guarantees the evidence outlives the workspace -- including
    # the paths that never reached the explicit calls in run_cell.
    _preserve_ws(ws, run_dir, report)
    _sweep_processes(ws, sim, report, "pre_teardown")
    try:
        outcome = staging.teardown_workspace_resilient(ws)
    except ValueError:
        raise                       # the repo-refusal guard stays fatal
    except Exception as exc:        # noqa: BLE001 -- postscript, never fatal
        outcome = {"ok": False, "severed": [], "attempts": 0,
                   "error": repr(exc), "pending": None}
    report["workspace_teardown"] = outcome
    if not outcome.get("ok"):
        print("workspace teardown failed (%s); marked for later sweep: %s"
              % (outcome.get("error"), outcome.get("pending")))
    _write_report(run_dir, report)
    return outcome


def run_cell(sim, task_id, *, root, out_dir, model=None,
             timeout_s=DEFAULT_TIMEOUT_S, keep_workspace=False,
             lane=None, engine_slots=concurrency.DEFAULT_ENGINE_SLOTS,
             lock_root=None, use_locks=True,
             rate_limit_backoff_s=DEFAULT_RATE_LIMIT_BACKOFF_S,
             max_rate_limit_retries=DEFAULT_MAX_RATE_LIMIT_RETRIES,
             repeat=0, layout_seed=None,
             runs_per_cell=PROTOCOL_RUNS_PER_CELL,
             guard_enabled=True, pin_engine_sha256=None):
    root = Path(root)
    run_dir = Path(out_dir).resolve()   # webots WSL path translation needs
    #                                     absolute paths (see main())
    run_dir.mkdir(parents=True, exist_ok=True)
    lane = lane or sim
    lock_root = Path(lock_root) if lock_root else (root / "locks")
    report = {"sim": sim, "task": task_id, "condition": CONDITION,
              "lane": lane, "utc_start": _utc()}

    # THE ROUND'S ENGINE, recorded before anything is spent and asserted
    # against the round's pin. Recorded on every cell whether or not a pin was
    # given: a row that cannot name the binary it ran cannot be compared to
    # one from another day.
    if sim == "omnisim":
        report["engine"] = engine_identity()
        blocked, why = engine_pin_blocks(report["engine"], pin_engine_sha256)
        if blocked:
            report["engine_pin"] = pin_engine_sha256
            report["blocker"] = why
            _write_report(run_dir, report)
            raise SystemExit(why)
        report["engine_pin"] = pin_engine_sha256

    # The seed R1's graded obstacle layout is drawn from. Resolution order,
    # most explicit first -- and RECORDED on the row whichever it was, so the
    # layout a cell was scored against is re-derivable from the row alone
    # (``r1_core.sample_layout`` is pure: one seed, one layout, any machine).
    # The default is per (task, repeat) and carries no simulator, so the same
    # repeat draws the SAME layout on every arm and a sim-vs-sim comparison
    # compares simulators rather than layouts.
    layout_seed_source = "argument (--layout-seed / campaign)"
    if not layout_seed:
        layout_seed = os.environ.get("AGENTBENCH_R1_LAYOUT_SEED") or None
        layout_seed_source = "$AGENTBENCH_R1_LAYOUT_SEED"
    if not layout_seed:
        layout_seed = r1_placement.default_seed(task_id, repeat)
        layout_seed_source = "default <campaign>/<task>/r<repeat>"
    report["layout_seed"] = layout_seed
    report["layout_seed_source"] = layout_seed_source

    # Refuse a declared-but-unbuilt comparator here, with its blocker named,
    # before any workspace is staged or a token is spent -- and refuse an arm
    # that cannot EXPRESS this task for the same reason at the same moment.
    #
    # Passing the task id is the whole point (SPEC 6.4): an arm whose fixture
    # set covers part of the suite is not a broken arm, and its unrunnable
    # cells must not become FAILs attributed to that simulator. MuJoCo can
    # express A1/R1/R2 today; B1/B2/B3/C1/C2/R3 each ship a `.wbt` fixture with
    # no MJCF equivalent, and running one would have measured OUR missing
    # fixture and printed MuJoCo's name on it.
    sims.require_implemented(sim, task_id)

    # The wall-clock budget is the TASK's (SPEC 2.4) -- already clamped to
    # tasks.TASK_HARD_CEILING_S by Task.timeout_s. A CLI --timeout-s may only
    # TIGHTEN it; it can never buy a cell a longer run than its task declares.
    _task = tasks_mod.get(task_id)
    task_budget_s = float(_task.timeout_s)
    timeout_s = min(float(timeout_s), task_budget_s)
    # The WHOLE cell -- every session, every deferral, every backoff -- is
    # bounded here, not just one session. Without this a rate-limited cell
    # multiplies its budget by the retry count and the campaign's cost bound
    # is fiction.
    cell_started_mono = time.monotonic()
    cell_wall_s = cell_wall_bound_s(task_budget_s)
    cell_deadline_mono = cell_started_mono + cell_wall_s
    report["budget"] = {
        "task_timeout_s": task_budget_s,
        "declared_timeout_s": _task.declared_timeout_s,
        "hard_ceiling_s": tasks_mod.TASK_HARD_CEILING_S,
        "effective_timeout_s": timeout_s,
        "par_s": _task.par_s,
        "cell_wall_bound_s": cell_wall_s,
        "cell_wall_allowance_s": CELL_WALL_ALLOWANCE_S,
        # One entry per session ATTEMPT. A cell that burns wall clock on a
        # rate-limit backoff gets a shorter retry than its task declares, and
        # a silently shortened budget is a hidden handicap -- so every
        # attempt's real budget is recorded and `curtailed` says plainly
        # whether any of them got less than the task's own timeout_s.
        "session_budgets_s": [],
        "session_budget_curtailed": False,
    }

    child_env, removed = scrub_env()
    report["env_scrubbed"] = removed
    # Headless is not optional and it is not one-sided: the shared scrub
    # carries both arms' windowless directives, and this tailors them to the
    # arm this cell runs on (see HEADLESS_ENV). A cell whose directive could
    # not be established is FLAGGED, never silently timed.
    report["headless"] = enforce_headless(child_env, sim)
    if not report["headless"]["enforced"]:
        print("headless NOT enforced for this %s cell: %s"
              % (sim, report["headless"]["detail"]))

    # Informational pre-cell resource reading (the campaign driver is the
    # component that WAITS on this guard; a standalone cell records it).
    guard_ok, guard_detail = concurrency.resource_guard()
    report["resource_guard"] = {"ok": guard_ok, **guard_detail}

    # 1. preflight (no engine, no lock needed) ------------------------------
    pf = preflight(run_dir, child_env)
    report["preflight"] = pf
    (run_dir / "preflight.json").write_text(json.dumps(pf, indent=2),
                                            encoding="utf-8")
    if not pf["ok"]:
        report["blocker"] = "preflight failed: %s" % pf["detail"]
        _write_report(run_dir, report)
        raise SystemExit("preflight failed: %s" % pf["detail"])
    # The model is PINNED to the benchmark's own id, not inherited from
    # whatever the local CLI happens to default to -- that inheritance is how
    # the superseded agentbench/v0 grid ended up scored on a different model.
    pinned = model or DEFAULT_MODEL
    report["pinned_model"] = pinned
    report["cli_default_model"] = pf.get("default_model")
    report["model_pin_source"] = ("--model" if model else "DEFAULT_MODEL")

    # Concurrency protocol (plan §2.7). The task lock is held for the WHOLE
    # cell: same-task cells never overlap (plan §5.3). The engine semaphore
    # is held around engine-heavy phases: the whole session for omnisim
    # cells, the grading pass always.
    engine = concurrency.EngineSlots(lock_root, slots=engine_slots, lane=lane)
    tlock = concurrency.task_lock(lock_root, task_id, lane=lane)
    samples = []

    # WHO WE ARE, published so the OTHER lane's hygiene sweeps can tell our
    # live work from a leak. Registered before the queue (a cell waiting on the
    # task lock owns nothing yet, but registering here means the claim exists
    # for every later phase) and re-published with the workspace path the
    # moment one is instantiated. See concurrency.active_cells: a claim stops
    # protecting anything the instant its pid dies, so this can never disarm
    # the leak sweep for a later cell.
    claim_ts = time.time()
    claim_path = concurrency.register_cell(
        lock_root, lane=lane, sim=sim, task=task_id, workspace=None,
        run_dir=run_dir, started_ts=claim_ts)
    my_claim = {"pid": os.getpid(), "lane": lane, "sim": sim, "task": task_id,
                "workspace": None, "started_ts": claim_ts}

    def _others():
        """Every OTHER live cell on this machine, freshly read."""
        try:
            return concurrency.active_cells(lock_root,
                                            exclude_pid=os.getpid())
        except Exception:                                 # noqa: BLE001
            return []

    def _sample(tag):
        samples.append({"tag": tag, "utc": _utc(),
                        "others_active": engine.others_active()})

    def _preserve(w):
        """The workspace copy. Called at every exit -- and once more from
        `_teardown_ws`, which is the backstop for paths that miss these."""
        return _preserve_ws(w, run_dir, report)

    def _claim_workspace(w):
        """Re-publish our claim once the workspace path is known."""
        my_claim["workspace"] = str(w) if w else None
        try:
            concurrency.register_cell(lock_root, lane=lane, sim=sim,
                                      task=task_id, workspace=w,
                                      run_dir=run_dir, started_ts=claim_ts)
        except OSError:
            pass

    # THE QUEUE IS NOT THE CELL'S TIME. The wall deadline was armed above,
    # BEFORE this blocking acquire -- so a cell that waits behind another
    # lane's same-task cell pays for the wait out of its own agent budget.
    #
    # MEASURED, and it is what destroyed `r1_3arm_20260810`: three lanes
    # started the same task at 13:35:26Z and serialised here. mujoco ran
    # first (908 s); omnisim waited 15 min then ran; webots waited **42 of
    # its 45 minutes** and its session was handed the 156.7 s that were left.
    # It was killed mid-work and published FAIL/no_artifact at progress 0 --
    # a row about our scheduler wearing the simulator's name.
    #
    # Queue time is therefore EXCLUDED from the bound and recorded on its own.
    # The bound still does its job (it exists so a badly-behaved SESSION
    # cannot walk through the ceiling) and the campaign's cost bound is
    # unchanged in the only sense that matters -- the machine is busy with one
    # cell at a time either way, because that is what the lock enforces.
    queue_waits = []
    if use_locks:
        _q0 = time.monotonic()
        tlock.acquire(on_wait=lambda p: print(
            "waiting on same-task lock %s (another %s cell is running)"
            % (p.name, task_id)))
        _qs = time.monotonic() - _q0
        if _qs > 1.0:
            cell_deadline_mono += _qs
            queue_waits.append({"what": "same-task lock", "waited_s":
                                round(_qs, 1)})
            print("queued %.0f s on the same-task lock; the cell's wall "
                  "deadline is extended by the wait (queue time is the "
                  "scheduler's, not the agent's)" % _qs)
    report["budget"]["queue_waits"] = queue_waits
    ws = None
    try:
        # 2+3. workspace + session, with rate-limit deferral ----------------
        if sim == "omnisim":
            tpl = staging.build_omnisim_template(root)
        elif sim == "mujoco":
            tpl = staging.build_mujoco_template(root)
        else:
            tpl = staging.build_webots_template(root)
        deferrals = []
        attempt = 0
        while True:
            inst_name = "%s_%s_%s" % (time.strftime("%Y%m%d_%H%M%S"), sim,
                                      task_id.split("_")[0])
            if attempt:
                inst_name += "_d%d" % attempt
            ws = root / "instances" / inst_name
            inst_manifest = staging.instantiate(tpl, ws)
            report["workspace"] = str(ws)
            _claim_workspace(ws)
            report["staging_manifest"] = str(ws.parent /
                                             (ws.name + ".manifest.json"))
            prompt, staged = staging.stage_task(ws, task_id, sim)
            report["prompt"] = prompt
            report["staged_task_files"] = staged
            if sim == "omnisim":
                child_env["OMNISIM_HOME"] = str(ws)
                report["child_env_set"] = {"OMNISIM_HOME": str(ws)}

            # CONTAINMENT, before a single token is spent. The guard config is
            # written per cell because it names THIS cell's workspace and run
            # dir; nothing about it reaches the child's environment (an env var
            # naming the benchmark would tell the agent it is being measured).
            guard = (install_guard(run_dir, ws, scratch_root=root)
                     if guard_enabled else
                     {"installed": False,
                      "reason": "disabled with --no-containment-guard"})
            report["containment_guard"] = guard

            # The omnisim session may launch the engine at any point, so the
            # WHOLE session is engine-heavy and holds a slot (plan §2.7).
            if use_locks and sim == "omnisim":
                _q0 = time.monotonic()
                engine.acquire("omnisim cell %s: session" % task_id,
                               on_wait=lambda p: print(
                                   "waiting for a free engine slot..."))
                _qs = time.monotonic() - _q0
                if _qs > 1.0:                 # same rule as the task lock
                    cell_deadline_mono += _qs
                    queue_waits.append({"what": "engine slot",
                                        "waited_s": round(_qs, 1)})
            # Contamination guard: the session must not find a harness some
            # previous cell leaked onto the canonical ports (it would mistake
            # the foreign scene for its task world -- measured, B2 r1/r2).
            others = _others()
            _sweep_ports(report, "pre_session", mine=my_claim, others=others)
            # ...nor leftovers a crashed (or pre-fix) prior cell wrote
            # through the projects/ junction into the real repo (the A1
            # r1/r3/r6 poisoning). Pre-session findings are QUARANTINED to
            # the campaign dir, attributed to no cell. Omnisim cells only:
            # the webots workspace has no junction into this repo, and a
            # webots-lane sweep could race a live omnisim lane's session.
            if sim == "omnisim":
                # `window=None` takes every eligible untracked file, which was
                # safe only while no other cell was running. MEASURED
                # 2026-08-12: it `preserved_and_deleted` a CONCURRENT cell's
                # live `husky_random_10.wbt`. Anything newer than the oldest
                # live cell's start may be that cell's work in progress.
                _sweep_repo_artifacts(
                    report, "pre_session",
                    _quarantine_dir(run_dir)
                    / time.strftime("%Y%m%d_%H%M%S"),
                    protect_after_ts=concurrency.oldest_active_start(others))
            _sample("session_start")
            start_ts = time.time()
            session_budget = max(
                1.0, min(timeout_s, cell_deadline_mono - time.monotonic()))
            report["budget"]["session_budgets_s"].append(round(session_budget, 1))
            if session_budget < timeout_s - 1.0:
                report["budget"]["session_budget_curtailed"] = True
            # REFUSE A STARVED SESSION BEFORE SPENDING ANYTHING ON IT. A run
            # granted a small fraction of its task's budget cannot produce a
            # comparable row, so starting it buys a FAIL that describes the
            # instrument and burns quota to do it (measured: 156.8 s of tokens
            # spent on a cell that was killed mid-work and scored 0).
            floor = MIN_SESSION_BUDGET_FRACTION * timeout_s
            if session_budget < floor:
                report["liveness"] = assess_liveness(
                    meta={}, stream={}, transcript=None,
                    session_budget_s=round(session_budget, 1),
                    task_budget_s=timeout_s, pinned_model=pinned)
                report["blocker"] = (
                    "cell wall clock exhausted before the session could get a "
                    "usable budget: %.0f s left of the %.0f s bound, which is "
                    "%.1f%% of the task's %.0f s budget (floor %.0f%%). "
                    "Running it would publish a FAIL that measures OUR "
                    "scheduling, not the agent, so the cell is abandoned "
                    "instead. This is the r1_3arm_20260810 webots case."
                    % (session_budget, cell_wall_s,
                       100.0 * session_budget / timeout_s, timeout_s,
                       100.0 * MIN_SESSION_BUDGET_FRACTION))
                _write_report(run_dir, report)
                _preserve(ws)
                if ws is not None:
                    _teardown_ws(ws, sim, report, run_dir)
                raise SystemExit(report["blocker"])
            # THE REPORT GOES TO DISK BEFORE THE SESSION STARTS, not after it
            # ends. Everything `--status` needs to describe a LIVE cell --
            # which sim, which task, which budget, which workspace -- is
            # known here, and a report written only at the end is a report
            # that exists only when it is no longer needed.
            _write_report(run_dir, report)
            # LIVE MIRROR of the agent's workspace into the cell dir while it
            # works, so "what is it building right now" is answerable from
            # disk (see --status) and so an unclean exit can never take the
            # evidence with it.
            # THE SESSION'S OWN START, distinct from the cell's. `--status`
            # judges a plateau against this, never against `utc_start` plus
            # however long the same-task queue held us (defect 8).
            # WHAT THE AGENT'S OWN SEARCH CANNOT SEE, recorded before it
            # starts. An instrument artefact that nothing measures is an
            # instrument artefact nobody corrects for (defect 7).
            try:
                report["search_visibility"] = search_visibility(ws)
                if report["search_visibility"].get("biased"):
                    print("search visibility: %s"
                          % json.dumps(report["search_visibility"]["probes"]))
            except Exception as exc:                   # noqa: BLE001
                report["search_visibility"] = {"error": repr(exc)}
            report["session_start_ts"] = start_ts
            report["session_started_utc"] = time.strftime(
                "%Y-%m-%dT%H:%M:%SZ", time.gmtime(start_ts))
            _write_report(run_dir, report)
            mirror = evidence.WorkspaceMirror(
                ws, run_dir / "workspace",
                interval_s=WORKSPACE_MIRROR_INTERVAL_S,
                # The deliverable is usually behind the projects/ junction, and
                # on a KILLED cell the mirror is the only pass that ever runs.
                newer_than=start_ts,
                link_dirs=evidence.DEFAULT_LINK_WINDOW_DIRS).start()
            try:
                result, meta = run_claude_cell(
                    prompt, ws, child_env, run_dir, model=pinned,
                    timeout_s=session_budget,
                    settings_path=guard.get("settings"))
            finally:
                report["workspace_mirror"] = mirror.stop()
            end_ts = time.time()
            _sample("session_end")
            report["cc_meta"] = meta
            # The authoritative copy, taken the moment the session exits and
            # before any sweep, teardown or grading can touch the tree.
            _preserve(ws)

            # The CHILD's evidence only -- see `deferral_reason` for the cell
            # this rule cost when our own summary prose was scanned instead.
            limit = deferral_reason(result, meta)
            room_left = cell_deadline_mono - time.monotonic()
            can_retry = room_left > (rate_limit_backoff_s + 60.0)
            if limit is not None and not can_retry:
                # Rate-limited AND out of wall clock. This is the INSTRUMENT
                # running out of room, not the agent failing, so it must not
                # be scored as a FAIL -- but it also may not keep the machine.
                report["blocker"] = (
                    "rate/usage limited (%r) with %.0f s of the cell's %.0f s "
                    "wall ceiling left: not enough room to retry, so the cell "
                    "is abandoned rather than scored (an instrument limit is "
                    "never an agent result)"
                    % (limit, max(0.0, room_left), cell_wall_s))
                report["rate_limit_deferrals"] = deferrals
                _write_report(run_dir, report)
                if ws is not None:
                    _teardown_ws(ws, sim, report, run_dir)
                raise SystemExit(report["blocker"])
            if limit is not None and attempt < max_rate_limit_retries:
                # A usage/rate limit is NOT a failed run and burns no quota:
                # record the attempt as deferred, tear down, back off, retry
                # the same cell with a fresh workspace. Only sessions where
                # Claude actually started working count against the
                # one-run-per-cell rule.
                if use_locks and sim == "omnisim":
                    engine.release()
                deferrals.append({"utc": _utc(), "marker": limit,
                                  "attempt": attempt,
                                  "backoff_s": rate_limit_backoff_s})
                report["rate_limit_deferrals"] = deferrals
                # A CELL ASLEEP IN A BACKOFF AND A HUNG CELL LOOKED IDENTICAL
                # to `--status` (both: a stale stream, no row, the task lock
                # held). Say what it is waiting for and until when, BEFORE
                # sleeping, so the operator's kill-at-plateau decision is made
                # on the truth. Measured need: `a1_omnisim_c2`.
                report["deferred_until_utc"] = time.strftime(
                    "%Y-%m-%dT%H:%M:%SZ",
                    time.gmtime(time.time() + rate_limit_backoff_s))
                _write_report(run_dir, report)
                _teardown_ws(ws, sim, report, run_dir)
                ws = None
                print("rate/usage limit (%r): attempt deferred; retrying the "
                      "same cell in %.0f s (deferral %d/%d)"
                      % (limit, rate_limit_backoff_s, attempt + 1,
                         max_rate_limit_retries))
                time.sleep(rate_limit_backoff_s)
                report.pop("deferred_until_utc", None)
                _write_report(run_dir, report)
                attempt += 1
                continue
            break
        report["rate_limit_deferrals"] = deferrals

        # The session has exited: reap anything it left running inside the
        # workspace NOW (engine/harness stragglers hold files open and would
        # otherwise poison teardown -- and could interfere with grading).
        _sweep_processes(ws, sim, report, "post_session")
        # ...and anything it leaked onto the canonical harness ports from
        # OUTSIDE the workspace (the B1-r4 escape: `cd <repo> && harness
        # --port 6789` matches no workspace path but poisons every later
        # cell on this machine).
        _sweep_ports(report, "post_session", mine=my_claim, others=_others())
        # ...and anything it wrote THROUGH the projects/ junction into the
        # real repo (the r1/r3/r6 poisoning: sequential same-task cells saw
        # predecessors' worlds through their own junction). Preserved into
        # the cell dir as evidence (sha256 in the report), then deleted from
        # the repo -- BEFORE grading, and before the next cell's junction
        # shows the tree. discover_artifact reads the preserved copies
        # (rule 2b), so the deliverable is still graded.
        if sim == "omnisim":
            _sweep_repo_artifacts(report, "post_session",
                                  run_dir / "repo_artifacts",
                                  window=(start_ts, end_ts))

        # THE SESSION'S STDOUT IS METADATA. THE WORKSPACE IS THE ARTIFACT.
        #
        # This used to read: no result JSON and no timeout -> BLOCK, no row,
        # nothing graded. That made the verdict depend on the agent's exit
        # channel rather than on what it built, and on 2026-08-11 it threw
        # away two cells whose agents had -- as far as anyone can now tell --
        # done the work: their stdout carried a prose summary because our own
        # launcher had eaten `--output-format json` (see
        # _resolve_cli_launcher), and the harness read "no JSON" as "no run".
        #
        # A missing result event is now a METADATA gap, recorded as such. The
        # deliverable on disk is graded either way. Publication is gated on
        # LIVENESS (did a session run at a real budget), not on parseability.
        stream_summary = meta.get("stream") or {}
        session_no_result = result is None
        if session_no_result:
            if meta.get("timed_out"):
                # BUDGET EXHAUSTION IS A RESULT (SPEC 2.4), so the cell must
                # still be graded and MUST land a row. Blocking here would
                # drop the cell out of the SCORED SET -- and since the cells
                # that time out are exactly the ones the agent was losing, a
                # simulator that runs out of time would score BETTER than one
                # that finishes and is wrong. Measured on the first v0.3
                # pilot: B2 timed out at 720 s and produced no row at all.
                report["budget_exhausted"] = True
                report["budget_exhausted_detail"] = (
                    "no result event: timed out after %ss (wall %ss)"
                    % (session_budget, meta.get("wall_s")))
            else:
                report["session_incomplete"] = True
                report["session_incomplete_detail"] = (
                    "the session exited without a result event (%s). Its "
                    "metadata is recovered from the stream where possible and "
                    "left null where not; the deliverable on disk is graded "
                    "as it stands." % (meta.get("launch_error")
                                       or "no result and no timeout"))
                print(report["session_incomplete_detail"])
            # No result event, so every field it would have carried stays null
            # rather than being invented. What the STREAM banked is real and
            # is used below.
            result = {}

        # 4. metrics ---------------------------------------------------------
        # session_id from the result event, else from the stream's init event
        # -- which exists from the session's first millisecond. That single
        # fallback is the difference between `session_id: null` + "transcript
        # not found" and a fully attributed row for a killed session.
        session_id = result.get("session_id") or stream_summary.get("session_id")
        transcript = find_transcript(session_id)
        # The session's own `~/.claude/projects/<slug>/` state -- memory above
        # all -- copied into the cell. It is state the agent produced, it lives
        # OUTSIDE the sandbox in the operator's real home, and until now no
        # cell report mentioned it existed (defect 9). Collection only: the
        # slug is derived from this cell's workspace, so the operator's own
        # project slugs are never read and never touched.
        try:
            report["session_home"] = collect_session_home(
                run_dir, workspace=ws, session_id=session_id)
        except Exception as exc:                       # noqa: BLE001
            report["session_home"] = {"error": repr(exc)}
        if transcript is not None:
            shutil.copy2(transcript, run_dir / "transcript.jsonl")
            tool_calls, tc_reason = count_tool_calls(transcript)
        elif stream_summary.get("tool_calls") is not None and \
                stream_summary.get("events"):
            tool_calls, tc_reason = stream_summary["tool_calls"], (
                "counted from the session STREAM (cc_stream.jsonl); the "
                "transcript under ~/.claude/projects was not found for "
                "session_id=%r" % session_id)
        else:
            tool_calls, tc_reason = None, (
                "session transcript not found under ~/.claude/projects for "
                "session_id=%r and the stream banked no events" % session_id)
        cc = cc_metrics_from_result(
            result, tool_calls=tool_calls, tool_calls_reason=tc_reason,
            version=pf["version"], permission_mode=meta["permission_mode"],
            transcript=transcript, cell_wall_s=meta["wall_s"],
            cli_command=meta["cli_command"])
        # Backfill from the stream ONLY where the result event said nothing.
        # Never overwrite a measured value with an inferred one.
        if cc.get("session_id") is None:
            cc["session_id"] = stream_summary.get("session_id")
        if cc.get("model") is None:
            cc["model"] = stream_summary.get("model")
        if cc.get("num_turns") is None and stream_summary.get("events"):
            cc["num_turns"] = stream_summary.get("assistant_turns")
        cc["stream"] = stream_summary
        report["cc_metrics"] = cc

        # LIVENESS -- computed here so it can be reported even on paths that
        # go on to block, and enforced after grading so the report still says
        # what the agent left behind.
        liveness = assess_liveness(
            meta=meta, stream=stream_summary, transcript=transcript,
            session_budget_s=round(session_budget, 1),
            task_budget_s=timeout_s, pinned_model=pinned,
            result_json=(None if session_no_result else result))
        report["liveness"] = liveness
        if not liveness["ok"]:
            print("LIVENESS FAILED: %s" % liveness["verdict"])

        # 5. artifact, per the task's deliverable convention -----------------
        final_text = cc.get("result_text") or ""
        art_dir = run_dir / "artifact"
        art_dir.mkdir(exist_ok=True)
        answer_path = None
        collected = None
        no_artifact = False
        ctrls, ctrls_missing = [], []
        driver_rec = None
        if task_id in ANSWER_TASKS or task_id in WORLD_PLUS_ANSWER_TASKS:
            # The session's final message IS (part of) the deliverable.
            answer_path = art_dir / "answer.txt"
            answer_path.write_text(final_text, encoding="utf-8")
            report["answer"] = {"collected": str(answer_path),
                                "chars": len(final_text)}

        if task_id in ANSWER_TASKS:
            # No world deliverable: the grader measures the task's own
            # pristine staged world; the artifact is the answer file.
            collected = answer_path
            report["artifact_rule"] = (
                "answer-deliverable task: the session's final message text "
                "(headless JSON result field) is the artifact; the graded "
                "world is the task's pristine staged copy")
            report["artifact"] = {"collected": str(collected)}
        else:
            artifact, rule = discover_artifact(
                ws, staged, start_ts,
                extra_roots=(run_dir / "repo_artifacts",),
                suffixes=artifact_suffixes(sim))
            report["artifact_rule"] = rule
            if artifact is None:
                # Nothing to grade at all. For a task that STAGES a world,
                # discover_artifact already falls back to the unchanged staged
                # copy, so this only fires for an authoring task starting from
                # an empty directory (A1) whose session produced no world --
                # which is SPEC 3.2's progress level 0, `no_artifact`, and
                # that is a FAIL.
                #
                # It must not block: blocking drops the cell out of the SCORED
                # SET, and a cell that produced nothing is exactly a
                # cell the agent lost -- so the grid would read better the
                # worse the agent did. Measured relevance: A1 is the flagship
                # and it scored 0/7 on this arm.
                #
                # No verdict is invented here. There is no artifact, so every
                # assertion is unsatisfiable and the outcome is FAIL by
                # construction; the row records the absence rather than a
                # measurement.
                report["no_artifact"] = True
                report["artifact_rule"] = (
                    "%s -- no deliverable exists, so the cell is scored FAIL "
                    "at progress 0 (SPEC 3.2 no_artifact) and still lands a "
                    "row" % rule)
                no_artifact = True
            else:
                collected = art_dir / (artifact_name(task_id, sim)
                                       or Path(artifact).name)
                # THIS is the only moment the authoring directory still
                # exists. The engine resolves URDFRobot urls against the world
                # file, so lifting the world out of the workspace breaks every
                # relative asset reference in it -- and the workspace is torn
                # down immediately after, taking the resolution context with
                # it. Measured on the first v0.3 pilot (A1/omnisim): the agent
                # wrote ten correct URDFRobot blocks pointing at
                # "../../../../robots/clearpath/.../husky.urdf", which resolved
                # through the workspace's projects/ junction; the collected
                # copy then re-based four levels into the results tree and
                # Phase B logged "Cannot open URDF file" ten times for
                # n_robots: 0. The agent's world was correct where it was
                # written; the collection broke it.
                rebased = []
                authored_in = _authoring_dir(artifact, run_dir)
                if Path(artifact).suffix.lower() in (".wbt", ".omniworld"):
                    try:
                        text = Path(artifact).read_text(encoding="utf-8",
                                                        errors="replace")
                        new_text, rebased = worldtext.rebase_relative_urls(
                            text, authored_in)
                    except OSError:
                        rebased = []
                if rebased:
                    collected.write_text(new_text, encoding="utf-8")
                else:
                    shutil.copy2(artifact, collected)
                # ...and the CONTROLLERS the world names. The deliverable of
                # an authoring task is a PROJECT: a world that says
                # controller "avoid_obstacles" is inert without the directory
                # beside it, and the workspace (and the repo copy the hygiene
                # sweep deleted) are both gone moments after this. Searched in
                # the agent's own project only -- where the world sits now
                # (workspace, or the sweep's mirror, which preserves the
                # repo-relative path and so the project shape) and where it
                # was originally written.
                authored_world = Path(authored_in) / Path(artifact).name
                if Path(artifact).suffix.lower() in (".wbt", ".omniworld"):
                    ctrls, ctrls_missing = collect_controllers(
                        artifact, art_dir,
                        search_roots=candidate_project_roots(artifact,
                                                             authored_world))
                # ...or, on an arm whose world file names no behaviour at all,
                # the DRIVER. Same defect class, different spelling: an MJCF
                # model has no controller field, so the program that steps it
                # is a separate file and the deliverable is a PAIR. Collecting
                # only the model would stage a scene that cannot move -- the
                # exact bug that zeroed R1 on the OmniSim arm.
                elif Path(artifact).suffix.lower() == ".xml":
                    driver_rec = collect_driver(artifact, collected)
                report["artifact"] = {
                    "found": str(artifact),
                    "collected": str(collected),
                    "rebased_urls": rebased or None,
                    "driver": driver_rec,
                    "controllers_referenced": (
                        controller_names(artifact)
                        if Path(artifact).suffix.lower() in (".wbt", ".omniworld") else []),
                    "collected_controllers": ctrls or None,
                    # Named by the world but NOT supplied by the agent's own
                    # project: they resolve (or do not) from the product
                    # install at phase B exactly as they did while the agent
                    # worked. Not an error on its own -- "void" lives in
                    # projects/default/controllers/.
                    "controllers_not_in_project": ctrls_missing or None,
                }

        # 5b. THE LIVENESS GATE ------------------------------------------
        #
        # A row is a claim about an AGENT. A session that never ran, or that
        # was starved of its budget by our own scheduler, supports no such
        # claim -- and the grid cannot tell the difference, because
        # `FAIL / no_artifact / progress 0` looks identical either way. That
        # is precisely how `r1_3arm_20260810` published a webots row for a
        # session killed 156.8 s into a 1800 s budget.
        #
        # So the cell BLOCKS rather than scoring: no row, an explicit
        # verdict, and -- unlike before -- the workspace and the collected
        # deliverable are already on disk for a human to open. The gate sits
        # AFTER collection and BEFORE grading so nothing is lost and no
        # engine launch is spent on a row that may not be published.
        #
        # Note the asymmetry, which is deliberate: a session that ran at a
        # real budget and produced nothing is an HONEST FAIL and is scored
        # (SPEC 3.2 no_artifact). Only the instrument's own failures block.
        if not liveness["ok"]:
            report["blocker"] = (
                "liveness assertion FAILED (%s): this cell's session cannot "
                "support a row about the agent, so it is abandoned rather "
                "than scored. Failed checks: %s. Session error: %s. The "
                "workspace is preserved at %s and the collected deliverable "
                "(if any) at %s."
                % (liveness["verdict"], ", ".join(liveness["vetoing"]),
                   meta.get("launch_error") or "(none recorded)",
                   run_dir / "workspace", art_dir))
            _write_report(run_dir, report)
            _preserve(ws)
            if not keep_workspace and ws is not None:
                _teardown_ws(ws, sim, report, run_dir)
            raise SystemExit(report["blocker"])

        # 5c. THE CONTAINMENT GATE ---------------------------------------
        #
        # The workspace filter removes the benchmark's own tree; it cannot
        # remove the checkout that tree lives in, and the first R4/omnisim
        # cell walked straight to it and read graders/r4_core.py, graders/
        # r4.py and the task's meta.json. A session holding the answer key
        # produces a score that is not a measurement, and -- before this gate
        # -- produced a report indistinguishable from a clean one.
        #
        # Same shape and same asymmetry as liveness: recorded on EVERY cell
        # (a clean run gets a measured green, not a silence), blocking rather
        # than scoring, evidence preserved, and placed before grading so no
        # engine time is spent on a row that may not be published.
        # Two layers, reported separately because they answer different
        # questions. The GUARD (prevention) answers "did the session stay
        # inside its workspace" and refused it in real time; the AUDIT
        # (detection) answers "did the answer key leak" from the session's own
        # record, and is what stands when the guard is off or evaded.
        # ...and re-assert the engine. A rebuild DURING a cell would leave the
        # session's observations and the grading pass on different binaries.
        if sim == "omnisim":
            report["engine_after_session"] = engine_identity()
            eb, ewhy = engine_pin_blocks(report["engine_after_session"],
                                         pin_engine_sha256)
            if eb:
                report["blocker"] = ewhy + " (the change happened DURING the "
                report["blocker"] += "cell: the session and the grading pass "
                report["blocker"] += "would be on different engines)"
                _write_report(run_dir, report)
                _preserve(ws)
                if not keep_workspace and ws is not None:
                    _teardown_ws(ws, sim, report, run_dir)
                raise SystemExit(report["blocker"])

        read_guard_log(guard)
        report["containment_guard"] = guard
        if guard.get("installed"):
            print("containment guard: saw %s tool call(s), denied %s"
                  % (guard.get("calls_seen"), guard.get("denied")))
        containment = audit_containment(
            (run_dir / "cc_stream.jsonl",
             find_transcript(cc.get("session_id"))),
            tool_calls=cc.get("tool_calls"))
        report["containment"] = containment
        blocked, why = containment_blocks(containment)
        if not blocked:
            blocked, why = guard_blocks(guard, tool_calls=cc.get("tool_calls"))
        if blocked:
            report["blocker"] = why
            print(report["blocker"])
            _write_report(run_dir, report)
            _preserve(ws)
            if not keep_workspace and ws is not None:
                _teardown_ws(ws, sim, report, run_dir)
            raise SystemExit(report["blocker"])

        # Teardown is deliberately DEFERRED to a postscript after the row is
        # on disk (2026-08-01 crash: teardown raised on a locked log file and
        # the finalized-after-teardown row was lost with the quota spent).
        # Grading cannot read the session's scene either way: it consumes
        # only the COLLECTED copies under run_dir (grade_webots re-copies
        # into its own scratch; the omnisim runner stages pristine worlds).

        # 6. grade (the grading/recorder pass ALWAYS holds an engine slot) ---
        if use_locks and sim != "omnisim":
            engine.acquire("%s cell %s: grading pass" % (sim, task_id),
                           on_wait=lambda p: print(
                               "waiting for a free engine slot (grading)..."))
        _sample("grade_start")
        notes = ("claude-code %s model=%s permission_mode=%s"
                 % (pf["version"], pinned, meta["permission_mode"]))
        # The grading PROJECT root, identical on both arms. The world each arm
        # launches sits under <grade_project>/worlds/..., so the engine's own
        # root walk (project_root_for_world) lands here and finds
        # <grade_project>/controllers/<name> -- the directories collected above.
        grade_project = run_dir / "grade"
        stage_controllers(art_dir, grade_project, report)
        # ...and the files the LANE published into the agent's workspace, at
        # the same relative paths, copied from the frozen task tree. A
        # deliverable that reads one of them at run time (R4's first webots
        # cell did, and died with FileNotFoundError at __init__ -- 0.00 m over
        # 150 s on a world that was otherwise exact) must resolve it here
        # exactly as it did while the agent worked. See stage_task_assets for
        # the safety rule: the bytes come from the repo, never the workspace.
        assets = stage_task_assets(task_id, sim, grade_project, report)
        if assets["files"]:
            print("staged %d published task asset(s) into the graded "
                  "project: %s" % (len(assets["files"]),
                                   ", ".join(f["path"]
                                             for f in assets["files"])))

        # 6a. GRADE-TIME OBSTACLE PLACEMENT -- R1's anti-hardcode mechanism.
        #
        # WHERE THIS SITS, AND WHY IT IS HERE AND NOWHERE ELSE. The agent's
        # session ended at step 3 (`run_claude_cell` returned); step 4 swept
        # its stragglers, its ports and anything it wrote through the
        # projects/ junction into the repo; step 5 COPIED its deliverable out
        # of the workspace into `run_dir/artifact/`. Only then does the layout
        # get drawn -- and it is drawn from a seed that lives in this process,
        # placed into the COLLECTED copy under the results tree, and declared
        # in `run_dir` and the grading project. The agent's workspace is never
        # touched (it is torn down at step 8 with its own copy unchanged), the
        # seed never entered the child environment (`SCRUB_PREFIXES` drops
        # every `AGENTBENCH_*` name), and the whole benchmark tree is excluded
        # from the staged workspace in the first place
        # (`stage_workspaces.OMNISIM_EXCLUDE_PREFIXES`). So there is no file,
        # no variable and no moment at which the drawn layout is reachable by
        # a session -- it does not exist until the session is over.
        #
        # It runs BEFORE the graders because each of them re-stages the
        # collected artifact into its own scratch, so an unplaced artifact
        # here is an unplaced world there.
        placement = None
        if task_id in r1_placement.TASKS and not no_artifact:
            try:
                if collected is None:
                    raise r1_placement.PlacementError(
                        "no deliverable was collected, so the drawn layout "
                        "could not be placed")
                placement = r1_placement.place_and_declare(
                    collected, seed=layout_seed,
                    # `run_dir` is the grader's run dir on the webots and
                    # mujoco arms (both are handed `grade_project`), and the
                    # forensic copy on every arm; `grade_project` is what the
                    # in-process graders actually search. The OmniSim arm's
                    # subprocess is told the directory (see grade_omnisim).
                    declare_dirs=(run_dir, grade_project))
                placement["seed_source"] = layout_seed_source
                placement["ok"] = True
                report["r1_placement"] = placement
                print("R1 grade-time placement: seed=%s, %d obstacles moved "
                      "(max %.2f m)"
                      % (layout_seed, len(placement["obstacles"]),
                         max(o["moved_m"] for o in placement["obstacles"])))
            except r1_placement.PlacementError as exc:
                # NO FALLBACK. Grading an unplaced R1 world scores the layout
                # the task PUBLISHES -- the layout a memorising controller
                # read out of benchmark_assets/obstacles.json -- and that row
                # is indistinguishable from a placed one while being worth
                # nothing. So the cell blocks, loudly, and lands no row.
                report["r1_placement"] = dict(exc.report or {},
                                              seed=layout_seed,
                                              seed_source=layout_seed_source,
                                              ok=False, error=str(exc))
                report["blocker"] = (
                    "R1 grade-time obstacle placement failed, so this cell "
                    "was NOT graded: %s. Scoring it would score the PUBLISHED "
                    "layout, which a memorising agent passes 6/6 (measured), "
                    "and the row would look exactly like one that proves "
                    "perception." % exc)
                _write_report(run_dir, report)
                if not keep_workspace and ws is not None:
                    _teardown_ws(ws, sim, report, run_dir)
                raise SystemExit(report["blocker"])

        if no_artifact:
            # Nothing exists to measure, so there is nothing to run a grader
            # on. The row records the ABSENCE -- every assertion unsatisfiable,
            # outcome FAIL -- rather than a fabricated measurement, and it
            # lands in the denominator where it belongs.
            row = no_artifact_row(sim, task_id, report["artifact_rule"])
        elif sim == "omnisim":
            # --out is <grade_project>/worlds, NOT <grade_project>: run_agentbench
            # builds <out>/<task>.<agent>.r<n>/scratch/ and stages the world
            # there, so the "worlds" component is what makes the engine resolve
            # <grade_project> as the project root (and it must be a component of
            # the SCRATCH path, not a sibling of it). run_agentbench rmtree's
            # only its own per-cell dir, so <grade_project>/controllers/ -- a
            # sibling of "worlds" -- survives that.
            row = grade_omnisim(task_id, collected, grade_project / "worlds",
                                notes,
                                answer_path=(answer_path
                                             if task_id in
                                             WORLD_PLUS_ANSWER_TASKS
                                             else None),
                                layout_dir=(run_dir if placement else None))
        elif sim == "mujoco":
            row = grade_mujoco(task_id, collected, grade_project,
                               answer=(final_text
                                       if task_id not in NO_ANSWER_GRADERS
                                       else ""))
        else:
            row = grade_webots(
                task_id,
                None if task_id in ANSWER_TASKS else collected,
                grade_project,
                answer=(final_text if task_id not in NO_ANSWER_GRADERS
                        else ""),
                project_assets=[f["path"] for f in assets["files"]])
        _sample("grade_end")
        # The grading pass spins up private harness instances of its own;
        # if one wedged on a canonical port, the next cell must not meet it.
        _sweep_ports(report, "post_grade", mine=my_claim, others=_others())
        (run_dir / "grader_row.json").write_text(
            json.dumps(row, indent=2, default=str), encoding="utf-8")
    finally:
        # UNCONDITIONAL. Covers the paths the explicit calls cannot: an
        # exception out of staging, discovery, placement or a grader. Nothing
        # between here and teardown may be allowed to lose the evidence.
        try:
            _preserve(ws)
        except Exception as exc:                   # noqa: BLE001
            print("workspace preservation failed: %r" % (exc,))
        if use_locks:
            engine.release()
            tlock.release()
        # The claim goes LAST: while it stands, another lane's sweeps spare
        # everything of ours, and dropping it early re-opens the window this
        # whole mechanism exists to close.
        concurrency.unregister_cell(claim_path)

    # 7. merge + write -- the row and cell report are fully written and
    # FLUSHED before any teardown is attempted (row-before-teardown rule;
    # the teardown result is a best-effort postscript, never part of the
    # row) ------------------------------------------------------------------
    merged = merge_cc_metrics(row, cc)
    merged["repeat"] = repeat
    # THE MEASUREMENT PROTOCOL, on the row, in the row's own words.
    #
    # This block exists because of what n = 1 makes possible to misread. A row
    # says PASS or FAIL; nothing in it, before this, said how many times the
    # cell was run. Under the old protocol the answer was "5, and the report
    # aggregates them"; under this one it is "once". A single PASS beside a
    # column header reading `pass@1` is a rate that was never measured, and
    # the arithmetic is not merely imprecise -- it is undefined: a fraction of
    # one observation has no confidence interval, and its variance is
    # UNMEASURED, which is a different claim from "small".
    #
    # So the row carries the sample count explicitly, says in words that it is
    # not a rate, and carries the protocol id that fences it off from rows
    # scored under a different ceiling. A consumer that aggregates must read
    # `samples`; one that finds `variance_measured: false` may not print a CI.
    merged["protocol"] = {
        "id": PROTOCOL_ID,
        "runs_per_cell": runs_per_cell,
        "samples": 1,
        "variance_measured": False,
        "is_rate": False,
        "hard_ceiling_s": tasks_mod.TASK_HARD_CEILING_S,
        "task_budget_s": task_budget_s,
        "cell_wall_bound_s": cell_wall_s,
        "session_budgets_s": report["budget"]["session_budgets_s"],
        "session_budget_curtailed":
            report["budget"]["session_budget_curtailed"],
        # Time this cell spent QUEUED behind another lane, which is excluded
        # from its wall bound (it is the scheduler's cost, not the agent's).
        "queue_waits": report["budget"].get("queue_waits") or [],
        "note": (
            "ONE observation of this (task, arm). It is an OUTCOME, not a "
            "rate: pass@1, a pass fraction and a confidence interval are "
            "UNDEFINED at one sample, and variance here is unmeasured rather "
            "than small. Quote it as 'passed/failed once, in <t> s, under a "
            "%ds ceiling'. Rows carrying a different protocol id -- or no "
            "protocol block at all, which is everything scored before "
            "2026-08-10 -- ran under a different wall-clock ceiling and a "
            "different repeat count; TASK_HARD_CEILING_S is global, so they "
            "may never be pooled, averaged or placed in one column."
            % tasks_mod.TASK_HARD_CEILING_S),
    }
    merged["agent_artifacts"]["staging_manifest_sha"] = \
        inst_manifest.get("filelist_sha256")
    merged["agent_artifacts"]["known_disclosures"] = \
        inst_manifest.get("known_disclosures", [])
    merged["agent_artifacts"]["staging_redactions"] = \
        len(inst_manifest.get("redactions") or [])
    # Concurrency attribution (plan §2.7): a row produced while another lane
    # was active carries the flag; the report side excludes its time columns
    # from any latency statement (plan §2.3 note; f_eval implements it).
    under_concurrency = any(s["others_active"] for s in samples)
    # Port hygiene is part of the row's provenance: a cell whose pre-session
    # sweep KILLED something started in a contaminated environment-candidate
    # state, and a forensic reader must not have to dig the cell report out
    # to know it. Empty kill lists are recorded too (the clean witness).
    merged["agent_artifacts"]["port_hygiene"] = [
        {"when": s.get("when"), "kills": s.get("kills")}
        for s in report.get("process_sweeps", []) if s.get("kind") == "port"]
    merged["agent_artifacts"]["measured_under_concurrency"] = under_concurrency
    # Why this cell ended where it did. A budget-exhausted or empty-handed
    # cell is a FAIL that COUNTS (SPEC 2.4 / 3.2), so the row says so plainly
    # rather than leaving a reader to infer it from null metrics.
    merged["agent_artifacts"]["budget_exhausted"] = bool(
        report.get("budget_exhausted"))
    merged["agent_artifacts"]["no_artifact"] = bool(report.get("no_artifact"))
    # WHETHER A SESSION DEMONSTRABLY RAN, with the criteria attached so a
    # reader re-checks rather than trusts. Present on every row and never
    # absent: a row with no liveness block is one scored before 2026-08-11,
    # when "the agent worked for 30 minutes and built nothing" and "our
    # scheduler killed it at 8.7% of its budget" were the same row.
    merged["agent_artifacts"]["liveness"] = report.get("liveness")
    # ...and whether the session reached its own end. A row whose agent
    # metrics are null because the result event never arrived says so here,
    # instead of looking like a session that cost nothing.
    merged["agent_artifacts"]["session_incomplete"] = bool(
        report.get("session_incomplete"))
    merged["agent_artifacts"]["session_stream"] = (
        cc.get("stream") if isinstance(cc, dict) else None)
    # WHERE THE EVIDENCE IS. The row names the preserved workspace, so a
    # reader never has to hope a %TEMP% path from the report still exists.
    merged["agent_artifacts"]["workspace_preserved"] = {
        "path": str(run_dir / "workspace"),
        "manifest": (report.get("workspace_preserved") or {}),
    }
    # The deliverable of an authoring task is a PROJECT, so the row says what
    # came with the world instead of leaving a reader to assume it did.
    # Measured need (R1/omnisim, 2026-08-09): only the .wbt was collected, the
    # robot got no controller and never moved -- path length 0.0 m, two
    # assertions failed on OUR bug and nothing in the row showed why.
    merged["agent_artifacts"]["collected_controllers"] = [
        c.get("name") for c in ctrls]
    merged["agent_artifacts"]["controllers_not_in_project"] = \
        list(ctrls_missing)
    # Same fact on an arm whose model names no behaviour: WITH what program
    # was this model stepped, and by which rule was it chosen. A pair graded
    # without its driver is a scene nobody stepped, and that must be readable
    # from the row rather than inferred from zero motion.
    merged["agent_artifacts"]["collected_driver"] = driver_rec
    # R1's anti-hardcode mechanism, on the row rather than only in the cell
    # report: WHICH layout this run was graded against, the seed it was drawn
    # from and where that seed came from. Without the seed on the row the
    # layout is unre-derivable and "the obstacles were moved" is a claim; with
    # it, `r1_core.sample_layout(seed)` reproduces the exact five poses on any
    # machine. `null` on every task but R1, and never absent -- a row with no
    # placement key at all would be indistinguishable from one where the
    # mechanism silently did not run.
    merged["agent_artifacts"]["r1_placement"] = (
        {k: placement.get(k) for k in
         ("ok", "seed", "seed_source", "mechanism", "format", "obstacles",
          "layout", "verification", "sidecars", "notes", "draw")}
        if placement else None)
    merged["agent_artifacts"]["layout_seed"] = (
        layout_seed if task_id in r1_placement.TASKS else None)
    # Whether this cell's engine launches were actually windowless. A cell
    # with enforced=False may have rendered a GUI, so its wall clock is not
    # comparable -- same rule as measured_under_concurrency.
    merged["agent_artifacts"]["headless"] = report.get("headless")
    merged["agent_artifacts"]["concurrency"] = {
        "lane": lane, "engine_slots": engine_slots,
        "locks_used": bool(use_locks), "samples": samples}
    if deferrals:
        merged["agent_artifacts"]["rate_limit_deferrals"] = deferrals
    report["measured_under_concurrency"] = under_concurrency
    with open(run_dir / "rows.jsonl", "a", encoding="utf-8") as fh:
        fh.write(json.dumps(merged, default=str) + "\n")
        fh.flush()
        os.fsync(fh.fileno())
    report["row"] = {k: merged.get(k) for k in
                     ("task", "sim", "condition", "outcome",
                      "failed_assertion")}
    report["row_metrics"] = merged.get("metrics")
    report["utc_end"] = _utc()
    _write_report(run_dir, report)

    # 8. teardown postscript (AFTER the row; sweeps stragglers first; a
    # locked temp file marks the dir *.pending_delete and the cell still
    # returns its row) -------------------------------------------------------
    if keep_workspace:
        _sweep_processes(ws, sim, report, "post_grade")
        _write_report(run_dir, report)
    else:
        _teardown_ws(ws, sim, report, run_dir)
    return merged


def _write_report(run_dir, report):
    run_dir = Path(run_dir)
    (run_dir / "cell_report.json").write_text(
        json.dumps(report, indent=2, default=str), encoding="utf-8")
    lines = ["# Claude Code cell report", ""]
    for k in ("sim", "task", "condition", "pinned_model", "workspace",
              "prompt", "artifact_rule", "blocker"):
        if k in report:
            lines.append("- **%s**: %s" % (k, report[k]))
    pf = report.get("preflight") or {}
    lines.append("- **claude_code_version**: %s" % pf.get("version"))
    lines.append("- **claude_launcher**: %s" % json.dumps(
        pf.get("launcher"), default=str))
    # THE EVIDENCE POINTER. The first thing a human reading this file wants
    # is "where is what the agent actually built" -- and for the campaign
    # this machinery was written for, the answer was a deleted %TEMP% path.
    wsp = report.get("workspace_preserved") or {}
    lines.append("- **workspace (preserved, open this)**: %s -- %s files, "
                 "%s bytes%s"
                 % (wsp.get("dest"), wsp.get("files"), wsp.get("bytes"),
                    " [TRUNCATED at %s]" % wsp.get("truncated_at")
                    if wsp.get("truncated") else ""))
    if wsp.get("excluded_links"):
        lines.append("  - links not followed: %s"
                     % ", ".join(x["path"] for x in wsp["excluded_links"]))
    live = report.get("liveness") or {}
    if live:
        lines.append("- **liveness**: %s -- %s"
                     % ("OK" if live.get("ok") else "FAILED",
                        live.get("verdict")))
    st = (report.get("cc_metrics") or {}).get("stream") or {}
    if st:
        lines.append("- **session**: model=%s turns=%s tool_calls=%s "
                     "events=%s result_event=%s"
                     % (st.get("model"), st.get("assistant_turns"),
                        st.get("tool_calls"), st.get("events"),
                        st.get("reached_result_event")))
    place = report.get("r1_placement")
    if place:
        lines.append(
            "- **r1_placement**: %s -- seed `%s` (%s), %d obstacle(s) moved%s"
            % ("PLACED" if place.get("ok") else "FAILED",
               place.get("seed"), place.get("seed_source"),
               len(place.get("obstacles") or []),
               "" if place.get("ok") else " -- %s" % place.get("error")))
    head = report.get("headless") or {}
    if head:
        lines.append("- **headless**: enforced=%s via %s%s"
                     % (head.get("enforced"), head.get("mechanism"),
                        "" if head.get("enforced")
                        else " -- WALL CLOCK NOT COMPARABLE (%s)"
                             % head.get("detail")))
    art = report.get("artifact") or {}
    if art.get("collected_controllers"):
        lines.append("- **collected_controllers**: %s"
                     % ", ".join(c.get("name") for c in
                                 art["collected_controllers"]))
    row = report.get("row")
    if row:
        lines.append("")
        lines.append("## Row")
        for k, v in row.items():
            lines.append("- %s: %s" % (k, v))
        lines.append("- metrics: %s" % json.dumps(
            report.get("row_metrics"), default=str))
    (run_dir / "cell_report.md").write_text("\n".join(lines) + "\n",
                                            encoding="utf-8")


def main(argv=None):
    # --status is a READ-ONLY view over a cell dir -- live or finished -- and
    # is parsed before anything else so it needs none of the run arguments
    # (`--sim` is required for a run and meaningless for a look).
    #
    #     python run_cc_cell.py --status <cell-dir> [--json]
    #
    # It opens no locks, sends no signals and writes nothing, so it is safe
    # against a session that is still working.
    if argv is None:
        argv = sys.argv[1:]
    argv = list(argv)
    if "--status" in argv:
        i = argv.index("--status")
        try:
            cell_dir = argv[i + 1]
        except IndexError:
            print("--status needs a cell directory", file=sys.stderr)
            return 2
        as_json = "--json" in argv
        if as_json:
            doc = json.dumps(evidence.cell_status(cell_dir), indent=2,
                             default=str)
            try:
                sys.stdout.write(doc + "\n")
            except UnicodeEncodeError:                # see print_status
                enc = sys.stdout.encoding or "utf-8"
                sys.stdout.write(doc.encode(enc, "replace")
                                 .decode(enc, "replace") + "\n")
        else:
            evidence.print_status(cell_dir)
        return 0

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    # Every comparator in the frozen design is accepted here; a DECLARED one
    # fails with sims.require_implemented's message naming its blocker, so
    # "not built yet" never masquerades as "not part of the benchmark".
    ap.add_argument("--sim", required=True, choices=sims.PRIMARY + sims.EXTENDED,
                    metavar="SIM",
                    help="comparator to run (implemented: %s; declared: %s)"
                         % (", ".join(sims.IMPLEMENTED),
                            ", ".join(sims.DECLARED)))
    ap.add_argument("--task", default="C2_fall_through_floor")
    ap.add_argument("--root", default=str(staging.DEFAULT_ROOT),
                    help="staging scratch root (default %(default)s)")
    ap.add_argument("--out", default=None,
                    help="cell run dir (default results/cc_lane/<utc>_<sim>)")
    ap.add_argument("--model", default=None,
                    help="pin a model id (default: %s -- the benchmark model; "
                         "NOT inherited from the local CLI default)"
                         % DEFAULT_MODEL)
    ap.add_argument("--timeout-s", type=float, default=DEFAULT_TIMEOUT_S,
                    help="wall-clock ceiling in seconds (default %(default)s "
                         "= the campaign hard ceiling). The effective budget "
                         "is min(this, the task's own timeout_s): this flag "
                         "can tighten a task's budget, never loosen it.")
    ap.add_argument("--keep-workspace", action="store_true")
    ap.add_argument("--lane", default=None,
                    help="lane label recorded in locks/rows (default: the "
                         "--sim value; lane A = omnisim cells, lane B = "
                         "webots cells)")
    ap.add_argument("--engine-slots", type=int,
                    default=concurrency.DEFAULT_ENGINE_SLOTS,
                    help="max concurrent engine-heavy phases on this machine "
                         "(global file-lock semaphore; default %(default)s)")
    ap.add_argument("--lock-root", default=None,
                    help="shared lock dir (default <root>/locks -- must be "
                         "the SAME for every lane on this machine)")
    ap.add_argument("--no-locks", action="store_true",
                    help="skip the concurrency locks (single-lane debugging "
                         "only; never during a campaign)")
    ap.add_argument("--rate-limit-backoff-s", type=float,
                    default=DEFAULT_RATE_LIMIT_BACKOFF_S,
                    help="wait between deferred (usage/rate-limited) "
                         "attempts; default %(default)s s")
    ap.add_argument("--max-rate-limit-retries", type=int,
                    default=DEFAULT_MAX_RATE_LIMIT_RETRIES)
    ap.add_argument("--layout-seed", default=None,
                    help="R1 only: the seed its graded obstacle layout is "
                         "drawn from (default <campaign>/<task>/r<repeat>; "
                         "$AGENTBENCH_R1_LAYOUT_SEED is read in between). Any "
                         "string; the same seed always gives the same layout, "
                         "and the seed used is recorded on the row.")
    ap.add_argument("--pin-engine-sha256", default=None,
                    help="refuse to run unless msys64/mingw64/bin/"
                         "omnisim-bin.exe has this sha256, and refuse to "
                         "grade if it changes mid-cell. Every cell in a round "
                         "must run the identical engine or its observations "
                         "are not comparable to the other cells.")
    ap.add_argument("--no-containment-guard", action="store_true",
                    help="do NOT install the PreToolUse containment hook. "
                         "The session can then read the real repo checkout, "
                         "including the graders and the task metas, and the "
                         "post-hoc audit will block the cell after the quota "
                         "is spent. Debugging only.")
    ap.add_argument("--status", metavar="CELL_DIR",
                    help="read-only status of a live or finished cell dir "
                         "(elapsed vs budget, turns, last tool calls, files "
                         "appearing in the preserved workspace). Disturbs "
                         "nothing. Add --json for the machine-readable form.")
    ap.add_argument("--json", action="store_true",
                    help="with --status: emit JSON instead of the text block")
    args = ap.parse_args(argv)

    # ALWAYS absolute: the webots launcher's WSL path translation refuses
    # relative paths (measured in the first webots smoke cell -- a relative
    # --out made win_to_wsl_path error out and the grading launch never ran).
    out = (Path(args.out) if args.out else (
        AGENTBENCH / "results" / "cc_lane"
        / ("%s_%s" % (time.strftime("%Y%m%d_%H%M%S"), args.sim)))).resolve()
    row = run_cell(args.sim, args.task, root=Path(args.root), out_dir=out,
                   model=args.model, timeout_s=args.timeout_s,
                   keep_workspace=args.keep_workspace,
                   lane=args.lane, engine_slots=args.engine_slots,
                   lock_root=args.lock_root, use_locks=not args.no_locks,
                   rate_limit_backoff_s=args.rate_limit_backoff_s,
                   max_rate_limit_retries=args.max_rate_limit_retries,
                   layout_seed=args.layout_seed,
                   guard_enabled=not args.no_containment_guard,
                   pin_engine_sha256=args.pin_engine_sha256)
    print("\n=== %s / %s / %s ===" % (row["task"], row["sim"],
                                      row["condition"]))
    print("outcome=%s failed=%s" % (row["outcome"],
                                    row.get("failed_assertion")))
    print("metrics=%s" % json.dumps(row["metrics"], default=str))
    print("rows -> %s" % (out / "rows.jsonl"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
