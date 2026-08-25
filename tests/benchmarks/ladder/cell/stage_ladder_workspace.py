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

"""Stage one **(tier, column)** workspace for a capability-ladder cell.

    python tests/benchmarks/ladder/cell/stage_ladder_workspace.py \\
        --column omnisim --task T1_arrive --verify

``capability-ladder-plan.md`` §4.1: *"Staging reuses
``cc_lane/stage_workspaces.py`` unchanged."* This module is that reuse plus
exactly two ladder-specific things, and it forks nothing:

**1. A bigger quarantine.** ``cc_lane`` removes the AgentBench answer key. A
ladder workspace must additionally remove the LADDER's answer key, because an
agent that can read it can read the numbers it is graded against and the
scripted oracle that already solves the task:

===============================================  =============================
removed                                           what it would have leaked
===============================================  =============================
``tests/benchmarks/ladder/**``                    every grader and core (the
                                                  thresholds), every negative
                                                  fixture, every ``meta.json``
                                                  (the constants, the
                                                  pre-registered expectation,
                                                  the demonstrated recipe),
                                                  and the committed SCRIPTED
                                                  ORACLES -- which solve the
                                                  tiers outright
``docs/developer/capability-ladder-plan.md``      the tier statements, the
                                                  thresholds, the blocker
                                                  taxonomy and §6.4's
                                                  expectation table
``docs/developer/g1-endurance-2026-08-01.md``     names the ladder, quotes
                                                  T4's own two-cell bounds and
                                                  the arena rule they came
                                                  from (§8 of that file is
                                                  literally "What this means
                                                  for the ladder's T4 bar")
everything ``cc_lane`` already removes            the AgentBench answer key
(``tests/benchmarks/agentbench/**``, the          and the private trees
plan/baseline/latency docs, ``CHANGELOG.md``,
``social/``, ``cloud/``, the roadmap)
===============================================  =============================

Excluded rather than doctored, for the reason ``cc_lane`` excluded
``CHANGELOG.md``: neither doc is on any required include list, and a
half-redacted threshold table is worse than an absent one. The **cost** of
excluding the endurance document is recorded in the manifest rather than
hidden -- a T4 agent on our own column cannot read our own measured humanoid
result. The redaction pass then runs as a **net**, not as the mechanism:
``cc_lane``'s own ruling, with the ladder's self-reference patterns appended,
over every remaining ``.md``.

**2. Columns that are not a product clone.** The ladder roster is five
simulators, not two (``capability-ladder-plan.md`` §4). A column whose product
is a pip wheel or a container image gets the **bare** template: an empty
workspace plus the tier's ``container/``, and not one file authored by us --
the same rule the ``webots`` arm has always had, and for the same reason (our
text in the workspace contaminates the product comparison).

What the workspace ships of the task
------------------------------------

**The tier's ``container/``, exactly the files ``meta.json`` declares, and
nothing else task-related.** Layout is preserved (T1's URDF resolves
``package://husky_description/...`` against ``container/`` as the package
root, and rewriting that would do one of the measured steps for the agent).
A file present in ``container/`` but **not** declared is NOT staged and is
reported -- the declaration is the contract.

The **prompt is returned, never planted as a file**: it is passed to
``claude -p`` verbatim and is byte-identical on every column, and a file in
the workspace would be a second, non-identical delivery channel.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import re
import shutil
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent                  # .../ladder/cell
LADDER = HERE.parent                                    # .../ladder
BENCHMARKS = LADDER.parent                              # .../tests/benchmarks
REPO = BENCHMARKS.parents[1]

if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench.cc_lane import stage_workspaces as cc   # noqa: E402
from ladder import tasks as ladder_tasks                # noqa: E402

DEFAULT_ROOT = Path(os.environ.get("TEMP") or os.environ.get("TMP")
                    or "/tmp") / "ladder_cell"

#: Columns whose workspace IS the product clone. Everything else is bare.
PRODUCT_CLONE_COLUMNS = ("omnisim",)
#: Columns staged as a link to a verified install (``cc_lane`` owns the recipe).
LINKED_INSTALL_COLUMNS = ("webots",)
#: Roster columns (``capability-ladder-plan.md`` §4). ``genesis`` is
#: deliberately absent from pass 1 and its absence is stated in every grid.
KNOWN_COLUMNS = ("omnisim", "webots", "mujoco", "gazebo", "isaac")

# --- the ladder's own quarantine ---------------------------------------------

LADDER_EXCLUDE_PREFIXES = cc.OMNISIM_EXCLUDE_PREFIXES + (
    # The ladder itself: graders + cores (every threshold), negative fixtures,
    # task meta.json (constants, pre-registered expectations, the demonstrated
    # recipes), the containers, the adapters, and the committed SCRIPTED
    # ORACLES that solve T1/T2/T3 outright.
    "tests/benchmarks/ladder/",
)
LADDER_EXCLUDE_FILES = cc.OMNISIM_EXCLUDE_FILES + (
    # The plan IS the answer key: tier statements, thresholds, the blocker
    # taxonomy, and §6.4's pre-registered expectation table.
    "docs/developer/capability-ladder-plan.md",
    # Names the ladder in four places, quotes T4's two-cell force/torque
    # bounds, and its §8 is "What this means for the ladder's T4 bar". A
    # product document that also carries the answer key is still the answer
    # key. COST, recorded rather than hidden: a T4 agent on our own column
    # cannot read our own measured humanoid endurance result.
    "docs/developer/g1-endurance-2026-08-01.md",
)

#: Appended to ``cc_lane``'s own patterns for the staged-doc redaction pass --
#: the NET behind the exclusions above, for a self-reference that lands in a
#: product doc later. Deliberately narrow: bare ``T1``/``T4`` are ordinary
#: words in this tree (Isaac T4 instances, tier-1 support, torque units).
LADDER_REDACT_PATTERNS = tuple(re.compile(p) for p in (
    r"\bT[1-4]_(?:arrive|transfer|quadruped|humanoid)\b",
    r"capability[\s_-]?ladder",
    r"\bladder[./]graders\b",
    r"\bT4-(?:un)?supported\b",
))

#: Path prefixes that must NEVER appear inside a staged workspace. A hit is
#: not a warning: the cell is INVALID (``__init__``'s rule 3).
QUARANTINE_PATH_PREFIXES = ("tests/benchmarks/ladder/",
                            "tests/benchmarks/agentbench/")
QUARANTINE_PATH_FILES = ("docs/developer/capability-ladder-plan.md",
                         "docs/developer/g1-endurance-2026-08-01.md")

#: Content markers. A staged text file containing one of these is carrying
#: ladder answer key whatever its path says. Threshold CONSTANT NAMES are in
#: the list because a leaked constant is a leaked bar.
#:
#: ⚠ **The constant markers are CASE-SENSITIVE, and that is a measured fix,
#: not a style choice.** Compiled case-insensitively they matched
#: ``net_displacement_m`` and ``arrive_tol_m`` in
#: ``tests/benchmarks/warehouse/`` -- an unrelated, older tool whose columns
#: are ordinary physical quantities with ordinary names -- and every omnisim
#: cell would have been declared INVALID on four false positives before a
#: single session ran. The ladder writes its bars in SCREAMING_SNAKE and
#: nothing else in this tree does, so the case IS the discriminator. Verified
#: against the whole tracked tree: zero hits outside ``ladder/``.
_MARKERS_ANY_CASE = (
    r"capability[\s_-]?ladder",
    r"\bT[1-4]_(?:arrive|transfer|quadruped|humanoid)\b",
    r"ladder[./]graders",
    r"tests[/\\]benchmarks[/\\]ladder",
)
_MARKERS_EXACT_CASE = (
    r"\bARRIVE_TOL_M\b", r"\bMIN_MEAN_SPEED_MPS\b",
    r"\bMAX_SUPPORT_FORCE_FRACTION\b", r"\bNET_DISPLACEMENT_M\b",
    r"\bMIN_CONTACT_TRANSITIONS\b", r"\bMIN_RUN_UP_FACTOR\b",
    r"\bFALL_Z_FRACTION\b", r"\bMAX_SUPPORT_TORQUE_NM\b",
)
ANSWER_KEY_MARKERS = (
    tuple(re.compile(p, re.IGNORECASE) for p in _MARKERS_ANY_CASE)
    + tuple(re.compile(p) for p in _MARKERS_EXACT_CASE))

#: Only these are scanned for content (a mesh cannot leak a threshold, and
#: walking 200 MB of binaries per cell would cost more than the cell).
TEXT_SUFFIXES = (".md", ".txt", ".rst", ".json", ".py", ".yml", ".yaml",
                 ".cfg", ".ini", ".html", ".wbt", ".proto", ".urdf", ".xml")
MAX_SCAN_BYTES = 4 * 1024 * 1024

KNOWN_DISCLOSURES = [
    "The product clone SHIPS THE PRODUCT, and on the omnisim column the "
    "product includes shipped worlds, the skill library and trained "
    "champions reachable through the projects/ junction. That is not a leak "
    "and it is not removed: composing shipped assets is exactly what "
    "reuse_class=assembled measures (capability-ladder-plan.md 3.2), and "
    "removing them would measure a product nobody ships.",
    "docs/developer/g1-endurance-2026-08-01.md is EXCLUDED as ladder answer "
    "key (it names the ladder, quotes T4's two-cell bounds and states 'what "
    "this means for the ladder's T4 bar'). The cost is that a T4 agent on "
    "our own column cannot read our own measured humanoid endurance result; "
    "recorded rather than hidden.",
    "READ-THROUGH RISK, inherited from cc_lane and NOT eliminated: the "
    "omnisim workspace reaches the real tree through directory junctions "
    "(msys64/, projects/, lib/, resources/). tests/ is NOT junctioned, so "
    "the ladder's graders are not reachable by walking the workspace -- but "
    "a tool that resolves '<ws>/projects/..' PHYSICALLY rather than "
    "lexically lands in the real repo. The countermeasure is detection, not "
    "prevention: every cell scans its own session transcript for ladder "
    "answer-key markers and a hit makes the cell INVALID.",
]


def _utc():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


# --- file selection (pure; testable with no git and no 200 MB repo) ----------


def select_ladder_files(file_list, *, junction_dirs=cc.OMNISIM_JUNCTION_DIRS,
                        exclude_prefixes=LADDER_EXCLUDE_PREFIXES,
                        exclude_files=LADDER_EXCLUDE_FILES):
    """``(copied, excluded)`` for the product-clone column.

    A thin call into ``cc_lane``'s own splitter with the ladder's bigger
    lists -- the rule lives there and is not restated here.
    """
    return cc.select_omnisim_files(file_list, junction_dirs=junction_dirs,
                                   exclude_prefixes=exclude_prefixes,
                                   exclude_files=exclude_files)


@contextlib.contextmanager
def ladder_redaction():
    """Run ``cc_lane``'s redaction pass with the ladder's patterns appended.

    The pass is a pure function of the text plus one module-level pattern
    tuple, so extending it is a scoped swap of that tuple -- not a copy of
    the 120-line block-wise redactor, which would then drift from the ruling
    it implements. Restored on the way out, always.
    """
    old = cc.REDACT_PATTERNS
    cc.REDACT_PATTERNS = tuple(old) + LADDER_REDACT_PATTERNS
    try:
        yield
    finally:
        cc.REDACT_PATTERNS = old


# --- templates ----------------------------------------------------------------


def build_template(column, root=DEFAULT_ROOT, *, force=False, repo=REPO):
    """Build (or reuse) the workspace template for one column.

    ``omnisim`` -> ``cc_lane``'s product clone with the ladder's quarantine
    and the ladder's redaction patterns; ``webots`` -> ``cc_lane``'s linked
    upstream install, unchanged; anything else -> :func:`build_bare_template`.
    Returns the template path.
    """
    root = Path(root)
    if column in PRODUCT_CLONE_COLUMNS:
        with ladder_redaction():
            tpl = cc.build_omnisim_template(
                root, repo=repo, force=force,
                exclude_prefixes=LADDER_EXCLUDE_PREFIXES,
                exclude_files=LADDER_EXCLUDE_FILES)
        _annotate_manifest(root, "omnisim", column)
        return tpl
    if column in LINKED_INSTALL_COLUMNS:
        tpl = cc.build_webots_template(root, force=force)
        _annotate_manifest(root, "webots", column)
        return tpl
    return build_bare_template(column, root, force=force)


def build_bare_template(column, root=DEFAULT_ROOT, *, force=False):
    """An EMPTY template for a column whose product is not this repo.

    MuJoCo is a pip wheel, Gazebo is a container image, Isaac is a pod. Their
    product is installed on the machine and documented on their own sites, so
    the workspace is the tier's ``container/`` and nothing else. Not one file
    authored by us goes in -- our text there would contaminate the product
    comparison, which is the same rule the webots arm has carried since
    Phase W.
    """
    root = Path(root)
    tpl = root / "templates" / column
    manifest_path = root / "templates" / ("%s.manifest.json" % column)
    if tpl.exists():
        if not force and manifest_path.is_file():
            return tpl
        cc.teardown_workspace(tpl)
    tpl.mkdir(parents=True, exist_ok=True)
    manifest = {
        "sim": column, "column": column, "kind": "bare",
        "utc": _utc(), "included_file_count": 0,
        "junctions": [], "links": [], "excluded": [],
        "exclude_prefixes": [], "exclude_files": [],
        "redaction_ruling": ("nothing authored by us is staged, so there is "
                             "nothing to redact"),
        "redactions": [],
        "note": ("the %r column's product is installed on the machine (a "
                 "wheel, an image or a pod), not in this repo. The workspace "
                 "carries the tier's container/ and nothing else; anything "
                 "we authored would contaminate the product comparison."
                 % column),
        "known_disclosures": [],
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return tpl


def _annotate_manifest(root, name, column):
    """Record the ladder's own staging facts in the reused manifest."""
    p = Path(root) / "templates" / ("%s.manifest.json" % name)
    try:
        man = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return
    man["column"] = column
    man["programme"] = "capability_ladder"
    man["ladder_exclude_prefixes"] = list(LADDER_EXCLUDE_PREFIXES)
    man["ladder_exclude_files"] = list(LADDER_EXCLUDE_FILES)
    man["ladder_redaction_patterns"] = [p_.pattern
                                        for p_ in LADDER_REDACT_PATTERNS]
    man["known_disclosures"] = (list(man.get("known_disclosures") or [])
                                + list(KNOWN_DISCLOSURES))
    p.write_text(json.dumps(man, indent=2), encoding="utf-8")


def instantiate(template, dest):
    """A fresh per-cell instance. ``cc_lane``'s, unchanged."""
    return cc.instantiate(Path(template), Path(dest))


# --- the task's container -----------------------------------------------------


#: Appended to every tier's prompt on every column, identical text. It is not
#: task guidance and says nothing about robotics or any simulator: it describes
#: how THIS INSTRUMENT behaves. Claude Code backgrounds a foreground command
#: that outruns its own limit, and headless ``-p`` has no later turn in which
#: to be notified -- so an agent that shells out to a multi-minute engine run
#: loses the rest of its session to a handoff it never hears about. Cell 1
#: (T2/omnisim, 2026-08-02) ended exactly that way at 42% of its cap, with an
#: authored world and two finished sweeps in hand. Withholding this would
#: measure familiarity with one CLI's timeout, not capability.
ENVIRONMENT_NOTE = (
    "\n\nA note about this environment, not about the task: a command "
    "you run in the foreground may be moved to the background if it takes "
    "more than a few minutes, and you will not get a later turn in which "
    "to be told it finished. If you need a long-running command, run it so "
    "that it completes within a few minutes, or start it deliberately in "
    "the background and poll for its result yourself before you finish.")




#: What phase B will re-run, per column, written into the workspace as
#: DELIVERABLE.md so the agent is TOLD rather than left to guess.
#:
#: ⚠ THIS EXISTS BECAUSE THE COLUMNS ARE NOT SYMMETRIC, AND PRETENDING THEY
#: WERE COST A CELL. On the simulator columns the deliverable is a scene file:
#: the engine runs it, the grader injects its own recorder beside it, and
#: "author a .wbt" needs no explanation because that is simply what a scene IS
#: there. MuJoCo is a LIBRARY -- it has no scene runner, so every deliverable
#: is a program, and a grader cannot re-run one independently unless the
#: program exposes a control callback it can drive.
#:
#: Measured 2026-08-03: a MuJoCo cell built a working scene and wrote its
#: controller as a self-contained script with its own mj_step loop -- the
#: obvious thing to write, and unrunnable by our phase B. The scene was stepped
#: for 60 s with ctrl_writes=0, the arm never moved, and three assertions went
#: red for a reason that was neither MuJoCo's nor the agent's.
#:
#: Every column gets this file, so no column is being handed help the others
#: lack -- each is simply told its own contract.
DELIVERABLE_CONTRACT = {
    "omnisim": """# What to leave behind

Author a **`.wbt` scene file** in this workspace, plus whatever controllers it
references. That file IS the deliverable.

It will be re-run **cold, on its own, with no agent present**, by loading it in
the simulator with an independent recorder attached. So it has to work from a
standing start: anything your session set up by hand -- an environment
variable, a process you left running, a file outside the workspace -- will not
be there. If the scene needs a physics setting, put the setting in the scene.
""",
    "webots": """# What to leave behind

Author a **`.wbt` scene file** in this workspace, plus whatever controllers it
references. That file IS the deliverable.

It will be re-run **cold, on its own, with no agent present**, by loading it in
the simulator with an independent recorder attached. So it has to work from a
standing start: anything your session set up by hand will not be there.
""",
    "mujoco": """# What to leave behind

A **directory** containing:

- **`scene.xml`** -- the MuJoCo model.
- **a Python file exposing a control callback**, which the grader will drive.
  Name the file whatever you like; it must define one of:

      def control(model, data, t): ...     # t = seconds since the run started
      def step(model, data, t): ...
      def policy(model, data, t): ...

  Write your commands into `data.ctrl` (or `data.qfrc_applied`) inside it.

**Why a callback and not a script.** MuJoCo is a library, so there is no
"run this scene" command a grader can call. Your deliverable is re-run **cold,
with no agent present**, by an independent harness that compiles `scene.xml`,
steps it itself, and records what happens. A self-contained script that runs
its own `mj_step` loop cannot be driven that way -- the harness would step your
scene with no control input at all and record an arm that never moves.

Keep any `__main__` block you like for your own testing; the callback is what
gets graded.
""",
}


def write_deliverable_contract(workspace, column):
    """Write DELIVERABLE.md into the workspace. Returns the text, or None."""
    text = DELIVERABLE_CONTRACT.get(column)
    if not text:
        return None
    (Path(workspace) / "DELIVERABLE.md").write_text(text, encoding="utf-8")
    return text

def stage_container(workspace, task, *, dest_name="container"):
    """Copy the tier's declared ``container/`` files into the workspace.

    Returns ``(prompt_text, record)``. ``record`` names every staged file,
    every declared file that is MISSING from the repo, and every file present
    in ``container/`` that the task did NOT declare (present-but-undeclared is
    not staged: the declaration is the contract, and a stray file in a
    container is exactly how a scene or a hint reaches an agent by accident).
    """
    if not hasattr(task, "meta"):
        task = ladder_tasks.get(str(task))
    ws = Path(workspace)
    root = ws / dest_name
    declared = list((task.meta.get("container", {}) or {}).get("files", []))
    staged, missing = [], []
    for rel in declared:
        src = task.container_dir / rel
        if not src.is_file():
            missing.append(rel)
            continue
        dst = root / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        staged.append(rel.replace("\\", "/"))
    present = set()
    if task.container_dir.is_dir():
        for p in task.container_dir.rglob("*"):
            if p.is_file():
                present.add(p.relative_to(task.container_dir).as_posix())
    undeclared = sorted(present - {d.replace("\\", "/") for d in declared})
    record = {
        "container_root": str(root),
        "declared": [d.replace("\\", "/") for d in declared],
        "staged": staged,
        "declared_but_missing": missing,
        "present_but_undeclared_NOT_staged": undeclared,
        "layout_note": (task.meta.get("container", {}) or {}).get("layout"),
        "rule": ("exactly the files meta.json declares, with the container's "
                 "layout preserved (T1's package:// references resolve "
                 "against this directory as the package root)"),
    }
    return task.prompt + ENVIRONMENT_NOTE, record


# --- the quarantine check -----------------------------------------------------


def answer_key_violations(root, *, scan_content=True,
                          markers=ANSWER_KEY_MARKERS,
                          path_prefixes=QUARANTINE_PATH_PREFIXES,
                          path_files=QUARANTINE_PATH_FILES,
                          skip_dirs=("container",)):
    """Every answer-key hit inside a staged tree. ``[]`` is the clean witness.

    Two independent readings, because a path rule and a content rule fail
    differently:

    * **path** -- anything under a quarantined prefix, or one of the
      quarantined files, however it got there;
    * **content** -- any staged text file carrying a ladder marker (a task id,
      the plan's name, a grader module path, or a threshold CONSTANT NAME).

    Links are never traversed: the junctioned runtime dirs are the real tree
    and scanning them would report the repo, not the workspace. The tier's own
    ``container/`` is skipped -- it is the task, deliberately shipped, and its
    ``PROVENANCE.txt`` legitimately names the tier.
    """
    root = Path(root)
    hits = []
    if not root.exists():
        return hits
    skip = set(skip_dirs or ())
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames
                       if not cc.is_link(Path(dirpath) / d)
                       and not (Path(dirpath) == root and d in skip)]
        for f in filenames:
            p = Path(dirpath) / f
            rel = p.relative_to(root).as_posix()
            if rel.startswith(path_prefixes) or rel in path_files:
                hits.append({"path": rel, "kind": "quarantined_path",
                             "marker": None})
                continue
            if not scan_content or p.suffix.lower() not in TEXT_SUFFIXES:
                continue
            try:
                if p.stat().st_size > MAX_SCAN_BYTES:
                    continue
                text = p.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            for m in markers:
                found = m.search(text)
                if found:
                    hits.append({"path": rel, "kind": "answer_key_content",
                                 "marker": m.pattern,
                                 "excerpt": _excerpt(text, found)})
                    break
    return hits


def _excerpt(text, match, span=70):
    lo = max(0, match.start() - span)
    hi = min(len(text), match.end() + span)
    return text[lo:hi].replace("\n", " ")


def transcript_answer_key_hits(transcript_path, *, markers=ANSWER_KEY_MARKERS,
                               limit=25):
    """Ladder answer-key markers in a Claude Code session transcript.

    The countermeasure for the read-through risk the manifest records: the
    workspace cannot show the grader, but a tool that resolves ``..`` past a
    junction physically can reach the real repo. If the session's own
    transcript quotes a marker, the answer key was in front of the agent and
    the cell is INVALID -- whatever the workspace scan said.

    Returns ``(hits, reason_or_None)``; a transcript that cannot be read
    yields ``([], reason)`` and the row records *unchecked*, never *clean*.
    """
    p = Path(transcript_path) if transcript_path else None
    if p is None or not p.is_file():
        return [], "no transcript available to scan"
    try:
        text = p.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        return [], "transcript unreadable: %r" % (exc,)
    hits = []
    for m in markers:
        for found in m.finditer(text):
            hits.append({"marker": m.pattern,
                         "excerpt": _excerpt(text, found, span=90)})
            if len(hits) >= limit:
                return hits, None
    return hits, None


# --- teardown (re-exported so a caller never reaches past this module) --------

teardown_workspace = cc.teardown_workspace
teardown_workspace_resilient = cc.teardown_workspace_resilient
sweep_pending_deletes = cc.sweep_pending_deletes
sweep_workspace_processes = cc.sweep_workspace_processes
reap_port_listeners = cc.reap_port_listeners
sweep_repo_junction_artifacts = cc.sweep_repo_junction_artifacts
is_link = cc.is_link


# --- CLI ----------------------------------------------------------------------


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--column", default="omnisim",
                    help="the roster column (default %(default)s); known: "
                         + ", ".join(KNOWN_COLUMNS))
    ap.add_argument("--task", default=None,
                    help="stage a tier's container into a scratch instance "
                         "and report what it holds")
    ap.add_argument("--root", default=str(DEFAULT_ROOT))
    ap.add_argument("--force", action="store_true",
                    help="rebuild the template even if one exists")
    ap.add_argument("--verify", action="store_true",
                    help="scan the built template for answer-key material "
                         "(path + content) and print every hit")
    ap.add_argument("--keep", action="store_true",
                    help="with --task: keep the scratch instance")
    args = ap.parse_args(argv)

    root = Path(args.root).resolve()
    if REPO.resolve() == root or REPO.resolve() in root.parents:
        print("refusing: --root must be outside the repo", file=sys.stderr)
        return 2

    tpl = build_template(args.column, root, force=args.force)
    man_path = tpl.parent / (tpl.name + ".manifest.json")
    man = json.loads(man_path.read_text(encoding="utf-8"))
    print("%s template -> %s" % (args.column, tpl))
    print("  files copied : %s" % man.get("included_file_count"))
    print("  junctions    : %s" % [j["name"] for j in
                                   man.get("junctions", [])])
    print("  excluded     : %d paths (agentbench + ladder answer key + "
          "private trees)" % len(man.get("excluded", [])))
    reds = man.get("redactions", [])
    print("  redactions   : %d in %d file(s)"
          % (len(reds), len({r["file"] for r in reds})))

    if args.verify:
        hits = answer_key_violations(tpl)
        print("  quarantine   : %s"
              % ("CLEAN (0 hits)" if not hits else "%d HIT(S)" % len(hits)))
        for h in hits[:20]:
            print("    ! %s [%s] %s" % (h["path"], h["kind"],
                                        (h.get("excerpt") or "")[:120]))

    if args.task:
        task = ladder_tasks.get(args.task)
        ws = root / "instances" / ("stagecheck_%s_%s"
                                   % (time.strftime("%Y%m%d_%H%M%S"),
                                      args.column))
        instantiate(tpl, ws)
        prompt, rec = stage_container(ws, task)
        print("\n%s -> %s" % (task.id, ws))
        print("  prompt   : %s" % prompt)
        print("  staged   : %s" % rec["staged"])
        if rec["declared_but_missing"]:
            print("  MISSING  : %s" % rec["declared_but_missing"])
        if rec["present_but_undeclared_NOT_staged"]:
            print("  undeclared (not staged): %s"
                  % rec["present_but_undeclared_NOT_staged"])
        hits = answer_key_violations(ws)
        print("  quarantine: %s"
              % ("CLEAN (0 hits)" if not hits else "%d HIT(S)" % len(hits)))
        for h in hits[:20]:
            print("    ! %s [%s]" % (h["path"], h["kind"]))
        if not args.keep:
            out = teardown_workspace_resilient(ws)
            print("  teardown : ok=%s" % out.get("ok"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
