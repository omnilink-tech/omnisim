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

"""LANE-DIVERGENCE LEDGER for the G1 / quadruped Newton physics configuration.

WHAT THIS IS NOT
----------------
This is NOT ``tests/test_g1_physics_spec_conformance.py``. That file pins the
values the spec declares and asserts its consumers agree -- it says "this is
right". This file says the opposite thing about a different set of numbers:
"these two lanes are KNOWN to disagree, here is exactly how much, and the
disagreement may not widen, shrink, or spread without somebody editing this
ledger."

THE SITUATION (measured; documented in ``bb0ada85d``)
-----------------------------------------------------
``projects/policies/research/backends/g1_physics_spec.py`` calls itself the
SINGLE SOURCE OF TRUTH for the G1 Newton walk train<->deploy model. Measured, it
does not govern the shipped flagship path:

    projects/policies/training/run_walk_rl.sh:40   (UNCONDITIONAL export)
        OMNISIM_NEWTON_TARGET_KE=200  TARGET_KD=30  GROUND_MU=2.0
    projects/policies/research/backends/g1_physics.json
        gains.ke=100.0  gains.kd=5.0  contact.ground_mu=1.0

``projects/policies/training/g1_walk_recipe.py`` -- the current flagship
trainer, reached via ``run_g1_decent_walker.ps1`` -> ``run_walk_rl.sh`` --
contains no import of the spec at all.

This is NOT currently a train/deploy mismatch. The export sits above
``RES_MODE="$MODE"`` and outside any mode branch, so probe, train and deploy all
take 200/30/2.0; train==deploy holds WITHIN that lane. The defect is that two
lanes disagree and nothing detects it: the conformance test enforces the spec
only on the lane that already obeys it.

⛔ DO NOT "FIX" THIS BY EDITING A VALUE. 200/30/2.0 is what the shipped champion
was trained at and deploys at; 100/5/1.0 is what the research lane and the
conformance test are built on. Reconciling them is a decision about the training
recipe, not a constant edit. This file exists so the decision can be deferred
without the divergence drifting in the dark.

COST
----
Pure file parsing: two shell scripts, one JSON, three MJCF XMLs. No engine, no
GPU, no torch, no newton, no training. stdlib only (numpy/pytest not required
beyond pytest itself). Runs in milliseconds.

WHAT MAKES IT GO RED
--------------------
1. A FOURTH key starts diverging (a key both lanes declare stops agreeing).
2. An existing divergence CHANGES MAGNITUDE on either side.
3. A divergence is RESOLVED and its ledger entry goes stale -- this fails
   loudly and on purpose, so the exception gets deleted rather than rotting
   into a guard that is green because it stopped looking.
4. The structural premise breaks: the flagship export stops being unconditional,
   moves below the mode branch, or is assigned twice (any of which could make
   train != deploy WITHIN the flagship lane -- the thing that is fine today).
5. The spec grows or loses an ``OMNISIM_NEWTON_*`` knob and the mirror in this
   file no longer reproduces ``SPEC.newton_env()``.
6. The quadruped launcher's per-robot gains stop matching the MJCF actuator
   gains each champion was trained against (that lane has NO exceptions today).

Run with:
    python -m pytest tests/test_g1_lane_divergence_pins.py -v
"""

from __future__ import annotations

import json
import re
import shlex
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]

FLAGSHIP_SH = REPO_ROOT / "projects" / "policies" / "training" / "run_walk_rl.sh"
QUAD_SH = REPO_ROOT / "projects" / "policies" / "training" / "run_quad_walk_rl.sh"
FLAGSHIP_TRAINER = REPO_ROOT / "projects" / "policies" / "training" / "g1_walk_recipe.py"
SPEC_JSON = (REPO_ROOT / "projects" / "policies" / "research" / "backends"
             / "g1_physics.json")
MJCF_DIR = REPO_ROOT / "projects" / "policies" / "research" / "training" / "mjcf"


# ===========================================================================
# THE LEDGER -- the explicitly recorded, commented exception list.
# ===========================================================================

@dataclass(frozen=True)
class Divergence:
    """One key that the flagship launcher and the spec knowingly disagree on."""
    flagship: float          # what run_walk_rl.sh exports, today, measured
    spec: float              # what g1_physics.json declares, today, measured
    recorded: str            # ISO date this exception was recorded
    why: str                 # why it is not being reconciled


# Every entry below was MEASURED on 2026-08-15 and re-measured when this guard
# was written. Each is a REAL disagreement between two self-consistent lanes,
# NOT a bug to be patched by editing one number.
#
#   flagship lane = projects/policies/training/run_walk_rl.sh
#                   (reached by run_g1_decent_walker.ps1; drives g1_walk_recipe.py;
#                    the shipped G1 champion was trained AND deploys through it)
#   spec lane     = projects/policies/research/backends/g1_physics.json
#                   (read by g1_physics_spec.py; drives the research GPU trainer
#                    + scripts/dev/g1_deploy_launch.py; what the conformance test
#                    tests/test_g1_physics_spec_conformance.py pins)
KNOWN_DIVERGENCES: Dict[str, Divergence] = {
    # ---- 1/3 -------------------------------------------------------------
    # Position-PD stiffness. The spec's 100 is the research lane's winning
    # walker (runs/gpu_newton_g1_walk_ft_pdoff_clamp, 5.9 m / 33.8 s). The
    # flagship's 200 was arrived at empirically in the foot-redesign campaign:
    # projects/policies/research/mpc/foot_redesign/RESULTS.md:138 records
    # "ke 400 (stand stiffness) over-tracks the gait -> TARGET_KE=200". The two
    # numbers belong to two different policies; neither is wrong for its own.
    "OMNISIM_NEWTON_TARGET_KE": Divergence(
        flagship=200.0, spec=100.0, recorded="2026-08-15",
        why="flagship KE tuned in the foot-redesign campaign (RESULTS.md:138); "
            "spec KE is the research lane's winning-walker value. Reconciling "
            "means retraining one champion, not editing a constant.",
    ),
    # ---- 2/3 -------------------------------------------------------------
    # Position-PD damping. Moves with KE above (same RESULTS.md:150 config line:
    # "TARGET_KE=200 KD=30"). Cannot be reconciled independently of KE.
    "OMNISIM_NEWTON_TARGET_KD": Divergence(
        flagship=30.0, spec=5.0, recorded="2026-08-15",
        why="paired with TARGET_KE above (RESULTS.md:150 pins the 200/30 pair); "
            "changing KD alone would detune the shipped champion.",
    ),
    # ---- 3/3 -------------------------------------------------------------
    # Ground contact friction. NOTE this is the divergence with the widest
    # blast radius: 2.0 is what essentially every LIVE launcher in the tree
    # uses (run_walk_rl.sh, run_quad_walk_rl.sh:115, the foot_redesign runners,
    # run_g1_shadowC_deploy.ps1, projects/policies/skills/profiles/*.json), and
    # 1.0 survives only in g1_physics.json and the two run_*_unitree_walk.ps1
    # re-host runners. The spec is the minority reading here. Do NOT resolve it
    # by "just setting the JSON to 2.0": the conformance test's regression pin
    # (SPEC.GROUND_MU == 1.0) and the research lane's winning walker both rest
    # on 1.0, so the change is a retrain, not an edit.
    "OMNISIM_NEWTON_GROUND_MU": Divergence(
        flagship=2.0, spec=1.0, recorded="2026-08-15",
        why="the spec is the minority reading; nearly every live launcher uses "
            "2.0, but the research winning walker and the conformance pin rest "
            "on 1.0. Resolving it is a retrain decision.",
    ),
}

# Keys the SPEC declares that the flagship launcher does NOT export at all, so
# the flagship run takes the ENGINE DEFAULT for them. This is divergence by
# omission and it is tracked as its own set so that adding or dropping one is
# just as red as changing a value.
#
# contact_ke / contact_kd: g1_physics.json declares 2500/100 and
# g1_physics_spec.newton_env() emits them, but run_walk_rl.sh exports neither.
# The spec's own "_residuals.contact_ke_kd" already documents that the RESEARCH
# TRAINER does not apply them either (spec.apply_contact_to_trainer is OFF), so
# the flagship omission is consistent with the trainer, not with the deploy
# launcher. Recorded 2026-08-15.
SPEC_KEYS_NOT_EXPORTED_BY_FLAGSHIP = {
    "OMNISIM_NEWTON_CONTACT_KE",
    "OMNISIM_NEWTON_CONTACT_KD",
}

# The flagship export block must stay UNCONDITIONAL and ABOVE the mode branch.
# That structural fact -- not the values -- is what makes "train == deploy holds
# within the flagship lane" true. See the module docstring.
_FLAGSHIP_MODE_MARKER = "RES_MODE"


# ===========================================================================
# Shell parsing -- no hardcoded copy of any value being guarded.
# ===========================================================================

_NAME = r"[A-Za-z_][A-Za-z0-9_]*"
_ASSIGN_RE = re.compile(rf"^({_NAME})=(.*)$", re.S)
_DEFAULTED_RE = re.compile(r"^\$\{" + _NAME + r":?[-=]")
_SEPARATORS = {";", ";;", "&&", "||", "|", "&"}
# Block structure, token-exact. Only bare tokens count, so `$((` / `))` inside an
# arithmetic expansion and `$(...)` inside a quoted token never move the depth.
_OPENERS = {"if", "for", "while", "until", "case", "{"}
_CLOSERS = {"fi", "done", "esac", "}"}


@dataclass(frozen=True)
class ShellAssign:
    name: str
    value: str
    line: int
    exported: bool
    defaulted: bool   # written as ${NAME:-default} -- caller-overridable
    depth: int        # 0 == top level; >0 == inside if/for/while/case/function


def _logical_lines(text: str):
    """Yield (first_lineno, joined_line), collapsing backslash continuations."""
    out, buf, start = [], "", None
    for i, raw in enumerate(text.splitlines(), 1):
        if start is None:
            start = i
        if raw.endswith("\\"):
            buf += raw[:-1] + " "
            continue
        out.append((start, buf + raw))
        buf, start = "", None
    if buf:
        out.append((start or 1, buf))
    return out


def parse_shell_assignments(path: Path) -> Dict[str, List[ShellAssign]]:
    """Every ``NAME=VALUE`` / ``export NAME=VALUE`` in a shell script.

    Deliberately tolerant of reformatting: values are read through ``shlex`` so
    quoting, trailing comments, multiple assignments per ``export``, reordering
    and backslash continuations all parse. Lines shlex cannot tokenize (an
    unbalanced quote inside a here-doc, say) are skipped rather than fatal --
    the callers below assert that the keys they care about WERE found, so a
    skipped line can never silently hide a value.

    This is a scanner, not a shell. It records WHERE an assignment is written
    (``depth``, ``exported``, ``defaulted``, ``line``); it does not evaluate
    anything. The structural test below turns those facts into the assertions
    that matter -- assigned exactly once, at top level, unconditionally, above
    the mode branch -- which is what makes the scanner sufficient.
    """
    text = path.read_text(encoding="utf-8")
    found: Dict[str, List[ShellAssign]] = {}
    depth = 0
    for lineno, line in _logical_lines(text):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        try:
            toks = shlex.split(stripped, comments=True, posix=True)
        except ValueError:
            continue
        # `capture` models bash's "assignment prefix" position: at the start of
        # a command, and after `export`. It RESETS at a separator rather than
        # abandoning the line, so `[ x = y ] && export FOO=1` is still seen --
        # a later conditional re-assignment is exactly the shadowing case the
        # structural test must catch.
        capture, exported = True, False
        for tok in toks:
            if tok in _SEPARATORS:
                capture, exported = True, False
                continue
            if tok in _OPENERS:
                depth += 1
                capture, exported = True, False
                continue
            if tok in _CLOSERS:
                depth = max(0, depth - 1)
                capture, exported = True, False
                continue
            if tok == "export":
                capture, exported = True, True
                continue
            match = _ASSIGN_RE.match(tok.rstrip(";")) if capture else None
            if match is None:
                capture, exported = False, False
                continue
            name, value = match.group(1), match.group(2)
            found.setdefault(name, []).append(ShellAssign(
                name=name, value=value, line=lineno, exported=exported,
                defaulted=bool(_DEFAULTED_RE.match(value)), depth=depth,
            ))
    return found


def _first_line_of(path: Path, name: str) -> Optional[int]:
    """1-based line of the first assignment to ``name`` (None if absent)."""
    hits = parse_shell_assignments(path).get(name)
    return hits[0].line if hits else None


def parse_case_defaults(path: Path, case_subject: str,
                        keys: tuple) -> Dict[str, Dict[str, str]]:
    """Read a ``case "$SUBJ" in  label) K=v; K2=v2 ;; ... esac`` block.

    Returns ``{label: {KEY: value}}`` for the requested KEYs. Used for the
    quadruped launcher's per-robot ``DEF_KE`` / ``DEF_KD`` table, which lives
    inside a case branch and therefore is not a top-level assignment.
    """
    text = path.read_text(encoding="utf-8")
    start = text.find(f"case {case_subject} in")
    if start < 0:
        start = text.find(f'case "{case_subject}" in')
    if start < 0:
        return {}
    end = text.find("esac", start)
    block = text[start:end if end > 0 else len(text)]
    out: Dict[str, Dict[str, str]] = {}
    for raw in block.splitlines()[1:]:
        line = raw.split("#", 1)[0].strip()
        m = re.match(rf"^({_NAME})\)\s*(.*?)\s*;;\s*$", line)
        if not m:
            continue
        label, body = m.group(1), m.group(2)
        vals: Dict[str, str] = {}
        for key in keys:
            km = re.search(rf"\b{re.escape(key)}=([^\s;]+)", body)
            if km:
                vals[key] = km.group(1)
        if vals:
            out[label] = vals
    return out


# ===========================================================================
# Spec side -- mirror of g1_physics_spec.newton_env(), driven by the JSON.
# ===========================================================================

def _load_spec_json() -> dict:
    with SPEC_JSON.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def _num(x: float) -> str:
    """Mirror of g1_physics_spec._num (compact numeric env string)."""
    if float(x) == int(x):
        return str(int(x))
    return repr(float(x))


def spec_newton_env_from_json(spec: dict) -> Dict[str, str]:
    """Reproduce ``SPEC.newton_env()`` from the JSON alone (no import needed).

    Kept byte-equal to the real loader by
    ``test_spec_env_mirror_matches_loader`` below, which is what forces this
    mirror to be updated whenever the spec grows or drops a knob -- and that in
    turn is what makes a NEW key automatically enter the comparison.
    """
    ke = float(spec["gains"]["ke"])
    kd = float(spec["gains"]["kd"])
    armature = float(spec["gains"]["armature"])
    mujoco_warp = spec["solver"]["kind"] in ("mujoco_warp", "mjwarp")
    env = {
        "OMNISIM_NEWTON_FORCE_MUJOCO": "1",
        "OMNISIM_NEWTON_MJWARP": "1" if mujoco_warp else "0",
        "OMNISIM_NEWTON_STATICS": "1",
        "OMNISIM_NEWTON_SUBSTEPS": str(int(spec["solver"]["substeps"])),
        "OMNISIM_URDF_USE_INERTIA": "1" if spec["model"]["use_inertia"] else "0",
        "OMNISIM_NEWTON_SEED_POSE": "1" if spec["model"]["seed_pose"] else "0",
        "OMNISIM_NEWTON_TARGET_KE": _num(ke),
        "OMNISIM_NEWTON_TARGET_KD": _num(kd),
        "OMNISIM_NEWTON_GROUND_MU": _num(float(spec["contact"]["ground_mu"])),
    }
    if armature:
        env["OMNISIM_NEWTON_JOINT_ARMATURE"] = _num(armature)
    if not spec["clamp"]["enabled"]:
        env["OMNISIM_NEWTON_DISABLE_JOINT_CLAMP"] = "1"
    if spec["model"]["self_collision"]:
        env["OMNISIM_NEWTON_SELF_COLLISION"] = "1"
    env["OMNISIM_NEWTON_CONTACT_KE"] = _num(float(spec["contact"]["contact_ke"]))
    env["OMNISIM_NEWTON_CONTACT_KD"] = _num(float(spec["contact"]["contact_kd"]))
    return env


def _as_number(text: str) -> Optional[float]:
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


# ===========================================================================
# (0) The parse itself must be load-bearing.
# ===========================================================================

def test_source_artifacts_exist():
    for path in (FLAGSHIP_SH, QUAD_SH, FLAGSHIP_TRAINER, SPEC_JSON):
        assert path.exists(), f"guarded artifact missing: {path}"


def test_flagship_launcher_parses():
    env = parse_shell_assignments(FLAGSHIP_SH)
    # If the parser silently returned nothing, every comparison below would
    # vacuously pass. Anchor on a key that must always be there.
    assert "OMNISIM_NEWTON_SUBSTEPS" in env, (
        f"parser found no OMNISIM_NEWTON_SUBSTEPS in {FLAGSHIP_SH} -- the "
        "parser is broken or the launcher was restructured; the divergence "
        "comparisons below are NOT trustworthy until this is fixed."
    )


# ===========================================================================
# (1) STRUCTURAL premise: the flagship export is unconditional and pre-branch.
# ===========================================================================

def test_flagship_physics_exports_are_unconditional_and_pre_mode_branch():
    """The reason today's divergence is not a train/deploy mismatch.

    ``run_walk_rl.sh`` exports the physics block ABOVE ``RES_MODE="$MODE"`` and
    outside any mode branch, so probe / train / deploy all take the same
    numbers. If that ever stops being true, the flagship lane can silently
    train at one stiffness and deploy at another -- a far worse defect than the
    cross-lane divergence this file exists to pin.
    """
    assigns = parse_shell_assignments(FLAGSHIP_SH)
    mode_line = _first_line_of(FLAGSHIP_SH, _FLAGSHIP_MODE_MARKER)
    assert mode_line is not None, (
        f"{_FLAGSHIP_MODE_MARKER} assignment not found in {FLAGSHIP_SH.name}; "
        "the mode-branch structure this guard reasons about has changed."
    )
    for key in KNOWN_DIVERGENCES:
        hits = assigns.get(key)
        assert hits, f"{key} is no longer assigned in {FLAGSHIP_SH.name}"
        assert len(hits) == 1, (
            f"{key} is assigned {len(hits)} times in {FLAGSHIP_SH.name} "
            f"(lines {[h.line for h in hits]}). A second assignment can "
            "override the first conditionally -- collapse it to one."
        )
        hit = hits[0]
        assert hit.exported, f"{key} at line {hit.line} is assigned but not exported"
        assert hit.depth == 0, (
            f"{key} at line {hit.line} is now assigned inside a block "
            f"(depth {hit.depth}) -- an if/for/while/case/function body. It "
            "must stay at top level, or some modes will not receive it."
        )
        assert not hit.defaulted, (
            f"{key} at line {hit.line} became caller-overridable "
            f"(${{...:-...}}): {hit.value!r}. The 'all modes take the same "
            "value' premise no longer holds unconditionally."
        )
        assert hit.line < mode_line, (
            f"{key} is exported at line {hit.line}, BELOW "
            f"{_FLAGSHIP_MODE_MARKER} at line {mode_line}. The physics block "
            "must stay above the mode branch or train != deploy."
        )


# ===========================================================================
# (2) THE LEDGER -- the core guard.
# ===========================================================================

def _compare_flagship_to_spec():
    """(shared_keys, {key: (flagship_value, spec_value)} for the ones that differ).

    Compares the INTERSECTION of what the launcher exports and what the spec
    declares. Keys only the launcher sets are outside the spec's remit; keys
    only the spec sets are handled by
    ``test_spec_keys_the_flagship_launcher_never_exports``.
    """
    shell = {k: v[0].value for k, v in parse_shell_assignments(FLAGSHIP_SH).items()}
    spec_env = spec_newton_env_from_json(_load_spec_json())
    shared = sorted(set(shell) & set(spec_env))
    differ: Dict[str, tuple] = {}
    for key in shared:
        got, want = shell[key], spec_env[key]
        gnum, wnum = _as_number(got), _as_number(want)
        if gnum is not None and wnum is not None:
            if gnum != wnum:
                differ[key] = (gnum, wnum)
        elif got != want:
            differ[key] = (got, want)
    return shared, differ


def test_flagship_vs_spec_divergence_ledger():
    """Every disagreement between the two lanes must be in KNOWN_DIVERGENCES,
    at exactly the recorded magnitude -- and every ledger entry must still be a
    real disagreement."""
    shared, differ = _compare_flagship_to_spec()

    assert shared, (
        "the flagship launcher and the spec share NO OMNISIM_NEWTON_* keys; "
        "one of them was restructured and this guard is now blind."
    )

    # (a) a fourth key started diverging
    undocumented = sorted(set(differ) - set(KNOWN_DIVERGENCES))
    assert not undocumented, (
        "NEW physics-config divergence between the flagship lane and the spec:\n"
        + "\n".join(
            f"  {k}: {FLAGSHIP_SH.name}={differ[k][0]!r}  vs  "
            f"{SPEC_JSON.name}={differ[k][1]!r}"
            for k in undocumented
        )
        + "\n\nDo NOT silence this by editing a value. Either reconcile the two "
          "lanes deliberately (a retrain decision), or add an entry to "
          "KNOWN_DIVERGENCES in this file with the reason and the date."
    )

    # (c) a divergence was RESOLVED -- the exception is now stale and lying.
    resolved = sorted((set(KNOWN_DIVERGENCES) - set(differ)) & set(shared))
    assert not resolved, (
        "GOOD NEWS, ACTION REQUIRED -- these lanes now AGREE, so their "
        "KNOWN_DIVERGENCES entries are stale:\n"
        + "\n".join(f"  {k} (was flagship={KNOWN_DIVERGENCES[k].flagship} "
                    f"spec={KNOWN_DIVERGENCES[k].spec})" for k in resolved)
        + "\n\nDelete those entries from KNOWN_DIVERGENCES in this file. A "
          "guard that stays green after the problem is fixed is the same class "
          "of defect as the one it was built to catch."
    )

    # a ledger entry naming a key that is no longer compared at all
    orphaned = sorted(set(KNOWN_DIVERGENCES) - set(shared))
    assert not orphaned, (
        "KNOWN_DIVERGENCES names keys that are no longer declared by BOTH "
        f"lanes, so nothing is being compared: {orphaned}. Either the launcher "
        "stopped exporting them or the spec stopped declaring them -- resolve "
        "and delete the entries."
    )

    # (b) an existing divergence changed magnitude
    drifted = []
    for key, entry in KNOWN_DIVERGENCES.items():
        got_flagship, got_spec = differ[key]
        if got_flagship != entry.flagship or got_spec != entry.spec:
            drifted.append(
                f"  {key}: recorded flagship={entry.flagship} spec={entry.spec} "
                f"(on {entry.recorded})  ->  NOW flagship={got_flagship} "
                f"spec={got_spec}"
            )
    assert not drifted, (
        "a KNOWN divergence CHANGED MAGNITUDE:\n" + "\n".join(drifted)
        + "\n\nOne of the two lanes was retuned. That may be entirely correct "
          "-- but it is a physics change to a shipped or a reference model, so "
          "it must be acknowledged by updating the KNOWN_DIVERGENCES entry "
          "(values + date + reason) in this file."
    )


def test_spec_keys_the_flagship_launcher_never_exports():
    """Divergence BY OMISSION is tracked as tightly as divergence by value."""
    shell = set(parse_shell_assignments(FLAGSHIP_SH))
    spec_env = set(spec_newton_env_from_json(_load_spec_json()))
    missing = spec_env - shell
    assert missing == SPEC_KEYS_NOT_EXPORTED_BY_FLAGSHIP, (
        "the set of spec knobs the flagship launcher leaves at the ENGINE "
        f"DEFAULT changed.\n  recorded: {sorted(SPEC_KEYS_NOT_EXPORTED_BY_FLAGSHIP)}"
        f"\n  measured: {sorted(missing)}\n"
        "Newly missing keys mean the flagship silently stopped setting a knob "
        "the spec considers part of the model; newly present keys mean a gap "
        "closed. Either way update SPEC_KEYS_NOT_EXPORTED_BY_FLAGSHIP with the "
        "reason."
    )


def test_ledger_entries_are_documented():
    """A bare number in the ledger is not an exception, it is a second copy."""
    for key, entry in KNOWN_DIVERGENCES.items():
        assert re.fullmatch(r"\d{4}-\d{2}-\d{2}", entry.recorded), (
            f"{key}: 'recorded' must be an ISO date, got {entry.recorded!r}"
        )
        assert len(entry.why.strip()) >= 40, (
            f"{key}: 'why' must actually explain why the divergence is not "
            f"being reconciled, got {entry.why!r}"
        )


# ===========================================================================
# (3) The flagship trainer's non-use of the spec, pinned.
# ===========================================================================

def test_flagship_trainer_does_not_import_the_spec():
    """Pins the measured fact behind the whole ledger.

    ``g1_walk_recipe.py`` reads its physics from the process environment the
    launcher exports and imports ``g1_physics_spec`` nowhere. If that changes,
    the flagship lane has (partly) come under the spec's governance and the
    ledger above must be re-derived -- so this is another guard that goes red
    on GOOD news, by design.
    """
    text = FLAGSHIP_TRAINER.read_text(encoding="utf-8", errors="replace")
    hits = [
        f"  line {i}: {ln.strip()}"
        for i, ln in enumerate(text.splitlines(), 1)
        if "g1_physics_spec" in ln and not ln.lstrip().startswith("#")
    ]
    assert not hits, (
        f"{FLAGSHIP_TRAINER.name} now references g1_physics_spec:\n"
        + "\n".join(hits)
        + "\n\nThe flagship trainer was outside the spec's governance when "
          "KNOWN_DIVERGENCES was written. Re-derive the ledger."
    )


# ===========================================================================
# (4) QUADRUPED LANE.
# ===========================================================================
#
# Same shape, different source of truth. run_quad_walk_rl.sh:115 exports
# OMNISIM_NEWTON_GROUND_MU=2.0 unconditionally -- the same value as the G1
# flagship lane and the same divergence from g1_physics.json's 1.0.
#
# It is NOT added to KNOWN_DIVERGENCES: g1_physics.json is the G1 model, and
# asserting a quadruped launcher against it would be a category error that
# invites someone to "fix" one by editing the other. Ground friction is a WORLD
# property though, so the honest invariant is that the two LAUNCHERS agree with
# each other -- which they do today, with no exception needed.

def test_quad_and_flagship_launchers_agree_on_ground_mu():
    g1 = parse_shell_assignments(FLAGSHIP_SH).get("OMNISIM_NEWTON_GROUND_MU")
    quad = parse_shell_assignments(QUAD_SH).get("OMNISIM_NEWTON_GROUND_MU")
    assert g1 and quad, (
        "OMNISIM_NEWTON_GROUND_MU missing from "
        f"{'flagship' if not g1 else 'quad'} launcher"
    )
    assert _as_number(g1[0].value) == _as_number(quad[0].value), (
        f"the two in-engine training launchers disagree on ground friction: "
        f"{FLAGSHIP_SH.name}:{g1[0].line}={g1[0].value} vs "
        f"{QUAD_SH.name}:{quad[0].line}={quad[0].value}. Ground mu is a world "
        "property; two lanes training on the same engine should not differ "
        "silently. If the split is deliberate, record it here."
    )


# The quadruped launcher's per-robot PD gains claim, in a source comment, to be
# "GROUND TRUTH = the actuator gains baked into the MJCF each champion was
# trained on ... verified by reading gainprm/biasprm", and the same comment
# warns that go2_walk_deploy.py and b2_walk_deploy.py both DOCUMENT the wrong
# numbers. That claim is machine-checkable and costs one XML parse, so check it
# rather than trusting the comment. ZERO exceptions today -- all three match.
_QUAD_MJCF = {
    "go2": "go2_newton.xml",
    "omniquad": "omniquad_newton_fixed2.xml",
    "b2": "b2_newton.xml",
}


def _mjcf_pd_gains(path: Path):
    """(kp, kv) from an MJCF <actuator> block of affine <general> pairs.

    Position actuators carry biasprm="0 -kp"; velocity actuators carry
    biasprm="0 0 -kv". gainprm holds the magnitude. Returns the unique value of
    each, or raises if the model is not uniform (which would itself invalidate
    the launcher's single-scalar KE/KD).
    """
    root = ET.parse(path).getroot()
    kps, kvs = set(), set()
    for gen in root.iter("general"):
        gain = gen.get("gainprm")
        bias = (gen.get("biasprm") or "").split()
        if gain is None or not bias:
            continue
        if len(bias) == 2:
            kps.add(float(gain))
        elif len(bias) == 3:
            kvs.add(float(gain))
    return kps, kvs


@pytest.mark.parametrize("robot", sorted(_QUAD_MJCF))
def test_quad_launcher_gains_match_trained_mjcf(robot):
    table = parse_case_defaults(QUAD_SH, "$ROBOT", ("DEF_KE", "DEF_KD"))
    assert robot in table, (
        f"{QUAD_SH.name} no longer declares a DEF_KE/DEF_KD case branch for "
        f"'{robot}' (parsed branches: {sorted(table)})"
    )
    mjcf = MJCF_DIR / _QUAD_MJCF[robot]
    if not mjcf.exists():
        pytest.skip(f"trained MJCF not present in this checkout: {mjcf}")

    kps, kvs = _mjcf_pd_gains(mjcf)
    assert len(kps) == 1 and len(kvs) == 1, (
        f"{mjcf.name} is not uniform-gain (kp={sorted(kps)} kv={sorted(kvs)}); "
        "the launcher's single scalar DEF_KE/DEF_KD cannot represent it."
    )
    want_ke, want_kd = kps.pop(), kvs.pop()
    got_ke = float(table[robot]["DEF_KE"])
    got_kd = float(table[robot]["DEF_KD"])
    assert (got_ke, got_kd) == (want_ke, want_kd), (
        f"{robot}: {QUAD_SH.name} exports KE={got_ke} KD={got_kd} but the MJCF "
        f"the champion was trained on ({mjcf.name}) bakes kp={want_ke} "
        f"kv={want_kd}. train != deploy for this robot."
    )


# ===========================================================================
# (5) Keep the JSON mirror honest against the real loader.
# ===========================================================================

def test_spec_env_mirror_matches_loader():
    """``spec_newton_env_from_json`` must reproduce ``SPEC.newton_env()``.

    This is what makes the ledger auto-cover a NEW knob: add one to the spec
    and this fails until the mirror above is extended, after which the new key
    enters the flagship-vs-spec comparison on its own.

    Skips only if the spec module is unimportable (it is stdlib-only today, so
    in practice it always runs).
    """
    import sys
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    try:
        from projects.policies.research.backends import g1_physics_spec as SPEC
    except Exception as exc:  # noqa: BLE001
        pytest.skip(f"g1_physics_spec not importable in this env: {exc!r}")

    assert spec_newton_env_from_json(_load_spec_json()) == SPEC.newton_env(), (
        "the JSON->env mirror in this file no longer reproduces "
        "g1_physics_spec.newton_env(). Update spec_newton_env_from_json() so "
        "the divergence ledger keeps seeing every knob."
    )
