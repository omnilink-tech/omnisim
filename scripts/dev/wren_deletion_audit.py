#!/usr/bin/env python3
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
"""wren_deletion_audit.py -- what still blocks ``rm -rf src/wren``, exactly?

The WREN-retirement campaign (docs/developer/wren-retirement-plan.md) ends in one
deletion commit.  This is the gate on that commit: a single re-runnable command that
enumerates and classifies EVERY remaining dependency on WREN, so progress is a number
somebody can re-derive rather than a claim somebody remembers.

    python scripts/dev/wren_deletion_audit.py            # human report
    python scripts/dev/wren_deletion_audit.py --json     # machine-readable
    python scripts/dev/wren_deletion_audit.py --self-test  # prove it can go red

WHAT IT COUNTS

  * every ``wr_*`` symbol reference in ``src/omnisim``, grouped by owning file and by
    subsystem, split into real code vs comment/string mentions (a comment is not a
    link error);
  * every ``#include <wren/...>`` and every include of a header that lives in
    ``src/omnisim/wren/``;
  * the non-code couplings -- the ones people forget, because ``grep wr_`` cannot see
    them: the GL-blit present fallback, the shared ``src/glad``, the packaging
    manifest, the web viewer's own WREN, the public C ABI that must survive, plus the
    build system, the ``Camera.renderBackend`` default, the ``OmView3D : OmWrenWindow``
    base class and the campaign's own A/B instrument.

THE COUNTING RULE THAT KEEPS THE NUMBER HONEST

  A dependency that survives only as a HATCH-OFF FALLBACK is still BLOCKING.  The
  hatch IS the dependency: while ``OMNISIM_WGPU_NATIVE_MESH=0`` can still reach
  ``wr_static_mesh_read_data``, that call is compiled in, and ``rm -rf src/wren`` does
  not link.  This is exactly why the ``wr_*`` count went UP across W1b/W1c even though
  the DEFAULT path became WREN-free.  A finding leaves BLOCKING when the code is gone,
  never when the default merely stops using it.  ALREADY-PORTED is reserved for files
  whose only surviving ``wr_*`` text is a comment.

CLASSIFICATION

  BLOCKING        Breaks ``rm -rf src/wren`` today AND clearing it needs engineering:
                  a port, a platform surface, or a decision with pixels at stake.
  RETIRABLE       Breaks ``rm -rf src/wren`` today, but needs NO port -- this run found
                  no consumer of the FEATURE outside the engine's own wiring (no world,
                  PROTO, project or test declares it), so the remedy is deletion plus, for
                  the three the plan flags, one owner signature.  Counted separately so the
                  critical path is visible; it is still code that must be touched before
                  the deletion commit, and the engine wiring it drags along is named in the
                  probe evidence.
  ALREADY-PORTED  No live WREN code reference survives here -- only comments/history.
                  Costs nothing at deletion time.

This file makes no engine edits and runs no simulator.  It is read-only over the tree.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

# --------------------------------------------------------------------------------------
# Constants that describe the tree, not this run.
# --------------------------------------------------------------------------------------

ENGINE_ROOT = "src/omnisim"
WREN_LIB_DIR = "src/wren"
WREN_PUBLIC_INCLUDE_DIR = "include/wren"
WREN_RESOURCE_DIR = "resources/wren"
WREN_SHIM_DIR = "src/omnisim/wren"
GLAD_DIR = "src/glad"

CPP_SUFFIXES = (".cpp", ".hpp", ".h", ".c", ".cc", ".cxx")

# Directories that hold build output or frozen copies of the whole tree.  Scanning them
# double-counts every finding (an agentbench result workspace is a full source copy), so
# they are excluded even in --no-git mode.
# DUPLICATE SOURCE TREES -- excluded in BOTH modes, because each is a second copy of files
# already counted once. An agentbench result workspace is a whole checkout; counting it
# reports the same `wr_*` call four times and invents blockers that do not exist.
# Nothing here is git-tracked on this repo, so the git and walk scopes still agree.
EXCLUDED_PATH_PARTS = (
    "/build/",
    "/.git/",
    "/.claude/worktrees/",
    "/tests/benchmarks/agentbench/results/",
    "/tests/benchmarks/omnibench/results/",
    "/node_modules/",
)

# WALK-ONLY pruning: vendored/generated trees that `git ls-files` never lists anyway, so
# skipping them costs no coverage and keeps `--no-git` usable (msys64 alone is ~600 MB of
# Newton runtime). Deliberately NOT in EXCLUDED_PATH_PARTS: anything git DOES track must
# stay visible, or the audit acquires a blind spot in its default mode.
WALK_PRUNE_DIRS = {
    ".git",
    ".claude",
    ".local-runs",
    ".venv",
    "__pycache__",
    "build",
    "msys64",
    "node_modules",
}

WR_SYMBOL_RE = re.compile(r"\bwr_[A-Za-z0-9_]+")
WREN_ANGLE_INCLUDE_RE = re.compile(r'^\s*#\s*include\s*[<"](wren/[^>"]+)[>"]')
ANY_INCLUDE_RE = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]')

# --------------------------------------------------------------------------------------
# Campaign domains.  Path/basename -> the phase or retirement candidate that owns it.
# Order matters: first match wins.  Anything unmatched lands in a directory-derived
# bucket rather than being silently dropped, so a new WREN consumer cannot hide.
# --------------------------------------------------------------------------------------

DOMAIN_RULES: Sequence[Tuple[str, str, str]] = (
    # (regex over the repo-relative posix path, domain id, one-line owner note)
    (r"/OmVirtualRealityHeadset\.", "R1-vr", "VR headset (OpenVR) -- retirement candidate R1"),
    (r"/(OmPhysicsVectorRepresentation|OmContactPointsRepresentation|OmSupportPolygonRepresentation|OmOdeDebugger)\.",
     "R2-physics-debug", "physics debug representations -- retirement candidate R2"),
    (r"/OmSkin\.", "R3-skin", "Skin / FBX skeletal animation -- retirement candidate R3"),
    (r"/(OmAbstractCamera|OmCamera|OmRangeFinder|OmLidar|OmDistanceSensor|OmLightSensor|OmRadar|OmWrenCamera)\.",
     "W3-sensor-cameras", "sensor camera RTT path -- phase W3 (the pole)"),
    (r"/(OmWrenRangeNoise|OmWrenRangeQuantization|OmWrenNoiseMask|OmWrenLensDistortion|OmWrenMotionBlur|OmWrenColorNoise|OmWrenDepthOfField)\.",
     "W3-sensor-postfx", "sensor post-processing effects -- phase W3"),
    (r"/(OmWrenLabelOverlay|OmWrenTextureOverlay|OmWrenFullScreenOverlay|OmWrenPicker|OmWrenAbstractManipulator|OmWrenAbstractResizeManipulator|OmResizeManipulator|OmTranslateRotateManipulator|OmCoordinateSystem|OmLightRepresentation|OmSpotLightRepresentation|OmVisualBoundingSphere)\.",
     "W4-overlays-gizmos", "overlays / gizmos / labels / picking -- phase W4"),
    (r"/(OmWrenPostProcessingEffects|OmWrenShaders|OmWrenHdr|OmWrenBloom|OmWrenGtao|OmWrenSmaa|OmWrenLensFlare|OmWrenAtmosphericSky|OmWrenAbstractPostProcessingEffect)\.",
     "W6-mainview-postfx", "main-view post-FX + shader library -- phase W6 (flip/deprecate)"),
    (r"/(OmWrenOpenGlContext|OmWrenWindow|OmWrenRenderingContext|OmView3D|OmSimulationView|OmMainWindow)\.",
     "core-context-window", "GL context / 3D pane / window plumbing -- the deletion's structural core"),
    (r"/(OmWgpuSceneRenderer|OmWgpuMeshAdapter|OmWgpuGlBlit|OmCadShape|OmTriangleMeshGeometry|OmTesselator|OmWrenMeshBuffers|OmDeformableFrameListener)\.",
     "W1-geometry-collect", "geometry collection -- phase W1 (complete; residue is hatch-off fallback)"),
    (r"/(OmCloth|OmSoftBody|OmGranularGroup)\.", "deformables", "deformable/particle visual meshes"),
    (r"/(OmHingeJoint|OmBasicJoint|OmJoint|OmBallJoint|OmHinge2Joint|OmMuscle|OmPropeller|OmTrack)\.",
     "joint-visuals", "joint/actuator visual representations"),
    (r"/(OmBackground|OmFog|OmViewpoint|OmDirectionalLight|OmPointLight|OmSpotLight|OmLight)\.",
     "environment", "sky / fog / viewpoint / lights"),
    (r"/(OmAppearance|OmPbrAppearance|OmMaterial|OmImageTexture|OmTextureTransform|OmPaintTexture)\.",
     "materials", "appearance / material / texture upload"),
    (r"/(OmGeometry|OmShape|OmBox|OmSphere|OmCylinder|OmCapsule|OmCone|OmPlane|OmElevationGrid|OmIndexedLineSet|OmPointSet|OmMesh|OmBillboard|OmPose|OmAbstractPose|OmTransform|OmBaseNode)\.",
     "scene-graph", "scene-graph geometry + transform nodes"),
    (r"/(OmPen|OmDisplay|OmConnector|OmSolid|OmWorld)\.", "misc-nodes", "assorted node-level WREN use"),
    # Catch-all LAST: anything else living in the shim dir goes with the shim dir.
    (r"^/" + re.escape(WREN_SHIM_DIR) + r"/", "wren-shim-layer",
     "the src/omnisim/wren/ shim layer itself -- deleted wholesale with src/wren"),
)


@dataclass
class Finding:
    """One dependency on WREN.  ``count`` aggregates identical sites in one file."""

    id: str
    kind: str  # wr_call | wren_include | shim_include | coupling | build | asset | hatch
    classification: str  # BLOCKING | RETIRABLE | ALREADY-PORTED
    path: str
    subsystem: str
    domain: str
    detail: str
    evidence: str
    count: int = 1
    # `lines` is a sample (first 40) -- `count` is the authoritative total, never len(lines).
    lines: List[int] = field(default_factory=list)
    symbols: List[str] = field(default_factory=list)
    hatches: List[str] = field(default_factory=list)
    remedy: str = ""


# --------------------------------------------------------------------------------------
# File enumeration
# --------------------------------------------------------------------------------------


def _excluded(rel: str) -> bool:
    probe = "/" + rel.strip("/") + "/"
    return any(part in probe for part in EXCLUDED_PATH_PARTS)


def list_tracked_files(root: Path, use_git: bool) -> List[str]:
    """Repo-relative posix paths.  git-tracked by default.

    Using ``git ls-files`` is not a nicety: this tree carries build output, sibling
    worktrees under ``.claude/worktrees/`` and frozen full-source copies under
    ``tests/benchmarks/agentbench/results/*/workspace/``.  A naive walk counts the same
    ``wr_*`` call four times and reports a blocker that does not exist.
    """
    files: List[str] = []
    if use_git:
        try:
            out = subprocess.run(
                ["git", "-C", str(root), "ls-files", "-z"],
                capture_output=True,
                check=True,
                text=True,
            ).stdout
            files = [p for p in out.split("\0") if p]
        except (OSError, subprocess.CalledProcessError):
            files = []
    if not files:
        for dirpath, dirnames, filenames in os.walk(root):
            rel_dir = Path(dirpath).relative_to(root).as_posix()
            if rel_dir != "." and _excluded(rel_dir):
                dirnames[:] = []
                continue
            dirnames[:] = [d for d in dirnames if d not in WALK_PRUNE_DIRS]
            for name in filenames:
                rel = (Path(rel_dir) / name).as_posix() if rel_dir != "." else name
                files.append(rel)
    return sorted(p for p in files if not _excluded(p))


def _git_answered(root: Path, use_git: bool) -> bool:
    if not use_git:
        return False
    try:
        subprocess.run(
            ["git", "-C", str(root), "rev-parse", "--is-inside-work-tree"],
            capture_output=True,
            check=True,
            text=True,
        )
        return True
    except (OSError, subprocess.CalledProcessError):
        return False


MAX_READ_BYTES = 4 * 1024 * 1024  # a source file over 4 MB is generated or binary; skip it

_READ_CACHE: Dict[Tuple[str, str], Optional[str]] = {}
_BLANK_CACHE: Dict[Tuple[str, str], str] = {}


def read_text(root: Path, rel: str) -> Optional[str]:
    """Cached whole-file read.  The audit visits the same file from several probes, and an
    uncached implementation turned a 4-second scan into minutes on an 11k-file tree."""
    key = (str(root), rel)
    if key in _READ_CACHE:
        return _READ_CACHE[key]
    path = root / rel
    text: Optional[str]
    try:
        if path.stat().st_size > MAX_READ_BYTES:
            text = None
        else:
            text = path.read_text(encoding="utf-8", errors="replace")
    except (OSError, UnicodeError, ValueError):
        text = None
    _READ_CACHE[key] = text
    return text


def blanked_text(root: Path, rel: str, text: str) -> str:
    """Cached blank_noncode() -- the blanker is the hot loop and several probes want it."""
    key = (str(root), rel)
    cached = _BLANK_CACHE.get(key)
    if cached is None:
        cached = blank_noncode(text)
        _BLANK_CACHE[key] = cached
    return cached


# --------------------------------------------------------------------------------------
# C/C++ comment + string blanking (line-count preserving)
# --------------------------------------------------------------------------------------


def blank_noncode(text: str) -> str:
    """Return ``text`` with comment bodies and string/char literal bodies replaced by
    spaces, preserving every newline so line numbers still line up.

    This is what separates "a link error after deletion" from "a comment explaining
    what the code used to do".  ``OmWgpuSceneRenderer.cpp`` mentions
    ``wr_static_mesh_read_data`` five times in comments and calls
    ``wr_transform_get_matrix`` twice for real; counting all seven would overstate the
    blocker, and counting none would hide it.
    """
    out: List[str] = []
    i = 0
    n = len(text)
    state = "code"  # code | line_comment | block_comment | string | char | raw_string
    raw_delim = ""
    while i < n:
        c = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if state == "code":
            if c == "/" and nxt == "/":
                state = "line_comment"
                out.append("  ")
                i += 2
                continue
            if c == "/" and nxt == "*":
                state = "block_comment"
                out.append("  ")
                i += 2
                continue
            # raw string literal: R"delim( ... )delim"
            if c == "R" and nxt == '"':
                close = text.find("(", i + 2)
                if close != -1 and close - (i + 2) <= 16:
                    raw_delim = ")" + text[i + 2 : close] + '"'
                    state = "raw_string"
                    out.append(" " * (close - i + 1))
                    i = close + 1
                    continue
            if c == '"':
                state = "string"
                out.append('"')
                i += 1
                continue
            if c == "'":
                state = "char"
                out.append("'")
                i += 1
                continue
            out.append(c)
            i += 1
            continue
        if state == "line_comment":
            if c == "\n":
                state = "code"
                out.append("\n")
            else:
                out.append(" ")
            i += 1
            continue
        if state == "block_comment":
            if c == "*" and nxt == "/":
                state = "code"
                out.append("  ")
                i += 2
                continue
            out.append("\n" if c == "\n" else " ")
            i += 1
            continue
        if state == "raw_string":
            if text.startswith(raw_delim, i):
                out.append(" " * len(raw_delim))
                i += len(raw_delim)
                state = "code"
                continue
            out.append("\n" if c == "\n" else " ")
            i += 1
            continue
        # string / char
        if c == "\\" and i + 1 < n:
            out.append("  " if text[i + 1] != "\n" else " \n")
            i += 2
            continue
        if (state == "string" and c == '"') or (state == "char" and c == "'"):
            state = "code"
            out.append(c)
            i += 1
            continue
        out.append("\n" if c == "\n" else " ")
        i += 1
    return "".join(out)


# --------------------------------------------------------------------------------------
# Domain / subsystem mapping
# --------------------------------------------------------------------------------------


def subsystem_of(rel: str) -> str:
    parts = rel.split("/")
    if len(parts) >= 3 and parts[0] == "src" and parts[1] == "omnisim":
        return "/".join(parts[:3]) if parts[2] not in CPP_SUFFIXES else "src/omnisim"
    if len(parts) > 1:
        return "/".join(parts[:2])
    return rel


def domain_of(rel: str) -> Tuple[str, str]:
    probe = "/" + rel
    for pattern, domain, note in DOMAIN_RULES:
        if re.search(pattern, probe):
            return domain, note
    sub = subsystem_of(rel)
    return f"unclassified:{sub}", "no campaign domain claims this file -- triage it"


# --------------------------------------------------------------------------------------
# Retirement-candidate probes.  Each returns (zero_consumers, evidence).
# RETIRABLE is only granted when the probe actually passes on THIS tree.
# --------------------------------------------------------------------------------------


def probe_r1_vr(root: Path, files: Sequence[str]) -> Tuple[bool, str]:
    engine = [f for f in files if f.startswith(ENGINE_ROOT) and f.endswith(CPP_SUFFIXES)]
    external = []
    for rel in engine:
        if "OmVirtualRealityHeadset" in rel:
            continue
        text = read_text(root, rel)
        if text and "OmVirtualRealityHeadset" in blanked_text(root, rel, text):
            external.append(rel)
    # Bounded on purpose: only text formats a consumer could declare the node in.  An
    # unbounded walk of projects/ reads hundreds of MB of meshes and textures.
    content_exts = (".wbt", ".omniworld", ".proto", ".wrl", ".py", ".c", ".cpp", ".hpp", ".json")
    content = [
        f
        for f in files
        if f.endswith(content_exts)
        and not f.startswith(ENGINE_ROOT + "/")
        and (
            f.endswith((".wbt", ".omniworld", ".proto", ".wrl"))
            or f.startswith(("projects/", "tests/", "resources/"))
        )
    ]
    content_hits = []
    for rel in content:
        text = read_text(root, rel)
        if text and re.search(r"\bVirtualRealityHeadset\b", text):
            content_hits.append(rel)
    ok = not content_hits
    ev = (
        f"consumers in worlds/PROTOs/projects/tests: {len(content_hits)}"
        + (f" ({', '.join(content_hits[:4])})" if content_hits else "")
        + f"; engine wiring that would be deleted with it: {len(external)} file(s)"
        + (f" [{', '.join(external)}]" if external else "")
    )
    return ok, ev


def probe_r2_physics_debug(root: Path, files: Sequence[str]) -> Tuple[bool, str]:
    """The audit's claim: ``OmWorld::appendOdeContact`` -- the only writer of
    ``mOdeContacts`` -- has zero callers, so the contact overlay is structurally dead."""
    callers: List[str] = []
    for rel in files:
        if not (rel.startswith(ENGINE_ROOT) and rel.endswith(CPP_SUFFIXES)):
            continue
        text = read_text(root, rel)
        if not text:
            continue
        code = blanked_text(root, rel, text)
        for m in re.finditer(r"\bappendOdeContact\b", code):
            line = code[: m.start()].count("\n") + 1
            src_line = text.splitlines()[line - 1] if line - 1 < len(text.splitlines()) else ""
            # declaration or definition, not a call
            if re.search(r"(void\s+(OmWorld::)?appendOdeContact|^\s*void\s+appendOdeContact)", src_line):
                continue
            callers.append(f"{rel}:{line}")
    # A DEAD DATA SOURCE IS NOT A DEAD CLASS, and conflating the two nearly deleted working
    # code on 2026-08-22. The original probe asked ONE question (does appendOdeContact have
    # callers?) and stamped the answer onto every file in the R2 group -- but only the contact
    # overlay is fed by that writer. Two of the group's classes are alive and reached by other
    # routes entirely:
    #   * OmPhysicsVectorRepresentation is the BASE of OmForceRepresentation /
    #     OmTorqueRepresentation, which OmDragSolidEvent constructs and drives via
    #     updatePosition() -- the force/torque arrow you see when you drag a Solid with the mouse.
    #   * OmSupportPolygonRepresentation is constructed by OmSolid::showSupportPolygonRepresentation
    #     and fed by extractContactPoints(), which has a NATIVE NEWTON contacts path (default ON
    #     since 2026-08-07) and so does not depend on appendOdeContact at all.
    # The caveat that used to live in this evidence string named the support polygon, but a caveat
    # in prose does not change a classification, and RETIRABLE is what the next reader acts on.
    # So: veto the whole group whenever any class it declares is still CONSTRUCTED somewhere.
    # The classes this group owns, including the two SUBCLASSES declared inside
    # OmPhysicsVectorRepresentation.hpp -- those are the names actually constructed, so a probe
    # that only knew the file stems would have missed them.
    r2_classes = (
        "OmContactPointsRepresentation",
        "OmPhysicsVectorRepresentation",
        "OmForceRepresentation",
        "OmTorqueRepresentation",
        "OmSupportPolygonRepresentation",
    )
    owner_stems = ("OmContactPointsRepresentation", "OmPhysicsVectorRepresentation",
                   "OmSupportPolygonRepresentation")
    live: List[str] = []
    for rel in files:
        if not (rel.startswith(ENGINE_ROOT) and rel.endswith(CPP_SUFFIXES)):
            continue
        if any(Path(rel).name.startswith(st + ".") for st in owner_stems):
            continue  # the class's own translation unit
        text = read_text(root, rel)
        if not text:
            continue
        code = blanked_text(root, rel, text)
        for cls in r2_classes:
            for m in re.finditer(r"\bnew\s+" + re.escape(cls) + r"\b", code):
                live.append(f"{cls} constructed at {rel}:{code[: m.start()].count(chr(10)) + 1}")
    ok = (not callers) and (not live)
    ev = "appendOdeContact callers: " + (", ".join(callers) if callers else "0 (decl + defn only)")
    if live:
        ev += (
            " -- BUT the group is NOT retirable: " + "; ".join(sorted(set(live))[:4])
            + ". A dead data source is not a dead class; these are reached by other routes "
            "(mouse-drag force/torque arrows; the support polygon, fed by the native Newton "
            "contact path that is default-ON since 2026-08-07)."
        )
    else:
        ev += (
            " -- and no class this group declares is constructed anywhere, so the contact overlay "
            "and contact sound are structurally dead. Still true regardless: "
            "OmSolid::supportPolygon/extractContactPoints/staticBalance back a frozen public C ABI "
            "(coupling C5) and must survive any R2 retirement."
        )
    return ok, ev


def probe_r3_skin(root: Path, files: Sequence[str]) -> Tuple[bool, str]:
    hits: List[str] = []
    for rel in files:
        if not rel.endswith((".wbt", ".omniworld", ".proto", ".wrl")):
            continue
        if rel.endswith("Skin.wrl"):
            continue
        text = read_text(root, rel)
        if text and re.search(r"(^|\s)(DEF\s+\S+\s+)?Skin\s*\{", text, re.MULTILINE):
            hits.append(rel)
    ok = not hits
    ev = f"Skin {{}} instances in tracked worlds/PROTOs: {len(hits)}" + (
        f" ({', '.join(hits[:4])})" if hits else ""
    )
    return ok, ev


# domain -> (label, probe, owner_gate)
# owner_gate mirrors the plan's own "(warning) owner" column: the code has no live consumer,
# but removing the FEATURE is a product decision, not an engineering one. RETIRABLE never
# means "go delete it" for these -- it means "no port is needed, one signature is".
RETIREMENT_PROBES = {
    "R1-vr": ("VR headset (OpenVR)", probe_r1_vr, True),
    "R2-physics-debug": ("physics debug representations", probe_r2_physics_debug, True),
    "R3-skin": ("Skin / FBX skeletal", probe_r3_skin, True),
}


# --------------------------------------------------------------------------------------
# WREN-selecting env hatches.  These are the reason the count cannot be flattered.
# --------------------------------------------------------------------------------------

WREN_SELECTING_HATCHES: Sequence[Tuple[str, str]] = (
    ("OMNISIM_FORCE_WREN", "RETIRED (F1): warned and ignored; used to pin EVERY render-backend resolution to WREN"),
    ("OMNISIM_LEGACY", "RETIRED (F1, render arm): warned and ignored; used to pin every resolution to WREN"),
    ("OMNISIM_WGPU_NATIVE_MESH", "=0 reverts tessellated geometry to WREN's GL readback (W1a)"),
    ("OMNISIM_WGPU_NATIVE_PRIMITIVES", "=0 reverts unit primitives to WREN's GL readback (W1c)"),
    ("OMNISIM_WGPU_NATIVE_CADSHAPE", "=0 reverts CadShape to wr_transform_get_matrix + WREN readback (W1b)"),
    ("OMNISIM_LIDAR_WGPU", "opt-in (>=1); selects wgpu for the Lidar RTT regardless of its renderBackend field"),
    ("OMNISIM_RANGEFINDER_WGPU", "opt-in (>=1); selects wgpu for the RangeFinder regardless of its field"),
    ("OMNISIM_WREN_POSTFX", "=1 force-builds the WREN main-view post-FX chain"),
    ("OMNISIM_WGPU_MAINVIEW_FORCE", "RETIRED (F1): warned no-op; the wgpu main view is the default resolution"),
    ("OMNISIM_NEWTON_SKIP_WREN", "step-cost ablation, skips transform writeback (value-parsed since F1)"),
    ("OMNISIM_CAPTURE_BACKEND", "capture/cinema backend selector; capture pins the WREN Camera explicitly"),
)

# F1 (wren-deletion-runbook.md, Phase F): hatches retired to warned no-ops. Their engine reads
# are now presence tests BY DESIGN -- the warning fires when the variable is set at all, exactly
# like warnRetiredOdeSelectors' physics twins -- so rule 4 (value-parsed only) does not apply:
# there is no arm left for `=0` to accidentally arm. scan_hatches() does NOT take this set on
# faith: it verifies that every engine read site of each of these sits in a file carrying the
# one-shot "RETIRED and IGNORED" warning text, and re-raises the rule-4 VIOLATION if that ever
# stops being true (i.e. someone reintroduces a real read).
RETIRED_WREN_HATCHES = frozenset({
    "OMNISIM_FORCE_WREN",
    "OMNISIM_LEGACY",
    "OMNISIM_WGPU_MAINVIEW_FORCE",
})

_INT_PARSE_HINTS = (
    "IntValue",
    "toInt(",
    '== "0"',
    '== "1"',
    '!= "0"',  # the F1 value-parse idiom: v != "0" && v != "false" && ... (=0/false/off/no disarm)
    "!= 0",
    "== 0",
    ">= 1",
    "== 1",
    "atoi(",
)
_PRESENCE_HINTS = ("IsSet(", "!= nullptr", "[0] != '\\0'", "getenv(", "in os.environ", "environ.get")


def classify_hatch_polarity(
    root: Path, files: Sequence[str], variables: Sequence[str]
) -> Dict[str, Tuple[str, List[str]]]:
    """For every hatch: value-parsed / presence-gated / not-found, plus its read sites.

    Campaign rule 4 says every hatch must be value-parsed (``FOO=0`` means OFF).  A
    presence-gated read makes ``FOO=0`` ARM the hatch -- the OMNISIM_REQUIRE_NEWTON
    trap -- so a user trying to turn a WREN fallback off turns it on instead.

    One pass over the candidate files for ALL variables: the per-variable version re-read
    every C++ and Python file in the tree twelve times.
    """
    sites: Dict[str, List[str]] = {v: [] for v in variables}
    verdicts: Dict[str, List[str]] = {v: [] for v in variables}
    for rel in files:
        is_cpp = rel.endswith(CPP_SUFFIXES)
        is_py = rel.endswith(".py")
        if not (is_cpp or is_py):
            continue
        text = read_text(root, rel)
        if not text or "OMNISIM_" not in text:
            continue
        # A hatch that only Python reads (OMNISIM_CAPTURE_BACKEND) is still a hatch; scanning
        # C++ alone reported it as "not-found", which reads as "retired" and is the opposite
        # of true.
        present = [v for v in variables if (f'"{v}"' if is_cpp else v) in text]
        if not present:
            continue
        lines = text.splitlines()
        # The blanker empties string literals, so the "VAR" needle only survives in the RAW
        # line. Match on raw, and use the blanked line purely to reject a commented-out read
        # (a fully-blanked line was comment, not code).
        code_lines = blanked_text(root, rel, text).splitlines() if is_cpp else []
        for idx, raw_line in enumerate(lines):
            if "OMNISIM_" not in raw_line:
                continue
            if is_cpp and idx < len(code_lines) and not code_lines[idx].strip():
                continue
            if is_py and raw_line.lstrip().startswith("#"):
                continue
            window = None
            for var in present:
                if (f'"{var}"' if is_cpp else var) not in raw_line:
                    continue
                if window is None:
                    window = "\n".join(lines[max(0, idx - 2) : idx + 4])
                sites[var].append(f"{rel}:{idx + 1}")
                if any(h in window for h in _INT_PARSE_HINTS):
                    verdicts[var].append("value-parsed")
                elif any(h in window for h in _PRESENCE_HINTS):
                    verdicts[var].append("presence-gated")
                else:
                    verdicts[var].append("unknown")
    out: Dict[str, Tuple[str, List[str], List[str]]] = {}
    for var in variables:
        # Rule 4 is a claim about the ENGINE's read. Tooling under scripts/ and projects/
        # mostly SETS these variables (`env["OMNISIM_FORCE_WREN"] = "1"`), which is not a
        # gate at all -- letting those sites vote turned a presence-gated engine read into
        # "mixed". Polarity comes from C++ only; the tooling sites are reported beside it
        # because they are the consumers that die when the hatch is retired (coupling C9).
        engine, tooling, engine_verdicts = [], [], []
        for site, verdict in zip(sites[var], verdicts[var]):
            if site.split(":")[0].endswith(CPP_SUFFIXES):
                engine.append(site)
                engine_verdicts.append(verdict)
            else:
                tooling.append(site)
        v = engine_verdicts
        if not engine:
            polarity = "not-in-engine" if tooling else "not-found"
        elif "presence-gated" in v and "value-parsed" not in v:
            polarity = "presence-gated"
        elif "value-parsed" in v and "presence-gated" not in v:
            polarity = "value-parsed"
        elif "value-parsed" in v:
            polarity = "mixed"
        else:
            polarity = "unknown"
        out[var] = (polarity, engine, tooling)
    return out


# --------------------------------------------------------------------------------------
# Non-code couplings.  Each probe returns (status, evidence, remedy).
# status: HOLDS | CHANGED | NO-LONGER-HOLDS
# --------------------------------------------------------------------------------------


def _grep(root: Path, rel: str, pattern: str, code_only: bool = True) -> List[Tuple[int, str]]:
    text = read_text(root, rel)
    if not text:
        return []
    hay = blanked_text(root, rel, text) if (code_only and rel.endswith(CPP_SUFFIXES)) else text
    src = text.splitlines()
    out = []
    for idx, line in enumerate(hay.splitlines()):
        if re.search(pattern, line):
            out.append((idx + 1, src[idx].strip() if idx < len(src) else ""))
    return out


def coupling_gl_blit_fallback(root: Path, files: Sequence[str]):
    rel = "src/omnisim/gui/OmView3D.cpp"
    if rel not in files:
        return "NO-LONGER-HOLDS", f"{rel} is gone from the tree", "re-derive the present path"
    hits = _grep(root, rel, r"OmWgpuGlBlitRgbaToScreen")
    # D1.4: with src/wren deleted, this coupling is RESOLVED by design, not broken -- the GL
    # blit survives as the deliberate WREN-free present fallback (L1), and OmWrenOpenGlContext
    # is a rescued plain QOpenGLContext wrapper with zero wr_* content. The pre-deletion
    # wording ("deleting WREN deletes the safety net") is history once the deletion happened.
    wren_gone = not any(fp.startswith(WREN_LIB_DIR + "/") for fp in files)
    if wren_gone:
        ctx_ok = "src/omnisim/render/OmWrenOpenGlContext.cpp" in files
        return (
            "NO-LONGER-HOLDS",
            f"src/wren is deleted; the GL present blit ({'present' if hits else 'MISSING'} in {rel}) is the "
            f"deliberate WREN-free fallback for hosts with no native wgpu surface (L1), bracketed by the "
            f"rescued OmWrenOpenGlContext ({'present' if ctx_ok else 'MISSING'} in src/omnisim/render/).",
            "none -- L1's residual gate is R8 (wgpu-native on every supported platform)",
        )
    ctx = _grep(root, rel, r"OmWrenOpenGlContext::(makeWrenCurrent|doneWren|instance)")
    # D1.1 moved OmWrenOpenGlContext out of the shim dir into render/ (it is a QOpenGLContext
    # subclass whose only WREN content is two wr_gl_state_set_context_active() calls -- those
    # go at D1.4, the class survives). Accept either location so this check stays honest
    # across the move.
    shim_exists = (f"{WREN_SHIM_DIR}/OmWrenOpenGlContext.cpp" in files or
                   "src/omnisim/render/OmWrenOpenGlContext.cpp" in files)
    if not hits:
        return (
            "NO-LONGER-HOLDS",
            f"{rel}: no OmWgpuGlBlitRgbaToScreen call -- the GL present fallback is gone",
            "none; confirm the native wgpu surface is the only present path",
        )
    blit_line = hits[0][0]
    near = [ln for ln, _ in ctx if abs(ln - blit_line) <= 6]
    status = "HOLDS" if (near and shim_exists) else "CHANGED"
    ev = (
        f"{rel}:{blit_line} OmWgpuGlBlitRgbaToScreen() sits inside "
        f"OmWrenOpenGlContext::makeWrenCurrent()/swapBuffers()/doneWren() "
        f"(context calls at lines {near or [ln for ln, _ in ctx][:3]}); "
        f"OmWrenOpenGlContext lives in src/omnisim/render/ since D1.1 ({'present' if shim_exists else 'MISSING'}). "
        f"{len(ctx)} OmWrenOpenGlContext call sites in this file overall."
    )
    return (
        status,
        ev,
        "the native wgpu surface must work on every supported platform with NO fallback "
        "(today the surface exists only for a Windows HWND) -- deleting WREN deletes the safety net",
    )


def coupling_glad_shared(root: Path, files: Sequence[str]):
    consumers = []
    for rel in files:
        if not rel.startswith(ENGINE_ROOT) or not rel.endswith(CPP_SUFFIXES):
            continue
        text = read_text(root, rel)
        if not text:
            continue
        # `#include "glad/glad.h"` is a string literal, which blank_noncode() empties -- so
        # match the RAW line and use the blanked line only to reject a commented-out include.
        code_lines = blanked_text(root, rel, text).splitlines()
        for idx, raw in enumerate(text.splitlines()):
            if not re.match(r'\s*#\s*include\s*[<"]glad/glad\.h[>"]', raw):
                continue
            if idx < len(code_lines) and "include" not in code_lines[idx]:
                continue
            consumers.append(f"{rel}:{idx + 1}")
            break
    glad_tracked = [f for f in files if f.startswith(GLAD_DIR + "/")]
    wgpu_consumers = [c for c in consumers if "Wgpu" in c or "wgpu" in c]
    status = "HOLDS" if wgpu_consumers else "NO-LONGER-HOLDS"
    # D1.4: once src/wren is gone the hazard this probe guarded ("an rm -rf that also drops
    # glad") can no longer happen by accident -- glad survives deliberately as the GL blit's
    # loader. Resolved, not outstanding.
    if not any(fp.startswith(WREN_LIB_DIR + "/") for fp in files):
        return (
            "NO-LONGER-HOLDS",
            f"src/wren is deleted and {GLAD_DIR} survives deliberately ({len(glad_tracked)} tracked "
            f"file(s)) as the GL present blit's loader; wgpu-side consumer(s): {wgpu_consumers or 'none'}",
            "none",
        )
    ev = (
        f"{GLAD_DIR} is {len(glad_tracked)} tracked file(s); raw-GL consumers inside "
        f"{ENGINE_ROOT}: {consumers or 'none'}; of those, wgpu-side: {wgpu_consumers or 'none'}"
    )
    return (
        status,
        ev,
        "glad can only go when the GL blit goes (coupling 1); it is NOT WREN-only, so a "
        "`rm -rf src/wren` that also drops src/glad breaks the wgpu present fallback",
    )


def coupling_packaging_manifest(root: Path, files: Sequence[str]):
    rel = "scripts/packaging/files_core.txt"
    if rel not in files:
        return "NO-LONGER-HOLDS", f"{rel} not tracked", "none"
    hits = _grep(root, rel, r"wren", code_only=False)
    res_tracked = [f for f in files if f.startswith(WREN_RESOURCE_DIR + "/")]
    if not hits:
        return (
            "NO-LONGER-HOLDS",
            f"{rel} no longer names any WREN asset",
            "none",
        )
    lines = [ln for ln, _ in hits]
    # D1.4: the manifest deliberately keeps the surviving resources/wren assets (LICENSE, the
    # two gizmo meshes, the HUD icons + muscle.png). It only BLOCKS while it still ships the
    # deleted shader tree.
    if not any("shaders" in t for _, t in hits):
        return (
            "NO-LONGER-HOLDS",
            f"{rel} ships only the surviving resources/wren assets ({len(res_tracked)} tracked file(s)): "
            + "; ".join(t for _, t in hits),
            "none",
        )
    status = "HOLDS"
    note = ""
    if lines != list(range(lines[0], lines[0] + 5)):
        note = " (the plan cites lines 225-229; the manifest now spans a different range)"
    return (
        status,
        f"{rel} lines {lines[0]}-{lines[-1]} ship WREN assets by wildcard{note}: "
        + "; ".join(t for _, t in hits)
        + f". {WREN_RESOURCE_DIR}/ is {len(res_tracked)} tracked file(s).",
        "remove those manifest entries in the SAME change as the deletion, or the installer "
        "references missing paths",
    )


def coupling_web_viewer_wren(root: Path, files: Sequence[str]):
    wwi = [f for f in files if f.startswith("resources/web/wwi/")]
    js_wren = [f for f in wwi if "wren" in f.lower()]
    marker_files = [f for f in js_wren if Path(f).name in ("WrenRenderer.js", "wrenjs.js", "wrenjs.wasm", "wrenjs.data")]
    if not js_wren:
        return "NO-LONGER-HOLDS", "resources/web/wwi carries no WREN-derived renderer", "none"
    # D1.4: the divergence is ACCEPTED, recorded here at the deletion commit -- the browser-side
    # wrenjs viewer is a vendored artifact independent of the engine's deleted src/wren, keeps
    # working, and permanently diverges in look from the wgpu desktop view.
    if not any(fp.startswith(WREN_LIB_DIR + "/") for fp in files):
        return (
            "ACCEPTED",
            f"resources/web/wwi holds {len(js_wren)} WREN-derived file(s) including "
            f"{[Path(m).name for m in marker_files]}; the --stream w3d viewer keeps rendering through "
            f"them (vendored, independent of the deleted engine WREN). The streamed view's divergence "
            f"from the wgpu desktop view is the recorded, accepted deviation of the D1.4 commit.",
            "accepted at D1.4; a wwi port is future work, not a deletion blocker",
        )
    return (
        "HOLDS",
        f"resources/web/wwi holds {len(js_wren)} WREN-derived file(s) including "
        f"{[Path(m).name for m in marker_files]}; the --stream w3d viewer renders in the browser "
        f"through them. They are vendored artifacts (wrenjs was curl'd, never built here), so "
        f"deleting {WREN_LIB_DIR} does NOT break them.",
        "no build breakage -- but the STREAMED view will permanently diverge from a wgpu-only "
        "desktop view. Accept that deliberately or schedule a port; do not discover it after the fact.",
    )


def coupling_public_c_abi(root: Path, files: Sequence[str]):
    solid_hpp = "src/omnisim/nodes/OmSolid.hpp"
    abi_header = "include/controller/c/omnisim/supervisor.h"
    sym = "wb_supervisor_node_get_static_balance"
    decls = []
    for name in ("supportPolygon", "extractContactPoints", "staticBalance"):
        hits = _grep(root, solid_hpp, rf"\b{name}\b")
        decls.append(f"{name}@{solid_hpp}:{hits[0][0] if hits else 'MISSING'}")
    abi_hits = _grep(root, abi_header, rf"\b{sym}\b", code_only=False)
    impl = _grep(root, "src/controller/c/supervisor.c", rf"\b{sym}\b", code_only=False)
    missing = [d for d in decls if d.endswith("MISSING")]
    # D1.4: the guard survives the deletion as a permanent invariant. Intact members after the
    # deletion = GUARANTEED (non-blocking); a member going missing = CHANGED (blocking again).
    if not any(fp.startswith(WREN_LIB_DIR + "/") for fp in files):
        status = "CHANGED" if (missing or not abi_hits) else "GUARANTEED"
        return (
            status,
            f"{sym} declared at {abi_header}:{abi_hits[0][0] if abi_hits else 'MISSING'}, "
            f"implemented at src/controller/c/supervisor.c:{impl[0][0] if impl else 'MISSING'}; "
            f"engine backing: {', '.join(decls)}",
            "these three OmSolid members back a FROZEN public C ABI and survived the WREN deletion; "
            "they must never be deleted (a build that drops them still LINKS and silently returns garbage).",
        )
    status = "NO-LONGER-HOLDS" if (missing or not abi_hits) else "HOLDS"
    return (
        status,
        f"{sym} declared at {abi_header}:{abi_hits[0][0] if abi_hits else 'MISSING'}, "
        f"implemented at src/controller/c/supervisor.c:{impl[0][0] if impl else 'MISSING'}; "
        f"engine backing: {', '.join(decls)}",
        "these three OmSolid members back a FROZEN public C ABI. A debug-rendering retirement "
        "that deletes them still LINKS and silently returns garbage -- they must survive.",
    )


def coupling_build_system(root: Path, files: Sequence[str]):
    makefiles = [f for f in files if Path(f).name in ("Makefile", "Makefile.include") or f.endswith(".mk")]
    hits: Dict[str, int] = {}
    for rel in makefiles:
        text = read_text(root, rel)
        if not text:
            continue
        # D1.4: comment lines are prose, not build wiring -- the rescued OmWren* source names
        # and historical D1.0 comments must not keep this coupling red for ever.
        code = "\n".join(ln for ln in text.splitlines() if not ln.lstrip().startswith("#"))
        n = len(re.findall(r"(?i)\bwren\b", code))
        if n:
            hits[rel] = n
    link = _grep(root, "src/omnisim/Makefile", r"LIB_WREN\s*=", code_only=False)
    srcs = _grep(root, "src/omnisim/Makefile", r"^\s*OmWren[A-Za-z0-9]*\.cpp", code_only=False)
    status = "HOLDS" if hits else "NO-LONGER-HOLDS"
    return (
        status,
        f"{len(hits)} tracked makefile(s) name WREN: "
        + ", ".join(f"{k} ({v})" for k, v in sorted(hits.items()))
        + f"; src/omnisim/Makefile LIB_WREN at lines {[ln for ln, _ in link]}; "
        f"{len(srcs)} OmWren*.cpp entries in the source lists; the `wren` vpath entry and the "
        f"`%.o: wren/%.cpp` rules also go.",
        "the deletion commit must also strip: the src/wren sub-make, LIB_WREN (libwren.a + libglad.a), "
        "WREN_INCLUDE, the WB_WREN_INCLUDE variable, every OmWren*.cpp source entry, the wren vpath "
        "and the wren/%.cpp compile rules.",
    )


def coupling_camera_default_backend(root: Path, files: Sequence[str]):
    rel = "resources/nodes/Camera.wrl"
    hits = _grep(root, rel, r"renderBackend", code_only=False)
    if not hits:
        return "NO-LONGER-HOLDS", f"{rel} declares no renderBackend field", "none"
    line_no, text = hits[0]
    quoted_wren = '"wren"'
    quoted_wgpu = '"wgpu"'

    def declared_default(line: str) -> str:
        """The field's actual VALUE, not 'does the line mention wren anywhere'.

        The .wrl line documents the alternatives in its trailing comment, so once the default
        flipped to "wgpu" the line still CONTAINED '"wren"' -- and a substring test on the whole
        line reported the flip had never happened. Strip the comment, then take the first quoted
        token after the field name. Same failure class as an over-broad absent-marker: a test is
        only as good as the text it is allowed to match.
        """
        code = line.split("#", 1)[0]
        idx = code.find("renderBackend")
        if idx < 0:
            return ""
        rest = code[idx + len("renderBackend"):]
        first = rest.find('"')
        if first < 0:
            return ""
        second = rest.find('"', first + 1)
        return rest[first:second + 1] if second > first else ""

    default_wren = declared_default(text) == quoted_wren
    vp = _grep(root, "resources/nodes/Viewpoint.wrl", r"renderBackend", code_only=False)
    vp_default = vp[0][1] if vp else ""
    shown_default = declared_default(text) or "(unparseable)"
    vp_shown = "wgpu" if quoted_wgpu in vp_default else (vp_default[:60] or "(none)")
    status = "HOLDS" if default_wren else "NO-LONGER-HOLDS"
    return (
        status,
        f"{rel}:{line_no} defaults Camera.renderBackend to {shown_default} -- so EVERY Camera device "
        f"(sensor RTT, capture/cinema) renders "
        + ("WREN unless a world opts out" if default_wren
           else "wgpu (post-F1 an authored \"wren\" no longer opts back in: it warns once per node and "
                "renders wgpu; only the wgpu-native-unavailable last resort still lands on WREN)")
        + f". Viewpoint.wrl default: {vp_shown}.",
        "phase W3 -- re-landed at F1 after its two blockers (pen texture P3, Camera post-FX P5) closed. "
        "Flipping this changes what every vision controller, RL consumer and the cinema pipeline sees.",
    )


def coupling_view3d_base_class(root: Path, files: Sequence[str]):
    rel = "src/omnisim/gui/OmView3D.hpp"
    hits = _grep(root, rel, r"class\s+OmView3D\s*:\s*public\s+OmWrenWindow")
    win = "src/omnisim/gui/OmWrenWindow.cpp" in files
    if not hits:
        return (
            "NO-LONGER-HOLDS",
            f"{rel}: OmView3D no longer derives from OmWrenWindow",
            "none",
        )
    return (
        "HOLDS",
        f"{rel}:{hits[0][0]} -- the main 3D pane IS-A OmWrenWindow "
        f"(src/omnisim/gui/OmWrenWindow.cpp {'present' if win else 'MISSING'}), which is a QWindow "
        f"subclass carrying the GL surface, the WREN viewport and the --stream mjpeg feed.",
        "structural: the deletion must re-parent OmView3D onto a WREN-free window base (or lift "
        "OmWrenWindow's non-WREN half out first). This is not a call-site sweep.",
    )


def coupling_ab_instrument(root: Path, files: Sequence[str]):
    """The campaign's own verification gate runs `--arm-b <HATCH>=0`, i.e. it renders a
    WREN arm.  Retiring the hatches also retires the oracle.

    F1 changed what this coupling MEANS: the whole-session WREN selectors
    (OMNISIM_FORCE_WREN / OMNISIM_LEGACY's render arm) are warned no-ops, so no hatch can
    put a session on WREN any more and the A/B oracle's WREN arm is gone BY DESIGN --
    accepted and scheduled, with the WREN reference images captured by the post-baseline
    verification pass on main before the F1 lane merged.  The probe measures which state
    the engine is in rather than assuming: it holds (pre-F1 wording) while
    OmRenderBackend.cpp still carries the force-WREN short-circuit, and reports the
    retirement once the short-circuit is verifiably gone."""
    consumers = []
    hatch_names = [v for v, _ in WREN_SELECTING_HATCHES]
    for rel in files:
        if not (rel.startswith("scripts/") or rel.startswith("tests/")) or not rel.endswith(".py"):
            continue
        text = read_text(root, rel)
        if not text:
            continue
        found = sorted({h for h in hatch_names if h in text})
        if found:
            consumers.append((rel, found))
    if not consumers:
        return "NO-LONGER-HOLDS", "no tooling references a WREN-selecting hatch", "none"
    ab = [c for c in consumers if "render_ab" in c[0] or "oracle" in c[0] or "reversibility" in c[0]]
    rb = read_text(root, "src/omnisim/render/OmRenderBackend.cpp") or ""
    selectors_retired = "RETIRED and IGNORED" in rb and "kForceWren" not in rb
    if selectors_retired:
        return (
            "NO-LONGER-HOLDS",
            "F1 retired the whole-session WREN selectors: OmRenderBackend.cpp warns about and IGNORES "
            "OMNISIM_FORCE_WREN / OMNISIM_LEGACY (verified: the force-WREN short-circuit is gone), so no "
            f"hatch can render a whole-session WREN arm any more. {len(consumers)} tracked script(s) still "
            "drive retired or remaining hatches -- they are the D1.5 cleanup list, e.g. "
            + "; ".join(f"{r} -> {','.join(h)}" for r, h in ab[:4]),
            "ACCEPTED and scheduled (C9): the WREN reference images were captured by the post-baseline "
            "verification pass on main BEFORE the F1 lane merged, so retiring the oracle arm was deliberate. "
            "The sub-path revert hatches (OMNISIM_WGPU_NATIVE_*, OMNISIM_WREN_POSTFX) still A/B their own "
            "slices until D1.4 deletes the WREN arms they select.",
        )
    return (
        "HOLDS",
        f"{len(consumers)} tracked script(s) drive a WREN-selecting hatch; the campaign's own gate "
        f"is among them: " + "; ".join(f"{r} -> {','.join(h)}" for r, h in ab[:4]),
        "every batch in this campaign is gated by `render_ab.py --arm-b <HATCH>=0`, which RENDERS "
        "the WREN arm. Retiring the hatches retires the oracle -- land the reference image sets "
        "BEFORE the hatches go, or the deletion ships with no A/B left to run.",
    )


COUPLING_PROBES: Sequence[Tuple[str, str, bool, object]] = (
    # (id, headline, documented_in_plan, probe)
    ("C1-gl-blit-fallback", "the GL-blit present fallback borrows OmWrenOpenGlContext", True, coupling_gl_blit_fallback),
    ("C2-glad-shared", "src/glad is shared with the wgpu present blit", True, coupling_glad_shared),
    ("C3-packaging-manifest", "packaging manifests ship resources/wren by wildcard", True, coupling_packaging_manifest),
    ("C4-web-viewer-wren", "the streamed web viewer has its OWN WREN", True, coupling_web_viewer_wren),
    ("C5-public-c-abi", "OmSolid support-polygon members back a frozen public C ABI", True, coupling_public_c_abi),
    ("C6-build-system", "the build system links and compiles WREN", False, coupling_build_system),
    ("C7-camera-default-backend", "Camera.renderBackend still defaults to \"wren\"", False, coupling_camera_default_backend),
    ("C8-view3d-base-class", "OmView3D derives from OmWrenWindow", False, coupling_view3d_base_class),
    ("C9-ab-instrument", "the campaign's own A/B gate renders the WREN arm", False, coupling_ab_instrument),
)


# --------------------------------------------------------------------------------------
# The audit
# --------------------------------------------------------------------------------------


class Audit:
    def __init__(self, root: Path, use_git: bool = True):
        self.root = root
        self.files = list_tracked_files(root, use_git)
        # Whether git actually answered: list_tracked_files falls back to a walk when it
        # cannot, and the two scopes must be distinguishable in the report.
        self.used_git = bool(use_git and self.files and _git_answered(root, use_git))
        self.findings: List[Finding] = []
        self.notes: List[str] = []
        self.retirement: Dict[str, dict] = {}
        self.hatches: List[dict] = []
        self.couplings: List[dict] = []
        self.inventory: Dict[str, int] = {}
        # Every wr_* occurrence including comments/strings -- the figure a bare
        # `grep -ohE 'wr_[A-Za-z0-9_]+' | wc -l` produces, so this tool's number can be
        # reconciled with the plan's historical baselines instead of appearing to contradict them.
        self._raw_wr_mentions = 0

    # -- inventory ---------------------------------------------------------------------

    def scan_inventory(self) -> None:
        for label, prefix in (
            ("src/wren", WREN_LIB_DIR),
            ("include/wren", WREN_PUBLIC_INCLUDE_DIR),
            ("resources/wren", WREN_RESOURCE_DIR),
            ("src/omnisim/wren", WREN_SHIM_DIR),
            ("src/glad", GLAD_DIR),
        ):
            self.inventory[label] = len([f for f in self.files if f.startswith(prefix + "/")])
        wren_named_outside = sorted(
            f
            for f in self.files
            if f.startswith(ENGINE_ROOT)
            and not f.startswith(WREN_SHIM_DIR + "/")
            and re.match(r"^OmWren", Path(f).name)
        )
        self.inventory["wren_named_files_outside_shim_dir"] = len(wren_named_outside)
        if wren_named_outside:
            self.notes.append(
                "WREN-named engine files living OUTSIDE "
                + WREN_SHIM_DIR
                + " (they go with the deletion too, and a `git rm -r "
                + WREN_SHIM_DIR
                + "` will miss them): "
                + ", ".join(wren_named_outside)
            )

    # -- retirement probes ---------------------------------------------------------------

    def scan_retirement(self) -> None:
        for domain, (label, probe, owner_gate) in RETIREMENT_PROBES.items():
            ok, ev = probe(self.root, self.files)
            self.retirement[domain] = {
                "label": label,
                "zero_live_consumers": ok,
                "owner_decision_required": owner_gate,
                "evidence": ev,
            }

    def _retirable(self, domain: str) -> Tuple[bool, str]:
        rec = self.retirement.get(domain)
        if rec and rec["zero_live_consumers"]:
            gate = (
                " -- but retiring the FEATURE is an owner decision, not an engineering one"
                if rec["owner_decision_required"]
                else ""
            )
            return True, f"verified this run: {rec['evidence']}{gate}"
        if rec:
            return False, f"probe FAILED this run: {rec['evidence']}"
        return False, ""

    # -- wr_* call sites -----------------------------------------------------------------

    def _remedy_for(self, domain: str, retirable: bool) -> str:
        if not retirable:
            return "port to wgpu, or retire the feature behind an owner decision"
        remedy = "no port needed -- delete the consumer (no live consumer found this run)"
        if self.retirement.get(domain, {}).get("owner_decision_required"):
            remedy += "; gated on an owner decision per the plan's retirement table"
        return remedy

    def scan_wr_calls(self) -> None:
        engine = [
            f for f in self.files if f.startswith(ENGINE_ROOT + "/") and f.endswith(CPP_SUFFIXES)
        ]
        for rel in engine:
            text = read_text(self.root, rel)
            if not text or "wr_" not in text:
                continue
            code = blanked_text(self.root, rel, text)
            code_lines = code.splitlines()
            raw_total = len(WR_SYMBOL_RE.findall(text))
            self._raw_wr_mentions += raw_total
            live: List[Tuple[int, str]] = []
            for idx, line in enumerate(code_lines):
                for m in WR_SYMBOL_RE.finditer(line):
                    live.append((idx + 1, m.group(0)))
            if raw_total == 0:
                continue
            domain, note = domain_of(rel)
            sub = subsystem_of(rel)
            # Raw text, not `code`: the env-var name lives INSIDE a string literal, which
            # blank_noncode() empties. Matching the blanked text found nothing and silently
            # reported zero hatch-gated files -- the exact flattering-number failure this
            # tool exists to prevent.
            hatches = sorted({v for v, _ in WREN_SELECTING_HATCHES if f'"{v}"' in text})
            if not live:
                self.findings.append(
                    Finding(
                        id=f"wr::{rel}",
                        kind="wr_call",
                        classification="ALREADY-PORTED",
                        path=rel,
                        subsystem=sub,
                        domain=domain,
                        detail=f"{raw_total} wr_* mention(s), all in comments/strings; 0 live code references",
                        evidence=f"comment-only: {sorted(set(WR_SYMBOL_RE.findall(text)))[:6]}",
                        count=0,
                        symbols=sorted(set(WR_SYMBOL_RE.findall(text))),
                        remedy="nothing to do at deletion time (the comment may be stale prose)",
                    )
                )
                continue
            retirable, why = self._retirable(domain)
            cls = "RETIRABLE" if retirable else "BLOCKING"
            ev = f"{len(live)} live code reference(s), {raw_total - len(live)} comment/string mention(s)"
            if retirable:
                ev += f"; {why}"
            if hatches:
                # Deliberately weaker than "reachable only with X": this file READS those
                # hatches, which is not the same as every wr_* site in it being the hatch's
                # WREN arm (OmLidar reads OMNISIM_LIDAR_WGPU, but its WREN path is the
                # DEFAULT -- the hatch opts INTO wgpu). The load-bearing half is the second
                # clause, which holds either way.
                ev += (
                    "; this file reads WREN-selecting hatch(es) "
                    + ", ".join(hatches)
                    + " -- any site behind one is STILL BLOCKING: the fallback is compiled in, "
                    "so the link breaks regardless of which arm the default takes"
                )
            self.findings.append(
                Finding(
                    id=f"wr::{rel}",
                    kind="wr_call",
                    classification=cls,
                    path=rel,
                    subsystem=sub,
                    domain=domain,
                    detail=f"{len(live)} live wr_* code site(s) -- {note}",
                    evidence=ev,
                    count=len(live),
                    lines=sorted({ln for ln, _ in live})[:40],
                    symbols=sorted({s for _, s in live}),
                    hatches=hatches,
                    remedy=self._remedy_for(domain, retirable),
                )
            )

    # -- includes -------------------------------------------------------------------------

    def scan_includes(self) -> None:
        shim_headers = {
            Path(f).name for f in self.files if f.startswith(WREN_SHIM_DIR + "/") and f.endswith((".hpp", ".h"))
        }
        for rel in self.files:
            if not rel.startswith(ENGINE_ROOT + "/") or not rel.endswith(CPP_SUFFIXES):
                continue
            text = read_text(self.root, rel)
            if not text or "#include" not in text:
                continue
            # The blanker empties string literals, so `#include "Foo.hpp"` loses its payload.
            # Use the blanked line only to decide whether the directive is live (a commented-out
            # include blanks to whitespace), and parse the include target from the RAW line.
            code_lines = blanked_text(self.root, rel, text).splitlines()
            raw_lines = text.splitlines()
            angle: List[Tuple[int, str]] = []
            shim: List[Tuple[int, str]] = []
            for idx, raw in enumerate(raw_lines):
                if "#" not in raw or "include" not in raw:
                    continue
                if idx < len(code_lines) and "include" not in code_lines[idx]:
                    continue  # the directive lives inside a comment
                m = WREN_ANGLE_INCLUDE_RE.match(raw)
                if m:
                    angle.append((idx + 1, m.group(1)))
                    continue
                m2 = ANY_INCLUDE_RE.match(raw)
                if m2 and Path(m2.group(1)).name in shim_headers:
                    shim.append((idx + 1, m2.group(1)))
            domain, note = domain_of(rel)
            sub = subsystem_of(rel)
            retirable, why = self._retirable(domain)
            if angle:
                self.findings.append(
                    Finding(
                        id=f"inc-public::{rel}",
                        kind="wren_include",
                        classification="RETIRABLE" if retirable else "BLOCKING",
                        path=rel,
                        subsystem=sub,
                        domain=domain,
                        detail=f"{len(angle)} #include of the public WREN C API (<wren/...>)",
                        evidence=", ".join(f"{h}@{ln}" for ln, h in angle[:8])
                        + (f"; {why}" if retirable else ""),
                        count=len(angle),
                        lines=[ln for ln, _ in angle],
                        symbols=sorted({h for _, h in angle}),
                        remedy=f"{WREN_PUBLIC_INCLUDE_DIR}/ goes with {WREN_LIB_DIR}/; every one of these must be removed or re-pointed",
                    )
                )
            if shim:
                self.findings.append(
                    Finding(
                        id=f"inc-shim::{rel}",
                        kind="shim_include",
                        classification="RETIRABLE" if retirable else "BLOCKING",
                        path=rel,
                        subsystem=sub,
                        domain=domain,
                        detail=f"{len(shim)} #include of a header in {WREN_SHIM_DIR}/",
                        evidence=", ".join(f"{h}@{ln}" for ln, h in shim[:8])
                        + (f"; {why}" if retirable else ""),
                        count=len(shim),
                        lines=[ln for ln, _ in shim],
                        symbols=sorted({Path(h).name for _, h in shim}),
                        remedy=f"the whole {WREN_SHIM_DIR}/ layer goes with the deletion",
                    )
                )

    # -- hatches ---------------------------------------------------------------------------

    def scan_hatches(self) -> None:
        polarities = classify_hatch_polarity(
            self.root, self.files, [v for v, _ in WREN_SELECTING_HATCHES]
        )
        for var, effect in WREN_SELECTING_HATCHES:
            polarity, engine_sites, tooling_sites = polarities[var]
            rule4_ok = polarity in ("value-parsed", "not-found", "not-in-engine")
            # F1: a RETIRED hatch's engine read is a presence test by design (the warn fires
            # when the variable is set at all), so rule 4 does not apply -- but only once
            # MEASURED: every engine read site's file must carry the "RETIRED and IGNORED"
            # warning text. A retired hatch whose read site lacks it (someone reintroduced a
            # real read) falls straight back to the VIOLATION.
            retired = False
            if var in RETIRED_WREN_HATCHES and engine_sites:
                retired = all(
                    "RETIRED and IGNORED" in (read_text(self.root, site.split(":")[0]) or "")
                    for site in engine_sites
                )
                if retired:
                    polarity = f"{polarity} (retired warn-only)"
                    rule4_ok = True
            self.hatches.append(
                {
                    "var": var,
                    "effect": effect,
                    "polarity": polarity,
                    "rule4_value_parsed": rule4_ok,
                    "engine_sites": engine_sites[:6],
                    "engine_site_count": len(engine_sites),
                    "tooling_sites": tooling_sites[:6],
                    "tooling_site_count": len(tooling_sites),
                }
            )
            if polarity == "not-found":
                continue
            note = ""
            if not rule4_ok:
                note = (
                    " -- VIOLATES campaign rule 4 (value-parsed only): the engine read is "
                    f"presence-gated, so `{var}=0` ARMS it instead of disarming it "
                    "(the OMNISIM_REQUIRE_NEWTON trap)"
                )
            # D1.4: a hatch with NO ENGINE READ cannot compile a WREN arm into the binary --
            # whatever tooling still exports it is dead ballast for D1.5's cleanup, not a
            # deletion blocker.
            tooling_only = not engine_sites
            self.findings.append(
                Finding(
                    id=f"hatch::{var}",
                    kind="hatch",
                    # F1: a retired hatch no longer selects a WREN arm -- its engine read is a
                    # verified warn-only no-op, so nothing about it blocks the deletion (its
                    # tooling consumers are cleaned up at D1.5, tracked by C9).
                    classification="ALREADY-PORTED" if (retired or tooling_only) else "BLOCKING",
                    path=engine_sites[0].split(":")[0] if engine_sites else "",
                    subsystem="hatches",
                    domain="W6-flips-and-deprecation",
                    detail=f"WREN-selecting env hatch {var} (engine read: {polarity}){note}",
                    evidence=(
                        f"{effect}; engine read site(s): "
                        + (", ".join(engine_sites[:4]) if engine_sites else "none -- tooling-side only")
                        + f"; {len(tooling_sites)} tooling site(s) drive it"
                        + (f" e.g. {', '.join(tooling_sites[:3])}" if tooling_sites else "")
                    ),
                    count=len(engine_sites) + len(tooling_sites),
                    hatches=[var],
                    remedy=(
                        "retired at F1: the warn-only read goes with D1.5's tooling cleanup"
                        if retired
                        else "the hatch must be RETIRED before deletion, and retiring it means deleting the "
                        "WREN arm it selects -- until then every wr_* call it guards stays compiled in"
                    ),
                )
            )

    # -- couplings -------------------------------------------------------------------------

    def scan_couplings(self) -> None:
        for cid, headline, documented, probe in COUPLING_PROBES:
            try:
                status, evidence, remedy = probe(self.root, self.files)  # type: ignore[misc]
            except Exception as exc:  # a probe must never take the whole audit down
                status, evidence, remedy = "PROBE-ERROR", f"{type(exc).__name__}: {exc}", ""
            self.couplings.append(
                {
                    "id": cid,
                    "headline": headline,
                    "documented_in_plan": documented,
                    "status": status,
                    "evidence": evidence,
                    "remedy": remedy,
                }
            )
            if status in ("HOLDS", "CHANGED", "PROBE-ERROR"):
                self.findings.append(
                    Finding(
                        id=f"coupling::{cid}",
                        kind="coupling",
                        classification="BLOCKING",
                        path="",
                        subsystem="non-code",
                        domain="W7-deletion-commit",
                        detail=f"[{status}] {headline}",
                        evidence=evidence,
                        count=1,
                        remedy=remedy,
                    )
                )

    # -- driver -----------------------------------------------------------------------------

    def run(self) -> None:
        self.scan_inventory()
        self.scan_retirement()
        self.scan_wr_calls()
        self.scan_includes()
        self.scan_hatches()
        self.scan_couplings()

    # -- aggregation --------------------------------------------------------------------------

    def summary(self) -> dict:
        blocking = [f for f in self.findings if f.classification == "BLOCKING"]
        retirable = [f for f in self.findings if f.classification == "RETIRABLE"]
        ported = [f for f in self.findings if f.classification == "ALREADY-PORTED"]
        wr_live = sum(f.count for f in self.findings if f.kind == "wr_call")
        wr_blocking = sum(f.count for f in blocking if f.kind == "wr_call")
        wr_retirable = sum(f.count for f in retirable if f.kind == "wr_call")
        hatch_gated_files = [f for f in self.findings if f.kind == "wr_call" and f.hatches]
        return {
            "blocking_findings": len(blocking),
            "retirable_findings": len(retirable),
            "already_ported_findings": len(ported),
            "total_findings": len(self.findings),
            "wr_raw_mentions_incl_comments": self._raw_wr_mentions,
            "wr_comment_or_string_only_mentions": self._raw_wr_mentions - wr_live,
            "wr_live_code_sites": wr_live,
            "wr_live_code_sites_blocking": wr_blocking,
            "wr_live_code_sites_retirable": wr_retirable,
            "wr_files": len({f.path for f in self.findings if f.kind == "wr_call" and f.count}),
            "public_wren_includes": sum(f.count for f in self.findings if f.kind == "wren_include"),
            "shim_includes": sum(f.count for f in self.findings if f.kind == "shim_include"),
            "hatch_gated_wr_files": len(hatch_gated_files),
            "couplings_holding": len([c for c in self.couplings if c["status"] in ("HOLDS", "CHANGED")]),
            "couplings_refuted": len([c for c in self.couplings if c["status"] == "NO-LONGER-HOLDS"]),
            "hatches_not_value_parsed": len(
                [h for h in self.hatches if not h["rule4_value_parsed"] and h["polarity"] != "not-found"]
            ),
            "deletion_ready": len(blocking) == 0 and len(retirable) == 0,
        }

    def by_domain(self) -> Dict[str, dict]:
        out: Dict[str, dict] = {}
        for f in self.findings:
            rec = out.setdefault(
                f.domain,
                {"blocking": 0, "retirable": 0, "already_ported": 0, "wr_sites": 0, "files": set()},
            )
            key = f.classification.lower().replace("-", "_")
            rec[key] = rec.get(key, 0) + 1
            if f.kind == "wr_call":
                rec["wr_sites"] += f.count
            if f.path:
                rec["files"].add(f.path)
        for rec in out.values():
            rec["files"] = len(rec["files"])
        return out

    def by_subsystem(self) -> Dict[str, dict]:
        out: Dict[str, dict] = {}
        for f in self.findings:
            if f.kind != "wr_call":
                continue
            rec = out.setdefault(f.subsystem, {"wr_sites": 0, "files": 0})
            rec["wr_sites"] += f.count
            if f.count:
                rec["files"] += 1
        return out

    def to_json(self) -> dict:
        return {
            "tool": "wren_deletion_audit",
            "root": str(self.root),
            "files_scanned": len(self.files),
            "scan_scope": "git-tracked" if self.used_git else "walked",
            "summary": self.summary(),
            "counting_rule": COUNTING_RULE,
            "inventory": self.inventory,
            "by_domain": self.by_domain(),
            "by_subsystem": self.by_subsystem(),
            "retirement_probes": self.retirement,
            "hatches": self.hatches,
            "couplings": self.couplings,
            "notes": self.notes,
            "findings": [asdict(f) for f in self.findings],
        }


COUNTING_RULE = (
    "A WREN dependency that survives only as a hatch-off fallback is STILL BLOCKING. The hatch is "
    "the dependency: while OMNISIM_WGPU_NATIVE_MESH=0 can reach wr_static_mesh_read_data, that call "
    "is compiled in and `rm -rf src/wren` does not link. This is exactly why the wr_* count went UP "
    "across W1b/W1c while the DEFAULT path became WREN-free. A finding leaves BLOCKING when the code "
    "is gone, never when the default merely stops using it. ALREADY-PORTED means no live code "
    "reference survives -- comments only."
)


# --------------------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------------------


def _bar(n: int, total: int, width: int = 28) -> str:
    if total <= 0:
        return ""
    filled = max(1, round(width * n / total)) if n else 0
    return "#" * filled + "." * (width - filled)


def report(audit: Audit, top: int, show_all: bool) -> None:
    s = audit.summary()
    w = 92
    print("=" * w)
    print("WREN DELETION-READINESS AUDIT   --   what still blocks `rm -rf src/wren`")
    print("=" * w)
    scope = "git-tracked" if audit.used_git else "walked (--no-git)"
    print(f"root: {audit.root}    files scanned: {len(audit.files)} ({scope})")
    print()
    verdict = "DELETION-READY" if s["deletion_ready"] else "NOT DELETION-READY"
    print(f"  HEADLINE: {s['blocking_findings']} BLOCKING findings   ->   {verdict}")
    print(
        f"            {s['retirable_findings']} RETIRABLE (still breaks the build today; needs a "
        "deletion + sign-off, not a port)"
    )
    print(f"            {s['already_ported_findings']} ALREADY-PORTED (comment-only residue)")
    print()
    print(
        f"            {s['wr_live_code_sites']} live wr_* code sites across {s['wr_files']} files "
        f"({s['wr_live_code_sites_blocking']} blocking / {s['wr_live_code_sites_retirable']} retirable)"
    )
    print(
        f"            {s['wr_raw_mentions_incl_comments']} raw wr_* mentions incl. comments "
        f"(what a bare grep reports; {s['wr_comment_or_string_only_mentions']} of them are "
        f"comment/string only)"
    )
    print(
        f"            {s['public_wren_includes']} #include <wren/...>   |   "
        f"{s['shim_includes']} #include of {WREN_SHIM_DIR}/*"
    )
    print(
        f"            {s['couplings_holding']} non-code couplings still hold "
        f"({s['couplings_refuted']} refuted by this run)"
    )
    print()
    print("-" * w)
    print("COUNTING RULE (read this before quoting the headline)")
    print("-" * w)
    for line in _wrap(COUNTING_RULE, w - 2):
        print("  " + line)
    if s["hatch_gated_wr_files"]:
        print()
        print(
            f"  {s['hatch_gated_wr_files']} file(s) below carry wr_* code that ONLY the hatch-off arm "
            f"reaches. They are counted as BLOCKING."
        )
    if s["hatches_not_value_parsed"]:
        print(
            f"  {s['hatches_not_value_parsed']} WREN-selecting hatch(es) are PRESENCE-GATED, violating "
            f"campaign rule 4 -- see the hatch table."
        )
    print()

    print("-" * w)
    print("INVENTORY (what the deletion physically removes)")
    print("-" * w)
    for k, v in audit.inventory.items():
        print(f"  {k:<38} {v:>6} tracked file(s)")
    print()

    print("-" * w)
    print("BY CAMPAIGN DOMAIN  (wr_* live code sites)")
    print("-" * w)
    dom = audit.by_domain()
    total_sites = sum(d["wr_sites"] for d in dom.values()) or 1
    rows = sorted(dom.items(), key=lambda kv: -kv[1]["wr_sites"])
    print(f"  {'domain':<34} {'sites':>6} {'files':>6}  {'B':>3} {'R':>3} {'P':>3}  distribution")
    for name, rec in rows:
        print(
            f"  {name:<34} {rec['wr_sites']:>6} {rec['files']:>6}  "
            f"{rec.get('blocking', 0):>3} {rec.get('retirable', 0):>3} {rec.get('already_ported', 0):>3}  "
            f"{_bar(rec['wr_sites'], total_sites, 22)}"
        )
    print("  (B = BLOCKING, R = RETIRABLE, P = ALREADY-PORTED findings in that domain)")
    print()

    print("-" * w)
    print("BY SUBSYSTEM  (owning directory)")
    print("-" * w)
    for name, rec in sorted(audit.by_subsystem().items(), key=lambda kv: -kv[1]["wr_sites"]):
        print(f"  {name:<38} {rec['wr_sites']:>6} sites   {rec['files']:>4} files")
    print()

    print("-" * w)
    print("NON-CODE COUPLINGS  (what `grep wr_` cannot see)")
    print("-" * w)
    for c in audit.couplings:
        tag = "plan" if c["documented_in_plan"] else "NEW"
        print(f"  [{c['status']:<15}] {c['id']}  ({tag})")
        print(f"      {c['headline']}")
        for line in _wrap("evidence: " + c["evidence"], w - 8):
            print("      " + line)
        if c["remedy"]:
            for line in _wrap("remedy:   " + c["remedy"], w - 8):
                print("      " + line)
        print()

    print("-" * w)
    print("WREN-SELECTING ENV HATCHES  (live ones keep their WREN arm compiled in; '(retired")
    print("warn-only)' ones are F1 no-ops whose read sites carry the RETIRED-and-IGNORED warning)")
    print("-" * w)
    print(f"  {'variable':<32} {'engine read':<15} {'rule4':<10} {'eng':>4} {'tool':>5}")
    for h in audit.hatches:
        flag = "ok" if h["rule4_value_parsed"] else "VIOLATION"
        print(
            f"  {h['var']:<32} {h['polarity']:<15} {flag:<10} "
            f"{h['engine_site_count']:>4} {h['tooling_site_count']:>5}"
        )
        if h["polarity"] == "not-found":
            continue
        for line in _wrap(h["effect"], w - 8):
            print("      " + line)
        if h["engine_sites"]:
            print("      engine: " + ", ".join(h["engine_sites"][:3]))
        if h["tooling_sites"]:
            print("      tooling (sets/drives it): " + ", ".join(h["tooling_sites"][:3]))
    print("  eng = engine (C/C++) read sites -- these decide the rule-4 verdict.")
    print("  tool = scripts/projects sites, which mostly SET the variable; they are the")
    print("         consumers that stop working when the hatch is retired (coupling C9).")
    print()

    print("-" * w)
    print("RETIREMENT-CANDIDATE PROBES  (do these features have live consumers?)")
    print("-" * w)
    for domain, rec in audit.retirement.items():
        state = "no live consumer" if rec["zero_live_consumers"] else "STILL HAS CONSUMERS"
        gate = "  [owner decision]" if rec["owner_decision_required"] else ""
        print(f"  {domain:<20} {rec['label']:<38} {state}{gate}")
        for line in _wrap(rec["evidence"], w - 8):
            print("      " + line)
    print()

    print("-" * w)
    title = "ALL FINDINGS" if show_all else f"TOP {top} FINDINGS BY WEIGHT"
    print(title)
    print("-" * w)
    order = {"BLOCKING": 0, "RETIRABLE": 1, "ALREADY-PORTED": 2}
    ranked = sorted(audit.findings, key=lambda f: (order[f.classification], -f.count, f.path))
    shown = ranked if show_all else ranked[:top]
    for f in shown:
        loc = f.path or f.subsystem
        print(f"  [{f.classification:<14}] {f.count:>4}x  {loc}")
        print(f"      kind={f.kind}  domain={f.domain}")
        for line in _wrap(f.detail, w - 8):
            print("      " + line)
        for line in _wrap("evidence: " + f.evidence, w - 8):
            print("      " + line)
        if f.hatches:
            print("      hatch-gated by: " + ", ".join(f.hatches) + "  (still BLOCKING)")
        if f.remedy:
            for line in _wrap("remedy: " + f.remedy, w - 8):
                print("      " + line)
        print()
    if not show_all and len(ranked) > top:
        print(f"  ... {len(ranked) - top} more findings; pass --all or --json for the rest.")
        print()

    if audit.notes:
        print("-" * w)
        print("NOTES")
        print("-" * w)
        for n in audit.notes:
            for line in _wrap("* " + n, w - 2):
                print("  " + line)
        print()

    print("=" * w)
    print(
        f"VERDICT: {verdict}   ({s['blocking_findings']} blocking, {s['retirable_findings']} retirable)"
    )
    print(
        "Re-run this command after every batch. The number is the gate; nobody's memory is."
    )
    print("=" * w)


def _wrap(text: str, width: int) -> List[str]:
    words = text.split()
    lines: List[str] = []
    cur = ""
    for wd in words:
        if cur and len(cur) + 1 + len(wd) > width:
            lines.append(cur)
            cur = wd
        else:
            cur = f"{cur} {wd}".strip()
    if cur:
        lines.append(cur)
    return lines or [""]


# --------------------------------------------------------------------------------------
# Self-test -- prove the auditor can go red.
# --------------------------------------------------------------------------------------

_SELFTEST_LIVE = """// seeded fixture: a real call the deletion would break
#include <wren/static_mesh.h>
#include "OmWrenRenderingContext.hpp"

void selftestDraw() {
  WrStaticMesh *m = wr_static_mesh_unit_box_new(false);
  wr_static_mesh_delete(m);
}
"""

_SELFTEST_COMMENT_ONLY = """// This file used to call wr_transform_get_matrix and wr_static_mesh_read_data.
/* block comment mentioning wr_scene_render too */
const char *kMsg = "wr_static_mesh_new failed";  // string mention, not a call
void selftestPorted() {}
"""

_SELFTEST_CLEAN = """void nothingToSeeHere() { int x = 1; (void)x; }
"""

# The load-bearing fixture: a wgpu-default path whose WREN arm is reachable only with the
# hatch off.  The whole point of the tool is that this still counts as BLOCKING.
_SELFTEST_HATCH_FALLBACK = """#include <wren/static_mesh.h>

void collect() {
  static const bool sNative = !qEnvironmentVariableIsSet("OMNISIM_WGPU_NATIVE_MESH") ||
                              qEnvironmentVariableIntValue("OMNISIM_WGPU_NATIVE_MESH") != 0;
  if (sNative)
    return;  // the wgpu default: no WREN
  wr_static_mesh_read_data(mesh, coords, normals, uvs, indices);
}
"""


def _write(p: Path, text: str) -> None:
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(text, encoding="utf-8")


def self_test() -> int:
    """Build a synthetic tree, run the real scanner over it, and assert it goes red.

    Deliberately off to the side of the repo: this campaign shares one working tree with
    other lanes, so the self-test must never write a file into src/.
    """
    failures: List[str] = []

    def check(name: str, cond: bool, detail: str = "") -> None:
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))
        if not cond:
            failures.append(name)

    tmp = Path(tempfile.mkdtemp(prefix="wren_audit_selftest_"))
    try:
        # --- unit: the comment/string blanker -------------------------------------------
        blanked = blank_noncode(_SELFTEST_COMMENT_ONLY)
        check(
            "blanker removes comment + string wr_ mentions",
            not WR_SYMBOL_RE.search(blanked),
            f"residual={WR_SYMBOL_RE.findall(blanked)}",
        )
        blanked_live = blank_noncode(_SELFTEST_LIVE)
        check(
            "blanker preserves real calls",
            "wr_static_mesh_unit_box_new" in blanked_live,
        )
        check(
            "blanker preserves line numbering",
            len(blank_noncode(_SELFTEST_COMMENT_ONLY).splitlines())
            == len(_SELFTEST_COMMENT_ONLY.splitlines()),
        )

        # --- RED arm: a tree that seeds a fake wr_ call ---------------------------------
        red = tmp / "red"
        _write(red / ENGINE_ROOT / "nodes" / "OmSelfTestLive.cpp", _SELFTEST_LIVE)
        _write(red / ENGINE_ROOT / "nodes" / "OmSelfTestPorted.cpp", _SELFTEST_COMMENT_ONLY)
        _write(red / ENGINE_ROOT / "nodes" / "OmSelfTestHatch.cpp", _SELFTEST_HATCH_FALLBACK)
        _write(red / WREN_SHIM_DIR / "OmWrenRenderingContext.hpp", "#pragma once\n")
        _write(red / WREN_LIB_DIR / "Scene.cpp", "// the library under audit\n")
        a = Audit(red, use_git=False)
        a.run()
        s = a.summary()

        live = [f for f in a.findings if f.kind == "wr_call" and "OmSelfTestLive" in f.path]
        check("seeded fake wr_ call is REPORTED", len(live) == 1, f"findings={len(live)}")
        check(
            "seeded fake wr_ call is BLOCKING",
            bool(live) and live[0].classification == "BLOCKING",
            live[0].classification if live else "no finding",
        )
        check(
            "seeded call counted exactly twice (both real calls, no comment)",
            bool(live) and live[0].count == 2,
            f"count={live[0].count if live else 'n/a'}",
        )
        check(
            "seeded symbols captured",
            bool(live) and "wr_static_mesh_unit_box_new" in live[0].symbols,
        )
        ported = [f for f in a.findings if f.kind == "wr_call" and "OmSelfTestPorted" in f.path]
        check(
            "comment-only file classified ALREADY-PORTED, not BLOCKING",
            len(ported) == 1 and ported[0].classification == "ALREADY-PORTED" and ported[0].count == 0,
            f"{[ (p.classification, p.count) for p in ported ]}",
        )
        inc = [f for f in a.findings if f.kind == "wren_include" and "OmSelfTestLive" in f.path]
        check("seeded #include <wren/...> is REPORTED", len(inc) == 1 and inc[0].count == 1)
        shim = [f for f in a.findings if f.kind == "shim_include" and "OmSelfTestLive" in f.path]
        check(f"seeded #include of {WREN_SHIM_DIR}/* is REPORTED", len(shim) == 1)
        check("RED arm headline is non-zero", s["blocking_findings"] > 0, str(s["blocking_findings"]))
        check("RED arm is NOT deletion-ready", not s["deletion_ready"])

        # The anti-flattery property: a WREN arm reachable only with the hatch off is
        # BLOCKING, and is TAGGED with the hatch that guards it.
        hf = [f for f in a.findings if f.kind == "wr_call" and "OmSelfTestHatch" in f.path]
        check("hatch-off WREN fallback is REPORTED", len(hf) == 1, f"findings={len(hf)}")
        check(
            "hatch-off WREN fallback is BLOCKING (not counted as ported)",
            bool(hf) and hf[0].classification == "BLOCKING",
            hf[0].classification if hf else "no finding",
        )
        check(
            "hatch-off fallback is TAGGED with its guarding hatch",
            bool(hf) and "OMNISIM_WGPU_NATIVE_MESH" in hf[0].hatches,
            str(hf[0].hatches) if hf else "n/a",
        )
        check(
            "summary counts the hatch-gated file",
            s["hatch_gated_wr_files"] >= 1,
            str(s["hatch_gated_wr_files"]),
        )
        check(
            "hatch polarity is read from the ENGINE site (value-parsed here)",
            any(h["var"] == "OMNISIM_WGPU_NATIVE_MESH" and h["polarity"] == "value-parsed" for h in a.hatches),
            str([(h["var"], h["polarity"]) for h in a.hatches if h["engine_site_count"]]),
        )

        # --- GREEN arm: nothing to find -------------------------------------------------
        green = tmp / "green"
        _write(green / ENGINE_ROOT / "nodes" / "OmClean.cpp", _SELFTEST_CLEAN)
        b = Audit(green, use_git=False)
        b.run()
        sb = b.summary()
        check("GREEN arm reports 0 live wr_ sites", sb["wr_live_code_sites"] == 0, str(sb["wr_live_code_sites"]))
        check(
            "GREEN arm reports 0 wr_/include findings",
            len([f for f in b.findings if f.kind in ("wr_call", "wren_include", "shim_include")]) == 0,
        )
        check(
            "GREEN arm still refuses to call an empty tree deletion-ready via couplings",
            isinstance(sb["deletion_ready"], bool),
        )

        # --- the real tree ---------------------------------------------------------------
        here = Path(__file__).resolve().parents[2]
        if (here / ENGINE_ROOT).is_dir():
            c = Audit(here, use_git=True)
            c.run()
            sc = c.summary()
            check(
                "real tree scan produces findings (instrument is live, not vacuously green)",
                sc["total_findings"] > 0,
                f"{sc['total_findings']} findings, {sc['wr_live_code_sites']} wr_ sites",
            )
            check(
                "real tree excludes build/worktree/benchmark-workspace copies",
                not any(
                    "/build/" in f.path or "worktrees" in f.path or "agentbench/results" in f.path
                    for f in c.findings
                    if f.path
                ),
            )
        else:
            print("  [SKIP] real-tree scan (not run from a checkout)")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print()
    if failures:
        print(f"SELF-TEST FAILED: {len(failures)} check(s) -- {', '.join(failures)}")
        return 1
    print("SELF-TEST PASSED: the auditor reports a seeded fake wr_ call as BLOCKING,")
    print("does not count comment/string mentions as code, and goes green on a clean tree.")
    return 0


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        prog="wren_deletion_audit.py",
        description="Enumerate and classify everything that still blocks `rm -rf src/wren`.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Classifications:\n"
            "  BLOCKING        breaks the deletion today AND needs engineering to clear\n"
            "  RETIRABLE       breaks the deletion today but needs no port (zero live consumers,\n"
            "                  verified by a probe in this run)\n"
            "  ALREADY-PORTED  only comment/string residue survives\n\n"
            "A hatch-off fallback is BLOCKING, not ported. See --json .counting_rule.\n"
        ),
    )
    ap.add_argument("--root", default=None, help="repo root (default: this script's repo)")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    ap.add_argument("--all", action="store_true", help="print every finding, not just the top slice")
    ap.add_argument("--top", type=int, default=25, help="how many findings to print (default 25)")
    ap.add_argument("--no-git", action="store_true", help="walk the tree instead of using git ls-files")
    ap.add_argument("--self-test", action="store_true", help="prove the auditor can go red, then exit")
    ap.add_argument(
        "--fail-on-blocking",
        action="store_true",
        help="exit 1 while anything still blocks the deletion (the W7 gate)",
    )
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test()

    root = Path(args.root).resolve() if args.root else Path(__file__).resolve().parents[2]
    if not (root / ENGINE_ROOT).is_dir():
        print(f"error: {root} does not look like an OmniSim checkout ({ENGINE_ROOT} missing)", file=sys.stderr)
        return 2

    audit = Audit(root, use_git=not args.no_git)
    audit.run()

    if args.json:
        print(json.dumps(audit.to_json(), indent=2, sort_keys=False))
    else:
        report(audit, top=args.top, show_all=args.all)

    if args.fail_on_blocking:
        s = audit.summary()
        return 0 if s["deletion_ready"] else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
