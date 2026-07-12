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

"""Bake a standard default camera ``Viewpoint`` into a hand-authored ``.wbt``.

This is the retrofit tool for the viewpoint convention
(docs/developer/viewpoint-convention.md). Procedurally generated worlds get
their camera from the generators (``omniworld.emit.wbt``,
``scripts/dev/gen_*``); this tool is for the hand-authored flagship / showcase /
deploy worlds.

It rewrites *only* the ``orientation`` and ``position`` lines of the world's
``Viewpoint`` block (and ``fieldOfView`` when ``--fov`` is given), leaving every
other field — ``follow``, ``followType``, ``near``, ``bloomThreshold``,
``ambientOcclusionRadius`` — untouched. So follow/cinematic cameras keep their
behaviour; only the *initial* framing is fixed.

Examples
--------
    # Frame a quadruped at the origin (preset radius/look-height for its class):
    set_viewpoint.py projects/policies/research/worlds/spot_walk_deploy.wbt \
        --subject spot --class quadruped

    # Explicit centre + radius, top-down (overview/nav worlds):
    set_viewpoint.py .../husky_fleet_arena.wbt --mode topdown \
        --center 0 0 0.3 --radius 12

    # Re-run a curated batch (the committed retrofit set):
    set_viewpoint.py --batch scripts/dev/viewpoint_targets.json
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src" / "python"))

from omniworld.viewpoint import (  # noqa: E402
    DEFAULT_FOV,
    SUBJECT_PRESETS,
    format_orientation,
    format_position,
    hero_view,
    top_down_view,
)


def _skip_strings_and_comments(text: str, i: int) -> int:
    """If ``text[i]`` opens a comment or string, return the index past it."""
    n = len(text)
    if i >= n:
        return i + 1
    c = text[i]
    if c == "#":
        while i < n and text[i] != "\n":
            i += 1
        return i
    if c == '"':
        i += 1
        while i < n and text[i] != '"':
            if text[i] == "\\":
                i += 1
            i += 1
        return i + 1
    return i + 1


def find_viewpoint_block(text: str) -> tuple[int, int] | None:
    """Return ``(start, end)`` char offsets of the top-level Viewpoint block.

    ``end`` is just past the closing brace. Returns ``None`` if not found.
    """
    m = re.search(r"(?m)^[ \t]*(?:DEF\s+\w+\s+)?Viewpoint[ \t]*\{", text)
    if not m:
        return None
    start = m.start()
    i = text.index("{", m.start())
    depth = 0
    while i < len(text):
        c = text[i]
        if c in "#\"":
            i = _skip_strings_and_comments(text, i)
            continue
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return start, i + 1
        i += 1
    return None


def rewrite_block(block: str, orient, eye, fov: float | None) -> str:
    """Swap the orientation/position (and optionally fieldOfView) lines."""
    lines = block.split("\n")
    out: list[str] = []
    have_o = have_p = have_fov = False
    for ln in lines:
        s = ln.strip()
        indent = ln[: len(ln) - len(ln.lstrip())] or "  "
        if s == "orientation" or s.startswith("orientation "):
            out.append(f"{indent}orientation {format_orientation(orient)}")
            have_o = True
        elif s == "position" or s.startswith("position "):
            out.append(f"{indent}position {format_position(eye)}")
            have_p = True
        elif (s == "fieldOfView" or s.startswith("fieldOfView ")) and fov is not None:
            out.append(f"{indent}fieldOfView {fov:.4f}")
            have_fov = True
        else:
            out.append(ln)

    # Insert any missing required fields right after the "Viewpoint {" line.
    if not (have_o and have_p) or (fov is not None and not have_fov):
        patched: list[str] = []
        for ln in out:
            patched.append(ln)
            if ln.rstrip().endswith("{") and "Viewpoint" in ln:
                if not have_o:
                    patched.append(f"  orientation {format_orientation(orient)}")
                if not have_p:
                    patched.append(f"  position {format_position(eye)}")
                if fov is not None and not have_fov:
                    patched.append(f"  fieldOfView {fov:.4f}")
        out = patched
    return "\n".join(out)


_TRANS_RE = re.compile(r"\btranslation\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)")


def _subject_translation(text: str, subject: str) -> tuple[float, float, float]:
    """Read a subject's ``translation``, by ``DEF <subject>`` or ``name "<subject>"``.

    Robots are usually identified by a ``name "..."`` field rather than a DEF, so
    we fall back to the ``translation`` *nearest* the matching ``name`` field.
    """
    m = re.search(
        r"DEF\s+" + re.escape(subject) + r"\b[\s\S]{0,4000}?\btranslation\s+"
        r"([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)",
        text,
    )
    if m:
        return (float(m.group(1)), float(m.group(2)), float(m.group(3)))

    name_match = re.search(r'\bname\s+"' + re.escape(subject) + r'"', text)
    if not name_match:
        raise SystemExit(
            f"could not find 'DEF {subject}' or 'name \"{subject}\"' in world")
    anchor = name_match.start()
    candidates = [(abs(t.start() - anchor),
                   (float(t.group(1)), float(t.group(2)), float(t.group(3))))
                  for t in _TRANS_RE.finditer(text)]
    if not candidates:
        raise SystemExit(f"no 'translation' found near subject {subject!r}")
    return min(candidates, key=lambda c: c[0])[1]


def compute_view(text: str, *, mode: str, center, radius, subject, klass, fov):
    """Resolve (center, radius, mode) from args and return (eye, orient, fov)."""
    look_z = 0.4
    if klass:
        preset = SUBJECT_PRESETS[klass]
        if radius is None:
            radius = preset["radius"]
        look_z = preset["look_z"]
        if mode is None:
            mode = preset["mode"]
    mode = mode or "hero"

    if subject:
        sx, sy, sz = _subject_translation(text, subject)
        center = (sx, sy, sz + look_z)
    elif center is None:
        raise SystemExit("need --subject or --center")

    if radius is None:
        raise SystemExit("need --radius (or --class to supply a default)")

    kw = {} if fov is None else {"fov": fov}
    if mode in ("topdown", "top_down", "overview"):
        eye, orient = top_down_view(center, radius, **kw)
    else:
        eye, orient = hero_view(center, radius, **kw)
    return eye, orient, fov


def apply_to_world(path: Path, *, mode, center, radius, subject, klass, fov,
                   dry_run: bool) -> bool:
    text = path.read_text(encoding="utf-8")
    span = find_viewpoint_block(text)
    if span is None:
        print(f"  SKIP {path} — no Viewpoint block")
        return False
    start, end = span
    block = text[start:end]
    eye, orient, fov = compute_view(
        text, mode=mode, center=center, radius=radius,
        subject=subject, klass=klass, fov=fov)
    new_block = rewrite_block(block, orient, eye, fov)
    if new_block == block:
        print(f"  ok   {path} — already framed")
        return False
    if dry_run:
        print(f"  WOULD update {path}")
        print("    orientation " + format_orientation(orient))
        print("    position    " + format_position(eye))
        return True
    path.write_text(text[:start] + new_block + text[end:], encoding="utf-8")
    print(f"  set  {path} -> position {format_position(eye)}")
    return True


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", nargs="?", help="path to a .wbt world")
    ap.add_argument("--mode", choices=["hero", "topdown"], default=None,
                    help="framing style (default: hero, or the class preset)")
    ap.add_argument("--center", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="explicit look-at point")
    ap.add_argument("--radius", type=float, help="framing (bounding-sphere) radius, metres")
    ap.add_argument("--subject", help="DEF name of the robot to frame (reads its translation)")
    ap.add_argument("--class", dest="klass", choices=sorted(SUBJECT_PRESETS),
                    help="subject class -> default radius/look-height/mode")
    ap.add_argument("--fov", type=float, help="vertical field of view, radians "
                    f"(default {DEFAULT_FOV:.4f})")
    ap.add_argument("--batch", help="JSON list of target dicts to apply")
    ap.add_argument("--dry-run", action="store_true", help="print, do not write")
    args = ap.parse_args(argv)

    if args.batch:
        targets = json.loads(Path(args.batch).read_text(encoding="utf-8"))
        changed = 0
        for t in targets:
            wpath = REPO_ROOT / t["world"]
            print(f"[{t['world']}]")
            changed += apply_to_world(
                wpath,
                mode=t.get("mode"),
                center=tuple(t["center"]) if "center" in t else None,
                radius=t.get("radius"),
                subject=t.get("subject"),
                klass=t.get("class"),
                fov=t.get("fov"),
                dry_run=args.dry_run,
            )
        print(f"\n{changed} world(s) {'would change' if args.dry_run else 'changed'}")
        return 0

    if not args.world:
        ap.error("need a world path or --batch")
    apply_to_world(
        Path(args.world), mode=args.mode, center=tuple(args.center) if args.center else None,
        radius=args.radius, subject=args.subject, klass=args.klass, fov=args.fov,
        dry_run=args.dry_run)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
