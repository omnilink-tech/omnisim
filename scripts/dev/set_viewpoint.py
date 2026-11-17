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
    set_viewpoint.py projects/policies/research/worlds/omniquad_walk_deploy.omniworld \
        --subject omniquad --class quadruped

    # Explicit centre + radius, top-down (overview/nav worlds):
    set_viewpoint.py .../husky_fleet_arena.omniworld --mode topdown \
        --center 0 0 0.3 --radius 12

    # Re-run a curated batch (the committed retrofit set):
    set_viewpoint.py --batch scripts/dev/viewpoint_targets.json
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "src" / "python"))

from omniworld.validation.viewpoint import (  # noqa: E402
    _half_angles,
    analyze_viewpoint,
)
from omniworld.viewpoint import (  # noqa: E402
    DEFAULT_ASPECT,
    DEFAULT_FOV,
    SUBJECT_PRESETS,
    format_orientation,
    format_position,
    hero_view,
    top_down_view,
)

# Above this aggregate framing radius, --auto prefers a top-down overview: the
# scene has become a layout rather than a portrait of one robot.
AUTO_TOPDOWN_RADIUS_M = 6.0


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


_NUM = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"


def _field_re(name: str, arity: int) -> re.Pattern:
    """Match ``<name> n n ...`` where ``name`` really is a field, not a substring."""
    return re.compile(
        r"(?<![A-Za-z0-9_])" + name + r"\s+" + r"\s+".join([_NUM] * arity)
    )


_ORIENT_RE = _field_re("orientation", 4)
_POSITION_RE = _field_re("position", 3)
_FOV_RE = _field_re("fieldOfView", 1)


def rewrite_block(block: str, orient, eye, fov: float | None) -> str:
    """Swap the orientation/position (and optionally fieldOfView) values.

    Value-span substitution rather than whole-line replacement, so it works on
    the single-line form (``Viewpoint { orientation ... position ... }``, used
    by 23 worlds in this tree) as well as the block form, and it keeps any
    trailing comment on the line. Every other field is untouched.
    """
    def _sub(pattern: re.Pattern, text: str, value: str) -> tuple[str, bool]:
        new, n = pattern.subn(lambda _m: value, text, count=1)
        return new, bool(n)

    block, have_o = _sub(_ORIENT_RE, block, f"orientation {format_orientation(orient)}")
    block, have_p = _sub(_POSITION_RE, block, f"position {format_position(eye)}")
    have_fov = True
    if fov is not None:
        block, have_fov = _sub(_FOV_RE, block, f"fieldOfView {fov:.4f}")

    missing = []
    if not have_o:
        missing.append(f"orientation {format_orientation(orient)}")
    if not have_p:
        missing.append(f"position {format_position(eye)}")
    if not have_fov:
        missing.append(f"fieldOfView {fov:.4f}")
    if not missing:
        return block

    # Insert whatever was absent just after the opening brace, matching the
    # block's own layout (one field per line, or inline for a one-liner).
    open_brace = block.index("{")
    head, tail = block[: open_brace + 1], block[open_brace + 1:]
    if "\n" in block:
        return head + "".join(f"\n  {f}" for f in missing) + tail
    return head + " " + " ".join(missing) + tail


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


def auto_target(path: Path, text: str):
    """Detect the world's subject and return ``(center, radius, mode, label)``.

    Uses the same scene walk as the framing validator
    (``omniworld.validation.viewpoint``), so it sees PROTO-instanced and
    nested/parented robots that the ``--subject`` DEF/name lookup misses, and
    it places them at their true world position rather than their local
    ``translation``. Returns ``None`` when there is nothing to frame.
    """
    verdict = analyze_viewpoint(path, text)
    if verdict.center is None:
        return None
    center, radius = verdict.center, verdict.radius
    if verdict.subject_kind == "robot" and len(verdict.subjects) == 1:
        s = verdict.subjects[0]
        return center, radius, "hero", f"{s.label} ({s.klass})"
    mode = "topdown" if radius > AUTO_TOPDOWN_RADIUS_M else "hero"
    what = (f"{len(verdict.subjects)} robots" if verdict.subjects
            else "scene extent")
    return center, radius, mode, what


def framing_facts(eye, center, radius, fov=None, aspect=None):
    """Measured framing facts for a computed pose: ``(distance, multiple, fill)``.

    * ``distance`` — metres from the eye to the look-at point.
    * ``multiple`` — ``distance / radius``. THE number ``--radius`` callers get
      wrong: a framing radius is the subject's bounding sphere, not a camera
      distance, and at the default 45 deg ``fieldOfView`` on a 16:9 viewport
      the eye lands ~5.7x (hero) / ~5.1x (topdown) further out than the number
      they typed.
    * ``fill`` — the subject's angular radius as a fraction of the tight
      (vertical) half-FOV, i.e. how much of the frame half-height it covers.
      Same definition the framing validator uses, so the two agree.

    Everything is derived from the eye position that was actually computed, not
    re-derived from the framing formula, so this report cannot silently drift
    away from ``omniworld.viewpoint`` if its margins ever change.
    """
    fov = DEFAULT_FOV if fov is None else fov
    aspect = DEFAULT_ASPECT if aspect is None else aspect
    dist = math.dist(tuple(eye), tuple(center))
    _half_h, half_v = _half_angles(fov, aspect)
    if dist <= radius or dist <= 0.0:
        return dist, (dist / radius if radius > 0 else float("inf")), float("inf")
    alpha = math.asin(min(1.0, radius / dist))
    return dist, (dist / radius if radius > 0 else float("inf")), alpha / half_v


def framing_lines(path: Path, new_text: str, *, eye, center, radius, fov, aspect):
    """The report lines printed under every computed viewpoint.

    Two independent statements, deliberately:

    1. What the tool did — distance, the radius multiple, and the fraction of
       the frame the *requested* sphere will cover. Self-consistent by
       construction, but it is what makes the radius->distance relationship
       visible without rendering anything.
    2. What the WORLD says — the framing validator re-parses the rewritten
       world, walks the real scene graph and measures the real bodies. This is
       the check that catches a radius that was never right: a 0.8 m crate
       framed at ``--radius 30`` reads "ok" on line 1 and "speck" on line 2.
    """
    dist, mult, fill = framing_facts(eye, center, radius, fov, aspect)
    fov_used = DEFAULT_FOV if fov is None else fov
    aspect_used = DEFAULT_ASPECT if aspect is None else aspect
    lines = [
        f"    camera distance {dist:.2f} m from centre "
        f"({center[0]:.2f} {center[1]:.2f} {center[2]:.2f}) "
        f"-- {mult:.2f}x the --radius {radius:.2f} m",
    ]
    if math.isinf(fill):
        lines.append("    framing: camera sits INSIDE the framing sphere "
                     f"(--radius {radius:.2f} m >= distance {dist:.2f} m)")
    else:
        lines.append(
            f"    framing: --radius {radius:.2f} m fills {fill * 100:.0f}% of the "
            f"frame half-height at fov {math.degrees(fov_used):.1f} deg, "
            f"aspect {aspect_used:.2f}")
    verdict = analyze_viewpoint(path, new_text, aspect=aspect_used)
    if verdict.subject_kind == "none":
        lines.append(f"    framing: world check -- {verdict.reason}")
    else:
        lines.append(f"    framing: world check -- {verdict.reason} "
                     f"[{verdict.status}]")
        if verdict.status == "broken" and verdict.radius > 0:
            lines.append(f"             the world's own subject radius is "
                         f"{verdict.radius:.2f} m; try --radius "
                         f"{verdict.radius:.2f}")
    return lines


def compute_view(text: str, *, mode: str, center, radius, subject, klass, fov,
                 aspect=None):
    """Resolve (center, radius, mode) and return (eye, orient, fov, center, radius)."""
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
    if aspect is not None:
        kw["aspect"] = aspect
    if mode in ("topdown", "top_down", "overview"):
        eye, orient = top_down_view(center, radius, **kw)
    else:
        eye, orient = hero_view(center, radius, **kw)
    return eye, orient, fov, center, radius


def apply_to_world(path: Path, *, mode, center, radius, subject, klass, fov,
                   dry_run: bool, aspect=None, auto: bool = False) -> bool:
    if not path.is_file():
        # A stale --batch entry must not abort the whole re-run; the list is
        # hand-maintained and worlds get moved or deleted out from under it.
        print(f"  SKIP {path} — world does not exist")
        return False
    text = path.read_text(encoding="utf-8")
    span = find_viewpoint_block(text)
    if span is None:
        print(f"  SKIP {path} — no Viewpoint block")
        return False
    start, end = span
    block = text[start:end]
    if auto and subject is None and center is None:
        detected = auto_target(path, text)
        if detected is None:
            print(f"  SKIP {path} — nothing to frame")
            return False
        center, auto_radius, auto_mode, label = detected
        radius = radius if radius is not None else auto_radius
        mode = mode or auto_mode
        print(f"       auto: {label} @ "
              f"({center[0]:.2f} {center[1]:.2f} {center[2]:.2f}) "
              f"r={radius:.2f} mode={mode}")
    eye, orient, fov, used_center, used_radius = compute_view(
        text, mode=mode, center=center, radius=radius,
        subject=subject, klass=klass, fov=fov, aspect=aspect)
    new_block = rewrite_block(block, orient, eye, fov)
    new_text = text[:start] + new_block + text[end:]
    # Report the framing BEFORE the early return: an "already framed" world is
    # exactly the case where an author wants to know whether the framing it
    # already has is any good.
    report = framing_lines(path, new_text, eye=eye, center=used_center,
                           radius=used_radius, fov=fov, aspect=aspect)
    if new_block == block:
        print(f"  ok   {path} — already framed")
        print("\n".join(report))
        return False
    if dry_run:
        print(f"  WOULD update {path}")
        print("    orientation " + format_orientation(orient))
        print("    position    " + format_position(eye))
        print("\n".join(report))
        return True
    path.write_text(new_text, encoding="utf-8")
    print(f"  set  {path} -> position {format_position(eye)}")
    print("\n".join(report))
    return True


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", nargs="?", help="path to a .wbt world")
    ap.add_argument("--mode", choices=["hero", "topdown"], default=None,
                    help="framing style (default: hero, or the class preset)")
    ap.add_argument("--center", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="explicit look-at point")
    ap.add_argument("--radius", type=float,
                    help="the SUBJECT's bounding-sphere radius in metres -- "
                         "NOT the camera distance. The eye is pushed back far "
                         "enough to fit that sphere in frame, which at the "
                         "default fov/aspect is about 5.7x radius for hero and "
                         "5.1x for topdown (so --radius 30 puts the camera "
                         "~172 m out). Every run prints the resulting distance "
                         "and a framing check")
    ap.add_argument("--subject", help="DEF name of the robot to frame (reads its translation)")
    ap.add_argument("--class", dest="klass", choices=sorted(SUBJECT_PRESETS),
                    help="subject class -> default radius/look-height/mode")
    ap.add_argument("--fov", type=float, help="field of view, radians, on the "
                    f"LARGER viewport dimension per VRML (default {DEFAULT_FOV:.4f})")
    ap.add_argument("--aspect", type=float, help="viewport aspect ratio (width/height) "
                    f"to frame for (default {DEFAULT_ASPECT:.4f}). The framing distance "
                    "is set by the tighter axis, so this matters: a subject framed for "
                    "1:1 overflows a 16:9 window.")
    ap.add_argument("--auto", action="store_true",
                    help="detect the subject (and its class) from the world "
                         "itself -- resolves PROTO-instanced and nested robots "
                         "and uses their true world position")
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
                aspect=t.get("aspect", args.aspect),
                auto=t.get("auto", False),
                dry_run=args.dry_run,
            )
        print(f"\n{changed} world(s) {'would change' if args.dry_run else 'changed'}")
        return 0

    if not args.world:
        ap.error("need a world path or --batch")
    apply_to_world(
        Path(args.world), mode=args.mode, center=tuple(args.center) if args.center else None,
        radius=args.radius, subject=args.subject, klass=args.klass, fov=args.fov,
        aspect=args.aspect, auto=args.auto, dry_run=args.dry_run)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
