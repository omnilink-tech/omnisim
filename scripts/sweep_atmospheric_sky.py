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

# =============================================================================
# RETIRED one-off (2026-07-09) — superseded by the canonical OmniSimSky lighting
# recipe (docs/WORLD_RECIPE.md: OmniSimSky + OmniSimSun + OmniSimSunMarker).
# Do NOT use this for new worlds. Kept for provenance of the completed
# atmospheric-sky migration sweep.
# =============================================================================
"""Rip out TexturedBackground / TexturedBackgroundLight / NightSky from
every .wbt under a given root and replace with the procedural
``Background { atmosphericSky ... }`` pipeline.

Usage:
    # Dry run — show what would change, no writes.
    python scripts/sweep_atmospheric_sky.py --root projects/samples/demos/worlds --dry-run

    # Live sweep.
    python scripts/sweep_atmospheric_sky.py --root projects/samples/demos/worlds

Rules
-----
* `TexturedBackground { ... }`  → `Background { skyColor [...] atmosphericSky "..." }`
* `TexturedBackgroundLight { ... }` → removed (atmosphericSky bakes its own IBL)
* `NightSky { ... }`            → `Background { skyColor [...] atmosphericSky "earth" }`
* Mars-themed worlds (path or filename contains "mars") use the
  "mars" preset.  Night-themed worlds (NightSky source, or filename
  contains "night") use "earth" with a hint to push the sun below
  the horizon (the script flips the first DirectionalLight's z so
  the procedural sky goes dark).
* Worlds that already declare `atmosphericSky` are left alone.
* `EXTERNPROTO` lines for the three replaced protos are stripped.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

EARTH_BG = """Background {
  skyColor [ 0.15 0.20 0.30 ]
  atmosphericSky "earth"
}"""

MARS_BG = """Background {
  skyColor [ 0.60 0.35 0.25 ]
  atmosphericSky "mars"
}"""

NIGHT_BG = """Background {
  skyColor [ 0.02 0.03 0.06 ]
  atmosphericSky "earth"
}"""


def classify(path: Path, text: str) -> str:
    """Return one of 'mars', 'night', 'earth'."""
    name = path.name.lower()
    parts = str(path).lower().replace("\\", "/")
    if "mars" in name or "/mars" in parts:
        return "mars"
    if "NightSky" in text:
        return "night"
    if "night" in name:
        return "night"
    return "earth"


def find_block(text: str, start_re: re.Pattern) -> tuple[int, int] | None:
    """Find the next `<keyword> {` block via brace matching.

    Returns (start, end) byte offsets of the whole `<keyword> { ... }` span,
    or None if no match.  Handles nested braces.
    """
    m = start_re.search(text)
    if not m:
        return None
    open_idx = text.index("{", m.end() - 1)
    depth = 0
    i = open_idx
    n = len(text)
    while i < n:
        c = text[i]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return (m.start(), i + 1)
        i += 1
    return None


def strip_externprotos(text: str) -> str:
    for name in ("TexturedBackground", "TexturedBackgroundLight", "NightSky"):
        text = re.sub(
            rf'^EXTERNPROTO\s+"[^"]*{name}\.proto"\s*\n',
            "",
            text,
            flags=re.MULTILINE,
        )
    return text


def replace_block(text: str, keyword: str, replacement: str | None) -> tuple[str, int]:
    """Replace every standalone `<keyword> { ... }` block.  If replacement
    is None, just remove the block.  Returns (new_text, count).
    """
    pattern = re.compile(rf"^(DEF\s+\S+\s+)?{keyword}\s*\{{", re.MULTILINE)
    count = 0
    while True:
        span = find_block(text, pattern)
        if span is None:
            break
        start, end = span
        # Eat the trailing newline after `}` if present, so we don't leave
        # blank lines stacked up.
        if end < len(text) and text[end] == "\n":
            end += 1
        text = text[:start] + (replacement + "\n" if replacement else "") + text[end:]
        count += 1
    return text, count


def flip_first_directional_light_z(text: str) -> tuple[str, bool]:
    """For night worlds: push the sun below the horizon by flipping the
    z component of the first DirectionalLight's `direction` field to be
    positive (light travels upward → sun below).
    """
    span = find_block(text, re.compile(r"^(DEF\s+\S+\s+)?DirectionalLight\s*\{", re.MULTILINE))
    if span is None:
        return text, False
    start, end = span
    block = text[start:end]
    m = re.search(r"(direction\s+)(-?\d*\.?\d+)\s+(-?\d*\.?\d+)\s+(-?\d*\.?\d+)", block)
    if not m:
        return text, False
    x, y, z = float(m.group(2)), float(m.group(3)), float(m.group(4))
    # Force z > 0 (sun below horizon).  If it's already positive, leave
    # alone; if negative or zero, flip.
    if z >= 0.05:
        return text, False
    new_block = block[:m.start()] + f"{m.group(1)}{x:g} {y:g} {abs(z) + 0.1:g}" + block[m.end():]
    return text[:start] + new_block + text[end:], True


def insert_background_after_viewpoint(text: str, bg: str) -> tuple[str, bool]:
    """If the file has a Viewpoint block but no Background, insert bg
    right after the Viewpoint closing brace.
    """
    if re.search(r"^(DEF\s+\S+\s+)?Background\s*\{", text, re.MULTILINE):
        return text, False
    vp_span = find_block(text, re.compile(r"^(DEF\s+\S+\s+)?Viewpoint\s*\{", re.MULTILINE))
    if vp_span is None:
        return text, False
    _, end = vp_span
    if end < len(text) and text[end] == "\n":
        end += 1
    return text[:end] + bg + "\n" + text[end:], True


# Default DirectionalLight inserted when a world has none.  Golden-
# hour-ish: warm white, side-lit, casts shadows.  Atmospheric sky
# picks up this direction to position the sun.
DEFAULT_SUN_EARTH = """DirectionalLight {
  color 1.0 0.95 0.85
  direction -0.5 -0.5 -0.3
  intensity 2.5
  castShadows TRUE
}"""

DEFAULT_SUN_MARS = """DirectionalLight {
  color 1.0 0.78 0.62
  direction -0.5 -0.5 -0.3
  intensity 2.0
  castShadows TRUE
}"""

DEFAULT_SUN_NIGHT = """DirectionalLight {
  color 0.7 0.75 0.95
  direction -0.3 -0.3 0.4
  intensity 0.4
  castShadows TRUE
}"""


def has_directional_light(text: str) -> bool:
    return bool(re.search(r"^(DEF\s+\S+\s+)?DirectionalLight\s*\{", text, re.MULTILINE))


SUN_MARKER_BLOCK = """DEF SUN_MARKER Solid {
  translation 0 0 8
  name "sun_marker"
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 1.0 0.9 0.55
        emissiveColor 1.0 0.85 0.45
        emissiveIntensity 35
        roughness 1.0
        metalness 0.0
      }
      geometry Sphere {
        radius 0.9
        subdivision 3
      }
      castShadows FALSE
    }
  ]
}
Robot {
  name "sun_marker_driver"
  controller "sun_marker"
  supervisor TRUE
  children [
  ]
}
"""


def has_sun_marker(text: str) -> bool:
    return ("DEF SUN_MARKER" in text or
            'controller "sun_marker"' in text or
            'controller "sun_control"' in text)


def def_first_directional_light_as_sun(text: str) -> tuple[str, bool]:
    """Prepend `DEF SUN ` to the first DirectionalLight if it isn't
    already DEF'd.  Returns (new_text, did_change).
    """
    # Already DEF'd as SUN — leave alone.
    if re.search(r"DEF\s+SUN\s+DirectionalLight\s*\{", text):
        return text, False
    # Find the first DirectionalLight not preceded by DEF.
    m = re.search(r"^DirectionalLight\s*\{", text, re.MULTILINE)
    if m is None:
        return text, False
    return text[:m.start()] + "DEF SUN " + text[m.start():], True


def insert_sun_marker_after_directional_light(text: str) -> tuple[str, bool]:
    """Insert the SUN_MARKER Solid + sun_marker_driver Robot block
    right after the first DirectionalLight block's closing brace.
    Handles \n, \r\n line endings.
    """
    span = find_block(text, re.compile(r"^(DEF\s+\S+\s+)?DirectionalLight\s*\{", re.MULTILINE))
    if span is None:
        return text, False
    _, end = span
    # Eat any combination of \r and \n that immediately follows `}`,
    # then re-add a single newline before the marker block so the
    # output lands on its own line regardless of source line endings.
    while end < len(text) and text[end] in "\r\n":
        end += 1
    return text[:end] + SUN_MARKER_BLOCK + text[end:], True


def insert_directional_light_after_background(text: str, light: str) -> tuple[str, bool]:
    """Place a DirectionalLight right after the first Background block.
    If no Background exists, fall back to after Viewpoint."""
    span = find_block(text, re.compile(r"^(DEF\s+\S+\s+)?Background\s*\{", re.MULTILINE))
    if span is None:
        span = find_block(text, re.compile(r"^(DEF\s+\S+\s+)?Viewpoint\s*\{", re.MULTILINE))
    if span is None:
        return text, False
    _, end = span
    if end < len(text) and text[end] == "\n":
        end += 1
    return text[:end] + light + "\n" + text[end:], True


def has_atmospheric_sky(text: str) -> bool:
    return bool(re.search(r'atmosphericSky\s+"[^"]+"', text))


def has_background_block(text: str) -> bool:
    return bool(re.search(r"^(DEF\s+\S+\s+)?Background\s*\{", text, re.MULTILINE))


def add_atmospheric_to_plain_background(text: str, preset: str) -> tuple[str, bool]:
    """If the world has a `Background { ... }` block without an
    `atmosphericSky` field, insert the field right before the
    closing brace.  Preserves any existing skyColor/luminosity as
    fallback values.
    """
    if has_atmospheric_sky(text):
        return text, False
    pattern = re.compile(r"^(DEF\s+\S+\s+)?Background\s*\{", re.MULTILINE)
    span = find_block(text, pattern)
    if span is None:
        return text, False
    start, end = span
    block = text[start:end]
    # Insert before the final `}`.
    close = block.rfind("}")
    if close < 0:
        return text, False
    insertion = f'  atmosphericSky "{preset}"\n'
    new_block = block[:close] + insertion + block[close:]
    return text[:start] + new_block + text[end:], True


def process(path: Path, dry_run: bool, add_marker: bool = False) -> dict:
    """Returns a dict describing what changed (or would change)."""
    raw = path.read_bytes()
    # Use surrogateescape so non-UTF8 bytes (e.g. legacy comment
    # accents) survive the read → write round-trip unchanged.  We
    # only edit ASCII regions (Background block, EXTERNPROTO lines,
    # DirectionalLight block) so the surrogates never get touched.
    try:
        text = raw.decode("utf-8")
        used_surrogate = False
    except UnicodeDecodeError:
        text = raw.decode("utf-8", errors="surrogateescape")
        used_surrogate = True
    original = text
    cls = classify(path, text)
    new_bg = {"mars": MARS_BG, "night": NIGHT_BG, "earth": EARTH_BG}[cls]

    skipped = has_atmospheric_sky(text)

    text = strip_externprotos(text)
    text, n_tb = replace_block(text, "TexturedBackground", new_bg if not skipped else None)
    text, n_tbl = replace_block(text, "TexturedBackgroundLight", None)
    text, n_ns = replace_block(text, "NightSky", new_bg if not skipped else None)
    # Plain-Background path: worlds with `Background { skyColor ... }`
    # but no TexturedBackground / NightSky.  Add `atmosphericSky` to
    # the existing block; don't touch anything else.
    bg_upgraded = False
    if not skipped and n_tb == 0 and n_tbl == 0 and n_ns == 0 and has_background_block(text):
        atmosphere_preset = "mars" if cls == "mars" else "earth"
        text, bg_upgraded = add_atmospheric_to_plain_background(text, atmosphere_preset)
    light_added = False
    if not skipped and (n_tb or n_tbl or n_ns) and not has_directional_light(text):
        default_sun = {"mars": DEFAULT_SUN_MARS,
                       "night": DEFAULT_SUN_NIGHT,
                       "earth": DEFAULT_SUN_EARTH}[cls]
        text, light_added = insert_directional_light_after_background(text, default_sun)
    flipped = False
    if cls == "night" and not skipped:
        text, flipped = flip_first_directional_light_z(text)

    # Optional: add a draggable sun marker + supervisor to every
    # world that has an atmospheric sky and a DirectionalLight but
    # doesn't yet have its own sun-control supervisor.
    sun_def_added = False
    marker_added = False
    if add_marker and has_directional_light(text) and not has_sun_marker(text):
        atmospheric = has_atmospheric_sky(text)
        if atmospheric:
            text, sun_def_added = def_first_directional_light_as_sun(text)
            text, marker_added = insert_sun_marker_after_directional_light(text)

    changed = text != original

    if changed and not dry_run:
        if used_surrogate:
            path.write_bytes(text.encode("utf-8", errors="surrogateescape"))
        else:
            path.write_text(text, encoding="utf-8")

    return {
        "path": str(path),
        "class": cls,
        "skipped_atmospheric": skipped,
        "tb_replaced": n_tb,
        "tbl_removed": n_tbl,
        "ns_replaced": n_ns,
        "bg_upgraded": bg_upgraded,
        "sun_flipped": flipped,
        "light_added": light_added,
        "marker_added": marker_added,
        "changed": changed,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", action="append", required=True,
                    help="Directory to sweep (can pass multiple).")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--sun-marker", action="store_true",
                    help="Also add the draggable sun marker + supervisor.")
    args = ap.parse_args()

    files: list[Path] = []
    for root in args.root:
        root_path = (REPO_ROOT / root).resolve() if not Path(root).is_absolute() else Path(root)
        files.extend([*root_path.rglob("*.omniworld"), *root_path.rglob("*.wbt")])
    files = sorted(set(files))

    stats = {"earth": 0, "mars": 0, "night": 0, "skip": 0,
             "tb_replaced": 0, "tbl_removed": 0, "ns_replaced": 0,
             "sun_flipped": 0, "changed": 0, "skipped_atmospheric": 0,
             "encoding_skipped": 0, "untouched": 0, "total": len(files)}

    for p in files:
        r = process(p, args.dry_run, add_marker=args.sun_marker)
        stats[r["class"]] += 1
        stats["tb_replaced"] += r["tb_replaced"]
        stats["tbl_removed"] += r["tbl_removed"]
        stats["ns_replaced"] += r["ns_replaced"]
        if r["sun_flipped"]:
            stats["sun_flipped"] += 1
        if r["skipped_atmospheric"]:
            stats["skipped_atmospheric"] += 1
        if r.get("encoding_skip"):
            stats["encoding_skipped"] += 1
        if r.get("light_added"):
            stats["light_added"] = stats.get("light_added", 0) + 1
        if r["changed"]:
            stats["changed"] += 1
            if r["tb_replaced"] or r["tbl_removed"] or r["ns_replaced"]:
                print(f"  {p.relative_to(REPO_ROOT)}  cls={r['class']}  "
                      f"TB={r['tb_replaced']} TBL={r['tbl_removed']} NS={r['ns_replaced']}"
                      f"{' SUN' if r['sun_flipped'] else ''}"
                      f"{' +SUN_LIGHT' if r.get('light_added') else ''}")
        else:
            stats["untouched"] += 1

    print()
    print(f"== summary ({'DRY RUN' if args.dry_run else 'APPLIED'}) ==")
    print(f"  files scanned:           {stats['total']}")
    print(f"  files changed:           {stats['changed']}")
    print(f"  files untouched:         {stats['untouched']}")
    print(f"  already atmospheric:     {stats['skipped_atmospheric']}")
    print(f"  TexturedBackground:      {stats['tb_replaced']} blocks replaced")
    print(f"  TexturedBackgroundLight: {stats['tbl_removed']} blocks removed")
    print(f"  NightSky:                {stats['ns_replaced']} blocks replaced")
    print(f"  sun pushed below:        {stats['sun_flipped']} worlds")
    print(f"  by class — earth: {stats['earth']}  mars: {stats['mars']}  night: {stats['night']}")


if __name__ == "__main__":
    main()
