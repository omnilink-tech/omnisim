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

"""Migrate every user-facing .wbt to the canonical OmniSim lighting recipe.

See docs/WORLD_RECIPE.md. Replaces inline `Background`, `DEF SUN DirectionalLight`,
`DEF SUN_MARKER Solid`, and the separate `sun_marker_driver` Robot block with the
three-PROTO recipe: `OmniSimSky`, `DEF SUN OmniSimSun`, `DEF SUN_MARKER OmniSimSunMarker`.

Also strips `NightSky`, `TexturedBackground`, `TexturedBackgroundLight` from
user-facing worlds — the recipe is one canonical look. Test worlds under `tests/`
are exempt.

Idempotent: re-running on a migrated world is a no-op.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

USER_FACING_ROOTS = [
    "projects/samples/demos/worlds",
    "projects/samples/devices/worlds",
    "projects/samples/rendering/worlds",
    "projects/samples/geometries/worlds",
    "projects/robots",
    "projects/policies/worlds",
    "projects/languages",
    "distribution/generated_worlds",
    "scripts/rotating_camera/worlds",
]

EXTERNPROTOS = [
    'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
    'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
    'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
]

RECIPE_BLOCK = "\n".join([
    "OmniSimSky { }",
    "DEF SUN OmniSimSun { }",
    "DEF SUN_MARKER OmniSimSunMarker { }",
])

# Block headers we strip (matched at top-level only, brace-balanced).
# Each entry: (regex matching the header line, predicate on body or None).
STRIP_HEADERS = [
    # Background variants
    (re.compile(r"^(?:DEF\s+\w+\s+)?Background\s*\{"), None),
    (re.compile(r"^(?:DEF\s+\w+\s+)?NightSky\s*\{"), None),
    (re.compile(r"^(?:DEF\s+\w+\s+)?TexturedBackground\s*\{"), None),
    (re.compile(r"^(?:DEF\s+\w+\s+)?TexturedBackgroundLight\s*\{"), None),
    # Sun light variants
    (re.compile(r"^DEF\s+SUN\s+DirectionalLight\s*\{"), None),
    # Sun marker visual variants
    (re.compile(r"^DEF\s+SUN_MARKER\s+(?:Solid|Robot)\s*\{"), None),
    # Standalone "sun_marker_driver" Robot
    (re.compile(r"^Robot\s*\{"), lambda body: '"sun_marker_driver"' in body or '"sun_marker"' in body),
]


def skip_strings_and_comments(text: str, i: int) -> int:
    """If text[i] is the start of a comment or string, advance past it. Else return i+1."""
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
                i += 2
            else:
                i += 1
        return i + 1
    return i + 1


def find_block_end(text: str, brace_open_idx: int) -> int:
    """Given the index of an opening '{', return the index *after* the matching '}'."""
    n = len(text)
    depth = 1
    i = brace_open_idx + 1
    while i < n and depth > 0:
        c = text[i]
        if c == "#" or c == '"':
            i = skip_strings_and_comments(text, i)
            continue
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
        i += 1
    return i


def find_top_level_blocks_to_strip(text: str):
    """Yield (start, end, header_match) for every top-level block matching a STRIP_HEADERS entry."""
    n = len(text)
    depth = 0
    i = 0
    while i < n:
        c = text[i]
        # Skip comments and strings
        if c == "#" or c == '"':
            i = skip_strings_and_comments(text, i)
            continue
        if depth == 0 and (c.isalpha() or c == "_"):
            # Possible header at top level. Only consider if at start of line.
            line_start = i == 0 or text[i - 1] == "\n"
            if line_start:
                # Scan ahead until '{' or newline twice (heuristic header span).
                lookahead = text[i:i + 200]
                for header_re, predicate in STRIP_HEADERS:
                    m = header_re.match(lookahead)
                    if m:
                        brace_open = i + m.end() - 1
                        end = find_block_end(text, brace_open)
                        body = text[i:end]
                        if predicate is None or predicate(body):
                            yield (i, end, m)
                            i = end
                            break
                else:
                    i += 1
                    continue
                continue
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
        i += 1


def strip_blocks(text: str) -> tuple[str, int]:
    """Remove all top-level blocks matched by STRIP_HEADERS. Returns (new_text, count)."""
    blocks = list(find_top_level_blocks_to_strip(text))
    if not blocks:
        return text, 0
    # Strip from end to start so indices stay valid. Also swallow trailing newline + any
    # blank lines so we don't leave 6 empty lines per strip.
    out = text
    for start, end, _m in reversed(blocks):
        # Eat trailing whitespace/newlines after the block
        j = end
        while j < len(out) and out[j] in " \t":
            j += 1
        if j < len(out) and out[j] == "\n":
            j += 1
        # Eat leading whitespace on the block's first line if it's at col 0 already (it is by find logic)
        out = out[:start] + out[j:]
    return out, len(blocks)


def already_migrated(text: str) -> bool:
    # Match instantiations, not EXTERNPROTO declarations.
    return bool(
        re.search(r"^\s*OmniSimSky\s*\{", text, re.MULTILINE)
        and re.search(r"OmniSimSun\s*\{", text)
        and re.search(r"OmniSimSunMarker\s*\{", text)
    )


def insert_externprotos(text: str) -> str:
    """Insert the 3 EXTERNPROTOs after the last existing EXTERNPROTO line, or after VRML_SIM header."""
    lines = text.split("\n")
    needed = [e for e in EXTERNPROTOS if e not in text]
    if not needed:
        return text

    # Find the last existing EXTERNPROTO line
    last_externproto_idx = -1
    for i, ln in enumerate(lines):
        if ln.startswith("EXTERNPROTO "):
            last_externproto_idx = i

    if last_externproto_idx >= 0:
        # Insert right after it
        insert_at = last_externproto_idx + 1
    else:
        # Insert after #VRML_SIM line (line 0), possibly past any leading comments
        insert_at = 1
        while insert_at < len(lines) and (
            lines[insert_at].startswith("#") or lines[insert_at].strip() == ""
        ):
            insert_at += 1

    new_lines = lines[:insert_at] + needed + lines[insert_at:]
    return "\n".join(new_lines)


def insert_recipe_block(text: str) -> str:
    """Insert OmniSimSky / SUN / SUN_MARKER instantiation block.

    Strategy: insert it on the line after the close-brace of Viewpoint{}.
    If Viewpoint isn't found, insert after WorldInfo{}. If neither, insert at top.
    """
    # Check for the *instantiations*, not just the name (which appears in
    # EXTERNPROTO declarations too).
    if (re.search(r"^\s*OmniSimSky\s*\{", text, re.MULTILINE)
            and re.search(r"OmniSimSun\s*\{", text)
            and re.search(r"OmniSimSunMarker\s*\{", text)):
        return text

    # Find Viewpoint
    m = re.search(r"^Viewpoint\s*\{", text, re.MULTILINE)
    anchor_node = m
    if not m:
        m = re.search(r"^WorldInfo\s*\{", text, re.MULTILINE)
        anchor_node = m
    if not m:
        # Just prepend
        return RECIPE_BLOCK + "\n" + text

    brace_open = text.find("{", m.start())
    end = find_block_end(text, brace_open)
    # Skip a trailing newline after the close brace
    insert_at = end
    if insert_at < len(text) and text[insert_at] == "\n":
        insert_at += 1
    return text[:insert_at] + RECIPE_BLOCK + "\n" + text[insert_at:]


def read_text_normalized(path: Path) -> tuple[str, str]:
    """Read text with all line endings normalized to '\\n'. Return (text, original_eol).

    Handles broken `\\r\\r\\n` triples that appear in Windows checkouts of files
    that were already CRLF in the index combined with core.autocrlf=true.
    Non-UTF-8 bytes are preserved as surrogate escapes for byte-faithful roundtrip.
    """
    raw = path.read_bytes().decode("utf-8", errors="surrogateescape")
    # Normalize: collapse \r\r\n → \r\n, then \r\n → \n, then lone \r → \n
    raw = raw.replace("\r\r\n", "\r\n")
    if "\r\n" in raw:
        eol = "\r\n"
        text = raw.replace("\r\n", "\n")
    elif "\r" in raw and "\n" not in raw:
        eol = "\r"
        text = raw.replace("\r", "\n")
    else:
        eol = "\n"
        text = raw
    text = text.replace("\r", "")
    return text, eol


def write_text_with_eol(path: Path, text: str, eol: str) -> None:
    out = text.replace("\n", eol) if eol != "\n" else text
    path.write_bytes(out.encode("utf-8", errors="surrogateescape"))


def migrate(path: Path) -> tuple[bool, str]:
    text, eol = read_text_normalized(path)
    original = text

    if already_migrated(text):
        # Still re-write to fix any \r\r\n damage in the file
        write_text_with_eol(path, text, eol)
        return False, "already-migrated"

    text, n_stripped = strip_blocks(text)
    text = insert_externprotos(text)
    text = insert_recipe_block(text)

    # Collapse 3+ consecutive blank lines into 2
    text = re.sub(r"\n{3,}", "\n\n", text)

    if text == original:
        write_text_with_eol(path, text, eol)
        return False, "no-change"

    write_text_with_eol(path, text, eol)
    return True, f"migrated (stripped {n_stripped} block(s))"


def find_world_files() -> list[Path]:
    found = []
    for root in USER_FACING_ROOTS:
        root_path = REPO_ROOT / root
        if not root_path.exists():
            continue
        for p in root_path.rglob("*.wbt"):
            # Skip anything under tests/
            try:
                rel = p.relative_to(REPO_ROOT).as_posix()
            except ValueError:
                continue
            if rel.startswith("tests/") or "/tests/" in rel:
                continue
            found.append(p)
    return sorted(set(found))


def main():
    only = sys.argv[1] if len(sys.argv) > 1 else None
    worlds = find_world_files()
    if only:
        worlds = [w for w in worlds if only in w.as_posix()]
    print(f"[migrate] {len(worlds)} user-facing world(s) to inspect")
    changed = 0
    for w in worlds:
        ok, status = migrate(w)
        marker = "*" if ok else " "
        print(f"  {marker} {w.relative_to(REPO_ROOT).as_posix()}: {status}")
        if ok:
            changed += 1
    print(f"[migrate] {changed}/{len(worlds)} migrated")


if __name__ == "__main__":
    main()
