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

"""Qt-include ratchet (core-evolution-plan.md, Phase Q2).

Qt is not "the GUI layer" of the engine -- 53% of engine files included it
directly when this ratchet was introduced (2026-07-18), reaching into nodes/,
vrml/, maths/ and physics/. The long-term direction (docs/developer/
core-evolution-plan.md, Axis 2) is a core whose compute directories do not
depend on Qt at all, drained leaf-first without a rewrite.

This test enforces the one rule that makes that direction real: the number of
files that include Qt directly, per src/omnisim subdirectory, may only go DOWN.

- Adding a Qt include to a file that already has one: fine (not measured here).
- Adding a Qt include to a file that had none: FAILS if it raises the
  directory's count above the recorded high-water mark. Prefer std:: types, or
  keep the Qt dependency in a directory that legitimately owns it (gui/,
  widgets/, editor/).
- Removing Qt includes from files: lower the baseline below in the same
  commit -- that is the ratchet clicking one notch tighter.

Run: python tests/test_qt_include_ratchet.py   (or via pytest)
"""

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
ENGINE_ROOT = REPO_ROOT / "src" / "omnisim"

QT_INCLUDE_RE = re.compile(r"^\s*#\s*include\s+<Qt?[A-Z]", re.MULTILINE)

# High-water marks, measured 2026-07-18 (commit 763fbb9e's session). A PR may lower a
# value (do it when you drain a directory!), never raise one. Directories absent from
# this table must stay Qt-free.
BASELINE = {
    "app": 7,
    "compute": 5,
    "control": 6,
    "core": 49,
    "editor": 8,
    "engine": 6,
    "gui": 75,
    "maths": 7,  # 2026-07-19: OmMathsUtilities + OmPolygon drained (Qt containers -> std::)
    "nodes": 92,
    "ode": 1,
    "physics": 0,  # 2026-07-19: OmNewtonBackend drained -- physics/ is Qt-free (Phase Q2 first click)
    "plugins": 4,
    "render": 4,
    "scene_tree": 41,
    "sound": 11,
    "user_commands": 11,
    "util": 2,
    "vrml": 47,
    "widgets": 10,
    "wren": 24,
}


def measure() -> dict:
    """Count files including Qt directly, per top-level src/omnisim subdirectory.

    Excludes the generated build/ tree (moc output). Methodology matches the
    baseline audit: a file counts once no matter how many Qt includes it has.
    """
    counts = {}
    for path in ENGINE_ROOT.rglob("*"):
        if path.suffix not in (".cpp", ".hpp", ".c", ".h"):
            continue
        relative = path.relative_to(ENGINE_ROOT)
        if relative.parts[0] == "build":
            continue
        try:
            text = path.read_text(errors="replace")
        except OSError:
            continue
        if QT_INCLUDE_RE.search(text):
            counts[relative.parts[0]] = counts.get(relative.parts[0], 0) + 1
    return counts


def test_qt_include_ratchet():
    counts = measure()
    regressions = []
    for directory, count in sorted(counts.items()):
        allowed = BASELINE.get(directory, 0)
        if count > allowed:
            regressions.append(
                f"src/omnisim/{directory}: {count} files include Qt directly "
                f"(ratchet allows {allowed})"
            )
    assert not regressions, (
        "Qt-include ratchet violated -- new Qt dependencies added to the engine core:\n  "
        + "\n  ".join(regressions)
        + "\nUse std:: types instead, or keep the Qt dependency in a directory that owns it "
        "(gui/, widgets/, editor/). The ratchet only goes down: see "
        "docs/developer/core-evolution-plan.md Axis 2 / Phase Q2."
    )
    # The pleasant direction: if a directory dropped below its high-water mark, say so,
    # so the author remembers to click the ratchet tighter in the same commit.
    for directory, allowed in sorted(BASELINE.items()):
        count = counts.get(directory, 0)
        if count < allowed:
            print(
                f"NOTE: src/omnisim/{directory} is at {count} Qt-including files, below its "
                f"ratchet of {allowed} -- lower the BASELINE entry in {Path(__file__).name}."
            )


if __name__ == "__main__":
    try:
        test_qt_include_ratchet()
    except AssertionError as error:
        print(f"FAIL: {error}")
        sys.exit(1)
    counts = measure()
    total = sum(counts.values())
    print(f"PASS: Qt-include ratchet holds ({total} Qt-including files across "
          f"{len(counts)} directories).")
