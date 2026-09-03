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
#
# 2026-09-02 RE-BASELINE -- the one time two values went UP; read this before
# treating it as precedent. The WREN deletion (976b9449d + 400a14f14,
# 2026-08-23) removed src/omnisim/wren/ (24 Qt-including files) and RELOCATED
# its survivors: OmTesselator.{cpp,hpp}, OmWrenAbstractManipulator.hpp,
# OmWrenLabelOverlay.hpp, OmWrenTextureOverlay.hpp -> nodes/utils/;
# OmWrenOpenGlContext.{cpp,hpp}, OmWrenRenderingContext.hpp -> render/. Every
# one of those eight already included Qt at its old path (verified against
# 7a1efd58d), so no Qt dependency was added by the move -- the ENGINE-WIDE
# count went DOWN, 410 -> 387, while nodes/ and render/ went up. The table
# records the move, and TOTAL below makes a move honest without letting the
# sum creep. Six files inside that +9 are NOT relocations and are owed a
# drain by whoever owns them: nodes/OmCadShape.cpp (<QtCore/QtGlobal>,
# 1c01524ad), nodes/OmCloth.cpp (<QtCore/QFileInfo>, 6f495ca01),
# nodes/OmWgpuSceneRenderer.hpp (<QtCore/QList>, 5cbbb78f2),
# render/OmWgpuSurface.cpp (<QtCore/QString>, 51a1f9f44), and the two new
# nodes/utils/OmWren{Label,Texture}Overlay.cpp split out in 976b9449d.
#
# 2026-09-02 DRAIN of that list, attributed file by file: OmCadShape.cpp
# (<QtGlobal> supplied nothing the file uses -- qDeleteAll is qalgorithms.h,
# reached via its own header's <QtCore/QMap>), OmWgpuSurface.cpp (QString comes
# from OmLog.hpp, whose API names it) and OmWgpuSceneRenderer.hpp (QList only
# ever a pointer in signatures -> forward declaration) are Qt-free: nodes 97 ->
# 95, render 8 -> 7, TOTAL 387 -> 384. Two stay as dated, commented exceptions
# because they use a Qt type as a COMPLETE type with no already-included
# supplier: OmCloth.cpp (QFileInfo) and OmWrenTextureOverlay.cpp (QFileInfo +
# QImageReader) -- the latter is not new at all: wren/OmWrenTextureOverlay.cpp
# already included both before 976b9449d, and OmWrenLabelOverlay.cpp has never
# included Qt directly (it was never in the count).
BASELINE = {
    "app": 6,  # 2026-09-02: 7 -> 6
    "compute": 5,
    "control": 6,
    "core": 48,  # 2026-09-02: 49 -> 48
    "editor": 8,
    "engine": 4,  # 2026-09-02: 6 -> 4 -- OmSimulationCluster deleted with the ODE stub layer
    "gui": 73,  # 2026-09-02: 75 -> 73
    "maths": 7,  # 2026-07-19: OmMathsUtilities + OmPolygon drained (Qt containers -> std::)
    "nodes": 95,  # 2026-09-02: 92 -> 97 (5 relocated from the deleted wren/, 4 new) -> 95 (3 drained, see above)
    "ode": 0,  # 2026-09-02: the directory was deleted with the ODE stub layer; it stays at 0
    "physics": 0,  # 2026-07-19: OmNewtonBackend drained -- physics/ is Qt-free (Phase Q2 first click)
    "plugins": 0,  # 2026-09-02: 4 -> 0 -- drained
    "render": 7,  # 2026-09-02: 4 -> 8 (3 relocated from the deleted wren/, 1 new) -> 7 (drained, see above)
    "scene_tree": 41,
    "sound": 8,  # 2026-09-02: 11 -> 8 -- OmContactSound / OmContactSoundManager deleted (ODE-keyed)
    "user_commands": 11,
    "util": 2,
    "vrml": 47,
    "widgets": 10,
    "wren": 0,  # 2026-08-23: the directory was deleted with WREN (976b9449d); it stays at 0
}

# The engine-wide high-water mark. Per-directory marks alone let a file move
# between directories read as a regression in one and headroom in the other;
# this one makes the direction unambiguous: the SUM never goes up.
TOTAL = 378  # 2026-09-02 (410 at the 2026-07-18 baseline; 387 after the WREN-move re-baseline, 384 after the drain, 378 after the ODE stub-layer deletion)


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
    total = sum(counts.values())
    if total > TOTAL:
        regressions.append(
            f"src/omnisim (every directory): {total} files include Qt directly "
            f"(ratchet allows {TOTAL})"
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
