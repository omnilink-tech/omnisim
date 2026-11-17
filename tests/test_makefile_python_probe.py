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

"""The Windows engine build embeds Python312 when it is installed, not the newest 3.x.

WHY THIS EXISTS. ``src/omnisim/Makefile`` probes the python.org per-user install
root for ``Python3NN/include/Python.h``. Until 2026-09-29 it took
``$(lastword $(sort ...))`` -- the newest -- so an external tester with 3.12.10
and 3.14.3 installed side by side got an engine embedding 3.14, the version the
quickstart calls wheel-fragile, and nothing in the build said which one it had
picked (report of 2026-08-29).

The test runs the Makefile's own two probe lines (copied out verbatim, so a
later edit to them is what gets tested) under a real ``make`` against a fake
install root. It skips when no ``make`` is on the machine.
"""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
MAKEFILE = REPO_ROOT / "src" / "omnisim" / "Makefile"

_MAKE_CANDIDATES = ("make", "gmake", "mingw32-make", "C:/msys64/usr/bin/make.exe")


def _find_make() -> str | None:
    for cand in _MAKE_CANDIDATES:
        found = shutil.which(cand)
        if found:
            return found
        if Path(cand).is_file():
            return cand
    return None


def _probe_lines() -> str:
    text = MAKEFILE.read_text(encoding="utf-8")
    all_line = re.search(r"^\s*PY_PROBE_ALL :=.*$", text, re.MULTILINE)
    pick_line = re.search(r"^\s*PYTHON_PROBE := \$\(firstword .*$", text, re.MULTILINE)
    assert all_line and pick_line, "the Makefile's Windows Python probe lines moved or changed shape"
    return all_line.group(0).strip() + "\n" + pick_line.group(0).strip() + "\n"


def _pick(tmp_path: Path, versions: list[str]) -> str:
    make = _find_make()
    if make is None:
        pytest.skip("no make on this machine")
    root = tmp_path / "Programs" / "Python"
    for v in versions:
        inc = root / f"Python{v}" / "include"
        inc.mkdir(parents=True)
        (inc / "Python.h").write_text("/* fake */\n", encoding="utf-8")
    mk = tmp_path / "probe.mk"
    mk.write_text(
        f"PY_ROOTS := {root.as_posix()}\n" + _probe_lines()
        + "print:\n\t@echo $(PYTHON_PROBE)\n",
        encoding="utf-8",
    )
    out = subprocess.run([make, "-s", "-f", str(mk), "print"], capture_output=True,
                         text=True, check=True).stdout.strip()
    return out


def test_python312_wins_over_a_newer_install(tmp_path: Path) -> None:
    assert _pick(tmp_path, ["310", "312", "314"]).endswith("/Python312/include/Python.h")


def test_newest_is_the_fallback_when_312_is_absent(tmp_path: Path) -> None:
    assert _pick(tmp_path, ["313", "314"]).endswith("/Python314/include/Python.h")


def test_single_install_is_taken(tmp_path: Path) -> None:
    assert _pick(tmp_path, ["312"]).endswith("/Python312/include/Python.h")


def test_nothing_installed_leaves_the_probe_empty(tmp_path: Path) -> None:
    assert _pick(tmp_path, []) == ""


def test_the_choice_is_reported() -> None:
    text = MAKEFILE.read_text(encoding="utf-8")
    assert '@echo "embedded python: ' in text
    assert "not Python312, the supported version" in text
