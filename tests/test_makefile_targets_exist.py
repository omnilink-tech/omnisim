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

"""Every hard-coded Makefile target must name a directory a fresh checkout has.

WHY THIS EXISTS. On 2026-09-02 a cleanup round deleted
``projects/default/controllers/braitenberg`` because no world named it, and left
``projects/default/Makefile``'s hard-coded ``TARGETS =
controllers/braitenberg.Makefile`` behind. Nothing local caught it: the Windows
engine build never descends into ``projects/``, the directory survived on disk as
untracked build residue, and the pre-push smoke does not build the sample
projects. The Linux CI build, which starts from a fresh checkout, failed with a
bare ``make[3]: *** controllers/braitenberg: No such file or directory``.

The check is deliberately narrow. It reads the TRACKED file list, not the working
tree, because that is what a clone gets, and it only inspects LITERAL entries: a
``TARGETS`` built from ``$(wildcard ...)`` re-globs at build time and cannot go
stale, which is exactly why the fix for the braitenberg break was to glob.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

# `TARGETS = a.Makefile b.Makefile`, continued over backslash-newlines.
TARGETS_RE = re.compile(r"^TARGETS\s*[:+]?=\s*(.+(?:\\\n.+)*)$", re.MULTILINE)


def _tracked_paths() -> set[str]:
    out = subprocess.run(
        ["git", "ls-files"], cwd=REPO_ROOT, capture_output=True, text=True, check=True
    ).stdout
    return set(out.split("\n")) - {""}


def _tracked_directories(paths: set[str]) -> set[str]:
    dirs: set[str] = set()
    for path in paths:
        parts = path.split("/")
        for i in range(1, len(parts)):
            dirs.add("/".join(parts[:i]))
    return dirs


def test_hardcoded_makefile_targets_name_a_tracked_directory() -> None:
    tracked = _tracked_paths()
    tracked_dirs = _tracked_directories(tracked)
    broken: list[str] = []

    for rel in sorted(p for p in tracked if p.endswith("Makefile")):
        text = (REPO_ROOT / rel).read_text(encoding="utf-8", errors="replace")
        match = TARGETS_RE.search(text)
        if not match:
            continue
        value = match.group(1).replace("\\\n", " ")
        if "$(" in value:
            # Expanded at build time (wildcard/patsubst, or a variable): it re-globs on
            # every build and cannot name a directory that is not there. Skipped as a
            # whole value, not per token -- make's function calls contain spaces.
            continue
        for entry in value.split():
            if entry == "\\":
                continue
            directory = entry[: -len(".Makefile")] if entry.endswith(".Makefile") else entry
            full = f"{Path(rel).parent.as_posix()}/{directory}"
            if full not in tracked_dirs:
                broken.append(f"{rel}: TARGETS names '{directory}', which has no tracked files")

    assert not broken, (
        "A Makefile names a build target that a fresh checkout does not contain, so the "
        "build fails on CI while a local tree with leftover residue stays green:\n  "
        + "\n  ".join(broken)
        + "\nEither restore the directory or (better) glob the targets: "
        "TARGETS = $(patsubst %/Makefile,%.Makefile,$(wildcard controllers/*/Makefile))"
    )
