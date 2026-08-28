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

"""The package must contain the commands the docs tell a new user to run.

WHY THIS FILE EXISTS
--------------------
v8.1.6 built green, published green, and was "latest" for three days without
the ``omnisim`` Python package. So ``python -m omnisim doctor`` -- step 2 of
both README.md and BETA.md -- failed with "No module named omnisim" on the only
artifact anyone could download, and nothing anywhere noticed: release.yml's
only content check was ``if-no-files-found: error``, which verifies that an
.exe exists.

release.yml now greps the generated Inno Setup script for these same paths, but
that gate only runs on a tag. This one runs in the ordinary test suite, against
the manifest itself, so the omission is caught when it is introduced rather
than when it ships.

It deliberately checks the MANIFEST rather than a built installer: building one
needs Windows, Inno Setup and a full release build, and the failure being
guarded against is an entry missing from ``files_core.txt``.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
MANIFEST = REPO_ROOT / "scripts" / "packaging" / "files_core.txt"

sys.path.insert(0, str(REPO_ROOT / "scripts" / "packaging"))


# Every entry is a path a document tells a new user to run, or that one of
# those commands needs in order to work. Keep the reason with the path: an
# unexplained list gets pruned by whoever next finds it inconvenient.
DOCUMENTED_ENTRY_POINTS = {
    "omnisim": "README/BETA step 2 is `python -m omnisim doctor`",
    "omnisim.bat": "runs that command when there is no system Python (clean Windows)",
    "AGENTS.md": "BETA step 4 points a coding agent at it",
    "README.md": "the installed tree is otherwise unreadable",
    "LICENSE": "Apache-2.0 binary distribution",
    "PROTOCOL.md": "the contract for anything driving OmniSim from outside",
    "DEMOS.md": "the demo index README links",
    "launch.bat": "the Windows launcher AGENTS.md documents",
    "projects/guided_tour.txt": "the first-run welcome menu is empty without it",
    "scripts/dev": "run-headless / omnisim_dev live here",
    "scripts/harness": "the agent-facing harness; the MCP server proxies it",
    "packages/omnisim-mcp": "README tells a package user to connect it",
    "plugins/omnisim": "README links the Codex plugin",
    ".mcp.json": "registers the MCP server for Claude Code with no install",
}


def _manifest_entries() -> set[str]:
    """Bare paths named by files_core.txt, with option tags stripped."""
    entries: set[str] = set()
    for raw in MANIFEST.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.endswith("]") and "[" in line:
            line = line[: line.rindex("[")].strip()
        entries.add(line.rstrip("/"))
    return entries


@pytest.mark.parametrize("entry", sorted(DOCUMENTED_ENTRY_POINTS))
def test_documented_entry_point_is_packaged(entry: str) -> None:
    assert (REPO_ROOT / entry).exists(), (
        f"{entry} does not exist in the tree, but is listed here as a documented "
        f"entry point ({DOCUMENTED_ENTRY_POINTS[entry]}). Fix the path or the list."
    )
    assert entry in _manifest_entries(), (
        f"{entry} is NOT in scripts/packaging/files_core.txt, so it will not be in "
        f"the installer -- and it is needed because {DOCUMENTED_ENTRY_POINTS[entry]}. "
        f"This is exactly how v8.1.6 shipped without the `omnisim` CLI."
    )


def test_manifest_recursion_excludes_build_residue() -> None:
    """A local `make distrib` must produce the same payload as a CI build.

    files_core.txt recurses into packages/ and plugins/. None of dist/,
    *.egg-info or the linter caches is ever tracked, so they exist only on a
    developer box -- and scripts/packaging's `git clean -fdf` has no -x, so
    without this filter a local build would package them.
    """
    from generate_projects_files import is_ignored_file, is_ignored_folder

    for junk in ("dist", ".ruff_cache", ".pytest_cache", ".mypy_cache",
                 "omnisim_mcp.egg-info", "__pycache__", "build", "node_modules"):
        assert is_ignored_folder(junk), f"{junk} would be packaged"
    for keep in ("src", "omnisim", "worlds", "controllers"):
        assert not is_ignored_folder(keep), f"{keep} would be dropped from the package"

    # Harness scratch worlds are gitignored but survive `git clean -fdf`.
    assert is_ignored_file(".harness_warehouse_husky.omniworld")
    assert is_ignored_file(".scratch_thing.omniworld")
    for keep in (".mcp.json", "server.py", "warehouse_husky.omniworld"):
        assert not is_ignored_file(keep), f"{keep} would be dropped from the package"


def test_recursed_package_dirs_carry_no_untracked_files() -> None:
    """The recursed on-ramp dirs resolve to tracked files only.

    Belt and braces for the rule above: if this box has residue the ignore
    rules do not cover, the packaged payload would differ from a CI build's.
    """
    import subprocess

    from generate_projects_files import is_ignored_file, is_ignored_folder

    tracked = set(
        subprocess.check_output(["git", "ls-files"], cwd=REPO_ROOT, text=True).splitlines()
    )
    stray = []
    for root in ("packages/omnisim-mcp", "packages/omnisim-bridges", "plugins/omnisim"):
        for dirpath, dirnames, filenames in os.walk(REPO_ROOT / root):
            dirnames[:] = [d for d in dirnames if not is_ignored_folder(d)]
            for name in filenames:
                if is_ignored_file(name):
                    continue
                rel = (Path(dirpath) / name).relative_to(REPO_ROOT).as_posix()
                if rel not in tracked:
                    stray.append(rel)
    assert not stray, (
        "these untracked files would be packaged by the [recurse] entries in "
        f"files_core.txt: {stray}"
    )
