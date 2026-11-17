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

"""Pin: the post-checkout orphan cleaner never deletes private, gitignored
folders inside a flat robot folder.

On 2026-11-13 a `git checkout -- <file>` ran `.githooks/post-checkout`, whose
`scripts/dev/clean_orphans.py` read `projects/robots/growbot/` as a VENDOR and
rmtree'd its gitignored `source/` (a creator's private MuJoCo model and STLs,
never committed). The fix: a depth-2 root descends only through folders with
no tracked file directly in them, and a folder an anchored .gitignore rule
names is never removed. Hollow vendor/model dirs holding only build debris are
still cleaned.

Engine-free: every case runs in a throwaway git repo under tmp_path; nothing
here touches this checkout's index or working tree.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "dev"))

import clean_orphans  # noqa: E402

pytestmark = pytest.mark.skipif(shutil.which("git") is None, reason="git not on PATH")


def _write(path: Path, text: str = "x\n") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _git(repo: Path, *args: str) -> None:
    subprocess.run(["git", *args], cwd=repo, check=True, capture_output=True, text=True)


@pytest.fixture()
def repo(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    # Keep the developer's global/system git config (and its excludes file)
    # out of the verdicts.
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", os.devnull)
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))

    root = tmp_path / "repo"
    root.mkdir()
    _git(root, "init", "-q")

    _write(
        root / ".gitignore",
        "# generic build debris\n"
        "build/\n"
        "*.exe\n"
        "results/\n"
        "# private files sent for a twin\n"
        "/projects/robots/flatbot/source/\n"
        "/projects/robots/acme/privbot/\n",
    )
    _write(root / "projects/robots/Makefile", "all:\n")

    # A FLAT robot: tracked files directly in its folder, private files in
    # ignored subfolders -- one named by an anchored rule, one only by the
    # generic `results/` rule.
    _write(root / "projects/robots/flatbot/PROVENANCE.md")
    _write(root / "projects/robots/flatbot/controllers/flatbot/flatbot.py")
    _write(root / "projects/robots/flatbot/source/model.xml", "<mujoco/>\n")
    _write(root / "projects/robots/flatbot/source/assets/link.stl")
    _write(root / "projects/robots/flatbot/results/run1/log.json", "{}\n")

    # A VENDOR folder: a live model, a hollow model with only build outputs,
    # and a hollow model an anchored rule names.
    _write(root / "projects/robots/acme/livebot/protos/Livebot.proto")
    _write(root / "projects/robots/acme/oldbot/controllers/oldbot/oldbot.exe")
    _write(root / "projects/robots/acme/oldbot/build/release/oldbot.o")
    _write(root / "projects/robots/acme/privbot/meshes/base.stl")

    # A second vendor whose own (untracked) .gitignore names one of its models.
    _write(root / "projects/robots/vend2/.gitignore", "/keep/\n")
    _write(root / "projects/robots/vend2/keep/data.bin")
    _write(root / "projects/robots/vend2/gone/build/debug/gone.o")

    _git(
        root,
        "add",
        ".gitignore",
        "projects/robots/Makefile",
        "projects/robots/flatbot/PROVENANCE.md",
        "projects/robots/flatbot/controllers/flatbot/flatbot.py",
        "projects/robots/acme/livebot/protos/Livebot.proto",
    )
    monkeypatch.setattr(clean_orphans, "REPO", root)
    return root


def _rel(repo: Path, paths: list[Path]) -> set[str]:
    return {p.relative_to(repo).as_posix() for p in paths}


def test_flat_robot_folder_is_not_a_vendor(repo: Path) -> None:
    flat = repo / "projects/robots/flatbot"
    vendor = repo / "projects/robots/acme"
    assert clean_orphans.has_tracked_files_directly(flat)
    assert not clean_orphans.has_tracked_files_directly(vendor)
    candidates = _rel(repo, list(clean_orphans.descend(repo / "projects/robots", 2, True)))
    assert not any(c.startswith("projects/robots/flatbot/") for c in candidates), candidates
    assert "projects/robots/acme/oldbot" in candidates


def test_anchored_rule_protects_generic_rule_does_not(repo: Path) -> None:
    named = clean_orphans.explicitly_ignored(repo / "projects/robots/flatbot/source")
    assert named and "/projects/robots/flatbot/source/" in named
    assert clean_orphans.explicitly_ignored(repo / "projects/robots/acme/privbot")
    assert clean_orphans.explicitly_ignored(repo / "projects/robots/vend2/keep")
    # `build/` and `results/` are generic: they name no particular folder.
    assert clean_orphans.explicitly_ignored(repo / "projects/robots/acme/oldbot/build") is None
    assert clean_orphans.explicitly_ignored(repo / "projects/robots/flatbot/results") is None
    assert clean_orphans.explicitly_ignored(repo / "projects/robots/acme/oldbot") is None


def test_orphan_dirs_lists_only_hollow_vendor_models(repo: Path) -> None:
    assert _rel(repo, clean_orphans.orphan_dirs()) == {
        "projects/robots/acme/oldbot",
        "projects/robots/vend2/gone",
    }


def test_main_keeps_private_files_and_removes_build_debris(
    repo: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(sys, "argv", ["clean_orphans.py"])
    assert clean_orphans.main() == 0

    # Private, gitignored files inside the flat robot survive.
    assert (repo / "projects/robots/flatbot/source/model.xml").is_file()
    assert (repo / "projects/robots/flatbot/source/assets/link.stl").is_file()
    assert (repo / "projects/robots/flatbot/results/run1/log.json").is_file()
    # So does a hollow model an anchored rule names, at either .gitignore level.
    assert (repo / "projects/robots/acme/privbot/meshes/base.stl").is_file()
    assert (repo / "projects/robots/vend2/keep/data.bin").is_file()
    # Tracked content is untouched.
    assert (repo / "projects/robots/acme/livebot/protos/Livebot.proto").is_file()
    # Hollow vendor/model dirs holding only build outputs are still removed.
    assert not (repo / "projects/robots/acme/oldbot").exists()
    assert not (repo / "projects/robots/vend2/gone").exists()


def test_dry_run_removes_nothing(repo: Path, monkeypatch: pytest.MonkeyPatch, capsys) -> None:
    monkeypatch.setattr(sys, "argv", ["clean_orphans.py", "--dry-run"])
    assert clean_orphans.main() == 0
    out = capsys.readouterr().out
    assert "projects/robots/acme/oldbot" in out
    assert "flatbot" not in out
    assert (repo / "projects/robots/acme/oldbot/build/release/oldbot.o").is_file()
