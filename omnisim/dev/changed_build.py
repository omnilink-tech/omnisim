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

"""Dependency-driven compile-only builds for the agent editing loop."""

from __future__ import annotations

import subprocess
from pathlib import Path

from ..paths import REPO_ROOT
from .runner import run


SIMULATOR_DIR = REPO_ROOT / "src" / "omnisim"
DEP_DIR = SIMULATOR_DIR / "build" / "release" / ".deps"
BUILD_INPUTS = {
    "Makefile", "resources/Makefile.include", "resources/Makefile.os.include",
    "src/omnisim/Makefile",
}


def _git_lines(*args: str) -> list[str]:
    result = subprocess.run(
        ["git", *args], cwd=REPO_ROOT, text=True, encoding="utf-8",
        errors="replace", capture_output=True, check=False,
    )
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "git failed")
    return [line.strip().replace("\\", "/") for line in result.stdout.splitlines() if line.strip()]


def changed_paths(base: str) -> set[str]:
    tracked = _git_lines("diff", "--name-only", "--diff-filter=ACMR", base, "--")
    untracked = _git_lines("ls-files", "--others", "--exclude-standard")
    return set(tracked + untracked)


def _unescape_make_word(word: str) -> str:
    return word.replace("\\ ", " ").replace("\\#", "#").replace("\\:", ":")


def dependency_prerequisites(depfile: Path) -> set[str]:
    """Return repository-relative prerequisites from a GCC depfile."""
    content = depfile.read_text(encoding="utf-8", errors="replace").replace("\\\n", " ")
    if ":" not in content:
        return set()
    prerequisites = content.split(":", 1)[1]
    result: set[str] = set()
    for raw in prerequisites.split():
        word = _unescape_make_word(raw)
        path = Path(word)
        if path.is_absolute():
            try:
                path = path.resolve().relative_to(REPO_ROOT.resolve())
            except ValueError:
                continue
        else:
            try:
                path = (SIMULATOR_DIR / path).resolve().relative_to(REPO_ROOT.resolve())
            except ValueError:
                continue
        result.add(path.as_posix())
    return result


def affected_objects(changes: set[str], dep_dir: Path = DEP_DIR) -> list[str]:
    normalized = {Path(path).as_posix() for path in changes}
    objects: set[str] = set()
    if dep_dir.exists():
        for depfile in dep_dir.glob("*.d"):
            if normalized.intersection(dependency_prerequisites(depfile)):
                objects.add(f"{depfile.stem}.o")
    # New/previously unbuilt simulator sources have no depfile yet. Object
    # targets are basenames by long-standing Makefile convention.
    for path in normalized:
        if path.startswith("src/omnisim/") and Path(path).suffix in {".cpp", ".c"}:
            objects.add(f"{Path(path).stem}.o")
    return sorted(objects)


def build_changed(*, base: str, jobs: int, link: bool, staged: bool,
                  env: dict[str, str] | None = None) -> int:
    try:
        changes = changed_paths(base)
    except RuntimeError as exc:
        print(f"[omnisim] Could not inspect changed files: {exc}")
        return 2
    if not changes:
        print(f"[omnisim] No working-tree changes relative to {base}; nothing to compile.")
        return 0
    if changes.intersection(BUILD_INPUTS):
        print("[omnisim] Build configuration changed; using the conservative GUI build.")
        goal = "sim-gui-staged" if staged else "sim-gui"
        return run(["make", f"-j{jobs}", goal], env=env)
    objects = affected_objects(changes)
    if not objects:
        print(f"[omnisim] {len(changes)} changed path(s), none affect simulator translation units.")
        return 0
    print(f"[omnisim] Compiling {len(objects)} affected object(s): {' '.join(objects)}")
    rc = run(
        ["make", f"-j{jobs}", "sim-check-objects", f"CHECK_OBJECTS={' '.join(objects)}"],
        env=env,
    )
    if rc or not link:
        return rc
    goal = "sim-gui-staged" if staged else "sim-gui"
    return run(["make", f"-j{jobs}", goal], env=env)
