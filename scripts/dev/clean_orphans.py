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

"""Remove project subdirs and build artifacts whose source is gone.

Two classes of orphan get cleaned:

1. Hollow project dirs. When a robot or asset dir is deleted upstream via
   `git rm`, build artifacts (.exe, .o, .dll, build/) that were never
   tracked stay behind on machines that previously built that target. The
   dir is removed only when BOTH `git ls-files` is empty AND `git status
   --porcelain` shows no untracked-not-ignored files (so a WIP robot you
   haven't committed yet is protected).

   Roots: projects/robots/<vendor>/<model>, projects/humans

2. Orphan compile outputs. When a .cpp/.hpp is deleted upstream but the
   binary on this clone was previously built, the corresponding .o (and
   moc_/qrc_/.d sidecars) sit in src/<...>/build/{debug,release}/ forever
   linking to dead source. Each .o is removed only when no source file
   with a matching basename exists anywhere under the build dir's parent.

Both passes only act on gitignored output, never on tracked content.

Usage:
  python scripts/dev/clean_orphans.py            # remove
  python scripts/dev/clean_orphans.py --dry-run  # preview only
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

# (relative root, depth) -- depth is how many levels below the root we
# evaluate. depth=0 evaluates the root itself; depth=2 evaluates grandchildren
# (vendor/model under projects/robots).
TARGETS: list[tuple[str, int]] = [
    ("projects/robots", 2),
    ("projects/humans", 0),
    # samples/<category>/controllers/<name> -- hollow controller dirs make
    # the parent controllers.Makefile fail with "No rule to make target
    # 'release'" because it globs subdirs and recurses into each.
    ("projects/samples", 3),
]


def _git(args: list[str]) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=REPO,
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout


def has_tracked_files(path: Path) -> bool:
    rel = path.relative_to(REPO).as_posix()
    return bool(_git(["ls-files", "--", rel]).strip())


def has_untracked_non_ignored(path: Path) -> bool:
    rel = path.relative_to(REPO).as_posix()
    for line in _git(["status", "--porcelain", "--", rel]).splitlines():
        if line.startswith("?? "):
            return True
    return False


def descend(root: Path, levels: int):
    if levels == 0:
        if root.is_dir():
            yield root
        return
    if not root.is_dir():
        return
    for child in sorted(root.iterdir()):
        if child.is_dir():
            yield from descend(child, levels - 1)


SOURCE_EXTS = {".cpp", ".cc", ".cxx", ".c", ".hpp", ".hxx", ".h", ".qrc"}
BUILD_SUFFIXES = (".o", ".d")
MOC_PREFIXES = ("moc_", "qrc_")


def find_build_dirs() -> list[Path]:
    """Discover {debug,release} build dirs anywhere under src/."""
    src = REPO / "src"
    if not src.is_dir():
        return []
    found = []
    for d in src.rglob("build"):
        if not d.is_dir():
            continue
        for cfg in ("debug", "release"):
            sub = d / cfg
            if sub.is_dir():
                found.append(sub)
    return found


def _all_source_basenames() -> set[str]:
    """Index every source-file basename anywhere under src/, excluding
    build dirs. Sources for one binary often live in sibling src/ subtrees
    (src/wren, src/lib, etc.), so per-build-dir scoping is too narrow."""
    src = REPO / "src"
    if not src.is_dir():
        return set()
    out: set[str] = set()
    for path in src.rglob("*"):
        if not path.is_file() or path.suffix.lower() not in SOURCE_EXTS:
            continue
        if "build" in path.parts:
            continue
        out.add(path.stem)
    return out


def orphan_build_files() -> list[Path]:
    """Find compile outputs (.o/.d) whose source file is gone.

    For each src/<x>/build/{debug,release}/<basename>.{o,d}, strip Qt's
    .moc suffix or moc_/qrc_ prefix to recover the original source stem,
    then check whether any source file with that stem exists anywhere
    under src/. If none, the output is orphan.
    """
    basenames = _all_source_basenames()
    orphans: list[Path] = []
    for build_dir in find_build_dirs():
        for out in build_dir.iterdir():
            if not out.is_file() or not out.name.endswith(BUILD_SUFFIXES):
                continue
            stem = out.stem
            if stem.endswith(".moc"):
                stem = stem[: -len(".moc")]
            for prefix in MOC_PREFIXES:
                if stem.startswith(prefix):
                    stem = stem[len(prefix):]
                    break
            if stem not in basenames:
                orphans.append(out)
    return orphans


def empty_parents(root: Path) -> list[Path]:
    """Find now-empty immediate children of `root` (e.g. vendor dirs with no
    surviving model subdirs). Returned in deterministic order."""
    if not root.is_dir():
        return []
    result = []
    for child in sorted(root.iterdir()):
        if child.is_dir() and not any(child.iterdir()):
            result.append(child)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dry-run", action="store_true", help="show what would be removed without deleting")
    args = parser.parse_args()

    to_remove: list[Path] = []
    for rel, depth in TARGETS:
        for d in descend(REPO / rel, depth):
            if has_tracked_files(d):
                continue
            if has_untracked_non_ignored(d):
                continue
            to_remove.append(d)

    verb = "would remove" if args.dry_run else "removed"
    if to_remove:
        print(f"clean_orphans: {verb} {len(to_remove)} orphan dir(s):")
        for d in to_remove:
            print(f"  {d.relative_to(REPO).as_posix()}")
            if not args.dry_run:
                shutil.rmtree(d)

    # Second pass: any vendor-level parent that's now empty (e.g.
    # projects/robots/adept/ with no models left) should also go.
    empties: list[Path] = []
    for rel, depth in TARGETS:
        if depth >= 2:
            empties.extend(empty_parents(REPO / rel))
    if empties:
        print(f"clean_orphans: {verb} {len(empties)} empty parent dir(s):")
        for d in empties:
            print(f"  {d.relative_to(REPO).as_posix()}")
            if not args.dry_run:
                d.rmdir()

    # Third pass: .o/.d outputs in src/.../build/{debug,release}/ whose
    # source file is gone (e.g. WbTextEditor.o after WbTextEditor.cpp was
    # deleted upstream).
    stale_outputs = orphan_build_files()
    if stale_outputs:
        print(f"clean_orphans: {verb} {len(stale_outputs)} orphan build output(s):")
        for f in stale_outputs:
            print(f"  {f.relative_to(REPO).as_posix()}")
            if not args.dry_run:
                f.unlink()

    return 0


if __name__ == "__main__":
    sys.exit(main())
