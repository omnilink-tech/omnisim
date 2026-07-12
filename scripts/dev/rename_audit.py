#!/usr/bin/env python
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

"""Live inventory of remaining 'Webots' surface, per category.

Prints one row per category with the current count and the expected
range (`min..max`). Exit code is 0 when every category is in range,
1 otherwise -- so this script is usable both as a tracker (`python
rename_audit.py`) and as a CI fence (`--check` returns nonzero on drift).

When a deliberate Webots-name reduction lands (path rename, header
forwarder removal, etc.), lower the matching `expected_max` so the next
regression flips the gate red. Naming policy: [AGENTS.md §0](../../AGENTS.md).
"""

from __future__ import annotations

import argparse
import fnmatch
import json
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]

# Directories whose contents we treat as historical / out-of-scope for the
# rebrand. The blog import contains upstream commit messages verbatim, so it
# legitimately keeps the Webots name; build/ holds compiled artefacts; vendored
# submodules (src/glm, src/stb) are not ours to rename.
# Path-prefix excludes (relative to repo root). Each entry matches the named
# directory and everything beneath it.
EXCLUDE_DIR_PATHS = {
    ".git",
    "msys64",       # MSYS2 toolchain + build outputs (the shipped binaries)
    "Contents",     # macOS bundle skeleton
    "include",      # third-party Qt mirror lives here -- handled separately
    "lib",          # libController build outputs and Qt DLLs
    "dependencies",
    "distribution",  # packaging staging area + generated worlds
    "_scratch",      # gitignored experiment logs (engine stdout dumps)
    "src/glm",
    "src/stb",
    "src/ode",
    "src/qt-msvc",
    "docs/blog/upstream-webots-history",
    "docs/reference/upstream-webots-history",
}

# Bare directory names that should be skipped wherever they appear in the tree.
# `build` is the big one -- every subsystem has its own src/<subsystem>/build/
# tree of `*.o`, `*.d`, and `*.moc.cpp` artefacts that contain Webots-named
# strings but are rebuilt on every compile.
EXCLUDE_DIR_BASENAMES = {
    "build",
    "build_release",
    "build_debug",
    "__pycache__",
    "node_modules",
    ".cache",
}

# Files that legitimately mention "webots" as part of the rebrand machinery
# itself (planning docs, audit tooling, this script's baseline snapshot).
# Excluding them keeps `tests/sources/test_naming.py` from policing its own
# scaffolding. Also: transient test-run artefacts (gitignored stdout/stderr
# dumps) so a recent test_suite run doesn't flip the gate red.
EXCLUDE_FILE_PATHS = {
    "scripts/dev/rename_audit.py",
    "tests/sources/test_naming.py",
    "tests/baselines/rename_audit_baseline.json",
    "tests/omnisim_stdout.txt",
    "tests/omnisim_stderr.txt",
}

EXCLUDE_FILE_PATTERNS = {
    "*.png",
    "*.jpg",
    "*.jpeg",
    "*.gif",
    "*.bmp",
    "*.ico",
    "*.icns",
    "*.tiff",
    "*.tif",
    "*.webp",
    "*.mp4",
    "*.webm",
    "*.ogg",
    "*.wav",
    "*.mp3",
    "*.zip",
    "*.tar",
    "*.tar.gz",
    "*.7z",
    "*.exe",
    "*.dll",
    "*.lib",
    "*.a",
    "*.o",
    "*.so",
    "*.dylib",
    "*.pdb",
    "*.pyc",
    "*.bin",
}

TEXT_EXTENSIONS = {
    ".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx",
    ".py", ".pyx", ".pxd",
    ".js", ".ts", ".jsx", ".tsx",
    ".sh", ".bash", ".bat", ".cmd", ".ps1",
    ".mk", ".cmake",
    ".yaml", ".yml", ".toml", ".json", ".ini", ".cfg",
    ".md", ".rst", ".txt",
    ".wbt", ".wbproj", ".proto",
    ".html", ".css", ".xml",
    ".vert", ".frag", ".glsl",
    # MSBuild project files. The C/C++ controller templates
    # (resources/templates/controllers/*.vcxproj*) reference the install-root env
    # var, and without these suffixes the audit could not see them at all -- a hole
    # big enough for a whole WEBOTS_HOME surface to creep back in unobserved.
    ".vcxproj", ".filter", ".user", ".props", ".targets", ".sln",
}


@dataclass
class Category:
    name: str
    description: str
    expected_min: int
    expected_max: int
    count: int = 0
    samples: list[str] = field(default_factory=list)

    @property
    def in_range(self) -> bool:
        return self.expected_min <= self.count <= self.expected_max


def _is_excluded_dir(path: Path) -> bool:
    if path.name in EXCLUDE_DIR_BASENAMES:
        return True
    rel = path.relative_to(REPO_ROOT).as_posix()
    for ex in EXCLUDE_DIR_PATHS:
        if rel == ex or rel.startswith(ex + "/"):
            return True
    return False


def _is_under_excluded_dir(path: Path) -> bool:
    """True when any ancestor directory of `path` is excluded."""
    rel = path.relative_to(REPO_ROOT)
    parts = rel.parts[:-1]  # drop the file name itself
    if any(p in EXCLUDE_DIR_BASENAMES for p in parts):
        return True
    prefix = ""
    for part in parts:
        prefix = f"{prefix}/{part}" if prefix else part
        if prefix in EXCLUDE_DIR_PATHS:
            return True
    return False


def _is_text_file(path: Path) -> bool:
    name = path.name
    for pat in EXCLUDE_FILE_PATTERNS:
        if fnmatch.fnmatch(name, pat):
            return False
    return path.suffix.lower() in TEXT_EXTENSIONS or path.suffix == ""


def _is_excluded_file(path: Path) -> bool:
    return path.relative_to(REPO_ROOT).as_posix() in EXCLUDE_FILE_PATHS


def _git_tracked_files() -> list[Path] | None:
    """Every file git tracks, or None when git is unavailable.

    This is the authoritative file set for the audit: it is exactly the
    repo's *source*. Build artefacts (`msys64/`, `src/*/build/`, `*.o`),
    engine stdout dumps (`_scratch/`, `omnisim_log.txt`) and every other
    gitignored file are absent by construction -- they are not tracked.

    Scanning the raw working tree instead is what silently broke this
    gate: a `_scratch/` full of engine logs pushed `wb_class_use` from
    ~43k to 7.4M, so `tests/sources/test_naming.py` sat permanently red
    and stopped guarding anything.
    """
    try:
        proc = subprocess.run(
            ["git", "ls-files", "-z"],
            cwd=REPO_ROOT,
            capture_output=True,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return [
        REPO_ROOT / name
        for name in proc.stdout.decode("utf-8", "replace").split("\0")
        if name
    ]


def _walk_worktree():
    """Fallback enumeration for a non-git export (no .gitignore to honour)."""
    for root, dirs, files in os.walk(REPO_ROOT):
        root_path = Path(root)
        # Prune excluded dirs in-place so os.walk doesn't descend into them.
        dirs[:] = [
            d for d in dirs
            if not _is_excluded_dir(root_path / d) and not d.startswith(".")
        ]
        for f in files:
            yield root_path / f


def _walk_repo():
    tracked = _git_tracked_files()
    candidates = tracked if tracked is not None else _walk_worktree()
    for path in candidates:
        # Tracked paths come straight from the index, so a file deleted in the
        # working tree can still be listed; and the dir excludes above must be
        # applied to the git-supplied paths too.
        if not path.is_file():
            continue
        if _is_under_excluded_dir(path):
            continue
        yield path


PATTERNS = {
    "include_c_webots":   re.compile(rb'#\s*include\s*[<"]webots/'),
    "wb_class_use":       re.compile(rb'\bWb[A-Z][A-Za-z0-9_]+'),
    "webots_url_scheme":  re.compile(rb'webots://'),
    "webots_home_env":    re.compile(rb'\bWEBOTS_HOME\b'),
}


def collect() -> dict[str, Category]:
    cats = {
        # Each expected_max is the post-Phase-I floor + small headroom for
        # adding back a few CHANGELOG entries / new doc lines without flipping
        # the gate red. The gate flips if any category grows past its ceiling.
        "file_paths_webots": Category(
            "file_paths_webots",
            "files/dirs whose path contains 'webots' (case-insensitive)",
            expected_min=0, expected_max=850,
        ),
        "webots_yaml_files": Category(
            "webots_yaml_files",
            "project metadata files literally named webots.yaml",
            expected_min=0, expected_max=0,
        ),
        "include_c_webots": Category(
            "include_c_webots",
            "C/C++ #include <webots/...> or \"webots/...\"",
            expected_min=0, expected_max=1_600,
        ),
        "wb_class_use": Category(
            "wb_class_use",
            "occurrences of Wb<Pascal> class-style identifiers",
            expected_min=0, expected_max=50_000,
        ),
        "webots_url_scheme": Category(
            "webots_url_scheme",
            "webots:// URL references inside text files",
            expected_min=0, expected_max=500,
        ),
        "webots_home_env": Category(
            "webots_home_env",
            "WEBOTS_HOME env-var reads / writes inside text files",
            expected_min=0, expected_max=200,
        ),
    }

    for path in _walk_repo():
        if _is_excluded_file(path):
            continue
        rel = path.relative_to(REPO_ROOT).as_posix()
        # Path-name categories first (cheap, run on every file).
        low = rel.lower()
        if "webots" in low:
            cat = cats["file_paths_webots"]
            cat.count += 1
            if len(cat.samples) < 10:
                cat.samples.append(rel)
        if path.name == "webots.yaml":
            cat = cats["webots_yaml_files"]
            cat.count += 1
            if len(cat.samples) < 10:
                cat.samples.append(rel)

        if not _is_text_file(path):
            continue
        try:
            data = path.read_bytes()
        except OSError:
            continue
        # Skip binary-looking files (NUL byte heuristic).
        if b"\x00" in data[:4096]:
            continue
        for key, rx in PATTERNS.items():
            hits = rx.findall(data)
            if not hits:
                continue
            cat = cats[key]
            cat.count += len(hits)
            if len(cat.samples) < 10:
                cat.samples.append(rel)

    return cats


def format_text(cats: dict[str, Category]) -> str:
    lines = []
    lines.append("rename_audit -- live Webots-surface inventory")
    lines.append("=" * 60)
    name_w = max(len(c.name) for c in cats.values())
    for c in cats.values():
        marker = "ok " if c.in_range else "OUT"
        lines.append(
            f"  [{marker}] {c.name:<{name_w}}  count={c.count:>7}"
            f"  expected={c.expected_min}..{c.expected_max}"
        )
    lines.append("")
    lines.append("Sample paths (first 10 per category):")
    for c in cats.values():
        if not c.samples:
            continue
        lines.append(f"  {c.name}:")
        for s in c.samples:
            lines.append(f"    {s}")
    return "\n".join(lines)


def main() -> int:
    p = argparse.ArgumentParser(
        prog="rename_audit",
        description="Live count of remaining Webots-named symbols per category.",
    )
    p.add_argument("--json", action="store_true", help="emit JSON instead of text")
    p.add_argument(
        "--check",
        action="store_true",
        help="exit 1 if any category is outside its expected range",
    )
    args = p.parse_args()

    cats = collect()

    if args.json:
        payload = {
            c.name: {
                "description": c.description,
                "count": c.count,
                "expected_min": c.expected_min,
                "expected_max": c.expected_max,
                "in_range": c.in_range,
                "samples": c.samples,
            }
            for c in cats.values()
        }
        print(json.dumps(payload, indent=2))
    else:
        print(format_text(cats))

    if args.check and any(not c.in_range for c in cats.values()):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
