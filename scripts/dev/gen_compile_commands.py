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

"""Generate compile_commands.json by parsing make dry-run output.

This avoids the need for `bear` which is not available on MSYS2 MinGW.
Run from the repo root:
    python scripts/dev/gen_compile_commands.py
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

# Match compiler invocations: g++ or gcc with -c flag (compilation, not linking)
COMPILE_RE = re.compile(
    r"(g\+\+|gcc|cc|c\+\+|ccache\s+g\+\+|ccache\s+gcc)"
    r"\s+(.+?)\s+-c\s+(.+?)(?:\s+-o\s+(\S+))?"
)


def scan_build_artifacts(repo_root: Path, env: dict) -> list[dict]:
    """Scan .o and .d files from past builds and reconstruct compile_commands entries."""
    entries = []
    seen = set()

    # Known build directories and their source roots
    build_dirs = [
        (repo_root / "src" / "webots" / "build" / "release", repo_root / "src" / "webots"),
        (repo_root / "src" / "wren" / "build" / "release", repo_root / "src" / "wren"),
        (repo_root / "src" / "ode" / "build" / "release", repo_root / "src" / "ode"),
        (repo_root / "src" / "glad" / "build" / "release", repo_root / "src" / "glad"),
        (repo_root / "src" / "controller" / "c" / "build" / "release", repo_root / "src" / "controller" / "c"),
        (repo_root / "src" / "controller" / "cpp" / "build" / "release", repo_root / "src" / "controller" / "cpp"),
    ]

    for build_dir, source_root in build_dirs:
        if not build_dir.exists():
            continue

        # Read .d files to find source -> object mappings
        for d_file in build_dir.glob("*.d"):
            try:
                content = d_file.read_text(errors="replace")
            except Exception:
                continue

            # .d files have format: target.o: source.cpp header1.hpp header2.hpp ...
            # Extract the source file (first .cpp or .c file after the colon)
            for line in content.replace("\\\n", " ").splitlines():
                if ":" not in line:
                    continue
                deps_part = line.split(":", 1)[1] if ":" in line else ""
                for token in deps_part.split():
                    token = token.strip()
                    if token.endswith((".cpp", ".c", ".cc", ".cxx")):
                        source_path = Path(token)
                        if not source_path.is_absolute():
                            source_path = source_root / token
                        if not source_path.exists():
                            # Try relative to build dir
                            source_path = build_dir / token
                        abs_source = str(source_path.resolve()) if source_path.exists() else str(source_root / token)

                        if abs_source in seen:
                            break
                        seen.add(abs_source)

                        # Construct a representative compile command
                        is_cpp = token.endswith((".cpp", ".cc", ".cxx"))
                        compiler = "g++" if is_cpp else "gcc"
                        obj_name = d_file.stem + ".o"

                        entries.append({
                            "directory": str(source_root),
                            "command": f"{compiler} -c {abs_source} -o {build_dir / obj_name}",
                            "file": abs_source,
                        })
                        break  # Only take the first source file per .d file
                    break  # Only process first dependency line

    return entries


def parse_compile_commands(dry_run_output: str) -> list[dict]:
    entries = []
    seen = set()

    for line in dry_run_output.splitlines():
        line = line.strip()
        if " -c " not in line:
            continue

        # Find the source file: look for .cpp, .c files in the command
        parts = line.split()
        source_file = None
        compiler = None
        for i, part in enumerate(parts):
            if part in ("g++", "gcc", "cc", "c++") or part.endswith("/g++") or part.endswith("/gcc"):
                compiler = part
            if part == "ccache" and i + 1 < len(parts):
                compiler = f"{part} {parts[i + 1]}"
            if part.endswith((".cpp", ".c", ".cc", ".cxx")) and not part.startswith("-"):
                source_file = part

        if not source_file:
            continue

        # Resolve to absolute path
        source_path = Path(source_file)
        if not source_path.is_absolute():
            # Try common base directories
            for base in [REPO_ROOT / "src" / "webots", REPO_ROOT / "src" / "wren",
                         REPO_ROOT / "src" / "ode", REPO_ROOT / "src" / "glad",
                         REPO_ROOT / "src" / "controller" / "c",
                         REPO_ROOT / "src" / "controller" / "cpp",
                         REPO_ROOT]:
                candidate = base / source_file
                if candidate.exists():
                    source_path = candidate
                    break

        abs_source = str(source_path.resolve()) if source_path.exists() else str(REPO_ROOT / source_file)

        if abs_source in seen:
            continue
        seen.add(abs_source)

        # Determine the directory (where the compilation runs)
        directory = str(source_path.parent.resolve()) if source_path.exists() else str(REPO_ROOT)

        entries.append({
            "directory": directory,
            "command": line,
            "file": abs_source,
        })

    return entries


def main() -> int:
    env = os.environ.copy()
    env["OMNISIM_HOME"] = str(REPO_ROOT)

    # Scan .d dependency files to reconstruct compile commands from the last build.
    # This is fast and doesn't require a dry-run.
    print("[gen-compdb] Scanning build artifacts for compile commands...")
    entries = scan_build_artifacts(REPO_ROOT, env)

    outpath = REPO_ROOT / "compile_commands.json"
    outpath.write_text(json.dumps(entries, indent=2) + "\n")
    print(f"[gen-compdb] Wrote {len(entries)} entries to {outpath}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
