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

"""Strip private-only blocks from files that otherwise ship in the public snapshot.

The deny-list (publish_deny.txt) removes whole files. Some files that DO ship carry a passage
about private work, such as a row in AGENTS.md, a section of README.md or a CLI verb. Wrap such a
passage in markers:

    <!-- public:strip-begin -->   ...   <!-- public:strip-end -->      (Markdown, HTML)
    # public:strip-begin          ...   # public:strip-end             (Python, shell, text)

Every line from the one holding a begin marker to the one holding its end marker, both
included, is removed from the public copy. The private tree keeps the passage as it is.
A single line (a Markdown table row, a list item) is removed by putting the marker on it,
for example at its end: `... | <!-- public:strip-line -->`.

A public-only replacement line (the public wording of a passage that was stripped) is a
comment in the private tree, so it renders and runs as nothing there:

    <!-- public:only - The text the public copy shows. -->
    # public:only - code_the_public_copy_runs()
    // public:only - a C or C++ comment line

In the public copy the line becomes its content: the text after "public:only - ", with the
comment's own indentation (and a Markdown "> " quote prefix) kept and a trailing " -->" removed.

publish_snapshot.sh calls this in its throwaway worktree, after the deny stage and before the
dangling-reference check:

    python scripts/release/public_strip.py --root <worktree> [--git-add]

It fails closed: an unbalanced or nested marker, or a marker that survives, is an error,
and nothing is half-stripped. With --check it only reports.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import re
import subprocess
import sys

BEGIN = "public:strip-begin"
END = "public:strip-end"
LINE = "public:strip-line"
ONLY = "public:only - "
_ONLY_RE = re.compile(r"^(?P<ind>[ \t>]*)(?:<!--|#|//)[ \t]*public:only - (?P<body>.*?)(?:[ \t]*-->)?(?P<eol>\r?\n?)$")


class StripError(ValueError):
    pass


# JSON files that cannot carry comments: {file: key of the object whose keys are repo paths}. An entry
# whose path does not ship is dropped from the public copy (the private file keeps every entry).
PRUNE_JSON = {"tests/goldens/roll_check_baseline.json": "worlds"}


def prune_json(text: str, key: str, ships) -> tuple[str, int]:
    """Drop d[key][path] for every path that `ships(path)` is False on; keep the file's own layout."""
    d = json.loads(text)
    ascii_ = json.dumps(d, indent=2, ensure_ascii=True) + "\n" == text
    gone = [p for p in d[key] if not ships(p)]
    for p in gone:
        del d[key][p]
    return json.dumps(d, indent=2, ensure_ascii=ascii_) + "\n", len(gone)


def strip_text(text: str, name: str = "<text>") -> tuple[str, int]:
    """Return (text without marked blocks, number of blocks removed or replaced)."""
    out: list[str] = []
    depth_line = 0  # line number of the open begin marker, 0 when outside a block
    blocks = 0
    for n, line in enumerate(text.splitlines(keepends=True), 1):
        has_begin, has_end = BEGIN in line, END in line
        if LINE in line and not depth_line:
            if has_begin or has_end:
                raise StripError(f"{name}:{n}: strip-line on a begin/end marker line")
            blocks += 1
            continue
        if has_begin and has_end:
            raise StripError(f"{name}:{n}: begin and end marker on one line")
        if has_begin:
            if depth_line:
                raise StripError(f"{name}:{n}: nested begin marker (block opened at line {depth_line})")
            depth_line = n
            continue
        if has_end:
            if not depth_line:
                raise StripError(f"{name}:{n}: end marker without a begin marker")
            depth_line = 0
            blocks += 1
            continue
        if depth_line:
            continue
        if ONLY.strip() in line:
            m = _ONLY_RE.match(line)
            if not m:
                raise StripError(f"{name}:{n}: a public:only line must be a whole-line <!-- -->, # or // comment")
            line = m.group("ind") + m.group("body") + m.group("eol")
            blocks += 1
        out.append(line)
    if depth_line:
        raise StripError(f"{name}:{depth_line}: begin marker never closed")
    return "".join(out), blocks


def marked_files(root: pathlib.Path) -> list[str]:
    """Tracked text files (the index: after the deny stage, the shipping set) that hold a marker."""
    r = subprocess.run(["git", "grep", "--cached", "-I", "-l", "-F", "-e", BEGIN, "-e", END, "-e", LINE,
                        "-e", ONLY.strip(), "--", "."], cwd=root, capture_output=True, text=True)
    if r.returncode not in (0, 1):
        raise SystemExit(f"git grep failed: {r.stderr.strip()}")
    return [p for p in r.stdout.splitlines() if p and p != "scripts/release/public_strip.py"
            and not p.startswith("tests/test_public_strip")]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", type=pathlib.Path, default=pathlib.Path("."))
    ap.add_argument("--git-add", action="store_true", help="stage every rewritten file (the publish worktree)")
    ap.add_argument("--check", action="store_true", help="report only; write nothing")
    a = ap.parse_args(argv)
    root = a.root.resolve()
    total = 0
    for rel in marked_files(root):
        p = root / rel
        text = p.read_bytes().decode("utf-8")
        try:
            new, blocks = strip_text(text, rel)
        except StripError as e:
            print(f"public_strip: ERROR {e}", file=sys.stderr)
            return 2
        total += blocks
        print(f"public_strip: {rel}: {blocks} block(s){' (check only)' if a.check else ''}")
        if a.check:
            continue
        p.write_bytes(new.encode("utf-8"))
        if a.git_add:
            subprocess.run(["git", "add", "--", rel], cwd=root, check=True)
    shipping = set(subprocess.run(["git", "ls-files"], cwd=root, capture_output=True, text=True, check=True).stdout.splitlines())
    for rel, key in PRUNE_JSON.items():
        if rel not in shipping:
            continue
        p = root / rel
        new, gone = prune_json(p.read_text(encoding="utf-8"), key, lambda path: path in shipping)
        print(f"public_strip: {rel}: {gone} entr(y/ies) for paths that do not ship{' (check only)' if a.check else ''}")
        if gone and not a.check:
            p.write_text(new, encoding="utf-8", newline="\n")
            if a.git_add:
                subprocess.run(["git", "add", "--", rel], cwd=root, check=True)
    if not a.check and a.git_add:
        left = marked_files(root)
        if left:
            print(f"public_strip: ERROR marker survived in {left}", file=sys.stderr)
            return 2
    print(f"public_strip: {total} block(s) stripped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
