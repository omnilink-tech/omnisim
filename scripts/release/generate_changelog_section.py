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

"""Generate a CHANGELOG.md section from git history.

publish_snapshot.sh calls this when CHANGELOG.md has no entry for the
version being cut. Output goes to stdout: a Keep-a-Changelog–style
section (`## [vX.Y.Z] — YYYY-MM-DD` heading + categorized bullets)
ready to be prepended to CHANGELOG.md.

Categorization is path-driven: every commit's changed files are matched
against CATEGORY_RULES in order; the commit lands in the first matching
bucket. Junk subjects (one-word checkpoints), pure-merge commits, and
commits that only touched deny-listed paths are dropped. Internal
codename prefixes ("T1.1:", "M2:", "Direction A:", etc.) are stripped
so they don't leak into the public release note.

The output captures *what changed*, not *why* — the auto-generated
section is meant to be edited before publish if you want narrative
framing. The generator just guarantees nothing slips through unrecorded.
"""

import argparse
import datetime
import re
import subprocess
import sys
from collections import OrderedDict

JUNK_SUBJECTS = {"ch", "plan", "agent", "demo", "wip", "tmp"}

CODENAME_PREFIX_RE = re.compile(
    r"^(?:"
    r"T\d+(?:\.\d+)*[a-z]?(?:\s+\([^)]+\))?:\s+"
    r"|M\d+:\s+"
    r"|Direction\s+[A-Z]:\s+"
    r"|A/B/C(?:\s+[a-z-]+)?:\s+"
    r"|OLink-agents:\s+"
    r")",
)

CATEGORY_RULES = [
    (re.compile(r"^omnilink-reports/"), None),
    (re.compile(r"^agents/production/"), "Agents (`agents/production/`)"),
    (re.compile(r"^projects/robots/[^/]+/"), "Robots & PROTOs"),
    (re.compile(r"^projects/samples/demos/worlds/.*\.wbt$"), "Demos & worlds"),
    (re.compile(r"^distribution/generated_worlds/.*\.wbt$"), "Demos & worlds"),
    (re.compile(r"^tests/cuda/.*\.wbt$"), "CUDA / physics"),
    (re.compile(r"^src/omnisim/nodes/WbGranularGroup\."), "CUDA / physics"),
    (re.compile(r"^src/cuda/"), "CUDA / physics"),
    (re.compile(r"^tests/cuda/"), "CUDA / physics"),
    (re.compile(r"^src/omnisim/vrml/WbUrdfImporter\."), "Robots & PROTOs"),
    (re.compile(r"^resources/nodes/"), "Robots & PROTOs"),
    (re.compile(r"^scripts/harness/"), "Authoring & harness"),
    (re.compile(r"^projects/samples/demos/controllers/.*_omnilink_bridge/"),
     "Authoring & harness"),
    (re.compile(r"^scripts/release/"), "Release infrastructure"),
    (re.compile(r"^scripts/dev/"), "Build / packaging"),
    (re.compile(r"^build_omni\.bat$|^launch\.bat$|^Makefile$"),
     "Build / packaging"),
    (re.compile(r"^\.github/"), "Build / packaging"),
    (re.compile(r"^docs/"), "Documentation"),
    (re.compile(r"\.md$"), "Documentation"),
]

CATEGORY_ORDER = [
    "Demos & worlds",
    "Robots & PROTOs",
    "CUDA / physics",
    "Agents (`agents/production/`)",
    "Authoring & harness",
    "Build / packaging",
    "Release infrastructure",
    "Documentation",
    "Fixed",
    "Removed",
]


def run(cmd):
    # Force UTF-8 for subprocess output. On Windows the default decoder
    # is the active code page (cp1252), which silently mojibakes em-dashes
    # and other non-ASCII characters that appear in git commit subjects.
    return subprocess.check_output(
        cmd, text=True, encoding="utf-8", errors="replace"
    ).rstrip("\n")


def commits_in_range(since_ref):
    """Return list of (sha, subject) for commits in <since>..HEAD, oldest first."""
    fmt = "%H%x09%s"
    args = ["git", "log", "--no-merges", "--reverse", f"--format={fmt}"]
    if since_ref:
        args.append(f"{since_ref}..HEAD")
    out = run(args)
    if not out:
        return []
    pairs = []
    for line in out.splitlines():
        sha, _, subject = line.partition("\t")
        pairs.append((sha, subject))
    return pairs


def files_in_commit(sha):
    out = run(["git", "diff-tree", "--no-commit-id", "--name-only", "-r", sha])
    return [f for f in out.splitlines() if f]


def clean_subject(subject):
    s = CODENAME_PREFIX_RE.sub("", subject).strip()
    s = re.sub(r"\s+", " ", s)
    return s


def categorize(sha, subject):
    files = files_in_commit(sha)
    if not files:
        return None
    if all(f.startswith("omnilink-reports/") for f in files):
        return None

    sub_lower = subject.lower().lstrip()
    if re.match(r"^fix\b", sub_lower):
        return "Fixed"
    if re.match(r"^(remove|strip|drop)\b", sub_lower):
        return "Removed"

    non_denied = [f for f in files if not f.startswith("omnilink-reports/")]
    for f in non_denied:
        for pattern, category in CATEGORY_RULES:
            if pattern.search(f):
                return category
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--version", required=True,
                   help="Release version, e.g. v1.0.5")
    p.add_argument("--since", default=None,
                   help="Git ref to diff against (commit, tag, or branch)")
    p.add_argument("--date", default=None,
                   help="Override release date (YYYY-MM-DD; default: today)")
    p.add_argument("--out", default=None,
                   help="Write UTF-8 output to this file instead of stdout. "
                        "Recommended on Windows where the default stdout "
                        "encoding mangles em-dashes.")
    args = p.parse_args()

    if not re.match(r"^v\d+\.\d+\.\d+(-[A-Za-z0-9.-]+)?$", args.version):
        sys.exit(f"error: '{args.version}' is not vMAJOR.MINOR.PATCH")

    date = args.date or datetime.date.today().isoformat()

    commits = commits_in_range(args.since)
    if not commits:
        sys.exit(f"error: no commits between {args.since or 'root'} and HEAD")

    by_category = OrderedDict((c, []) for c in CATEGORY_ORDER)
    for sha, subject in commits:
        if subject.strip().lower() in JUNK_SUBJECTS:
            continue
        cleaned = clean_subject(subject)
        if not cleaned:
            continue
        category = categorize(sha, subject)
        if category is None:
            continue
        if cleaned not in by_category[category]:
            by_category[category].append(cleaned)

    has_any = any(items for items in by_category.values())
    if not has_any:
        sys.exit("error: every commit since the prior release was filtered "
                 "out (junk-subject or deny-listed). Refusing to write an "
                 "empty release note — author one manually instead.")

    out = [f"## [{args.version}] — {date}", ""]
    for category in CATEGORY_ORDER:
        items = by_category.get(category, [])
        if not items:
            continue
        out.append(f"### {category}")
        out.append("")
        for item in items:
            out.append(f"- {item}")
        out.append("")
    while out and not out[-1]:
        out.pop()

    rendered = "\n".join(out) + "\n"
    if args.out:
        with open(args.out, "w", encoding="utf-8", newline="\n") as f:
            f.write(rendered)
    else:
        # Force UTF-8 on stdout so Windows consoles don't mangle em-dashes.
        try:
            sys.stdout.reconfigure(encoding="utf-8")
        except AttributeError:
            pass
        sys.stdout.write(rendered)


if __name__ == "__main__":
    main()
