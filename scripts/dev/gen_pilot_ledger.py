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

"""Generate the pilot capability ledger.

Why this exists
---------------
Pilot evaluations are version-controlled under ``social/launch/pilots/`` -- 1,300+
tracked files, dozens of worlds and controllers. They are stored, but they are not
*indexed*, and an unindexed asset is indistinguishable from a missing one. On
2026-09-11 the outreach handover recorded a correspondent as blocked because
"no hexapod model exists in the tree", eleven days after a 72-link hexapod had
been imported and validated in ``pilots/hexapod_controller/``.

The same gap produced a claim error in the other direction: round 16 told three
recipients that "hexapods walk free-standing in it" on the strength of that clean
import, which had to be retracted. The import was real; the gait was not.

So the ledger carries two columns that matter more than the file counts:
what an evaluation **proved**, and what a reader would wrongly infer from it.

Facts (file counts, worlds, controllers, results) are derived here from git.
Claims are curated in ``social/launch/pilots/ledger_claims.json`` because no
script can read a result file and know which sentence would mislead a stranger.

Usage
-----
    python scripts/dev/gen_pilot_ledger.py            # write the ledger
    python scripts/dev/gen_pilot_ledger.py --check    # exit 1 if it would change

``--check`` is the drift gate: it is how the ledger avoids becoming the handover
it was written to replace.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PILOTS_DIR = REPO_ROOT / "social" / "launch" / "pilots"
CLAIMS_PATH = PILOTS_DIR / "ledger_claims.json"
OUTPUT_PATH = PILOTS_DIR / "CAPABILITY_LEDGER.md"

WORLD_SUFFIXES = (".omniworld", ".wbt")
RESULT_TOKENS = ("result", "findings", "report", "evidence")


def tracked_files(rel_dir: str) -> list[str]:
    """Files git actually tracks under *rel_dir*, POSIX-separated."""
    try:
        out = subprocess.run(
            ["git", "ls-files", "--", rel_dir],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=True,
        ).stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        return []
    return [line.strip() for line in out.splitlines() if line.strip()]


def is_ignored(rel_dir: str) -> bool:
    """True when .gitignore excludes this path outright.

    This is the condition worth alarming about, and it is NOT the same as
    "has no tracked files". An un-ignored directory that nobody has committed
    yet only needs `git add`; an *ignored* one is invisible to add, to status,
    and to every future session -- which is how a whole reply wave went missing.
    """
    try:
        return (
            subprocess.run(
                ["git", "check-ignore", "-q", "--", rel_dir],
                cwd=REPO_ROOT,
                capture_output=True,
            ).returncode
            == 0
        )
    except (OSError, subprocess.SubprocessError):
        return False


def derive_facts(pilot: str) -> dict:
    """Everything about a pilot that can be measured rather than asserted."""
    files = tracked_files(f"social/launch/pilots/{pilot}")
    lower = [f.lower() for f in files]

    worlds = [f for f in files if f.lower().endswith(WORLD_SUFFIXES)]
    controllers = sorted(
        {
            f.split("/controllers/", 1)[1].split("/", 1)[0]
            for f in files
            if "/controllers/" in f and "/" in f.split("/controllers/", 1)[1]
        }
    )
    results = [
        f
        for f, low in zip(files, lower)
        if any(tok in Path(low).name for tok in RESULT_TOKENS)
    ]
    has_readme = any(Path(low).name == "readme.md" for low in lower)

    on_disk = count_on_disk(f"social/launch/pilots/{pilot}")

    # "At risk" means *accidentally* excluded, which is a narrower thing than
    # "untracked". Several directories are ignored on purpose and by name --
    # luis_nav2/fork/ is a 10,973-file third-party fork, and counting it here
    # would inflate the headline into something nobody would act on. The
    # accident is specifically replies_<date>/<name>/, where the rule meant to
    # exclude a correspondent's clone now also excludes our own evaluation.
    ignored = is_ignored(f"social/launch/pilots/{pilot}")
    stageable = 0 if ignored else count_stageable(f"social/launch/pilots/{pilot}")

    return {
        "tracked": len(files),
        "on_disk": on_disk,
        "at_risk": ignored and on_disk > 0,
        "stageable": stageable,
        "uncommitted": stageable > 0,
        "worlds": worlds,
        "controllers": controllers,
        "results": results,
        "has_readme": has_readme,
    }


SKIP_DIRS = {"__pycache__", "_send_bundles", "build", ".git", ".venv"}


def _is_unit(path: Path) -> bool:
    if not path.is_dir() or path.name.startswith(".") or path.name in SKIP_DIRS:
        return False
    # Warp JIT kernel caches land beside evaluations and are ignored on purpose.
    # They are build residue, not somebody's evaluation, and counting them would
    # keep the "IGNORED" alarm permanently lit for a non-problem.
    return "warp_cache" not in path.name


def discover_pilots() -> list[str]:
    """Every evaluation unit, as a path relative to ``pilots/``.

    Three shapes exist, and the difference is not cosmetic -- it decides whether
    the work is in git at all:

    * a long-running pilot is one top-level directory (``hexapod_controller``);
    * most reply waves put our work under ``replies_<date>/evaluations/<name>``,
      which ``.gitignore`` protects by an explicit exception;
    * the two newest waves put it directly at ``replies_<date>/<name>``, which
      the rule ``/social/launch/pilots/replies_*/*/`` **excludes**. That rule was
      written to keep each correspondent's cloned repo out of git; when the
      layout changed it began excluding our own evaluations instead.

    The unit is the inner directory either way, because that is what one
    correspondent's evaluation is.
    """
    if not PILOTS_DIR.is_dir():
        return []

    units: list[str] = []
    for top in sorted(PILOTS_DIR.iterdir()):
        if not _is_unit(top):
            continue
        if top.name.startswith("replies_"):
            container = top / "evaluations"
            base = container if container.is_dir() else top
            prefix = f"{top.name}/evaluations" if container.is_dir() else top.name
            children = sorted(c.name for c in base.iterdir() if _is_unit(c))
            if children:
                units.extend(f"{prefix}/{c}" for c in children)
                continue
        units.append(top.name)
    return units


def count_stageable(rel_dir: str) -> int:
    """Files git would actually add: untracked and not excluded."""
    try:
        out = subprocess.run(
            ["git", "ls-files", "--others", "--exclude-standard", "--", rel_dir],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=True,
        ).stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        return 0
    return sum(1 for line in out.splitlines() if line.strip())


def count_on_disk(rel_dir: str) -> int:
    """Files present on disk, ignoring nested clones, venvs and caches."""
    root = REPO_ROOT / rel_dir
    if not root.is_dir():
        return 0
    total = 0
    for _path, dirnames, filenames in os.walk(root):
        dirnames[:] = [
            d
            for d in dirnames
            if d not in {".git", ".venv", "__pycache__"} and not d.startswith("warp_cache")
        ]
        total += len(filenames)
    return total


def bullets(items: list[str], indent: str = "") -> str:
    return "\n".join(f"{indent}- {item}" for item in items)


def plural(count: int, noun: str) -> str:
    return f"{count} {noun}" if count == 1 else f"{count} {noun}s"


def render_pilot(pilot: str, facts: dict, claim: dict | None) -> str:
    lines: list[str] = [f"### `{pilot}`", ""]

    if claim is None or claim.get("status") != "reviewed":
        lines += [
            "> **Unreviewed.** The evaluation artifacts below are tracked, but nobody has "
            "yet written down what they prove and what they do not. Treat this as "
            "*unknown*, not as *nothing* -- read the artifacts before telling anyone "
            "we cannot do this.",
            "",
        ]
    else:
        lines += [f"**{claim['capability']}**", ""]
        lines += ["**Proved:**", "", bullets(claim.get("proved", [])), ""]
        lines += [
            "**NOT proved** (the inference to avoid in correspondence):",
            "",
            bullets(claim.get("not_proved", [])),
            "",
        ]

        redistributable = claim.get("redistributable")
        if redistributable is False:
            lic = claim.get("licence", {})
            lines += ["**⛔ Not redistributable.** Do not copy these assets into `projects/`.", ""]
            for key, value in lic.items():
                if key == "note":
                    continue
                lines.append(f"- `{key}`: {value}")
            if lic.get("note"):
                lines += ["", f"  {lic['note']}"]
            lines.append("")
        elif redistributable is True:
            lines += ["**Redistributable.** No third-party licence constraint recorded.", ""]

        topics = claim.get("answers_requests_about") or []
        if topics:
            joined = ", ".join(f"`{t}`" for t in topics)
            lines += [f"**Reach for this when a request mentions:** {joined}", ""]

    if facts["at_risk"]:
        lines += [
            f"> 🚨 **IGNORED by `.gitignore`.** {plural(facts['on_disk'], 'file')} on disk, "
            "invisible to `git add`. Fix the rule, do not work around it.",
            "",
        ]
    elif facts["uncommitted"]:
        lines += [
            f"> 📌 **Not yet committed.** {plural(facts['stageable'], 'file')} stageable.",
            "",
        ]

    lines += [
        f"*{plural(facts['tracked'], 'tracked file')} · "
        f"{plural(facts['on_disk'], 'on disk')} · "
        f"{plural(len(facts['worlds']), 'world')} · "
        f"{plural(len(facts['controllers']), 'controller')} · "
        f"{plural(len(facts['results']), 'result file')}"
        f"{' · has README' if facts['has_readme'] else ''}*",
        "",
    ]
    return "\n".join(lines)


def build() -> str:
    claims_doc = json.loads(CLAIMS_PATH.read_text(encoding="utf-8"))
    claims = claims_doc.get("pilots", {})
    pilots = discover_pilots()

    facts = {p: derive_facts(p) for p in pilots}
    reviewed = [p for p in pilots if claims.get(p, {}).get("status") == "reviewed"]
    unreviewed = [p for p in pilots if p not in reviewed]
    total_tracked = sum(f["tracked"] for f in facts.values())
    at_risk = [p for p in pilots if facts[p]["at_risk"]]
    at_risk_files = sum(facts[p]["on_disk"] for p in at_risk)
    uncommitted = [p for p in pilots if facts[p]["uncommitted"]]
    uncommitted_files = sum(facts[p]["stageable"] for p in uncommitted)

    out: list[str] = [
        "# Pilot capability ledger",
        "",
        "<!-- GENERATED by scripts/dev/gen_pilot_ledger.py -- do not hand-edit.",
        "     Claims live in ledger_claims.json; facts are derived from git. -->",
        "",
        "What the pilot evaluations under `social/launch/pilots/` have actually",
        "established, and -- the column that matters more -- what they have not.",
        "",
        "**Read this before telling any correspondent we cannot do something.** The",
        "handover recorded a request as blocked on \"no hexapod model exists in the",
        "tree\" eleven days after one was imported and validated. The assets were",
        "tracked the whole time; they just were not indexed.",
        "",
        "**And read the NOT-proved lines before making a claim.** A clean URDF import",
        "was once described to three recipients as \"hexapods walk free-standing in",
        "it\", and had to be retracted. Both failures are the same missing index.",
        "",
        f"Coverage: **{len(reviewed)} of {len(pilots)} evaluations reviewed**, "
        f"{total_tracked:,} tracked files.",
        "",
        *(
            [
                f"🚨 **{len(at_risk)} evaluations ({at_risk_files:,} files) are IGNORED by "
                "`.gitignore`** — invisible to `git add`, to `git status`, and to every future "
                "session. A `git clean` would delete them. This is the failure that hid a whole "
                "reply wave; if this line is non-zero, fix the rule before anything else.",
                "",
            ]
            if at_risk
            else []
        ),
        *(
            [
                f"📌 **{len(uncommitted)} evaluations, {uncommitted_files:,} files "
                "stageable but not yet committed.** Until someone runs `git add`, this work "
                "lives on one machine only.",
                "",
            ]
            if uncommitted
            else []
        ),
        "⚠️ `social/` is publish-denied. Nothing here is a source for a public claim",
        "about the simulator, and one correspondent's evaluation data is never quoted",
        "to another.",
        "",
        "---",
        "",
        "## Reviewed",
        "",
    ]

    for pilot in reviewed:
        out.append(render_pilot(pilot, facts[pilot], claims.get(pilot)))

    out += [
        "---",
        "",
        "## Unreviewed",
        "",
        "Tracked, but with no recorded claim. **Unknown, not empty.**",
        "",
    ]
    def summarise(unit: str) -> str:
        f = facts[unit]
        parts = [plural(f["tracked"], "tracked file")]
        if f["worlds"]:
            parts.append(plural(len(f["worlds"]), "world"))
        if f["controllers"]:
            parts.append(plural(len(f["controllers"]), "controller"))
        if f["results"]:
            parts.append(plural(len(f["results"]), "result file"))
        if f["at_risk"]:
            return f"🚨 IGNORED — {plural(f['on_disk'], 'file')} on disk"
        if f["uncommitted"]:
            return f"📌 uncommitted — {plural(f['stageable'], 'file')} stageable"
        return ", ".join(parts)

    standalone = [u for u in unreviewed if "/" not in u]
    waves: dict[str, list[str]] = {}
    for unit in unreviewed:
        if "/" in unit:
            wave, name = unit.split("/", 1)
            waves.setdefault(wave, []).append(name)

    for unit in standalone:
        out.append(f"- **`{unit}`** — {summarise(unit)}")

    for wave in sorted(waves):
        out += ["", f"**`{wave}`** — {plural(len(waves[wave]), 'evaluation')}", ""]
        for name in waves[wave]:
            out.append(f"- `{name}` — {summarise(f'{wave}/{name}')}")

    out += [
        "",
        "---",
        "",
        "Regenerate: `python scripts/dev/gen_pilot_ledger.py`  ·  "
        "drift gate: `--check`",
        "",
    ]
    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="exit non-zero if the ledger on disk is stale",
    )
    args = parser.parse_args()

    if not CLAIMS_PATH.exists():
        print(f"error: missing {CLAIMS_PATH.relative_to(REPO_ROOT)}", file=sys.stderr)
        return 2

    rendered = build()

    if args.check:
        current = OUTPUT_PATH.read_text(encoding="utf-8") if OUTPUT_PATH.exists() else ""
        if current != rendered:
            print(
                "Pilot ledger is stale. Run: python scripts/dev/gen_pilot_ledger.py",
                file=sys.stderr,
            )
            return 1
        print("Pilot ledger is current.")
        return 0

    OUTPUT_PATH.write_text(rendered, encoding="utf-8")
    print(f"wrote {OUTPUT_PATH.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
