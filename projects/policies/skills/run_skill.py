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

"""Back-compat shim -> skill_lib.py (the manifest-driven skill runner).

The skill library grew from a deterministic-stand-overlay catalogue into the general
Shadowing + BATON pipeline; ``skill_lib.py`` is now the entry point. This shim keeps
the historical ``run_skill.py`` invocations working and forwards to it:

    python run_skill.py --list                       ->  skill_lib.py list
    python run_skill.py g1_arm_motion --throw --gui  ->  skill_lib.py run g1_arm_motion --throw --gui gui
    python run_skill.py balance_two_legs --throw     ->  skill_lib.py run balance_two_legs --throw

(Skill names are ``<robot>_<skill>`` for single-robot skills -- ``arm_motion`` is now
``g1_arm_motion``; multi-robot skills like ``balance_two_legs`` keep the bare name.)

Prefer ``skill_lib.py`` directly for the full surface (preview / train / sequence /
verify-demos). See README.md.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import skill_lib  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="Run an OmniSim policy skill (legacy shim -> skill_lib.py).")
    ap.add_argument("skill", nargs="?", help="skill name (see --list)")
    ap.add_argument("--list", action="store_true", help="list all skills")
    ap.add_argument("--throw", action="store_true", help="compose with -Throw (external cube pushes)")
    ap.add_argument("--gui", action="store_true", help="windowed run (else headless)")
    ap.add_argument("--duration", type=int, default=None, help="run duration seconds")
    ap.add_argument("--dry-run", action="store_true", help="print the launch command and exit")
    args = ap.parse_args()

    if args.list or not args.skill:
        return skill_lib.main(["list"])

    fwd = ["run", args.skill]
    if args.throw:
        fwd.append("--throw")
    fwd += ["--gui", "gui" if args.gui else "headless"]
    if args.duration is not None:
        fwd += ["--duration", str(args.duration)]
    if args.dry_run:
        fwd.append("--dry-run")
    return skill_lib.main(fwd)


if __name__ == "__main__":
    sys.exit(main())
