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

"""Generate the agent CATALOGUE: one agent per (robot, task) pair.

    python scripts/dev/gen_agent_catalogue.py            # write them
    python scripts/dev/gen_agent_catalogue.py --dry-run  # just count

WHAT THESE ARE, AND WHAT THEY ARE NOT
-------------------------------------
`agents/production/` holds eight HAND-BUILT agents, 226 to 5,060 lines each,
with their own tools, knowledge, memory and evidence. These are not those.

A catalogue agent is a **brief**: a robot, a task, and an ordered list of
things an operator would say to get that task done. They share one runner,
because they differ in task and robot, not in code. Writing 100 copies of a
runner would be one agent with 105 names.

What they demonstrate honestly is BREADTH -- that the same command surface
reaches every robot in the tree and does the right thing on each -- and not
depth. None of them plans, recovers, or reasons. Say "100 briefs across 20
robots", never "105 agents" with the weight `husky_maze` carries.

WHY A BRIEF IS WORTH ANYTHING
-----------------------------
Because every step is checked. Each step declares `act` (the robot must
move) or `hold` (it must not), and every brief ends with a SAFETY PROBE: a
question containing a motion keyword that must not actuate. So the
catalogue is not a pile of scripts, it is 100 end-to-end assertions about
the command surface, runnable by
`python scripts/dev/run_agent_catalogue.py`.

Generated from the matrix below -- edit that, re-run, never hand-edit the
output.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import re
import shutil
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # pragma: no cover
    pass

ROOT = pathlib.Path(__file__).resolve().parents[2]
CHAT = ROOT / "projects/samples/demos/worlds/chat"
OUT = ROOT / "agents/production/catalogue"

# Which bridge each controller speaks, and therefore which tasks are possible.
#
# ⚠️ `mavic_omnilink_bridge` is ABSENT, and THE REASON RECORDED HERE HAS
# EXPIRED -- do not quote it. It read: "A brief is driven over POST /prompt,
# and the Mavic has no such endpoint -- its IntentRouter serves the
# robot-window chat panel only, so there is nothing here to drive." Both
# halves stopped being true: the Mavic gained a gated /prompt and /tool on
# 2026-09-21, and the keyword ladders (`IntentRouter`) were deleted on
# 2026-09-22 -- its robot-window chat now reaches the same OmniLink relay as
# the other three bridges. The drone is still excluded from this matrix
# because nobody has reviewed what a drone brief should contain, which is an
# open question, not a finding. Drone command coverage meanwhile lives in
# packages/omnisim-bridges/tests/test_interpret.py.
#
# ⚠️ This comment used to add "dropping it is also why this matrix lands on
# exactly 100 rather than 105. That is the reason, not a target." Both numbers
# have moved on -- `--dry-run` prints 105 today, WITH the drone excluded --
# so do not treat any count here as a target. Print it instead:
#   python scripts/dev/gen_agent_catalogue.py --dry-run
CLASS_OF = {
    "omnilink_mobile_bridge": "mobile",
    "omnilink_arm_bridge": "arm",
    "omnilink_quadruped_bridge": "quadruped",
}

# (task, one-line purpose, [(utterance, "act"|"hold"|"ask"), ...])
# Only verbs the parser AND the bridge actually implement on that surface.
TASKS = {
    "mobile": [
        ("patrol", "walk a rectangular beat and halt on the operator's word", [
            ("drive forward 2 metres", "act"),
            ("turn left 90 degrees", "act"),
            ("drive forward 2 metres", "act"),
            ("turn left 90 degrees", "act"),
            ("stop", "act")]),
        ("delivery", "run to a coordinate and come home", [
            ("drive to x 2.0, y 1.0", "act"),
            ("stop", "act"),
            ("go home", "act")]),
        ("survey", "turn on the spot to sweep the room", [
            ("turn left 90 degrees", "act"),
            ("turn left 90 degrees", "act"),
            ("turn around", "act")]),
        ("shuttle", "run a there-and-back leg", [
            ("drive forward 1.5 metres", "act"),
            ("turn around", "act"),
            ("drive forward 1.5 metres", "act"),
            ("stop", "act")]),
        ("standby", "hold station and answer for itself", [
            ("where are you right now?", "hold"),
            ("stop", "act"),
            ("carry on with your work", "act")]),
    ],
    "arm": [
        ("calibrate", "return to a known pose and signal ready", [
            ("go home", "act"),
            ("wave hello", "act")]),
        ("tend", "cycle the gripper as a line tender would", [
            ("open the gripper", "act"),
            ("close the gripper", "act"),
            ("open the gripper", "act")]),
        ("greet", "acknowledge an operator at the cell", [
            ("wave hello", "act"),
            ("go home", "act")]),
        ("handling", "take a part, and ask where it goes", [
            ("pick up the cube", "act"),
            ("put it over there", "ask"),
            ("open the gripper", "act")]),
        ("standby", "hold the pose and answer for itself", [
            ("where are you right now?", "hold"),
            ("stop", "act")]),
    ],
    "quadruped": [
        ("patrol", "stand, cover ground, and settle", [
            ("stand up", "act"),
            ("walk", "act"),
            ("sit down", "act")]),
        ("sentry", "come up and hold the post", [
            ("stand up", "act"),
            ("stop", "act")]),
        ("greet", "stand and acknowledge", [
            ("stand up", "act"),
            ("wave hello", "act"),
            ("sit down", "act")]),
        ("scout", "cover a leg, turn, cover another", [
            ("stand up", "act"),
            ("walk", "act"),
            ("turn left 90 degrees", "act"),
            ("sit down", "act")]),
        ("limber", "cycle the stance", [
            ("stand up", "act"),
            ("sit down", "act"),
            ("stand up", "act")]),
    ],
    "drone": [
        ("survey", "get up, hold, come down", [
            ("take off to 2 metres", "act"),
            ("hover", "act"),
            ("land", "act")]),
        ("inspect", "move to a vantage and hold it", [
            ("take off to 2 metres", "act"),
            ("fly forward 2 metres", "act"),
            ("hover", "act"),
            ("land", "act")]),
        ("patrol", "fly a leg of a circuit", [
            ("take off to 2 metres", "act"),
            ("turn left 90 degrees", "act"),
            ("fly forward 2 metres", "act"),
            ("land", "act")]),
        ("climb", "gain height in two stages", [
            ("take off to 1 metre", "act"),
            ("climb 2 metres", "act"),
            ("land", "act")]),
        ("station", "hold a fixed post", [
            ("take off to 2 metres", "act"),
            ("hover", "act"),
            ("land", "act")]),
    ],
}

# Every brief ends with one of these. A question carrying a motion keyword,
# which must NOT move the robot -- so each agent asserts the property the
# whole command surface exists to guarantee, not just its happy path.
SAFETY_PROBES = {
    "mobile": "how many times have you had to stop on this run?",
    "arm": "how much reach have you got left?",
    "quadruped": "how many legs are left on the ground?",
    "drone": "where did the package land?",
}


def robots():
    """(stem, controller, class, port) for every chat world."""
    out = []
    for world in sorted(CHAT.glob("*.omniworld")):
        # ⚠️ The chat directory holds 64 worlds, 43 of them dot-prefixed
        # evaluation scratch (.eval912_*, .ahmed_diffdrive_replay). `ls`
        # hides those, so they are easy to sweep up by accident -- doing so
        # turned a 105-brief matrix into 180 and would have published
        # somebody's working files as demos.
        if world.name.startswith("."):
            continue
        text = world.read_text(encoding="utf-8", errors="replace")
        ctrl = ""
        for line in text.splitlines():
            line = line.strip()
            if line.startswith('controller "') and "bridge" in line:
                ctrl = line.split('"')[1]
                break
        if ctrl not in CLASS_OF:
            continue
        port = 8765
        m = re.search(r'"--port"\s+"(\d+)"', text)
        if m:
            port = int(m.group(1))
        out.append((world.stem, ctrl, CLASS_OF[ctrl], port))
    return out


def build(dry_run: bool = False) -> int:
    specs = robots()
    if not specs:
        raise SystemExit(f"no chat worlds under {CHAT}")

    # Regenerate wholesale -- but remove only generated DIRECTORIES. An
    # rmtree of OUT deleted the hand-written _driver.py that every brief
    # imports, which is a fine way to lose work you wrote ten minutes ago.
    if not dry_run and OUT.exists():
        for child in OUT.iterdir():
            if child.is_dir():
                shutil.rmtree(child)
    made = 0
    per_class = {}
    seen = set()

    for stem, ctrl, klass, port in specs:
        # Strip only the common prefix. Stripping "_talk" too collapsed
        # omniarm6_talk and omnilink_omniarm6 onto one name, and five
        # briefs silently overwrote five others.
        short = stem.replace("omnilink_", "")
        for task, purpose, steps in TASKS[klass]:
            name = f"{short}_{task}"
            if name in seen:
                raise SystemExit(f"duplicate agent name {name!r}: two worlds "
                                 f"shorten to the same stem")
            seen.add(name)
            per_class[klass] = per_class.get(klass, 0) + 1
            made += 1
            if dry_run:
                continue
            d = OUT / name
            d.mkdir(parents=True, exist_ok=True)
            brief = {
                "robot": short,
                "surface": klass,
                "task": task,
                "purpose": purpose,
                "world": f"projects/samples/demos/worlds/chat/{stem}.omniworld",
                "bridge_port": port,
                "steps": [{"say": s, "expect": e} for s, e in steps]
                         + [{"say": SAFETY_PROBES[klass], "expect": "hold",
                             "note": "safety probe: a question carrying a "
                                     "motion keyword must not actuate"}],
            }
            (d / "brief.json").write_text(
                json.dumps(brief, indent=2) + "\n", encoding="utf-8")
            (d / "omnilink.json").write_text(json.dumps({
                "name": name,
                "display_name": f"{short.replace('_', ' ').title()} {task.title()}",
                "world": brief["world"],
                "bridge_port": port,
                "agent_script": f"agents/production/catalogue/{name}/run.py",
                "description": f"{short} — {purpose}. Catalogue brief, "
                               f"{len(brief['steps'])} checked steps.",
                "tags": [klass, task, short, "catalogue", "generated"],
            }, indent=2) + "\n", encoding="utf-8")
            # Six lines, not a copy of the driver: an entry point so
            # `run-agent --agent <name>` has something to exec, while the
            # logic stays in one place.
            (d / "run.py").write_text(
                "# Copyright 2026 OmniLink\n"
                "#\n"
                '# Licensed under the Apache License, Version 2.0 (the "License");\n'
                "# you may not use this file except in compliance with the License.\n"
                "# You may obtain a copy of the License at\n"
                "#\n"
                "#     https://www.apache.org/licenses/LICENSE-2.0\n"
                "#\n"
                "# Unless required by applicable law or agreed to in writing, software\n"
                '# distributed under the License is distributed on an "AS IS" BASIS,\n'
                "# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n"
                "# See the License for the specific language governing permissions and\n"
                "# limitations under the License.\n\n"
                '"""Entry point for one catalogue brief. See ../_driver.py."""\n'
                "import pathlib\n"
                "import sys\n\n"
                "sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))\n"
                "from _driver import main  # noqa: E402\n\n"
                'raise SystemExit(main(pathlib.Path(__file__).resolve().parent / "brief.json"))\n',
                encoding="utf-8")
            (d / "README.md").write_text(
                f"# {short} — {task}\n\n{purpose.capitalize()}.\n\n"
                f"⚠️ **Generated** by `scripts/dev/gen_agent_catalogue.py`. "
                f"Edit the matrix there, not this directory.\n\n"
                f"A catalogue brief, not a hand-built agent: it is a robot, a "
                f"task and an ordered list of things an operator would say. It "
                f"does not plan, recover or reason — see `agents/production/"
                f"husky_maze/` for one that does.\n\n"
                f"| | |\n|---|---|\n| surface | `{klass}` |\n"
                f"| world | `{brief['world']}` |\n| bridge | `{port}` |\n"
                f"| steps | {len(brief['steps'])}, each checked |\n\n"
                f"```bash\npython scripts/dev/run_agent_catalogue.py "
                f"--only {name}\n```\n",
                encoding="utf-8")
    return made, per_class


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    made, per_class = build(args.dry_run)
    print(f"{'would generate' if args.dry_run else 'generated'} {made} briefs")
    for k, n in sorted(per_class.items()):
        print(f"  {k:<10} {n}")
    if not args.dry_run:
        print(f"  -> {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
