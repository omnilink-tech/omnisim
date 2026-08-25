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

"""`python -m omnisim cinema` — agent-facing entry point.

Subcommands:
  render   <storyboard.json>           Render + edit + brand a storyboard
  new      [--subject ...] [--world ...]   Print a starter storyboard JSON
  primitives                           List all camera primitives + describe
  beats                                List all beats + their default shots
  looks                                List all named looks
  subjects                             List all known subject profiles
  inspect  <world.omniworld>                 Load world, list robots/poses, exit

The render path expects an existing capture service on 127.0.0.1:6791.
Start it once with `python -m omnisim capture --port 6791` and keep it
up between storyboards.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

from . import beats, camera, director, looks, storyboard, subjects


def _cmd_render(args: argparse.Namespace) -> int:
    sb = storyboard.parse(Path(args.storyboard))
    opts = director.DirectorOptions(
        svc_base_url=args.svc,
        output_root=Path(args.out_root).resolve() if args.out_root else director.REPO_ROOT
            / "social" / "youtube_videos" / "captures",
        skip_critique=args.no_critique,
        skip_edit=args.no_edit,
    )
    result = director.direct(sb, opts)
    print()
    print(f"OUTPUT DIR: {result.output_dir}")
    for label, p in result.deliverables.items():
        print(f"  [{label:>6}] {p.name}  ({p.stat().st_size // 1024} KB)")
    if result.notes:
        print("\nNOTES:")
        for n in result.notes:
            print(f"  - {n}")
    return 0 if result.deliverables or opts.skip_edit else 1


def _cmd_new(args: argparse.Namespace) -> int:
    tpl = storyboard.template(
        title=args.title, subject=args.subject, world=args.world,
    )
    print(json.dumps(tpl, indent=2))
    return 0


def _cmd_primitives(_args: argparse.Namespace) -> int:
    print("Camera primitives (use as 'shot' in a storyboard):\n")
    for name, fn in camera.PRIMITIVES.items():
        doc = (fn.__doc__ or "").strip().splitlines()[0] if fn.__doc__ else ""
        print(f"  {name:<16}  {doc}")
    return 0


def _cmd_beats(_args: argparse.Namespace) -> int:
    print("Story beats (use as 'beat' in a storyboard):\n")
    for name, b in beats.BEATS.items():
        print(f"  {name:<14}  default_shot={b.default_shot:<14} {b.default_duration_s}s")
        print(f"  {'':14}  {b.feels_like}")
    return 0


def _cmd_looks(_args: argparse.Namespace) -> int:
    print("Named looks (use as 'look' in a storyboard):\n")
    for name, lk in looks.LOOKS.items():
        print(f"  {name:<18}  lens={lk.preferred_lens:<6} {lk.description}")
    return 0


def _cmd_subjects(_args: argparse.Namespace) -> int:
    print("Known subject profiles (use as 'subject' in a storyboard):\n")
    for name, prof in subjects.PROFILES.items():
        print(f"  {name:<14}  char_dim={prof.char_dim_m:>4.2f}m  "
              f"personality={prof.personality}  aliases={list(prof.aliases)}")
    return 0


def _cmd_inspect(args: argparse.Namespace) -> int:
    svc = args.svc
    print(f"[inspect] loading {args.world} via {svc}", file=sys.stderr)
    body = json.dumps({"path": args.world, "wait_s": 75,
                       "width": 1920, "height": 1080}).encode("utf-8")
    req = urllib.request.Request(f"{svc}/world/load", data=body,
                                  headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as resp:
        load = json.loads(resp.read().decode("utf-8"))
    if not load.get("ok"):
        print(f"world load failed: {load.get('error')}", file=sys.stderr)
        return 2
    # Settle.
    settle_body = json.dumps({"steps": 30}).encode("utf-8")
    settle_req = urllib.request.Request(f"{svc}/sim/step", data=settle_body,
                                         headers={"Content-Type": "application/json"})
    urllib.request.urlopen(settle_req, timeout=30).read()
    robots = subjects.list_robots_in_world(svc)
    print(json.dumps({"world": args.world, "robots": robots}, indent=2))
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="omnisim cinema",
                                description="Agent-driven cinematic capture pipeline.")
    p.add_argument("--svc", default="http://127.0.0.1:6791",
                   help="Capture service base URL (default: %(default)s)")
    sp = p.add_subparsers(dest="cmd", required=True)

    pr = sp.add_parser("render", help="Render a storyboard end-to-end")
    pr.add_argument("storyboard", help="Path to storyboard JSON")
    pr.add_argument("--out-root", default=None,
                    help="Output root (default: social/youtube_videos/captures/)")
    pr.add_argument("--no-critique", action="store_true",
                    help="Skip the vision-critique reshoot loop")
    pr.add_argument("--no-edit", action="store_true",
                    help="Render shots only — don't assemble or brand")
    pr.set_defaults(func=_cmd_render)

    pn = sp.add_parser("new", help="Print a starter storyboard JSON")
    pn.add_argument("--title", default="New Cinema Piece")
    pn.add_argument("--subject", default="omniquad")
    pn.add_argument("--world", default="projects/policies/research/worlds/omniquad_rl_deploy.omniworld")
    pn.set_defaults(func=_cmd_new)

    pp = sp.add_parser("primitives", help="List camera primitives")
    pp.set_defaults(func=_cmd_primitives)
    pb = sp.add_parser("beats", help="List story beats")
    pb.set_defaults(func=_cmd_beats)
    pl = sp.add_parser("looks", help="List named looks")
    pl.set_defaults(func=_cmd_looks)
    ps = sp.add_parser("subjects", help="List subject profiles")
    ps.set_defaults(func=_cmd_subjects)

    pi = sp.add_parser("inspect", help="Load a world and dump its robots")
    pi.add_argument("world", help="Path to .wbt")
    pi.set_defaults(func=_cmd_inspect)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
