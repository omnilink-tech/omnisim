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
  agent-build-new                           Print the locked Agent Build manifest
  agent-build-preflight <film.json>          Probe all media/evidence/voice inputs
  agent-build-proxy <film.json>              Render the exact fast editorial proxy
  agent-build-review <film.json>             Run local proxy critique + cut sheets
  agent-build-capture <capture.json>         Run/resume one persistent shot session
  agent-build-make <film.json>               Cached, proxy-gated end-to-end workflow
  agent-build-render <film.json>            Assemble the evidence-led film
  agent-build-verify <film.json>            Run the fail-closed release gate

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

from . import (
    agent_build,
    agent_build_capture,
    agent_build_pipeline,
    agent_build_review,
    agent_build_voice,
    beats,
    camera,
    director,
    looks,
    storyboard,
    subjects,
)


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


def _cmd_agent_build_new(args: argparse.Namespace) -> int:
    print(json.dumps(agent_build.template(args.title), indent=2))
    return 0


def _cmd_agent_build_validate(args: argparse.Namespace) -> int:
    spec = agent_build.parse(Path(args.manifest))
    print(json.dumps({
        "valid": True, "style": agent_build.STYLE_VERSION, "title": spec.title,
        "duration_s": spec.duration_s, "segments": len(spec.segments),
        "structure": spec.structure or "evidence_arc",
        "simulator_footage_ratio": round(spec.simulator_footage_ratio, 3),
        "locked_intro": {"disclosure_s": [0, 5], "story_signature_s": [5, 10],
                           "voiceover": False},
        "locked_outro": agent_build.GITHUB_DESTINATION,
    }, indent=2))
    return 0


def _cmd_agent_build_preflight(args: argparse.Namespace) -> int:
    spec = agent_build.parse(Path(args.manifest))
    print(json.dumps(agent_build.preflight(spec), indent=2))
    return 0


def _cmd_agent_build_voice_setup(args: argparse.Namespace) -> int:
    root = agent_build_voice.setup_runtime(Path(args.runtime) if args.runtime else None)
    print(f"Agent Build voice runtime ready: {root}")
    return 0


def _cmd_agent_build_voice(args: argparse.Namespace) -> int:
    spec = agent_build.parse(Path(args.manifest))
    wav, timings = agent_build_voice.generate(
        spec, Path(args.runtime) if args.runtime else None,
    )
    print(f"NARRATION: {wav}")
    print(f"TIMINGS:   {timings}")
    return 0


def _cmd_agent_build_render(args: argparse.Namespace) -> int:
    spec = agent_build.parse(Path(args.manifest))
    result = agent_build.render(spec, Path(args.out).resolve() if args.out else None)
    for label, path in result.items():
        print(f"{label.upper():>12}: {path}")
    return 0


def _cmd_agent_build_proxy(args: argparse.Namespace) -> int:
    spec = agent_build.parse(Path(args.manifest))
    result = agent_build.render(
        spec, Path(args.out).resolve() if args.out else None,
        profile=agent_build.PROXY_PROFILE,
    )
    for label, path in result.items():
        print(f"{label.upper():>12}: {path}")
    return 0


def _cmd_agent_build_review(args: argparse.Namespace) -> int:
    spec = agent_build.parse(Path(args.manifest))
    result = agent_build_review.review_proxy(
        spec, Path(args.out).resolve() if args.out else None,
    )
    print(json.dumps(result, indent=2))
    return 0


def _cmd_agent_build_capture_new(args: argparse.Namespace) -> int:
    print(json.dumps(agent_build_capture.template(args.world), indent=2))
    return 0


def _cmd_agent_build_capture(args: argparse.Namespace) -> int:
    result = agent_build_capture.run(
        Path(args.plan), service=args.svc,
        receipt_path=Path(args.receipt).resolve() if args.receipt else None,
    )
    print(json.dumps(result, indent=2))
    return 0


def _cmd_agent_build_make(args: argparse.Namespace) -> int:
    spec = agent_build.parse(Path(args.manifest))
    result = agent_build_pipeline.make(
        spec, out_dir=Path(args.out).resolve() if args.out else None,
        capture_plan=Path(args.capture_plan).resolve() if args.capture_plan else None,
        service=args.svc, target_minutes=args.target_minutes,
    )
    print(json.dumps(result, indent=2))
    return 0


def _cmd_agent_build_verify(args: argparse.Namespace) -> int:
    spec = agent_build.parse(Path(args.manifest))
    report = agent_build.verify(spec, Path(args.out).resolve() if args.out else None)
    print(json.dumps(report, indent=2))
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

    pan = sp.add_parser("agent-build-new", help="Print a starter Agent Build Film manifest")
    pan.add_argument("--title", default="My OmniSim Agent Build")
    pan.set_defaults(func=_cmd_agent_build_new)

    pav = sp.add_parser("agent-build-validate", help="Validate story/style contracts before rendering")
    pav.add_argument("manifest", help="Path to Agent Build Film JSON")
    pav.set_defaults(func=_cmd_agent_build_validate)

    pap = sp.add_parser("agent-build-preflight", help="Probe captures, ranges, evidence, and voice budgets")
    pap.add_argument("manifest", help="Path to Agent Build Film JSON")
    pap.set_defaults(func=_cmd_agent_build_preflight)

    pavs = sp.add_parser("agent-build-voice-setup", help="Install the pinned local natural-voice runtime")
    pavs.add_argument("--runtime", default=None, help="Optional runtime directory")
    pavs.set_defaults(func=_cmd_agent_build_voice_setup)

    pavr = sp.add_parser("agent-build-voice", help="Render natural narration from manifest windows")
    pavr.add_argument("manifest", help="Path to Agent Build Film JSON")
    pavr.add_argument("--runtime", default=None, help="Optional runtime directory")
    pavr.set_defaults(func=_cmd_agent_build_voice)

    par = sp.add_parser("agent-build-render", help="Assemble the locked Agent Build Film style")
    par.add_argument("manifest", help="Path to Agent Build Film JSON")
    par.add_argument("--out", default=None, help="Output directory")
    par.set_defaults(func=_cmd_agent_build_render)

    papr = sp.add_parser("agent-build-proxy", help="Render the exact fast editorial proxy")
    papr.add_argument("manifest", help="Path to Agent Build Film JSON")
    papr.add_argument("--out", default=None, help="Output directory")
    papr.set_defaults(func=_cmd_agent_build_proxy)

    parc = sp.add_parser("agent-build-review", help="Run the deterministic proxy critique gate")
    parc.add_argument("manifest", help="Path to Agent Build Film JSON")
    parc.add_argument("--out", default=None, help="Output directory")
    parc.set_defaults(func=_cmd_agent_build_review)

    pacn = sp.add_parser("agent-build-capture-new", help="Print a resumable capture-plan template")
    pacn.add_argument("--world", default="projects/samples/demos/worlds/flagship/build.omniworld")
    pacn.set_defaults(func=_cmd_agent_build_capture_new)

    pac = sp.add_parser("agent-build-capture", help="Run or resume one persistent capture session")
    pac.add_argument("plan", help="Path to capture plan JSON")
    pac.add_argument("--receipt", default=None, help="Optional receipt path")
    pac.set_defaults(func=_cmd_agent_build_capture)

    pam = sp.add_parser("agent-build-make", help="Run the cached proxy-gated production workflow")
    pam.add_argument("manifest", help="Path to Agent Build Film JSON")
    pam.add_argument("--capture-plan", default=None, help="Optional resumable capture plan")
    pam.add_argument("--out", default=None, help="Output directory")
    pam.add_argument("--target-minutes", type=float,
                     default=agent_build_pipeline.BENCHMARK_TARGET_MINUTES)
    pam.set_defaults(func=_cmd_agent_build_make)

    pave = sp.add_parser("agent-build-verify", help="Run the fail-closed Agent Build release gate")
    pave.add_argument("manifest", help="Path to Agent Build Film JSON")
    pave.add_argument("--out", default=None, help="Output directory used by render")
    pave.set_defaults(func=_cmd_agent_build_verify)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
