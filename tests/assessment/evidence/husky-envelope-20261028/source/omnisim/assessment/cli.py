"""The first assess CLI: run, rerun, regrade, and a complete braking proof."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from .schema import BrakingSpec
from .runner import prove, read_json, regrade, rerun, run


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m omnisim assess")
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("run", help="Run one fresh Husky braking episode")
    p.add_argument("--spec", type=Path, help="Frozen JSON spec; defaults to unloaded 0.6 m/s braking")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--capture", action="store_true", help="Enable native rendering and request/final stills")
    p = sub.add_parser("rerun", help="Fresh engine reproduction with the recorded spec and source hashes")
    p.add_argument("case", type=Path)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--capture", action="store_true")
    p = sub.add_parser("regrade", help="Recompute the behavioral verdict from saved evidence without an engine")
    p.add_argument("case", type=Path)
    p = sub.add_parser("prove", help="Five baselines, delayed failure, fresh reproduction, fix and independent-speed confirmation")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--capture", action="store_true")
    p = sub.add_parser("robots", help="Inventory all URDF models and available task adapters; availability is not qualification")
    p.add_argument("--supported", action="store_true")
    p = sub.add_parser("sweep", help="Immediate-braking slope/friction/payload campaign (schema 2)")
    p.add_argument("--robots", nargs="+", default=["husky"], help="Physical-wheel robot ids, or all")
    p.add_argument("--slopes", nargs="+", type=float, default=[0,5,10,20])
    p.add_argument("--frictions", nargs="+", type=float, default=[.1,.2,.4,.8])
    p.add_argument("--payloads", nargs="+", type=float, default=[0,10,20])
    p.add_argument("--refine", type=int, default=0, help="Additional sampled-transition refinement episodes")
    p.add_argument("--confirm", type=int, default=0, help="Fresh repeats per side of one qualified braking transition")
    p.add_argument("--workers", type=int, default=1)
    p.add_argument("--max-episodes", type=int, default=120)
    p.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "robots":
            from .registry import inventory
            items=inventory()
            print(json.dumps([i for i in items if i["supported_tasks"]] if args.supported else items,indent=2))
            return 0
        if args.command == "sweep":
            from .campaign import sweep
            result=sweep(args.out,args.robots,args.slopes,args.frictions,args.payloads,args.refine,args.confirm,args.workers,args.max_episodes)
            print(json.dumps({"cases":len(result["cases"]),"boundaries":len(result["boundaries"]),"all_evidence_valid":result["all_evidence_valid"]}))
            return 0 if result["all_evidence_valid"] else 2
        if args.command == "prove":
            return 0 if prove(args.out, args.capture) else 1
        if args.command == "run":
            verdict = run(BrakingSpec.from_dict(read_json(args.spec)) if args.spec else BrakingSpec(), args.out, args.capture)
        elif args.command == "rerun":
            verdict = rerun(args.case, args.out, args.capture)
        else:
            verdict = regrade(args.case)
        print(json.dumps(verdict, indent=2))
        return 0 if verdict["status"] == "PASS" else 1 if verdict["status"] == "FAIL" else 2
    except (ValueError, OSError) as exc:
        parser.exit(2, f"Assessment error: {exc}\n")
