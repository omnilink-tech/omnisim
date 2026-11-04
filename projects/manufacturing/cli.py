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

"""OmniSim Manufacturing: one command over every machine, process and product twin.

    python -m omnisim manufacturing                         what is here
    python -m omnisim manufacturing setup [--yes]           install the Python packages the stages import
    python -m omnisim manufacturing doctor [--json]         per stage: can it run here, and what is missing
    python -m omnisim manufacturing machines                every machine, its process and files
    python -m omnisim manufacturing worlds                  every world that runs one
    python -m omnisim manufacturing <process> <tool> ...    run a process's tool, e.g.
        python -m omnisim manufacturing printing slice part.stl --out part.gcode
        python -m omnisim manufacturing printing preflight part.gcode
        python -m omnisim manufacturing cnc preflight job.nc --machine <grbl config>
        python -m omnisim manufacturing laser run <world> <job.gcode> --out <dir>
    python -m omnisim manufacturing cad-pack <manifest> --out <dir>   CAD -> a machine's mesh pack
    python -m omnisim manufacturing products                every product, its stages and its results
    python -m omnisim manufacturing product <name> <stage>  a product's own script for a stage, e.g.
        python -m omnisim manufacturing product spark_rover check
        python -m omnisim manufacturing product spark_rover make --plan
        python -m omnisim manufacturing product spark_rover all --dry-run
      (stages: check, wire, make, print, print-ideal, assemble, drive, release, all;
       product_cli.py --help has the details)

A tool is the process's own script, run unchanged with the arguments given (each
answers --help). This is a router, not a second interface: the scripts stay the source
of truth for what each tool does.
"""
from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
WORLDS = ROOT / "projects/samples/demos/worlds/manufacturing"

# process -> {tool: script}. The process's world generator, planner and checks.
PROCESSES = {
    "cnc": {
        "about": "subtractive: milling on a GRBL router (MillingStock carves the stock)",
        "tools": {"preflight": "preflight.py", "world": "world.py", "inspect": "inspect_part.py",
                  "toolpath": "toolpath.py", "toolpath3d": "toolpath3d.py", "parts": "parts.py",
                  "server": "server.py", "bridge": "cnc_bridge.py"},
    },
    "laser": {
        "about": "laser cutting and engraving: a GRBL laser module on a CNC (MillingStock burns the sheet)",
        "tools": {"kerf": "laser.py", "job": "job.py", "world": "world.py", "run": "run_job.py",
                  "inspect": "inspect_cut.py"},
    },
    "printing": {
        "about": "additive: FDM printing on a Klipper printer (PrintedPart lays the plastic)",
        "tools": {"slice": "cura_slice.py", "preflight": "preflight.py", "world": "world.py",
                  "inspect": "inspect_print.py", "printer": "printer.py"},
    },
}


def _machines():
    """Every machine folder: what process drives it, its config, and its CAD pack if any."""
    out = []
    for d in sorted(p for p in (HERE / "machines").iterdir() if p.is_dir()):
        configs = sorted(p.name for p in d.iterdir() if p.is_file() and p.suffix in (".grbl", ".yaml", ".json"))
        process = "printing" if any(_is_printer_profile(d / c) for c in configs if c.endswith(".json")) else "cnc"
        lasers = [c for c in configs if c.endswith(".json") and _json(d / c).get("kind") == "laser"]
        pack = d / "cad" / "pack.json"
        licence = None
        if pack.exists():
            licence = json.loads(pack.read_text(encoding="utf-8")).get("source", {}).get("licence")
        out.append({"machine": d.name, "process": process, "configs": configs, "lasers": lasers,
                    "cad": str((d / "cad").relative_to(ROOT).as_posix()) if pack.exists() else None,
                    "cad_licence": licence})
    return out


def _json(path: Path) -> dict:
    try:
        d = json.loads(path.read_text(encoding="utf-8"))
        return d if isinstance(d, dict) else {}
    except (ValueError, OSError):
        return {}


def _is_printer_profile(path: Path) -> bool:
    return "klipper" in _json(path)


def _worlds():
    rows = []
    for w in sorted(w for w in WORLDS.glob("*.omniworld") if not w.name.startswith(".")):   # not harness copies
        head = w.read_text(encoding="utf-8", errors="replace")[:2000]
        m = re.search(r'controller "([^"]+)"', head) or re.search(r'controller "([^"]+)"', w.read_text(encoding="utf-8", errors="replace"))
        rows.append((w.name, m.group(1) if m else "-"))
    return rows


def _overview() -> int:
    print("OmniSim Manufacturing: machine twins that make parts, and the products made with them.")
    print("Simulated only: no twin here has been compared against its real machine yet.\n")
    for name, spec in PROCESSES.items():
        print(f"  {name:<9} {spec['about']}")
        print(f"            tools: {', '.join(spec['tools'])}")
    print(f"\n  machines  {', '.join(m['machine'] for m in _machines())}")
    print(f"  products  {', '.join(p.name for p in sorted((HERE / 'products').iterdir()) if p.is_dir())}")
    print(f"  worlds    {len(_worlds())} in {WORLDS.relative_to(ROOT).as_posix()}/")
    print("\n  manufacturing setup                    install the Python packages the stages import (asks first)")
    print("  manufacturing doctor                   per stage: can it run here, and what is missing")
    print("  manufacturing products                 every product's stages and results")
    print("  manufacturing product <name> <stage>   check, wire, make, print, print-ideal, assemble, drive, release, all")
    print("\nGuides: projects/manufacturing/README.md, docs/guide/manufacturing-make-your-own-product.md")
    return 0


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        return _overview()
    head = argv[0]
    if head in ("-h", "--help", "help"):
        print(__doc__)
        return 0
    if head == "machines":
        for m in _machines():
            cad = f"  cad: {m['cad']} ({m['cad_licence']})" if m["cad"] else ""
            process = m["process"] + ("+laser" if m["lasers"] else "")
            print(f"{m['machine']:<22} {process:<15} {', '.join(m['configs'])}{cad}")
        return 0
    if head == "worlds":
        for name, controller in _worlds():
            print(f"{name:<42} controller {controller}")
        return 0
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    if head in ("setup", "doctor"):
        import deps
        return deps.setup(argv[1:]) if head == "setup" else deps.doctor(argv[1:])
    if head in ("products", "product"):
        import product_cli
        return product_cli.main(argv)
    if head == "cad-pack":
        return _run_tool("cad-pack", "", HERE / "core/cad_pack.py", argv[1:])
    if head in PROCESSES:
        tools = PROCESSES[head]["tools"]
        if len(argv) < 2 or argv[1] not in tools:
            print(f"{head} tools: {', '.join(tools)}  (each answers --help)", file=sys.stderr)
            return 2
        return _run_tool(head, argv[1], HERE / head / tools[argv[1]], argv[2:])
    print(f"unknown: {head!r}. Try: setup, doctor, machines, worlds, cad-pack, products, product, "
          f"{', '.join(PROCESSES)}", file=sys.stderr)
    return 2


def _run_tool(process: str, tool: str, script: Path, args) -> int:
    """A process tool's script, unless a package it imports is missing: then name it, and the fix."""
    import deps
    problem = deps.tool_gate(process, tool)
    if problem:
        print(problem, file=sys.stderr)
        return 3
    return subprocess.run([sys.executable, str(script), *args], cwd=ROOT).returncode


if __name__ == "__main__":
    raise SystemExit(main())
