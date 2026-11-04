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

"""OmniSim Manufacturing products: one command over each product's own scripts.

    python -m omnisim manufacturing products [--json]          every product, its stages, its results
    python -m omnisim manufacturing product <name>              the product's stages and the scripts they run
    python -m omnisim manufacturing product <name> <stage> [args]
    python -m omnisim manufacturing product <name> all [--keep-going]

Stages, in build order (a product has the ones whose scripts it has):

    check        skeleton.py, then integrate.py      the contract's checks; zones, FCL interference, mounts, mass
    wire         harness.py                          route every wire through the assembled robot
    make         make.py                             the laser and CNC parts on their twins, measured   [engine]
    print        make_print.py --run                 the printed parts on the printer twin, plate by plate [engine]
    print-ideal  make_print.py --ideal --only <plate> the same plates in ideal mode (no engine)
    assemble     assemble.py                         the assembly in build order: FCL sweeps, then Newton [engine]
    drive        robot.py, then run-headless         write the drive-test world and run it on Newton  [engine]
    release      release.py                          regenerate release/ from the result files (no engine)

Arguments after the stage go to that stage's script unchanged (each argparse script answers
--help; a script that takes no arguments has its docstring printed instead of being run).
With no arguments, `print` runs `make_print.py --run`; any argument replaces that, so
`print --plan` plans without printing. The router's own flags, which no product script uses:

    --dry-run       print the commands, run nothing (any stage, and `all`)
    --keep-going    `all` only: run the later stages after one fails
    --duration N    `drive` only: run-headless --duration (default 60 s of wall time)

Before a stage runs, the router checks what it imports (deps.py): a missing Python package,
or CuraEngine for a stage that must slice, is named with its fix and the stage exits 3
without starting. `python -m omnisim manufacturing setup` installs the packages;
`python -m omnisim manufacturing doctor` shows every stage.

This is a router, not a second interface: the scripts stay the source of truth.
"""
from __future__ import annotations

import ast
import json
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PRODUCTS = HERE / "products"
DRIVE_DURATION_S = 60          # the SparkRover README's run-headless --duration


@dataclass(frozen=True)
class Stage:
    name: str
    what: str
    engine: bool
    # (script, args always given, args given only when the user passes none)
    commands: Tuple[Tuple[str, Tuple[str, ...], Tuple[str, ...]], ...]


STAGES: Tuple[Stage, ...] = (
    Stage("check", "the skeleton contract's checks, then integration (zones, FCL interference, mounts, mass)",
          False, (("skeleton.py", (), ()), ("integrate.py", (), ()))),
    Stage("wire", "route every wire through the assembled robot", False, (("harness.py", (), ()),)),
    Stage("make", "the laser and CNC parts on their twins, measured against their drawings", True,
          (("make.py", (), ()),)),
    Stage("print", "the printed parts on the printer twin, plate by plate, measured", True,
          (("make_print.py", (), ("--run",)),)),
    Stage("print-ideal", "the --only plates in ideal mode (printing/ideal_print.py, no engine)", False,
          (("make_print.py", ("--ideal",), ()),)),
    Stage("assemble", "the assembly in build order: each unit swept (FCL), seated, then let go on Newton", True,
          (("assemble.py", (), ()),)),
    Stage("drive", "write the drive-test world, then run it headless on Newton", True, (("robot.py", (), ()),)),
    Stage("release", "regenerate release/ from the result files (no engine)", False, (("release.py", (), ()),)),
)
BY_NAME = {s.name: s for s in STAGES}
ALL_ORDER = ("check", "wire", "make", "print", "assemble", "drive", "release")   # print-ideal is print's alternative

# A product that predates the convention: its stages as its own README runs them. A product
# laid out like spark_rover (every stage script, each with working defaults) needs no entry.
OVERRIDES: Dict[str, Dict[str, Tuple[Tuple[str, Tuple[str, ...], Tuple[str, ...]], ...]]] = {
    "desk_robot": {   # products/desk_robot/README.md and design.py's docstring
        "check": (("design.py", ("--out", ".tmp/desk_robot"), ()), ("bought_parts.py", ("clearance",), ())),
        "wire": (("wiring.py", (), ()),),
        "make": (("make.py", ("--out", ".tmp/desk_robot"), ()),),
        "release": (("release.py", ("--made", ".tmp/desk_robot", "--drive", ".tmp/desk_robot/drive.json"), ()),),
    },
}
NOTES = {   # what a product does instead of a stage it lacks
    "desk_robot": {"print": "its one print plate is printed by make: `make --only print`",
                   "print-ideal": "no ideal-mode entry point; printing/ideal_print.py runs on its plate's G-code"},
}


# ---- discovery ------------------------------------------------------------------------------

def commands(product: Path, stage: str):
    """The (script, always, default) triples a stage runs for this product, or None if it lacks one."""
    cmds = OVERRIDES.get(product.name, {}).get(stage) or BY_NAME[stage].commands
    return cmds if all((product / script).is_file() for script, _, _ in cmds) else None


def products(products_dir: Path = PRODUCTS) -> List[Path]:
    """Every folder under products/ with at least one stage's scripts."""
    return [d for d in sorted(products_dir.iterdir())
            if d.is_dir() and not d.name.startswith((".", "_")) and any(commands(d, s.name) for s in STAGES)]


def others(products_dir: Path = PRODUCTS) -> List[str]:
    return [d.name for d in sorted(products_dir.iterdir())
            if d.is_dir() and not d.name.startswith((".", "_", "__")) and d not in products(products_dir)]


def _source(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def takes_args(script: Path) -> bool:
    return "ArgumentParser" in _source(script)


def docstring(script: Path) -> str:
    try:
        return ast.get_docstring(ast.parse(_source(script))) or "(no docstring)"
    except SyntaxError:
        return "(unreadable)"


def title(product: Path) -> str:
    readme = product / "README.md"
    if readme.exists():
        for line in _source(readme).splitlines():
            if line.startswith("# "):
                return line[2:].strip()
    return product.name


# ---- what the result files say ---------------------------------------------------------------

def _json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _rel(path: Path) -> str:
    try:
        return path.relative_to(ROOT).as_posix()
    except ValueError:
        return str(path)


def _relp(path: Path, product: Path) -> str:
    """A file inside the product folder by its path there; anything else from the repo root."""
    try:
        return path.relative_to(product).as_posix()
    except ValueError:
        return _rel(path)


def _first(*paths: Path) -> Optional[Path]:
    return next((p for p in paths if p.exists()), None)


def made_status(product: Path) -> dict:
    """make's results: release/make/made.json, else .tmp/<product>/made.json (make.py --out)."""
    path = _first(product / "release/make/made.json", ROOT / ".tmp" / product.name / "made.json")
    d = _json(path) if path else None
    if not isinstance(d, dict):
        return {"state": "not run", "file": None}
    out = {"file": _relp(path, product), "in_release": "release" in path.parts, "by_process": {}}
    if "jobs" in d:                                    # spark_rover: one record per job, parts inside
        jobs = d["jobs"]
        for name, j in jobs.items():
            src = jobs.get(j.get("same_as")) if j.get("same_as") else j
            proc = (src or {}).get("process", "?")
            parts = (src or {}).get("parts") or {}
            row = out["by_process"].setdefault(proc, {"parts": 0, "cut_free": 0, "jobs": 0})
            row["jobs"] += 1
            row["parts"] += len(parts)
            row["cut_free"] += sum(1 for p in parts.values() if p.get("cut_free"))
    else:                                              # desk_robot: one record per process
        for proc, rec in d.items():
            if not isinstance(rec, dict):
                continue
            if proc == "print":
                done = (rec.get("done") or {}).get("kind") == "done"
                out["by_process"]["print"] = {"plates": 1, "printed": int(done), "mode": "twin"}
            elif "hubs" in rec:
                out["by_process"][proc] = {"parts": len(rec["hubs"]), "cut_free": int(rec.get("cut_free") or 0)}
            else:
                parts = [p for p in rec.values() if isinstance(p, dict) and "cut_free" in p]
                out["by_process"][proc] = {"parts": len(parts), "cut_free": sum(1 for p in parts if p["cut_free"])}
    return out


def print_status(product: Path) -> dict:
    """make_print's printed.json: every current part by the mode it printed in."""
    path = product / "release/make/print/printed.json"
    d = _json(path) if path.exists() else None
    if not isinstance(d, dict):
        return {"state": "no printed.json", "file": None}
    current = [p for p in d.get("parts", []) if not p.get("superseded")]
    modes = [set(p.get("modes_printed") or ()) for p in current]
    return {"file": _relp(path, product), "parts": len(current), "twin": sum(1 for m in modes if "twin" in m),
            "ideal_only": sum(1 for m in modes if m == {"ideal"}), "pending": sum(1 for m in modes if not m),
            "superseded": sum(1 for p in d.get("parts", []) if p.get("superseded")),
            "plates": [pl.get("name") for pl in d.get("plates", [])]}


def assembly_status(product: Path) -> dict:
    path = product / "release/assembly/assembly_log.json"
    d = _json(path) if path.exists() else None
    if not isinstance(d, dict):
        return {"state": "not run", "file": None}
    out = {"file": _relp(path, product), "error": d.get("error")}
    s = d.get("summary")
    if isinstance(s, dict):          # spark_rover
        out.update(units=s.get("units"), blocked=len(s.get("path_blocked") or []),
                   interfering=len(s.get("interference_at_seat") or []), standing=s.get("standing"))
    else:                            # desk_robot
        steps = d.get("steps") or []
        out.update(steps=len(steps), all_pass=d.get("all_pass"))
    return out


def drive_status(product: Path) -> dict:
    """release/drive/*.json, else robot.py's default result, .tmp/<product>/drive.json."""
    files = sorted((product / "release/drive").glob("*.json"))
    if not files and (ROOT / ".tmp" / product.name / "drive.json").exists():
        files = [ROOT / ".tmp" / product.name / "drive.json"]
    runs = []
    for f in files:
        d = _json(f)
        if not isinstance(d, dict):
            continue
        straight = d.get("straight") if isinstance(d.get("straight"), dict) else {}
        spin = d.get("spin") if isinstance(d.get("spin"), dict) else {}
        stop = ((d.get("phases") or {}).get("stop") or {}) if isinstance(d.get("phases"), dict) else {}
        runs.append({"file": _relp(f, product), "straight_m_s": straight.get("steady_speed_m_s", d.get("straight_speed_m_s")),
                     "spin_rad_s": spin.get("yaw_rate_rad_s", d.get("spin_yaw_rate_rad_s")),
                     "stop_mm": stop.get("moved_mm"), "complete": d.get("complete")})
    return {"runs": runs, "in_release": bool(runs) and "release" in Path(runs[0]["file"]).parts}


def status(product: Path) -> dict:
    have = {s.name: commands(product, s.name) is not None for s in STAGES}
    return {"product": product.name, "title": title(product),
            "stages": [k for k, v in have.items() if v], "lacks": [k for k, v in have.items() if not v],
            "instead": NOTES.get(product.name, {}), "made": made_status(product), "printed": print_status(product),
            "assembly": assembly_status(product), "drive": drive_status(product)}


def _lines(st: dict) -> List[str]:
    L = [f"{st['product']}: {st['title']}",
         f"  stages    {' '.join(st['stages'])}"]
    for k in st["lacks"]:
        L.append(f"  lacks     {k}" + (f" ({st['instead'][k]})" if k in st["instead"] else ""))
    m = st["made"]
    if m.get("file"):
        bits = []
        for proc, r in m["by_process"].items():
            if "cut_free" in r:
                bits.append(f"{proc} {r['cut_free']} of {r['parts']} cut free")
            else:
                bits.append(f"{proc} {r['printed']} of {r['plates']} plate printed ({r['mode']})")
        L.append(f"  made      {', '.join(bits)}   [{m['file']}" + ("" if m["in_release"] else ", not in release/") + "]")
    else:
        L.append("  made      not run")
    p = st["printed"]
    if p.get("file"):
        L.append(f"  printed   {p['parts']} parts: {p['twin']} on the twin, {p['ideal_only']} in ideal mode only, "
                 f"{p['pending']} pending ({p['superseded']} superseded drawings)   [{p['file']}]")
    a = st["assembly"]
    if a.get("file"):
        if "units" in a:
            L.append(f"  assembly  {a['units']} units, {a['blocked']} blocked, {a['interfering']} interfering at a seat, "
                     f"stands on its wheels: {'yes' if a['standing'] else 'no'}   [{a['file']}]")
        else:
            L.append(f"  assembly  {a['steps']} steps, all pass: {'yes' if a['all_pass'] else 'no'}   [{a['file']}]")
    else:
        L.append("  assembly  not run")
    runs = st["drive"]["runs"]
    if runs:
        for r in runs:
            bits = [f"straight {r['straight_m_s']} m/s", f"spin {r['spin_rad_s']} rad/s"]
            if r["stop_mm"] is not None:
                bits.append(f"stop {r['stop_mm']} mm")
            where = r["file"] + ("" if st["drive"]["in_release"] else ", not in release/")
            L.append(f"  drive     {', '.join(bits)}   [{where}]")
    else:
        L.append("  drive     not run")
    return L


# ---- running a stage -------------------------------------------------------------------------

def _take(args: List[str], flag: str, value: bool = False):
    """Remove a router flag from args; return its value (True for a bare flag), or None."""
    for i, a in enumerate(args):
        if a == flag:
            if not value:
                del args[i]
                return True
            if i + 1 >= len(args):
                raise SystemExit(f"{flag} needs a value")
            v = args[i + 1]
            del args[i:i + 2]
            return v
        if value and a.startswith(flag + "="):
            del args[i]
            return a.split("=", 1)[1]
    return None


def _py(product: Path, script: str, args: Sequence[str]) -> List[str]:
    return [sys.executable, str(product / script), *args]


def _show(cmd: Sequence[str]) -> str:
    def q(a: str) -> str:
        a = _rel(Path(a)) if (a.endswith(".py") and Path(a).is_absolute()) else a
        return f'"{a}"' if " " in a else a
    return " ".join(["python" if c == sys.executable else q(c) for c in cmd])


def plan(product: Path, stage: str, args: Sequence[str], duration=DRIVE_DURATION_S) -> List[List[str]]:
    """The commands a stage runs, user args placed on its (single) script."""
    cmds = commands(product, stage)
    if cmds is None:
        raise LookupError(stage)
    if args and len(cmds) > 1:
        raise ValueError(f"{stage} runs {len(cmds)} scripts and takes no arguments; run them directly")
    out = []
    for script, always, default in cmds:
        extra = list(always) + (list(args) if args else list(default))
        if stage == "drive" and '"--quit"' in _source(product / script) and "--quit" not in extra:
            extra.insert(0, "--quit")             # the controller ends the run once its result is written
        out.append(_py(product, script, extra))
    if stage == "drive":
        out.append([sys.executable, "-m", "omnisim", "run-headless", "<the world robot.py wrote>",
                    "--duration", str(duration)])
    return out


def _run(cmd: List[str]) -> int:
    return subprocess.run(cmd, cwd=ROOT, check=False).returncode


def _run_drive(cmds: List[List[str]]) -> int:
    """robot.py (its output shown as it comes), then run-headless on the world it says it wrote."""
    proc = subprocess.Popen(cmds[0], cwd=ROOT, stdout=subprocess.PIPE, text=True, encoding="utf-8", errors="replace")
    world = None
    for line in proc.stdout:
        sys.stdout.write(line)
        m = re.match(r"wrote (.+\.(?:omniworld|wbt))\s*$", line.strip())
        if m:
            world = m.group(1)
    if proc.wait() != 0:
        return proc.returncode
    if not world:
        print("drive: robot.py did not say which world it wrote ('wrote <path>')", file=sys.stderr)
        return 1
    headless = [c if c != "<the world robot.py wrote>" else world for c in cmds[1]]
    print(f"$ {_show(headless)}", flush=True)
    return _run(headless)


def _missing(product: Path, stage: str) -> Optional[str]:
    """What the stage needs and this Python lacks (deps.gate), named with the setup command."""
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    try:
        import deps
    except ImportError:                 # the router still works without the inventory beside it
        return None
    return deps.gate(product, stage)


def run_stage(product: Path, stage: str, args: List[str]) -> int:
    args = list(args)
    dry = bool(_take(args, "--dry-run"))
    duration = _take(args, "--duration", value=True) if stage == "drive" else None
    cmds = commands(product, stage)
    if cmds is None:
        note = NOTES.get(product.name, {}).get(stage)
        need = " + ".join(s for s, _, _ in BY_NAME[stage].commands)
        print(f"{product.name} has no {stage} stage (no {need})" + (f": {note}" if note else ""), file=sys.stderr)
        return 2
    problem = _missing(product, stage)
    if "-h" in args or "--help" in args:
        print(f"{stage}: {BY_NAME[stage].what}" + ("  [launches the engine]" if BY_NAME[stage].engine else ""))
        if problem and any(takes_args(product / s) for s, _, _ in cmds):    # their --help imports them
            print(problem, file=sys.stderr)
            return 3
        if stage == "drive":
            print(f"then: python -m omnisim run-headless <world> --duration {duration or DRIVE_DURATION_S}"
                  " (the router's --duration N sets it)")
        for script, _, _ in cmds:
            path = product / script
            print(f"\n=== {_rel(path)}")
            if takes_args(path):
                sys.stdout.flush()
                _run(_py(product, script, ["--help"]))
            else:
                print(docstring(path) + "\n\n(takes no arguments)")
        return 0
    for script, _, _ in cmds:
        if args and not takes_args(product / script):
            print(f"{script} takes no arguments (got {' '.join(args)})", file=sys.stderr)
            return 2
    if stage == "print-ideal" and "--only" not in args and not any(a.startswith("--only=") for a in args):
        plates = print_status(product).get("plates") or []
        print("print-ideal prints the plates you name: --only <plate> (repeatable)"
              + (f"; this product's plates: {', '.join(plates)}" if plates else "; `print --plan` makes the plan"),
              file=sys.stderr)
        return 2
    try:
        cmds_ = plan(product, stage, args, duration or DRIVE_DURATION_S)
    except ValueError as e:
        print(str(e), file=sys.stderr)
        return 2
    if dry:
        for c in cmds_:
            print(f"$ {_show(c)}", flush=True)
        if problem:
            print("(this stage cannot run here yet)\n" + problem, file=sys.stderr)
        return 0
    if problem:
        print(problem, file=sys.stderr)
        return 3
    if stage == "drive":
        print(f"$ {_show(cmds_[0])}", flush=True)
        return _run_drive(cmds_)
    for c in cmds_:
        print(f"$ {_show(c)}", flush=True)
        rc = _run(c)
        if rc != 0:
            return rc
    return 0


def run_all(product: Path, args: List[str]) -> int:
    args = list(args)
    dry = bool(_take(args, "--dry-run"))
    keep = bool(_take(args, "--keep-going"))
    if args:
        print(f"all takes only --dry-run and --keep-going (got {' '.join(args)}); "
              "give a stage its arguments by running it on its own", file=sys.stderr)
        return 2
    rows, failed = [], False
    for stage in ALL_ORDER:
        if commands(product, stage) is None:
            rows.append((stage, "skipped", "-", NOTES.get(product.name, {}).get(stage, "no script")))
            continue
        if failed and not keep:
            rows.append((stage, "not run", "-", "an earlier stage failed"))
            continue
        print(f"\n== {product.name} {stage}: {BY_NAME[stage].what}", flush=True)
        t = time.time()
        rc = run_stage(product, stage, ["--dry-run"] if dry else [])
        rows.append((stage, "planned" if dry else ("ok" if rc == 0 else f"exit {rc}"),
                     "-" if dry else f"{time.time() - t:.0f} s", ""))
        failed = failed or rc != 0
    print(f"\n{product.name}: all stages")
    for stage, state, wall, note in rows:
        print(f"  {stage:<10} {state:<9} {wall:>7}  {note}")
    return 1 if failed else 0


# ---- entry --------------------------------------------------------------------------------------

def _describe(product: Path) -> int:
    print(f"{product.name}: {title(product)}   [{_rel(product)}]")
    for s in STAGES:
        cmds = commands(product, s.name)
        if cmds is None:
            note = NOTES.get(product.name, {}).get(s.name)
            print(f"  {s.name:<12} -- lacks " + " + ".join(c for c, _, _ in s.commands) + (f" ({note})" if note else ""))
            continue
        shown = "; ".join(" ".join(cmd[2:4] + ["<world>"] + cmd[5:]) if cmd[1] == "-m"
                          else " ".join([Path(cmd[1]).name, *cmd[2:]]) for cmd in plan(product, s.name, ()))
        print(f"  {s.name:<12} {shown}" + ("   [engine]" if s.engine else ""))
    print(f"\npython -m omnisim manufacturing product {product.name} <stage> --help")
    return 0


def main(argv=None, products_dir: Optional[Path] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    pdir = Path(products_dir) if products_dir else PRODUCTS
    if not argv or argv[0] in ("-h", "--help", "help"):
        print(__doc__)
        return 0
    head, rest = argv[0], argv[1:]
    found = {p.name: p for p in products(pdir)}
    if head == "products":
        rows = [status(p) for p in found.values()]
        if "--json" in rest:
            print(json.dumps(rows, indent=1))
            return 0
        print("Simulation only: every result below is a twin's or Newton's, read from the product's own files.\n")
        for st in rows:
            print("\n".join(_lines(st)) + "\n")
        rest_ = others(pdir)
        if rest_:
            print(f"also in products/, with none of the stage scripts: {', '.join(rest_)} (see each README)")
        return 0
    if head != "product":
        print(f"unknown: {head!r}", file=sys.stderr)
        return 2
    if not rest or rest[0] in ("-h", "--help"):
        print(__doc__)
        return 0
    name = rest[0]
    if name not in found:
        print(f"no product {name!r}; products: {', '.join(found)}", file=sys.stderr)
        return 2
    product = found[name]
    if len(rest) == 1:
        return _describe(product)
    stage, args = rest[1], rest[2:]
    if stage == "all":
        return run_all(product, args)
    if stage not in BY_NAME:
        print(f"unknown stage {stage!r}; stages: {', '.join(BY_NAME)}, all", file=sys.stderr)
        return 2
    return run_stage(product, stage, args)


if __name__ == "__main__":
    raise SystemExit(main())
