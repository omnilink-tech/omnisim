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

"""OmniSim Manufacturing: what each stage needs beyond the repository, and installing it.

    python -m omnisim manufacturing doctor [--json] [--import]   per stage: can it run here, what is missing
    python -m omnisim manufacturing setup [--yes] [--dry-run]    pip-install requirements.txt, then the table

A stage needs up to three things: Python packages (requirements.txt, installed into the
Python that runs `python -m omnisim`), CuraEngine (an UltiMaker Cura install, for slicing),
and the OmniSim engine (the twins and Newton). `setup` installs only the Python packages,
after showing exactly what pip will install; it never downloads Cura or the engine.

`doctor` checks packages with importlib's finder, which is fast and imports nothing;
`--import` imports each one too, which also catches a package that is present but broken
(a missing DLL); that took 17 s on the 2026-11-04 venv.

The per-stage tables (PRODUCT_NEEDS) was measured on 2026-11-04 by tracing every import that
code under projects/manufacturing makes while each stage's scripts start (and, for `check`,
while it runs to the end), plus the function-level imports of every module they load, then
confirmed on an empty Python 3.12 venv. tests/test_manufacturing_setup.py checks that every
third-party import under projects/manufacturing is in PACKAGES and requirements.txt.
"""
from __future__ import annotations

import importlib
import importlib.metadata
import importlib.util
import json
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
REQUIREMENTS = HERE / "requirements.txt"
SETUP_COMMAND = "python -m omnisim manufacturing setup"
CURA_TESTED = "5.3.0"
PYTHON_TESTED = (3, 12)         # the pins install from wheels here; open3d==0.19.0 has no 3.13 wheel
CURA_SOURCE = ("UltiMaker Cura 5.x (tested: 5.3.0) from https://ultimaker.com/software/ultimaker-cura/ "
               "(releases: https://github.com/Ultimaker/Cura/releases). cura_slice.py finds it under "
               "C:/Program Files/UltiMaker Cura*, /usr/share/cura or /opt/cura*, or wherever CURA_HOME points "
               "(the folder that holds CuraEngine and share/cura/resources)")


@dataclass(frozen=True)
class Package:
    module: str                 # the import name
    dist: str                   # the pip name, as requirements.txt spells it
    optional: bool = False      # its absence only slows a stage down, or a fallback takes over
    note: str = ""              # what happens without it (optional) or why it is needed


PACKAGES: Dict[str, Package] = {p.module: p for p in (
    Package("numpy", "numpy"),
    Package("shapely", "shapely"),
    Package("trimesh", "trimesh"),
    Package("fcl", "python-fcl", note="trimesh's CollisionManager, the FCL interference check"),
    Package("rtree", "rtree", note="trimesh's triangle tree"),
    Package("networkx", "networkx", note="trimesh's path graph"),
    Package("scipy", "scipy"),
    Package("OCP", "cadquery-ocp", note="OpenCASCADE: SparkRover's B-rep shell and electronics"),
    Package("matplotlib", "matplotlib"),
    Package("PIL", "Pillow"),
    Package("psutil", "psutil"),
    Package("open3d", "open3d"),
    Package("svgelements", "svgelements"),
    Package("cascadio", "cascadio", note="STEP to meshes"),
    Package("numba", "numba", optional=True,
            note="the wire router and ideal_print then run as plain Python, much slower"),
    Package("yaml", "PyYAML", optional=True, note="cnc/machine.py then parses FluidNC YAML itself"),
)}

# Imported somewhere under projects/manufacturing, but not from pip: the repository's own
# packages (on the path from the checkout), and the engine's controller library.
NOT_FROM_PIP = frozenset({"omnisim", "omnisim_bridges", "_chat_page", "env_fingerprint"})

# Every stage that touches a mesh: trimesh, and what trimesh itself imports for the calls
# these scripts make. Measured in an empty venv: without rtree, integrate.py fails in
# shell.wall_thickness (trimesh's ray tests); without networkx, a section's polygons_full
# fails (inspect_print, make_print's layer cuts); fcl is trimesh's CollisionManager.
_CORE = ("numpy", "shapely", "trimesh", "fcl", "scipy", "rtree", "networkx")
# SparkRover, each stage's scripts as product_cli.STAGES names them. integrate.load() imports
# the four subsystems (shell and electrical need OCP) in every stage but print's own scripts
# reach them through it too. The table errs toward listing: a package here that a particular
# run does not import costs one line of requirements.txt, a package missing from it costs a
# traceback. A product with no entry here (your own) is read from its scripts (scan_needs).
SPARK_NEEDS: Dict[str, Tuple[str, ...]] = {
    "check": _CORE + ("OCP",),
    "wire": _CORE + ("OCP", "numba"),
    "make": _CORE + ("OCP",),
    "print": _CORE + ("OCP", "psutil"),
    "print-ideal": _CORE + ("OCP", "psutil", "numba"),
    "assemble": _CORE + ("OCP", "open3d", "PIL"),
    "drive": _CORE + ("OCP",),
    "release": _CORE + ("OCP", "matplotlib"),
}
_DESK = _CORE + ("matplotlib", "svgelements", "numba")     # design.py's drawings and wordmark, wiring.py's JIT
PRODUCT_NEEDS: Dict[str, Dict[str, Tuple[str, ...]]] = {
    "spark_rover": SPARK_NEEDS,
    "desk_robot": {"check": _DESK, "wire": _DESK, "make": _DESK, "assemble": _DESK + ("PIL",),
                   "drive": _DESK, "release": _DESK},
}

# Stages that slice with CuraEngine. A path: the plan the stage slices into; with that file
# present the stage reuses it and Cura is not needed, without it Cura is required. None: the
# stage slices only for some of its arguments (Cura is reported, not required).
CURA_NEEDS: Dict[str, Dict[str, Optional[str]]] = {
    "spark_rover": {"print": ".tmp/spark_print/plan.json", "print-ideal": ".tmp/spark_print/plan.json"},
    "desk_robot": {"make": None},       # make --only print slices the tray plate
}
DEFAULT_CURA_NEEDS: Dict[str, Optional[str]] = {"print": None, "print-ideal": None}
PRODUCTS = HERE / "products"

# python -m omnisim manufacturing <process> <tool>: what each tool's script imports.
TOOL_NEEDS: Dict[Tuple[str, str], Tuple[str, ...]] = {
    ("cnc", "preflight"): (), ("cnc", "world"): (), ("cnc", "server"): (),
    ("cnc", "inspect"): ("numpy", "scipy", "shapely"), ("cnc", "toolpath"): ("shapely",),
    ("cnc", "toolpath3d"): ("numpy", "scipy"), ("cnc", "parts"): ("numpy", "scipy"),
    ("cnc", "bridge"): ("shapely",),
    ("laser", "kerf"): (), ("laser", "job"): ("shapely",), ("laser", "world"): (), ("laser", "run"): (),
    ("laser", "inspect"): ("numpy", "scipy", "shapely"),
    ("printing", "slice"): (), ("printing", "preflight"): ("numpy",), ("printing", "world"): (),
    ("printing", "inspect"): ("numpy", "shapely", "trimesh", "rtree", "networkx"), ("printing", "printer"): (),
    ("cad-pack", ""): ("numpy", "trimesh", "cascadio", "scipy"),
}
TOOL_CURA = {("printing", "slice")}
ENGINE_TOOLS = {("laser", "run")}


# ---- requirements.txt ---------------------------------------------------------------------------

def requirements(path: Path = REQUIREMENTS) -> Dict[str, str]:
    """{pip name: pinned version} from requirements.txt, in file order."""
    out: Dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        m = re.fullmatch(r"([A-Za-z0-9_.\-]+)\s*==\s*([A-Za-z0-9_.+\-]+)", line)
        if not m:
            raise ValueError(f"{path.name}: not a `name==version` pin: {line!r}")
        out[m.group(1)] = m.group(2)
    return out


def pin(module: str) -> str:
    dist = PACKAGES[module].dist
    v = requirements().get(dist)
    return f"{dist}=={v}" if v else dist


def _imports(path: Path) -> List[str]:
    import ast
    try:
        tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
    except (OSError, SyntaxError):
        return []
    names = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names += [a.name.split(".")[0] for a in node.names]
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            names.append(node.module.split(".")[0])
    return names


def scan_needs(product_dir: Path, scripts: Sequence[str]) -> Tuple[str, ...]:
    """The PACKAGES a product's scripts import, following its own modules (a sibling .py of
    that name) at any depth. For a product with no PRODUCT_NEEDS entry: yours."""
    found: List[str] = []
    todo, seen = [product_dir / s for s in scripts], set()
    while todo:
        path = todo.pop()
        if path in seen or not path.is_file():
            continue
        seen.add(path)
        src = path.read_text(encoding="utf-8", errors="replace")
        for name in _imports(path):
            if (product_dir / f"{name}.py").is_file():
                todo.append(product_dir / f"{name}.py")
            elif name in PACKAGES and name not in found:
                found.append(name)
        if "CollisionManager" in src and "fcl" not in found:
            found.append("fcl")
    if "trimesh" in found:                       # trimesh's own optional parts that our calls reach
        found += [m for m in ("scipy", "rtree", "networkx") if m not in found]
    return tuple(found)


def needs(product, stage: str) -> Tuple[str, ...]:
    """product: a name from PRODUCT_NEEDS, or a product folder (scanned unless it is one of
    the products/ folders PRODUCT_NEEDS describes)."""
    path = Path(product) if not isinstance(product, str) else PRODUCTS / product
    if path.parent == PRODUCTS and path.name in PRODUCT_NEEDS:
        return PRODUCT_NEEDS[path.name].get(stage, ())
    import product_cli
    cmds = product_cli.commands(path, stage) if path.is_dir() else None
    return scan_needs(path, [c[0] for c in cmds]) if cmds else ()


# ---- what is here -------------------------------------------------------------------------------

def present(module: str, do_import: bool = False) -> bool:
    try:
        if importlib.util.find_spec(module) is None:
            return False
    except (ImportError, ValueError):
        return False
    if do_import:
        try:
            importlib.import_module(module)
        except Exception:       # a DLL that does not load, a version clash: present but unusable
            return False
    return True


def installed_version(dist: str) -> Optional[str]:
    try:
        return importlib.metadata.version(dist)
    except importlib.metadata.PackageNotFoundError:
        return None


def package_rows(do_import: bool = False) -> List[dict]:
    pins = requirements()
    rows = []
    for p in PACKAGES.values():
        ok = present(p.module, do_import)
        rows.append({"module": p.module, "dist": p.dist, "pinned": pins.get(p.dist),
                     "installed": installed_version(p.dist) if ok else None, "ok": ok,
                     "optional": p.optional, "note": p.note})
    return rows


def cura_status() -> dict:
    """Where cura_slice.py would find CuraEngine, or None (cura_slice.find_cura's own search)."""
    path = HERE / "printing" / "cura_slice.py"
    spec = importlib.util.spec_from_file_location("_mfg_cura_slice_probe", path)
    mod = importlib.util.module_from_spec(spec)
    saved = list(sys.path)                      # cura_slice puts printing/ first on the path
    try:
        spec.loader.exec_module(mod)
        home = mod.find_cura()
    except SystemExit:
        return {"ok": False, "path": None, "version": None}
    except Exception as e:                      # cura_slice itself did not load
        return {"ok": False, "path": None, "version": None, "error": f"{type(e).__name__}: {e}"}
    finally:
        sys.path[:] = saved
    m = re.search(r"(\d+\.\d+(?:\.\d+)?)", Path(home).name)
    return {"ok": True, "path": str(home), "version": m.group(1) if m else None}


def engine_status() -> dict:
    if str(ROOT) not in sys.path:
        sys.path.append(str(ROOT))
    try:
        from omnisim.paths import resolve_omnisim_binary
        binary = resolve_omnisim_binary()
    except Exception:
        binary = None
    return {"ok": bool(binary), "path": binary}


# ---- per stage ----------------------------------------------------------------------------------

def _stage_engine(stage: str) -> bool:
    import product_cli
    return product_cli.BY_NAME[stage].engine


def readiness(product, stage: str, *, have: Optional[Dict[str, bool]] = None,
              cura: Optional[dict] = None, engine: Optional[dict] = None) -> dict:
    """Can this product's stage run here: the required packages missing, the optional ones
    missing (slower), and CuraEngine and the engine when the stage uses them. product: a
    name, or the product's folder."""
    mods = needs(product, stage)
    product = Path(product).name
    have = have if have is not None else {m: present(m) for m in mods}
    missing = [m for m in mods if not have.get(m) and not PACKAGES[m].optional]
    slower = [m for m in mods if not have.get(m) and PACKAGES[m].optional]
    out = {"product": product, "stage": stage, "missing": missing, "slower": slower,
           "cura": None, "engine": None}
    cura_rule = CURA_NEEDS.get(product, DEFAULT_CURA_NEEDS)
    if stage in cura_rule:
        cura = cura if cura is not None else cura_status()
        plan = cura_rule[stage]
        if cura["ok"]:
            out["cura"] = "ok"
        elif plan and (ROOT / plan).exists():
            out["cura"] = f"not needed ({plan} exists)"
        else:
            out["cura"] = "missing" if plan else "missing (needed when it slices)"
    if _stage_engine(stage):
        engine = engine if engine is not None else engine_status()
        out["engine"] = "ok" if engine["ok"] else "missing"
    out["can_run"] = not missing and out["cura"] != "missing" and out["engine"] != "missing"
    return out


def gate(product, stage: str) -> Optional[str]:
    """The router's check before it runs a stage: a message naming what is missing, or None.
    Blocks on missing packages, and on CuraEngine when the stage must slice. The engine is
    named but not blocked on: `make --plan` and `print --plan` run without one."""
    r = readiness(product, stage, engine={"ok": True, "path": None})
    product = r["product"]
    lines = []
    if r["missing"]:
        lines.append(f"{product} {stage} needs Python packages that are not installed in {sys.executable}:")
        lines += [f"    {m:<12} pip: {pin(m)}" for m in r["missing"]]
        lines.append(f"  install them (it shows the list and asks first):  {SETUP_COMMAND}")
    if r["cura"] == "missing":
        lines.append(f"{product} {stage} slices with CuraEngine, and none was found.")
        lines.append(f"  get {CURA_SOURCE}.")
    if lines:
        lines.append("  check every stage:  python -m omnisim manufacturing doctor")
    return "\n".join(lines) or None


def tool_gate(process: str, tool: str) -> Optional[str]:
    mods = TOOL_NEEDS.get((process, tool), ())
    missing = [m for m in mods if not present(m) and not PACKAGES[m].optional]
    if not missing:
        return None
    name = f"{process} {tool}".strip()
    return "\n".join([f"manufacturing {name} needs Python packages that are not installed in {sys.executable}:",
                      *[f"    {m:<12} pip: {pin(m)}" for m in missing],
                      f"  install them (it shows the list and asks first):  {SETUP_COMMAND}"])


# ---- doctor --------------------------------------------------------------------------------------

def report(do_import: bool = False) -> dict:
    import product_cli
    pkgs = package_rows(do_import)
    have = {r["module"]: r["ok"] for r in pkgs}
    cura, engine = cura_status(), engine_status()
    prods = []
    for p in product_cli.products():
        rows = [readiness(p, s.name, have=have, cura=cura, engine=engine)
                for s in product_cli.STAGES if product_cli.commands(p, s.name) is not None]
        prods.append({"product": p.name, "stages": rows})
    tools = []
    for (process, tool), mods in TOOL_NEEDS.items():
        missing = [m for m in mods if not have.get(m) and not PACKAGES[m].optional]
        c = None if (process, tool) not in TOOL_CURA else ("ok" if cura["ok"] else "missing")
        e = None if (process, tool) not in ENGINE_TOOLS else ("ok" if engine["ok"] else "missing")
        tools.append({"tool": f"{process} {tool}".strip(), "missing": missing, "cura": c, "engine": e,
                      "can_run": not missing and c != "missing" and e != "missing"})
    return {"python": sys.executable, "python_version": sys.version.split()[0], "packages": pkgs,
            "cura": cura, "engine": engine, "products": prods, "tools": tools,
            "others": product_cli.others()}


def _what(r: dict) -> str:
    bits = []
    if r["missing"]:
        bits.append("needs " + ", ".join(r["missing"]))
    if r.get("cura") == "missing":
        bits.append("needs CuraEngine")
    if r.get("engine") == "missing":
        bits.append("needs the engine")
    if r.get("slower"):
        bits.append("slower without " + ", ".join(r["slower"]))
    if r.get("cura") and r["cura"].startswith(("missing (", "not needed")):
        bits.append("CuraEngine " + r["cura"])
    return "; ".join(bits)


def print_table(rep: dict) -> None:
    print(f"Python  {rep['python_version']}  {rep['python']}")
    print(f"        packages from {REQUIREMENTS.relative_to(ROOT).as_posix()}:")
    for r in rep["packages"]:
        if r["ok"]:
            state = r["installed"] or "?"
            if r["pinned"] and r["installed"] and r["installed"] != r["pinned"]:
                state += f" (pinned {r['pinned']})"
        else:
            state = "missing, optional: " + r["note"] if r["optional"] else f"MISSING  ({r['dist']}=={r['pinned']})"
        print(f"          {r['module']:<12} {r['dist']:<14} {state}")
    c = rep["cura"]
    if c["ok"]:
        tested = "" if c["version"] == CURA_TESTED else f" (tested with {CURA_TESTED})"
        print(f"CuraEngine  {c['version'] or '?'}  {c['path']}{tested}")
    else:
        print(f"CuraEngine  NOT FOUND: get {CURA_SOURCE}")
    e = rep["engine"]
    print(f"Engine  {e['path']}" if e["ok"] else "Engine  NOT FOUND: python -m omnisim doctor says how to build or install it")
    for p in rep["products"]:
        print(f"\n{p['product']:<14} engine   ready  what is missing")
        for r in p["stages"]:
            eng = "yes" if r["engine"] else "no"
            print(f"  {r['stage']:<12} {eng:<8} {'yes' if r['can_run'] else 'NO':<6} {_what(r)}")
    print(f"\n{'tools':<14} engine   ready  what is missing")
    for t in rep["tools"]:
        print(f"  {t['tool']:<20} {'yes' if t['engine'] else 'no':<8} {'yes' if t['can_run'] else 'NO':<6} {_what(t)}")
    if rep["others"]:
        print(f"\nalso in products/, run by their own scripts: {', '.join(rep['others'])} (see each README)")


def doctor(argv: Sequence[str] = ()) -> int:
    argv = list(argv)
    if "-h" in argv or "--help" in argv:
        print(__doc__)
        return 0
    rep = report(do_import="--import" in argv)
    if "--json" in argv:
        print(json.dumps(rep, indent=1))
    else:
        print("OmniSim Manufacturing doctor: what each stage needs, and whether this machine has it.\n")
        print_table(rep)
        if any(not r["ok"] and not r["optional"] for r in rep["packages"]):
            print(f"\nInstall the missing packages:  {SETUP_COMMAND}")
    blocked = [r for p in rep["products"] for r in p["stages"] if not r["can_run"]]
    return 1 if blocked or any(not t["can_run"] for t in rep["tools"]) else 0


# ---- setup ---------------------------------------------------------------------------------------

def _pip(*args: str) -> List[str]:
    return [sys.executable, "-m", "pip", *args]


def pip_plan() -> Optional[List[dict]]:
    """What `pip install -r requirements.txt` would install, dependencies included, from pip's
    own resolver (`--dry-run --report`). None when pip cannot say (offline, an old pip)."""
    with tempfile.TemporaryDirectory() as tmp:
        rep = Path(tmp) / "report.json"
        run = subprocess.run(_pip("install", "--dry-run", "--quiet", "--disable-pip-version-check",
                                  "--report", str(rep), "-r", str(REQUIREMENTS)),
                             capture_output=True, text=True)
        if run.returncode != 0 or not rep.exists():
            return None
        data = json.loads(rep.read_text(encoding="utf-8"))
    out = []
    for item in data.get("install", []):
        meta = item.get("metadata", {})
        name = meta.get("name", "?")
        out.append({"name": name, "version": meta.get("version", "?"), "requested": bool(item.get("requested")),
                    "installed": installed_version(name)})
    return out


def setup(argv: Sequence[str] = ()) -> int:
    argv = list(argv)
    if "-h" in argv or "--help" in argv:
        print(__doc__)
        return 0
    yes, dry = "--yes" in argv or "-y" in argv, "--dry-run" in argv
    pins = requirements()
    print("OmniSim Manufacturing setup")
    print(f"  into     {sys.executable} (Python {sys.version.split()[0]})"
          + ("" if sys.prefix != sys.base_prefix else "; not a venv: pip installs into this Python itself"))
    print(f"  from     {REQUIREMENTS.relative_to(ROOT).as_posix()} ({len(pins)} pinned packages)")
    if sys.version_info[:2] != PYTHON_TESTED:
        print(f"  note     the pins were measured on CPython {PYTHON_TESTED[0]}.{PYTHON_TESTED[1]}; on another version "
              "a pin may have no wheel (open3d==0.19.0 has none for 3.13), and pip then fails on it")
    print("  resolving with pip (--dry-run) ...", flush=True)
    plan = pip_plan()
    if plan is None:
        todo = [(d, v, installed_version(d)) for d, v in pins.items() if installed_version(d) != v]
        print("  pip could not resolve the plan (offline?); the pins themselves:")
        for d, v, have in todo:
            print(f"    {d}=={v}" + (f"   (replaces {have})" if have else ""))
        n = len(todo)
    else:
        n = len(plan)
        if plan:
            print(f"  pip will install {n} package(s); * = pinned in requirements.txt, the rest are their dependencies:")
            for it in plan:
                was = f"   (replaces {it['installed']})" if it["installed"] else ""
                print(f"    {'*' if it['requested'] else ' '} {it['name']}=={it['version']}{was}")
    if n == 0:
        print("  every pinned package is already installed")
    elif dry:
        print("  --dry-run: nothing installed")
    else:
        if not yes:
            if not sys.stdin or not sys.stdin.isatty():
                print(f"  not installing: no terminal to ask. Re-run with --yes:  {SETUP_COMMAND} --yes")
                return 2
            if input(f"  install these {n} package(s) with pip? [y/N] ").strip().lower() not in ("y", "yes"):
                print("  nothing installed")
                return 2
        cmd = _pip("install", "--disable-pip-version-check", "-r", str(REQUIREMENTS))
        print("$ " + " ".join(["python" if c == sys.executable else c for c in cmd]), flush=True)
        rc = subprocess.run(cmd).returncode
        if rc != 0:
            print(f"pip failed (exit {rc}); nothing below is changed by this run", file=sys.stderr)
            return rc
        importlib.invalidate_caches()
    c = cura_status()
    print()
    if c["ok"]:
        print(f"CuraEngine: found {c['path']}" + ("" if c["version"] == CURA_TESTED else f" (tested with {CURA_TESTED})"))
    else:
        print("CuraEngine: not found. setup does not download it. `print`, `print-ideal` (until a plan exists) and "
              f"`printing slice` need it; get {CURA_SOURCE}.")
    print()
    print_table(report())
    return 0
