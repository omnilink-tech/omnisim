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

"""Slice an STL for a twin printer with an installed CuraEngine, no Cura GUI needed.

CuraEngine on its own does not evaluate Cura's setting formulas (`"value": "..."` in
fdmprinter.def.json): the GUI does, then hands the engine every setting. Called bare it
writes an empty file and logs "Trying to retrieve setting with no value given". This
does the GUI's part: it reads the definitions shipped with the installed Cura, applies
the printer profile's settings (../machines/<name>/<name>.json, "slicer"), evaluates every
formula to a fixed point and passes the lot on the command line.

The vendor's slicer profile is usually an OrcaSlicer one; the profile records how its
values were carried over. One rule OrcaSlicer applies and Cura does not is the hotend's
volumetric limit, so a print speed that would need more flow than the hotend melts is
capped here and the cap is reported.

    python cura_slice.py part.stl --printer ../machines/sovol_sv08/sovol_sv08.json --out part.gcode
    python cura_slice.py part.stl --set infill_sparse_density=20 --at 120,80 --out part.gcode

The part is placed with its footprint centred on the bed, or on --at x,y (bed mm).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import printer as printer_mod  # noqa: E402

CURA_GLOBS = ["C:/Program Files/UltiMaker Cura*", "C:/Program Files/Ultimaker Cura*", "/usr/share/cura", "/opt/cura*"]
MAX_PASSES = 50
# (speed key, line width key) pairs the volumetric cap applies to.
FLOW_LIMITED = [("speed_wall_0", "wall_line_width_0"), ("speed_wall_x", "wall_line_width_x"),
                ("speed_infill", "infill_line_width"), ("speed_topbottom", "skin_line_width"),
                ("speed_roofing", "skin_line_width"), ("speed_print", "line_width")]


def find_cura(explicit: Optional[str] = None) -> Path:
    """The Cura install: CuraEngine plus its definitions."""
    candidates = [Path(explicit)] if explicit else []
    if os.environ.get("CURA_HOME"):
        candidates.append(Path(os.environ["CURA_HOME"]))
    for pattern in CURA_GLOBS:
        root, _, glob = pattern.rpartition("/")
        if Path(root).is_dir():
            candidates += sorted(Path(root).glob(glob), reverse=True)
    for c in candidates:
        exe = c / ("CuraEngine.exe" if os.name == "nt" else "CuraEngine")
        if exe.exists() and (c / "share/cura/resources/definitions/fdmprinter.def.json").exists():
            return c
    raise SystemExit("cura_slice: no Cura install found (CuraEngine + share/cura/resources); pass --cura or set CURA_HOME")


class Settings:
    """Cura's setting tree, flattened: default values, formulas and types."""

    def __init__(self, definitions: List[Path]):
        self.default: Dict[str, object] = {}
        self.formula: Dict[str, str] = {}
        self.kind: Dict[str, str] = {}
        for path in definitions:
            self._walk(json.loads(path.read_text(encoding="utf-8")).get("settings", {}))

    def _walk(self, node: dict) -> None:
        for key, spec in node.items():
            if not isinstance(spec, dict):
                continue
            if "type" in spec and spec["type"] != "category":
                self.kind[key] = spec["type"]
                if "default_value" in spec:
                    self.default[key] = spec["default_value"]
                if "value" in spec:
                    self.formula[key] = str(spec["value"])
            self._walk(spec.get("children", {}))

    def resolve(self, fixed: Dict[str, object]) -> Dict[str, object]:
        """Evaluate every formula against the others until nothing changes."""
        values = dict(self.default)
        values.update(fixed)
        for key in self.formula:
            values.setdefault(key, None)

        def get(k):
            return values.get(k)

        env = {
            "math": math, "max": max, "min": min, "sum": sum, "any": any, "all": all, "map": map, "round": round,
            "int": int, "float": float, "abs": abs, "len": len, "str": str, "list": list,
            "ceil": math.ceil, "floor": math.floor, "tan": math.tan, "radians": math.radians, "sqrt": math.sqrt,
            "extruderValue": lambda e, k: get(k), "extruderValues": lambda k: [get(k)],
            "resolveOrValue": get, "defaultExtruderPosition": lambda: "0",
            "anyExtruderWithMaterial": lambda k: "0", "anyExtruderNrWithOrDefault": lambda k: "0",
            "valueFromContainer": lambda i: None, "valueFromExtruderContainer": lambda e, i: None,
        }
        failed: Dict[str, str] = {}
        for _ in range(MAX_PASSES):
            changed = False
            for key, expr in self.formula.items():
                if key in fixed:
                    continue
                scope = _Scope(values, env)
                try:
                    v = eval(compile(expr, key, "eval"), {"__builtins__": {}}, scope)
                except Exception as exc:          # a formula over a value not resolved yet
                    failed[key] = f"{type(exc).__name__}: {exc}"
                    continue
                failed.pop(key, None)
                if v != values.get(key):
                    values[key] = v
                    changed = True
            if not changed:
                break
        self.failed = failed
        return values

    def format(self, key: str, v) -> str:
        if isinstance(v, bool):
            return "true" if v else "false"
        if v is None:
            return ""
        if isinstance(v, float):
            return repr(round(v, 6))
        if isinstance(v, (list, tuple)):
            return json.dumps(v)
        return str(v)


class _Scope(dict):
    """Names in a formula: helpers first, then settings by key."""

    def __init__(self, values, env):
        super().__init__()
        self.values, self.env = values, env

    def __getitem__(self, name):
        if name in self.env:
            return self.env[name]
        if name in self.values:
            return self.values[name]
        raise KeyError(name)


def flow_capped(cura: dict, max_mm3_s: float, layer_h: float) -> Dict[str, tuple]:
    """Cap each print speed at the hotend's volumetric limit; returns {key: (asked, capped)}."""
    caps = {}
    for speed_key, width_key in FLOW_LIMITED:
        if speed_key not in cura:
            continue
        width = float(cura.get(width_key, cura.get("line_width", 0.4)))
        limit = max_mm3_s / (width * layer_h)
        if float(cura[speed_key]) > limit:
            caps[speed_key] = (float(cura[speed_key]), round(limit, 1))
            cura[speed_key] = round(limit, 1)
    return caps


def build_gcode(lines: List[str], values: Dict[str, object]) -> str:
    """The profile's start/end G-code with {setting} replaced, as the Cura GUI does."""
    def sub(m):
        v = values.get(m.group(1))
        return m.group(0) if v is None else (f"{v:g}" if isinstance(v, (int, float)) else str(v))
    return "\n".join(re.sub(r"\{(\w+)\}", sub, line) for line in lines)


def slice_stl(stl: Path, prof: dict, out: Path, cura_home: Optional[str] = None,
              overrides: Optional[Dict[str, str]] = None, at: Optional[List[float]] = None) -> dict:
    home = find_cura(cura_home)
    defs = home / "share/cura/resources/definitions"
    settings = Settings([defs / "fdmprinter.def.json", defs / "fdmextruder.def.json"])
    slicer = prof["slicer"]
    cura = dict(slicer["cura"])
    for k, v in (overrides or {}).items():
        cura[k] = _typed(settings.kind.get(k, "str"), v)
    # CuraEngine centres the part on the bed (center_object) and then shifts it by
    # mesh_position_x/y; a bare STL's own coordinates are read about the bed centre.
    cura["center_object"] = True
    if at is not None:
        cura["mesh_position_x"] = at[0] - 0.5 * float(cura["machine_width"])
        cura["mesh_position_y"] = at[1] - 0.5 * float(cura["machine_depth"])
    caps = flow_capped(cura, float(prof["hotend"]["max_volumetric_mm3_s"]), float(cura.get("layer_height", 0.2)))
    values = settings.resolve(cura)
    values["machine_start_gcode"] = build_gcode(slicer["start_gcode"], values)
    values["machine_end_gcode"] = build_gcode(slicer["end_gcode"], values)
    unresolved = sorted(k for k in settings.failed if values.get(k) is None)
    args = [str(home / ("CuraEngine.exe" if os.name == "nt" else "CuraEngine")), "slice",
            "-j", str(defs / "fdmprinter.def.json"), "-o", str(out.resolve())]
    for k in sorted(values):
        if values[k] is not None and k in settings.kind:
            args += ["-s", f"{k}={settings.format(k, values[k])}"]
    # Extruder 0 keeps its OWN copy of every setting fdmextruder.def.json defines, defaulted
    # from that file and not inherited from the global ones: given only globally,
    # material_diameter stayed 2.85 mm and every E value was computed for the wrong
    # filament (the twin printed 38 % of the plastic). So those are given again here.
    args += ["-e0"]
    for k in sorted(Settings([defs / "fdmextruder.def.json"]).kind):
        if values.get(k) is not None:
            args += ["-s", f"{k}={settings.format(k, values[k])}"]
    args += ["-l", str(stl.resolve())]
    run = subprocess.run(args, capture_output=True, text=True, errors="replace")
    errors = [ln for ln in (run.stdout + run.stderr).splitlines() if "[error]" in ln]
    if run.returncode != 0 or not out.exists() or out.stat().st_size == 0:
        raise SystemExit("cura_slice: CuraEngine produced no G-code:\n" + "\n".join(errors[:20] or [run.stderr[-2000:]]))
    # CuraEngine names the mesh by its absolute path in every ;MESH: comment; keep the name only.
    text = out.read_text(encoding="utf-8", errors="replace")
    out.write_text(text.replace(str(stl.resolve()), stl.name), encoding="utf-8")
    return {"gcode": str(out), "cura": str(home), "speed_caps": caps, "unresolved": unresolved,
            "engine_errors": sorted(set(errors))[:20]}


def _typed(kind: str, text: str):
    if kind == "bool":
        return text.lower() in ("1", "true", "yes")
    if kind in ("float",):
        return float(text)
    if kind in ("int", "extruder", "optional_extruder"):
        return int(float(text))
    return text


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("stl")
    ap.add_argument("--printer", default=str(printer_mod.DEFAULT_PROFILE))
    ap.add_argument("--out", required=True)
    ap.add_argument("--cura", help="Cura install directory (default: search the usual places)")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="override a Cura setting")
    ap.add_argument("--at", help="x,y: where the part's footprint centre goes on the bed (mm; default the centre)")
    args = ap.parse_args(argv)
    out = Path(args.out)
    if out.exists():
        raise SystemExit(f"cura_slice: {out} exists; pick a new name, this tool never overwrites")
    prof = printer_mod.load(args.printer).profile
    overrides = dict(s.split("=", 1) for s in args.set)
    at = [float(v) for v in args.at.split(",")] if args.at else None
    report = slice_stl(Path(args.stl), prof, out, args.cura, overrides, at)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
