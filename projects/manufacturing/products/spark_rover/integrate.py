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

"""SparkRover Mini: join the subsystems and check them against each other.

Each subsystem (shell, chassis, arms, electrical) is designed on its own against
skeleton.py. Here they meet:

- zones: every solid of a subsystem inside that subsystem's zones (the shell inside
  the envelope instead), by its bounding box;
- interference: every solid against every other, across subsystems and within them,
  with FCL; a pair that meets is TOUCHING (a 0.05 mm nudge along some axis frees it:
  a face on its seat), THREADED (a screw in its nut, insert or tapped hole) or
  INTERFERENCE (they occupy the same space): the desk robot's method (its
  bought_parts.clearance);
- mounts: every interface named in skeleton.py provided by one side and used by the
  other;
- mass: the whole robot's mass and centre of mass against skeleton.MASS_TARGET_G.

    python integrate.py            # prints the report; non-zero exit on interference
"""
from __future__ import annotations

import importlib
import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

import skeleton

HERE = Path(__file__).resolve().parent
SUBSYSTEMS = ("shell", "chassis", "arms", "electrical")
NUDGE_MM = 0.05
THREADED_HINTS = ("screw", "bolt", "insert", "nut", "standoff")


def load() -> Dict[str, object]:
    """The subsystem modules that exist so far (a missing one is reported, not fatal)."""
    sys.path.insert(0, str(HERE))
    mods = {}
    for name in SUBSYSTEMS:
        if (HERE / f"{name}.py").exists():
            mods[name] = importlib.import_module(name)
    return mods


def _bounds(tris: np.ndarray) -> skeleton.Box:
    v = tris.reshape(-1, 3)
    lo, hi = v.min(0), v.max(0)
    return skeleton.Box(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])


INTERFACE_HINTS = ("mount", "insert", "boss", "shoulder_bolt", "interface")


def _in_union(bb: skeleton.Box, boxes: List[skeleton.Box], n: int = 6) -> bool:
    """Every point of a grid over the box lies in at least one of `boxes`: a part may span
    two zones of its owner (a screw holding the spine down crosses base and spine)."""
    xs, ys, zs = (np.linspace(lo, hi, n) for lo, hi in ((bb.x0, bb.x1), (bb.y0, bb.y1), (bb.z0, bb.z1)))
    for x in xs:
        for y in ys:
            for z in zs:
                if not any(b.x0 <= x <= b.x1 and b.y0 <= y <= b.y1 and b.z0 <= z <= b.z1 for b in boxes):
                    return False
    return True


def zones(solids: Dict[str, Dict[str, np.ndarray]], tol: float = 0.05, declared: Dict[str, set] = None) -> List[dict]:
    """Solids whose bounding box leaves the union of their owner's zones (shell: the
    envelope). Interface parts (a mount's insert or boss, a shoulder bolt) cross into the
    other side's zone by design and are exempt (by label, or listed in the module's
    INTERFACE_PARTS); the interference check still covers them.
    A bounding box is conservative: a curved part may fail here by its box alone; such a
    case is reported with its box, for a person to look at."""
    out = []
    for owner, parts in solids.items():
        boxes = [skeleton.ENVELOPE] if owner == "shell" else skeleton.zone_of(owner)
        grown = [skeleton.Box(b.x0 - tol, b.x1 + tol, b.y0 - tol, b.y1 + tol, b.z0 - tol, b.z1 + tol) for b in boxes]
        for label, tris in parts.items():
            dec = (declared or {}).get(owner, ())
            if any(h in label.lower() for h in INTERFACE_HINTS) or label in dec or label.split("_", 1)[-1] in dec:
                continue
            bb = _bounds(tris)
            if _in_union(bb, grown):
                continue
            # The box may span two zones round a corner (a frame); the part's own vertices decide.
            v = tris.reshape(-1, 3)
            if all(any(g.x0 <= x <= g.x1 and g.y0 <= y <= g.y1 and g.z0 <= z <= g.z1 for g in grown) for x, y, z in v):
                continue
            if True:
                out.append({"owner": owner, "part": label,
                            "box_mm": [round(v, 2) for v in (bb.x0, bb.x1, bb.y0, bb.y1, bb.z0, bb.z1)]})
    return out


def interference(solids: Dict[str, Dict[str, np.ndarray]]) -> dict:
    import trimesh
    flat = {f"{owner}/{label}": t for owner, parts in solids.items() for label, t in parts.items()}
    meshes = {}
    for k, t in flat.items():
        m = trimesh.Trimesh(**trimesh.triangles.to_kwargs(t))
        m.merge_vertices()
        meshes[k] = m
    # Pair by pair, for every pair whose boxes overlap: one manager over all the solids keeps at
    # most 100,000 contacts in all (trimesh's FCL request) and silently drops the pairs past
    # it; at 259 solids it found 110,997 and lost two real interferences (2026-11-03).
    keys = sorted(meshes)
    lo = np.array([meshes[k].bounds[0] for k in keys]) - NUDGE_MM
    hi = np.array([meshes[k].bounds[1] for k in keys]) + NUDGE_MM
    over = np.all((lo[:, None, :] <= hi[None, :, :]) & (lo[None, :, :] <= hi[:, None, :]), axis=-1)
    single: Dict[str, object] = {}
    depth: Dict[Tuple[str, str], float] = {}
    for i, j in zip(*np.nonzero(np.triu(over, 1))):
        a, b = keys[i], keys[j]
        if b not in single:
            single[b] = trimesh.collision.CollisionManager()
            single[b].add_object(b, meshes[b])
        hit, data = single[b].in_collision_single(meshes[a], return_data=True)
        if hit:
            depth[(a, b)] = max(float(c.depth) for c in data) if data else 0.0
    rows = {"touching": [], "threaded": [], "interference": []}
    for (a, b), d in sorted(depth.items()):
        la, lb = a.split("/", 1)[1].lower(), b.split("/", 1)[1].lower()
        if any(h in la for h in THREADED_HINTS) or any(h in lb for h in THREADED_HINTS):
            # a fastener in its hole: threaded if it overlaps by no more than a thread's depth
            if d <= 0.6:
                rows["threaded"].append({"a": a, "b": b, "depth_mm": round(d, 3)})
                continue
        one = single[b]
        freed = None
        for axis in range(3):
            for sgn in (1, -1):
                T = np.eye(4)
                T[axis, 3] = sgn * NUDGE_MM
                if not one.in_collision_single(meshes[a], transform=T):
                    freed = "+-"[sgn < 0] + "xyz"[axis]
                    break
            if freed:
                break
        if freed:
            rows["touching"].append({"a": a, "b": b, "freed_by": freed})
        else:
            rows["interference"].append({"a": a, "b": b, "depth_mm": round(d, 3),
                                         "across": a.split("/")[0] != b.split("/")[0]})
    return {"solids": len(meshes), **rows}


def mounts(mods: Dict[str, object], r=None) -> List[dict]:
    """Every skeleton interface and who provides / uses it, from each module's mounts(r)."""
    named = {m.name: m for m in skeleton.SHELL_MOUNTS + skeleton.SHOULDER_BOLTS + skeleton.ARM_POST_MOUNTS}
    seen: Dict[str, List[str]] = {k: [] for k in named}
    for owner, mod in mods.items():
        if hasattr(mod, "mounts"):
            for m in mod.mounts(r):
                name = m.name if hasattr(m, "name") else m.get("name")
                seen.setdefault(name, []).append(owner)
    out = []
    for name, owners in seen.items():
        need = {named[name].holder, named[name].part} if name in named else set()
        out.append({"mount": name, "provided_by": sorted(set(owners)), "missing": sorted(need - set(owners))})
    return out


def mass(mods: Dict[str, object], r=None) -> dict:
    total, moment = 0.0, np.zeros(3)
    by = {}
    for owner, mod in mods.items():
        if not hasattr(mod, "parts"):
            continue
        g = 0.0
        for p in mod.parts(r):
            m = float(p.get("mass_g") or 0.0)
            g += m
            moment += m * np.asarray(p.get("centre_mm") or (0, 0, 0), float)
        by[owner] = round(g, 1)
        total += g
    com = (moment / total).round(1).tolist() if total else None
    lo, hi = skeleton.MASS_TARGET_G
    return {"total_g": round(total, 1), "by_subsystem_g": by, "com_mm": com, "target_g": [lo, hi],
            "in_target": lo <= total <= hi}


def report(r=None) -> dict:
    mods = load()
    solids = {owner: mod.solids(r) for owner, mod in mods.items() if hasattr(mod, "solids")}
    rep = {"subsystems": sorted(mods), "missing": [s for s in SUBSYSTEMS if s not in mods],
           "skeleton": skeleton.check_skeleton(),
           "zones_left": zones(solids, declared={o: set(getattr(m, "INTERFACE_PARTS", ()) or ()) for o, m in mods.items()}),
           # the interface screws reach into the other side's zone by design (no zone check),
           # but they must still clear or thread into what they pass
           "interference": interference({**solids, "interface": {k: v for m in mods.values() if hasattr(m, "interface_solids")
                                                                 for k, v in m.interface_solids(r).items()}})
           if solids else None,
           "mounts": mounts(mods, r), "mass": mass(mods, r)}
    rep["checks"] = {owner: mod.checks(r) for owner, mod in mods.items() if hasattr(mod, "checks")}
    return rep


if __name__ == "__main__":
    rep = report()
    print(json.dumps(rep, indent=1, default=str))
    bad = rep["interference"] and rep["interference"]["interference"]
    raise SystemExit(1 if bad or rep["zones_left"] else 0)
