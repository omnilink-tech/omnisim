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

"""SparkRover Mini, assembled step by step on Newton, every part checked as it goes on.

The order is the design's: skeleton.ASSEMBLY_NOTE ("shoulder brackets onto the spine first;
then the shell and the electronics; each arm module goes on LAST"), inside it the chassis'
part steps, electrical.STEPS and shell.STEPS, and the arms docstring's order (STAGES below
lists every step and what it adds). Bench sub-assemblies go on as one unit, as the steps
say they are made: the glued spine box, each lower shell half with its bumper, diffuser
and LED strip (shell step 1), each upper half with its ring, each arm module.

For every unit, in order, this writes one world: everything placed so far as ONE static
body (the assembly in its fixture, in the robot frame) and the unit as a dynamic body.
Over its own harness (headless, its own ports) it then:

1. EXACT PATH (FCL, the parts' own meshes): the unit is swept along its insertion path
   (UNIT_PATHS: the direction a screw goes in, a half slides on, a shaft enters its hole)
   from where it is clear of its seat to the seat, in 0.5 mm steps, against every part
   already placed; a collision that a 0.05 mm nudge does not free is a BLOCK.
2. AT THE SEAT (FCL): every part it meets, TOUCHING (a nudge frees it), THREADED (a
   fastener within 0.6 mm) or INTERFERENCE.
3. ON NEWTON: the unit comes the last APPROACH_MM to its seat in INCREMENT_MM set_pose
   steps, /sim/contacts read after each (a contact before the seat: refined at REFINE_MM,
   the blocker named); then it is LET GO for SETTLE_S under gravity and read back: it
   RESTS (within SEAT_TOL_MM of its seat), SHIFTS (within FALL_MM) or FALLS (it needs its
   screws, or something that is not there yet, to stay). Its contacts say what it rests on.
4. It is then fixed at its seat (its screws are in) and joins the static assembly.

Fasteners (screws, nuts, washers, inserts, standoffs) and the electrical parts under 1 g
are not placed by physics: they join their step at their seat and are in every FCL check.
The finished robot is then stood on the bench on its four (braked) wheels, as one dynamic
body at the engine's default contact stiffness, and must come to rest on all four.

Collision shapes for physics: each part's solid split into its pieces; a convex piece is
its own hull (a Mesh, which Newton convexifies), a concave one is voxelised at 1 mm (2 mm
past 400 boxes) and merged into boxes. Voxels are taken where the piece's material is, so
a box may stand up to half a voxel proud of a curved face: the physics contacts are
approximate there, the FCL checks are exact.

Writes release/assembly/assembly_log.json and a picture per step (release/assembly/
physical_step_<k>.png). Simulation only: a screw is not simulated (no thread, no preload).

    python projects/manufacturing/products/spark_rover/assemble.py [--port 6941] [--until N] [--no-physics]
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import os
import pickle
import platform
import subprocess
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def _ours(name: str):
    """This product's module `name` (the desk robot has a robot.py and an electrical.py too, and
    the subsystem modules put its folder on sys.path)."""
    import importlib.util
    m = sys.modules.get(name)
    if m is not None and Path(getattr(m, "__file__", "")).resolve().parent == HERE:
        return m
    key = f"spark_rover_{name}"
    if key not in sys.modules:
        spec = importlib.util.spec_from_file_location(key, HERE / f"{name}.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules[key] = mod
        spec.loader.exec_module(mod)
    return sys.modules[key]


rb = _ours("robot")
import skeleton as sk  # noqa: E402

ROOT = rb.ROOT
WORLDS = ROOT / "projects/samples/demos/worlds/manufacturing"
OUT = HERE / "release/assembly"
TMP = ROOT / ".tmp/spark_rover_assemble"
MM = 1e-3

# ---- tolerances, fixed before any run (the desk robot's, desk_robot/assemble.py) ------------
SEAT_TOL_MM = 0.2          # the tightest screw clearance: further than that and the screw does not go in
FALL_MM = 2.0              # let go, it moved further than this: it falls without its fasteners
NUDGE_MM = 0.05            # a contact a nudge this small frees is a face on a face
THREAD_MM = 0.6            # a fastener this far into a part is in its thread
PATH_STEP_MM = 0.5
SETTLE_S = 0.5
APPROACH_MM = 5.0
INCREMENT_MM = 0.5
REFINE_MM = 0.05
DT_MS = 2
ASSEMBLY_CONTACT_KE = 50000.0    # as the desk robot's assembly: 2.5 kN/m rests parts ~0.5 mm into their seats
VOXEL_MM = (0.5, 1.0, 2.0)          # the finest that stays under MAX_BOXES
HULL_FRAC = 0.93                    # a piece within 7 % of its hull's volume collides as the hull (a deck's
                                    # screw holes are filled: nothing in this simulation passes through them)
MAX_BOXES = 400
FASTENER_HINTS = ("screw", "nut_", "washer", "insert", "standoff", "bolt")


def _f(v: float) -> str:
    return rb._f(v)


def _v(*vals: float) -> str:
    return rb._v(*vals)


# ---- the plan: stages and units -------------------------------------------------------------------

@dataclass
class Unit:
    name: str
    parts: List[str]                          # part labels moved together
    path: List[Tuple[Tuple[float, float, float], float]]   # [(direction it comes FROM, mm), ...] last = final
    note: str = ""


@dataclass
class Stage:
    title: str
    source: str                               # which steps of the design it is
    units: List[Unit]
    fasteners: List[str] = field(default_factory=list)
    # A BENCH stage: its units go onto this fixture (a carrier with its inserts) on the bench,
    # in a scene of their own; the order and each path are found by taking the populated
    # fixture apart (find_order). Nothing of it joins the robot until a later stage mounts it.
    bench: Optional[List[str]] = None


X, Y, Z = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)


def _neg(a):
    return tuple(-v for v in a)


def _is_fastener(label: str) -> bool:
    return any(h in label for h in FASTENER_HINTS)


def plan(d: dict) -> List[Stage]:
    """The build order, every part in exactly one stage."""
    el = _ours("electrical")
    P = el.placements(None)
    table = {p["label"]: p for p in rb.part_table(d)}
    used: set = set()

    def take(labels):
        out = [x for x in labels if x in table and x not in used]
        used.update(out)
        return out

    def fasteners_where(pred):
        return take([k for k, p in table.items() if _is_fastener(k) and pred(p)])

    tags = list(rb.WHEEL_TAGS)
    side_y = {"fl": 1, "fr": -1, "rl": 1, "rr": -1}
    S: List[Stage] = []
    S.append(Stage("Deck: the two plywood layers, glued", "chassis step 1",
                   [Unit(n, take([n]), [(Z, 20.0)]) for n in ("deck_lower", "deck_upper")]))
    # Each gearmotor goes into its bracket on the bench (chassis step 2: the gearbox-face screws
    # and washers), and the unit then goes up under the deck on its spacer (step 3: the foot
    # screws and nuts). Brackets first and motors after cannot be built: the second motor of an
    # axle has no path past the first (1 mm between the encoders; found 2026-11-03).
    u = []
    for t in tags:
        u += [Unit(f"spacer_{t}", take([f"spacer_{t}"]), [(_neg(Z), 20.0)]),
              Unit(f"motor_unit_{t}", take([f"bracket_{t}", f"motor_{t}", f"motor_{t}_shaft"]), [(_neg(Z), 20.0)],
                   "gearmotor and bracket joined on the bench, offered up from below")]
    S.append(Stage("Gearmotors into their brackets (bench), then each unit on its spacer under the deck",
                   "chassis steps 2-3", u,
                   fasteners_where(lambda p: p["owner"] == "chassis" and p["step"] in (2, 3))))
    u = []
    for t in tags:
        out_ = (0.0, float(side_y[t]), 0.0)
        u += [Unit(f"hex_adapter_{t}", take([f"hex_adapter_{t}"]), [(out_, 12.0)]),
              Unit(f"wheel_{t}", take([f"wheel_{t}"]), [(out_, 30.0)])]
    S.append(Stage("Hex adapters and wheels", "chassis step 4", u,
                   fasteners_where(lambda p: p["owner"] == "chassis" and p["step"] == 4)))
    spine = [k for k in table if k.startswith("spine_") and k != "spine_foot"]
    S.append(Stage("Spine foot, then the spine (its six plates glued on the bench)", "chassis steps 5-6",
                   [Unit("spine_foot", take(["spine_foot"]), [(Z, 20.0)]),
                    Unit("spine", take(spine), [(Z, 30.0)], "a glued box: placed whole")],
                   fasteners_where(lambda p: p["owner"] == "chassis" and p["step"] in (5, 6))
                   + take([k for k in table if k.startswith("nut_spine_")])))
    S.append(Stage("Battery under the deck, strapped", "chassis step 8, electrical step 6 (the pack)",
                   [Unit("battery", take(["el_BT1"]), [(_neg(Z), 30.0)])],
                   take(["battery_strap"])))
    S.append(Stage("Shoulder brackets onto the spine", "skeleton.ASSEMBLY_NOTE, arms step 1",
                   [Unit(f"shoulder_bracket_{s}", take([f"shoulder_bracket_{s}"]), [((0.0, float(sy), 0.0), 20.0)])
                    for s, sy in rb.SIDES.items()],
                   take([k for k in table if k.startswith("screw_interface_shoulder")])))
    # The electronics are built the way a person builds them: each carrier is populated on the
    # bench (electrical steps 1-3 onto the bay carrier, 4-5 onto the front carrier), then the
    # loaded carrier goes onto the spine as one unit and is screwed from inside the spine.
    # Electrical step 0 is bench work with shell step 1: the LED strips go in with the bumpers,
    # the charger and button boards onto the lower rear half's inserts, C3 at the front strip's
    # end; the touch pads go with the upper halves.
    lower_units = {"front": [k for k in ("el_D1", "el_C3") if k in table],
                   "rear": [k for k in ("el_D2", "el_CHG1", "el_B2", "el_S1", "el_R10") if k in table]}
    where = {p["label"]: p.get("where") for p in d["parts"]}

    def normal(ref):
        if ref in P:
            return tuple(float(v) for v in P[ref].R[:, 2])
        return Z

    def board(name, labs, ref, note=""):
        return Unit(name, take(labs), [(normal(ref), 20.0)], note)

    def el_rest(steps, wheres):
        return [k for k, p in table.items() if p["owner"] == "electrical" and p["step"] in steps
                and where.get(k) in wheres]

    for carrier, wheres, steps, mount_from, units_spec, mount_screws, title in (
            ("bay", ("bay",), (1, 2, 3), _neg(X),
             [("pi_unit", ["el_U1", "el_pi_standoffs", "el_U10"], "U1"),          # electrical step 1
              ("el_U5", ["el_U5"], "U5"), ("el_U6", ["el_U6"], "U6"), ("el_SW1", ["el_SW1"], "SW1"),   # step 2
              ("el_MA", ["el_MA"], "MA"), ("el_MB", ["el_MB"], "MB"), ("el_F1", ["el_F1"], "F1"),
              ("el_X1", ["el_X1"], "X1"), ("el_X2", ["el_X2"], "X2"), ("el_X3", ["el_X3"], "X3"),
              ("el_X4", ["el_X4"], "X4"),
              ("io_board", ["el_B1", "el_U2", "el_U3", "el_U4", "el_U7"], "B1"),      # step 3
              ("el_cooler_duct", ["el_cooler_duct"], "U1")],
             [k for k in table if k.startswith("el_screw_bay_row") or k.startswith("screw_interface_bay_pi5")],
             "the bay carrier"),
            ("front", ("head", "chest"), (4, 5), X,
             [("el_U9", ["el_U9"], "U9"), ("el_U8", ["el_U8"], "U8"), ("el_LS1", ["el_LS1"], "LS1"),
              ("el_U11", ["el_U11"], "U11"), ("el_U12", ["el_U12"], "U12")],
             [k for k in table if k.startswith("el_screw_head_grid")],
             "the front carrier")):
        fix = take([f"el_{carrier}_carrier"] + [k for k in table if k.startswith("el_insert_")
                                                and where.get(k) == f"{carrier}_carrier"])
        units = [board(n, labs, ref, "IO board populated first (Pico, logic ICs, headers)" if n == "io_board" else
                       "the Pi on its standoffs with its Active Cooler, as one unit" if n == "pi_unit" else "")
                 for n, labs, ref in units_spec]
        mount = set(mount_screws)
        small = take([k for k in el_rest(steps, wheres) if k not in mount]
                     + [k for k in table if k.startswith("el_screw_") and where.get(k) == f"{carrier}_carrier"
                        and k not in mount])
        S.append(Stage(f"Bench: populate {title}", f"electrical steps {steps[0]}-{steps[-1]}, on the bench",
                       [u for u in units if u.parts], small, bench=fix))
        loaded = fix + [x for u in units for x in u.parts] + small
        side = "behind (-x)" if carrier == "bay" else "the front (+x)"
        S.append(Stage(f"{title[0].upper()}{title[1:]}, loaded, onto the spine from {side}; its screws from inside the spine",
                       f"electrical step {steps[-1]}",
                       [Unit(f"{carrier}_carrier_loaded", loaded, [(mount_from, 20.0)], "the populated carrier as one unit")],
                       take(sorted(mount))))
    rest = take([k for k, p in table.items() if p["owner"] == "electrical" and 1 <= p["step"] <= 6
                 and k not in ("el_TP1", "el_TP2", "el_TP3")])
    S.append(Stage(f"Electronics: {el.STEPS[6]}", "electrical step 6", [], rest))
    # shell
    shell_bench = {"front": ["shell_lower_front", "bumper_front", "diffuser_front"] + lower_units["front"],
                   "rear": ["shell_lower_rear", "bumper_rear", "diffuser_rear"] + lower_units["rear"]}
    ins = lambda sec: [k for k, p in table.items() if p["owner"] == "shell" and k.startswith("insert") and _insert_in(d, k, sec)]
    S.append(Stage("Lower shell halves, slid on along x, lugs under the deck corners", "shell steps 1-2",
                   [Unit(f"shell_lower_{s}", take(shell_bench[s] + ins(f"shell_lower_{s}")), [((float(sx), 0.0, 0.0), 25.0)],
                         "bench: bumper, diffuser and LED strip on, inserts in (shell step 1)")
                    for s, sx in (("front", 1), ("rear", -1))],
                   take([k for k in table if k.startswith("screw_interface_lower") or k.startswith("screw_bumper")])
                   + take([k for k, p in table.items() if p["owner"] == "shell" and p["step"] == 2 and _is_fastener(k)])))
    S.append(Stage("Side seam screws", "shell step 3", [],
                   take([k for k, p in table.items() if p["owner"] == "shell" and p["step"] == 3])))
    S.append(Stage("Upper shell halves, slid on along y onto the spine (left, then right)", "shell steps 1, 4",
                   [Unit(f"shell_upper_{s}", take([f"shell_upper_{s}", f"shoulder_ring_{s}"] + ins(f"shell_upper_{s}")),
                         [((0.0, float(sy), 0.0), 90.0)], "bench: teal ring glued in, inserts in")
                    for s, sy in rb.SIDES.items()],
                   take([k for k in table if k.startswith("screw_interface_upper")])
                   + take(["el_TP1", "el_TP2", "el_TP3"])))      # copper tape on the shell's inside (electrical.INTERFACE_PARTS)
    S.append(Stage("Top slot socket, then the accessory board", "shell step 5, electrical step 7",
                   [Unit("top_socket", take(["top_socket"]), [(Z, 20.0)]), Unit("el_B3", take(["el_B3"]), [(Z, 15.0)])],
                   take([k for k, p in table.items() if (p["owner"] == "shell" and p["step"] == 5) or
                         (p["owner"] == "electrical" and p["step"] == 7)])))
    S.append(Stage("Rear hatch", "shell step 6", [Unit("rear_hatch", take(["rear_hatch"]), [(_neg(X), 15.0)])],
                   take([k for k, p in table.items() if p["owner"] == "shell" and p["step"] == 6])))
    S.append(Stage("Face cover, bonded", "shell step 7", [Unit("face_cover", take(["face_cover"]), [(X, 10.0)])]))
    arm_units = []
    for s, sy in rb.SIDES.items():
        labs = [k for k, p in table.items() if p["owner"] == "arms" and k.endswith(f"_{s}") and k not in used]
        arm_units.append(Unit(f"arm_{s}", take(labs), [((0.0, float(sy), 0.0), 30.0)],
                              "the arm module (post, servos, links, screws), built on the bench; post through the 40 mm opening"))
    S.append(Stage("Arm modules, last, from outside", "skeleton.ASSEMBLY_NOTE, arms steps 2-5", arm_units))
    left = [k for k in table if k not in used]
    if left:
        S[-1].fasteners += take(left)
    return S


_INSERT_PART: Dict[str, str] = {}


def _insert_in(d: dict, label: str, section: str) -> bool:
    """An insert belongs to the shell section shell.insert_sites() puts it in."""
    if not _INSERT_PART:
        shell = _ours("shell")
        _INSERT_PART.update({o["label"]: o["part"] for o in shell.insert_sites(shell.config(None))})
    return _INSERT_PART.get(label) == section


# ---- collision shapes --------------------------------------------------------------------------

def _hash(tris: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(tris, dtype=np.float32).tobytes()).hexdigest()[:16]


def _voxel_boxes(tris: np.ndarray, pitch: float) -> List[Tuple[np.ndarray, np.ndarray]]:
    import open3d as o3d
    v = tris.reshape(-1, 3).astype(np.float32)
    f = np.arange(len(v), dtype=np.uint32).reshape(-1, 3)
    sc = o3d.t.geometry.RaycastingScene()
    sc.add_triangles(o3d.core.Tensor(v), o3d.core.Tensor(f))
    lo, hi = v.min(0), v.max(0)
    n = np.maximum(1, np.ceil((hi - lo) / pitch).astype(int))
    lo = (lo + hi) / 2 - n * pitch / 2
    axes = [lo[i] + (np.arange(n[i]) + 0.5) * pitch for i in range(3)]
    G = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3).astype(np.float32)
    occ = sc.compute_occupancy(o3d.core.Tensor(G), nsamples=3).numpy().reshape(n) > 0.5
    out = []
    nx, ny, nz = occ.shape
    for i in range(nx):
        for j in range(ny):
            k = 0
            while k < nz:
                if not occ[i, j, k]:
                    k += 1
                    continue
                k1 = k
                while k1 < nz and occ[i, j, k1]:
                    k1 += 1
                j1 = j + 1
                while j1 < ny and occ[i, j1, k:k1].all():
                    j1 += 1
                i1 = i + 1
                while i1 < nx and occ[i1, j:j1, k:k1].all():
                    i1 += 1
                occ[i:i1, j:j1, k:k1] = False
                out.append((lo + np.array([i, j, k]) * pitch, lo + np.array([i1, j1, k1]) * pitch))
                k = k1
    return out


UNIT_MAX_PRIMS = 600       # a moving unit with more shapes than this collides as its pieces' hulls (below)


def unit_prims(ups: Sequence[dict]) -> List[tuple]:
    """A moving unit's collision shapes. A big bench-built unit (an arm module: ~2,700 voxel boxes)
    against the ~6,000-shape assembly overflows MuJoCo's arena in mj_narrowphase ("mj_stackAlloc:
    out of memory"): the engine skips every step and the unit hangs motionless with no contacts
    (found 2026-11-04). Its outside is what meets the robot, so it collides as each piece's hull."""
    prims = [pr for p in ups for pr in collision_prims(p["tris"])]
    if len(prims) > UNIT_MAX_PRIMS:
        prims = [pr for p in ups for pr in collision_prims(p["tris"], hull_frac=0.0)]
    return prims


def collision_prims(tris: Optional[np.ndarray], hull_frac: float = None) -> List[tuple]:
    """[("hull", vertices mm, faces) | ("box", lo mm, hi mm)] for one part, robot frame. Cached."""
    if tris is None or len(tris) == 0:
        return []
    frac = HULL_FRAC if hull_frac is None else hull_frac
    key = _hash(tris) + ("" if hull_frac is None else f"_h{frac:g}")
    f = TMP / "prims" / f"{key}.pkl"
    if f.exists():
        with f.open("rb") as fh:
            return pickle.load(fh)
    import trimesh
    m = trimesh.Trimesh(**trimesh.triangles.to_kwargs(np.asarray(tris, float)))
    m.merge_vertices()
    out = []
    for c in m.split(only_watertight=False):
        if len(c.faces) < 4:
            continue
        try:
            hull = c.convex_hull
            hv = float(hull.volume)
        except Exception:  # noqa: BLE001  (flat or degenerate: no volume to collide with)
            continue
        vol = abs(float(trimesh.triangles.mass_properties(c.triangles, density=1.0)["volume"]))
        if hv < 1e-3:
            continue
        if vol / hv >= frac:
            out.append(("hull", np.asarray(hull.vertices, float), np.asarray(hull.faces, int)))
            continue
        boxes = []
        for pitch in VOXEL_MM:
            boxes = _voxel_boxes(np.asarray(c.triangles, float), pitch)
            if len(boxes) <= MAX_BOXES:
                break
        out += [("box", lo, hi) for lo, hi in boxes]
    f.parent.mkdir(parents=True, exist_ok=True)
    with f.open("wb") as fh:
        pickle.dump(out, fh)
    return out


# ---- worlds ----------------------------------------------------------------------------------------

def _hull_file(v: np.ndarray, faces: np.ndarray) -> Path:
    tris = v[faces] * MM
    p = TMP / "hulls" / f"{_hash(tris)}.stl"
    if not p.exists():
        rb.write_stl(p, tris, "hull")
    return p


def _url(p: Path) -> str:
    return Path(os.path.relpath(p, WORLDS)).as_posix()


def prims_vrml(prims: Sequence[tuple], shift_mm=(0.0, 0.0, 0.0)) -> List[str]:
    s = np.asarray(shift_mm, float)
    out = []
    for pr in prims:
        if pr[0] == "box":
            lo, hi = pr[1] - s, pr[2] - s
            out.append(f"Pose {{ translation {_v(*((lo + hi) / 2 * MM))} children [ Box {{ size {_v(*((hi - lo) * MM))} }} ] }}")
        else:
            f = _hull_file(pr[1], pr[2])
            out.append(f'Pose {{ translation {_v(*(-s * MM))} children [ Mesh {{ url "{_url(f)}" }} ] }}')
    return out


def visual_file(label: str, tris: np.ndarray) -> Path:
    p = TMP / "vis" / f"{label}_{_hash(tris)}.stl"
    if not p.exists():
        rb.write_stl(p, tris * MM, label)
    return p


def visuals_vrml(parts: Sequence[dict], shift_mm=(0.0, 0.0, 0.0)) -> str:
    s = np.asarray(shift_mm, float) * MM
    shapes = []
    for p in parts:
        if p["tris"] is None:
            continue
        shapes.append(rb._shape(_url(visual_file(p["label"], p["tris"])), p["rgb"], p["transparency"], len(p["tris"])))
    return f"Pose {{ translation {_v(*(-s))} children [ {' '.join(shapes)} ] }}" if shapes else ""


def unit_physics(parts: Sequence[dict]) -> Tuple[np.ndarray, str]:
    """(centre of mass mm, Physics node about it) of a unit."""
    M = sum(p["mass_g"] for p in parts) / 1000.0
    com = sum(p["mass_g"] * p["centre_mm"] for p in parts) / max(M * 1000.0, 1e-9)
    I = np.zeros((3, 3))
    for p in parts:
        m = p["mass_g"] / 1000.0
        I += rb._solid_inertia(p["tris"], m, p["centre_mm"])
        dd = (p["centre_mm"] - com) * MM
        I += m * (dd @ dd * np.eye(3) - np.outer(dd, dd))
    I += np.eye(3) * 1e-9
    M = max(M, 1e-4)
    return com, (f"physics Physics {{ density -1 mass {_f(M)} centerOfMass [ 0 0 0 ] inertiaMatrix [ {I[0, 0]:.6g} "
                 f"{I[1, 1]:.6g} {I[2, 2]:.6g} {I[0, 1]:.6g} {I[0, 2]:.6g} {I[1, 2]:.6g} ] }}")


def world_text(title: str, nodes: Sequence[str], contact_ke: Optional[float] = ASSEMBLY_CONTACT_KE,
               eye=(0.55, -0.62, 0.48), target=(0.0, 0.0, 0.12)) -> str:
    sys.path.insert(0, str(rb.MANUFACTURING / "core"))
    from worldkit import look_at
    L = ["#OMNISIM R2025a utf8",
         'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
         'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
         'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
         "# Written by projects/manufacturing/products/spark_rover/assemble.py; deleted after the run.",
         f'WorldInfo {{ basicTimeStep {DT_MS} title "{title}" newtonSolver "mujoco" newtonSubsteps 2 FPS 30'
         + (f" newtonContactKe {_f(contact_ke)} newtonIterations 150 newtonLsIterations 50" if contact_ke else "") + " }",
         f"Viewpoint {{ position {_v(*eye)} orientation {look_at(eye, target)} fieldOfView .7 }}",
         "OmniSimSky { }", "DEF SUN OmniSimSun { }", "DEF SUN_MARKER OmniSimSunMarker { }",
         'DEF BENCH Solid { name "bench" translation 0 0 -0.01 children [ Shape { appearance PBRAppearance '
         '{ baseColor .9 .9 .88 roughness .8 metalness 0 } geometry Box { size 1.4 1 .02 } } ] '
         'boundingObject Box { size 1.4 1 .02 } }']
    return "\n".join(L + list(nodes) + [""])


def assy_node(placed: Sequence[dict], prims: Sequence[tuple], visuals: bool, dynamic: bool = False,
              lift_mm: float = 0.0) -> str:
    if not placed:
        return ""
    vis = visuals_vrml(placed) if visuals else ""
    if not dynamic:
        bo = "Group { children [ " + " ".join(prims_vrml(prims)) + " ] }"
        return f'DEF ASSY Solid {{ name "assembly" children [ {vis} ] boundingObject {bo} }}'
    com, phys = unit_physics(placed)
    bo = "Group { children [ " + " ".join(prims_vrml(prims, com)) + " ] }"
    vis = visuals_vrml(placed, com) if visuals else ""
    t = (com + np.array([0.0, 0.0, lift_mm])) * MM
    return f'DEF ASSY Solid {{ name "assembly" translation {_v(*t)} children [ {vis} ] boundingObject {bo} {phys} }}'


# The build stand under the first part (the deck): four blocks under its corners, x 72-84,
# |y| 10-30, up to the deck's underside (z 50). Once the deck is on, the assembly is the fixture.
STAND = " ".join(
    f'DEF STAND_{i} Solid {{ name "stand_{i}" translation {_v(sx * 0.078, sy * 0.020, 0.025)} children [ Shape {{ appearance '
    f'PBRAppearance {{ baseColor .35 .37 .4 roughness .6 metalness .3 }} geometry Box {{ size 0.012 0.02 0.05 }} }} ] '
    f'boundingObject Box {{ size 0.012 0.02 0.05 }} }}'
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))))


def unit_node(u: Unit, parts: Sequence[dict], prims: Sequence[tuple], at_mm: np.ndarray, visuals: bool) -> Tuple[str, np.ndarray]:
    com, phys = unit_physics(parts)
    bo = "Group { children [ " + " ".join(prims_vrml(prims, com)) + " ] }"
    vis = visuals_vrml(parts, com) if visuals else ""
    t = (com + at_mm) * MM
    return (f'DEF U_{u.name.upper()} Solid {{ name "{u.name}" translation {_v(*t)} children [ {vis} ] '
            f"boundingObject {bo} {phys} }}"), com


# ---- the harness (desk_robot/assemble.py's) --------------------------------------------------------

class Harness:
    """Our own harness on our own ports (headless). Stopped, with its engine tree, by stop()."""

    def __init__(self, port: int, log_path: Path):
        self.port, self.base, self.log_path, self.proc = port, f"http://127.0.0.1:{port}", log_path, None

    def start(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log = self.log_path.open("w", encoding="utf-8")
        env = dict(os.environ, OMNISIM_LOG_PATH=str(self.log_path.with_suffix(".engine.log")))
        self.proc = subprocess.Popen([sys.executable, "-m", "omnisim", "harness", "--port", str(self.port),
                                      "--supervisor-port", str(self.port + 1)], cwd=ROOT, stdout=self._log,
                                     stderr=subprocess.STDOUT, env=env)
        for _ in range(180):
            try:
                self.call("/healthz", timeout=5)
                return
            except Exception:  # noqa: BLE001
                if self.proc.poll() is not None:
                    raise RuntimeError(f"the harness exited ({self.proc.returncode}); see {self.log_path}")
                time.sleep(1.0)
        raise RuntimeError("the harness did not come up in 180 s")

    def call(self, path: str, body=None, timeout: float = 300, raw: bool = False):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.base + path, data=data, headers={"Content-Type": "application/json"},
                                     method="POST" if body is not None else "GET")
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                b = resp.read()
        except urllib.error.HTTPError as e:
            raise RuntimeError(f"{path}: HTTP {e.code}: {e.read()[:800]!r}") from None
        return b if raw else json.loads(b or b"{}")

    def stop(self) -> None:
        if self.proc is None:
            return
        if os.name == "nt":
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(self.proc.pid)], capture_output=True)
        else:
            self.proc.terminate()
        try:
            self.proc.wait(timeout=20)
        except Exception:  # noqa: BLE001
            pass
        self._log.close()
        self.proc = None

    def load(self, world: Path) -> dict:
        d = self.call("/world/load", {"path": world.relative_to(ROOT).as_posix(), "light": True}, timeout=600)
        if d.get("ok") is False:
            raise RuntimeError(f"world load failed: {json.dumps(d)[:2000]}")
        for _ in range(600):              # a big world can return before its supervisor is up
            st = self.call("/sim/state")
            if st.get("supervisor_connected"):
                break
            time.sleep(0.5)
        # ... and before Newton has finalised it (thousands of collision shapes): steps taken
        # before that move nothing. The engine log is rewritten on every load.
        log = self.log_path.with_suffix(".engine.log")
        for _ in range(600):
            try:
                if "world finalised" in log.read_text(encoding="utf-8", errors="replace"):
                    break
            except OSError:
                pass
            time.sleep(0.5)
        else:
            raise RuntimeError(f"Newton never finalised {world.name} (see {log})")
        return d

    def pause(self) -> None:
        self.call("/sim/pause", {"lease_ms": 300000})

    def step(self, n: int) -> dict:
        self.pause()
        return self.call("/sim/step", {"steps": int(n)})

    def position(self, def_: str) -> Tuple[np.ndarray, np.ndarray]:
        d = self.call(f"/scene/node/{def_}")
        return np.asarray(d["position"], float), np.asarray(d["orientation"], float).reshape(3, 3)

    def set_translation(self, def_: str, t_m: np.ndarray, settle: int = 1) -> dict:
        return self.call("/scene/set_pose", {"def": def_, "translation": [float(v) for v in t_m],
                                             "rotation": [0.0, 0.0, 1.0, 0.0], "reset_physics": True,
                                             "settle_steps": settle})

    def contacts(self) -> List[dict]:
        return self.call("/sim/contacts").get("contacts", [])

    def screenshot(self, path: Path) -> None:
        from PIL import Image
        png = self.call("/world/screenshot", {}, raw=True)
        im = Image.open(io.BytesIO(png)).convert("RGB")
        im.thumbnail((1200, 1200))
        path.parent.mkdir(parents=True, exist_ok=True)
        im.save(path, optimize=True)


def _touching(cs: Sequence[dict], def_: str) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for c in cs:
        a, b = c.get("a_def"), c.get("b_def")
        if def_ in (a, b):
            o = (b or f"name:{c.get('b_name')}") if a == def_ else (a or f"name:{c.get('a_name')}")
            out[o] = out.get(o, 0) + 1
    return out


# ---- exact geometry (FCL) ----------------------------------------------------------------------------

class Placed:
    """Every part placed so far, as its own mesh, for exact collision queries."""

    def __init__(self):
        import trimesh
        self.tm = trimesh
        self.cm = trimesh.collision.CollisionManager()
        self.meshes: Dict[str, object] = {}

    def mesh(self, tris: np.ndarray):
        m = self.tm.Trimesh(**self.tm.triangles.to_kwargs(np.asarray(tris, float)))
        m.merge_vertices()
        return m

    def add(self, label: str, tris: Optional[np.ndarray]) -> None:
        if tris is None:
            return
        m = self.mesh(tris)
        self.meshes[label] = m
        self.cm.add_object(label, m)

    def remove(self, label: str) -> None:
        if label in self.meshes:
            self.cm.remove_object(label)
            del self.meshes[label]

    def bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        b = np.array([m.bounds for m in self.meshes.values()])
        return b[:, 0].min(0), b[:, 1].max(0)

    def hits(self, m, offset_mm: np.ndarray) -> Dict[str, float]:
        """{placed part: depth} that the mesh m, moved by offset, meets and a nudge does not free."""
        T = np.eye(4)
        T[:3, 3] = offset_mm
        hit, names, data = self.cm.in_collision_single(m, transform=T, return_names=True, return_data=True)
        if not hit:
            return {}
        depth: Dict[str, float] = {}
        for c in data:
            nm = next(iter(n for n in c.names if n in self.meshes), None)
            if nm is None:
                continue
            depth[nm] = max(depth.get(nm, 0.0), float(c.depth))
        out = {}
        for nm, dpt in depth.items():
            one = self.tm.collision.CollisionManager()
            one.add_object(nm, self.meshes[nm])
            freed = False
            for ax in range(3):
                for sg in (1, -1):
                    T2 = T.copy()
                    T2[ax, 3] += sg * NUDGE_MM
                    if not one.in_collision_single(m, transform=T2):
                        freed = True
                        break
                if freed:
                    break
            if not freed:
                out[nm] = dpt
        return out

    def at_seat(self, labels: Sequence[str], parts: Dict[str, dict]) -> dict:
        """Each unit part against everything placed: touching / threaded / interference."""
        rows = {"touching": [], "threaded": [], "interference": []}
        for lab in labels:
            p = parts[lab]
            if p["tris"] is None:
                continue
            m = self.mesh(p["tris"])
            hit, names = self.cm.in_collision_single(m, return_names=True)
            if not hit:
                continue
            hard = self.hits(m, np.zeros(3))
            for other in names:
                if other not in self.meshes:
                    continue
                if other not in hard:
                    rows["touching"].append({"part": lab, "with": other})
                elif (_is_fastener(lab) or _is_fastener(other)) and hard[other] <= THREAD_MM:
                    rows["threaded"].append({"part": lab, "with": other, "depth_mm": round(hard[other], 3)})
                else:
                    rows["interference"].append({"part": lab, "with": other, "depth_mm": round(hard[other], 3)})
        for k in rows:          # one row per pair
            seen, uniq = set(), []
            for r in rows[k]:
                key = (r["part"], r["with"])
                if key not in seen:
                    seen.add(key)
                    uniq.append(r)
            rows[k] = uniq
        return rows


def path_points(u: Unit) -> List[Tuple[np.ndarray, float]]:
    """(offset mm, distance still to go) from the start of the path to the seat. Each path
    segment (direction it comes FROM, mm) is travelled in order; the start is their sum."""
    segs = [(np.asarray(dv, float) / np.linalg.norm(dv), float(dist)) for dv, dist in u.path]
    pts = []
    for i, (dv, dist) in enumerate(segs):
        later = sum((dv2 * d2 for dv2, d2 in segs[i + 1:]), np.zeros(3))
        later_d = sum(d2 for _, d2 in segs[i + 1:])
        n = max(1, int(round(dist / PATH_STEP_MM)))
        for j in range(n + 1):
            s_ = dist * (1 - j / n)
            pts.append((later + dv * s_, later_d + s_))
    return pts


ALT_PRE_MM = 80.0
ALT_FINAL_MM = 5.0         # second try: only the last 5 mm along the designed direction (onto standoffs, a post)


ESCAPE_MM = 100.0
ESCAPE_FIRST_MM = (2.0, 5.0, 10.0, 20.0, 40.0)


def _escape_sweep(placed: Placed, m, legs, ignore=(), bounds=None) -> Tuple[bool, list]:
    """Move m from its seat along legs [(direction, mm)], first leg first. Clear once its box is
    1 mm clear of everything placed. Returns (clear, the legs actually travelled)."""
    lo, hi = bounds if bounds is not None else placed.bounds()
    mlo, mhi = m.bounds
    off = np.zeros(3)
    done = []
    for dv, dist in legs:
        dv = np.asarray(dv, float)
        n = max(1, int(round(dist / PATH_STEP_MM)))
        for i in range(1, n + 1):
            p = off + dv * dist * i / n
            h = {k: v for k, v in placed.hits(m, p).items() if k not in ignore}
            if h:
                return False, done + [(tuple(dv), round(dist * i / n, 2), sorted(h))]
            if np.any(mlo + p > hi + 1.0) or np.any(mhi + p < lo - 1.0):
                return True, done + [(tuple(dv), round(dist * i / n, 2))]
        off = off + dv * dist
        done.append((tuple(dv), dist))
    return True, done


def escape_path(placed: Placed, m, u: Unit) -> Tuple[Optional[list], int]:
    """A way OUT of the seat with everything in `placed` present (straight along an axis, or a
    short first leg and then a perpendicular one), returned as an assembly path (its reverse)."""
    f = tuple(float(v) for v in np.asarray(u.path[-1][0], float) / np.linalg.norm(u.path[-1][0]))
    axes = [f, _neg(f)] + [a for a in (X, _neg(X), Y, _neg(Y), Z, _neg(Z)) if abs(float(np.dot(a, f))) < 0.9]
    ignore = set(placed.hits(m, np.zeros(3))) if placed.meshes else set()   # a seat interference is reported there
    bounds = placed.bounds() if placed.meshes else (np.zeros(3), np.zeros(3))
    cands = [[(a, ESCAPE_MM)] for a in axes]
    cands += [[(a, s1), (b, ESCAPE_MM)] for a in axes for s1 in ESCAPE_FIRST_MM for b in axes
              if abs(float(np.dot(a, b))) < 0.1]
    for tried, legs in enumerate(cands, start=1):
        ok, done = _escape_sweep(placed, m, legs, ignore, bounds)
        if ok:
            return [(tuple(float(x) for x in dv), float(dist)) for dv, dist in reversed(done)], tried
    return None, len(cands)


def find_order(fixture: Sequence[str], units: Sequence[Unit], parts: Dict[str, dict]):
    """Assembly by disassembly: the populated fixture is taken apart, each time removing the
    unit latest in the preferred order that has a way out with every other unit still on; the
    reverse is an order that builds, each unit's path the reverse of its way out. Units that
    never come out are returned as stuck."""
    pl = Placed()
    for lab in fixture:
        pl.add(lab, parts[lab]["tris"])
    for u in units:
        for lab in u.parts:
            pl.add(lab, parts[lab]["tris"])
    present, removed = list(units), []
    while present:
        for u in reversed(present):
            tris = [parts[k]["tris"] for k in u.parts if parts[k]["tris"] is not None]
            for lab in u.parts:
                pl.remove(lab)
            path, tried = escape_path(pl, pl.mesh(np.concatenate(tris)), u)
            if path:
                removed.append((u, path, tried))
                present.remove(u)
                break
            for lab in u.parts:
                pl.add(lab, parts[lab]["tris"])
        else:
            break
    return list(reversed(removed)), present


def exact_path(placed: Placed, u: Unit, parts: Dict[str, dict]) -> dict:
    """The designed path; if it is blocked, every L-shaped path that ends with the designed
    final segment and starts ALT_PRE_MM off along one of the four other axis directions."""
    tris = [parts[k]["tris"] for k in u.parts if parts[k]["tris"] is not None]
    if not tris or not placed.meshes:
        return {"clear_start": True, "blocked": False}
    m = placed.mesh(np.concatenate(tris))
    res = sweep(placed, m, u)
    if not res["blocked"]:
        return res
    f = np.asarray(u.path[-1][0], float)
    tried = []
    finals = [u.path[-1][1]] + ([ALT_FINAL_MM] if u.path[-1][1] > ALT_FINAL_MM else [])
    # A board's mounting normal is its own +z, which points into its carrier when it is mounted
    # component side down (the Pi on its standoffs): it then goes on from the other side. So the
    # reversed direction is tried too, straight and as the last leg of an L.
    for last in (tuple(f), tuple(-f)):
        if last != tuple(f):
            alt = Unit(u.name, u.parts, [(last, u.path[-1][1])])
            r2 = sweep(placed, m, alt)
            tried.append({"path": [{"from": list(last), "mm": u.path[-1][1]}], "reversed": True,
                          "blocked": r2["blocked"], "blocked_by": r2.get("blocked_by")})
            if not r2["blocked"]:
                res["clear_alternative"] = tried[-1]["path"]
                break
        for fin in finals:
            for ax in (X, _neg(X), Y, _neg(Y), Z, _neg(Z)):
                if abs(float(np.dot(ax, f))) > 0.9:
                    continue
                alt = Unit(u.name, u.parts, [(ax, ALT_PRE_MM), (last, fin)])
                r2 = sweep(placed, m, alt)
                tried.append({"path": [{"from": list(ax), "mm": ALT_PRE_MM}, {"from": list(last), "mm": fin}],
                              "blocked": r2["blocked"], "blocked_by": r2.get("blocked_by")})
                if not r2["blocked"]:
                    res["clear_alternative"] = tried[-1]["path"]
                    break
            if "clear_alternative" in res:
                break
        if "clear_alternative" in res:
            break
    res["alternatives_tried"] = tried
    return res


def sweep(placed: Placed, m, u: Unit) -> dict:
    pts = path_points(u)
    first = placed.hits(m, pts[0][0])
    if first:
        return {"clear_start": False, "blocked": True, "blocked_at_mm": round(pts[0][1], 2),
                "blocked_by": sorted(first), "note": "already in collision where its path starts"}
    for off, togo in pts[1:]:
        if togo <= 1e-6:
            break
        h = placed.hits(m, off)
        if h:
            return {"clear_start": True, "blocked": True, "blocked_at_mm": round(togo, 2), "blocked_by": sorted(h),
                    "depth_mm": {k: round(v, 3) for k, v in h.items()}}
    return {"clear_start": True, "blocked": False}


# ---- one unit on Newton ----------------------------------------------------------------------------

def place_on_newton(h: Harness, u: Unit, parts: Dict[str, dict], placed_parts: List[dict], assy_prims: List[tuple],
                    written: List[Path], k: int) -> dict:
    ups = [parts[x] for x in u.parts]
    prims = unit_prims(ups)
    if not prims:
        return {"skipped": "no collision shape"}
    final_dir = np.asarray(u.path[-1][0], float)
    final_dir /= np.linalg.norm(final_dir)
    d0 = min(APPROACH_MM, u.path[-1][1])
    node, com = unit_node(u, ups, prims, final_dir * d0, visuals=False)
    world = WORLDS / f".spark_rover_assemble_{k:03d}.omniworld"
    world.write_text(world_text(f"SparkRover assembly: {u.name}", [assy_node(placed_parts, assy_prims, False) or STAND, node]),
                     encoding="utf-8", newline="\n")
    written.append(world)
    h.load(world)
    h.pause()
    DEF = f"U_{u.name.upper()}"

    def at(off_mm: float) -> Dict[str, int]:
        """Its contacts at off_mm from its seat; the bench is support, not an obstacle."""
        h.set_translation(DEF, (com + final_dir * off_mm) * MM)
        return {k2: v2 for k2, v2 in _touching(h.contacts(), DEF).items() if k2 != "BENCH"}

    blocked_at, blocker = None, {}
    n = int(round(d0 / INCREMENT_MM))
    for i in range(n + 1):
        off = d0 - i * INCREMENT_MM
        touch = at(off)
        if off > 1e-9 and touch:
            o = off + INCREMENT_MM if i else off
            if i:
                at(o)
            while True:
                o2 = round(o - REFINE_MM, 6)
                touch = at(o2)
                if touch or o2 <= 1e-9:
                    break
                o = o2
            if touch and o2 > 1e-9:
                blocked_at, blocker = o2, touch
            break
    at(0.0)                                      # the builder puts it on its seat
    st0 = h.call("/sim/state").get("engine_time_ms")
    stepped = h.step(int(SETTLE_S * 1000 / DT_MS))
    st1 = h.call("/sim/state").get("engine_time_ms")
    pos, R = h.position(DEF)
    touch: Dict[str, int] = {}
    for i in range(3):
        h.step(1)
        for k2, v2 in _touching(h.contacts(), DEF).items():
            touch[k2] = touch.get(k2, 0) + v2
    v = np.concatenate([p["tris"].reshape(-1, 3) for p in ups if p["tris"] is not None])
    lo, hi = v.min(0), v.max(0)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    now = (corners - com) @ R.T + pos / MM
    err = float(np.max(np.linalg.norm(now - corners, axis=1)))
    verdict = "rests" if err <= SEAT_TOL_MM else ("shifts" if err <= FALL_MM else "falls")
    try:                                  # a step MuJoCo refused is logged, not reported by /sim/step
        step_errors = h.log_path.with_suffix(".engine.log").read_text(encoding="utf-8", errors="replace").count("step raised")
    except OSError:
        step_errors = None
    if err < 1e-3 and not touch:          # did not move and touches nothing: keep the engine's log of it
        import shutil
        shutil.copy(h.log_path.with_suffix(".engine.log"), TMP / f"motionless_{u.name}_{k:03d}.log")
    return {"physics_approach_mm": d0, "physics_blocked_at_mm": blocked_at, "physics_blocked_by": sorted(blocker),
            "released": verdict, "seat_error_mm": round(err, 3), "drop_mm": round(float((pos / MM - com)[2]), 3),
            "rests_on": touch, "collision_prims": len(prims),
            "released_for_engine_ms": (st1 - st0) if (st0 is not None and st1 is not None) else None,
            "engine_step_errors": step_errors,
            "steps_executed": stepped.get("steps_executed")}


def stage_picture(h: Harness, placed_parts: List[dict], prims: List[tuple], k: int, written: List[Path]) -> str:
    world = WORLDS / f".spark_rover_assemble_pic_{k:02d}.omniworld"
    world.write_text(world_text(f"SparkRover assembly, after step {k}", [assy_node(placed_parts, prims, True)]),
                     encoding="utf-8", newline="\n")
    written.append(world)
    h.load(world)
    try:
        h.call("/scene/frame", {"def": "ASSY", "margin": 0.35})
    except Exception:  # noqa: BLE001
        pass
    h.step(5)
    name = f"physical_step_{k}.png"
    h.screenshot(OUT / name)
    return name


def standing_check(h: Harness, placed_parts: List[dict], prims: List[tuple], written: List[Path]) -> dict:
    """The finished robot, one rigid body (gearmotors braked), dropped 0.5 mm onto the bench at
    the engine's default contact stiffness (as the drive test runs)."""
    world = WORLDS / ".spark_rover_assemble_standing.omniworld"
    world.write_text(world_text("SparkRover assembled, standing", [assy_node(placed_parts, prims, True, dynamic=True,
                                                                              lift_mm=0.5)], contact_ke=None),
                     encoding="utf-8", newline="\n")
    written.append(world)
    h.load(world)
    h.pause()
    h.step(int(1.0 * 1000 / DT_MS))
    pA, RA = h.position("ASSY")
    h.step(int(SETTLE_S * 1000 / DT_MS))
    pB, RB = h.position("ASSY")
    feet = {t: 0 for t in rb.WHEEL_TAGS}
    other = 0
    com, _ = unit_physics(placed_parts)
    for c in h.contacts():
        if "ASSY" not in (c.get("a_def"), c.get("b_def")):
            continue
        q = RB.T @ (np.asarray(c.get("point"), float) - pB) / MM + com
        near = min(rb.WHEEL_TAGS, key=lambda t: np.linalg.norm(q[:2] - np.asarray(sk.WHEELS[rb.WHEEL_TAGS[t]][:2])))
        if q[2] < 5.0 and np.linalg.norm(q[:2] - np.asarray(sk.WHEELS[rb.WHEEL_TAGS[near]][:2])) < 20.0:
            feet[near] += 1
        else:
            other += 1
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(RB[2, 2])))))
    drift = float(np.linalg.norm(pB - pA) / MM)
    h.call("/scene/frame", {"def": "ASSY", "margin": 0.5})
    h.step(5)
    h.screenshot(OUT / "physical_standing.png")
    reasons = [f"not standing on wheel {t}" for t, n in feet.items() if not n]
    if other:
        reasons.append(f"{other} contacts that are not under a wheel")
    if drift > 0.05:
        reasons.append(f"moved {drift:.3f} mm in {SETTLE_S} s after settling")
    return {"contacts_by_wheel": feet, "other_contacts": other, "tilt_deg": round(tilt, 3), "drift_mm": round(drift, 4),
            "com_height_mm": round(float(pB[2] / MM), 2), "designed_com_height_mm": round(float(com[2]), 2),
            "screenshot": "physical_standing.png", "pass": not reasons, "reasons": reasons}


def _machine() -> dict:
    try:
        sys.path.insert(0, str(ROOT / "projects/policies/common"))
        import env_fingerprint
        return env_fingerprint._machine_identity(env_fingerprint._nvidia_gpu())
    except Exception as e:  # noqa: BLE001
        return {"error": str(e), "os": platform.platform(terse=True)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Assemble SparkRover Mini step by step on Newton, checking each part.")
    ap.add_argument("--port", type=int, default=6941, help="harness port (the supervisor takes port + 1)")
    ap.add_argument("--until", type=int, default=0, help="stop after this many steps (0: all)")
    ap.add_argument("--no-physics", action="store_true", help="the exact (FCL) checks only, no engine")
    ap.add_argument("--check-from", type=int, default=1,
                    help="place the steps before this one without checking them (to re-check a late step quickly)")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    global OUT
    if a.out:
        OUT = Path(a.out)
    OUT.mkdir(parents=True, exist_ok=True)
    TMP.mkdir(parents=True, exist_ok=True)
    t_all = time.time()
    d = rb.design()
    parts = {p["label"]: p for p in rb.part_table(d)}
    stages = plan(d)
    if a.until:
        stages = stages[:a.until]
    log = {"what": "SparkRover Mini assembled step by step on Newton (MuJoCo CPU solver), each unit checked",
           "simulation_only": True,
           "tolerances": {"seat_mm": SEAT_TOL_MM, "fall_mm": FALL_MM, "nudge_mm": NUDGE_MM, "thread_mm": THREAD_MM,
                          "path_step_mm": PATH_STEP_MM, "settle_s": SETTLE_S, "approach_mm": APPROACH_MM,
                          "increment_mm": INCREMENT_MM, "refine_mm": REFINE_MM},
           "solver": {"basicTimeStep_ms": DT_MS, "newtonSubsteps": 2, "newtonSolver": "mujoco",
                      "newtonContactKe": ASSEMBLY_CONTACT_KE},
           "machine": _machine(), "steps": []}
    h = None if a.no_physics else Harness(a.port, TMP / "harness.log")
    written: List[Path] = []
    placed = Placed()
    placed_parts: List[dict] = []
    assy_prims: List[tuple] = []
    error = None
    k_world = 0
    try:
        if h:
            h.start()
        for si, st in enumerate(stages, start=1):
            t0 = time.time()
            print(f"step {si}: {st.title} ({st.source})", flush=True)
            entry = {"step": si, "title": st.title, "source": st.source, "units": [], "fasteners": len(st.fasteners)}
            scene_placed, scene_parts, scene_prims = placed, placed_parts, assy_prims
            found: Dict[str, dict] = {}
            units = list(st.units)
            if st.bench is not None:
                entry["bench_fixture"] = st.bench
                scene_placed, scene_parts, scene_prims = Placed(), [], []
                for lab in st.bench:
                    scene_placed.add(lab, parts[lab]["tris"])
                    scene_parts.append(parts[lab])
                    scene_prims += collision_prims(parts[lab]["tris"])
                order, stuck = find_order(st.bench, units, parts) if si >= a.check_from else ([(u, u.path, 0) for u in units], [])
                units = []
                for u, path, tried in order:
                    units.append(Unit(u.name, u.parts, path, u.note))
                    found[u.name] = {"clear_start": True, "blocked": False, "found_by": "disassembly",
                                     "designed_from": list(u.path[-1][0]), "candidates_tried": tried,
                                     "path": [{"from": list(dv), "mm": dist} for dv, dist in path]}
                for u in stuck:
                    units.append(u)
                    found[u.name] = {"blocked": True, "found_by": "disassembly", "blocked_by": ["no way out"],
                                     "blocked_at_mm": None, "note": "no straight or L-shaped way out with the "
                                     "units ordered before it present"}
                entry["bench_order"] = [u.name for u in units]
            for u in units:
                if not u.parts:
                    continue
                row = {"unit": u.name, "parts": u.parts, "path": [{"from": list(dv), "mm": dist} for dv, dist in u.path],
                       "note": u.note}
                quick = si < a.check_from
                row["exact_path"] = ({"blocked": False, "skipped": True} if quick else found[u.name] if u.name in found
                                     else exact_path(scene_placed, u, parts))
                row["path"] = [{"from": list(dv), "mm": dist} for dv, dist in u.path]
                row["at_seat"] = ({"touching": [], "threaded": [], "interference": []} if quick
                                  else scene_placed.at_seat(u.parts, parts))
                if h and not quick:
                    k_world += 1
                    row["newton"] = place_on_newton(h, u, parts, scene_parts, scene_prims, written, k_world)
                for lab in u.parts:
                    scene_placed.add(lab, parts[lab]["tris"])
                    scene_parts.append(parts[lab])
                    scene_prims += collision_prims(parts[lab]["tris"])
                nw = row.get("newton", {})
                ep = row["exact_path"]
                print(f"  {u.name}: path {('BLOCKED at ' + str(ep.get('blocked_at_mm')) + ' mm by ' + str(ep.get('blocked_by')) + ('; alternative ' + str([q['from'] for q in ep['clear_alternative']]) if ep.get('clear_alternative') else '; no alternative')) if ep.get('blocked') else 'clear'}"
                      f", seat interference {[(r['with'], r['depth_mm']) for r in row['at_seat']['interference']]}"
                      + (f", newton: {nw.get('released')} ({nw.get('seat_error_mm')} mm), on {sorted(nw.get('rests_on', {}))}"
                         + (f", blocked {nw['physics_blocked_at_mm']} mm by {nw['physics_blocked_by']}" if nw.get('physics_blocked_at_mm') else "")
                         if nw else ""), flush=True)
                entry["units"].append(row)
            fast = ({"touching": [], "threaded": [], "interference": []} if si < a.check_from
                    else scene_placed.at_seat(st.fasteners, parts))
            entry["fastener_check"] = {"threaded": len(fast["threaded"]), "touching": len(fast["touching"]),
                                       "interference": fast["interference"]}
            for lab in st.fasteners:
                scene_placed.add(lab, parts[lab]["tris"])
                scene_parts.append(parts[lab])
            if h and si >= a.check_from:
                entry["screenshot"] = stage_picture(h, scene_parts, scene_prims, si, written)
            entry["wall_s"] = round(time.time() - t0, 1)
            log["steps"].append(entry)
        if h and not a.until and a.check_from <= len(stages):
            log["standing_check"] = standing_check(h, placed_parts, assy_prims, written)
    except Exception as e:  # noqa: BLE001
        import traceback
        error = f"{type(e).__name__}: {e}"
        traceback.print_exc()
    finally:
        if h:
            h.stop()
        for w in written:
            w.unlink(missing_ok=True)
            (w.parent / f".harness_{w.name}").unlink(missing_ok=True)
            (w.parent / f"..harness_{w.stem}.omniperspective").unlink(missing_ok=True)
    log["summary"] = summary(log)
    log["error"] = error
    log["wall_s"] = round(time.time() - t_all, 1)
    (OUT / "assembly_log.json").write_text(json.dumps(log, indent=1, default=_json), encoding="utf-8")
    print(json.dumps(log["summary"], indent=1, default=_json))
    print(f"wrote {OUT / 'assembly_log.json'} ({log['wall_s']} s)")
    return 0 if error is None else 1


def _json(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    return str(o)


def summary(log: dict) -> dict:
    rests, shifts, falls, blocked, interf = [], [], [], [], []
    for st in log["steps"]:
        for u in st["units"]:
            nw = u.get("newton") or {}
            tag = f"{st['step']}:{u['unit']}"
            if nw.get("released") == "rests":
                rests.append(tag)
            elif nw.get("released") == "shifts":
                shifts.append(f"{tag} ({nw['seat_error_mm']} mm)")
            elif nw.get("released") == "falls":
                falls.append(f"{tag} ({nw['seat_error_mm']} mm)")
            if u["exact_path"].get("blocked"):
                alt = u["exact_path"].get("clear_alternative")
                blocked.append(f"{tag} at {u['exact_path']['blocked_at_mm']} mm by {u['exact_path']['blocked_by']}; "
                               + (f"clear: in from {alt[0]['from']} then the last {alt[1]['mm']} mm from {alt[1]['from']}" if alt else
                                  "no clear L-shaped alternative"))
            for r in u["at_seat"]["interference"]:
                interf.append(f"{tag}: {r['part']} x {r['with']} ({r['depth_mm']} mm)")
        for r in st.get("fastener_check", {}).get("interference", []):
            interf.append(f"{st['step']}: {r['part']} x {r['with']} ({r['depth_mm']} mm)")
    return {"units": sum(len(s["units"]) for s in log["steps"]), "rests": len(rests), "shifts": shifts, "falls": falls,
            "path_blocked": blocked, "interference_at_seat": interf,
            "standing": (log.get("standing_check") or {}).get("pass")}


if __name__ == "__main__":
    raise SystemExit(main())
