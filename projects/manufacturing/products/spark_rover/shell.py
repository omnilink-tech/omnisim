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

"""SparkRover Mini: the shell, designed against skeleton.py and printed on the SV08 twin.

What it is
----------
A warm white rounded box for the trunk and head (the UPPER body), a dark grey base that
roofs the four wheels (the LOWER body), a dark grey TPU bumper front and rear with a
translucent diffuser for the status LEDs, teal accents round the shoulders and the top
slot, a clear face cover, a removable rear hatch. Every made part is a closed solid built
with OpenCascade (ruled lofts of rounded sections, then booleans) and tessellated.

The sections and why they are split where they are
---------------------------------------------------
The split is set by how the shell goes ON, not by the bed (each body fits the SV08's
350 x 350 x 345 whole):

- The upper body's low mounts (skeleton.SHELL_MOUNTS upper_*_low, x 0, y +-55, z 110) sit
  directly under the shoulder servos (arms.shoulder_*, z 120-190). A boss there cannot be
  lowered past the servo, so the upper body cannot go on from above in one piece. It is
  split LEFT / RIGHT on y = 0 and each half slides in sideways onto the spine, its two
  bosses meeting the spine's faces square on.
- The lower mounts (x +-75, y +-60, z 50, screw axis -z) put the thread UNDER the chassis
  deck's corners. A lug under the deck cannot come down past it either, so the lower body
  is split FRONT / REAR on x = 0 and each half slides in along x, its lugs passing under
  the deck corners. Its screws are driven from above before the upper body goes on.

Print orientations (each chosen for no support, see checks()["overhang"]):

- upper halves lie on their SIDE face. The top, front and back become vertical walls; the
  face window, camera bar, hatch, vent and top-slot openings all run out to the y = 0 seam,
  which is the top of the print, so none of them is a bridge; the bosses are vertical.
  The side face's edges touch the bed, so their profile is a "printable fillet": a round
  down to 45 degrees, then a 45 degree chamfer to the bed (`Profile`).
- lower halves stand on their END face (front or back). The deck, the wheel roofs, the
  lugs and the side tongues are all vertical extrusions; the end face's edges have the
  printable fillet.
- bumpers lie on their back face, the hatch on its outer face, the socket upright, rings
  and diffusers flat.

Joints (each screw is a bought part, its length from desk_robot's screw_length())
-------------------------------------------------------------------------------
- upper halves: a 1.5 mm tongue on the right half's seam, inside the left half's walls;
  the halves are then clamped across the seam by the top-slot socket (4 M3 into inserts,
  2 per half), the rear hatch (4 M3, 2 per half) and the face cover (bonded), so no screw
  needs a hand inside the finished robot.
- lower halves: on each side a tongue from the front half under a boss of the rear half,
  one M3 from below, between the wheels.
- upper on lower: 2.3 mm tongues on the lower body's deck, inside the upper body's side
  walls (location only; the upper body is held by the spine mounts).
- bumper: 2 M2 x 12 per segment from below, thread-forming into the lower body's sill.

Walls, mass and what is carried
-------------------------------
Walls are 2.0 mm (SHELL_WALL says 2.5: at 2.5 the shell was ~580 g), thickened where
something is held: 3.0 round the face cover, a land behind the hatch, the port wall, bosses,
pads; two ribs up each side wall and two under the top. Inside: the camera and the screen
behind one clear cover; a 1.2 mm touch skin thinned from outside so the inside is flat for
the copper-tape pads; a 3 mm acoustic duct from each mic hole down to the mic (z 262); the
LED strip's channel in the bumper (4.1 mm behind a flush diffuser); recessed USB-C and power
button on the lower rear face with a shelf and posts (M2 inserts) for their boards; the
cooler duct's flat seat ring round REAR_INLET on the hatch. skeleton_issues() lists what
skeleton.py should change.

Units: mm and g. Robot frame as skeleton.py. Print frames: z up from the bed, the part
centred on x = y = 0 (cura_slice.py centres it anyway). STL jobs are binary, in mm.

    python projects/manufacturing/products/spark_rover/shell.py --out .tmp/spark_rover_shell
"""
from __future__ import annotations

import argparse
import functools
import json
import math
import struct
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
from shapely import affinity
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import skeleton as sk  # noqa: E402

# The desk robot's helpers (read-only): rounded_rect, write_stl's format, screw_length and
# the ISO 4762 table, the CNC G-code writer. Appended, so a module of the same name here wins.
_DESK = HERE.parent / "desk_robot"
if str(_DESK) not in sys.path:
    sys.path.append(str(_DESK))
import bought_parts as desk_bought  # noqa: E402
import design as desk  # noqa: E402
import toolpath  # noqa: E402  (desk_robot.design puts manufacturing/cnc on the path)

from OCP.BRep import BRep_Tool  # noqa: E402
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse  # noqa: E402
from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace,  # noqa: E402
                                BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeWire)
from OCP.BRepClass3d import BRepClass3d_SolidClassifier  # noqa: E402
from OCP.BRepGProp import BRepGProp  # noqa: E402
from OCP.BRepMesh import BRepMesh_IncrementalMesh  # noqa: E402
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections  # noqa: E402
from OCP.BRepPrimAPI import (BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCone,  # noqa: E402
                             BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakePrism)
from OCP.GC import GC_MakeArcOfCircle, GC_MakeSegment  # noqa: E402
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Vec  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain  # noqa: E402
from OCP.TopAbs import TopAbs_FACE, TopAbs_IN, TopAbs_ON, TopAbs_REVERSED, TopAbs_SOLID  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from OCP.TopLoc import TopLoc_Location  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402
from OCP.TopTools import TopTools_ListOfShape  # noqa: E402

# ---- materials, bought parts, machines -----------------------------------------------------
PETG_G_CM3 = 1.27
TPU_G_CM3 = 1.21
PMMA_G_CM3 = 1.19            # cast acrylic, typical (1.18-1.19)
BRASS_G_CM3 = 8.5
STEEL_G_CM3 = 7.85
COLOURS = {"warm_white": "#F4F1ED", "teal": "#26B6A4", "dark_grey": "#5A5F66",
           "translucent": "natural (uncoloured) PETG", "clear": "clear"}

SV08 = json.loads((HERE.parents[1] / "machines/sovol_sv08/sovol_sv08.json").read_text(encoding="utf-8"))
# OrcaSlicer printable_area 350 x 350, printable_height 345 (the profile's own "source" line).
BED_MM = (float(SV08["bed"]["size_mm"][0]), float(SV08["bed"]["size_mm"][1]), 345.0)

# A brass heat-set insert, M3 x 5.7, OD 4.6 (ruthex RX-M3x5.7; CNC Kitchen's M3 "standard" is
# the same size). Both vendors give a 4.0 mm hole. The hole is 0.8 mm deeper than the insert:
# room for the PETG it displaces and the screw's tip. The boss leaves 2.0 mm of PETG round
# the hole (an 8 mm boss), over our 1.2 mm wall rule, because the knurl pushes into it.
INSERT = {"spec": "brass heat-set insert M3 x 5.7, OD 4.6 (ruthex RX-M3x5.7 or CNC Kitchen M3 standard)",
          "length": 5.7, "od": 4.6, "hole_d": 4.0, "hole_depth": 6.5, "boss_d": 8.0,
          "source": "https://www.ruthex.de (RX-M3x5.7: hole 4.0 mm); https://cnckitchen.store (M3 standard insert)"}
INSERT["g"] = round(math.pi / 4 * (INSERT["od"] ** 2 - 2.5 ** 2) * INSERT["length"] / 1000 * BRASS_G_CM3, 2)
# The port boards' M2 inserts: brass M2 x 4.0, OD 3.6 (ruthex RX-M2x4: hole 3.2 mm); 1.2 of
# PETG round the hole (a 5.6 boss): at the button board a bigger boss would meet the cap.
INSERT_M2 = {"spec": "brass heat-set insert M2 x 4.0, OD 3.6 (ruthex RX-M2x4)", "length": 4.0, "od": 3.6,
             "hole_d": 3.2, "hole_depth": 4.5, "boss_d": 5.6, "source": "https://www.ruthex.de (RX-M2x4: hole 3.2 mm)"}
INSERT_M2["g"] = round(math.pi / 4 * (INSERT_M2["od"] ** 2 - 1.6 ** 2) * INSERT_M2["length"] / 1000 * BRASS_G_CM3, 2)
M2_PILOT_MM = desk.M2_TAP_PLA_MM      # an M2 screw forms its own thread in a 1.7 mm printed hole (desk robot)
CLEARANCE = desk.CLEARANCE            # ISO 273 normal fit
# ISO 10642 countersunk M3: head diameter 6.72, 90 degree head (fasteners.eu tables).
CSK_M3 = {"dk": 6.72, "k": 1.86}
LASER_NOTE = ("the LongMill's laser is a 445 nm diode (Sienci LaserBeam): clear acrylic passes that light "
              "and does not cut, so the clear cover is routed on the LongMill instead")
WALL_RULE_MM = 1.2
MASS_CEILING_G = 550.0       # the integrator: shell plus bumpers under ~550 g (the robot's 2.4 kg ceiling)


@dataclass(frozen=True)
class Shell:
    """The shell's own numbers. Everything else comes from skeleton.py."""
    # 2.0, not SHELL_WALL's 2.5: the robot is over its mass target, and the walls are
    # thickened where something is held (cover frame 3.0, hatch land, bosses, pads). The
    # bodies are closed rounded boxes, stiff in shape; 2.0 PETG is 5 perimeters of 0.4.
    wall: float = 2.0
    gap: float = sk.SHELL_GAP
    # Upper body (x +-80, y +-82, z 85-272). Its side faces print on the bed, so the edges
    # round those faces are printable fillets; the top-to-front and top-to-back edges are
    # plain rounds (vertical in the print).
    ub_r_vert: float = 10.0         # front/back-to-side edges. At most 10.4: its inside must clear the
                                    # head zone's corner (x 75, y +-76) by SHELL_GAP. The face is
                                    # flat to |y| 72; the cover (window +-66, land 2) needs +-68
    ub_r_side_top: float = 16.0     # top-to-side edges
    ub_r_top: float = 14.0          # top-to-front/back. At most 16.3: its inside must clear the head
                                    # zone's top front corner (75, 262) by SHELL_GAP. The face is
                                    # then flat to z 258 (the cover's top is 255) and the top from
                                    # x 66 (the mics are at 62)
    # Lower body (x +-92.5, y +-105, z 45-85 for the shell, 15-45 the bumper)
    lb_r_plan: float = 20.0         # its corners in plan
    lb_r_end: float = 8.0           # deck-to-front/back edge (the deck ring in front of the face is 12.5)
    lb_r_side: float = 8.0          # deck-to-side edge over the wheels (the side roof is 23 wide)
    # Joints
    ub_tongue: float = 5.0          # the right upper half's tongue reaches 5 mm into the left
    ub_tongue_t: float = 1.5        # and is 1.5 thick (the head and chest zones are 3.0 inside the
                                    # skin's inner face, less the 1.0 gap)
    lb_tongue_h: float = 10.0       # the deck's tongues inside the upper side walls
    lb_tongue_t: float = 2.3
    rib_t: float = 1.6
    slide_clear: float = 0.2        # every sliding fit (tongue in wall, hatch, cover, ring)
    # Face and touch
    cover_t: float = 1.5            # clear cast acrylic, 1.5 mm
    cover_land: float = 2.0         # bonded 2 mm wide; 1 mm of face is left above the touch area
    touch_skin: float = 1.2         # the brief: 1.2 mm over the capacitive pads, thinned from OUTSIDE
                                    # (a 0.8 mm recess) so the inside stays flat for copper-tape pads
    # Rear
    hatch_t: float = 2.5
    hatch_land: float = 9.0         # the hatch overlaps its opening by 9: room for the inserts
    hatch_corner_r: float = 8.0
    vent_z: float = 176.0           # skeleton REAR_VENT centre 168 overlaps the hatch (z 96-164): moved up
    vent_slot_w: float = 2.5
    inlet_rib_w: float = 1.2        # two ribs across REAR_INLET (a finger guard); it stays >= 800 mm2 open
    inlet_seat: Tuple[float, float] = (69.2, 23.2)   # the flat ring the cooler duct's gasket seals on
                                    # (electrical: duct end 63.2 x 17.2 at x -75, 3 mm foam)
    inlet_land_min: float = 2.0     # the shell's land under the hatch, below that ring
    # Speaker, mics
    speaker_slot_w: float = 2.0
    mic_d: float = 1.6
    mic_duct_d: float = 3.0
    mic_duct_wall: float = 1.9     # the bore is a teardrop (its point 2.1 out): 1.3 of wall there
    # Top slot socket
    socket_insert_xy: Tuple[float, float] = (8.0, 22.0)
    socket_floor_t: float = 4.0
    socket_wire_hole: Tuple[float, float] = (12.0, 24.0)   # the header's wires through the top (x, y)
    header_slot: Tuple[float, float] = (5.4, 13.0)        # a 2 x 5 female 2.54 mm header, 5.08 x 12.7, +0.3
    # Bumper (TPU)
    bumper_back_wall: float = 1.2
    bumper_skin: float = 1.2        # also the diffuser's thickness, flush in the light window
    strip_clear: float = 4.1        # clear behind the diffuser: electrical's strip is 12.5 x 4 (sleeved)
    channel_h: float = 13.2         # the strip's 12.5 and 0.35 a side
    strip_w: float = 10.0           # the light window's height (a 5050 LED is 5 mm)
    led_window_half: float = 56.0   # |y| of the light window's ends: the diffuser runs 2.5 past them,
                                    # the bumper screws' heads (in the channel's line) 1.6 past that
    # Shoulder accent rings (teal)
    ring_od: float = 52.0

    # ---- what follows from the skeleton --------------------------------------------------
    @property
    def skirt_z0(self) -> float:
        """The lower body's skirt stops SHELL_GAP above the wheel wells (z 72)."""
        return max(_zone("chassis", "wheel_wells").z1, _zone("chassis", "wheel_wells_l").z1) + self.gap

    @property
    def bumper_x_in(self) -> float:
        """The bumper's back face: SHELL_GAP in front of the chassis base zone (x 85)."""
        return _zone("chassis", "base").x1 + self.gap

    @property
    def bumper_t(self) -> float:
        return sk.LOWER_BODY.x1 - self.bumper_x_in

    @property
    def bumper_half_y(self) -> float:
        """The bumper stops SHELL_GAP short of the wheel wells (|y| 70)."""
        return _zone("chassis", "wheel_wells_l").y0 - self.gap

    @property
    def inner_x_front(self) -> float:
        """The nearest anything of the shell may come to the head and chest zones (x 75)."""
        return max(_zone("electrical", "head").x1, _zone("electrical", "chest").x1) + self.gap

    @property
    def inner_x_rear(self) -> float:
        return _zone("electrical", "bay").x0 - self.gap


def _zone(owner: str, name: str) -> sk.Box:
    return dict(sk.ZONES[owner])[name]


def config(r=None) -> Shell:
    """The shell's numbers: `r` itself if it is a Shell, else the defaults (an integrator's
    robot object carries no shell numbers today)."""
    if isinstance(r, Shell):
        return r
    sub = getattr(r, "shell", None)
    return sub if isinstance(sub, Shell) else Shell()


# ---- the printable fillet ------------------------------------------------------------------

@dataclass(frozen=True)
class Profile:
    """An edge that touches the print bed: a round of radius R from the wall down to 45
    degrees, then a 45 degree chamfer to the bed face, so no layer overhangs the one under
    it by more than 45 degrees. u: distance from the bed face into the part; s(u): how far
    the wall is set in from its plane at u. The inner surface (a wall t thick) is the exact
    offset: the chamfer moved in by t, the round's radius less t about the same centre."""
    R: float
    t: float

    @property
    def a(self) -> float:
        return self.R * (1.0 - 1.0 / math.sqrt(2.0))     # where the round meets the chamfer

    def s(self, u: float) -> float:
        R, a = self.R, self.a
        if u <= a:
            return 2.0 * a - u
        if u <= R:
            return R - math.sqrt(max(0.0, R * R - (R - u) ** 2))
        return 0.0

    @property
    def u1(self) -> float:
        return self.R - (self.R - self.t) / math.sqrt(2.0)

    def si(self, u: float) -> float:
        R, t = self.R, self.t
        if u <= self.u1:
            return 2.0 * self.a - u + t * math.sqrt(2.0)
        if u <= R:
            return R - math.sqrt(max(0.0, (R - t) ** 2 - (R - u) ** 2))
        return t

    def knots(self, n: int = 7) -> List[float]:
        return [0.0, self.a] + [self.R - self.R * math.cos(th) for th in np.linspace(math.pi / 4, math.pi / 2, n)[1:]]

    def iknots(self, n: int = 7) -> List[float]:
        return [self.t] + [self.R - (self.R - self.t) * math.cos(th) for th in np.linspace(math.pi / 4, math.pi / 2, n)]


# ---- OpenCascade helpers ---------------------------------------------------------------------

def _pnt(plane: str, a: float, b: float, c: float) -> gp_Pnt:
    """A point of a 2D outline (a, b) in a plane at height c along the plane's normal."""
    if plane == "yz":
        return gp_Pnt(c, a, b)
    if plane == "xz":
        return gp_Pnt(a, c, b)
    return gp_Pnt(a, b, c)


def _normal(plane: str) -> Tuple[float, float, float]:
    return {"yz": (1.0, 0.0, 0.0), "xz": (0.0, 1.0, 0.0), "xy": (0.0, 0.0, 1.0)}[plane]


def _slice(plane: str, at: float, half: float, z0: float, z1: float, r: float):
    """A section wire: |a| <= half, z0 <= b <= z1, the two top corners rounded r, the bottom
    corners square. Always six edges, so any two of them loft."""
    c = math.cos(math.pi / 4)
    a0, a1 = -half, half
    w = BRepBuilderAPI_MakeWire()

    def seg(p, q):
        w.Add(BRepBuilderAPI_MakeEdge(GC_MakeSegment(_pnt(plane, *p, at), _pnt(plane, *q, at)).Value()).Edge())

    def arc(p, m, q):
        w.Add(BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(_pnt(plane, *p, at), _pnt(plane, *m, at),
                                                         _pnt(plane, *q, at)).Value()).Edge())
    seg((a0, z0), (a1, z0))
    seg((a1, z0), (a1, z1 - r))
    arc((a1, z1 - r), (a1 - r + r * c, z1 - r + r * c), (a1 - r, z1))
    seg((a1 - r, z1), (a0 + r, z1))
    arc((a0 + r, z1), (a0 + r - r * c, z1 - r + r * c), (a0, z1 - r))
    seg((a0, z1 - r), (a0, z0))
    return w.Wire()


def _loft(wires):
    ts = BRepOffsetAPI_ThruSections(True, True, 1e-6)
    for wire in wires:
        ts.AddWire(wire)
    ts.Build()
    if not ts.IsDone():
        raise RuntimeError("loft failed")
    return ts.Shape()


def _box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1))
    y0, y1 = sorted((y0, y1))
    z0, z1 = sorted((z0, z1))
    return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), gp_Pnt(x1, y1, z1)).Shape()


def _cyl(at, axis, d, length):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*at), gp_Dir(*axis)), d / 2.0, length).Shape()


def _hole(at, axis, d, depth, up=None):
    """A hole from its mouth `at` along `axis` (a principal axis). `up`: the print's up
    direction when the hole runs horizontally in the print; the hole is then a teardrop
    pointing up, so its roof is two 45 degree faces instead of a flat arc."""
    if up is None:
        return _cyl(at, axis, d, depth)
    k = int(np.argmax(np.abs(axis)))
    plane, ia, ib = {0: ("yz", 1, 2), 1: ("xz", 0, 2), 2: ("xy", 0, 1)}[k]
    poly = teardrop(at[ia], at[ib], d, (up[ia], up[ib]))
    lo = at[k]
    hi = at[k] + axis[k] * depth
    return _prism(poly, plane, min(lo, hi), max(lo, hi))


def _cone(at, axis, d0, d1, length):
    return BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(*at), gp_Dir(*axis)), d0 / 2.0, d1 / 2.0, length).Shape()


def _polys(geom) -> List[Polygon]:
    if isinstance(geom, Polygon):
        return [] if geom.is_empty else [geom]
    return [g for g in getattr(geom, "geoms", []) if isinstance(g, Polygon) and not g.is_empty]


def _prism(poly, plane: str, lo: float, hi: float):
    """A shapely polygon (holes allowed) in `plane` extruded from lo to hi along its normal."""
    n = _normal(plane)
    shapes = []
    for p in _polys(poly):
        def wire(ring):
            mp = BRepBuilderAPI_MakePolygon()
            pts = list(ring.coords)[:-1]
            for a, b in pts:
                mp.Add(_pnt(plane, a, b, lo))
            mp.Close()
            return mp.Wire()
        face = BRepBuilderAPI_MakeFace(wire(p.exterior), True).Face()
        solid = BRepPrimAPI_MakePrism(face, gp_Vec(*(k * (hi - lo) for k in n))).Shape()
        holes = [_prism(Polygon(h), plane, lo - 1.0, hi + 1.0) for h in p.interiors]
        shapes.append(_cut(solid, holes) if holes else solid)
    if len(shapes) == 1:
        return shapes[0]
    return _fuse(shapes[0], shapes[1:])


def _list(shapes) -> TopTools_ListOfShape:
    out = TopTools_ListOfShape()
    for s in shapes:
        out.Append(s)
    return out


def _bool(op, base, tools):
    tools = [t for t in tools if t is not None]
    if not tools:
        return base
    op.SetArguments(_list([base]))
    op.SetTools(_list(tools))
    op.SetFuzzyValue(1e-5)
    op.Build()
    if not op.IsDone():
        raise RuntimeError("boolean failed")
    return op.Shape()


def _fuse(base, tools):
    return _bool(BRepAlgoAPI_Fuse(), base, tools)


def _cut(base, tools):
    return _bool(BRepAlgoAPI_Cut(), base, tools)


def _clean(shape):
    u = ShapeUpgrade_UnifySameDomain(shape, True, True, True)
    u.Build()
    return u.Shape()


def _solids(shape) -> list:
    out = []
    ex = TopExp_Explorer(shape, TopAbs_SOLID)
    while ex.More():
        out.append(TopoDS.Solid_s(ex.Current()))
        ex.Next()
    return out


def volume_mm3(shape) -> float:
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return float(props.Mass())


def centre_mm(shape) -> Tuple[float, float, float]:
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    c = props.CentreOfMass()
    return (round(c.X(), 2), round(c.Y(), 2), round(c.Z(), 2))


def tessellate(shape, lin: float = 0.05, ang: float = 0.25) -> np.ndarray:
    """Triangles (n, 3, 3), mm, outward-wound: OpenCascade meshes each edge once, so the
    faces' triangulations share their boundary vertices and the result is closed."""
    BRepMesh_IncrementalMesh(shape, lin, False, ang, True)
    out = []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        f = TopoDS.Face_s(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is not None:
            T = loc.Transformation()
            nodes = np.empty((tri.NbNodes(), 3))
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(T)
                nodes[i - 1] = (p.X(), p.Y(), p.Z())
            idx = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)], dtype=np.int64) - 1
            if f.Orientation() == TopAbs_REVERSED:
                idx = idx[:, [0, 2, 1]]
            out.append(nodes[idx])
        ex.Next()
    return np.concatenate(out)


# ---- 2D shapes -------------------------------------------------------------------------------

def rrect(w: float, h: float, r: float, cx: float = 0.0, cy: float = 0.0) -> Polygon:
    return desk.rounded_rect(w, h, max(r, 1e-3), cx, cy)


def teardrop(cx: float, cy: float, d: float, toward: Tuple[float, float]) -> Polygon:
    """A circle with a 45 degree point toward `toward` (a unit 2D direction): the shape of
    a boss or hole that runs horizontally in the print, pointing down the print, so its
    underside is never flatter than 45 degrees."""
    r = d / 2.0
    tx, ty = toward
    apex = (cx + tx * r * math.sqrt(2.0), cy + ty * r * math.sqrt(2.0))
    # tangent points at +-45 degrees either side of the direction
    pts = []
    for sgn in (1, -1):
        ang = math.atan2(ty, tx) + sgn * math.pi / 4
        pts.append((cx + r * math.cos(ang), cy + r * math.sin(ang)))
    return Point(cx, cy).buffer(r, quad_segs=24).union(Polygon([pts[0], apex, pts[1], (cx, cy)]))


def pointed_slot(cx: float, cy: float, length: float, width: float) -> Polygon:
    """A slot along a, its ends pointed at 45 degrees (prints in either direction along a)."""
    h, w2 = length / 2.0, width / 2.0
    return Polygon([(cx - h, cy), (cx - h + w2, cy - w2), (cx + h - w2, cy - w2), (cx + h, cy),
                    (cx + h - w2, cy + w2), (cx - h + w2, cy + w2)])


def diamond(cx: float, cy: float, d: float) -> Polygon:
    r = d / math.sqrt(2.0)
    return Polygon([(cx + r, cy), (cx, cy + r), (cx - r, cy), (cx, cy - r)])


# ---- the interfaces as outlines --------------------------------------------------------------

def face_opening() -> Polygon:
    """(y, z): the screen window and the camera bar as ONE opening under one clear cover.
    The bar (z 152-170) ends 1 mm under the window (z 171): a 1 mm web between two holes
    is under the 1.2 mm wall rule and would break, so the gap is closed (a morphological
    close of 1 mm) and the camera and its two LEDs look through the cover."""
    fw = sk.FACE_WINDOW
    (_, y0, z0), (w, h) = fw["centre"], fw["size"]
    win = rrect(w, h, fw["corner_r"], y0, z0)
    (_, cy, cz), (bw, bh) = sk.CAMERA["lens_at"], sk.CAMERA["bar_size"]
    bar = rrect(bw, bh, 4.0, cy, cz)
    gap = max(0.0, (z0 - h / 2) - (cz + bh / 2))
    joined = win.union(bar)
    if gap < WALL_RULE_MM + 0.5:
        joined = joined.union(box(cy - bw / 2, cz, cy + bw / 2, z0 - h / 2 + 1.0))   # the bar's full width: no sliver
    return joined


def face_cover_outline(c: Shell) -> Polygon:
    """(y, z): the clear cover, the opening plus its land."""
    return face_opening().buffer(c.cover_land, quad_segs=16)


def hatch_outline(c: Shell) -> Polygon:
    (_, y0, z0), (w, h) = sk.REAR_HATCH["centre"], sk.REAR_HATCH["size"]
    return rrect(w, h, c.hatch_corner_r, y0, z0)


def hatch_insert_points(c: Shell) -> List[Tuple[float, float]]:
    (_, y0, z0), (w, h) = sk.REAR_HATCH["centre"], sk.REAR_HATCH["size"]
    dy = w / 2.0 - (c.hatch_land / 2.0 + 0.5)          # in the land, 4.5 from the hatch edge
    dz = h / 2.0 - (c.hatch_land / 2.0 + 0.5) - 1.0
    return [(y0 + sy * dy, z0 + sz * dz) for sy in (1, -1) for sz in (-1, 1)]


def socket_insert_points(c: Shell) -> List[Tuple[float, float]]:
    x, y = c.socket_insert_xy
    return [(sx * x, sy * y) for sy in (1, -1) for sx in (-1, 1)]


def ub_inner_footprint(c: Shell, n: int = 60) -> Polygon:
    """(x, y): the upper body's inside at its bottom (z 85): the lower body's deck opening."""
    pv = Profile(c.ub_r_vert, c.wall)
    X1, Y1 = sk.UPPER_BODY.x1, sk.UPPER_BODY.y1
    us = sorted(set(pv.iknots(25) + list(np.linspace(c.wall, pv.R + 1, n)) + [Y1]))
    half = [(Y1 - u, X1 - pv.si(u)) for u in us]                 # (|y|, x half)
    pts = [(x, y) for y, x in half] + [(x, -y) for y, x in reversed(half)]
    pts += [(-x, y) for x, y in reversed(pts)]
    return Polygon(pts).buffer(0)


# ---- the upper body -------------------------------------------------------------------------

def _ub_loft(c: Shell, sy: int, inner: bool, u_end: float):
    pv, pt = Profile(c.ub_r_vert, c.wall), Profile(c.ub_r_side_top, c.wall)
    ub = sk.UPPER_BODY
    if inner:
        us = sorted(set([u for u in pv.iknots() + pt.iknots() if u < u_end] + [u_end]))
        f = lambda u: (ub.x1 - pv.si(u), ub.z0 - 1.0, ub.z1 - pt.si(u), max(0.5, c.ub_r_top - max(pv.si(u), pt.si(u))))
    else:
        us = sorted(set([u for u in pv.knots() + pt.knots() if u < u_end] + [u_end]))
        f = lambda u: (ub.x1 - pv.s(u), ub.z0, ub.z1 - pt.s(u), max(1.0, c.ub_r_top - max(pv.s(u), pt.s(u))))
    return _loft([_slice("xz", sy * (ub.y1 - u), *f(u)) for u in us])


def _ub_half(c: Shell, sy: int):
    """One upper half: sy = +1 left (y >= 0), -1 right."""
    ub = sk.UPPER_BODY
    t = c.wall
    shell = _cut(_ub_loft(c, sy, False, ub.y1), [_ub_loft(c, sy, True, ub.y1 + 2.0)])
    add, cut = [], []
    xr = c.inner_x_rear                                   # -76
    toward_bed = (0.0, float(sy))                         # (x, y): +y is down the print for the left half
    up = (0.0, -float(sy), 0.0)                           # the print's up, robot frame

    # Face: one opening (window + camera bar) under a 1.5 mm clear cover in a rebate. The
    # wall behind the rebate is thickened inward to 3.0 (1.5 under the cover), still 2.0
    # clear of the head zone; its end toward the side face is a 45 degree slope. It stops
    # above the chest touch area, whose inside stays flat.
    opening = face_opening()
    cover = face_cover_outline(c)
    yb0, zb0, yb1, zb1 = cover.bounds
    ylim = max(abs(yb0), abs(yb1)) + 2.0
    th_x0 = ub.x1 - 2 * c.cover_t                         # 77.0
    ramp = ub.x1 - t - th_x0 + 0.1
    thick = Polygon([(th_x0, 0.0), (th_x0, sy * ylim), (ub.x1 - t + 0.1, sy * (ylim + ramp)), (ub.x1 - t + 0.1, 0.0)])
    (_, ty, tz), (tw, th) = sk.CHEST_TOUCH["centre"], sk.CHEST_TOUCH["size"]
    add.append(_prism(thick, "xy", max(zb0 - 2.0, tz + th / 2 + 0.5), zb1 + 2.0))
    cut.append(_prism(opening, "yz", ub.x1 - 10.0, ub.x1 + 1.0))
    cut.append(_prism(cover.buffer(c.slide_clear / 2.0), "yz", ub.x1 - c.cover_t, ub.x1 + 1.0))

    # Chest touch: the wall thinned to 1.2 from OUTSIDE (a 0.8 mm recess, the touch panel the
    # finger finds), so the inside is one flat plane for the copper-tape pads.
    # (carried up to the cover's rebate: the 1 mm of face between them would be a sliver)
    rec = rrect(tw, th, 3.0, ty, tz).union(box(ty - tw / 2, tz, ty + tw / 2, cover.bounds[1] + 0.5))
    cut.append(_prism(rec, "yz", ub.x1 - (t - c.touch_skin), ub.x1 + 1.0))

    # Speaker grille: six slots along y, pointed ends, 4.5 pitch.
    (_, gy, gz), (gw, gh) = sk.SPEAKER_GRILLE["centre"], sk.SPEAKER_GRILLE["size"]
    pitch = gh / 6.0
    slots = unary_union([pointed_slot(gy, gz - gh / 2 + pitch * (k + 0.5), gw, c.speaker_slot_w) for k in range(6)])
    cut.append(_prism(slots, "yz", ub.x1 - 10.0, ub.x1 + 1.0))

    # Mics: a diamond hole through the top (corners along x and y: 45 degree faces in the
    # print), then an acoustic duct down to the mic on the head zone's top (z 262): a tube
    # whose outside points down the print and whose 3 mm bore points up it.
    head = _zone("electrical", "head")
    for x, y, _ in sk.MIC_HOLES:
        if y * sy > 0:
            cut.append(_prism(diamond(x, y, c.mic_d), "xy", ub.z1 - t - 0.5, ub.z1 + 1.0))
            add.append(_prism(teardrop(x, y, c.mic_duct_d + 2 * c.mic_duct_wall, toward_bed), "xy", head.z1, ub.z1 - t + 0.3))
            cut.append(_hole((x, y, head.z1 - 0.01), (0, 0, 1), c.mic_duct_d, ub.z1 - t - head.z1 + 0.02, up))

    # Top slot: the header's wire hole and four insert pads under the top wall, each pointed
    # down the print (toward the side face).
    wx, wy = c.socket_wire_hole
    cut.append(_box(-wx / 2, wx / 2, -wy / 2, wy / 2, ub.z1 - 10.0, ub.z1 + 1.0))
    spine = _zone("chassis", "spine")
    pad_z0 = spine.z1 + c.gap                              # 263: 1 clear of the spine's top
    for x, y in socket_insert_points(c):
        if y * sy > 0:
            add.append(_prism(teardrop(x, y, INSERT["boss_d"], toward_bed), "xy", pad_z0, ub.z1 - t + 0.5))
            cut.append(_hole((x, y, ub.z1 + 0.01), (0, 0, -1), INSERT["hole_d"], INSERT["hole_depth"] + 0.01, up))

    # Rear hatch: a rebate the hatch fills flush, a land 1.5 thick behind it (thickened in
    # to x -76), four insert bosses (the bay zone starts at x -75: they reach into it, see
    # CLAIMS), the access opening inside the land.
    hatch = hatch_outline(c)
    hyb0, hzb0, hyb1, hzb1 = hatch.bounds
    hylim = max(abs(hyb0), abs(hyb1)) + 2.0
    land_x = ub.x0 + c.hatch_t                            # -77.5
    ramp = xr - (ub.x0 + t - 0.1)                      # 1.6
    land = Polygon([(xr, 0.0), (xr, sy * hylim), (ub.x0 + t - 0.1, sy * (hylim + ramp)), (ub.x0 + t - 0.1, 0.0)])
    add.append(_prism(land, "xy", hzb0 - 2.0, hzb1 + c.slide_clear / 2 + 1.3))   # 1.3 past the rebate: no sliver
    boss_x1 = hatch_boss_x1(c)                            # -69.7: 1.3 of floor past the hole
    for y, z in hatch_insert_points(c):
        if y * sy > 0:
            add.append(_prism(teardrop(y, z, INSERT["boss_d"], (float(sy), 0.0)), "yz", xr - 0.5, boss_x1))
            cut.append(_hole((land_x - 0.01, y, z), (1, 0, 0), INSERT["hole_d"], INSERT["hole_depth"] + 0.01, up))
    cut.append(_prism(hatch.buffer(c.slide_clear / 2.0), "yz", ub.x0 - 1.0, land_x))
    (_, hy, hz), (hw, hh) = sk.REAR_HATCH["centre"], sk.REAR_HATCH["size"]
    access = rrect(hw - 2 * c.hatch_land, hh - 2 * c.hatch_land, 4.0, hy, hz).union(inlet_seat_rect(c).buffer(0.5, join_style=2))
    cut.append(_prism(access, "yz", ub.x0 - 1.0, xr + 0.5))

    # Rear vent: four slots along y, above the hatch.
    (_, vy, _), (vw, vh) = sk.REAR_VENT["centre"], sk.REAR_VENT["size"]
    vpitch = vh / 4.0
    vslots = unary_union([pointed_slot(vy, c.vent_z - vh / 2 + vpitch * (k + 0.5), vw, c.vent_slot_w) for k in range(4)])
    cut.append(_prism(vslots, "yz", ub.x0 - 1.0, ub.x0 + 10.0))

    # Shoulder: the seat of the teal ring, whose bore is SHELL_ARM_OPENING_D.
    sx_, sy_, sz_ = sk.SHOULDERS["left" if sy > 0 else "right"]
    cut.append(_cyl((sx_, sy * (ub.y1 - 10.0), sz_), (0, sy, 0), c.ring_od + c.slide_clear, 12.0))

    # Upper mounts: bosses from the spine's face (y +-55) out to the side wall, vertical in
    # the print; the insert's hole opens on the spine face.
    for m in sk.SHELL_MOUNTS:
        if m.name.startswith("upper_") and m.axis[1] * sy > 0:
            x, y, z = m.at
            add.append(_cyl((x, y, z), (0, sy, 0), INSERT["boss_d"], ub.y1 - t - abs(y) + 0.5))
            cut.append(_cyl((x, y - sy * 0.01, z), (0, sy, 0), INSERT["hole_d"], INSERT["hole_depth"] + 0.01))

    # Ribs (vertical in the print): two up each side wall, kept SHELL_GAP off the head and
    # bay zones and above the lower body's tongues, and two across the top, under it.
    for xr_, depth in UB_SIDE_RIBS:
        add.append(_box(xr_ - c.rib_t / 2, xr_ + c.rib_t / 2, sy * (ub.y1 - t - depth), sy * (ub.y1 - t + 0.3),
                        sk.LOWER_BODY.z1 + c.lb_tongue_h + 1.5, UB_RIB_TOP))
    for xr_ in UB_TOP_RIBS_X:
        z0, z1 = ub.z1 - t - UB_TOP_RIB_DEPTH, ub.z1 - t + 0.3
        rib = Polygon([(sy * (c.ub_tongue + 0.5), z0), (sy * UB_TOP_RIB_Y1, z0), (sy * (UB_TOP_RIB_Y1 + z1 - z0), z1),
                       (sy * (c.ub_tongue + 0.5), z1)])
        add.append(_prism(rib, "yz", xr_ - c.rib_t / 2, xr_ + c.rib_t / 2))

    shell = _fuse(shell, add)
    shell = _cut(shell, cut)
    if sy < 0:
        shell = _fuse(shell, [_ub_tongue(c, (zb0, zb1), (hzb0, hzb1))])
        shell = _cut(shell, cut)
    return _clean(shell)


# Ribs. Side: (x, depth): at x 40 the head zone (|y| <= 76) leaves 3 under the 1.0 gap, at
# x -45 (beside the bay, |y| <= 55) 5. Top: across y at x +-30, 5 deep (the spine and head
# end at z 262), stopping at |y| 60 with a 45 degree end (it would leave the top's round).
UB_SIDE_RIBS = ((40.0, 3.0), (-45.0, 5.0))
UB_RIB_TOP = 255.0
UB_TOP_RIBS_X = (-30.0, 30.0)
UB_TOP_RIB_DEPTH = 5.0
UB_TOP_RIB_Y1 = 60.0


def inlet_z(c: Shell) -> float:
    """REAR_INLET's centre: the skeleton's, raised (to the next 0.5 mm) until the duct's
    seat ring round it lies on the hatch inside the access opening, with inlet_land_min of
    the shell's land left under the hatch below it."""
    (_, _, hz), (_, hh) = sk.REAR_HATCH["centre"], sk.REAR_HATCH["size"]
    need = hz - hh / 2 + c.slide_clear / 2 + c.inlet_land_min + c.inlet_seat[1] / 2
    return max(sk.REAR_INLET["centre"][2], math.ceil(need * 2.0) / 2.0)


def inlet_seat_rect(c: Shell) -> Polygon:
    iy = sk.REAR_INLET["centre"][1]
    w, h = c.inlet_seat
    return box(iy - w / 2, inlet_z(c) - h / 2, iy + w / 2, inlet_z(c) + h / 2)


def inlet_rect(c: Shell) -> Polygon:
    (_, iy, _), (iw, ih) = sk.REAR_INLET["centre"], sk.REAR_INLET["size"]
    return box(iy - iw / 2, inlet_z(c) - ih / 2, iy + iw / 2, inlet_z(c) + ih / 2)


def inlet_open(c: Shell) -> Polygon:
    """(y, z): REAR_INLET in the hatch, crossed by two ribs (a finger guard)."""
    (_, iy, _), (iw, _) = sk.REAR_INLET["centre"], sk.REAR_INLET["size"]
    r = inlet_rect(c)
    for k in (-1, 1):
        yc = iy + k * iw / 6.0
        r = r.difference(box(yc - c.inlet_rib_w / 2, -1e3, yc + c.inlet_rib_w / 2, 1e3))
    return r


def hatch_boss_x1(c: Shell) -> float:
    return sk.UPPER_BODY.x0 + c.hatch_t + INSERT["hole_depth"] + 1.3


def _touch_keepout(c: Shell):
    """Behind CHEST_TOUCH the inside stays flat: nothing stands on it."""
    (_, ty, tz), (tw, th) = sk.CHEST_TOUCH["centre"], sk.CHEST_TOUCH["size"]
    return _box(sk.UPPER_BODY.x1 - 10.0, sk.UPPER_BODY.x1 + 1.0, ty - tw / 2 - 1, ty + tw / 2 + 1, tz - th / 2 - 1, tz + th / 2 + 1)


def _ub_inset_slice(c: Shell, y: float, inset: float):
    """The upper body's section at y (away from the side faces), set in by `inset`, carried
    below z 85 (the ring it bounds is cut off at z 85)."""
    ub = sk.UPPER_BODY
    return _slice("xz", y, ub.x1 - inset, ub.z0 - 10.0, ub.z1 - inset, max(0.5, c.ub_r_top - inset))


def _ub_tongue(c: Shell, cover_z: Tuple[float, float], hatch_z: Tuple[float, float]):
    """The right half's seam tongue: ub_tongue_t inside the left half's walls, 0.1 clear of
    them, reaching ub_tongue past the seam. On the right half's side it grows out of the
    wall at 45 degrees (it prints above the wall, the seam being the print's top). It
    stops where the left half's walls are thickened (round the cover and the hatch)."""
    t, T, ub = c.wall, c.ub_tongue_t, sk.UPPER_BODY
    y0 = -(T + 0.1) * 1.1                                 # the wedge's face a little steeper than 45
    wedge = _cut(_loft([_ub_inset_slice(c, y0, t - 0.1), _ub_inset_slice(c, 0.0, t - 0.1)]),
                 [_loft([_ub_inset_slice(c, y0 - 0.01, t - 0.1), _ub_inset_slice(c, 0.0, t + T)]),
                  _box(ub.x0 - 5, ub.x1 + 5, y0 - 1, 1, ub.z0 - 20, ub.z0)])
    tongue = _cut(_loft([_ub_inset_slice(c, 0.0, t + 0.1), _ub_inset_slice(c, c.ub_tongue, t + 0.1)]),
                  [_loft([_ub_inset_slice(c, -0.1, t + T), _ub_inset_slice(c, c.ub_tongue + 0.1, t + T)]),
                   _box(ub.x0 - 5, ub.x1 + 5, -1, c.ub_tongue + 1, ub.z0 - 20, ub.z0),
                   _box(0.0, ub.x1 + 1, -0.01, c.ub_tongue + 1, cover_z[0] - 2.1, cover_z[1] + 2.1),
                   _box(ub.x0 - 1, 0.0, -0.01, c.ub_tongue + 1, hatch_z[0] - 2.1, hatch_z[1] + 2.1),
                   _touch_keepout(c)])
    # and it keeps SHELL_GAP from the zones inside (the head zone's top front corner sits
    # 1.3 inside the wall's inner round, less than the tongue's thickness)
    g = c.gap
    keep = [_box(b.x0 - g, b.x1 + g, b.y0 - g, b.y1 + g, b.z0 - g, b.z1 + g)
            for owner in ("electrical", "chassis") for _, b in sk.ZONES[owner]]
    # where the head zone's top front corner cuts the tongue inside the wall's curved corner,
    # it would leave a sliver: there the tongue goes altogether (its thickness more)
    hb, e = _zone("electrical", "head"), g + T + 0.1
    keep.append(_box(hb.x0, hb.x1 + e, hb.y0 - g, hb.y1 + g, hb.z0, hb.z1 + e))
    return _cut(_fuse(wedge, [tongue]), keep)


# ---- the lower body -------------------------------------------------------------------------

def _lb_loft(c: Shell, sx: int, inner: bool, u_end: float):
    pc, pe = Profile(c.lb_r_plan, c.wall), Profile(c.lb_r_end, c.wall)
    lb = sk.LOWER_BODY
    if inner:
        us = sorted(set([u for u in pc.iknots() + pe.iknots() if u < u_end] + [u_end]))
        f = lambda u: (lb.y1 - pc.si(u), c.skirt_z0 - 1.0, lb.z1 - pe.si(u), max(0.5, c.lb_r_side - max(pc.si(u), pe.si(u))))
    else:
        us = sorted(set([u for u in pc.knots() + pe.knots() if u < u_end] + [u_end]))
        f = lambda u: (lb.y1 - pc.s(u), c.skirt_z0, lb.z1 - pe.s(u), max(1.0, c.lb_r_side - max(pc.s(u), pe.s(u))))
    return _loft([_slice("yz", sx * (lb.x1 - u), *f(u)) for u in us])


def _mx(poly: Polygon, sx: int) -> Polygon:
    """Mirror an (x, *) outline to the rear half."""
    return poly if sx > 0 else affinity.scale(poly, xfact=-1.0, yfact=1.0, origin=(0, 0))


def lug_profile(c: Shell) -> Polygon:
    """(x, z), front: a lower mount's lug. The deck rests on its top at the mount's z; the
    insert opens there. It reaches back from the sill; under the bumper (x > 86) it keeps
    above BUMPER_Z, and the step down to its full depth is at 45 degrees (it prints
    standing on the end face, x vertical)."""
    m = next(m for m in sk.SHELL_MOUNTS if m.name == "lower_fl")
    x, _, z = m.at
    r = INSERT["boss_d"] / 2.0
    z_bot = z - INSERT["hole_depth"] - 2.5                 # 41: 2.5 of floor under the hole
    zb = sk.BUMPER_Z[1]
    x_step = x + r
    return Polygon([(x - r, z_bot), (x_step, z_bot), (x_step + (zb - z_bot), zb), (sk.LOWER_BODY.x1 - c.wall + 0.5, zb),
                    (sk.LOWER_BODY.x1 - c.wall + 0.5, z), (x - r, z)])


def _lb_half(c: Shell, sx: int):
    """One lower half: sx = +1 front (x >= 0), -1 rear."""
    lb = sk.LOWER_BODY
    t = c.wall
    seam = LB_SEAM_GAP / 2.0                              # the halves meet 0.1 apart, not face to face
    shell = _cut(_lb_loft(c, sx, False, lb.x1 - seam), [_lb_loft(c, sx, True, lb.x1 + 3.0),
                                                 _prism(ub_inner_footprint(c), "xy", lb.z1 - 5.0, lb.z1 + 5.0)])
    add, cut = [], []
    zb = sk.BUMPER_Z[1]
    hy = c.bumper_half_y
    up = (-float(sx), 0.0, 0.0)                           # the print's up (it stands on its end face)
    # The end panel down to the bumper, and the sill the bumper screws into.
    add.append(_box(sx * (lb.x1 - t), sx * lb.x1, -hy, hy, zb, c.skirt_z0 + 1.0))
    sill_z1 = zb + 8.0                                    # the M2 pilot (6 deep) and 2 of floor
    add.append(_box(sx * c.bumper_x_in, sx * lb.x1, -hy, hy, zb, sill_z1))
    for y in bumper_screw_y(c):
        cut.append(_hole((sx * bumper_screw_x(c), y, zb - 0.01), (0, 0, 1), M2_PILOT_MM, 6.01, up))
    # Lugs for the lower mounts.
    lug = _mx(lug_profile(c), sx)
    for m in sk.SHELL_MOUNTS:
        if m.name.startswith("lower_") and m.at[0] * sx > 0:
            x, y, z = m.at
            r = INSERT["boss_d"] / 2.0
            add.append(_prism(lug, "xz", y - r, y + r))
            cut.append(_hole((x, y, z + 0.01), (0, 0, -1), INSERT["hole_d"], INSERT["hole_depth"] + 0.01, up))
    # Tongues up inside the upper body's side walls (location), their far ends at 45 degrees.
    ub_in = sk.UPPER_BODY.y1 - t - c.slide_clear           # 79.3
    L = 60.0
    tongue = _mx(Polygon([(seam, lb.z1 - 0.1), (L, lb.z1 - 0.1), (L - c.lb_tongue_h, lb.z1 + c.lb_tongue_h - 0.1),
                          (seam, lb.z1 + c.lb_tongue_h - 0.1)]), sx)
    for sy in (1, -1):
        add.append(_prism(tongue, "xz", sy * (ub_in - c.lb_tongue_t), sy * ub_in))
        # its foot: the deck opening is the upper body's inside (|y| 79.5), so the tongue
        # stands on a strip of deck carried in under it
        add.append(_box(sx * seam, sx * L, sy * (ub_in - c.lb_tongue_t), sy * (sk.UPPER_BODY.y1 - t + 0.3), lb.z1 - t, lb.z1))
    # The side seam, one screw a side from below between the wheels.
    skirt_in = lb.y1 - t                                   # 102.5
    for sy in (1, -1):
        (ix, iy) = lb_seam_point(c, sy)
        if sx < 0:
            boss = teardrop(ix, iy, INSERT["boss_d"], (-1.0, 0.0))
            add.append(_prism(boss, "xy", lb_seam_z(c) + 2.0, lb.z1 - t + 0.1))
            cut.append(_hole((ix, iy, lb_seam_z(c) + 2.0 - 0.01), (0, 0, 1), INSERT["hole_d"], INSERT["hole_depth"] + 0.01, up))
        else:
            y_in, y_out = sy * (skirt_in - 9.5), sy * (skirt_in - 0.1)
            plate = Polygon([(-12.0, y_in), (10.6, y_in), (20.0, y_out), (-12.0, y_out)])
            add.append(_prism(plate, "xy", lb_seam_z(c), lb_seam_z(c) + 2.0))
            add.append(_box(seam, 20.0, sy * (skirt_in - 0.2), sy * (skirt_in + 0.2), lb_seam_z(c), lb_seam_z(c) + 2.0))
            cut.append(_hole((ix, iy, lb_seam_z(c) - 1.0), (0, 0, 1), CLEARANCE["M3"], 4.0, up))
    # USB-C (the charger module's receptacle) and the power button (skeleton: x LOWER_BODY.x0,
    # z 72). electrical.py puts both faces PORT_FACE_IN behind the rear face (x -90), so the
    # wall there is thickened to that and each port is recessed: the USB-C opening takes the
    # plug's overmould down to the receptacle (full engagement); the button's cap sits in a
    # hole under a finger pocket (no knock switches it). Inside: a shelf under the charger
    # board's outer end and two posts for the button board, each with two M2 inserts.
    if sx < 0:
        x0 = lb.x0
        face = x0 + PORT_FACE_IN
        ys = [sk.USB_C["at"][1], sk.POWER_SWITCH["at"][1]]
        pz = sk.USB_C["at"][2]
        add.append(_box(x0 + t - 0.1, face, min(ys) - 11.0, max(ys) + 11.0, pz - 10.0, pz + 10.0))
        _, uy, uz = sk.USB_C["at"]
        cut.append(_prism(USB_OPENING(uy, uz), "yz", x0 - 1.0, face + 0.5))
        _, by, bz = sk.POWER_SWITCH["at"]
        cut.append(_prism(BUTTON_POCKET(by, bz), "yz", x0 - 1.0, x0 + BUTTON_POCKET_DEPTH))
        cut.append(_prism(BUTTON_HOLE(by, bz), "yz", x0 - 1.0, face + 0.5))
        cb = CHARGER_BOARD
        add.append(_box(x0 + t - 0.1, cb["x_near"] + cb["shelf"], cb["y"][0] - 0.5, cb["y"][1] + 0.5,
                        cb["z_under"] - M2_SHELF_T, cb["z_under"]))
        for y_ in cb["holes_y"]:
            cut.append(_hole((cb["holes_x"], y_, cb["z_under"] + 0.01), (0, 0, -1), INSERT_M2["hole_d"], INSERT_M2["hole_depth"] + 0.01, up))
        bb = BUTTON_BOARD
        for y_, z_ in bb["holes"]:
            add.append(_cyl((face - 0.1, y_, z_), (1, 0, 0), INSERT_M2["boss_d"], bb["x_face"] - face + 0.1))
            cut.append(_cyl((bb["x_face"] + 0.01, y_, z_), (-1, 0, 0), INSERT_M2["hole_d"], INSERT_M2["hole_depth"] + 0.01))
    shell = _fuse(shell, add)
    shell = _cut(shell, cut)
    return _clean(shell)


LB_SEAM_GAP = 0.1
# The rear ports. electrical.py (CHG1, B2, S1 placements) has the USB-C receptacle's face and
# the button cap's face at x -90, 2.5 behind the rear face. The USB-C opening takes the plug's
# overmould (12.35 x 6.5 max, USB Type-C spec) with 0.4 a side, so the plug seats fully; the
# button: Adafruit #3104, a 9.8 mm round cap 8 mm tall on a tactile switch
# (adafruit.com/product/3104): a 10.4 mm hole under a 16 mm finger pocket 1.3 deep.
PORT_FACE_IN = 2.5
BUTTON_POCKET_DEPTH = 1.3
USB_OPENING = lambda y, z: rrect(13.2, 7.4, 3.2, y, z)                # noqa: E731
BUTTON_POCKET = lambda y, z: Point(y, z).buffer(8.0, quad_segs=24)     # noqa: E731
BUTTON_HOLE = lambda y, z: Point(y, z).buffer(5.2, quad_segs=24)       # noqa: E731
# The boards, as electrical.py places them (its solids el_CHG1, el_B2): the charger 22 x 18,
# horizontal, underside z 68.8, its near edge at x -85.8 (the receptacle overhangs it to -90);
# the button board 14 x 14, vertical, its front face at x -82, y 13-27, z 65-79. No hole
# positions are given: ours are 2.5 in from the charger's near corners and 1 mm in from two
# opposite corners of the button board (clear of the 9.8 mm cap by 0.8).
CHARGER_BOARD = {"x_near": -85.8, "y": (-29.0, -11.0), "z_under": 68.8, "shelf": 5.5,
                 "holes_x": -85.8 + 2.5, "holes_y": (-29.0 + 2.5, -11.0 - 2.5)}
BUTTON_BOARD = {"x_face": -82.0, "holes": ((14.0, 66.0), (26.0, 78.0))}
M2_SHELF_T = 6.2


def lb_seam_point(c: Shell, sy: int) -> Tuple[float, float]:
    return (-4.0, sy * (sk.LOWER_BODY.y1 - c.wall - 3.5))          # (-4, +-99)


def lb_seam_z(c: Shell) -> float:
    return c.skirt_z0                                              # the tongue's underside, 73


def bumper_screw_y(c: Shell) -> List[float]:
    y = c.led_window_half + 6.0             # beyond the window; the 117 mm strip ends at |y| 58.4, the heads start at 60.1
    return [-y, y]


def bumper_screw_x(c: Shell) -> float:
    return (c.bumper_x_in + sk.LOWER_BODY.x1) / 2.0                # 89.25


# ---- the bumper and its diffuser (TPU and translucent PETG) ----------------------------------

def bumper_channel(c: Shell) -> Tuple[float, float, float, float]:
    """(x0, x1, z0, z1), front: the cavity the LED strip and the diffuser slide into from
    the bumper's end, centred on LED_STRIP_Z."""
    x0 = c.bumper_x_in + c.bumper_back_wall                       # 87.2
    x1 = sk.LOWER_BODY.x1 - c.bumper_skin                          # 91.3
    h = c.channel_h
    return x0, x1, sk.LED_STRIP_Z - h / 2.0, sk.LED_STRIP_Z + h / 2.0


def _bumper(c: Shell, sx: int):
    lb = sk.LOWER_BODY
    z0, z1 = sk.BUMPER_Z
    hy = c.bumper_half_y
    body = _box(sx * c.bumper_x_in, sx * lb.x1, -hy, hy, z0, z1)
    cx0, cx1, cz0, cz1 = bumper_channel(c)
    cut = [_box(sx * cx0, sx * cx1, -hy - 1, hy + 1, cz0, cz1),
           _box(sx * (cx1 - 0.1), sx * (lb.x1 + 1.0), -c.led_window_half, c.led_window_half,
                sk.LED_STRIP_Z - c.strip_w / 2, sk.LED_STRIP_Z + c.strip_w / 2)]
    head = desk_bought.SHCS["M2"]
    cb_top = z1 - BUMPER_SCREW["grip"]
    up = (float(sx), 0.0, 0.0)                            # it prints on its back face
    for y in bumper_screw_y(c):
        x = sx * bumper_screw_x(c)
        cut.append(_hole((x, y, z0 - 1.0), (0, 0, 1), head["head_d"] + 0.2, cb_top - z0 + 1.0, up))
        cut.append(_hole((x, y, cb_top - 0.5), (0, 0, 1), CLEARANCE["M2"], z1 - cb_top + 1.5, up))
    return _clean(_cut(body, cut))


def _diffuser(c: Shell, sx: int):
    """Translucent PETG: a plug pressed (and glued) flush into the bumper's light window, so
    the channel behind it is strip_clear deep for the strip."""
    cx0, cx1, cz0, cz1 = bumper_channel(c)
    w = c.led_window_half - 0.05
    return _box(sx * cx1, sx * sk.LOWER_BODY.x1, -w, w, sk.LED_STRIP_Z - c.strip_w / 2 + 0.05, sk.LED_STRIP_Z + c.strip_w / 2 - 0.05)


def diffuser_half(c: Shell) -> float:
    return c.led_window_half - 0.05


# ---- hatch, socket, rings, cover -------------------------------------------------------------

def _hatch(c: Shell):
    ub = sk.UPPER_BODY
    plate = _prism(hatch_outline(c).buffer(-c.slide_clear / 2.0), "yz", ub.x0, ub.x0 + c.hatch_t)
    cut = []
    for y, z in hatch_insert_points(c):
        cut.append(_cyl((ub.x0 - 0.5, y, z), (1, 0, 0), CLEARANCE["M3"], c.hatch_t + 1.0))
        # 90 degree head; the seat is cut a little steeper (it prints on the head's side)
        cut.append(_cone((ub.x0 - 0.01, y, z), (1, 0, 0), CSK_M3["dk"] + 0.3, CLEARANCE["M3"] - 0.01,
                         (CSK_M3["dk"] + 0.3 - CLEARANCE["M3"]) / 2.0 * 1.1))
    cut.append(_prism(inlet_open(c), "yz", ub.x0 - 1.0, ub.x0 + c.hatch_t + 1.0))
    return _clean(_cut(plate, cut))


def _socket(c: Shell):
    ts = sk.TOP_SLOT
    body = _prism(rrect(ts.x1 - ts.x0, ts.y1 - ts.y0, 6.0), "xy", ts.z0, ts.z1)
    floor = ts.z0 + c.socket_floor_t
    cut = [_prism(rrect(ts.x1 - ts.x0 - 2 * c.wall, ts.y1 - ts.y0 - 2 * c.wall, 3.0), "xy", floor, ts.z1 + 1.0),
           _box(-c.header_slot[0] / 2, c.header_slot[0] / 2, -c.header_slot[1] / 2, c.header_slot[1] / 2, ts.z0 - 1.0, floor + 1.0)]
    head = desk_bought.SHCS["M3"]
    for x, y in socket_insert_points(c):
        cut.append(_cyl((x, y, ts.z0 + SOCKET_SCREW["grip"]), (0, 0, 1), head["head_d"] + 0.5, ts.z1))
        cut.append(_cyl((x, y, ts.z0 - 1.0), (0, 0, 1), CLEARANCE["M3"], SOCKET_SCREW["grip"] + 2.0))
    return _clean(_cut(body, cut))


def _ring(c: Shell, sy: int):
    x, _, z = sk.SHOULDERS["left" if sy > 0 else "right"]
    y1 = sk.UPPER_BODY.y1
    outer = _cyl((x, sy * (y1 - c.wall), z), (0, sy, 0), c.ring_od - c.slide_clear, c.wall)
    return _clean(_cut(outer, [_cyl((x, sy * (y1 - c.wall - 1.0), z), (0, sy, 0), sk.SHELL_ARM_OPENING_D, c.wall + 2.0)]))


def _cover(c: Shell):
    x1 = sk.UPPER_BODY.x1
    return _prism(face_cover_outline(c), "yz", x1 - c.cover_t, x1)


# ---- bought parts placed ---------------------------------------------------------------------
# Screws the shell owns (its own joints). The mounts' screws pass through the chassis and are
# the chassis's (their length depends on its frame); the shell gives them 6.5 mm of hole.
SOCKET_SCREW = {"size": "M3", "grip": 3.0, "thread_in": 5.0}     # socket floor below the counterbore
LB_SEAM_SCREW = {"size": "M3", "grip": 2.0, "thread_in": 6.0}    # the tongue, 2 thick
BUMPER_SCREW = {"size": "M2", "grip": 6.4, "thread_in": 5.0}     # TPU above the counterbore (its head just over
                                                                 # the strip's channel), then the sill
HATCH_SCREW = {"size": "M3", "grip": 2.5, "thread_in": 5.5}      # countersunk: the length includes the head


def insert_sites(c: Shell) -> List[dict]:
    """Every heat-set insert: its hole's mouth, the axis into the hole, which part holds it."""
    out = []
    for m in sk.SHELL_MOUNTS:
        part = ("shell_lower_front" if m.at[0] > 0 else "shell_lower_rear") if m.name.startswith("lower_") else \
               ("shell_upper_left" if m.axis[1] > 0 else "shell_upper_right")
        out.append({"label": f"insert_{m.name}", "at": tuple(m.at), "axis": tuple(m.axis), "part": part, "mount": m.name})
    for k, (x, y) in enumerate(socket_insert_points(c)):
        out.append({"label": f"insert_socket_{k}", "at": (x, y, sk.UPPER_BODY.z1), "axis": (0.0, 0.0, -1.0),
                    "part": "shell_upper_left" if y > 0 else "shell_upper_right"})
    for k, (y, z) in enumerate(hatch_insert_points(c)):
        out.append({"label": f"insert_hatch_{k}", "at": (sk.UPPER_BODY.x0 + c.hatch_t, y, z), "axis": (1.0, 0.0, 0.0),
                    "part": "shell_upper_left" if y > 0 else "shell_upper_right"})
    for k, sy in enumerate((1, -1)):
        x, y = lb_seam_point(c, sy)
        out.append({"label": f"insert_lb_seam_{k}", "at": (x, y, lb_seam_z(c) + 2.0), "axis": (0.0, 0.0, 1.0),
                    "part": "shell_lower_rear"})
    cb = CHARGER_BOARD
    for k, y in enumerate(cb["holes_y"]):
        out.append({"label": f"insert_charger_{k}", "at": (cb["holes_x"], y, cb["z_under"]), "axis": (0.0, 0.0, -1.0),
                    "part": "shell_lower_rear", "ins": INSERT_M2, "size": "M2"})
    for k, (y, z) in enumerate(BUTTON_BOARD["holes"]):
        out.append({"label": f"insert_button_{k}", "at": (BUTTON_BOARD["x_face"], y, z), "axis": (-1.0, 0.0, 0.0),
                    "part": "shell_lower_rear", "ins": INSERT_M2, "size": "M2"})
    for o in out:
        o.setdefault("ins", INSERT)
        o.setdefault("size", "M3")
    return out


def screw_sites(c: Shell) -> List[dict]:
    """The shell's own screws: the bearing face of the head, the shank's direction, the spec."""
    out = []
    sp = desk_bought.screw_length("M3", SOCKET_SCREW["grip"], nut=False, thread_in_mm=SOCKET_SCREW["thread_in"])
    for k, (x, y) in enumerate(socket_insert_points(c)):
        out.append({"label": f"screw_socket_{k}", "kind": "shcs", "at": (x, y, sk.TOP_SLOT.z0 + SOCKET_SCREW["grip"]),
                    "dir": (0.0, 0.0, -1.0), **sp})
    hp = {"size": "M3", "length": next(L for L in desk_bought.SHCS_LENGTHS if L >= HATCH_SCREW["grip"] + HATCH_SCREW["thread_in"]),
          "grip_mm": HATCH_SCREW["grip"], "thread_in_mm": None}
    hp["thread_in_mm"] = round(hp["length"] - HATCH_SCREW["grip"], 3)
    for k, (y, z) in enumerate(hatch_insert_points(c)):
        out.append({"label": f"screw_hatch_{k}", "kind": "csk", "at": (sk.UPPER_BODY.x0, y, z), "dir": (1.0, 0.0, 0.0), **hp})
    lp = desk_bought.screw_length("M3", LB_SEAM_SCREW["grip"], nut=False, thread_in_mm=LB_SEAM_SCREW["thread_in"])
    for k, sy in enumerate((1, -1)):
        x, y = lb_seam_point(c, sy)
        out.append({"label": f"screw_lb_seam_{k}", "kind": "shcs", "at": (x, y, lb_seam_z(c)), "dir": (0.0, 0.0, 1.0), **lp})
    bp = desk_bought.screw_length("M2", BUMPER_SCREW["grip"], nut=False, thread_in_mm=BUMPER_SCREW["thread_in"])
    k = 0
    for sx in (1, -1):
        for y in bumper_screw_y(c):
            out.append({"label": f"screw_bumper_{k}", "kind": "shcs",
                        "at": (sx * bumper_screw_x(c), y, sk.BUMPER_Z[1] - BUMPER_SCREW["grip"]), "dir": (0.0, 0.0, 1.0), **bp})
            k += 1
    return out


# Screws are drawn at their thread's minor diameter (ISO 724 d1), inserts as tubes 0.05 inside
# their hole and 0.05 outside that bore: so a screw in its insert or its thread-forming pilot,
# and an insert in its hole, neither overlap nor share a face (the two tessellations of one
# cylinder would cross each other by their chord heights).
MINOR_D = {"M2": 1.567, "M3": 2.459}


def _insert_solid(site: dict):
    x, y, z = site["at"]
    ins = site["ins"]
    tube = _cyl((x, y, z), site["axis"], ins["hole_d"] - 0.1, ins["length"])
    return _clean(_cut(tube, [_cyl(tuple(np.asarray(site["at"]) - np.asarray(site["axis"])), site["axis"],
                                   MINOR_D[site["size"]] + 0.1, ins["length"] + 2.0)]))


def _screw_solid(site: dict):
    s = desk_bought.SHCS[site["size"]]
    at, d = np.asarray(site["at"], float), np.asarray(site["dir"], float)
    if site["kind"] == "csk":
        head = _cone(tuple(at), tuple(d), CSK_M3["dk"], s["d"], (CSK_M3["dk"] - s["d"]) / 2.0)
        shank = _cyl(tuple(at), tuple(d), MINOR_D[site["size"]], site["length"])
    else:
        head = _cyl(tuple(at - d * s["head_k"]), tuple(d), s["head_d"], s["head_k"])
        shank = _cyl(tuple(at), tuple(d), MINOR_D[site["size"]], site["length"])
    return _clean(_fuse(head, [shank]))


# ---- the parts table -------------------------------------------------------------------------
# label: (builder, material, colour, process, assembly step, print rotation robot -> bed)
_RX = {"side_left": np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float),     # bed z = -y
       "side_right": np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float),    # bed z = +y
       "front_down": np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float),    # bed z = -x
       "rear_down": np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float),     # bed z = +x
       "upright": np.eye(3)}

STEPS = {1: "bench: heat-set every insert; LED strip and diffuser into each bumper, bumper onto its lower half "
            "(2 M2 x 12 from below); teal rings into the upper halves (glued)",
         2: "lower front and rear halves slide onto the chassis along x, their lugs under the deck corners; "
            "the four lower mount screws go in from above through the deck",
         3: "the two side seam screws from below, between the wheels",
         4: "upper left half slides in along -y onto the spine, then the right half (its tongue inside the left); "
            "the four upper mount screws from inside the spine (the arm modules go on after step 7, last, "
            "from outside: skeleton.ASSEMBLY_NOTE)",
         5: "top slot socket: 4 M3 x 8 into the halves' inserts (clamps the seam at the top)",
         6: "rear hatch: 4 M3 x 8 countersunk (clamps the seam at the back)",
         7: "face cover bonded into its rebate (clear silicone or acrylic-safe adhesive), across the seam"}

MADE = {
    "shell_upper_left": (lambda c: _ub_half(c, 1), "PETG", "warm_white", "print", 4, "side_left"),
    "shell_upper_right": (lambda c: _ub_half(c, -1), "PETG", "warm_white", "print", 4, "side_right"),
    "shell_lower_front": (lambda c: _lb_half(c, 1), "PETG", "dark_grey", "print", 2, "front_down"),
    "shell_lower_rear": (lambda c: _lb_half(c, -1), "PETG", "dark_grey", "print", 2, "rear_down"),
    "bumper_front": (lambda c: _bumper(c, 1), "TPU 95A", "dark_grey", "print", 1, "rear_down"),
    "bumper_rear": (lambda c: _bumper(c, -1), "TPU 95A", "dark_grey", "print", 1, "front_down"),
    "diffuser_front": (lambda c: _diffuser(c, 1), "PETG", "translucent", "print", 1, "rear_down"),
    "diffuser_rear": (lambda c: _diffuser(c, -1), "PETG", "translucent", "print", 1, "front_down"),
    "rear_hatch": (lambda c: _hatch(c), "PETG", "warm_white", "print", 6, "rear_down"),
    "top_socket": (lambda c: _socket(c), "PETG", "teal", "print", 5, "upright"),
    "shoulder_ring_left": (lambda c: _ring(c, 1), "PETG", "teal", "print", 1, "side_left"),
    "shoulder_ring_right": (lambda c: _ring(c, -1), "PETG", "teal", "print", 1, "side_right"),
    "face_cover": (lambda c: _cover(c), "cast acrylic 1.5 mm, clear", "clear", "cnc", 7, None),
}
SECTIONS = ("shell_upper_left", "shell_upper_right", "shell_lower_front", "shell_lower_rear")
DENSITY = {"PETG": PETG_G_CM3, "TPU 95A": TPU_G_CM3, "cast acrylic 1.5 mm, clear": PMMA_G_CM3}


@functools.lru_cache(maxsize=4)
def build(c: Shell = Shell()) -> Dict[str, object]:
    """label -> OpenCascade shape, robot frame: every made part, insert and screw."""
    out = {label: spec[0](c) for label, spec in MADE.items()}
    for s in insert_sites(c):
        out[s["label"]] = _insert_solid(s)
    for s in screw_sites(c):
        out[s["label"]] = _screw_solid(s)
    return out


@functools.lru_cache(maxsize=4)
def _meshes(c: Shell = Shell()) -> Dict[str, np.ndarray]:
    return {label: tessellate(shape) for label, shape in build(c).items()}


def print_triangles(label: str, c: Shell = Shell()) -> np.ndarray:
    """A printed part's triangles in its print orientation: on the bed (z 0), centred."""
    R = _RX[MADE[label][5]]
    tris = _meshes(c)[label] @ R.T
    v = tris.reshape(-1, 3)
    lo, hi = v.min(0), v.max(0)
    return tris - np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])


# ---- the contract ----------------------------------------------------------------------------

def solids(r=None) -> Dict[str, np.ndarray]:
    """label -> triangles (N, 3, 3) mm, robot frame, closed: every made part, insert and screw."""
    return {k: v.copy() for k, v in _meshes(config(r)).items()}


def mounts(r=None) -> List[sk.Mount]:
    """The shell holds the thread of every SHELL_MOUNT."""
    return list(sk.SHELL_MOUNTS)


def _bought(c: Shell) -> List[dict]:
    screws = screw_sites(c)
    def steel_g(size, length, csk=False):
        s = desk_bought.SHCS[size]
        head = (math.pi / 12 * (CSK_M3["k"]) * (CSK_M3["dk"] ** 2 + CSK_M3["dk"] * s["d"] + s["d"] ** 2)) if csk else \
            math.pi / 4 * s["head_d"] ** 2 * s["head_k"]
        return (head + math.pi / 4 * (0.92 * s["d"]) ** 2 * length) / 1000 * STEEL_G_CM3
    rows = []
    for ins, lab in ((INSERT, "insert_m3"), (INSERT_M2, "insert_m2")):
        n_ins = sum(1 for s_ in insert_sites(c) if s_["ins"] is ins)
        rows.append({"label": lab, "qty": n_ins, "g": ins["g"], "spec": ins["spec"], "source": ins["source"]})
    for kind, prefix, spec in (("shcs", "screw_socket", "ISO 4762"), ("csk", "screw_hatch", "ISO 10642 countersunk"),
                               ("shcs", "screw_lb_seam", "ISO 4762"), ("shcs", "screw_bumper", "ISO 4762, thread-forming into PETG")):
        s = [x for x in screws if x["label"].startswith(prefix)]
        rows.append({"label": prefix, "qty": len(s), "g": round(steel_g(s[0]["size"], s[0]["length"], kind == "csk"), 2),
                     "spec": f"{s[0]['size']} x {s[0]['length']} {spec}", "source": "https://www.fasteners.eu/standards/ISO/4762/"})
    rows.append({"label": "cover_adhesive", "qty": 1, "g": 0.5, "spec": "clear neutral-cure silicone or acrylic-safe "
                 "adhesive, a 3 mm bead round the cover's land", "source": "any"})
    return rows


def parts(r=None) -> List[dict]:
    c = config(r)
    shapes = build(c)
    out = []
    for label, (_, material, colour, process, step, _rot) in MADE.items():
        v = volume_mm3(shapes[label])
        out.append({"label": label, "step": step, "kind": "made", "process": process, "material": material,
                    "colour": COLOURS[colour], "volume_cm3": round(v / 1000, 2),
                    "mass_g": round(v / 1000 * DENSITY[material], 1), "centre_mm": centre_mm(shapes[label]), "source": None})
    for s in insert_sites(c):
        out.append({"label": s["label"], "step": 1, "kind": "bought", "process": "bought", "material": "brass",
                    "mass_g": s["ins"]["g"], "centre_mm": tuple(round(v, 2) for v in s["at"]), "source": s["ins"]["source"]})
    grams = {b["label"]: b["g"] for b in _bought(c)}
    for s in screw_sites(c):
        prefix = "_".join(s["label"].split("_")[:-1])
        step = {"screw_socket": 5, "screw_hatch": 6, "screw_lb_seam": 3, "screw_bumper": 1}[prefix]
        out.append({"label": s["label"], "step": step, "kind": "bought", "process": "bought", "material": "steel",
                    "mass_g": grams[prefix], "centre_mm": tuple(round(v, 2) for v in s["at"]),
                    "spec": f"{s['size']} x {s['length']}", "source": "https://www.fasteners.eu/standards/ISO/4762/"})
    return out


def write_stl_bytes(tris: np.ndarray, name: str) -> bytes:
    """Binary STL, mm (desk_robot.design.write_stl's format, to memory)."""
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, norm, out=np.zeros_like(n), where=norm > 0)
    rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tris
    return name.encode()[:80].ljust(80, b"\0") + struct.pack("<I", len(tris)) + rec.tobytes()


def cover_program(c: Shell, tip_above_top_mm: float = 10.0, tool_d: float = 3.175) -> str:
    """The clear cover routed from a 1.5 mm cast acrylic sheet on the LongMill: work zero on
    the sheet's top centre (X along robot y, Y along robot z), the sheet taped down as the
    desk robot's hub blank is. A 1/8 in single-flute O-flute (plastic) at 18000 rpm, 900
    mm/min, 3 passes to 0.2 mm into the tape and spoilboard, outside the line."""
    outline = face_cover_outline(c)
    _, z0, _, z1 = outline.bounds
    outline = affinity.translate(outline, 0.0, -(z0 + z1) / 2.0)
    path = outline.buffer(tool_d / 2.0, quad_segs=24).exterior
    depth = c.cover_t + 0.2
    lines = [f"(SparkRover face cover, {c.cover_t} mm clear cast acrylic, routed: a blue diode laser does not cut clear)",
             "G21 G90 G17 G94",
             f"G10 L20 P1 X0 Y0 Z{tip_above_top_mm:.3f}", "G54"]
    w = toolpath.GcodeWriter(5.0, 900.0, 150.0)
    w.emit("G0 Z5")
    w.emit(f"T1 M6 (1/8 in single-flute O-flute, {tool_d} mm)")
    w.emit("M3 S18000")
    w.emit("G4 P1")
    coords = list(path.coords)
    passes = 3
    for k in range(1, passes + 1):
        z = -depth * k / passes
        w.travel(*coords[0])
        w.plunge_to(z)
        for x, y in coords[1:]:
            w.cut(x, y)
        w.retract()
    w.emit("M5")
    w.emit("G0 Z20")
    w.emit("M2")
    return "\n".join(lines + w.lines) + "\n"


def jobs(r=None) -> Dict[str, object]:
    """Printed parts: binary STL (bytes), mm, in the print orientation, on the bed and centred.
    The face cover: a GRBL program (str) for the LongMill."""
    c = config(r)
    out: Dict[str, object] = {}
    for label, spec in MADE.items():
        if spec[3] == "print":
            out[label] = write_stl_bytes(print_triangles(label, c), f"SparkRover {label}")
    out["face_cover"] = cover_program(c)
    return out


# ---- checks ----------------------------------------------------------------------------------

def _classifier(shape):
    return [BRepClass3d_SolidClassifier(s) for s in _solids(shape)]


def inside(shape, pts, tol: float = 1e-4) -> np.ndarray:
    """True where a point is inside (or on) the solid."""
    cls = _classifier(shape)
    out = np.zeros(len(pts), bool)
    for i, p in enumerate(np.asarray(pts, float)):
        q = gp_Pnt(*p)
        for cl in cls:
            cl.Perform(q, tol)
            if cl.State() in (TopAbs_IN, TopAbs_ON):
                out[i] = True
                break
    return out


def overhang(tris: np.ndarray, max_angle_deg: float = 44.0) -> dict:
    """Downward faces flatter than `max_angle_deg` from horizontal, off the bed: the area a
    slicer would support. Horizontal ones (bridges, or a ledge) are reported apart."""
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    a = np.linalg.norm(n, axis=1) / 2.0
    nz = n[:, 2] / np.maximum(np.linalg.norm(n, axis=1), 1e-12)
    zc = tris[:, :, 2].mean(1)
    down = (nz < -math.cos(math.radians(max_angle_deg))) & (zc > 0.05)
    flat = down & (nz < -0.999)
    return {"overhang_mm2": round(float(a[down].sum()), 1), "horizontal_mm2": round(float(a[flat].sum()), 1),
            "sloped_mm2": round(float(a[down & ~flat].sum()), 1)}


def wall_thickness(tris: np.ndarray, samples: int = 6000, seed: int = 0) -> dict:
    """Local wall thickness: from points on the surface, a ray straight in, to the first face
    that faces the other way (within 30 degrees). Edges and wedge tips, where the ray leaves
    through a face at an angle, are not walls and are skipped."""
    import trimesh
    m = trimesh.Trimesh(**trimesh.triangles.to_kwargs(tris))
    m.merge_vertices(digits_vertex=5)
    rng = np.random.default_rng(seed)
    a = m.area_faces
    fi = rng.choice(len(a), size=samples, p=a / a.sum())
    w = rng.random((samples, 2))
    w[w.sum(1) > 1] = 1 - w[w.sum(1) > 1]
    v = m.triangles[fi]
    p = v[:, 0] + w[:, :1] * (v[:, 1] - v[:, 0]) + w[:, 1:] * (v[:, 2] - v[:, 0])
    nrm = m.face_normals[fi]
    loc, ri, ti = m.ray.intersects_location(p - nrm * 1e-4, -nrm, multiple_hits=True)
    d = np.linalg.norm(loc - p[ri], axis=1)
    keep = d > 1e-3
    d, ri, ti = d[keep], ri[keep], ti[keep]
    near = np.full(samples, np.inf)
    face = np.full(samples, -1)
    order = np.argsort(-d)                     # nearest written last
    near[ri[order]] = d[order]
    face[ri[order]] = ti[order]
    hit = face >= 0
    opposite = np.zeros(samples, bool)
    opposite[hit] = np.einsum("ij,ij->i", m.face_normals[face[hit]], nrm[hit]) < -math.cos(math.radians(30))
    vals = near[hit & opposite]
    return {"min_mm": round(float(vals.min()), 3), "p1_mm": round(float(np.percentile(vals, 1)), 3),
            "median_mm": round(float(np.median(vals)), 3), "rays": int(len(vals))}


def _tri_box_overlap(tris: np.ndarray, lo: np.ndarray, hi: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Separating-axis test, triangles against one box: True where they overlap by more
    than eps (touching is not overlapping)."""
    c = (lo + hi) / 2.0
    h = (hi - lo) / 2.0
    v = tris - c
    e = [v[:, 1] - v[:, 0], v[:, 2] - v[:, 1], v[:, 0] - v[:, 2]]
    ok = np.ones(len(tris), bool)
    for k in range(3):                                    # the box's faces
        mn, mx = v[:, :, k].min(1), v[:, :, k].max(1)
        ok &= (mn < h[k] - eps) & (mx > -h[k] + eps)
    n = np.cross(e[0], e[1])                              # the triangle's plane
    d = np.einsum("ij,ij->i", n, v[:, 0])
    rad = np.abs(n) @ h
    ok &= np.abs(d) < rad - eps * np.linalg.norm(n, axis=1)
    for ed in e:                                          # edge x box axis
        for k in range(3):
            ax = np.zeros_like(ed)
            ax[:, (k + 1) % 3], ax[:, (k + 2) % 3] = -ed[:, (k + 2) % 3], ed[:, (k + 1) % 3]
            ax_n = np.linalg.norm(ax, axis=1)
            live = ax_n > 1e-12
            p = np.einsum("ijk,ik->ij", v, ax)
            r = np.abs(ax) @ h
            sep = (p.min(1) >= r - eps * ax_n) | (p.max(1) <= -r + eps * ax_n)
            ok &= ~(sep & live)
    return ok


def _box_minus(a: sk.Box, b: sk.Box) -> List[sk.Box]:
    """a without b, as boxes."""
    if not a.overlaps(b):
        return [a]
    out = []
    x0, x1, y0, y1, z0, z1 = a.x0, a.x1, a.y0, a.y1, a.z0, a.z1
    if b.x0 > x0:
        out.append(sk.Box(x0, b.x0, y0, y1, z0, z1))
    if b.x1 < x1:
        out.append(sk.Box(b.x1, x1, y0, y1, z0, z1))
    mx0, mx1 = max(x0, b.x0), min(x1, b.x1)
    if b.y0 > y0:
        out.append(sk.Box(mx0, mx1, y0, b.y0, z0, z1))
    if b.y1 < y1:
        out.append(sk.Box(mx0, mx1, b.y1, y1, z0, z1))
    my0, my1 = max(y0, b.y0), min(y1, b.y1)
    if b.z0 > z0:
        out.append(sk.Box(mx0, mx1, my0, my1, z0, b.z0))
    if b.z1 < z1:
        out.append(sk.Box(mx0, mx1, my0, my1, b.z1, z1))
    return out


def claims(c: Shell) -> List[dict]:
    """Where the shell goes into another subsystem's zone on purpose, and why: the
    interfaces themselves (a mount's thread is in the shell, at a point inside the frame's
    zone) and two places the zones leave no room."""
    out = []
    g = c.gap
    for m in sk.SHELL_MOUNTS:
        x, y, z = m.at
        r = INSERT["boss_d"] / 2.0 + 0.5
        if m.name.startswith("lower_"):
            sx = 1 if x > 0 else -1
            xs = sorted((sx * (abs(x) - r), sx * (c.bumper_x_in + 0.01)))
            out.append({"zone": "chassis.base", "why": f"{m.name}: the lug the deck corner rests on (the thread under the deck)",
                        "box": sk.Box(xs[0], xs[1], y - r, y + r, z - INSERT["hole_depth"] - 3.0, z + 0.01)})
        else:
            sy = 1 if y > 0 else -1
            ys = sorted((y - sy * 0.01, y + sy * (g + 0.2)))
            out.append({"zone": "chassis.spine", "why": f"{m.name}: the boss's face is the mount face on the spine",
                        "box": sk.Box(x - r, x + r, ys[0], ys[1], z - r, z + r)})
    bay = _zone("electrical", "bay")
    for y, z in hatch_insert_points(c):
        r = INSERT["boss_d"] / 2.0 * math.sqrt(2.0) + 0.5
        out.append({"zone": "electrical.bay", "why": "a rear hatch insert's boss: the bay starts 4 mm behind the skin, an "
                    "insert needs 6.5 + 1.3", "box": sk.Box(bay.x0 - g - 0.01, hatch_boss_x1(c) + 0.01,
                                                          y - r, y + r, z - r, z + r)})
    hz = _zone("electrical", "head")
    for x, y, _ in sk.MIC_HOLES:
        r = (c.mic_duct_d / 2 + c.mic_duct_wall) * math.sqrt(2.0) + 0.5
        out.append({"zone": "electrical.head", "why": "a mic's acoustic duct ends on the mic, at the head zone's top",
                    "box": sk.Box(x - r, x + r, y - r, y + r, hz.z1 - 0.01, hz.z1 + c.gap + 0.01)})
    for sy in (1, -1):
        x, y = lb_seam_point(c, sy)
        z = lb_seam_z(c)
        head = desk_bought.SHCS["M3"]
        out.append({"zone": "chassis.wheel_wells" + ("_l" if sy > 0 else ""), "why": "a side seam screw's head, between the "
                    "wheels: the wells cover the whole side below z 72", "box": sk.Box(x - 3.5, x + 3.5, y - 3.5, y + 3.5,
                                                                                         z - head["head_k"] - 0.01, z + 0.01)})
    return out


def skeleton_issues(c: Shell) -> List[dict]:
    """What in skeleton.py the shell could not meet, what it did instead, what to change."""
    ub = sk.UPPER_BODY
    hz0 = sk.REAR_HATCH["centre"][2] - sk.REAR_HATCH["size"][1] / 2
    hz1 = sk.REAR_HATCH["centre"][2] + sk.REAR_HATCH["size"][1] / 2
    vz0 = sk.REAR_VENT["centre"][2] - sk.REAR_VENT["size"][1] / 2
    bz0 = sk.CAMERA["lens_at"][2] - sk.CAMERA["bar_size"][1] / 2
    bz1 = sk.CAMERA["lens_at"][2] + sk.CAMERA["bar_size"][1] / 2
    wz0 = sk.FACE_WINDOW["centre"][2] - sk.FACE_WINDOW["size"][1] / 2
    sh = _zone("arms", "shoulder_l")
    ts_z = _zone("electrical", "top_slot")
    hz_ = _zone("electrical", "head")
    arm = _zone("arms", "arm_l")
    chest = _zone("electrical", "chest")
    return [
        {"id": "shoulder_zone_in_wall", "zones": ["arms.shoulder_l", "arms.shoulder_r"],
         "box": [sk.Box(sh.x0 - c.gap, sh.x1 + c.gap, *sorted((s * (ub.y1 - c.wall - c.gap), s * (ub.y1 + 0.01))),
                        sh.z0 - c.gap, sh.z1 + c.gap) for s in (1, -1)],
         "what": f"shoulder_l/r reach y +-{sh.y1:g} = the upper body's outer face, so they overlap the side wall "
                 f"({c.wall:g}) and its gap. The shell keeps its wall (with the opening) and does not move.",
         "change": f"shoulder zones y {sh.y0:g} to {ub.y1 - c.wall - c.gap:g} (the servo's outer face 3.5 mm in)"},
        {"id": "arm_zone_touches_skin", "zones": ["arms.arm_l", "arms.arm_r"],
         "box": [sk.Box(arm.x0 - c.gap, arm.x1 + c.gap, *sorted((s * (ub.y1 - c.gap - 0.01), s * (ub.y1 + c.gap))),
                        arm.z0 - c.gap, arm.z1 + c.gap) for s in (1, -1)],
         "what": f"arm_l/r start at y +-{arm.y0:g}, the upper body's face: 0 clearance where SHELL_GAP asks {c.gap:g}",
         "change": f"arm zones from |y| {ub.y1 + c.gap:g}"},
        {"id": "hatch_vent_overlap", "what": f"REAR_HATCH spans z {hz0:g}-{hz1:g}, REAR_VENT z {vz0:g}-"
                 f"{sk.REAR_VENT['centre'][2] + sk.REAR_VENT['size'][1] / 2:g}: {hz1 - vz0:g} mm of overlap. "
                 f"The vent is cut at z {c.vent_z - sk.REAR_VENT['size'][1] / 2:g}-{c.vent_z + sk.REAR_VENT['size'][1] / 2:g}",
         "change": f"REAR_VENT centre z {c.vent_z:g}"},
        {"id": "camera_bar_under_window", "what": f"CAMERA bar z {bz0:g}-{bz1:g} ends {wz0 - bz1:g} mm under FACE_WINDOW "
                 f"(from z {wz0:g}): a web that thin breaks, so they are one opening under one clear cover and the camera "
                 "and its LEDs look through 1.5 mm cast acrylic",
         "change": f"none needed; for a real web between them, CAMERA lens_at z <= {wz0 - 3.0 - sk.CAMERA['bar_size'][1] / 2:g}"},
        {"id": "top_slot_zone_is_the_socket", "zones": ["electrical.top_slot"],
         "box": [sk.Box(ts_z.x0 - c.gap - 0.01, ts_z.x1 + c.gap + 0.01, ts_z.y0 - c.gap - 0.01, ts_z.y1 + c.gap + 0.01,
                        ts_z.z0 - c.gap - 0.01, ts_z.z1 + c.gap + 0.01)],
         "what": f"electrical.top_slot is the whole TOP_SLOT box and down to z {ts_z.z0:g}: it holds the socket itself, "
                 "the top wall under it and the socket's inserts. The shell's socket cannot keep SHELL_GAP from it",
         "change": "the zone as the header board only, e.g. x +-4, y +-9, z 262-286 (the socket's slot is 5.4 x 13.0, "
                   "its floor z 272-276, the wire hole 12 x 24 through the top)"},
        {"id": "inlet_raised", "what": f"REAR_INLET centre z {sk.REAR_INLET['centre'][2]:g}: the duct's seat ring "
                 f"({c.inlet_seat[0]:g} x {c.inlet_seat[1]:g}) round it would reach z {sk.REAR_INLET['centre'][2] - c.inlet_seat[1] / 2:g}, "
                 f"below the hatch (from z {hz0:g}). Cut at centre z {inlet_z(c):g}: the ring on the hatch inside the access "
                 f"opening, {c.inlet_land_min:g} mm of the shell's land under it; two {c.inlet_rib_w:g} mm ribs, "
                 f"{inlet_open(c).area:.0f} mm2 open. The duct's end moves up with it",
         "change": f"REAR_INLET centre z {inlet_z(c):g}"},
        {"id": "bumper_thickness", "what": f"BUMPER_T {sk.BUMPER_T:g}: between the base zone (x {_zone('chassis', 'base').x1:g}) "
                 f"+ gap and DEPTH/2 there is {c.bumper_t:g}; the bumper is {c.bumper_t:g} thick, and only |y| <= "
                 f"{c.bumper_half_y:g} (the wheel wells fill the corners and the sides below z 72): front and rear "
                 "segments, not a ring",
         "change": f"chassis.base x to +-{sk.LOWER_BODY.x1 - sk.BUMPER_T - c.gap:g} for an 8 mm bumper"},
        {"id": "wheel_housings", "what": "the wheel wells reach |y| 105 (the envelope) from z 0 to 72, so the shell can roof "
                 f"the wheels (z {c.skirt_z0:g}-85) but not wrap their sides",
         "change": "wheel wells to |y| <= 101.5 (tread face 98 + 2.5 + 1) for housings down the sides"},
        {"id": "hatch_inserts_in_bay", "what": f"the bay starts at x {_zone('electrical', 'bay').x0:g}, 4 mm behind the rear "
                 "skin: a flush hatch's M3 inserts need 2.5 + 6.5 + 1; their 4 bosses reach 5 mm into the bay (claimed)",
         "change": "bay x0 to -69, or keep 4 x 10 mm round the hatch corners clear"},
        {"id": "lower_mount_lugs", "what": "lower mounts put the thread under the deck corners (axis -z): the lugs reach "
                 "11 mm into chassis.base and the lower body must slide on along x, the deck's screws driven before the "
                 "upper body goes on",
         "change": "none needed; the chassis keeps x 70.5-86, |y| 55.5-64.5, z 40.5-50 clear at each corner"},
        {"id": "touch_pad_gap", "what": f"the chest zone ends at x {chest.x1:g}; the chest wall's flat inside is at x "
                 f"{ub.x1 - c.wall:g}: copper tape on it is the shell's side of the interface, {ub.x1 - c.wall - chest.x1:g} mm "
                 "from the zone (the touch controller's leads cross that)", "change": "none needed if electrical treats the "
                 "tape as stuck to the shell"},
        {"id": "led_strip_in_the_bumper", "what": "electrical.py's el_D1/el_D2 strips sit at x +-80.5..84.5, behind the "
                 "bumper's back face (x 86) and in chassis.base: the bumper's channel (x 87.2-91.3, 4.1 deep, z 23.4-36.6, "
                 "open at both ends) is where the strip goes, the diffuser in front of it",
         "change": "electrical: D1 at x 87.2-91.2 (D2 mirrored), the strip slid into the channel from the bumper's end"},
        {"id": "head_zone_corners", "zones": ["electrical.head", "electrical.head"],
         "box": [sk.Box(hz_.x1 - 6.0, hz_.x1 + c.gap + 0.01, *sorted((s_ * (hz_.y1 - 6.0), s_ * (hz_.y1 + c.gap + 0.01))),
                        hz_.z1 - 8.0, hz_.z1 + c.gap + 0.01) for s_ in (1, -1)],
         "what": f"the head zone (|y| {hz_.y1:g}, to z {hz_.z1:g}, x {hz_.x1:g}) reaches into the corners where the face, "
                 "side and top rounds meet: 0.6 mm from the wall's inside there, not 1.0",
         "change": "chamfer the head zone's two top front corners 6 mm (a 5 in panel is 120.7 wide: |y| 60)"},
        {"id": "low_mounts_under_shoulders", "what": "upper_*_low (z 110) is directly under shoulder_* (z 120-190): "
                 "no one-piece upper body can be lowered past the servos. Hence the left/right split.",
         "change": "none needed"},
        {"id": "shell_wall", "what": f"SHELL_WALL {sk.SHELL_WALL:g} printed ~580 g of shell and bumpers (solid-body), over "
                 f"the 350-500 g brief; the shell is built at {c.wall:g} with local thickening and ribs",
         "change": f"SHELL_WALL {c.wall:g}"},
    ]


def internal_fit(meshes: Dict[str, np.ndarray], nudge: float = 0.05) -> dict:
    """The shell's own parts against each other, assembled, as integrate.interference
    sorts them: TOUCHING (a 0.05 mm nudge frees them) or INTERFERENCE."""
    import trimesh
    ms = {}
    for k, t in meshes.items():
        m = trimesh.Trimesh(**trimesh.triangles.to_kwargs(t))
        m.merge_vertices(digits_vertex=5)
        ms[k] = m
    cm = trimesh.collision.CollisionManager()
    for k, m in ms.items():
        cm.add_object(k, m)
    _, pairs = cm.in_collision_internal(return_names=True)
    rows = {"touching": [], "interference": []}
    for a, b in sorted(tuple(sorted(p)) for p in pairs):
        one = trimesh.collision.CollisionManager()
        one.add_object(b, ms[b])
        freed = None
        for axis in range(3):
            for sgn in (1, -1):
                T = np.eye(4)
                T[axis, 3] = sgn * nudge
                if not one.in_collision_single(ms[a], transform=T):
                    freed = "+-"[sgn < 0] + "xyz"[axis]
                    break
            if freed:
                break
        rows["touching" if freed else "interference"].append((a, b))
    return rows


def _check_insert(shape, site: dict) -> Tuple[bool, str]:
    at, ax = np.asarray(site["at"], float), np.asarray(site["axis"], float)
    perp = np.cross(ax, [1.0, 0.0, 0.0] if abs(ax[0]) < 0.9 else [0.0, 1.0, 0.0])
    perp /= np.linalg.norm(perp)
    perp2 = np.cross(ax, perp)
    ins = site.get("ins", INSERT)
    depth = ins["hole_depth"]
    hole = [at + ax * d for d in np.linspace(0.3, depth - 0.3, 6)]
    floor = [at + ax * (depth + 0.6)]
    rr = ins["hole_d"] / 2.0 + 1.0                        # past a teardrop hole's point (r * 1.41)
    wall = [at + ax * d + rr * (math.cos(t) * perp + math.sin(t) * perp2)
            for d in np.linspace(0.5, ins["length"], 5) for t in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
    a, b, w = inside(shape, hole), inside(shape, floor), inside(shape, wall)
    ok = (not a.any()) and b.all() and w.all()
    return ok, f"hole clear {int((~a).sum())}/{len(a)}, floor {bool(b.all())}, PETG 1.0 out round it {int(w.sum())}/{len(w)}"


@functools.lru_cache(maxsize=4)
def _checks(c: Shell) -> Dict[str, Tuple[bool, str]]:
    import trimesh
    shapes = build(c)
    meshes = _meshes(c)
    out: Dict[str, Tuple[bool, str]] = {}

    # Closed solids, one each.
    bad = []
    for label, tris in meshes.items():
        m = trimesh.Trimesh(**trimesh.triangles.to_kwargs(tris))
        m.merge_vertices(digits_vertex=5)
        n = len(_solids(shapes[label]))
        if not m.is_watertight or n != 1:
            bad.append(f"{label}: watertight {m.is_watertight}, {n} solids")
    out["closed_solids"] = (not bad, "; ".join(bad) or f"{len(meshes)} solids, each one closed body")

    printed = [k for k, v in MADE.items() if v[3] == "print"]
    # Bed fit, overhang, wall thickness, per printed part.
    fit, oh, walls = [], {}, {}
    for label in printed:
        t = print_triangles(label, c)
        v = t.reshape(-1, 3)
        size = v.max(0) - v.min(0)
        fit.append((label, size, bool(np.all(size <= np.array(BED_MM) + 1e-6))))
        oh[label] = overhang(t)
        walls[label] = wall_thickness(t)
    out["fits_bed"] = (all(f[2] for f in fit), "; ".join(f"{k} {s[0]:.1f} x {s[1]:.1f} x {s[2]:.1f}" for k, s, _ in fit)
                       + f" (bed {BED_MM[0]:g} x {BED_MM[1]:g} x {BED_MM[2]:g})")
    limit = 60.0      # mm2 of sloped overhang a part may have and still need no support (hole tops, facets)
    out["overhang"] = (all(o["sloped_mm2"] <= limit for o in oh.values()),
                       "; ".join(f"{k}: support-needing {o['sloped_mm2']} mm2, horizontal (bridges/ledges) {o['horizontal_mm2']} mm2"
                                 for k, o in oh.items()) + f" (sloped limit {limit:g} mm2: round hole tops)")
    out["min_wall"] = (all(w["min_mm"] >= WALL_RULE_MM - 0.02 for w in walls.values()),
                       "; ".join(f"{k}: min {w['min_mm']}, 1st pct {w['p1_mm']}, median {w['median_mm']} ({w['rays']} rays)"
                                 for k, w in walls.items()) + f" (rule {WALL_RULE_MM} mm)")

    # Inserts.
    rows, okall = [], True
    for s in insert_sites(c):
        ok, d = _check_insert(shapes[s["part"]], s)
        okall &= ok
        if not ok:
            rows.append(f"{s['label']}: {d}")
    n3 = sum(1 for s_ in insert_sites(c) if s_["ins"] is INSERT)
    n2 = len(insert_sites(c)) - n3
    out["insert_bosses"] = (okall, "; ".join(rows) or f"{n3} M3 inserts (a {INSERT['hole_d']} mm hole {INSERT['hole_depth']} deep) "
                            f"and {n2} M2 ({INSERT_M2['hole_d']} x {INSERT_M2['hole_depth']}): every hole blind, 1.0+ of PETG round it")

    # Screws: long enough, short enough.
    rows = []
    for s in screw_sites(c):
        if s["label"].startswith("screw_bumper"):
            room = 6.0
        else:
            room = INSERT["hole_depth"] - 0.3
        if not (s["thread_in_mm"] <= room + 1e-9):
            rows.append(f"{s['label']}: {s['thread_in_mm']} into {room}")
    out["screw_lengths"] = (not rows, "; ".join(rows) or "every shell screw ends inside its hole: " +
                            ", ".join(sorted({f"{s['size']} x {s['length']} ({s['thread_in_mm']} in)" for s in screw_sites(c)})))

    # Envelope and the bodies.
    allv = np.concatenate([t.reshape(-1, 3) for t in meshes.values()])
    env = sk.ENVELOPE
    lo, hi = allv.min(0), allv.max(0)
    tol = 1e-3
    out["inside_envelope"] = (bool(lo[0] >= env.x0 - tol and hi[0] <= env.x1 + tol and lo[1] >= env.y0 - tol and hi[1] <= env.y1 + tol
                                   and lo[2] >= env.z0 - tol and hi[2] <= env.z1 + tol), f"shell {np.round(lo, 2).tolist()} to {np.round(hi, 2).tolist()}")

    def bb(labels):
        v = np.concatenate([meshes[k].reshape(-1, 3) for k in labels])
        return v.min(0), v.max(0)
    fills = []
    for name, labels, target in (("UPPER_BODY", ["shell_upper_left", "shell_upper_right"], sk.UPPER_BODY),
                                 ("LOWER_BODY", ["shell_lower_front", "shell_lower_rear", "bumper_front", "bumper_rear"], sk.LOWER_BODY),
                                 ("TOP_SLOT", ["top_socket"], sk.TOP_SLOT)):
        v = np.concatenate([meshes[k].reshape(-1, 3) for k in labels])
        if name == "LOWER_BODY":
            # its tongues stand up inside the upper body's walls: within UPPER_BODY, not above LOWER_BODY
            up_ = v[v[:, 2] > target.z1 + 1e-3]
            u = sk.UPPER_BODY
            if len(up_) and not (up_[:, 0].min() >= u.x0 and up_[:, 0].max() <= u.x1 and up_[:, 1].min() >= u.y0
                                 and up_[:, 1].max() <= u.y1 and up_[:, 2].max() <= u.z1):
                fills.append((name + " (tongues)", False, up_.min(0), up_.max(0)))
            v = v[v[:, 2] <= target.z1 + 1e-3]
        a, b = v.min(0), v.max(0)
        want = np.array([target.x0, target.y0, target.z0]), np.array([target.x1, target.y1, target.z1])
        ok = bool(np.all(np.abs(a - want[0]) < 0.05) and np.all(np.abs(b - want[1]) < 0.05))
        fills.append((name, ok, a, b))
    out["fills_bodies"] = (all(f[1] for f in fills), "; ".join(f"{n} {'filled' if ok else 'NOT filled'}: {np.round(a, 2).tolist()}"
                                                                 f" to {np.round(b, 2).tolist()}" for n, ok, a, b in fills))

    # Gap to every other zone.
    shell_tris = np.concatenate(list(meshes.values()))
    exempt: Dict[str, List[sk.Box]] = {}
    for cl in claims(c):
        exempt.setdefault(cl["zone"], []).append(cl["box"])
    for iss in skeleton_issues(c):
        for zname, bx in zip(iss.get("zones", []), iss.get("box", [])):
            exempt.setdefault(zname, []).append(bx)
    unexplained, claimed = [], []
    g = c.gap
    for owner, zs in sk.ZONES.items():
        for name, zb in zs:
            grown = sk.Box(zb.x0 - g, zb.x1 + g, zb.y0 - g, zb.y1 + g, zb.z0 - g, zb.z1 + g)
            pieces = [grown]
            for ex in exempt.get(f"{owner}.{name}", []):
                pieces = [q for p in pieces for q in _box_minus(p, ex)]
            for p in pieces:
                lo_, hi_ = np.array([p.x0, p.y0, p.z0]), np.array([p.x1, p.y1, p.z1])
                if np.any(hi_ - lo_ <= 1e-9):
                    continue
                hit = _tri_box_overlap(shell_tris, lo_, hi_)
                if hit.any():
                    v = shell_tris[hit].reshape(-1, 3)
                    unexplained.append(f"{owner}.{name}: {int(hit.sum())} faces near {np.round(v.mean(0), 1).tolist()}")
            if exempt.get(f"{owner}.{name}"):
                claimed.append(f"{owner}.{name} ({len(exempt[f'{owner}.{name}'])})")
    out["zones_gap"] = (not unexplained, ("; ".join(unexplained) + " | " if unexplained else "")
                        + f"{g:g} mm clear of every zone except the claimed interfaces and skeleton issues: " + ", ".join(claimed))

    # Interfaces.
    sec = {k: shapes[k] for k in SECTIONS}
    ub_l, ub_r, lb_f, lb_r = (sec[k] for k in SECTIONS)
    rows = {}

    def empty_in_sections(pts):
        pts = np.asarray(pts, float)
        hit = np.zeros(len(pts), bool)
        for s in sec.values():
            hit |= inside(s, pts)
        return not hit.any(), int(hit.sum())

    ub = sk.UPPER_BODY
    for m in sk.SHELL_MOUNTS:
        site = next(s for s in insert_sites(c) if s.get("mount") == m.name)
        ok, d = _check_insert(shapes[site["part"]], site)
        rows[f"mount_{m.name}"] = (ok, d)
    for side, (x, y, z) in sk.SHOULDERS.items():
        sy = 1 if y > 0 else -1
        rr = sk.SHELL_ARM_OPENING_D / 2.0
        ring = shapes[f"shoulder_ring_{side}"]
        pts_in = [(x + (rr - 0.3) * math.cos(t), sy * (ub.y1 - c.wall / 2), z + (rr - 0.3) * math.sin(t)) for t in np.linspace(0, 2 * math.pi, 24)]
        pts_ring = [(x + (rr + 0.5) * math.cos(t), sy * (ub.y1 - c.wall / 2), z + (rr + 0.5) * math.sin(t)) for t in np.linspace(0, 2 * math.pi, 24)]
        e, n = empty_in_sections(pts_in)
        e2 = not inside(ring, pts_in).any()
        r2 = inside(ring, pts_ring).all()
        rows[f"shoulder_opening_{side}"] = (e and e2 and r2, f"D {sk.SHELL_ARM_OPENING_D:g} clear {e and e2}, ring round it {r2}")
    op = face_opening()
    gx = ub.x1 - c.wall / 2
    pts = [(gx, y, z) for y in np.linspace(-65, 65, 27) for z in np.linspace(150, 260, 45) if op.buffer(-0.5).contains(Point(y, z))]
    e, n = empty_in_sections(pts)
    cov = inside(shapes["face_cover"], [(ub.x1 - c.cover_t / 2, y, z) for _, y, z in pts]).all()
    rows["face_window"] = (e and cov, f"{len(pts)} points of the window and camera bar open {e}, under the cover {cov}")
    lens = sk.CAMERA["lens_at"]
    e, _ = empty_in_sections([(lens[0] - c.wall / 2, lens[1], lens[2])])
    rows["camera"] = (e, f"lens at {lens} open behind the cover {e}")
    (_, ty, tz), (tw, th) = sk.CHEST_TOUCH["centre"], sk.CHEST_TOUCH["size"]
    pads = [(y, z) for y in np.linspace(-(tw / 2 - 1), tw / 2 - 1, 11) for z in np.linspace(tz - th / 2 + 1, tz + th / 2 - 1, 6)
            if abs(y) > 0.3]
    half_of = lambda y: sec["shell_upper_left" if y > 0 else "shell_upper_right"]   # noqa: E731
    skin = all(inside(half_of(y), [(ub.x1 - (c.wall - c.touch_skin) - 0.1, y, z), (ub.x1 - c.wall + 0.1, y, z)]).all()
               for y, z in pads)
    recess = not any(inside(half_of(y), [(ub.x1 - (c.wall - c.touch_skin) + 0.1, y, z)]).any() for y, z in pads)
    # flat and clean inside: nothing of the shell within 3 mm behind the inner face
    flat = not any(empty_in_sections([(ub.x1 - c.wall - d, y, z) for d in (0.1, 1.5, 3.0)])[1] for y, z in pads)
    rows["chest_touch"] = (skin and recess and flat, f"skin {c.touch_skin} over all of {tw:g} x {th:g} (a 0.8 recess outside) "
                           f"{skin and recess}, inside flat and clean 3 mm deep {flat}")
    (_, gy, gz), (gw, gh) = sk.SPEAKER_GRILLE["centre"], sk.SPEAKER_GRILLE["size"]
    pitch = gh / 6.0
    e, _ = empty_in_sections([(gx, y, gz - gh / 2 + pitch * (k + 0.5)) for k in range(6) for y in np.linspace(-gw / 2 + 2, gw / 2 - 2, 9)])
    rows["speaker_grille"] = (e, f"six {c.speaker_slot_w} mm slots across {gw:g} x {gh:g}: open {e}")
    hz1 = _zone("electrical", "head").z1
    e, _ = empty_in_sections([(x, y, z) for x, y, _ in sk.MIC_HOLES for z in np.linspace(hz1 + 0.2, ub.z1 - 0.1, 10)])
    tube = all(inside(half_of(y), [(x + (c.mic_duct_d / 2 + c.mic_duct_wall / 2) * math.cos(a_), y + (c.mic_duct_d / 2 + c.mic_duct_wall / 2)
                                    * math.sin(a_), z) for a_ in np.linspace(0, 2 * math.pi, 8, endpoint=False)
                                   for z in (hz1 + 0.5, ub.z1 - c.wall - 0.5)]).all() for x, y, _ in sk.MIC_HOLES)
    rows["mic_holes"] = (e and tube, f"{len(sk.MIC_HOLES)} holes through the top, each with a {c.mic_duct_d:g} mm duct down to the "
                                     f"mic at z {hz1:g}: open {e}, duct walls {tube}")
    cx0, cx1, cz0, cz1 = bumper_channel(c)
    led_ok = True
    for sx, lab in ((1, "front"), (-1, "rear")):
        b, d = shapes[f"bumper_{lab}"], shapes[f"diffuser_{lab}"]
        ys = np.linspace(-c.led_window_half + 2, c.led_window_half - 2, 7)
        behind = [(sx * x, y, z) for x in np.linspace(cx0 + 0.1, cx1 - 0.1, 5) for y in ys
                  for z in np.linspace(sk.LED_STRIP_Z - c.strip_w / 2 + 1.2, sk.LED_STRIP_Z + c.strip_w / 2 - 1.2, 4)]
        led_ok &= not inside(b, behind).any() and not inside(d, behind).any()                   # strip_clear behind the diffuser
        led_ok &= inside(d, [(sx * (sk.LOWER_BODY.x1 - 0.6), y, sk.LED_STRIP_Z) for y in ys]).all()   # the diffuser in the window
        led_ok &= not inside(b, [(sx * (sk.LOWER_BODY.x1 - 0.6), y, sk.LED_STRIP_Z) for y in ys]).any()
    led_ok &= cx1 - cx0 >= c.strip_clear - 1e-9
    rows["led_strip"] = (led_ok, f"channel at z {sk.LED_STRIP_Z:g} for a {c.strip_w:g} mm strip, {cx1 - cx0:.1f} mm clear behind the "
                                 f"diffuser, which fills the light window |y| <= {c.led_window_half:g}, front and rear: {led_ok}")
    hv = meshes["rear_hatch"].reshape(-1, 3)
    size_ok = abs((hv[:, 1].max() - hv[:, 1].min()) - (sk.REAR_HATCH["size"][0] - c.slide_clear)) < 0.05 and \
        abs((hv[:, 2].max() - hv[:, 2].min()) - (sk.REAR_HATCH["size"][1] - c.slide_clear)) < 0.05
    ins_ok = all(_check_insert(shapes[s["part"]], s)[0] for s in insert_sites(c) if s["label"].startswith("insert_hatch"))
    e, _ = empty_in_sections([(ub.x0 + c.hatch_t + 0.3, sk.REAR_HATCH["centre"][1], sk.REAR_HATCH["centre"][2])])
    rows["rear_hatch"] = (size_ok and ins_ok and e, f"hatch {sk.REAR_HATCH['size'][0]:g} x {sk.REAR_HATCH['size'][1]:g} (less "
                          f"{c.slide_clear}), 4 M3 into inserts {ins_ok}, access opening behind it {e}")
    (_, vy, _), (vw, vh) = sk.REAR_VENT["centre"], sk.REAR_VENT["size"]
    vp = vh / 4.0
    e, _ = empty_in_sections([(ub.x0 + c.wall / 2, y, c.vent_z - vh / 2 + vp * (k + 0.5)) for k in range(4) for y in np.linspace(-vw / 2 + 2, vw / 2 - 2, 7)])
    rows["rear_vent"] = (e, f"four slots {vw:g} x {vh:g} at z {c.vent_z:g} (moved from {sk.REAR_VENT['centre'][2]:g}: "
                            f"see skeleton_issues): open {e}")
    lbr = sec["shell_lower_rear"]
    for name, spec, open_ in (("usb_c", sk.USB_C, USB_OPENING), ("power_switch", sk.POWER_SWITCH, BUTTON_HOLE)):
        x, y, z = spec["at"]
        axis = np.asarray(spec["axis"], float)
        e, _ = empty_in_sections([np.asarray(spec["at"]) - axis * d for d in np.linspace(-1.0, PORT_FACE_IN + 0.3, 8)])
        p0, p1, p2, p3 = open_(y, z).buffer(-0.3).bounds
        e2, _ = empty_in_sections([(x + PORT_FACE_IN / 2, yy, zz) for yy in (p0, p2) for zz in (z,)])
        rim = inside(lbr, [(x + PORT_FACE_IN - 0.2, yy, z) for yy in (p0 - 1.5, p2 + 1.5)]).all()
        rows[name] = (e and e2 and rim, f"at {spec['at']}: recessed {PORT_FACE_IN:g} mm to the board's face at x "
                      f"{x + PORT_FACE_IN:g} (electrical), opening {'13.2 x 7.4 (the plug overmould)' if name == 'usb_c' else 'D10.4 under a D16 finger pocket'}: "
                      f"{e and e2 and rim}")
    boards = []
    for s_ in insert_sites(c):
        if s_["label"].startswith(("insert_charger", "insert_button")):
            boards.append(_check_insert(lbr, s_)[0])
    match = "electrical.py not checked"
    try:
        import electrical as el_mod
        es = el_mod.solids()
        chg, b2 = es["el_CHG1"].reshape(-1, 3), es["el_B2"].reshape(-1, 3)
        ok_c = abs(chg[:, 2].min() - CHARGER_BOARD["z_under"]) < 0.05 and chg[:, 0].min() <= CHARGER_BOARD["x_near"] - 4.0 + 0.05
        ok_b = abs(b2[:, 0].min() - BUTTON_BOARD["x_face"]) < 0.05 and b2[:, 1].min() <= 13.05 and b2[:, 1].max() >= 26.95
        match = f"electrical.py's el_CHG1 / el_B2 where the seats expect them: {bool(ok_c)} / {bool(ok_b)}"
        boards += [bool(ok_c), bool(ok_b)]
    except Exception as exc:           # electrical.py is another agent's: report, do not fail on it
        match = f"electrical.py not readable ({type(exc).__name__})"
    rows["port_boards"] = (all(boards), f"charger shelf and button posts, 4 M2 inserts: {all(boards)}; {match}")
    open_area = inlet_open(c).area
    hx_in = ub.x0 + c.hatch_t
    seat = inlet_seat_rect(c)
    ring = seat.difference(inlet_rect(c))
    ring_pts = [(p_.x, p_.y) for p_ in (ring.representative_point(),)] + [
        (yy, zz) for yy in np.linspace(seat.bounds[0] + 0.5, seat.bounds[2] - 0.5, 15) for zz in (seat.bounds[1] + 0.5, seat.bounds[3] - 0.5)]
    flat_on = inside(shapes["rear_hatch"], [(hx_in - 0.1, y, z) for y, z in ring_pts]).all()
    clear, _ = empty_in_sections([(hx_in + d, y, z) for y, z in ring_pts for d in (0.2, 1.0, 1.9)])
    e, _ = empty_in_sections([(hx_in + 0.5, y, inlet_z(c)) for y in np.linspace(-29, 29, 9)])
    rows["rear_inlet"] = (open_area >= 800 - 1e-6 and flat_on and clear and e,
                          f"in the hatch at z {inlet_z(c):g} (skeleton {sk.REAR_INLET['centre'][2]:g}, see skeleton_issues): "
                          f"{open_area:.0f} mm2 open (two {c.inlet_rib_w:g} mm ribs); the duct's {c.inlet_seat[0]:g} x "
                          f"{c.inlet_seat[1]:g} seat ring flat on the hatch's inside {flat_on}, the shell clear of it {clear}")
    sv = meshes["top_socket"].reshape(-1, 3)
    ts = sk.TOP_SLOT
    sock_ok = np.allclose(sv.min(0), [ts.x0, ts.y0, ts.z0], atol=0.05) and np.allclose(sv.max(0), [ts.x1, ts.y1, ts.z1], atol=0.05)
    ins_ok = all(_check_insert(shapes[s["part"]], s)[0] for s in insert_sites(c) if s["label"].startswith("insert_socket"))
    rows["top_slot"] = (bool(sock_ok and ins_ok), f"socket fills TOP_SLOT {bool(sock_ok)}, 4 M3 into inserts {ins_ok}, "
                        f"header slot {c.header_slot[0]} x {c.header_slot[1]}")
    bv = np.concatenate([meshes["bumper_front"].reshape(-1, 3), meshes["bumper_rear"].reshape(-1, 3)])
    bz_ok = abs(bv[:, 2].min() - sk.BUMPER_Z[0]) < 0.05 and abs(bv[:, 2].max() - sk.BUMPER_Z[1]) < 0.05
    rows["bumper"] = (bool(bz_ok), f"TPU front and rear, z {bv[:, 2].min():.2f}-{bv[:, 2].max():.2f} (BUMPER_Z {sk.BUMPER_Z}), "
                                   f"{c.bumper_t:g} thick (BUMPER_T {sk.BUMPER_T:g}: see skeleton_issues)")
    for k, v in rows.items():
        out[f"interface_{k}"] = v

    fit = internal_fit(meshes)
    out["internal_fit"] = (not fit["interference"], ("interference: " + ", ".join(f"{a} x {b}" for a, b in fit["interference"]) + "; "
                                                     if fit["interference"] else "") + f"{len(fit['touching'])} pairs touch "
                           "(seats, inserts in holes, screws in inserts), none overlap")

    # Mass.
    ps = parts(c)
    made = [p for p in ps if p["kind"] == "made"]
    shell_g = sum(p["mass_g"] for p in made if p["process"] == "print")
    total_g = sum(p["mass_g"] for p in ps)
    area = sum(trimesh.Trimesh(**trimesh.triangles.to_kwargs(meshes[k])).area for k in SECTIONS) / 2.0
    lo_t, hi_t = 350.0, 500.0
    out["mass"] = (lo_t <= shell_g <= hi_t and total_g <= MASS_CEILING_G, f"printed shell + bumpers {shell_g:.0f} g (target {lo_t:g}-{hi_t:g}); "
                   f"with the cover, inserts and screws {total_g:.0f} g (the lead's ceiling {MASS_CEILING_G:g}); solid-body volumes x "
                   f"density (a {c.wall:g} mm wall prints solid). "
                   f"Each 0.5 mm of wall over the {area / 100:.0f} cm2 of the four sections is {area * 0.5 / 1000 * PETG_G_CM3:.0f} g")
    import machine as cnc_machine
    import preflight as cnc_preflight
    lm = cnc_machine.load(str(HERE.parents[1] / "machines/longmill_mk2_30x30/longmill_mk2_30x30.grbl"))
    rep = cnc_preflight.preflight(lm, cover_program(c))
    out["face_cover_preflight"] = (rep["verdict"] != "fail", f"LongMill preflight: {rep['verdict']}, {rep['error_count']} errors, "
                                   f"work X {rep['bounds']['x']['work']}, Y {rep['bounds']['y']['work']}")
    out["skeleton_issues"] = (False, f"{len(skeleton_issues(c))} things skeleton.py should change (shell.skeleton_issues())")
    return out


def checks(r=None) -> Dict[str, Tuple[bool, str]]:
    return dict(_checks(config(r)))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Write the SparkRover shell's jobs, parts and checks.")
    p.add_argument("--out", required=True)
    a = p.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    c = Shell()
    for label, data in jobs(c).items():
        if isinstance(data, bytes):
            (out / f"{label}.stl").write_bytes(data)
        else:
            (out / f"{label}.nc").write_text(data, encoding="utf-8", newline="\n")
    (out / "parts.json").write_text(json.dumps(parts(c), indent=1), encoding="utf-8")
    rep = {"shell": asdict(c), "checks": {k: {"ok": v[0], "detail": v[1]} for k, v in checks(c).items()},
           "skeleton_issues": [{k: v for k, v in i.items() if k != "box"} for i in skeleton_issues(c)],
           "steps": STEPS}
    (out / "shell.json").write_text(json.dumps(rep, indent=1, default=str), encoding="utf-8")
    for k, v in rep["checks"].items():
        print(f"{'ok ' if v['ok'] else 'BAD'} {k}: {v['detail']}")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
