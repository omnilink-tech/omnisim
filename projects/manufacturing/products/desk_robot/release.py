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

"""The desk robot's release: everything someone needs to make it on real machines.

    python projects/manufacturing/products/desk_robot/release.py --made .tmp/desk_robot

writes `release/`:

- drawings/   one SVG per made part: top view, section, hole table, title block (mm)
- cam/        the machine files the twins ran: the two plates' laser G-code, the hubs'
              CNC program, the print plate's G-code for the SV08
- models/     the made parts as STL (mm), and the print plate as sliced
- BOM.md      made and bought parts, quantities, materials, masses, sources
- REPORT.md   each part as the twins made it, measured against its drawing, and the
              assembled robot's drive test
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import PathPatch  # noqa: E402
from matplotlib.path import Path as MPath  # noqa: E402
from shapely.geometry import LineString, MultiPolygon, Polygon  # noqa: E402

import design  # noqa: E402
from design import BOUGHT, Robot  # noqa: E402

HERE = Path(__file__).resolve().parent
RELEASE = HERE / "release"
INK, CUT, HATCH = "#1b1f24", "#c0392b", "#9aa4ad"


def _patch(poly, face="none", edge=INK, lw=0.9, hatch=None):
    polys = list(poly.geoms) if isinstance(poly, MultiPolygon) else [poly]
    verts, codes = [], []
    for p in polys:
        for ring in [p.exterior, *p.interiors]:
            pts = list(ring.coords)
            verts += pts
            codes += [MPath.MOVETO] + [MPath.LINETO] * (len(pts) - 2) + [MPath.CLOSEPOLY]
    return PathPatch(MPath(verts, codes), facecolor=face, edgecolor=edge, lw=lw, hatch=hatch)


def _dim(ax, a, b, text, offset, horizontal=True):
    """A dimension line from a to b, offset perpendicular, with its value."""
    (x0, y0), (x1, y1) = a, b
    if horizontal:
        y = offset
        ax.annotate("", (x0, y), (x1, y), arrowprops=dict(arrowstyle="<->", lw=0.6, color=INK))
        ax.plot([x0, x0], [y0, y], lw=0.3, color=INK)
        ax.plot([x1, x1], [y1, y], lw=0.3, color=INK)
        ax.text((x0 + x1) / 2, y, text, ha="center", va="bottom", fontsize=7, color=INK)
    else:
        x = offset
        ax.annotate("", (x, y0), (x, y1), arrowprops=dict(arrowstyle="<->", lw=0.6, color=INK))
        ax.plot([x0, x], [y0, y0], lw=0.3, color=INK)
        ax.plot([x1, x], [y1, y1], lw=0.3, color=INK)
        ax.text(x, (y0 + y1) / 2, text, ha="right", va="center", fontsize=7, rotation=90, color=INK)


def _holes(poly) -> List[Tuple[float, float, str]]:
    """Every interior ring: its centre and what it is (a diameter, or a slot's size)."""
    polys = list(poly.geoms) if isinstance(poly, MultiPolygon) else [poly]
    out = []
    for p in polys:
        for ring in p.interiors:
            h = Polygon(ring)
            x0, y0, x1, y1 = h.bounds
            w, l = x1 - x0, y1 - y0
            round_ = abs(w - l) < 0.05 and abs(h.area - math.pi * (w / 2) ** 2) < 0.03 * h.area
            label = f"⌀{w:.2f}" if round_ else f"{w:.2f} × {l:.2f}"
            if not round_ and abs(h.area - math.pi * (w / 2) ** 2) < 0.1 * h.area:
                label = f"⌀{w:.2f} D ({h.area:.2f} mm²)"
            out.append((round(h.centroid.x, 3) + 0.0, round(h.centroid.y, 3) + 0.0, label))
    return out


def _section(bands, y: float = 0.0) -> List[Polygon]:
    """Section through a band stack along the line at y: rectangles in (x, z)."""
    rects = []
    for z0, z1, p in bands:
        x0, _, x1, _ = p.bounds
        hit = p.intersection(LineString([(x0 - 1, y), (x1 + 1, y)]))
        segs = [hit] if hit.geom_type == "LineString" else [g for g in getattr(hit, "geoms", []) if g.geom_type == "LineString"]
        for s in segs:
            if s.is_empty:
                continue
            xs = [c[0] for c in s.coords]
            rects.append(Polygon([(min(xs), z0), (max(xs), z0), (max(xs), z1), (min(xs), z1)]))
    return rects


def drawing(path: Path, title: str, top, section_bands=None, thickness: Optional[float] = None, meta: Sequence[str] = (),
            engraving=None) -> None:
    fig = plt.figure(figsize=(11.69, 8.27))          # A4 landscape
    ax = fig.add_axes([0.05, 0.30, 0.55 if section_bands else 0.9, 0.62])
    ax.add_patch(_patch(top, face="#f4efe6"))
    if engraving is not None:
        ax.add_patch(_patch(engraving, face=HATCH, edge="none"))
    x0, y0, x1, y1 = top.bounds
    pad = 0.12 * max(x1 - x0, y1 - y0)
    _dim(ax, (x0, y1), (x1, y1), f"{x1 - x0:.2f}", y1 + pad * 0.5)
    _dim(ax, (x0, y0), (x0, y1), f"{y1 - y0:.2f}", x0 - pad * 0.5, horizontal=False)
    holes = _holes(top)
    for k, (hx, hy, _) in enumerate(holes, 1):
        ax.text(hx, hy, f"H{k}", fontsize=5.5, ha="center", va="center", color=CUT)
    ax.plot([x0 - pad * 0.2, x1 + pad * 0.2], [0, 0], lw=0.3, ls="-.", color=HATCH)
    ax.plot([0, 0], [y0 - pad * 0.2, y1 + pad * 0.2], lw=0.3, ls="-.", color=HATCH)
    ax.set_xlim(x0 - pad, x1 + pad)
    ax.set_ylim(y0 - pad, y1 + pad)
    ax.set_aspect("equal")
    ax.set_title("TOP (datum X0 Y0 at the centre lines)", fontsize=8, loc="left")
    ax.tick_params(labelsize=6)
    if section_bands:
        sx = fig.add_axes([0.64, 0.42, 0.32, 0.45])
        from shapely.ops import unary_union
        rects = _section(section_bands)
        sx.add_patch(_patch(unary_union([rc.buffer(1e-4, join_style="mitre") for rc in rects]).buffer(-1e-4, join_style="mitre"),
                            face=HATCH, lw=0.7, hatch="////"))
        zs = [c[1] for rct in rects for c in rct.exterior.coords]
        xs = [c[0] for rct in rects for c in rct.exterior.coords]
        sx.set_xlim(min(xs) - 2, max(xs) + 2)
        sx.set_ylim(min(zs) - 2, max(zs) + 2)
        sx.set_aspect("equal")
        sx.set_title("SECTION A-A (through Y0, Z up from the print bed)", fontsize=8, loc="left")
        sx.tick_params(labelsize=6)
        _dim(sx, (min(xs), max(zs)), (max(xs), max(zs)), f"{max(xs) - min(xs):.2f}", max(zs) + 1.0)
        _dim(sx, (min(xs), min(zs)), (min(xs), max(zs)), f"{max(zs) - min(zs):.2f}", min(xs) - 1.0, horizontal=False)
    tx = fig.add_axes([0.05, 0.03, 0.9, 0.22])
    tx.axis("off")
    fmt = lambda v: f"{(0.0 if abs(v) < 5e-4 else v):+8.3f}"   # noqa: E731
    rows = [f"H{k}  X {fmt(hx)}  Y {fmt(hy)}  {label}" for k, (hx, hy, label) in enumerate(holes, 1)]
    cols = [rows[i:i + 9] for i in range(0, len(rows), 9)] or [["(no holes)"]]
    for c, col in enumerate(cols[:3]):
        tx.text(0.0 + 0.27 * c, 1.0, "\n".join(col), fontsize=6.5, family="monospace", va="top")
    block = [title, *meta] + ([f"Thickness {thickness:g} mm"] if thickness else []) + \
            ["Units mm. Unless stated: holes through, ±0.1", "OmniSim desk robot · projects/manufacturing/products/desk_robot/design.py"]
    tx.text(1.0, 1.0, "\n".join(block), fontsize=7.5, ha="right", va="top",
            bbox=dict(boxstyle="square,pad=0.6", fc="white", ec=INK, lw=0.8))
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)


def write_drawings(r: Robot, out: Path) -> List[str]:
    made = {p["part"]: p for p in design.made_parts(r)}
    clamp, rim, tray = design.motor_clamp(r), design.rim(r, step=0.1), design.tray(r)
    from shapely.ops import unary_union
    specs = [
        ("bottom_plate", "Bottom plate", design.bottom_plate(r), None, r.plate_t,
         ["Laser cut, 3 mm birch plywood, qty 1", "Sienci LaserBeam 7W: 100 %, 2.2 mm/s, 3 passes, Z -1 mm/pass",
          f"Cut with a {design.LASER_KERF_MM} mm kerf offset (cam/bottom_plate.gcode)"], None),
        ("top_plate", "Top plate", design.top_plate(r), None, r.plate_t,
         ["Laser cut, 3 mm birch plywood, qty 1", "Engrave the name: 100 %, 25.4 mm/s, 0.1 mm line interval",
          f"Cut with a {design.LASER_KERF_MM} mm kerf offset (cam/top_plate.gcode)"], design.name_engraving(r)),
        ("hub", "Wheel hub", design.hub_outline(r), None, r.hub_t,
         ["CNC milled, 6061 aluminium, qty 2", "H1: D-bore for the 3 mm D-shaft (flat 1.06 from centre)",
          "T3 1 mm end mill for H1-H4, T1 1/4 in for the outline (cam/hubs.nc)"], None),
        ("motor_clamp", "Motor clamp", unary_union([p for _, _, p in clamp]), clamp, None,
         ["3D printed, PLA, qty 2, print as shown (bridge up)", "Fits the Pololu micro metal gearbox, 12 x 10 mm",
          f"{made['motor_clamp']['g']:.2f} g solid"], None),
        ("rim", "Wheel rim", unary_union([p for _, _, p in rim]), rim, None,
         ["3D printed, PLA, qty 2, hub face down", f"Tyre groove root ⌀{r.groove_root_d}: O-ring 34 x 2 NBR stretched 4.1 %",
          "Screw holes ⌀1.7: M2 self-tapping"], None),
        ("tray", "Electronics tray", unary_union([p for _, _, p in tray]), tray, None,
         ["3D printed, PLA, qty 1", "Pockets: XIAO ESP32-S3 (21.4 x 18.2), DRV8833 carrier (20.7 x 13.1)"], None),
    ]
    files = []
    for name, title, top, bands, t, meta, engraving in specs:
        path = out / "drawings" / f"{name}.svg"
        drawing(path, title, top, bands, t, meta, engraving)
        files.append(path.name)
    return files


def bom_md(r: Robot) -> str:
    b = design.bom(r)
    lines = ["# Desk robot: bill of materials", "",
             f"Total mass, as designed: **{b['total_g']} g** ({b['note']}).", "",
             "## Made", "", "| Part | Process | Material | Qty | Mass each (g) | Drawing | Machine file |",
             "|---|---|---|---|---|---|---|"]
    cam = {"bottom_plate": "cam/bottom_plate.gcode", "top_plate": "cam/top_plate.gcode", "hub": "cam/hubs.nc",
           "motor_clamp": "cam/print_plate.gcode", "rim": "cam/print_plate.gcode", "tray": "cam/print_plate.gcode"}
    for p in b["made"]:
        lines.append(f"| {p['part']} | {p['process']} | {p['material']} | {p['qty']} | {p['g']:.2f} | "
                     f"[drawings/{p['part']}.svg](drawings/{p['part']}.svg) | [{cam[p['part']]}]({cam[p['part']]}) |")
    lines += ["", "## Bought", "", "| Item | Spec | Qty | Mass each (g) | Source |", "|---|---|---|---|---|"]
    for p in b["bought"]:
        note = f" ({p['note']})" if p.get("note") else ""
        lines.append(f"| {p['part']} | {p['spec']}{note} | {p['qty']} | {p['g']} | {p['source']} |")
    lines += ["", "## Stock", "",
              "- 3 mm birch plywood, two sheets of at least 98 x 88 mm (one per plate).",
              f"- 6061 aluminium flat bar, 4 mm, one blank {design.HUB_STOCK_MM[0]:g} x {design.HUB_STOCK_MM[1]:g} mm "
              "(both hubs), with blue tape and CA glue to hold it to the spoilboard.",
              "- PLA filament, about 13 g (the slicer's figure for the print plate).", ""]
    lines += electrical_bom_md(r)
    return "\n".join(lines)


def electrical_bom_md(r: Robot) -> list:
    """The electrical design's parts, every screwed joint, and the cut list of the routed wires."""
    import bought_parts
    import electrical
    import wiring
    L = ["## Electrical", "", "From `electrical.py`; the schematic, wiring diagram, wire list and power budget are in "
         "[electrical/](electrical/).", "", "| Ref | Part | Pins | Source |", "|---|---|---|---|"]
    for ref, c in electrical.components().items():
        L.append(f"| {ref} | {c.get('part', '')} | {', '.join(c.get('pins', []))} | {c.get('source', '')} |")
    L += ["", "Mounted on the bottom plate (foam tape, no holes):", ""]
    for ref, m in electrical.MOUNTS.items():
        if isinstance(m["at"], tuple):
            L.append(f"- **{ref}** at x {m['at'][0]:g}, y {m['at'][1]:g} mm: {m['fix']}.")
    L += ["", "## Fasteners, joint by joint", "",
          "Each length is the shortest ISO 4762 length that clamps the stack with a full pitch proud of the nut, or "
          "1.5 d into a standoff (`bought_parts.joints`).", "",
          "| Joint | Screw | Clamps (mm) | Nut | Proud of the nut / into the thread (mm) |", "|---|---|---|---|---|"]
    seen = {}
    for j in bought_parts.joints(r):
        key = (j["joint"], j["size"], j["length"])
        seen[key] = seen.get(key, 0) + 1
    for (joint, size, length), n in seen.items():
        j = next(x for x in bought_parts.joints(r) if (x["joint"], x["size"], x["length"]) == (joint, size, length))
        tail = j["proud_mm"] if j["proud_mm"] is not None else j["thread_in_mm"]
        L.append(f"| {joint} x {n} | {size} x {length} | {j['grip_mm']} | {j['nut'] or 'into the standoff'} | {tail} |")
    if wiring.ROUTED.exists():
        wires = json.loads(wiring.ROUTED.read_text(encoding="utf-8"))
        L += ["", "## Wires: the cut list", "",
              f"Routed in 3D through the assembled chassis (`wiring.py`): each wire keeps its insulation "
              f"{wiring.CLEAR_MM} mm off every part, lies along the surfaces, and is cut to its routed length plus "
              f"{wiring.SERVICE_LOOP_MM:g} mm at each end.", "",
              "| Wire | From | To | Net | AWG | Colour | Cut (mm) | Clear of parts (mm) |", "|---|---|---|---|---|---|---|---|"]
        for w in wires:
            L.append(f"| {w['id']} | {'.'.join(w['from'])} | {'.'.join(w['to'])} | {w['net']} | {w['awg']} | {w['colour']} | "
                     f"{w.get('cut_mm', 'NO ROUTE')} | {w.get('clear_of_parts_mm', '')} |")
        L.append(f"\nTotal wire: {sum(w.get('cut_mm', 0) for w in wires)} mm.")
    L += ["", "## The vendors' CAD", "",
          "The renders use the manufacturers' own 3D models where they have been fetched (`python bought_parts.py fetch`). "
          "They are not redistributed here: Pololu states no licence for its models, and Seeed's licence names its "
          "documents, not its models. Our own models, drawn from the vendors' dimension drawings, stand in for them "
          "everywhere else.", "", "| Part | File | SHA-256 | Licence |", "|---|---|---|---|"]
    for name, src in bought_parts.SOURCES.items():
        L.append(f"| {name} | {src['url']} | `{src['sha256'][:16]}...` | {src['licence']} |")
    return L + [""]


def report_md(r: Robot, made: dict, drive: Optional[dict]) -> str:
    import robot as robot_mod
    pred = robot_mod.predictions(r)
    c = design.checks(r)
    L = ["# Desk robot: as made on the twins", "",
         "Every number here is the **twin's**: the parts were made on OmniSim's machine twins and measured from the "
         "engine's own record of the material (the laser and CNC sheets' height maps, the printer's beads). "
         "No part has been made on a real machine yet.", "",
         "## The design's fits", "", "| Check | Value |", "|---|---|"]
    for k, v in c.items():
        L.append(f"| {k} | {v} |")
    lz = made.get("laser", {})
    if lz:
        L += ["", "## Laser: the plates (3 mm plywood, Sienci LaserBeam 7W twin)", "",
              "| Plate | Cut free | Size (mm) | Drawn (mm) | Holes cut | Hole ⌀ error (mm) | Job | Wall (s) |",
              "|---|---|---|---|---|---|---|---|"]
        for name, v in lz.items():
            e = v.get("hole_d_error_mm") or {}
            L.append(f"| {name} | {v.get('cut_free')} | {v.get('size_mm')} | {v.get('drawn_mm')} | {v.get('holes_cut')} | "
                     f"{e.get('min')} to {e.get('max')} | {(v.get('job') or {}).get('lines_total')} lines, "
                     f"{(v.get('job') or {}).get('state')} | {v.get('wall_s')} |")
        eng = (lz.get("top_plate") or {}).get("engraving")
        if eng:
            L.append(f"\nThe engraving: {eng['depth_mm_in_letters']} mm deep in the letters, letters "
                     f"{eng['letters_area_mm2']} mm² (drawn {eng['drawn_area_mm2']}), char {eng['char_in_letters']} "
                     f"inside them and {eng['char_around_letters']} around them.")
    cn = made.get("cnc")
    if cn:
        L += ["", "## CNC: the hubs (6061 aluminium, LongMill twin)", "",
              "| Hub at (mm) | OD (mm) | Drawn | Bore area (mm²) | Drawn | The D-shaft fits |", "|---|---|---|---|---|---|"]
        for h in cn["hubs"]:
            L.append(f"| {h['centre_mm']} | {h['od_mm']} | {h['drawn_od_mm']} | {h['bore_area_mm2']} | "
                     f"{h['drawn_bore_area_mm2']} | {h['shaft_fits']} ({h['shaft_cells_blocked']} of {h['shaft_cells']} cells blocked) |")
        L.append(f"\nCut free: {cn['cut_free']} of 2. Job: {(cn.get('job') or {}).get('lines_total')} lines, "
                 f"{(cn.get('job') or {}).get('state')}, {(cn.get('job') or {}).get('tool_changes')} tool changes.")
    pr = made.get("print")
    if pr:
        d = pr.get("done") or {}
        L += ["", "## Printing: the clamps, rims and tray (PLA, Sovol SV08 twin)", "",
              f"- Planned print time {d.get('planned_s')} s; plastic out of the nozzle {d.get('extruded_mm3')} mm³, "
              f"laid {d.get('laid_mm3')} mm³; nozzle crashes {d.get('nozzle_crashes')}."]
        ins = pr.get("inspect") or {}
        if ins:
            o, h = ins.get("outer_surface_error_mm") or {}, ins.get("height_mm") or {}
            L += [f"- {ins.get('layers_measured')} layers; height {h.get('printed')} mm (drawn {h.get('drawing')}).",
                  f"- Outer walls against the STL: {o.get('p05')} to {o.get('p95')} mm (5th to 95th percentile), "
                  f"mean {o.get('mean')} mm. The first two layers read millimetres off: that is the slicer's "
                  "skirt line around the parts, not the parts."]
    L += ["", "## The assembled robot on Newton", "", "| | Predicted | Measured |", "|---|---|---|",
          f"| Mass (g) | {pred['mass_g']} | (from the BOM) |",
          f"| Top speed, no load (m/s) | {pred['no_load_speed_m_s']} | {(drive or {}).get('straight_speed_m_s')} |",
          f"| Spin rate, no load (rad/s) | {pred['spin_no_load_rad_s']} | {(drive or {}).get('spin_yaw_rate_rad_s')} |",
          f"| Wheel radius (mm) | {pred['wheel_radius_mm']} | {(drive or {}).get('straight_effective_radius_mm')} (speed / wheel rate) |",
          f"| Track (mm) | {pred['track_mm']} | {(drive or {}).get('spin_effective_track_mm')} (from the spin) |"]
    if drive:
        L.append(f"\nHeading drift over the straight run: {drive.get('heading_drift_deg_straight')} degrees.")
        if (drive.get("spin_wheel_rad_s") or 0) > 1.01 * pred["no_load_speed_m_s"] / (pred["wheel_radius_mm"] / 1000):
            L.append(f"\n**Check the motor model:** in the spin the wheels turned at {drive['spin_wheel_rad_s']} rad/s, "
                     "faster than the motors' no-load speed. A motor cannot drive a wheel past it, but the ground can "
                     "back-drive a FREE one: the drive controller must brake above no-load speed, not set its "
                     "available torque to 0 (README, \"Driving it\").")
    return "\n".join(L) + "\n"


def assembly_md(r: Robot, images: bool = False) -> str:
    """The build, step by step, with what each step needs and the check that proves it."""
    c = design.checks(r)
    steps = [
        ("The bottom plate", "Lay the bottom plate flat. Its front is the end with the slot (the motors go over it); "
         "the two holes on the centre line at the back take the caster.",
         ["Check the 13 holes are clear: the M2 holes take an M2 screw, the M3 holes an M3."]),
        ("The caster, under the plate", "Fit the Pololu ball caster UNDER the plate at its two M2 holes near the back "
         "(X -33), with its 1/16 in spacer between caster and plate. M2 x 6 screws from above, nuts below.",
         [f"Caster plus spacer stands {c['caster_mm']} mm: the plate's underside sits {c['plate_underside_mm']} mm up "
          f"on the wheels, so the robot stands level ({c['level_error_mm']} mm)."]),
        ("The motors, in their clamps", "Lay each gearmotor on the plate, shaft outward, its gearbox face 0.5 mm past "
         "the plate's side edge, and press its clamp over the gearbox. Two M2 x 6 screws through the clamp's tabs and the "
         "plate, nuts below.",
         ["The clamp's channel is 12.1 x 10.1 mm over a 12 x 10 mm gearbox: snug, not forced."]),
        ("The electronics tray", "Seat the XIAO ESP32-S3 and the DRV8833 carrier in their pockets, then screw the tray "
         "down at its two tabs (M2 x 6, nuts below). Feed the motor leads up through the slot between the motors.",
         ["Motors to the driver's AOUT1/2 and BOUT1/2; driver inputs to four XIAO GPIOs; VM and GND to the battery."]),
        ("The standoffs", "Four M3 x 25 mm standoffs on the bottom plate's corner holes, M3 x 6 screws from below.", []),
        ("The wheels: hub, rim, tyre", "Screw each aluminium hub to its printed rim with three M2 x 8 screws (they cut "
         "their own thread in the rim's 1.7 mm holes). Roll the 34 x 2 mm O-ring into the rim's groove. Press the wheel "
         "onto the motor's D-shaft, hub first, until the hub stands 1 mm off the gearbox face.",
         [f"The tyre stretches {c['tyre_stretch_pct']} % (Parker's limit 5 %) and the wheel comes to {c['wheel_d_mm']} mm.",
          f"The shaft ends {c['shaft_end_inside_rim_mm']} mm inside the rim, clear of its bore by {c['shaft_clear_of_rim_mm']} mm."]),
        ("The top plate and the battery", "Screw the top plate onto the standoffs (M3 x 6), engraving up, the name to "
         "the front. Strap the 4 x AA holder on with the hook-and-loop strap through the two slots; its leads go down "
         "through the hole at the front left.",
         [f"Track {c['track_mm']} mm, top speed with no load {c['speed_no_load_m_s']} m/s."]),
    ]
    L = ["# Desk robot: assembly", "",
         "Tools: 1.5 and 2.5 mm hex keys, small pliers for the M2 nuts. Parts: see [BOM.md](BOM.md). "
         "Drawings: [drawings/](drawings/).", ""]
    for k, (title, text, checks) in enumerate(steps, 1):
        L += [f"## {k}. {title}", "", text, ""]
        if images:
            L += [f"![Step {k}](assembly/step_{k}.png)", ""]
        L += [f"- {ck}" for ck in checks] + ([""] if checks else [])
    return "\n".join(L)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Write the desk robot's release folder.")
    p.add_argument("--made", required=True, help="make.py's --out (parts, runs, made.json)")
    p.add_argument("--drive", help="the drive test's result JSON")
    p.add_argument("--out", default=str(RELEASE))
    a = p.parse_args(argv)
    r = Robot()
    out, made_dir = Path(a.out), Path(a.made)
    out.mkdir(parents=True, exist_ok=True)
    write_drawings(r, out)
    (out / "cam").mkdir(exist_ok=True)
    for name in ("bottom_plate.gcode", "top_plate.gcode", "hubs.nc", "print_plate.gcode"):
        src = made_dir / "parts" / name
        if src.exists():
            shutil.copyfile(src, out / "cam" / name)
    models = out / "models"
    models.mkdir(exist_ok=True)
    design.write_stl(models / "motor_clamp.stl", design.part_triangles(design.motor_clamp(r)), "motor clamp")
    design.write_stl(models / "rim.stl", design.part_triangles(design.rim(r)), "rim")
    design.write_stl(models / "tray.stl", design.part_triangles(design.tray(r)), "tray")
    design.write_stl(models / "hub.stl", design.part_triangles([(0, r.hub_t, design.hub_outline(r))]), "hub")
    for plate in ("bottom_plate", "top_plate"):
        fn = getattr(design, plate)
        design.write_stl(models / f"{plate}.stl", design.part_triangles([(0, r.plate_t, fn(r))]), plate)
    made = json.loads((made_dir / "made.json").read_text(encoding="utf-8")) if (made_dir / "made.json").exists() else {}
    drive = json.loads(Path(a.drive).read_text(encoding="utf-8")) if a.drive and Path(a.drive).exists() else None
    (out / "BOM.md").write_text(bom_md(r), encoding="utf-8")
    (out / "REPORT.md").write_text(report_md(r, made, drive), encoding="utf-8")
    (out / "ASSEMBLY.md").write_text(assembly_md(r, images=(out / "assembly").is_dir()), encoding="utf-8")
    import electrical
    import wiring
    lengths = None
    if wiring.ROUTED.exists():
        lengths = {w["id"]: w["cut_mm"] for w in json.loads(wiring.ROUTED.read_text(encoding="utf-8")) if "cut_mm" in w}
    electrical.write(out / "electrical", lengths=lengths)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
