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
"""The crew-order set: three robots, one order, one line.

    Picker   OmniArm 6 (vacuum) at the parts shelf       :8765
    Tug      OmniTug 500 towing CART                    :8766
    Packer   OmniArm 6 (vacuum) at the shipping crate    :8767

The line runs west -> east along y = 0, so the tug never turns with the cart
behind it: it starts at the west end, stops at the PICK CELL (the cart then
sits in front of Picker), and goes on to SHIPPING (the cart in front of
Packer). Every robot keeps its own OmniLink agent; they talk through the
crew channel (omnisim_bridges.crew, OMNILINK_CREW).

    python agents/omnilink_demos/crew_order/world.py      prints the layout check
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Dict, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "warehouse_shift" / "film"))
import scene as _cam  # noqa: E402  (camera maths shared with the warehouse-shift film)

PORTS = {"Picker": 8765, "Tug": 8766, "Packer": 8767}
CREW_ENV = ",".join(f"{n}={p}" for n, p in PORTS.items())


def ports_from(base: int) -> dict:
    """Picker, Tug, Packer on base, base+1, base+2 (parallel runs use distinct bases)."""
    return {n: base + i for i, n in enumerate(PORTS)}


def crew_env(ports: dict) -> str:
    return ",".join(f"{n}={p}" for n, p in ports.items())

PICKER = (-1.8, 0.55)
PACKER = (1.8, 0.55)
TUG_START = (-3.4, 0.0)
CART_BEHIND_M = 1.33                    # tug centre -> cart centre (OmniTug rear offset 0.63 + hitch 0.70)
PLACES = {"pick cell": (PICKER[0] + CART_BEHIND_M, 0.0), "shipping": (PACKER[0] + CART_BEHIND_M, 0.0)}
# The shelf and the crate sit on their arm's +x side, ~0.42 m out, with the cart
# on its -y side: the warehouse flagship's geometry (parts at base (+0.38,
# -0.16), trolley at (0, -0.70)). Measured with probe_reach.py, 2026-10-20: a
# shelf on +y (opposite the cart) left place() with no IK solution for 2 of 3
# drops on the cart; a shelf on -x left pick() with none; 0.6 m out at 0.42 m
# up has none at all (best miss 6-8 cm). +x and -y both solve.
SHELF = (PICKER[0] + 0.42, PICKER[1] + 0.07, 0.40)   # centre x, y, top height
CRATE = (PACKER[0] + 0.45, PACKER[1] + 0.07)          # on a stand, floor of the crate at 0.32
# Drop slots on the cart, as (dx, dy) from its centre. Measured from Picker with
# probe_reach.py: the centre and 10 cm east solve; 12 cm west and 20 cm east
# have no top-down IK solution.
CART_SLOTS = ((0.0, 0.0), (0.10, 0.0), (0.05, 0.10))
CRATE_FLOOR_Z = 0.32
PARTS = {"RED": (0.85, 0.12, 0.10), "BLUE": (0.10, 0.30, 0.85), "GREEN": (0.12, 0.62, 0.20)}

# Rendered at the shape of the picture area the edit leaves beside the crew
# chat (1290 x 1080). Chosen by a sweep (2026-10-20): the wide shot holds the
# whole line; each close shot holds its arm, its shelf or crate, the cart on
# its pad and the tug's nose at the stop.
CAMERAS: Dict[str, Tuple[Tuple[float, float, float], Tuple[float, float, float], float, int, int]] = {
    "wide": ((-0.7, -6.0, 3.6), (-0.65, 0.2, 0.2), 1.2, 1290, 1080),
    # A small 16:9 view of the whole line for the many-runs montage.
    "tile": ((-0.7, -7.8, 4.4), (-0.7, 0.4, 0.2), 1.0, 640, 360),
    "picker": ((-0.3, -2.2, 1.7), (-1.55, 0.25, 0.45), 1.0, 1290, 1080),
    "packer": ((0.4, -2.2, 1.7), (2.0, 0.25, 0.45), 1.0, 1290, 1080),
}

HEADER = """#OMNISIM R2025a utf8
EXTERNPROTO "omnisim://projects/objects/floors/protos/Floor.proto"
EXTERNPROTO "omnisim://projects/objects/factory/containers/protos/PlasticCrate.proto"
EXTERNPROTO "omnisim://projects/objects/factory/containers/protos/CardboardBox.proto"
EXTERNPROTO "omnisim://projects/objects/factory/pallet/protos/WoodenPalletStack.proto"
EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"
WorldInfo {
  basicTimeStep 32
  title "OmniLink crew: one order, three robots"
  newtonSolver "mujoco"
  newtonStatics TRUE
  newtonRobotColliders TRUE
  newtonCompoundColliders TRUE
  newtonSubsteps 6
}
Viewpoint { orientation -0.18 0.18 0.97 1.6 position 0 -7.5 4.2 }
OmniSimSky { }
DEF SUN OmniSimSun { }
DEF SUN_MARKER OmniSimSunMarker { }
Floor {
  size 30 14
  tileSize 30 14
  appearance PBRAppearance { baseColor 0.055 0.058 0.065 roughness 0.35 metalness 0.05 }
}
"""


def _box(def_name, xyz, size, rgb, collide=True, extra=""):
    x, y, z = xyz
    sx, sy, sz = size
    r, g, b = rgb
    shape = (f"Shape {{ appearance PBRAppearance {{ baseColor {r} {g} {b} roughness 0.8 metalness 0 }} "
             f"geometry Box {{ size {sx} {sy} {sz} }} }}")
    bo = f" boundingObject Box {{ size {sx} {sy} {sz} }}" if collide else ""
    head = f"DEF {def_name} " if def_name else ""
    return (f"{head}Solid {{ translation {x:.3f} {y:.3f} {z:.4f} children [ {shape} ] "
            f"name \"{(def_name or 'deco').lower()}_{abs(hash((x, y, z))) % 10000}\"{bo}{extra} }}\n")


def _pad(xy, size, rgb, z=0.003):
    return _box(None, (xy[0], xy[1], z), (size[0], size[1], 0.004), rgb, collide=False)


CART_MESHES = "../../../../objects/factory/tugger_cart/meshes/tugger_cart_{}.obj"
CART_MATERIALS = {  # material: (rgb, roughness, metalness)
    "deck": ((0.22, 0.23, 0.25), 0.55, 0.6), "frame": ((0.10, 0.27, 0.52), 0.45, 0.2),
    "rubber": ((0.05, 0.05, 0.05), 0.9, 0.0), "steel": ((0.66, 0.67, 0.69), 0.35, 1.0),
    "wheel": ((0.78, 0.78, 0.76), 0.6, 0.0), "accent": ((0.965, 0.78, 0.2), 0.5, 0.0),
    "reflector": ((0.85, 0.12, 0.05), 0.3, 0.0),
}


def _mesh_shapes(url_fmt, materials):
    return "\n".join(
        f'    Shape {{ appearance PBRAppearance {{ baseColor {c[0]} {c[1]} {c[2]} roughness {r} metalness {m} }} '
        f'geometry Mesh {{ url "{url_fmt.format(name)}" }} castShadows TRUE }}'
        for name, (c, r, m) in materials.items())


def _cart(x, y):
    # Tugger cart (projects/objects/factory/tugger_cart, Blender-built): the
    # OmniTug trolley's contract -- 0.70 m square, deck top 0.325, tow eye at
    # local (0.70, 0, 0.25) on +x, toward the tug. Collides as one box.
    return f"""DEF CART Solid {{
  translation {x:.3f} {y:.3f} 0
  physicsBackend "newton"
  children [
{_mesh_shapes(CART_MESHES, CART_MATERIALS)}
  ]
  name "cart"
  # It ROLLS on casters, so its floor contact is low-drag. A box collider at
  # mu 0.35 dragged: each step friction bled speed the tow servo restored, a
  # deck-acceleration sawtooth that ratcheted the parts 0.35 m to the rail.
  # The parts state their own grip (0.6), and two stated surfaces take the
  # larger, so the parts hold the deck while the cart rolls.
  newtonFriction 0.02
  boundingObject Transform {{ translation 0 0 0.1625 children [ Box {{ size 0.70 0.70 0.325 }} ] }}
  physics Physics {{ density -1 mass 22.0 centerOfMass [ 0 0 0.12 ] }}
}}
"""


KITTING = "../../../../objects/factory/kitting/meshes/{}.obj"
BENCH_MATERIALS = {  # suffix: (rgb, roughness, metalness)
    "steel": ((0.62, 0.64, 0.66), 0.35, 0.9), "mat": ((0.05, 0.05, 0.055), 0.9, 0.0),
    "frame": ((0.10, 0.27, 0.52), 0.45, 0.2), "feet": ((0.04, 0.04, 0.04), 0.9, 0.0),
}
TOTE_MATERIALS = {"tote_plastic": ((0.08, 0.30, 0.62), 0.55, 0.0), "tote_dark": ((0.03, 0.10, 0.22), 0.7, 0.0)}


def _collider(def_name, xyz, size):
    """An invisible box collider (the visual is a mesh elsewhere)."""
    x, y, z = xyz
    return (f"DEF {def_name} Solid {{ translation {x:.3f} {y:.3f} {z:.4f} name \"{def_name.lower()}\" "
            f"boundingObject Box {{ size {size[0]} {size[1]} {size[2]} }} }}\n")


def _table(def_name, bench, xy):
    """A steel workbench (kitting/meshes bench_<bench>_*): collides as one box up to its top."""
    sx, sy, top = {"shelf": (0.26, 0.46, 0.40), "stand": (0.38, 0.48, 0.30)}[bench]
    x, y = xy
    shapes = _mesh_shapes(KITTING.format(f"bench_{bench}_{{}}"), BENCH_MATERIALS)
    return (f"DEF {def_name} Solid {{ translation {x:.3f} {y:.3f} 0 children [\n{shapes}\n  ] name \"{def_name.lower()}\" "
            f"boundingObject Transform {{ translation 0 0 {top / 2:.4f} children [ Box {{ size {sx} {sy} {top} }} ] }} }}\n")


def _crate(x, y):
    """The shipping tote on its stand: floor at CRATE_FLOOR_Z, walls to +0.14.
    Visual: kitting/meshes tote_*; colliders: five invisible boxes."""
    z0, h, lx, ly, t = CRATE_FLOOR_Z, 0.14, 0.32, 0.42, 0.02
    out = [_table("CRATE_STAND", "stand", (x, y))]
    tote = _mesh_shapes(KITTING, TOTE_MATERIALS)
    out.append(f"Solid {{ translation {x:.3f} {y:.3f} {z0 - t:.3f} children [\n{tote}\n  ] name \"tote_visual\" }}\n")
    out.append(_collider("CRATE", (x, y, z0 - t / 2), (lx, ly, t)))
    for i, (dx, dy, sx, sy) in enumerate(((0, ly / 2 - t / 2, lx, t), (0, -ly / 2 + t / 2, lx, t),
                                          (lx / 2 - t / 2, 0, t, ly), (-lx / 2 + t / 2, 0, t, ly))):
        out.append(_collider(f"CRATE_WALL_{i}", (x + dx, y + dy, z0 + h / 2), (sx, sy, h)))
    return "".join(out)


def _arm(def_name, name, xy, port):
    return f"""DEF {def_name} URDFRobot {{
  url "../../../../robots/omnisim/omniarm6/omniarm6_gumgrip.urdf"
  translation {xy[0]} {xy[1]} 0
  name "{name}"
  supervisor TRUE
  staticBase TRUE
  physicsBackend "newton"
  controller "omnilink_arm_bridge"
  controllerArgs [ "--robot" "omniarm6" "--gripper" "vacuum" "--port" "{port}" "--name" "{name}" ]
  window "omnilink_chat"
}}
"""


def _back_wall() -> str:
    """Set dressing behind the line, out of every robot's reach: a wall with a
    safety stripe, and static factory props (no physics) along it."""
    out = [_box("BACK_WALL", (0.0, 3.2, 1.5), (30.0, 0.2, 3.0), (0.32, 0.34, 0.37)),
           _box(None, (0.0, 3.09, 1.2), (30.0, 0.01, 0.18), (0.90, 0.62, 0.02), collide=False)]
    props = [
        'WoodenPalletStack { translation -4.6 2.45 0 rotation 0 0 1 0 name "pallets_w" palletNumber 6 }',
        'PlasticCrate { translation -3.2 2.6 0 name "crate_a" size 0.6 0.4 0.32 color 0.08 0.30 0.62 }',
        'PlasticCrate { translation -3.2 2.6 0.32 name "crate_b" size 0.6 0.4 0.32 color 0.08 0.30 0.62 }',
        'PlasticCrate { translation -2.55 2.6 0 name "crate_c" size 0.6 0.4 0.32 color 0.08 0.30 0.62 }',
        'CardboardBox { translation 0.2 2.55 0.25 name "box_a" size 0.6 0.5 0.5 }',
        'CardboardBox { translation 0.85 2.6 0.2 name "box_b" size 0.5 0.4 0.4 }',
        'CardboardBox { translation 0.45 2.6 0.65 name "box_c" size 0.5 0.4 0.3 }',
        'WoodenPalletStack { translation 4.4 2.45 0 name "pallets_e" palletNumber 9 }',
        'PlasticCrate { translation 3.1 2.6 0 name "crate_d" size 0.6 0.4 0.32 color 0.90 0.62 0.02 }',
    ]
    return "".join(out) + "\n".join(props) + "\n"


FILM_CAMERAS = ("wide", "picker", "packer")


def world_text(film_period_ms: int = 0, controllers: bool = True, ports: dict = None,
               cameras=FILM_CAMERAS) -> str:
    ports = ports or PORTS
    out = [HEADER]
    # Floor signage: where the cart stands for each arm, and the lane between.
    for ly in (-0.47, 0.47):                       # yellow lane edge lines
        out.append(_pad((0.0, ly), (9.6, 0.05), (0.90, 0.62, 0.02), z=0.003))
    out.append(_back_wall())
    for xy in ((PICKER[0], 0.0), (PACKER[0], 0.0)):
        out.append(_pad(xy, (0.95, 0.95), (0.05, 0.05, 0.06), z=0.004))
        out.append(_pad(xy, (0.80, 0.80), (0.90, 0.62, 0.02), z=0.008))
    # Picker's shelf with the three parts; Packer's crate.
    sx, sy, top = SHELF
    out.append(_table("SHELF", "shelf", (sx, sy)))
    for i, (colour, rgb) in enumerate(PARTS.items()):
        x, y = sx, sy + (i - 1) * 0.12
        # An anodised 50 mm block (kitting/meshes/part.obj); collides as its box.
        out.append(f"""DEF GRASP_{colour} Solid {{
  translation {x:.3f} {y:.3f} {top + 0.026:.3f}
  physicsBackend "newton"
  children [ Shape {{ appearance PBRAppearance {{ baseColor {rgb[0]} {rgb[1]} {rgb[2]} roughness 0.35 metalness 0.6 }} geometry Mesh {{ url "{KITTING.format('part')}" }} }} ]
  name "part_{colour.lower()}"
  newtonFriction 0.6
  boundingObject Box {{ size 0.05 0.05 0.05 }}
  physics Physics {{ density -1 mass 0.06 }}
}}
""")
    out.append(_crate(*CRATE))
    tx, ty = TUG_START
    out.append(_cart(tx - CART_BEHIND_M, ty))
    out.append(_arm("PICKER", "picker", PICKER, ports["Picker"]))
    out.append(_arm("PACKER", "packer", PACKER, ports["Packer"]))
    out.append(f"""DEF TUG URDFRobot {{
  url "../../../../robots/omnisim/omnitug500/urdf/omnitug500.urdf"
  translation {tx} {ty} 0
  # The OmniTug drives along its local +Y: node yaw -pi/2 faces it east, down the line.
  rotation 0 0 1 -1.5708
  name "tug"
  supervisor TRUE
  controller "omnilink_mobile_bridge"
  controllerArgs [ "--robot" "omnitug500" "--port" "{ports['Tug']}" "--name" "tug" "--pallets" "CART" ]
  window "omnilink_chat"
}}
""")
    if film_period_ms and cameras:
        cams = []
        for name, (pos, target, fov, w, h) in ((n, CAMERAS[n]) for n in cameras):
            ax, ay, az, angle = _cam._axis_angle(_cam._look_rotation(pos, target))
            cams.append(f'Camera {{ name "{name}" translation {pos[0]} {pos[1]} {pos[2]} '
                        f'rotation {ax:.6f} {ay:.6f} {az:.6f} {angle:.6f} fieldOfView {fov} '
                        f'width {w} height {h} far 40 }}')
        out.append(f'Robot {{ name "film_camera" controller "film_camera" controllerArgs [ "{film_period_ms}" ] '
                   f'children [ {" ".join(cams)} ] }}\n')
    text = "".join(out)
    if not controllers:
        for c in ("omnilink_arm_bridge", "omnilink_mobile_bridge"):
            text = text.replace(f'controller "{c}"', 'controller "<none>"')
    return text


def project(camera: str, point) -> Tuple[float, float]:
    _cam.CAMERAS[camera] = CAMERAS[camera]
    return _cam.project(camera, point)


if __name__ == "__main__":
    for name, (x, y) in PLACES.items():
        print(f"{name:10} tug stops at ({x:+.2f}, {y:+.2f}); cart centre ({x - CART_BEHIND_M:+.2f}, {y:+.2f})")
    for arm, (ax, ay) in (("Picker", PICKER), ("Packer", PACKER)):
        print(f"{arm}: base ({ax:+.2f}, {ay:+.2f}); cart centre {math.hypot(0, ay):.2f} m away")
    print(f"shelf {math.hypot(SHELF[0] - PICKER[0], SHELF[1] - PICKER[1]):.2f} m from Picker; "
          f"crate {math.hypot(CRATE[0] - PACKER[0], CRATE[1] - PACKER[1]):.2f} m from Packer")
    for cam, (_, _, _, w, h) in CAMERAS.items():
        for label, p in (("tug start", (*TUG_START, 0)), ("cart start", (TUG_START[0] - 1.2, 0, 0)),
                         ("shipping", (*PLACES["shipping"], 0)), ("shelf", (SHELF[0], SHELF[1], 0.4)),
                         ("crate", (*CRATE, 0.4))):
            u, v = project(cam, p)
            print(f"  {cam:5} {label:10} u={u:6.0f} v={v:6.0f} {'' if 0 < u < w and 0 < v < h else '<-- outside'}")
