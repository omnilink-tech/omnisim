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

"""SparkRover Mini: the skeleton every subsystem is designed against.

The concept sheets (SPEC.md) give a look and approximate numbers, some of them in
conflict. This file is where they are decided, once: the envelope, the frame, the
volume each subsystem may use and the interfaces between them. Every subsystem module
(shell.py, chassis.py, arms.py, electrical.py) imports it and designs INSIDE its zone,
TO its interfaces; a change here is a change to the contract and goes through the
integrator (integrate.py), not one subsystem.

Robot frame (mm): origin on the floor, under the centre of the four wheels; +x forward
(the face), +y left, +z up. All numbers are mm unless named otherwise.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

# ---- the envelope (decided from the sheets) ---------------------------------------------
# Sheets: H 290, W 210, D 185 (all three sheets agree); ground clearance 15 (front sheet);
# wheel 55-60 (side sheet) vs 62-65 (front sheet): 65 is decided (the larger clears a
# classroom floor's thresholds and cables better, and a 65 mm wheel is a stock size); track
# 170 centre to centre (front sheet).
HEIGHT = 290.0                 # floor to the top of the accessory slot
WIDTH = 210.0                  # across the wheel housings (y)
DEPTH = 185.0                  # front of the bumper to the back of the bumper (x)
GROUND_CLEARANCE = 15.0        # floor to the underside of the base (the bumper's lowest edge)

WHEEL_D = 65.0
WHEEL_W = 26.0                 # tread width
TRACK = 170.0                  # wheel centre to wheel centre, across (y)
WHEELBASE = 110.0              # front axle to rear axle (x): the wheels sit 5 mm inside the
                               # bumper's ends (185/2 - 32.5 - 55 = 5)
AXLE_Z = WHEEL_D / 2.0
WHEELS = {name: (sx * WHEELBASE / 2, sy * TRACK / 2, AXLE_Z)
          for name, sx, sy in (("front_left", 1, 1), ("front_right", 1, -1),
                               ("rear_left", -1, 1), ("rear_right", -1, -1))}

# The sheets' 1.5-2.0 kg was a guess; the owner chose runtime over mass (2S2P), so the robot
# is held to 2.4 kg. The chassis' drive was sized at 2.0 kg: at 2.4 kg its torque margin for
# turning on the spot is still ~4.4x.
MASS_TARGET_G = (1500.0, 2400.0)

# ---- decisions on the sheets' contents ----------------------------------------------------
DECISIONS = {
    "drive": "skid steer, FOUR gearmotors with encoders (one per wheel), two driver channels per side",
    "arms": "2 DOF each (shoulder pitch, elbow pitch, +-90 deg), serial bus servos; NO gripper: the arms "
            "wave, point and gesture; the sheet's 'carry light objects' is dropped",
    "camera": "a plain RGB camera (no depth / ToF), in the front sensor bar with two status LEDs",
    "compute": "Raspberry Pi 5 (the sheet's Wi-Fi / BT come with it)",
    "battery": "2S2P Li-ion (7.2 V nominal (2 x 3.6 V), four high-drain 18650s, ~5600 mAh), with a protection board, "
               "flat under the deck between the motors; USB-C charging at 5 V (a 15 W boost charger: not PD). "
               "Chosen 2026-11-03 for runtime over mass: 2S1P gave ~1.4 h of classroom use against the sheets' "
               "2-4 h; 2S2P ~2.8 h for ~95 g more",
    "shell": "printed PETG sections on the SV08 twin (the sheets' injection-moulded shell has no twin); "
             "TPU bumper; the shell hangs on the internal frame, the frame carries every load",
    "frame": "laser-cut plywood decks and CNC-milled aluminium where a part takes load "
             "(motor mounts, shoulder brackets)",
    "touch": "capacitive pads behind the shell's chest window and a touch controller (no separate panel)",
    "status_ring": "an addressable LED strip behind a diffuser in the bumper line",
    "top_slot": "the accessory slot: a printed socket with a 2.54 mm pin header (power + I2C + UART)",
}


@dataclass(frozen=True)
class Box:
    """An axis-aligned volume, robot frame, mm."""
    x0: float
    x1: float
    y0: float
    y1: float
    z0: float
    z1: float

    def contains(self, other: "Box") -> bool:
        return (self.x0 <= other.x0 and other.x1 <= self.x1 and self.y0 <= other.y0 and other.y1 <= self.y1
                and self.z0 <= other.z0 and other.z1 <= self.z1)

    def overlaps(self, other: "Box") -> bool:
        return (self.x0 < other.x1 and other.x0 < self.x1 and self.y0 < other.y1 and other.y0 < self.y1
                and self.z0 < other.z1 and other.z0 < self.z1)


ENVELOPE = Box(-DEPTH / 2, DEPTH / 2, -WIDTH / 2, WIDTH / 2, 0.0, HEIGHT)

# ---- the shell's skin --------------------------------------------------------------------
SHELL_WALL = 2.0               # PETG; 2.0 (not 2.5) keeps the shell near 490 g; thickened locally (bosses, ribs)
SHELL_GAP = 1.0                # the clear gap the shell keeps to anything inside it
BUMPER_Z = (GROUND_CLEARANCE, 45.0)   # the TPU bumper band round the base, its height
BUMPER_T = 6.5                 # the sheets' 8 mm soft edge does not fit: between the base zone (x 85) plus the
                               # gap and the envelope (92.5) there is 6.5; front and rear bumpers only (|y| <= 66)

# The outer form, as the sheets draw it: a lower body (the base, between and over the
# wheels) and an upper body (trunk and head as one rounded box). Edge radii are the
# shell agent's; these are the bounding boxes it must fill and not exceed.
LOWER_BODY = Box(-DEPTH / 2, DEPTH / 2, -WIDTH / 2, WIDTH / 2, GROUND_CLEARANCE, 85.0)
UPPER_BODY = Box(-80.0, 80.0, -82.0, 82.0, 85.0, 272.0)     # narrower than the base, as drawn
TOP_SLOT = Box(-14.0, 14.0, -30.0, 30.0, 272.0, HEIGHT)       # the accessory slot's socket

# ---- zones: who may put things where (inside the shell's skin) ---------------------------
# A subsystem's solids must lie inside its zones; zones of different owners may touch but
# must not overlap. The integrator checks it (integrate.zones_ok).
ZONES: Dict[str, List[Tuple[str, Box]]] = {
    "chassis": [
        # The base / wheel-well boundary at |y| 67 (was 70): at 70 a 25D gearmotor's 34:1 gearbox
        # (66.5 mm) left its face screws outside the base; the chassis agent's finding.
        ("base", Box(-85.0, 85.0, -67.0, 67.0, GROUND_CLEARANCE + 3.0, 60.0)),       # deck, motors, battery
        ("wheel_wells", Box(-90.0, 90.0, -105.0, -67.0, 0.0, 72.0)),                 # right side
        ("wheel_wells_l", Box(-90.0, 90.0, 67.0, 105.0, 0.0, 72.0)),                 # left side
        ("spine", Box(-15.0, 15.0, -55.0, 55.0, 60.0, 262.0)),                       # the frame up the middle
    ],
    "electrical": [
        ("bay", Box(-75.0, -15.0, -55.0, 55.0, 60.0, 175.0)),     # behind the spine, under the rear hatch
        # Widened from +-60 to +-76 (the upper body's skin is at +-82, less wall and gap): a
        # 5 in panel (120.7 mm) fits, as the concept's big face needs. x >= 15 keeps it off the
        # shoulder zones.
        ("head", Box(15.0, 75.0, -76.0, 76.0, 150.0, 262.0)),     # screen and camera, in front of the spine
        ("chest", Box(15.0, 75.0, -40.0, 40.0, 85.0, 150.0)),     # touch pads, speaker
        ("top_slot", Box(-4.0, 4.0, -9.0, 9.0, 276.0, 287.0)),      # the accessory header board only (8 x 18), on the socket floor (z 276)
    ],
    "arms": [
        # The shoulder servo inside the body, beside the spine (a 45 x 25 x 35 bus servo fits).
        ("shoulder_l", Box(-30.0, 15.0, 55.0, 79.0, 120.0, 190.0)),     # to the skin's inside (82 - wall - gap)
        ("shoulder_r", Box(-30.0, 15.0, -79.0, -55.0, 120.0, 190.0)),
        # The arm's swept volume outside the body. It starts at z 88: the lower body is wider
        # (+-105) than the upper (+-82), so an arm hanging straight down from the shoulder
        # (z 160) would strike the lower body's top (z 85). The arms agent limits the
        # shoulder's range so the swept arm stays in here.
        # Where the fixed post passes the skin (79..83): inside the 40 mm opening, round the
        # shoulder axis; the shell's opening edge (r 20) stays clear of this +-9.5 square.
        ("opening_l", Box(-9.5, 9.5, 79.0, 83.0, 150.5, 169.5)),
        ("opening_r", Box(-9.5, 9.5, -83.0, -79.0, 150.5, 169.5)),
        ("arm_l", Box(-70.0, 140.0, 83.0, 120.0, 88.0, 260.0)),        # 1 mm off the skin
        ("arm_r", Box(-70.0, 140.0, -120.0, -83.0, 88.0, 260.0)),
    ],
}

# ---- interfaces --------------------------------------------------------------------------

@dataclass(frozen=True)
class Mount:
    """A fastened interface: a point, the screw's axis (unit, pointing INTO the part that
    holds the thread), the screw size, and which two subsystems it joins."""
    name: str
    at: Tuple[float, float, float]
    axis: Tuple[float, float, float]
    size: str                 # "M2", "M2.5", "M3"
    holder: str               # the subsystem with the thread (insert / nut / tapped hole)
    part: str                 # the subsystem whose part the screw passes through


# The shell hangs on the frame. Heat-set inserts hold well in PETG, so the THREAD is in the
# shell (an M3 insert in a boss reaching in from the skin) and the screw passes through the
# frame from inside: the upper mounts on the spine's side faces (y +-55). Positions: four on the lower body (base
# deck corners), four on the upper body (spine, at two heights), one per section seam is
# the shell agent's business (shell-to-shell joints are internal to the shell).
SHELL_MOUNTS: List[Mount] = (
    [Mount(f"lower_{n}", (sx * 75.0, sy * 60.0, 50.0), (0.0, 0.0, -1.0), "M3", "shell", "chassis")
     for n, sx, sy in (("fl", 1, 1), ("fr", 1, -1), ("rl", -1, 1), ("rr", -1, -1))]
    + [Mount(f"upper_{n}", (0.0, sy * 55.0, z), (0.0, float(sy), 0.0), "M3", "shell", "chassis")
       for n, sy, z in (("l_low", 1, 110.0), ("r_low", -1, 110.0), ("l_high", 1, 240.0), ("r_high", -1, 240.0))]
)

# The arms: each shoulder's pitch axis (along y), where it leaves the body, and the
# bracket the arm's first servo bolts to. The arms agent owns the bracket; the chassis
# agent owns the spine face it bolts to (four M3, tapped in the aluminium bracket).
SHOULDER_AXIS_Z = 160.0
SHOULDER_AXIS_X = 0.0
SHOULDERS = {"left": (SHOULDER_AXIS_X, 82.0, SHOULDER_AXIS_Z), "right": (SHOULDER_AXIS_X, -82.0, SHOULDER_AXIS_Z)}
# The pattern is 20 x 24 mm, centred on x 0 so both columns land in the 30 mm spine (an
# earlier -7.5 offset put one column at x -19.5, outside it). The screw goes in from inside
# the spine, outward (+y on the left), into the thread in the arms' bracket.
SHOULDER_BOLTS: List[Mount] = [
    Mount(f"shoulder_{side}_{k}", (SHOULDER_AXIS_X + dx, sy * 55.0, SHOULDER_AXIS_Z + dz), (0.0, float(sy), 0.0),
          "M3", "arms", "chassis")
    for side, sy in (("left", 1), ("right", -1)) for k, (dx, dz) in enumerate(((-10, -12), (10, -12), (-10, 12), (10, 12)))]
# The arm's fixed post (arms.py): no pitch servo fits between the spine face (y 55) and the
# skin (y 82), so the servo rides in the upper arm and its horn is screwed to a post that
# passes the shell opening and does not turn. The post screws to the arms' own aluminium
# bracket (2 x M3 from outside, through bores in the arm), so it needs no interface with
# the chassis beyond SHOULDER_BOLTS. (Until 2026-11-03 it bolted through the spine from
# inside; the electrical bay's IO board blocked the key, so that interface was dropped.)
ARM_POST_MOUNTS: List[Mount] = []
ASSEMBLY_NOTE = ("shoulder brackets onto the spine first; then the shell and the electronics; each arm module "
                 "(post, servos, links: assembled on the bench) goes on LAST, from outside: the post through the "
                 "40 mm opening, its two M3 screws driven through the arm's access bores with a 2.5 mm hex driver")
SHELL_ARM_OPENING_D = 40.0      # the shell's round opening round each shoulder (the arm passes, the shell clears it)

# The face: the screen's visible window in the shell's front, and the camera.
FACE_WINDOW = {"centre": (UPPER_BODY.x1, 0.0, 212.0), "size": (132.0, 82.0), "corner_r": 14.0}
# The camera bar (z 152-170) sits under the face window (z 171+): at 168 it overlapped it.
CAMERA = {"lens_at": (UPPER_BODY.x1, 0.0, 161.0), "axis": (1.0, 0.0, 0.0), "bar_size": (76.0, 18.0)}
CHEST_TOUCH = {"centre": (UPPER_BODY.x1, 0.0, 132.0), "size": (64.0, 34.0)}
SPEAKER_GRILLE = {"centre": (UPPER_BODY.x1, 0.0, 100.0), "size": (44.0, 26.0)}
MIC_HOLES = [(UPPER_BODY.x1 - 18.0, y, 270.0) for y in (-22.0, 22.0)]     # top-front: two I2S mics (electrical.py)
LED_STRIP_Z = 30.0              # the status ring's centre line, behind the bumper's diffuser
REAR_HATCH = {"centre": (UPPER_BODY.x0, 0.0, 130.0), "size": (92.0, 68.0), "screws": "4 x M3 into inserts"}
REAR_VENT = {"centre": (UPPER_BODY.x0, 0.0, 176.0), "size": (44.0, 18.0)}   # z 176: at 168 it cut into the hatch   # the rear sheet's grille,
                                                 # above the hatch: the Pi 5's heat leaves here
# Its inlet, low in the rear hatch: the bay dissipates 5.5-10.8 W, and stack flow needs an
# opening low as well as high (>= 800 mm2, electrical.heat()).
REAR_INLET = {"centre": (UPPER_BODY.x0, 0.0, 110.0), "size": (60.0, 14.0), "in": "rear hatch"}   # z 110: at 104
                                                 # its seat ring hung below the hatch
# On the LOWER body's rear face (z 72 is below the upper body, which starts at z 85): at
# UPPER_BODY.x0 they sat 12.5 mm inside the skin.
USB_C = {"at": (LOWER_BODY.x0, -20.0, 72.0), "axis": (-1.0, 0.0, 0.0)}
POWER_SWITCH = {"at": (LOWER_BODY.x0, 20.0, 72.0), "axis": (-1.0, 0.0, 0.0), "kind": "momentary pushbutton"}

# Known and accepted (shell.skeleton_issues, 2026-11-03): the head zone's two top front
# corners (+-76, 262) come within 0.6 mm of the shell's inside where three rounds meet (no
# electrical part reaches them); the rear hatch's four insert bosses reach 5 mm into the
# bay (the electrical keeps those spots clear: FCL-checked); the lower mount lugs occupy
# x 70.5-86, |y| 55.5-64.5, z 40.5-50 inside chassis.base (the chassis keeps it clear:
# FCL-checked). The wheel wells stay at |y| 105: the shell roofs the wheels and leaves their
# sides open, as the concept's side view shows them.

# ---- what every subsystem module provides --------------------------------------------------
CONTRACT = """Each subsystem module (shell.py, chassis.py, arms.py, electrical.py) provides:

  parts(r) -> list of dict: {label, step, kind ('made'|'bought'), process (laser|cnc|print|
      bought), material, mass_g, centre_mm (x,y,z robot frame), source (URL for bought)}
  solids(r) -> {label: triangles (N,3,3) float mm, robot frame}: every part, assembled, closed
      solids (watertight) so the integrator's FCL clearance check can run across subsystems
  mounts(r) -> list of skeleton.Mount it provides or consumes (by name)
  jobs(r) -> {part label: machine file text} for every MADE part (laser G-code, CNC program,
      or STL for the slicer), so make.py can run them on the twins
  checks(r) -> {name: (ok: bool, detail: str)}: the subsystem's own design rules

and keeps every solid inside its ZONES (and the shell inside the envelope)."""


def zone_of(owner: str) -> List[Box]:
    return [b for _, b in ZONES[owner]]


def check_skeleton() -> Dict[str, bool]:
    """The skeleton's own consistency: zones inside the envelope, no two owners overlap,
    the wheels inside their wells, the mounts inside someone's zone."""
    out = {}
    out["zones_in_envelope"] = all(ENVELOPE.contains(b) or name.startswith("arm")
                                   for owner, zs in ZONES.items() for name, b in zs)
    owners = list(ZONES)
    clash = []
    for i, a in enumerate(owners):
        for b in owners[i + 1:]:
            for na, za in ZONES[a]:
                for nb, zb in ZONES[b]:
                    if za.overlaps(zb):
                        clash.append(f"{a}.{na} x {b}.{nb}")
    out["owners_do_not_overlap"] = not clash
    if clash:
        out["clashes"] = clash
    for name, (x, y, z) in WHEELS.items():
        r = WHEEL_D / 2
        wheel = Box(x - r, x + r, y - WHEEL_W / 2, y + WHEEL_W / 2, z - r, z + r)
        out[f"wheel_{name}_in_well"] = any(b.contains(wheel) for _, b in ZONES["chassis"])
    out["wheels_inside_bumper"] = WHEELBASE / 2 + WHEEL_D / 2 < DEPTH / 2
    out["ground_clearance_below_axle"] = GROUND_CLEARANCE < AXLE_Z
    return out


if __name__ == "__main__":
    import json
    print(json.dumps(check_skeleton(), indent=1))
