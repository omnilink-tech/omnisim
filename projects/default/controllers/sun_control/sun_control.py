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

"""Interactive sun + time-of-day controller for atmospheric-sky worlds.

Reads keyboard input and drives:
  * the first DirectionalLight (DEF SUN) — azimuth + elevation
  * the Viewpoint (DEF VIEW)             — exposure
  * the Background (DEF SKY)             — Earth/Mars preset swap

Keys
----
  Arrow Left / Right     : sun azimuth -/+ 2 deg
  Arrow Up / Down        : sun elevation +/- 2 deg
  Shift + arrows         : 10x speed (coarse)
  [ / ]                  : exposure -/+ 0.1
  E                      : Earth atmosphere
  M                      : Mars atmosphere
  1                      : preset — midnight  (sun -90 deg)
  2                      : preset — sunrise   (sun  +1 deg, east)
  3                      : preset — noon      (sun +89 deg, overhead)
  4                      : preset — sunset    (sun  +1 deg, west)
  R                      : reset to current world's starting sun
  H                      : toggle on-screen help

The DirectionalLight `direction` field stores the direction *light
travels* (away from sun), so a positive z component places the sun
below the horizon.  The atmospheric sky engine negates this internally
to recover the toward-sun vector — see WbBackground.cpp.
"""

import math

from omnisim import Supervisor, Keyboard

AZIMUTH_STEP_DEG = 2.0
ELEVATION_STEP_DEG = 2.0
COARSE_MULTIPLIER = 5.0
EXPOSURE_STEP = 0.1
MIN_EXPOSURE = 0.05
MAX_EXPOSURE = 12.0

LABEL_COLOR = 0xFFFFFF
LABEL_DIM = 0xAAAAAA
HELP_KEYS = [
    "arrows : sun (shift = coarse)",
    "[ ]    : exposure",
    "E / M  : earth / mars",
    "1 2 3 4: midnight sunrise noon sunset",
    "R      : reset    H : toggle help",
]


def direction_from_azel(az_deg: float, el_deg: float) -> list:
    """toward-sun in Webots ENU (Z-up), then negated for DirectionalLight."""
    az = math.radians(az_deg)
    el = math.radians(el_deg)
    tx = math.sin(az) * math.cos(el)
    ty = math.cos(az) * math.cos(el)
    tz = math.sin(el)
    return [-tx, -ty, -tz]


def azel_from_direction(d: list) -> tuple:
    """Recover (az_deg, el_deg) from a DirectionalLight.direction value."""
    tx, ty, tz = -d[0], -d[1], -d[2]
    el = math.degrees(math.asin(max(-1.0, min(1.0, tz))))
    az = math.degrees(math.atan2(tx, ty))
    return az, el


def find_first_node_by_type(supervisor: Supervisor, type_name: str):
    """Walk the root children, return first node whose type matches."""
    root = supervisor.getRoot()
    children = root.getField("children")
    for i in range(children.getCount()):
        n = children.getMFNode(i)
        if n is not None and n.getTypeName() == type_name:
            return n
    return None


def main():
    supervisor = Supervisor()
    time_step = int(supervisor.getBasicTimeStep())

    sun = supervisor.getFromDef("SUN") or find_first_node_by_type(supervisor, "DirectionalLight")
    if sun is None:
        print("[sun_control] no DirectionalLight in scene — nothing to drive")
        return
    direction_field = sun.getField("direction")

    viewpoint = supervisor.getFromDef("VIEW") or find_first_node_by_type(supervisor, "Viewpoint")
    exposure_field = viewpoint.getField("exposure") if viewpoint else None

    sky = supervisor.getFromDef("SKY") or find_first_node_by_type(supervisor, "Background")
    sky_preset_field = sky.getField("atmosphericSky") if sky else None

    start_direction = list(direction_field.getSFVec3f())
    az_deg, el_deg = azel_from_direction(start_direction)
    exposure = exposure_field.getSFFloat() if exposure_field else 1.0
    preset = sky_preset_field.getSFString() if sky_preset_field else "earth"
    show_help = True

    keyboard = supervisor.getKeyboard()
    keyboard.enable(time_step)

    def apply():
        direction_field.setSFVec3f(direction_from_azel(az_deg, el_deg))
        if exposure_field is not None:
            exposure_field.setSFFloat(exposure)
        if sky_preset_field is not None:
            sky_preset_field.setSFString(preset)

    def render_overlay():
        status = (f"sun az={az_deg:+7.1f}°  el={el_deg:+6.1f}°   "
                  f"exposure={exposure:4.2f}   atmosphere={preset}")
        supervisor.setLabel(0, status, 0.02, 0.02, 0.07, LABEL_COLOR, 0.0, "DejaVuSansMono")
        if show_help:
            for i, line in enumerate(HELP_KEYS):
                supervisor.setLabel(10 + i, line, 0.02, 0.10 + i * 0.045,
                                    0.05, LABEL_DIM, 0.0, "DejaVuSansMono")
        else:
            for i in range(len(HELP_KEYS)):
                supervisor.setLabel(10 + i, "", 0.02, 0.10, 0.05,
                                    LABEL_DIM, 1.0, "DejaVuSansMono")

    apply()
    render_overlay()
    print("[sun_control] ready — press H for help, arrows to move the sun")

    pressed_h = False
    pressed_e = False
    pressed_m = False
    pressed_presets = {ord("1"): False, ord("2"): False,
                       ord("3"): False, ord("4"): False, ord("R"): False}

    while supervisor.step(time_step) != -1:
        keys = []
        k = keyboard.getKey()
        while k != -1:
            keys.append(k)
            k = keyboard.getKey()

        if not keys:
            pressed_h = pressed_e = pressed_m = False
            for kk in pressed_presets:
                pressed_presets[kk] = False
            continue

        coarse = any(kk & Keyboard.SHIFT for kk in keys)
        mult = COARSE_MULTIPLIER if coarse else 1.0
        codes = {kk & ~Keyboard.SHIFT & ~Keyboard.CONTROL & ~Keyboard.ALT for kk in keys}
        changed = False

        if Keyboard.LEFT in codes:
            az_deg = (az_deg - AZIMUTH_STEP_DEG * mult) % 360.0
            if az_deg > 180.0:
                az_deg -= 360.0
            changed = True
        if Keyboard.RIGHT in codes:
            az_deg = (az_deg + AZIMUTH_STEP_DEG * mult) % 360.0
            if az_deg > 180.0:
                az_deg -= 360.0
            changed = True
        if Keyboard.UP in codes:
            el_deg = min(90.0, el_deg + ELEVATION_STEP_DEG * mult)
            changed = True
        if Keyboard.DOWN in codes:
            el_deg = max(-90.0, el_deg - ELEVATION_STEP_DEG * mult)
            changed = True
        if ord("[") in codes:
            exposure = max(MIN_EXPOSURE, exposure - EXPOSURE_STEP)
            changed = True
        if ord("]") in codes:
            exposure = min(MAX_EXPOSURE, exposure + EXPOSURE_STEP)
            changed = True

        if ord("E") in codes and not pressed_e and sky_preset_field is not None:
            preset = "earth"
            changed = True
            pressed_e = True
        elif ord("E") not in codes:
            pressed_e = False

        if ord("M") in codes and not pressed_m and sky_preset_field is not None:
            preset = "mars"
            changed = True
            pressed_m = True
        elif ord("M") not in codes:
            pressed_m = False

        for keycode, (a, e) in {ord("1"): (0.0, -90.0),
                                 ord("2"): (90.0, 1.0),
                                 ord("3"): (0.0, 89.0),
                                 ord("4"): (-90.0, 1.0)}.items():
            if keycode in codes and not pressed_presets[keycode]:
                az_deg, el_deg = a, e
                changed = True
                pressed_presets[keycode] = True
            elif keycode not in codes:
                pressed_presets[keycode] = False

        if ord("R") in codes and not pressed_presets[ord("R")]:
            az_deg, el_deg = azel_from_direction(start_direction)
            pressed_presets[ord("R")] = True
            changed = True
        elif ord("R") not in codes:
            pressed_presets[ord("R")] = False

        if ord("H") in codes and not pressed_h:
            show_help = not show_help
            pressed_h = True
            changed = True
        elif ord("H") not in codes:
            pressed_h = False

        if changed:
            apply()
            render_overlay()


if __name__ == "__main__":
    main()
