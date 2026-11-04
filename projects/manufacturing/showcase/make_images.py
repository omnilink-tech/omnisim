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

"""Render the OmniSim Manufacturing showcase stills into docs/images/manufacturing/.

    python projects/manufacturing/showcase/make_images.py rover      # sparkrover_hero, face_camera
    python projects/manufacturing/showcase/make_images.py concept    # concept_vs_built
    python projects/manufacturing/showcase/make_images.py exploded   # exploded
    python projects/manufacturing/showcase/make_images.py wiring     # wiring
    python projects/manufacturing/showcase/make_images.py assembly   # assembly_steps
    python projects/manufacturing/showcase/make_images.py printed    # printed_parts
    (any of them with --review: every frame to .tmp_showcase/review/, nothing published)

Each subcommand writes its world into showcase/worlds/ (hidden, deleted after the run),
runs one headless engine (OMNISIM_NO_WINDOW=1, nothing on the desktop) and publishes the
frames. README.md lists every image with its world, camera and job progress.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import showcase_lib as lib  # noqa: E402

ROOT = lib.ROOT
DRIVE_WORLD = ROOT / "projects/samples/demos/worlds/manufacturing/spark_rover_drive_test.omniworld"

HEADER = """#OMNISIM R2025a utf8
EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"
EXTERNPROTO "omnisim://projects/appearances/protos/RoughOak.proto"
"""


def rover_node(pose: str = "wave", face: str = "happy", translation=(0, 0, 0.001), yaw: float = 0.0) -> str:
    """The SparkRover as robot.py wrote it for the drive test, on the showcase pose controller."""
    text = lib.read_world(DRIVE_WORLD)
    i = text.index("DEF SPARK_ROVER Robot {")
    node = text[i:].rstrip()
    node = re.sub(r'controller "spark_rover_drive" controllerArgs \[[^\]]*\]',
                  f'controller "spark_rover_pose" controllerArgs [ "--pose" "{pose}" "--face" "{face}" ]', node, count=1)
    node = node.replace("translation -1 0 0.001", "translation {:g} {:g} {:g} rotation 0 0 1 {:g}".format(*translation, yaw), 1)
    return clear_acrylic(node) + "\n"


def clear_acrylic(text: str) -> str:
    """robot.py draws every clear part at transparency 0.75, roughness 0.5: a milky veil over the
    screen. Cast acrylic passes about 92 % (robot.py's own camera note), so the stills draw the
    clear parts at 0.92 and glossy. Only the look changes; the parts and their masses do not."""
    return text.replace("roughness 0.5 metalness 0 transparency 0.75", "roughness 0.08 metalness 0 transparency 0.92")


def fill_light(direction=(-0.6, -0.8, -0.35), intensity: float = 0.9) -> str:
    """An unshadowed fill (the camera renders every light past the sun as one)."""
    return (f'DirectionalLight {{ direction {direction[0]} {direction[1]} {direction[2]} intensity {intensity} '
            f'color 0.92 0.96 1 castShadows FALSE }}\n')


def desk(sun=(2.5, 1.8, 3.2), wall_rgb=(0.60, 0.68, 0.68)) -> str:
    """A classroom desk: a varnished-oak top under the robot and a soft wall well behind it."""
    wall = " ".join(f"{c:g}" for c in wall_rgb)
    return (f'Viewpoint {{ position 1 0 0.3 orientation 0 0 1 3.1416 }}\n'
            f'OmniSimSky {{ }}\nDEF SUN OmniSimSun {{ }}\n'
            f'DEF SUN_MARKER OmniSimSunMarker {{ translation {sun[0]} {sun[1]} {sun[2]} }}\n'
            f'DEF DESK Solid {{ name "desk" translation 0.3 0 -0.02 children [ Shape {{ appearance RoughOak {{ '
            f'textureTransform TextureTransform {{ scale 4 4 }} }} geometry Box {{ size 2.6 3.0 0.04 }} }} ] '
            f'boundingObject Box {{ size 2.6 3.0 0.04 }} }}\n'
            f'DEF WALL Solid {{ name "wall" translation -1.0 0 0.7 children [ Shape {{ appearance PBRAppearance {{ '
            f'baseColor {wall} roughness 0.9 metalness 0 }} geometry Box {{ size 0.02 20 3 }} }} ] }}\n'
            f'DEF SIDE_WALL Solid {{ name "side wall" translation 2.0 1.5 0.7 children [ Shape {{ appearance PBRAppearance {{ '
            f'baseColor {wall} roughness 0.9 metalness 0 }} geometry Box {{ size 6 0.02 3 }} }} ] }}\n'
            f'DEF UNDER Solid {{ name "room floor" translation 0 0 -0.8 children [ Shape {{ appearance PBRAppearance {{ '
            f'baseColor 0.35 0.36 0.38 roughness 0.9 metalness 0 }} geometry Box {{ size 6 6 0.02 }} }} ] }}\n')


def look_board(x: float = 1.3) -> str:
    """Something for the head camera's inset to see, behind the photographer (out of every shot)."""
    return (f'DEF LOOK Solid {{ name "camera board" translation {x} 0 0 children [ '
            f'Pose {{ translation 0 0 0.35 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.95 0.95 0.95 roughness 0.9 metalness 0 }} geometry Box {{ size 0.02 0.9 0.7 }} }} ] }} '
            f'Pose {{ translation -0.012 0.22 0.45 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.15 0.75 0.65 roughness 0.8 metalness 0 }} geometry Box {{ size 0.002 0.2 0.2 }} }} ] }} '
            f'Pose {{ translation -0.012 -0.05 0.25 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.95 0.55 0.15 roughness 0.8 metalness 0 }} geometry Box {{ size 0.002 0.16 0.16 }} }} ] }} '
            f'Pose {{ translation -0.012 -0.27 0.48 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.2 0.35 0.85 roughness 0.8 metalness 0 }} geometry Box {{ size 0.002 0.14 0.14 }} }} ] }} ] }}\n')


def world_info(title: str, extra: str = "") -> str:
    return (f'WorldInfo {{ basicTimeStep 2 title "{title}" newtonSolver "mujoco" newtonSubsteps 2 '
            f'newtonGroundMu 1 FPS 30 {extra}}}\n')


T_SHOT = 2.5                       # s: the arms have eased into their pose and the face is drawn
FACE_C = [0.0752, 0.0, 0.212]      # the display's centre (robot.py display_node)
KEY_SUN = (1.8, -3.0, 2.4)         # the key light: front right, high (the face and the right side)
ROVER_SHOTS = {
    "sparkrover_hero": {"position": [0.80, -0.49, 0.20], "target": [0.0, 0.0, 0.142]},
    "face_camera": {"position": [0.34, -0.15, 0.245], "target": FACE_C},
    "hero_left": {"position": [0.74, 0.50, 0.19], "target": [0.0, 0.0, 0.150]},
    "hero_high": {"position": [0.70, -0.42, 0.36], "target": [0.0, 0.0, 0.140]},
}
CONCEPT_SHOT = {"position": [0.80, -0.43, 0.40], "target": [0.0, -0.005, 0.135]}   # sparkrover_1's hero angle
CONCEPT_CROP = (0, 118, 688, 905)  # the hero and its callouts on sparkrover_1.png


def rover_scene(pose: str, sun=KEY_SUN) -> str:
    return (HEADER + world_info("OmniSim SparkRover Mini - showcase") + desk(sun) + fill_light()
            + look_board() + rover_node(pose, "happy"))


def cmd_rover(a) -> None:
    """sparkrover_hero.png and face_camera.png (plus two alternates for review)."""
    shots = [dict(name=k, at_s=T_SHOT + (0.2 if k == "face_camera" else 0.0), **v) for k, v in ROVER_SHOTS.items()]
    if a.debug:
        shots += [{"name": "dbg_side", "position": [0.0, 1.0, 0.2], "target": [0.0, 0.0, 0.15], "at_s": T_SHOT},
                  {"name": "dbg_top", "position": [0.3, 0.3, 1.2], "target": [0.0, 0.0, 0.0], "at_s": T_SHOT}]
    sun = tuple(float(c) for c in a.sun.split(","))
    run = lib.shoot("rover" + a.tag, rover_scene(a.pose, sun), shots, 3200, 2000, fov=0.6, timeout_s=400, far=3.0)
    for s in shots:
        src = run / "frames" / f"{s['name']}.png"
        if not src.exists():
            continue
        if s["name"] == "face_camera" and not a.review:     # the hero is make_images.py hero
            lib.publish(src, lib.OUT / f"{s['name']}.png")
        else:
            lib.publish(src, lib.SCRATCH / "review" / f"rover{a.tag}_{s['name']}.png")


def srgb_to_linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def design_colours(node: str) -> str:
    """The robot's node as robot.py writes it. Until 2026-11-04 robot.py wrote the design's sRGB
    colours straight into baseColor (read as linear, so every part rendered lighter than designed)
    and the hero converted them here; robot.py now writes linear values itself, so converting
    again would darken every part twice. Kept as the one place the hero's colours pass through."""
    return node


DIFFUSER_MESH = "base__cedede6.stl"     # robot.py's group for diffuser_front and diffuser_rear
LED_TEAL = (0.27, 0.94, 0.88)           # the concept sheet's status light, #44F0E1


def lit_diffusers(node: str, intensity: float = 1.6) -> str:
    """The status ring's diffusers drawn as lit teal. The LEDs behind them are visual meshes in
    the design, not simulated devices; the hero shows them on."""
    def fix(m):
        if DIFFUSER_MESH not in m.group(0):
            return m.group(0)
        r, g, b = LED_TEAL
        return (f'Shape {{ appearance PBRAppearance {{ baseColor {r} {g} {b} roughness 0.4 metalness 0 '
                f'emissiveColor {r} {g} {b} emissiveIntensity {intensity} }} geometry Mesh {{ url "{m.group(1)}" }} }}')
    return re.sub(r'Shape \{ appearance PBRAppearance \{[^}]*\} geometry Mesh \{ url "([^"]*)" \} \}', fix, node)


HERO_SUN = (1.6, 1.4, 1.6)             # key: front left, 35 deg up: lights the face, shadows the right side and desk
HERO_SHOT = {"position": [0.74, -0.46, 0.17], "target": [0.0, 0.0, 0.136]}   # 3/4, at the robot's chest


def studio(sun=HERO_SUN, wall_rgb=(0.20, 0.21, 0.23), sun_rgb=(1.0, 0.86, 0.70), sun_intensity=3.2) -> str:
    """A neutral studio: the oak desk, a dark warm-grey backdrop close behind (inside a 2.6 m far
    plane), a warm key and a cool front fill."""
    wall = " ".join(f"{c:g}" for c in wall_rgb)
    return (f'Viewpoint {{ position 1 0 0.3 orientation 0 0 1 3.1416 }}\n'
            f'OmniSimSky {{ }}\nDEF SUN OmniSimSun {{ color {sun_rgb[0]} {sun_rgb[1]} {sun_rgb[2]} intensity {sun_intensity} }}\n'
            f'DEF SUN_MARKER OmniSimSunMarker {{ translation {sun[0]} {sun[1]} {sun[2]} }}\n'
            f'DEF DESK Solid {{ name "desk" translation 0.2 0 -0.02 children [ Shape {{ appearance RoughOak {{ '
            f'textureTransform TextureTransform {{ scale 3 3 }} }} geometry Box {{ size 2.0 2.4 0.04 }} }} ] '
            f'boundingObject Box {{ size 2.0 2.4 0.04 }} }}\n'
            f'DEF BACKDROP Solid {{ name "backdrop" translation -0.75 0 0.6 children [ Shape {{ appearance PBRAppearance {{ '
            f'baseColor {wall} roughness 0.95 metalness 0 }} geometry Box {{ size 0.02 6 1.4 }} }} ] }}\n'
            f'DEF BACKDROP_SIDE Solid {{ name "backdrop side" translation 0.5 1.0 0.6 children [ Shape {{ appearance PBRAppearance {{ '
            f'baseColor {wall} roughness 0.95 metalness 0 }} geometry Box {{ size 2.6 0.02 1.4 }} }} ] }}\n'
            )


def cmd_hero(a) -> None:
    """sparkrover_hero.png: the robot in its design colours, lit diffusers, a wave, a studio."""
    node = rover_node(a.pose, "happy")
    node = node.replace('"--pose" "{}" "--face" "happy" ]'.format(a.pose),
                        '"--pose" "{}" "--face" "happy" "--right" "{}" ]'.format(a.pose, a.right), 1)
    node = lit_diffusers(design_colours(node))
    sun = tuple(float(c) for c in a.sun.split(","))
    text = (HEADER + world_info("OmniSim SparkRover Mini - hero") + studio(sun, sun_intensity=a.key)
            + fill_light((-0.85, 0.35, -0.40), a.fill) + look_board() + node)
    shots = [dict(name="sparkrover_hero", at_s=T_SHOT, **HERO_SHOT)]
    if a.debug:
        shots += [{"name": "dbg_top", "position": [0.35, -0.35, 1.1], "target": [0.0, 0.0, 0.0], "at_s": T_SHOT},
                  {"name": "dbg_side", "position": [0.45, -0.60, 0.17], "target": [0.0, 0.0, 0.15], "at_s": T_SHOT}]
    run = lib.shoot("hero" + a.tag, text, shots, 3200, 2000, fov=0.62, timeout_s=400, far=2.6, exposure=a.exposure)
    publish_named(run, shots, () if a.review else ("sparkrover_hero",), a)


def cmd_concept(a) -> None:
    """concept_vs_built.png: the concept sheet's hero beside the simulated robot at its angle."""
    from PIL import Image, ImageDraw, ImageFont
    sheet = Image.open(lib.OUT / "concept/sparkrover_1.png").convert("RGB").crop(CONCEPT_CROP)
    pw, ph = sheet.size
    shots = [dict(name="concept_angle", at_s=T_SHOT, **CONCEPT_SHOT)]
    run = lib.shoot("concept" + a.tag, rover_scene("concept"), shots, 2 * pw, 2 * ph, fov=0.37, timeout_s=400, far=3.0)
    built = Image.open(run / "frames" / "concept_angle.png").convert("RGB").resize((pw, ph), Image.LANCZOS)
    pad, cap = 24, 64
    out = Image.new("RGB", (2 * pw + 3 * pad, ph + 2 * pad + cap), (246, 244, 239))
    out.paste(sheet, (pad, pad + cap))
    out.paste(built, (2 * pad + pw, pad + cap))
    d = ImageDraw.Draw(out)
    font = ROOT / "resources/branding/omnisim/fonts/SpaceGrotesk.ttf"
    f1 = ImageFont.truetype(str(font), 30)
    f2 = ImageFont.truetype(str(font), 20)
    for x, head, sub in ((pad, "Concept", "AI-generated concept sheet"),
                         (2 * pad + pw, "Built", "the design, simulated in OmniSim at the sheet's angle")):
        d.text((x, pad), head, font=f1, fill=(20, 20, 13))
        d.text((x + d.textlength(head, font=f1) + 14, pad + 9), sub, font=f2, fill=(90, 90, 80))
    dst = lib.OUT / "concept_vs_built.png" if not a.review else lib.SCRATCH / "review" / "concept_vs_built.png"
    out.save(dst, optimize=True)
    print(f"[showcase] {dst} {out.size}")


def _parts_scene(sun=(1.8, -3.0, 2.4)) -> str:
    return (HEADER + world_info("OmniSim SparkRover Mini - parts") + desk(sun)
            + fill_light((-0.6, -0.8, -0.35), 0.9))


def _side(label: str) -> int:
    return 1 if label.endswith("_left") or "_left_" in label else (-1 if label.endswith("_right") or "_right_" in label else 0)


def cmd_exploded(a) -> None:
    import parts
    idx = parts.export_parts()
    by = {}
    for p in idx:
        by.setdefault(p["owner"], []).append(p)
    lift = a.spread
    text = _parts_scene()
    text += parts.group_node(by.get("chassis", []), (0, 0, 0), def_name="CHASSIS")
    text += parts.group_node(by.get("electrical", []), (0.30 * lift, 0, 0.03 * lift), def_name="ELECTRICAL")
    text += parts.group_node(by.get("shell", []), (0, 0, 0.25 * lift), def_name="SHELL")
    arms_l = [p for p in by.get("arms", []) if _side(p["label"]) >= 0]
    arms_r = [p for p in by.get("arms", []) if _side(p["label"]) < 0]
    text += parts.group_node(arms_l, (0.0, 0.18 * lift, 0.06 * lift), def_name="ARMS_L")
    text += parts.group_node(arms_r, (0.0, -0.18 * lift, 0.06 * lift), def_name="ARMS_R")
    shots = [{"name": "exploded", "position": [1.30, -1.05, 0.80], "target": [0.10, 0.0, 0.24], "at_s": 0.3},
             {"name": "exploded_b", "position": [0.85, -1.40, 0.70], "target": [0.10, 0.0, 0.24], "at_s": 0.3},
             {"name": "exploded_c", "position": [1.45, -0.70, 0.95], "target": [0.10, 0.0, 0.22], "at_s": 0.3}]
    run = lib.shoot("exploded" + a.tag, text, shots, 3200, 2000, fov=0.6, timeout_s=300, far=5.0)
    publish_named(run, shots, ("exploded",), a)


def publish_named(run: Path, shots, published, a) -> None:
    """The shots named in `published` go to docs/images/manufacturing/; the rest are alternates."""
    for s in shots:
        src = run / "frames" / f"{s['name']}.png"
        if not src.exists():
            continue
        if s["name"] in published and not getattr(a, "review", False):
            lib.publish(src, lib.OUT / f"{s['name']}.png")
        else:
            lib.publish(src, lib.SCRATCH / "review" / f"{s['name']}{a.tag}.png")


def cmd_wiring(a) -> None:
    import parts
    idx = parts.export_parts()
    keep = [p for p in idx if p["owner"] != "shell"]
    text = _parts_scene()
    text += parts.group_node([p for p in keep if p["owner"] == "chassis"], def_name="CHASSIS", fade=a.fade)
    text += parts.group_node([p for p in keep if p["owner"] in ("electrical", "arms")], def_name="BOARDS")
    ws = parts.wires()
    text += ('DEF WIRES Pose { children [ ' + " ".join(parts.shape(w["file"], w["rgb"], 0.0, 0.35, 0.0) for w in ws)
             + " ] }\n")
    shots = [{"name": "wiring", "position": [-0.58, -0.50, 0.35], "target": [0.0, 0.0, 0.145], "at_s": 0.3},
             {"name": "wiring_close", "position": [-0.36, -0.30, 0.28], "target": [-0.02, 0.0, 0.12], "at_s": 0.3},
             {"name": "wiring_left", "position": [-0.55, 0.48, 0.36], "target": [0.0, 0.0, 0.155], "at_s": 0.3}]
    run = lib.shoot("wiring" + a.tag, text, shots, 3200, 2000, fov=0.6, timeout_s=300, far=5.0)
    print(f"[showcase] wiring: {len(ws)} wire meshes")
    publish_named(run, shots, ("wiring",), a)


def cmd_printed(a) -> None:
    """printed_parts.png: every printed part's model (release/models, in its print orientation, as
    make_print.py slices it) laid flat on the desk, in the design's colours. These are the models
    the SV08 twin printed, not the twin's beads: PRINT.md has what the twin laid."""
    import numpy as np
    import parts
    product = ROOT / "projects/manufacturing/products/spark_rover"
    printed = json.loads((product / "release/make/print/printed.json").read_text(encoding="utf-8"))["parts"]
    colour = {p["label"]: p["rgb"] for p in parts.export_parts()}
    sys.path.insert(0, str(product))
    import robot  # noqa: E402
    items = []
    for p in printed:
        stl = product / "release/models" / f"{p['label']}.stl"
        if p.get("superseded") or not stl.exists():      # a superseded drawing is not a part
            continue
        import struct
        b = stl.read_bytes()
        n = struct.unpack("<I", b[80:84])[0]
        v = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))["v"]
        v = v.reshape(-1, 3, 3).astype(float) * 1e-3
        lo, hi = v.reshape(-1, 3).min(0), v.reshape(-1, 3).max(0)
        v = v - np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])
        out = lib.SCRATCH / "meshes" / "printed" / f"{p['label']}.stl"
        robot.write_stl(out, v, p["label"])
        rgb = colour.get(p["label"]) or ((0.25, 0.25, 0.27) if p.get("material") == "TPU" else (0.93, 0.93, 0.90))
        items.append({"label": p["label"], "file": str(out), "rgb": rgb, "size": hi - lo})
    items.sort(key=lambda it: it["size"][0] * it["size"][1])   # small parts in front, the shells behind
    # shelf packing in rows along y, rows stepping back in -x, 12 mm between parts
    gap, row_w, x, y, depth = 0.025, 0.56, 0.0, 0.0, 0.0
    for it in items:
        sx, sy = it["size"][0], it["size"][1]
        if y + sy > row_w and y > 0:
            x -= depth + gap
            y, depth = 0.0, 0.0
        it["at"] = (x - sx / 2, y + sy / 2 - row_w / 2)
        y += sy + gap
        depth = max(depth, sx)
    xs = [it["at"][0] for it in items]
    cx = (max(xs) + min(xs)) / 2
    print(f"[showcase] layout x {min(xs) - cx:.3f}..{max(xs) - cx:.3f} m, {len(items)} parts")
    text = _parts_scene()
    for it in items:
        px, py = it["at"]
        text += (f'Pose {{ translation {px - cx + 0.05:.4f} {py:.4f} 0.0005 children [ '
                 + parts.shape(it["file"], it["rgb"], 0.0, 0.55, 0.0) + " ] }\n")
    shots = [{"name": "printed_parts", "position": [1.32, -0.24, 1.02], "target": [0.06, 0.0, 0.0], "at_s": 0.3},
             {"name": "printed_low", "position": [0.70, -0.30, 0.30], "target": [0.0, 0.0, 0.02], "at_s": 0.3}]
    run = lib.shoot("printed" + a.tag, text, shots, 3200, 2000, fov=0.6, timeout_s=300, far=4.0)
    print(f"[showcase] printed parts: {len(items)}")
    publish_named(run, shots, ("printed_parts",), a)


ASSEMBLY_PICK = (1, 3, 4, 6, 8, 10, 14, 18)
ASSEMBLY_CAPTION = {     # short forms of assemble.py's titles (each picture holds every step up to it)
    1: "Deck: the two plywood layers, glued", 3: "Gearmotors, hex adapters and wheels",
    4: "Spine foot, then the glued spine", 6: "Shoulder brackets onto the spine",
    8: "Bay carrier, loaded, from behind", 10: "Front carrier, loaded, from the front",
    14: "Lower, then upper shell halves", 18: "Arm modules, last, from outside"}


def assembly_groups(idx, steps):
    """step -> parts: assemble.py's units, and every part no unit names (screws, nuts) at the
    step of the nearest part that one does."""
    import numpy as np
    step_of = {lab: s["step"] for s in steps for lab in s["parts"]}
    placed = [p for p in idx if p["label"] in step_of]
    cent = np.array([p["centre_mm"] for p in placed], float)
    out = {}
    for p in idx:
        k = step_of.get(p["label"])
        if k is None:
            j = int(np.argmin(np.linalg.norm(cent - np.asarray(p["centre_mm"], float), axis=1)))
            k = step_of[placed[j]["label"]]
        out.setdefault(k, []).append(p)
    return out


def cmd_assembly(a) -> None:
    import parts
    idx = parts.export_parts()
    steps = parts.steps()
    groups = assembly_groups(idx, steps)
    text = _parts_scene()
    for k in sorted(groups):
        text += parts.group_node(groups[k], def_name=f"STEP_{k}")
    shots = []
    for k in ASSEMBLY_PICK:
        hide = [f"STEP_{j}" for j in groups if j > k]
        shots.append({"name": f"step_{k:02d}", "position": [0.56, -0.47, 0.40], "target": [0.0, 0.0, 0.125],
                      "at_s": 0.3, "hide": hide, "settle_steps": 2})
    run = lib.shoot("assembly" + a.tag, text, shots, 1600, 1200, fov=0.62, timeout_s=300, far=3.0)
    sheet = compose_assembly(run, steps)
    if not a.review:
        lib.publish(sheet, lib.OUT / "assembly_steps.png")


def compose_assembly(run: Path, steps) -> Path:
    from PIL import Image, ImageDraw, ImageFont
    titles = {s["step"]: s["title"] for s in steps}
    cols, rows, tw, th, pad, cap = 4, 2, 800, 600, 16, 104
    sheet = Image.new("RGB", (cols * tw + (cols + 1) * pad, rows * (th + cap) + (rows + 1) * pad), (246, 244, 239))
    draw = ImageDraw.Draw(sheet)
    fonts = ROOT / "resources/branding/omnisim/fonts"
    try:
        f_big = ImageFont.truetype(str(next(fonts.glob("SpaceGrotesk*.ttf"))), 38)
        f_small = ImageFont.truetype(str(next(fonts.glob("SpaceGrotesk*.ttf"))), 26)
    except (StopIteration, OSError):
        f_big = ImageFont.truetype("arial.ttf", 30)
        f_small = ImageFont.truetype("arial.ttf", 21)
    for i, k in enumerate(ASSEMBLY_PICK):
        src = run / "frames" / f"step_{k:02d}.png"
        if not src.exists():
            continue
        im = Image.open(src).convert("RGB").resize((tw, th), Image.LANCZOS)
        x = pad + (i % cols) * (tw + pad)
        y = pad + (i // cols) * (th + cap + pad)
        sheet.paste(im, (x, y))
        draw.text((x + 4, y + th + 10), f"Step {k}", font=f_big, fill=(20, 20, 13))
        t = ASSEMBLY_CAPTION.get(k) or titles.get(k, "")
        draw.text((x + 4, y + th + 58), t, font=f_small, fill=(70, 70, 60))
    out = lib.SCRATCH / "review" / "assembly_steps.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out)
    return out


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd")
    r = sub.add_parser("rover", help="face_camera.png (and the first hero, for review)")
    r.add_argument("--pose", default="greet_r")
    r.add_argument("--sun", default=",".join(f"{c:g}" for c in KEY_SUN))
    r.add_argument("--tag", default="")
    r.add_argument("--review", action="store_true", help="write every frame to the review folder only")
    r.add_argument("--debug", action="store_true")
    h = sub.add_parser("hero", help="sparkrover_hero.png")
    h.add_argument("--pose", default="rest")
    h.add_argument("--right", default="0.9,1.0", help="the right arm shoulder,elbow (rad)")
    h.add_argument("--exposure", type=float, default=1.0)
    h.add_argument("--sun", default=",".join(f"{c:g}" for c in HERO_SUN))
    h.add_argument("--key", type=float, default=3.2, help="the key light's intensity")
    h.add_argument("--fill", type=float, default=0.35)
    h.add_argument("--tag", default="")
    h.add_argument("--review", action="store_true")
    h.add_argument("--debug", action="store_true")
    c = sub.add_parser("concept", help="concept_vs_built.png")
    c.add_argument("--tag", default="")
    c.add_argument("--review", action="store_true")
    e = sub.add_parser("exploded")
    e.add_argument("--review", action="store_true")
    e.add_argument("--spread", type=float, default=1.0)
    e.add_argument("--tag", default="")
    w = sub.add_parser("wiring")
    w.add_argument("--review", action="store_true")
    w.add_argument("--fade", type=float, default=0.0, help="transparency of the chassis parts")
    w.add_argument("--tag", default="")
    s = sub.add_parser("assembly")
    s.add_argument("--review", action="store_true")
    s.add_argument("--tag", default="")
    q = sub.add_parser("printed", help="printed_parts.png")
    q.add_argument("--tag", default="")
    q.add_argument("--review", action="store_true")
    a = p.parse_args(argv)
    if a.cmd == "rover":
        cmd_rover(a)
    elif a.cmd == "hero":
        cmd_hero(a)
    elif a.cmd == "concept":
        cmd_concept(a)
    elif a.cmd == "exploded":
        cmd_exploded(a)
    elif a.cmd == "wiring":
        cmd_wiring(a)
    elif a.cmd == "assembly":
        cmd_assembly(a)
    elif a.cmd == "printed":
        cmd_printed(a)
    else:
        p.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
