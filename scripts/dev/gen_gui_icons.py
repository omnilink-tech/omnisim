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

"""Regenerate OmniSim's GUI icon set from Apache-2.0 sources.

Why: resources/icons/{dark,light} were icons8 glyphs under CC BY-NoDerivs, and the
license file itself says "slightly modified" -- ND forbids derivatives, so the set was
being used outside its licence and no NOTICE entry can cure that.

What ships instead:
  * 44 toolbar/menu glyphs  -> Google Material Symbols (Apache-2.0), rendered from SVG.
  * 6 viewpoint glyphs      -> drawn here (an isometric cube with one face lit).
                               Material has no cube-face icon; these are OmniSim's own.
  * 16 small UI primitives  -> drawn here. At 10-14 px a downscaled Material glyph is
                               mush; crisp geometry is both cleaner and licence-free.

dark/ = enabled state (#333333), light/ = disabled state (#CCCCCC). The two differ only
in colour -- identical alpha silhouette -- which is exactly how the .qss themes use them
(qproperty-enabledIconPath / qproperty-disabledIconPath).
"""
import io
import os
import re
import sys
import urllib.request

from PIL import Image, ImageDraw
from svgelements import SVG, Path

SRC = ("https://raw.githubusercontent.com/google/material-design-icons/master/"
       "symbols/web/{n}/materialsymbolsoutlined/{n}_24px.svg")

ENABLED = (51, 51, 51)     # #333333  -> icons/dark
DISABLED = (204, 204, 204)  # #CCCCCC -> icons/light
SS = 4

# ---------------------------------------------------------------- Material mappings
# omnisim icon name -> Material Symbols name.  Chosen for semantic match with the
# action the button actually performs in the OmniSim GUI.
MATERIAL = {
    "clean_button":                  "cleaning_services",
    "close_button":                  "close",
    "copy_button":                   "content_copy",
    "cut_button":                    "content_cut",
    "delete_button":                 "delete",
    "edit_field_button":             "edit",
    "export_button":                 "upload",
    "fast_button":                   "fast_forward",
    "find_button":                   "search",
    "front_view":                    None,   # drawn
    "help_button":                   "help",
    "hide_side_bar":                 "left_panel_close",
    "import_button":                 "download",
    "insert_after_button":           "playlist_add",
    "left_arrow_button":             "chevron_left",
    "make_button":                   "build",
    "move_viewpoint_to_object_button": "center_focus_strong",
    "movie_black_button":            "movie",
    "movie_red_button":              "movie",
    "new_button":                    "note_add",
    "no_rendering":                  "visibility_off",
    "open_button":                   "folder_open",
    "paste_button":                  "content_paste",
    "pause_button":                  "pause",
    "real_time_button":              "play_arrow",
    "reload_button":                 "refresh",
    "rendering":                     "visibility",
    "replace_button":                "find_replace",
    "reset_button":                  "restart_alt",
    "reset_simulation_button":       "replay",
    "restore_viewpoint_button":      "settings_backup_restore",
    "right_arrow_button":            "chevron_right",
    "save_as_button":                "save_as",
    "save_button":                   "save",
    "screenshot_button":             "photo_camera",
    "share_button":                  "share",
    "share_red_button":              "share",
    "show_side_bar":                 "left_panel_open",
    "sound_mute_button":             "volume_off",
    "sound_unmute_button":           "volume_up",
    "step_button":                   "skip_next",
    "transform_button":              "open_with",
    "macos_hover_tab_close_button":       "close",
    "macos_selected_tab_close_button":    "close",
    "macos_unselected_tab_close_button":  "close",
}

VIEWS = {  # face of the cube to light up
    "front_view": "front", "back_view": "back", "left_view": "left",
    "right_view": "right", "top_view": "top", "bottom_view": "bottom",
}

_cache = {}


def fetch(name):
    if name not in _cache:
        _cache[name] = urllib.request.urlopen(SRC.format(n=name), timeout=40).read()
    return _cache[name]


def _pts(sub, steps=64):
    out = []
    for seg in sub:
        try:
            for i in range(steps + 1):
                p = seg.point(i / steps)
                out.append((float(p.real), float(p.imag)))
        except Exception:
            pass
    return out


def render_material(mname, size):
    """SVG -> alpha mask.  Even-odd via XOR so interior counters survive."""
    b = fetch(mname)
    m = re.search(rb'\bwidth\s*=\s*"([\d.]+)"', b)
    extent = float(m.group(1)) if m else 24.0
    big = size * SS
    k = big / extent
    acc = Image.new("L", (big, big), 0)
    for el in SVG.parse(io.BytesIO(b)).elements():
        if not isinstance(el, Path):
            continue
        for sub in el.as_subpaths():
            p = _pts(sub)
            if len(p) < 3:
                continue
            layer = Image.new("L", (big, big), 0)
            ImageDraw.Draw(layer).polygon([(x * k, y * k) for x, y in p], fill=255)
            acc = Image.frombytes("L", acc.size, bytes(
                a ^ c for a, c in zip(acc.tobytes(), layer.tobytes())))
    return acc.resize((size, size), Image.LANCZOS)


# Where the camera sits for each named view, in icon space (unit offsets from centre),
# and which isometric face (if any) that view is looking at.  All six get a DISTINCT
# arrow direction, so the glyphs are unambiguous -- the original set solved this by
# rotating a stylised eye, which is a design we must not reproduce.
VIEW_ARROW = {
    "top":    ((0, -1), "top"),      # camera above, looking down
    "bottom": ((0, 1), None),        # camera below, looking up  (face hidden)
    "left":   ((-1, 0), "left"),     # camera to the left
    "right":  ((1, 0), "right"),     # camera to the right
    "front":  ((-0.72, 0.72), "left"),   # camera front-lower-left
    "back":   ((0.72, -0.72), None),     # camera back-upper-right (face hidden)
}


def render_view(face, size):
    """Wireframe isometric cube + a bold arrow showing where the camera looks from.

    OmniSim's own glyph. Six distinct arrow directions => six distinguishable icons;
    the face being viewed is filled when it is one of the three visible faces.
    """
    big = size * SS
    im = Image.new("L", (big, big), 0)
    d = ImageDraw.Draw(im)
    u = big / 24.0
    cx, cy, r = 12 * u, 12.2 * u, 5.6 * u

    top = [(cx, cy - r), (cx + r, cy - r / 2), (cx, cy), (cx - r, cy - r / 2)]
    left = [(cx - r, cy - r / 2), (cx, cy), (cx, cy + r), (cx - r, cy + r / 2)]
    right = [(cx + r, cy - r / 2), (cx, cy), (cx, cy + r), (cx + r, cy + r / 2)]
    faces = {"top": top, "left": left, "right": right}

    w = max(2, int(1.1 * u))
    for poly in (top, left, right):
        d.polygon(poly, outline=255, width=w)

    (ax, ay), lit = VIEW_ARROW[face]
    if lit:
        d.polygon(faces[lit], fill=255)

    # bold arrow, tip pointing at the cube from the camera's side
    L = 10.4 * u          # how far out the arrow tail sits
    tip = (cx + ax * (r + 1.5 * u), cy + ay * (r + 1.5 * u))
    tail = (cx + ax * L, cy + ay * L)
    # perpendicular for the arrowhead
    px, py = -ay, ax
    hw = 2.1 * u
    head = [tip,
            (tip[0] + (tail[0] - tip[0]) * 0.45 + px * hw,
             tip[1] + (tail[1] - tip[1]) * 0.45 + py * hw),
            (tip[0] + (tail[0] - tip[0]) * 0.45 - px * hw,
             tip[1] + (tail[1] - tip[1]) * 0.45 - py * hw)]
    d.line([tail, tip], fill=255, width=max(2, int(1.5 * u)))
    d.polygon(head, fill=255)
    return im.resize((size, size), Image.LANCZOS)


def _prim(size, fn):
    big = size * SS
    im = Image.new("L", (big, big), 0)
    fn(ImageDraw.Draw(im), big)
    return im.resize((size, size), Image.LANCZOS)


def tri(dr, n, direction):
    m = n * 0.22
    pts = {
        "down":  [(m, m * 1.4), (n - m, m * 1.4), (n / 2, n - m)],
        "up":    [(m, n - m * 1.4), (n - m, n - m * 1.4), (n / 2, m)],
        "right": [(m * 1.4, m), (m * 1.4, n - m), (n - m, n / 2)],
        "left":  [(n - m * 1.4, m), (n - m * 1.4, n - m), (m, n / 2)],
    }[direction]
    dr.polygon(pts, fill=255)


PRIMS = {
    "tree_branch_closed":        lambda d, n: tri(d, n, "right"),
    "tree_branch_closed_hover":  lambda d, n: tri(d, n, "right"),
    "tree_branch_open":          lambda d, n: tri(d, n, "down"),
    "tree_branch_open_hover":    lambda d, n: tri(d, n, "down"),
    "menu-drop-down":            lambda d, n: tri(d, n, "down"),
    "spinbox_up_arrow":          lambda d, n: tri(d, n, "up"),
    "spinbox_down_arrow":        lambda d, n: tri(d, n, "down"),
    "checkboxCheckmark":         lambda d, n: d.line(
        [(n * .18, n * .52), (n * .42, n * .78), (n * .84, n * .22)],
        fill=255, width=max(2, int(n * .14))),
    "dock_close_button":         lambda d, n: (
        d.line([(n * .2, n * .2), (n * .8, n * .8)], fill=255, width=max(1, int(n * .12))),
        d.line([(n * .8, n * .2), (n * .2, n * .8)], fill=255, width=max(1, int(n * .12)))),
    "dock_float_button":         lambda d, n: d.rectangle(
        [n * .22, n * .22, n * .78, n * .78], outline=255, width=max(1, int(n * .12))),
    "dock_maximize_button":      lambda d, n: d.rectangle(
        [n * .16, n * .16, n * .84, n * .84], outline=255, width=max(1, int(n * .12))),
    "dock_minimize_button":      lambda d, n: d.rectangle(
        [n * .18, n * .56, n * .82, n * .70], fill=255),
    "field":                     lambda d, n: d.ellipse(
        [n * .24, n * .24, n * .76, n * .76], fill=255),
    "node":                      lambda d, n: d.rectangle(
        [n * .2, n * .2, n * .8, n * .8], fill=255),
    "proto":                     lambda d, n: d.rectangle(
        [n * .18, n * .18, n * .82, n * .82], outline=255, width=max(1, int(n * .18))),
}


def colourise(mask, rgb):
    out = Image.new("RGBA", mask.size, rgb + (0,))
    out.putalpha(mask)
    return out


# --- state-carrying variants -------------------------------------------------------
# These are NOT decoration: the red dot means "recording / active", and the three macOS
# tab-close greys are hover / selected / unselected. Dropping them would be a functional
# regression, so they are reproduced explicitly.
RED = (200, 60, 55)

# icon -> the exact fill colour used in dark/ (enabled). light/ (disabled) stays grey.
MACOS_TAB = {
    "macos_hover_tab_close_button":      (199, 199, 239),
    "macos_selected_tab_close_button":   (51, 51, 51),
    "macos_unselected_tab_close_button": (214, 214, 214),
}
RED_BADGE = {"movie_red_button", "share_red_button"}


def with_red_badge(img):
    """Overlay the red 'active' dot the original carried."""
    n = img.size[0]
    d = ImageDraw.Draw(img)
    r = n * 0.17
    cx, cy = n * 0.74, n * 0.74
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=RED + (255,))
    return img


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else "resources/icons"
    light_dir, dark_dir = os.path.join(root, "light"), os.path.join(root, "dark")

    # IMPORTANT: dark/ and light/ do not always agree on size (the dock buttons are
    # 13x13 enabled but 10x10 disabled). Size each output from ITS OWN directory, never
    # from the other one, or the GUI gets subtly wrong-sized chrome.
    def size_of(d, name, fallback):
        p = os.path.join(d, name + ".png")
        return Image.open(p).size[0] if os.path.exists(p) else fallback

    names = sorted(f[:-4] for f in os.listdir(light_dir) if f.endswith(".png"))

    made, skipped = 0, []
    for name in names:
        if name == "time_indicator_background":
            continue  # 1x3 px UI strip, no creative content
        try:
            def build(px):
                if name in VIEWS:
                    return render_view(VIEWS[name], px)
                if name in PRIMS:
                    return _prim(px, PRIMS[name])
                if MATERIAL.get(name):
                    return render_material(MATERIAL[name], px)
                return None

            px_l = size_of(light_dir, name, 512)
            px_d = size_of(dark_dir, name, px_l)
            m_l, m_d = build(px_l), build(px_d)
            if m_l is None:
                skipped.append(name)
                continue

            dk = colourise(m_d, MACOS_TAB.get(name, ENABLED))
            lt = colourise(m_l, DISABLED)
            if name in RED_BADGE:
                dk = with_red_badge(dk)
                lt = with_red_badge(lt)
            dk.save(os.path.join(dark_dir, name + ".png"))
            lt.save(os.path.join(light_dir, name + ".png"))
            made += 1
        except Exception as e:  # noqa: BLE001
            skipped.append(f"{name} ({e})")

    print(f"  generated {made} icons (dark+light pairs)")
    if skipped:
        print("  SKIPPED (no mapping):")
        for s in skipped:
            print("   -", s)


if __name__ == "__main__":
    main()
