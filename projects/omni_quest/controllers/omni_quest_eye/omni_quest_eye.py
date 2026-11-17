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

"""omni_quest_eye — the robot's cameras + vision, on a tracking sidecar.

Device children mis-bind on an imported URDFRobot, so the robot's camera(s) ride
this separate Robot which snaps onto the tracked robot every tick (the proven
husky_eye pattern). It carries:

  * front_camera (required) — forward free-space: obstacle fraction in the
    Left / Centre / Right thirds of the near field, for steering avoidance.
  * left_camera / right_camera (optional) — look forward-diagonally to the
    sides, so the robot can see CROSSING traffic approaching an intersection
    before it is dead ahead. Reported as a single obstacle fraction each. This
    is what lets a 4-way crossing be solved camera-only ("yield to the right").

Obstacle = a near-field pixel that is neither grass (green) nor sky (bright):
a tree, rock, barrel, crate, wall — or another robot. Vision is vectorised with
numpy so the eye stays in lockstep with the nav controller (a pure-Python pixel
loop lagged badly under load and the nav read stale perception). Writes
_perception[_id].json.
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

try:
    import cv2
    _CV2 = True
except Exception:
    _CV2 = False

try:
    import learned_seg          # portable numpy free-space segmenter (model_seg.npz)
except Exception:
    learned_seg = None
LEARNED_MODEL = None            # loaded in main() when OQ_LEARNED_SEG is set

from omnisim import Supervisor

PROJ = Path(__file__).resolve().parents[2]

CAM_GREEN_MARGIN = 12      # green must lead red by this to count as grass
CAM_SKY_BRIGHT = 165       # brighter than this => sky / glare (free)
# --road: also treat grey, low-saturation, mid-brightness pixels as free DRIVABLE
# surface (asphalt road / pavement / concrete), for city worlds. Coloured walls
# and most cars are saturated or dark, so they still read as obstacles.
ROAD_SAT_MAX = 30          # max channel spread (max-min) to count as grey road
ROAD_BRIGHT_LO = 38
ROAD_BRIGHT_HI = 205
ROAD_MODE = False          # set from --road in main()
GROUND_TOL = 78            # L1 colour distance to the ground-in-front to count as free
# Stereo depth (when stereo_left/stereo_right cameras are present)
STEREO_BASELINE_M = 0.10   # camera separation
STEREO_MAXR = 8.0          # report this where nothing near stands up (free)
STEREO_MIN_DISP = 0.8      # disparities below this are "no match" -> treat as far/free
STEREO_PITCH = 0.30        # camera down-pitch (rad), matches the world camera rotation
STEREO_CAM_H = 0.45        # camera height above the walkable surface (m)
STEREO_OBST_H = 0.25       # a point this far above the ground counts as an obstacle


def _obstacle_mask(image, w, h):
    """Bool (lower-band x w) mask: pixels that are NOT free walkable surface. Free =
    similar in colour to the ground directly in front of the robot (adapts to whatever
    the pavement is, the robust cue) OR grass OR sky/glare OR (--road) generic grey."""
    if LEARNED_MODEL is not None and learned_seg is not None:
        # LEARNED path: a trained segmenter (sim + domain randomisation, transfers to
        # real) replaces the hand thresholds. Same lower-band (h//2 x w) bool interface.
        bgr = np.frombuffer(image, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
        full = learned_seg.segment(bgr, LEARNED_MODEL)        # bool at SEG_SIZE
        m = cv2.resize(full.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST)
        return m[h // 2:, :].astype(bool)
    arr = np.frombuffer(image, dtype=np.uint8).reshape(h, w, 4).astype(np.int16)
    band = arr[h // 2:, :, :3]           # near field = lower half (BGR)
    b, g, r = band[:, :, 0], band[:, :, 1], band[:, :, 2]
    bright = (r + g + b) / 3.0
    # reference ground colour = the patch right in front of the robot (bottom-centre)
    ref = arr[int(h * 0.86):, int(w * 0.30):int(w * 0.70), :3]
    if ref.size:
        rb, rg, rr = (float(ref[:, :, 0].mean()), float(ref[:, :, 1].mean()),
                      float(ref[:, :, 2].mean()))
    else:
        rb = rg = rr = 128.0
    near_ground = (np.abs(b - rb) + np.abs(g - rg) + np.abs(r - rr)) < GROUND_TOL
    free = near_ground | (bright > CAM_SKY_BRIGHT) \
        | ((g > r + CAM_GREEN_MARGIN) & (g >= b))
    if ROAD_MODE:
        spread = (np.maximum(np.maximum(r, g), b)
                  - np.minimum(np.minimum(r, g), b))
        free = free | ((spread < ROAD_SAT_MAX) & (bright >= ROAD_BRIGHT_LO)
                       & (bright <= ROAD_BRIGHT_HI))
    return ~free


def obstacle_sectors(image, w, h):
    """(left, centre, right) obstacle fractions in the lower image band."""
    if not image:
        return (0.0, 0.0, 0.0)
    m = _obstacle_mask(image, w, h)
    t1, t2 = w // 3, 2 * w // 3
    return (round(float(m[:, :t1].mean()), 4),
            round(float(m[:, t1:t2].mean()), 4),
            round(float(m[:, t2:].mean()), 4))


def obstacle_fraction(image, w, h):
    """Single near-field obstacle fraction (for the side cameras)."""
    if not image:
        return 0.0
    return round(float(_obstacle_mask(image, w, h).mean()), 4)


def free_profile(image, w, h, nbins=24):
    """Per-column-bin obstacle fraction across the near band (0 = open, 1 = blocked).
    Much finer than L/C/R so the nav can FOLLOW-THE-GAP around street furniture
    instead of only knowing 'something is on the left'."""
    if not image:
        return [0.0] * nbins
    m = _obstacle_mask(image, w, h)            # (h//2, w) bool near-field mask
    cols = m.shape[1]
    edges = np.linspace(0, cols, nbins + 1).astype(int)
    out = []
    for k in range(nbins):
        seg = m[:, edges[k]:edges[k + 1]]
        out.append(round(float(seg.mean()), 3) if seg.size else 1.0)
    return out


def stereo_depth_profile(img_l, img_r, w, h, fov, matcher, nbins=24):
    """Per-column-bin nearest-obstacle distance (m) from a stereo pair. Low-texture
    pavement gives no disparity -> reads far/free; structures and pedestrians (edges)
    match and read close. Returns nbins distances (left..right), or None."""
    if not (img_l and img_r and matcher is not None):
        return None, None
    al = np.frombuffer(img_l, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float32)
    ar = np.frombuffer(img_r, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float32)
    gl = (0.114 * al[:, :, 0] + 0.587 * al[:, :, 1] + 0.299 * al[:, :, 2]).astype(np.uint8)
    gr = (0.114 * ar[:, :, 0] + 0.587 * ar[:, :, 1] + 0.299 * ar[:, :, 2]).astype(np.uint8)
    disp = matcher.compute(gl, gr).astype(np.float32) / 16.0
    f = (w / 2.0) / math.tan(fov / 2.0)
    depth = np.full((h, w), STEREO_MAXR, dtype=np.float32)
    valid = disp > STEREO_MIN_DISP
    depth[valid] = np.clip(f * STEREO_BASELINE_M / disp[valid], 0.05, STEREO_MAXR)
    # Reconstruct each pixel's HEIGHT above the ground and its horizontal distance,
    # then keep only points that STAND UP (height over a threshold). This excludes the
    # flat ground plane, so a column reads "near" only when a real obstacle is there.
    rows = np.arange(h, dtype=np.float32)[:, None]
    y_cam = (rows - h / 2.0) * depth / f
    cth, sth = math.cos(STEREO_PITCH), math.sin(STEREO_PITCH)
    height = STEREO_CAM_H - (depth * sth + y_cam * cth)
    horiz = depth * cth - y_cam * sth
    obst = ((depth < STEREO_MAXR - 0.1) & (height > STEREO_OBST_H)
            & (horiz > 0.3) & (horiz < STEREO_MAXR))
    horiz_obst = np.where(obst, horiz, np.float32(STEREO_MAXR))
    edges = np.linspace(0, w, nbins + 1).astype(int)
    out = []
    for k in range(nbins):
        seg = horiz_obst[:, edges[k]:edges[k + 1]]
        hit = seg[seg < STEREO_MAXR - 0.1]
        # need a few obstacle pixels (reject stray mismatches) and use a low percentile
        # rather than the single closest, so one bad pixel can't block a direction.
        out.append(round(float(np.percentile(hit, 12)), 2) if hit.size >= 5
                   else STEREO_MAXR)
    return out, obst


def parse_args(argv):
    target, rid, road = "HUSKY", None, False
    i = 0
    while i < len(argv):
        if argv[i] == "--target" and i + 1 < len(argv):
            target = argv[i + 1]; i += 2
        elif argv[i] == "--id" and i + 1 < len(argv):
            rid = argv[i + 1]; i += 2
        elif argv[i] == "--road":
            road = True; i += 1
        else:
            i += 1
    return target, rid, road


def main() -> int:
    global ROAD_MODE, LEARNED_MODEL
    sup = Supervisor()
    ts = int(sup.getBasicTimeStep())
    target, rid, road = parse_args(sys.argv[1:])
    ROAD_MODE = road
    if os.environ.get("OQ_LEARNED_SEG") and learned_seg is not None:
        try:
            LEARNED_MODEL = learned_seg.load()
            print("[eye] learned free-space segmenter ENABLED (model_seg.npz)", flush=True)
        except Exception as exc:
            print(f"[eye] learned segmenter unavailable: {exc}", flush=True)
    percept_file = PROJ / (f"_perception_{rid}.json" if rid else "_perception.json")

    self_node = sup.getSelf()
    tracked = sup.getFromDef(target)
    front = sup.getDevice("front_camera")
    sleft = sup.getDevice("stereo_left")
    sright = sup.getDevice("stereo_right")
    stereo = (sleft is not None and sright is not None and _CV2)
    if self_node is None or tracked is None or (front is None and not stereo):
        print(f"[eye] FATAL self={self_node} target({target})={tracked} "
              f"front={front} stereo(L={sleft is not None},R={sright is not None},"
              f"cv2={_CV2})", file=sys.stderr, flush=True)
        return 1
    fw = fh = 0
    fov_f = 0.0
    if front is not None:
        front.enable(ts); fw, fh = front.getWidth(), front.getHeight()
        fov_f = front.getFov()
    matcher = None
    sw = sh = 0
    sfov = 0.0
    if stereo:
        sleft.enable(ts); sright.enable(ts)
        sw, sh = sleft.getWidth(), sleft.getHeight()
        sfov = sleft.getFov()
        matcher = cv2.StereoBM_create(numDisparities=32, blockSize=11)
        matcher.setUniquenessRatio(12)     # reject ambiguous matches
        matcher.setTextureThreshold(12)    # reject low-texture (flat) regions
        matcher.setSpeckleWindowSize(60)   # remove isolated speckle mismatches
        matcher.setSpeckleRange(2)
    left = sup.getDevice("left_camera")
    right = sup.getDevice("right_camera")
    for cam in (left, right):
        if cam is not None:
            cam.enable(ts)
    tr = self_node.getField("translation")
    rot = self_node.getField("rotation")
    print(f"[eye] tracking {target}: "
          f"{('front %dx%d' % (fw, fh)) if front is not None else 'no-cam'}"
          f"{(' +STEREO %dx%d@%.2f' % (sw, sh, sfov)) if stereo else (' (no-stereo cv2=%s)' % _CV2)}"
          f"{' +left' if left else ''}{' +right' if right else ''}"
          f" -> {percept_file.name}", flush=True)

    cap_dir = os.environ.get("OQ_CAPTURE_DIR")   # sim-frame capture for sim-to-real work
    cap_n = 0
    cap_step = 0

    while sup.step(ts) != -1:
        hp = tracked.getPosition()
        ho = tracked.getOrientation()
        yaw = math.atan2(ho[3], ho[0])
        tr.setSFVec3f([hp[0], hp[1], hp[2]])
        rot.setSFRotation([0.0, 0.0, 1.0, yaw])
        t_now = round(sup.getTime(), 3)
        if front is not None:
            img = front.getImage()
            l, c, r = obstacle_sectors(img, fw, fh)
            prof = free_profile(img, fw, fh)
            lf = obstacle_fraction(left.getImage(), left.getWidth(),
                                   left.getHeight()) if left is not None else 0.0
            rf = obstacle_fraction(right.getImage(), right.getWidth(),
                                   right.getHeight()) if right is not None else 0.0
            try:
                percept_file.write_text(
                    json.dumps({"t": t_now, "l": l, "c": c, "r": r,
                                "left": lf, "right": rf,
                                "profile": prof, "cam_fov": round(fov_f, 5)}),
                    encoding="utf-8")
            except Exception:
                pass
        dmask = None
        if stereo:
            dp, dmask = stereo_depth_profile(sleft.getImage(), sright.getImage(),
                                             sw, sh, sfov, matcher)
            if dp is not None:
                try:
                    percept_file.write_text(
                        json.dumps({"t": t_now, "depth": dp,
                                    "cam_fov": round(sfov, 5)}), encoding="utf-8")
                except Exception:
                    pass
        if cap_dir and stereo and cap_n < 150 and sup.getTime() > 3.0:
            cap_step += 1
            if cap_step % 6 == 0:                # sample across the route, not 150 in a row
                try:
                    os.makedirs(cap_dir, exist_ok=True)
                    sleft.saveImage(os.path.join(cap_dir, "sim_%03d.png" % cap_n), 100)
                    if dmask is not None:        # teacher label = stereo obstacle mask
                        cv2.imwrite(os.path.join(cap_dir, "sim_%03d_mask.png" % cap_n),
                                    dmask.astype(np.uint8) * 255)
                    cap_n += 1
                except Exception:
                    pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
