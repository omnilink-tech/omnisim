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

"""Top-down PERCEPTION camera for the AnyPick line.

Publishes, for every part the Recognition node currently sees (occlusion on,
so only unblocked parts), a WORLD-FRAME pose estimate + a shape class to this
robot's customData:

    "<nodeId>:<kind>:<x>:<y>:<z>:<ax>:<ay>:<az>:<angle>,..."

The supervisor picks WHICH part to grasp, WHAT shape it is and WHERE to put
the pad purely from these estimates -- it does not read part poses from the
scene tree. Position estimates carry +-3 mm uniform noise so the consumer's
contact machinery has to absorb realistic perception error.

Kind comes from the part's recognition colour (nearest reference colour). A
colour that matches NO reference is classified "unknown" -- and for unknown
parts this camera goes MODEL-FREE: it finds a graspable anchor in its DEPTH
image (a RangeFinder co-mounted with the camera): the highest locally-flat
patch inside the detection's neighbourhood. For unknown tokens the x:y:z
fields carry that SURFACE ANCHOR (not a body origin -- there is no model to
have an origin of). The depth convention (ray distance vs planar depth) is
self-calibrated at startup against the known belt height.
"""

import math
import os
import random

from omnisim import Robot

robot = Robot()
dt = int(robot.getBasicTimeStep())
cam = robot.getDevice("bin_camera")
cam.enable(dt)
cam.recognitionEnable(dt)
rf = None
try:
    rf = robot.getDevice("bin_depth")
    rf.enable(dt)
except Exception:
    rf = None

CAM_P = (0.46, 0.0, 1.15)       # camera position (must match the world file)
# Camera node rotation 0 1 0 1.5708: camera +x (view axis) -> world -z,
# +y -> world +y, +z -> world +x.
R_CAM = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]

KIND_COLORS = {                 # reference recognitionColors per shape kind
    "cube": (0.88, 0.18, 0.16),
    "ltube": (0.16, 0.78, 0.30),
    "tube": (0.16, 0.36, 0.88),
    "bar": (0.88, 0.45, 0.16),
    "tbar": (0.10, 0.60, 0.45),
    "gear": (0.62, 0.63, 0.66),
}
KIND_TOL = 0.06                 # max colour distance^2 to accept a known class
NOISE = 0.003                   # +-3 mm uniform position noise
# ANYPICK_SEED varies the perception noise between runs -- the sim is
# otherwise deterministic, so reliability sweeps NEED a varied seed.
rng = random.Random(int(os.environ.get("ANYPICK_SEED", "7")))

CAM_W, CAM_H = 320, 320

# ── Learned classifier (shape from the IMAGE, not the colour) ────────
# When a trained ONNX is present (scripts/dev/train_anypick_classifier.py)
# the kind comes from a small CNN over a 48x48 GRAYSCALE crop -- colour is
# only the dataset-labelling oracle, never the runtime classifier. Low
# confidence -> "unknown" (the model-free black-bin lane). In capture mode
# (ANYPICK_CAPTURE) the colour classifier labels saved crops instead.
CAP_DIR = os.environ.get("ANYPICK_CAPTURE")
_cls = _cls_meta = None
if not CAP_DIR:
    try:
        import json as _json
        import numpy as _np
        import onnxruntime as _ort
        _p = os.path.join(os.environ.get("OMNISIM_HOME", "."),
                          "projects", "rl", "inference", "policies", "anypick_cls")
        if os.path.exists(os.path.join(_p, "cls.onnx")):
            _cls = _ort.InferenceSession(os.path.join(_p, "cls.onnx"),
                                         providers=["CPUExecutionProvider"])
            with open(os.path.join(_p, "classes.json")) as _fh:
                _cls_meta = _json.load(_fh)
            print("[anypick_cam] learned classifier ON: %s (conf>=%.2f)"
                  % (_cls_meta["classes"], _cls_meta["conf_threshold"]), flush=True)
    except Exception as e:
        print("[anypick_cam] classifier unavailable (%s); colour fallback" % e, flush=True)
        _cls = None


def gray_crop48(img, px, py, s):
    """48x48 grayscale crop of the BGRA camera image around pixel (px, py),
    window scaled to the detection size s."""
    half = max(16, int(s * 0.75) + 6)
    out = bytearray(48 * 48)
    for jj in range(48):
        y = min(CAM_H - 1, max(0, py - half + (2 * half * jj) // 48))
        row = 4 * y * CAM_W
        for ii in range(48):
            x = min(CAM_W - 1, max(0, px - half + (2 * half * ii) // 48))
            k = row + 4 * x
            out[jj * 48 + ii] = (img[k] + img[k + 1] + img[k + 2]) // 3
    return out


_cap_counts = {}


def save_crop(kind, crop):
    d = os.path.join(CAP_DIR, kind)
    os.makedirs(d, exist_ok=True)
    n = _cap_counts.get(kind, 0)
    _cap_counts[kind] = n + 1
    with open(os.path.join(d, "%05d.pgm" % n), "wb") as f:
        f.write(b"P5\n48 48\n255\n")
        f.write(bytes(crop))


def classify(obj, img):
    """Runtime kind: learned CNN over the image crop when available
    (low-confidence -> unknown); colour lookup otherwise."""
    if _cls is None or img is None:
        return kind_of(obj)
    import numpy as _np
    px, py = (int(v) for v in obj.getPositionOnImage())
    s = max(obj.getSizeOnImage())
    crop = gray_crop48(img, px, py, s)
    x = _np.frombuffer(bytes(crop), dtype=_np.uint8).astype(_np.float32) / 255.0
    logits = _cls.run(None, {"img": x.reshape(1, 1, 48, 48)})[0][0]
    e = _np.exp(logits - logits.max())
    p = e / e.sum()
    dbg = os.environ.get("ANYPICK_DEBUG_CROPS")
    if dbg and robot.getTime() > 30.0 and _cap_counts.get("_dbg", 0) < 40:
        n = _cap_counts["_dbg"] = _cap_counts.get("_dbg", 0) + 1
        os.makedirs(dbg, exist_ok=True)
        with open(os.path.join(dbg, "%02d_%s_%.2f.pgm"
                               % (n, _cls_meta["classes"][int(p.argmax())], float(p.max()))), "wb") as f:
            f.write(b"P5\n48 48\n255\n")
            f.write(bytes(crop))
    # MULTI-CUE FUSION (shape CNN + colour): "unknown" only when BOTH cues
    # fail -- exactly the never-seen-part signature.
    #  * CNN confident + colour agrees        -> CNN class
    #  * CNN confident + colour matches NOTHING -> unknown (the veto: a
    #    small CNN is overconfident on novel parts, OOD rejection ~21%)
    #  * CNN unsure   + colour known          -> colour class (safety net;
    #    e.g. a rolling tube's motion-blurred crop)
    #  * CNN unsure   + colour unknown        -> unknown
    ck = kind_of(obj)
    if float(p.max()) < _cls_meta["conf_threshold"]:
        return ck                                   # 'unknown' if colour fails too
    if ck == "unknown":
        return "unknown"                            # colour can't confirm -> novel
    k = _cls_meta["classes"][int(p.argmax())]
    if k != ck:
        # The cues DISAGREE: a confident-but-wrong CNN call must not win
        # (one run picked a tube with L-tube geometry off the back of one
        # and cascaded a whole bin). The colour cue arbitrates.
        return ck
    return k


def kind_of(obj):
    """Nearest reference colour, or 'unknown' when nothing matches -- a part
    the system has never seen."""
    try:
        c = obj.getColors()
        best, bd = "unknown", 1e9
        for k, ref in KIND_COLORS.items():
            d = sum((float(c[i]) - ref[i]) ** 2 for i in range(3))
            if d < bd:
                best, bd = k, d
        return best if bd <= KIND_TOL else "unknown"
    except Exception:
        return "unknown"


def _aa_to_R(ax, ay, az, t):
    n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    x, y, z = ax / n, ay / n, az / n
    c, s = math.cos(t), math.sin(t)
    return [[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
            [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
            [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]]


def _R_to_aa(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    t = math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))
    s = 2.0 * math.sin(t)
    if abs(s) < 1e-9:
        return 0.0, 0.0, 1.0, 0.0
    return (R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s, t


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


# ── Model-free depth anchors (for UNKNOWN parts) ─────────────────────
RF_W = RF_H = 160
RF_FOV = 0.6
_f = (RF_W / 2.0) / math.tan(RF_FOV / 2.0)      # focal length, pixels
_depth_is_ray = None                            # self-calibrated below
BELT_TOP = 0.08


def _pixel_world(img, i, j):
    """World (x, y, z) of depth pixel (i=column, j=row), honouring the
    self-calibrated depth convention."""
    d = img[j * RF_W + i]
    if d is None or d != d or d <= 0.0 or d > 5.0:
        return None
    # ray direction in camera frame (view axis +x; +y left ~ world +y when
    # looking down; +z ~ world +x), normalized:
    vy = (RF_W / 2.0 - i) / _f
    vz = (RF_H / 2.0 - j) / _f
    n = math.sqrt(1.0 + vy * vy + vz * vz)
    if _depth_is_ray:
        dx, dy, dz = d / n, d * vy / n, d * vz / n
    else:
        dx, dy, dz = d, d * vy, d * vz
    # camera frame -> world (R_CAM): world = CAM_P + (dz_c, dy_c, -dx_c)
    return (CAM_P[0] + dz, CAM_P[1] + dy, CAM_P[2] - dx)


def _calibrate_depth(img):
    """Decide ray-vs-planar by checking which convention puts an off-centre
    pixel over the (known flat, empty at startup) belt at z=BELT_TOP."""
    global _depth_is_ray
    i, j = RF_W // 2, int(RF_H * 0.78)          # views the out-belt south of centre
    errs = []
    for is_ray in (True, False):
        _set = is_ray
        d = img[j * RF_W + i]
        vy = (RF_W / 2.0 - i) / _f
        vz = (RF_H / 2.0 - j) / _f
        n = math.sqrt(1.0 + vy * vy + vz * vz)
        dx = d / n if is_ray else d
        errs.append(abs((CAM_P[2] - dx) - BELT_TOP))
    _depth_is_ray = errs[0] < errs[1]
    print("[anypick_cam] depth convention: %s (err ray=%.3f planar=%.3f)"
          % ("RAY" if _depth_is_ray else "PLANAR", errs[0], errs[1]), flush=True)


def _world_to_pixel(x, y):
    """Approximate pixel under world (x, y) at belt height (good enough to
    centre a search ROI)."""
    dz_c = x - CAM_P[0]
    dy_c = y - CAM_P[1]
    depth = CAM_P[2] - BELT_TOP
    i = int(RF_W / 2.0 - (dy_c / depth) * _f)
    j = int(RF_H / 2.0 - (dz_c / depth) * _f)
    return max(2, min(RF_W - 3, i)), max(2, min(RF_H - 3, j))


def model_free_anchor(img, wx, wy):
    """MODEL-FREE grasp anchor near world (wx, wy): the highest locally-flat
    depth pixel in a small ROI -- no registry, no CAD, just the depth image.
    Returns (x, y, z) or None."""
    ci, cj = _world_to_pixel(wx, wy)
    roi = 26                                     # ~0.11 m at belt depth
    cands = []
    for j in range(max(2, cj - roi), min(RF_H - 3, cj + roi), 2):
        for i in range(max(2, ci - roi), min(RF_W - 3, ci + roi), 2):
            p = _pixel_world(img, i, j)
            if p is None or p[2] < BELT_TOP + 0.035:
                continue                         # belt / bin floor: not a part top
            cands.append((p[2], i, j, p))
    cands.sort(reverse=True)
    if not cands:
        return None
    ztop = cands[0][0]
    best, best_cover = None, -1
    for z, i, j, p in cands[:60]:
        if ztop - z > 0.006:
            break                                # only the top surface
        zs = []
        ok = True
        for dj in (-2, 0, 2):
            for di in (-2, 0, 2):
                q = _pixel_world(img, i + di, j + dj)
                if q is None:
                    ok = False
                    break
                zs.append(q[2])
            if not ok:
                break
        if not ok or max(zs) - min(zs) >= 0.008:
            continue                             # not locally flat
        # SOLID COVERAGE: count same-height pixels in a pad-sized window.
        # "Any highest flat pixel" used to land on a limb EDGE (or beside
        # the U's hollow), leaving the pad half-overhanging -- visually a
        # force-field grab. The widest solid patch is where a sticky pad
        # actually seals.
        cover = 0
        for dj in range(-4, 5, 2):
            for di in range(-4, 5, 2):
                q = _pixel_world(img, i + di, j + dj)
                if q is not None and abs(q[2] - z) < 0.006:
                    cover += 1
        if cover > best_cover:
            best, best_cover = p, cover
    return best


_tick = 0
while robot.step(dt) != -1:
    _tick += 1
    img = rf.getRangeImage() if rf is not None else None
    if img is not None and _depth_is_ray is None:
        try:
            _calibrate_depth(img)
        except Exception:
            _depth_is_ray = True
    rgb = cam.getImage() if (_cls is not None or CAP_DIR) else None
    toks = []
    try:
        for obj in cam.getRecognitionObjects():
            p = obj.getPosition()                       # camera frame
            o = obj.getOrientation()                    # axis-angle, camera frame
            wx = CAM_P[0] + p[2] + rng.uniform(-NOISE, NOISE)
            wy = CAM_P[1] + p[1] + rng.uniform(-NOISE, NOISE)
            wz = CAM_P[2] - p[0] + rng.uniform(-NOISE, NOISE)
            px, py = (int(v) for v in obj.getPositionOnImage())
            central = 50 <= px <= CAM_W - 50 and 50 <= py <= CAM_H - 50
            if CAP_DIR and rgb is not None and _tick % 8 == 0 and central:
                # dataset capture: label = recognition colour (ground truth);
                # CENTRAL detections only -- border slivers of the queued
                # bins poison the training set.
                s = max(obj.getSizeOnImage())
                save_crop(kind_of(obj), gray_crop48(rgb, px, py, s))
            # Classify CENTRAL detections only (matching the training
            # distribution); border detections are queue parts the
            # supervisor filters out anyway -- skip their tokens.
            if not central:
                continue
            kind = classify(obj, rgb)
            if kind == "unknown":
                # never-seen part: publish a MODEL-FREE depth anchor instead
                # of a body pose (there is no model to pose).
                a = model_free_anchor(img, wx, wy) if img is not None else None
                if a is None:
                    continue
                toks.append("%d:unknown:%.4f:%.4f:%.4f:0:0:1:0"
                            % (obj.getId(), a[0], a[1], a[2]))
                continue
            rw = _mul(R_CAM, _aa_to_R(o[0], o[1], o[2], o[3]))
            ax, ay, az, t = _R_to_aa(rw)
            toks.append("%d:%s:%.4f:%.4f:%.4f:%.4f:%.4f:%.4f:%.4f"
                        % (obj.getId(), kind, wx, wy, wz, ax, ay, az, t))
    except Exception:
        pass
    robot.setCustomData(",".join(toks))
