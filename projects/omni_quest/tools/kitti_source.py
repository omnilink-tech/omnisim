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

"""Real sensor source: KITTI raw (rectified stereo + GPS + IMU).

The first REAL-data adapter of the native outdoor stack. Same role the sim eye
plays, but the images, GPS and IMU are recorded from a real car driving outdoors
(KITTI, Karlsruhe) — so we can run our perception + state estimation against real
sensor noise, lighting and geometry instead of a renderer. Stereo depth uses
cv2.StereoSGBM with the dataset's real rectified calibration (focal + baseline).

    python tools/kitti_source.py        # depth on a real frame + the real GPS/IMU
"""

from __future__ import annotations

import glob
import os

import numpy as np

try:
    import cv2
except Exception:
    cv2 = None

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "_kitti")
SEQ = os.path.join(ROOT, "seq", "2011_09_26", "2011_09_26_drive_0001_sync")
CALIB = os.path.join(ROOT, "2011_09_26", "calib_cam_to_cam.txt")

# OXTS row layout (KITTI raw): the real GPS + IMU + orientation channels.
OXTS = ["lat", "lon", "alt", "roll", "pitch", "yaw", "vn", "ve", "vf", "vl", "vu",
        "ax", "ay", "az", "af", "al", "au", "wx", "wy", "wz", "wf", "wl", "wu",
        "pos_acc", "vel_acc", "navstat", "numsats", "posmode", "velmode", "orimode"]


def read_calib(path):
    """-> (fx, baseline_m, cx, cy) from the rectified stereo projection matrices."""
    v = {}
    for line in open(path):
        if ":" in line:
            k, rest = line.split(":", 1)
            try:
                v[k.strip()] = [float(x) for x in rest.split()]
            except ValueError:
                pass
    p2, p3 = v["P_rect_02"], v["P_rect_03"]
    fx, cx, cy = p2[0], p2[2], p2[6]
    baseline = (p2[3] - p3[3]) / fx            # metres between the colour cameras
    return fx, baseline, cx, cy


def read_oxts(path):
    return dict(zip(OXTS, [float(x) for x in open(path).read().split()]))


class KittiSource:
    def __init__(self, seq=SEQ, calib=CALIB):
        self.fx, self.baseline, self.cx, self.cy = read_calib(calib)
        self.left = sorted(glob.glob(os.path.join(seq, "image_02", "data", "*.png")))
        self.seq = seq
        bs = 5
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=128, blockSize=bs,
            P1=8 * bs * bs, P2=32 * bs * bs, uniquenessRatio=10,
            speckleWindowSize=100, speckleRange=2, disp12MaxDiff=1) if cv2 else None

    def __len__(self):
        return len(self.left)

    def frame(self, i):
        lp = self.left[i]
        l = cv2.imread(lp, cv2.IMREAD_GRAYSCALE)
        r = cv2.imread(lp.replace("image_02", "image_03"), cv2.IMREAD_GRAYSCALE)
        op = lp.replace(os.path.join("image_02", "data"),
                        os.path.join("oxts", "data")).replace(".png", ".txt")
        return l, r, read_oxts(op)

    def depth(self, left, right):
        disp = self.sgbm.compute(left, right).astype(np.float32) / 16.0
        depth = np.full(disp.shape, np.inf, np.float32)
        m = disp > 0.5
        depth[m] = self.fx * self.baseline / disp[m]
        return depth


def _demo():
    if cv2 is None:
        print("cv2 unavailable"); return
    src = KittiSource()
    print(f"[kitti] {len(src)} real frames | fx={src.fx:.1f} baseline={src.baseline:.3f} m")
    i = len(src) // 2
    l, r, ox = src.frame(i)
    h, w = l.shape
    depth = src.depth(l, r)
    # sample depth in a road-ahead window (lower-centre of the image)
    band = depth[int(h * 0.55):int(h * 0.85), int(w * 0.40):int(w * 0.60)]
    valid = band[np.isfinite(band) & (band > 0) & (band < 80)]
    print(f"[kitti] frame {i}: image {w}x{h}")
    if valid.size:
        print(f"[kitti] STEREO DEPTH ahead (road window): "
              f"median {np.median(valid):.1f} m, min {valid.min():.1f} m "
              f"({valid.size} px) -> real depth from real images")
    # nearest valid obstacle anywhere in the lower half
    low = depth[h // 2:, :]
    lv = low[np.isfinite(low) & (low > 1) & (low < 80)]
    if lv.size:
        print(f"[kitti] nearest return in lower half: {lv.min():.1f} m")
    print(f"[kitti] REAL GPS: lat={ox['lat']:.6f} lon={ox['lon']:.6f} alt={ox['alt']:.1f} m")
    print(f"[kitti] REAL IMU: accel=({ox['ax']:+.2f},{ox['ay']:+.2f},{ox['az']:+.2f}) m/s^2  "
          f"gyro=({ox['wx']:+.3f},{ox['wy']:+.3f},{ox['wz']:+.3f}) rad/s  "
          f"yaw={ox['yaw']:+.3f} rad  v_fwd={ox['vf']:.1f} m/s  sats={int(ox['numsats'])}")


if __name__ == "__main__":
    _demo()
