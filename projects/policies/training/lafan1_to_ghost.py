#!/usr/bin/env python3
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

"""lafan1_to_ghost.py -- adapt a community-retargeted LAFAN1 G1 clip into an OmniSim ghost `lut`.

This is Phase-0 of the universal-tracker plan: it lets us
reuse the Unitree LAFAN1_Retargeting_Dataset (public mirror: lvhaidong/LAFAN1_Retargeting_Dataset)
directly as Shadowing ghosts, instead of hand-rolling BVH retargeting in seq_ghost_retarget.py.

INPUT  -- a LAFAN1 G1 CSV: 30 FPS, no header, 36 columns per frame, in the repo README order:
    [0:3]   root X Y Z                       (metres)
    [3:7]   root quaternion QX QY QZ QW       (xyzw!)
    [7:19]  12 leg joints  (L hip p/r/y, knee, ank p/r ; R hip p/r/y, knee, ank p/r)
    [19:22] 3 waist joints (waist_yaw, waist_roll, waist_pitch)
    [22:29] 7 left-arm joints  (shoulder p/r/y, elbow, wrist roll/pitch/yaw)
    [29:36] 7 right-arm joints (shoulder p/r/y, elbow, wrist roll/pitch/yaw)
  NB: the LAFAN1 retarget is KINEMATIC-only (no dynamics / actuator limits) -- clips are not
  perfectly executable. That is expected; the tracker (adaptive sampling + tolerance) absorbs it.
  We therefore run our balance/embodiment gates as DIAGNOSTICS here, never as blockers.

OUTPUT -- our ghost `lut` JSON (same schema the g1_walk_recipe trainer + g1_ghost controller read):
    leg_lut [nb x 12]  -- the 12 leg joints, our wb_joints[0:12] order
    arm_lut [nb x 2]   -- [left_shoulder_pitch, right_shoulder_pitch]
    att_lut [nb x 2]   -- base [roll, pitch] from the root quaternion
    root_lut[nb x 4]   -- [x, y, z, yaw]
    wb_lut  [nb x 23]  -- the full 23-DOF whole-body pose, our wb_joints order
    wb_joints[23], nb, freq(Hz=1/cycle_s), cycle_s, vx=0.0, seq=True, arm_A, source

The 23-DOF <- 29-DOF map is a NAME-based subset: 12 legs direct, waist keeps waist_yaw only
(drop waist_roll/pitch -- G1-23 has no waist pitch/roll), arms keep shoulder p/r/y + elbow +
wrist_roll per side (drop wrist pitch/yaw). All 23 of our joints exist in the 29, so nothing is
invented.
"""
import argparse
import json
import math
import os

# --- the LAFAN1 G1 29-DOF configuration order (after the 7 root columns), verbatim from the repo README ---
LAFAN1_JOINTS_29 = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

# --- our OmniSim G1 23-DOF whole-body order (the ghost lut wb_joints; single source = an existing lut) ---
WB_JOINTS_23_DEFAULT = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint",
]


def _quat_xyzw_to_rpy(x, y, z, w):
    """ZYX (yaw-pitch-roll) euler from an xyzw quaternion."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _load_wb_joints(ref_lut):
    if ref_lut and os.path.exists(ref_lut):
        d = json.load(open(ref_lut))
        wb = d.get("wb_joints")
        if wb and len(wb) == 23:
            return list(wb)
    return list(WB_JOINTS_23_DEFAULT)


def adapt(csv_path, wb_joints, fps=30.0, heading_rebase=True, frames=None, arm_A=0.30):
    # --- read the raw CSV (numeric, no header) ---
    rows = []
    with open(csv_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rows.append([float(v) for v in line.split(",")])
    if frames is not None:
        rows = rows[frames[0]:frames[1]]
    nb = len(rows)
    if nb == 0:
        raise ValueError(f"{csv_path}: no frames")
    ncol = len(rows[0])
    if ncol != 36:
        raise ValueError(f"{csv_path}: expected 36 cols (7 root + 29 joints), got {ncol}")

    # --- name -> column index within the 29-joint block ---
    name2idx = {n: 7 + i for i, n in enumerate(LAFAN1_JOINTS_29)}
    missing = [n for n in wb_joints if n not in name2idx]
    if missing:
        raise ValueError(f"our wb_joints not present in LAFAN1 29-DOF: {missing}")
    leg_names = wb_joints[:12]
    l_sh_p, r_sh_p = "left_shoulder_pitch_joint", "right_shoulder_pitch_joint"

    # --- heading rebase: rotate the whole trajectory so frame-0 faces +x (matches our pipeline) ---
    x0, y0, z0 = rows[0][0], rows[0][1], rows[0][2]
    _, _, yaw0 = _quat_xyzw_to_rpy(rows[0][3], rows[0][4], rows[0][5], rows[0][6])
    c, s = math.cos(-yaw0), math.sin(-yaw0)

    leg_lut, arm_lut, att_lut, root_lut, wb_lut = [], [], [], [], []
    for r in rows:
        rx, ry, rz = r[0], r[1], r[2]
        qx, qy, qz, qw = r[3], r[4], r[5], r[6]
        roll, pitch, yaw = _quat_xyzw_to_rpy(qx, qy, qz, qw)
        if heading_rebase:
            dx, dy = rx - x0, ry - y0
            rx, ry = c * dx - s * dy, s * dx + c * dy
            yaw = (yaw - yaw0 + math.pi) % (2 * math.pi) - math.pi
        leg_lut.append([r[name2idx[n]] for n in leg_names])
        arm_lut.append([r[name2idx[l_sh_p]], r[name2idx[r_sh_p]]])
        att_lut.append([roll, pitch])
        root_lut.append([rx, ry, rz, yaw])
        wb_lut.append([r[name2idx[n]] for n in wb_joints])

    cycle_s = nb / fps
    return {
        "nb": nb,
        "freq": 1.0 / cycle_s,
        "cycle_s": cycle_s,
        "vx": 0.0,
        "seq": True,
        "arm_A": arm_A,
        "leg_lut": leg_lut,
        "arm_lut": arm_lut,
        "att_lut": att_lut,
        "root_lut": root_lut,
        "wb_joints": list(wb_joints),
        "wb_lut": wb_lut,
        "source": f"LAFAN1 retarget adapter: {os.path.basename(csv_path)} "
                  f"(lvhaidong/LAFAN1_Retargeting_Dataset, G1 29->23 name-subset, "
                  f"{'heading-rebased' if heading_rebase else 'raw-heading'}, {fps:g} FPS)",
    }


def _summary(lut, csv_path):
    import statistics as st
    nb = lut["nb"]
    zs = [r[2] for r in lut["root_lut"]]
    # forward path length in the rebased frame
    xs = [r[0] for r in lut["root_lut"]]
    ys = [r[1] for r in lut["root_lut"]]
    path = sum(math.dist((xs[i], ys[i]), (xs[i - 1], ys[i - 1])) for i in range(1, nb))
    rolls = [a[0] for a in lut["att_lut"]]
    pitches = [a[1] for a in lut["att_lut"]]
    print(f"  frames nb={nb}  cycle_s={lut['cycle_s']:.3f}s  freq={lut['freq']:.4f}Hz")
    print(f"  root z: min={min(zs):.3f} mean={st.mean(zs):.3f} max={max(zs):.3f} m")
    print(f"  planar path length (rebased): {path:.2f} m")
    print(f"  base roll  [deg]: {math.degrees(min(rolls)):+.1f} .. {math.degrees(max(rolls)):+.1f}")
    print(f"  base pitch [deg]: {math.degrees(min(pitches)):+.1f} .. {math.degrees(max(pitches)):+.1f}")


def main():
    ap = argparse.ArgumentParser(description="Adapt a LAFAN1 G1 CSV into an OmniSim ghost lut.")
    ap.add_argument("csv", help="input LAFAN1 G1 csv (36 cols, 30 FPS)")
    ap.add_argument("out", help="output ghost lut json")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--no-heading-rebase", action="store_true",
                    help="keep the studio heading instead of rotating frame-0 to +x")
    ap.add_argument("--frames", default=None, help="slice A:B (frame indices) before adapting")
    ap.add_argument("--arm-A", type=float, default=0.30)
    ap.add_argument("--wb-ref", default="projects/policies/ghosts/g1/ghost_walkstop_achieved_lut.json",
                    help="an existing lut to read the canonical 23 wb_joints order from")
    args = ap.parse_args()

    wb_joints = _load_wb_joints(args.wb_ref)
    frames = None
    if args.frames:
        a, b = args.frames.split(":")
        frames = (int(a) if a else 0, int(b) if b else None)
    lut = adapt(args.csv, wb_joints, fps=args.fps,
                heading_rebase=not args.no_heading_rebase, frames=frames, arm_A=args.arm_A)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    json.dump(lut, open(args.out, "w"))
    print(f"[lafan1_to_ghost] {args.csv} -> {args.out}")
    _summary(lut, args.csv)


if __name__ == "__main__":
    main()
