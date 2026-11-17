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

"""build_keypoints.py -- add body-frame KEYPOINT positions to a ghost lut via mujoco FK.

The community's motion-tracking recipes (DeepMimic -> GMT -> BeyondMimic) reward matching WHERE THE
BODIES ARE in space (feet/hands/head), not exact joint angles. Keypoint tracking is forgiving -- it
lets the policy deviate its joints to stay balanced while still "looking right" -- which is precisely
what defeats the balance-vs-shape wall that stalls angle-only tracking.

This precomputes, per frame, the reference keypoints in the PELVIS BODY FRAME (relative to the pelvis,
orientation-invariant) by forward-kinematics of the reference joint angles through the canonical G1
mujoco model. Stored as `kp_lut` (nb x n_kp*3) + `kp_bodies` (names) in the lut. The trainer's MTRACK
mode reads kp_lut and rewards the sim's own body-frame keypoints matching it.

Keypoints: feet (ankle_roll), hands (wrist_roll_rubber_hand), head -- the balance + arm-shape anchors.
The model's joint order matches our wb_joints exactly (verified), so wb_lut columns map by name.

Usage:  python build_keypoints.py <lut.json>            # adds kp_lut/kp_bodies (4 keypoints, MTRACK)
        python build_keypoints.py <lut.json> --links     # ALSO adds lp_lut/lp_bodies (10 skeleton links,
                                                         # the WBMATCH v2 link-position component)
        ... [--elbow 0.30]   # elbow hang angle used when the lut has no wb_lut/elbow channel

Luts WITHOUT a wb_lut (e.g. the v3c walk ruler: legs + shoulder-pitch only) get a SYNTHESIZED
whole-body pose per frame: legs from leg_lut, shoulder pitch from arm_lut, elbows at --elbow
(the training corridor's hang reference), every other joint 0. That synthesized pose IS the
reference the trainer holds the champion to, so its FK link positions are the honest ruler.

Link set note: literal "all bodies" is ill-posed for position matching -- hip pitch/roll/yaw
link origins are co-located at the hip (same for shoulder triplets), so nearest-body FK matching
cannot tell them apart and their positions carry no extra pose information. LP_BODIES is the
position-DISTINCT skeleton cover: feet, knees, elbows (forearm origin), hands --
every limb segment is pinned by its endpoints (trunk = the attitude component).
"""
import json
import sys

import numpy as np
import mujoco as mj

MJCF = "projects/robots/unitree/g1/mjcf/g1_23dof_omnisim.mjcf"
KP_BODIES = ["left_ankle_roll_link", "right_ankle_roll_link",
             "left_wrist_roll_rubber_hand", "right_wrist_roll_rubber_hand"]  # feet + hands (un-fused in both models)
LP_BODIES = ["left_ankle_roll_link", "right_ankle_roll_link",   # feet
             "left_knee_link", "right_knee_link",               # shins
             "left_elbow_link", "right_elbow_link",             # forearms
             "left_wrist_roll_rubber_hand", "right_wrist_roll_rubber_hand"]  # hands
# (torso_link/head_link origins are degenerate -- at the pelvis -- in this model: no pose info;
#  trunk orientation is already WBMATCH's attitude component)

# canonical channel orders (same as g1_ghost.py / the trainer's leg map)
LEG12 = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
         "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
         "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
         "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"]
SHOULDER_P = ["left_shoulder_pitch_joint", "right_shoulder_pitch_joint"]
ELBOWS = ["left_elbow_joint", "right_elbow_joint"]


def _wb_of(d, elbow_hang, shroll=0.0, shyaw=0.0):
    """(nb, J) whole-body pose table + joint-name list. Uses wb_lut when present, else
    synthesizes from leg_lut (+ arm_lut shoulder pitch, elbows at the hang angle,
    optional mirrored outward shoulder ROLL -- thigh clearance for straight-arm swings)."""
    nb = d["nb"]
    if "wb_lut" in d:
        return np.asarray(d["wb_lut"], np.float32), list(d["wb_joints"]), False
    legs = np.asarray(d["leg_lut"], np.float32)                     # (nb, 12)
    names = list(LEG12); cols = [legs[:, i] for i in range(12)]
    if "arm_lut" in d:
        arms = np.asarray(d["arm_lut"], np.float32)                 # (nb, 2) L/R shoulder pitch
        names += SHOULDER_P; cols += [arms[:, 0], arms[:, 1]]
    names += ELBOWS
    cols += [np.full(nb, elbow_hang, np.float32), np.full(nb, elbow_hang, np.float32)]
    if shroll != 0.0:   # mirrored: + = arm OUT on the left, - on the right (corridor convention)
        names += ["left_shoulder_roll_joint", "right_shoulder_roll_joint"]
        cols += [np.full(nb, shroll, np.float32), np.full(nb, -shroll, np.float32)]
    if shyaw != 0.0:    # mirrored shoulder YAW (carry poses turn the forearms inward)
        names += ["left_shoulder_yaw_joint", "right_shoulder_yaw_joint"]
        cols += [np.full(nb, shyaw, np.float32), np.full(nb, -shyaw, np.float32)]
    return np.stack(cols, 1), names, True


def main(lut_path, want_links=False, elbow_hang=0.30, shroll=0.0, shyaw=0.0):
    d = json.load(open(lut_path))
    wb, wb_joints, synth = _wb_of(d, elbow_hang, shroll, shyaw)
    if synth:
        print(f"[keypoints] no wb_lut -> synthesized pose (legs + shoulders + elbows@{elbow_hang:.2f}, rest 0)")
    nb = d["nb"]

    m = mj.MjModel.from_xml_path(MJCF)
    data = mj.MjData(m)
    # map each wb_joints[i] -> its qpos address in the model (by name)
    jqadr = {}
    for j in range(m.njnt):
        nm = mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, j)
        if nm in wb_joints:
            jqadr[nm] = int(m.jnt_qposadr[j])
    miss = [n for n in wb_joints if n not in jqadr]
    if miss:
        raise SystemExit(f"joints not in model: {miss}")
    pelvis = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, "pelvis")
    kp_ids = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, b) for b in KP_BODIES]
    lp_ids = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, b) for b in LP_BODIES] if want_links else []
    if pelvis < 0 or any(k < 0 for k in kp_ids) or any(k < 0 for k in lp_ids):
        raise SystemExit("missing pelvis/keypoint/link body")

    kp_lut = np.zeros((nb, len(kp_ids) * 3), np.float32)
    lp_lut = np.zeros((nb, len(lp_ids) * 3), np.float32) if want_links else None
    lo_lut = np.zeros((nb, len(lp_ids) * 4), np.float32) if want_links else None  # link ORIENTATIONS (wxyz quats)
    # HEADING FRAME for lp (v2.1): FK through the ghost's RECORDED pelvis sway (att_lut roll,pitch;
    # yaw=0 -> world frame == heading frame). A body-frame comparison re-projects the 0.7m leg lever
    # by the pelvis attitude: 5 deg of normal walking sway reads as ~6cm of fake foot error,
    # concentrated at stride turnover (measured: the links-profile U-shape). In the heading frame
    # the sway displaces ref and sim links alike, so only TRUE spatial deviation is charged.
    att = np.asarray(d["att_lut"], np.float32) if "att_lut" in d else None
    for f in range(nb):
        data.qpos[:] = 0.0
        data.qpos[0:3] = [0.0, 0.0, 1.0]            # pelvis up; identity orientation (body frame)
        data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        for i, n in enumerate(wb_joints):
            data.qpos[jqadr[n]] = float(wb[f, i])
        mj.mj_kinematics(m, data)                    # FK only (no dynamics)
        p0 = data.xpos[pelvis].copy()
        row = []
        for k in kp_ids:
            row.extend((data.xpos[k] - p0).tolist())  # pelvis-relative (body frame, orientation ~identity)
        kp_lut[f] = row
        if want_links:
            if att is not None:                      # re-FK with the recorded sway for the lp table
                r, pch = float(att[f, 0]), float(att[f, 1])
                cr, sr = np.cos(r / 2), np.sin(r / 2); cp, sp = np.cos(pch / 2), np.sin(pch / 2)
                data.qpos[3:7] = [cp * cr, cp * sr, sp * cr, -sp * sr]   # qy(pitch)*qx(roll), yaw=0
                mj.mj_kinematics(m, data)
                p0 = data.xpos[pelvis].copy()
            lp_lut[f] = [c for k in lp_ids for c in (data.xpos[k] - p0).tolist()]
            lo_lut[f] = [c for k in lp_ids for c in data.xquat[k].tolist()]  # heading frame (FK yaw=0)

    d["kp_lut"] = kp_lut.tolist()
    d["kp_bodies"] = KP_BODIES
    if want_links:
        d["lp_lut"] = lp_lut.tolist()
        d["lp_bodies"] = LP_BODIES
        d["lp_frame"] = "heading"    # yaw-removed gravity-aligned; sim side must compare in the same frame
        d["lo_lut"] = lo_lut.tolist()   # WBMATCH v3 (owner 2026-07-05): link orientations, same bodies/frame
    json.dump(d, open(lut_path, "w"))
    # quick sanity: foot separation + hand height ranges
    kp = kp_lut.reshape(nb, len(kp_ids), 3)
    lf, rf, lh, rh = 0, 1, 2, 3
    print(f"[keypoints] {lut_path}: nb={nb}  {len(kp_ids)} bodies={KP_BODIES}")
    print(f"  foot lateral gap (m): {np.mean(np.abs(kp[:,lf,1]-kp[:,rf,1])):.3f}"
          f"   foot z ~ {np.mean(kp[:,lf,2]):.3f}")
    print(f"  hand height rel pelvis (m): L {np.mean(kp[:,lh,2]):+.3f}  R {np.mean(kp[:,rh,2]):+.3f}"
          f"   swing range L {kp[:,lh,2].max()-kp[:,lh,2].min():.2f}  R {kp[:,rh,2].max()-kp[:,rh,2].min():.2f}")


if __name__ == "__main__":
    _elb = 0.30
    if "--elbow" in sys.argv:
        _elb = float(sys.argv[sys.argv.index("--elbow") + 1])
    _shr = 0.0
    if "--shroll" in sys.argv:
        _shr = float(sys.argv[sys.argv.index("--shroll") + 1])
    _shy = 0.0
    if "--shyaw" in sys.argv:
        _shy = float(sys.argv[sys.argv.index("--shyaw") + 1])
    main(sys.argv[1], want_links="--links" in sys.argv, elbow_hang=_elb, shroll=_shr, shyaw=_shy)
