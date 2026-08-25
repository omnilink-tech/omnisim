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

"""SEQUENCE-GHOST RETARGET v4 -- human mocap (BVH) -> verified whole-body spatial ghost (G1).

The generalized method (owner-approved on the Charleston, 2026-07-04). Pipeline:
  1. Full-skeleton FK of the BVH (the bone structure; also exportable for a side-by-side
     comparison world -- the visual companion of GATE 1).
  2. HEADING REBASE at frame 0 (a hidden -44.8deg studio yaw once caused a start snap).
  3. WHOLE-BODY WEIGHTED IK (GMR-style), one solve/frame over all 23 joints:
     feet w=1.0 (exact) + elbows w=0.45 + hands w=0.40 + chest w=0.20, posture regularization
     on redundant axes, joint limits + step damping inside the solve.
     PER-LIMB SCALING: legs by leg-length ratio (pelvis-relative); ARM targets shoulder-anchored,
     scaled by the robot's OWN segment lengths (uniform scaling is WRONG across morphologies --
     the G1 forearm is ~28% proportionally shorter; leg-scaled arm targets sat beyond reach ->
     perpetual full-stretch arms, owner-caught).
  4. Intro (stand -> routine start) with base height DERIVED from its own legs (feet glued).
  5. GATES: 1 fidelity (feet exact + elbow tracking), 2 grounding. REFUSES to emit on failure.
     GATE 1.5 (dynamic feasibility map, SEQ_EVAL_MAP=1) runs in the trainer; GATE 3 = the owner.

Usage:
  python seq_ghost_retarget.py <in.bvh> <out_lut.json> [--nb 512] [--lift 0.06]
                               [--intro-s 0.93] [--skeleton <out_skeleton.json>]
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys

import numpy as np

_RT = os.environ.get("OMNISIM_HOME", ".")
LEGS12 = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
          "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
          "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
          "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"]
SOLE = 0.035
SKEL_KEYS = ["Hips", "LeftUpLeg", "LeftLeg", "LeftFoot", "RightUpLeg", "RightLeg", "RightFoot",
             "Spine1", "Neck1", "Head", "LeftArm", "LeftForeArm", "LeftHand",
             "RightArm", "RightForeArm", "RightHand"]
SKEL_BONES = [["Hips", "LeftUpLeg"], ["LeftUpLeg", "LeftLeg"], ["LeftLeg", "LeftFoot"],
              ["Hips", "RightUpLeg"], ["RightUpLeg", "RightLeg"], ["RightLeg", "RightFoot"],
              ["Hips", "Spine1"], ["Spine1", "Neck1"], ["Neck1", "Head"],
              ["Spine1", "LeftArm"], ["LeftArm", "LeftForeArm"], ["LeftForeArm", "LeftHand"],
              ["Spine1", "RightArm"], ["RightArm", "RightForeArm"], ["RightForeArm", "RightHand"]]


def _fk_model(src=None):
    import mujoco
    src = src or os.path.join(_RT, "projects/robots/unitree/g1/urdf/g1_23dof_omnisim_prim.urdf")
    if not os.path.isabs(src):
        src = os.path.join(_RT, src)
    s = open(src, encoding="utf-8").read()
    s = re.sub(r"<visual>.*?</visual>", "", s, flags=re.S)
    s = re.sub(r"<collision>.*?</collision>", "", s, flags=re.S)
    tmp = os.path.join(_RT, "_scratch", os.path.basename(src).replace(".urdf", "") + "_fk_only_gen.urdf")
    os.makedirs(os.path.dirname(tmp), exist_ok=True)
    open(tmp, "w", encoding="utf-8").write(s)
    return mujoco, mujoco.MjModel.from_xml_path(tmp)


def parse_bvh(path):
    txt = open(path).read()
    hier, motion = txt.split("MOTION")
    nodes, order, stack, chan_list = {}, [], [], []
    for ln in hier.splitlines():
        t = ln.strip()
        if t.startswith(("ROOT", "JOINT")):
            nm = t.split()[1]
            nodes[nm] = dict(parent=stack[-1] if stack else None, offset=None, chans=[])
            order.append(nm); stack.append(nm)
        elif t.startswith("End"):
            nm = stack[-1] + "_End"
            nodes[nm] = dict(parent=stack[-1], offset=None, chans=[])
            order.append(nm); stack.append(nm)
        elif t.startswith("OFFSET"):
            nodes[stack[-1]]["offset"] = np.array([float(v) for v in t.split()[1:4]])
        elif t.startswith("CHANNELS"):
            cs = t.split()[2:]
            nodes[stack[-1]]["chans"] = cs
            for c in cs:
                chan_list.append((stack[-1], c))
        elif t.startswith("}"):
            stack.pop()
    frames = [l for l in motion.splitlines() if l.strip() and not l.startswith(("Frames", "Frame"))]
    F = np.array([[float(v) for v in l.split()] for l in frames])
    ft = float([l for l in motion.splitlines() if l.strip().startswith("Frame Time")][0].split()[-1])
    return nodes, order, {jc: i for i, jc in enumerate(chan_list)}, F, ft


def bvh_fk(nodes, order, cidx, F, keys):
    def rot(axis, a):
        c, s = np.cos(a), np.sin(a)
        if axis == "X":
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        if axis == "Y":
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    out = {k: np.zeros((len(F), 3)) for k in keys}
    for fi in range(len(F)):
        P, Rw = {}, {}
        for j in order:
            nd = nodes[j]
            Rl = np.eye(3)
            tl = nd["offset"].astype(float).copy()
            for c in nd["chans"]:
                v = F[fi, cidx[(j, c)]]
                if c.endswith("position"):
                    tl["XYZ".index(c[0])] = v
                else:
                    Rl = Rl @ rot(c[0], np.radians(v))
            if nd["parent"] is None:
                Rw[j] = Rl; P[j] = tl
            else:
                Rw[j] = Rw[nd["parent"]] @ Rl
                P[j] = P[nd["parent"]] + Rw[nd["parent"]] @ tl
        for k in keys:
            if k in P:
                out[k][fi] = P[k]
    return out


def _ma(a, w):
    a = np.atleast_2d(a.T).T
    o = np.zeros_like(a); pad = w // 2
    for c in range(a.shape[1]):
        x = np.pad(a[:, c], pad, mode="edge")
        o[:, c] = np.convolve(x, np.ones(w) / w, "same")[pad:-pad]
    return o


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bvh"); ap.add_argument("out")
    ap.add_argument("--nb", type=int, default=512)
    ap.add_argument("--lift", type=float, default=0.06)
    ap.add_argument("--intro-s", type=float, default=0.93)
    ap.add_argument("--skeleton", default=None)
    # GENERALIZATION (owner directive 2026-07-04: any ghost, any motion, any robot): every
    # robot-specific constant -- FK model, body-name map, foot geometry, nominal pose,
    # posture regularization, leg-joint list -- comes from a RETARGET SPEC json. G1 is just
    # the first spec (specs/retarget_g1.json); a new robot = a new spec file, no code edit.
    ap.add_argument("--audit", action="store_true",
                    help="screening mode: report every gate but do NOT refuse to emit -- "
                         "the lut is written regardless so downstream audits (GATE 1d, "
                         "ghost_screen.py ranking) can run on failing candidates too")
    ap.add_argument("--robot-spec",
                    default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "specs", "retarget_g1.json"))
    # STANCE ACHIEVABILITY (salsa campaign, 2026-07-04): human dance footwork lives on a
    # LINE -- salsa 60_03 is laterally <6cm or crossed for 77% of beats, and BOTH feasibility
    # maps (raw + 0.75x tempo) showed the policy falling everywhere except the wide-stance
    # intro; slowing it made it WORSE (longer tightrope holds). Foot-exact retargeting
    # faithfully reproduces a support polygon the robot cannot own. --stance-min enforces a
    # minimum lateral gap between the foot targets (symmetric push in the heading frame,
    # fore-aft travel preserved; crossings become tight step-togethers) BEFORE the IK, so
    # the whole chain stays consistent and every gate re-runs on the altered choreography.
    ap.add_argument("--stance-min", type=float, default=0.0,
                    help="minimum lateral foot-target gap in m (0 = off; G1 walks at ~0.20)")
    ap.add_argument("--balance-min", type=float, default=0.0,
                    help="GATE-1d fix: minimum COM-to-support margin (m); >0 enables the "
                         "balance re-plan (root path shifted so the robot's own COM rides "
                         "its own support polygon, then re-IK). Try 0.02.")
    ap.add_argument("--min-skelmatch", type=float, default=90.0,
                    help="GATE1c floor; lower DELIBERATELY (with a note in the lut source) when "
                         "an achievability alteration like --stance-min moves legs off the "
                         "skeleton by design")
    a = ap.parse_args()
    SPEC = json.load(open(a.robot_spec))
    SB = SPEC["bodies"]
    mujoco, m = _fk_model(SPEC.get("fk_source") or SPEC.get("model"))
    d = mujoco.MjData(m)
    bid = lambda b: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)
    JN = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.njnt)]
    qadr = np.array([m.jnt_qposadr[j] for j in range(m.njnt)])
    LIM = np.stack([m.jnt_range[j] for j in range(m.njnt)])
    mujoco.mj_forward(m, d)
    FT_B = {"LeftFoot": bid(SB["foot_l"]), "RightFoot": bid(SB["foot_r"])}
    SH_B = {"Left": bid(SB["shoulder_l"]), "Right": bid(SB["shoulder_r"])}
    EL_B = {"Left": bid(SB["elbow_l"]), "Right": bid(SB["elbow_r"])}
    HA_B = {"Left": bid(SB["hand_l"]), "Right": bid(SB["hand_r"])}
    TORSO = bid(SB["torso"])
    SOLE = float(SPEC.get("sole", globals()["SOLE"])); LEGS12 = SPEC.get("legs12") or globals()["LEGS12"]
    robot_leg = float(np.linalg.norm(d.xpos[bid(SB["hip_l"])] - d.xpos[FT_B["LeftFoot"]])) + SOLE
    r_up = float(np.linalg.norm(d.xpos[EL_B["Left"]] - d.xpos[SH_B["Left"]]))
    r_fo = float(np.linalg.norm(d.xpos[HA_B["Left"]] - d.xpos[EL_B["Left"]]))

    nodes, order, cidx, F, ft = parse_bvh(a.bvh)
    need = set(SKEL_KEYS) | {"Hips", "LeftFoot", "RightFoot", "LeftArm", "RightArm",
                             "LeftForeArm", "RightForeArm", "LeftHand", "RightHand", "Spine1"}
    keys = [k for k in need if k in nodes]
    H = bvh_fk(nodes, order, cidx, F, keys)
    to_rob = lambda v: np.stack([v[:, 2], v[:, 0], v[:, 1]], 1)
    P = {k: to_rob(H[k]) for k in keys}
    n = len(F)
    yaw = np.radians(F[:, cidx[("Hips", "Yrotation")]])
    yaw0 = float(yaw[0]); yaw = yaw - yaw0
    c0, s0 = np.cos(-yaw0), np.sin(-yaw0)
    for k in keys:
        x = P[k][:, 0].copy()
        P[k][:, 0] = c0 * x - s0 * P[k][:, 1]; P[k][:, 1] = s0 * x + c0 * P[k][:, 1]
    scale = robot_leg / float(np.linalg.norm(P["Hips"] - P["LeftFoot"], axis=1).max())
    h_up = float(np.mean(np.linalg.norm(P["LeftForeArm"] - P["LeftArm"], axis=1)))
    h_fo = float(np.mean(np.linalg.norm(P["LeftHand"] - P["LeftForeArm"], axis=1)))
    s_up, s_fo = r_up / (h_up + 1e-9), r_fo / (h_fo + 1e-9)
    print("scales: leg %.4f | arm per-limb: upper %.4f fore %.4f" % (scale, s_up, s_fo))

    def rel(k):
        r = (P[k] - P["Hips"]) * scale
        cy, sy = np.cos(-yaw), np.sin(-yaw)
        return np.stack([cy * r[:, 0] - sy * r[:, 1], sy * r[:, 0] + cy * r[:, 1], r[:, 2]], 1)

    def relseg(k, anchor, s_):
        r = (P[k] - P[anchor]) * s_
        cy, sy = np.cos(-yaw), np.sin(-yaw)
        return np.stack([cy * r[:, 0] - sy * r[:, 1], sy * r[:, 0] + cy * r[:, 1], r[:, 2]], 1)
    tgt = {k: rel(k) for k in ("LeftFoot", "RightFoot", "Spine1")}
    for k in ("LeftFoot", "RightFoot"):
        tgt[k][:, 2] -= a.lift
    if a.stance_min > 0:
        # rel() targets are heading-derotated: lateral = y. Push feet apart symmetrically
        # to the minimum gap; smooth the correction so contacts don't get lateral steps.
        gap_y = tgt["LeftFoot"][:, 1] - tgt["RightFoot"][:, 1]
        push = np.maximum(0.0, a.stance_min - gap_y) / 2.0
        _pp = np.concatenate([np.full(7, push[0]), push, np.full(7, push[-1])])
        push = np.convolve(_pp, np.ones(15) / 15.0, "valid")[:len(push)]
        tgt["LeftFoot"][:, 1] += push
        tgt["RightFoot"][:, 1] -= push
        print("stance-min %.2f: widened %.0f%% of frames (mean push %.3f, max %.3f)"
              % (a.stance_min, (push > 1e-4).mean() * 100, push.mean(), push.max()))
    armvec = {s: dict(elb=relseg(s + "ForeArm", s + "Arm", s_up),
                      hnd=relseg(s + "Hand", s + "ForeArm", s_fo)) for s in ("Left", "Right")}

    NQ = m.nq
    REG = np.zeros(NQ)
    for j, nm in enumerate(JN):
        if "shoulder_yaw" in nm or "wrist" in nm:
            REG[qadr[j]] = 0.10
        elif "shoulder_roll" in nm:
            REG[qadr[j]] = 0.04
    Q = np.zeros((n, NQ)); q = np.zeros(NQ)
    lq = np.array([m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in LEGS12])
    q[lq] = np.array([-0.10, 0, 0, 0.30, -0.20, 0] * 2)
    SOLEv = np.array([0, 0, SOLE])
    NOMQ = q.copy()

    LHIP = {"L": bid(SB["hip_l"]), "R": bid(SB["hip_r"])}
    LKNE = {"L": bid(SB["knee_l"]), "R": bid(SB["knee_r"])}
    LANK = {"L": FT_B["LeftFoot"], "R": FT_B["RightFoot"]}
    PELV = bid(SB["pelvis"])
    # PER-LIMB scaling for LEGS too (skelmatch lesson: knees went wherever foot-IK sent
    # them -- thigh/shin directions drifted 14-28 deg on bend-heavy motions). Knee target =
    # robot hip + human thigh DIRECTION x robot's own thigh length; same doctrine as arms.
    r_th = float(np.linalg.norm(d.xpos[LKNE["L"]] - d.xpos[LHIP["L"]]))
    h_th = float(np.mean(np.linalg.norm(P["LeftLeg"] - P["LeftUpLeg"], axis=1)))
    s_th = r_th / (h_th + 1e-9)
    thighvec = {"L": relseg("LeftLeg", "LeftUpLeg", s_th),
                "R": relseg("RightLeg", "RightUpLeg", s_th)}
    # CHEST axis = HIP-CENTER -> SHOULDER-CENTER on both bodies. Neither torso_link nor
    # the pelvis BODY frame is anatomical (torso_link's origin sits at the waist joint;
    # the pelvis body frame is offset (0.12,-0.15,0.10) -- chest read 44 deg off vertical
    # in the ZERO pose off it). Virtual midpoints of paired joints are frame-independent.
    _hipc0 = 0.5 * (d.xpos[LHIP["L"]] + d.xpos[LHIP["R"]])
    r_to = float(np.linalg.norm(0.5 * (d.xpos[SH_B["Left"]] + d.xpos[SH_B["Right"]]) - _hipc0))
    SCp = 0.5 * (P["LeftArm"] + P["RightArm"])
    HCp = 0.5 * (P["LeftUpLeg"] + P["RightUpLeg"])
    h_to = float(np.mean(np.linalg.norm(SCp - HCp, axis=1)))
    s_to = r_to / (h_to + 1e-9)
    _rto = (SCp - HCp) * s_to
    _cy, _sy = np.cos(-yaw), np.sin(-yaw)
    chestvec = np.stack([_cy * _rto[:, 0] - _sy * _rto[:, 1],
                         _sy * _rto[:, 0] + _cy * _rto[:, 1], _rto[:, 2]], 1)

    def _ss(p1, p2, q1, q2):
        """segment-segment: distance + closest points."""
        u, v, w0 = p2 - p1, q2 - q1, p1 - q1
        A, Bc, C = float(u @ u), float(u @ v), float(v @ v)
        D0, E = float(u @ w0), float(v @ w0)
        den = A * C - Bc * Bc
        sc = np.clip((Bc * E - C * D0) / den if den > 1e-9 else 0.0, 0, 1)
        tc = np.clip((A * E - Bc * D0) / den if den > 1e-9 else (E / C if C > 1e-9 else 0.0), 0, 1)
        pa, pb = p1 + sc * u, q1 + tc * v
        return float(np.linalg.norm(pa - pb)), pa, pb

    def _leg_pairs():
        """cross-side thigh/shin capsule pairs; A-seg side, segA, segB."""
        th = {s: (d.xpos[LHIP[s]], d.xpos[LKNE[s]]) for s in "LR"}
        sh = {s: (d.xpos[LKNE[s]], d.xpos[LANK[s]]) for s in "LR"}
        return (("L", th["L"], th["R"]), ("L", sh["L"], sh["R"]),
                ("L", th["L"], sh["R"]), ("R", th["R"], sh["L"]))

    def _leg_clear():
        return min(_ss(*A_, *B_)[0] for _, A_, B_ in _leg_pairs())

    def _solve_frame(fi, q, iters):
        for _ in range(iters):
            d.qpos[:] = q; mujoco.mj_forward(m, d)
            Js, es = [], []

            def task(b, pt, w):
                e = pt - d.xpos[b]
                jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv))
                mujoco.mj_jacBody(m, d, jacp, jacr, b)
                Js.append(jacp * w); es.append(e * w)
            def task_mid(b1, b2, pt, w):
                j1 = np.zeros((3, m.nv)); j2 = np.zeros((3, m.nv)); jr_ = np.zeros((3, m.nv))
                mujoco.mj_jacBody(m, d, j1, jr_, b1); mujoco.mj_jacBody(m, d, j2, jr_, b2)
                e = pt - 0.5 * (d.xpos[b1] + d.xpos[b2])
                Js.append(0.5 * (j1 + j2) * w); es.append(e * w)
            for fk_, b in FT_B.items():
                task(b, tgt[fk_][fi] + SOLEv, 1.0)
            _hipc = 0.5 * (d.xpos[LHIP["L"]] + d.xpos[LHIP["R"]])
            task_mid(SH_B["Left"], SH_B["Right"], _hipc + chestvec[fi], 0.35)
            for s_ in ("L", "R"):
                task(LKNE[s_], d.xpos[LHIP[s_]] + thighvec[s_][fi], 0.20)
            _axy = d.xpos[TORSO][:2]
            for s in ("Left", "Right"):
                elb_t = d.xpos[SH_B[s]] + armvec[s]["elb"][fi]
                hnd_t = elb_t + armvec[s]["hnd"][fi]
                # SELF-CLEARANCE (salsa lesson; CALIBRATED vs owner exemplars: charleston8-good
                # p5 clearance 0.108, salsa-bad p5 0.023 -- 0.10 separates them at 2% vs 81%).
                # The gated metric is the forearm CHORD vs the torso axis, so clamp the target
                # PAIR: endpoints out of the keep-out cylinder, then translate both along the
                # chord's outward normal until the chord clears 0.13 -- arm shape is preserved,
                # it just rides off the body (what a dancer's chest does for them for free).
                # PURE-TRANSLATION clearance (skelmatch lesson: per-endpoint clamps moved
                # elbow and hand INDEPENDENTLY, bending the forearm 35 deg on salsa's
                # crossed-arm bars). One shift along the chord's outward normal, sized to
                # cover chord AND endpoint deficits together -- forearm direction is
                # preserved exactly; the arm rides off the body as a unit.
                a2, b2 = elb_t[:2] - _axy, hnd_t[:2] - _axy
                seg2 = b2 - a2; L2 = float(seg2 @ seg2)
                t_ = float(np.clip(-(a2 @ seg2) / L2, 0, 1)) if L2 > 1e-9 else 0.0
                cp = a2 + t_ * seg2; dch = float(np.linalg.norm(cp))
                if max(elb_t[2], hnd_t[2]) > 0.10:
                    need = max(0.13 - dch, 0.13 - float(np.linalg.norm(a2)),
                               0.13 - float(np.linalg.norm(b2)))
                    if need > 0:
                        nrm = cp / dch if dch > 1e-6 else np.array([1.0, 0.0])
                        elb_t[:2] += nrm * need; hnd_t[:2] += nrm * need
                task(EL_B[s], elb_t, 0.45)
                task(HA_B[s], hnd_t, 0.40)
                # ...and repel the SOLVED bodies too (targets clamped != solve landed clear)
                for b_ in (EL_B[s], HA_B[s]):
                    pxy = d.xpos[b_][:2] - _axy
                    r_ = float(np.linalg.norm(pxy))
                    if r_ < 0.13 and d.xpos[b_][2] > 0.10:
                        outward = np.array([pxy[0], pxy[1], 0.0]) / (r_ + 1e-6)
                        task(b_, d.xpos[b_] + outward * (0.13 - r_), 1.2)
            # LEG cross-clearance repulsion (owner: salsa low section = "crossing legs
            # inside each other". CALIBRATED: stand 0.129 / charleston8 min 0.092 /
            # lambada min 0.078 / salsa-bad min 0.000, 10% below 0.05). Keep cross-side
            # thigh/shin capsules apart by pushing the KNEES along the mutual normal --
            # knees flare around each other the way a human's do in crossing steps and
            # deep squats; feet stay exact (weight 1.0 wins the fight at the ankle).
            for side_, A_, B_ in _leg_pairs():
                dist, pa_, pb_ = _ss(*A_, *B_)
                if dist < 0.06:
                    if dist > 1e-6:
                        nrm = (pa_ - pb_) / dist
                    else:
                        nrm = d.xpos[LHIP["L"]] - d.xpos[LHIP["R"]]
                        nrm = nrm / (np.linalg.norm(nrm) + 1e-9)
                        if side_ == "R":
                            nrm = -nrm
                    ba, bb = (LKNE["L"], LKNE["R"]) if side_ == "L" else (LKNE["R"], LKNE["L"])
                    push = (0.06 - dist) * 0.5
                    task(ba, d.xpos[ba] + nrm * push, 1.4)
                    task(bb, d.xpos[bb] - nrm * push, 1.4)
            J = np.vstack(Js); e = np.concatenate(es)
            dq = J.T @ np.linalg.solve(J @ J.T + 3e-3 * np.eye(len(e)), e)
            q = q + np.clip(dq, -0.25, 0.25) - REG * q
            for j in range(m.njnt):
                q[qadr[j]] = np.clip(q[qadr[j]], LIM[j, 0] + 0.02, LIM[j, 1] - 0.02)
        return q

    def _foot_err(fi, q):
        d.qpos[:] = q; mujoco.mj_forward(m, d)
        return max(float(np.linalg.norm((tgt[fk_][fi] + SOLEv) - d.xpos[b])) for fk_, b in FT_B.items())
    def _solve_all():
        qq = NOMQ.copy()
        reseeds = 0
        for fi in range(n):
            qq = _solve_frame(fi, qq, 8 if fi == 0 else 4)
            # SELF-HEALING (salsa lesson): a bad basin at one frame must not poison the
            # warm-start chain -- if a foot misses, re-seed from nominal, bigger budget.
            if _foot_err(fi, qq) > 0.05:
                q2 = _solve_frame(fi, NOMQ.copy(), 16)
                if _foot_err(fi, q2) < _foot_err(fi, qq):
                    qq = q2; reseeds += 1
            Q[fi] = qq
        if reseeds:
            print("solver: %d frames re-seeded out of bad basins" % reseeds)
        # TEMPORAL-CONSENSUS second pass (salsa lesson #2: re-seeded frames land in isolated
        # basins with good feet but mediocre limb DIRECTIONS). Re-solve every frame from its
        # neighbours' smoothed consensus; keep whichever tracks feet better.
        Qs = _ma(Q.copy(), 7)
        fixed = 0
        for fi in range(n):
            q2 = _solve_frame(fi, Qs[fi].copy(), 6)
            if _foot_err(fi, q2) <= _foot_err(fi, Q[fi]) + 0.002:
                if not np.allclose(q2, Q[fi], atol=1e-4):
                    fixed += 1
                Q[fi] = q2
        print("solver: consensus pass refined %d frames" % fixed)

    _solve_all()

    # ── BALANCE RE-PLAN (GATE 1d fix; owner root-cause call 2026-07-04): the retarget
    # transfers KINEMATICS but not DYNAMICS -- the human's weight placement is baked into
    # the mocap and is wrong for the robot's mass model and foot size (salsa: COM outside
    # the support polygon 67% of bins; segment-isolation survival tracked the per-16th COM
    # margin rank-for-rank). Fix stage, general for any motion/robot: measure the robot's
    # OWN COM (mujoco masses) against its OWN support rects, then minimally shift the ROOT
    # PATH so the weight rides the support (a dancer's weight shift, re-solved for this
    # body), smooth it, and re-IK. Limb style is untouched -- arm/chest/knee targets are
    # anchored to the moving body. bal_shift is added to the exported root path.
    bal_shift = np.zeros((n, 2))
    if a.balance_min > 0:
        # v1 (quasi-static pull toward support) under-delivered: 63% of a dance is
        # single-support with the stance foot ALTERNATING at beat rate, so the needed
        # weight shift oscillates and any smoothed pull cancels itself. The correct
        # general re-plan is the robot's own pendulum: LIPM/DCM -- keep the skeleton's
        # FOOTHOLDS and TIMING, place the ZMP on the robot's own stance schedule, solve
        # the PERIODIC COM trajectory its dynamics allow (DCM backward recursion =
        # bounded/convergent solution), and move the root so the robot's COM rides it.
        MTOT = float(m.body_mass.sum())
        hw_xy = (P["Hips"][:, :2] - P["Hips"][0, :2]) * scale
        cy_a, sy_a = np.cos(yaw), np.sin(yaw)

        def _world_xy(v_local, fi):
            return np.array([cy_a[fi] * v_local[0] - sy_a[fi] * v_local[1],
                             sy_a[fi] * v_local[0] + cy_a[fi] * v_local[1]])

        # current COM (world) + COM-vs-root offset, from the solved poses
        com_w = np.zeros((n, 2)); com_z = np.zeros(n)
        for fi in range(n):
            d.qpos[:] = Q[fi]; mujoco.mj_forward(m, d)
            cl = np.sum(m.body_mass[:, None] * d.xipos, 0) / MTOT
            com_w[fi] = _world_xy(cl[:2], fi) + hw_xy[fi]
            com_z[fi] = cl[2]
        z_c = float(np.median(com_z) + 0.72)          # COM height above ground (approx)
        om0 = np.sqrt(9.81 / max(0.3, z_c))

        # ZMP reference from the CONTACT SCHEDULE (stance foot = the lower target):
        # single support -> stance foot center; double support -> mean; airborne -> hold.
        pz = np.zeros((n, 2)); last = None
        for fi in range(n):
            zl, zr = tgt["LeftFoot"][fi, 2], tgt["RightFoot"][fi, 2]
            zmin = min(zl, zr); ct = []
            for fk_, zz in (("LeftFoot", zl), ("RightFoot", zr)):
                if zz < zmin + 0.03:
                    ct.append(_world_xy(tgt[fk_][fi][:2], fi) + hw_xy[fi])
            last = np.mean(ct, 0) if ct else (last if last is not None else hw_xy[fi])
            pz[fi] = last
        _w = max(3, int(0.10 / ft) | 1)
        pz = np.stack([np.convolve(np.concatenate([pz[:_w // 2 + 1][::-1][:_w // 2, c] * 0 + pz[0, c],
                                                   pz[:, c], np.full(_w // 2, pz[-1, c])]),
                       np.ones(_w) / _w, "valid")[:n] for c in range(2)], 1)

        # DCM backward recursion (periodic): xi_k = p_k + e^(-om0*dt) (xi_{k+1} - p_k);
        # then COM forward: x_{k+1} = x_k + dt * om0 * (xi_k - x_k). Loop twice each for
        # the periodic fixed point (backward pass is contracting, forward pass converges).
        e = np.exp(-om0 * ft)
        xi = pz.copy()
        for _rep in range(3):
            for fi in range(n - 1, -1, -1):
                xi[fi] = pz[fi] + e * (xi[(fi + 1) % n] - pz[fi])
        xcom = com_w[0].copy(); X = np.zeros((n, 2))
        for _rep in range(3):
            for fi in range(n):
                X[fi] = xcom
                xcom = xcom + ft * om0 * (xi[fi] - xcom)
        bal_shift = X - com_w
        # keep the loop seam clean + bound the correction
        bal_shift = np.clip(bal_shift, -0.25, 0.25)
        print("balance-replan (LIPM/DCM): z_c=%.2f om0=%.2f root shift mean %.3f max %.3f"
              % (z_c, om0, np.linalg.norm(bal_shift, axis=1).mean(),
                 np.linalg.norm(bal_shift, axis=1).max()))
        # world-fixed feet: pelvis-frame foot targets move opposite the root shift
        for fk_ in ("LeftFoot", "RightFoot"):
            dxl = cy_a * bal_shift[:, 0] + sy_a * bal_shift[:, 1]
            dyl = -sy_a * bal_shift[:, 0] + cy_a * bal_shift[:, 1]
            tgt[fk_][:, 0] -= dxl
            tgt[fk_][:, 1] -= dyl
        _solve_all()

    FL = np.zeros((n, 3)); FR = np.zeros((n, 3)); ELt = np.zeros((n, 3))
    for fi in range(n):
        d.qpos[:] = Q[fi]; mujoco.mj_forward(m, d)
        FL[fi] = d.xpos[FT_B["LeftFoot"]]; FR[fi] = d.xpos[FT_B["RightFoot"]]
        ELt[fi] = d.xpos[EL_B["Left"]] - d.xpos[SH_B["Left"]]
    tl, tr = tgt["LeftFoot"] + SOLEv, tgt["RightFoot"] + SOLEv
    corr = lambda x, y: float(np.corrcoef(x, y)[0, 1])
    g1 = {"foot_rms_L": round(float(np.linalg.norm(FL - tl, axis=1).mean()), 3),
          "foot_rms_R": round(float(np.linalg.norm(FR - tr, axis=1).mean()), 3),
          "footz_corr_L": round(corr(FL[:, 2], tl[:, 2]), 3),
          "footz_corr_R": round(corr(FR[:, 2], tr[:, 2]), 3),
          "elbow_rms": round(float(np.linalg.norm(ELt - armvec["Left"]["elb"], axis=1).mean()), 3)}
    print("GATE1 fidelity:", json.dumps(g1))
    if not (g1["foot_rms_L"] < 0.03 and g1["foot_rms_R"] < 0.03
            and g1["footz_corr_L"] > 0.85 and g1["footz_corr_R"] > 0.85 and g1["elbow_rms"] < 0.12):
        print("GATE1 FAIL -- refusing to emit");
        print("AUDIT: continuing") if a.audit else sys.exit(1)

    # ── GATE 1b: EMBODIMENT REPORT (owner: deterministic bone-closeness, not eye testimony).
    # CALIBRATED vs owner exemplars (2026-07-04): forearm-capsule min distance to the pelvis->chest
    # axis segment. stand=0.148; charleston8 (owner-GOOD) p5=0.108 -> 1.2% below 0.10;
    # salsa pre-fix (owner-BAD "arms into the body") p5=0.023 -> 81% below 0.10.
    # LEG metric added after the salsa low-section finding ("crossing legs inside each
    # other"): min cross-side thigh/shin capsule distance. CALIBRATED: stand 0.129 /
    # charleston8 min 0.092 / lambada min 0.078 / salsa-bad min 0.000 (10% below 0.05)
    # -> threshold 0.05 m.
    clear_min = np.zeros(n); leg_min = np.zeros(n); track = {"hand_L": 0.0, "hand_R": 0.0}
    for fi in range(n):
        d.qpos[:] = Q[fi]; mujoco.mj_forward(m, d)
        tor_lo = np.array([0.0, 0.0, 0.05]); tor_hi = d.xpos[TORSO] + np.array([0, 0, 0.20])
        cm = 9.9
        for s2 in ("Left", "Right"):
            el, ha = d.xpos[EL_B[s2]], d.xpos[HA_B[s2]]
            cm = min(cm, _ss(el, ha, tor_lo, tor_hi)[0])
            elb_t = d.xpos[SH_B[s2]] + armvec[s2]["elb"][fi]
            track["hand_" + s2[0]] += float(np.linalg.norm(
                (elb_t + armvec[s2]["hnd"][fi]) - d.xpos[HA_B[s2]])) / n
        clear_min[fi] = cm
        leg_min[fi] = _leg_clear()
    # Two-tier arm gate (box-motion lesson): the exemplars label <0.065 = interpenetration
    # (salsa-bad p5 0.023) and >=0.097 = clean (charleston8) -- the 0.065..0.10 band is
    # UNLABELED (front reaches like a box pick legitimately sit at ~0.08). Hard-fail only
    # on the interpenetration tier; report the close band for the owner preview to judge.
    g1b = {"self_clear_p5_m": round(float(np.percentile(clear_min, 5)), 3),
           "self_clear_min_m": round(float(clear_min.min()), 3),
           "self_violation_pct": round(float((clear_min < 0.065).mean() * 100), 1),
           "arm_close_pct": round(float(((clear_min >= 0.065) & (clear_min < 0.10)).mean() * 100), 1),
           "leg_clear_p5_m": round(float(np.percentile(leg_min, 5)), 3),
           "leg_clear_min_m": round(float(leg_min.min()), 3),
           "leg_violation_pct": round(float((leg_min < 0.05).mean() * 100), 1),
           "hand_rms_L": round(track["hand_L"], 3), "hand_rms_R": round(track["hand_R"], 3)}
    print("GATE1b embodiment:", json.dumps(g1b))
    if g1b["self_violation_pct"] > 3.0:
        print("GATE1b FAIL (arm-torso self-intersection) -- refusing to emit")
        print("AUDIT: continuing") if a.audit else sys.exit(1)
    if g1b["leg_violation_pct"] > 3.0:
        print("GATE1b FAIL (leg-leg self-intersection) -- refusing to emit")
        print("AUDIT: continuing") if a.audit else sys.exit(1)

    NB_D = a.nb; idx = np.linspace(0, n - 1, NB_D)
    rs = lambda x: np.stack([np.interp(idx, np.arange(n), np.atleast_2d(x.T).T[:, c])
                             for c in range(np.atleast_2d(x.T).T.shape[1])], 1)
    Wb = _ma(rs(Q[:, qadr]), 9)
    hips_w = P["Hips"] * scale
    root = np.stack([hips_w[:, 0] - hips_w[0, 0] + bal_shift[:, 0],
                     hips_w[:, 1] - hips_w[0, 1] + bal_shift[:, 1],
                     hips_w[:, 2] + a.lift, yaw], 1)
    R = _ma(rs(root), 11)
    INTRO = int(round(a.intro_s * NB_D / (n * ft))) if a.intro_s > 0 else 0
    stand = np.zeros(len(JN))
    for j, nm in enumerate(JN):
        for pat, v in SPEC.get("nominal", {"hip_pitch": -0.10, "knee": 0.30,
                                           "ankle_pitch": -0.20, "elbow": 0.5}).items():
            if pat in nm:
                stand[j] = v
    if INTRO > 0:
        w = (1 - np.cos(np.linspace(0, np.pi, INTRO)))[:, None] / 2
        Wall = np.vstack([stand[None, :] * (1 - w) + Wb[0][None, :] * w, Wb])
    else:
        Wall = Wb
    NB = len(Wall)
    feet = list(FT_B.values())
    relz = np.zeros(NB)
    RB_BONES = [(LHIP["L"], LKNE["L"]), (LKNE["L"], LANK["L"]),
                (LHIP["R"], LKNE["R"]), (LKNE["R"], LANK["R"]),
                (SH_B["Left"], EL_B["Left"]), (EL_B["Left"], HA_B["Left"]),
                (SH_B["Right"], EL_B["Right"]), (EL_B["Right"], HA_B["Right"])]
    rdir = np.zeros((NB, len(RB_BONES) + 1, 3))
    for i in range(NB):
        d.qpos[:] = 0; d.qpos[qadr] = Wall[i]; mujoco.mj_forward(m, d)
        relz[i] = min(float(d.xpos[f][2]) for f in feet)
        for bi_, (a_, b_) in enumerate(RB_BONES):
            v_ = d.xpos[b_] - d.xpos[a_]
            rdir[i, bi_] = v_ / (np.linalg.norm(v_) + 1e-9)
        v_ = 0.5 * (d.xpos[SH_B["Left"]] + d.xpos[SH_B["Right"]]) \
            - 0.5 * (d.xpos[LHIP["L"]] + d.xpos[LHIP["R"]])
        rdir[i, len(RB_BONES)] = v_ / (np.linalg.norm(v_) + 1e-9)
    zoff = SOLE - np.percentile(R[:, 2] + relz[INTRO:], 5)
    Rz_dance = R[:, 2] + zoff
    if INTRO > 0:
        Rz_intro = SOLE - relz[:INTRO]
        k = min(16, INTRO)
        Rz_intro[-k:] = np.linspace(Rz_intro[-k], Rz_dance[0], k)
        Rall = np.vstack([np.concatenate([np.full((INTRO, 1), R[0, 0]), np.full((INTRO, 1), R[0, 1]),
                                          Rz_intro[:, None], np.zeros((INTRO, 1))], 1),
                          np.concatenate([R[:, 0:2], Rz_dance[:, None], R[:, 3:4]], 1)])
    else:
        Rall = np.concatenate([R[:, 0:2], Rz_dance[:, None], R[:, 3:4]], 1)
    wmin = Rall[:, 2] + relz
    dur = n * ft + a.intro_s
    hsp = np.sqrt(np.diff(Rall[:-1, 0]) ** 2 + np.diff(Rall[:-1, 1]) ** 2) / (dur / NB)
    g2 = {"float_pct": round(float((wmin > 0.12).mean() * 100), 1),
          "pen_pct": round(float((wmin < -0.02).mean() * 100), 1),
          "vmax": round(float(hsp.max()), 2),
          "path_m": round(float(np.sum(np.sqrt(np.diff(Rall[:, 0]) ** 2 + np.diff(Rall[:, 1]) ** 2))), 2)}
    print("GATE2 grounding:", json.dumps(g2))
    if not (g2["float_pct"] < 8 and g2["pen_pct"] < 8 and g2["vmax"] < 2.0):
        print("GATE2 FAIL -- refusing to emit")
        print("AUDIT: continuing") if a.audit else sys.exit(1)

    # ── GATE 1c: SKELMATCH (owner 2026-07-04: "make them match >90% with a deterministic
    # measurement, not my eye testimony"). Field practice (GMR / H2O) evaluates retargets
    # root-RELATIVE because global position is morphology-confounded; the scale-invariant
    # version of that is BONE DIRECTION agreement: does each robot limb point where the
    # mocap limb points. Score = share of (dance-bin, bone) pairs within 25 deg, over 9
    # bones (thighs, shins, upper arms, forearms, chest), on the same bins/clock both
    # exported files play back at. Time desync or a wrong limb tanks this number.
    SKB = [("LeftUpLeg", "LeftLeg"), ("LeftLeg", "LeftFoot"),
           ("RightUpLeg", "RightLeg"), ("RightLeg", "RightFoot"),
           ("LeftArm", "LeftForeArm"), ("LeftForeArm", "LeftHand"),
           ("RightArm", "RightForeArm"), ("RightForeArm", "RightHand"),
           ("Hips", None)]                 # None = chest: Hips -> shoulder-center
    if all(k in P for pr in SKB for k in pr if k):
        di = np.linspace(0, n - 1, NB_D).astype(int)
        yawd = R[:, 3]
        cy, sy = np.cos(yawd), np.sin(yawd)
        ang = np.zeros((NB_D, len(SKB)))
        for bi_, (ka, kb) in enumerate(SKB):
            hvf = (SCp - HCp) if kb is None else (P[kb] - P[ka])
            hv = hvf[di]
            hv = hv / (np.linalg.norm(hv, axis=1, keepdims=True) + 1e-9)
            rv = rdir[INTRO:, bi_]
            rw = np.stack([cy * rv[:, 0] - sy * rv[:, 1],
                           sy * rv[:, 0] + cy * rv[:, 1], rv[:, 2]], 1)
            ang[:, bi_] = np.degrees(np.arccos(np.clip(np.sum(hv * rw, axis=1), -1, 1)))
        bnames = ["thighL", "shinL", "thighR", "shinR", "uarmL", "farmL", "uarmR", "farmR", "chest"]
        g1c = {"skelmatch_pct": round(float((ang < 25.0).mean() * 100), 1),
               "mean_deg": round(float(ang.mean()), 1),
               "per_bone_deg": {nm: round(float(ang[:, i_].mean()), 1)
                                for i_, nm in enumerate(bnames)}}
        print("GATE1c skelmatch:", json.dumps(g1c))
        if a.min_skelmatch < 90.0:
            print("GATE1c floor LOWERED to %.0f%% (deliberate achievability alteration)" % a.min_skelmatch)
        if g1c["skelmatch_pct"] < a.min_skelmatch:
            print("GATE1c FAIL (<%.0f%% skeleton match) -- refusing to emit" % a.min_skelmatch)
            print("AUDIT: continuing") if a.audit else sys.exit(1)
    out = {"nb": NB, "freq": 1.0 / dur, "cycle_s": dur, "vx": 0.0, "seq": True, "arm_A": 0.35,
           "wb_joints": JN, "wb_lut": [[float(v) for v in r] for r in Wall],
           "leg_lut": [[float(v) for v in r] for r in Wall[:, [JN.index(j) for j in LEGS12]]],
           "att_lut": [[0.0, 0.0] for _ in range(NB)],
           "root_lut": [[float(x) for x in r] for r in Rall],
           "source": "GHOST v4 (%s): whole-body per-limb IK -- GATE1 %s GATE2 %s"
                     % (os.path.basename(a.bvh), json.dumps(g1), json.dumps(g2))}
    json.dump(out, open(a.out, "w"))
    print("wrote", a.out)

    if a.skeleton:
        sk_keys = [k for k in SKEL_KEYS if k in P]
        Ps = np.stack([P[k] for k in sk_keys], 1) * scale
        Ps[:, :, 0] -= hips_w[0, 0]; Ps[:, :, 1] -= hips_w[0, 1]
        allfeet = np.concatenate([P["LeftFoot"][:, 2], P["RightFoot"][:, 2]]) * scale
        Ps[:, :, 2] -= np.percentile(allfeet, 5) - 0.03
        Pd = Ps[np.linspace(0, n - 1, NB_D).astype(int)]
        Pall = np.concatenate([np.repeat(Pd[:1], INTRO, axis=0), Pd], 0) if INTRO else Pd
        bones = [[x, y] for x, y in SKEL_BONES if x in sk_keys and y in sk_keys]
        json.dump({"nb": NB, "freq": 1.0 / dur, "joints": sk_keys, "bones": bones,
                   "pos": [[[round(float(c), 4) for c in j] for j in fr] for fr in Pall]},
                  open(a.skeleton, "w"))
        print("wrote", a.skeleton)


if __name__ == "__main__":
    main()
