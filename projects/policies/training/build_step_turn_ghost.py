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

"""build_step_turn_ghost.py -- CONSTRUCT a statically-stable step-turn ghost by SOLVING for it.

Owner challenge (2026-07-07): "a turn is an equation we can solve -- design how the G1 should step
and move its legs/body to turn. The ghost IS achievable." Correct. This builds the turn ghost by
FOOTSTEP PLANNING + INVERSE KINEMATICS against the G1's own model, keeping the centre of mass over
the support foot the whole time (statically stable => achievable BY CONSTRUCTION), instead of
recording a human (a different body's dynamics, infeasible) or eyeballing joint angles.

The maneuver -- turn-in-place, CCW, in INC increments of THETA each (default 3 x 30deg = 90deg):
  the two feet orbit the body centre; per increment we (1) shift COM over the stance foot,
  (2) lift+rotate the swing foot to the next yaw and plant it, (3) alternate. The base (pelvis)
  yaw follows the mean of the planted feet. Every single-support instant keeps the COM inside the
  stance foot's footprint -- the achievability equation, checked numerically per frame.

⛔⛔ STATIC MARGIN IS NOT THE OBJECTIVE -- MEASURED, 3-arm controlled experiment (2026-07-09).
Three turn ghosts, three turn specialists trained FROM SCRATCH, identical recipe/seed/iters; only the
ghost differs. Deployed in walk-turn-walk (A' and B repeated to confirm):

    arm  COM margin  cycle_s  peak|vby|   turn peak   fell?
    A       5.3 mm    10.4 s    0.287       179 deg    YES
    A'      5.3 mm    13.0 s    0.229        66/68     no  (A' is A, geometry byte-identical, slowed)
    B      30.0 mm    13.0 s    0.287      156/179     YES

At MATCHED DURATION the 30 mm ghost FALLS and the 5.3 mm ghost SURVIVES. Both 0.287 m/s arms fell,
at 5.3 mm AND at 30 mm. The controlling variable is PEAK LATERAL PELVIS VELOCITY, not COM margin --
and because centring the COM costs ~35% more pelvis excursion, "maximise the margin" PAYS FOR ITSELF
IN VELOCITY and makes the reference harder to track. This matches the robust-MPC result that a margin
is a constraint RESTRICTION sized to a disturbance (|R| <= d/2), chosen to MINIMISE the loss of
feasibility region -- never maximised (Smaldone/Scianca/Lanari/Oriolo, ICRA 2020; RAS 2025).
So: use --vby-budget as the PRIMARY knob (measured trackable envelope: <=0.229 m/s from scratch for
this ghost family), and treat --com-frac as a feasibility floor, not a thing to maximise.
Two intermediate explanations died to measurement: it was NOT the warm start (from-scratch A also
falls) and NOT a mis-calibrated budget (ghost_turn_180, which wr_stepturn trained on from scratch,
also peaks at 0.287).

Output: the standard ghost lut (leg_lut, wb_lut, att_lut, root_lut, arm_lut, wb_joints, nb, freq,
cycle_s, seq=True, hold_end=True) that the trainer + g1_ghost controller read.

Usage: python build_step_turn_ghost.py [out.json] [--deg 90] [--inc 3] [--cycle-s 9]
"""
import argparse
import json
import math
import os

import numpy as np
import mujoco as mj

RT = os.environ.get("OMNISIM_HOME", os.getcwd())
MJCF = os.path.join(RT, "projects/robots/unitree/g1/mjcf/g1_23dof_omnisim.mjcf")
WB_JOINTS = [
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
# a neutral, arms-hanging upper body (the turn is legs; arms just hang naturally)
ARM_NOMINAL = {"left_shoulder_pitch_joint": 0.30, "left_elbow_joint": 1.6,
               "right_shoulder_pitch_joint": 0.30, "right_elbow_joint": 1.6}
# CARRY upper body (matches ghost_carry_v1_lut: hands chest-high, box between them). shoulder_pitch
# lifts the arms forward, roll/yaw bring the hands together, elbow slightly bent. Used with --carry
# so a carry-turn specialist holds the box THROUGH the footwork corner (BATON box-delivery demo).
# roll/yaw mirror L/R: sd=+1 left, -1 right; carrier trained SHRY_TARGET=-0.1 SHRY_YAW_TARGET=0.3.
CARRY_ARM = {  # per-joint absolute target for the IK seed + wb_lut arm slots
    "left_shoulder_pitch_joint": -0.45, "left_shoulder_roll_joint": -0.1,
    "left_shoulder_yaw_joint": 0.3, "left_elbow_joint": 0.25, "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": -0.45, "right_shoulder_roll_joint": 0.1,
    "right_shoulder_yaw_joint": -0.3, "right_elbow_joint": 0.25, "right_wrist_roll_joint": 0.0,
}
CARRY_SHPITCH = -0.45   # arm_lut value (shoulder pitch L,R)
CARRY_ELBOW = 0.25      # elbow_lut value (L,R)
CARRY_SHROLL = -0.1     # shroll scalar (matches carrier; SHRY_TARGET at train time)
CARRY_SHYAW = 0.3       # shyaw scalar

FOOT_HALF_X = 0.085   # G1 foot ~0.17 long -> half 0.085
FOOT_HALF_Y = 0.03    # ~0.06 wide -> half 0.03


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_quat(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


class G1IK:
    def __init__(self, carry=False):
        self.m = mj.MjModel.from_xml_path(MJCF)
        self.d = mj.MjData(self.m)
        self.Lf = mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_BODY, "left_ankle_roll_link")
        self.Rf = mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_BODY, "right_ankle_roll_link")
        self.jqadr = {n: self.m.jnt_qposadr[mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_JOINT, n)] for n in WB_JOINTS}
        self.jdadr = {n: self.m.jnt_dofadr[mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_JOINT, n)] for n in WB_JOINTS}
        self.Lleg = WB_JOINTS[0:6]
        self.Rleg = WB_JOINTS[6:12]
        # upper body once: CARRY pose (arms up holding a box) if --carry, else neutral hang. Set
        # BEFORE the COM checks so the raised-arm mass is honestly in the static-stability solve.
        self.arm_pose = CARRY_ARM if carry else ARM_NOMINAL
        for n, v in self.arm_pose.items():
            self.d.qpos[self.jqadr[n]] = v
        # bent-knee warm start (straight legs = a bad IK seed): hip pitch / knee / ankle pitch
        for side in ("left", "right"):
            self.d.qpos[self.jqadr[side + "_hip_pitch_joint"]] = -0.35
            self.d.qpos[self.jqadr[side + "_knee_joint"]] = 0.70
            self.d.qpos[self.jqadr[side + "_ankle_pitch_joint"]] = -0.35

    def set_base(self, pos, yaw):
        self.d.qpos[0:3] = pos
        self.d.qpos[3:7] = yaw_quat(yaw)

    def _foot_err(self, bid, tgt_pos, tgt_yaw):
        """6-vector error [dpos(3), drot(3)] to drive the foot to tgt_pos, flat, yaw=tgt_yaw."""
        cur_pos = self.d.xpos[bid].copy()
        cur_mat = self.d.xmat[bid].reshape(3, 3)
        cur_q = np.zeros(4); mj.mju_mat2Quat(cur_q, cur_mat.reshape(-1))
        tgt_q = yaw_quat(tgt_yaw)
        dq = np.zeros(3)
        # orientation error as a rotation vector (target * current^-1)
        neg = cur_q.copy(); neg[1:] *= -1.0
        qerr = np.zeros(4); mj.mju_mulQuat(qerr, tgt_q, neg)
        mj.mju_quat2Vel(dq, qerr, 1.0)
        return np.concatenate([tgt_pos - cur_pos, dq])

    def ik_legs(self, Lpos, Lyaw, Rpos, Ryaw, iters=400, tol=5e-4):
        """Solve both legs (12 joints) so the feet hit their 6-DOF targets, given the current base.
        Position is weighted 2x over foot-flatness orientation (position is what defines the turn;
        ankle range limits flatness slightly and shouldn't dominate)."""
        Lcols = [self.jdadr[n] for n in self.Lleg]; Rcols = [self.jdadr[n] for n in self.Rleg]
        rng = {n: self.m.jnt_range[mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_JOINT, n)] for n in self.Lleg + self.Rleg}
        w = np.array([2, 2, 2, 1, 1, 1, 2, 2, 2, 1, 1, 1], float)   # pos>ori
        last = 1e9
        for it in range(iters):
            mj.mj_kinematics(self.m, self.d); mj.mj_comPos(self.m, self.d)
            err = np.concatenate([self._foot_err(self.Lf, Lpos, Lyaw), self._foot_err(self.Rf, Rpos, Ryaw)])
            perr = np.linalg.norm(np.concatenate([err[0:3], err[6:9]]))   # position-only norm
            if perr < tol:
                break
            jpL = np.zeros((3, self.m.nv)); jrL = np.zeros((3, self.m.nv))
            jpR = np.zeros((3, self.m.nv)); jrR = np.zeros((3, self.m.nv))
            mj.mj_jacBody(self.m, self.d, jpL, jrL, self.Lf)
            mj.mj_jacBody(self.m, self.d, jpR, jrR, self.Rf)
            J = np.zeros((12, 12))
            J[0:3, 0:6] = jpL[:, Lcols]; J[3:6, 0:6] = jrL[:, Lcols]
            J[6:9, 6:12] = jpR[:, Rcols]; J[9:12, 6:12] = jrR[:, Rcols]
            We = w * err
            JW = J * w[:, None]
            lam = 0.03
            dqv = JW.T @ np.linalg.solve(JW @ JW.T + lam * lam * np.eye(12), We)
            step = 1.0
            for i, n in enumerate(self.Lleg + self.Rleg):
                a = self.jqadr[n]
                self.d.qpos[a] = float(np.clip(self.d.qpos[a] + step * dqv[i], rng[n][0], rng[n][1]))
            last = perr
        mj.mj_kinematics(self.m, self.d); mj.mj_comPos(self.m, self.d)
        err = np.concatenate([self._foot_err(self.Lf, Lpos, Lyaw), self._foot_err(self.Rf, Rpos, Ryaw)])
        return np.linalg.norm(np.concatenate([err[0:3], err[6:9]]))   # report POSITION error

    def leg_angles(self):
        return [float(self.d.qpos[self.jqadr[n]]) for n in WB_JOINTS[0:12]]

    def wb_angles(self):
        return [float(self.d.qpos[self.jqadr[n]]) for n in WB_JOINTS]

    def com(self):
        mj.mj_kinematics(self.m, self.d); mj.mj_comPos(self.m, self.d)
        return self.d.subtree_com[0].copy()   # whole-robot COM (world)

    def solve_base_for_com(self, com_tgt_xy, base_xy0, pelvis_z, base_yaw,
                           Lpos, Lyaw, Rpos, Ryaw, iters=6, tol=2e-4):
        """SOLVE the pelvis xy that puts the WHOLE-BODY COM on com_tgt_xy (Newton, numeric 2x2 Jacobian).

        The heuristic path sets the PELVIS at shift*stance_foot and then merely *measures* where the COM
        landed -- but COM != pelvis (the swing leg swings out, the arms hang, the torso shifts), so the
        COM ends up wherever, and --shift/--lat get hand-cranked until the static check barely passes
        (measured on ghost_turn_90: 5.3 mm of lateral margin inside a 30 mm half-width). Inverting the
        relationship instead -- pick the pelvis so the COM hits its target -- centres the COM and buys
        the full half-width of margin. dCOM/dbase is NOT identity: the feet are IK-pinned, so moving the
        pelvis drags only the free mass. Hence the numeric Jacobian.
        """
        b = np.asarray(base_xy0, float).copy()
        e = 0.0
        for _ in range(iters):
            self.set_base([b[0], b[1], pelvis_z], base_yaw)
            e = self.ik_legs(Lpos, Lyaw, Rpos, Ryaw)
            c = self.com()[:2]
            err = np.asarray(com_tgt_xy, float) - c
            if np.linalg.norm(err) < tol:
                break
            q0 = self.d.qpos.copy()          # keep the warm start clean across probes
            J = np.zeros((2, 2)); h = 1e-3
            for k in range(2):
                bb = b.copy(); bb[k] += h
                self.set_base([bb[0], bb[1], pelvis_z], base_yaw)
                self.ik_legs(Lpos, Lyaw, Rpos, Ryaw, iters=60)
                J[:, k] = (self.com()[:2] - c) / h
                self.d.qpos[:] = q0
            try:
                db = np.linalg.solve(J, err)
            except np.linalg.LinAlgError:
                db = err
            b += np.clip(db, -0.06, 0.06)
        self.set_base([b[0], b[1], pelvis_z], base_yaw)
        e = self.ik_legs(Lpos, Lyaw, Rpos, Ryaw)
        return b, e


def foot_world(center, yaw, side, lat):
    """foot world (x,y) for the given facing yaw; side=+1 left, -1 right (perp to facing)."""
    left_dir = np.array([-math.sin(yaw), math.cos(yaw)])
    return np.array([center[0], center[1]]) + side * lat * left_dir


def com_in_foot(com_xy, foot_xy, foot_yaw, mx=FOOT_HALF_X, my=FOOT_HALF_Y):
    """is the COM ground-projection inside the foot footprint (foot frame), with margin?"""
    dx, dy = com_xy[0] - foot_xy[0], com_xy[1] - foot_xy[1]
    c, s = math.cos(-foot_yaw), math.sin(-foot_yaw)
    fx, fy = c * dx - s * dy, s * dx + c * dy      # into foot frame
    return abs(fx) <= mx and abs(fy) <= my, (fx, fy)


def peak_lateral_vel(root_lut, cycle_s, nb):
    """Peak |body-frame lateral pelvis velocity| -- the quantity the real robot actually has to track."""
    rl = np.asarray(root_lut, float)
    dtb = cycle_s / nb
    vw = (np.roll(rl, -1, 0) - rl) / dtb
    vw[-1] = vw[-2]
    cy, sy = np.cos(rl[:, 3]), np.sin(rl[:, 3])
    return float(np.abs(-sy * vw[:, 0] + cy * vw[:, 1]).max())


def build(deg, inc, cycle_s, lat, pelvis_z, lift, shift, frames_per_phase, waist_lead=0.0, carry=False,
          solve_com=False, com_frac=1.0, com_ramp="linear"):
    ik = G1IK(carry=carry)
    theta = math.radians(deg) / inc
    center = np.array([0.0, 0.0])
    # keyframe schedule: list of (base_xy_target_weight, base_yaw, Lyaw, Ryaw, swing) building the turn
    # swing in {None,'L','R'}: which foot lifts this phase. Foot planted yaw held until it swings.
    kfs = []
    Lyaw = Ryaw = 0.0
    kfs.append(("center", 0.0, Lyaw, Ryaw, None))           # settle, DS
    for k in range(inc):
        # increment k: swing RIGHT up by theta, then LEFT up by theta
        kfs.append(("L", (Lyaw + Ryaw) / 2, Lyaw, Ryaw, None))     # shift onto L (stance)
        Ryaw2 = Ryaw + theta
        kfs.append(("L", (Lyaw + Ryaw2) / 2, Lyaw, Ryaw2, "R"))    # swing R to +theta
        Ryaw = Ryaw2
        kfs.append(("R", (Lyaw + Ryaw) / 2, Lyaw, Ryaw, None))     # shift onto R (stance)
        Lyaw2 = Lyaw + theta
        kfs.append(("R", (Lyaw2 + Ryaw) / 2, Lyaw2, Ryaw, "L"))    # swing L to +theta
        Lyaw = Lyaw2
    kfs.append(("center", (Lyaw + Ryaw) / 2, Lyaw, Ryaw, None))    # settle, DS

    leg_lut, wb_lut, att_lut, root_lut = [], [], [], []
    worst_ikerr = 0.0
    com_reports = []          # (frame, single_support, inside, margin)
    # planted-foot world positions (updated when a foot finishes a swing)
    Lw = foot_world(center, 0.0, +1, lat); Rw = foot_world(center, 0.0, -1, lat)
    Lyaw_w = Ryaw_w = 0.0

    for ki in range(len(kfs) - 1):
        w0, by0, Ly0, Ry0, sw0 = kfs[ki]
        w1, by1, Ly1, Ry1, sw1 = kfs[ki + 1]
        for f in range(frames_per_phase):
            u = (f + 1) / frames_per_phase
            base_yaw = wrap(by0 + wrap(by1 - by0) * u)
            swing = sw1
            # planted-foot targets stay at their last planted world pose; swing foot interpolates
            if swing == "R":
                Lt_pos = np.array([Lw[0], Lw[1], 0.0]); Lt_yaw = Lyaw_w
                r_from = foot_world(center, Ry0, -1, lat); r_to = foot_world(center, Ry1, -1, lat)
                Rt_xy = r_from + (r_to - r_from) * u
                zlift = lift * math.sin(math.pi * u)
                Rt_pos = np.array([Rt_xy[0], Rt_xy[1], zlift]); Rt_yaw = wrap(Ry0 + wrap(Ry1 - Ry0) * u)
                stance_xy, stance_yaw, single = Lw, Lyaw_w, True
            elif swing == "L":
                Rt_pos = np.array([Rw[0], Rw[1], 0.0]); Rt_yaw = Ryaw_w
                l_from = foot_world(center, Ly0, +1, lat); l_to = foot_world(center, Ly1, +1, lat)
                Lt_xy = l_from + (l_to - l_from) * u
                zlift = lift * math.sin(math.pi * u)
                Lt_pos = np.array([Lt_xy[0], Lt_xy[1], zlift]); Lt_yaw = wrap(Ly0 + wrap(Ly1 - Ly0) * u)
                stance_xy, stance_yaw, single = Rw, Ryaw_w, True
            else:
                Lt_pos = np.array([Lw[0], Lw[1], 0.0]); Lt_yaw = Lyaw_w
                Rt_pos = np.array([Rw[0], Rw[1], 0.0]); Rt_yaw = Ryaw_w
                single = False; stance_xy = (Lw + Rw) / 2; stance_yaw = base_yaw
            # base xy: shift toward the stance foot, RAMPED smoothly from the previous keyframe's
            # target to this one over the phase (a discrete per-phase jump spiked the lateral
            # velocity command to ~1.5 m/s -> the robot swayed instead of committing to the turn).
            mid = (Lw + Rw) / 2

            def _wtgt(w):
                return {"center": mid, "L": (1 - shift) * mid + shift * Lw, "R": (1 - shift) * mid + shift * Rw}[w]
            base_xy = _wtgt(w0) + (_wtgt(w1) - _wtgt(w0)) * u
            if solve_com:
                # SOLVE, don't guess: drive the whole-body COM to the stance foot's CENTRE (max margin),
                # and let the solver decide where the pelvis must go. Smoothstep the COM reference so it
                # leaves/arrives at each keyframe with zero lateral velocity -- a linear ramp puts a
                # curvature kink at every phase boundary, which shows up as a COM-acceleration (and ZMP)
                # spike. Double-support keyframes target the foot midpoint.
                # com_frac trades STATIC margin against DYNAMIC aggressiveness, and the trade is real:
                # driving the COM all the way onto the stance foot (com_frac=1) forces the PELVIS to swing
                # ~35% further out (the swing leg's mass pulls the COM the other way), which -- over the same
                # cycle time -- nearly DOUBLES the peak lateral pelvis velocity (0.287 -> 0.537 m/s measured).
                # The ghost stays self-consistent (ZMP fine) but the real robot cannot track that swing: it
                # oscillated and fell. So maximise margin SUBJECT TO the proven velocity envelope, don't just
                # maximise margin. Sweep com_frac and pick the knee.
                def _ctgt(w):
                    return {"center": mid,
                            "L": mid + com_frac * (Lw - mid),
                            "R": mid + com_frac * (Rw - mid)}[w]
                # ⛔ ramp shape is NOT free: a smoothstep leaves/arrives with zero velocity (C1, no
                # acceleration kink) but its PEAK velocity is 1.5x a linear ramp over the same displacement
                # and time. Measured: smoothstep + centred COM = 0.537 m/s peak lateral, vs 0.287 the robot
                # has actually tracked -- it oscillated and fell. Default to the proven linear ramp and buy
                # smoothness back through TIMING (--vby-budget) instead of through the shape.
                us = (u * u * (3.0 - 2.0 * u)) if com_ramp == "smoothstep" else u
                com_tgt = _ctgt(w0) + (_ctgt(w1) - _ctgt(w0)) * us
                base_xy, e = ik.solve_base_for_com(com_tgt, base_xy, pelvis_z, base_yaw,
                                                   Lt_pos, Lt_yaw, Rt_pos, Rt_yaw)
            else:
                ik.set_base([base_xy[0], base_xy[1], pelvis_z], base_yaw)
                e = ik.ik_legs(Lt_pos, Lt_yaw, Rt_pos, Rt_yaw)
            worst_ikerr = max(worst_ikerr, e)
            com = ik.com()
            inside, marg = com_in_foot(com[:2], stance_xy, stance_yaw) if single else (True, (0, 0))
            com_reports.append((len(leg_lut), single, inside, marg))
            _wb = ik.wb_angles()
            # TORSO LEAD (breaks the CW/CCW symmetry at the reference so a moderate direction reward
            # pins the turn without costing magnitude): the waist_yaw (wb slot 12) leads the base
            # into the turn, peaking mid-turn and re-aligning at the end -- how people initiate a turn.
            if waist_lead != 0.0:
                _prog = (base_yaw - kfs[0][1]) / (math.radians(deg) + 1e-9)   # 0..1 over the turn
                _wb[12] = float(waist_lead) * math.sin(math.pi * max(0.0, min(1.0, _prog)))
            leg_lut.append(ik.leg_angles()); wb_lut.append(_wb)
            att_lut.append([0.0, 0.0]); root_lut.append([float(base_xy[0]), float(base_xy[1]), pelvis_z, base_yaw])
            if swing == "R" and u >= 1.0 - 1e-9:
                Rw = foot_world(center, Ry1, -1, lat); Ryaw_w = Ry1
            if swing == "L" and u >= 1.0 - 1e-9:
                Lw = foot_world(center, Ly1, +1, lat); Lyaw_w = Ly1

    nb = len(leg_lut)
    _shp = CARRY_SHPITCH if carry else ARM_NOMINAL["left_shoulder_pitch_joint"]
    lut = {"nb": nb, "freq": 1.0 / cycle_s, "cycle_s": cycle_s, "vx": 0.0, "seq": True,
           "hold_end": True, "arm_A": 0.0, "leg_lut": leg_lut, "wb_lut": wb_lut,
           "att_lut": att_lut, "root_lut": root_lut,
           "arm_lut": [[_shp, _shp]] * nb,
           "wb_joints": WB_JOINTS,
           "source": "build_step_turn_ghost.py: footstep-planned + IK, COM-over-support (achievable by construction)"}
    if carry:   # carry arms: bent elbows + hands-together shoulders (matches ghost_carry_v1_lut; SHRY at train)
        lut["elbow_lut"] = [[CARRY_ELBOW, CARRY_ELBOW]] * nb
        lut["shroll"] = CARRY_SHROLL; lut["shyaw"] = CARRY_SHYAW
        lut["carry"] = True
        lut["source"] += " +CARRY arms (box held through the turn)"
    return lut, worst_ikerr, com_reports


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?", default="projects/policies/controllers/g1_ghost/ghost_step_turn_lut.json")
    ap.add_argument("--deg", type=float, default=90.0)
    ap.add_argument("--inc", type=int, default=3)
    ap.add_argument("--cycle-s", type=float, default=9.0)
    ap.add_argument("--lat", type=float, default=0.12)
    ap.add_argument("--pelvis-z", type=float, default=0.62)
    ap.add_argument("--lift", type=float, default=0.05)
    ap.add_argument("--shift", type=float, default=0.85)
    ap.add_argument("--fpp", type=int, default=9)
    ap.add_argument("--waist-lead", type=float, default=0.0, help="torso (waist_yaw) lead into the turn, rad at peak (breaks CW/CCW symmetry)")
    ap.add_argument("--carry", action="store_true", help="carry arms (box held): arms up + bent elbows, hands together (matches ghost_carry_v1_lut)")
    ap.add_argument("--solve-com", action="store_true",
                    help="SOLVE the pelvis so the whole-body COM sits on the stance-foot centre (max static margin), "
                         "instead of placing the pelvis by the --shift heuristic and hoping the COM lands inside. "
                         "Also smoothsteps the COM reference. Off by default so existing ghosts stay reproducible.")
    ap.add_argument("--com-frac", type=float, default=1.0,
                    help="with --solve-com: how far to drive the COM from the foot midpoint toward the stance-foot "
                         "centre. 1.0 = dead centre (max static margin). Below ~0.75 the COM leaves the foot entirely "
                         "(statically unstable), so this is not the knob to soften the motion with -- use --vby-budget. "
                         "⛔ MEASURED: maximising this buys NOTHING. See the A/A'/B note in the module docstring.")
    ap.add_argument("--com-ramp", choices=("linear", "smoothstep"), default="linear",
                    help="COM reference shape between keyframes. smoothstep is C1 but its PEAK velocity is 1.5x "
                         "linear for the same move; the robot tracks the linear ramp. Default linear.")
    ap.add_argument("--vby-budget", type=float, default=0.0,
                    help="if >0, rescale cycle_s UNIFORMLY so the peak lateral pelvis velocity equals this budget. "
                         "⚠️ SUPERSEDED by ghost_topp.py, which stretches only the segments that actually violate the "
                         "cap instead of the whole turn: for the 90deg ghost at cap 0.229 this gives 11.19s vs the "
                         "13.04s a uniform rescale costs (70%% of the penalty recovered, same cap, same geometry, and "
                         "the trained policy is indistinguishable). Kept for reproducing older ghosts.")
    a = ap.parse_args()
    lut, ikerr, com = build(a.deg, a.inc, a.cycle_s, a.lat, a.pelvis_z, a.lift, a.shift, a.fpp, a.waist_lead, a.carry,
                            a.solve_com, a.com_frac, a.com_ramp)
    if a.vby_budget > 0:
        vpk = peak_lateral_vel(lut["root_lut"], lut["cycle_s"], lut["nb"])
        scale = vpk / a.vby_budget
        if scale > 1.0:
            lut["cycle_s"] = float(lut["cycle_s"] * scale)
            lut["freq"] = 1.0 / lut["cycle_s"]
            print("  TIMING SOLVE: |vby|peak %.3f > budget %.3f -> cycle_s %.2f -> %.2f s (peak now %.3f m/s)"
                  % (vpk, a.vby_budget, a.cycle_s, lut["cycle_s"],
                     peak_lateral_vel(lut["root_lut"], lut["cycle_s"], lut["nb"])))
        else:
            print("  TIMING SOLVE: |vby|peak %.3f already within budget %.3f -- cycle_s unchanged" % (vpk, a.vby_budget))
    json.dump(lut, open(a.out, "w"))
    ss = [c for c in com if c[1]]
    bad = [c for c in ss if not c[2]]
    print("STEP-TURN GHOST built: %s" % a.out)
    print("  nb=%d  cycle_s=%.1f  deg=%.0f  inc=%d" % (lut["nb"], a.cycle_s, a.deg, a.inc))
    print("  worst IK foot error = %.4f m  (want < ~0.01)" % ikerr)
    print("  single-support frames = %d ; COM-outside-stance = %d  %s"
          % (len(ss), len(bad), "OK -- statically stable" if not bad else "FAIL -- raise --shift / --lat"))
    if bad[:5]:
        for c in bad[:5]:
            print("    frame %d margin(fx,fy)=(%.3f,%.3f)  foot half=(%.3f,%.3f)"
                  % (c[0], c[3][0], c[3][1], FOOT_HALF_X, FOOT_HALF_Y))
    # worst-case COM margin over single support
    if ss:
        wx = max(abs(c[3][0]) for c in ss); wy = max(abs(c[3][1]) for c in ss)
        print("  worst COM excursion in stance foot: fx=%.3f (half %.3f), fy=%.3f (half %.3f)"
              % (wx, FOOT_HALF_X, wy, FOOT_HALF_Y))


if __name__ == "__main__":
    main()
