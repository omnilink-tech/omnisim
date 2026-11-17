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

"""GHOST VALIDATOR -- pre-training achievability screening for ghost reference gaits.

Owner directive (2026-07-03): formalize ghost design so bad references are rejected in SECONDS,
not discovered after GPU-hours. Every check below is anchored to a measured outcome from the
2026-07-03 campaign (4 training collapses + 2 successes on the G1); the validator is CALIBRATED:
it must PASS the ghosts that trained (official, v3c) and FAIL/WARN the ones that collapsed
(v3e/v3f/v3g stance edits) or were scratched by the owner (v4 asymmetry).

Usage:
    python projects/policies/training/ghost_validator.py <ghost_lut.json> [--baseline <recorded.json>]

Checks (tiered by cost, all sub-second):
  T0  kinematic legality   joint position limits (vs the robot model), velocity/accel at cycle
                           rate, phase-wrap continuity, harmonic jitter, L/R symmetry
  T1  coupling laws        stance-edit envelope vs baseline (THE dominant rule: parametric edits
                           to a recorded gait beyond ~12% collapsed training EVERY time, with or
                           without compensations), elbow-extension x arm-amplitude budget
  T2  provenance           recorded/achieved >> smoothed refold >> parametric edit >> hand-made

Verdicts: PASS / WARN (train with eyes open) / FAIL (do not train; redesign or re-record).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

_RT = os.environ.get("OMNISIM_HOME", ".")
if _RT not in sys.path:
    sys.path.insert(0, _RT)

# slot order shared by every ghost lut: L[hipP,hipR,hipY,knee,ankP,ankR] then R[...]
JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]
SHOULDERS = ["left_shoulder_pitch_joint", "right_shoulder_pitch_joint"]


# ── the robot registry ───────────────────────────────────────────────────────
# T0 is the only MODEL-AWARE gate: it checks the ghost's joint samples against the
# REAL robot's URDF limits. It must therefore load THE ROBOT THE LUT IS FOR.
#
# ⛔ It did not, until 2026-07-12. `_limits_from_model()` loaded the G1's URDF
# unconditionally, and unknown joints fell back to a no-limit default
# `(-99, 99, 99)` -- so handing it an H1 ghost checked H1's shared-name joints
# against the WRONG ROBOT'S limits and skipped its ankle joints entirely, then
# printed `VERDICT: PASS`. A vacuous PASS on the one gate the whole method rests
# on. Every future robot would have inherited the same false pass.
#
# ⛔⛔ AND THAT FIX WAS HALF OF IT (closed 2026-07-13). It closed WRONG-MODEL but not
# NO-MODEL: an unregistered robot returned {} limits, and every model-aware gate sat
# behind `if lim:` -- so it SKIPPED them all and still printed VERDICT: PASS. The Go2
# shadow ghost -- the artifact the quadruped `method: shadowing` claim rests on -- was
# passing exactly that way. The rule now, in one line:
#
#     A GHOST THAT CANNOT BE MODEL-CHECKED IS A FAILED GHOST, NEVER A PASSED ONE.
#
# The robot -> model map is no longer local to this file; it is the shared registry in
# projects/policies/common/robot_registry.py (the same law the trainer's corridor roles
# are derived from), so the validator and the trainer cannot drift apart.
#
# A lut declares its robot with a top-level "robot" key (the G1 luts predate the
# convention and default to g1).
from projects.policies.common import robot_registry as RR  # noqa: E402

# T3 (corridor adequacy) config. Module-level defaults so `validate()` is importable; the CLI
# overrides them from --corridor / --kp. CORRIDOR=None -> T3 reports the requirement but cannot
# grade a training config it was not told about.
CORRIDOR = None
KP = 200.0

# The hand/thigh clearance gate FKs G1-specific body names (rubber_hand, elbow_link...)
# and is meaningless on a robot without them. Gate it on the robot, do not fake it.
_CLEARANCE_ROBOTS = {"g1"}


def _robot_of(d):
    """Which robot is this ghost FOR? Declared by the lut; G1 luts predate the key."""
    return str(d.get("robot") or "g1").strip().lower()


def _hand_thigh_clearance(rep, d, leg, arm, nb):
    """T0.hand-thigh-clearance -- FK the full designed pose per bin and measure the minimum
    hand/forearm distance to each thigh axis, at the design pose AND at the worst-case corridor
    pose (shoulder roll pulled the standard SHRY residual 0.10 rad INWARD -- the achieved motion
    is allowed to sit there, and it did). CALIBRATED on live ground truth (2026-07-06, owner's
    eye): the +/-0.3-recentered ghost WITHOUT outward roll penetrated the thighs live
    (worst-case 0.095 m); with shroll 0.15 it walked clean (0.127 m) -> threshold 0.11 m.
    The engine simulates NO self-collision, so designs must clear by construction."""
    try:
        import mujoco as mj
    except Exception:
        rep.add("WARN", "T0.hand-thigh-clearance", "mujoco unavailable -- clearance not checked")
        return
    mjcf = os.path.join(_RT, "projects/robots/unitree/g1/mjcf/g1_23dof_omnisim.mjcf")
    if not os.path.exists(mjcf):
        rep.add("WARN", "T0.hand-thigh-clearance", "canonical mjcf missing -- clearance not checked")
        return
    m = mj.MjModel.from_xml_path(mjcf)
    dat = mj.MjData(m)
    qa = {nm: int(m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, nm)])
          for nm in JOINTS + SHOULDERS + ["left_elbow_joint", "right_elbow_joint",
                                          "left_shoulder_roll_joint", "right_shoulder_roll_joint"]}
    bid = {nm: mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, nm) for nm in
           ("left_wrist_roll_rubber_hand", "right_wrist_roll_rubber_hand", "left_elbow_link",
            "right_elbow_link", "left_hip_pitch_link", "right_hip_pitch_link",
            "left_knee_link", "right_knee_link")}
    elb = np.asarray(d["elbow_lut"], np.float64) if "elbow_lut" in d else np.full((nb, 2), 0.3)
    att = np.asarray(d["att_lut"], np.float64) if "att_lut" in d else np.zeros((nb, 2))
    yqa = {nm: int(m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, nm)])
           for nm in ("left_shoulder_yaw_joint", "right_shoulder_yaw_joint")}
    shroll = float(d.get("shroll", 0.0))

    def seg_dist(p, a, b):
        ab = b - a; t = np.clip(np.dot(p - a, ab) / max(1e-9, np.dot(ab, ab)), 0, 1)
        return float(np.linalg.norm(p - (a + t * ab)))

    def sweep(droll, yaw, use_att):
        # droll: shoulder-roll offset from the design (negative = inward, the corridor allows it);
        # yaw: shoulder-yaw magnitude tried in BOTH signs (corridor twist); use_att: tilt the pelvis
        # by the recorded sway (the thigh leans toward one hand -- part of the live penetration).
        worst = 9.0
        for f in range(nb):
            for ys in ((-yaw, yaw) if yaw else (0.0,)):
                dat.qpos[:] = 0; dat.qpos[0:3] = [0, 0, 1]
                if use_att:
                    r2, p2 = att[f, 0] / 2, att[f, 1] / 2
                    dat.qpos[3:7] = [math.cos(p2) * math.cos(r2), math.cos(p2) * math.sin(r2),
                                     math.sin(p2) * math.cos(r2), -math.sin(p2) * math.sin(r2)]
                else:
                    dat.qpos[3:7] = [1, 0, 0, 0]
                for i, nm in enumerate(JOINTS):
                    dat.qpos[qa[nm]] = leg[f, i]
                dat.qpos[qa["left_shoulder_pitch_joint"]] = arm[f, 0]
                dat.qpos[qa["right_shoulder_pitch_joint"]] = arm[f, 1]
                dat.qpos[qa["left_elbow_joint"]] = elb[f, 0]; dat.qpos[qa["right_elbow_joint"]] = elb[f, 1]
                dat.qpos[qa["left_shoulder_roll_joint"]] = shroll + droll
                dat.qpos[qa["right_shoulder_roll_joint"]] = -(shroll + droll)
                dat.qpos[yqa["left_shoulder_yaw_joint"]] = ys
                dat.qpos[yqa["right_shoulder_yaw_joint"]] = -ys
                mj.mj_kinematics(m, dat)
                for side in ("left", "right"):
                    hand = dat.xpos[bid[f"{side}_wrist_roll_rubber_hand"]]
                    fam = (hand + dat.xpos[bid[f"{side}_elbow_link"]]) / 2
                    hip = dat.xpos[bid[f"{side}_hip_pitch_link"]]; knee = dat.xpos[bid[f"{side}_knee_link"]]
                    worst = min(worst, seg_dist(hand, hip, knee), seg_dist(fam, hip, knee))
        return worst

    design = sweep(0.0, 0.0, False)
    # worst-case achieved: SHRY corridor exploited fully (roll 0.10 inward + yaw 0.10 either way)
    # under the recorded pelvis sway. This is what the live robot is ALLOWED to do -- and did.
    achieved = sweep(-0.10, 0.10, True)
    # thresholds: centerline contact ~0.10 (thigh ~0.06 + hand ~0.04 mesh half-widths); the design
    # the owner saw CLIPPING on screen reads 0.122 (graze), the clean one 0.147 -> WARN <0.135.
    # (The deep right-arm-in-body sighting was the deploy SHRY sign bug, not a lut property.)
    rep.add("FAIL" if achieved < 0.115 else ("WARN" if achieved < 0.135 else "ok"),
            "T0.hand-thigh-clearance",
            f"min hand/forearm-to-thigh: design {design:.3f} m, corridor-worst {achieved:.3f} m "
            f"(live-calibrated 2026-07-06: 0.122 grazed on screen, 0.147 clean; no self-collision in-engine)")


class Report:
    def __init__(self):
        self.rows = []

    def add(self, level, check, msg):
        self.rows.append((level, check, msg))
        print(f"  [{level:4s}] {check:28s} {msg}")

    @property
    def verdict(self):
        levels = [r[0] for r in self.rows]
        return "FAIL" if "FAIL" in levels else ("WARN" if "WARN" in levels else "PASS")


def validate(path, baseline_path=None, rep=None):
    d = json.loads(open(path).read())
    global JOINTS
    if "joints" in d:                       # self-describing lut (every lut, as of schema 2)
        JOINTS = list(d["joints"])
    leg = np.asarray(d["leg_lut"], np.float64)
    nb = int(d["nb"]); freq = float(d.get("freq", 1.25))
    dt_bin = (1.0 / freq) / nb
    robot = _robot_of(d)
    # ── CYCLIC vs SEQUENCE ───────────────────────────────────────────────────
    # Three T0 gates (phase-wrap, harmonic-jitter, leg-symmetry) are STATEMENTS
    # ABOUT A PERIODIC SIGNAL. A sequence ghost -- a stair climb, a step-turn --
    # has a start and an end: it is NOT required to close at phase 2*pi, its
    # spectrum is not a clean harmonic series, and a turn is asymmetric BY
    # DESIGN. Scoring a sequence on those gates is a category error, and it was
    # producing false verdicts: the climb ghost FAILED on a 0.70 rad "wrap
    # discontinuity" that is simply the gap between the top of the stairs and
    # the start of the climb. The manifests were papering over this in prose
    # ("validator is walk-calibrated"); it belongs in the tool.
    # Sequence luts self-declare with `seq` (+ `hold_end`); cyclic luts have no
    # such key, so their calibration is untouched -- verified old==new on all
    # five G1 skill ghosts.
    is_seq = bool(d.get("seq"))
    # ── TURN ghosts (declared `wz` != 0, 2026-07-17) ─────────────────────────
    # A steering gait is CYCLIC (wrap/jitter gates apply in full) but its L/R
    # phase-mirror symmetry is broken BY DESIGN: differential stride means the
    # left legs sweep opposite the right, and the mirror of a +wz turn is a -wz
    # turn, not itself. Scoring it on the mirror-correlation gate is the same
    # category error the `seq` flag fixed for step-turns/climbs -- and the
    # go2_stand manifest already recorded that this kind of fix belongs IN THE
    # TOOL, not in prose papering over a WARN. Amplitude balance still applies
    # (a turn-in-place has equal-magnitude opposite sweeps, so a grossly
    # lopsided amp ratio remains a defect); only the sign-sensitive mirror
    # correlation is n/a. No shipped lut declares `wz`, so every existing
    # verdict is bit-unchanged by construction.
    try:
        wz_decl = float(d.get("wz", 0.0) or 0.0)
    except (TypeError, ValueError):
        wz_decl = 0.0
    is_turn = abs(wz_decl) > 1e-6
    rep = rep if rep is not None else Report()
    print(f"GHOST VALIDATOR: {os.path.basename(path)}  (robot={robot} nb={nb} freq={freq}Hz "
          f"vx={d.get('vx', '?')})\n  source: {d.get('source', 'MISSING')[:100]}")

    # ── T0: kinematic legality ──────────────────────────────────────────────
    # FAIL-CLOSED. No model => no verdict. Not a skipped gate, not a WARN: a FAIL.
    # (An unregistered robot used to skip T0 entirely and still print PASS.)
    try:
        lim = RR.joint_limits(robot)
    except (RR.UnknownRobot, RR.ModelUnavailable) as e:
        lim = {}
        rep.add("FAIL", "T0.model", f"{e} -- T0 CANNOT vouch for this ghost")
    if lim:
        # ⛔ THE VACUOUS-PASS GUARD. A joint the model does not know cannot be
        # limit-checked, and silently defaulting it to (-99, 99) is how an H1 ghost
        # used to sail through the G1's URDF: unknown joints "passed" because
        # nothing constrained them. An unchecked joint is a FAILED check, not a
        # passed one -- if this fires, the lut and the robot disagree, and the
        # verdict below it is worthless.
        unknown = [jn for jn in JOINTS if jn not in lim]
        rep.add("FAIL" if unknown else "ok", "T0.model-coverage",
                (f"lut declares robot={robot!r} but its URDF has no such joint(s): {unknown} "
                 f"-- T0 CANNOT vouch for this ghost (wrong robot, or a stale lut)"
                 if unknown else
                 f"all {len(JOINTS)} lut joints exist in the {robot} URDF"))
        hard, soft = [], []
        for i, jn in enumerate(JOINTS):
            if jn not in lim:
                continue                        # already FAILED above; do not fake a limit
            lo, hi, _ = lim[jn]
            over = max(lo - float(leg[:, i].min()), float(leg[:, i].max()) - hi)
            if over > 0.05:
                hard.append(f"{jn}:+{over:.3f}rad")
            elif over > 0:
                soft.append(f"{jn}:+{over:.3f}rad")
        # calibration anchor: the flagship champion's own gait exceeds the G1 ankle-roll URDF limit
        # by 0.033 rad (trainable in-sim because the deploy recipe disables the joint clamp) --
        # small overshoots WARN as a HARDWARE-TRANSFER flag; gross ones FAIL.
        rep.add("FAIL" if hard else ("WARN" if soft else "ok"), "T0.position-limits",
                (f"GROSS violations: {hard}" if hard else
                 (f"over URDF limit (sim-ok w/ clamp off; HARDWARE flag): {soft}" if soft
                  else "all leg samples inside URDF limits")))
        vbad = []
        vel = np.gradient(np.vstack([leg, leg[:1]]), dt_bin, axis=0)[:-1]  # wrap-aware
        for i, jn in enumerate(JOINTS):
            if jn not in lim:
                continue                        # unknown joint: FAILED above, not vouched here
            vl = lim[jn][2]
            if np.abs(vel[:, i]).max() > 0.7 * vl:
                vbad.append(f"{jn}:{np.abs(vel[:, i]).max():.1f}/{vl:.0f}")
        rep.add("WARN" if vbad else "ok", "T0.velocity-limits",
                f">70% of URDF vel limit: {vbad}" if vbad else "peak joint velocities < 70% of limits")
    # wrap continuity: the lut must close smoothly (C0/C1 at phase 2pi -> 0). Threshold is
    # RELATIVE to the lut's own bin resolution (a coarse 64-bin fold has ~0.05 rad bin steps
    # that are discretization, not discontinuity -- the official G1 lut anchors this).
    c0 = float(np.abs(leg[0] - leg[-1]).max())
    c1 = float(np.abs((leg[1] - leg[0]) - (leg[0] - leg[-1])).max())
    p95 = float(np.percentile(np.abs(np.diff(leg, axis=0)).max(1), 95))
    thr = max(0.05, 2.0 * p95)
    if is_seq:
        rep.add("ok", "T0.phase-wrap",
                f"n/a: SEQUENCE ghost (start != end by design). C0 jump {c0:.4f} rad is the "
                f"reset from the end pose back to the start pose, not a discontinuity.")
    else:
        rep.add("FAIL" if c0 > thr else ("WARN" if c0 > p95 * 1.5 and c0 > 0.05 else "ok"), "T0.phase-wrap",
                f"C0 jump {c0:.4f} rad (own p95 bin step {p95:.4f}), C1 kink {c1:.4f} rad at wrap")
    # jitter: energy above the 8th harmonic should be negligible in a clean gait
    F = np.fft.rfft(leg, axis=0)
    hi_e = float(np.sqrt((np.abs(F[9:]) ** 2).sum()) / max(1e-9, np.sqrt((np.abs(F[1:]) ** 2).sum())))
    rep.add("ok" if is_seq else ("WARN" if hi_e > 0.08 else "ok"), "T0.harmonic-jitter",
            (f"n/a: SEQUENCE ghost is not a periodic signal (high-harmonic energy "
             f"{hi_e * 100:.1f}% is its transients, not jitter)" if is_seq else
             f"energy above 8th harmonic: {hi_e * 100:.1f}% (clean recorded gaits: <5%)"))
    # L/R symmetry: the right leg, half-cycle-shifted, should mirror the left (pitch-plane
    # joints only -- the rolls are antisymmetric, not mirror-symmetric).
    #
    # Pairing is by MIRROR NAME, not by slot arithmetic. The old rule paired (i, i + n/2),
    # which silently assumes a [all-left | all-right] block layout -- true on the G1, FALSE
    # on a quadruped (FL,FR,RL,RR is interleaved). That assumption is why the Go2 ghost
    # builder had to write FAKE humanoid joint names into the lut to make this gate engage.
    # It now pairs FL<->FR and RL<->RR natively, and the alias hack is gone.
    half = nb // 2
    sym_err = []
    _idx = {jn: i for i, jn in enumerate(JOINTS)}
    _pairs = []
    for i, jn in enumerate(JOINTS):
        if not RR.is_pitch_plane(jn) or RR.side_row(jn)[0] != "L":
            continue                            # walk the left side; find its mirror
        mate = RR.mirror_joint(jn)
        if mate in _idx:
            _pairs.append((i, _idx[mate], 1.0))
    for li_, ri_, sgn in _pairs:   # pitch-plane joints, by NAME (robot-agnostic)
        L = leg[:, li_] - leg[:, li_].mean()
        Rr = np.roll(leg[:, ri_] - leg[:, ri_].mean(), half) * sgn
        den = max(1e-6, np.std(L) * np.std(Rr))
        sym_err.append(1.0 - float((L * Rr).mean() / den))
    amp_ratio = []
    for li_, ri_, _s2 in _pairs[:2]:
        aL = np.ptp(leg[:, li_]); aR = np.ptp(leg[:, ri_])
        amp_ratio.append(min(aL, aR) / max(aL, aR, 1e-6))
    if not _pairs:
        # No mirror pair could be formed from the declared joint names. The gate did not
        # run -- so it does not get to pass. (Fail-closed, same rule as T0.model.)
        rep.add("FAIL", "T0.leg-symmetry",
                f"no L/R mirror pair derivable from the lut's joint names ({JOINTS}) -- "
                f"the gate CANNOT run, so it cannot vouch for this ghost")
    else:
        sy = float(np.mean(sym_err)); ar = float(np.min(amp_ratio))
        if is_seq:
            rep.add("ok", "T0.leg-symmetry",
                    f"n/a: SEQUENCE ghost (a step-turn / climb is asymmetric BY DESIGN; "
                    f"mirror error {sy:.2f})")
        elif is_turn:
            rep.add("WARN" if ar < 0.6 else "ok", "T0.leg-symmetry",
                    f"TURN ghost (wz={wz_decl:+.2f} rad/s declared): mirror correlation n/a "
                    f"-- differential stride is L/R-antisymmetric BY DESIGN (mirror error "
                    f"{sy:.2f} is the sweep, not a defect); amp ratio {ar:.2f} still checked "
                    f"(ok>0.6: opposite sweeps should be equal-magnitude)")
        else:
            rep.add("WARN" if (sy > 0.25 or ar < 0.7) else "ok", "T0.leg-symmetry",
                    f"phase-mirror error {sy:.2f} (ok<0.25), amp ratio {ar:.2f} (ok>0.7)")
    if "arm_lut" in d:
        arm = np.asarray(d["arm_lut"], np.float64)
        aL, aR = np.ptp(arm[:, 0]), np.ptp(arm[:, 1])
        # STATIC arms (both amplitudes ~0) are a legitimate design choice for a legs-only
        # maneuver (e.g. a step-turn holds the arms) -- symmetric by definition, not a defect.
        # The rule targets ASYMMETRIC swing (one arm flails), so only judge when arms actually move.
        if max(aL, aR) < 0.05:
            rep.add("ok", "T0.arm-symmetry", f"arms held static (L {aL:.2f} / R {aR:.2f} rad) -- legs-only design, symmetric by construction")
        else:
            r = min(aL, aR) / max(aL, aR, 1e-6)
            rep.add("FAIL" if r < 0.5 else ("WARN" if r < 0.75 else "ok"), "T0.arm-symmetry",
                    f"arm swing amplitudes L {aL:.2f} / R {aR:.2f} rad (ratio {r:.2f}; owner scratched a 0.2-ratio ghost)")
        if robot in _CLEARANCE_ROBOTS:
            _hand_thigh_clearance(rep, d, leg, arm, nb)
        else:
            rep.add("ok", "T0.hand-thigh-clearance",
                    f"not applicable to {robot} (gate FKs G1-specific bodies)")

    # ── T1: coupling laws (anchored to 2026-07-03 measurements) ─────────────
    if baseline_path:
        b = np.asarray(json.loads(open(baseline_path).read())["leg_lut"], np.float64)
        # resample baseline to nb
        idx = (np.arange(nb) * (len(b) / nb)).astype(int) % len(b)
        b = b[idx]
        # THE DOMINANT RULE: parametric edits to a recorded gait beyond the measured envelope
        # collapsed training EVERY time (stance -25%: falls 47-80%), with or without sway
        # compensation. Free-zone measured at ~10%; hard limit set at 15% of the joint's own range.
        edits = []
        for i, jn in enumerate(JOINTS):
            dmean = abs(float(leg[:, i].mean() - b[:, i].mean()))
            scale = max(0.05, np.ptp(b[:, i]))
            if dmean > 0.15 * scale and dmean > 0.03:
                edits.append(f"{jn}: mean shifted {dmean:.3f} rad ({dmean / scale * 100:.0f}% of range)")
        rep.add("FAIL" if edits else "ok", "T1.edit-envelope",
                ("beyond the measured fine-tune envelope -- RE-RECORD instead of editing: " + "; ".join(edits))
                if edits else "within the ~15% parametric-edit envelope of the baseline")
    else:
        _src0 = str(d.get("source", "")).lower()
        if any(k in _src0 for k in ("recorded", "achieved", "folded", "fold")):
            rep.add("ok", "T1.edit-envelope",
                    "primary recording (no baseline to diff against) -- the dominant rule governs EDITS only")
        else:
            rep.add("WARN", "T1.edit-envelope", "no --baseline given; cannot check the dominant rule")
    # elbow-extension x arm-amplitude budget (straight arms ~double the hand arc)
    elbow = float(d.get("elbow", 0.0))
    if "arm_lut" in d:
        arm = np.asarray(d["arm_lut"], np.float64)
        A = float(max(np.ptp(arm[:, 0]), np.ptp(arm[:, 1]))) / 2
        ext = 1.0 + max(0.0, elbow) / 1.6            # 1.0 bent .. 2.0 straight (arc factor proxy)
        budget = A * ext
        rep.add("FAIL" if budget > 0.45 else ("WARN" if budget > 0.35 else "ok"),
                "T1.arm-momentum-budget",
                f"amplitude {A:.2f} x extension {ext:.2f} = {budget:.2f} "
                f"(bent-arm recorded baseline ~0.30; straight elbows with bent-arm amplitude collapsed)")

    # ── T2: provenance ──────────────────────────────────────────────────────
    src = str(d.get("source", "")).lower()
    if any(k in src for k in ("achieved", "recorded", "folded", "fold")):
        lvl, msg = "ok", "recorded/achieved provenance (trains cleanly)"
    elif any(k in src for k in ("footstep", "inverse kinematic", " ik", "ik,", "com-over-support",
                                "constructed", "achievable by construction", "trajectory opt", "solved")):
        # SOLVED / constructed-feasible: a trajectory computed to satisfy the robot's own physics
        # (COM-over-support + joint/vel limits) is achievable BY CONSTRUCTION -- proven 2026-07-07
        # (build_step_turn_ghost.py: shadows with 0 NaN, deploys footwork). NOT eyeballed hand-design.
        lvl, msg = "ok", "constructed-feasible (footstep/IK, COM-over-support) -- solved to satisfy the physics, valid"
    elif any(k in src for k in ("narrow", "scaled", "re-centered", "recentered", "becalmed", "consistent")):
        lvl, msg = "WARN", "parametric edit of a recording -- valid ONLY inside the T1 envelope; else re-record"
    else:
        lvl, msg = "WARN", ("unknown provenance -- EYEBALLED hand-design (joint angles by feel) never "
                            "trained durably; but a trajectory SOLVED for feasibility is fine (see T2 note)")
    rep.add(lvl, "T2.provenance", msg)

    # ── T3: corridor adequacy (2026-07-13) ──────────────────────────────────
    # THE SECOND HALF OF THE CORRIDOR-vs-TORQUE LAW. GHOST_FF fixed the corridor's CENTRE (it must
    # carry the reference's own stance torque, tau_ff/kp). This is its WIDTH: to balance on the ankle
    # the policy must drive the centre of pressure to the edge of the sole, which costs
    # tau = m*g*(foot_len/2), i.e. dq = tau/kp of joint deviation through the PD plant. A corridor
    # narrower than dq CANNOT reach full ankle authority -- and whatever it denies, the crane supplies.
    #
    # This gate exists because that was discovered the expensive way. Measured (same ghost, same
    # iteration, corridor the only difference): 0.12 rad = 84% of dq_cop -> 43.6% of action-dims
    # saturated, crane lean 25%, campaign stalled. 0.24 rad = 169% -> 0.0% saturated, crane lean 6.4%,
    # and the ghost was matched BETTER (gmatch 0.931 -> 0.946). A clipped corrector cannot fix an error
    # while it is small, so the error grows until the correction it needs is one the corridor refuses:
    # the narrow corridor MANUFACTURES the deviation it appears to prevent.
    #
    # Pass --corridor <GHOST_RESIDUAL> to grade a specific training config; with no --corridor the gate
    # just reports the minimum this robot's own geometry demands.
    # FAIL CLOSED, like every other model-aware gate here: an unregistered robot has no class, so
    # this gate cannot run -- and a gate that cannot run does not get to pass. (It used to raise
    # UnknownRobot straight out of the validator: still non-zero, but a traceback is not a verdict,
    # and the caller loses every other gate's result.)
    try:
        _rclass = RR.robot_class(robot)
    except (RR.UnknownRobot, RR.ModelUnavailable) as _e3:
        _rclass = None
        rep.add("FAIL", "T3.corridor-adequacy", f"{_e3} -- cannot grade the corridor")
    if _rclass is None:
        pass
    elif _rclass != RR.HUMANOID:
        # Point-footed quadrupeds have NO ankle CoP authority by construction -- balance is footstep
        # placement, not sole torque. The law does not apply; say so rather than pass silently.
        rep.add("ok", "T3.corridor-adequacy",
                f"n/a: {robot} is a {_rclass} (no ankle CoP lever; balance is footstep placement)")
    else:
        try:
            law = RR.corridor_min_rad(robot, kp=KP)
            dq = law["dq_cop_pitch_rad"]; rec = law["recommended_rad"]
            base = (f"{robot} {law['mass_kg']:.1f} kg, sole {2*law['foot_half_len_m']:.3f} m, kp={KP:.0f} "
                    f"-> full ankle CoP authority costs dq={dq:.3f} rad; recommend corridor >= {rec:.3f}")
            if CORRIDOR is None:
                rep.add("WARN", "T3.corridor-adequacy",
                        f"no --corridor given, cannot grade the training config. {base}")
            else:
                pct = 100.0 * CORRIDOR / dq
                if CORRIDOR < dq:
                    rep.add("FAIL", "T3.corridor-adequacy",
                            f"corridor {CORRIDOR:.3f} rad = {pct:.0f}% of CoP authority: the policy is "
                            f"STRUCTURALLY FORBIDDEN from reaching the edge of its own foot -- the crane "
                            f"will pay the difference. {base}")
                elif CORRIDOR < rec:
                    rep.add("WARN", "T3.corridor-adequacy",
                            f"corridor {CORRIDOR:.3f} rad = {pct:.0f}% of CoP authority: reachable but no "
                            f"margin for tracking error/dynamics. {base}")
                else:
                    rep.add("ok", "T3.corridor-adequacy",
                            f"corridor {CORRIDOR:.3f} rad = {pct:.0f}% of CoP authority. {base}")
        except (RR.UnknownRobot, RR.ModelUnavailable) as e:
            # FAIL-CLOSED: an unmeasurable foot means an ungradeable corridor, and an ungraded corridor
            # is exactly how every shipped G1 skill ended up capped at 70% of its own ankle authority.
            rep.add("FAIL", "T3.corridor-adequacy", f"cannot measure this robot's foot: {e}")

    print(f"  VERDICT: {rep.verdict}")
    return rep.verdict


def stamp(path, verdict, rows):
    """Write the verdict INTO the lut (`validator` key), so the attestation travels with the
    artifact instead of being retyped as prose in a skill manifest that can silently go stale."""
    import collections
    import datetime
    d = json.loads(open(path).read(), object_pairs_hook=collections.OrderedDict)
    d["validator"] = collections.OrderedDict([
        ("verdict", verdict),
        ("date", datetime.date.today().isoformat()),
        ("tool", "projects/policies/training/ghost_validator.py"),
        ("gates", collections.OrderedDict((check, lvl) for lvl, check, _ in rows)),
    ])
    json.dump(d, open(path, "w"), separators=(", ", ": "))
    print(f"  [stamp] validator.verdict={verdict} written into {os.path.basename(path)}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("ghost")
    ap.add_argument("--baseline", default=None,
                    help="the recorded gait this ghost derives from (enables the dominant edit-envelope rule)")
    ap.add_argument("--stamp", action="store_true",
                    help="write the verdict into the lut's `validator` key (the attestation "
                         "belongs ON the artifact, not in a manifest's prose)")
    ap.add_argument("--corridor", type=float, default=None,
                    help="the GHOST_RESIDUAL this ghost will be TRAINED with (rad). Grades T3: a corridor "
                         "below full ankle-CoP authority (m*g*sole/2/kp) cannot reach the edge of its own "
                         "foot, and the crane pays the difference. Every shipped G1 skill runs 0.100 = 70%%.")
    ap.add_argument("--kp", type=float, default=200.0,
                    help="PD stiffness the policy acts through (default 200 = OMNISIM_NEWTON_TARGET_KE "
                         "in run_walk_rl.sh). The corridor law is dq = tau/kp, so kp sets the bar.")
    args = ap.parse_args()
    CORRIDOR = args.corridor
    KP = args.kp
    _rep = Report()
    _v = validate(args.ghost, args.baseline, rep=_rep)
    if args.stamp:
        stamp(args.ghost, _v, _rep.rows)
    sys.exit(0 if _v != "FAIL" else 1)
