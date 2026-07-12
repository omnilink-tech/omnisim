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

"""ghost_doctor.py -- prescriptive ghost classifier, calibrated per robot from its OWN recordings.

Owner design (2026-07-05): "a classifier the ghost passes through that tells us HOW TO IMPROVE
it based on the dynamics of the robot we are shadowing -- general for any robot."

Tiers (certainty decreasing, each violation carries a computed PRESCRIPTION):
  T0 ANALYTIC   -- joint position/velocity limits at the lut's cadence (URDF; hard physics).
  T1 LAWS       -- self-consistency constants CALIBRATED from the robot's achieved library:
                   speed = k * hip_swing * cadence (k measured, not assumed);
                   arm momentum budget (amplitude x extension <= library max);
                   rock amplitude vs library.
  T2 ENVELOPE   -- each channel vs the robot's PROVEN envelope (achieved-recording library):
                   inside = safe; outside = risk score + smallest edit to re-enter.
  T3 RANK       -- balance-gate static margin RANKED against the library's proven margins
                   (the gate flags all walking; only the RANK is meaningful -- calibrated
                   2026-07-05: proven gait -195mm, infeasible hum70-class < -246mm).
FINAL WORD stays with a tracking burst: the doctor makes bursts rare, not obsolete. Its
verdict tiers: PASS (inside proven envelope), TARGET (outside but prescribed -- burst-verify),
FAIL (analytic violation -- fix before any training).

Usage:
  python ghost_doctor.py <lut.json> --robot-spec specs/retarget_g1.json \
      [--library "projects/policies/controllers/g1_ghost/ghost_{v3c,v5,v62,v8,unitree,achieved,walkstop_achieved}_lut.json"]
"""
import argparse
import glob
import json
import re
import subprocess
import sys

import numpy as np

TRAINER_CYCLE_SCALE = 2.0   # trainer phase-dt = action-dt/2: wall cycle = 2/freq_label (measured 2026-07-05)


def _load(path):
    d = json.load(open(path))
    legs = np.asarray(d["leg_lut"], np.float32)
    return d, legs


def channel_metrics(d, legs):
    nb = len(legs)
    freq = float(d.get("freq", 1.25))
    cyc_s = TRAINER_CYCLE_SCALE / freq
    vel = np.abs(np.diff(np.concatenate([legs, legs[:1]], 0), axis=0)) * nb / cyc_s
    m = dict(
        cadence_hz=1.0 / cyc_s,
        vx=float(d.get("vx", np.nan)),
        hip_swing=float(np.degrees(max(legs[:, 0].max() - legs[:, 0].min(),
                                       legs[:, 6].max() - legs[:, 6].min()))),
        knee_min=float(np.degrees(min(legs[:, 3].min(), legs[:, 9].min()))),
        knee_max=float(np.degrees(max(legs[:, 3].max(), legs[:, 9].max()))),
        rock_deg=0.0, arm_amp=0.0, arm_mean=0.0, elbow_mean=0.0,
        peak_knee_vel=float(max(vel[:, 3].max(), vel[:, 9].max())),
    )
    if "att_lut" in d:
        att = np.asarray(d["att_lut"], np.float32)
        m["rock_deg"] = float(np.degrees(att[:, 0].max() - att[:, 0].min()))
    if "arm_lut" in d:
        arm = np.asarray(d["arm_lut"], np.float32)
        m["arm_amp"] = float(max(arm[:, 0].max() - arm[:, 0].min(), arm[:, 1].max() - arm[:, 1].min()))
        m["arm_mean"] = float(arm.mean())
    if "wb_lut" in d and "wb_joints" in d:
        wb = np.asarray(d["wb_lut"], np.float32)
        ei = [i for i, n in enumerate(d["wb_joints"]) if "elbow" in n]
        if ei:
            m["elbow_mean"] = float(wb[:, ei].mean())
    return m


def urdf_limits(urdf_path):
    u = open(urdf_path).read()
    lims = {}
    for mm in re.finditer(r'<joint name="([^"]+)"[^>]*>.*?<limit[^>]*?(?:effort="[^"]*")?[^>]*'
                          r'lower="([-\d.eE]+)"[^>]*upper="([-\d.eE]+)"(?:[^>]*velocity="([-\d.eE]+)")?',
                          u, re.S):
        lims[mm.group(1)] = (float(mm.group(2)), float(mm.group(3)),
                             float(mm.group(4)) if mm.group(4) else None)
    return lims


LEG12 = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
         "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
         "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
         "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"]


def _t0_wb(d, lims, findings):
    """T0 for the WHOLE BODY via wb_lut (the leg-only loop was a blind spot: an arm-channel
    NaN hunt on 2026-07-05 chased a limit violation T0 never checked for)."""
    if "wb_lut" not in d or "wb_joints" not in d:
        return
    wb = np.asarray(d["wb_lut"], np.float32)
    for i, jn in enumerate(d["wb_joints"]):
        if jn not in lims or i < 12:                 # legs already covered with velocity checks
            continue
        lo, hi, _ = lims[jn]
        if wb[:, i].min() < lo - 1e-3 or wb[:, i].max() > hi + 1e-3:
            over = max(lo - wb[:, i].min(), wb[:, i].max() - hi)
            findings.append(("T0", "FAIL", f"{jn} exceeds position limit by {over:.3f} rad",
                             f"clamp wb channel {i} into [{lo:.2f},{hi:.2f}]"))


def diagnose(lut_path, spec_path, library_paths, urdf_path):
    d, legs = _load(lut_path)
    m = channel_metrics(d, legs)
    lib = []
    for pth in library_paths:
        if pth == lut_path:
            continue
        try:
            dl, ll = _load(pth)
            lib.append((pth.split("/")[-1], channel_metrics(dl, ll)))
        except Exception:
            pass
    findings = []   # (tier, severity, message, prescription)

    # ---- T0 analytic ----
    lims = urdf_limits(urdf_path)
    nb = len(legs)
    cyc_s = TRAINER_CYCLE_SCALE / float(d.get("freq", 1.25))
    for c, jn in enumerate(LEG12):
        if jn not in lims:
            continue
        lo, hi, vlim = lims[jn]
        if legs[:, c].min() < lo - 1e-3 or legs[:, c].max() > hi + 1e-3:
            over = max(lo - legs[:, c].min(), legs[:, c].max() - hi)
            findings.append(("T0", "FAIL", f"{jn} exceeds position limit by {over:.3f} rad",
                             f"clip/scale channel {c} into [{lo:.2f},{hi:.2f}]"))
        if vlim:
            pv = float(np.abs(np.diff(np.concatenate([legs[:, c], legs[:1, c]]))).max() * nb / cyc_s)
            if pv > vlim:
                f_ok = float(d.get("freq", 1.25)) * vlim / pv
                findings.append(("T0", "FAIL", f"{jn} needs {pv:.1f} rad/s > limit {vlim:.1f} at this cadence",
                                 f"slow cadence to <= {f_ok:.2f} Hz-label OR scale amplitude x{vlim/pv:.2f}"))

    _t0_wb(d, lims, findings)

    # ---- T1 laws (calibrated) ----
    ks = [lm["vx"] / max(1e-6, lm["hip_swing"] * lm["cadence_hz"])
          for _, lm in lib if np.isfinite(lm["vx"]) and lm["hip_swing"] > 5]
    if ks and np.isfinite(m["vx"]):
        k = float(np.median(ks))
        vx_pred = k * m["hip_swing"] * m["cadence_hz"]
        if abs(vx_pred - m["vx"]) > 0.35 * max(vx_pred, m["vx"]):
            findings.append(("T1", "WARN",
                             f"speed-consistency: {m['hip_swing']:.0f}deg @ {m['cadence_hz']:.2f}Hz predicts "
                             f"{vx_pred:.2f} m/s (robot constant k={k:.4f}) but lut says {m['vx']:.2f}",
                             f"set vx={vx_pred:.2f} OR adjust swing/cadence to match the label"))
    # G1 elbow convention: 0 = factory ~90deg BENT carry, +1.6 = full EXTENSION (this trap has
    # now cost two sessions -- 2026-07-03 and 2026-07-05). Hand-arc ~ (1 + extension fraction).
    def _bud(lm):
        return lm["arm_amp"] * (1.0 + max(0.0, min(1.0, lm["elbow_mean"] / 1.6)))
    buds = [_bud(lm) for _, lm in lib if lm["arm_amp"] > 0]
    if buds and m["arm_amp"] > 0:
        bud = _bud(m)
        bmax = max(buds)
        if bud > 1.15 * bmax:
            amp_ok = m["arm_amp"] * bmax / bud
            findings.append(("T1", "WARN",
                             f"arm momentum budget {bud:.2f} > proven max {bmax:.2f} "
                             f"(straighter elbows need SMALLER swing -- the 4-collapse lesson)",
                             f"scale arm amplitude to <= {amp_ok:.2f} rad"))

    # ---- T2 envelope ----
    for ch, label, unit in (("hip_swing", "hip swing", "deg"), ("cadence_hz", "cadence", "Hz"),
                            ("vx", "speed", "m/s"), ("rock_deg", "roll rock", "deg"),
                            ("arm_amp", "arm swing", "rad")):
        vals = [lm[ch] for _, lm in lib if np.isfinite(lm[ch])]
        if not vals or not np.isfinite(m[ch]):
            continue
        lo, hi = min(vals), max(vals)
        if m[ch] > hi * 1.05:
            findings.append(("T2", "TARGET",
                             f"{label} {m[ch]:.2f}{unit} exceeds the proven envelope (max achieved {hi:.2f}{unit}, "
                             f"+{100*(m[ch]/hi-1):.0f}%)",
                             f"burst-verify required; safe fallback: scale toward {hi:.2f}{unit}"))
        elif m[ch] < lo * 0.7 and ch in ("vx", "cadence_hz"):
            findings.append(("T2", "TARGET", f"{label} {m[ch]:.2f}{unit} well below proven envelope (min {lo:.2f})",
                             "verify the label; slow references change the balance regime"))

    # ---- T3 balance-gate rank ----
    try:
        out = subprocess.run([sys.executable, "projects/policies/training/ghost_balance_gate.py",
                              lut_path, "--robot-spec", spec_path],
                             capture_output=True, text=True, timeout=600).stdout
        mm = re.search(r"static margin: mean ([+-][\d.]+)", out)
        if mm:
            margin = float(mm.group(1)) * 1000
            # calibration 2026-07-05: proven walking gait -195mm; infeasible-class < -246mm
            if margin < -260:
                findings.append(("T3", "TARGET", f"static-margin rank {margin:.0f}mm -- beyond every proven gait",
                                 "expect a hop ladder, not a single burst"))
            elif margin < -210:
                findings.append(("T3", "WARN", f"static-margin rank {margin:.0f}mm -- more dynamic than proven (-195mm)",
                                 "burst-verify with the erosion watch"))
    except Exception as e:
        findings.append(("T3", "WARN", f"balance gate unavailable ({e})", "run it manually"))

    return m, findings


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("lut")
    ap.add_argument("--robot-spec", default="projects/policies/training/specs/retarget_g1.json")
    ap.add_argument("--urdf", default="projects/robots/unitree/g1/urdf/g1_23dof.urdf")
    ap.add_argument("--library", default="projects/policies/controllers/g1_ghost/ghost_v3c_lut.json,"
                                          "projects/policies/controllers/g1_ghost/ghost_v5_lut.json,"
                                          "projects/policies/controllers/g1_ghost/ghost_v62_lut.json,"
                                          "projects/policies/controllers/g1_ghost/ghost_v8_lut.json,"
                                          "projects/policies/controllers/g1_ghost/ghost_unitree_lut.json,"
                                          "projects/policies/controllers/g1_ghost/ghost_achieved_lut.json,"
                                          "projects/policies/controllers/g1_ghost/ghost_walkstop_achieved_lut.json")
    a = ap.parse_args()
    libs = [p for pat in a.library.split(",") for p in glob.glob(pat)]
    m, findings = diagnose(a.lut, a.robot_spec, libs, a.urdf)
    print(f"GHOST DOCTOR: {a.lut}  (library: {len(libs)} achieved recordings)")
    print("  channels: " + "  ".join(f"{k}={v:.2f}" for k, v in m.items()))
    worst = "PASS"
    for tier, sev, msg, rx in sorted(findings):
        print(f"  [{tier} {sev:6s}] {msg}")
        print(f"             Rx: {rx}")
        if sev == "FAIL":
            worst = "FAIL"
        elif sev == "TARGET" and worst != "FAIL":
            worst = "TARGET (burst-verify)"
    if not findings:
        print("  no findings -- inside the proven envelope on every channel")
    print(f"  VERDICT: {worst}")
    sys.exit(2 if worst == "FAIL" else 0)


if __name__ == "__main__":
    main()
