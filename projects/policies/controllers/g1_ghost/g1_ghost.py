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

"""Kinematic GHOST controller -- "the ideal we optimise toward".

Drives a SECOND, physics-free G1 (staticBase TRUE) with the PURE human-gait
MODEL reference (projects/policies/control/gait/g1_human_gait): no RL, no balance, no
physics. The ghost stands beside the real robot (DEF G1_REAL) and its leg
cycle is locked to the real robot's FORWARD PROGRESS, so the two stay in
step: wherever the real robot is, the ghost shows the ideal pose it is
tracking. The visible gap between them IS the RL correction + sim2deploy
error.

Reads the same G1_GAIT_* env as the real controller so model params match.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

# OmniSim controllers do NOT inherit custom env vars from the headless launcher, but DO receive
# controllerArgs as sys.argv. Accept a ghost lut path passed as a controllerArg (any *.json arg) so a
# world can replay a specific lut without relying on env. Env still wins if G1_GHOST_LUT is set.
if not os.environ.get("G1_GHOST_LUT"):
    for _a in sys.argv[1:]:
        if isinstance(_a, str) and _a.endswith(".json"):
            _ap = _a if os.path.isabs(_a) else str(Path(__file__).resolve().parent / _a)
            os.environ["G1_GHOST_LUT"] = _ap if os.path.exists(_ap) else _a  # resolve rel to this dir
            break

from projects.policies.control.gait import g1_human_gait as ghg  # noqa: E402

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot

LEGS_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
)
ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
)


def _genv(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


_VISIT_LOG = []


def _ghostify(node, transparency, tint, depth=0):
    """Recursively make every Shape under `node` translucent + tinted, so the
    ghost reads as a hologram rather than a second solid robot. Walks the
    scene tree via supervisor field access (children / endPoint); sets
    PBRAppearance.transparency and baseColor in place. Returns #shapes hit."""
    if node is None or depth > 40:
        return 0
    n = 0
    try:
        type_name = node.getTypeName()
    except Exception:
        return 0
    _VISIT_LOG.append("  " * depth + type_name)
    if type_name == "Shape":
        try:
            app = node.getField("appearance").getSFNode()
            if app is not None:
                tf = app.getField("transparency")
                if tf is not None:
                    tf.setSFFloat(transparency)
                if tint is not None:
                    cf = app.getField("baseColor")
                    if cf is not None:
                        cf.setSFColor(list(tint))
                n += 1
        except Exception:
            pass
        return n
    for fname in ("children", "endPoint", "device"):
        fields = []
        for getter in ("getField", "getProtoField"):
            try:
                f = getattr(node, getter)(fname)
                if f is not None:
                    fields.append(f)
            except Exception:
                pass
        for f in fields:
            # getCount() returns -1 (not an exception) on SF fields.
            try:
                cnt = f.getCount()
            except Exception:
                cnt = -1
            if cnt is not None and cnt >= 0:
                for i in range(cnt):
                    n += _ghostify(f.getMFNode(i), transparency, tint, depth + 1)
            else:
                try:
                    n += _ghostify(f.getSFNode(), transparency, tint, depth + 1)
                except Exception:
                    pass
    return n


def _load_lut(path=None):
    """G1_GHOST_LUT: RECORD-REPLAY mode. Load a recorded official-gait cycle (JSON from the
    g1_unitree_deploy joint log, phase-folded) -- the ACHIEVABLE-ghost rule: the reference is
    what a real policy actually does, not a hand-designed curve. An explicit `path` overrides
    the env (FOLLOW mode lazy-loads whatever lut the deploy is currently tracking)."""
    import json
    path = path or os.environ.get("G1_GHOST_LUT")
    if not path:
        return None
    d = json.loads(Path(path).read_text())
    lut = np.array(d["leg_lut"], float)          # (nb, 12) slot order L6+R6
    hip_mean = (float(lut[:, 0].mean()), float(lut[:, 6].mean()))
    hip_amp = (float((lut[:, 0].max() - lut[:, 0].min()) / 2.0),
               float((lut[:, 6].max() - lut[:, 6].min()) / 2.0))
    arm_lut = np.array(d["arm_lut"], float) if "arm_lut" in d else None
    att_lut = np.array(d["att_lut"], float) if "att_lut" in d else None
    root_lut = np.array(d["root_lut"], float) if "root_lut" in d else None   # (nb,4) x y z yaw: SEQUENCE ghost
    waist_lut = np.array(d["waist_lut"], float) if "waist_lut" in d else None  # (nb,) v2 channel
    elbow_lut = np.array(d["elbow_lut"], float) if "elbow_lut" in d else None  # (nb,2) v2 channel
    wb_lut = np.array(d["wb_lut"], float) if "wb_lut" in d else None           # (nb,J) v3: ALL joints, one IK solve
    wb_joints = d.get("wb_joints")
    return dict(lut=lut, nb=len(lut), freq=float(d.get("freq", 1.25)),
                vx=float(d.get("vx", 0.45)), arm_A=float(d.get("arm_A", 0.3)),
                seq=bool(d.get("seq", False)), hold_end=bool(d.get("hold_end", False)),
                hip_mean=hip_mean, hip_amp=hip_amp, arm_lut=arm_lut, att_lut=att_lut,
                root_lut=root_lut, waist_lut=waist_lut, elbow_lut=elbow_lut,
                wb_lut=wb_lut, wb_joints=wb_joints,
                # non-upright ghosts (e.g. CRAWL): nominal base pitch + low pelvis
                # height, carried in the lut so the hologram doesn't need env.
                base_pitch=float(d.get("base_pitch", 0.0)),
                ghost_z=d.get("ghost_z"), ghost_y=d.get("ghost_y"))


def _lut_targets(L, phase, arm_A):
    """Interpolate the recorded cycle at `phase` [0,2pi). Arms: swing proportional to the
    OPPOSITE side's recorded hip-pitch deviation (left arm forward with the right leg) --
    phase-locked to the recorded gait by construction. Returns (legs13, arms10)."""
    nb = L["nb"]
    f = (phase % (2.0 * math.pi)) / (2.0 * math.pi) * nb
    i0 = int(f) % nb; i1 = (i0 + 1) % nb; w = f - int(f)
    legs = (1.0 - w) * L["lut"][i0] + w * L["lut"][i1]
    if L.get("arm_lut") is not None:             # ghost v3: RECORDED shoulder swing
        shp = (1.0 - w) * L["arm_lut"][i0] + w * L["arm_lut"][i1]
        l_sh, r_sh = float(shp[0]), float(shp[1])
    else:
        hipL = (legs[0] - L["hip_mean"][0]) / max(L["hip_amp"][0], 1e-6)
        hipR = (legs[6] - L["hip_mean"][1]) / max(L["hip_amp"][1], 1e-6)
        l_sh = arm_A * float(np.clip(hipR, -1, 1))   # left arm follows the right leg
        r_sh = arm_A * float(np.clip(hipL, -1, 1))
    att = None
    if L.get("att_lut") is not None:             # ghost v3: RECORDED base attitude (roll, pitch)
        att = (1.0 - w) * L["att_lut"][i0] + w * L["att_lut"][i1]
    # v2 channels (owner 2026-07-04: transfer MORE of the skeleton): recorded waist yaw + elbows
    _wst = 0.0
    if L.get("waist_lut") is not None:
        _wst = float((1.0 - w) * L["waist_lut"][i0] + w * L["waist_lut"][i1])
    legs13 = np.concatenate([legs, [_wst]])      # + waist_yaw
    # G1_GHOST_ELBOW: elbow bend in rad (0 = fully extended). Default 0.5 = the G1 factory carry;
    # ~0.2 = human walk (mostly extended, slight natural bend). G1_GHOST_SH_ROLL: arm-out angle.
    _elb = float(os.environ.get("G1_GHOST_ELBOW", "0.5"))
    _shr = float(os.environ.get("G1_GHOST_SH_ROLL", "0.16"))
    _eL = _eR = _elb
    if L.get("elbow_lut") is not None:           # recorded per-arm elbows override the constant
        _eL = float((1.0 - w) * L["elbow_lut"][i0][0] + w * L["elbow_lut"][i1][0])
        _eR = float((1.0 - w) * L["elbow_lut"][i0][1] + w * L["elbow_lut"][i1][1])
    arms = np.array([l_sh, _shr, 0.0, _eL, 0.0,  # shoulder P/R/Y, elbow, wrist
                     r_sh, -_shr, 0.0, _eR, 0.0])
    return legs13, arms, att


def main() -> int:
    GP = ghg.GaitParams(
        vx=_genv("G1_GAIT_VX", 0.4),
        freq=_genv("G1_GAIT_FREQ", 1.3),
        sway=_genv("G1_GAIT_A_LAT", 0.05),
        arm_swing=_genv("G1_GAIT_A_ARM", 0.25),
        ramp_s=_genv("G1_GAIT_RAMP_S", 2.0),
        style=os.environ.get("G1_GAIT_STYLE", "ik"),
        winter_hip_scale=_genv("G1_GAIT_HIP_SCALE", 0.75),
        # ── improved-shadow modes (A LIPM / B achieved / C human-3D) ──
        lateral=os.environ.get("G1_GAIT_LATERAL", "sway"),
        yaw=os.environ.get("G1_GAIT_YAW", "none"),
        lat_hip_amp=_genv("G1_GAIT_LAT_HIP_AMP", 0.09),
        step_width=_genv("G1_GAIT_STEP_WIDTH", 0.12),
    )
    Y_OFFSET = _genv("G1_GHOST_Y", 1.1)      # sideways offset from the real robot
    Z_BASE = _genv("G1_GHOST_Z", 0.78)

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())

    motors = {}
    for jn in LEGS_JOINTS + ARM_JOINTS:
        m = robot.getDevice(f"{jn}_motor")
        if m is not None:
            motors[jn] = m

    self_node = robot.getSelf()
    try:
        trans_field = self_node.getField("translation")
        rot_field = self_node.getField("rotation")
    except Exception:
        trans_field = rot_field = None
    if rot_field is not None:
        rot_field.setSFRotation([0, 0, 1, 0])    # upright, facing +x

    # The real robot, to lock the ghost's phase to its forward progress.
    real = None
    try:
        real = robot.getFromDef("G1_REAL")
    except Exception:
        real = None
    # SELF-WALK: with no real robot (or G1_GHOST_SELF_WALK=1) the ghost walks
    # on its OWN clock and translates forward at vx -> shows the pure shadow
    # walking across the floor (it is physics-free, so it never falls).
    self_walk = (real is None) or os.environ.get("G1_GHOST_SELF_WALK", "") == "1"
    x_start = None

    _glog_path = os.environ.get("G1_GHOST_LOG")
    _glog = open(_glog_path, "w", buffering=1) if _glog_path else None

    # Hologram look: translucent + pale tint (G1_GHOST_ALPHA=0 -> solid).
    alpha = _genv("G1_GHOST_ALPHA", 0.6)
    if alpha > 0.0:
        tint_s = os.environ.get("G1_GHOST_TINT", "0.62,0.82,1.0")
        try:
            tint = tuple(float(x) for x in tint_s.split(","))
        except ValueError:
            tint = (0.62, 0.82, 1.0)
        n_shapes = _ghostify(self_node, alpha, tint)
        sys.stderr.write(f"[g1_ghost] ghostified {n_shapes} shapes "
                         f"(transparency={alpha})\n")
        if _glog is not None:
            _glog.write(f"ghostified={n_shapes} alpha={alpha}\n")
            if os.environ.get("G1_GHOST_DEBUG_TREE"):
                for line in _VISIT_LOG:
                    _glog.write(f"TREE {line}\n")

    sys.stderr.write("[g1_ghost] kinematic gait-model ghost running "
                     f"(style={GP.style}, y_offset={Y_OFFSET})\n")
    sys.stderr.flush()
    _sim_ms = 0
    _last_log = -1000

    LUT = _load_lut()
    if LUT is not None:
        sys.stderr.write(f"[g1_ghost] RECORD-REPLAY mode: recorded official gait "
                         f"({LUT['nb']} bins, {LUT['freq']:.2f} Hz, vx={LUT['vx']:.2f})\n")
        # non-upright ghosts carry their own base height/offset (crawl: low pelvis)
        if LUT.get("ghost_z") is not None:
            Z_BASE = float(LUT["ghost_z"])
        if LUT.get("ghost_y") is not None:
            Y_OFFSET = float(LUT["ghost_y"])
        if LUT.get("base_pitch"):
            sys.stderr.write(f"[g1_ghost] non-upright ghost: base_pitch="
                             f"{LUT['base_pitch']:.3f} rad, z={Z_BASE:.2f}\n")

    # TURN-GHOST YAW-LOCK (2026-07-08): with a "yawlock" controllerArg, drive the ghost's phase from the
    # REAL robot's YAW PROGRESS instead of a free clock, so a turning ghost stays in LOCK-STEP with the
    # puppet (same rate, holds where the robot holds) instead of spinning ahead on its own fast clock.
    # GHOST-FOLLOW (2026-07-10, owner: "the ghost must execute the SAME motion as the puppet"):
    # with a "follow[=path]" controllerArg, the ghost renders the DEPLOY'S ACTIVE REFERENCE each
    # tick from a small state file the deploy hook publishes (GHOST_FOLLOW=1 in g1_walk_recipe):
    # mode walk|stand|turn, lut path, phase, held, robot xy, display yaw. The hologram then plays
    # the ENTIRE routine -- walking lut on the walk legs, the step-turn lut pivoting in LOCK-STEP
    # during the turn (base yaw = the robot's actual yaw, the yawlock semantics), frozen through
    # the stands -- beside the robot the whole way (the lateral offset rotates with the heading).
    # A missing or STALE (>5 s) file falls back to the legacy behavior, so worlds sharing this
    # controller (box_delivery) are untouched unless their deploy publishes.
    follow_path = None
    for _a in sys.argv[1:]:
        if isinstance(_a, str) and _a.lower().startswith("follow"):
            follow_path = _a.split("=", 1)[1] if "=" in _a else "_scratch/foot_redesign/ghost_follow.json"
    if follow_path and not os.path.isabs(follow_path):
        follow_path = str(_REPO / follow_path)
    _flw_luts = {}
    _flw_last = None
    _flw_frozen = None    # (lut_path, phase) latched at stand entry -- stands freeze the pose
    _flw_dbg = None
    _flw_dbg_t = -10000
    if follow_path:
        sys.stderr.write(f"[g1_ghost] FOLLOW mode armed: {follow_path} (falls back until fresh)\n")
        try:   # controller stderr is not captured by the run logs -- keep a side log (yawlock precedent)
            _flw_dbg = open(str(_REPO / "_scratch" / "foot_redesign" / "ghost_follow.log"), "w", buffering=1)
        except Exception:
            _flw_dbg = None

    yaw_lock = any(isinstance(a, str) and a.lower() == "yawlock" for a in sys.argv[1:])
    _yaw0 = None
    _yaw_span = 0.0
    if LUT is not None and LUT.get("root_lut") is not None:
        _rly = np.array(LUT["root_lut"], float)[:, 3]
        _yaw_span = float(_rly.max() - _rly.min())
    _yl_dbg = None
    _yl_dbg_t = -1000
    if yaw_lock:
        sys.stderr.write(f"[g1_ghost] YAW-LOCK: phase driven by G1_REAL yaw progress "
                         f"(ghost yaw span={_yaw_span:.2f} rad)\n")
        try:
            _yl_dbg = open(str(_REPO / "_scratch" / "foot_redesign" / "ghost_yawlock.log"), "w", buffering=1)
        except Exception:
            _yl_dbg = None

    while robot.step(step_ms) != -1:
        _sim_ms += step_ms
        # ── FOLLOW: read the deploy's published reference state (fresh file only) ──
        fst = None
        if follow_path:
            try:
                if os.path.exists(follow_path) and (time.time() - os.path.getmtime(follow_path)) < 5.0:
                    with open(follow_path) as _fh:
                        fst = json.loads(_fh.read())
                    _flw_last = fst
            except Exception:
                fst = _flw_last          # partial read mid-replace: keep the last good state
        gvx = LUT["vx"] if LUT is not None else GP.vx
        gfreq = LUT["freq"] if LUT is not None else GP.freq
        L_act = LUT
        _fmode = None
        if fst is not None:
            _fmode = str(fst.get("mode", "walk"))
            _lp = fst.get("lut") or ""
            if _fmode == "stand" and _flw_frozen is not None:
                _lp, phase = _flw_frozen         # a stand FREEZES the last moving pose
            else:
                phase = float(fst.get("phase", 0.0)) % (2.0 * math.pi)
                if _fmode != "stand":
                    _flw_frozen = (_lp, phase)
            if _lp and _lp not in _flw_luts:
                try:
                    _flw_luts[_lp] = _load_lut(_lp if os.path.isabs(_lp) else str(_REPO / _lp))
                    sys.stderr.write(f"[g1_ghost] FOLLOW: loaded lut {_lp}\n")
                except Exception as _fe:
                    _flw_luts[_lp] = None
                    sys.stderr.write(f"[g1_ghost] FOLLOW: lut load FAILED {_lp}: {_fe}\n")
            _Lf = _flw_luts.get(_lp)
            if _Lf is not None:
                L_act = _Lf
            _held = bool(fst.get("held", 0)) or _fmode == "stand"
            pos_x = float(fst.get("x", 0.0))
            t_equiv = _sim_ms / 1000.0
            if _flw_dbg is not None and _sim_ms - _flw_dbg_t >= 1000:
                _flw_dbg_t = _sim_ms
                _flw_dbg.write(f"t={_sim_ms/1000:6.1f} mode={_fmode:<5} lut={os.path.basename(_lp):<36} "
                               f"phase={phase:5.2f} held={int(_held)} x={pos_x:+7.2f} "
                               f"y={float(fst.get('y', 0.0)):+7.2f} yaw={float(fst.get('yaw', 0.0)):+6.2f}\n")
        elif self_walk:
            # Walk on the ghost's own clock; translate forward at vx.
            # SEQUENCE ghosts read the WORLD clock: per-controller sim_ms counts from
            # each controller's own first step, and the ghost's init (ghostify + lut
            # load) lags ~1 s behind light peers (box prop, skeleton renderer) -- the
            # box ran a second ahead of the hands (owner-caught "telepathy", 2026-07-04).
            t_equiv = robot.getTime() if (LUT is not None and LUT.get("seq")) else _sim_ms / 1000.0
            pos_x = gvx * t_equiv
        elif os.environ.get("G1_GHOST_LOCK", "") == "clock":
            # MATCHED-CLOCK lock: phase advances on TIME at the gait frequency -- the SAME clock
            # the deployed policy steps to -- while the position stays abreast of the robot.
            # (The default distance-lock paces the hologram by robot progress at the reference
            # vx; next to a slower robot it cycles in slow motion -- a display artifact that
            # made a structurally-0.81 mimic look far worse. G1_GHOST_PHASE_OFF tunes the
            # residual constant offset between the two clocks.)
            real_x = 0.0
            if real is not None:
                try:
                    real_x = float(real.getPosition()[0])
                except Exception:
                    real_x = 0.0
            t_equiv = _sim_ms / 1000.0 + _genv("G1_GHOST_PHASE_OFF", 0.0)
            pos_x = real_x
        else:
            # Lock phase + stride ramp to the real robot's forward progress.
            real_x = 0.0
            if real is not None:
                try:
                    real_x = float(real.getPosition()[0])
                except Exception:
                    real_x = 0.0
            if x_start is None:
                x_start = real_x
            t_equiv = max(0.0, real_x - x_start) / max(gvx, 1e-3)
            pos_x = real_x
        # SEQUENCE ghosts start at phase 0 (bin 0 = the routine's start; the mocap-skeleton
        # renderer starts there too). DS_PHASE is a WALK-hologram display convention -- on a
        # ~20 s routine it put the ghost 65% into the dance vs the skeleton's bin 0, which
        # the eye read as a huge speed desync (owner-caught, 2026-07-04).
        if fst is not None:
            pass                                 # FOLLOW already set phase/_held from the deploy state
        elif yaw_lock and real is not None and _yaw_span > 1e-3:
            # phase = (robot yaw progress / ghost yaw span) * 2pi, clamped -> the ghost's displayed yaw
            # tracks the robot's yaw 1:1 and freezes when the robot stops turning. getOrientation() is
            # the world 3x3 (row-major); yaw about +z = atan2(R10, R00).
            try:
                _ori = real.getOrientation()
                _yaw_now = math.atan2(float(_ori[3]), float(_ori[0]))
            except Exception:
                _yaw_now = 0.0
            if _yaw0 is None:
                _yaw0 = _yaw_now
            _yprog = math.atan2(math.sin(_yaw_now - _yaw0), math.cos(_yaw_now - _yaw0))
            _frac = max(0.0, min(1.0, abs(_yprog) / _yaw_span))
            phase = _frac * 2.0 * math.pi * (1.0 - 0.5 / max(1, LUT["nb"]))
            _held = False
            if _yl_dbg is not None and _sim_ms - _yl_dbg_t >= 500:
                _yl_dbg_t = _sim_ms
                _yl_dbg.write(f"t={_sim_ms/1000:5.1f} robot_yaw_deg={_yprog*57.2958:6.1f} "
                              f"frac={_frac:.3f} ghost_phase={phase:5.2f}\n")
        else:
            _ph0 = 0.0 if (LUT is not None and LUT.get("seq")) else ghg.DS_PHASE
            phase = (_ph0 + 2.0 * math.pi * gfreq * t_equiv) % (2.0 * math.pi)
            # hold_end (finite routines, e.g. walk-stop-walk): once the routine has played
            # through, PIN on the final bin forever instead of looping back to the start.
            # NOTE the interpolators below blend bin i with bin (i+1) % nb -- when held, they
            # must CLAMP at the final bin, or the held pose blends halfway back to bin 0
            # (owner-caught: the ghost "transported back to the start" at the end).
            _held = bool(LUT is not None and LUT.get("hold_end") and gfreq * t_equiv >= 1.0)
            if _held:
                phase = 2.0 * math.pi * (1.0 - 0.5 / max(1, LUT["nb"]))

        att = None
        if L_act is not None:
            legs, arms, att = _lut_targets(L_act, phase, L_act["arm_A"])
        else:
            legs, arms, _ = ghg.targets_np(phase, GP, t_since_start=t_equiv)
        if L_act is not None and L_act.get("wb_lut") is not None:
            # v3 WHOLE-BODY ghost: every joint from the single-solve IK track (overrides the split)
            _wf = (phase % (2.0 * math.pi)) / (2.0 * math.pi) * L_act["nb"]
            _wi0 = int(_wf) % L_act["nb"]; _wi1 = (_wi0 + 1) % L_act["nb"]; _ww = _wf - int(_wf)
            if _held and (fst is None or _fmode != "stand"):
                _wi0 = _wi1 = L_act["nb"] - 1; _ww = 0.0
            _row = (1.0 - _ww) * L_act["wb_lut"][_wi0] + _ww * L_act["wb_lut"][_wi1]
            for _j, jn in enumerate(L_act["wb_joints"]):
                m = motors.get(jn)
                if m is not None:
                    m.setPosition(float(_row[_j]))
        else:
            for i, jn in enumerate(LEGS_JOINTS):
                m = motors.get(jn)
                if m is not None:
                    m.setPosition(float(legs[i]))
            for k, jn in enumerate(ARM_JOINTS):
                m = motors.get(jn)
                if m is not None:
                    m.setPosition(float(arms[k]))

        # Position the ghost. SEQUENCE ghosts (root_lut) drive the FULL base pose through
        # space -- translation x/y/z + yaw from the verified root trajectory (dance v2:
        # time is the axis, position is a variable). Otherwise: self-walk / beside-robot.
        _yawq = 0.0
        if fst is not None and trans_field is not None:
            # FOLLOW base: ride beside the ROBOT the whole routine -- base xy = robot xy + the
            # lateral offset ROTATED into the display heading (the ghost stays on the robot's
            # left through the turn), yaw = the deploy's display heading (walk: heading target;
            # turn: the robot's actual yaw, lock-step). z comes from the active lut when it
            # carries one (the turn lut's root z), else the walk-hologram base height.
            _yq = float(fst.get("yaw", 0.0))
            _zb = Z_BASE
            if L_act is not None and L_act.get("ghost_z") is not None:
                _zb = float(L_act["ghost_z"])
            elif L_act is not None and L_act.get("seq") and L_act.get("root_lut") is not None:
                _rl9 = L_act["root_lut"]
                _zb = float(_rl9[min(int((phase / (2.0 * math.pi)) * len(_rl9)), len(_rl9) - 1)][2])
            trans_field.setSFVec3f([float(fst.get("x", 0.0)) - math.sin(_yq) * Y_OFFSET,
                                    float(fst.get("y", 0.0)) + math.cos(_yq) * Y_OFFSET, _zb])
            _yawq = _yq
        elif LUT is not None and LUT.get("root_lut") is not None and LUT.get("seq") and trans_field is not None:
            # root_lut drives the base ONLY for SEQUENCE ghosts. Recorded CYCLIC luts also
            # carry a root_lut (one cycle, ~0.5m) -- replaying it looped the hologram in
            # place (owner-caught 2026-07-05); cyclic ghosts glide at vx below instead.
            _rl = LUT["root_lut"]; _nbr = len(_rl)
            _f = (phase % (2.0 * math.pi)) / (2.0 * math.pi) * _nbr
            _i0 = int(_f) % _nbr; _i1 = (_i0 + 1) % _nbr; _w = _f - int(_f)
            if _held:
                _i0 = _i1 = _nbr - 1; _w = 0.0
            _rp = (1.0 - _w) * _rl[_i0] + _w * _rl[_i1]
            trans_field.setSFVec3f([float(_rp[0]), float(_rp[1]) + Y_OFFSET, float(_rp[2])])
            _yawq = float(_rp[3])
        elif trans_field is not None:
            trans_field.setSFVec3f([pos_x, Y_OFFSET, Z_BASE])
        # Ghost v3: animate the BASE ATTITUDE from the recorded cycle -- the hologram rocks
        # like a real walker instead of gliding unnaturally level. Sequence ghosts add YAW.
        _base_pitch = float(L_act["base_pitch"]) if L_act is not None else 0.0
        if (att is not None or _base_pitch or fst is not None) and rot_field is not None:
            _r = float(att[0]) if att is not None else 0.0
            _p = (float(att[1]) if att is not None else 0.0) + _base_pitch
            _cy2, _sy2 = math.cos(_yawq / 2), math.sin(_yawq / 2)
            _cp2, _sp2 = math.cos(_p / 2), math.sin(_p / 2)
            _cr2, _sr2 = math.cos(_r / 2), math.sin(_r / 2)
            # quat = Rz(yaw) * Rx(roll) * Ry(pitch)
            qw = _cy2*_cr2*_cp2 + _sy2*_sr2*_sp2
            qx = _cy2*_sr2*_cp2 - _sy2*_cr2*_sp2
            qy = _cy2*_cr2*_sp2 + _sy2*_sr2*_cp2
            qz = _sy2*_cr2*_cp2 - _cy2*_sr2*_sp2
            ang = 2 * math.acos(max(-1.0, min(1.0, qw)))
            sn = math.sqrt(max(1e-12, 1 - qw * qw))
            rot_field.setSFRotation([qx / sn, qy / sn, qz / sn, ang if ang > 1e-6 else 0.0])

        if _glog is not None and _sim_ms - _last_log >= 1000:
            _last_log = _sim_ms
            _glog.write(f"t={_sim_ms/1000:5.1f} x={pos_x:+.2f} "
                        f"phase={phase:.2f} L_knee={legs[3]:+.3f} "
                        f"L_hip={legs[0]:+.3f} L_ankle={legs[4]:+.3f} "
                        f"L_hipR={legs[1]:+.3f} L_hipY={legs[2]:+.3f}\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
