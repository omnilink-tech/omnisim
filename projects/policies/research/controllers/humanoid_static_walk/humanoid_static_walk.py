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

"""Generic DETERMINISTIC humanoid QUASI-STATIC walk (no RL).

The walking sibling of ``humanoid_stand_deploy.py``. It reuses that controller's
PROVEN control system VERBATIM and adds ONE thing: a slow, phase-clocked,
statically-stable gait layered on the squat nominal.

Reused verbatim from the stand:
  * the robot is a ``Supervisor`` (falls back to ``Robot``);
  * every joint is held by the stiff Newton position PD set GLOBALLY by the
    launcher (``OMNISIM_NEWTON_TARGET_KE`` / ``_KD``) -- we only ever call
    ``motor.setPosition(target)``, never a per-motor PD;
  * the SAME ``HUMANOID_STAND_SPEC`` JSON describes the robot (joint_order, the
    statically-stable squat ``nominal``, the per-leg joint map under
    ``capture_step.legs``, ``ank_bias``, ``upright`` thresholds);
  * the same warm-reload, env fingerprint, base-state read, and side-log.

Why QUASI-STATIC (and not the dynamic shadow walk in ``humanoid_walk_deploy``):
the repo's deterministic-brain work (docs/developer/g1-deterministic-brain.md)
proved a *dynamic* reactive walk caps at ~2.3 s because the fall timescale
sqrt(z/g) ~= 0.28 s is shorter than a step. A *quasi-static* gait sidesteps that
entirely: it is slow and keeps the CoM over the stance foot at EVERY instant, so
the robot is always in a stand-like statically-stable configuration -- the exact
property the deterministic stand already guarantees. It looks like a careful
shuffle, not an athletic stride, but it is deterministic and robust.

Each leg cycles SWING -> STANCE:
  * SWING: lift the foot (knee), sweep the hip from behind to ahead -> the foot
    is placed a step forward. The OTHER leg is in stance and the body is leaned
    onto it (lateral weight-shift), so single support stays statically stable.
  * STANCE: foot planted flat; the hip sweeps from ahead to behind as the pelvis
    advances over it. Propulsion is STANCE-foot progression, NOT push-off (the
    deterministic-brain finding). The ankle counter-rotates to keep the foot flat.
The lateral lean is a smooth cosine phased so the body is over the stance leg
whenever the other leg swings.

Every gait amplitude is env-overridable (WALK_*) so the gait can be swept
headless (forward progress vs no-fall) without editing code. With all gait
amplitudes 0 this controller reduces EXACTLY to the deterministic stand.

Gait env vars (spec ``walk`` block supplies defaults):
    WALK_PERIOD      full-stride period, s (slow => quasi-static)
    WALK_STRIDE      hip-pitch swing amplitude, rad (forward step size)
    WALK_LIFT        swing-leg knee lift, rad
    WALK_LATERAL     lateral weight-shift (ankle-roll), rad
    WALK_HIP_ROLL    hip-roll share of the lateral shift, rad
    WALK_FWD_LEAN    constant forward CoM bias (hip fwd + ankle comp), rad
    WALK_DUTY        swing fraction of each leg's cycle (rest is stance)
    WALK_LAT_LEAD    phase lead of the lateral lean vs swing mid (cycles)
    WALK_LAT_SIGN    +1/-1 sign of the lateral lean (tune once per robot)
    WALK_STRIDE_SIGN +1/-1 sign of forward progress (tune once per robot)
    WALK_RAMP_S      ramp gait amplitudes 0->full over this many s
    WALK_START_S     stand still this long before walking
    WALK_ANKLE_FLAT  fraction the ankle counter-rotates the hip sweep (flat foot)
    WALK_STOP_AT     hold the stand after this forward distance, m (0=off)
    WALK_LEAN        1 = enable the stand's reactive fore/aft trim on top (off)
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


def _envf(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


def _ori_matrix_to_rpy(o):
    return (math.atan2(o[7], o[8]),
            math.asin(max(-1.0, min(1.0, -o[6]))),
            math.atan2(o[3], o[0]))


def _load_spec() -> dict:
    p = os.environ.get("HUMANOID_STAND_SPEC")
    if not p:
        raise SystemExit("[swalk] HUMANOID_STAND_SPEC not set (path to robot spec JSON)")
    p = Path(p)
    if not p.is_absolute():
        p = _REPO / p
    with open(p) as f:
        return json.load(f)


def _smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def main() -> int:
    spec = _load_spec()
    name = spec.get("name", "humanoid")
    joint_order = list(spec["joint_order"])
    nominal_map = {j: 0.0 for j in joint_order}
    nominal_map.update({k: float(v) for k, v in spec.get("nominal", {}).items()})

    ankle_pitch = list(spec.get("ankle_pitch_joints", []))
    ankle_roll = list(spec.get("ankle_roll_joints", []))
    hip_pitch = list(spec.get("hip_pitch_joints", []))

    ank_bias = _envf("HSTAND_ANK_BIAS", spec.get("ank_bias", 0.0))
    hip_bias = _envf("HSTAND_HIP_BIAS", spec.get("hip_bias", 0.0))
    for jn in ankle_pitch:
        if jn in nominal_map:
            nominal_map[jn] += ank_bias
    for jn in hip_pitch:
        if jn in nominal_map:
            nominal_map[jn] += hip_bias
    arm_hold = set(spec.get("arm_hold", []))
    upright = spec.get("upright", {})
    bz_min = float(upright.get("bz_min", 0.45))
    roll_max = float(upright.get("roll_max", 0.8))
    pitch_max = float(upright.get("pitch_max", 0.8))
    settle_s = _envf("HSTAND_SETTLE_S", spec.get("settle_s", 0.2))

    # ── Per-leg joint map (reuse the stand spec's capture_step.legs) ──
    legs = (spec.get("capture_step") or {}).get("legs") or {}
    if not legs:
        raise SystemExit("[swalk] spec has no capture_step.legs map (per-leg joint names)")
    LJ = legs.get("left", {})
    RJ = legs.get("right", {})

    # ── Gait parameters (env override > spec.walk > built-in default) ──
    walk = spec.get("walk") or {}

    def gp(env, key, dflt):
        return _envf(env, walk.get(key, dflt))

    period = gp("WALK_PERIOD", "period", 2.6)
    stride = gp("WALK_STRIDE", "stride", 0.16)
    lift = gp("WALK_LIFT", "lift", 0.34)
    lateral = gp("WALK_LATERAL", "lateral", 0.11)
    hip_roll_amp = gp("WALK_HIP_ROLL", "hip_roll", 0.06)
    fwd_lean = gp("WALK_FWD_LEAN", "fwd_lean", 0.03)
    duty = gp("WALK_DUTY", "duty", 0.32)
    lat_lead = gp("WALK_LAT_LEAD", "lat_lead", 0.06)
    lat_sign = gp("WALK_LAT_SIGN", "lat_sign", 1.0)
    stride_sign = gp("WALK_STRIDE_SIGN", "stride_sign", 1.0)
    ramp_s = gp("WALK_RAMP_S", "ramp_s", 2.0)
    ankle_flat = gp("WALK_ANKLE_FLAT", "ankle_flat", 0.9)
    stop_at = gp("WALK_STOP_AT", "stop_at", 0.0)
    start_s = gp("WALK_START_S", "start_s", 1.2)
    # ── SHUFFLE mode (no-lift, always double support) ──
    # The G1's narrow feet make single-support static balance infeasible
    # open-loop (a lifted foot tips it). The shuffle keeps BOTH feet on the
    # ground at all times: the advancing foot slides forward (knee extends to
    # keep it at ground level) while only PARTIALLY unweighted, so the CoM never
    # leaves the support polygon. Forward progress is stance-foot progression.
    shuffle = os.environ.get("WALK_SHUFFLE", str(walk.get("shuffle", 1))).strip() not in ("0", "")
    knee_slide = gp("WALK_KNEE_SLIDE", "knee_slide", 0.7)   # knee-extend per hip slide (foot stays down)
    stance_width = gp("WALK_STANCE_WIDTH", "stance_width", 0.06)  # base hip-roll abduction (widen base)
    squat_extra = gp("WALK_SQUAT_EXTRA", "squat_extra", 0.10)     # extra knee flex (lower CoM)

    # Bake the wider + deeper WALK STANCE into the home pose so the robot settles
    # into it (bigger support polygon + lower CoM = more statically stable for a
    # double-support shuffle). Keep the foot flat: extra knee is matched by toe-up
    # ankle. hip-roll abduction widens the base (sign mirrored L/R).
    if stance_width:
        if LJ.get("hip_roll") in nominal_map:
            nominal_map[LJ["hip_roll"]] += stance_width
        if RJ.get("hip_roll") in nominal_map:
            nominal_map[RJ["hip_roll"]] -= stance_width
    if squat_extra:
        for _LL in (LJ, RJ):
            if _LL.get("knee") in nominal_map:
                nominal_map[_LL["knee"]] += squat_extra
            if _LL.get("ankle_pitch") in nominal_map:
                nominal_map[_LL["ankle_pitch"]] -= squat_extra

    side_log_path = os.environ.get("HUMANOID_STAND_LOG") or os.environ.get("OMNISIM_DEPLOY_LOG")
    side_log = open(side_log_path, "w", buffering=1) if side_log_path else None

    def say(msg: str) -> None:
        sys.stderr.write(msg)
        sys.stderr.flush()
        if side_log is not None:
            side_log.write(msg)
            side_log.flush()

    say(f"[swalk:{name}] QUASI-STATIC walk mode={'SHUFFLE' if shuffle else 'LIFT'}: "
        f"period={period:.2f}s stride={stride:.3f} lateral={lateral:.3f} duty={duty:.2f} "
        f"fwd_lean={fwd_lean:.3f} knee_slide={knee_slide:.2f} width={stance_width:.3f} "
        f"squat_extra={squat_extra:.2f} lat_sign={lat_sign:+.0f} stride_sign={stride_sign:+.0f}\n")

    # Reactive fore/aft trim (the stand's lean), OFF by default while walking so it
    # cannot fight the deliberate gait pitch. Opt in with WALK_LEAN=1.
    lean = spec.get("lean") or {}
    lean_on = os.environ.get("WALK_LEAN", "0").strip() != "0"
    lean_kv = _envf("HSTAND_LEAN_KV", lean.get("kv", 0.14))
    lean_kp = _envf("HSTAND_LEAN_KP", lean.get("kp", 1.6))
    lean_kd = _envf("HSTAND_LEAN_KD", lean.get("kd", 0.25))
    lean_hip = _envf("HSTAND_LEAN_HIP", lean.get("hip", 0.35))
    lean_clamp = _envf("HSTAND_LEAN_CLAMP", lean.get("clamp", 0.30))
    lean_db = _envf("HSTAND_LEAN_DEADBAND", lean.get("deadband", 0.008))
    lean_smooth = _envf("HSTAND_LEAN_SMOOTH", lean.get("smooth", 1.0))

    robot = _Robot()

    _warm = os.environ.get("HSTAND_WARMUP_RELOAD", "0").strip() != "0"
    if _warm:
        try:
            _bridge = _REPO / "projects" / "samples" / "demos" / "controllers" / "omnilink_arm_bridge"
            if str(_bridge) not in sys.path:
                sys.path.insert(0, str(_bridge))
            from omnilink_arm_bridge import warmup_reload as _warmup_reload
            _warmup_reload(robot)
            say(f"[swalk:{name}] warmup_reload done\n")
        except Exception as e:
            say(f"[swalk:{name}] warmup_reload skipped ({e})\n")
            _warm = False

    # Pre-first-step: the engine finalizes the physics backend on the FIRST
    # tick, so no verdict exists yet -- pre_step=True draws a neutral label and
    # the real verdict is redrawn after the settle loop below.
    _envfp = None
    _envfp_fp = {}
    try:
        if str(_REPO) not in sys.path:
            sys.path.append(str(_REPO))
        from projects.policies.common import env_fingerprint as _envfp
        _envfp_fp = _envfp.report(robot, say, warm=_warm, repo_root=_REPO,
                                  pre_step=True)
    except Exception as _e:
        say(f"[swalk:{name}] env fingerprint skipped ({_e})\n")

    step_ms = int(robot.getBasicTimeStep())
    step_dt = step_ms / 1000.0

    motors = {}
    missing = []
    for jn in joint_order:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            missing.append(jn)
            motors[jn] = None
            continue
        motors[jn] = m
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
        except Exception:
            pass
        if jn in arm_hold:
            try:
                m.setAvailableTorque(120.0)
            except Exception:
                pass
            try:
                m.setControlPID(60.0, 0.0, 2.0)
            except Exception:
                pass
    if missing:
        say(f"[swalk:{name}] WARN missing {len(missing)} motors: {missing[:6]}"
            f"{' ...' if len(missing) > 6 else ''}\n")

    for jn in joint_order:
        if motors[jn] is not None:
            motors[jn].setPosition(float(nominal_map[jn]))

    self_node = None
    try:
        self_node = robot.getSelf()
    except Exception as e:
        say(f"[swalk:{name}] getSelf failed: {e}\n")

    n_settle = max(2, int(settle_s / step_dt))
    for _si in range(n_settle):
        if robot.step(step_ms) == -1:
            return 0

    # World has ticked -> the backend verdict (sidecar + finalise line) exists;
    # resolve the deferred fingerprint and redraw the physics label for real.
    if _envfp is not None and _envfp_fp.get("pending"):
        try:
            _envfp_fp = _envfp.report(robot, say, warm=_warm, repo_root=_REPO)
        except Exception as _e:
            say(f"[swalk:{name}] env fingerprint recheck skipped ({_e})\n")

    x0 = y0 = 0.0
    if self_node is not None:
        try:
            _p = self_node.getPosition() or [0, 0, 0]
            _o = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
            x0, y0 = float(_p[0]), float(_p[1])
            _r, _pi, _ = _ori_matrix_to_rpy(_o)
            say(f"[swalk:{name}] settle done: bz={_p[2]:.3f} "
                f"roll={_r:+.3f} pitch={_pi:+.3f} x0={x0:+.3f}\n")
        except Exception:
            pass

    def leg_offsets(local, swinging):
        """Per-leg (hip_pitch, knee, ankle_pitch) offsets from the squat nominal,
        for that leg's cycle phase ``local`` in [0,1). Hip-pitch sign: the squat
        hip nominal is NEGATIVE = thigh forward, so 'foot ahead' = -stride and
        'foot behind' = +stride.

        LIFT mode swings the foot through the air (knee bump). SHUFFLE mode keeps
        the foot on the ground throughout: as the hip sweeps the thigh forward
        (hip more negative) the knee EXTENDS (knee_slide*hip) so the foot tip
        stays near ground level and just slides forward; the ankle keeps it flat."""
        if swinging:
            s = local / max(duty, 1e-3)               # 0..1 through swing/advance
            hip = stride * (1.0 - 2.0 * s)            # behind (+) -> ahead (-)
            if shuffle:
                knee = knee_slide * hip               # hip fwd (neg) -> knee extends (neg) -> foot stays down
                ank = -(hip + knee) * ankle_flat      # flat-foot constraint
            else:
                knee = lift * math.sin(math.pi * s)   # lift mid-swing, 0 at ends
                ank = -hip * ankle_flat * 0.5
        else:
            s = (local - duty) / max(1.0 - duty, 1e-3)  # 0..1 through stance
            hip = stride * (2.0 * s - 1.0)            # ahead (-) -> behind (+): pelvis advances
            if shuffle:
                # PLANTED + WEIGHTED foot: keep the knee CONSTANT. Modulating a
                # weighted knee straightens the leg against the ground = push-off,
                # which LAUNCHES the body (the failure mode). The pelvis advances
                # purely from the hip sweep over the planted foot; the ankle
                # counter-rotates so the planted sole stays flat.
                knee = 0.0
                ank = -hip * ankle_flat
            else:
                knee = 0.0
                ank = -hip * ankle_flat               # foot stays flat through the sweep
        return hip * stride_sign, knee, ank * stride_sign

    sim_ms = settle_s * 1000.0
    last_log_ms = 0.0
    survived = 0
    fell_at = None
    gait_t = 0.0
    walking = True
    amp = 0.0
    peak_tilt = 0.0
    pitch_lp = 0.0
    lean_vx_lp = 0.0
    lean_pr_lp = 0.0

    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        t = sim_ms / 1000.0

        targets = dict(nominal_map)

        bx = by = bz = roll = pitch = 0.0
        vx = vy = 0.0
        pitch_rate = 0.0
        if self_node is not None:
            try:
                pos = self_node.getPosition() or [0.0, 0.0, 0.0]
                ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
                bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
                roll, pitch, _ = _ori_matrix_to_rpy(ori)
                vel6 = self_node.getVelocity() or [0.0] * 6
                w0, w1, w2 = float(vel6[3]), float(vel6[4]), float(vel6[5])
                pitch_rate = ori[1] * w0 + ori[4] * w1 + ori[7] * w2
                vx, vy = float(vel6[0]), float(vel6[1])
            except Exception:
                pass

        dx = bx - x0
        dy = by - y0

        if stop_at > 0.0 and abs(dx) >= stop_at:
            walking = False

        # Amplitude schedule: stand for start_s, then ease the gait in over ramp_s.
        if t < start_s or not walking:
            amp = max(0.0, amp - step_dt / 0.5) if not walking else 0.0
        else:
            gait_t += step_dt
            amp = _smoothstep((t - start_s) / max(ramp_s, 1e-3))

        if amp > 1e-4:
            phi = (gait_t / max(period, 1e-3)) % 1.0
            l_local = phi
            r_local = (phi + 0.5) % 1.0
            l_hip, l_knee, l_ank = leg_offsets(l_local, l_local < duty)
            r_hip, r_knee, r_ank = leg_offsets(r_local, r_local < duty)

            lat = lateral * math.cos(2.0 * math.pi * (phi - duty * 0.5 + lat_lead)) * lat_sign
            hr = hip_roll_amp * math.cos(2.0 * math.pi * (phi - duty * 0.5 + lat_lead)) * lat_sign
            fl = fwd_lean

            def addj(j, v):
                if j and j in targets:
                    targets[j] = nominal_map[j] + amp * v

            addj(LJ.get("hip_pitch"), l_hip - fl)
            addj(RJ.get("hip_pitch"), r_hip - fl)
            addj(LJ.get("knee"), l_knee)
            addj(RJ.get("knee"), r_knee)
            addj(LJ.get("ankle_pitch"), l_ank + fl * ankle_flat)
            addj(RJ.get("ankle_pitch"), r_ank + fl * ankle_flat)
            addj(LJ.get("ankle_roll"), lat)
            addj(RJ.get("ankle_roll"), lat)
            addj(LJ.get("hip_roll"), hr)
            addj(RJ.get("hip_roll"), -hr)

        if lean_on and self_node is not None:
            pitch_lp += 0.02 * (pitch - pitch_lp)
            lean_vx_lp += lean_smooth * (vx - lean_vx_lp)
            lean_pr_lp += lean_smooth * (pitch_rate - lean_pr_lp)
            fwd = (lean_kv * lean_vx_lp + lean_kd * (-lean_pr_lp)
                   + lean_kp * (-(pitch - pitch_lp)))
            if abs(fwd) <= lean_db:
                fwd = 0.0
            else:
                fwd = math.copysign(abs(fwd) - lean_db, fwd)
            fwd = float(np.clip(fwd, -lean_clamp, lean_clamp))
            for jn in ankle_pitch:
                if jn in targets:
                    targets[jn] -= fwd
            for jn in hip_pitch:
                if jn in targets:
                    targets[jn] += fwd * lean_hip

        for jn in joint_order:
            if motors[jn] is not None:
                motors[jn].setPosition(float(targets[jn]))

        peak_tilt = max(peak_tilt, abs(roll), abs(pitch))
        up = (bz > bz_min and abs(roll) < roll_max and abs(pitch) < pitch_max)
        if up:
            survived += 1
        elif fell_at is None and self_node is not None:
            fell_at = t

        if sim_ms - last_log_ms >= 1000:
            tag = "OK" if up else (f"FALL@{fell_at:.2f}s" if fell_at else "?")
            spd = dx / max(t - start_s, 1e-3) if t > start_s else 0.0
            say(f"[swalk:{name}] t={t:5.1f}s bz={bz:.3f} roll={roll:+.3f} "
                f"pitch={pitch:+.3f} dx={dx:+.3f} dy={dy:+.3f} "
                f"v={spd:+.3f}m/s peakTilt={peak_tilt:.3f} {tag} "
                f"{'WALK' if walking else 'STAND'}\n")
            last_log_ms = sim_ms
            peak_tilt = 0.0

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except BaseException as _e:
        import traceback
        _lp = os.environ.get("HUMANOID_STAND_LOG") or os.environ.get("OMNISIM_DEPLOY_LOG")
        _tb = traceback.format_exc()
        sys.stderr.write(_tb)
        sys.stderr.flush()
        if _lp:
            try:
                with open(_lp, "a", buffering=1) as _f:
                    _f.write("[swalk] FATAL:\n" + _tb)
            except Exception:
                pass
        sys.exit(1)
