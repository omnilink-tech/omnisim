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

"""OmniQuad BATON deploy -- the SAME policy-switching library, on a THIRD body.

    walk (Shadowing RL policy, ONNX)  ->  stand (deterministic hold)  ->  walk

⭐ WHY THIS CONTROLLER EXISTS. go2_baton_deploy.py proved BATON survives leaving the humanoid:
no `world`, no torch, no arms, no crane, and a support model the library does not own. But ONE
quadruped is an anecdote -- it leaves open the reading that BATON was quietly re-fitted to the
Go2 (its joint names, its 0.30 m body, its 1.8 Hz clock, its KE=250 PD). OmniQuad is a different
quadruped: 12 DOF but a different naming scheme (NO "_joint" suffix), different limits, a
2x-taller stance (body_height 0.55 vs 0.30), a slower clock (1.4 Hz), and an order-of-magnitude
stiffer PD (KE=500/KD=60 vs 250/6). If BATON needed ONE line changed to take it, it would not
be a library.

It did not. `projects/policies/training/baton.py` is imported UNCHANGED, and the entire
robot-specific surface is the ~60-line `OmniQuadBatonHost` below -- which is the Go2 host with its
gait module and env prefix swapped. What is robot-specific is the HOST. What is general is the
arbiter.

THE STAND NEEDS NO POLICY. Four feet on the ground is statically stable, so the `stand`
specialist is a DETERMINISTIC HOLD -- `policy=None`, zero residual, tracking the nominal-stance
ghost exactly. The G1's stand is a *trained* specialist on a weight-bearing crane. The library
does not care, and that is the whole point.

⛔ NEVER TRUST THE EXIT CODE. A ghost lut replays well enough on its own that a zero-residual run
LOOKS like a good result -- it walks, it does not fall, and it scores a near-ceiling gmatch
because it IS the ghost. An entire Go2 head-to-head was once run and nearly believed on policies
that never loaded (onnxruntime missing from the CONTROLLER interpreter, which is a different
python than the engine's embed). So: a missing/unloadable walk policy is FATAL here, and the log
must show `ONNX loaded:` -- run_omniquad_baton_deploy.sh greps for exactly that string.

Env:
  OMNIQUAD_GHOST_LUT     walk ghost lut (REQUIRED)
  OMNIQUAD_STAND_LUT     stand ghost lut (REQUIRED -- the second specialist)
  OMNIQUAD_POLICY_ONNX   the walk champion (REQUIRED; absence is FATAL, see above)
  BATON_SCHEDULE     e.g. "walk:12,stand:6,walk:12"   (the library parses this)
  BATON_MORPH_TICKS  morph length in ticks (default 30)
  BATON_DS_GATE=1    gate the hand-over on FOUR-FOOT SUPPORT (see support_gate)
  OMNIQUAD_SUPPORT_TOL   max per-leg swing weight that still counts as "planted" (default 0.05)
  BATON_SPECIALISTS  assembled here from OMNIQUAD_STAND_LUT (a hold: empty ckpt)
  + the OMNIQUAD_* gait/ghost env of omniquad_shadow_deploy (ramp, steer, corridor, sig)
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = next(_p for _p in Path(__file__).resolve().parents
             if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists()
             or (_p / ".git").exists())
sys.path.append(str(_REPO))   # lowest priority: don't shadow the runtime `import omnisim`

from projects.policies.control.gait import omniquad_trot_gait as stg  # noqa: E402
from projects.policies.control.omniquad_motor_safety import (  # noqa: E402
    apply_realistic_limits, RateLimitedMotorBank,
)
from projects.policies.training import baton as BATON  # noqa: E402  -- THE library, unchanged

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


# ⛔ OmniQuad's joints carry NO "_joint" suffix (the Go2's do). Device = "<name>_motor".
URDF_LEGS = ("front_left", "front_right", "rear_left", "rear_right")
PARTS = ("hip_x", "hip_y", "knee")
JOINT_NAMES = [f"{leg}_{part}" for leg in URDF_LEGS for part in PARTS]
ACT_DIM = 12
NJ = 12
JOINT_LIMITS_LO = np.array([-1.50, -0.50, -1.20] * 4, dtype=np.float32)
JOINT_LIMITS_HI = np.array([+1.50, +3.13, -0.01] * 4, dtype=np.float32)


def _envf(key: str, default: float) -> float:
    v = os.environ.get(key)
    try:
        return float(v) if v not in (None, "") else default
    except ValueError:
        return default


def _envi(key: str, default: int) -> int:
    return int(_envf(key, default))


def _lut_interp(table: np.ndarray, phase: float, nb: int) -> np.ndarray:
    x = (phase % (2.0 * math.pi)) / (2.0 * math.pi) * nb
    b0 = int(math.floor(x)) % nb
    b1 = (b0 + 1) % nb
    f = x - math.floor(x)
    return table[b0] * (1.0 - f) + table[b1] * f


# ── THE HOST: the entire robot-specific surface of BATON on OmniQuad ─────────────
class OmniQuadBatonHost:
    """Answers BATON's three questions for a OmniQuad running as an ONNX controller."""

    # (1) WHICH reference channels this robot has, and where each lives in a ghost lut.
    #     No arm. No elbow. No base-attitude ghost. It DOES have a feedforward table, which
    #     the humanoid does not blend. The library never sees this map -- the host owns it.
    LUT_KEYS = {"glut": "leg_lut", "ref": "leg_lut", "ffdq": "ffdq_lut"}
    channels = tuple(LUT_KEYS)

    def tables_from_lut(self, gd: dict, vx: float) -> dict:
        return BATON.lut_tables(gd, vx, self.LUT_KEYS)

    def __init__(self, say):
        self._say = say
        self.phase_v = 0.0
        self.glut = None      # the command-centre leg track (blended)
        self.ref = None       # the SCORED reference pose track (blended)
        self.ffdq = None      # the ghost's declared feedforward (blended; a hold's is zeros)
        self.vx = 0.0

    def phase(self) -> float:
        return self.phase_v

    def log(self, msg: str) -> None:
        self._say(f"[omniquad_baton] {msg}\n")

    # (2) HOW a policy is evaluated -- here: onnxruntime, not torch.
    def load_policy(self, ckpt):
        import onnxruntime as ort
        # single-threaded on purpose: multi-threaded CPU reductions sum in
        # nondeterministic order; one thread keeps the same checkpoint
        # bit-stable across runs and machines at zero cost for a net this size.
        so = ort.SessionOptions()
        so.intra_op_num_threads = 1
        so.inter_op_num_threads = 1
        sess = ort.InferenceSession(str(ckpt), sess_options=so,
                                    providers=["CPUExecutionProvider"])
        self.log(f"ONNX loaded: {ckpt}")     # ⛔ the launcher greps for this exact string
        return sess, None                    # no recurrent state

    def act(self, sp, obs):
        a = sp.policy.run(None, {"obs": obs.reshape(1, -1)})[0][0]
        return np.clip(a, -1.0, 1.0).astype(np.float32)

    def reset_hidden(self, sp) -> None:
        return                               # feedforward policy: nothing to cool

    # ── THE QUADRUPED SUPPORT GATE (measured into existence on the Go2, 2026-07-13) ──────
    # BATON's naive quadruped default is `always_ok`: "a statically stable body has a support
    # polygon at all times, so there is no forbidden hand-over instant." A LIVE GO2 RUN REFUTED
    # IT. With schedule walk:12 the switch landed MID-SWING: the reference froze to a stance hold
    # while a diagonal pair was airborne, the robot tripped, flipped onto its back at t=12.83 s,
    # and then kept "walking" inverted -- still scoring gmatch 0.92, because a pose metric cannot
    # see that you are upside down. walk:10 and walk:8 happened to switch at benign phases and
    # survived. Phase dependence is exactly what a support gate is for.
    #
    # The mechanism is morphology, not tuning, so it ports as-is: OmniQuad's trot also runs
    # duty=0.6 > 0.5, which gives FOUR-FOOT SUPPORT WINDOWS twice per cycle (omniquad_trot_gait's own
    # docstring says so, and QS_PHASE is defined as the middle of one). So do not re-derive the
    # phase math -- ASK THE GAIT MODEL: hand over only when every leg's swing weight is ~0.
    # (t_since_start=None -> no stride ramp: the swing WEIGHTS are ramp-independent anyway, and
    # passing None keeps the gate a pure function of phase.)
    def support_gate(self, gp):
        def gate(phase: float, tol: float = 0.15) -> bool:
            try:
                _feet, swings = stg.foot_targets_np(phase, gp, t_since_start=None)
            except Exception:
                return True                  # never deadlock the arbiter on a model hiccup
            return float(np.max(np.abs(np.asarray(swings)))) <= _envf("OMNIQUAD_SUPPORT_TOL", 0.05)
        return gate

    # (3) WHERE the blended references go, in THIS runtime.
    def write_tables(self, eff: dict) -> None:
        self.glut = eff["glut"]
        self.ref = eff["ref"]
        self.ffdq = eff.get("ffdq")
        self.vx = eff["vx"]


def main() -> int:
    log_path = os.environ.get("OMNIQUAD_DEPLOY_LOG") or os.environ.get("OMNISIM_DEPLOY_LOG")
    side = open(log_path, "w", buffering=1) if log_path else None

    def say(msg):
        try:
            sys.stderr.write(msg); sys.stderr.flush()
        except Exception:
            pass
        if side is not None:
            try:
                side.write(msg); side.flush()
            except Exception:
                pass

    walk_lut = os.environ.get("OMNIQUAD_GHOST_LUT", "")
    stand_lut = os.environ.get("OMNIQUAD_STAND_LUT", "")
    policy = os.environ.get("OMNIQUAD_POLICY_ONNX", "")
    for nm, p in (("OMNIQUAD_GHOST_LUT", walk_lut), ("OMNIQUAD_STAND_LUT", stand_lut)):
        if not p or not Path(p).exists():
            say(f"[omniquad_baton] FATAL: {nm} missing/not found: {p!r}\n")
            return 1
    if not policy or not Path(policy).exists():
        # See the module docstring: a bare ghost walks and scores a near-ceiling gmatch.
        say(f"[omniquad_baton] FATAL: OMNIQUAD_POLICY_ONNX missing/not found: {policy!r}. Refusing to "
            f"run the bare ghost and report it as a Shadowing result.\n")
        return 2

    host = OmniQuadBatonHost(say)

    # BATON is armed from the env exactly as it is for the G1 and the Go2. The `stand`
    # specialist has an EMPTY checkpoint -> policy=None -> a deterministic hold (zero residual).
    # No training, no crane.
    os.environ.setdefault("BATON_SPECIALISTS", f"stand||{stand_lut}|0")
    os.environ.setdefault("BATON_SCHEDULE", "walk:12,stand:6,walk:12")
    os.environ.setdefault("BATON_DS_GATE", "1")   # gate on FOUR-FOOT SUPPORT (see support_gate)

    import json as _json
    gd = _json.loads(open(walk_lut).read())
    got = list(gd.get("joints") or [])
    if got != JOINT_NAMES:
        say(f"[omniquad_baton] FATAL: lut joint-order mismatch: {got} != {JOINT_NAMES}\n")
        return 1
    nb = int(gd["nb"])
    gsig = _envf("OMNIQUAD_GHOST_SIG", 0.35)
    res_scale = _envf("OMNIQUAD_ACT_SCALE", 0.15)
    use_ff = os.environ.get("OMNIQUAD_GHOST_FF", "").strip() == "1"

    # Gait params MUST equal the lut's own `gait` block (and the training run's): the lut is a
    # phase-folded recording, so a different duty/body_h moves the nominal and thus the corridor.
    gp = stg.GaitParams(vx=_envf("OMNIQUAD_GAIT_VX", 0.4),
                        freq=_envf("OMNIQUAD_GAIT_FREQ", float(gd.get("freq", 1.4))),
                        duty=_envf("OMNIQUAD_GAIT_DUTY", 0.6),
                        step_height=_envf("OMNIQUAD_GAIT_STEP_H", 0.06),
                        body_height=_envf("OMNIQUAD_GAIT_BODY_H", 0.55),
                        x0=_envf("OMNIQUAD_GAIT_X0", 0.0),
                        ramp_s=_envf("OMNIQUAD_GAIT_RAMP_S", 1.0))
    nominal = stg.standing_pose(gp).astype(np.float32)
    omega = 2.0 * math.pi * float(gd.get("freq", gp.freq))

    # the PRIMARY ("walk") specialist: the Shadowing champion + the walk ghost's own tables
    walk_pol, _ = host.load_policy(policy)
    primary_tables = host.tables_from_lut(gd, float(gd.get("vx", 0.0)))
    if not use_ff:
        primary_tables["ffdq"] = None

    st = BATON.setup(host, primary_tables=primary_tables,
                     load_policy=host.load_policy, geti=_envi)
    if st is None:
        say("[omniquad_baton] FATAL: BATON did not arm (no specialists)\n")
        return 1
    st.reg["walk"].policy = walk_pol          # the primary's policy (the host evaluates it)

    # ⛔ THE ffdq=None TRAP. BATON._blend is null-tolerant by design: if one side of a channel is
    # None it returns the OTHER side unchanged. So a `stand` ghost with NO ffdq_lut does not fade
    # the feedforward out -- it INHERITS the walk's, and the robot holds a stance while still
    # being pushed by the walk's torque offset. The Go2's stand ghost dodges this by carrying an
    # explicit ZEROS ffdq_lut of the same shape (BATON blends element-wise, so shape parity with
    # the walk lut is required anyway). Assert it rather than discover it in a video.
    if use_ff:
        for nm, sp in st.reg.items():
            if sp.primary:
                continue
            if sp.tables.get("ffdq") is None:
                say(f"[omniquad_baton] FATAL: OMNIQUAD_GHOST_FF=1 but specialist {nm!r} has no "
                    f"'ffdq_lut'. It would silently inherit the WALK's feedforward through the "
                    f"morph. Give the ghost an explicit zeros ffdq_lut (same nb x 12 shape).\n")
                return 1

    host.write_tables({**primary_tables})     # seed the tables before the first tick
    # THE morphology seam. BATON.gate_for("quadruped") deliberately fails closed because a
    # robot class cannot identify a trot/pace/crawl support window. The host knows its own
    # gait model and supplies the measured-safe four-foot-support gate here.
    gate = host.support_gate(gp)
    say(f"[omniquad_baton] armed: schedule={os.environ['BATON_SCHEDULE']} morph={st.morph} "
        f"corridor={res_scale} ff={'ON' if use_ff else 'off'} gate={gate.__name__}\n")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    dt = step_ms / 1000.0
    motors, sensors = [], []
    for jn in JOINT_NAMES:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            say(f"[omniquad_baton] missing motor {jn}_motor\n")
            return 1
        motors.append(m)
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors.append(s)
        except Exception:
            sensors.append(None)
    apply_realistic_limits(motors, max_torque_nm=_envf("OMNIQUAD_MAX_TORQUE_NM", 80.0),
                          max_vel_rad_s=_envf("OMNIQUAD_MAX_VEL_RAD_S", 20.0))
    bank = RateLimitedMotorBank(motors, dt, max_vel_rad_s=_envf("OMNIQUAD_TARGET_RATE_RAD_S", 1e6))
    try:
        self_node = robot.getSelf()
    except Exception:
        self_node = None

    bank.set_pose(nominal.tolist())
    for _ in range(max(1, int(_envf("OMNIQUAD_SETTLE_S", 1.5) / dt))):
        if robot.step(step_ms) == -1:
            return 0

    gait_t, sim_ms, tick = 0.0, 0, 0
    last_action = np.zeros(ACT_DIM, np.float32)
    last_q = None
    gm_n = 0
    gm_sum = 0.0
    fell = False
    seg_stats: dict[str, list] = {}

    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        tick += 1
        elapsed = sim_ms / 1000.0

        # ── THE LIBRARY. Identical call the G1's and the Go2's deploys make. ──
        host.phase_v = stg.QS_PHASE + omega * gait_t
        mode = BATON.mode_at(st, elapsed)
        BATON.step(host, st, mode, tick, gate=gate, geti=_envi)

        if self_node is not None:
            pos = self_node.getPosition() or [0, 0, 0]
            ori = self_node.getOrientation() or [1, 0, 0, 0, 1, 0, 0, 0, 1]
            vel = self_node.getVelocity() or [0] * 6
        else:
            pos, ori, vel = [0, 0, 0], [1, 0, 0, 0, 1, 0, 0, 0, 1], [0] * 6
        bx, by, bz = float(pos[0]), float(pos[1]), float(pos[2])
        roll = math.atan2(ori[7], ori[8])
        pitch = math.asin(max(-1.0, min(1.0, -ori[6])))
        # OmniQuad stands at 0.55 m (the Go2 at 0.30) -- fall floor 0.30, per omniquad_walk_deploy.
        if not fell and (bz < 0.30 or abs(roll) > 0.8 or abs(pitch) > 0.8):
            say(f"FALL@{elapsed:.2f}s bz={bz:.2f} roll={roll:.2f} pitch={pitch:.2f}\n")
            fell = True

        q = np.zeros(NJ, np.float32)
        for i, s in enumerate(sensors):
            try:
                q[i] = s.getValue() if s is not None else 0.0
            except Exception:
                q[i] = 0.0
        if last_q is None:
            last_q = q.copy()
        qd = (q - last_q) / dt
        last_q = q.copy()

        v_lin = np.array(vel[:3], np.float32)
        _R = np.array(ori, np.float32).reshape(3, 3)
        v_ang = (_R.T @ np.array(vel[3:6], np.float32)).astype(np.float32)
        proj_g = np.array([-ori[2], -ori[5], -ori[8]], np.float32)
        gait_obs = np.array([math.sin(host.phase_v), math.cos(host.phase_v)], np.float32)
        # wz_cmd = 0: BATON sequences MODES, not headings. (The heading-hold channel lives in
        # omniquad_shadow_deploy; mixing a steering PD into a hand-over demo would confound the
        # switch's effect with the steerer's.)
        obs = np.concatenate([v_lin, v_ang, proj_g, q - nominal, qd, last_action,
                              gait_obs, [np.float32(0.0)]]).astype(np.float32)
        obs = np.clip(np.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0), -10.0, 10.0)

        # every specialist on the same obs; applied action crossfades with the morph u.
        # the `stand` hold contributes ZEROS -> at u=1 on stand, the robot tracks the stance
        # ghost with no residual at all.
        action = BATON.act(host, st, obs, host.act(st.reg["walk"], obs))

        r_ramp = min(1.0, max(0.0, gait_t / gp.ramp_s)) if gp.ramp_s > 0 else 1.0
        ref_pose = nominal + r_ramp * (_lut_interp(host.ref, host.phase_v, nb) - nominal)
        q_model = nominal + r_ramp * (_lut_interp(host.glut, host.phase_v, nb) - nominal)
        if use_ff and host.ffdq is not None:
            q_model = q_model + r_ramp * _lut_interp(host.ffdq, host.phase_v, nb)

        gm = math.exp(-float(np.mean((q - ref_pose) ** 2)) / (gsig * gsig))
        gm_sum += gm; gm_n += 1
        seg = seg_stats.setdefault(st.target, [0, 0.0, 0.0])
        # the engine can hand back a NaN body velocity on a tick (seen live on the Go2: it
        # poisoned the segment mean into `vx_mean=+nan` while every per-second sample was
        # finite). A summary metric that one bad sample can silently destroy is not a metric.
        _vx = float(vel[0])
        seg[0] += 1; seg[1] += gm; seg[2] += (_vx if math.isfinite(_vx) else 0.0)

        q_cmd = np.clip(q_model.astype(np.float32) + action * res_scale,
                        JOINT_LIMITS_LO, JOINT_LIMITS_HI)
        last_action = action
        bank.set_pose(q_cmd.tolist())

        if sim_ms % 1000 < step_ms:
            say(f"[t={elapsed:.0f}s] mode={st.target:<5} u={st.u:.2f} sw={st.switches} "
                f"x={bx:+.2f} y={by:+.2f} bz={bz:.2f} roll={roll:+.2f} "
                f"vx={float(vel[0]):+.2f} gm={gm:.3f}\n")

        # the gait clock STOPS while standing (a hold has no cadence) and resumes on walk.
        gait_t += dt * (1.0 if st.target == "walk" else 0.0)

    say(f"BATON FINAL switches={st.switches} fell={fell} "
        f"gmatch_mean={gm_sum / max(gm_n, 1):.3f}\n")
    for nm, (n, g, v) in seg_stats.items():
        say(f"  segment {nm:<5} ticks={n:<6} gmatch={g / max(n, 1):.3f} "
            f"vx_mean={v / max(n, 1):+.3f}\n")
    if side is not None:
        side.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
