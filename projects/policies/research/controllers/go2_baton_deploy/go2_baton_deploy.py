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

"""Go2 BATON deploy -- the SAME policy-switching library that sequences the G1, on a quadruped.

    walk (Shadowing RL policy, ONNX)  ->  stand (deterministic hold)  ->  walk

⭐ WHY THIS CONTROLLER EXISTS. It is the falsifier for the claim "BATON is a library". BATON was
born inside `g1_walk_recipe.py`'s in-engine deploy hook, and every assumption it ever made was a
humanoid one: it blended an `arm` and an `elb` channel, it reached into `world._wr_*`, it ran
torch nets, and it only let a specialist go at DOUBLE SUPPORT. This host breaks every one of
those at once:

    no `world`   -- this is a Webots CONTROLLER, a separate OS process
    no torch     -- policies here are ONNX sessions
    no arms      -- a quadruped's channels are (glut, ref, ffdq); there is no elbow to blend
    no crane     -- quads carry no harness; a G1 BATON demo rides a weight-bearing rig
    no double-support gate -- a statically stable trotter has no forbidden hand-over instant

If BATON had needed ONE line changed to accommodate this, it would not be a library. It did not:
`projects/policies/training/baton.py` is imported here UNCHANGED, and the entire robot-specific
surface is the ~60-line `Go2BatonHost` below.

THE STAND NEEDS NO POLICY. Four feet on the ground is statically stable, so the `stand`
specialist is a DETERMINISTIC HOLD -- `policy=None`, zero residual, tracking the nominal-stance
ghost exactly (build_go2_stand_ghost.py). The G1's stand is a *trained* specialist on a crane.
The library does not care, and that is the whole point.

⛔ NEVER TRUST THE EXIT CODE. A ghost lut replays well enough on its own that a zero-residual run
LOOKS like a good result -- it walks, it does not fall, and it scores a near-ceiling gmatch
because it IS the ghost. An entire Go2 head-to-head was once run and nearly believed on policies
that never loaded (onnxruntime missing from the controller interpreter). So: a missing/unloadable
walk policy is FATAL here, and the log must show `ONNX loaded:`.

Env:
  GO2_GHOST_LUT     walk ghost lut (REQUIRED)
  GO2_STAND_LUT     stand ghost lut (REQUIRED -- the second specialist)
  GO2_POLICY_ONNX   the walk champion (REQUIRED; absence is FATAL, see above)
  BATON_SCHEDULE    e.g. "walk:12,stand:6,walk:12"   (the library parses this)
  BATON_MORPH_TICKS morph length in ticks (default 30)
  BATON_SPECIALISTS assembled here from GO2_STAND_LUT (a hold: empty ckpt)
  + the GO2_* gait/ghost env of go2_shadow_deploy (ramp, steer, corridor, sig)
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

from projects.policies.control.gait import go2_trot_gait as stg  # noqa: E402
from projects.policies.control.omniquad_motor_safety import (  # noqa: E402
    apply_realistic_limits, RateLimitedMotorBank,
)
from projects.policies.training import baton as BATON  # noqa: E402  -- THE library, unchanged

try:
    from omnisim import Supervisor as _Robot
except Exception:
    from omnisim import Robot as _Robot


LEGS = ("FL", "FR", "RL", "RR")
PARTS = ("hip", "thigh", "calf")
JOINT_NAMES = [f"{leg}_{part}_joint" for leg in LEGS for part in PARTS]
ACT_DIM = 12
NJ = 12
JOINT_LIMITS_LO = np.array([-1.0472, -0.5236, -2.7227] * 4, dtype=np.float32)
JOINT_LIMITS_HI = np.array([+1.0472, +3.1316, -0.83776] * 4, dtype=np.float32)


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


def _lut_wz(gd: dict) -> float:
    """A ghost lut's declared ACHIEVED yaw rate (rad/s), or 0.0 if it declares none.

    The walk and stand ghosts carry no `wz_meas` key (they do not turn) -> 0.0; the turn ghost
    declares its measured turn-in-place rate (0.5895 for go2_turn). Never crash on a missing or
    malformed value -- an absent/garbage wz is a zero command, not a fatal error.
    """
    try:
        return float(gd.get("wz_meas", 0.0) or 0.0)
    except (TypeError, ValueError):
        return 0.0


def _wz_by_specialist(walk_gd: dict) -> dict:
    """Map specialist name -> its ghost lut's declared `wz_meas`, mirroring BATON.setup's registry.

    The primary `walk` reads the already-parsed walk lut; the rest are parsed from the SAME
    BATON_SPECIALISTS (and legacy BATON_POLICY_B|BATON_LUT_B) env BATON.setup consumes, so the
    keys line up with `st.reg` / `st.out` / `st.target`. A specialist whose lut declares no
    `wz_meas` (walk, stand) -> 0.0, which is why a walk-only schedule feeds a bit-identical zero.
    """
    import json as _json
    wz = {"walk": _lut_wz(walk_gd)}
    specs = os.environ.get("BATON_SPECIALISTS", "")
    if not specs:                                    # legacy two-specialist ("stand") spelling
        bp, bl = os.environ.get("BATON_POLICY_B"), os.environ.get("BATON_LUT_B")
        if bp and bl:
            specs = "stand|%s|%s|0" % (bp, bl)
    for entry in specs.split(";"):
        if not entry.strip():
            continue
        pp = entry.split("|")
        if len(pp) < 3:
            continue
        nm, lu = pp[0].strip(), BATON.resolve_artifact(pp[2].strip())
        try:
            wz[nm] = _lut_wz(_json.loads(open(lu).read()))
        except Exception as exc:
            # An unreadable lut must never crash the deploy -- but it must not pass
            # UNNOTICED either: wz=0 means this specialist commands zero yaw rate, so
            # a "turn" silently stops turning and the run still reports PASS. That is
            # what a repo-relative path used to do here (the controller's cwd is its
            # own directory), which is why the path goes through resolve_artifact now.
            wz[nm] = 0.0
            print("[go2_baton] WARNING: specialist %r lut unreadable (%s: %s) -- "
                  "commanding wz=0 for it" % (nm, type(exc).__name__, lu), flush=True)
    return wz


# ── THE HOST: the entire robot-specific surface of BATON on a quadruped ──────
class Go2BatonHost:
    """Answers BATON's three questions for a Go2 running as an ONNX controller."""

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
        self._say(f"[go2_baton] {msg}\n")

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
        self.log(f"ONNX loaded: {ckpt}")
        return sess, None                    # no recurrent state

    def act(self, sp, obs):
        a = sp.policy.run(None, {"obs": obs.reshape(1, -1)})[0][0]
        return np.clip(a, -1.0, 1.0).astype(np.float32)

    def reset_hidden(self, sp) -> None:
        return                               # feedforward policy: nothing to cool

    # ── THE QUADRUPED SUPPORT GATE (measured into existence, 2026-07-13) ──────────────
    # BATON's naive quadruped default was `always_ok`: "a statically stable body has a support
    # polygon at all times, so there is no forbidden hand-over instant." A LIVE RUN REFUTED IT.
    # With schedule walk:12 the switch landed MID-SWING: the reference froze to a stance hold
    # while a diagonal pair was airborne, the Go2 tripped, flipped onto its back at t=12.83 s,
    # and then kept "walking" inverted -- still scoring gmatch 0.92, because a pose metric cannot
    # see that you are upside down. walk:10 and walk:8 happened to switch at benign phases and
    # survived. Phase dependence is exactly what a support gate is for.
    #
    # A trot with duty > 0.5 has FOUR-FOOT SUPPORT WINDOWS twice per cycle (go2_trot_gait's own
    # docstring says so). So do not re-derive the phase math -- ASK THE GAIT MODEL: hand over
    # only when every leg's swing weight is ~0, i.e. all four feet are planted.
    def support_gate(self, gp):
        def gate(phase: float, tol: float = 0.15) -> bool:
            try:
                _feet, swings = stg.foot_targets_np(phase, gp, t_since_start=None)
            except Exception:
                return True                  # never deadlock the arbiter on a model hiccup
            return float(np.max(np.abs(np.asarray(swings)))) <= _envf("GO2_SUPPORT_TOL", 0.05)
        return gate

    # (3) WHERE the blended references go, in THIS runtime.
    def write_tables(self, eff: dict) -> None:
        self.glut = eff["glut"]
        self.ref = eff["ref"]
        self.ffdq = eff.get("ffdq")
        self.vx = eff["vx"]


def main() -> int:
    log_path = os.environ.get("GO2_DEPLOY_LOG") or os.environ.get("OMNISIM_DEPLOY_LOG")
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

    walk_lut = os.environ.get("GO2_GHOST_LUT", "")
    stand_lut = os.environ.get("GO2_STAND_LUT", "")
    policy = os.environ.get("GO2_POLICY_ONNX", "")
    for nm, p in (("GO2_GHOST_LUT", walk_lut), ("GO2_STAND_LUT", stand_lut)):
        if not p or not Path(p).exists():
            say(f"[go2_baton] FATAL: {nm} missing/not found: {p!r}\n")
            return 1
    if not policy or not Path(policy).exists():
        # See the module docstring: a bare ghost walks and scores a near-ceiling gmatch.
        say(f"[go2_baton] FATAL: GO2_POLICY_ONNX missing/not found: {policy!r}. Refusing to run "
            f"the bare ghost and report it as a Shadowing result.\n")
        return 2

    host = Go2BatonHost(say)

    # BATON is armed from the env exactly as it is for the G1. The `stand` specialist has an
    # EMPTY checkpoint -> policy=None -> a deterministic hold (zero residual). No training.
    os.environ.setdefault("BATON_SPECIALISTS", f"stand||{stand_lut}|0")
    os.environ.setdefault("BATON_SCHEDULE", "walk:12,stand:6,walk:12")
    os.environ.setdefault("BATON_DS_GATE", "1")     # gate on FOUR-FOOT SUPPORT (see support_gate)

    import json as _json
    gd = _json.loads(open(walk_lut).read())
    got = list(gd.get("joints") or [])
    if got != JOINT_NAMES:
        say(f"[go2_baton] FATAL: lut joint-order mismatch: {got} != {JOINT_NAMES}\n")
        return 1
    nb = int(gd["nb"])
    gsig = _envf("GO2_GHOST_SIG", 0.35)
    res_scale = _envf("GO2_ACT_SCALE", 0.15)
    use_ff = os.environ.get("GO2_GHOST_FF", "").strip() == "1"

    gp = stg.GaitParams(vx=_envf("GO2_GAIT_VX", 0.4),
                        freq=_envf("GO2_GAIT_FREQ", float(gd.get("freq", 1.8))),
                        duty=_envf("GO2_GAIT_DUTY", 0.6),
                        step_height=_envf("GO2_GAIT_STEP_H", 0.05),
                        body_height=_envf("GO2_GAIT_BODY_H", 0.30),
                        x0=_envf("GO2_GAIT_X0", 0.0),
                        ramp_s=_envf("GO2_GAIT_RAMP_S", 1.0))
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
        say("[go2_baton] FATAL: BATON did not arm (no specialists)\n")
        return 1
    st.reg["walk"].policy = walk_pol          # the primary's policy (the host evaluates it)
    host.write_tables({**primary_tables})     # seed the tables before the first tick
    # THE WZ-COMMAND HOOK. obs slot 47 -- the trailing wz_cmd of the trainer's _build_obs_t
    # (skill.json calls it "slot 48", 1-indexed) -- must carry the active specialist's declared
    # yaw command. BATON sequences MODES not headings, so the walk champion never saw a non-zero
    # yaw command (its lut declares none -> 0.0); but the turn specialist's ghost DECLARES the
    # rate it achieves (wz_meas=0.5895) and the turn policy was trained to read it. Read each
    # specialist's wz_meas ONCE here (keyed by name, aligned with st.reg); blend it per tick with
    # the SAME morph u BATON.act uses for the action, so walk-only stays byte-identical and a
    # walk<->turn hand-over ramps the command smoothly.
    wz_by_spec = _wz_by_specialist(gd)
    # THE morphology seam. BATON.gate_for("quadruped") deliberately fails closed because a
    # robot class cannot identify a trot/pace/crawl support window. The host knows its own
    # gait model and supplies the measured-safe four-foot-support gate here.
    gate = host.support_gate(gp)
    _wz_note = "".join(f" wz[{k}]={v:.4f}" for k, v in wz_by_spec.items() if abs(v) > 1e-12)
    say(f"[go2_baton] armed: schedule={os.environ['BATON_SCHEDULE']} morph={st.morph} "
        f"corridor={res_scale} ff={'ON' if use_ff else 'off'} gate={gate.__name__}{_wz_note}\n")

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())
    dt = step_ms / 1000.0
    motors, sensors = [], []
    for jn in JOINT_NAMES:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            say(f"[go2_baton] missing motor {jn}_motor\n")
            return 1
        motors.append(m)
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
            sensors.append(s)
        except Exception:
            sensors.append(None)
    apply_realistic_limits(motors, max_torque_nm=_envf("GO2_MAX_TORQUE_NM", 50.0),
                          max_vel_rad_s=_envf("GO2_MAX_VEL_RAD_S", 30.0))
    bank = RateLimitedMotorBank(motors, dt, max_vel_rad_s=_envf("GO2_TARGET_RATE_RAD_S", 1e6))
    try:
        self_node = robot.getSelf()
    except Exception:
        self_node = None

    bank.set_pose(nominal.tolist())
    for _ in range(max(1, int(_envf("GO2_SETTLE_S", 1.5) / dt))):
        if robot.step(step_ms) == -1:
            return 0

    gait_t, sim_ms, tick = 0.0, 0, 0
    last_action = np.zeros(ACT_DIM, np.float32)
    last_q = None
    gm_sum = gm_n = 0
    gm_sum = 0.0
    fell = False
    seg_stats: dict[str, list] = {}

    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        tick += 1
        elapsed = sim_ms / 1000.0

        # ── THE LIBRARY. Identical call the G1's deploy makes. ──
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
        if not fell and (bz < 0.18 or abs(roll) > 0.8 or abs(pitch) > 0.8):
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
        # obs slot 47 (wz_cmd): the morph-blended wz_meas of the active specialists, using the
        # EXACT crossfade BATON.act applies to the action -- outgoing = (st.out or st.target),
        # incoming = st.target, weight st.u. Walk-only => both 0.0 => byte-identical obs.
        _out_spec = st.out or st.target
        wz_cmd = ((1.0 - st.u) * wz_by_spec.get(_out_spec, 0.0)
                  + st.u * wz_by_spec.get(st.target, 0.0))
        obs = np.concatenate([v_lin, v_ang, proj_g, q - nominal, qd, last_action,
                              gait_obs, [np.float32(wz_cmd)]]).astype(np.float32)
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
        # the engine can hand back a NaN body velocity on a tick (seen live: it poisoned the
        # segment mean into `vx_mean=+nan` while every per-second sample was finite). A summary
        # metric that can be silently destroyed by one bad sample is not a metric.
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
        # turn is a gait too (turn-in-place trot, go2_turn skill) -- freezing the
        # clock outside "walk" degenerated a turn to a twisted stance.
        gait_t += dt * (1.0 if st.target in ("walk", "turn") else 0.0)

    say(f"BATON FINAL switches={st.switches} fell={fell} gmatch_mean={gm_sum / max(gm_n, 1):.3f}\n")
    for nm, (n, g, v) in seg_stats.items():
        say(f"  segment {nm:<5} ticks={n:<6} gmatch={g / max(n, 1):.3f} vx_mean={v / max(n, 1):+.3f}\n")
    if side is not None:
        side.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
