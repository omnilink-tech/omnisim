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

"""Standing-basin test for the foot-redesign experiment.

Measures the DURABLE-STAND quantity directly: with the deployed deterministic stand
pose held by a stiff position servo (NO balance feedback, NO gait), how large a push
(an instantaneous base-velocity kick) can the PASSIVE stand absorb before it tips?

That is exactly the cube-defense question. The CoP-moment hypothesis predicts a longer
foot widens the forward basin (toe reach sets max restoring ankle moment) and a wider
foot widens the lateral basin.

Per model it sweeps forward (+x) and lateral (+y) push magnitudes and reports the
largest push survived for `--hold` seconds. Same servo gains (ke=400/kd=60) and same
nominal pose for every model -> a fair morphology A/B.

  python -u stand_basin.py --model models/g1_orig_legs.mjcf.xml --robot g1
  python -u stand_basin.py --model models/g1_bigfoot_legs.mjcf.xml --robot g1
"""
from __future__ import annotations
import argparse
import math
from pathlib import Path

import numpy as np
import mujoco
import warp as wp           # noqa: F401  (imported for side effects / parity with harness)
import mujoco_warp as mjw

HERE = Path(__file__).resolve().parent
REPO = next(p for p in HERE.parents if (p / "AGENTS.md").exists())

# deployed deterministic-stand nominal (humanoid_stand_deploy/specs/g1.json)
G1_NOM = {"left_hip_pitch_joint": -0.30, "left_knee_joint": 0.52, "left_ankle_pitch_joint": -0.23 - 0.06,
          "right_hip_pitch_joint": -0.30, "right_knee_joint": 0.52, "right_ankle_pitch_joint": -0.23 - 0.06,
          "left_shoulder_roll_joint": 0.2, "right_shoulder_roll_joint": -0.2}
# H1 deterministic stand (specs/h1.json): hip -0.30 knee 0.60 ankle -0.30, ank_bias -0.06
H1_NOM = {"left_hip_pitch_joint": -0.30, "left_knee_joint": 0.60, "left_ankle_joint": -0.30 - 0.06,
          "right_hip_pitch_joint": -0.30, "right_knee_joint": 0.60, "right_ankle_joint": -0.30 - 0.06}


def rpy(q):
    qw, qx, qy, qz = q
    r = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    p = math.asin(max(-1, min(1, 2 * (qw * qy - qz * qx))))
    return r, p


def set_gains(mjm, ke, kd):
    for a in range(mjm.nu):
        nm = mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or ""
        if nm.endswith("_pos"):
            mjm.actuator_gainprm[a, 0] = ke; mjm.actuator_biasprm[a, 1] = -ke
        elif nm.endswith("_vel"):
            mjm.actuator_gainprm[a, 0] = kd; mjm.actuator_biasprm[a, 2] = -kd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--robot", choices=["g1", "h1"], default="g1")
    ap.add_argument("--ke", type=float, default=400.0)
    ap.add_argument("--kd", type=float, default=60.0)
    ap.add_argument("--hold", type=float, default=3.0)
    ap.add_argument("--sub", type=int, default=8)
    # optional REACTIVE ankle lean (the humanoid_stand_deploy mechanism): the real
    # controller leans the ankle back against a forward fall. A bigger foot gives this
    # lean more CoP room, so the forward basin should widen with foot size when --lean.
    ap.add_argument("--lean", action="store_true")
    ap.add_argument("--lean-kv", type=float, default=0.10)   # forward vel -> ankle lean
    ap.add_argument("--lean-kp", type=float, default=0.6)    # pitch -> ankle lean
    ap.add_argument("--lean-kd", type=float, default=0.05)   # pitch rate -> ankle lean
    ap.add_argument("--lean-lim", type=float, default=0.30)  # clamp (rad)
    ap.add_argument("--lean-sign", type=float, default=-1.0)
    args = ap.parse_args()

    nom = G1_NOM if args.robot == "g1" else H1_NOM
    mjm = mujoco.MjModel.from_xml_path(str((REPO / args.model) if not Path(args.model).is_absolute()
                                           and not (HERE / args.model).exists() else (HERE / args.model)))
    set_gains(mjm, args.ke, args.kd)
    nq, nv, nu = mjm.nq, mjm.nv, mjm.nu

    # actuated hinges, [pos,vel] interleaved in hinge order
    hinge = [j for j in range(mjm.njnt) if mjm.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE]
    name2qpos, name2ctrl = {}, {}
    for rank, j in enumerate(hinge):
        nm = mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_JOINT, j)
        name2qpos[nm] = int(mjm.jnt_qposadr[j]); name2ctrl[nm] = 2 * rank

    # build the standing qpos + matching position targets
    mjd = mujoco.MjData(mjm)
    q0 = mjd.qpos.copy()
    q0[3:7] = [1, 0, 0, 0]
    for nm, v in nom.items():
        if nm in name2qpos:
            q0[name2qpos[nm]] = v
    # drop the base so the lowest foot geom sole rests on z=0
    q0[0:3] = [0, 0, 1.2]
    mjd.qpos[:] = q0; mujoco.mj_forward(mjm, mjd)
    foot_geoms = [g for g in range(mjm.ngeom)
                  if (mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith("shape_")
                  and mjm.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX]
    # lowest box bottom z across all geoms (proxy for sole height)
    zmin = min(mjd.geom_xpos[g][2] - mjm.geom_size[g][2] for g in range(mjm.ngeom)
               if mjm.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and
               (mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith("shape_"))
    q0[2] += (0.002 - zmin)  # sole 2 mm above ground, settle onto it

    ctrl0 = np.zeros(nu)
    for nm, c in name2ctrl.items():
        ctrl0[c] = nom.get(nm, 0.0)

    rm = mjw.put_model(mjm)
    # ankle-pitch control indices + nominal for the reactive lean
    ap_names = (["left_ankle_pitch_joint", "right_ankle_pitch_joint"] if args.robot == "g1"
                else ["left_ankle_joint", "right_ankle_joint"])
    ap_ctrl = [name2ctrl[n] for n in ap_names if n in name2ctrl]
    ap_nom = [nom.get(n, 0.0) for n in ap_names if n in name2ctrl]

    def apply_lean(data):
        """Reactive ankle lean against forward fall (humanoid_stand_deploy mechanism)."""
        q = data.qpos.numpy()[0]; v = data.qvel.numpy()[0]
        _, p = rpy(q[3:7]); vx = float(v[0]); wy = float(v[4])
        lean = args.lean_sign * max(-args.lean_lim, min(args.lean_lim,
                                    args.lean_kv * vx + args.lean_kp * p + args.lean_kd * wy))
        c = data.ctrl.numpy().copy()
        for ci, nomv in zip(ap_ctrl, ap_nom):
            c[0, ci] = nomv + lean
        data.ctrl.assign(c)

    def trial(push_x, push_y, settle=0.4):
        d = mujoco.MjData(mjm); d.qpos[:] = q0; mujoco.mj_forward(mjm, d)
        data = mjw.put_data(mjm, d, nworld=1, njmax=256, nconmax=256)
        c = data.ctrl.numpy().copy(); c[0, :] = ctrl0; data.ctrl.assign(c)
        dt = mjm.opt.timestep
        n_settle = int(settle / (args.sub * dt))
        n_hold = int(args.hold / (args.sub * dt))
        # settle in place
        for _ in range(n_settle):
            if args.lean:
                apply_lean(data)
            for _ in range(args.sub):
                mjw.step(rm, data)
        # apply the push (instantaneous base linear velocity kick)
        v = data.qvel.numpy().copy(); v[0, 0] += push_x; v[0, 1] += push_y; data.qvel.assign(v)
        z0 = float(data.qpos.numpy()[0, 2])
        for t in range(n_hold):
            if args.lean:
                apply_lean(data)
            for _ in range(args.sub):
                mjw.step(rm, data)
            q = data.qpos.numpy()[0]
            r, p = rpy(q[3:7])
            if q[2] < 0.5 * z0 or abs(r) > 0.6 or abs(p) > 0.6:
                return False, t * args.sub * dt
        return True, args.hold

    # baseline static hold (no push)
    ok0, t0 = trial(0.0, 0.0)
    print(f"[{Path(args.model).name}] static hold ({args.hold}s): "
          f"{'HELD' if ok0 else f'FELL@{t0:.2f}s'}")

    def basin(axis):
        best = 0.0
        for push in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.3, 1.6, 2.0]:
            ok, tf = trial(push if axis == "x" else 0.0, push if axis == "y" else 0.0)
            tag = "ok " if ok else f"FELL@{tf:.2f}"
            print(f"    push_{axis}={push:.2f} m/s -> {tag}")
            if ok:
                best = push
            else:
                break
        return best

    print("  forward (+x) basin:")
    fx = basin("x")
    print("  lateral (+y) basin:")
    fy = basin("y")
    print(f"[{Path(args.model).name}] BASIN: forward={fx:.2f} m/s  lateral={fy:.2f} m/s  "
          f"(static {'held' if ok0 else 'fell'})")


if __name__ == "__main__":
    main()
