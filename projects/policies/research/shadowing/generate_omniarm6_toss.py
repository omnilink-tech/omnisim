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

"""Generate a FEASIBLE OMNIARM6 toss-to-place ghost (Shadowing Component 1, fixed-base arm).

The intent: throw a held part into a bin BEYOND the arm's kinematic reach -- something no
quasi-static IK controller can do (the object cannot be *carried* there). The ghost is a
designed sagittal swing (windup -> whip -> follow-through) played through a self-limiting
OMNIARM6 model whose actuators carry BOTH a torque clamp (forcerange = URDF effort) and a motor
speed cap (joint damping = effort / vel_limit). So every state of the generated swing is
feasible w.r.t. the real motor torque AND joint-velocity limits *by construction* -- no penalty
to tune. The thrown object's flight is exact projectile physics, decoupled from the arm
dynamics (the TossingBot factoring: plan the release, ballistics does the rest).

We deliberately do NOT use the MPPI ghost generator here: predictive sampling goes chaotic on a
fast fling (the shadowing doc's own warning), and a designed swing + feasibility-limited
playback is both reliable and a legitimate trajectory generator for this task. Aiming is by
RELEASE-INSTANT selection: a single swing sweeps the payload through a family of release states,
each of which (if let go then) lands at a different distance -- we pick the instant that hits
the target bin. The set of all reachable landings IS the feasibility frontier.

Run from a native shell with CPU mujoco:
  python projects/policies/research/shadowing/generate_omniarm6_toss.py [--bin-x 1.3] [--out PATH]
"""
import argparse
import os
import sys

import numpy as np
import mujoco

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
sys.path.insert(0, REPO)

MJCF = os.path.join(REPO, "projects/robots/omnisim/omniarm6/mjcf/omniarm6_throw.mjcf.xml")
G = 9.81
BIN_RIM_Z = 0.20      # target landing height (bin rim), m
KIN_REACH_X = 1.0     # max horizontal TCP reach (FK); floor-bin reach is ~0.9 m (arm reaching down)
JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
VEL_LIMITS = np.array([3.1415, 3.1415, 3.1415, 3.1415, 3.4906, 3.4906])  # URDF rad/s

# --- the designed sagittal throw (joint1/4/6 stay 0; the swing uses shoulder/elbow/wrist) ---
# pose = [j1, j2, j3, j4, j5, j6]
WINDUP = [0.0, -1.30, 0.80, 0.0, 0.60, 0.0]    # arm cocked back-and-up, elbow + wrist loaded
RELEASE = [0.0, 0.30, 0.00, 0.0, -0.30, 0.0]   # arm whips up-and-forward through the top
FOLLOW = [0.0, 1.00, 0.00, 0.0, -0.50, 0.0]    # follow through down-forward
T_HOLD = 0.12         # hold the cocked pose before launching
T_RELEASE = 0.60      # swing duration windup -> release pose
T_TOTAL = 1.00        # + follow-through
SIM_DT = 0.002


def ballistic_landing(p, v, zb=BIN_RIM_Z):
    """Where a projectile released at world pos p with world vel v lands at height zb.
    Returns (land_x, land_y, flight_t) or None if it never descends to zb going forward."""
    z, vz = float(p[2]), float(v[2])
    disc = vz * vz + 2.0 * G * (z - zb)
    if disc < 0:
        return None
    t = (vz + np.sqrt(disc)) / G
    if t <= 0:
        return None
    return p[0] + v[0] * t, p[1] + v[1] * t, t


def _smoothstep(a):
    a = min(1.0, max(0.0, a))
    return a * a * (3.0 - 2.0 * a)


def _interp(keys, t):
    if t <= keys[0][0]:
        return np.array(keys[0][1])
    if t >= keys[-1][0]:
        return np.array(keys[-1][1])
    for i in range(len(keys) - 1):
        t0, q0 = keys[i]
        t1, q1 = keys[i + 1]
        if t0 <= t <= t1:
            f = _smoothstep((t - t0) / (t1 - t0)) if t1 > t0 else 0.0
            return np.array(q0) + f * (np.array(q1) - np.array(q0))
    return np.array(keys[-1][1])


class TossGhostGenerator:
    """Plays the designed swing through the self-limiting OMNIARM6 model and records the ghost."""

    def __init__(self, mjcf=MJCF, sim_dt=SIM_DT):
        self.m = mujoco.MjModel.from_xml_path(mjcf)
        self.m.opt.timestep = sim_dt
        self.d = mujoco.MjData(self.m)
        self.sim_dt = sim_dt
        self.bid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "payload")

    def generate(self, windup=WINDUP, release=RELEASE, follow=FOLLOW,
                 t_hold=T_HOLD, t_release=T_RELEASE, t_total=T_TOTAL, dt=0.01):
        keys = [(0.0, windup), (t_hold, windup), (t_release, release), (t_total, follow)]
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[:6] = windup
        mujoco.mj_forward(self.m, self.d)
        n_sub = max(1, int(round(dt / self.sim_dt)))
        Q, QV, C, TCP, TCPV = [], [], [], [], []
        n = int(round(t_total / dt))
        for k in range(n):
            ctrl = _interp(keys, k * dt)
            self.d.ctrl[:] = ctrl
            for _ in range(n_sub):
                mujoco.mj_step(self.m, self.d)
            res = np.zeros(6)
            mujoco.mj_objectVelocity(self.m, self.d, mujoco.mjtObj.mjOBJ_BODY, self.bid, res, 0)
            Q.append(self.d.qpos.copy()); QV.append(self.d.qvel.copy()); C.append(ctrl.copy())
            TCP.append(self.d.xpos[self.bid].copy()); TCPV.append(res[3:6].copy())
        return dict(dt=dt, joints=list(JOINTS), q=np.array(Q), qvel=np.array(QV),
                    ctrl=np.array(C), tcp=np.array(TCP), tcpvel=np.array(TCPV))


def release_map(g):
    """For each step, where the payload would land if released then (valid forward releases
    only). Returns list of (k, land_x, speed, angle_deg, rel_z)."""
    out = []
    for k in range(g["tcp"].shape[0]):
        p, v = g["tcp"][k], g["tcpvel"][k]
        if v[0] <= 0.1 or p[2] < 0.4:
            continue
        land = ballistic_landing(p, v)
        if land is None:
            continue
        out.append((k, land[0], float(np.linalg.norm(v)),
                    float(np.degrees(np.arctan2(v[2], v[0]))), float(p[2])))
    return out


def feasibility(g):
    """Per-joint peak velocity ratio (max|qd|/limit). <=1 with margin => velocity-feasible."""
    qd = g["qvel"]
    return float((np.abs(qd) / VEL_LIMITS).max())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin-x", type=float, default=1.3, help="target bin distance from base (m)")
    ap.add_argument("--out", default=os.path.join(REPO, "projects/policies/research/shadowing/ghosts/omniarm6_toss_ghost.npz"))
    args = ap.parse_args()

    gen = TossGhostGenerator()
    g = gen.generate()
    rmap = release_map(g)
    if not rmap:
        print("[toss-gen] swing produced no valid forward release -- redesign the poses.")
        return 1

    velr = feasibility(g)
    max_land = max(r[1] for r in rmap)
    # pick the release instant that lands closest to the target bin
    k, land_x, spd, ang, rel_z = min(rmap, key=lambda r: abs(r[1] - args.bin_x))

    print(f"[toss-gen] swing feasibility: peak velocity ratio = {velr:.2f} "
          f"({'FEASIBLE' if velr <= 1.05 else 'OVER LIMIT'})")
    print(f"[toss-gen] feasibility frontier: max throw range = {max_land:.2f} m "
          f"(kinematic reach ~{KIN_REACH_X:.2f} m)")
    print(f"[toss-gen] target bin x={args.bin_x:.2f} m -> RELEASE @ t={k*g['dt']:.2f}s  "
          f"TCP=({g['tcp'][k][0]:.3f},{rel_z:.3f})  |v|={spd:.2f} m/s  angle={ang:+.1f} deg")
    land = ballistic_landing(g["tcp"][k], g["tcpvel"][k])
    err = abs(land[0] - args.bin_x)
    print(f"[toss-gen] LANDS at x={land[0]:.3f} m (err {err*100:.1f} cm), flight {land[2]*1000:.0f} ms")
    if args.bin_x > KIN_REACH_X:
        print(f"[toss-gen] => bin is BEYOND kinematic reach: a carry/IK controller CANNOT place here.")
    if args.bin_x > max_land + 1e-6:
        print(f"[toss-gen] WARNING: target exceeds the feasible frontier -- INFEASIBLE throw "
              f"(best achievable {max_land:.2f} m).")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    save = {kk: vv for kk, vv in g.items() if kk != "joints"}
    save["joints"] = np.array(g["joints"])
    save["release_k"] = k
    save["bin_x"] = args.bin_x
    save["land"] = np.array(land)
    save["max_range"] = max_land
    save["vel_ratio"] = velr
    save["vel_limits"] = VEL_LIMITS
    save["bin_rim_z"] = BIN_RIM_Z
    save["kin_reach_x"] = KIN_REACH_X
    save["sim_dt"] = SIM_DT
    np.savez(args.out, **save)
    print(f"[toss-gen] saved ghost -> {os.path.relpath(args.out, REPO)} "
          f"({g['q'].shape[0]} steps, release step {k})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
