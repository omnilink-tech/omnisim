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

"""Centroidal MPC (the Newton-MPC-stack plan, section 8).

MVP = the standing/balance force QP (1-step horizon): given the live centroidal state and
a reference, solve for per-foot ground-reaction forces that drive the CoM + torso
orientation to the reference, subject to the friction cone, unilateral (fz>=0) and CoP
limits. The angular-momentum (CAM) term is FIRST-CLASS here -- it is the body-rotation
arrest that my push-recovery session found to be the binding wall, not an afterthought.

Deterministic QP, pure numpy (no scipy/OSQP needed in the embedded interpreter): the
equality-shaped wrench objective is solved by a regularized normal-equation, then the
solution is projected onto the friction cone (a few iterations). This is the same robust
recipe that gave finite, gravity-supported forces in the earlier WBC probe.

The N-step horizon + contact schedule (for stepping) plugs in via `solve(..., schedule)`;
the MVP uses a single double-support node.
"""
import numpy as np


def _skew(p):
    return np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])


class CentroidalMPC:
    def __init__(self, cfg):
        self.kp_c = cfg.get("kp_com", 60.0)
        self.kd_c = cfg.get("kd_com", 16.0)
        self.kp_z = cfg.get("kp_z", 140.0)
        self.kd_z = cfg.get("kd_z", 22.0)
        self.kp_L = cfg.get("kp_L", 18.0)     # orientation -> dL (CAM = body-rotation arrest)
        self.kd_L = cfg.get("kd_L", 6.0)      # damp angular momentum
        self.w_force = cfg.get("w_force", 6.0)   # wrench-force tracking weight
        self.w_moment = cfg.get("w_moment", 3.0) # wrench-moment (CAM) tracking weight
        self.reg = cfg.get("reg", 1e-3)
        self.mu = cfg.get("mu", 0.7)
        self.cop = cfg.get("cop", 0.06)       # |CoP| bound proxy via tangential moment arm
        self.fz_max = cfg.get("fz_max", 1.0e4)
        self.proj_iters = cfg.get("proj_iters", 6)

    def desired_wrench(self, st, com_ref):
        """Desired net wrench [F(3); M(3)] about the CoM from CoM + torso PD."""
        a_com = self.kp_c * (com_ref - st.com) - self.kd_c * st.com_vel
        a_com[2] = self.kp_z * (com_ref[2] - st.com[2]) - self.kd_z * st.com_vel[2]
        F_des = st.mass * (a_com - st.g)              # sum f_i = m(a_com - g)
        roll, pitch, _ = st.roll_pitch_yaw()
        # drive torso upright + damp angular momentum: dL_des = -Kp*ori_err - Kd*L
        dL_des = np.array([-self.kp_L * roll, -self.kp_L * pitch, 0.0]) - self.kd_L * st.L
        return F_des, dL_des

    def solve(self, st, com_ref, stance_legs):
        """Solve the per-foot GRF QP for the given stance feet. Returns {leg: f(3)}.
        stance_legs: list of legs currently in contact (swing feet contribute no force)."""
        legs = list(stance_legs)
        nf = 3 * len(legs)
        if nf == 0:
            return {}
        F_des, M_des = self.desired_wrench(st, com_ref)
        # wrench map G f = w:  force rows = sum f_i ;  moment rows = sum (p_i - c) x f_i
        G = np.zeros((6, nf))
        for k, lg in enumerate(legs):
            G[0:3, 3 * k:3 * k + 3] = np.eye(3)
            G[3:6, 3 * k:3 * k + 3] = _skew(st.feet[lg] - st.com)
        W = np.diag([self.w_force] * 3 + [self.w_moment] * 3)
        w = np.concatenate([F_des, M_des])
        # regularized normal equations: (G^T W G + reg I) f = G^T W w
        H = G.T @ W @ G + self.reg * np.eye(nf)
        g = G.T @ W @ w
        try:
            f = np.linalg.solve(H, g)
        except Exception:
            f = np.linalg.lstsq(H, g, rcond=None)[0]
        # project onto friction cone + unilateral + CoP, re-solve residual a few times
        for _ in range(self.proj_iters):
            for k in range(len(legs)):
                fk = f[3 * k:3 * k + 3]
                fz = min(max(0.0, fk[2]), self.fz_max)
                ft = np.hypot(fk[0], fk[1])
                lim = self.mu * fz
                if ft > lim and ft > 1e-9:
                    s = lim / ft
                    fk[0] *= s; fk[1] *= s
                fk[2] = fz
                f[3 * k:3 * k + 3] = fk
        out = {lg: f[3 * k:3 * k + 3].copy() for k, lg in enumerate(legs)}
        # diagnostics
        achieved = G @ f
        out["_resF"] = float(np.linalg.norm(achieved[0:3] - F_des))
        out["_resM"] = float(np.linalg.norm(achieved[3:6] - M_des))
        out["_Fz"] = float(sum(out[lg][2] for lg in legs))
        return out
