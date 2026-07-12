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

"""Pure-math forward kinematics + damped-least-squares IK for the manipulation
overlay in ``humanoid_stand_deploy``.

These are copied VERBATIM (same math, same conventions) from the proven
``omnilink_arm_bridge`` IK stack so the two agree exactly. They are copied
rather than imported because importing ``omnilink_arm_bridge`` pulls in
``from omnisim import Supervisor`` at module load (it is a controller), which
couples this generic stand controller to the demo bridge and breaks a plain
``import``. This module has NO omnisim dependency — it is pure float math.

A "chain" is a list of per-joint tuples ``(origin_xyz, origin_rpy, axis)`` in
URDF parent-frame convention: ``origin_xyz`` is the joint origin [x,y,z],
``origin_rpy`` are Euler angles [r,p,y] applied as ZYX-intrinsic (Rz·Ry·Rx),
``axis`` is the joint rotation axis [x,y,z]. ``tcp_offset`` is the tool point in
the final joint frame. FK returns the TCP position in the chain's BASE frame
(for the G1 arm chains here, the base frame is ``torso_link``).
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def clamp_q(q: List[float], limits: Sequence[Tuple[float, float]]) -> List[float]:
    return [clamp(q[i], limits[i][0], limits[i][1]) for i in range(len(q))]


def _rot(axis: Tuple[float, float, float], theta: float) -> List[List[float]]:
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(theta), math.sin(theta)
    C = 1.0 - c
    return [
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


def _mat_mul(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat_vec(A: List[List[float]], v: Sequence[float]) -> Tuple[float, float, float]:
    return (
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    )


def _rot_rpy(rpy: Tuple[float, float, float]) -> List[List[float]]:
    r, p, y = rpy
    return _mat_mul(_mat_mul(_rot((0, 0, 1), y), _rot((0, 1, 0), p)), _rot((1, 0, 0), r))


def forward_kinematics(chain, q, tcp_offset) -> Tuple[float, float, float]:
    """TCP position in the chain base frame given joint angles ``q``."""
    R = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    p = [0.0, 0.0, 0.0]
    for (offset, rpy, axis), qi in zip(chain, q):
        off_world = _mat_vec(R, offset)
        p = [p[0] + off_world[0], p[1] + off_world[1], p[2] + off_world[2]]
        R = _mat_mul(R, _rot_rpy(rpy))
        R = _mat_mul(R, _rot(axis, qi))
    tcp = _mat_vec(R, tcp_offset)
    return (p[0] + tcp[0], p[1] + tcp[1], p[2] + tcp[2])


def numerical_jacobian(chain, q, tcp_offset, eps: float = 1e-4):
    """Position-only Jacobian (3xN), central difference."""
    N = len(q)
    J = [[0.0] * N for _ in range(3)]
    for j in range(N):
        qp = list(q); qp[j] += eps
        qm = list(q); qm[j] -= eps
        fp = forward_kinematics(chain, qp, tcp_offset)
        fm = forward_kinematics(chain, qm, tcp_offset)
        for i in range(3):
            J[i][j] = (fp[i] - fm[i]) / (2 * eps)
    return J


def _invert3(M):
    a, b, c = M[0]
    d, e, f = M[1]
    g, h, i = M[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        return None
    inv_det = 1.0 / det
    return [
        [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
        [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
        [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
    ]


def dls_ik(chain, q_seed, target_xyz, ik_cfg, joint_limits, q_rest=None, posture_gain=0.0):
    """Damped least squares IK, position-only. Returns (q, err_norm, iters).

    Optional NULL-SPACE POSTURE bias: with the 5-DOF arm reaching a 3-DOF position
    there are 2 redundant DOF that a plain DLS resolves arbitrarily — which yields
    contorted, self-colliding poses (elbow through the torso, wrist twisted). Passing
    ``q_rest`` (a natural reference posture) + ``posture_gain`` > 0 pulls the redundant
    DOF toward ``q_rest`` IN THE NULL SPACE of the position task, so the TCP still hits
    the target but the arm stays in a natural, body-clear configuration. This is the
    standard way real-robot IK keeps poses sensible."""
    q = list(q_seed)
    N = len(q)
    max_iters = int(ik_cfg.get("max_iters", 200))
    tol = float(ik_cfg.get("tol", 1e-3))
    damping = float(ik_cfg.get("damping", 0.1))
    max_dq = float(ik_cfg.get("max_dq", 0.2))
    tcp = ik_cfg["tcp_offset"]
    use_posture = q_rest is not None and posture_gain > 0.0
    err_norm = 0.0
    for it in range(max_iters):
        x = forward_kinematics(chain, q, tcp)
        err = [target_xyz[i] - x[i] for i in range(3)]
        err_norm = math.sqrt(sum(e * e for e in err))
        if err_norm < tol and not use_posture:
            return q, err_norm, it
        J = numerical_jacobian(chain, q, tcp)
        JJt = [[0.0] * 3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                JJt[i][j] = sum(J[i][k] * J[j][k] for k in range(N))
            JJt[i][i] += damping * damping
        inv = _invert3(JJt)
        if inv is None:
            return q, err_norm, it
        # DLS pseudo-inverse Jp = J^T (J J^T + lam^2 I)^-1  (N x 3)
        Jp = [[sum(J[r][i] * inv[r][k] for r in range(3)) for k in range(3)] for i in range(N)]
        dq = [Jp[i][0] * err[0] + Jp[i][1] * err[1] + Jp[i][2] * err[2] for i in range(N)]
        if use_posture:
            # null-space projector term: (I - Jp J) (q_rest - q) * gain
            JpJ = [[sum(Jp[i][k] * J[k][m] for k in range(3)) for m in range(N)] for i in range(N)]
            dqn = []
            for i in range(N):
                s = 0.0
                for m in range(N):
                    proj = (1.0 if i == m else 0.0) - JpJ[i][m]
                    s += proj * (q_rest[m] - q[m])
                dqn.append(posture_gain * s)
            dq = [dq[i] + dqn[i] for i in range(N)]
            if err_norm < tol and max(abs(d) for d in dqn) < 1e-4:
                return q, err_norm, it
        max_abs = max(abs(d) for d in dq) or 1e-9
        if max_abs > max_dq:
            scale = max_dq / max_abs
            dq = [d * scale for d in dq]
        q = [q[j] + dq[j] for j in range(N)]
        q = clamp_q(q, joint_limits)
    return q, err_norm, max_iters


def parse_chain(flat_list):
    """Convert spec JSON chain (list of 9-float rows [ox,oy,oz,rr,rp,ry,ax,ay,az])
    into the tuple form FK expects."""
    chain = []
    for row in flat_list:
        chain.append(((row[0], row[1], row[2]), (row[3], row[4], row[5]), (row[6], row[7], row[8])))
    return chain
