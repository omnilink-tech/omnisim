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

"""omnilink_arm_bridge — generic OmniLink bridge for URDF manipulator arms.

Drop this in as the URDFRobot's controller (`supervisor TRUE`). One
controllerArg picks which arm config to load from `_arm_configs.py`:

    controller "omnilink_arm_bridge"
    controllerArgs ["--robot" "ur5e" "--port" "8765"]

Adding a new arm: append a config to ARM_CONFIGS and you're done.

Surfaces
--------
1. HTTP on 127.0.0.1:<port>  (default 8765 -- matches Axis's
   AXIS_BRIDGE_URL default). Axis-normalized endpoints; see
   `omnilink-bridge.md` in olink/agents/axis/knowledge/.

       POST /list_robots          -> [{id, model, capabilities}]
       POST /get_robot_state      -> {q, qdot, tcp, fault, last_tick_at}
       POST /read_joints          -> {q}
       POST /read_tcp_pose        -> {xyz, rpy}
       POST /set_joint_positions  -> {accepted, clamped_q}
       POST /set_tcp_target       -> {accepted, solved_q, err_norm}
       POST /solve_ik             -> {q, err_norm}
       POST /stop_robot           -> {halted_at}
       POST /reset_to_home        -> {q}
       POST /open_gripper         -> {state, gripper}   (if a gripper is set)
       POST /close_gripper        -> {state, gripper}   (ditto)
       POST /set_gripper_width    -> {gripper}          width in metres
       POST /grasp                -> {gripper}          close + hold (force/width)
       POST /release              -> {gripper}          open + drop
       POST /prompt               -> {response, actions}  natural-language
       GET  /capabilities         -> as listed in /list_robots
       GET  /state                -> alias for get_robot_state
       GET  /hardware_status      -> {enabled} or the hardware backend's status

2. Webots robot window: omnilink_chat plugin. The window POSTs
   "prompt:<text>" / "stop" / "configure" via wwi; the bridge replies
   with "agent:<text>", "tool:<name>:<ok|err>:<summary>", "status:<state>",
   "system:<text>" lines back through wwiSendText.

Without OmniLink configured, the bridge ships a regex-based intent
router that maps prompts directly to its own surface (no LLM). Set
OMNILINK_RELAY=1 + OMNI_KEY to relay prompts through OmniLink (future
PR; the relay hook is wired but the network path is left stubbed).

Real hardware (optional)
------------------------
The bridge is sim-first, but the same commands can also drive a real arm
through a pluggable *hardware backend* -- a sibling `<name>_backend.py`
module, selected with `--hardware-backend <name> [--hardware-ip <addr>]`.
See the HardwareBackend protocol below for the contract. No backend ships
in the box; with none installed the bridge is pure sim and the option is
simply not offered.
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Protocol, Tuple

from omnisim import Supervisor

# Make sibling modules importable when Webots invokes the controller.
import os as _os
import sys as _sys
_THIS_DIR = _os.path.dirname(_os.path.abspath(__file__))
if _THIS_DIR not in _sys.path:
    _sys.path.insert(0, _THIS_DIR)
# Make the shared OmniLink relay package importable.
_RELAY_PARENT = _os.path.abspath(_os.path.join(_THIS_DIR, ".."))
if _RELAY_PARENT not in _sys.path:
    _sys.path.insert(0, _RELAY_PARENT)

from _arm_configs import ARM_CONFIGS, get_config  # noqa: E402
from _gripper_configs import (  # noqa: E402
    GRIPPER_CONFIGS,
    get_gripper_config,
    legacy_gripper_config,
)
from gripper_effectors import make_effector  # noqa: E402
from _chat_page import CHAT_HTML  # noqa: E402

# Optional -- only available when the relay package is on the path.
try:
    from _omnilink_relay import OmniLinkRelay, Tool, is_enabled as omnilink_enabled, get_omni_key  # noqa: E402
except Exception:
    OmniLinkRelay = None  # type: ignore[assignment]
    Tool = None  # type: ignore[assignment]
    def omnilink_enabled() -> bool: return False
    def get_omni_key() -> str: return ""


# ── Hardware backends (pluggable, discovered by name) ────────────────
#
# The bridge is sim-first: it always drives the simulated arm. A *hardware
# backend* is an OPTIONAL adapter that lets the same commands ALSO drive a
# real robot over that robot's own control API, and feeds the real robot's
# measured joints back so the simulated arm mirrors it (a live digital twin).
#
# A backend is a sibling module in this directory named `<name>_backend.py`,
# selected at runtime with `--hardware-backend <name> [--hardware-ip <addr>]`
# (or OMNILINK_HARDWARE_BACKEND / OMNILINK_HARDWARE_IP). Nothing about any
# particular robot vendor is known to, or named in, this file: drop the module
# in and the option appears; take it away and the bridge is pure sim.
#
# A backend module must expose one factory:
#
#     maybe_build(cfg, robot_id, ip_arg=None, on_event=None)
#         -> HardwareBackend | None
#
#   cfg       the arm config dict from _arm_configs (the backend may read its
#             own optional block out of it, e.g. a joint sign/offset map).
#   robot_id  the bridge's robot id.
#   ip_arg    the address from --hardware-ip, or None.
#   on_event  callable(kind, text) -- "status" / "error" lines the bridge
#             forwards to the robot window.
#
#   It MUST return None when the operator has not opted in, so that merely
#   having the module present never touches hardware on a plain sim launch.
#
# The object it returns must implement HardwareBackend. All command methods
# are fire-and-forget: they must NOT block the simulation tick.


class HardwareBackend(Protocol):
    """The surface the bridge uses to drive a real arm alongside the sim."""

    ip: str            # address the backend is talking to (for status/logs)
    dry_run: bool      # True when running against an in-process mock
    connected: bool    # flips True once the link is up; commands are no-ops
                       # until it does

    # -- lifecycle --------------------------------------------------------
    def start(self) -> None: ...
    def shutdown(self) -> None: ...

    # -- reads ------------------------------------------------------------
    def status(self) -> Dict[str, Any]: ...
    def get_joints(self) -> Optional[List[float]]: ...   # measured q, sim-frame

    # -- commands (non-blocking) ------------------------------------------
    def move_joint(self, q: List[float]) -> None: ...
    def move_linear(self, xyz: Tuple[float, float, float]) -> None: ...
    def reset_to_home(self) -> None: ...
    def wave(self, amplitudes: List[float]) -> None: ...
    def grasp(self) -> None: ...
    def release(self) -> None: ...
    def stop(self) -> None: ...


_BACKEND_SUFFIX = "_backend.py"


def discover_hardware_backends() -> List[str]:
    """Names of the hardware-backend modules sitting next to this controller."""
    found: List[str] = []
    try:
        for fn in sorted(_os.listdir(_THIS_DIR)):
            if fn.endswith(_BACKEND_SUFFIX) and not fn.startswith("_"):
                found.append(fn[: -len(_BACKEND_SUFFIX)])
    except OSError:
        pass
    return found


def load_hardware_backend(name: str, cfg: dict, robot_id: str,
                          ip_arg: Optional[str] = None,
                          on_event=None) -> Tuple[Optional[Any], Optional[str]]:
    """Import `<name>_backend.py` from this directory and ask it to build.

    Returns (backend, error). `backend` is None either because the module is
    absent / malformed (then `error` says so) or because the backend declined
    to activate -- the operator did not opt in (then `error` is None too).
    """
    if not re.fullmatch(r"[A-Za-z0-9_]+", name or ""):
        return None, f"invalid hardware backend name {name!r}"
    available = discover_hardware_backends()
    if not _os.path.isfile(_os.path.join(_THIS_DIR, name + _BACKEND_SUFFIX)):
        return None, (
            f"hardware backend '{name}' is not installed: no "
            f"{name}{_BACKEND_SUFFIX} next to this controller. "
            f"Available: {available if available else 'none'}.")
    try:
        mod = importlib.import_module(name + "_backend")
    except Exception as e:
        return None, f"hardware backend '{name}' failed to import: {e!r}"
    build = getattr(mod, "maybe_build", None)
    if not callable(build):
        return None, (f"hardware backend '{name}' exposes no "
                      f"maybe_build(cfg, robot_id, ip_arg=..., on_event=...)")
    try:
        return build(cfg, robot_id, ip_arg=ip_arg, on_event=on_event), None
    except Exception as e:
        return None, f"hardware backend '{name}' failed to build: {e!r}"


def attach_hardware(bridge: "ArmBridge", cfg: dict, robot_id: str,
                    name: Optional[str], ip: Optional[str]) -> None:
    """Resolve, build and attach a hardware backend, or leave the bridge pure-sim.

    Off by default. With an explicit `--hardware-backend`, a module that cannot
    be loaded is a hard error (the operator asked for hardware; silently running
    sim-only would be a lie). With no name given, each discovered backend is
    offered the chance to activate itself from its own environment -- every
    backend returns None unless the operator opted in, so a plain sim launch
    attaches nothing.
    """
    on_event = lambda k, t: bridge.queue_window(          # noqa: E731
        ("error:" + t) if k == "error" else ("system:" + t))

    candidates: List[str]
    if name:
        candidates = [name]
    elif ip:
        found = discover_hardware_backends()
        if len(found) != 1:
            raise SystemExit(
                "[omnilink_arm_bridge] --hardware-ip given but no "
                "--hardware-backend: cannot tell which backend to use "
                f"(available: {found if found else 'none'}).")
        candidates = found
    else:
        candidates = discover_hardware_backends()

    for cand in candidates:
        be, err = load_hardware_backend(cand, cfg, robot_id,
                                        ip_arg=ip, on_event=on_event)
        if err is not None:
            if name:                       # explicit request -> fail loudly
                raise SystemExit(f"[omnilink_arm_bridge] {err}")
            print(f"[omnilink_arm_bridge] hardware backend '{cand}' "
                  f"unavailable: {err}")
            continue
        if be is None:
            continue                       # backend declined: not opted in
        bridge.hw = be
        bridge.hw_name = cand
        be.start()
        print(f"[omnilink_arm_bridge] hardware backend '{cand}' attached "
              f"(ip={getattr(be, 'ip', '?')}, "
              f"dry_run={getattr(be, 'dry_run', False)})")
        return

    if name:
        print(f"[omnilink_arm_bridge] hardware backend '{name}' did not "
              "activate (no address given and no opt-in in its environment); "
              "running sim-only.")


# ── CLI ──────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--robot", default=next(iter(ARM_CONFIGS), None),
                        help=f"Robot id, one of {sorted(ARM_CONFIGS.keys())}. "
                             "Defaults to the first registered arm.")
    parser.add_argument("--port", type=int, default=8765,
                        help="HTTP port for the Axis-normalized surface")
    parser.add_argument("--gripper", default=None,
                        help=("Gripper id to attach, one of "
                              f"{sorted(GRIPPER_CONFIGS.keys())}. Omit to use "
                              "the arm's default_gripper / legacy inline "
                              "gripper fields, if any."))
    parser.add_argument("--name", default=None,
                        help=("Override the agent id used in the OmniLink "
                              "profile name and Axis robot_id. Defaults to "
                              "--robot. Use when multiple arms of the same "
                              "kind share a world (e.g. several UR5es)."))
    _backends = discover_hardware_backends()
    parser.add_argument("--hardware-backend", default=None,
                        help=("Drive a REAL arm alongside the sim through the "
                              "named hardware backend "
                              f"({_backends if _backends else 'none installed'})"
                              ". A backend is a sibling <name>_backend.py "
                              "module; omit for a pure-sim run. Also settable "
                              "via OMNILINK_HARDWARE_BACKEND."))
    parser.add_argument("--hardware-ip", default=None,
                        help=("Address of the real arm (or its offline-sim VM) "
                              "handed to the hardware backend. Also settable "
                              "via OMNILINK_HARDWARE_IP."))
    args, _unknown = parser.parse_known_args()
    if not args.hardware_backend:
        args.hardware_backend = _os.environ.get("OMNILINK_HARDWARE_BACKEND") or None
    if not args.hardware_ip:
        args.hardware_ip = _os.environ.get("OMNILINK_HARDWARE_IP") or None
    return args


# ── Math helpers ─────────────────────────────────────────────────────

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def clamp_q(q: List[float], limits: List[Tuple[float, float]]) -> List[float]:
    return [clamp(qi, lo, hi) for qi, (lo, hi) in zip(q, limits)]


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


def _rot_rpy(rpy: Tuple[float, float, float]) -> List[List[float]]:
    r, p, y = rpy
    Rz = _rot((0, 0, 1), y)
    Ry = _rot((0, 1, 0), p)
    Rx = _rot((1, 0, 0), r)
    # ZYX intrinsic == roll about X, then pitch about Y, then yaw about Z
    return _mat_mul(_mat_mul(Rz, Ry), Rx)


def _mat_mul(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
    return [
        [sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def _mat_vec(A: List[List[float]], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    )


def forward_kinematics(chain: List[Tuple], q: List[float], tcp_offset: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """TCP world position. Chain is parent-frame URDF tuples
    (origin_xyz, origin_rpy, joint_axis)."""
    R = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    p = [0.0, 0.0, 0.0]
    for (offset, rpy, axis), qi in zip(chain, q):
        R_offset = _rot_rpy(rpy)
        # Apply parent-frame offset (rotated into current frame).
        off_world = _mat_vec(R, offset)
        p = [p[0] + off_world[0], p[1] + off_world[1], p[2] + off_world[2]]
        R = _mat_mul(R, R_offset)
        # Joint rotation about its local axis.
        R = _mat_mul(R, _rot(axis, qi))
    tcp_world = _mat_vec(R, tcp_offset)
    return (p[0] + tcp_world[0], p[1] + tcp_world[1], p[2] + tcp_world[2])


def forward_kinematics_pose(chain: List[Tuple], q: List[float],
                            tcp_offset: Tuple[float, float, float]):
    """Like forward_kinematics but also returns the TCP rotation matrix
    (base frame). Used to orient a mounted gripper to the wrist."""
    R = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    p = [0.0, 0.0, 0.0]
    for (offset, rpy, axis), qi in zip(chain, q):
        off_world = _mat_vec(R, offset)
        p = [p[0] + off_world[0], p[1] + off_world[1], p[2] + off_world[2]]
        R = _mat_mul(R, _rot_rpy(rpy))
        R = _mat_mul(R, _rot(axis, qi))
    tcp = _mat_vec(R, tcp_offset)
    return [p[0] + tcp[0], p[1] + tcp[1], p[2] + tcp[2]], R


def _mat3_to_axis_angle(m) -> List[float]:
    """3x3 rotation matrix -> Webots [x, y, z, angle] axis-angle."""
    trace = m[0][0] + m[1][1] + m[2][2]
    c = clamp((trace - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(c)
    if angle < 1e-6:
        return [0.0, 0.0, 1.0, 0.0]
    if abs(math.pi - angle) < 1e-3:
        # Near 180 deg: pull axis from the diagonal.
        ax = math.sqrt(max(0.0, (m[0][0] + 1.0) / 2.0))
        ay = math.sqrt(max(0.0, (m[1][1] + 1.0) / 2.0))
        az = math.sqrt(max(0.0, (m[2][2] + 1.0) / 2.0))
        return [ax, ay, az, angle]
    s = 2.0 * math.sin(angle)
    return [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s, angle]


def numerical_jacobian(chain, q, tcp_offset, eps: float = 1e-4):
    """Position-only Jacobian, central-difference. 3xN matrix as list of rows."""
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


def dls_ik(chain, q_seed, target_xyz, ik_cfg, joint_limits):
    """Damped least squares IK, position-only. Returns (q, err_norm, iters)."""
    q = list(q_seed)
    max_iters = ik_cfg["max_iters"]
    tol = ik_cfg["tol"]
    damping = ik_cfg["damping"]
    max_dq = ik_cfg["max_dq"]
    err_norm = 0.0
    for it in range(max_iters):
        x = forward_kinematics(chain, q, ik_cfg["tcp_offset"])
        err = [target_xyz[i] - x[i] for i in range(3)]
        err_norm = math.sqrt(sum(e * e for e in err))
        if err_norm < tol:
            return q, err_norm, it
        J = numerical_jacobian(chain, q, ik_cfg["tcp_offset"])
        # DLS: dq = J^T (J J^T + lambda^2 I)^-1 err
        JJt = [[0.0] * 3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                JJt[i][j] = sum(J[i][k] * J[j][k] for k in range(len(q)))
            JJt[i][i] += damping * damping
        # Invert 3x3
        inv = _invert3(JJt)
        if inv is None:
            return q, err_norm, it  # singular
        rhs = (
            inv[0][0] * err[0] + inv[0][1] * err[1] + inv[0][2] * err[2],
            inv[1][0] * err[0] + inv[1][1] * err[1] + inv[1][2] * err[2],
            inv[2][0] * err[0] + inv[2][1] * err[1] + inv[2][2] * err[2],
        )
        dq = [J[0][j] * rhs[0] + J[1][j] * rhs[1] + J[2][j] * rhs[2] for j in range(len(q))]
        # Cap step.
        max_abs = max(abs(d) for d in dq) or 1e-9
        if max_abs > max_dq:
            scale = max_dq / max_abs
            dq = [d * scale for d in dq]
        q = [q[j] + dq[j] for j in range(len(q))]
        q = clamp_q(q, joint_limits)
    return q, err_norm, max_iters


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


# ── 6-DOF (pose) IK: position + orientation, for top-down grasping ───

def _transpose3(M):
    return [[M[0][0], M[1][0], M[2][0]],
            [M[0][1], M[1][1], M[2][1]],
            [M[0][2], M[1][2], M[2][2]]]


def _rotvec(R) -> List[float]:
    """Log map: 3x3 rotation -> axis*angle as a 3-vector (rad)."""
    tr = R[0][0] + R[1][1] + R[2][2]
    c = clamp((tr - 1.0) / 2.0, -1.0, 1.0)
    ang = math.acos(c)
    if ang < 1e-8:
        return [0.0, 0.0, 0.0]
    if abs(math.pi - ang) < 1e-4:
        # Near 180 deg: axis from the largest diagonal of (R + I)/2.
        d = [(R[0][0] + 1.0) / 2.0, (R[1][1] + 1.0) / 2.0, (R[2][2] + 1.0) / 2.0]
        k = max(range(3), key=lambda i: d[i])
        axis = [0.0, 0.0, 0.0]
        axis[k] = math.sqrt(max(0.0, d[k]))
        for i in range(3):
            if i != k:
                axis[i] = (R[i][k] + R[k][i]) / (4.0 * axis[k]) if axis[k] > 1e-9 else 0.0
        n = math.sqrt(sum(a * a for a in axis)) or 1.0
        return [a / n * ang for a in axis]
    s = 2.0 * math.sin(ang)
    return [(R[2][1] - R[1][2]) / s * ang,
            (R[0][2] - R[2][0]) / s * ang,
            (R[1][0] - R[0][1]) / s * ang]


def _solve_linear(A, b):
    """Solve A x = b for square A (Gaussian elimination, partial pivot)."""
    n = len(b)
    M = [list(A[i]) + [b[i]] for i in range(n)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-12:
            return None
        M[col], M[piv] = M[piv], M[col]
        pv = M[col][col]
        for r in range(n):
            if r == col:
                continue
            f = M[r][col] / pv
            for k in range(col, n + 1):
                M[r][k] -= f * M[col][k]
    return [M[i][n] / M[i][i] for i in range(n)]


def numerical_jacobian_pose(chain, q, tcp_offset, eps: float = 1e-4):
    """6xN pose Jacobian (rows 0-2 position, 3-5 orientation), central diff."""
    N = len(q)
    J = [[0.0] * N for _ in range(6)]
    for j in range(N):
        qp = list(q); qp[j] += eps
        qm = list(q); qm[j] -= eps
        pp, Rp = forward_kinematics_pose(chain, qp, tcp_offset)
        pm, Rm = forward_kinematics_pose(chain, qm, tcp_offset)
        for i in range(3):
            J[i][j] = (pp[i] - pm[i]) / (2 * eps)
        w = _rotvec(_mat_mul(Rp, _transpose3(Rm)))
        for i in range(3):
            J[3 + i][j] = w[i] / (2 * eps)
    return J


def dls_ik_pose(chain, q_seed, target_pos, target_R, tcp_offset, ik_cfg,
                joint_limits, w_rot: float = 0.4):
    """Damped least squares IK to a full pose (position + orientation).
    Returns (q, pos_err, rot_err, iters)."""
    q = list(q_seed)
    max_iters = ik_cfg.get("pose_max_iters", 200)
    pos_tol = ik_cfg.get("tol", 5e-3)
    damping = ik_cfg.get("damping", 0.08)
    max_dq = ik_cfg.get("max_dq", 0.08)
    pos_err = rot_err = 0.0
    for it in range(max_iters):
        p, R = forward_kinematics_pose(chain, q, tcp_offset)
        ep = [target_pos[i] - p[i] for i in range(3)]
        eo = _rotvec(_mat_mul(target_R, _transpose3(R)))
        pos_err = math.sqrt(sum(e * e for e in ep))
        rot_err = math.sqrt(sum(e * e for e in eo))
        if pos_err < pos_tol and rot_err < 0.03:
            return q, pos_err, rot_err, it
        J = numerical_jacobian_pose(chain, q, tcp_offset)
        for i in range(3):
            for j in range(len(q)):
                J[3 + i][j] *= w_rot
        e = [ep[0], ep[1], ep[2], w_rot * eo[0], w_rot * eo[1], w_rot * eo[2]]
        JJt = [[sum(J[i][k] * J[r][k] for k in range(len(q))) for r in range(6)]
               for i in range(6)]
        for i in range(6):
            JJt[i][i] += damping * damping
        y = _solve_linear(JJt, e)
        if y is None:
            return q, pos_err, rot_err, it
        dq = [sum(J[i][j] * y[i] for i in range(6)) for j in range(len(q))]
        ma = max(abs(d) for d in dq) or 1e-9
        if ma > max_dq:
            dq = [d * (max_dq / ma) for d in dq]
        q = [q[j] + dq[j] for j in range(len(q))]
        q = clamp_q(q, joint_limits)
    return q, pos_err, rot_err, max_iters


# ── Cold-first-load warm-up ──────────────────────────────────────────

def warmup_reload(robot) -> bool:
    """No-op by default. Historically reloaded the world ONCE to dodge the COLD-FIRST-LOAD bug.

    BACKGROUND: on an early Newton build a FRESH world build's MuJoCo articulation under-tracked
    its position targets (arm undershot its commanded pose by ~1 cm) so precise grasps failed cold
    but worked after a world reload. Every precise-manipulation controller therefore reloaded once
    at startup.

    RESOLVED (verified 2026-07-05): the under-track NO LONGER reproduces on the current binary.
    Cold and warm loads settle bit-identically (bare-arm probe: identical joint + end-effector to
    6 decimals; a full arm+gripper grasp: identical every phase), with cold correctly building
    MuJoCo (no XPBD fallback). Root fix: `eb86f888` (Newton solver choice survives the multi-build
    load) + the finalize-time solver re-assert. See docs/developer/real-grasp-and-the-cold-first-
    load-trap.md. The startup reload is now pure overhead, so it is OFF by default.

    Kept as a one-line safety valve: set OMNISIM_FORCE_WARMUP=1 to re-enable the reload if a
    regression ever resurfaces (OMNISIM_NO_WARMUP=1 still forces it off, and wins). Returns True
    only if a reload was actually triggered."""
    import os
    import tempfile
    # Off by default now that the cold-load bug is fixed; opt back in with OMNISIM_FORCE_WARMUP=1.
    if os.environ.get("OMNISIM_NO_WARMUP") or not os.environ.get("OMNISIM_FORCE_WARMUP"):
        return False
    # OMNISIM_WARMUP_TOKEN (set per-launch by the headless runner / launcher) is the
    # most robust session key; fall back to the parent (simulator) pid otherwise.
    key = os.environ.get("OMNISIM_WARMUP_TOKEN")
    if not key:
        try:
            key = str(os.getppid())
        except Exception:
            key = "0"
    flag = os.path.join(tempfile.gettempdir(), "_omnisim_warmup_%s.flag" % key)
    if os.path.exists(flag):
        return False
    try:
        with open(flag, "w") as _f:
            _f.write("1")
    except Exception:
        return False            # can't set the loop-guard -> never reload
    try:
        dt = int(robot.getBasicTimeStep())
        # worldReload() raises if called before the controller has stepped, so run a
        # few steps to fully initialise first; the reload then takes effect cleanly.
        for _ in range(10):
            if robot.step(dt) == -1:
                return False
        robot.worldReload()
    except Exception:
        return False            # reload didn't fire -> let the (cold) demo run
    # The reload IS requested and this controller is being torn down + restarted.
    # CRITICAL: do NOT fall back into the caller and run the cold demo body here --
    # in the windowed GUI that races the reload and closes the window. Step until the
    # teardown delivers -1, then hard-exit so only the warm restart runs the demo.
    try:
        while robot.step(dt) != -1:
            pass
    except Exception:
        pass
    os._exit(0)


# ── Bridge state ─────────────────────────────────────────────────────

class ArmBridge:
    """Owns motors, motion plans, and the wire protocol for one arm.

    The Webots simulation step calls `tick()` once per basicTimeStep,
    which advances the active motion plan and pushes motor setpoints.
    All other surfaces (HTTP, wwi prompt) just mutate `motion_plan`
    under `lock`.
    """

    def __init__(self, robot: Supervisor, cfg: dict, robot_id: str,
                 gripper_id: Optional[str] = None) -> None:
        self.robot = robot
        self.cfg = cfg
        self.robot_id = robot_id
        self.timestep = int(robot.getBasicTimeStep())
        self.joint_names = list(cfg["joint_names"])
        self.joint_limits = list(cfg["joint_limits"])
        self.home_pose = list(cfg["home_pose"])
        self.motors = []
        self.sensors = []
        for jn in self.joint_names:
            motor = robot.getDevice(jn + "_motor")
            if motor is None:
                # Some URDF importers emit `<jn>` directly.
                motor = robot.getDevice(jn)
            if motor is None:
                print(f"[omnilink_arm_bridge] WARNING: missing motor for {jn!r}")
            else:
                # Snappy tracking: run the position controller at the motor's
                # full velocity (URDF velocity limit) so the arm doesn't lag
                # behind the interpolated setpoint.
                try:
                    vmax = motor.getMaxVelocity()
                    if vmax and vmax > 0:
                        motor.setVelocity(vmax)
                except Exception:
                    pass
                # Try to attach a position sensor; URDF importer pairs them.
                try:
                    s = motor.getPositionSensor()
                    if s is not None:
                        s.enable(self.timestep)
                        self.sensors.append(s)
                    else:
                        self.sensors.append(None)
                except Exception:
                    self.sensors.append(None)
            self.motors.append(motor)

        # End effector. Resolution order:
        #   1. explicit --gripper <id>           (registry)
        #   2. arm cfg "default_gripper" id      (registry)
        #   3. legacy inline gripper_* fields    (back-compat shim)
        # None of these present -> no gripper.
        self.effector = None
        self.gripper_cfg = None
        gid = gripper_id or cfg.get("default_gripper")
        if gid:
            self.gripper_cfg = get_gripper_config(gid)
        elif cfg.get("gripper_motors"):
            self.gripper_cfg = legacy_gripper_config(cfg)
        if self.gripper_cfg is not None:
            self.effector = make_effector(robot, self.gripper_cfg)

        # Kinematic-attach grasp (Phase 3): on grasp the nearest graspable
        # Solid within `grasp_radius` of the TCP is welded to the tool and
        # teleported to follow it each tick; release drops it. Objects opt
        # in by giving their node a DEF that starts with "GRASP_". This
        # avoids physics-contact instability (see plan: kinematic attach).
        self.grasp_radius = float((self.gripper_cfg or {}).get("grasp_radius", 0.08))
        self.held_node = None
        self.held_tfield = None
        self._self_node = robot.getSelf()

        # Mounted gripper visual: an optional top-level Solid named
        # DEF GRIPPER_<robot_id> (with finger sub-nodes _FINGER_L / _R).
        # When present the bridge teleports it to the wrist (flange) each
        # tick and animates finger spacing from the effector width, so a
        # bare URDF arm shows a gripper at its tool without editing the URDF.
        self.gripper_visual = robot.getFromDef("GRIPPER_" + robot_id)
        self.gripper_fingers = (
            robot.getFromDef("GRIPPER_%s_FINGER_L" % robot_id),
            robot.getFromDef("GRIPPER_%s_FINGER_R" % robot_id),
        )
        # Anchor to the REAL tool-mount link's pose (read from the scene
        # tree), not an approximate FK chain -- otherwise the gripper clips
        # into the wrist mesh and the grasp point disagrees with where the
        # gripper is drawn. The mount link is the URDF "flange" (fallbacks
        # cover other arms' naming). Resolved whenever a gripper is set so
        # both the visual AND the grasp weld ride the same real pose.
        self.gripper_anchor = (self._find_mount_node()
                               if self.effector is not None else None)
        self.tool_reach = float((self.gripper_cfg or {}).get("tool_reach", 0.13))

        # Motion plan: a tuple (kind, params). The tick() loop owns
        # interpolation. Lock guards mutations.
        self.lock = threading.RLock()
        self.motion = ("hold", {"q": list(self.home_pose)})
        self.last_q = list(self.home_pose)
        self.last_tick_at = time.time()
        self.fault: Optional[str] = None

        # Real-hardware link (a HardwareBackend). Attached by main() when the
        # operator opts in (--hardware-backend / --hardware-ip); None keeps the
        # bridge pure-sim. When connected, act_* forward commands to the real
        # arm and tick() mirrors its measured joints back onto the simulated
        # arm (a live digital twin). _hw_suppress_joint stops the inner joint
        # move (from a TCP solve) from double-commanding the arm when the
        # task-space path already forwarded a move_linear.
        self.hw: Optional[HardwareBackend] = None
        self.hw_name: Optional[str] = None
        self.hw_mirror = True
        self._hw_suppress_joint = False

        # Capabilities surface for /list_robots etc.
        self.capabilities = {
            "joint_names": self.joint_names,
            "joint_limits": [list(lim) for lim in self.joint_limits],
            "home_pose": list(self.home_pose),
            "has_gripper": self.effector is not None,
            "ik_available": cfg.get("ik") is not None,
        }
        if self.effector is not None:
            self.capabilities["gripper"] = self.effector.capabilities()
        if cfg.get("ik"):
            self.capabilities["workspace"] = {
                "min_radius": cfg["ik"]["workspace_min_radius"],
                "max_radius": cfg["ik"]["workspace_max_radius"],
                "min_z": cfg["ik"]["workspace_min_z"],
            }

        # Pending wwi outbox -- bridge -> robot window. Each entry is a
        # raw string already prefixed with the protocol tag.
        self.window_outbox: List[str] = []
        # Set a flag the main loop checks each tick to push the configure
        # handshake to the robot window once it opens.
        self.window_configured = False

        # Apply home pose immediately so we don't drop under gravity
        # before the first command lands.
        for motor, q in zip(self.motors, self.home_pose):
            if motor is not None:
                motor.setPosition(q)

        # One-time diagnostic: walk the URDF subtree and dump every named
        # Solid's world position. Set OMNILINK_ARM_DUMP_TREE=1 to enable.
        # Used during arm-attachment debugging (gripper hand, etc.)
        # to confirm each link landed where the URDF says it should.
        if _os.environ.get("OMNILINK_ARM_DUMP_TREE") == "1":
            try:
                self._dump_tree()
            except Exception as e:
                print(f"[omnilink_arm_bridge] tree dump failed: {e}")

    def _walk_tree_json(self):
        """Return a flat list of {name, model, world_pos} for every named
        Solid in the robot subtree. Used by the /dump_tree HTTP endpoint
        to confirm link positions stay where they should be over time
        (e.g. that the gripper hand doesn't drift off the wrist flange)."""
        out = []
        self_node = self.robot.getSelf()
        if self_node is None:
            return out

        def walk(node):
            try:
                name = ""
                nf = node.getField("name")
                if nf is not None:
                    name = nf.getSFString()
                if name:
                    try:
                        p = node.getPosition() or [None, None, None]
                    except Exception:
                        p = [None, None, None]
                    out.append({
                        "name": name,
                        "model": node.getTypeName(),
                        "world_pos": [
                            float(p[0]) if p[0] is not None else None,
                            float(p[1]) if p[1] is not None else None,
                            float(p[2]) if p[2] is not None else None,
                        ],
                    })
            except Exception:
                pass
            for fname in ("children", "endPoint"):
                try:
                    fld = node.getField(fname)
                    if fld is None:
                        continue
                    try:
                        n = fld.getCount()
                        if n is not None and n > 0:
                            for i in range(n):
                                c = fld.getMFNode(i)
                                if c is not None:
                                    walk(c)
                            continue
                    except Exception:
                        pass
                    try:
                        c = fld.getSFNode()
                        if c is not None:
                            walk(c)
                    except Exception:
                        pass
                except Exception:
                    pass

        walk(self_node)
        return out

    def _dump_tree(self):
        """Walk the Robot's children tree, log each link's world position."""
        out_path = _os.environ.get("OMNILINK_ARM_DUMP_FILE", "/tmp/arm_tree_dump.log")
        try:
            f = open(out_path, "w", encoding="utf-8", buffering=1)
        except Exception as e:
            print(f"[omnilink_arm_bridge] dump_tree: cannot open {out_path}: {e}")
            return
        self_node = self.robot.getSelf()
        if self_node is None:
            f.write("no self node\n")
            f.close()
            return
        f.write(f"=== tree dump for {self.robot_id} ===\n")

        def walk(node, depth=0):
            try:
                model = node.getTypeName()
            except Exception:
                model = "?"
            name = ""
            try:
                nf = node.getField("name")
                if nf is not None:
                    name = nf.getSFString()
            except Exception:
                pass
            pos = ""
            try:
                p = node.getPosition()
                if p is not None and len(p) >= 3:
                    pos = f"world=({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f})"
            except Exception:
                pass
            indent = "  " * depth
            f.write(f"{indent}{model} {name!r} {pos}\n")
            # Try several fields that carry the kinematic chain forward:
            #  - children (MFNode): Solid / Group children
            #  - endPoint (SFNode): HingeJoint / SliderJoint / BallJoint
            for field_name in ("children", "endPoint"):
                try:
                    fld = node.getField(field_name)
                    if fld is None:
                        continue
                    # MFNode: getCount() > 0 (returns -1 for SFNode in some
                    # Webots Python bindings, so use > 0 not >= 0).
                    handled = False
                    try:
                        n = fld.getCount()
                        if n is not None and n > 0:
                            for i in range(n):
                                child = fld.getMFNode(i)
                                if child is not None:
                                    walk(child, depth + 1)
                            handled = True
                    except Exception:
                        pass
                    if not handled:
                        # SFNode (e.g. HingeJoint.endPoint)
                        try:
                            child = fld.getSFNode()
                            if child is not None:
                                walk(child, depth + 1)
                        except Exception:
                            pass
                except Exception as e:
                    f.write(f"{indent}<<error on {field_name}: {e}>>\n")

        walk(self_node, 0)
        f.write("=== end tree dump ===\n")
        f.close()

    # ── Helpers ────────────────────────────────────────────────────

    def _read_q(self) -> List[float]:
        q = []
        for sensor, fallback in zip(self.sensors, self.last_q):
            if sensor is not None:
                try:
                    q.append(sensor.getValue())
                    continue
                except Exception:
                    pass
            q.append(fallback)
        return q

    def _set_q(self, q: List[float]) -> List[float]:
        clamped = clamp_q(q, self.joint_limits)
        for motor, qi in zip(self.motors, clamped):
            if motor is not None:
                motor.setPosition(qi)
        return clamped

    def tcp_xyz(self) -> Optional[Tuple[float, float, float]]:
        ik_cfg = self.cfg.get("ik")
        if not ik_cfg:
            return None
        return forward_kinematics(ik_cfg["chain"], self._read_q(), ik_cfg["tcp_offset"])

    def queue_window(self, line: str) -> None:
        with self.lock:
            self.window_outbox.append(line)

    def _hw_connected(self) -> bool:
        return self.hw is not None and getattr(self.hw, "connected", False)

    def _hw_fwd(self, method: str, *args) -> None:
        """Forward a command to the attached real arm, if a hardware backend is
        connected. No-op in pure sim. Errors are swallowed -- the sim stays the
        source of truth for the UI even if the hardware link hiccups."""
        be = self.hw
        if be is not None and getattr(be, "connected", False):
            try:
                getattr(be, method)(*args)
            except Exception:
                pass

    # ── Tick loop ─────────────────────────────────────────────────

    def tick(self, sim_time_s: float) -> None:
        be = self.hw
        mirror_q = (be.get_joints()
                    if (be is not None and self.hw_mirror
                        and getattr(be, "connected", False))
                    else None)
        with self.lock:
            motion = self.motion
            self.last_q = self._read_q()
            self.last_tick_at = time.time()
            if mirror_q is not None and len(mirror_q) == len(self.joint_names):
                # Digital twin: drive the sim motors straight from the real
                # arm's measured joints; the local motion plan is bypassed
                # (the hardware IS the plan while mirroring).
                self._set_q(mirror_q)
                kind = None
            else:
                kind = motion[0]
            params = motion[1]

            if kind == "hold":
                # Re-apply target each tick so motors don't drift.
                self._set_q(params["q"])

            elif kind == "interp":
                # Linear interp between params["from_q"] and ["to_q"]
                # over ["duration_s"] starting at ["start_s"].
                t = sim_time_s - params["start_s"]
                d = max(params["duration_s"], 1e-3)
                a = clamp(t / d, 0.0, 1.0)
                # Smooth cubic ease.
                a = a * a * (3.0 - 2.0 * a)
                q = [
                    params["from_q"][j] + (params["to_q"][j] - params["from_q"][j]) * a
                    for j in range(len(self.joint_names))
                ]
                self._set_q(q)
                if a >= 1.0:
                    self.motion = ("hold", {"q": list(params["to_q"])})

            elif kind == "wave":
                # Oscillate each joint around home with the wave amps.
                t = sim_time_s - params["start_s"]
                amps = self.cfg.get("wave_amplitudes") or [0.0] * len(self.joint_names)
                omega = 2 * math.pi * 0.8  # 0.8 Hz
                q = [
                    self.home_pose[j] + amps[j] * math.sin(omega * t)
                    for j in range(len(self.joint_names))
                ]
                self._set_q(q)
                if t > params["duration_s"]:
                    self.motion = ("hold", {"q": list(self.home_pose)})

            elif kind == "sequence":
                # Multi-step plan (pick / place): move / wait / grasp / release.
                self._tick_sequence(params, sim_time_s)

        # Mount the gripper visual on the wrist + animate fingers.
        if self.gripper_visual is not None:
            self._update_gripper_visual()

        # Kinematic-attach: teleport the held object to the TCP each tick.
        # Done outside the motion branches so it tracks under any motion.
        if self.held_tfield is not None:
            tcp = self._tcp_world()
            if tcp is not None:
                try:
                    self.held_tfield.setSFVec3f(tcp)
                    self.held_node.resetPhysics()
                except Exception:
                    pass

    def _tick_sequence(self, params: dict, sim_time_s: float) -> None:
        """Advance a multi-step plan (built by act_pick / act_place). Steps:
        {"t":"move","to_q":[...],"dur":s} | {"t":"wait","dur":s}
        | {"t":"grasp"} | {"t":"release"}. Runs under self.lock (tick holds
        it); grasp/release reuse the bridge handlers (reentrant lock)."""
        steps = params["steps"]
        i = params["i"]
        n = len(self.joint_names)
        if i >= len(steps):
            self.motion = ("hold", {"q": self._read_q()})
            return
        step = steps[i]
        st = step.get("t")
        if st == "cmove" and "to_q" not in step:
            # Solve top-down IK fresh at execution time, seeded from the current
            # pose -- the path the proven flagship poses take, so the descent
            # stays vertical and the fingers straddle the cube squarely (a
            # precomputed solution lands a slightly tilted wrist and misses).
            qn = self._topdown_q(step["xyz"], seed=self._read_q(),
                                 tcp_offset_z=step.get("oz"))
            step["to_q"] = qn if qn is not None else list(params["from_q"])
        if st in ("move", "cmove"):
            t = sim_time_s - params["start_s"]
            d = max(float(step.get("dur", 1.2)), 1e-3)
            a = clamp(t / d, 0.0, 1.0)
            a = a * a * (3.0 - 2.0 * a)         # smooth cubic ease
            frm = params["from_q"]
            to = step["to_q"]
            self._set_q([frm[j] + (to[j] - frm[j]) * a for j in range(n)])
            if a >= 1.0:
                params["i"] = i + 1
                params["from_q"] = list(to)
                params["start_s"] = sim_time_s
        elif st == "wait":
            self._set_q(params["from_q"])
            if sim_time_s - params["start_s"] >= float(step.get("dur", 0.3)):
                params["i"] = i + 1
                params["start_s"] = sim_time_s
        elif st in ("grasp", "release", "open"):
            self._set_q(params["from_q"])
            if st == "grasp":
                self.act_grasp()
            elif st == "release":
                self.act_release()
            else:                            # "open": spread fingers (physics)
                self.act_open_gripper()
            params["i"] = i + 1
            params["start_s"] = sim_time_s
        else:
            params["i"] = i + 1

    def _find_mount_node(self):
        """Find the tool-mount link node in the URDF subtree by name. The
        gripper visual rides this node's exact world pose, so it sits on
        the real flange instead of an approximate FK point."""
        cands = self.cfg.get("mount_link") or ["flange", "tool0", "gripper_tcp", "tcp"]
        if isinstance(cands, str):
            cands = [cands]
        found: Dict[str, Any] = {}
        self_node = self.robot.getSelf()
        if self_node is None:
            return None

        def walk(node):
            try:
                nf = node.getField("name")
                if nf is not None:
                    nm = nf.getSFString()
                    if nm in cands and nm not in found:
                        found[nm] = node
            except Exception:
                pass
            for fn in ("children", "endPoint"):
                f = node.getField(fn)
                if f is None:
                    continue
                try:
                    c = f.getCount()
                    if c and c > 0:
                        for i in range(c):
                            ch = f.getMFNode(i)
                            if ch is not None:
                                walk(ch)
                        continue
                except Exception:
                    pass
                try:
                    ch = f.getSFNode()
                    if ch is not None:
                        walk(ch)
                except Exception:
                    pass

        walk(self_node)
        for c in cands:
            if c in found:
                return found[c]
        return None

    def _update_gripper_visual(self) -> None:
        """Ride the real mount-link pose each tick (position + orientation
        straight from the scene tree) and set finger spacing from width."""
        node = self.gripper_anchor
        if node is None:
            return
        try:
            p = node.getPosition()
            o = node.getOrientation()  # flat 9, row-major
            r = [[o[0], o[1], o[2]], [o[3], o[4], o[5]], [o[6], o[7], o[8]]]
            tf = self.gripper_visual.getField("translation")
            rf = self.gripper_visual.getField("rotation")
            if tf is not None:
                tf.setSFVec3f([p[0], p[1], p[2]])
            if rf is not None:
                rf.setSFRotation(_mat3_to_axis_angle(r))
        except Exception:
            return
        # Finger spacing from width (half-stroke each side).
        if self.effector is None:
            return
        w = self.effector.state().get("width")
        if w is None:
            return
        half = clamp(w * 0.5, 0.0, 0.05)
        fl, fr = self.gripper_fingers
        for node, sign in ((fl, 1.0), (fr, -1.0)):
            if node is None:
                continue
            try:
                t = node.getField("translation")
                v = t.getSFVec3f()
                t.setSFVec3f([sign * half, v[1], v[2]])
            except Exception:
                pass

    # ── Action handlers (HTTP + intent share these) ───────────────

    def act_stop(self) -> dict:
        with self.lock:
            q = self._read_q()
            self.motion = ("hold", {"q": q})
        self._hw_fwd("stop")
        return {"halted_at": time.time(), "q": q}

    def act_reset_to_home(self, duration_s: float = 1.5) -> dict:
        with self.lock:
            from_q = self._read_q()
            self.motion = ("interp", {
                "from_q": from_q,
                "to_q": list(self.home_pose),
                "start_s": self.robot.getTime(),
                "duration_s": duration_s,
            })
        self._hw_fwd("reset_to_home")
        return {"q": list(self.home_pose)}

    def act_set_joint_positions(self, q: List[float], duration_s: float = 1.2) -> dict:
        if len(q) != len(self.joint_names):
            return {"error": f"q must have {len(self.joint_names)} entries"}
        clamped = clamp_q(q, self.joint_limits)
        with self.lock:
            from_q = self._read_q()
            self.motion = ("interp", {
                "from_q": from_q,
                "to_q": clamped,
                "start_s": self.robot.getTime(),
                "duration_s": duration_s,
            })
        # Forward the joint move to the real arm -- unless a TCP solve already
        # forwarded a move_linear (then the inner joint move is sim-only).
        if not self._hw_suppress_joint:
            self._hw_fwd("move_joint", clamped)
        return {"accepted": True, "clamped_q": clamped}

    def act_solve_ik(self, xyz: Tuple[float, float, float]) -> dict:
        ik_cfg = self.cfg.get("ik")
        if not ik_cfg:
            return {"error": "ik_unavailable",
                    "hint": f"{self.robot_id} has no pre-baked IK chain; use set_joint_positions instead."}
        # Workspace check
        r = math.sqrt(xyz[0] ** 2 + xyz[1] ** 2 + xyz[2] ** 2)
        if r > ik_cfg["workspace_max_radius"] or r < ik_cfg["workspace_min_radius"] or xyz[2] < ik_cfg["workspace_min_z"]:
            return {"error": "unreachable_target", "radius": r}
        q, err, iters = dls_ik(ik_cfg["chain"], self._read_q(), xyz, ik_cfg, self.joint_limits)
        return {"q": q, "err_norm": err, "iters": iters}

    def act_set_tcp_target(self, xyz: Tuple[float, float, float]) -> dict:
        out = self.act_solve_ik(xyz)
        if "error" in out:
            return out
        # The real arm uses its OWN cartesian IK via move_linear; the sim uses
        # our DLS solution. Suppress the inner joint forward so we don't also
        # double-command the hardware in joint space.
        self._hw_suppress_joint = True
        try:
            clamped = self.act_set_joint_positions(out["q"], duration_s=1.5)
        finally:
            self._hw_suppress_joint = False
        self._hw_fwd("move_linear", tuple(xyz))
        return {"accepted": True, "solved_q": out["q"], "err_norm": out["err_norm"],
                "clamped_q": clamped.get("clamped_q")}

    def act_set_tcp_pose(self, xyz: Tuple[float, float, float],
                         tcp_offset_z: Optional[float] = None,
                         duration_s: float = 1.5) -> dict:
        """6-DOF IK: put the tool at xyz with its +Z axis pointing straight
        DOWN (top-down approach), then move there. `tcp_offset_z` overrides
        the tool point distance from link6 (e.g. the finger throat for a
        grasp); defaults to the config tcp_offset."""
        ik = self.cfg.get("ik")
        if not ik:
            return {"error": "ik_unavailable"}
        oz = ik["tcp_offset"][2] if tcp_offset_z is None else float(tcp_offset_z)
        off = (0.0, 0.0, oz)
        # Tool +Z -> world -Z (rotation pi about world X): top-down.
        r_target = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
        q, perr, rerr, iters = dls_ik_pose(
            ik["chain"], self._read_q(), list(xyz), r_target, off,
            ik, self.joint_limits)
        self._hw_suppress_joint = True
        try:
            self.act_set_joint_positions(q, duration_s=duration_s)
        finally:
            self._hw_suppress_joint = False
        self._hw_fwd("move_linear", tuple(xyz))
        return {"accepted": True, "solved_q": q, "pos_err": perr,
                "rot_err": rerr, "iters": iters, "tcp_offset_z": oz}

    def _topdown_q(self, xyz, seed=None, tcp_offset_z=None):
        """6-DOF top-down IK (tool +Z pointing down) to a base/world xyz.
        Returns the joint solution, or None if no IK chain is configured."""
        ik = self.cfg.get("ik")
        if not ik:
            return None
        oz = ik["tcp_offset"][2] if tcp_offset_z is None else float(tcp_offset_z)
        off = (0.0, 0.0, oz)
        r_target = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
        q, _perr, _rerr, _it = dls_ik_pose(
            ik["chain"], list(seed if seed is not None else self._read_q()),
            list(xyz), r_target, off, ik, self.joint_limits)
        return q

    def act_pick(self, name: Optional[str] = None, approach_h: float = 0.12,
                 grasp_dz: float = 0.02, lift_h: float = 0.18,
                 duration_s: float = 1.4) -> dict:
        """Reach a graspable object top-down, grasp (weld), and lift. Targets
        the named DEF GRASP_<name> object, else the nearest graspable. Falls
        back to a stationary grasp if there's no IK chain or no object. On a
        connected real arm the reach+grip is forwarded to the hardware backend
        (move_linear + grasp) and the sim mirrors it."""
        with self.lock:
            items = list(self._iter_graspables())
        cand = None
        if name:
            key = name.strip().lower()
            for node, tf in items:
                try:
                    d = (node.getDef() or "").lower()
                except Exception:
                    d = ""
                if key and key in d:
                    cand = (node, tf)
                    break
        if cand is None and items:
            tcp = self._tcp_world() or [0.0, 0.0, 0.0]
            best_d = 1e18
            for node, tf in items:
                try:
                    p = tf.getSFVec3f()
                except Exception:
                    continue
                dd = sum((p[k] - tcp[k]) ** 2 for k in range(3))
                if dd < best_d:
                    best_d = dd
                    cand = (node, tf)
        if cand is None or self.cfg.get("ik") is None:
            return self.act_grasp()           # nothing to plan -> close in place
        node, tf = cand
        try:
            p = list(tf.getSFVec3f())
        except Exception:
            return self.act_grasp()
        try:
            target_def = node.getDef()
        except Exception:
            target_def = None
        seed = self._read_q()
        # Physics grip must straddle the cube precisely: put the finger throat
        # on the cube CENTRE (grasp_dz 0), the way the validated pick-place demo
        # does -- a throat 2 cm high catches only the top edge and cams the cube
        # out. The kinematic weld tolerates slack so it keeps its small offset.
        phys = bool((self.gripper_cfg or {}).get("physics_grasp"))
        if phys:
            grasp_dz = 0.0
        # Reach so the GRASP POINT (flange + tool_reach along the tool axis),
        # not the flange itself, lands on the cube -- the flange stays one
        # gripper-length above. Matches _tcp_world's anchor+tool_reach weld.
        goz = self.cfg["ik"]["tcp_offset"][2] + float(self.tool_reach)
        above_xyz = [p[0], p[1], p[2] + approach_h]
        at_xyz = [p[0], p[1], p[2] + grasp_dz]
        lift_xyz = [p[0], p[1], p[2] + lift_h]
        q_above = self._topdown_q(above_xyz, seed=seed, tcp_offset_z=goz)
        q_at = self._topdown_q(at_xyz, seed=q_above or seed, tcp_offset_z=goz)
        q_lift = self._topdown_q(lift_xyz, seed=q_at or seed, tcp_offset_z=goz)
        if q_above is None or q_at is None or q_lift is None:
            return self.act_grasp()           # unreachable -> close in place
        if self._hw_connected():
            self.hw.move_linear((p[0], p[1], p[2] + grasp_dz))
            self.hw.grasp()
            return {"accepted": True, "mode": "hardware",
                    "backend": self.hw_name, "target": target_def, "pos": p}
        if phys:
            # Physics pick: spread the fingers, then descend/lift with the IK
            # solved fresh at each waypoint (cmove) so the wrist stays square and
            # the fingers straddle the cube; ~1.5 s for the force-grip fingers
            # (0.04 m/s) to reach the cube and squeeze to the effort cap before a
            # gentle lift -- mirrors the flagship demo.
            steps = [
                {"t": "open"},
                {"t": "cmove", "xyz": above_xyz, "oz": goz, "dur": duration_s},
                {"t": "cmove", "xyz": at_xyz, "oz": goz, "dur": 0.8},
                {"t": "wait", "dur": 0.3},
                {"t": "grasp"},
                {"t": "wait", "dur": 1.5},
                {"t": "cmove", "xyz": lift_xyz, "oz": goz, "dur": 1.0},
            ]
        else:
            # Kinematic weld: precomputed joint targets are fine (the magnet
            # tolerates pose slack) and the grasp is instant.
            steps = [
                {"t": "move", "to_q": q_above, "dur": duration_s},
                {"t": "move", "to_q": q_at, "dur": 0.8},
                {"t": "wait", "dur": 0.2},
                {"t": "grasp"},
                {"t": "wait", "dur": 0.2},
                {"t": "move", "to_q": q_lift, "dur": 0.8},
            ]
        with self.lock:
            self.motion = ("sequence", {"steps": steps, "i": 0,
                                        "from_q": self._read_q(),
                                        "start_s": self.robot.getTime()})
        return {"accepted": True, "target": target_def, "pos": p}

    def act_place(self, xyz=None, approach_h: float = 0.14, drop_dz: float = 0.04,
                  lift_h: float = 0.18, duration_s: float = 1.4) -> dict:
        """Carry the held object to a drop location top-down and release it.
        Defaults to the arm cfg's drop_zone. Falls back to a stationary
        release if there's no IK chain."""
        if xyz is None:
            xyz = self.cfg.get("drop_zone") or [0.30, 0.34, 0.0]
        p = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
        if self.cfg.get("ik") is None:
            return self.act_release()
        if self._hw_connected():
            self.hw.move_linear((p[0], p[1], p[2] + drop_dz))
            self.hw.release()
            return {"accepted": True, "mode": "hardware",
                    "backend": self.hw_name, "place": p}
        seed = self._read_q()
        phys = bool((self.gripper_cfg or {}).get("physics_grasp"))
        goz = self.cfg["ik"]["tcp_offset"][2] + float(self.tool_reach)
        above_xyz = [p[0], p[1], p[2] + approach_h]
        at_xyz = [p[0], p[1], p[2] + drop_dz]
        lift_xyz = [p[0], p[1], p[2] + lift_h]
        q_above = self._topdown_q(above_xyz, seed=seed, tcp_offset_z=goz)
        q_at = self._topdown_q(at_xyz, seed=q_above or seed, tcp_offset_z=goz)
        q_lift = self._topdown_q(lift_xyz, seed=q_at or seed, tcp_offset_z=goz)
        if q_above is None or q_at is None:
            return self.act_release()
        if phys:
            # Carry the gripped cube with fresh IK at each waypoint so the wrist
            # stays square and the friction grip holds until the release.
            steps = [
                {"t": "cmove", "xyz": above_xyz, "oz": goz, "dur": duration_s},
                {"t": "cmove", "xyz": at_xyz, "oz": goz, "dur": 0.8},
                {"t": "wait", "dur": 0.2},
                {"t": "release"},
                {"t": "wait", "dur": 0.3},
                {"t": "cmove", "xyz": lift_xyz, "oz": goz, "dur": 0.8},
            ]
        else:
            steps = [
                {"t": "move", "to_q": q_above, "dur": duration_s},
                {"t": "move", "to_q": q_at, "dur": 0.8},
                {"t": "wait", "dur": 0.2},
                {"t": "release"},
                {"t": "wait", "dur": 0.2},
                {"t": "move", "to_q": q_lift or q_above, "dur": 0.8},
            ]
        with self.lock:
            self.motion = ("sequence", {"steps": steps, "i": 0,
                                        "from_q": self._read_q(),
                                        "start_s": self.robot.getTime()})
        return {"accepted": True, "place": p}

    def act_node_pose(self, def_name: str) -> dict:
        """World position of a scene node by DEF name (verification/debug)."""
        try:
            node = self.robot.getFromDef(def_name)
            if node is None:
                return {"error": "not_found", "def": def_name}
            p = node.getPosition()
            return {"def": def_name, "pos": [p[0], p[1], p[2]]}
        except Exception as e:
            return {"error": repr(e)}

    def act_wave(self, duration_s: float = 6.0) -> dict:
        with self.lock:
            self.motion = ("wave", {
                "start_s": self.robot.getTime(),
                "duration_s": duration_s,
            })
        self._hw_fwd("wave",
                        self.cfg.get("wave_amplitudes")
                        or [0.0] * len(self.joint_names))
        return {"started": True, "duration_s": duration_s}

    def act_open_gripper(self) -> dict:
        if self.effector is None:
            return {"error": "effector_unavailable"}
        st = self.effector.open()
        out = {"state": "open", "gripper": st}
        # Opening the fingers also releases any cube the grasp is holding (the
        # physics-grasp attach or the kinematic weld) so it drops under gravity
        # -- otherwise it stays stuck to the tool even with the fingers wide
        # open. "put it down" / "drop" go via act_release; this covers a bare
        # "open the gripper".
        with self.lock:
            if self.held_node is not None:
                try:
                    out["released"] = self.held_node.getDef()
                    self.held_node.resetPhysics()  # let it fall under gravity
                except Exception:
                    pass
            self.held_node = None
            self.held_tfield = None
        return out

    def act_close_gripper(self) -> dict:
        if self.effector is None:
            return {"error": "effector_unavailable"}
        st = self.effector.close()
        return {"state": "closed", "gripper": st}

    def act_set_gripper_width(self, width_m: float) -> dict:
        if self.effector is None:
            return {"error": "effector_unavailable"}
        return {"gripper": self.effector.set_width(width_m)}

    # ── Kinematic-attach helpers (Phase 3) ────────────────────────
    def _tcp_world(self) -> Optional[List[float]]:
        """World-frame grasp point. Prefers the REAL mount-link pose
        (flange world pose + tool_reach along its +Z), so the grasp weld
        lands exactly where the gripper is drawn. Falls back to FK
        (base pose x FK TCP) when no anchor is available."""
        node = getattr(self, "gripper_anchor", None)
        if node is not None:
            try:
                p = node.getPosition()
                o = node.getOrientation()  # flat 9, row-major; col2 = tool +Z
                rr = self.tool_reach
                return [p[0] + o[2] * rr, p[1] + o[5] * rr, p[2] + o[8] * rr]
            except Exception:
                pass
        local = self.tcp_xyz()
        if local is None or self._self_node is None:
            return None
        try:
            bp = self._self_node.getPosition()           # world translation
            r = self._self_node.getOrientation()         # 3x3 row-major
        except Exception:
            return list(local)
        lx, ly, lz = local
        return [
            bp[0] + r[0] * lx + r[1] * ly + r[2] * lz,
            bp[1] + r[3] * lx + r[4] * ly + r[5] * lz,
            bp[2] + r[6] * lx + r[7] * ly + r[8] * lz,
        ]

    def _iter_graspables(self):
        """Yield (node, translation_field) for every world node whose DEF
        starts with 'GRASP_'. Walks the scene-tree root children."""
        root = self.robot.getRoot()
        if root is None:
            return
        kids = root.getField("children")
        if kids is None:
            return
        try:
            n = kids.getCount()
        except Exception:
            return
        for i in range(n):
            node = kids.getMFNode(i)
            if node is None:
                continue
            try:
                d = node.getDef() or ""
            except Exception:
                d = ""
            if not d.startswith("GRASP_"):
                continue
            tf = node.getField("translation")
            if tf is not None:
                yield node, tf

    def _attach_nearest(self, tcp: List[float]) -> Optional[str]:
        """Weld the nearest GRASP_ object within grasp_radius to the tool."""
        best = None
        best_d = self.grasp_radius
        for node, tf in self._iter_graspables():
            try:
                p = tf.getSFVec3f()
            except Exception:
                continue
            d = math.sqrt((p[0] - tcp[0]) ** 2 + (p[1] - tcp[1]) ** 2
                          + (p[2] - tcp[2]) ** 2)
            if d <= best_d:
                best_d = d
                best = (node, tf)
        if best is None:
            return None
        self.held_node, self.held_tfield = best
        try:
            self.held_node.resetPhysics()
        except Exception:
            pass
        try:
            return self.held_node.getDef()
        except Exception:
            return "GRASP_object"

    def act_grasp(self, force: Optional[float] = None,
                  width: Optional[float] = None) -> dict:
        # The real arm's gripper is driven by the hardware backend even when no
        # sim effector is configured (e.g. a bare arm demo with no gripper).
        self._hw_fwd("grasp")
        if self.effector is None:
            if self._hw_connected():
                return {"gripper": {"kind": "hardware", "holding": True},
                        "mode": "hardware", "backend": self.hw_name}
            return {"error": "effector_unavailable"}
        gstate = self.effector.grasp(force=force, width=width)
        out = {"gripper": gstate}
        # Physics grasp: the real fingers close on the part and contact friction
        # does the grabbing (visible pinch). A position-servo pinch alone slowly
        # creeps under gravity in the MuJoCo contact over a long hold, so for a
        # reliable interactive hold we ALSO attach the gripped part to the tool
        # once the fingers are around it -- it stays where the fingers hold it
        # instead of drifting out. The fingers still do the grasping; release
        # opens them and detaches so the part falls under physics.
        if (self.gripper_cfg or {}).get("physics_grasp"):
            out["mode"] = "physics"
            tcp = self._tcp_world()
            if tcp is not None:
                with self.lock:
                    out["attached"] = self._attach_nearest(tcp)
            return out
        tcp = self._tcp_world()
        if tcp is not None:
            with self.lock:
                attached = self._attach_nearest(tcp)
            out["attached"] = attached
            out["tcp_world"] = tcp
            if attached is None:
                out["note"] = "no graspable (DEF GRASP_*) within grasp_radius"
        return out

    def act_release(self) -> dict:
        self._hw_fwd("release")
        if self.effector is None:
            if self._hw_connected():
                return {"gripper": {"kind": "hardware", "holding": False},
                        "released": None, "mode": "hardware",
                        "backend": self.hw_name}
            return {"error": "effector_unavailable"}
        gstate = self.effector.release()
        with self.lock:
            dropped = None
            if self.held_node is not None:
                try:
                    dropped = self.held_node.getDef()
                    self.held_node.resetPhysics()  # let it fall under gravity
                except Exception:
                    pass
            self.held_node = None
            self.held_tfield = None
        return {"gripper": gstate, "released": dropped}

    def hw_status(self) -> dict:
        """Hardware-link status. {"enabled": False} in a pure-sim run."""
        if self.hw is None:
            return {"enabled": False}
        st = dict(self.hw.status())
        st.setdefault("backend", self.hw_name)
        return st

    def get_state(self) -> dict:
        q = self._read_q()
        tcp = self.tcp_xyz()
        return {
            "id": self.robot_id,
            "model": self.cfg["model"],
            "q": q,
            "tcp": list(tcp) if tcp else None,
            "gripper": self.effector.state() if self.effector else None,
            "fault": self.fault,
            "last_tick_at": self.last_tick_at,
            "sim_time": self.robot.getTime(),
            "mode": self.motion[0],
            "hardware": self.hw_status(),
        }


# ── Intent router ────────────────────────────────────────────────────

class IntentRouter:
    """Maps free-text prompts to bridge actions.

    Designed for demo prompts -- "go home", "wave hello", "joint 3 to 1",
    "move to 0.4 0.2 0.3", "open the gripper", "stop". Returns a result
    dict the bridge surfaces as agent + tool lines in the robot window.
    """

    NUMBER = r"(-?\d+\.?\d*)"

    def __init__(self, bridge: ArmBridge):
        self.bridge = bridge

    def dispatch(self, text: str) -> dict:
        s = text.strip().lower()
        if not s:
            return {"agent": "(empty prompt)", "tools": []}

        # ── stop / halt ─────────────────────────────────────────
        if re.search(r"\b(stop|halt|freeze|hold)\b", s):
            res = self.bridge.act_stop()
            return {
                "agent": "Stopping. Freezing at current joint angles.",
                "tools": [("stop_robot", "ok", f"q frozen at {self._fmt_q(res['q'])}")],
            }

        # ── home / reset ────────────────────────────────────────
        if re.search(r"\b(home|reset|park|tuck)\b", s) or "go to home" in s:
            res = self.bridge.act_reset_to_home()
            return {
                "agent": f"Moving to home pose ({len(res['q'])} joints).",
                "tools": [("reset_to_home", "ok", "interpolating 1.5 s")],
            }

        # ── wave / dance ────────────────────────────────────────
        if re.search(r"\b(wave|hello|dance|demo|show ?off)\b", s):
            res = self.bridge.act_wave()
            return {
                "agent": "Waving hello — give me ~6 seconds.",
                "tools": [("wave", "ok", "0.8 Hz oscillation")],
            }

        # ── pick / place / grasp / release / width / open / close ───
        # "pick up [the] [colour] cube" -> reach top-down, grasp, lift
        if re.search(r"\b(grab|grasp)\b", s) or ("pick" in s and "up" in s):
            name = None
            mc = re.search(r"\b(red|blue|green|yellow|orange|purple)\b", s)
            if mc:
                name = mc.group(1)
            res = self.bridge.act_pick(name)
            ok = "error" not in res
            tgt = res.get("target") or (name or "the nearest object")
            return {
                "agent": (f"Picking up {tgt}." if ok
                          else "I couldn't plan that pick."),
                "tools": [("pick", "ok" if ok else "err",
                           str(res.get("pos") or res.get("error", "")))],
            }
        # "put it down" / "place it" / "set it down" -> carry to drop zone + release
        if re.search(r"\b(put (it |that )?(down|away|back)|place|set (it |that )?down|drop (it )?off)\b", s):
            res = self.bridge.act_place()
            ok = "error" not in res
            return {
                "agent": ("Setting it down." if ok
                          else "I couldn't plan that place."),
                "tools": [("place", "ok" if ok else "err",
                           str(res.get("place") or res.get("error", "")))],
            }
        # "release" / "let go" / "drop it" -> open + drop in place
        if re.search(r"\b(release|let go|drop)\b", s):
            res = self.bridge.act_release()
            ok = "error" not in res
            return {
                "agent": "Releasing." if ok else "This arm has no gripper.",
                "tools": [("release", "ok" if ok else "err", self._gsum(res))],
            }
        # set width -> "open to 3 cm", "40 mm", "halfway"
        if "gripper" in s or re.search(r"\b(width|wide|halfway|half)\b", s):
            w = self._parse_width(s)
            if w is not None:
                res = self.bridge.act_set_gripper_width(w)
                ok = "error" not in res
                return {
                    "agent": (f"Setting gripper to {w * 1000:.0f} mm." if ok
                              else "This gripper has no width control."),
                    "tools": [("set_gripper_width", "ok" if ok else "err",
                               self._gsum(res))],
                }
        # open / close
        if re.search(r"\b(open)\b", s) and "gripper" in s or s.strip() == "open":
            res = self.bridge.act_open_gripper()
            ok = "error" not in res
            return {
                "agent": "Opening the gripper." if ok else "This arm has no gripper.",
                "tools": [("open_gripper", "ok" if ok else "err", res.get("state") or res.get("error", ""))],
            }
        if re.search(r"\b(close|grip)\b", s) and "gripper" in s or s.strip() == "close":
            res = self.bridge.act_close_gripper()
            ok = "error" not in res
            return {
                "agent": "Closing the gripper." if ok else "This arm has no gripper.",
                "tools": [("close_gripper", "ok" if ok else "err", res.get("state") or res.get("error", ""))],
            }

        # ── move / go to (x y z) ────────────────────────────────
        m = re.search(
            r"(?:go|move|tcp|target)[^-\d]*"
            r"\(?\s*" + self.NUMBER + r"[ ,]+" + self.NUMBER + r"[ ,]+" + self.NUMBER + r"\s*\)?",
            s,
        )
        if m:
            xyz = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
            res = self.bridge.act_set_tcp_target(xyz)
            if "error" in res:
                return {
                    "agent": f"Can't reach {xyz}: {res['error']}",
                    "tools": [("set_tcp_target", "err", res["error"])],
                }
            return {
                "agent": f"Moving to TCP {xyz}, err {res['err_norm']:.4f}.",
                "tools": [("set_tcp_target", "ok", f"err={res['err_norm']:.4f}")],
            }

        # ── joint N to V ────────────────────────────────────────
        m = re.search(r"joint\s*(\d+)\s*(?:to|=)\s*" + self.NUMBER, s)
        if m:
            idx = int(m.group(1)) - 1
            val = float(m.group(2))
            if not (0 <= idx < len(self.bridge.joint_names)):
                return {
                    "agent": f"Joint index out of range (have {len(self.bridge.joint_names)} joints).",
                    "tools": [("set_joint_positions", "err", "index out of range")],
                }
            q = list(self.bridge._read_q())
            q[idx] = val
            res = self.bridge.act_set_joint_positions(q)
            return {
                "agent": f"Setting joint {idx + 1} to {val:.3f} rad.",
                "tools": [("set_joint_positions", "ok", self._fmt_q(res["clamped_q"]))],
            }

        # ── joints to [a b c d e f] ─────────────────────────────
        m = re.search(r"joints?\s*(?:to|=)\s*\[?\s*([-\d. ,]+?)\s*\]?$", s)
        if m:
            try:
                q = [float(x) for x in re.split(r"[,\s]+", m.group(1)) if x]
                if len(q) == len(self.bridge.joint_names):
                    res = self.bridge.act_set_joint_positions(q)
                    return {
                        "agent": f"Moving all {len(q)} joints.",
                        "tools": [("set_joint_positions", "ok", self._fmt_q(res["clamped_q"]))],
                    }
            except Exception:
                pass

        # ── status / state ──────────────────────────────────────
        if re.search(r"\b(status|state|where|pose|telemetry)\b", s):
            st = self.bridge.get_state()
            q_s = self._fmt_q(st["q"])
            tcp_s = (f", TCP=({st['tcp'][0]:.2f}, {st['tcp'][1]:.2f}, {st['tcp'][2]:.2f})"
                     if st["tcp"] else "")
            return {
                "agent": f"q={q_s}{tcp_s}, mode={st['mode']}.",
                "tools": [("get_robot_state", "ok", q_s)],
            }

        # ── unknown ─────────────────────────────────────────────
        # Offline regex router (no OmniLink LLM attached). Say so, so the
        # operator knows free-form chat needs OMNI_KEY, and list commands
        # that actually work -- including the pick/place verbs.
        return {
            "agent": ("I'm on the offline command router (no OmniLink agent "
                      "connected), so I only understand set phrases. Try: "
                      "\"pick up the red cube\", \"put it down\", \"wave\", "
                      "\"go home\", \"stop\", \"move to 0.4 0.2 0.3\", "
                      "\"joint 3 to 1.5\", or \"open the gripper\"."),
            "tools": [],
        }

    @staticmethod
    def _fmt_q(q: List[float]) -> str:
        return "[" + ", ".join(f"{qi:+.2f}" for qi in q) + "]"

    @staticmethod
    def _gsum(res: dict) -> str:
        """One-line gripper summary for a tool-result line."""
        if "error" in res:
            return res["error"]
        g = res.get("gripper") or {}
        w = g.get("width")
        bits = [g.get("kind", "gripper")]
        if w is not None:
            bits.append(f"{w * 1000:.0f}mm")
        if g.get("holding"):
            bits.append("holding")
        return " ".join(bits)

    def _parse_width(self, s: str) -> Optional[float]:
        """Parse a target opening width (metres) from free text.

        Understands "40 mm" / "3 cm" / "0.04 m" and the words
        "halfway" / "half" (-> half of max_width). Returns None if no
        width is expressed or the gripper has no width control."""
        eff = self.bridge.effector
        if eff is None or eff.max_width <= 0.0:
            return None
        if re.search(r"\b(halfway|half)\b", s):
            return eff.max_width * 0.5
        m = re.search(r"(-?\d+\.?\d*)\s*(mm|millimet|cm|centimet|m\b|metre|meter)", s)
        if not m:
            return None
        val = float(m.group(1))
        unit = m.group(2)
        if unit.startswith("mm") or unit.startswith("millimet"):
            return val / 1000.0
        if unit.startswith("cm") or unit.startswith("centimet"):
            return val / 100.0
        return val  # metres


# ── HTTP server ──────────────────────────────────────────────────────

def make_handler(bridge: ArmBridge, router: IntentRouter, relay: Any = None):
    class _H(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return  # quiet

        def _json(self, code: int, obj: Any) -> None:
            data = json.dumps(obj, default=str).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self) -> dict:
            n = int(self.headers.get("Content-Length", "0"))
            if n <= 0:
                return {}
            try:
                return json.loads(self.rfile.read(n).decode("utf-8") or "{}")
            except Exception:
                return {}

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_GET(self):
            if self.path in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if self.path in ("/capabilities", "/list_robots"):
                return self._json(200, [{
                    "id": bridge.robot_id,
                    "model": bridge.cfg["model"],
                    "capabilities": bridge.capabilities,
                }])
            if self.path == "/dump_tree":
                return self._json(200, bridge._walk_tree_json())
            if self.path == "/hardware_status":
                return self._json(200, bridge.hw_status())
            if self.path == "/usage":
                # Latest per-turn usage delta (tokens + credits) from the
                # OmniLink platform rollup. None until at least one chat
                # turn has completed. Bridge surfaces this so the side
                # menu can show a running tally without intercepting
                # chat events.
                if relay is None:
                    return self._json(200, {"enabled": False})
                return self._json(200, {
                    "enabled": True,
                    "latest": relay.latest_usage(),
                })
            path0 = self.path.split("?")[0]
            if path0 == "/chat":
                # Full-page browser chat (same-origin: it fetches /prompt,
                # /get_robot_state, /chat_config from this very server).
                data = CHAT_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if path0 == "/chat_config":
                return self._json(200, build_window_config(bridge, relay))
            return self._json(404, {"error": "not_found"})

        def do_POST(self):
            body = self._read_json()
            path = self.path.rstrip("/")
            if path in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if path in ("/list_robots", "/capabilities"):
                return self._json(200, [{
                    "id": bridge.robot_id,
                    "model": bridge.cfg["model"],
                    "capabilities": bridge.capabilities,
                }])
            if path == "/read_joints":
                return self._json(200, {"q": bridge._read_q()})
            if path == "/read_tcp_pose":
                tcp = bridge.tcp_xyz()
                return self._json(200, {"xyz": list(tcp) if tcp else None})
            if path == "/stop_robot":
                return self._json(200, bridge.act_stop())
            if path == "/reset_to_home":
                return self._json(200, bridge.act_reset_to_home())
            if path == "/set_joint_positions":
                q = body.get("q") or []
                return self._json(200, bridge.act_set_joint_positions(q))
            if path == "/set_tcp_target":
                xyz = body.get("xyz") or []
                if len(xyz) != 3:
                    return self._json(400, {"error": "xyz must have 3 entries"})
                return self._json(200, bridge.act_set_tcp_target(tuple(xyz)))
            if path == "/set_tcp_pose":
                xyz = body.get("xyz") or []
                if len(xyz) != 3:
                    return self._json(400, {"error": "xyz must have 3 entries"})
                return self._json(200, bridge.act_set_tcp_pose(
                    tuple(xyz), tcp_offset_z=body.get("tcp_offset_z"),
                    duration_s=float(body.get("duration_s", 1.5))))
            if path == "/solve_ik":
                xyz = body.get("xyz") or []
                if len(xyz) != 3:
                    return self._json(400, {"error": "xyz must have 3 entries"})
                return self._json(200, bridge.act_solve_ik(tuple(xyz)))
            if path == "/node_pose":
                return self._json(200, bridge.act_node_pose(body.get("def", "")))
            if path == "/open_gripper":
                return self._json(200, bridge.act_open_gripper())
            if path == "/close_gripper":
                return self._json(200, bridge.act_close_gripper())
            if path == "/set_gripper_width":
                w = body.get("width")
                if w is None:
                    return self._json(400, {"error": "width (metres) required"})
                return self._json(200, bridge.act_set_gripper_width(float(w)))
            if path == "/grasp":
                return self._json(200, bridge.act_grasp(
                    force=body.get("force"), width=body.get("width")))
            if path == "/release":
                return self._json(200, bridge.act_release())
            if path == "/pick":
                return self._json(200, bridge.act_pick(body.get("object")))
            if path == "/place":
                return self._json(200, bridge.act_place(body.get("xyz")))
            if path == "/dump_tree":
                # Diagnostic: world position of every named Solid in the
                # robot's subtree. Used to verify fixed-joint child links
                # stay attached over time (e.g. the gripper hand vs. flange).
                return self._json(200, bridge._walk_tree_json())
            if path == "/prompt":
                text = (body.get("text") or "").strip()
                if not text:
                    return self._json(400, {"error": "text required"})
                if relay is not None:
                    return self._json(200, relay.dispatch_sync(text))
                result = router.dispatch(text)
                return self._json(200, {
                    "response": result["agent"],
                    "actions": [{"tool": t[0], "result": t[1], "summary": t[2]}
                                for t in result["tools"]],
                })
            if path == "/tool":
                # Platform-side tool callback. The omnilink-agents.com web
                # UI POSTs {"tool": "<name>", ...args} here after a chat
                # turn produces toolCalls. We dispatch via the relay's
                # registered Tool and return the dispatch result.
                tool_name = (body.pop("tool", None) or "").strip()
                if not tool_name:
                    return self._json(400, {"error": "tool name required"})
                if relay is None or tool_name not in getattr(relay, "tools", {}):
                    return self._json(503, {
                        "status": "err",
                        "tool": tool_name,
                        "error": "tool_not_registered",
                    })
                try:
                    result = relay.tools[tool_name].dispatch(body)
                    return self._json(200, {
                        "status": "ok",
                        "tool": tool_name,
                        "result": result,
                    })
                except Exception as e:
                    return self._json(500, {
                        "status": "err",
                        "tool": tool_name,
                        "error": repr(e),
                    })
            return self._json(404, {"error": "not_found", "path": path})
    return _H


def start_http(bridge: ArmBridge, router: IntentRouter, port: int, relay: Any = None) -> ThreadingHTTPServer:
    handler = make_handler(bridge, router, relay)
    server = ThreadingHTTPServer(("127.0.0.1", port), handler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    print(f"[omnilink_arm_bridge] HTTP listening on http://127.0.0.1:{port}")
    return server


# ── OmniLink tool builders ───────────────────────────────────────────

def build_arm_tools(bridge: ArmBridge) -> List[Any]:
    """Wrap the arm bridge's actions as OmniLink Tool definitions.

    Returns an empty list if the relay package isn't importable -- the
    bridge will fall back to its local intent router.
    """
    if Tool is None:
        return []
    n = len(bridge.joint_names)
    tools: List[Any] = [
        Tool(
            name="reset_to_home",
            description=f"Move the {bridge.cfg['model']} arm to its home pose ({n} joints).",
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_reset_to_home(),
        ),
        Tool(
            name="set_joint_positions",
            description=(
                f"Command joint-space setpoint for {n} joints. q is a list of "
                f"radians ordered: {bridge.joint_names}. The bridge clamps each "
                f"value to its joint limit; oversize moves are interpolated over ~1.2 s."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "q": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": n,
                        "maxItems": n,
                        "description": f"Joint angles (rad). Length must be {n}.",
                    },
                },
                "required": ["q"],
            },
            dispatch=lambda args: bridge.act_set_joint_positions(list(args.get("q", []))),
        ),
        Tool(
            name="wave",
            description="Oscillate the joints for ~6 s as a 'hello' gesture.",
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_wave(),
        ),
        Tool(
            name="stop_robot",
            description="Emergency halt — freeze the arm at its current joint angles.",
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_stop(),
        ),
        Tool(
            name="get_robot_state",
            description="Read the freshest q, TCP, fault, and motion mode.",
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.get_state(),
        ),
    ]
    if bridge.cfg.get("ik"):
        tools.append(Tool(
            name="set_tcp_target",
            description=(
                "Command TCP (world-frame) position via damped-least-squares IK. "
                "Bridge rejects targets outside the reachable workspace shell."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "xyz": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 3, "maxItems": 3,
                        "description": "World-frame TCP position [x, y, z] in metres.",
                    },
                },
                "required": ["xyz"],
            },
            dispatch=lambda args: bridge.act_set_tcp_target(tuple(args.get("xyz") or [])),
        ))
    if bridge.effector is not None:
        tools.append(Tool(
            name="open_gripper",
            description="Open the gripper (fingers move to the configured open width).",
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_open_gripper(),
        ))
        tools.append(Tool(
            name="close_gripper",
            description="Close the gripper.",
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_close_gripper(),
        ))
        tools.append(Tool(
            name="grasp",
            description=(
                "Grasp: close the gripper to hold an object. Optionally pass a "
                "target opening width (metres) or grip force."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "width": {"type": "number",
                              "description": "Target opening in metres (optional)."},
                    "force": {"type": "number",
                              "description": "Grip force, 0-1 normalised (optional)."},
                },
            },
            dispatch=lambda args: bridge.act_grasp(
                force=args.get("force"), width=args.get("width")),
        ))
        tools.append(Tool(
            name="release",
            description="Release: open the gripper and drop any held object.",
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_release(),
        ))
        if bridge.effector.capabilities().get("has_width_control"):
            tools.append(Tool(
                name="set_gripper_width",
                description=(
                    "Set the gripper opening width in metres "
                    f"(0 = closed, {bridge.effector.max_width:.3f} = fully open)."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "width": {"type": "number",
                                  "description": "Opening in metres."},
                    },
                    "required": ["width"],
                },
                dispatch=lambda args: bridge.act_set_gripper_width(
                    float(args.get("width") or 0.0)),
            ))
    if bridge.cfg.get("ik") and bridge.effector is not None:
        tools.append(Tool(
            name="pick",
            description=(
                "Pick up an object: reach it top-down, close the gripper, and "
                "lift. Optionally name the object (e.g. a colour like 'red') to "
                "target a specific one; otherwise the nearest graspable is used."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "object": {
                        "type": "string",
                        "description": "Name/colour of the object to pick (optional).",
                    },
                },
            },
            dispatch=lambda args: bridge.act_pick(args.get("object")),
        ))
        tools.append(Tool(
            name="place",
            description=(
                "Place the held object: carry it to a drop location and release. "
                "Defaults to the configured drop zone if no xyz is given."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "xyz": {
                        "type": "array", "items": {"type": "number"},
                        "minItems": 3, "maxItems": 3,
                        "description": "World-frame drop position [x, y, z] in metres (optional).",
                    },
                },
            },
            dispatch=lambda args: bridge.act_place(args.get("xyz")),
        ))
    return tools


def build_arm_main_task(bridge: ArmBridge) -> str:
    has_ik = bool(bridge.cfg.get("ik"))
    has_gripper = bridge.effector is not None
    gripper_label = bridge.effector.model if has_gripper else ""
    persona = bridge.cfg.get("persona")
    capabilities = (
        f"You control a {bridge.cfg['model']} robot arm in OmniSim through "
        f"the OmniLink-OmniSim bridge. Joints: {bridge.joint_names}. "
        f"Home pose: {bridge.home_pose}. "
        f"{'You can issue task-space TCP targets via set_tcp_target. ' if has_ik else 'No IK is wired for this arm — use set_joint_positions for motion. '}"
        f"{f'A {gripper_label} end effector is available: grasp / release to pick and drop, open_gripper / close_gripper, and set_gripper_width when width control is supported.' if has_gripper else ''}"
    )
    rules = (
        "Rules:\n"
        "- Translate the operator's request into ONE tool call when motion is implied.\n"
        "- Read state via get_robot_state if you need it before commanding.\n"
        "- 'home' / 'reset' -> reset_to_home. 'wave' / 'hello' -> wave.\n"
        "- 'stop' / 'halt' -> stop_robot, always.\n"
        "- Refuse and explain if a target is clearly outside the workspace.\n"
    )
    # Persona mode (when the arm config ships one): lead with the character, then the body it
    # has to drive tools, then the rules. Conversation that doesn't imply
    # motion gets a plain spoken reply with no tool call.
    if persona:
        return (
            persona + "\n\n"
            "How your body works (use this to pick the right tool):\n"
            + capabilities + "\n\n"
            + rules
            + "- Pure chat or questions about yourself need NO tool — just answer.\n"
            + "- Replies may be read aloud: keep them to one or two short, "
            "natural sentences."
        )
    return (
        capabilities + "\n\n" + rules
        + "- Keep the final text response short -- one sentence."
    )


def setup_omnilink_relay(bridge: ArmBridge, http_port: int = 8765) -> Optional[Any]:
    if OmniLinkRelay is None or not omnilink_enabled():
        return None
    try:
        agent_name = f"OmniSim-{bridge.robot_id}"
        tools = build_arm_tools(bridge)
        main_task = build_arm_main_task(bridge)
        relay = OmniLinkRelay(
            omni_key=get_omni_key(),
            agent_name=agent_name,
            main_task=main_task,
            tools=tools,
        )
        # Profile sync: push an agent profile to the platform so the
        # operator can pick this robot in the omnilink-agents.com web
        # UI and chat to it from there. The platform's UI POSTs
        # structured tool calls back to toolCallbackUrl, which we
        # serve at /tool below.
        from _omnilink_relay import profile_sync
        if profile_sync.is_enabled():
            profile_sync.ensure_profile(
                client=relay._client,
                agent_name=agent_name,
                main_task=main_task,
                tool_defs=[t.to_definition() for t in tools],
                engine=relay.engine,
                tool_callback_url=f"http://127.0.0.1:{http_port}/tool",
            )
        print(f"[omnilink_arm_bridge] OmniLink relay ON (agent='{agent_name}')")
        return relay
    except Exception as e:
        print(f"[omnilink_arm_bridge] OmniLink relay setup failed: {e}")
        return None


# ── Robot window plumbing ────────────────────────────────────────────

def build_window_config(bridge: ArmBridge, relay: Any) -> Dict[str, Any]:
    """Shared UI config used by BOTH the docked robot window (configure
    handshake) and the GET /chat_config endpoint the browser chat reads,
    so the two surfaces stay in lockstep (same persona, suggestions, etc.)."""
    # A persona config ships its own chat-flavoured suggestions;
    # otherwise build the generic command set from the arm's capabilities.
    suggestions = list(bridge.cfg.get("suggestions") or [])
    if not suggestions:
        suggestions = ["home", "wave hello", "joint 1 to 0.5", "stop"]
        if bridge.cfg.get("ik"):
            suggestions.insert(2, "move to 0.4 0.2 0.4")
        if bridge.effector is not None:
            suggestions.append("open the gripper")
            suggestions.append("grasp")
            if bridge.effector.capabilities().get("has_width_control"):
                suggestions.append("open gripper to 4 cm")
    agent_label = (
        f"OmniLink relay ({_os.environ.get('OMNILINK_ENGINE', 'g4-engine')})"
        if relay is not None else "local intent (regex)"
    )
    return {
        "robot": bridge.cfg["model"],
        "robot_class": "arm",
        "agent": agent_label,
        "suggestions": suggestions,
        "home": list(bridge.home_pose),
        # Persona UI hints (the window shows a name + tagline + avatar when
        # display_name is present; otherwise the generic robot console).
        "display_name": bridge.cfg.get("display_name"),
        "tagline": bridge.cfg.get("tagline"),
        "greeting": bridge.cfg.get("greeting"),
        # Voice in the docked panel needs the relay (STT via audio_in). The
        # browser chat uses the Web Speech API and ignores this flag.
        "voice": relay is not None,
        "has_ik": bool(bridge.cfg.get("ik")),
        "has_gripper": bridge.effector is not None,
        "hardware": bridge.hw_status(),
    }


def push_configure(bridge: ArmBridge, relay: Any) -> None:
    cfg = build_window_config(bridge, relay)
    bridge.queue_window("configure:" + json.dumps(cfg))
    bridge.queue_window("status:connected")
    # A warm opening line so the panel greets the operator instead of
    # sitting empty. Text-only — TTS fires on real relay replies.
    greeting = cfg.get("greeting")
    if greeting:
        bridge.queue_window("agent:" + greeting)
    bridge.window_configured = True


def _on_relay_event(bridge: ArmBridge, kind: str, payload: Dict[str, Any]) -> None:
    if kind == "status":
        bridge.queue_window(f"status:{payload.get('state', 'idle')}")
    elif kind == "tool":
        name = payload.get("name", "?")
        status = payload.get("status", "ok")
        summary = payload.get("summary", "")
        bridge.queue_window(f"tool:{name}:{status}:{summary}")
    elif kind == "agent":
        bridge.queue_window("agent:" + str(payload.get("text", "")))
    elif kind == "usage":
        # Per-turn token/credit usage delta from the platform's rollup.
        # The robot-window plugin renders this as a footer line so the
        # operator can see what the last prompt cost.
        bridge.queue_window("usage:" + json.dumps(payload, default=str))
    elif kind == "audio_out":
        # Synthesized agent voice; deliver the base64-MP3 to the chat
        # panel as `audio_out:<json>` so the JS Audio API can play it.
        bridge.queue_window("audio_out:" + json.dumps(payload, default=str))
    elif kind == "error":
        bridge.queue_window("error:" + str(payload.get("text", "")))


def handle_wwi_message(
    bridge: ArmBridge,
    router: IntentRouter,
    relay: Any,
    msg: str,
) -> None:
    if not msg:
        return
    if msg.startswith("configure"):
        push_configure(bridge, relay)
        return
    if msg.startswith("open_chat"):
        # Pop the full-page chat into the operator's default browser. The
        # controller is a normal Python process, so webbrowser.open works;
        # each robot's bridge has its own port -> its own tab -> own agent.
        import webbrowser
        url = "http://127.0.0.1:%d/chat" % getattr(bridge, "http_port", 8765)
        try:
            webbrowser.open(url, new=2)
            bridge.queue_window("system:Opening chat in your browser — " + url)
        except Exception as e:
            bridge.queue_window("error:could not open browser: %r" % e)
        return
    if msg.startswith("stop"):
        bridge.act_stop()
        bridge.queue_window("agent:Stop received. Holding current pose.")
        bridge.queue_window("tool:stop_robot:ok:halted")
        bridge.queue_window("status:idle")
        return
    if msg.startswith("prompt:"):
        text = msg[len("prompt:"):]
        if relay is not None:
            relay.dispatch_async(text, lambda k, p: _on_relay_event(bridge, k, p))
            return
        bridge.queue_window("status:thinking")
        result = router.dispatch(text)
        for (tool, status, summary) in result["tools"]:
            bridge.queue_window(f"tool:{tool}:{status}:{summary}")
        bridge.queue_window("agent:" + result["agent"])
        bridge.queue_window("status:idle")
        return
    if msg.startswith("audio_in:"):
        # The chat panel captured a mic clip via MediaRecorder and
        # base64-encoded the webm blob. Decode -> STT via the relay
        # (or surface an error if the relay isn't attached) -> route
        # the transcribed text back as a prompt.
        if relay is None:
            bridge.queue_window("error:audio_in requires OMNI_KEY (no relay attached)")
            return
        import base64 as _b64
        payload = msg[len("audio_in:"):]
        try:
            info = json.loads(payload)
            audio = _b64.b64decode(info.get("audio_b64", ""))
            mime = info.get("mime_type", "audio/webm")
        except Exception as e:
            bridge.queue_window(f"error:audio_in decode failed: {e}")
            return
        bridge.queue_window("status:transcribing")
        # Run STT off the wwi loop so we don't stall the simulation tick.
        def _stt_worker():
            text = relay.transcribe(audio, mime_type=mime)
            if not text:
                bridge.queue_window("error:could not transcribe audio")
                bridge.queue_window("status:idle")
                return
            # Surface the transcript so the operator sees what was heard.
            bridge.queue_window("transcript:" + text)
            relay.dispatch_async(text, lambda k, p: _on_relay_event(bridge, k, p))

        threading.Thread(target=_stt_worker, name="omnilink-stt", daemon=True).start()
        return
    # Unknown -- echo back for debugging.
    bridge.queue_window("system:Unknown window message: " + msg[:200])


# ── Main loop ────────────────────────────────────────────────────────

def main() -> int:
    args = _parse_args()
    cfg = get_config(args.robot)
    robot = Supervisor()
    # --name overrides the agent id so that two arms with the same
    # underlying kinematic config (e.g. ur5e_left and ur5e_right when
    # several share a world) get distinct OmniLink profiles + Axis ids.
    robot_id = args.name or args.robot
    bridge = ArmBridge(robot, cfg, robot_id, gripper_id=args.gripper)
    bridge.http_port = args.port   # so open_chat builds the right /chat URL
    router = IntentRouter(bridge)
    relay = setup_omnilink_relay(bridge, http_port=args.port)

    # Optional: attach a real arm (or its offline-sim VM) through a hardware
    # backend. Opt-in only (--hardware-backend / --hardware-ip, or a backend's
    # own environment). When attached, act_* forward commands to the hardware
    # and tick() mirrors its measured joints onto the simulated arm.
    attach_hardware(bridge, cfg, robot_id,
                    name=args.hardware_backend, ip=args.hardware_ip)

    # HTTP server runs on its own thread.
    start_http(bridge, router, args.port, relay)

    print(f"[omnilink_arm_bridge] {cfg['model']} ready as id '{args.robot}' "
          f"({len(bridge.joint_names)} joints, "
          f"{'IK' if cfg.get('ik') else 'joint-space only'}, "
          f"gripper={bridge.effector.model if bridge.effector else 'none'}, "
          f"{'OmniLink' if relay else 'local'})")

    timestep = bridge.timestep
    while robot.step(timestep) != -1:
        sim_t = robot.getTime()
        # Drain wwi inbox.
        while True:
            msg = robot.wwiReceiveText()
            if msg is None or msg == "":
                break
            try:
                handle_wwi_message(bridge, router, relay, msg)
            except Exception as e:
                bridge.queue_window(f"error:bridge_exception: {e!r}")
        # Drain outbox.
        with bridge.lock:
            outbox = bridge.window_outbox
            bridge.window_outbox = []
        for line in outbox:
            try:
                robot.wwiSendText(line)
            except Exception as e:
                print(f"[omnilink_arm_bridge] wwiSendText failed: {e}")
        # Advance motion plan.
        bridge.tick(sim_t)

    if bridge.hw is not None:
        bridge.hw.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
