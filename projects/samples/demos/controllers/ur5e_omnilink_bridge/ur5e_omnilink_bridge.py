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

"""UR5e OmniLink bridge — supervisor controller for OmniSim.

Drop-in replacement for the ``ur5e_teleop_supervisor`` keyboard loop.
Instead of reading arrow keys, this supervisor exposes an HTTP surface
on 127.0.0.1:6060 so OmniLink agents (see
``omnilink/examples/omnisim_ur5e`` in the OmniLink repo) can drive the
UR5e remotely.

Protocol
--------
* Writes ``{"angles": [6 floats], "velocity": v}`` into the UR5E DEF
  node's ``customData`` field, same as ``ur5e_teleop_supervisor``.
  The ``ur5e_ik_slave`` controller on the arm polls ``customData`` each
  tick and calls ``motor.setPosition``.
* Reads real joint angles by walking the URDF subtree for the
  ``HingeJoint > jointParameters > position`` fields (same helper as
  the teleop supervisor).
* Reads real TCP by resolving ``wrist_3_link`` and calling
  ``Node.getPosition()``.
* Moves the red ``TARGET_MARKER`` sphere when in TCP-tracking mode.

HTTP surface
------------
``GET  /state``         live snapshot
``GET  /capabilities``  joint names, limits, workspace shell, IK constants
``POST /action``        ``{"action": ..., ...params}``. Actions:
    * ``stop``                  freeze at current real angles
    * ``reset_home``            return to HOME_POSE / HOME_TARGET
    * ``set_joint_positions``   ``{"q": [6 floats]}``
    * ``set_tcp_target``        ``{"xyz": [x, y, z]}`` — solves IK first
    * ``solve_ik``              ``{"xyz": [x, y, z]}`` — no motion

Kinematics, IK constants, and workspace shell mirror
``ur5e_teleop_supervisor.py`` exactly so the two controllers agree on
the UR5e model.
"""

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from controller import Supervisor

# ---------------------------------------------------------------------------
# Kinematic model — copied verbatim from ur5e_teleop_supervisor.py
# ---------------------------------------------------------------------------

JOINT_CHAIN = [
    ((0.0,  0.000, 0.163), (0.0, 0.0,       0.0), (0.0, 0.0, 1.0)),
    ((0.0,  0.138, 0.000), (0.0, 1.570796,  0.0), (0.0, 1.0, 0.0)),
    ((0.0, -0.131, 0.425), (0.0, 0.0,       0.0), (0.0, 1.0, 0.0)),
    ((0.0,  0.000, 0.392), (0.0, 1.570796,  0.0), (0.0, 1.0, 0.0)),
    ((0.0,  0.127, 0.000), (0.0, 0.0,       0.0), (0.0, 0.0, 1.0)),
    ((0.0,  0.000, 0.100), (0.0, 0.0,       0.0), (0.0, 1.0, 0.0)),
]
TCP_OFFSET = (0.0, 0.0, 0.0)  # tracked point is the wrist_3_link origin

JOINT_LIMITS = [
    (-6.28, 6.28),
    (-6.28, 6.28),
    (-3.14, 3.14),
    (-6.28, 6.28),
    (-6.28, 6.28),
    (-6.28, 6.28),
]

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
MOTOR_NAMES = [f"{n}_motor" for n in JOINT_NAMES]

HOME_POSE = [0.0, -1.0, 1.4, -1.2, -1.57, 0.0]
HOME_TARGET = (0.4, 0.0, 0.4)

WORKSPACE_MIN_RADIUS = 0.15
WORKSPACE_MAX_RADIUS = 0.82
WORKSPACE_MIN_Z = 0.05

IK_MAX_ITERS = 80
IK_TOL = 5e-4
IK_DAMPING = 0.05
IK_MAX_DQ = 0.12

MOTOR_VELOCITY = 2.0  # rad/s — forwarded to the slave via customData

# Bridge config
BRIDGE_HOST = "127.0.0.1"
BRIDGE_PORT = 6060
LIMIT_MARGIN = 0.02           # keep commanded angles this far from hard limits
IK_ACCEPT_ERR = 0.05          # reject set_tcp_target if IK err_norm > this (m)
DRIFT_RESTART = 0.6           # restart IK from real angles if drift exceeds this
IK_FAULT_ERR = 0.20           # raise fault if tcp tracking error exceeds this


# ---------------------------------------------------------------------------
# 4x4 matrix helpers
# ---------------------------------------------------------------------------

def mat_identity():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def mat_mul(a, b):
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        ai = a[i]
        for j in range(4):
            out[i][j] = (
                ai[0] * b[0][j]
                + ai[1] * b[1][j]
                + ai[2] * b[2][j]
                + ai[3] * b[3][j]
            )
    return out


def mat_translate(x, y, z):
    m = mat_identity()
    m[0][3] = x
    m[1][3] = y
    m[2][3] = z
    return m


def mat_rot_axis(ax, ay, az, angle):
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm > 0.0:
        ax, ay, az = ax / norm, ay / norm, az / norm
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    m = mat_identity()
    m[0][0] = t * ax * ax + c
    m[0][1] = t * ax * ay - s * az
    m[0][2] = t * ax * az + s * ay
    m[1][0] = t * ax * ay + s * az
    m[1][1] = t * ay * ay + c
    m[1][2] = t * ay * az - s * ax
    m[2][0] = t * ax * az - s * ay
    m[2][1] = t * ay * az + s * ax
    m[2][2] = t * az * az + c
    return m


def mat_rpy(roll, pitch, yaw):
    rx = mat_rot_axis(1.0, 0.0, 0.0, roll)
    ry = mat_rot_axis(0.0, 1.0, 0.0, pitch)
    rz = mat_rot_axis(0.0, 0.0, 1.0, yaw)
    return mat_mul(rz, mat_mul(ry, rx))


def forward_kinematics(angles):
    t = mat_identity()
    for (xyz, rpy, axis), theta in zip(JOINT_CHAIN, angles):
        t = mat_mul(t, mat_translate(*xyz))
        t = mat_mul(t, mat_rpy(*rpy))
        t = mat_mul(t, mat_rot_axis(axis[0], axis[1], axis[2], theta))
    t = mat_mul(t, mat_translate(*TCP_OFFSET))
    return (t[0][3], t[1][3], t[2][3])


def numeric_jacobian(angles, eps=1e-4):
    columns = []
    for j in range(6):
        plus = list(angles)
        minus = list(angles)
        plus[j] += eps
        minus[j] -= eps
        p_plus = forward_kinematics(plus)
        p_minus = forward_kinematics(minus)
        columns.append(
            (
                (p_plus[0] - p_minus[0]) / (2 * eps),
                (p_plus[1] - p_minus[1]) / (2 * eps),
                (p_plus[2] - p_minus[2]) / (2 * eps),
            )
        )
    return [[columns[j][i] for j in range(6)] for i in range(3)]


def mat3_inv_damped(m, lam_sq):
    a00 = m[0][0] + lam_sq; a01 = m[0][1]; a02 = m[0][2]
    a10 = m[1][0]; a11 = m[1][1] + lam_sq; a12 = m[1][2]
    a20 = m[2][0]; a21 = m[2][1]; a22 = m[2][2] + lam_sq
    det = (
        a00 * (a11 * a22 - a12 * a21)
        - a01 * (a10 * a22 - a12 * a20)
        + a02 * (a10 * a21 - a11 * a20)
    )
    if abs(det) < 1e-12:
        return None
    inv_det = 1.0 / det
    return [
        [(a11 * a22 - a12 * a21) * inv_det,
         -(a01 * a22 - a02 * a21) * inv_det,
         (a01 * a12 - a02 * a11) * inv_det],
        [-(a10 * a22 - a12 * a20) * inv_det,
         (a00 * a22 - a02 * a20) * inv_det,
         -(a00 * a12 - a02 * a10) * inv_det],
        [(a10 * a21 - a11 * a20) * inv_det,
         -(a00 * a21 - a01 * a20) * inv_det,
         (a00 * a11 - a01 * a10) * inv_det],
    ]


def solve_ik(start_angles, target):
    angles = list(start_angles)
    err_norm = float("inf")
    for _ in range(IK_MAX_ITERS):
        pos = forward_kinematics(angles)
        e = [target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]]
        err_norm = math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2])
        if err_norm < IK_TOL:
            break
        jac = numeric_jacobian(angles)
        jjt = [[0.0] * 3 for _ in range(3)]
        for i in range(3):
            for k in range(3):
                s = 0.0
                for col in range(6):
                    s += jac[i][col] * jac[k][col]
                jjt[i][k] = s
        inv = mat3_inv_damped(jjt, IK_DAMPING * IK_DAMPING)
        if inv is None:
            break
        temp = [
            inv[0][0] * e[0] + inv[0][1] * e[1] + inv[0][2] * e[2],
            inv[1][0] * e[0] + inv[1][1] * e[1] + inv[1][2] * e[2],
            inv[2][0] * e[0] + inv[2][1] * e[1] + inv[2][2] * e[2],
        ]
        dq = [
            jac[0][j] * temp[0] + jac[1][j] * temp[1] + jac[2][j] * temp[2]
            for j in range(6)
        ]
        step_norm = math.sqrt(sum(x * x for x in dq))
        if step_norm > IK_MAX_DQ:
            scale = IK_MAX_DQ / step_norm
            dq = [x * scale for x in dq]
        for j in range(6):
            lo, hi = JOINT_LIMITS[j]
            angles[j] = max(lo, min(hi, angles[j] + dq[j]))
    return angles, err_norm


def clamp_to_workspace(target):
    x, y, z = target
    if z < WORKSPACE_MIN_Z:
        z = WORKSPACE_MIN_Z
    r = math.sqrt(x * x + y * y + z * z)
    if r > WORKSPACE_MAX_RADIUS:
        scale = WORKSPACE_MAX_RADIUS / r
        x *= scale
        y *= scale
        z *= scale
    elif 1e-6 < r < WORKSPACE_MIN_RADIUS:
        scale = WORKSPACE_MIN_RADIUS / r
        x *= scale
        y *= scale
        z *= scale
    return [x, y, z]


def clamp_joints(q):
    out = []
    for (lo, hi), qi in zip(JOINT_LIMITS, q):
        out.append(max(lo + LIMIT_MARGIN, min(hi - LIMIT_MARGIN, qi)))
    return out


# ---------------------------------------------------------------------------
# Scene-tree helpers — copied from ur5e_teleop_supervisor.py
# ---------------------------------------------------------------------------

def find_joint_position_fields(root):
    found = [None] * len(MOTOR_NAMES)

    def hinge_motor_name(hinge):
        device_field = hinge.getField("device")
        if device_field is None:
            return None
        try:
            count = device_field.getCount()
        except Exception:
            return None
        for i in range(count):
            device = device_field.getMFNode(i)
            if device is None:
                continue
            nf = device.getField("name")
            if nf is None:
                continue
            try:
                return nf.getSFString()
            except Exception:
                return None
        return None

    def recurse(node):
        if node is None:
            return
        if node.getTypeName() == "HingeJoint":
            name = hinge_motor_name(node)
            if name in MOTOR_NAMES:
                idx = MOTOR_NAMES.index(name)
                jp_field = node.getField("jointParameters")
                if jp_field is not None:
                    jp = jp_field.getSFNode()
                    if jp is not None:
                        pos = jp.getField("position")
                        if pos is not None:
                            found[idx] = pos
        for fname in ("children", "endPoint"):
            field = node.getField(fname)
            if field is None:
                continue
            try:
                if fname == "endPoint":
                    recurse(field.getSFNode())
                else:
                    count = field.getCount()
                    for i in range(count):
                        recurse(field.getMFNode(i))
            except Exception:
                pass

    recurse(root)
    return found


def read_real_angles(position_fields, fallback):
    real = list(fallback)
    for i, field in enumerate(position_fields):
        if field is None:
            continue
        try:
            real[i] = float(field.getSFFloat())
        except Exception:
            pass
    return real


def find_link_by_name(node, target):
    if node is None:
        return None
    name_field = node.getField("name")
    if name_field is not None:
        try:
            if name_field.getSFString() == target:
                return node
        except Exception:
            pass
    children = node.getField("children")
    if children is not None:
        try:
            count = children.getCount()
        except Exception:
            count = 0
        for i in range(count):
            found = find_link_by_name(children.getMFNode(i), target)
            if found is not None:
                return found
    endpoint = node.getField("endPoint")
    if endpoint is not None:
        found = find_link_by_name(endpoint.getSFNode(), target)
        if found is not None:
            return found
    return None


# ---------------------------------------------------------------------------
# Shared bridge state
# ---------------------------------------------------------------------------

class BridgeState:
    def __init__(self, home, home_target):
        self.lock = threading.Lock()
        self.commanded_q = list(home)
        self.real_q = list(home)
        self.tcp_real = [0.0, 0.0, 0.0]
        self.target_xyz = list(home_target)
        self.mode = "joint"           # joint | tcp | stopped
        self.last_tick_at = time.time()
        self.sim_time = 0.0
        self.fault = None
        self.tick_period_s = 0.016
        # Arm's world-frame origin. Populated at startup by the
        # supervisor reading DEF UR5E's translation. The legacy
        # /action contract speaks arm-frame coordinates (the arm
        # base is the origin, IK math is local). The Axis-shape
        # endpoints speak world-frame coordinates because Axis was
        # built that way and the productized agent doesn't carry a
        # per-world arm-origin offset. The shim translates by adding
        # arm_origin on read and subtracting on write.
        self.arm_origin = [0.0, 0.0, 0.0]

    def snapshot(self):
        with self.lock:
            return {
                "q": list(self.real_q),
                "commanded_q": list(self.commanded_q),
                "tcp": list(self.tcp_real),
                "target_xyz": list(self.target_xyz),
                "mode": self.mode,
                "last_tick_at": self.last_tick_at,
                "sim_time": self.sim_time,
                "fault": self.fault,
            }


# ---------------------------------------------------------------------------
# HTTP surface
# ---------------------------------------------------------------------------

def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _json(self, code, obj):
            body = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _ok(self, extra):
            payload = {"status": "ok"}
            payload.update(extra)
            self._json(200, payload)

        def _read_json(self):
            n = int(self.headers.get("Content-Length") or "0")
            if n == 0:
                return {}
            raw = self.rfile.read(n).decode("utf-8") or "{}"
            return json.loads(raw)

        def do_GET(self):
            if self.path == "/state":
                self._json(200, state.snapshot())
            elif self.path == "/capabilities":
                self._json(200, {
                    "joint_names": JOINT_NAMES,
                    "joint_limits": [list(pair) for pair in JOINT_LIMITS],
                    "home_pose": list(HOME_POSE),
                    "home_target": list(HOME_TARGET),
                    "workspace": {
                        "min_radius": WORKSPACE_MIN_RADIUS,
                        "max_radius": WORKSPACE_MAX_RADIUS,
                        "min_z": WORKSPACE_MIN_Z,
                    },
                    "ik_max_dq": IK_MAX_DQ,
                    "ik_tol": IK_TOL,
                    "ik_damping": IK_DAMPING,
                    "ik_accept_err": IK_ACCEPT_ERR,
                    "motor_velocity": MOTOR_VELOCITY,
                    "tick_period_s": state.tick_period_s,
                })
            else:
                self._json(404, {"error": "not found"})

        # ────────────────────────────────────────────────────────────
        # Axis-compatibility shim. The productized Axis agent
        # (/o/olink/agents/axis/) was built against an older bridge
        # protocol where each tool POSTed to its own endpoint
        # (`/set_tcp_target`, `/list_robots`, …) rather than the
        # consolidated `/action` we ship today. The Warehouse Foreman's
        # iteration-4 demo depends on Axis driving this exact UR5e, so
        # rather than fork the productized Axis or pin it to a legacy
        # bridge, we accept its endpoint shape here and translate.
        #
        # Three categories:
        #   ACTION_ALIASES     — POST /<axis_endpoint> {id, ...}     ->
        #                        /action {action: <bridge_name>, ...}
        #   AXIS_READ_ENDPOINTS — POST /<axis_endpoint> {id} reads
        #                        live state and returns Axis's shape
        #   AXIS_NOOP_ENDPOINTS — endpoints this hardware can't honour
        #                        (no gripper, no trajectory planner);
        #                        respond 200 with a {noop: true, note}
        #                        payload so Axis doesn't error out and
        #                        the Foreman's delegation completes.
        # ────────────────────────────────────────────────────────────
        AXIS_ACTION_ALIASES = {
            "/set_tcp_target":      "set_tcp_target",
            "/set_joint_positions": "set_joint_positions",
            "/reset_to_home":       "reset_home",
            "/stop_robot":          "stop",
            "/solve_ik":            "solve_ik",
        }
        AXIS_NOOP_ENDPOINTS = {
            "/open_gripper":      "no gripper on this UR5e",
            "/close_gripper":     "no gripper on this UR5e",
            "/plan_trajectory":   "no trajectory planner — use set_tcp_target instead",
            "/execute_trajectory": "no trajectory planner — use set_tcp_target instead",
            "/pause_simulation":  "manual pause/resume only",
            "/resume_simulation": "manual pause/resume only",
        }

        def do_POST(self):
            try:
                body = self._read_json()
            except Exception as exc:
                self._json(400, {"error": f"bad json: {exc}"})
                return

            path = self.path

            # Route 1: legacy /action protocol (bridge-native, used by
            # any controller that already POSTs the full action shape).
            if path == "/action":
                action = body.get("action", "")
                self._dispatch_action(state, action, body)
                return

            # Route 2: Axis sends most actions as POST /<verb> {id, ...}.
            # The set_tcp_target / solve_ik branches need a world->arm
            # frame translation: Axis speaks world coordinates (the
            # warehouse is a multi-robot scene so a single global frame
            # is the only sensible API), but the bridge's IK is local
            # to the arm base. Subtract on the way in, add on the way
            # back out via _xyz_capture below.
            if path in self.AXIS_ACTION_ALIASES:
                action = self.AXIS_ACTION_ALIASES[path]
                if action in ("set_tcp_target", "solve_ik") and isinstance(body.get("xyz"), list):
                    try:
                        wx, wy, wz = (float(v) for v in body["xyz"])
                        ox, oy, oz = state.arm_origin
                        body = dict(body)
                        body["xyz"] = [wx - ox, wy - oy, wz - oz]
                        body["_world_xyz_in"] = [wx, wy, wz]
                    except (TypeError, ValueError):
                        pass  # let the dispatch surface the bad-xyz error
                self._dispatch_action(state, action, body)
                return

            # Route 3: Axis read endpoints — serve directly from state.
            if path == "/list_robots":
                self._json(200, {
                    "robots": [{
                        "id": "ur5e",
                        "model": "Universal Robots UR5e",
                        "dof": 6,
                        "has_gripper": False,
                    }],
                })
                return
            if path == "/get_robot_state":
                snap = state.snapshot()
                snap["id"] = "ur5e"
                self._json(200, snap)
                return
            if path == "/read_joints":
                snap = state.snapshot()
                self._json(200, {
                    "id": "ur5e",
                    "q": snap["q"],
                    "commanded_q": snap["commanded_q"],
                    "joint_names": JOINT_NAMES,
                })
                return
            if path == "/read_tcp_pose":
                snap = state.snapshot()
                ox, oy, oz = state.arm_origin
                # state.tcp is already in WORLD coords (read from
                # wrist3.getPosition() in the supervisor loop), but
                # state.target_xyz is in ARM-LOCAL coords (set by the
                # /action handlers post-clamp_to_workspace). Translate
                # consistently so Axis sees world coordinates throughout.
                tcp_world = list(snap["tcp"])
                tgt_local = snap["target_xyz"]
                tgt_world = [tgt_local[0] + ox, tgt_local[1] + oy, tgt_local[2] + oz]
                self._json(200, {
                    "id": "ur5e",
                    "tcp": tcp_world,
                    "target_xyz": tgt_world,
                    "tcp_arm_local": [tcp_world[0] - ox, tcp_world[1] - oy, tcp_world[2] - oz],
                    "target_arm_local": list(tgt_local),
                    "arm_origin_world": list(state.arm_origin),
                })
                return
            if path == "/read_sensor":
                self._json(200, {
                    "id": "ur5e",
                    "sensor": body.get("sensor"),
                    "available": False,
                    "note": "no auxiliary sensors on this UR5e — use read_joints / read_tcp_pose instead",
                })
                return
            if path == "/get_simulation_time":
                self._json(200, {"sim_time": state.snapshot()["sim_time"]})
                return

            # Route 4: graceful no-op for capabilities this hardware
            # can't honour. 200 OK keeps Axis's chat loop progressing.
            if path in self.AXIS_NOOP_ENDPOINTS:
                self._json(200, {
                    "id": "ur5e",
                    "noop": True,
                    "note": self.AXIS_NOOP_ENDPOINTS[path],
                })
                return

            self._json(404, {"error": "not found", "path": path})

        def _dispatch_action(self, state, action, body):
            """The bridge's original /action dispatch table, factored out
            so both /action and the Axis-shape aliases can call it."""

            if action == "stop":
                with state.lock:
                    state.mode = "stopped"
                    state.commanded_q = list(state.real_q)
                    state.target_xyz = list(state.tcp_real) if state.tcp_real else list(HOME_TARGET)
                    state.fault = None
                self._ok({"halted_at": time.time()})
                return

            if action == "reset_home":
                with state.lock:
                    state.mode = "joint"
                    state.commanded_q = list(HOME_POSE)
                    state.target_xyz = list(HOME_TARGET)
                    state.fault = None
                self._ok({
                    "target_q": list(HOME_POSE),
                    "target_xyz": list(HOME_TARGET),
                })
                return

            if action == "set_joint_positions":
                q = body.get("q")
                if not isinstance(q, list) or len(q) != 6:
                    self._json(400, {"error": "q must be a list of 6 numbers"})
                    return
                try:
                    clamped = clamp_joints([float(v) for v in q])
                except (TypeError, ValueError) as exc:
                    self._json(400, {"error": f"bad q: {exc}"})
                    return
                fk = forward_kinematics(clamped)
                with state.lock:
                    state.mode = "joint"
                    state.commanded_q = clamped
                    state.target_xyz = [fk[0], fk[1], fk[2]]
                    state.fault = None
                self._ok({"accepted": True, "clamped_q": clamped, "fk_xyz": list(fk)})
                return

            if action == "set_tcp_target":
                xyz = body.get("xyz")
                if not isinstance(xyz, list) or len(xyz) != 3:
                    self._json(400, {"error": "xyz must be a list of 3 numbers"})
                    return
                try:
                    target = clamp_to_workspace([float(v) for v in xyz])
                except (TypeError, ValueError) as exc:
                    self._json(400, {"error": f"bad xyz: {exc}"})
                    return
                with state.lock:
                    start = list(state.commanded_q)
                solved, err = solve_ik(start, target)
                if err > IK_ACCEPT_ERR:
                    self._json(422, {
                        "error": "ik did not converge",
                        "err_norm": err,
                        "solved_q": solved,
                        "target_clamped": target,
                        "hint": "target is likely unreachable — try a smaller delta",
                    })
                    return
                clamped = clamp_joints(solved)
                with state.lock:
                    state.mode = "tcp"
                    state.commanded_q = clamped
                    state.target_xyz = target
                    state.fault = None
                self._ok({
                    "accepted": True,
                    "solved_q": clamped,
                    "target_clamped": target,
                    "err_norm": err,
                })
                return

            if action == "solve_ik":
                xyz = body.get("xyz")
                if not isinstance(xyz, list) or len(xyz) != 3:
                    self._json(400, {"error": "xyz must be a list of 3 numbers"})
                    return
                try:
                    target = clamp_to_workspace([float(v) for v in xyz])
                except (TypeError, ValueError) as exc:
                    self._json(400, {"error": f"bad xyz: {exc}"})
                    return
                with state.lock:
                    start = list(state.commanded_q)
                solved, err = solve_ik(start, target)
                self._json(200, {
                    "q": solved,
                    "err_norm": err,
                    "reachable": err <= IK_ACCEPT_ERR,
                    "target_clamped": target,
                })
                return

            self._json(400, {"error": f"unknown action: {action}"})

    return Handler


def start_http(state):
    server = ThreadingHTTPServer((BRIDGE_HOST, BRIDGE_PORT), make_handler(state))
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    print(f"[ur5e_omnilink_bridge] HTTP listening on http://{BRIDGE_HOST}:{BRIDGE_PORT}")
    return server


# ---------------------------------------------------------------------------
# Main supervisor loop
# ---------------------------------------------------------------------------

def main():
    supervisor = Supervisor()
    time_step = int(supervisor.getBasicTimeStep())

    ur5e = supervisor.getFromDef("UR5E")
    if ur5e is None:
        print("[ur5e_omnilink_bridge] ERROR: DEF UR5E not found")
        return
    custom_data = ur5e.getField("customData")

    marker = supervisor.getFromDef("TARGET_MARKER")
    marker_translation = marker.getField("translation") if marker is not None else None

    # Let the URDF expansion materialise before walking it
    supervisor.step(time_step)
    supervisor.step(time_step)

    wrist3 = find_link_by_name(ur5e, "wrist_3_link")
    joint_position_fields = find_joint_position_fields(ur5e)
    found = sum(1 for f in joint_position_fields if f is not None)
    print(f"[ur5e_omnilink_bridge] resolved {found}/6 HingeJoint position fields")
    if wrist3 is None:
        print("[ur5e_omnilink_bridge] WARNING: wrist_3_link not resolved; tcp reads will be zero")

    state = BridgeState(HOME_POSE, HOME_TARGET)
    state.tick_period_s = time_step / 1000.0

    # Cache the arm's world-frame origin for the Axis-shape shim's
    # world<->arm-local coordinate translation. Read once at startup —
    # the URDFRobot has staticBase TRUE so its translation never
    # changes after world load.
    try:
        arm_origin = list(ur5e.getField("translation").getSFVec3f())
        state.arm_origin = [float(v) for v in arm_origin]
        print(f"[ur5e_omnilink_bridge] arm world origin: {state.arm_origin}")
    except Exception as exc:
        print(f"[ur5e_omnilink_bridge] WARN: could not read UR5E translation ({exc}); "
              "Axis-shape endpoints will assume arm origin = (0,0,0)")

    # Prime slave + marker
    custom_data.setSFString(
        json.dumps({"angles": list(HOME_POSE), "velocity": MOTOR_VELOCITY})
    )
    if marker_translation is not None:
        marker_translation.setSFVec3f(list(HOME_TARGET))

    start_http(state)
    print("[ur5e_omnilink_bridge] ready. HOME_POSE loaded.")

    while supervisor.step(time_step) != -1:
        real = read_real_angles(joint_position_fields, state.commanded_q)
        tcp = [0.0, 0.0, 0.0]
        if wrist3 is not None:
            try:
                tcp = list(wrist3.getPosition())
            except Exception:
                pass

        with state.lock:
            mode = state.mode
            target_xyz = list(state.target_xyz)
            commanded = list(state.commanded_q)
            state.real_q = real
            state.tcp_real = tcp
            state.last_tick_at = time.time()
            state.sim_time += state.tick_period_s

        if mode == "tcp":
            # Drift recovery: if IK has run ahead of the physical arm,
            # re-seed from the real angles before the next IK burst.
            # Joint mode must NOT do this — it would wipe the operator's
            # commanded pose back to whatever the arm happens to be at.
            drift = sum(abs(a - b) for a, b in zip(commanded, real))
            if drift > DRIFT_RESTART:
                commanded = list(real)
            commanded, err = solve_ik(commanded, target_xyz)
            commanded = clamp_joints(commanded)
            with state.lock:
                state.commanded_q = commanded
                state.fault = f"ik_drift:{err:.3f}" if err > IK_FAULT_ERR else None
            if marker_translation is not None:
                marker_translation.setSFVec3f(target_xyz)
        elif mode == "joint":
            # commanded_q was set atomically by the handler; the slave ramps
            # at MOTOR_VELOCITY. Nothing per-tick to do.
            if marker_translation is not None:
                marker_translation.setSFVec3f(target_xyz)
        else:  # stopped
            # Hold at real angles (already written by stop handler)
            pass

        custom_data.setSFString(
            json.dumps({"angles": commanded, "velocity": MOTOR_VELOCITY})
        )


if __name__ == "__main__":
    main()
