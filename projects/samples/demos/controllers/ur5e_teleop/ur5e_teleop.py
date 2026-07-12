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

"""UR5e keyboard teleop with inverse kinematics.

Type a target XYZ with the keyboard and the arm will solve IK and
track it. Forward kinematics walks the URDF joint chain in pure
Python; inverse kinematics is a damped-least-squares iteration on
position only (orientation is free).

Controls (click the 3D view first so keys reach the controller):

  Arrow Left / Right : nudge target X  -/+
  Arrow Up / Down    : nudge target Y  -/+
  Q / E              : nudge target Z  -/+
  Shift + any of the above : fine step
  R                  : return to home pose (target follows FK of home)
  P                  : print current target and joint angles
  H                  : print this help
"""

import math

from controller import Robot, Keyboard

# ---------------------------------------------------------------------------
# UR5e kinematic chain, extracted from
#   tests/api/controllers/robot_urdf/ur5e_reference.urdf
# Each entry is the parent->child joint's static (xyz, rpy) origin plus the
# joint rotation axis in the parent frame. Walking the chain from base_link
# to wrist_3_link with these transforms reproduces the Webots simulation.
# ---------------------------------------------------------------------------

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_CHAIN = [
    # (origin xyz, origin rpy, axis)
    ((0.0,  0.000, 0.163), (0.0, 0.0,       0.0), (0.0, 0.0, 1.0)),
    ((0.0,  0.138, 0.000), (0.0, 1.570796,  0.0), (0.0, 1.0, 0.0)),
    ((0.0, -0.131, 0.425), (0.0, 0.0,       0.0), (0.0, 1.0, 0.0)),
    ((0.0,  0.000, 0.392), (0.0, 1.570796,  0.0), (0.0, 1.0, 0.0)),
    ((0.0,  0.127, 0.000), (0.0, 0.0,       0.0), (0.0, 0.0, 1.0)),
    ((0.0,  0.000, 0.100), (0.0, 0.0,       0.0), (0.0, 1.0, 0.0)),
]

# Tool tip offset in the wrist_3_link frame (flange ~0.1 m along +y).
TCP_OFFSET = (0.0, 0.1, 0.0)

JOINT_LIMITS = [
    (-6.28, 6.28),
    (-6.28, 6.28),
    (-3.14, 3.14),
    (-6.28, 6.28),
    (-6.28, 6.28),
    (-6.28, 6.28),
]

HOME_POSE = [0.0, -1.0, 1.4, -1.2, -1.57, 0.0]
MOTOR_VELOCITY = 1.5   # rad/s — smooth
COARSE_STEP = 0.02     # m/tick when arrow keys are held
FINE_STEP = 0.005      # m/tick with Shift

IK_MAX_ITERS = 20
IK_TOL = 1e-3
IK_DAMPING = 0.08
IK_MAX_DQ = 0.08       # radians per iteration, clamps wild jumps

HELP_LINES = [
    "=" * 60,
    "UR5e IK teleop",
    "  Left / Right     : target X  -/+",
    "  Up / Down        : target Y  -/+",
    "  Q / E            : target Z  -/+",
    "  Shift + arrow/Q/E: fine step",
    "  R                : return to home pose",
    "  P                : print target & joints",
    "  H                : print this help",
    "Click the 3D view first, then press keys.",
    "=" * 60,
]


# ---------------------------------------------------------------------------
# 4x4 matrix helpers (pure python, row-major list of lists)
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
        oi = out[i]
        for j in range(4):
            oi[j] = ai[0] * b[0][j] + ai[1] * b[1][j] + ai[2] * b[2][j] + ai[3] * b[3][j]
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
    # URDF fixed-axis XYZ: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    rx = mat_rot_axis(1.0, 0.0, 0.0, roll)
    ry = mat_rot_axis(0.0, 1.0, 0.0, pitch)
    rz = mat_rot_axis(0.0, 0.0, 1.0, yaw)
    return mat_mul(rz, mat_mul(ry, rx))


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------

def forward_kinematics(angles):
    t = mat_identity()
    for (xyz, rpy, axis), theta in zip(JOINT_CHAIN, angles):
        t = mat_mul(t, mat_translate(*xyz))
        t = mat_mul(t, mat_rpy(*rpy))
        t = mat_mul(t, mat_rot_axis(axis[0], axis[1], axis[2], theta))
    t = mat_mul(t, mat_translate(*TCP_OFFSET))
    return (t[0][3], t[1][3], t[2][3])


def numeric_jacobian(angles, eps=1e-4):
    """3x6 position Jacobian by central differences."""
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
    # Convert to row-major 3x6
    jac = [[columns[j][i] for j in range(6)] for i in range(3)]
    return jac


def mat3_inv_damped(m, lam_sq):
    """Invert a 3x3 matrix plus lam_sq*I. Returns None on singular."""
    a00 = m[0][0] + lam_sq
    a01 = m[0][1]
    a02 = m[0][2]
    a10 = m[1][0]
    a11 = m[1][1] + lam_sq
    a12 = m[1][2]
    a20 = m[2][0]
    a21 = m[2][1]
    a22 = m[2][2] + lam_sq

    det = (
        a00 * (a11 * a22 - a12 * a21)
        - a01 * (a10 * a22 - a12 * a20)
        + a02 * (a10 * a21 - a11 * a20)
    )
    if abs(det) < 1e-12:
        return None
    inv_det = 1.0 / det
    return [
        [
            (a11 * a22 - a12 * a21) * inv_det,
            -(a01 * a22 - a02 * a21) * inv_det,
            (a01 * a12 - a02 * a11) * inv_det,
        ],
        [
            -(a10 * a22 - a12 * a20) * inv_det,
            (a00 * a22 - a02 * a20) * inv_det,
            -(a00 * a12 - a02 * a10) * inv_det,
        ],
        [
            (a10 * a21 - a11 * a20) * inv_det,
            -(a00 * a21 - a01 * a20) * inv_det,
            (a00 * a11 - a01 * a10) * inv_det,
        ],
    ]


def ik_step(angles, target):
    """Run a damped-least-squares IK burst toward target. Returns new angles
    and final position error norm."""
    angles = list(angles)
    err_norm = 0.0
    for _ in range(IK_MAX_ITERS):
        pos = forward_kinematics(angles)
        e = [target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]]
        err_norm = math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2])
        if err_norm < IK_TOL:
            break

        jac = numeric_jacobian(angles)

        # JJT = jac @ jac^T, 3x3
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

        # temp = inv @ e
        temp = [
            inv[0][0] * e[0] + inv[0][1] * e[1] + inv[0][2] * e[2],
            inv[1][0] * e[0] + inv[1][1] * e[1] + inv[1][2] * e[2],
            inv[2][0] * e[0] + inv[2][1] * e[1] + inv[2][2] * e[2],
        ]

        # dq = jac^T @ temp (6-vector)
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


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    motors = []
    for name in JOINT_NAMES:
        motor = robot.getDevice(f"{name}_motor")
        motor.setVelocity(MOTOR_VELOCITY)
        motors.append(motor)

    angles = list(HOME_POSE)
    for motor, target in zip(motors, angles):
        motor.setPosition(target)

    target = list(forward_kinematics(angles))

    keyboard = robot.getKeyboard()
    keyboard.enable(time_step)

    for line in HELP_LINES:
        print(line)
    print(f"Home FK target: x={target[0]:+.3f} y={target[1]:+.3f} z={target[2]:+.3f}")

    def print_status(err):
        joint_str = " ".join(f"{a:+.2f}" for a in angles)
        print(
            f"target=({target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f}) "
            f"err={err:.4f} joints=[{joint_str}]"
        )

    def banner():
        pass  # setLabel is Supervisor-only; target is echoed via prints.

    while robot.step(time_step) != -1:
        dx = dy = dz = 0.0
        shift_held = False

        keys = []
        k = keyboard.getKey()
        while k != -1:
            keys.append(k)
            k = keyboard.getKey()

        # Detect shift on any key
        for k in keys:
            if k & Keyboard.SHIFT:
                shift_held = True

        step = FINE_STEP if shift_held else COARSE_STEP

        reset_requested = False
        for k in keys:
            ch = k & 0xFFFF
            if ch == Keyboard.LEFT:
                dx -= step
            elif ch == Keyboard.RIGHT:
                dx += step
            elif ch == Keyboard.DOWN:
                dy -= step
            elif ch == Keyboard.UP:
                dy += step
            elif ch == ord('Q'):
                dz -= step
            elif ch == ord('E'):
                dz += step
            elif ch == ord('R'):
                reset_requested = True
            elif ch == ord('P'):
                print_status(0.0)
            elif ch == ord('H'):
                for line in HELP_LINES:
                    print(line)

        if reset_requested:
            angles = list(HOME_POSE)
            for motor, a in zip(motors, angles):
                motor.setPosition(a)
            target = list(forward_kinematics(angles))
            banner()
            print("Home pose.")
            continue

        if dx or dy or dz:
            target[0] += dx
            target[1] += dy
            target[2] += dz
            print(
                f"target x={target[0]:+.3f} y={target[1]:+.3f} z={target[2]:+.3f}"
            )

        angles, err = ik_step(angles, target)
        for motor, a in zip(motors, angles):
            motor.setPosition(a)


if __name__ == "__main__":
    main()
