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

"""Build the deploy-faithful Spot as a NATIVE Newton articulation.

WHY NOT add_mjcf: the exported MJCF encodes the joints as MuJoCo <general>
position+velocity actuators. Newton's add_mjcf imports those as MuJoCo
actuators whose control is mjData.ctrl -- which the Newton `control` object
(joint_target_pos / joint_act) does NOT drive (verified: writes are silently
ignored, the robot runs on ctrl=0). The OmniSim deploy (WbNewtonBackend)
instead builds NATIVE Newton revolute joints with target_ke/target_kd in
POSITION_VELOCITY mode and drives them via control.joint_target_pos. So to
train on the true deploy control path we must build native joints too.

Geometry / masses / inertias / gains below are taken verbatim from the
exported deploy MJCF (C:/tmp/spot_newton_fixed.xml), so the dynamics match
OmniSim. Joint order = FL,FR,RL,RR x [hip_x,hip_y,knee] (the controller's
JOINT_ORDER), so joint_q[7:19] / joint_qd[6:18] line up with the deploy obs.
"""
from __future__ import annotations

import warp as wp
import newton

# from MJCF <inertial> / <geom> / <general>
CHASSIS_MASS = 32.0
CHASSIS_INERTIA = (0.2, 1.14, 1.17)
CHASSIS_BOX = (0.445831, 0.128149, 0.106907)        # half-extents
CHASSIS_BOX_OFF = (0.00636841, -4.36455e-05, -0.00394037)
HIP_MASS = 1.0
HIP_INERTIA = (0.0011, 0.0011, 0.00125)
HIP_BOX = (0.0805, 0.0599, 0.0567)        # half-extents (deploy shape_2_2..)
HIP_BOX_OFF = (-0.0240, 0.0118, 0.0)      # x*=sx, y*=sy applied per leg
THIGH_MASS = 1.5
THIGH_INERTIA = (0.0128, 0.0128, 0.00125)
THIGH_BOX = (0.0604, 0.0753, 0.2189)      # half-extents (deploy shape_3_3..)
THIGH_BOX_OFF = (0.0, -0.0084, -0.135)    # y*=sy applied per leg
LOWER_MASS = 0.8
LOWER_INERTIA = (0.00682, 0.00682, 0.0005)
FOOT_R = 0.035
FOOT_OFF = (0.0, 0.0, -0.32)

HIP_DX, HIP_DY = 0.29785, 0.055      # hip rel chassis
THIGH_DY = 0.110945                  # thigh rel hip (y)
KNEE_DX, KNEE_DZ = 0.025, -0.3205    # lower rel thigh
STAND_Z = 0.7

# Gains MUST match the deploy backend (WbNewtonBackend default
# OMNISIM_NEWTON_TARGET_KE/KD = 20/3). Training at 250/60 produced a gait
# that "punches the body across the floor" at the deploy's soft 20/3 springs
# (verified: ke=250 in deploy -> falls + slides 4 m sideways). The deploy
# also has NO chassis collision box (a 0.001 marker sphere) and ground
# friction mu=1 -- matched below so the trained gait transfers.
KE = 20.0
KD = 3.0
EFFORT = 80.0
# (sx, sy) per leg in controller order FL, FR, RL, RR
LEG_SIGNS = [(+1, +1), (+1, -1), (-1, +1), (-1, -1)]


def _mat(d):
    return wp.mat33((d[0], 0, 0), (0, d[1], 0), (0, 0, d[2]))


def build_spot_native(mu=1.0, ke=KE, kd=KD):
    b = newton.ModelBuilder()
    chassis = b.add_link(
        xform=wp.transform((0.0, 0.0, STAND_Z), (0.0, 0.0, 0.0, 1.0)),
        mass=CHASSIS_MASS, inertia=_mat(CHASSIS_INERTIA), label="chassis")
    joint_indices = [b.add_joint_free(child=chassis)]
    # Deploy has NO chassis collision box (only a 0.001 marker sphere); a real
    # box here would add spurious chassis-ground/leg contacts the deploy lacks.
    b.add_shape_sphere(chassis, xform=wp.transform((0, 0, 0), (0, 0, 0, 1.0)),
                       radius=0.001, cfg=newton.ModelBuilder.ShapeConfig(density=0.0, mu=mu))

    def motor(parent, child, axis, anchor, lo, hi):
        idx = b.add_joint_revolute(
            parent=parent, child=child, axis=axis,
            parent_xform=wp.transform(anchor, (0.0, 0.0, 0.0, 1.0)),
            child_xform=wp.transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            target_pos=0.0, target_vel=0.0, target_ke=ke, target_kd=kd,
            armature=0.0, limit_ke=10000.0, limit_kd=100.0,
            limit_lower=lo, limit_upper=hi, effort_limit=EFFORT,
            actuator_mode=newton.JointTargetMode.POSITION_VELOCITY,
            collision_filter_parent=True)
        joint_indices.append(idx)

    for (sx, sy) in LEG_SIGNS:
        hx, hy = sx * HIP_DX, sy * HIP_DY
        ty = hy + sy * THIGH_DY
        cfgm = newton.ModelBuilder.ShapeConfig(density=0.0, mu=mu)
        hip = b.add_link(xform=wp.transform((hx, hy, STAND_Z), (0, 0, 0, 1.0)),
                         mass=HIP_MASS, inertia=_mat(HIP_INERTIA), label="hip")
        # hip + thigh COLLISION BOXES (deploy has these; foot-sphere-only made
        # the gait march in place instead of grip+propel like deploy).
        b.add_shape_box(hip, xform=wp.transform((HIP_BOX_OFF[0] * sx, HIP_BOX_OFF[1] * sy, HIP_BOX_OFF[2]), (0, 0, 0, 1.0)),
                        hx=HIP_BOX[0], hy=HIP_BOX[1], hz=HIP_BOX[2], cfg=cfgm)
        thigh = b.add_link(xform=wp.transform((hx, ty, STAND_Z), (0, 0, 0, 1.0)),
                           mass=THIGH_MASS, inertia=_mat(THIGH_INERTIA), label="thigh")
        b.add_shape_box(thigh, xform=wp.transform((THIGH_BOX_OFF[0], THIGH_BOX_OFF[1] * sy, THIGH_BOX_OFF[2]), (0, 0, 0, 1.0)),
                        hx=THIGH_BOX[0], hy=THIGH_BOX[1], hz=THIGH_BOX[2], cfg=cfgm)
        lower = b.add_link(xform=wp.transform((hx + KNEE_DX, ty, STAND_Z + KNEE_DZ), (0, 0, 0, 1.0)),
                           mass=LOWER_MASS, inertia=_mat(LOWER_INERTIA), label="lower")
        b.add_shape_sphere(lower, xform=wp.transform(FOOT_OFF, (0, 0, 0, 1.0)),
                           radius=FOOT_R, cfg=cfgm)
        # hip_x: range depends on side (left +, right -); hip_y: [0.001,0.6]; knee [-1.2,-0.01]
        hx_lo, hx_hi = (0.001, 0.6) if sy > 0 else (-0.6, -0.001)
        motor(chassis, hip, (1.0, 0.0, 0.0), (hx, hy, 0.0), hx_lo, hx_hi)
        motor(hip, thigh, (0.0, 1.0, 0.0), (0.0, sy * THIGH_DY, 0.0), 0.001, 0.6)
        motor(thigh, lower, (0.0, 1.0, 0.0), (KNEE_DX, 0.0, KNEE_DZ), -1.2, -0.01)

    b.add_ground_plane()
    b.add_articulation(joint_indices, label="spot")
    return b.finalize()
