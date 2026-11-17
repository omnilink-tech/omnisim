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

"""Whole-body control layer (the Newton-MPC-stack plan, section 9).

MVP realization: map the centroidal MPC's per-foot ground-reaction forces to actuated
joint feedforward torques via the contact-Jacobian transpose (tau_a = -sum J_c,a^T f), and
apply them ON TOP of the deterministic position servo (FF-on-servo -- the proven OmniSim
substrate that already stands and survives throws). The servo holds the squat posture; this
FF adds the centroidal balance (incl. gravity support, since the GRFs carry the weight).

Swing feet (M5/M6) are tracked as position targets by the leg IK on the servo (no force),
so this WBC only realizes the STANCE forces. The later inverse-dynamics QP (plan's full
WBC) would replace this with a single QP over (qddot, tau, lambda); this MVP is the
debuggable first version.
"""
import numpy as np


class WBC:
    def __init__(self, world, leg_map, act_dofs):
        self._mj = world._cmpc_mj
        self.leg = leg_map                 # leg -> {"foot_bid", ...}
        self.act_dofs = list(act_dofs)     # actuated-joint mjc dof indices (for tau columns)
        self.act_nd = None                 # newton dof per actuated joint (joint_f index)

    def contact_jacobian(self, world, d, leg):
        mj = self._mj; mjm = world.solver.mj_model
        nv = int(mjm.nv)
        Jp = np.zeros((3, nv)); Jr = np.zeros((3, nv))
        mj.mj_jacBodyCom(mjm, d, Jp, Jr, self.leg[leg]["foot_bid"])
        return Jp                          # 3 x nv (linear part; point-contact GRF)

    def forces_to_tau(self, world, d, forces):
        """tau_a (per actuated joint, mjc-dof order) = -sum_legs J_c[leg]_a^T f[leg]."""
        tau = np.zeros(len(self.act_dofs))
        for leg, f in forces.items():
            if leg.startswith("_"):
                continue
            J = self.contact_jacobian(world, d, leg)        # 3 x nv
            Ja = J[:, self.act_dofs]                          # 3 x nact
            tau += -(Ja.T @ f)
        return tau
