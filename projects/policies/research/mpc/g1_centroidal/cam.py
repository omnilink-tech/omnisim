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

"""Arm centroidal-angular-momentum (CAM) strategy -- the hip/arm strategy that produces
balance moment BEYOND the ankle CoP limit (the Newton-MPC-stack plan, the WBC angular-
momentum task; my build-derived finding: the SRBM contact-force moment is CoP-limited
~38 N.m, so a pitch beyond that needs limb angular momentum from the arms).

Realization (MVP): a reactive vigorous arm windmill -- shoulder pitch/roll (+ elbow) driven
by the torso orientation error + rate, applied as position targets the servo tracks. A fast,
large-amplitude arm swing carries angular momentum whose reaction moment on the torso arrests
the body rotation. This is the same mechanism as the deterministic arm_balance but at much
higher authority (full arm range, fast), to push the recovery limit past ankle-only.

Arm joints by newton DOF (G1 breadth-first layout, verified vs the squat nominal):
  shoulder_pitch = 11,12 ; shoulder_roll = 15,16 ; shoulder_yaw = 19,20 ; elbow = 23,24.
"""


def _clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class ArmCAM:
    # (left_dof, right_dof) per arm joint
    SH_PITCH = (11, 12)
    SH_ROLL = (15, 16)
    ELBOW = (23, 24)

    def __init__(self, cfg):
        self.kp_p = cfg.get("kp_pitch", 7.0)
        self.kd_p = cfg.get("kd_pitch", 0.9)
        self.kp_r = cfg.get("kp_roll", 5.0)
        self.kd_r = cfg.get("kd_roll", 0.6)
        self.clamp = cfg.get("clamp", 1.8)       # max arm-swing deviation (rad) -- big = vigorous
        self.elbow_gain = cfg.get("elbow_gain", 0.6)
        self.sign = cfg.get("sign", 1.0)         # shoulder_pitch sign vs forward pitch (tunable)

    def deviations(self, roll, pitch, roll_rate, pitch_rate):
        """Return {newton_dof: target_deviation} to ADD on the nominal arm pose.
        Forward pitch (pitch>0) -> swing both arms forward/up (reaction moment back).
        Roll -> asymmetric shoulder-roll (reaction moment to counter the roll)."""
        sp = self.sign * _clip(self.kp_p * pitch + self.kd_p * pitch_rate, -self.clamp, self.clamp)
        el = self.elbow_gain * sp
        rr = _clip(self.kp_r * roll + self.kd_r * roll_rate, -self.clamp, self.clamp)
        dev = {}
        for d in self.SH_PITCH:
            dev[d] = sp
        for d in self.ELBOW:
            dev[d] = el
        # roll: push both shoulder-rolls the same way to shift arm mass laterally
        dev[self.SH_ROLL[0]] = rr
        dev[self.SH_ROLL[1]] = rr
        return dev
