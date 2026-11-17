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

"""Gait scheduler (the Newton-MPC-stack plan, section E / M5).

A tick-based state machine for the disciplined ladder: SHIFT weight over the stance foot
(double support) -> lift the swing foot (single support) -> HOLD -> lower -> re-center, then
alternate. The centroidal MPC reads `com_frac` (how far to move com_ref toward the stance
foot) and `stance`/`swing` (the contact schedule); the swing planner reads `swing_z`.

This is the M5 (single-foot lift) cycle; M6 (stepping) adds a forward foothold to the swing.
"""
import math


class GaitScheduler:
    def __init__(self, cfg):
        self.t_shift = int(cfg.get("t_shift", 60))   # ticks to shift weight (double support)
        self.t_swing = int(cfg.get("t_swing", 40))   # ticks to lift / lower
        self.t_hold = int(cfg.get("t_hold", 40))     # ticks to hold the foot up
        self.lift_h = float(cfg.get("lift_h", 0.05))
        self.k = 0
        self.swing = cfg.get("first_swing", "right")

    def update(self):
        T1 = self.t_shift
        T2 = T1 + self.t_swing
        T3 = T2 + self.t_hold
        T4 = T3 + self.t_swing
        T5 = T4 + self.t_shift
        k = self.k
        self.k += 1
        if k >= T5:                      # cycle done -> alternate the swing foot
            self.k = 0
            self.swing = "left" if self.swing == "right" else "right"
            k = 0
        stance = "left" if self.swing == "right" else "right"
        if k < T1:                       # shift weight onto the stance foot (double support)
            return {"stance": ["left", "right"], "swing": None,
                    "stance_foot": stance, "com_frac": k / max(1, T1), "swing_z": 0.0}
        if k < T2:                       # lift the swing foot (single support)
            ph = (k - T1) / max(1, self.t_swing)
            return {"stance": [stance], "swing": self.swing,
                    "stance_foot": stance, "com_frac": 1.0,
                    "swing_z": self.lift_h * math.sin(0.5 * math.pi * ph)}
        if k < T3:                       # hold up
            return {"stance": [stance], "swing": self.swing,
                    "stance_foot": stance, "com_frac": 1.0, "swing_z": self.lift_h}
        if k < T4:                       # lower
            ph = (k - T3) / max(1, self.t_swing)
            return {"stance": [stance], "swing": self.swing,
                    "stance_foot": stance, "com_frac": 1.0,
                    "swing_z": self.lift_h * math.cos(0.5 * math.pi * ph)}
        # re-center (double support)
        ph = (k - T4) / max(1, self.t_shift)
        return {"stance": ["left", "right"], "swing": None,
                "stance_foot": stance, "com_frac": 1.0 - ph, "swing_z": 0.0}
