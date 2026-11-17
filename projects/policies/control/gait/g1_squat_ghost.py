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

"""Generate a FEASIBLE walking ghost for the G1 at the canonical trainer's stable
operating point (deep squat hip -0.30 / knee 0.52 / ankle -0.23, kp300/kd15).

Why a NEW ghost (not the BC-clone-calibrated one in _scratch/g1_ghost_calibrated.npz):
that ghost lives at the SHALLOW Unitree operating point (hip -0.1 / knee 0.3, kp100/kd5)
where a *from-scratch* policy cannot bootstrap balance (marginal stability -> the
synchronized-fall cascade). The canonical pure-RL trainer SOLVED the G1 walk by moving
to a deeper, statically-stable squat. A ghost-built (from-scratch) policy must live in
that same stable basin, so we DESIGN the ghost there.

The ghost is parametric and feasible BY CONSTRUCTION: small sinusoidal leg motion around
the stable squat (thigh fore/aft swing + swing-phase knee lift + ankle counter-rotate +
LIPM-ish hip-roll sway). Left/right are a half-cycle apart. It is the MOTION REFERENCE
the RL tracker imitates -- generated, never recorded.

Output: _scratch/g1_squat_ghost.npz  q (W,12) in g1_cfg legs order:
  [L_hip_pitch,L_hip_roll,L_hip_yaw,L_knee,L_ank_pitch,L_ank_roll,  R_* ]
"""
import math
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[3]

# Canonical trainer's stable deep-squat default (g1_cfg.default_pose).
DEFAULT = np.array([-0.30, 0.0, 0.0, 0.52, -0.23, 0.0,
                    -0.30, 0.0, 0.0, 0.52, -0.23, 0.0], np.float32)
W = 256
DUTY = 0.6          # stance fraction


def _leg(phi):
    """Per-leg offsets (hip_pitch, hip_roll, hip_yaw, knee, ank_pitch, ank_roll)
    as a function of that leg's local phase phi in [0,1). Stance [0,duty), swing
    [duty,1). Returns deltas to ADD to the squat default."""
    # thigh swings fore/aft over the whole cycle (sinusoid); foot lands forward,
    # pushes back through stance -> forward progress.
    hip_pitch = -0.28 * math.sin(2 * math.pi * phi)          # - = forward (more flexed)
    # knee flexes during SWING to lift the foot for ground clearance.
    sw = max(0.0, (phi - DUTY) / (1.0 - DUTY))               # 0->1 across swing
    knee = 0.35 * math.sin(math.pi * sw) if phi >= DUTY else 0.0
    # ankle counter-rotates to keep the foot roughly flat through stance + toe-off.
    ank_pitch = 0.12 * math.sin(2 * math.pi * phi + 0.6)
    return np.array([hip_pitch, 0.0, 0.0, knee, ank_pitch, 0.0], np.float32)


def build(out=None):
    out = out or (REPO / "_scratch/g1_squat_ghost.npz")
    q = np.zeros((W, 12), np.float32)
    for i in range(W):
        ph = i / W
        phL = ph
        phR = (ph + 0.5) % 1.0
        legL = _leg(phL)
        legR = _leg(phR)
        # LIPM-ish lateral sway: shift hip_roll toward the stance leg (common mode).
        sway = 0.05 * math.sin(2 * math.pi * ph)
        legL[1] = sway                                       # L hip_roll
        legR[1] = sway                                       # R hip_roll (common mode)
        q[i, 0:6] = DEFAULT[0:6] + legL
        q[i, 6:12] = DEFAULT[6:12] + legR
    # feasibility sanity: bounded deviation from the stable squat
    dev = np.abs(q - DEFAULT[None]).max()
    assert dev < 0.6, f"ghost deviates {dev:.2f} rad from squat -- too aggressive"
    np.savez(str(out), q=q, default=DEFAULT, W=W, duty=DUTY,
             note="parametric feasible walking ghost at the stable deep-squat operating point")
    print(f"[g1_squat_ghost] wrote {out}  q{q.shape}  max|dev from squat|={dev:.3f} rad")
    print(f"  hip_pitch range [{q[:,0].min():.2f},{q[:,0].max():.2f}] "
          f"knee range [{q[:,3].min():.2f},{q[:,3].max():.2f}]")
    return out


if __name__ == "__main__":
    build()
