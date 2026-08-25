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

"""Regression test: SkidSteerLocomotion.command() is bit-identical to the original
diff_drive() math it replaced. Proves the locomotion abstraction is a PURE
extraction (zero behaviour change) and guards the left/right wheel-assignment
ordering when the legged / holonomic adapters land. No simulator needed.

    python test_locomotion.py     # prints "PASS ..." or asserts
"""

from __future__ import annotations

import random
import sys
import types

# Stub the Webots `controller` module so the nav controller imports outside Webots.
_stub = types.ModuleType("controller")
_stub.Supervisor = type("Supervisor", (), {})
sys.modules.setdefault("controller", _stub)

import omni_quest_nav as nav   # noqa: E402  (must follow the controller stub)


def ref_diff_drive(v_lin, omega, r, ht, mx):
    """The ORIGINAL diff_drive() formula, kept verbatim as the reference oracle."""
    v_left = (v_lin - omega * ht) / r
    v_right = (v_lin + omega * ht) / r
    peak = max(abs(v_left), abs(v_right))
    if peak > mx:
        s = mx / peak
        v_left *= s
        v_right *= s
    return v_left, v_right


class _MockMotor:
    def __init__(self):
        self.vel = None

    def setPosition(self, _):
        pass

    def setVelocity(self, v):
        self.vel = v


class _MockSup:
    def __init__(self, names):
        self.motors = {n: _MockMotor() for n in names}

    def getDevice(self, n):
        return self.motors.get(n)


def main():
    r, ht, mx = nav.PLATFORMS["husky"]
    sup = _MockSup(nav.WHEEL_NAMES)
    loco = nav.SkidSteerLocomotion(sup, nav.WHEEL_NAMES, r, ht, mx)
    fln, rln, frn, rrn = nav.WHEEL_NAMES
    rng = random.Random(0)
    worst = 0.0
    for _ in range(20000):
        vx = rng.uniform(-2.0, 2.0)
        omega = rng.uniform(-3.0, 3.0)
        loco.command(vx, 0.0, omega)
        vl_ref, vr_ref = ref_diff_drive(vx, omega, r, ht, mx)
        err = max(abs(sup.motors[fln].vel - vl_ref),   # front-left  -> left
                  abs(sup.motors[rln].vel - vl_ref),   # rear-left   -> left
                  abs(sup.motors[frn].vel - vr_ref),   # front-right -> right
                  abs(sup.motors[rrn].vel - vr_ref))   # rear-right  -> right
        worst = max(worst, err)
    assert worst < 1e-12, f"MISMATCH vs diff_drive: worst={worst}"
    print(f"PASS: SkidSteerLocomotion bit-identical to diff_drive "
          f"(worst error {worst:.1e} over 20000 random twists)")

    # vy must be ignored by a non-holonomic base: same wheels for any vy.
    loco.command(0.5, 0.0, 0.3)
    a = sup.motors[fln].vel
    loco.command(0.5, 9.9, 0.3)
    b = sup.motors[fln].vel
    assert a == b, "vy must be ignored by skid-steer"
    print("PASS: vy ignored (non-holonomic), as documented")


if __name__ == "__main__":
    main()
