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

"""engine_facts.py -- what this ARM must know about THIS engine, and nothing
else.

The split this file exists to keep honest:

* ``rungs.py`` owns PHYSICS.  Rung 8's contract declares a grip force in
  newtons and the Coulomb bound that justifies it.  It does not, and must not,
  know what a ``LinearMotor`` is.
* this file owns the ACTUATOR that produces that force on OmniSim.  It is the
  same split the MuJoCo arm makes when it derives its ``kv`` from ``I/dt``: the
  contract says "a velocity servo", the arm says "and here is the gain that
  makes one on this engine".

Both ``worldgen.py`` (which writes the world) and ``ladder0_probe`` (which
writes the commands) read it, so the world's motor and the driver's target can
never describe different actuators.

WHY RUNG 8 CANNOT JUST CALL ``setForce``
----------------------------------------
On the Newton path ``setForce`` does NOT put a joint in force mode.  Every
joint is built ``POSITION_VELOCITY`` with ``targetKe = effortLimit * 10``, so
the PD servo stays live, anchored at the last ``setPosition``, and a call that
reads like "squeeze with N newtons" is really a spring pulling toward wherever
the joint was last told to go.  The documented consequence is a gripper that
buries its pads inside the part and launches it on the lift -- which is exactly
the failure ``part_speed_max`` exists to catch, so producing it by accident
here would make the rung measure this file instead of the engine.

The supported way to get a known force is a POSITION TARGET WITH A KNOWN
INTERFERENCE.  The pad's servo and the contact act in series, so

    bite = F (1/kp + 1/ke),    kp = 10 * maxForce,   ke = newtonContactKe

and ``rungs.rung8_bite_m`` writes that algebra once for whichever arm needs it.

CHOOSING ``maxForce``
---------------------
``kp`` is 10x it, so a LARGER limit means a STIFFER servo and therefore a
SMALLER interference for the same force.  200 N gives kp = 2000 N/m and a
2.7 mm bite -- under half the pad's own thickness, and small enough that the
pad stops at the part's surface rather than inside it.  It is not a force the
motor ever delivers: at the commanded target the servo is developing
RUNG8_GRIP_N, i.e. 1.5 % of the limit.

``newtonContactKe`` is READ, NOT SET.  2500 N/m is the engine's default and
this arm leaves it there, for the same reason the MuJoCo arm runs on MuJoCo's
own defaults: a contact stiffness set by the scene file moves the grasp's
outcome out of the engine and into the file.  If the number ever changes in the
engine, this constant is where the arm finds out it was wrong -- the bite would
be off and the grasp would report it.
"""

from __future__ import annotations

import os
import sys

_HERE = os.path.abspath(os.path.dirname(__file__))
_LADDER0 = os.path.dirname(_HERE)
if _LADDER0 not in sys.path:
    sys.path.insert(0, _LADDER0)

import rungs                                     # noqa: E402

# The finger motor's effort limit, in newtons.  Sets the servo stiffness at
# 10x, per the engine's own POSITION_VELOCITY joint construction.
RUNG8_FINGER_MAXFORCE = 200.0                    # N
RUNG8_FINGER_KP = 10.0 * RUNG8_FINGER_MAXFORCE   # = 2000 N/m

# The engine's default contact stiffness.  Not declared by the world; recorded
# here because the interference depends on it.
NEWTON_CONTACT_KE_DEFAULT = 2500.0               # N/m

# ...which is exactly why an ATTRIBUTION RUN THAT CHANGES ke MUST SAY SO.
# ``variants.py`` sweeps the shipped friction-grasp recipe one field at a time,
# and one of those fields is ``newtonContactKe``.  With the interference held
# fixed, raising ke from 2500 to 8000 raises the delivered grip force by 44%
# at the same time -- so the variant would be changing TWO things and its red
# would not be attributable to contact stiffness at all.  MEASURED before this
# hook existed: the ke variant ejected the payload at 1.04 m/s and read as
# "a firmer contact breaks the grasp", which is not what it showed.
NEWTON_CONTACT_KE = float(os.environ.get("LADDER0_NEWTON_KE")
                          or NEWTON_CONTACT_KE_DEFAULT)

# How far past the part's surface each pad is commanded, to develop
# rungs.RUNG8_GRIP_N through the servo and the contact in series.
RUNG8_BITE = rungs.rung8_bite_m(RUNG8_FINGER_KP, NEWTON_CONTACT_KE)

# Pad centre positions: where it starts, and where it is commanded to.  The
# joint's zero is the OPEN position (the pad Solid is authored there), so the
# commanded joint target is the displacement from open, not the pad's y.
RUNG8_PAD_CLOSED_Y = rungs.RUNG8_PAD_TOUCH_Y - RUNG8_BITE
RUNG8_FINGER_CLOSE_Q = rungs.RUNG8_PAD_OPEN_Y - RUNG8_PAD_CLOSED_Y
RUNG8_FINGER_OPEN_Q = 0.0


# --------------------------------------------------------------------------
# CONTRACT.md 3b -- this arm's rung-8 R2 declarations
# --------------------------------------------------------------------------
#
# The rule: rung 8 declares a friction MODEL, and an arm may spell that model
# in whatever its engine requires -- but ONLY settings whose effect is how
# accurately the solver enforces the Coulomb model the contract already
# declared, each shown by a published sweep to be a converged budget rather
# than a fitted value, each named here with the engine default it departs from.
#
# {field: (value, engine default, why it is model-accuracy and not tuning)}
RUNG8_DECLARATIONS = {
    "newtonCone": (
        '"elliptic"', '"" (pyramidal)',
        "the EXACT Coulomb cone. MuJoCo's default is a pyramid INSCRIBED in "
        "it, i.e. a polygonal approximation of the model rung 8 declares -- so "
        "asking for the ellipse is asking for the declared model, not a "
        "better one. MEASURED alone: the payload is carried the full 0.45 m "
        "but slips 21.3 mm through the pads (carry_rel 0.021294), against "
        "474.7 mm and a dropped payload on the pyramid"),
    "newtonImpratio": (
        "10", "0 (unset -> MuJoCo stock 1)",
        "the frictional-to-normal constraint impedance ratio. At 1 the "
        "friction constraint is exactly as soft as the normal one, so a "
        "contact under sustained tangential load DRIFTS while its normal "
        "force sits at the commanded value -- the Coulomb bound is satisfied "
        "and the part still slides. It changes how strictly mu binds, never "
        "mu. MEASURED converged: carry_rel 0.002559 / 0.002615 / 0.002640 / "
        "0.002648 at 10 / 30 / 100 / 300 -- 0.09 mm of spread over a 30x "
        "range. 4 is the knee (0.004448) and 1-2 are red, so 10 is the first "
        "value inside the plateau"),
}

# NOT DECLARED, and the reason is a measurement rather than a preference.
# ``newtonNoslipIterations`` is admissible under R2 (it is MuJoCo's own
# post-solve friction pass, and it is what the MuJoCo arm declares) and this
# engine did not expose it until it was plumbed for exactly this rung -- but on
# THIS scene it does not move the answer: on the declared configuration it
# changes carry_rel from 0.002559 to 0.002652, and on the engine-default cone
# it does not rescue the grasp at all (0.4747 -> 0.4796 at 5 iterations, the
# payload still dropped). R2 ADMITS a declaration; it does not oblige one, and
# a field that changes nothing does not belong in a world file.
RUNG8_NOT_DECLARED = {
    "newtonNoslipIterations": (
        "measured to change carry_rel by 0.09 mm on the declared "
        "configuration and to not rescue the default one"),
}


def facts():
    """Everything above, for ``meta`` -- so a row says which actuator produced
    it.  A grasp measured through a 2.7 mm interference and a grasp measured
    through a true force mode are not the same measurement, and after the fact
    they are indistinguishable without this."""
    return {
        "rung8_grip_mechanism": "position target with a known interference",
        "rung8_grip_force_n": rungs.RUNG8_GRIP_N,
        "rung8_grip_force_min_n": rungs.rung8_grip_force_bound(),
        "rung8_finger_maxforce_n": RUNG8_FINGER_MAXFORCE,
        "rung8_finger_kp_n_per_m": RUNG8_FINGER_KP,
        "rung8_contact_ke_n_per_m": NEWTON_CONTACT_KE,
        "rung8_contact_ke_is_default": (
            NEWTON_CONTACT_KE == NEWTON_CONTACT_KE_DEFAULT),
        "rung8_bite_m": RUNG8_BITE,
        "rung8_finger_close_q_m": RUNG8_FINGER_CLOSE_Q,
        # CONTRACT.md 3b R4: a cell whose meta does not list its declarations
        # is not a comparable cell.
        "rung8_solver_declarations": {
            k: {"value": v[0], "engine_default": v[1], "why": v[2]}
            for k, v in RUNG8_DECLARATIONS.items()},
        "rung8_not_declared": dict(RUNG8_NOT_DECLARED),
    }


if __name__ == "__main__":
    for k, v in facts().items():
        print("%-28s %s" % (k, v))
