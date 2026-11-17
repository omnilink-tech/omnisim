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

"""harness_rig -- harness visuals + box-delivery telemetry/legacy rigging.

With ``PHYS_GRASP=1`` (the shipped box_delivery path), the 1 kg box is an untouched
Newton body. This controller only draws the weight-bearing harness and logs box
telemetry; the deploy controller establishes a cup-surface seal and applies finite
suction forces with equal-and-opposite reaction on the robot. No supervisor box pose
or velocity writes occur in that branch.

Without ``PHYS_GRASP=1``, the legacy kinematic sequence draws the harness
(semi-transparent yellow line from the puppet's pelvis to a crane hook
tracking overhead; visuals only -- the carrying force is the deploy harness inside the
physics), and runs the DELIVERY state machine for the BATON real-box demo:

  PARKED   the REAL 1.5 kg box rests on the static cart under genuine contact physics
           (zero writes).
  CARRIED  the real box rides the FK hand centroid: pose + velocity-zero per tick.
  PLACING  the box lerps back over the cart and is released 4 cm up: the landing/settle
           is genuine contact physics.

⛔ THE BACKEND LESSON (2026-07-06, measured twice): the box MUST be physicsBackend "ode"
(a Robot node, like the ghost). A Newton-backed box makes every supervisor write dirty
the solver state: per-tick writes = ~40x re-import slowdown + an untrained physics
regime; even a ONE-TIME write mid-run clobbered the robot to a stale state (instant
face-plant). ODE bodies live outside the Newton state entirely -- writes are safe and
cheap, gravity/contact stay real, and the mixed-backend pattern is already proven by
the ghost robot in this same world. (No box<->robot contact across backends -- fine,
the carried pose keeps 1 cm hand clearance by design.) No runtime weld exists in the
engine, so the carried phase is kinematic-honest; the payload dynamics are the carry
policy's TRAINED plant (CARRY_PAYLOAD_KG). Stated plainly in policy-switching.md.

⚠️ AND THE LESSON'S SOLUTION IS GONE (2026-08-08). src/ode was DELETED (commit
bdc02139). `physicsBackend "ode"` still parses AND is still honoured on a Solid, and
the engine now tells you exactly what that buys (verified with a two-ball probe --
one pinned, one not; only the unpinned one got a Newton body):

    WARNING: DEF BALL_PINNED Solid: This Solid asks for physicsBackend "ode", which
    no longer selects a physics engine: ODE was removed and Newton is the only
    backend. The node will have NO gravity and NO contact -- it is a visual-only
    body. Delete the field to simulate it with Newton.

No gravity and no contact breaks exactly the two phases this state machine relies on
being real physics -- the PARKED rest on the cart and the PLACING landing/settle.
Note the engine's advice ("Delete the field") is NOT safe to follow blindly here,
which is the whole difficulty. The pin is still in
projects/policies/worlds/g1_walk_puppet.omniworld (CARRY_BOX) with a matching note, because
NEITHER value is right today: removing it makes the box a Newton body and reinstates
the ~40x/face-plant regression measured above. The 17 sibling `physicsBackend "ode"`
pins elsewhere in projects/policies/ were on GHOST robots and were removed after being
measured as no-ops (a ghost is staticBase + a physics-stripped URDF, driven purely by
supervisor pose writes -- g1_ghost_preview.omniworld walked bit-identically with and without
the pin). The engineered successor for THIS case is the native Newton kinematic-body
path (commit 31d75482, "kinematic bodies go native"): a body the solver knows about but
does not integrate is precisely what "writes are safe, contact stays real" asks for.
Porting CARRY_BOX to it is an engine-side decision, not a cleanup edit.
"""
import json
import math
import os
import time

from omnisim import Supervisor

CRANE_Z = 2.4
MODE_FILE = os.environ.get("BATON_MODE_FILE", "")
# HAND-TRACK (2026-07-10, owner: "the robot should actually GRAB the box with its hands"): the
# deploy publishes the live FK hand centroid here (HAND_TRACK=1). When fresh, the box binds to
# the REAL hands -- the pick fires only when the hands actually REACH the box, and the carried
# box rides the true hand motion. Stale/missing -> the old pelvis+0.20m heuristic (box_delivery).
HANDS_FILE = os.environ.get("HAND_TRACK_FILE", os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "..", "_scratch", "foot_redesign", "rig_hands.json"))
CART_SIDE = 1.2          # cart keeps this lateral offset from the robot
CART_TOP = 0.87          # cart surface height
BOX_HALF = 0.09          # box half-height
PLACE_S = 1.2            # seconds to lerp the box back to the cart

sup = Supervisor()
ts = int(sup.getBasicTimeStep())
dt = ts / 1000.0

g1 = sup.getFromDef("G1_REAL")
line = sup.getFromDef("HARNESS_LINE")
hook = sup.getFromDef("HARNESS_HOOK")
box = sup.getFromDef("CARRY_BOX")            # Newton in PHYS_GRASP; legacy demos may use ODE
cart = sup.getFromDef("DELIVERY_CART")
cart_b = sup.getFromDef("DELIVERY_CART_B")
box_tr = box.getField("translation") if box else None
box_rot = box.getField("rotation") if box else None
cart_tr = cart.getField("translation") if cart else None
CART_POS = cart_tr.getSFVec3f() if cart_tr else [1.6, -1.4, 0.44]
CART_B_POS = cart_b.getField("translation").getSFVec3f() if cart_b else CART_POS
PICK_S = 0.9             # seconds for the visible lift from the cart to the hands
cyl_h = sup.getFromDef("HARNESS_CYL").getField("height") if sup.getFromDef("HARNESS_CYL") else None
line_tr = line.getField("translation") if line else None
hook_tr = hook.getField("translation") if hook else None

state = "parked"          # parked | carried | placing
place_t = 0.0
place_from = None

# side log (2026-07-10): controller stderr is not captured by the run logs; the box's actual
# trajectory (pick lift, carried track, place + settle) is the delivery demo's numeric evidence.
_rig_log = None
try:
    # BOXRIG_LOG override (2026-07-11): parallel verification instances must not share this
    # telemetry file (interleaved writes once masqueraded as fresh results)
    _rig_log = open(os.environ.get("BOXRIG_LOG", os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "..",
        "_scratch", "foot_redesign", "boxrig.log")), "w", buffering=1)
except OSError:
    _rig_log = None
_rig_t = 0.0
_rig_last = -10.0

while sup.step(ts) != -1:
    _rig_t += dt
    if g1 is None or line_tr is None:
        continue
    px, py, pz = g1.getPosition()
    o = g1.getOrientation()
    yaw = math.atan2(o[3], o[0])
    # harness visuals
    mid = (CRANE_Z + pz) / 2.0
    line_tr.setSFVec3f([px, py, mid])
    if cyl_h is not None:
        cyl_h.setSFFloat(max(0.05, CRANE_Z - pz))
    if hook_tr is not None:
        hook_tr.setSFVec3f([px, py, CRANE_Z])
    # the cart is STATIC (per-tick writes on dynamic bodies force a per-tick solver
    # re-import: measured 40x slowdown + regime shift -- all writes here are ONE-TIME)
    cx, cy = CART_POS[0], CART_POS[1]
    if box_tr is None or not MODE_FILE:
        continue
    try:
        mode = open(MODE_FILE).read().strip()
    except OSError:
        mode = ""
    hand = [px + 0.20 * math.cos(yaw), py + 0.20 * math.sin(yaw), pz + 0.13]
    hands_true = False
    try:
        if os.path.exists(HANDS_FILE) and (time.time() - os.path.getmtime(HANDS_FILE)) < 5.0:
            _hj = json.loads(open(HANDS_FILE).read())
            hand = [float(_hj["x"]), float(_hj["y"]), float(_hj["z"])]
            hands_true = True
    except Exception:
        pass   # partial read mid-replace: keep the heuristic this tick
    cart_rest = [cx, cy, CART_TOP + BOX_HALF]
    if os.environ.get("PHYS_GRASP", "") == "1":
        # PHYS-GRASP (2026-07-10, owner: "physically grab the box only with physics"): the box is
        # a REAL Newton body and the deploy's cup choreography seals and lifts it by suction.
        # The rig NEVER writes it (one write would clobber the Newton state -- the measured 40x
        # lesson); it only logs the box trajectory as the run's physics evidence.
        if _rig_log is not None and _rig_t - _rig_last >= 1.0:
            _rig_last = _rig_t
            _bp = box_tr.getSFVec3f()
            _dhb = math.sqrt((hand[0] - _bp[0]) ** 2 + (hand[1] - _bp[1]) ** 2 + (hand[2] - _bp[2]) ** 2)
            _rig_log.write("t=%6.1f state=PHYS     mode=%-7s box=%+7.2f %+7.2f %5.3f robot=%+7.2f %+7.2f hands=%s dhb=%5.2f\n"
                           % (_rig_t, mode or "-", _bp[0], _bp[1], _bp[2], px, py,
                              "TRUE" if hands_true else "HEUR", _dhb))
        continue
    if _rig_log is not None and _rig_t - _rig_last >= 1.0:
        _rig_last = _rig_t
        _bp = box_tr.getSFVec3f()
        _dhb = math.sqrt((hand[0] - _bp[0]) ** 2 + (hand[1] - _bp[1]) ** 2 + (hand[2] - _bp[2]) ** 2)
        _rig_log.write("t=%6.1f state=%-8s mode=%-7s box=%+7.2f %+7.2f %5.3f robot=%+7.2f %+7.2f hands=%s dhb=%5.2f\n"
                       % (_rig_t, state, mode or "-", _bp[0], _bp[1], _bp[2], px, py,
                          "TRUE" if hands_true else "HEUR", _dhb))
    if state == "parked":
        # the REAL box rests on its cart under genuine contact physics -- zero writes.
        # ⛔ PROXIMITY GATE (owner 2026-07-10, "don't levitate the box to the robot"): the pick
        # only begins once the HANDS are actually at the box (the course now walks the robot to
        # it); a far-away mode flip leaves the box resting until the robot arrives.
        if mode in ("carry", "carryback"):
            _bp0 = box_tr.getSFVec3f()
            if hands_true:
                # TRUE hands: bind only when the REAL hands reach the box (3D, tight) -- the carry
                # pose raises the hands to the box as the specialist blends in, so the grab happens
                # exactly when the hands arrive. Timeout fallback (heuristic radius) so arrival
                # scatter can never strand the box on the cart.
                _d3 = ((hand[0] - _bp0[0]) ** 2 + (hand[1] - _bp0[1]) ** 2 + (hand[2] - _bp0[2]) ** 2) ** 0.5
                place_t += dt        # reuse as the reach timer while parked in carry mode
                # bind EARLY, while the rising carry arms converge on the box and the robot has
                # barely left the pick stand (measured: a 4 s timeout let it walk 0.4 m away and
                # the box chased it from 0.58 m; at ~1.5-2 s the hands are up and ~0.4-0.55 out).
                if _d3 < 0.55 or (place_t > 2.0 and _d3 < 0.9):
                    if _rig_log is not None:
                        _rig_log.write("t=%6.1f GRAB: hands at %5.2f m from the box (%s)\n"
                                       % (_rig_t, _d3, "reach" if _d3 < 0.55 else "timeout"))
                    state = "picking"; place_t = 0.0; place_from = list(_bp0)
            else:
                _d2 = (hand[0] - _bp0[0]) ** 2 + (hand[1] - _bp0[1]) ** 2
                if _d2 < 0.85 ** 2:
                    state = "picking"; place_t = 0.0; place_from = list(_bp0)
        else:
            place_t = 0.0            # reach timer only runs while the mode asks for a carry
    elif state == "picking":
        # TWO-PHASE take (owner 2026-07-10): first straight UP off the cart (a real "take"),
        # then the short move into the hands -- never a cross-room diagonal glide.
        place_t += dt
        u = min(1.0, place_t / PICK_S)
        if u < 0.4:
            v = u / 0.4                      # phase A: vertical lift 14 cm above the rest point
            pos = [place_from[0], place_from[1], place_from[2] + 0.14 * v]
        else:
            v = (u - 0.4) / 0.6              # phase B: into the hands
            lift = [place_from[0], place_from[1], place_from[2] + 0.14]
            pos = [(1 - v) * lift[i] + v * hand[i] for i in range(3)]
        box_tr.setSFVec3f(pos)
        box.setVelocity([0, 0, 0, 0, 0, 0])
        if u >= 1.0:
            state = "carried"
    elif state == "carried":
        # the real ODE box rides the FK hand centroid (suction-demo pattern: pose + vel-zero
        # per tick -- safe and cheap on an ODE body, no Newton state involved)
        box_tr.setSFVec3f(hand)
        if box_rot is not None:
            box_rot.setSFRotation([0, 0, 1, yaw])
        box.setVelocity([0, 0, 0, 0, 0, 0])
        if mode not in ("carry", "carryback"):
            state = "placing"; place_t = 0.0; place_from = list(hand)
    elif state == "placing":
        place_t += dt
        u = min(1.0, place_t / PLACE_S)
        # place onto whichever cart is CLOSER to the robot (cart B outbound, cart A on the return)
        _dA = (px - CART_POS[0]) ** 2 + (py - CART_POS[1]) ** 2
        _dB = (px - CART_B_POS[0]) ** 2 + (py - CART_B_POS[1]) ** 2
        _cp = CART_B_POS if _dB < _dA else CART_POS
        cart_rest = [_cp[0], _cp[1], CART_TOP + BOX_HALF]
        # TWO-PHASE set-down (owner 2026-07-10): carry the box OVER the rest point at hand
        # height first, then lower and release 4 cm up -- reads as "sets it on the cart",
        # not a diagonal glide. The landing/settle stays genuine contact physics.
        _hover = [cart_rest[0], cart_rest[1], max(place_from[2], cart_rest[2] + 0.12)]
        if u < 0.6:
            v = u / 0.6                      # phase A: horizontal, to above the rest point
            pos = [(1 - v) * place_from[i] + v * _hover[i] for i in range(3)]
        else:
            v = (u - 0.6) / 0.4              # phase B: straight down to the release height
            tgt = [cart_rest[0], cart_rest[1], cart_rest[2] + 0.04]
            pos = [(1 - v) * _hover[i] + v * tgt[i] for i in range(3)]
        box_tr.setSFVec3f(pos)
        box.setVelocity([0, 0, 0, 0, 0, 0])
        if u >= 1.0:
            state = "parked"   # release 4 cm up: the settle onto the cart is real contact
        elif mode in ("carry", "carryback"):
            state = "carried"
