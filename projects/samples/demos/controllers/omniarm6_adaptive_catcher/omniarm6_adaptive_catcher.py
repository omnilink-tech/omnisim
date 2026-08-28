"""Live ballistic interception for the OmniArm 6 adaptive-catch demo.

Two modes share the same world and launch:

* ``fixed`` holds the cup on the nominal centreline.  It is the control run.
* ``adaptive`` samples the disturbed part after the deflector, predicts where its
  trajectory crosses the catch plane, and retargets the cup once.

The catch itself is a proximity-triggered suction capture.  The controller
records the prediction, measured crossing, closest approach, catch error, and
post-catch hold so the edited film can keep every public claim bounded.
"""

import json
import math
import os
import sys
from pathlib import Path

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "omnilink_arm_bridge"))

from omnisim import Supervisor  # noqa: E402
from omnilink_arm_bridge import (  # noqa: E402
    ArmBridge,
    _mat3_to_axis_angle,
    forward_kinematics_pose,
    warmup_reload,
)
from _arm_configs import get_config  # noqa: E402


MODE = os.environ.get("OMNISIM_INTERCEPT_MODE", "adaptive").strip().lower()
if MODE not in {"fixed", "adaptive"}:
    MODE = "adaptive"

B_X = 1.55
INTERCEPT_X = float(os.environ.get("OMNISIM_INTERCEPT_X", "1.02"))
NOMINAL_Y = 0.0
NOMINAL_Z = 0.70
# The deflector is centred at x=0.50 m.  Sampling after x=0.65 m makes the
# estimate a post-contact measurement instead of a pre-impact guess.
OBSERVE_AFTER_X = float(os.environ.get("OMNISIM_INTERCEPT_OBSERVE_X", "0.65"))
TCP_OFFSET_Z = 0.25
GRAVITY = 9.81
CAPTURE_RADIUS_M = 0.24

robot = Supervisor()
warmup_reload(robot)
dt_ms = int(robot.getBasicTimeStep())
dt_s = dt_ms / 1000.0
bridge = ArmBridge(robot, get_config("omniarm6"), "omniarm6", gripper_id=None)
ik = bridge.cfg["ik"]
part = robot.getFromDef("PART")

result_path = Path(os.environ.get(
    "OMNISIM_INTERCEPT_RESULT",
    str(Path(__file__).with_name(f"_adaptive_intercept_{MODE}.json")),
))
log_path = result_path.with_suffix(".log")
result_path.parent.mkdir(parents=True, exist_ok=True)
log_handle = log_path.open("w", encoding="utf-8", buffering=1)


def emit(message):
    print(message, flush=True)
    log_handle.write(message + "\n")


def world_to_local(world_xyz):
    return (B_X - world_xyz[0], -world_xyz[1], world_xyz[2])


def local_to_world(local_xyz):
    return (B_X - local_xyz[0], -local_xyz[1], local_xyz[2])


def tcp_local():
    return forward_kinematics_pose(
        ik["chain"], bridge._read_q(), (0.0, 0.0, TCP_OFFSET_Z)
    )[0]


def tcp_world():
    return local_to_world(tcp_local())


held = None


def engage_suction(node):
    global held
    orientation = node.getOrientation()
    rotation = _mat3_to_axis_angle([
        [orientation[0], orientation[1], orientation[2]],
        [orientation[3], orientation[4], orientation[5]],
        [orientation[6], orientation[7], orientation[8]],
    ])
    node.setVelocity([0.0] * 6)
    held = (
        node,
        node.getField("translation"),
        node.getField("rotation"),
        rotation,
    )


def apply_suction():
    if held is None:
        return
    node, translation, rotation, initial_rotation = held
    cup = tcp_world()
    translation.setSFVec3f([cup[0], cup[1], cup[2]])
    rotation.setSFRotation(initial_rotation)
    node.setVelocity([0.0] * 6)


def step_for(seconds, keep_held=False):
    for _ in range(max(1, int(seconds * 1000.0 / dt_ms))):
        if robot.step(dt_ms) == -1:
            return False
        bridge.tick(robot.getTime())
        if keep_held:
            apply_suction()
    return True


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def predict_crossing(position, velocity):
    vx = velocity[0]
    if vx <= 0.2:
        return None
    flight_s = (INTERCEPT_X - position[0]) / vx
    if flight_s <= 0.0 or flight_s > 1.5:
        return None
    y = position[1] + velocity[1] * flight_s
    z = position[2] + velocity[2] * flight_s - 0.5 * GRAVITY * flight_s * flight_s
    return {
        "flight_s": flight_s,
        "raw_world_m": [INTERCEPT_X, y, z],
        "target_world_m": [
            INTERCEPT_X,
            clamp(y, -0.34, 0.34),
            clamp(z, 0.30, 1.10),
        ],
    }


emit(f"[intercept] mode={MODE}")
if robot.step(dt_ms) == -1:
    raise SystemExit(0)
bridge.tick(robot.getTime())
nominal_local = world_to_local((INTERCEPT_X, NOMINAL_Y, NOMINAL_Z))
bridge.act_set_tcp_pose(
    nominal_local,
    tcp_offset_z=TCP_OFFSET_Z,
    duration_s=2.2,
)
step_for(2.8)
nominal_actual = list(tcp_world())
emit(
    "[intercept] nominal cup world=(%.3f,%.3f,%.3f)"
    % tuple(nominal_actual)
)

launched = False
prediction = None
first_prediction = None
prediction_samples = 0
retarget_result = None
previous_part = None
crossing = None
closest = {"distance_m": 999.0, "part_world_m": None, "cup_world_m": None}
caught = False
catch_error = None
miss_reason = None

while robot.step(dt_ms) != -1:
    bridge.tick(robot.getTime())
    p = list(part.getPosition())
    v = list(part.getVelocity()[:3])
    speed = math.sqrt(sum(x * x for x in v))
    cup = list(tcp_world())
    distance = math.dist(p, cup)
    if distance < closest["distance_m"]:
        closest = {
            "distance_m": distance,
            "part_world_m": list(p),
            "cup_world_m": list(cup),
        }

    if not launched and speed > 1.5 and v[0] > 1.0 and p[0] >= 0.06 and p[2] > 0.80:
        launched = True
        emit(
            "[intercept] launch observed p=(%.3f,%.3f,%.3f) "
            "v=(%.3f,%.3f,%.3f)" % tuple(p + v)
        )

    if (
        launched
        and MODE == "adaptive"
        and first_prediction is None
        and p[0] >= OBSERVE_AFTER_X
    ):
        candidate = predict_crossing(p, v)
        if candidate is not None:
            prediction = candidate
            prediction_samples = 1
            first_prediction = candidate
            emit(
                "[intercept] crossing estimate=(%.3f,%.3f,%.3f) eta=%.3fs"
                % tuple(candidate["target_world_m"] + [candidate["flight_s"]])
            )
            target_local = world_to_local(candidate["target_world_m"])
            q_target, ik_residual = bridge._topdown_solve(
                target_local,
                seed=bridge._read_q(),
                tcp_offset_z=TCP_OFFSET_Z,
            )
            if q_target is not None:
                # One observed trajectory produces one new hold target.  Motor
                # velocity and effort limits still bound the physical arm.
                with bridge.lock:
                    bridge.motion = ("hold", {"q": list(q_target)})
                retarget_result = {
                    "accepted": True,
                    "tracking": True,
                    "ik_residual_m": ik_residual,
                }

    if launched and previous_part is not None and previous_part[0] < INTERCEPT_X <= p[0]:
        alpha = (INTERCEPT_X - previous_part[0]) / max(1e-9, p[0] - previous_part[0])
        crossing = [
            INTERCEPT_X,
            previous_part[1] + alpha * (p[1] - previous_part[1]),
            previous_part[2] + alpha * (p[2] - previous_part[2]),
        ]
        emit(
            "[intercept] measured crossing=(%.3f,%.3f,%.3f) "
            "cup=(%.3f,%.3f,%.3f)"
            % tuple(crossing + cup)
        )

    near_plane = INTERCEPT_X - 0.16 <= p[0] <= INTERCEPT_X + 0.16
    if launched and near_plane and distance <= CAPTURE_RADIUS_M:
        catch_error = distance
        engage_suction(part)
        apply_suction()
        caught = True
        emit(
            f"[intercept] CAUGHT live proximity={distance:.3f}m "
            f"at p=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})"
        )
        break

    if launched and (p[0] > INTERCEPT_X + 0.38 or p[2] < 0.08):
        miss_reason = "part passed catch volume"
        break

    previous_part = p

held_after_lift = False
final_part = list(part.getPosition())
final_cup = list(tcp_world())
if caught:
    trophy_world = (1.05, 0.30, 0.88)
    bridge.act_set_tcp_pose(
        world_to_local(trophy_world),
        tcp_offset_z=TCP_OFFSET_Z,
        duration_s=1.5,
    )
    step_for(2.0, keep_held=True)
    final_part = list(part.getPosition())
    final_cup = list(tcp_world())
    held_after_lift = math.dist(final_part, final_cup) < 0.04
    emit(
        f"[intercept] PROOF lift held={held_after_lift} "
        f"part=({final_part[0]:.3f},{final_part[1]:.3f},{final_part[2]:.3f})"
    )
else:
    emit(
        f"[intercept] MISS closest={closest['distance_m']:.3f}m "
        f"reason={miss_reason or 'no valid crossing'}"
    )

payload = {
    "mode": MODE,
    "launched": launched,
    "nominal_cup_world_m": nominal_actual,
    "prediction": prediction,
    "first_prediction": first_prediction,
    "prediction_samples": prediction_samples,
    "retarget_accepted": bool(retarget_result and retarget_result.get("accepted")),
    "measured_crossing_world_m": crossing,
    "closest_approach": closest,
    "capture_radius_m": CAPTURE_RADIUS_M,
    "caught": caught,
    "catch_error_m": catch_error,
    "held_after_lift": held_after_lift,
    "final_part_world_m": final_part,
    "final_cup_world_m": final_cup,
    "miss_reason": miss_reason,
    "claim_boundary": (
        "The part follows simulated rigid-body flight and glances off an authored static deflector. "
        "The catcher observes supervisor world-state after contact, predicts one crossing, retargets once, "
        "and engages a proximity-triggered kinematic suction capture."
    ),
}
result_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
emit(f"[intercept] RESULT {'CAUGHT_AND_HELD' if held_after_lift else 'MISS'}")

while robot.step(dt_ms) != -1:
    bridge.tick(robot.getTime())
    if caught:
        apply_suction()
