"""Record the physical deflection in the adaptive-intercept demo.

This supervisor is observation-only: it never writes PART's pose, velocity, or
forces.  The cyan deflector in the world is an authored static collider, so the
trajectory change comes from the solver's rigid-body contact.
"""

import json
import math
import os
from pathlib import Path

from omnisim import Supervisor


robot = Supervisor()
dt_ms = int(robot.getBasicTimeStep())
part = robot.getFromDef("PART")
result_path = Path(os.environ.get(
    "OMNISIM_INTERCEPT_OBSERVER_OUT",
    str(Path(__file__).with_name("_deflection_result.json")),
))

launch = None
post_contact = None
peak_lateral_speed = 0.0
min_forward_speed = 999.0

while robot.step(dt_ms) != -1:
    p = list(part.getPosition())
    v = list(part.getVelocity()[:3])
    speed = math.sqrt(sum(x * x for x in v))

    if launch is None and speed > 1.5 and v[0] > 1.0 and p[0] >= 0.06 and p[2] > 0.80:
        launch = {
            "time_s": robot.getTime(),
            "position_m": p,
            "velocity_m_s": v,
        }
        print(
            "[observer] free flight p=(%.3f,%.3f,%.3f) v=(%.3f,%.3f,%.3f)"
            % tuple(p + v),
            flush=True,
        )

    if launch is not None:
        peak_lateral_speed = max(peak_lateral_speed, abs(v[1]))
        min_forward_speed = min(min_forward_speed, v[0])
        if post_contact is None and p[0] >= 0.74:
            post_contact = {
                "time_s": robot.getTime(),
                "position_m": p,
                "velocity_m_s": v,
            }
            result = {
                "disturbance": "authored static deflector; simulated rigid-body contact",
                "observer_writes_part_state": False,
                "launch": launch,
                "post_contact": post_contact,
                "peak_abs_lateral_speed_m_s": peak_lateral_speed,
                "min_forward_speed_m_s": min_forward_speed,
            }
            result_path.parent.mkdir(parents=True, exist_ok=True)
            result_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
            print(
                "[observer] post-contact p=(%.3f,%.3f,%.3f) v=(%.3f,%.3f,%.3f)"
                % tuple(p + v),
                flush=True,
            )
