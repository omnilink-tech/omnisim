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

"""Per-step braking probe. Only wheel motors actuate; no pose or velocity writes.

This standalone deterministic assessment is independent of OmniLink. Its test
fixture requests a stop at a measured stable speed. The intentionally defective
controller variant delays applying that request; the offline oracle never reads
the expected label or uses the configured delay to decide the outcome.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import runpy
import traceback

ROOT = Path(__file__).resolve().parents[4]
from omnisim import Supervisor


def main():
    out = Path(os.environ["OMNISIM_ASSESSMENT_OUT"])
    # The controller API owns the `omnisim` module in this interpreter; load
    # the pure schema by file without shadowing it with the CLI package.
    schema = runpy.run_path(str(ROOT / "omnisim/assessment/schema.py"))
    spec = schema["BrakingSpec"].from_dict(json.loads((out / "spec.json").read_text()))
    cfgmod = runpy.run_path(str(ROOT / "projects/samples/demos/controllers/omnilink_mobile_bridge/_mobile_configs.py"))
    cfg = cfgmod["get_config"](spec.robot)
    names = cfgmod["WHEEL_MOTORS"][cfg["layout"]]
    sim = Supervisor()
    node = sim.getSelf()
    motors = [sim.getDevice(n) for side in ("left", "right") for n in names[side]]
    sensors = []
    for motor in motors:
        if motor is None:
            raise RuntimeError(f"{spec.robot} wheel motor unavailable")
        motor.setPosition(float("inf"))
        motor.setVelocity(0)
        sensor = motor.getPositionSensor()
        if sensor is None:
            raise RuntimeError(f"{spec.robot} wheel position sensor unavailable")
        sensor.enable(spec.timestep_ms)
        sensors.append(sensor)
    requested = applied = None
    stable_since = None
    start_x = None
    complete = False
    captured = []
    capture = os.environ.get("OMNISIM_ASSESSMENT_CAPTURE") == "1"
    sequence = 0
    last_t = 0.0
    terminal = "controller_interrupted"
    theta = math.radians(spec.slope_deg)
    sn, cs = math.sin(theta), math.cos(theta)
    with (out / "trace.jsonl").open("w", encoding="utf-8", buffering=1) as trace, (out / "commands.jsonl").open("w", encoding="utf-8", buffering=1) as commands:
        while sim.step(spec.timestep_ms) != -1:
            t = float(sim.getTime())
            p, v, orientation = node.getPosition(), node.getVelocity(), node.getOrientation()
            along = p[0]*cs - p[2]*sn
            forward = v[0]*cs - v[2]*sn
            if start_x is None:
                start_x = along
            driving = t >= spec.settle_s
            speed_ok = 0.9 * spec.target_speed_m_s <= forward <= 1.15 * spec.target_speed_m_s
            if driving and speed_ok:
                stable_since = t if stable_since is None else stable_since
            else:
                stable_since = None
            new_request = (requested is None and stable_since is not None and
                           t - stable_since >= spec.speed_hold_s and along - start_x >= spec.min_drive_distance_m)
            if new_request:
                requested = t
                commands.write(json.dumps({"event": "stop_requested", "t_s": t, "seq": sequence}) + "\n")
            if requested is not None and applied is None and t - requested + 1e-9 >= spec.brake_delay_s:
                applied = t
                commands.write(json.dumps({"event": "brake_applied", "t_s": t, "seq": sequence}) + "\n")
            wheel_command = spec.target_speed_m_s / cfg["wheel_radius_m"] if driving and applied is None else 0.0
            for motor in motors:
                motor.setVelocity(wheel_command)
            row = {"seq": sequence, "t_s": t, "x_m": p[0], "y_m": p[1], "z_m": p[2],
                   "vx_m_s": v[0], "vy_m_s": v[1], "vz_m_s": v[2],
                   "tilt_rad": math.acos(max(-1.0, min(1.0, orientation[8]))),
                   "wheel_position_rad": [s.getValue() for s in sensors],
                   "contacts": len(node.getContactPoints(True)),
                   "stop_requested": requested is not None, "brake_applied": applied is not None,
                   "wheel_command_rad_s": wheel_command}
            row.update(along_m=along, forward_m_s=forward, normal_height_m=p[0]*sn+p[2]*cs,
                       ground_tilt_rad=math.acos(max(-1.0, min(1.0, orientation[2]*sn+orientation[8]*cs))))
            trace.write(json.dumps(row, allow_nan=False) + "\n")
            sequence += 1
            last_t = t
            if capture and new_request:
                sim.setLabel(0, f"OmniSim | {spec.robot} braking assessment", 0.02, 0.02, 0.065, 0xFFFFFF, 0)
                sim.setLabel(1, "STOP REQUESTED | delay %.2f s" % spec.brake_delay_s, 0.02, 0.09, 0.05, 0xFFCC55, 0)
                sim.exportImage(str(out / "stop-request.jpg"), 90)
                captured.append("stop-request.jpg")
            if requested is not None and t - requested >= spec.observe_after_request_s:
                complete = True
                terminal = "observation_complete"
                if capture:
                    sim.setLabel(1, "FINAL POSITION | delay %.2f s" % spec.brake_delay_s, 0.02, 0.09, 0.05, 0xFFCC55, 0)
                    sim.exportImage(str(out / "stopped.jpg"), 90)
                    captured.append("stopped.jpg")
                break
            if requested is None and t > spec.prepare_timeout_s:
                complete = spec.schema_version >= 2
                terminal = "approach_timeout"
                break
            if spec.schema_version >= 2 and (abs(along-start_x) > 10 or abs(p[1]) > 1 or row["ground_tilt_rad"] > 1.1):
                complete = True
                terminal = "fixture_departure_or_tip"
                break
    for motor in motors:
        motor.setVelocity(0)
    completion = {"complete": complete, "samples": sequence, "last_t_s": last_t,
                  "request_t_s": requested, "applied_t_s": applied,
                  "motion_mode": "physical_wheel_velocity", "capture_files": captured,
                  "terminal": terminal, "wheel_count": len(motors),
                  "wheel_radius_m": cfg["wheel_radius_m"], "controller": str(Path(__file__).relative_to(ROOT))}
    (out / "completion.json").write_text(json.dumps(completion, indent=2), encoding="utf-8")
    print("[assessment] complete=%s samples=%d" % (complete, sequence), flush=True)
    sim.simulationQuit(0)


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        # Preserve failures even when the controller cannot ask the engine to quit.
        path = os.environ.get("OMNISIM_ASSESSMENT_OUT")
        if path:
            (Path(path) / "controller-error.txt").write_text(traceback.format_exc(), encoding="utf-8")
        raise
