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

"""Robot-agnostic deterministic physics probe -- DEPLOY side (runs in omnisim-bin).

Generalizes the g1_parity_probe controller to ANY robot via a parity-probe spec
(``projects/policies/research/backends/parity_probe_spec.py``). Applies the SAME scripted,
no-RL targets the trainer harness (``robot_parity_probe.py``) applies and dumps
the SAME per-tick trace schema. ``g1_parity_compare.py`` diffs the two.

The robot is selected by ``controllerArgs ["--robot" "<name>"]`` in the .wbt (or
the ``PROBE_ROBOT`` env). Trace -> ``PROBE_TRACE`` (default _scratch/parity).
``PROBE_TICKS`` / ``PROBE_SETTLE`` / ``PROBE_SEQ`` / ``PROBE_RECORD_SETTLE`` tune it.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

from projects.policies.research.backends import parity_probe_spec as PP  # noqa: E402

try:
    from omnisim import Supervisor as _Robot
except Exception:  # pragma: no cover
    from omnisim import Robot as _Robot

TRACE_SCHEMA = 1


def _say(msg: str) -> None:
    sys.stderr.write(msg)
    sys.stderr.flush()


def _arg(flag, default=None):
    a = sys.argv
    if flag in a:
        i = a.index(flag)
        if i + 1 < len(a):
            return a[i + 1]
    return default


def main() -> int:
    robot = _arg("--robot", os.environ.get("PROBE_ROBOT"))
    if not robot:
        _say("[parity_probe] no --robot given\n")
        return 1
    spec = PP.load(robot)
    out_path = Path(os.environ.get(
        "PROBE_TRACE", str(_REPO / f"_scratch/parity/{robot}_deploy.json")))
    n_ticks = int(os.environ.get("PROBE_TICKS", spec.duration_ticks))
    settle = int(os.environ.get("PROBE_SETTLE", spec.settle_ticks))
    seq = os.environ.get("PROBE_SEQ", "sinusoid")
    record_settle = os.environ.get("PROBE_RECORD_SETTLE", "0").strip() != "0"

    robot_node = _Robot()
    step_ms = int(robot_node.getBasicTimeStep())
    motors = []
    sensors = []
    for jn in spec.joint_order:
        m = robot_node.getDevice(f"{jn}_motor")
        if m is None:
            _say(f"[parity_probe:{robot}] missing motor {jn}_motor\n")
            return 1
        motors.append(m)
        s = None
        try:
            s = m.getPositionSensor()
            if s is not None:
                s.enable(step_ms)
        except Exception:
            s = None
        sensors.append(s)
    self_node = None
    try:
        self_node = robot_node.getSelf()
    except Exception:
        self_node = None

    _say(f"[parity_probe:{robot}] start njoints={len(spec.joint_order)} "
         f"ticks={n_ticks} settle={settle} seq={seq} ke={spec.ke} kd={spec.kd}\n")

    ticks = []

    def _capture(k, target, phase):
        q = [float(s.getValue()) if s is not None else float("nan") for s in sensors]
        if self_node is not None:
            base_pos = [float(x) for x in self_node.getPosition()]
            base_rot = [float(x) for x in self_node.getOrientation()]
        else:
            base_pos = [float("nan")] * 3
            base_rot = [float("nan")] * 9
        ticks.append({"k": k, "phase": phase,
                      "target": [float(x) for x in target],
                      "q": q, "base_pos": base_pos, "base_rot": base_rot})

    for s in range(settle):
        st = spec.settle_target(s)
        for j in range(len(motors)):
            motors[j].setPosition(float(st[j]))
        if robot_node.step(step_ms) == -1:
            break
        if record_settle:
            _capture(s - settle, st, "settle")
    for k in range(n_ticks):
        tk = spec.targets(k, sequence=seq)
        for j in range(len(motors)):
            motors[j].setPosition(float(tk[j]))
        if robot_node.step(step_ms) == -1:
            break
        _capture(k, tk, "probe")

    out = {
        "schema": TRACE_SCHEMA, "side": "deploy",
        "meta": {
            "robot": robot, "construction": "add_link (C++ WbNewtonBackend)",
            "urdf": spec.urdf.name, "sequence": seq, "settle_ticks": settle,
            "substeps": spec.substeps, "dt": step_ms / 1000.0,
            "njoints": len(spec.joint_order), "joint_order": spec.joint_order,
            "use_link_com": os.environ.get("OMNISIM_NEWTON_USE_LINK_COM", "unset"),
            "target_ke": os.environ.get("OMNISIM_NEWTON_TARGET_KE", "unset"),
        },
        "ticks": ticks,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out), encoding="utf-8")
    _say(f"[parity_probe:{robot}] wrote {out_path} ticks={len(ticks)} "
         f"final_base_z={ticks[-1]['base_pos'][2] if ticks else float('nan'):.4f}\n")
    try:
        robot_node.simulationQuit(0)
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
