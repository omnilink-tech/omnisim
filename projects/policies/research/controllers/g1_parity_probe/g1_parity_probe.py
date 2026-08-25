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

"""Deterministic open-loop physics probe -- DEPLOY side (runs inside omnisim-bin).

Applies the SAME scripted targets as the trainer harness
(``projects/policies/research/training/g1_parity_probe.py``) -- ``SPEC.probe_targets(k)`` each
control step -- and dumps the SAME per-tick trace schema (joint angles + base
pose). ``g1_parity_compare.py`` then diffs the trainer trace against this deploy
trace. This is the FIRST parity check that steps the REAL binary (the C++ Newton
backend building the model via add_link from the .wbt scene graph), not the
``g1_deploy_runtime.py`` Python extract.

NO RL, NO policy: a deterministic open-loop controlled experiment so any
divergence from the trainer trace is pure model/stepping, not policy/durability.

Tick contract (IDENTICAL to the trainer): at control step k, command
``probe_targets(k)``, integrate ONE control step, then record the post-step
joint angles + base pose.

Env:
    G1_PROBE_TRACE   output path for the deploy trace JSON
                     (default _scratch/parity/deploy_trace.json)
    G1_PROBE_TICKS   override tick count (default SPEC.PROBE_DURATION_TICKS)
    G1_PROBE_SEQ     "hold" | "sinusoid" (default SPEC.PROBE_SEQUENCE)
    G1_PROBE_AMP / G1_PROBE_PERIOD  sinusoid knobs (default SPEC)
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.append(str(_REPO))

from projects.policies.research.backends import g1_physics_spec as SPEC  # noqa: E402

try:
    from omnisim import Supervisor as _Robot
except Exception:  # pragma: no cover - non-supervisor fallback
    from omnisim import Robot as _Robot

TRACE_SCHEMA = 1
LEGS_JOINTS = list(SPEC.LEGS_JOINTS)
NJ = len(LEGS_JOINTS)


def _say(msg: str) -> None:
    sys.stderr.write(msg)
    sys.stderr.flush()


def main() -> int:
    out_path = Path(os.environ.get(
        "G1_PROBE_TRACE", str(_REPO / "_scratch/parity/deploy_trace.json")))
    n_ticks = int(os.environ.get("G1_PROBE_TICKS", SPEC.PROBE_DURATION_TICKS))
    settle = int(os.environ.get("G1_PROBE_SETTLE", SPEC.PROBE_SETTLE_TICKS))
    seq = os.environ.get("G1_PROBE_SEQ", SPEC.PROBE_SEQUENCE)
    amp = float(os.environ.get("G1_PROBE_AMP", SPEC.PROBE_SINE_AMP))
    period = int(os.environ.get("G1_PROBE_PERIOD", SPEC.PROBE_SINE_PERIOD_TICKS))

    robot = _Robot()
    step_ms = int(robot.getBasicTimeStep())

    motors = []
    sensors = []
    for jn in LEGS_JOINTS:
        m = robot.getDevice(f"{jn}_motor")
        if m is None:
            _say(f"[g1_parity_probe] missing motor {jn}_motor\n")
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
        self_node = robot.getSelf()
    except Exception:
        self_node = None

    _say(f"[g1_parity_probe] start ticks={n_ticks} settle={settle} seq={seq} "
         f"amp={amp} period={period} step_ms={step_ms} njoints={NJ}\n")

    record_settle = os.environ.get("G1_PROBE_RECORD_SETTLE", "0").strip() != "0"
    ticks = []

    def _capture(k, target, phase):
        q = []
        for j in range(NJ):
            v = sensors[j].getValue() if sensors[j] is not None else float("nan")
            q.append(float(v))
        if self_node is not None:
            base_pos = [float(x) for x in self_node.getPosition()]
            base_rot = [float(x) for x in self_node.getOrientation()]
        else:
            base_pos = [float("nan")] * 3
            base_rot = [float("nan")] * 9
        ticks.append({"k": k, "phase": phase,
                      "target": [float(x) for x in target],
                      "q": q, "base_pos": base_pos, "base_rot": base_rot})

    # SETTLE: ramp from the straight-leg spawn to the pose, then hold, so the
    # deploy converges to the SAME position-PD equilibrium as the trainer BEFORE
    # recording -- removes the launch-IC asymmetry. Identical recipe + the SAME
    # SPEC.probe_settle_target() the trainer harness uses.
    for s in range(settle):
        st = SPEC.probe_settle_target(s, settle)
        for j in range(NJ):
            motors[j].setPosition(float(st[j]))
        if robot.step(step_ms) == -1:
            _say(f"[g1_parity_probe] sim ended during settle at s={s}\n")
            break
        if record_settle:
            _capture(s - settle, st, "settle")

    for k in range(n_ticks):
        tk = SPEC.probe_targets(k, sequence=seq, amp=amp, period_ticks=period)
        for j in range(NJ):
            motors[j].setPosition(float(tk[j]))
        if robot.step(step_ms) == -1:
            _say(f"[g1_parity_probe] sim ended early at k={k}\n")
            break
        _capture(k, tk, "probe")

    out = {
        "schema": TRACE_SCHEMA,
        "side": "deploy",
        "meta": {
            "construction": "add_link (C++ OmNewtonBackend from .wbt scene graph)",
            "urdf": "g1_legs_omnisim.urdf (prim feet)",
            "sequence": seq, "amp": amp, "period": period,
            "substeps": SPEC.SUBSTEPS, "dt": step_ms / 1000.0, "njoints": NJ,
            "joint_order": LEGS_JOINTS,
            "use_link_com": os.environ.get("OMNISIM_NEWTON_USE_LINK_COM", "unset"),
            "target_ke": os.environ.get("OMNISIM_NEWTON_TARGET_KE", "unset"),
            "target_kd": os.environ.get("OMNISIM_NEWTON_TARGET_KD", "unset"),
        },
        "ticks": ticks,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out), encoding="utf-8")
    qf = ticks[-1]["q"] if ticks else []
    _say(f"[g1_parity_probe] wrote {out_path}  ticks={len(ticks)} "
         f"final_q[knee]={qf[3] if len(qf) > 3 else float('nan'):+.4f}/"
         f"{qf[9] if len(qf) > 9 else float('nan'):+.4f} "
         f"final_base_z={ticks[-1]['base_pos'][2] if ticks else float('nan'):.4f}\n")

    # Clean headless exit so run-headless / the launcher returns promptly.
    try:
        robot.simulationQuit(0)
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
