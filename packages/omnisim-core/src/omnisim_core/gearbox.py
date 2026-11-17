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

"""The Newton runtime's gearbox joint, on the core.

The same law, in the same float64 operations, as `World.gearbox_attach`, `_gearbox_elastic` and
`_gearbox_advance` in src/omnisim/physics/omnisim_newton_runtime.py (tests/test_core_gearbox.py runs
both on the same inputs and compares the results to the bit). What the engine does around it, the
core does in `Core.step`: the coupling torque is computed before every physics sub-step from the
float32 joint state the engine's solver hands the runtime, and reaches MuJoCo as a float32
`qfrc_applied`.

DECLARING ONE is unchanged for a controller: gears/runtime_gearbox.py appends a JSON line to the
channel file and sends the token as the motor's first setTorque. The engine names the channel after
its pid (the controller's OMNISIM_IPC_NONCE); a core run sets OMNISIM_NEWTON_GEARBOX_CMD instead,
which runtime_gearbox.py and the runtime both honour. OMNISIM_NEWTON_GEARBOX=0 ignores every
declaration here as it does in the engine.
"""

from __future__ import annotations

import json
import math
import os


def channel_path():
    """The declaration channel a core run reads, or None when none is set."""
    return os.environ.get("OMNISIM_NEWTON_GEARBOX_CMD") or None


def disabled():
    return (os.environ.get("OMNISIM_NEWTON_GEARBOX") or "").strip().lower() in ("0", "false", "off", "no")


def find_declaration(tau):
    """The declaration whose token is `tau` in the channel (the last one wins, as in the runtime), or None."""
    p = channel_path()
    if not p or not os.path.exists(p):
        return None
    decl = None
    with open(p, "r", encoding="utf-8") as fh:
        for ln in fh:
            ln = ln.strip()
            if not ln:
                continue
            try:
                d = json.loads(ln)
            except ValueError:
                continue
            if d.get("op") == "declare" and float(d.get("token", -1.0)) == tau:
                decl = d
    return decl


def write_ack(ack):
    p = channel_path()
    if not p:
        return
    try:
        with open(p + ".ack", "a", encoding="utf-8") as af:
            af.write(json.dumps(ack) + "\n")
    except OSError:
        pass


def prepare(spec, q_idx, dof_idx):
    """The runtime's gearbox_attach without the Newton bookkeeping. -> (g, ack) or (None, {"error": ...})."""
    N = float(spec["ratio"])
    T = [float(v) for v in spec["twist_T_Nm"]]
    tw = [float(v) for v in spec["twist_rad"]]
    if len(T) < 2 or len(T) != len(tw) or N <= 0:
        return None, {"error": "spec needs ratio > 0 and a torque-twist curve of >= 2 points"}
    ks_tw = [(T[i] - T[i - 1]) / (tw[i] - tw[i - 1]) for i in range(1, len(T)) if tw[i] - tw[i - 1] > 1e-9]
    if not ks_tw:
        return None, {"error": "the torque-twist curve has no twist"}
    k_ref = ks_tw[0]
    tw2 = [tw[0]]
    for i in range(1, len(T)):
        dtw = tw[i] - tw2[-1]
        if dtw <= 1e-9:
            later = [(T[j] - T[j - 1]) / (tw[j] - tw[j - 1]) for j in range(i + 1, len(T))
                     if tw[j] - tw[j - 1] > 1e-9]
            k_use = later[0] if later else k_ref
            tw2.append(tw2[-1] + max(T[i] - T[i - 1], 0.0) / k_use)
        else:
            tw2.append(tw[i])
    tw = tw2
    ks = [(T[i] - T[i - 1]) / max(tw[i] - tw[i - 1], 1e-12) for i in range(1, len(T))]
    k_max = max(ks)
    J = max(float(spec.get("J_rotor_kgm2", 0.0)), 1e-9)
    k50 = float(spec.get("k_50_Nm_per_rad") or ks[len(ks) // 2])
    zeta = float(spec.get("damping_ratio", 0.05))
    g = {
        "N": N, "T": T, "tw": tw, "k_last": ks[-1], "half": max(float(spec.get("backlash_rad", 0.0)), 0.0) / 2,
        "ef": float(spec.get("efficiency_forward", 1.0)), "eb": float(spec.get("efficiency_back", 1.0)),
        "J": J, "c": 2.0 * zeta * math.sqrt(k50 * J * N * N), "wb": float(spec.get("omega_blend", 1e-3)),
        "locked": bool(spec.get("input_locked", False)),
        "q_idx": int(q_idx), "dof_idx": int(dof_idx),
        "T_motor": 0.0, "th_m": None, "w_m": 0.0,
        "w_rot": math.sqrt(k_max / (J * N * N)),
        "name": str(spec.get("name", "")),
    }
    return g, {"attached": True, "ratio": N, "backlash_rad": 2 * g["half"], "k_max_Nm_per_rad": k_max,
               "rotor_omega_rad_s": g["w_rot"], "name": g["name"]}


def elastic(g, x):
    T, tw = g["T"], g["tw"]
    if x >= tw[-1]:
        return T[-1] + g["k_last"] * (x - tw[-1])
    for i in range(1, len(tw)):
        if x <= tw[i]:
            f = (x - tw[i - 1]) / max(tw[i] - tw[i - 1], 1e-15)
            return T[i - 1] + f * (T[i] - T[i - 1])
    return T[-1]


def advance(g, dt, th_out, w_out):
    """Advance one gearbox over dt with the output at (th_out, w_out); the mean torque on the output (N m)."""
    N = g["N"]
    if g["th_m"] is None:
        g["th_m"] = th_out * N
    n = min(max(1, int(math.ceil(g["w_rot"] * dt / 0.25))), 200)
    h = dt / n
    acc = 0.0
    s = math.tanh(abs(w_out) / g["wb"])
    for i in range(n):
        tho = th_out + w_out * (i + 0.5) * h
        d = g["th_m"] / N - tho
        if d > g["half"]:
            Ts = elastic(g, d - g["half"]) + g["c"] * (g["w_m"] / N - w_out)
        elif d < -g["half"]:
            Ts = -elastic(g, -d - g["half"]) + g["c"] * (g["w_m"] / N - w_out)
        else:
            Ts = 0.0
        if Ts * w_out >= 0.0:
            out, react = Ts * (1.0 - s * (1.0 - g["ef"])), Ts
        else:
            out, react = Ts, Ts * (1.0 - s * (1.0 - max(g["eb"], 0.0)))
        if not g["locked"]:
            g["w_m"] += h * (g["T_motor"] - react / N) / g["J"]
            g["th_m"] += h * g["w_m"]
        acc += out
    return acc / n
