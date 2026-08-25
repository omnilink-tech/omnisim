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

"""omnibench_recorder — OmniBench lane-1 trajectory recorder (supervisor).

Generic supervisor controller for the generated lane-1 worlds
(tests/benchmarks/omnibench/lane1/gen_worlds.py). Once per basic step it
records, for every tracked DEF'd body:

    world position           (supervisor getPosition)
    world lin/ang velocity   (supervisor getVelocity -> [vx vy vz wx wy wz])
    world orientation        (supervisor getOrientation 3x3 -> quaternion wxyz)

plus joint angles for DEF'd HingeJointParameters nodes (their `position`
field, read through the supervisor field API — works identically for passive
joints under both backends and is valid at t=0, unlike a PositionSensor).

At the end it writes a single .npz to $OMNIBENCH_OUT in the layout shared by
every OmniBench engine runner (see SPEC.md "Recording format" and
common/recording.py):

    t             float64 [N]      sim time, seconds (row 0 is t=0, pre-step)
    pos_<name>    float64 [N,3]
    vel_<name>    float64 [N,3]
    quat_<name>   float64 [N,4]    wxyz
    omega_<name>  float64 [N,3]
    joint_<name>  float64 [N]      (articulated scenes only; extra keys)

and a sidecar JSON `$OMNIBENCH_OUT.meta.json` with steps, dt_ms, sim seconds
and wall-clock ms/step (timed around the step() calls only), then quits the
simulation cleanly (supervisor simulationQuit(0)).

Controller args (baked into the world by gen_worlds.py):
    --bodies=DEF1,DEF2      required, tracked bodies
    --duration=S            required, recorded sim seconds
    --label=stem            scene stem, echoed into the meta sidecar
    --joints=J1P,J2P        DEF names of HingeJointParameters nodes
    --couple=PARENT:CHILD:AMP:HZ:TEND
                            T5's actuation phase: internal hinge torque
                            tau = AMP*sin(2*pi*HZ*t) N*m while t<TEND applied
                            as an equal-and-opposite supervisor addTorque pair
                            about world y (+tau on CHILD, -tau on PARENT).
                            Exact for the planar xz chain (its hinge axis
                            stays world-y). Deliberately NOT Motor.setTorque:
                            the Newton backend gives motorized hinges a
                            hardcoded position servo that pins the joint and
                            pumps linear momentum (~17 kg*m/s measured in the
                            gravity-off chain); addTorque plumbs to both
                            backends and keeps the joints passive.
    --set-omega=DEF:wx,wy,wz     initial angular velocity — tries supervisor
                            setVelocity; if it does not take (Newton bodies
                            ignore it, measured), falls back to a closed-loop
                            torque-impulse spin-up (needs --spin-inertia)
    --spin-inertia=Ixx,Iyy,Izz   world-aligned diagonal inertia used by the
                            spin-up fallback. Recording starts AFTER spin-up;
                            achieved omega + spin-up step count land in the
                            meta sidecar.
"""

import json
import math
import os
import sys
import time

import numpy as np

try:  # `omnisim` is the only module name at HEAD (the `controller`
    from omnisim import Supervisor  # alias was deleted 2026-08-16)
except ImportError:  # older trees (e.g. a RunPod volume) still ship it
    from controller import Supervisor


def parse_args(argv):
    out = {}
    for a in argv:
        if a.startswith("--") and "=" in a:
            k, v = a[2:].split("=", 1)
            out[k] = v
    return out


def quat_wxyz(m):
    """3x3 row-major rotation matrix (len-9 list) -> unit quaternion (w,x,y,z)."""
    r00, r01, r02, r10, r11, r12, r20, r21, r22 = m
    tr = r00 + r11 + r22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (r21 - r12) / s
        y = (r02 - r20) / s
        z = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        w = (r21 - r12) / s
        x = 0.25 * s
        y = (r01 + r10) / s
        z = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        w = (r02 - r20) / s
        x = (r01 + r10) / s
        y = 0.25 * s
        z = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        w = (r10 - r01) / s
        x = (r02 + r20) / s
        y = (r12 + r21) / s
        z = 0.25 * s
    return (w, x, y, z)


def main():
    args = parse_args(sys.argv[1:])
    sv = Supervisor()

    out_path = os.environ.get("OMNIBENCH_OUT", "omnibench_out.npz")
    body_defs = [b for b in args.get("bodies", "").split(",") if b]
    joint_defs = [j for j in args.get("joints", "").split(",") if j]
    duration = float(args.get("duration", "5"))
    label = args.get("label", "")

    dt_ms = sv.getBasicTimeStep()  # SFFloat; the sweep uses integers
    step_ms = int(round(dt_ms))
    n_steps = int(round(duration * 1000.0 / dt_ms))

    nodes = []
    for d in body_defs:
        n = sv.getFromDef(d)
        if n is None:
            print("[omnibench_recorder] FATAL: DEF '%s' not found" % d, flush=True)
            sv.simulationQuit(1)
            sv.step(step_ms)
            return
        nodes.append((d, n))

    joint_fields = []
    for d in joint_defs:
        n = sv.getFromDef(d)
        f = n.getField("position") if n is not None else None
        if f is None:
            print("[omnibench_recorder] FATAL: joint DEF '%s' has no position field" % d,
                  flush=True)
            sv.simulationQuit(1)
            sv.step(step_ms)
            return
        joint_fields.append((d, f))

    # T5: sinusoidal internal-couple actuation (see module docstring).
    couple = None  # (parent_node, child_node, amp, hz, t_end)
    if "couple" in args:
        pn, cn, amp, hz, tend = args["couple"].split(":")
        pnode, cnode = sv.getFromDef(pn), sv.getFromDef(cn)
        if pnode is None or cnode is None:
            print("[omnibench_recorder] FATAL: couple DEFs '%s'/'%s' not found"
                  % (pn, cn), flush=True)
            sv.simulationQuit(1)
            sv.step(step_ms)
            return
        couple = (pnode, cnode, float(amp), float(hz), float(tend))

    # T7: initial angular velocity (a .wbt cannot express one).
    spin_meta = None
    if "set-omega" in args:
        d, w = args["set-omega"].split(":")
        w_target = np.array([float(v) for v in w.split(",")])
        node = sv.getFromDef(d)
        node.setVelocity([0.0, 0.0, 0.0] + list(w_target))
        # Verify the write took; if not, closed-loop torque-impulse spin-up.
        #
        # HISTORY, because the old note here has misled at least one reader
        # into planning around a workaround that is no longer needed: this
        # used to say "Newton bodies ignore supervisor setVelocity — measured
        # omega stays ~0". That WAS true (a t=0 write arrived before body
        # registration and was dropped) and is why the fallback exists. It was
        # fixed in the backend by queueing pre-registration velocity writes and
        # draining them after finalize.
        #
        # Measured 2026-08-09 on the full T7 dt sweep, machine 9722d23d12a3:
        # all six runs report method "setVelocity" with spinup_steps 1 and
        # achieved omega_y 5.0000000 against a target of 5.0. The fallback did
        # not fire once. It is kept anyway -- it is the thing that PROVES the
        # write took rather than assuming it, and every run records which path
        # it used in the .meta.json "spin" block.
        sv.step(step_ms)
        w_now = np.array(node.getVelocity()[3:])
        spin_steps = 1
        method = "setVelocity"
        if np.linalg.norm(w_now - w_target) > 0.5 * np.linalg.norm(w_target):
            method = "torque_impulse"
            inertia = np.array([float(v) for v in
                                args.get("spin-inertia", "1,1,1").split(",")])
            for _ in range(40):
                dw = w_target - w_now
                if np.linalg.norm(dw) <= 0.01 * np.linalg.norm(w_target):
                    break
                tau = inertia * dw / (dt_ms / 1000.0)
                node.addTorque(list(tau), False)
                sv.step(step_ms)
                spin_steps += 1
                w_now = np.array(node.getVelocity()[3:])
        spin_meta = {
            "method": method,
            "target_omega": list(w_target),
            "achieved_omega": [float(v) for v in w_now],
            "spinup_steps": spin_steps,
        }
        print("[omnibench_recorder] initial omega of %s: target %s achieved %s "
              "via %s in %d steps"
              % (d, list(w_target), [round(float(v), 4) for v in w_now],
                 method, spin_steps), flush=True)

    N = n_steps + 1  # row 0 is the pre-step state at t=0
    t_arr = np.zeros(N)
    pos = {d: np.zeros((N, 3)) for d, _ in nodes}
    vel = {d: np.zeros((N, 3)) for d, _ in nodes}
    quat = {d: np.zeros((N, 4)) for d, _ in nodes}
    omega = {d: np.zeros((N, 3)) for d, _ in nodes}
    jang = {d: np.zeros(N) for d, _ in joint_fields}

    def record(row, t):
        t_arr[row] = t
        for d, n in nodes:
            p = n.getPosition()
            v = n.getVelocity()
            pos[d][row] = p
            vel[d][row] = v[:3]
            omega[d][row] = v[3:]
            quat[d][row] = quat_wxyz(n.getOrientation())
        for d, f in joint_fields:
            jang[d][row] = f.getSFFloat()

    record(0, 0.0)
    wall = 0.0
    rows = 1
    for k in range(n_steps):
        t = k * dt_ms / 1000.0  # couple uses the time at the START of the step
        if couple is not None and t < couple[4]:
            tau = couple[2] * math.sin(2.0 * math.pi * couple[3] * t)
            # equal-and-opposite pure couples about world y: net external
            # force AND torque are zero, exactly like an internal joint motor
            couple[0].addTorque([0.0, -tau, 0.0], False)
            couple[1].addTorque([0.0, tau, 0.0], False)
        w0 = time.perf_counter()
        rc = sv.step(step_ms)
        wall += time.perf_counter() - w0
        if rc == -1:
            print("[omnibench_recorder] step() returned -1 at step %d/%d"
                  % (k + 1, n_steps), flush=True)
            break
        record(rows, (k + 1) * dt_ms / 1000.0)
        rows += 1

    arrays = {"t": t_arr[:rows]}
    for d, _ in nodes:
        arrays["pos_%s" % d] = pos[d][:rows]
        arrays["vel_%s" % d] = vel[d][:rows]
        arrays["quat_%s" % d] = quat[d][:rows]
        arrays["omega_%s" % d] = omega[d][:rows]
    for d, _ in joint_fields:
        arrays["joint_%s" % d] = jang[d][:rows]

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    np.savez(out_path, **arrays)

    steps_done = rows - 1
    meta = {
        "label": label,
        "world": sv.getWorldPath(),
        "dt_ms": dt_ms,
        "steps": steps_done,
        "sim_seconds": steps_done * dt_ms / 1000.0,
        "wall_ms_per_step": (wall * 1000.0 / steps_done) if steps_done else None,
        "bodies": [d for d, _ in nodes],
        "joints": [d for d, _ in joint_fields],
    }
    if spin_meta is not None:
        meta["spin"] = spin_meta
    with open(out_path + ".meta.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)

    print("[omnibench_recorder] wrote %d rows (%.3f sim s, %.3f wall ms/step) -> %s"
          % (rows, meta["sim_seconds"], meta["wall_ms_per_step"] or -1, out_path),
          flush=True)
    sv.simulationQuit(0)
    sv.step(step_ms)


main()
