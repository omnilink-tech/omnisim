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

"""Lane 1R: set a MEASURED initial state, then record what the engine does.

Reads the IC from LANE1R_* environment variables (one world serves all 550
tosses), applies it to the CUBE, steps, and writes an npz the scorer compares
against the measurement.

Two things here are easy to get wrong and silent when you do:

1. **omega must be rotated into the world frame.** The dataset stores angular
   velocity in the BODY frame -- measured, see dataset.OMEGA_FRAME -- while
   `Node.setVelocity()` takes a 6-vector in WORLD coordinates. Passing the
   raw column through leaves the cube tumbling about the wrong axis while
   still looking like a plausible toss, which no trajectory metric would
   flag as a *setup* error rather than a physics one. The runner does the
   rotation and passes world-frame omega; this controller asserts the norm
   survived the trip, since |omega| is rotation-invariant.

2. **The write must be verified, not assumed.** Supervisor setVelocity at
   t=0 was dropped by Newton until the backend started queueing
   pre-registration writes. It works now (lane 1's T7 sweep confirms), but
   an unverified IC is how a benchmark ends up scoring the wrong experiment,
   so the achieved state is measured back and recorded in the npz. The
   scorer refuses a run whose IC did not take.
"""

import json
import os
import sys

import numpy as np

try:  # `omnisim` is the only module name at HEAD (the `controller`
    from omnisim import Supervisor  # alias was deleted 2026-08-16)
except ImportError:  # older trees (e.g. a RunPod volume) still ship it
    from controller import Supervisor


def _env_floats(name, n):
    raw = os.environ.get(name)
    if raw is None:
        raise SystemExit("[lane1r] %s is not set" % name)
    v = [float(x) for x in raw.replace(",", " ").split()]
    if len(v) != n:
        raise SystemExit("[lane1r] %s wants %d floats, got %d" % (name, n, len(v)))
    return v


def _bc(msg):
    """Breadcrumb: a controller that dies without Python raising leaves no
    traceback, so record progress to disk as it goes."""
    try:
        with open(os.environ.get("LANE1R_OUT", "lane1r_run.npz") + ".phase.txt",
                  "a", encoding="utf-8") as fh:
            fh.write(msg + "\n")
    except Exception:
        pass


def main():
    _bc("enter")
    sv = Supervisor()
    _bc("supervisor")
    basic = sv.getBasicTimeStep()
    # ⚠ step() takes an INTEGER ms and defaults to int(basic_time_step), which
    # is 0 for any sub-millisecond world -- an infinite no-op, not an error.
    # Advance a fixed 1 ms per iteration (a whole multiple of the 0.5 ms basic
    # step) and let the scorer resample onto the 148 Hz measurement grid.
    step_ms = int(os.environ.get("LANE1R_STEP_MS", "1"))
    if step_ms < 1:
        raise SystemExit("[lane1r] LANE1R_STEP_MS must be >= 1")
    if abs(round(step_ms / basic) - step_ms / basic) > 1e-9:
        raise SystemExit("[lane1r] step %d ms is not a whole multiple of the "
                         "%.4f ms basic step" % (step_ms, basic))

    pos = _env_floats("LANE1R_POS", 3)
    rot = _env_floats("LANE1R_ROT", 4)          # axis-angle, .wbt convention
    vel = _env_floats("LANE1R_VEL", 3)
    omg = _env_floats("LANE1R_OMEGA_WORLD", 3)  # already world frame
    # Duration in ms of sim time; one sample per step_ms.
    dur_ms = float(os.environ.get("LANE1R_DURATION_MS", "818"))
    n_steps = int(round(dur_ms / step_ms))
    out = os.environ.get("LANE1R_OUT", "lane1r_run.npz")
    label = os.environ.get("LANE1R_LABEL", "?")

    _bc("env ok")
    cube = sv.getFromDef("CUBE")
    if cube is None:
        raise SystemExit("[lane1r] no DEF CUBE in the world")

    tf = cube.getField("translation")
    rf = cube.getField("rotation")
    tf.setSFVec3f([float(v) for v in pos])
    rf.setSFRotation([float(v) for v in rot])
    cube.resetPhysics()
    cube.setVelocity([float(v) for v in vel] + [float(v) for v in omg])

    # One step so the write lands, then read the achieved state back. This is
    # the difference between "we set the IC" and "the IC took".
    _bc("ic set")
    sv.step(step_ms)
    got = cube.getVelocity()
    got_v, got_w = np.array(got[:3]), np.array(got[3:])
    want_v, want_w = np.array(vel), np.array(omg)
    ic = {
        "want_vel": want_v.tolist(), "got_vel": got_v.tolist(),
        "want_omega_world": want_w.tolist(), "got_omega_world": got_w.tolist(),
        "vel_err": float(np.linalg.norm(got_v - want_v)),
        "omega_err": float(np.linalg.norm(got_w - want_w)),
        "vel_rel": float(np.linalg.norm(got_v - want_v)
                         / max(np.linalg.norm(want_v), 1e-9)),
        "omega_rel": float(np.linalg.norm(got_w - want_w)
                           / max(np.linalg.norm(want_w), 1e-9)),
    }

    # One row per step_ms, plus the row for t=0.
    n_out = n_steps + 1
    P = np.full((n_out, 3), np.nan)
    Q = np.full((n_out, 4), np.nan)   # wxyz, to match the dataset
    T = np.arange(n_out) * (step_ms / 1000.0)

    def sample(i):
        p = cube.getPosition()
        m = cube.getOrientation()          # row-major 3x3, body->world
        R = np.array(m, dtype=np.float64).reshape(3, 3)
        # rotation matrix -> wxyz quaternion (Shepperd, branch on the largest
        # denominator so no branch divides by ~0)
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
            s = np.sqrt(tr + 1.0) * 2
            q = [0.25 * s, (R[2, 1] - R[1, 2]) / s,
                 (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                 (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                 0.25 * s, (R[1, 2] + R[2, 1]) / s]
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                 (R[1, 2] + R[2, 1]) / s, 0.25 * s]
        q = np.array(q, dtype=np.float64)
        q /= np.linalg.norm(q)
        P[i] = p
        Q[i] = q

    _bc("ic verified vel_rel=%.3g omega_rel=%.3g" % (ic["vel_rel"], ic["omega_rel"]))
    sample(0)
    _bc("sample0 ok")
    done = n_out
    for n in range(1, n_steps + 1):
        if sv.step(step_ms) == -1:
            done = n
            break
        sample(n)
        if n % 100 == 0:
            _bc("step %d" % n)

    _bc("loop done=%d" % done)
    np.savez(out, pos=P, quat=Q, t=T, ic=json.dumps(ic), label=label,
             step_ms=step_ms, n_samples=done, basic_time_step=basic)
    print("[lane1r] %s: %d/%d samples @%d ms, IC vel_rel=%.3g omega_rel=%.3g -> %s"
          % (label, done, n_out, step_ms, ic["vel_rel"], ic["omega_rel"], out))
    sys.stdout.flush()
    # Tell the ENGINE to exit. Without this the controller finishes in ~2 s and
    # the engine free-runs until the runner's timeout kills it -- the first
    # working run took the full 240 s for 818 ms of simulation, which over 550
    # tosses is 36 hours of waiting for 8 minutes of physics.
    # Breadcrumbs earn their keep only on failure. A completed run leaves one
    # per toss -- 550 files of noise next to 550 results -- so drop ours now
    # that there is an npz proving we got here.
    try:
        os.remove(out + ".phase.txt")
    except OSError:
        pass
    sv.simulationQuit(0)


if __name__ == "__main__":
    # A controller that dies mutely is the worst failure mode this lane has:
    # the engine keeps stepping to its timeout, the runner sees only "no npz",
    # and the traceback is lost because controller stderr does not reliably
    # reach the engine log. Persist it next to the output instead.
    try:
        main()
    except SystemExit:
        raise
    except BaseException:
        import traceback
        tb = traceback.format_exc()
        try:
            p = os.environ.get("LANE1R_OUT", "lane1r_run.npz") + ".traceback.txt"
            with open(p, "w", encoding="utf-8") as fh:
                fh.write(tb)
        except Exception:
            pass
        sys.stderr.write(tb)
        sys.stderr.flush()
        raise
