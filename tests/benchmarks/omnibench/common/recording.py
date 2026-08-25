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

"""OmniBench trajectory-recording contract — shared by ALL engines.

One .npz file per (test, engine, dt) run. Keys:

    t             (S,)   float64  sim time of each sample, t[0] == 0.0
    pos_<name>    (S,3)  float64  body COM position, world frame  [m]
    vel_<name>    (S,3)  float64  body COM linear velocity, world [m/s]
    quat_<name>   (S,4)  float64  body orientation, **wxyz**, world<-body
    omega_<name>  (S,3)  float64  body angular velocity, world    [rad/s]

Contract details (OmniSim lane: follow these exactly):

* One sample per physics step, PLUS the initial state — S = steps + 1.
* ``pos``/``vel`` refer to the body's centre of mass, not a joint/frame
  origin (for the T4/T5 chain links that means the capsule centre).
* ``quat`` is world-from-body in scalar-first (w, x, y, z) order. MuJoCo's
  xquat is already wxyz; PyBullet/most engines are xyzw — reorder before
  writing.
* ``omega`` is the angular velocity expressed in the WORLD frame.
* Body names come from ``scenes.BODIES[test]`` — exact spelling, all bodies.
* The body's local frame (which ``quat`` maps to world) must be the
  principal-inertia frame assumed by scenes.py (e.g. chain-link capsule
  axis along local +x) — score.py reconstructs world-frame inertia as
  R diag(I) R^T from these quats.

Writer/reader below are the only supported access paths.
"""

import numpy as np

FIELDS = ("pos", "vel", "quat", "omega")
_WIDTH = {"pos": 3, "vel": 3, "quat": 4, "omega": 3}


def write_recording(path, t, bodies):
    """Write a recording .npz.

    t      : (S,) array-like of sample times.
    bodies : {name: {"pos": (S,3), "vel": (S,3), "quat": (S,4) wxyz,
                     "omega": (S,3)}}
    """
    t = np.asarray(t, dtype=np.float64)
    out = {"t": t}
    for name, data in bodies.items():
        for f in FIELDS:
            arr = np.asarray(data[f], dtype=np.float64)
            if arr.shape != (t.shape[0], _WIDTH[f]):
                raise ValueError("body %r field %r has shape %s, want (%d, %d)"
                                 % (name, f, arr.shape, t.shape[0], _WIDTH[f]))
            out["%s_%s" % (f, name)] = arr
    np.savez_compressed(path, **out)
    return path


def load_recording(path):
    """Load a recording -> (t, {name: {"pos":..., "vel":..., "quat":...,
    "omega":...}}). Inverse of write_recording."""
    with np.load(path) as z:
        data = {k: z[k] for k in z.files}
    t = data.pop("t")
    bodies = {}
    for key, arr in data.items():
        field, _, name = key.partition("_")
        if field not in FIELDS:
            raise ValueError("unrecognized key %r in %s" % (key, path))
        bodies.setdefault(name, {})[field] = arr
    for name, d in bodies.items():
        missing = [f for f in FIELDS if f not in d]
        if missing:
            raise ValueError("body %r missing fields %s in %s"
                             % (name, missing, path))
    return t, bodies


class Recorder:
    """Convenience accumulator: append one sample per step, then save.

    rec = Recorder(["ball"])
    rec.add(t, {"ball": (pos3, vel3, quat4_wxyz, omega3)})   # per step
    rec.save(path)
    """

    def __init__(self, names):
        self.names = list(names)
        self.t = []
        self.rows = {n: {f: [] for f in FIELDS} for n in self.names}

    def add(self, t, sample):
        self.t.append(float(t))
        for n in self.names:
            pos, vel, quat, omega = sample[n]
            r = self.rows[n]
            r["pos"].append(pos)
            r["vel"].append(vel)
            r["quat"].append(quat)
            r["omega"].append(omega)

    def save(self, path):
        bodies = {n: {f: np.asarray(self.rows[n][f], dtype=np.float64)
                      for f in FIELDS} for n in self.names}
        return write_recording(path, np.asarray(self.t), bodies)
