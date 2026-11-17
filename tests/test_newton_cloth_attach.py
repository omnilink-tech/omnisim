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

"""The cloth particle attach (kinematic grab) does what it acks.

WHY THIS MECHANISM EXISTS
-------------------------
The t-shirt fold campaign measured -- 12 laptop configurations, 8 cloud seeds
with valid negative controls, one genuine 5.5 mm pinch that still shed under
lift -- that SolverVBD cannot hold cloth by FRICTION: its contact friction is
velocity-regularised (friction_epsilon), so under a constant tangential load W
the contact creeps at u* ~ W*eps_u/(2*mu*N), nonzero for every mu (upstream
issue newton-physics/newton#3943). The carry mechanism is therefore a runtime
particle attach: OMNISIM_CLOTH_ATTACH_CMD names a command file; an `attach`
line pins the cloth particles nearest a point to a rigid body (both
particle_mass and particle_inv_mass to 0 -- VBD's forward_step gates on
inv_mass == 0 and its local solve gates on mass == 0, so BOTH), a `detach`
line restores them, and every command is acked with the MEASURED selection
count in <cmd>.ack.

WHAT IS PINNED HERE
-------------------
The full protocol on a real CPU-VBD solve (13x13 grid, ~80 steps, no engine
binary): a grab selects and acks >0 particles, the pinned patch follows a
force-driven dynamic anchor to millimetres while the free cloth is dragged
elastically rather than co-moving, detach restores the masses and the fabric
resumes falling, and an attach at a point with no fabric acks attached=0 --
the negative-control arm, which must report the miss rather than assume a
hold.

RUN:  python tests/test_newton_cloth_attach.py   (or under pytest)
Skips cleanly when the newton runtime is not importable.
"""

import importlib.util
import json
import os
import sys
import tempfile
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, os.pardir))
_RUNTIME = os.path.join(_ROOT, "src", "omnisim", "physics",
                        "omnisim_newton_runtime.py")
_BUNDLE = os.path.join(_ROOT, "msys64", "mingw64", "bin", "newton-runtime",
                       "site-packages")


def _newton_available():
    try:
        import newton  # noqa: F401
        return True
    except ImportError:
        if os.path.isdir(_BUNDLE):
            sys.path.insert(0, _BUNDLE)
            try:
                import newton  # noqa: F401
                return True
            except ImportError:
                return False
        return False


@unittest.skipUnless(_newton_available(), "newton runtime not importable")
class ClothAttachTest(unittest.TestCase):

    def test_attach_protocol_end_to_end(self):
        tmp = tempfile.mkdtemp(prefix="cloth_attach_")
        cmd = os.path.join(tmp, "attach_cmd.jsonl")
        os.environ["OMNISIM_CLOTH_ATTACH_CMD"] = cmd
        try:
            # Import the SOURCE runtime by path -- a plain import could pick
            # up the stale bundle copy, the exact trap that once let a
            # 23-minute-old bundle pass a full suite.
            spec = importlib.util.spec_from_file_location(
                "_omnisim_newton_runtime_attach_test", _RUNTIME)
            rt = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(rt)
            import numpy as np

            w = rt.World()
            w.set_solver_preference("vbd")
            pad = w.add_body(1.0, 0.0, 0.0, 0.60)
            w.add_shape_box(pad, 0.03, 0.03, 0.01)
            lo, hi = w.add_cloth_grid(-0.30, -0.30, 0.50, dim_x=12, dim_y=12,
                                      cell_x=0.05, cell_y=0.05, mass=0.05,
                                      particle_radius=0.005)
            w.finalize()
            dt = 1.0 / 60.0

            def pq():
                return np.asarray(w.state_a.particle_q.numpy())[:, 0:3]

            def send(obj):
                with open(cmd, "a", encoding="utf-8") as f:
                    f.write(json.dumps(obj) + "\n")

            def acks():
                ap = cmd + ".ack"
                if not os.path.exists(ap):
                    return []
                with open(ap, encoding="utf-8") as f:
                    return [json.loads(x) for x in f if x.strip()]

            # control: gravity works
            z0 = pq()[:, 2].mean()
            for _ in range(20):
                w.step(dt)
            self.assertLess(pq()[:, 2].mean(), z0 - 0.02,
                            "cloth did not fall -- no valid baseline")

            # attach near the (mid-fall) cloth centre
            centre = pq()[(lo + hi) // 2]
            send({"op": "attach", "body": int(pad),
                  "point": [float(centre[0]), float(centre[1]),
                            float(centre[2])], "radius": 0.08})
            w.step(dt)
            a = acks()
            self.assertEqual(len(a), 1)
            n_att = a[0].get("attached", 0)
            self.assertGreater(n_att, 0, "grab selected no particles")

            inv = np.asarray(w.model.particle_inv_mass.numpy())
            pinned = np.nonzero(inv[lo:hi] == 0.0)[0] + lo
            free = np.setdiff1d(np.arange(lo, hi), pinned)
            self.assertEqual(pinned.size, n_att,
                             "acked count != zero-inv-mass count")

            # follow: drive the anchor sideways with an external force (a
            # position-based solver derives velocity from position history,
            # so set_body_vel is not a usable motion injector here)
            b0 = np.asarray(w.state_a.body_q.numpy())[pad, 0:3].copy()
            p0 = pq()[pinned].mean(axis=0)
            fx0 = pq()[free, 0].mean()
            for _ in range(15):
                w.add_body_force(pad, 40.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                w.step(dt)
            b1 = np.asarray(w.state_a.body_q.numpy())[pad, 0:3]
            dp = pq()[pinned].mean(axis=0) - p0
            db = b1 - b0
            self.assertGreater(abs(db[0]), 0.05, "anchor never moved")
            self.assertLess(float(np.linalg.norm(dp - db)), 0.02,
                            "pinned patch did not track the anchor")
            # free cloth is DRAGGED (elastic coupling -- that is what a carry
            # is), never co-moving
            self.assertLess(abs(pq()[free, 0].mean() - fx0),
                            0.2 * abs(dp[0]),
                            "free cloth co-moved -- pin is not what carried")

            # detach restores mass and the fabric resumes falling
            send({"op": "detach"})
            w.step(dt)
            a = acks()
            self.assertEqual(a[-1].get("released"), n_att)
            zr0 = pq()[pinned, 2].mean()
            for _ in range(20):
                w.step(dt)
            self.assertLess(pq()[pinned, 2].mean(), zr0 - 0.02,
                            "released particles did not resume falling")
            inv = np.asarray(w.model.particle_inv_mass.numpy())
            self.assertTrue(bool(np.all(inv[lo:hi] > 0.0)),
                            "inv mass not restored on detach")

            # the miss arm: no fabric near the point -> attached=0, acked
            send({"op": "attach", "body": int(pad),
                  "point": [5.0, 5.0, 5.0], "radius": 0.08})
            w.step(dt)
            self.assertEqual(acks()[-1].get("attached"), 0,
                             "a whole-garment miss must ack 0, not hold air")
        finally:
            os.environ.pop("OMNISIM_CLOTH_ATTACH_CMD", None)


if __name__ == "__main__":
    unittest.main(verbosity=2)
