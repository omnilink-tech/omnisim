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

"""A slider's child keeps its AUTHORED orientation when the joint reaches Newton.

WHAT THIS PINS
--------------
Until 2026-09-27 the revolute path carried the child's rotation relative to its
parent (R_child^T * R_parent) into newton's child_xform, and the prismatic path
did not: add_joint_prismatic had no rotation argument, so the child_xform was
identity and newton registered every slider child AT ITS PARENT'S ORIENTATION.
An external user's minimal probe (a slider whose child Solid is authored at
90 deg about X) read 1.570796327 rad of orientation error after sliding while
the slider readback said the correct 0.1 m, and on a Galbot R1 gripper the
rotated finger colliders intruded into the object being grasped.

The engine now passes the rotation (OmBasicJoint::flushPendingNewtonRegistrations
-> OmNewtonBackend::addJointPrismatic). This file pins the runtime half: the
rotation must arrive in the builder's child_xform unmodified, the default must
stay identity, and an engine binary that predates the argument (18 positional
arguments, initial_q last) must still land initial_q in initial_q.

The engine half (the axis re-expression into a merged leader's frame and the
rotation reaching the runtime) needs a real engine; it is pinned by
tests/test_newton_joint_frames.py, which runs the same two probes.

RUN:  python -m pytest tests/test_newton_prismatic_child_frame.py -q
"""

import importlib.util
import math
import os
import unittest

import pytest

# Loads the REAL runtime, which imports warp/newton at module level; skip cleanly
# where the GPU physics stack is absent (the hosted engine-free lane), exactly
# as tests/test_newton_collider_orientation.py does.
pytest.importorskip("warp", reason="needs the GPU physics stack (warp)")
pytest.importorskip("newton", reason="needs the GPU physics stack (newton)")

_HERE = os.path.dirname(os.path.abspath(__file__))
_RUNTIME = os.path.join(_HERE, os.pardir, "src", "omnisim", "physics",
                        "omnisim_newton_runtime.py")


def _load_runtime():
    """Import the SOURCE runtime by path, never the (possibly stale) bundle copy."""
    spec = importlib.util.spec_from_file_location("_omnisim_newton_runtime_prismatic",
                                                  os.path.normpath(_RUNTIME))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _RecordingBuilder(object):
    """Stands in for newton.ModelBuilder: records every joint call's kwargs."""

    def __init__(self):
        self.calls = []

    def _record(self, kind, kw):
        self.calls.append((kind, kw))
        return len(self.calls) - 1

    def add_joint_prismatic(self, **kw):
        return self._record("prismatic", kw)

    def add_joint_revolute(self, **kw):
        return self._record("revolute", kw)


def _quat(xform):
    q = xform.q if hasattr(xform, "q") else xform[1]
    return tuple(float(v) for v in (q[0], q[1], q[2], q[3]))


IDENTITY = (0.0, 0.0, 0.0, 1.0)
_H = math.pi / 4.0
# R_child^T * R_parent for a child authored at +90 deg about X: -90 deg about X.
ROT = (-math.sin(_H), 0.0, 0.0, math.cos(_H))
PLACES = 6  # wp.transform stores float32


class PrismaticChildFrameTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = _load_runtime()

    def _world(self):
        World = self.mod.World
        w = World.__new__(World)
        w.builder = _RecordingBuilder()
        w.pending_revolutes = []
        return w

    def _built(self, w):
        spec = w.pending_revolutes[-1]
        w._add_revolute_to_builder(spec)
        return spec, w.builder.calls[-1]

    def assertQuat(self, got, want, msg):
        for i, (g, e) in enumerate(zip(got, want)):
            self.assertAlmostEqual(g, e, places=PLACES,
                                   msg="%s (component %d: got %r, want %r)" % (msg, i, got, want))

    def test_rotation_reaches_child_xform(self):
        w = self._world()
        w.add_joint_prismatic(0, 1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              1000.0, 50.0, 0.0, 0.2, 100.0, 0.0,
                              0.0, *ROT)
        spec, (kind, kw) = self._built(w)
        self.assertEqual(kind, "prismatic")
        self.assertQuat(spec["c_rot"], ROT, "the queued spec must keep the rotation")
        self.assertQuat(_quat(kw["child_xform"]), ROT,
                        "the slider child's authored rotation must reach newton's child_xform; "
                        "identity here is the child snapped to its parent's orientation")

    def test_default_is_identity(self):
        w = self._world()
        w.add_joint_prismatic(0, 1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        _spec, (_kind, kw) = self._built(w)
        self.assertQuat(_quat(kw["child_xform"]), IDENTITY, "no rotation given -> identity")

    def test_old_binary_argument_order_still_works(self):
        """A binary built before the argument passes 18 positionals, initial_q last."""
        w = self._world()
        w.add_joint_prismatic(0, 1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              1000.0, 50.0, 0.0, 0.2, 100.0, 0.0, 0.05)
        spec = w.pending_revolutes[-1]
        self.assertAlmostEqual(spec["initial_q"], 0.05,
                               msg="initial_q must not be shifted into the quaternion")
        self.assertQuat(spec["c_rot"], IDENTITY, "an old binary gets its old (identity) behaviour")

    def test_revolute_path_unchanged(self):
        """Control: the revolute path already carried the rotation and still does."""
        w = self._world()
        w.add_joint_revolute(0, 1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             1000.0, 50.0, -1.0, 1.0, 100.0, 0.0, *ROT)
        _spec, (kind, kw) = self._built(w)
        self.assertEqual(kind, "revolute")
        self.assertQuat(_quat(kw["child_xform"]), ROT, "revolute child_xform rotation")


if __name__ == "__main__":
    unittest.main()
