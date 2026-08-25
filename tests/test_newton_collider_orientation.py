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

"""A collider's AUTHORED orientation must reach the solver, unmodified.

WHAT THIS PINS, AND WHY NOTHING DID BEFORE
------------------------------------------
The Newton path used to apply a hard-coded -90 deg about X to every cylinder
collider, on the stated premise that "a Webots Cylinder bounding object extends
along its body-local Y". That premise is false in three independent ways:

  * an OmniSim Cylinder is Z-ALIGNED  (docs/reference/cylinder.md,
    OmCylinder::rescale -- "the radius spans x/y, the height spans z"),
  * a URDF <cylinder> is Z-aligned    (ROS convention), and
  * a newton capsule is Z-aligned     (what the cylinder is substituted with).

Nothing needed rotating. Compounding it, BOTH of OmSolid's boundingObject
walkers accumulated Pose *translation* only and never read OmPose::rotation(),
so the authored orientation was thrown away first.

The two defects cancelled on WHEELS and only on wheels. The URDF importer wraps
every cylinder collision in a Pose, and a wheel's carries rpy 1.570795 about X;
that +90 was dropped and a -90 invented, and because a capsule is symmetric
about its own centre, +90 and -90 about X describe the SAME capsule. Wheels
rolled, the 8-Husky datum held, and every collider that was NOT pre-rotated was
silently corrupted -- OMNIARM6's seven arm cylinders are rpy "0 0 0", so each was
tipped from Z to Y and left lying crosswise through its own link.

It survived two months because no test pinned the axis, the one world anybody
re-ran was a wheeled one, and the OMNIARM6 demos had been validated BEFORE the
capsule substitution landed (a 2026-06-05 artifact still shows those colliders
as point spheres; the substitution is 2026-06-08, W1.2).

So the assertion below is deliberately blunt and deliberately cheap: identity in
must give identity out. A test that can only fail when a GPU is free is a test
that does not run, and this one needs no engine, no world and no CUDA.

RUN:  python tests/test_newton_collider_orientation.py
      (or under pytest -- both work)
"""

import importlib.util
import math
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_RUNTIME = os.path.join(_HERE, os.pardir, "src", "omnisim", "physics",
                        "omnisim_newton_runtime.py")


def _load_runtime():
    """Import the runtime module by PATH.

    Deliberately not `import omnisim_newton_runtime`: the engine loads this file
    out of the BUNDLE (msys64/mingw64/bin/newton-runtime/site-packages/), and a
    plain import could silently pick that stale copy up instead of the source
    being tested -- which is the exact trap that let a 23-minute-old bundle pass
    a full suite while the fix under test was never loaded.
    """
    spec = importlib.util.spec_from_file_location("_omnisim_newton_runtime_under_test",
                                                  os.path.normpath(_RUNTIME))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _RecordingBuilder(object):
    """Stands in for newton.ModelBuilder and records the xform of every shape.

    Only the add_shape_* entry points the runtime actually calls are needed, so
    a real ModelBuilder (and a real newton device) never has to exist.
    """

    def __init__(self):
        self.calls = []

    def _record(self, kind, kwargs):
        self.calls.append((kind, kwargs))
        return len(self.calls) - 1

    def add_shape_capsule(self, body, **kw):
        return self._record("capsule", kw)

    def add_shape_box(self, body, **kw):
        return self._record("box", kw)

    def add_shape_sphere(self, body, **kw):
        return self._record("sphere", kw)

    def add_shape_mesh(self, body, **kw):
        return self._record("mesh", kw)


def _quat_of(kw):
    """Pull the (x, y, z, w) rotation out of the recorded wp.transform."""
    xform = kw["xform"]
    q = xform.q if hasattr(xform, "q") else xform[1]
    return tuple(float(v) for v in (q[0], q[1], q[2], q[3]))


def _pos_of(kw):
    xform = kw["xform"]
    p = xform.p if hasattr(xform, "p") else xform[0]
    return tuple(float(v) for v in (p[0], p[1], p[2]))


# ⚠ wp.transform stores FLOAT32, so a float64 quaternion does not survive the
# round trip bit-for-bit (0.7071067811865 comes back 0.70710676908). Compare to
# ~1e-6, which is far tighter than any rotation error that could matter and far
# looser than float32 noise. An exact-equality assertion here is a test that
# fails for a reason that has nothing to do with the bug.
PLACES = 6

IDENTITY = (0.0, 0.0, 0.0, 1.0)
# +90 deg about X, the orientation a Husky wheel's Pose actually authors.
_H = math.pi / 4.0
WHEEL_Q = (math.sin(_H), 0.0, 0.0, math.cos(_H))


class ColliderOrientationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = _load_runtime()

    def assertVecAlmost(self, got, want, msg):
        self.assertEqual(len(got), len(want))
        for i, (g, wv) in enumerate(zip(got, want)):
            self.assertAlmostEqual(g, wv, places=PLACES,
                                   msg="%s (component %d: got %r, want %r)" % (msg, i, got, want))

    def _world(self):
        """A World with no __init__ -- only `builder` is exercised here."""
        World = self.mod.World
        w = World.__new__(World)
        w.builder = _RecordingBuilder()
        # _shape_cfg is a cached classmethod; reset so ordering can't leak.
        World._SHAPE_CFG = None
        return w

    # -- the headline: no invented rotation ---------------------------------
    def test_unrotated_cylinder_stays_z_aligned(self):
        """An UNROTATED Cylinder must reach the solver unrotated.

        This is the assertion the -90 violated. It is worth stating in terms of
        the physical claim rather than the quaternion: a cylinder authored with
        no rotation is Z-aligned in OmniSim, in URDF and in newton, so the
        collider handed to the solver must be Z-aligned too.
        """
        w = self._world()
        w.add_shape_cylinder(0, 0.05, 0.30)
        kind, kw = w.builder.calls[-1]
        self.assertEqual(kind, "capsule",
                         "cylinder is substituted with a capsule (W1.2, probe 7)")
        self.assertVecAlmost(_quat_of(kw), IDENTITY,
                             "an unrotated Cylinder must NOT be rotated by the backend; "
                             "a non-identity quaternion here is the -90 deg about X coming back")

    def test_cylinder_passes_authored_rotation_through_verbatim(self):
        """A rotated Cylinder gets exactly the rotation it was given -- no more."""
        w = self._world()
        w.add_shape_cylinder(0, 0.05, 0.30, 0.1, 0.2, 0.3,
                             WHEEL_Q[0], WHEEL_Q[1], WHEEL_Q[2], WHEEL_Q[3])
        _kind, kw = w.builder.calls[-1]
        self.assertVecAlmost(_quat_of(kw), WHEEL_Q,
                             "the authored quaternion must arrive unmodified")
        self.assertVecAlmost(_pos_of(kw), (0.1, 0.2, 0.3), "offset must arrive unmodified")

    def test_wheel_case_is_unchanged_by_the_fix(self):
        """The Husky wheel is +90 about X; the old code shipped -90 about X.

        Both describe the same capsule (a capsule is symmetric about its own
        centre), which is the whole reason the defect hid behind a wheeled
        world. Pinned here so the equivalence is a stated fact rather than a
        thing someone has to re-derive when the 56.579 m datum is questioned.
        """
        # A capsule's axis is its local Z; +90 and -90 about X map Z to -Y and
        # +Y respectively -- the same infinite line, hence the same solid.
        axis_plus = _rotate_z_axis(WHEEL_Q)
        axis_minus = _rotate_z_axis((-WHEEL_Q[0], 0.0, 0.0, WHEEL_Q[3]))
        for a, b in zip(axis_plus, axis_minus):
            self.assertAlmostEqual(a, -b, places=9,
                                   msg="+90 and -90 about X must be antiparallel, i.e. the same capsule")

    # -- axial extent: the capsule must not outgrow the cylinder ------------
    def test_capsule_substitute_matches_the_cylinder_extent(self):
        """The substitute must not be LONGER than the cylinder it replaces.

        newton's capsule spans (half_height + radius) each way; the authored
        cylinder spans half_height. Passing half_height through verbatim adds a
        hemisphere at each end, growing the collider by 2*radius along an axis
        the author never gave it.

        This is the second half of the same defect as the invented -90. While
        every cylinder was tipped, the overhang pointed sideways into free
        space and only made links fatter. With the axis corrected it points
        ALONG the link, and on omniarm6_2f140_pick_place that drove the base-yaw
        link's collider 45 mm THROUGH the ground plane (authored z 0.055..0.285,
        capsule z -0.045..0.385), where mu=6 locked the base yaw and the arm
        never swung to the place table.
        """
        w = self._world()
        r, h = 0.10, 0.115          # the OMNIARM6 base-yaw link, measured
        w.add_shape_cylinder(0, r, h)
        _kind, kw = w.builder.calls[-1]
        self.assertAlmostEqual(
            kw["half_height"] + kw["radius"], h, places=PLACES,
            msg="capsule half-extent (half_height + radius) must equal the "
                "cylinder's half_height; a larger value is the overhang that "
                "sank the OMNIARM6 base link into the floor")

    def test_disc_shaped_cylinder_keeps_todays_capsule(self):
        """half_height <= radius is left alone, deliberately -- the wheel case.

        Shortening the core by the radius would drive it to zero and collapse
        the capsule to a SPHERE, which is exactly the point-contact collider
        W1.2 replaced because it locked wheels. A Husky wheel is this shape
        (length 0.1143 -> half_height 0.05715, radius 0.1651), so it must come
        through untouched or the 56.579 m datum moves.
        """
        w = self._world()
        r, h = 0.1651, 0.05715      # the Husky wheel, measured
        w.add_shape_cylinder(0, r, h)
        _kind, kw = w.builder.calls[-1]
        self.assertAlmostEqual(
            kw["half_height"], h, places=PLACES,
            msg="a disc-shaped cylinder must keep its authored half_height; "
                "shortening it here degenerates the wheel to a sphere")
        self.assertGreater(kw["half_height"], 0.0,
                           "the capsule core must never reach zero")

    # -- the sibling shapes -------------------------------------------------
    def test_capsule_carries_offset_and_rotation(self):
        """An authored Capsule used to lose BOTH its offset and its rotation."""
        w = self._world()
        w.add_shape_capsule(0, 0.04, 0.20, 1.0, 2.0, 3.0,
                            WHEEL_Q[0], WHEEL_Q[1], WHEEL_Q[2], WHEEL_Q[3])
        _kind, kw = w.builder.calls[-1]
        self.assertVecAlmost(_pos_of(kw), (1.0, 2.0, 3.0),
                             "add_shape_capsule took no xform at all before this fix")
        self.assertVecAlmost(_quat_of(kw), WHEEL_Q, "capsule rotation must arrive unmodified")

    def test_box_carries_rotation(self):
        """A rotated box collider is a real case and used to be flattened."""
        w = self._world()
        w.add_shape_box(0, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0, -1.0,
                        WHEEL_Q[0], WHEEL_Q[1], WHEEL_Q[2], WHEEL_Q[3])
        _kind, kw = w.builder.calls[-1]
        self.assertVecAlmost(_quat_of(kw), WHEEL_Q, "box rotation must arrive unmodified")

    def test_defaults_are_identity_everywhere(self):
        """Every shape must default to identity, so a positional caller is safe.

        The C++ side calls these positionally over an FFI signature string; the
        defaults are what keep an older engine binary compatible with a newer
        bundle (and vice versa) instead of raising a TypeError mid-world-build.
        """
        w = self._world()
        w.add_shape_cylinder(0, 0.05, 0.3)
        w.add_shape_capsule(0, 0.05, 0.3)
        w.add_shape_box(0, 0.1, 0.1, 0.1)
        for kind, kw in w.builder.calls:
            self.assertVecAlmost(_quat_of(kw), IDENTITY, "%s default must be identity" % kind)
            self.assertVecAlmost(_pos_of(kw), (0.0, 0.0, 0.0), "%s default must be at the origin" % kind)


def _rotate_z_axis(q):
    """The capsule's axis: unit +Z rotated by quaternion (x, y, z, w).

    This is just the third column of the rotation matrix of q.
    """
    x, y, z, w = q
    return (2.0 * (x * z + y * w),
            2.0 * (y * z - x * w),
            1.0 - 2.0 * (x * x + y * y))


if __name__ == "__main__":
    sys.exit(0 if unittest.main(exit=False).result.wasSuccessful() else 1)
