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

"""A Radar with `occlusion TRUE` must actually drop the target behind a wall.

WHY THIS EXISTS
---------------
A Radar with `occlusion TRUE` drops a target whose device->target ray is blocked.
The ray is answered by OmObjectDetection::refreshCollisionDepthsFromNewton ->
OmNewtonBackend::raycastBatch (mj_ray over the LIVE mjModel), excluding the
radar's own robot AND the target's own bodies (replicating the historical
rayCollisionCallback target filter), behind the value-parsed
OMNISIM_NEWTON_RAYCAST gate (default ON). Camera recognition occlusion rides the
exact same OmObjectDetection code path.

THE MEASUREMENT
---------------
One generated world, one radar, two static targets with radarCrossSection 1 and
IDENTICAL cross-section geometry (0.4 m boxes), one wall covering exactly one of
them (newtonStatics TRUE so the wall exists as a collider):

    rt_clear   at (2, 0,   0.5) -- clear line of sight, distance 2.0
    rt_blocked at (2, 1.5, 0.5) -- wall at (1, 0.75) blocks the ray, distance 2.5

THE ODE ARM IS GONE
-------------------
This test used to run OMNISIM_FORCE_ODE=1 as a live oracle beside the Newton
arm and require the surviving-target count and distance to match. src/ode has
been deleted, so the ODE arm has been DELETED from this file. Its answers were
measured first and frozen into

    tests/goldens/ode_oracle_goldens.json  ->  families.radar_occlusion

(measured: count 1, distance 2.0 exactly). The frozen values are asserted at the
same 1e-3 budget, and the ABSOLUTE expectation -- exactly one surviving target
at ~2.0 m -- is asserted too, since it is the geometry rather than either
backend's answer.

WHY THE GOLDEN TEST ALONE WAS NOT ENOUGH  (read this before trusting it)
------------------------------------------------------------------------
The golden test above was written and frozen while the occlusion rays were still
carried by ODE ray geoms. When src/ode went away, the carrier went with it:
OmObjectDetection::createRays stopped producing any ray at all, so
refreshCollisionDepthsFromNewton found an empty ray list, returned immediately
having tested nothing, and EVERY target reported as unoccluded. The golden test
did not catch that regression, for two independent reasons:

  1. It only ever ran ONE arm. "occlusion TRUE keeps 1 of 2 targets" and
     "occlusion is not evaluated at all" are only distinguishable if you also
     measure what the SAME world reports with occlusion switched OFF. Without
     that control, a count of 2 reads as "the wall did not block" rather than as
     "no ray was cast".
  2. Its verdict is reachable only when the Newton runtime comes up. A clone
     where the embedded interpreter cannot import warp/mujoco trips the
     `_BRINGUP` guard, `_run_with_retry` returns None, and the test SKIPS --
     which a pytest summary reports as green. A skip is not a pass.

So this file now also runs the DIFFERENTIAL below. It fails, rather than skips,
when occlusion stops being simulated -- and it asserts the surviving target's
range is a real measured range rather than the 0.0 an unfiltered candidate
carries, which is the other half of the same regression (with the ray carrier
dead, nothing re-aimed the rays or computed the targets' properties either).

CAMERA RECOGNITION
------------------
No Camera arm is added here. Camera recognition occlusion goes through the same
OmObjectDetection carrier and the same refreshCollisionDepthsFromNewton call, and
it already HAS an occlusion differential in the C test-suite lane:
tests/api/worlds/camera_recognition.omniworld + tests/api/controllers/camera_recognition
assert 8 recognized objects with `occlusion 0` and exactly 7 after the supervisor
sets `occlusion 2` -- the eighth being the solid modelled "occluded box", which is
also name-checked against the visible-solid list. That lane needs the rendered
image (segmentation, position_on_image), which this Popen/--no-rendering harness
cannot provide, so duplicating it here would be weaker, not stronger.

    python -m pytest tests/test_newton_radar_occlusion_parity.py -v
"""
from __future__ import annotations

import json
import math
import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLDS = REPO / "tests" / "physics" / "worlds"

EXPECTED_COUNT = 1
EXPECTED_DISTANCE = 2.0
#: rt_blocked sits at (2, 1.5, 0.5); the radar at (0, 0, 0.5) -> |(2, 1.5, 0)|
EXPECTED_BLOCKED_DISTANCE = 2.5
#: with occlusion OFF both boxes are detectable -- this is the control arm
EXPECTED_COUNT_NO_OCCLUSION = 2
DIST_ABS_TOL = 0.05   # frustum-AABB midpoint vs geometric center
#: the SAME budget the live ODE-vs-Newton comparison used, now applied against
#: the frozen ODE value instead of a live one.
GOLDEN_TOL = 1e-3
GOLDENS = REPO / "tests" / "goldens" / "ode_oracle_goldens.json"


def _goldens():
    return json.loads(GOLDENS.read_text(encoding="utf-8"))["families"]["radar_occlusion"]["measurements"]

_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up",
            "Refusing to run it on ODE")


def _world_text(occlusion="TRUE"):
    def target(name, x, y):
        return """
DEF %(N)s Solid {
  translation %(x)s %(y)s 0.5
  name "%(N)s"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.8 0.3 0.2 roughness 1 metalness 0 }
      geometry Box { size 0.4 0.4 0.4 }
    }
  ]
  boundingObject Box { size 0.4 0.4 0.4 }
  radarCrossSection 1
}
""" % {"N": name, "x": x, "y": y}

    return """#VRML_SIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_radar_occlusion_parity.py -- do not commit.

WorldInfo {
  basicTimeStep 8
  gravity 0
  newtonStatics TRUE
  coordinateSystem "ENU"
}
Viewpoint { position -3 1 1 }
Background { skyColor [ 0.2 0.2 0.25 ] }
DEF PROBE Robot {
  name "probe"
  controller "radar_occlusion_probe"
  children [
    Radar {
      translation 0 0 0.5
      name "radar"
      minRange 0.5
      maxRange 4
      horizontalFieldOfView 2
      verticalFieldOfView 0.5
      occlusion %(OCCLUSION)s
    }
  ]
}
%(T_CLEAR)s%(T_BLOCKED)s
DEF WALL Solid {
  translation 1 0.75 0.5
  name "wall"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.3 0.3 0.8 roughness 1 metalness 0 }
      geometry Box { size 0.2 1 1 }
    }
  ]
  boundingObject Box { size 0.2 1 1 }
}
""" % {"OCCLUSION": occlusion,
       "T_CLEAR": target("RT_CLEAR", 2, 0),
       "T_BLOCKED": target("RT_BLOCKED", 2, 1.5)}


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone")


def _run_newton(tmp_path, attempt, occlusion="TRUE"):
    """Run the Newton arm. -> {"count", "distances"} or None (bring-up flake)."""
    tag = "occl" if occlusion == "TRUE" else "noocc"
    world = WORLDS / (".radar_occlusion_parity_%s.wbt" % tag)
    world.write_text(_world_text(occlusion), encoding="utf-8")
    out = tmp_path / ("probe_newton_%s_%d.json" % (tag, attempt))
    log = tmp_path / ("engine_newton_%s_%d.log" % (tag, attempt))
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    env["OMNISIM_RADAR_PROBE_OUT"] = str(out)
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
            env.pop(k)
    env["OMNISIM_NEWTON_RAYCAST"] = "1"
    proc = subprocess.Popen(
        [str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", "--stdout", "--stderr"],
        cwd=str(REPO), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(120):          # the probe writes after ~12 ticks; poll
            if out.exists():
                break
            try:
                proc.wait(timeout=1)
                break
            except subprocess.TimeoutExpired:
                continue
    finally:
        if proc.poll() is None:
            if os.name == "nt":
                subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                               capture_output=True)
            else:
                proc.kill()
            proc.wait()
        try:
            world.unlink()
        except OSError:
            pass
    blob = log.read_text(errors="replace") if log.exists() else ""
    if any(sig in blob for sig in _BRINGUP):
        return None
    if not out.exists():
        pytest.fail("the Newton arm (occlusion %s) produced no probe output\n%s"
                    % (occlusion, blob[-1200:]))
    return json.loads(out.read_text())


def _run_with_retry(tmp_path, occlusion="TRUE"):
    """The Newton FFI bring-up flakes ~3% of launches; retry once before skipping."""
    for attempt in (1, 2):
        got = _run_newton(tmp_path, attempt, occlusion)
        if got is not None:
            return got
    return None


#: both arms are needed by two tests each; one launch per arm is enough.
_ARMS: dict = {}


def _arm(occlusion, tmp_path):
    if occlusion not in _ARMS:
        _ARMS[occlusion] = _run_with_retry(tmp_path, occlusion)
    return _ARMS[occlusion]


def test_newton_radar_occlusion_matches_frozen_ode_goldens(tmp_path):
    newton = _arm("TRUE", tmp_path)
    if newton is None:
        pytest.skip("Newton bring-up flake on both attempts -- no data, re-run")
    goldens = _goldens()
    count_g = goldens["surviving_target_count"]["ode_value"]
    dist_g = goldens["surviving_target_distance"]["ode_value"]

    problems = []
    # (1) absolute geometry -- backend-independent
    if newton["count"] != EXPECTED_COUNT:
        problems.append("Newton kept %d targets, expected %d (the occluded target must be "
                        "dropped, the clear one kept)" % (newton["count"], EXPECTED_COUNT))
    elif not math.isclose(newton["distances"][0], EXPECTED_DISTANCE, abs_tol=DIST_ABS_TOL):
        problems.append("Newton distance %.4f vs the geometric %.1f m (tol %g)"
                        % (newton["distances"][0], EXPECTED_DISTANCE, DIST_ABS_TOL))
    # (2) the frozen ODE oracle
    if newton["count"] != count_g:
        problems.append("Newton kept %d targets vs the FROZEN ODE-oracle count %d"
                        % (newton["count"], count_g))
    elif newton["distances"] and not math.isclose(newton["distances"][0], dist_g,
                                                  abs_tol=GOLDEN_TOL):
        problems.append("surviving target: Newton distance %.6f vs the FROZEN ODE-oracle "
                        "value %.6f (|d|=%.2e > %g)"
                        % (newton["distances"][0], dist_g,
                           abs(newton["distances"][0] - dist_g), GOLDEN_TOL))
    assert not problems, (
        "radar occlusion parity against the frozen ODE goldens failed:\n  "
        + "\n  ".join(problems) +
        "\n\nThe reference numbers are FROZEN ODE-ORACLE VALUES, measured before src/ode was "
        "removed and committed to tests/goldens/ode_oracle_goldens.json "
        "(families.radar_occlusion). THE ODE ARM NO LONGER EXISTS. Newton answers via "
        "OmObjectDetection::refreshCollisionDepthsFromNewton -> raycastBatch (mj_ray on the "
        "live mjModel) behind OMNISIM_NEWTON_RAYCAST=1; camera recognition occlusion rides the "
        "same path. A mismatch is a Newton regression, not a stale golden. Do not retune the "
        "golden.")


def test_both_targets_are_detectable_without_occlusion(tmp_path):
    """Control arm: with `occlusion FALSE` the wall is irrelevant, so BOTH boxes
    must be reported, each at its own geometric range.

    This is what makes the differential below meaningful: it proves the two
    targets have equal, sufficient radar cross-section, sit inside the frustum,
    and clear the received-power threshold. Without it, "one target survived"
    could just as well mean the second one was never detectable at all.
    """
    control = _arm("FALSE", tmp_path)
    if control is None:
        pytest.skip("Newton bring-up flake on both attempts -- no data, re-run")
    problems = []
    if control["count"] != EXPECTED_COUNT_NO_OCCLUSION:
        problems.append("occlusion FALSE reported %d targets, expected %d (rt_clear at 2.0 m and "
                        "rt_blocked at 2.5 m are both in frustum, in range, and above the "
                        "minDetectableSignal threshold)"
                        % (control["count"], EXPECTED_COUNT_NO_OCCLUSION))
    else:
        # the probe sorts the distances
        for got, want in zip(control["distances"], (EXPECTED_DISTANCE, EXPECTED_BLOCKED_DISTANCE)):
            if not math.isclose(got, want, abs_tol=DIST_ABS_TOL):
                problems.append("occlusion FALSE distance %.4f vs the geometric %.1f m (tol %g)"
                                % (got, want, DIST_ABS_TOL))
    assert not problems, (
        "the no-occlusion control arm failed:\n  " + "\n  ".join(problems) +
        "\n\nThis arm does not cast a single ray -- it is the plain frustum/range/power path "
        "(OmRadar::computeTargets(finalSetup=true)). If it fails, the occlusion differential "
        "cannot be interpreted: fix this first.")


def test_occlusion_actually_removes_the_walled_target(tmp_path):
    """THE DIFFERENTIAL -- the assertion that catches a dead ray carrier.

    Same world, same geometry, same two equal-cross-section targets; the only
    change is `occlusion FALSE` -> `occlusion TRUE`. Turning occlusion on MUST
    remove exactly the target the wall covers, and MUST leave the clear one at
    its real measured range.

    A build where occlusion is not evaluated (no ray carrier, no raycast
    service, or a raycast that tests nothing) reports the SAME target set for
    both arms, and fails here. That is the regression this test exists for: the
    single-arm golden test above cannot see it.
    """
    control = _arm("FALSE", tmp_path)
    occluded = _arm("TRUE", tmp_path)
    if control is None or occluded is None:
        pytest.skip("Newton bring-up flake on both attempts -- no data, re-run")

    problems = []
    if occluded["count"] >= control["count"]:
        problems.append("occlusion TRUE kept %d of the %d targets the no-occlusion control "
                        "reports -- the wall changed NOTHING, so no occlusion ray was actually "
                        "cast and answered" % (occluded["count"], control["count"]))
    if occluded["count"] != EXPECTED_COUNT:
        problems.append("occlusion TRUE kept %d targets, expected exactly %d (rt_blocked is "
                        "fully behind the 0.2 x 1 x 1 wall at (1, 0.75); rt_clear is not)"
                        % (occluded["count"], EXPECTED_COUNT))
    else:
        d = occluded["distances"][0]
        # the clear target, at ITS range -- not the blocked one, and not the 0.0
        # an unfiltered candidate carries before its properties are computed
        if not math.isclose(d, EXPECTED_DISTANCE, abs_tol=DIST_ABS_TOL):
            if math.isclose(d, 0.0, abs_tol=1e-9):
                problems.append("the surviving target reports distance 0.0 -- the occlusion path "
                                "never ran the frustum/target-property pass, so the range was "
                                "never measured (OmRadar::updateRaysSetupIfNeeded was not called)")
            elif math.isclose(d, EXPECTED_BLOCKED_DISTANCE, abs_tol=DIST_ABS_TOL):
                problems.append("the surviving target is at %.4f m -- that is rt_blocked, so the "
                                "occlusion test dropped the WRONG target (the ray geometry or the "
                                "target-exclusion list is inverted)" % d)
            else:
                problems.append("the surviving target is at %.4f m, expected rt_clear at %.1f m "
                                "(tol %g)" % (d, EXPECTED_DISTANCE, DIST_ABS_TOL))
    assert not problems, (
        "radar occlusion is not being simulated:\n  " + "\n  ".join(problems) +
        "\n\nThe occlusion ray for each target is carried by "
        "OmObjectDetection::RaySegment (start/direction/length) and cast by "
        "OmObjectDetection::refreshCollisionDepthsFromNewton -> "
        "OmNewtonBackend::raycastBatch under OMNISIM_NEWTON_RAYCAST (default ON). It used to be "
        "carried by an ODE ray geom; when src/ode was deleted the carrier went with it and every "
        "target silently read as unoccluded. Camera Recognition occlusion rides the identical "
        "OmObjectDetection path, so this differential covers both consumers.")
