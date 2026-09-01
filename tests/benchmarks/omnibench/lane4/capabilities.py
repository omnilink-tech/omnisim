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

"""OmniBench lane 4a — the CAPABILITY REGISTRY.

The single source of truth for "what can this simulator actually simulate?".
Every probe here is EXECUTED, and its verdict comes from a measurement in
physical units — never from a doc, a code reading, or the absence of an error.

Why this file exists
--------------------
The capability matrix in docs/developer/simulator-comparison.md §4 marks its
own OmniSim column ⊘ — "checked by us, by us, with no external source and no
independent audit", the weakest evidence tier in that document. Every other
lane of OmniBench measures a NUMBER (error, ms/step, determinism grade); none
of them answers the prior question a user actually asks first: *can it
simulate the thing I have?* That question was answered only in prose, and
prose drifts — AGENTS.md's known-broken list is maintained by hand and this
lane exists partly to catch it going stale in either direction (a feature
listed broken that now works is as much a defect in the record as the
reverse).

The four verdicts
-----------------
    works     the capability is present AND the measurement lands where
              physics says it must.
    degraded  present and does something, but the measurement misses the
              physical target (or only lands under a non-default knob).
              `note` must say what and by how much.
    broken    present in the schema — the world loads, the field or device
              is accepted — and the measurement proves it does NOTHING.
              This is the most valuable verdict in the file: it is exactly
              the failure a load-only smoke test reports as PASS.
    absent    not in the schema at all. Established by the ENGINE REFUSING
              the declaration (an unknown-node / unknown-field diagnostic),
              never by "we did not try".

`broken` vs `absent` is the distinction the whole lane is built around, and
neither a static parse test nor a dynamic run test can make it alone:

    a static test  sees BallJoint's motor accepted   -> calls it present
    a dynamic test sees the joint never move         -> calls it broken

Both are run. A capability that parses and then does nothing is strictly
worse than one that is absent, because the world author gets no signal.

Probe kinds
-----------
    KIND_DYNAMIC  generate a .wbt, run it headless through omnisim-bin, have
                  the prober controller record physical quantities, then
                  apply `assertion` to the recorded arrays.
    KIND_STATIC   generate a .wbt that DECLARES the feature, load it, and
                  classify the engine's own diagnostics. No physics involved:
                  this is how `absent` is established as a fact about the
                  schema rather than an absence of effort.

Adding a probe
--------------
Append a Probe(...) below. A dynamic probe needs `world` (a function
returning the scene's VRML body) and `assertion` (a function from the
recorded arrays to a Verdict). The assertion's DOCSTRING is published in the
report as the physical claim being tested, so write it as a claim a reader
can check — "a 0.1 m sphere resting on a floor whose top face is at z=0.55
must settle at z=0.65 +/- 5 mm", not "checks the sphere".
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Callable, Optional

# ---------------------------------------------------------------------------
# verdict vocabulary
# ---------------------------------------------------------------------------
WORKS = "works"
DEGRADED = "degraded"
BROKEN = "broken"
ABSENT = "absent"
#: Not a capability verdict — the probe itself failed to produce a
#: measurement (engine crash, no sidecar, controller never ran). It is
#: reported separately and NEVER folded into a coverage score, because an
#: instrument failure is not evidence about the engine.
INCONCLUSIVE = "inconclusive"

VERDICTS = (WORKS, DEGRADED, BROKEN, ABSENT, INCONCLUSIVE)

KIND_DYNAMIC = "dynamic"
KIND_STATIC = "static"

# families, in report order
FAM_OBJECT = "object"
FAM_JOINT = "joint"
FAM_DEVICE = "device"
FAM_PHENOMENON = "phenomenon"
FAMILIES = (FAM_OBJECT, FAM_JOINT, FAM_DEVICE, FAM_PHENOMENON)


@dataclass
class Verdict:
    """One probe's outcome. `evidence` carries the measured numbers WITH
    units so the row is auditable without re-running; `note` is the
    one-sentence failure mode when the verdict is not `works`."""
    verdict: str
    evidence: dict = field(default_factory=dict)
    note: Optional[str] = None

    def __post_init__(self):
        if self.verdict not in VERDICTS:
            raise ValueError("bad verdict %r" % (self.verdict,))


@dataclass
class Probe:
    id: str                      # e.g. "object.static_floor_collides"
    family: str
    claim: str                   # the capability, one line, user-facing
    kind: str = KIND_DYNAMIC
    duration: float = 3.0        # recorded sim seconds
    dt_ms: float = 4.0
    gravity: float = 9.81
    world: Optional[Callable[[], str]] = None       # VRML body
    measure: tuple = ()          # prober --measure specs
    act: tuple = ()              # prober --act specs
    #: Nodes mounted INSIDE the prober Robot. Every device and every joint
    #: under test lives here, not in a separate rig: OmniSim restricts device
    #: APIs to the owning controller (a supervisor cannot honestly read a
    #: sibling robot's sensors — /robot/<def>/sensor/<name> returns 501 for
    #: the same reason), so a motor or sensor mounted anywhere else would be
    #: unreadable and the probe would measure our own plumbing, not the engine.
    prober_children: str = ""
    #: The prober Robot's own pose/collision. It defaults to an INTANGIBLE
    #: observer (no boundingObject, physics NULL) parked clear of the scene,
    #: so it cannot perturb the body under test. Probes that need the prober
    #: itself to fall (the TouchSensor ones) override all three.
    prober_translation: str = "0 0 1.5"
    prober_bounding: str = ""
    prober_physics: str = "  physics NULL\n"
    world_info: str = ""         # extra WorldInfo lines (newton* knobs, ...)
    #: WorldInfo.newtonSolver. A FIRST-CLASS field rather than something a probe
    #: appends via `world_info`, because the solver profile is not a knob — it
    #: decides which solvers the runtime BUILDS, and therefore which nodes can
    #: reach a solver at all. `world_info` cannot express it: those lines are
    #: emitted AFTER the templated `newtonSolver` inside the same WorldInfo
    #: block, so a probe declaring it there would declare the field twice and
    #: the first value wins.
    #:
    #: Set "mujoco+vbd" on a probe whose subject is a particle node
    #: (`Cloth` / `SoftBody`), which is what resources/nodes/Cloth.wrl and
    #: docs/developer/cloth-simulation.md tell an author to declare. Leave it
    #: at the default otherwise: a world with no deformable in it would pay
    #: for a second solver's setup and get nothing.
    #:
    #: ⚠ MEASURED 2026-08-16, AND IT CONTRADICTS THOSE TWO DOCS — READ THIS
    #: BEFORE YOU BUILD A DIAGNOSIS ON THEM. Both state that a `Cloth` ONLY
    #: simulates under "mujoco+vbd". On this engine that is NOT true: the
    #: runtime gates the coupled solver on `self.has_cloth()` — does the
    #: builder hold a particle source — and not on the declared value
    #: (omnisim_newton_runtime.py, the `if self.has_cloth():` branch beside
    #: `SolverMuJoCo(...)`). Only "vbd" (whole-world) takes a different
    #: branch; "mujoco" and "mujoco+vbd" resolve to the same `_force_mujoco`
    #: path and differ ONLY in the provenance label the sidecar prints.
    #:
    #: Verified as a negative control rather than by reading the code: the
    #: cloth probe's own world, edited to `newtonSolver "mujoco"` and run
    #: directly, still logged `registered 441 particles` and finalised on
    #: `MuJoCo (mujoco_warp, WorldInfo.newtonSolver) + VBD cloth via
    #: SolverCoupledProxy`. So this field is NOT what fixed the cloth
    #: verdict — see the probe's own comment for what was.
    solver: str = "mujoco"
    #: WorldInfo.coordinateSystem. Its own probe overrides it — the up-axis is
    #: a capability, not a formatting detail: before c77cbe98 every one of the
    #: 210 NUE worlds had gravity projected to zero and never fell.
    coordinate_system: str = "ENU"
    env: dict = field(default_factory=dict)         # extra child-process env
    assertion: Optional[Callable] = None            # arrays -> Verdict
    #: For KIND_STATIC: tokens that identify a diagnostic ABOUT THIS
    #: DECLARATION. The rule the runner applies is deliberately narrow — a
    #: probe is `absent` iff some ERROR/WARNING line from the engine's own log
    #: contains one of these tokens. Generic words ("unknown", "Skipped") are
    #: NOT usable as tokens: they match unrelated diagnostics and would let an
    #: unrelated warning certify a feature as absent. When nothing matches, the
    #: runner does NOT conclude "present and working" — it returns `degraded`
    #: with the full diagnostic list attached, because a silently-accepted
    #: declaration is precisely the case a static test cannot resolve alone.
    absent_markers: tuple = ()
    #: Substrings of ENGINE LOG lines this probe's assertion needs. Every
    #: matching line (at any severity — `absent_markers` sees only
    #: ERROR/WARNING) is handed to the assertion under the reserved recording
    #: key `engine_log`. Empty by default, so 43 of 44 probes pay nothing.
    #:
    #: Why this exists, and its limit. The particle nodes (`Cloth`,
    #: `SoftBody`, `GranularGroup`) have NO supervisor accessor for particle
    #: state, so an assertion restricted to the prober's recordings can say
    #: only "the node is in the scene tree" — which is equally true of a node
    #: that reached no solver at all. The engine's own registration line
    #: ("registered 441 particles (20 x 20 cells) at Newton particle offset 0")
    #: closes exactly that gap, and its count is CHECKABLE against the world's
    #: authored dimensions, so it is not a bare "no error appeared".
    #:
    #: ⚠ It is still an engine SELF-REPORT, not a measurement in physical
    #: units, and no verdict built on it may be published as `works`. It
    #: proves the node reached the solver; it says nothing about whether what
    #: the solver then did is right.
    log_capture: tuple = ()
    #: This probe's device needs the RENDERER to produce a reading. OmniBench
    #: runs headless with `--no-rendering`, which leaves a Camera or Lidar
    #: with no image pipeline: the controller blocks on a frame that never
    #: arrives and the engine free-runs (measured — a 250-step lidar probe
    #: reached 61,440 engine steps). Such probes drop `--no-rendering` and
    #: record it as a deviation, because a render-dependent sensor is exactly
    #: the kind of capability a "pure simulation" suite must not quietly
    #: claim it measured under conditions that cannot produce it.
    needs_rendering: bool = False
    #: Free-text pointer to where the capability is documented, so a drifting
    #: doc can be found from a failing row.
    doc: str = ""
    #: What AGENTS.md/docs currently CLAIM, as of the date in the comment.
    #: The runner raises a finding when the measurement disagrees — this lane
    #: audits the documentation as well as the engine.
    documented_as: Optional[str] = None


# ---------------------------------------------------------------------------
# small VRML helpers (kept local: lane 1's are shaped for its own scenes)
# ---------------------------------------------------------------------------
def _g(x):
    return "%.10g" % x


def floor(top_z=0.55, size=20.0, thickness=0.2):
    """A static box floor whose TOP FACE is at `top_z`.

    Deliberately above Newton's implicit z=0 ground plane: a probe that rests
    a body on z=0 cannot tell the authored floor from the phantom plane, and
    that ambiguity is what hid the statics-off defect for months.
    """
    cz = top_z - thickness / 2.0
    return """DEF FLOOR Solid {
  translation 0 0 %s
  name "floor"
  children [
    DEF FLOOR_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.45 0.47 0.5 roughness 1 metalness 0 }
      geometry Box { size %s %s %s }
    }
  ]
  boundingObject USE FLOOR_SHAPE
}
""" % (_g(cz), _g(size), _g(size), _g(thickness))


def body(defname, geometry, translation, mass=1.0, name=None,
         bounding=True, physics=True, color="0.85 0.35 0.25", extra=""):
    name = name or defname.lower()
    bo = "  boundingObject USE %s_SHAPE\n" % defname if bounding else ""
    ph = ("  physics Physics {\n    density -1\n    mass %s\n  }\n" % _g(mass)
          if physics else "")
    return """DEF %s Solid {
  translation %s
  name "%s"
  children [
    DEF %s_SHAPE Shape {
      appearance PBRAppearance { baseColor %s roughness 0.9 metalness 0 }
      geometry %s
    }
  ]
%s%s%s}
""" % (defname, translation, name, defname, color, geometry, bo, ph, extra)


#: Analytic rest height of the TouchSensor prober's centre: floor top 0.55 +
#: 0.11 (the pad protrudes 10 mm below the 0.2 m body). Shared by the rig and
#: both assertions so the reference pose can never drift apart from the check.
TOUCH_REST_Z = 0.66
#: What the prober rests at if ONLY the robot body collides and the
#: TouchSensor's own boundingObject never reaches the solver. The 10 mm gap
#: between the two is what makes the two hypotheses separable by measurement
#: instead of by argument.
BODY_REST_Z = 0.65


def _bumper_pad(mass, color, sensor_type="bumper"):
    """Prober-body children for the TouchSensor probes: a 0.2 m box body plus
    a thin sensing pad on its underside.

    The pad PROTRUDES 10 mm below the body (pad underside -0.11, body
    underside -0.10), so the pad is the only geometry that can reach the
    floor and the analytic rest height is 0.66 m. Both details were learned
    the hard way and both are load-bearing: hanging the pad 0.15 m below the
    origin settled the prober at 0.5506 m -- neither predicted height -- and
    making the two surfaces flush let the ROBOT BODY take the contact, which
    would have scored a working bumper `broken` for never seeing a touch that
    was landing on something else.

    The caller must also declare `newtonRobotColliders TRUE`: a Robot's own
    boundingObject is NOT a Newton collider by default (worldinfo.md -- the
    default is wheel/foot-only collision so a chassis envelope cannot pin the
    body), which is the other half of why the first rig floated.
    """
    # ⚠ THE FORCE TYPE NEEDS THREE THINGS THE BUMPER DOES NOT, AND OMITTING ANY
    # ONE OF THEM READS 0 N FROM A WORKING DEVICE. This probe asserted a bare
    # `physics NULL` pad for both types and therefore certified a working force
    # sensor as `broken` from 2026-08-13 (when ee069b326 fixed the collider
    # precondition) until this was corrected. Measured on this exact geometry,
    # restoring one field at a time:
    #
    #     as it was authored here .....................     0.0         N
    #     + physics + lookupTable [], no rotation .....     8.2e-16     N
    #     + rotation 0 1 0 1.5708 .....................    19.620000839 N  truth
    #     + the SHIPPED default lookupTable ...........   196.200008392 N
    #
    #   physics      the FORCE type reports the MOUNT WRENCH, served from
    #                mjData.cfrc_int -- a pad with no body has no cfrc_int and
    #                so nothing to report. The BUMPER needs no body: its
    #                precondition is a collider, which `boundingObject` alone
    #                now supplies, and it is measured working with physics NULL.
    #   rotation     the device is DEFINED as the projection of that wrench onto
    #                its own +X axis, keeping only a push. Unrotated, +X is
    #                horizontal and a vertical load projects to nothing.
    #   lookupTable  the shipped default [ 0 0 0, 5000 50000 0 ] is a 10x GAIN,
    #                so an uncleared table reports a number that is not newtons.
    # ⚠ AND THE ROTATION ROTATES THE PAD'S GEOMETRY WITH IT, so the box must be
    # counter-sized or the collider changes shape and the rest height moves.
    # Measured: rotating the shipped 0.2 x 0.2 x 0.025 pad 90 deg about Y makes
    # it 0.025 x 0.2 x 0.2 in world -- 0.2 m THICK -- and the prober settled at
    # z=0.7473 against the 0.66 this probe asserts, which the rest-height arm
    # correctly refused to score. Authoring it 0.025 x 0.2 x 0.2 in LOCAL space
    # maps back to 0.2 x 0.2 x 0.025 in world (a +90 deg Y rotation swaps local
    # x and z), so both types present an identical collider to the floor and the
    # 0.66 analytic rest height holds for both.
    is_force = sensor_type != "bumper"
    extra = ""
    pad_size = "0.2 0.2 0.025"
    physics = "      physics NULL\n"
    if is_force:
        extra = "      rotation 0 1 0 1.5708\n"
        pad_size = "0.025 0.2 0.2"
        physics = ("      physics Physics { density -1 mass 0.001 }\n"
                   "      lookupTable [ ]\n")
    return """    DEF PROBER_BODY Shape {
      appearance PBRAppearance { baseColor %s roughness 1 metalness 0 }
      geometry Box { size 0.2 0.2 0.2 }
    }
    TouchSensor {
      translation 0 0 -0.0975
%s      name "ts"
      type "%s"
      children [
        DEF PAD_SHAPE Shape {
          appearance PBRAppearance { baseColor 0.2 0.2 0.25 roughness 1 metalness 0 }
          geometry Box { size %s }
        }
      ]
      boundingObject USE PAD_SHAPE
%s    }
    # carried load: %s kg -> %.2f N of weight on the pad at rest
    # pad underside z=-0.11 protrudes below body underside z=-0.10
    #   -> only the pad reaches the floor -> rest z = %s
""" % (color, extra, sensor_type, pad_size, physics, _g(mass), mass * 9.81, _g(TOUCH_REST_Z))


# ---------------------------------------------------------------------------
# assertion helpers
# ---------------------------------------------------------------------------
def _finite(seq):
    """Scalar series -> the finite floats in it.

    Every enable()-gated device reads NaN until its first post-enable sample
    lands, and the recorder's row 0 is taken BEFORE the first step. Letting
    that row through poisons max()/mean() and reports a working sensor as
    'reads nan' -- an instrument artefact that is indistinguishable from a
    dead device unless it is filtered here, once, for every probe.
    """
    if not seq:
        return []
    out = []
    for v in seq:
        if v is None or isinstance(v, (list, tuple, dict)):
            continue
        try:
            f = float(v)
        except (TypeError, ValueError):
            continue
        if f == f and abs(f) != float("inf"):
            out.append(f)
    return out


def _final(arrays, key, idx=None):
    a = arrays.get(key)
    if a is None or len(a) == 0:
        return None
    v = a[-1]
    return float(v[idx]) if idx is not None else v


def _travel(arrays, key, idx):
    a = arrays.get(key)
    if a is None or len(a) < 2:
        return None
    return float(a[-1][idx] - a[0][idx])


def _span(arrays, key):
    """max - min over the FINITE samples of a scalar series (how much did this
    quantity move at all). Filtered, because a joint whose `position` field is
    unreadable records None and a sensor records NaN before its first enabled
    sample -- both would otherwise propagate into every travel figure."""
    a = _finite(arrays.get(key))
    if not a:
        return None
    return max(a) - min(a)


def _finite_vecs(seq, dim=3):
    """Vector series -> the finite `dim`-vectors in it.

    `_finite` deliberately DROPS lists (it is the scalar filter), so a 3-axis
    device that reuses it silently measures an empty series and its probe
    reports `inconclusive` about a working sensor. The NaN-before-the-first-
    enabled-sample rule (rule 5) applies identically here: a Gyro, GPS or
    InertialUnit reads NaN until its first post-enable sample lands, and the
    recorder's row 0 is taken before the first step.
    """
    out = []
    for v in (seq or []):
        if v is None or isinstance(v, dict) or not hasattr(v, "__len__"):
            continue
        if len(v) < dim:
            continue
        try:
            f = [float(x) for x in list(v)[:dim]]
        except (TypeError, ValueError):
            continue
        if all(x == x and abs(x) != float("inf") for x in f):
            out.append(f)
    return out


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


def _quat_rotation_total(seq):
    """Total rotation angle (rad) swept by a recorded orientation series.

    Accumulated across CONSECUTIVE samples rather than taken end-to-end, for
    the same reason `joint.hinge_motor_velocity` refuses to trust the joint's
    `position` field: an end-to-end angle saturates at pi and silently reports
    a turntable that has gone round twice as one that barely moved. Each step
    is a few milliradians, so the per-step arccos is nowhere near its
    ill-conditioned region.

    This is the SUPERVISOR's own witness of the rotation -- it comes from
    Node.getOrientation(), not from the device under test and not from the
    commanded rate -- which is what lets a sensor assertion be checked against
    the motion that actually happened rather than the motion that was asked
    for.
    """
    tot, n, prev = 0.0, 0, None
    for q in _finite_vecs(seq, 4):
        if prev is not None:
            d = abs(sum(a * b for a, b in zip(prev, q)))
            tot += 2.0 * math.acos(min(1.0, max(0.0, d)))
            n += 1
        prev = q
    return tot, n


def _path_len(seq):
    """Total distance travelled by a recorded position series.

    The discriminator a single-sample position comparison cannot make: a
    readout FROZEN at the body's spawn pose agrees with the supervisor exactly
    once (at t=0) and has a path length of zero, while the body it is bolted
    to has a path length of metres.
    """
    pts = _finite_vecs(seq, 3)
    if len(pts) < 2:
        return None
    return sum(math.dist(tuple(a), tuple(b)) for a, b in zip(pts, pts[1:]))


def _unwrapped_travel(seq, idx, dim=3):
    """Total travel of a WRAPPED angle channel (rad).

    An InertialUnit's yaw lives in (-pi, pi], so a turntable that goes round
    once shows a 2*pi discontinuity and an end-to-end difference reports a body
    that swept 4 rad as one that swept 0.28. Every per-step change here is
    milliradians, so any jump above pi is the wrap and is corrected as one.
    """
    vals = [v[idx] for v in _finite_vecs(seq, dim)]
    if len(vals) < 2:
        return None
    tot = 0.0
    for a, b in zip(vals, vals[1:]):
        d = b - a
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        tot += abs(d)
    return tot


def _paired(arrays, sensor_key, pos_key):
    """Index-aligned (sensor_xyz, supervisor_xyz) pairs.

    Both series are appended in the SAME record() call, so index k is one
    instant; pairs whose sensor sample is not yet a finite vector (the
    pre-enable NaN rows) are dropped rather than compared against a pose that
    is perfectly valid.
    """
    s, p = arrays.get(sensor_key), arrays.get(pos_key)
    if s is None or p is None:
        return []
    out = []
    for sv, pv in zip(s, p):
        a = _finite_vecs([sv], 3)
        b = _finite_vecs([pv], 3)
        if a and b:
            out.append((a[0], b[0]))
    return out


def _rest_z(top_z, radius_or_half):
    return top_z + radius_or_half


def _accel_along(arrays, key, axis, lo=1.0 / 3.0, hi=1.0):
    """Constant acceleration (m/s^2) along `axis`, from three EVENLY SPACED
    recorded positions between fractions `lo` and `hi` of the run.

    a = (z2 - 2*z1 + z0) / h^2 is exact for a constant acceleration and, unlike
    2*dz/dt^2, it does NOT assume the body started from rest. That matters for
    every probe that turns an actuator on partway through the run: the body
    already carries a velocity when the window opens, and the naive form folds
    that velocity into the acceleration and reports a thrust that is not there.

    Returns (a, window) with `window` naming the three sample times, so an
    evidence dict can show WHICH part of the run a number came from.
    """
    p = _finite_vecs(arrays.get(key), 3)
    t = _finite(arrays.get("t"))
    if len(p) < 3 or len(t) != len(p):
        return None, None
    last = len(p) - 1
    i0 = max(0, min(last - 2, int(round(lo * last))))
    i2 = max(i0 + 2, min(last, int(round(hi * last))))
    if (i2 - i0) % 2:
        i0 += 1
    k = (i2 - i0) // 2
    if k < 1:
        return None, None
    i1 = i0 + k
    h = float(t[i1]) - float(t[i0])
    if h <= 0.0:
        return None, None
    a = (p[i2][axis] - 2.0 * p[i1][axis] + p[i0][axis]) / (h * h)
    return a, {"t0_s": float(t[i0]), "t1_s": float(t[i1]),
               "t2_s": float(t[i2]), "samples": len(p)}


def _speed_along(arrays, key, axis, frac):
    """Signed speed (m/s) along `axis` at fraction `frac` of the run, from a
    central difference over +/- 1% of the samples.

    Exists so an aerodynamic assertion can publish the AIRSPEED its two windows
    actually saw instead of a hard-coded 0.0. `device.propeller_inflow`'s note
    printed a literal "0.000 m/s (early)" for the low-speed window while the
    airframe was in fact already falling at 1.5 m/s there -- a number in a note
    that nothing measured, in a probe whose entire claim is about speed.
    """
    p = _finite_vecs(arrays.get(key), 3)
    t = _finite(arrays.get("t"))
    if len(p) < 3 or len(t) != len(p):
        return None
    last = len(p) - 1
    w = max(1, int(round(0.01 * last)))
    i = max(w, min(last - w, int(round(frac * last))))
    dt = float(t[i + w]) - float(t[i - w])
    if dt <= 0.0:
        return None
    return (p[i + w][axis] - p[i - w][axis]) / dt


#: "[OmNewtonBackend] registered 3 dynamic + 1 static Newton bodies ..." --
#: OmSolid.cpp's own registration report, captured via `log_capture`.
_RE_NEWTON_DYNAMIC = re.compile(r"registered\s+(\d+)\s+dynamic")


def _newton_dynamic_bodies(arrays):
    """How many DYNAMIC bodies the engine says it handed to Newton, or None if
    the line was not captured.

    The premise guard for every probe whose subject must be a free rigid body.
    It is worth having as a MEASURED number rather than an assumption because
    the naive rigs are wrong in a way nothing else reveals: a Solid nested
    inside a Robot, carrying its own `physics` and `boundingObject`, is NOT a
    body of its own -- it is merged into its parent's. Measured on exactly that
    scene, the engine reported `registered 2 dynamic` for a world holding a
    Robot, a nested Solid and one free Solid. A probe that mounted its subject
    there and then reported "the force did nothing" would be reporting its own
    rig.
    """
    best = None
    for line in (arrays.get("engine_log") or []):
        m = _RE_NEWTON_DYNAMIC.search(line)
        if m:
            n = int(m.group(1))
            best = n if best is None else max(best, n)
    return best


def _log_hits(arrays, needle, limit=2):
    return [l for l in (arrays.get("engine_log") or []) if needle in l][:limit]


# ---------------------------------------------------------------------------
# particle-stats helpers (the 2026-09-01 Node.getParticleStats readback)
# ---------------------------------------------------------------------------
# The three particle nodes (Cloth / SoftBody / GranularBed, plus the CUDA
# GranularGroup) had NO supervisor accessor for particle state, which is why
# their probes were capped at `degraded` on an engine self-report. The
# `particles:DEF` measure spec closes that gap: one stats frame per recorded
# step, {status, count, min[3], max[3], centroid[3], non_finite}. These
# helpers keep the INCONCLUSIVE discipline in ONE place: a readback that is
# missing (stale libController), refuses (status != 0) or allocates nothing
# (count == 0) is an environment/instrument condition and must NEVER be
# published as `broken` -- per the prober's standing robustness contract.

def _particle_frames(arrays, defname):
    """(ok, statuses, dicts) for a particles:DEF series.

    `ok` is the index-aligned [(i, frame)] list of frames whose status is 0
    (index i maps into the recording's `t` array, both appended in the same
    record() call); `statuses` is every status seen; `dicts` counts frames
    that were dicts at all (0 == the binding never produced a frame).
    """
    ok, statuses, dicts = [], [], 0
    for i, d in enumerate(arrays.get("particles_%s" % defname) or []):
        if not isinstance(d, dict):
            continue
        dicts += 1
        s = d.get("status")
        try:
            s = int(s)
        except (TypeError, ValueError):
            s = None
        if s is not None:
            statuses.append(s)
        if s == 0:
            ok.append((i, d))
    return ok, statuses, dicts


def _particle_triage(ok, statuses, dicts, label, cuda_absent=False):
    """None when the series is measurable; else the Verdict its failure
    demands. `cuda_absent=True` maps status -5 (GranularGroup CUDA-inert) to
    ABSENT -- an honest scope statement about a build without engine CUDA --
    which only the GranularGroup probe opts into; everywhere else every
    refusal is an environment condition and scores `inconclusive`."""
    if ok:
        return None
    ev = {"frames_with_dict": dicts,
          "statuses_seen": sorted(set(statuses))[:8]}
    if dicts == 0:
        return Verdict(
            INCONCLUSIVE, ev,
            "%s: no getParticleStats frame was recorded -- the binding is "
            "missing from this libController (stale libController; see "
            "meta.problems) or the node was never found. Instrument failure, "
            "not a capability verdict." % label)
    if -9 in statuses:
        return Verdict(
            INCONCLUSIVE, ev,
            "%s: getParticleStats reports status -9 (stale libController) -- "
            "instrument failure, not a capability verdict" % label)
    if cuda_absent and -5 in statuses:
        return Verdict(
            ABSENT, ev,
            "%s: getParticleStats reports status -5 (GranularGroup CUDA-"
            "inert) -- requires engine CUDA; this build reports it "
            "unavailable" % label)
    return Verdict(
        INCONCLUSIVE, ev,
        "%s: getParticleStats never returned status 0 (statuses seen: %s) -- "
        "the readback refused, which is an environment/instrument condition, "
        "never `broken`" % (label, sorted(set(statuses))))


def _pnum(frame, key, axis=None):
    """One finite float out of a stats frame, or None on any malformation."""
    try:
        v = frame[key]
        if axis is not None:
            v = v[axis]
        f = float(v)
    except (KeyError, TypeError, ValueError, IndexError):
        return None
    if f != f or abs(f) == float("inf"):
        return None
    return f


def _pext(frame, axis):
    """max - min extent of a stats frame along `axis`, or None."""
    hi = _pnum(frame, "max", axis)
    lo = _pnum(frame, "min", axis)
    if hi is None or lo is None:
        return None
    return hi - lo


def _particle_nonfinite(ok):
    """Peak non_finite count across the status-0 frames (0 when clean)."""
    worst = 0
    for _, f in ok:
        v = _pnum(f, "non_finite")
        if v is not None:
            worst = max(worst, int(v))
    return worst


def _particle_arrest(ok, t, window_s=0.5):
    """Centroid-z span (m) over the final `window_s` of status-0 frames --
    the 'motion arrests' number -- or None when the window is unpopulated."""
    if not ok or not t:
        return None
    try:
        t_end = float(t[ok[-1][0]])
    except (IndexError, TypeError, ValueError):
        return None
    zs = []
    for i, f in ok:
        try:
            ti = float(t[i])
        except (IndexError, TypeError, ValueError):
            continue
        if ti >= t_end - window_s:
            z = _pnum(f, "centroid", 2)
            if z is not None:
                zs.append(z)
    if len(zs) < 2:
        return None
    return max(zs) - min(zs)


PROBES: list[Probe] = []


def _p(probe):
    PROBES.append(probe)
    return probe


# ===========================================================================
# FAMILY: object / geometry — what SHAPES can carry physics?
# ===========================================================================

def _drop_assertion(rest_z, tol=0.005, label="body"):
    """Build a 'falls and settles at the analytic rest height' assertion."""
    def _a(arrays):
        z = _final(arrays, "pos_SUBJECT", 2)
        if z is None:
            return Verdict(INCONCLUSIVE, note="no pose recorded for SUBJECT")
        err = abs(z - rest_z)
        ev = {"rest_z_m": z, "expected_rest_z_m": rest_z, "abs_err_m": err}
        if err <= tol:
            return Verdict(WORKS, ev)
        # Distinguish "fell through" (the collider is a hologram) from
        # "landed in the wrong place" (a real but inaccurate collision).
        if z < rest_z - 0.5:
            return Verdict(
                BROKEN, ev,
                "%s passed THROUGH the collider and reached z=%.4f m; the "
                "geometry is not colliding at all" % (label, z))
        return Verdict(
            DEGRADED, ev,
            "%s settled %.1f mm from the analytic rest height" % (label, err * 1e3))
    _a.__doc__ = (
        "A body dropped onto a floor whose top face is at z=0.55 m must come "
        "to rest with its centre at z=%.4f m (+/- %d mm). Resting far below "
        "that means the collider is a hologram." % (rest_z, int(tol * 1e3)))
    return _a


_p(Probe(
    id="object.rigid_box",
    family=FAM_OBJECT,
    claim="Box primitive as a dynamic, colliding rigid body",
    world=lambda: floor() + body("SUBJECT", "Box { size 0.2 0.2 0.2 }",
                                 "0 0 1.2"),
    measure=("pos:SUBJECT",),
    duration=3.0,
    assertion=_drop_assertion(_rest_z(0.55, 0.1), label="the box"),
    doc="docs/reference/",
))

_p(Probe(
    id="object.rigid_sphere",
    family=FAM_OBJECT,
    claim="Sphere primitive as a dynamic, colliding rigid body",
    world=lambda: floor() + body("SUBJECT", "Sphere { radius 0.1 subdivision 3 }",
                                 "0 0 1.2"),
    measure=("pos:SUBJECT",),
    duration=3.0,
    assertion=_drop_assertion(_rest_z(0.55, 0.1), label="the sphere"),
))

_p(Probe(
    id="object.rigid_cylinder",
    family=FAM_OBJECT,
    claim="Cylinder primitive as a dynamic, colliding rigid body",
    # Authored UNROTATED on purpose. The first version rotated it 90 deg about
    # x to "lay it on its side", which silently assumed an axis convention;
    # the probe then failed by 149.6 mm and the failure was unattributable
    # between the convention, the collider and the engine. A capability probe
    # must not encode a guess about the thing it is testing.
    world=lambda: floor() + body(
        "SUBJECT", "Cylinder { radius 0.1 height 0.3 }", "0 0 1.2"),
    measure=("pos:SUBJECT",),
    duration=3.0,
    assertion=None,
))


def _cylinder_assertion(arrays):
    """A cylinder r=0.1, h=0.3 dropped on a floor whose top is at z=0.55 has
    exactly two physically admissible rest heights: 0.65 m lying on its side
    (rest = radius) or 0.70 m standing on an end face (rest = half-height).
    Which one it picks depends on the axis convention and is not the point —
    the probe accepts EITHER and reports which. Any other height means the
    collider does not match the authored geometry; measured 0.7996 m on the
    first run, i.e. 0.25 m of half-extent for a body whose largest half-extent
    is 0.15, which is what a radius-sized end cap would add."""
    z = _final(arrays, "pos_SUBJECT", 2)
    if z is None:
        return Verdict(INCONCLUSIVE, note="no pose recorded for SUBJECT")
    on_side, on_end = 0.65, 0.70
    ev = {"rest_z_m": z, "admissible_on_side_m": on_side,
          "admissible_on_end_m": on_end,
          "half_extent_above_floor_m": z - 0.55}
    if abs(z - on_side) <= 0.01:
        ev["settled_as"] = "on its side (rest = radius)"
        return Verdict(WORKS, ev)
    if abs(z - on_end) <= 0.01:
        ev["settled_as"] = "on an end face (rest = half-height)"
        return Verdict(WORKS, ev)
    if z < 0.3:
        return Verdict(BROKEN, ev,
                       "the cylinder passed through the floor and reached "
                       "z=%.4f m" % z)
    return Verdict(
        DEGRADED, ev,
        "the cylinder rests %.1f mm above the floor, which is neither its "
        "radius (100 mm) nor its half-height (150 mm) -- the collider does "
        "not match the authored geometry" % ((z - 0.55) * 1e3))


PROBES[-1].assertion = _cylinder_assertion

_p(Probe(
    id="object.rigid_capsule",
    family=FAM_OBJECT,
    claim="Capsule primitive as a dynamic, colliding rigid body",
    world=lambda: floor() + body(
        "SUBJECT", "Capsule { radius 0.1 height 0.3 }", "0 0 1.2",
        extra="  rotation 1 0 0 1.5707963\n"),
    measure=("pos:SUBJECT",),
    duration=3.0,
    assertion=_drop_assertion(_rest_z(0.55, 0.1), tol=0.01,
                              label="the capsule"),
))

_p(Probe(
    id="object.mesh_indexedfaceset",
    family=FAM_OBJECT,
    claim="Arbitrary triangle mesh (IndexedFaceSet) as a collider",
    # A unit tetrahedron-ish wedge, 0.2 m tall, authored so its lowest vertex
    # sits 0.1 m below the node origin -> analytic rest z = 0.55 + 0.1.
    world=lambda: floor() + body(
        "SUBJECT",
        """IndexedFaceSet {
        coord Coordinate { point [ -0.1 -0.1 -0.1, 0.1 -0.1 -0.1,
                                    0.1  0.1 -0.1, -0.1  0.1 -0.1,
                                    0 0 0.1 ] }
        coordIndex [ 0 3 2 1 -1, 0 1 4 -1, 1 2 4 -1, 2 3 4 -1, 3 0 4 -1 ]
      }""",
        "0 0 1.2"),
    measure=("pos:SUBJECT",),
    duration=3.0,
    assertion=_drop_assertion(_rest_z(0.55, 0.1), tol=0.02,
                              label="the mesh body"),
))

_p(Probe(
    id="object.static_collider",
    family=FAM_OBJECT,
    claim="A static Solid (no physics) is solid and stops dynamic bodies",
    # This is the 2026-08-07 newtonStatics default flip, pinned as a
    # capability rather than only as a regression test.
    world=lambda: floor(top_z=0.55) + body("SUBJECT",
                                           "Sphere { radius 0.1 subdivision 3 }",
                                           "0 0 1.2"),
    measure=("pos:SUBJECT",),
    duration=4.0,
    assertion=_drop_assertion(_rest_z(0.55, 0.1), label="the sphere"),
    doc="AGENTS.md — 'STATIC FLOORS ARE SOLID UNDER NEWTON BY DEFAULT'",
    documented_as=WORKS,
))

_p(Probe(
    id="object.no_boundingobject_is_intangible",
    family=FAM_OBJECT,
    claim="A Solid WITHOUT boundingObject does not collide (documented "
          "behaviour — the C2 fall-through trap)",
    world=lambda: (
        # floor with visual geometry only: no boundingObject
        """DEF FLOOR Solid {
  translation 0 0 0.45
  name "floor"
  children [
    DEF FLOOR_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.45 0.47 0.5 roughness 1 metalness 0 }
      geometry Box { size 20 20 0.2 }
    }
  ]
}
""" + body("SUBJECT", "Sphere { radius 0.1 subdivision 3 }", "0 0 1.2")),
    measure=("pos:SUBJECT",),
    duration=3.0,
    assertion=None,  # set below (inverted sense)
    doc="AGENTS.md §3b — a bare run-headless PASS cannot see this",
))


def _intangible_assertion(arrays):
    """A Solid with no boundingObject must NOT collide: a sphere dropped onto
    it must pass through, i.e. end well below the visual surface it would
    otherwise rest on (0.65 m).

    ⚠ WHERE IT ENDS UP DEPENDS ON THE BUILD, which is why the threshold is a
    loose z<0.3 and not an equality. Before 2026-08-12 Newton added an implicit
    ground plane at z=0 unconditionally, so the sphere stopped at z=0.0996 (its
    own radius) — and the first draft of this assertion demanded z<0 and duly
    reported that phantom plane as an engine defect. Since 2026-08-12 the plane
    is added only to substitute for a dropped authored `Plane` collider, and
    this world declares none, so the sphere free-falls instead. BOTH outcomes
    satisfy z<0.3 and both are correct answers to the question this probe
    asks — which is about the intangible floor, not about the plane.
    `phenomenon.implicit_ground_plane` is the row that tracks the plane
    itself."""
    z = _final(arrays, "pos_SUBJECT", 2)
    if z is None:
        return Verdict(INCONCLUSIVE, note="no pose recorded for SUBJECT")
    ev = {"final_z_m": z, "visual_surface_rest_z_m": 0.65,
          "rest_z_if_unconditional_implicit_plane": 0.1,
          "implicit_plane_unconditional_before": "2026-08-12"}
    if z < 0.3:
        return Verdict(WORKS, ev)
    return Verdict(
        BROKEN, ev,
        "the sphere stopped at z=%.4f m on a floor that declares NO "
        "boundingObject — the engine collided against geometry the world "
        "never made collidable" % z)


PROBES[-1].assertion = _intangible_assertion

_p(Probe(
    id="object.elevationgrid_terrain",
    family=FAM_OBJECT,
    claim="ElevationGrid heightfield as a collider (terrain)",
    world=lambda: (
        """DEF TERRAIN Solid {
  translation -1 -1 0.55
  name "terrain"
  children [
    DEF TERRAIN_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.4 0.5 0.35 roughness 1 metalness 0 }
      geometry ElevationGrid {
        xDimension 3
        yDimension 3
        xSpacing 1
        ySpacing 1
        height [ 0 0 0 0 0 0 0 0 0 ]
      }
    }
  ]
  boundingObject USE TERRAIN_SHAPE
}
""" + body("SUBJECT", "Sphere { radius 0.1 subdivision 3 }", "0 0 1.2")),
    measure=("pos:SUBJECT",),
    duration=3.0,
    assertion=_drop_assertion(_rest_z(0.55, 0.1), tol=0.02,
                              label="the sphere on the heightfield"),
))

_p(Probe(
    id="object.granular_group",
    family=FAM_OBJECT,
    claim="GranularGroup — bulk particulate media (sand/gravel, CUDA kernel)",
    # src/omnisim/nodes/OmGranularGroup.cpp — a self-contained CUDA particle
    # system, NOT a Newton solver client: it seeds its own particles (up-axis
    # in [1.5, 2.0] m, seedInitialState's baseUp 1.5 + spread 0.5 for this
    # count/radius) and clamps them against its OWN box walls whose floor is
    # up = 0 — NOT against the rigid scene, so the rigid floor at 0.55 is
    # irrelevant to the particles and the reference sphere is parked at
    # x = -2, clear of the ±0.5 m seed footprint, to measure the rigid scene
    # alone.
    # ⚠ THESE FIELD NAMES ARE THE SCHEMA'S, AND THEY WERE WRONG TWICE.
    # 2026-08-15: this probe declared `particleCount` / `particleRadius`;
    # GranularGroup.wrl has `count` / `radius`. 2026-09-01: it still declared
    # `translation`, which GranularGroup (not a Solid) has NEVER had — the
    # shipped coverage row carries the resulting "Skipped unknown
    # 'translation' field" ERROR, i.e. the world was again failing on its own
    # authoring, the same defect class both times. An undeclared field is an
    # ERROR that takes a headless exit code to 1.
    # ⚠ THE ROW WAS NEVER READBACK-BLOCKED. The old verdict said "no way to
    # read particle state"; the shipped row's own diagnostics carry the real
    # scope statement — "GranularGroup is inert: CUDA is not available on
    # this build/box" — which is `absent` (a build-scope fact), not
    # `degraded` with a readback excuse. With engine CUDA present the
    # 2026-09-01 getParticleStats readback measures the settle directly.
    world=lambda: floor() + """DEF SUBJECT_GRAINS GranularGroup {
  count 64
  radius 0.02
}
""" + body("SUBJECT", "Sphere { radius 0.1 subdivision 3 }", "-2 0 1.2"),
    measure=("pos:SUBJECT", "node_exists:SUBJECT_GRAINS",
             "particles:SUBJECT_GRAINS"),
    log_capture=("GranularGroup is inert",),
    duration=4.0,
    assertion=None,   # set below
    doc="src/omnisim/nodes/OmGranularGroup.cpp + "
        "resources/nodes/GranularGroup.wrl (CUDA-inert contract)",
    # What the shipped matrix claims as of 2026-09-01. On a no-CUDA build this
    # probe now measures `absent` (the engine's own inert line names the
    # scope), so the audit will flag the flip and the doc reconciliation
    # happens AFTER the measurement — the round-2 protocol.
    documented_as=WORKS,  # flipped 2026-09-01 round 3: engine CUDA present on this build; particles simulate and settle, measured via the particle-stats verb
))


def _granular_assertion(arrays):
    """A 64-particle 0.02 m GranularGroup must SETTLE, measured through
    getParticleStats: the node seeds its particles at up = 1.5–2.0 m (its own
    seeding rule) above its OWN floor at up = 0, so the centroid must drop by
    at least 1.0 m, the z-extent must collapse from the ~0.5 m seed column to
    under 0.3 m (and under half its initial value), the motion must arrest
    (centroid-z span <= 10 mm over the final 0.5 s), every particle must stay
    finite, and the reference sphere parked 2 m away must still rest at the
    rigid 0.65 m. On a build without engine CUDA the node is documented-inert
    and the verdict is `absent` — 'requires engine CUDA; this build reports
    it unavailable' is a scope statement, never `broken`. A missing or
    refusing readback (no frames, status != 0, count == 0) is an instrument
    condition and scores `inconclusive`."""
    exists = arrays.get("node_exists_SUBJECT_GRAINS")
    z = _final(arrays, "pos_SUBJECT", 2)
    inert = _log_hits(arrays, "GranularGroup is inert")
    ev = {"granular_node_in_scene_tree": bool(exists),
          "reference_sphere_rest_z_m": z,
          "cuda_inert_lines": inert}
    if not exists:
        return Verdict(
            ABSENT, ev,
            "GranularGroup did not survive into the scene tree — the node is "
            "not usable from a .wbt even though the C++ class exists")
    if inert:
        return Verdict(
            ABSENT, ev,
            "requires engine CUDA; this build reports it unavailable — the "
            "engine's own line: %s" % inert[0][:180])
    ok, statuses, dicts = _particle_frames(arrays, "SUBJECT_GRAINS")
    bad = _particle_triage(ok, statuses, dicts, "GranularGroup",
                           cuda_absent=True)
    if bad is not None:
        bad.evidence.update(ev)
        return bad
    first, last = ok[0][1], ok[-1][1]
    cnt = _pnum(last, "count")
    c0, c1 = _pnum(first, "centroid", 2), _pnum(last, "centroid", 2)
    e0, e1 = _pext(first, 2), _pext(last, 2)
    arrest = _particle_arrest(ok, arrays.get("t"))
    nf = _particle_nonfinite(ok)
    ev.update({"particle_count": cnt, "centroid_z_first_m": c0,
               "centroid_z_final_m": c1, "z_extent_first_m": e0,
               "z_extent_final_m": e1, "arrest_span_m": arrest,
               "non_finite_peak": nf, "status0_frames": len(ok)})
    if None in (cnt, c0, c1, e0, e1):
        return Verdict(INCONCLUSIVE, ev,
                       "getParticleStats frames are malformed (a stats field "
                       "is missing or non-finite) — instrument failure")
    if cnt == 0:
        return Verdict(INCONCLUSIVE, ev,
                       "getParticleStats reports 0 particles at status 0 — "
                       "nothing allocated; environment, not a capability "
                       "verdict")
    if cnt != 64:
        return Verdict(DEGRADED, ev,
                       "the group allocated %d particles, not the authored "
                       "count 64" % int(cnt))
    if nf > 0:
        return Verdict(DEGRADED, ev,
                       "%d particle position(s) went non-finite during the "
                       "run" % nf)
    if c0 - c1 < 1.0:
        if c0 - c1 < 0.05:
            return Verdict(BROKEN, ev,
                           "the particles allocated and their centroid never "
                           "fell (%.3f m over the run) — the kernel is not "
                           "integrating" % (c0 - c1))
        return Verdict(DEGRADED, ev,
                       "centroid dropped only %.3f m against the >= 1.0 m the "
                       "1.5–2.0 m seed above the up=0 floor demands"
                       % (c0 - c1))
    if not (e1 < 0.3 and e1 < 0.5 * e0):
        return Verdict(DEGRADED, ev,
                       "z-extent ended at %.3f m (started %.3f) — the seed "
                       "column never collapsed into a pile" % (e1, e0))
    if arrest is None or arrest > 0.01:
        return Verdict(DEGRADED, ev,
                       "centroid still moving %.1f mm over the final 0.5 s — "
                       "the pile has not arrested"
                       % ((arrest or float("nan")) * 1e3))
    if z is None:
        return Verdict(INCONCLUSIVE, ev, "reference sphere not recorded")
    if abs(z - 0.65) > 0.02:
        return Verdict(DEGRADED, ev,
                       "the reference sphere rests at z=%.4f m instead of "
                       "0.6500 — declaring the group perturbed the rigid "
                       "scene" % z)
    return Verdict(WORKS, ev)


PROBES[-1].assertion = _granular_assertion

# --- absent-by-schema probes (static): the engine must REFUSE these ---------
_p(Probe(
    id="object.fluid_buoyancy",
    family=FAM_OBJECT,
    claim="Fluid volumes / buoyancy / drag (Fluid + ImmersionProperties)",
    kind=KIND_STATIC,
    world=lambda: floor() + """DEF POOL Fluid {
  translation 0 0 0.3
  name "pool"
  density 1000
}
""" + body("SUBJECT", "Sphere { radius 0.1 subdivision 3 }", "0 0 1.2"),
    absent_markers=("Fluid",),
    doc="AGENTS.md — Fluid/ImmersionProperties removed with ODE (bdc02139)",
    documented_as=ABSENT,
))

_p(Probe(
    id="object.immersion_properties",
    family=FAM_OBJECT,
    claim="Solid.immersionProperties (per-body buoyancy/drag coefficients)",
    kind=KIND_STATIC,
    world=lambda: floor() + body(
        "SUBJECT", "Sphere { radius 0.1 subdivision 3 }", "0 0 1.2",
        extra="  immersionProperties [ ]\n"),
    absent_markers=("immersionProperties",),
    doc="AGENTS.md — the FIELD is gone from the schema, so it ERRORs (unlike "
        "the ODE tuning fields, which degrade quietly)",
    documented_as=ABSENT,
))

_p(Probe(
    id="object.deformable_cloth",
    family=FAM_OBJECT,
    claim="Deformable surfaces (cloth / membranes)",
    # ⚠ THIS PROBE PUBLISHED `absent` UNTIL 2026-08-16 AND THE ENGINE WAS NEVER
    # AT FAULT. 17 cloth worlds ship and a gripper picks up a T-shirt to
    # -1.50 mm of tracking error, while this lane published "OmniSim cannot do
    # cloth" -- and AGENTS.md's own rule is that the matrix wins over prose.
    #
    # THE SOLE CAUSE, and it is worth being exact because a plausible second
    # cause was investigated and MEASURED NOT TO BE ONE (below):
    #
    #   The probe declared `size 0.5 0.5`. `size` has NEVER been a field of
    #   `Cloth` (resources/nodes/Cloth.wrl: dimX/dimY/cellX/cellY/mass/triKe/
    #   ...), so the engine logged `Skipped unknown 'size' field in Cloth
    #   node` -- and the probe's `absent_markers=("Cloth",)` matched THAT and
    #   read a complaint about a FIELD as proof the NODE does not exist. The
    #   two diagnostics are not the same fact: a genuinely absent node reads
    #   `Missing declaration for 'Fluid', unknown node` (object.fluid_buoyancy,
    #   which is honestly absent). A marker broad enough to match a field
    #   complaint cannot tell a mis-authored field from a missing node --
    #   exactly the conflation `classify_static` warns about. The old row's
    #   own evidence names it: matched_diagnostics = ["ERROR: ... Skipped
    #   unknown 'size' field in Cloth node."].
    #
    # It survived because `documented_as=ABSENT` and a doc string reading "no
    # cloth solver is compiled in" AGREED with the wrong measurement, so the
    # lane's doc-audit -- the thing built to catch exactly this -- saw no
    # disagreement to report. A probe and a doc can be wrong together.
    #
    # ⚠ NOT A CAUSE, THOUGH IT LOOKS LIKE ONE: the lane hardcoding
    # `newtonSolver "mujoco"`. Cloth.wrl and cloth-simulation.md both say a
    # Cloth ONLY simulates under "mujoco+vbd", which predicts an inert sheet
    # here. Measured 2026-08-16, it is not: this world at `newtonSolver
    # "mujoco"` still logs `registered 441 particles` and finalises on
    # `... + VBD cloth via SolverCoupledProxy`, because the runtime gates the
    # coupled solver on `has_cloth()` and not on the declared value. The
    # declaration below is kept anyway -- it is what the field contract tells
    # an author to write, and it keeps the probe correct if that gate is ever
    # narrowed -- but do NOT record it as the fix. See Probe.solver.
    solver="mujoco+vbd",
    # The sheet hangs from its pinned +Y edge (fixTop defaults TRUE) at z=2.0
    # so it drops clear of the floor: cloth-vs-STATICS is a known open defect
    # (Cloth.wrl -- unpinned fabric sinks through a static body), and nothing
    # here should be contingent on it. The reference sphere is parked at
    # x=-0.6, half a metre clear of the patch's x=0..1 span, so the two cannot
    # interact and the sphere measures the RIGID scene alone.
    world=lambda: floor() + """DEF SHEET Cloth {
  translation 0 0 2.0
  dimX 20
  dimY 20
  cellX 0.05
  cellY 0.05
  mass 0.001
}
""" + body("SUBJECT", "Sphere { radius 0.1 subdivision 3 }", "-0.6 0 1.2"),
    # `particles:SHEET` is the 2026-09-01 getParticleStats readback — the
    # supervisor accessor whose absence is what capped this row at `degraded`
    # ("particle state has NO supervisor accessor"). The drape is now measured
    # in metres instead of inferred from the engine's registration line; the
    # line is still captured, as checkable evidence beside the measurement.
    measure=("pos:SUBJECT", "node_exists:SHEET", "particles:SHEET"),
    log_capture=("Cloth '",),
    duration=3.0,
    assertion=None,   # set below
    doc="resources/nodes/Cloth.wrl (field contract) + "
        "docs/developer/cloth-simulation.md (the solver requirement is "
        "section 0)",
    # The matrix's standing claim as of 2026-09-01 (drape unmeasurable in this
    # lane). Now that the drape IS measured, a passing run lands on `works`
    # and the audit flags the flip for the parent to reconcile the docs — the
    # same protocol round 2 used.
    documented_as=WORKS,  # flipped 2026-09-01 round 3: drape measured via the particle-stats verb: z-extent grows, pinned edge holds
))


#: (dimX + 1) * (dimY + 1) for the patch declared above -- particles, not cells.
#: Asserting the exact count is what makes the engine's registration line
#: evidence rather than a reassuring sentence: a wrong count means the node
#: reached the solver carrying geometry the world did not author.
CLOTH_PARTICLES = 21 * 21


def _cloth_assertion(arrays):
    """A 20x20-cell `Cloth` pinned along its +Y edge at z=2.0 must MEASURABLY
    DRAPE, read through getParticleStats (the 2026-09-01 supervisor particle
    readback): exactly 441 particles (its authored dimX/dimY), a z-extent that
    grows from ~0 (the sheet is authored flat) to between 0.05 and 1.05 m by
    run end (the free edge hangs at most 20 x 0.05 = 1.0 m below the pin), a
    pinned edge that HOLDS (bbox max z within 15 mm of the authored 2.0 on
    every frame) while the centroid drops at least 0.05 m, zero non-finite
    particles, and a reference sphere beside it still resting at the rigid
    0.65 m. A sheet whose extent never grows registered and does NOTHING —
    `broken`. A readback that is missing or refuses (no frames, status != 0,
    count == 0) is an environment/instrument condition: `inconclusive`,
    never `broken`."""
    exists = arrays.get("node_exists_SHEET")
    z = _final(arrays, "pos_SUBJECT", 2)
    # The engine's own registration line, kept as evidence BESIDE the
    # measurement (it is what the pre-2026-09-01 verdict rested on entirely).
    registered = None
    for ln in (arrays.get("engine_log") or []):
        m = re.search(r"registered (\d+) particles", ln)
        if m:
            registered = int(m.group(1))
    ev = {"cloth_node_in_scene_tree": bool(exists),
          "engine_registration_line_particles": registered,
          "expected_particles": CLOTH_PARTICLES,
          "reference_sphere_rest_z_m": z}
    if not exists:
        return Verdict(
            ABSENT, ev,
            "Cloth did not survive into the scene tree — the node is not "
            "usable from a .wbt even though the C++ class exists")
    ok, statuses, dicts = _particle_frames(arrays, "SHEET")
    bad = _particle_triage(ok, statuses, dicts, "Cloth")
    if bad is not None:
        bad.evidence.update(ev)
        return bad
    first, last = ok[0][1], ok[-1][1]
    cnt = _pnum(last, "count")
    e0, e1 = _pext(first, 2), _pext(last, 2)
    c0, c1 = _pnum(first, "centroid", 2), _pnum(last, "centroid", 2)
    nf = _particle_nonfinite(ok)
    pin_dev = None
    for _, f in ok:
        top = _pnum(f, "max", 2)
        if top is not None:
            d = abs(top - 2.0)
            pin_dev = d if pin_dev is None else max(pin_dev, d)
    ev.update({"particle_count": cnt, "z_extent_first_m": e0,
               "z_extent_final_m": e1, "centroid_z_first_m": c0,
               "centroid_z_final_m": c1,
               "pinned_edge_max_deviation_m": pin_dev,
               "non_finite_peak": nf, "status0_frames": len(ok)})
    if None in (cnt, e0, e1, c0, c1, pin_dev):
        return Verdict(INCONCLUSIVE, ev,
                       "getParticleStats frames are malformed (a stats field "
                       "is missing or non-finite) — instrument failure")
    if cnt == 0:
        return Verdict(INCONCLUSIVE, ev,
                       "getParticleStats reports 0 particles at status 0 — "
                       "nothing allocated; environment, not a capability "
                       "verdict")
    if e0 > 0.10:
        return Verdict(INCONCLUSIVE, ev,
                       "the first readable frame already shows a %.3f m "
                       "z-extent — sampled too late to anchor the flat-sheet "
                       "premise the growth assertion needs" % e0)
    if nf > 0:
        return Verdict(DEGRADED, ev,
                       "%d particle position(s) went non-finite during the "
                       "drape" % nf)
    if e1 <= 0.05:
        return Verdict(
            BROKEN, ev,
            "the sheet's z-extent never grew (%.3f -> %.3f m): 441 particles "
            "are allocated and the fabric never moves" % (e0, e1))
    if cnt != CLOTH_PARTICLES:
        return Verdict(
            DEGRADED, ev,
            "getParticleStats reports %d particles, not the %d the authored "
            "dimX/dimY imply — the node reached the solver carrying geometry "
            "the world did not author" % (int(cnt), CLOTH_PARTICLES))
    if e1 >= 1.05:
        return Verdict(DEGRADED, ev,
                       "the sheet stretched to a %.3f m z-extent, past the "
                       "1.0 m free-edge bound its geometry allows" % e1)
    if pin_dev > 0.015:
        return Verdict(DEGRADED, ev,
                       "the pinned edge moved %.1f mm off the authored "
                       "z=2.0 — fixTop is not holding its particles"
                       % (pin_dev * 1e3))
    if c0 - c1 < 0.05:
        return Verdict(DEGRADED, ev,
                       "the extent grew but the centroid only dropped "
                       "%.3f m — the sheet is not draping under gravity"
                       % (c0 - c1))
    if z is None:
        return Verdict(INCONCLUSIVE, ev, "reference sphere not recorded")
    if abs(z - 0.65) > 0.02:
        return Verdict(
            BROKEN, ev,
            "the reference sphere rests at z=%.4f m instead of 0.6500 — moving "
            "to the coupled VBD solver perturbed the rigid scene" % z)
    return Verdict(WORKS, ev)


PROBES[-1].assertion = _cloth_assertion

_p(Probe(
    id="object.soft_body_fem",
    family=FAM_OBJECT,
    claim="Volumetric soft bodies (FEM / MPM)",
    # Same shape as object.deformable_cloth above, and for the same reason: the
    # node is declared, a reference sphere is dropped alongside it, and the
    # probe claims ONLY what it can measure. Tet state has no supervisor
    # accessor, so "the tets are simulated" is not reachable from this lane
    # however the world is arranged.
    #
    # Declares "mujoco+vbd" for the same reason the cloth probe does -- it is
    # what OmSoftBody.cpp's own "registered no particles" warning tells an
    # author to write -- and, as measured there, for the same reason it is NOT
    # load-bearing on this engine: the coupled solver is gated on `has_cloth()`,
    # which a SoftBody satisfies too (that predicate means "a particle source
    # exists", not "fabric"). This probe was ALREADY registering its 125
    # particles under the old hardcoded value; the row's verdict is unchanged
    # by the switch, and its rest-height evidence is bit-identical to the
    # 2026-08-15 run at 0.6496329307556152 m.
    #
    # object.granular_group is deliberately NOT changed: a GranularGroup is a
    # separate CUDA particle system with its own solver (OmGranularGroup.cpp),
    # not a SolverVBD client, so it has no such requirement, logs no
    # registration line, and would pay for a solver it never uses.
    solver="mujoco+vbd",
    log_capture=("SoftBody '",),
    world=lambda: floor() + """DEF SUBJECT_BLOB SoftBody {
  translation -0.1 -0.1 1.0
  dimX 4
  dimY 4
  dimZ 4
  cellX 0.05
  cellY 0.05
  cellZ 0.05
  density 1000
  kMu 10000
  kLambda 10000
  kDamp 30
  particleRadius 0.01
}
""" + body("SUBJECT", "Sphere { radius 0.1 subdivision 3 }", "0.6 0 1.2"),
    # `particles:SUBJECT_BLOB` is the 2026-09-01 getParticleStats readback —
    # the accessor whose absence made "the tets are simulated" unreachable
    # from this lane, and whose arrival is what let the withdrawn
    # force-transmission probe (see the assertion's history note) be replaced
    # by a direct deformation measurement.
    measure=("pos:SUBJECT", "node_exists:SUBJECT_BLOB",
             "particles:SUBJECT_BLOB"),
    duration=3.0,
    assertion=None,   # set below
    absent_markers=("SoftBody",),
    doc="src/omnisim/nodes/OmSoftBody.cpp",
    # The matrix's standing claim as of 2026-09-01 (tet state unmeasurable in
    # this lane). The deformation is now measured; a passing run lands on
    # `works` and the audit flags the flip — round-2 protocol.
    documented_as=WORKS,  # flipped 2026-09-01 round 3: deformation measured via the particle-stats verb: falls, arrests, squashes
))


#: (dimX+1) * (dimY+1) * (dimZ+1) for the 4x4x4-cell block declared above.
SOFT_BODY_PARTICLES = 5 * 5 * 5
#: The blob's authored extent on every axis: 4 cells x 0.05 m.
SOFT_BODY_EXTENT = 0.2


def _soft_body_assertion(arrays):
    """A 4x4x4-cell `SoftBody` (5^3 = 125 particles, authored 0.2 m on every
    side, minimum corner at z=1.0) must FALL, ARREST and DEFORM, read through
    getParticleStats (the 2026-09-01 supervisor particle readback): exactly
    125 particles; a centroid that drops at least 0.2 m (from ~1.1 m onto the
    0.55 m floor) and then arrests (centroid-z span <= 10 mm over the final
    0.5 s); a shape that deforms rather than rigid-translates — final
    z-extent at least 5 mm below the authored 0.2 m while the x or y extent
    grows by at least 5 mm; zero non-finite particles; and a reference sphere
    beside it still resting at the rigid 0.65 m. A blob that never falls
    registered and does NOTHING — `broken`. A readback that is missing or
    refuses (no frames, status != 0, count == 0) is an environment/instrument
    condition: `inconclusive`, never `broken`.

    History: a force-transmission probe (27 kg soft block on a 2 kg rigid
    box) was attempted before this readback existed and withdrawn — staged in
    this lane's world the box was driven through the floor to z = -0.32,
    unattributed to the capability. The decisive probe needed the deformable
    readback surface, which is exactly what this now uses."""
    exists = arrays.get("node_exists_SUBJECT_BLOB")
    z = _final(arrays, "pos_SUBJECT", 2)
    ev = {"soft_body_node_in_scene_tree": bool(exists),
          "reference_sphere_rest_z_m": z}
    if not exists:
        return Verdict(
            ABSENT, ev,
            "SoftBody did not survive into the scene tree — the node is not "
            "usable from a .wbt even though the C++ class exists")
    ok, statuses, dicts = _particle_frames(arrays, "SUBJECT_BLOB")
    bad = _particle_triage(ok, statuses, dicts, "SoftBody")
    if bad is not None:
        bad.evidence.update(ev)
        return bad
    first, last = ok[0][1], ok[-1][1]
    cnt = _pnum(last, "count")
    c0, c1 = _pnum(first, "centroid", 2), _pnum(last, "centroid", 2)
    ez0, ez1 = _pext(first, 2), _pext(last, 2)
    ex0, ex1 = _pext(first, 0), _pext(last, 0)
    ey0, ey1 = _pext(first, 1), _pext(last, 1)
    arrest = _particle_arrest(ok, arrays.get("t"))
    nf = _particle_nonfinite(ok)
    ev.update({"particle_count": cnt,
               "centroid_z_first_m": c0, "centroid_z_final_m": c1,
               "z_extent_first_m": ez0, "z_extent_final_m": ez1,
               "x_extent_first_m": ex0, "x_extent_final_m": ex1,
               "y_extent_first_m": ey0, "y_extent_final_m": ey1,
               "arrest_span_m": arrest, "non_finite_peak": nf,
               "status0_frames": len(ok)})
    if None in (cnt, c0, c1, ez0, ez1, ex0, ex1, ey0, ey1):
        return Verdict(INCONCLUSIVE, ev,
                       "getParticleStats frames are malformed (a stats field "
                       "is missing or non-finite) — instrument failure")
    if cnt == 0:
        return Verdict(INCONCLUSIVE, ev,
                       "getParticleStats reports 0 particles at status 0 — "
                       "nothing allocated; environment, not a capability "
                       "verdict")
    if nf > 0:
        return Verdict(DEGRADED, ev,
                       "%d particle position(s) went non-finite during the "
                       "run" % nf)
    drop = c0 - c1
    if drop < 0.2:
        if drop < 0.02:
            return Verdict(BROKEN, ev,
                           "125 particles are allocated and the blob never "
                           "fell (centroid moved %.3f m) — the tets are not "
                           "integrating" % drop)
        return Verdict(DEGRADED, ev,
                       "centroid dropped only %.3f m against the >= 0.2 m the "
                       "1.1 m spawn above the 0.55 m floor demands" % drop)
    if cnt != SOFT_BODY_PARTICLES:
        return Verdict(DEGRADED, ev,
                       "getParticleStats reports %d particles, not the %d "
                       "the authored dimX/dimY/dimZ imply" %
                       (int(cnt), SOFT_BODY_PARTICLES))
    if arrest is None or arrest > 0.01:
        return Verdict(DEGRADED, ev,
                       "centroid still moving %.1f mm over the final 0.5 s — "
                       "the blob has not come to rest"
                       % ((arrest or float("nan")) * 1e3))
    squash = ez0 - ez1
    spread = max(ex1 - ex0, ey1 - ey0)
    if squash < 0.005 or spread < 0.005:
        return Verdict(
            DEGRADED, ev,
            "the blob rigid-translates instead of deforming: z-extent shrank "
            "%.1f mm and the widest lateral growth is %.1f mm (>= 5 mm of "
            "each is the deformation bar)" % (squash * 1e3, spread * 1e3))
    if z is None:
        return Verdict(INCONCLUSIVE, ev, "reference sphere not recorded")
    if abs(z - 0.65) > 0.02:
        return Verdict(
            BROKEN, ev,
            "the reference sphere rests at z=%.4f m instead of 0.6500 — declaring a "
            "SoftBody perturbed the rigid scene around it" % z)
    return Verdict(WORKS, ev)


PROBES[-1].assertion = _soft_body_assertion


def _mpm_wall(defname, translation, size):
    """One static retaining wall for the MPM bed pen. Sand at the bed's
    default material is COHESIONLESS (yieldStress 0), so without walls the
    pile spreads for the whole run and the arrest assertion can never land —
    the same reason newton_granular_bed_drop.omniworld carries four."""
    return """DEF %s Solid {
  translation %s
  name "%s"
  children [
    DEF %s_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.3 0.31 0.34 roughness 0.9 metalness 0 }
      geometry Box { size %s }
    }
  ]
  boundingObject USE %s_SHAPE
}
""" % (defname, translation, defname.lower(), defname, size, defname)


_p(Probe(
    id="object.granular_bed_mpm",
    family=FAM_OBJECT,
    claim="GranularBed — MPM granular matter (newton SolverImplicitMPM)",
    # The Newton-path granular node (vs object.granular_group's self-contained
    # CUDA kernel): a Drucker-Prager MPM bed two-way coupled to the rigid
    # scene through SolverCoupledProxy. Authored per the reference world
    # projects/samples/demos/worlds/physics/newton_granular_bed_drop.omniworld:
    # `translation` is the bed's MINIMUM CORNER (newton's add_particle_grid
    # convention, the opposite of Solid), rigidSubsteps 4 is load-bearing (1
    # measurably GAINS energy), gridType "sparse" because a "fixed" grid
    # silently NaNs material that leaves its box. The bed's min corner is at
    # z=0.7 over the 0.55 floor top, so it drops 0.15 m into a walled pen and
    # settles — centroid drops, z-extent collapses, motion arrests — which
    # getParticleStats (2026-09-01) reads directly.
    #
    # newtonSolver stays the template's "mujoco" deliberately: the MPM
    # coupling is selected by the NODES PRESENT, never by that string
    # (GranularBed.wrl — the schema enum has no "mujoco+mpm" value), the same
    # measured rule as the cloth probe's has_cloth() gate.
    #
    # ⛔ ON A BOX WITHOUT CUDA THE RUNTIME REFUSES THE WHOLE WORLD, loudly and
    # by design (omnisim_newton_runtime.py raises "GranularBed requires CUDA"
    # at finalize; OMNISIM_MPM_ALLOW_CPU=1 is the smoke-test override at a
    # measured 42-351 ms/step). The refusal is a named scope statement, so
    # the verdict there is `absent` via log_capture — INCONCLUSIVE is
    # reserved for instrument failures.
    world=lambda: (
        floor()
        + _mpm_wall("MPM_WALL_XN", "-0.26 0 0.7", "0.02 0.54 0.3")
        + _mpm_wall("MPM_WALL_XP", "0.26 0 0.7", "0.02 0.54 0.3")
        + _mpm_wall("MPM_WALL_YN", "0 -0.26 0.7", "0.54 0.02 0.3")
        + _mpm_wall("MPM_WALL_YP", "0 0.26 0.7", "0.54 0.02 0.3")
        + """DEF BED GranularBed {
  translation -0.2 -0.2 0.7
  size 0.4 0.4 0.2
  voxelSize 0.05
  particlesPerCell 3
  count 20000
  density 2500
  friction 0.75
  rigidSubsteps 4
  proxyIterations 1
  gridType "sparse"
}
"""),
    measure=("node_exists:BED", "particles:BED"),
    log_capture=("GranularBed requires CUDA", "OMNISIM_MPM_ALLOW_CPU"),
    duration=4.0,
    dt_ms=8.0,
    world_info="  newtonSubsteps 2\n",
    assertion=None,   # set below
    doc="resources/nodes/GranularBed.wrl + projects/samples/demos/worlds/"
        "physics/newton_granular_bed_drop.omniworld",
    documented_as=None,   # new row (2026-09-01): no standing matrix claim
))


def _granular_bed_assertion(arrays):
    """A 0.4 x 0.4 x 0.2 m GranularBed whose minimum corner is authored at
    z=0.7 must DROP 0.15 m into a walled pen on the 0.55 m floor and SETTLE,
    read through getParticleStats (2026-09-01): particles allocate (an exact
    count is deliberately not asserted — the lattice count is voxel-rounding
    dependent, the reference world documents the FP wart — but zero is), the
    centroid drops at least 0.05 m, the z-extent collapses measurably below
    its authored 0.2 m start, the motion arrests (centroid-z span <= 10 mm
    over the final 0.5 s), and every particle stays finite (a NaN bed is the
    documented "fixed"-grid escape failure). On a box without CUDA the
    runtime REFUSES the world by name — that named refusal scores `absent`
    (requires engine CUDA), never `broken`; `inconclusive` is reserved for
    instrument failures (missing binding, refused readback, malformed
    frames)."""
    exists = arrays.get("node_exists_BED")
    refusal = _log_hits(arrays, "GranularBed requires CUDA")
    allow_cpu = _log_hits(arrays, "OMNISIM_MPM_ALLOW_CPU")
    ev = {"bed_node_in_scene_tree": bool(exists),
          "cuda_refusal_lines": refusal,
          "allow_cpu_lines": allow_cpu}
    if refusal:
        return Verdict(
            ABSENT, ev,
            "requires engine CUDA; the runtime refused to build the MPM "
            "solver on this box (its own line: %s). OMNISIM_MPM_ALLOW_CPU=1 "
            "is the smoke-test override." % refusal[0][:180])
    if not exists:
        return Verdict(
            ABSENT, ev,
            "GranularBed did not survive into the scene tree — the node is "
            "not usable from a .wbt even though the schema ships it")
    ok, statuses, dicts = _particle_frames(arrays, "BED")
    bad = _particle_triage(ok, statuses, dicts, "GranularBed")
    if bad is not None:
        bad.evidence.update(ev)
        return bad
    first, last = ok[0][1], ok[-1][1]
    cnt = _pnum(last, "count")
    c0, c1 = _pnum(first, "centroid", 2), _pnum(last, "centroid", 2)
    e0, e1 = _pext(first, 2), _pext(last, 2)
    arrest = _particle_arrest(ok, arrays.get("t"))
    nf = _particle_nonfinite(ok)
    ev.update({"particle_count": cnt, "centroid_z_first_m": c0,
               "centroid_z_final_m": c1, "z_extent_first_m": e0,
               "z_extent_final_m": e1, "arrest_span_m": arrest,
               "non_finite_peak": nf, "status0_frames": len(ok)})
    if None in (cnt, c0, c1, e0, e1):
        return Verdict(INCONCLUSIVE, ev,
                       "getParticleStats frames are malformed (a stats field "
                       "is missing or non-finite) — instrument failure")
    if cnt == 0:
        return Verdict(INCONCLUSIVE, ev,
                       "getParticleStats reports 0 particles at status 0 — "
                       "nothing allocated; environment, not a capability "
                       "verdict")
    if nf > 0:
        return Verdict(DEGRADED, ev,
                       "%d particle position(s) went non-finite — the "
                       "documented grid-escape failure mode" % nf)
    drop = c0 - c1
    if drop < 0.05:
        if drop < 0.005:
            return Verdict(BROKEN, ev,
                           "particles allocated and the bed never moved "
                           "(centroid dropped %.4f m) — the documented "
                           "silently-skipped-MPM failure, the exact case the "
                           "reference world's control arm exists to catch"
                           % drop)
        return Verdict(DEGRADED, ev,
                       "centroid dropped only %.3f m against the >= 0.05 m "
                       "the 0.15 m authored drop demands" % drop)
    if e1 >= e0 - 0.02:
        return Verdict(DEGRADED, ev,
                       "z-extent ended at %.3f m from an authored %.3f — the "
                       "bed fell without collapsing into the pen" % (e1, e0))
    if arrest is None or arrest > 0.01:
        return Verdict(DEGRADED, ev,
                       "centroid still moving %.1f mm over the final 0.5 s — "
                       "the bed has not arrested"
                       % ((arrest or float("nan")) * 1e3))
    return Verdict(WORKS, ev)


PROBES[-1].assertion = _granular_bed_assertion


# ===========================================================================
# FAMILY: joint / articulation
# ===========================================================================
def _hinge_rig(joint_node):
    """A pendulum arm hanging off the PROBER ROBOT ITSELF, which acts as the
    static anchor (physics NULL). The joint under test is the only thing that
    differs between joint probes.

    The rig deliberately lives inside the prober rather than in a separate
    `Robot { controller "<none>" }`: the motors and PositionSensors under test
    are devices, and OmniSim restricts device APIs to the owning controller,
    so a joint mounted anywhere else could be commanded by nobody and read by
    nobody. There is no floor in these worlds — the arm must be free to swing
    without landing on anything.
    """
    return """    DEF POST Shape {
      appearance PBRAppearance { baseColor 0.3 0.3 0.35 roughness 1 metalness 0 }
      geometry Box { size 0.08 0.08 0.08 }
    }
%s""" % (joint_node,)


_ARM = """      DEF SUBJECT Solid {
        translation 0.25 0 0
        name "arm"
        children [
          DEF ARM_SHAPE Shape {
            appearance PBRAppearance { baseColor 0.85 0.55 0.2 roughness 0.9 metalness 0 }
            geometry Box { size 0.5 0.05 0.05 }
          }
        ]
        boundingObject USE ARM_SHAPE
        physics Physics { density -1 mass 1 }
      }
"""

_p(Probe(
    id="joint.hinge_passive",
    family=FAM_JOINT,
    claim="HingeJoint, passive — swings freely under gravity",
    world=lambda: "",
    prober_children=_hinge_rig("""    HingeJoint {
      jointParameters DEF JP HingeJointParameters {
        axis 0 1 0
        anchor 0 0 0
      }
      endPoint
%s    }
""" % _ARM),
    measure=("jointpos:JP", "pos:SUBJECT"),
    duration=2.0,
    assertion=None,
    doc="docs/reference/",
))


def _hinge_passive_assertion(arrays):
    """A 0.5 m arm released horizontally about a frictionless hinge must swing
    through at least 45 deg (0.785 rad) within 2 s — a free pendulum of this
    length has a quarter-period well under 0.4 s. Zero travel means the joint
    is welded, not hinged."""
    span = _span(arrays, "joint_JP")
    ev = {"joint_travel_rad": span,
          "joint_travel_deg": None if span is None else math.degrees(span)}
    if span is None:
        return Verdict(INCONCLUSIVE, ev, "joint angle not recorded")
    if span >= 0.785:
        return Verdict(WORKS, ev)
    if span < 1e-4:
        return Verdict(BROKEN, ev,
                       "the hinge never moved (travel %.2e rad) — it is "
                       "behaving as a weld" % span)
    return Verdict(DEGRADED, ev,
                   "the hinge moved only %.2f deg in 2 s" % math.degrees(span))


PROBES[-1].assertion = _hinge_passive_assertion

_p(Probe(
    id="joint.hinge_motor_position",
    family=FAM_JOINT,
    claim="RotationalMotor position control — setPosition reaches its target",
    world=lambda: "",
    prober_children=_hinge_rig("""    HingeJoint {
      jointParameters DEF JP HingeJointParameters {
        axis 0 1 0
        anchor 0 0 0
      }
      device [
        RotationalMotor { name "m" maxTorque 100 }
      ]
      endPoint
%s    }
""" % _ARM),
    measure=("jointpos:JP",),
    act=("motor_position:m:0.8:0.0",),
    duration=3.0,
    assertion=None,
))


def _motor_pos_assertion(arrays):
    """A RotationalMotor commanded to 0.8 rad with 100 N.m of torque must
    arrive within 0.05 rad of the target in 3 s. Never arriving at all is
    `broken`; arriving with a steady offset is `degraded` and the offset is
    reported (the cold-load under-tracking defect had this shape)."""
    a = _finite(arrays.get("joint_JP"))
    if not a:
        return Verdict(INCONCLUSIVE, note="joint angle not recorded")
    finalv = a[-1]
    err = abs(finalv - 0.8)
    ev = {"target_rad": 0.8, "achieved_rad": finalv, "abs_err_rad": err,
          "travel_rad": _span(arrays, "joint_JP")}
    if err <= 0.05:
        return Verdict(WORKS, ev)
    if abs(finalv) < 1e-4:
        return Verdict(BROKEN, ev,
                       "the motor was accepted but the joint never left 0 rad")
    return Verdict(DEGRADED, ev,
                   "the motor settled %.4f rad from its target" % err)


PROBES[-1].assertion = _motor_pos_assertion

_p(Probe(
    id="joint.hinge_motor_position_unloaded",
    family=FAM_JOINT,
    claim="RotationalMotor position control with NO gravity load -- "
          "does the servo track its target at all?",
    # The attribution probe for the loaded one above, which reached 0.0147 rad
    # of an 0.8 rad command. Two very different defects produce that number --
    # a servo that does not track, or a servo that tracks but has no authority
    # against a 2.45 N.m gravity load -- and the fix, the severity and the
    # workaround differ for each. Removing gravity is the one-variable change
    # that separates them.
    gravity=0.0,
    prober_children=_hinge_rig("""    HingeJoint {
      jointParameters DEF JP HingeJointParameters {
        axis 0 1 0
        anchor 0 0 0
      }
      device [
        RotationalMotor { name "m" maxTorque 100 }
      ]
      endPoint
%s    }
""" % _ARM),
    world=lambda: "",
    measure=("jointpos:JP",),
    act=("motor_position:m:0.8:0.0",),
    duration=3.0,
    assertion=None,
))


def _motor_pos_unloaded_assertion(arrays):
    """The same 0.8 rad position command as the loaded probe, with gravity
    switched off. Arriving here while the loaded probe stalls means position
    control WORKS and simply has no authority against load; stalling here too
    means the position servo does not track at all. The loaded number alone
    cannot tell those apart, and they are not the same defect."""
    a = _finite(arrays.get("joint_JP"))
    if not a:
        return Verdict(INCONCLUSIVE, note="joint angle not recorded")
    finalv = a[-1]
    err = abs(finalv - 0.8)
    ev = {"target_rad": 0.8, "achieved_rad": finalv, "abs_err_rad": err,
          "gravity_m_s2": 0.0}
    if err <= 0.05:
        return Verdict(WORKS, ev)
    if abs(finalv) < 1e-4:
        return Verdict(BROKEN, ev,
                       "with no load at all the motor never left 0 rad -- the "
                       "position servo does not track")
    return Verdict(DEGRADED, ev,
                   "unloaded, the motor still settled %.4f rad from its "
                   "target" % err)


PROBES[-1].assertion = _motor_pos_unloaded_assertion

_p(Probe(
    id="joint.hinge_motor_position_with_gain",
    family=FAM_JOINT,
    claim="RotationalMotor position control WITH the documented gain override "
          "(OMNISIM_NEWTON_TARGET_KE)",
    # The third leg of the motor diagnosis. The backend hardcodes
    # target_ke=0, target_kd=500 for motorised hinges -- gains tuned for WHEEL
    # VELOCITY drive, where a position gain is not wanted -- so setPosition
    # has no proportional term at all and cannot track. The engine exposes
    # OMNISIM_NEWTON_TARGET_KE/KD as an opt-in override (the Spot residual
    # recipe uses KE=250/KD=60). If that restores tracking, the honest finding
    # is "position control is off by default and recoverable", which is a
    # completely different thing to tell a user than "position control is
    # broken".
    gravity=0.0,
    env={"OMNISIM_NEWTON_TARGET_KE": "250", "OMNISIM_NEWTON_TARGET_KD": "60"},
    prober_children=_hinge_rig("""    HingeJoint {
      jointParameters DEF JP HingeJointParameters {
        axis 0 1 0
        anchor 0 0 0
      }
      device [
        RotationalMotor { name "m" maxTorque 100 }
      ]
      endPoint
%s    }
""" % _ARM),
    world=lambda: "",
    measure=("jointpos:JP",),
    act=("motor_position:m:0.8:0.0",),
    duration=3.0,
    assertion=None,
))


def _motor_pos_gain_assertion(arrays):
    """The same unloaded 0.8 rad command, run with OMNISIM_NEWTON_TARGET_KE=250
    and TARGET_KD=60. Arriving here while the default-gain probe sits at 0 rad
    proves position control is not missing but merely UNGAINED by default --
    a configuration finding with a one-line workaround, not a dead feature."""
    a = _finite(arrays.get("joint_JP"))
    if not a:
        return Verdict(INCONCLUSIVE, note="joint angle not recorded")
    finalv = a[-1]
    err = abs(finalv - 0.8)
    ev = {"target_rad": 0.8, "achieved_rad": finalv, "abs_err_rad": err,
          "OMNISIM_NEWTON_TARGET_KE": 250, "OMNISIM_NEWTON_TARGET_KD": 60}
    if err <= 0.05:
        return Verdict(WORKS, ev)
    if abs(finalv) < 1e-4:
        return Verdict(BROKEN, ev,
                       "even with the documented gain override the joint "
                       "never left 0 rad")
    return Verdict(DEGRADED, ev,
                   "with the gain override the motor settled %.4f rad from "
                   "its target" % err)


PROBES[-1].assertion = _motor_pos_gain_assertion

_p(Probe(
    id="joint.hinge_motor_force",
    family=FAM_JOINT,
    claim="RotationalMotor torque control — setTorque produces motion "
          "(the grasp path: a servo that has arrived pushes with nothing)",
    world=lambda: "",
    prober_children=_hinge_rig("""    HingeJoint {
      jointParameters DEF JP HingeJointParameters {
        axis 0 1 0
        anchor 0 0 0
      }
      device [
        RotationalMotor { name "m" maxTorque 100 }
      ]
      endPoint
%s    }
""" % _ARM),
    measure=("jointpos:JP",),
    act=("motor_torque:m:20:0.0",),
    duration=2.0,
    assertion=None,
    doc="docs/guide/friction-grasp.md — setForce, never a position target",
))


def _motor_torque_assertion(arrays):
    """20 N.m applied to a 1 kg, 0.5 m arm must rotate it by more than 0.2 rad
    within 2 s. No motion means setTorque is accepted and ignored — the exact
    failure that makes a gripper unable to hold a part."""
    span = _span(arrays, "joint_JP")
    ev = {"joint_travel_rad": span, "applied_torque_Nm": 20.0}
    if span is None:
        return Verdict(INCONCLUSIVE, ev, "joint angle not recorded")
    if span >= 0.2:
        return Verdict(WORKS, ev)
    if span < 1e-4:
        return Verdict(BROKEN, ev,
                       "setTorque(20 N.m) produced no motion at all")
    return Verdict(DEGRADED, ev,
                   "setTorque(20 N.m) produced only %.4f rad of travel" % span)


PROBES[-1].assertion = _motor_torque_assertion

_p(Probe(
    id="joint.hinge_motor_velocity",
    family=FAM_JOINT,
    claim="RotationalMotor velocity control -- setPosition(inf) + setVelocity "
          "spins the joint at the commanded rate",
    # THE DISCRIMINATOR. The position-control probe measured 0.0147 rad of an
    # 0.8 rad command and the torque probe 0.0898 rad from 20 N.m, while the
    # PASSIVE hinge swings freely -- so the articulation exists and something
    # specific to motor commands is weak. Velocity control is the mode every
    # working wheeled demo in this repo actually uses, so measuring it
    # separates "motors are broken" from "position control is weak", and
    # those two findings have completely different consequences for a user.
    gravity=0.0,
    prober_children=_hinge_rig("""    HingeJoint {
      jointParameters DEF JP HingeJointParameters {
        axis 0 1 0
        anchor 0 0 0
      }
      device [
        RotationalMotor { name "m" maxTorque 100 maxVelocity 12 }
      ]
      endPoint
%s    }
""" % _ARM),
    world=lambda: "",
    measure=("jointpos:JP", "pos:SUBJECT"),
    act=("motor_velocity:m:2.0:0.0",),
    duration=2.0,
    assertion=None,
))


def _motor_velocity_assertion(arrays):
    """A hinge commanded to 2.0 rad/s in a gravity-free world must turn about
    4.0 rad in 2 s. The joint's `position` field wraps, so the ARM POSE is the
    honest witness of gross rotation: a 0.5 m arm spinning through more than a
    radian cannot stay within 5 cm of where it started."""
    span = _span(arrays, "joint_JP")
    p = arrays.get("pos_SUBJECT")
    disp = None
    if p is not None and len(p) >= 2:
        disp = max(math.dist(tuple(p[0]), tuple(q)) for q in p)
    ev = {"commanded_rad_s": 2.0, "window_s": 2.0,
          "joint_angle_span_rad": span, "max_arm_excursion_m": disp}
    if disp is None:
        return Verdict(INCONCLUSIVE, ev, "arm pose not recorded")
    if disp >= 0.05:
        return Verdict(WORKS, ev)
    if disp < 1e-3:
        return Verdict(BROKEN, ev,
                       "setVelocity(2.0 rad/s) moved the arm %.2e m in 2 s -- "
                       "velocity control does nothing either" % disp)
    return Verdict(DEGRADED, ev,
                   "setVelocity(2.0 rad/s) moved the arm only %.1f mm in 2 s"
                   % (disp * 1e3))


PROBES[-1].assertion = _motor_velocity_assertion

_p(Probe(
    id="joint.slider_motor",
    family=FAM_JOINT,
    claim="SliderJoint + LinearMotor — prismatic actuation",
    world=lambda: "",
    prober_children=_hinge_rig("""    SliderJoint {
      jointParameters DEF JP JointParameters {
        axis 1 0 0
      }
      device [
        LinearMotor { name "m" maxForce 200 }
      ]
      endPoint
%s    }
""" % _ARM),
    measure=("jointpos:JP",),
    act=("motor_position:m:0.15:0.0",),
    duration=3.0,
    assertion=None,
))


def _slider_assertion(arrays):
    """A LinearMotor commanded to 0.15 m with 200 N must arrive within 5 mm
    in 3 s."""
    a = _finite(arrays.get("joint_JP"))
    if not a:
        return Verdict(INCONCLUSIVE, note="joint position not recorded")
    finalv = a[-1]
    err = abs(finalv - 0.15)
    ev = {"target_m": 0.15, "achieved_m": finalv, "abs_err_m": err}
    if err <= 0.005:
        return Verdict(WORKS, ev)
    if abs(finalv) < 1e-5:
        return Verdict(BROKEN, ev,
                       "the linear motor was accepted but never moved")
    return Verdict(DEGRADED, ev,
                   "the slider settled %.1f mm from its target" % (err * 1e3))


PROBES[-1].assertion = _slider_assertion

_p(Probe(
    id="joint.ball_motor",
    family=FAM_JOINT,
    claim="BallJoint with motors — 3-DoF spherical actuation",
    world=lambda: "",
    # Every motor declares min/maxPosition, and that is LOAD-BEARING, not
    # decoration. OmBasicJoint::newtonAxisSpec classifies a motor with no
    # position range (and no minStop/maxStop on the joint) as a VELOCITY WHEEL:
    # it sets ke = 0 and setPosition() is then ignored BY DESIGN, which the
    # engine says out loud per device. Authored limit-less, this probe measured
    # `broken` while commanding a servo that the engine had already told it was
    # a wheel -- i.e. it was measuring internal parity plan, item W1.4 (setPosition on a
    # limit-less motor, still open, and already covered by
    # joint.hinge_motor_position_unloaded) and never reached W1.3, the
    # BALL_HINGE2 gate this probe claims to test. The range is deliberately
    # WIDER than the 0.8 rad command, so it bounds nothing the assertion reads.
    prober_children=_hinge_rig("""    BallJoint {
      jointParameters DEF JP BallJointParameters {
        anchor 0 0 0
      }
      device [
        RotationalMotor { name "m" maxTorque 100 minPosition -1.4 maxPosition 1.4 }
      ]
      device2 [
        RotationalMotor { name "m2" maxTorque 100 minPosition -1.4 maxPosition 1.4 }
      ]
      device3 [
        RotationalMotor { name "m3" maxTorque 100 minPosition -1.4 maxPosition 1.4 }
      ]
      endPoint
%s    }
""" % _ARM),
    measure=("jointpos:JP", "pos:SUBJECT"),
    # ⚠ PROBE-CALIBRATION ERROR, CORRECTED 2026-09-01 (rule 3's failure class:
    # a geometric assumption encoded into the rig). Until this date the probe
    # commanded ONLY motor "m", which drives axis 1 -- and with just
    # BallJointParameters declared (anchor only, no parameters2/parameters3),
    # OmBallJoint::axis() defaults axis 1 to (1, 0, 0): exactly the ray the
    # measured arm's origin (0.25, 0, 0) sits on. A rotation about an axis
    # through a point displaces that point by ZERO for ANY angle, so the probe
    # was structurally unable to see actuation even from a perfect engine --
    # its `broken` verdict (2.67e-07 m, published 2026-08-17 and echoed into
    # AGENTS.md) was guaranteed by its own geometry, the mirror image of
    # rule 9: a broken that could not go green. The passing hinge2 sibling
    # never had this problem: it drives +Z and displaces
    # 2 * 0.25 * sin(0.4) = 0.195 m. The primary command now goes to "m3"
    # (device3 -> motor3 -> axis 3, default (0, 0, 1) per
    # OmBallJoint::axis3(), emitted as the joint frame's z gear in
    # registerNewtonMultiDof) -- the same perpendicular-drive geometry the
    # hinge2 probe passes on. "m" is STILL commanded, as a secondary
    # INFORMATIONAL arm only: the axis-1 readback span
    # (`joint_angle_travel_rad`) keeps witnessing "sensor live while the body
    # is still", and axis-1 rotation contributes zero displacement whether or
    # not it actuates, so it confounds nothing the assertion reads.
    act=("motor_position:m3:0.8:0.0", "motor_position:m:0.8:0.0"),
    # GRAVITY OFF, and this is the whole validity of the probe. These joints
    # are free to swing when their motors are ignored, so under gravity the
    # arm moves either way and "the arm moved" proves nothing about the motor.
    # Measured on the first run: the ball probe scored `degraded` on 36.2 mm
    # of pure gravity sag and would have been reported as partial actuation.
    # With g=0 the motor is the ONLY thing that can move the arm.
    gravity=0.0,
    duration=3.0,
    assertion=None,
    # Whether the motorised BallJoint actuates is OPEN until the first
    # post-correction run: AGENTS.md's "does not actuate (measured
    # 2026-08-17)" rests entirely on the axis-1-blind probe above, and the
    # supporting history still stands -- 2094660ef flipped
    # OMNISIM_NEWTON_BALL_HINGE2 on for BallJoint AND Hinge2Joint on the
    # evidence of tests/test_newton_ball_hinge2.py, whose BALL arm is PASSIVE
    # (PositionSensor only, a gravity pendulum), so no test anywhere drives a
    # motorised BallJoint. `documented_as` stays BROKEN because that is what
    # the docs currently claim; if the corrected probe measures `works`, the
    # doc-audit finding it raises is this lane doing its job.
    doc="AGENTS.md — claims motorised BallJoint does NOT actuate; the "
        "2026-08-17 measurement behind that claim commanded axis 1, which the "
        "arm lies along (geometrically blind) — axis-corrected 2026-09-01, "
        "re-measure",
    documented_as=WORKS,  # flipped 2026-09-01: AGENTS.md re-measured (probe geometry corrected; arm displaced 0.1884 m)
))


def _ball_assertion(arrays):
    """A motorised BallJoint commanded to 0.8 rad about its THIRD axis (motor
    "m3", axis3 default (0,0,1) = +Z) in a GRAVITY-FREE world must displace
    the arm origin — 0.25 m out along axis 1 — by 2*0.25*sin(0.4) = 0.195 m,
    at least 5 cm. Axis 3 is the one that carries the verdict because the arm
    lies ALONG the default axis 1 (+X): rotation about an axis through a
    point displaces that point by exactly zero, which is why this probe's
    pre-2026-09-01 form (commanding "m", the axis-1 motor) measured
    2.67e-07 m regardless of what the engine did. Axis 1 is still commanded
    as a secondary, informational arm: `joint_angle_travel_rad` records its
    readback, and it cannot confound the displacement (zero either way).
    Gravity is off so the motors are the only thing that can move the arm.
    The probe reads the ARM POSE, not the joint angle, because the documented
    failure is that the motors are accepted and silently ignored while a
    readback stays live — a joint-angle-only test cannot tell a working joint
    with a dead sensor from a dead joint."""
    p = arrays.get("pos_SUBJECT")
    if p is None or len(p) < 2:
        return Verdict(INCONCLUSIVE, note="arm pose not recorded")
    disp = math.dist(tuple(p[-1]), tuple(p[0]))
    ev = {"arm_displacement_m": disp,
          "expected_displacement_m": 2.0 * 0.25 * math.sin(0.4),
          "joint_angle_travel_rad": _span(arrays, "joint_JP")}
    if disp >= 0.05:
        return Verdict(WORKS, ev)
    if disp < 1e-3:
        return Verdict(
            BROKEN, ev,
            "the BallJoint's motors were accepted but the arm never moved "
            "(%.2e m) with OMNISIM_NEWTON_BALL_HINGE2 ON (its default since "
            "2094660ef), commanded 0.8 rad about axis 3 (+Z), PERPENDICULAR "
            "to the arm -- geometry that must displace the origin by 0.195 m "
            "if the motor tracks. The axis-1 angle READBACK meanwhile travels "
            "%.3f rad, so a sensor is live while the body is not -- the "
            "engine's own warning says the BALL element is emitted with its "
            "per-axis limits unmapped. NOTE: this probe was axis-corrected on "
            "2026-09-01; every earlier broken row (2.67e-07 m) commanded only "
            "axis 1, which the arm lies along, and could not have seen "
            "actuation -- this row, unlike those, is a real engine finding."
            % (disp, _span(arrays, "joint_JP")))
    return Verdict(DEGRADED, ev,
                   "the arm moved only %.1f mm" % (disp * 1e3))


PROBES[-1].assertion = _ball_assertion

_p(Probe(
    id="joint.hinge2_motor",
    family=FAM_JOINT,
    claim="Hinge2Joint with motors — 2-DoF steered/driven axle",
    world=lambda: "",
    # TWO authoring bugs were confounding this probe, both of which the engine
    # reported and neither of which is about Hinge2Joint actuation.
    #
    # 1. `jointParameters2` is declared SFNode JointParameters (Hinge2Joint.wrl),
    #    NOT HingeJointParameters -- the engine refused the node outright
    #    ("Cannot insert HingeJointParameters node in 'jointParameters2' field"),
    #    so axis2 fell back to its default (0,0,1), which EQUALS the authored
    #    axis1, and the engine then reported "Hinge axes are aligned: using x and
    #    z axes instead". The probe never tested the axis pair it authored.
    # 2. The motors declared no position range, so newtonAxisSpec built them as
    #    VELOCITY WHEELS with ke = 0 and setPosition() ignored by design -- see
    #    the BallJoint probe above for the full note.
    #
    # Same shape as the shipped samples (motor2/gyro) and as the passing
    # tests/test_newton_ball_hinge2.py fixture. Limits bound nothing the
    # assertion reads: 1.4 rad against a 0.8 rad command.
    prober_children=_hinge_rig("""    Hinge2Joint {
      jointParameters DEF JP HingeJointParameters {
        axis 0 0 1
        anchor 0 0 0
        minStop -1.5
        maxStop 1.5
      }
      jointParameters2 JointParameters {
        axis 0 1 0
        anchor 0 0 0
        minStop -1.5
        maxStop 1.5
      }
      device [
        RotationalMotor { name "m" maxTorque 100 minPosition -1.4 maxPosition 1.4 }
      ]
      device2 [
        RotationalMotor { name "m2" maxTorque 100 minPosition -1.4 maxPosition 1.4 }
      ]
      endPoint
%s    }
""" % _ARM),
    measure=("jointpos:JP", "pos:SUBJECT"),
    act=("motor_position:m:0.8:0.0",),
    # GRAVITY OFF, and this is the whole validity of the probe. These joints
    # are free to swing when their motors are ignored, so under gravity the
    # arm moves either way and "the arm moved" proves nothing about the motor.
    # Measured on the first run: the ball probe scored `degraded` on 36.2 mm
    # of pure gravity sag and would have been reported as partial actuation.
    # With g=0 the motor is the ONLY thing that can move the arm.
    gravity=0.0,
    duration=3.0,
    assertion=None,
    # See the BallJoint probe: `documented_as` tracks the AGENTS.md claim, and
    # 2094660ef retracted the "does not actuate" entry for both joint types.
    doc="AGENTS.md — OMNISIM_NEWTON_BALL_HINGE2 default ON since 2094660ef",
    documented_as=WORKS,
))


def _hinge2_assertion(arrays):
    """A motorised Hinge2Joint commanded to 0.8 rad in a GRAVITY-FREE world
    must move the arm by at least 5 cm. Read from the arm POSE, and with
    gravity off, for the same two reasons as the BallJoint probe."""
    p = arrays.get("pos_SUBJECT")
    if p is None or len(p) < 2:
        return Verdict(INCONCLUSIVE, note="arm pose not recorded")
    disp = math.dist(tuple(p[-1]), tuple(p[0]))
    ev = {"arm_displacement_m": disp,
          "joint_angle_travel_rad": _span(arrays, "joint_JP")}
    if disp >= 0.05:
        return Verdict(WORKS, ev)
    if disp < 1e-3:
        return Verdict(
            BROKEN, ev,
            "the Hinge2Joint's motors were accepted but the arm never moved "
            "(%.2e m) -- a REGRESSION since 2094660ef, which made "
            "OMNISIM_NEWTON_BALL_HINGE2 default ON. Check that variable is "
            "not set to 0 in the environment before reporting one." % disp)
    return Verdict(DEGRADED, ev, "the arm moved only %.1f mm" % (disp * 1e3))


PROBES[-1].assertion = _hinge2_assertion

_p(Probe(
    id="joint.limits_enforced",
    family=FAM_JOINT,
    claim="HingeJointParameters minStop/maxStop — hard joint limits hold",
    world=lambda: "",
    prober_children=_hinge_rig("""    HingeJoint {
      jointParameters DEF JP HingeJointParameters {
        axis 0 1 0
        anchor 0 0 0
        minStop -0.3
        maxStop 0.3
      }
      endPoint
%s    }
""" % _ARM),
    measure=("jointpos:JP",),
    duration=3.0,
    assertion=None,
))


def _limit_assertion(arrays):
    """A passive arm falling against a maxStop/minStop of +/-0.3 rad must not
    exceed 0.35 rad in magnitude. Sailing past the stop means the limit is
    parsed and not enforced."""
    a = _finite(arrays.get("joint_JP"))
    if not a:
        return Verdict(INCONCLUSIVE, note="joint angle not recorded")
    worst = max(abs(v) for v in a)
    ev = {"max_abs_angle_rad": worst, "stop_rad": 0.3}
    if worst <= 0.35:
        return Verdict(WORKS, ev)
    return Verdict(BROKEN, ev,
                   "the joint reached %.3f rad past a 0.3 rad stop — the "
                   "limit is not enforced" % worst)


PROBES[-1].assertion = _limit_assertion

_p(Probe(
    id="joint.position_sensor",
    family=FAM_JOINT,
    claim="PositionSensor reads back the joint angle the solver holds",
    world=lambda: "",
    prober_children=_hinge_rig("""    HingeJoint {
      jointParameters DEF JP HingeJointParameters {
        axis 0 1 0
        anchor 0 0 0
      }
      device [
        RotationalMotor { name "m" maxTorque 100 }
        PositionSensor { name "ps" }
      ]
      endPoint
%s    }
""" % _ARM),
    measure=("jointpos:JP", "sensor:ps"),
    act=("motor_position:m:0.6:0.0",),
    duration=3.0,
    assertion=None,
))


def _possensor_assertion(arrays):
    """A PositionSensor must agree with the supervisor's own reading of the
    joint's `position` field to within 0.01 rad. Reading a constant 0 while
    the joint moves is the documented BallJoint/Hinge2Joint failure shape,
    and it must not appear on a plain hinge."""
    sens = arrays.get("sensor_ps")
    jp = arrays.get("joint_JP")
    if sens is None or jp is None or len(sens) == 0 or len(jp) == 0:
        return Verdict(INCONCLUSIVE, note="sensor or joint angle not recorded")
    # A PositionSensor reads NaN until its first post-enable sample lands, and
    # the recorder's row 0 is taken BEFORE the first step. Comparing that row
    # propagates NaN through max() and reports "disagree by up to nan" -- an
    # instrument artefact that reads exactly like a dead sensor.
    pairs = [(float(sens[i]), float(jp[i])) for i in range(min(len(sens), len(jp)))
             if sens[i] is not None and jp[i] is not None
             and float(sens[i]) == float(sens[i])
             and float(jp[i]) == float(jp[i])]
    if not pairs:
        return Verdict(INCONCLUSIVE, {"finite_samples": 0},
                       "the PositionSensor produced no finite sample")
    err = max(abs(a - b) for a, b in pairs)
    ev = {"max_abs_disagreement_rad": err, "finite_samples": len(pairs),
          "sensor_travel_rad": max(a for a, _ in pairs) - min(a for a, _ in pairs),
          "field_travel_rad": _span(arrays, "joint_JP")}
    if err <= 0.01:
        return Verdict(WORKS, ev)
    if (_span(arrays, "sensor_ps") or 0.0) < 1e-6:
        return Verdict(BROKEN, ev,
                       "the PositionSensor stayed at 0 while the joint moved")
    return Verdict(DEGRADED, ev,
                   "sensor and solver disagree by up to %.4f rad" % err)


PROBES[-1].assertion = _possensor_assertion


# ===========================================================================
# FAMILY: device / sensor
# ===========================================================================
#: Commanded turntable rate, rad/s. Shared by the rig and by every assertion
#: built on it, so the reference number cannot drift away from the world that
#: produced it.
TURNTABLE_RAD_S = 2.0
#: SUBJECT's offset from the hinge anchor, m. The sensor rides a circle of this
#: radius, which is what gives the GPS probe a moving target instead of a
#: parked one.
TURNTABLE_RADIUS = 0.25


def _turntable(carried, on_prober="", reference=""):
    """A DRIVEN YAW TURNTABLE: the device under test rides a body the engine is
    demonstrably rotating, with an identical device parked on the static prober
    beside it.

    ⚠ WHY THIS RIG EXISTS. `device.gyro`, `device.inertial_unit` and
    `device.gps` all used to measure a STATIONARY robot, and all three
    published `works`. A gyro at rest correctly reads zero -- so
    `omega = [0,0,0]` from a parked body is equally the signature of a working
    device and of a dead one, and the verdict could not be made to go red by
    any engine defect whatsoever. That is rule 9 ("a green that cannot go red
    is not evidence"), and it was not a probe measuring the WRONG thing the way
    the friction probe was: it was a probe measuring NOTHING. The turntable
    supplies the one ingredient that was missing -- a known, independently
    checkable angular rate -- and keeps the old stationary reading as the
    NEGATIVE ARM rather than deleting it, so a device that reports a constant
    (of any value, zero or not) now fails on one arm or the other.

    Three details are load-bearing:

    * **The hinge turns about +Z** (world up under ENU) and SUBJECT starts
      unrotated, so SUBJECT's local Z stays parallel to world Z for the whole
      run. A Gyro riding it reads the rate on one axis and the assertions can
      judge a MAGNITUDE, with no axis-convention guess anywhere in them
      (rule 3 -- the cylinder probe's 149.6 mm miss came from exactly such a
      guess).
    * **Gravity is 0.** A horizontal arm on a vertical hinge is already
      gravity-neutral about the driven axis, so gravity would change no
      reading -- but leaving it on means arguing that, and rule 4 says remove
      the confound instead of reasoning around it. It also matches
      `joint.hinge_motor_velocity`, whose rig this deliberately mirrors.
    * **The devices ride INSIDE the endPoint Solid.** They are still devices of
      the prober Robot (the engine walks the whole robot subtree), so the one
      controller that is allowed to read them still can -- while the body they
      are bolted to is a genuine Newton body driven by a joint, which is the
      case a URDF-imported robot's sensors are in.

    The rotation is verified per-run from the SUPERVISOR's own
    `getOrientation()`, never from the commanded rate: if the motor does not
    turn the arm, every assertion below returns `inconclusive` (rule 2 --
    guard the premise before judging the sensor) rather than blaming the
    device for a rig that never moved.

    ⚠ `reference` MOUNTS THE NEGATIVE ARM ON A REAL PHYSICS BODY, AND THAT IS
    NOT A STYLE CHOICE. `OmGyro::computeValue()` and
    `OmAccelerometer::computeValue()` both early-out unless
    `upperSolid()->bodyHandle()` is non-null -- they read the body's angular
    velocity from the physics backend -- while `OmInertialUnit` and the GPS's
    position channel are computed from `matrix()`, i.e. from the scene graph,
    and need no body at all. So a Gyro parked on the prober Robot (which is
    `physics NULL`, the intangible-observer default) would read zero because it
    has NO BODY TO ASK, not because its body is still: the negative arm would
    be as vacuous as the probe being repaired. It therefore hangs off a
    PASSIVE hinge -- a genuine Newton body, undriven, at gravity 0, so it
    cannot move -- one metre below the driven arm's sweep plane so the two can
    never touch. `quat:REFERENCE` is recorded so that "it did not move" is a
    measurement rather than an assumption.
    """
    ref = ""
    if reference:
        ref = """    HingeJoint {
      jointParameters DEF JP_REF HingeJointParameters {
        axis 0 0 1
        anchor 0 0 -1
      }
      endPoint DEF REFERENCE Solid {
        translation 0 0 -1
        name "reference"
        children [
          DEF REF_SHAPE Shape {
            appearance PBRAppearance { baseColor 0.4 0.6 0.85 roughness 0.9 metalness 0 }
            geometry Box { size 0.1 0.1 0.1 }
          }
%s        ]
        boundingObject USE REF_SHAPE
        physics Physics { density -1 mass 1 }
      }
    }
""" % reference
    return """    DEF POST Shape {
      appearance PBRAppearance { baseColor 0.3 0.3 0.35 roughness 1 metalness 0 }
      geometry Box { size 0.08 0.08 0.08 }
    }
%s    HingeJoint {
      jointParameters DEF JP HingeJointParameters {
        axis 0 0 1
        anchor 0 0 0
      }
      device [
        RotationalMotor { name "m" maxTorque 100 maxVelocity 12 }
      ]
      endPoint DEF SUBJECT Solid {
        translation %s 0 0
        name "arm"
        children [
          DEF ARM_SHAPE Shape {
            appearance PBRAppearance { baseColor 0.85 0.55 0.2 roughness 0.9 metalness 0 }
            geometry Box { size 0.5 0.05 0.05 }
          }
%s        ]
        boundingObject USE ARM_SHAPE
        physics Physics { density -1 mass 1 }
      }
    }
%s""" % (on_prober, _g(TURNTABLE_RADIUS), carried, ref)


def _turntable_rotation(arrays, tail_frac=None):
    """(measured_rad_s, total_rad, elapsed_s) from the supervisor's own record
    of SUBJECT's orientation. `None` rate when the rig did not produce a
    usable orientation series at all.

    `tail_frac` restricts the measurement to the last fraction of the run.
    A velocity motor spends its first steps ramping up, so a whole-window
    average sits BELOW the steady rate -- and comparing a device's tail-mean
    against a whole-window truth would charge the device for the ramp. The
    comparison arms pass tail_frac=0.2 and read the same window the device
    reading is taken from; the premise guard deliberately uses the whole
    window, where "did this thing turn at all" is the only question.
    """
    q = _finite_vecs(arrays.get("quat_SUBJECT"), 4)
    t = arrays.get("t")
    # Slice only when the two series are index-aligned. getOrientation() does
    # not produce NaN, so they normally are -- but a dropped sample would
    # shift the window against the clock, and silently mis-timing the ground
    # truth is exactly the class of error this probe is being repaired for.
    if tail_frac and len(q) > 2 and t is not None and len(t) == len(q):
        k = max(2, int(len(q) * tail_frac))
        q, t = q[-k:], list(t)[-k:]
    total, n = _quat_rotation_total(q)
    elapsed = ((float(t[-1]) - float(t[0]))
               if t is not None and len(t) >= 2 else None)
    if not n or not elapsed:
        return None, total, elapsed
    return total / elapsed, total, elapsed


#: Below this measured rate the turntable did not turn, and nothing about the
#: device riding it follows. Set well under the 2.0 rad/s command but far above
#: solver noise.
TURNTABLE_MIN_RAD_S = 0.5


def _turntable_premise(arrays, device_label):
    """Shared premise guard. Returns a Verdict to hand straight back when the
    rig failed, else None. This is the check whose absence is the entire
    defect these three probes are being repaired for."""
    rate, total, elapsed = _turntable_rotation(arrays)
    if rate is None:
        return Verdict(INCONCLUSIVE,
                       {"supervisor_rotation_rad": total,
                        "elapsed_s": elapsed},
                       "SUBJECT's orientation was not recorded, so the "
                       "turntable's motion is unwitnessed and nothing about "
                       "the %s follows" % device_label)
    if rate < TURNTABLE_MIN_RAD_S:
        return Verdict(INCONCLUSIVE,
                       {"supervisor_measured_rad_s": rate,
                        "commanded_rad_s": TURNTABLE_RAD_S,
                        "supervisor_rotation_rad": total,
                        "elapsed_s": elapsed},
                       "the turntable did not turn (supervisor measured "
                       "%.4f rad/s against a %.1f rad/s command), so this is a "
                       "RIG failure and says nothing about the %s"
                       % (rate, TURNTABLE_RAD_S, device_label))
    return None


_p(Probe(
    id="device.distance_sensor",
    family=FAM_DEVICE,
    claim="DistanceSensor — native raycast against scene geometry",
    world=lambda: floor() + body("WALL", "Box { size 0.2 4 4 }", "2 0 1.5",
                                 physics=False, name="wall"),
    prober_children="""    DistanceSensor {
      name "ds"
      translation 0 0 0
      rotation 0 0 1 0
      lookupTable [ 0 0 0, 5 5 0 ]
    }
""",
    measure=("sensor:ds",),
    duration=1.0,
    assertion=None,
    doc="AGENTS.md — raycast went native + default ON in 6eb35675",
    documented_as=WORKS,
))


def _distance_assertion(arrays):
    """A DistanceSensor at the origin facing +x must read the wall standing at
    x=2.0 m (near face at 1.9 m) to within 0.15 m. A reading pinned at the
    lookup-table maximum means no ray is being cast."""
    a = _finite(arrays.get("sensor_ds"))
    if not a:
        return Verdict(INCONCLUSIVE, note="no finite DistanceSensor reading")
    v = a[-1]
    ev = {"reading_m": v, "expected_m": 1.9, "abs_err_m": abs(v - 1.9)}
    if abs(v - 1.9) <= 0.15:
        return Verdict(WORKS, ev)
    if v >= 4.99:
        return Verdict(BROKEN, ev,
                       "the sensor is pinned at its range maximum (%.3f m) "
                       "with a wall 1.9 m away — no ray is being cast" % v)
    return Verdict(DEGRADED, ev,
                   "the sensor read %.3f m against a wall at 1.9 m" % v)


PROBES[-1].assertion = _distance_assertion

_p(Probe(
    id="device.touch_bumper",
    family=FAM_DEVICE,
    claim="TouchSensor, BUMPER type — boolean contact detection",
    # The prober robot itself is dropped onto the floor with a bumper on it.
    world=lambda: floor(),
    # The prober itself is the falling body: a 0.2 m box with a thin bumper
    # pad on its underside, so the pad is the first thing to touch the floor.
    prober_translation="0 0 1.2",
    # Without this the prober's own boundingObject is not a collider at all
    # (worldinfo.md: robot wrappers collide through wheels/feet by default).
    world_info="  newtonRobotColliders TRUE\n",
    prober_children=_bumper_pad(mass=1.0, color="0.9 0.7 0.2"),
    prober_bounding="  boundingObject USE PROBER_BODY\n",
    prober_physics="  physics Physics { density -1 mass 1 }\n",
    measure=("sensor:ts", "pos:OMNIBENCH_PROBER"),
    duration=3.0,
    assertion=None,
    doc="ee069b326 (2026-08-13) made a bumper's precondition a COLLIDER rather "
        "than a body -- isUnfoldedTouchSensor() gates on boundingObject() "
        "instead of physics() -- so the pad becomes its own Newton body with "
        "its own shapes. This probe refuted the old 'native and working' claim "
        "in 2026-08-10 and then went on certifying the FIXED device as broken "
        "until 2026-08-15; the rest-height arm is what makes it honest either "
        "way. src/omnisim/nodes/OmSolid.cpp + AGENTS.md",
    documented_as=WORKS,
))


def _bumper_assertion(arrays):
    """A bumper TouchSensor whose pad protrudes 10 mm below the robot body
    must bring the prober to rest at z=0.66 m -- proving the PAD is the
    geometry in contact -- and then read 1 while resting and 0 while airborne.

    Resting at 0.65 m instead is a sharper measurement than a sensor value:
    it means the body's own collider took the contact and the TouchSensor's
    boundingObject never became a Newton collider, so the device cannot see a
    touch that is not happening to its geometry. The 10 mm offset exists
    solely to make those two hypotheses separable by measurement."""
    # A TouchSensor reads NaN until its first post-enable sample lands, and
    # row 0 is taken before the first step -- so the raw series starts NaN and
    # a naive read reports "already read nan at t=0".
    a = _finite(arrays.get("sensor_ts"))
    if not a:
        return Verdict(INCONCLUSIVE, note="no finite TouchSensor reading")
    first = a[0]
    ever = max(a)
    z = _final(arrays, "pos_OMNIBENCH_PROBER", 2)
    ev = {"first_finite_value": first, "max_value": ever,
          "finite_samples": len(a), "prober_rest_z_m": z,
          "expected_rest_z_m": TOUCH_REST_Z,
          "fraction_in_contact": sum(1 for v in a if v > 0.5) / len(a)}
    ev["rest_z_if_pad_collides"] = TOUCH_REST_Z
    ev["rest_z_if_only_body_collides"] = BODY_REST_Z
    # The REST HEIGHT decides which of two very different things is being
    # measured, so it is read before the sensor value is judged at all.
    if z is None:
        return Verdict(INCONCLUSIVE, ev, "prober pose not recorded")
    pad_collides = abs(z - TOUCH_REST_Z) <= 0.005
    body_only = abs(z - BODY_REST_Z) <= 0.005
    if not pad_collides and not body_only:
        return Verdict(INCONCLUSIVE, ev,
                       "the prober rests at z=%.4f m, matching neither the "
                       "pad-contact (%.2f) nor the body-contact (%.2f) "
                       "hypothesis, so nothing about the sensor follows"
                       % (z, TOUCH_REST_Z, BODY_REST_Z))
    if body_only:
        return Verdict(
            BROKEN, ev,
            "the TouchSensor's boundingObject is not a collider: its pad "
            "protrudes 10 mm below the body, yet the body took the contact "
            "(rest z=%.4f; pad contact would rest at %.2f) and the sensor "
            "reads 0 throughout. The device cannot see a contact that is not "
            "happening to its own geometry." % (z, TOUCH_REST_Z))
    if ever > 0.5 and first < 0.5:
        return Verdict(WORKS, ev)
    if ever <= 0.5:
        return Verdict(BROKEN, ev,
                       "the bumper's own pad IS the contact geometry (rest "
                       "z=%.4f) and it still never reported a touch" % z)
    return Verdict(DEGRADED, ev,
                   "the bumper already read %.1f at t=0, before touchdown"
                   % first)


PROBES[-1].assertion = _bumper_assertion

_p(Probe(
    id="device.touch_force",
    family=FAM_DEVICE,
    claim="TouchSensor, FORCE type — contact force in newtons",
    world=lambda: floor(),
    prober_translation="0 0 1.2",
    # Without this the prober's own boundingObject is not a collider at all
    # (worldinfo.md: robot wrappers collide through wheels/feet by default).
    world_info="  newtonRobotColliders TRUE\n",
    prober_children=_bumper_pad(mass=2.0, color="0.9 0.4 0.2",
                                sensor_type="force"),
    prober_bounding="  boundingObject USE PROBER_BODY\n",
    prober_physics="  physics Physics { density -1 mass 2 }\n",
    measure=("sensor:ts", "pos:OMNIBENCH_PROBER"),
    duration=4.0,
    assertion=None,
    doc="the un-fold lives in src/omnisim/nodes/OmSolid.cpp (ee069b326), NOT in "
        "OmNewtonBackend.cpp as this entry used to say. The 0 N this probe "
        "reported after that fix was its OWN rig: a force-type pad needs "
        "physics (the wrench is served from cfrc_int), a rotation (the value "
        "is the projection onto +X) and a cleared lookupTable (the default is "
        "a 10x gain). See _bumper_pad. Canonical rig: "
        "tests/test_newton_touch_force_parity.py",
    documented_as=WORKS,
))


def _touchforce_assertion(arrays):
    """A force-type TouchSensor under a resting 2 kg body must read its weight,
    2*9.81 = 19.62 N, to within 30% once settled (contact springs make an
    exact figure unreasonable; an order of magnitude is the real question).

    Like the bumper probe, the REST HEIGHT is read first: at 0.65 m the pad
    never touched anything and the 0 N is a consequence of the sensor's
    boundingObject not being a collider, not an independent force-readout
    defect. The two failure modes have different fixes, so the probe
    distinguishes them rather than reporting whichever is more dramatic."""
    a = _finite(arrays.get("sensor_ts"))
    z = arrays.get("pos_OMNIBENCH_PROBER")
    if not a:
        return Verdict(INCONCLUSIVE, note="no finite TouchSensor reading")
    tail = a[-max(1, len(a) // 5):]
    mean_tail = sum(tail) / len(tail)
    settled_z = float(z[-1][2]) if z is not None and len(z) else None
    ev = {"mean_force_N_last_fifth": mean_tail, "expected_N": 19.62,
          "prober_rest_z_m": settled_z}
    ev["rest_z_if_pad_collides"] = TOUCH_REST_Z
    ev["rest_z_if_only_body_collides"] = BODY_REST_Z
    if settled_z is None:
        return Verdict(INCONCLUSIVE, ev, "prober pose not recorded")
    if abs(settled_z - BODY_REST_Z) <= 0.005:
        return Verdict(
            BROKEN, ev,
            "the TouchSensor's boundingObject is not a collider: its pad "
            "protrudes 10 mm below the body, yet the body took the contact "
            "(rest z=%.4f; pad contact would rest at %.2f). A force-type "
            "sensor whose geometry never touches anything necessarily reads "
            "0 N, so the 0 N is a CONSEQUENCE of this, not an independent "
            "defect." % (settled_z, TOUCH_REST_Z))
    if abs(settled_z - TOUCH_REST_Z) > 0.005:
        return Verdict(INCONCLUSIVE, ev,
                       "the sensor body did not come to rest on the floor "
                       "(z=%s), so no contact force is expected"
                       % round(settled_z, 4))
    if abs(mean_tail - 19.62) <= 0.30 * 19.62:
        return Verdict(WORKS, ev)
    if abs(mean_tail) < 1e-6:
        return Verdict(BROKEN, ev,
                       "a force-type TouchSensor under a resting 2 kg body "
                       "reads 0.000 N")
    return Verdict(DEGRADED, ev,
                   "the sensor read %.3f N under a 19.62 N load" % mean_tail)


PROBES[-1].assertion = _touchforce_assertion

_p(Probe(
    id="device.contact_points_api",
    family=FAM_DEVICE,
    claim="Supervisor getContactPoints — a resting body reports its contacts",
    world=lambda: floor() + body("SUBJECT", "Box { size 0.2 0.2 0.2 }",
                                 "0 0 1.0"),
    measure=("pos:SUBJECT", "contacts:SUBJECT"),
    duration=3.0,
    assertion=None,
    doc="AGENTS.md — native contact readback default ON since 2026-08-07",
    documented_as=WORKS,
))


def _contacts_assertion(arrays):
    """A 0.2 m box demonstrably at rest on the floor (z=0.65 m) must report at
    least one contact point. Zero contacts under a resting body is the
    'contact-blind' state measured before 2026-08-07 — and it is
    indistinguishable from 'nothing is touching' to any caller."""
    c = _finite(arrays.get("contacts_SUBJECT"))
    z = _final(arrays, "pos_SUBJECT", 2)
    if not c:
        return Verdict(INCONCLUSIVE, note="no contact counts recorded")
    tail = c[-max(1, len(c) // 5):]
    ev = {"mean_contacts_last_fifth": sum(tail) / len(tail),
          "max_contacts": max(c), "rest_z_m": z}
    if z is None or abs(z - TOUCH_REST_Z) > 0.02:
        return Verdict(INCONCLUSIVE, ev,
                       "the box is not resting on the floor (z=%s), so a "
                       "contact count proves nothing" % (None if z is None
                                                         else round(z, 4)))
    if ev["mean_contacts_last_fifth"] >= 1.0:
        return Verdict(WORKS, ev)
    return Verdict(BROKEN, ev,
                   "a box measurably at rest on the floor reports %.2f "
                   "contact points" % ev["mean_contacts_last_fifth"])


PROBES[-1].assertion = _contacts_assertion

_p(Probe(
    id="device.gps",
    family=FAM_DEVICE,
    claim="GPS — absolute position readout, TRACKED while the body moves",
    # Was: one GPS on a parked prober, compared against the supervisor at a
    # single instant. Two of that reading's three components were 0.0 and the
    # body never moved, so a GPS frozen at its spawn pose passed it exactly --
    # which is the defect class the ROS 2 lane found on a URDF Husky. The
    # turntable makes the target move; `gps_path_len_m` is the discriminator a
    # single-instant comparison cannot make.
    gravity=0.0,
    world=lambda: "",
    prober_children=_turntable(
        carried="""          GPS { name "gps_spin" }
""",
        on_prober="""    GPS { name "gps_static" }
"""),
    measure=("sensor:gps_spin", "sensor:gps_static", "pos:SUBJECT",
             "pos:OMNIBENCH_PROBER", "quat:SUBJECT"),
    act=("motor_velocity:m:%s:0.0" % _g(TURNTABLE_RAD_S),),
    duration=2.0,
    assertion=None,
))


def _gps_assertion(arrays):
    """A GPS riding a turntable arm must TRACK it: agreeing with the
    supervisor's own world position for the same body to within 1 cm at every
    sample, while covering the same path length (the 0.25 m arm sweeps about
    1.0 m in the 2 s window).

    The path length is the load-bearing half. A GPS frozen at its spawn pose
    agrees with the supervisor exactly once, at t=0, and travels 0 m -- and
    the parked-robot version of this probe, which compared one instant on a
    body that never moved, scored that failure `works`."""
    prem = _turntable_premise(arrays, "GPS")
    if prem is not None:
        return prem
    pairs = _paired(arrays, "sensor_gps_spin", "pos_SUBJECT")
    truth_len = _path_len(arrays.get("pos_SUBJECT"))
    gps_len = _path_len(arrays.get("sensor_gps_spin"))
    static = _paired(arrays, "sensor_gps_static", "pos_OMNIBENCH_PROBER")
    ev = {"gps_path_len_m": gps_len, "supervisor_path_len_m": truth_len,
          "samples_compared": len(pairs)}
    if not pairs or truth_len is None:
        return Verdict(INCONCLUSIVE, ev, "GPS or pose not recorded")
    err = max(max(abs(g[i] - p[i]) for i in range(3)) for g, p in pairs)
    ev["max_abs_err_m"] = err
    ev["final_gps_xyz_m"] = pairs[-1][0]
    ev["final_supervisor_xyz_m"] = pairs[-1][1]
    # NEGATIVE ARM: the parked GPS beside it. Its job is to fail if the device
    # reports motion that is not happening -- the mirror of the frozen-readout
    # failure the positive arm catches.
    if static:
        static_err = max(max(abs(g[i] - p[i]) for i in range(3))
                         for g, p in static)
        static_len = _path_len(arrays.get("sensor_gps_static"))
        ev["static_gps_err_m"] = static_err
        ev["static_gps_path_len_m"] = static_len
    else:
        static_err, static_len = None, None
    if gps_len is not None and truth_len >= 0.2 and gps_len < 0.02:
        return Verdict(BROKEN, ev,
                       "the GPS is FROZEN: the body it is mounted on travelled "
                       "%.3f m and the device's own readout travelled %.4f m"
                       % (truth_len, gps_len))
    if err > 0.01:
        return Verdict(DEGRADED, ev,
                       "GPS and supervisor pose disagree by up to %.4f m while "
                       "the body moves" % err)
    if static_err is not None and static_err > 0.01:
        return Verdict(DEGRADED, ev,
                       "the PARKED GPS disagrees with its own body by %.4f m"
                       % static_err)
    if static_len is not None and static_len > 0.02:
        return Verdict(DEGRADED, ev,
                       "the PARKED GPS reports %.4f m of travel on a body that "
                       "never moves" % static_len)
    return Verdict(WORKS, ev)


PROBES[-1].assertion = _gps_assertion

#: Authored tilt of the static reference IMU, rad. An ANALYTIC target: the
#: device is rotated by exactly this much and nothing in the sim can change it.
IMU_TILT_RAD = 0.3

_p(Probe(
    id="device.inertial_unit",
    family=FAM_DEVICE,
    claim="InertialUnit — orientation readout, TRACKED through a rotation",
    # Was: one IMU on a level, parked prober, asserting roll and pitch are
    # ~0 -- which reads 0.0/0.0/0.0, exactly what a dead device returns. Three
    # IMUs now: one riding the turntable (yaw must sweep with the body), one
    # authored at a known tilt (an analytic angle a zeros-returning device
    # cannot produce), and the original level one kept as the negative arm.
    gravity=0.0,
    world=lambda: "",
    prober_children=_turntable(
        carried="""          InertialUnit { name "imu_spin" }
""",
        on_prober="""    InertialUnit { name "imu_level" }
    InertialUnit { name "imu_tilt" rotation 1 0 0 %s }
""" % _g(IMU_TILT_RAD)),
    measure=("sensor:imu_spin", "sensor:imu_level", "sensor:imu_tilt",
             "quat:SUBJECT"),
    act=("motor_velocity:m:%s:0.0" % _g(TURNTABLE_RAD_S),),
    duration=2.0,
    assertion=None,
))


def _imu_assertion(arrays):
    """An InertialUnit riding a turntable must sweep a yaw travel matching the
    ~4 rad rotation the supervisor independently measured to within 10%, while
    a second unit at an authored 0.3 rad tilt reads 0.3 +/- 0.02 and a third,
    mounted level, reads flat -- all three in one run.

    The tilt is asserted as sqrt(roll^2 + pitch^2) rather than as a named
    channel deliberately: a rotation of 0.3 rad about ANY horizontal axis has
    tilt magnitude 0.3, so the claim holds whichever axis the engine calls
    roll, and the probe cannot fail on a convention disagreement it was never
    trying to test (rule 3). Zeros fail it either way, which is the point --
    the parked, level-only version of this probe published `works` on a
    reading of 0.0/0.0/0.0."""
    prem = _turntable_premise(arrays, "InertialUnit")
    if prem is not None:
        return prem
    rate, total, _ = _turntable_rotation(arrays)
    # Take the LARGEST channel travel rather than indexing yaw directly. The
    # turntable is a pure rotation about world Z, so a correct RPY conversion
    # puts all of it in yaw -- but hardcoding index 2 would make this probe
    # fail on an Euler-convention disagreement it is not trying to test, which
    # is precisely how the cylinder probe missed by 149.6 mm (rule 3).
    travels = [_unwrapped_travel(arrays.get("sensor_imu_spin"), i)
               for i in range(3)]
    yaw_travel = max([t for t in travels if t is not None], default=None)
    tilt = _finite_vecs(arrays.get("sensor_imu_tilt"))
    level = _finite_vecs(arrays.get("sensor_imu_level"))
    ev = {"yaw_travel_rad": yaw_travel, "supervisor_rotation_rad": total,
          "supervisor_measured_rad_s": rate,
          "channel_travel_rad": travels}
    if tilt:
        ev["tilt_magnitude_rad"] = _norm(tilt[-1][:2])
        ev["expected_tilt_rad"] = IMU_TILT_RAD
        ev["tilt_rpy_rad"] = tilt[-1]
    if level:
        ev["level_tilt_rad"] = _norm(level[-1][:2])
    if yaw_travel is None:
        return Verdict(INCONCLUSIVE, ev,
                       "the turntable InertialUnit produced no reading")
    # The verdict a stationary probe could never reach: the body demonstrably
    # turned and the device did not notice.
    if yaw_travel < 0.05:
        return Verdict(BROKEN, ev,
                       "the InertialUnit is FROZEN: the supervisor measured "
                       "%.3f rad of rotation and the device's yaw travelled "
                       "%.4f rad" % (total, yaw_travel))
    if abs(yaw_travel - total) > 0.10 * max(total, 1e-9):
        return Verdict(DEGRADED, ev,
                       "the InertialUnit's yaw travelled %.3f rad while the "
                       "body turned %.3f rad" % (yaw_travel, total))
    if not tilt:
        return Verdict(INCONCLUSIVE, ev, "the tilted InertialUnit produced no "
                                         "reading")
    if abs(ev["tilt_magnitude_rad"] - IMU_TILT_RAD) > 0.02:
        return Verdict(DEGRADED, ev,
                       "an InertialUnit authored at a %.2f rad tilt reports a "
                       "tilt magnitude of %.4f rad"
                       % (IMU_TILT_RAD, ev["tilt_magnitude_rad"]))
    # NEGATIVE ARM: the level device must still read level. This is the
    # original probe, kept rather than deleted, so a device that reports a
    # constant non-zero attitude fails here even though it passes above.
    if level and ev["level_tilt_rad"] > 0.02:
        return Verdict(DEGRADED, ev,
                       "a LEVEL InertialUnit reads a tilt of %.4f rad"
                       % ev["level_tilt_rad"])
    return Verdict(WORKS, ev)


PROBES[-1].assertion = _imu_assertion

_p(Probe(
    id="device.gyro",
    family=FAM_DEVICE,
    claim="Gyro — angular velocity readout, MEASURED under rotation",
    # ⚠ THE PROBE THIS WHOLE REPAIR IS NAMED AFTER. It used to read one Gyro on
    # a PARKED prober and publish `works` on evidence of
    # `omega = [0,0,0], magnitude = 0.0` -- a reading a dead gyro produces
    # identically, so the verdict could not distinguish the two and was
    # certifying the device on that basis. Its own scope string said as much
    # ("accuracy under rotation is NOT tested by this probe") and the verdict
    # was published as `works` anyway, which is how a documented limitation
    # became a capability claim.
    #
    # It mattered immediately: the ROS 2 sensor lane measured a URDF-imported
    # Husky's Gyro reading a constant [0,0,0] WHILE THE ROBOT WAS ROTATING,
    # with the InertialUnit in the same carrier Solid tracking yaw to 4
    # decimals. Lane 4 said the gyro worked, the ROS lane said it was dead, and
    # both were honest -- this probe had never asked the question.
    gravity=0.0,
    world=lambda: "",
    prober_children=_turntable(
        carried="""          Gyro { name "gyro_spin" }
""",
        # NOT on the prober: a Gyro there has no body to ask (see _turntable).
        reference="""          Gyro { name "gyro_static" }
"""),
    measure=("sensor:gyro_spin", "sensor:gyro_static", "quat:SUBJECT",
             "pos:SUBJECT", "quat:REFERENCE", "jointpos:JP"),
    act=("motor_velocity:m:%s:0.0" % _g(TURNTABLE_RAD_S),),
    duration=2.0,
    assertion=None,
    doc="src/omnisim/nodes/OmGyro.cpp computeValue() reads the angular "
        "velocity of upperSolid()->bodyHandle() and, when that is null, writes "
        "NO value and warns \"this node or its parents requires a 'physics' "
        "field to be functional\". OmAccelerometer does the same; "
        "OmInertialUnit and the GPS position channel are computed from "
        "matrix() and need no body. That asymmetry is a complete explanation "
        "of the ROS 2 lane's URDF-Husky finding (Gyro constant [0,0,0] and "
        "Accelerometer silent while the InertialUnit in the SAME carrier "
        "Solid tracked yaw), so this probe mounts BOTH gyros on real Newton "
        "bodies -- otherwise it would measure that authoring requirement and "
        "misreport it as a device defect.",
))


def _gyro_assertion(arrays):
    """A Gyro riding a turntable driven at 2.0 rad/s must report that rate to
    within 10% of the rotation the supervisor independently measured, while a
    second Gyro on a body proven at rest reads |omega| < 0.05 rad/s in the
    same run.

    The two arms together are what make the verdict falsifiable: the spinning
    arm fails a dead device (which reads zero while the body turns) and the
    resting arm fails a device that reports a constant (which would otherwise
    pass the spinning arm by luck). The ground truth is the supervisor's
    measured rotation, not the commanded 2.0 rad/s, so a motor that
    under-delivers is a rig finding rather than a gyro finding."""
    prem = _turntable_premise(arrays, "Gyro")
    if prem is not None:
        return prem
    _, total, elapsed = _turntable_rotation(arrays)
    # Ground truth over the SAME tail window the device reading is taken from
    # (see _turntable_rotation): a velocity motor's ramp-up otherwise shows up
    # as a device error it did not commit.
    rate, _, _ = _turntable_rotation(arrays, tail_frac=0.2)
    spin = _finite_vecs(arrays.get("sensor_gyro_spin"))
    static = _finite_vecs(arrays.get("sensor_gyro_static"))
    ev = {"supervisor_measured_rad_s": rate, "commanded_rad_s": TURNTABLE_RAD_S,
          "supervisor_rotation_rad": total, "elapsed_s": elapsed}
    if not spin:
        return Verdict(INCONCLUSIVE, ev,
                       "the turntable Gyro produced no finite reading")
    # Tail mean: the motor needs a few steps to reach its commanded rate, and
    # the same window is used for the supervisor's own figure, so both sides
    # are measured over the identical interval.
    tail = spin[-max(1, len(spin) // 5):]
    mag = sum(_norm(v) for v in tail) / len(tail)
    axis = max(range(3), key=lambda i: abs(sum(v[i] for v in tail) / len(tail)))
    ev["gyro_magnitude_rad_s"] = mag
    ev["gyro_omega_rad_s"] = [sum(v[i] for v in tail) / len(tail)
                              for i in range(3)]
    ev["dominant_axis"] = "xyz"[axis]
    ev["ratio_to_supervisor"] = (mag / rate) if rate else None
    # The negative arm's OWN premise: the reference body must be provably at
    # rest, or "its gyro reads zero" says nothing either.
    ref_total, ref_n = _quat_rotation_total(arrays.get("quat_REFERENCE"))
    ref_still = bool(ref_n) and ref_total < 0.01
    ev["reference_rotation_rad"] = ref_total
    if static:
        ev["static_gyro_magnitude_rad_s"] = max(_norm(v) for v in static)
    # THE VERDICT THE PARKED PROBE COULD NEVER REACH.
    if mag < 0.05:
        return Verdict(BROKEN, ev,
                       "the Gyro reads |omega| = %.4f rad/s while the body it "
                       "is mounted on is measurably turning at %.4f rad/s -- "
                       "the device is accepted and reports nothing"
                       % (mag, rate))
    if abs(mag - rate) > 0.10 * rate:
        return Verdict(DEGRADED, ev,
                       "the Gyro reads %.4f rad/s against a supervisor-measured "
                       "%.4f rad/s" % (mag, rate))
    # NEGATIVE ARM, and the ONLY thing the old probe measured: a Gyro on a body
    # that is not moving must read zero. Kept so a device that returns a
    # constant fails here even after passing the arm above. Judged only when
    # the supervisor confirms the reference body really did stay put.
    if static and ref_still and ev["static_gyro_magnitude_rad_s"] > 0.05:
        return Verdict(DEGRADED, ev,
                       "the RESTING Gyro reads |omega| = %.4f rad/s on a body "
                       "the supervisor measured as motionless (%.5f rad)"
                       % (ev["static_gyro_magnitude_rad_s"], ref_total))
    return Verdict(WORKS, ev)


PROBES[-1].assertion = _gyro_assertion

_p(Probe(
    id="device.accelerometer",
    family=FAM_DEVICE,
    claim="Accelerometer — proper-acceleration readout, MEASURED under "
          "rotation and under gravity",
    # NEW 2026-09-01. The tree ships the device, the URDF importer emits one
    # in every IMU cluster, and this lane had NO row for it — while
    # tests/api/worlds/accelerometer.omniworld has carried a RED, correctly
    # diagnosed assertion for years behind `skip: true` in
    # tests/smoke/smoke_worlds.json ("Parent of Accelerometer node has no
    # physics: measurements may be wrong"). A matrix that cannot see skipped
    # tests reports a known-broken capability as merely untested — rule 12.
    #
    # GRAVITY STAYS ON, unlike the Gyro/IMU/GPS turntable probes, and that is
    # deliberate rather than a lapse of rule 4: an accelerometer's contract
    # is PROPER acceleration, so its at-rest reading is g upward — the one
    # analytic constant a dead device's zeros cannot fake. At g=0 the resting
    # arm would read [0,0,0], the exact ambiguous evidence rule 10 exists to
    # ban. Gravity adds no confound to this rig: the arm rides a VERTICAL
    # hinge (axis +Z), so gravity has no moment about the driven axis and the
    # joint carries the weight — the same reasoning _turntable's own comment
    # already makes, resolved the other way because here gravity IS the
    # signal.
    world=lambda: "",
    prober_children=_turntable(
        carried="""          Accelerometer { name "acc_spin" }
""",
        # On a real, passive-hinge body, NOT the prober: OmAccelerometer
        # reads upperSolid()->bodyHandle() and writes NO value when that is
        # null (see _turntable and README rule 11).
        reference="""          Accelerometer { name "acc_static" }
"""),
    measure=("sensor:acc_spin", "sensor:acc_static",
             "device_exists:acc_spin", "device_exists:acc_static",
             "quat:SUBJECT", "pos:SUBJECT", "quat:REFERENCE"),
    act=("motor_velocity:m:%s:0.0" % _g(TURNTABLE_RAD_S),),
    duration=2.0,
    assertion=None,
    doc="AGENTS.md ROS-2 sensor lane: 'Accelerometer never produces a sample "
        "at all — not even gravity', measured on a URDF Husky whose IMU "
        "carrier is a folded nested Solid with no Newton body (README rule 11 "
        "localises that defect to the CARRIER, not the device). This probe "
        "mounts the device DIRECTLY on the driven endPoint Solid — a real "
        "body — so it measures the DEVICE; the folded-carrier case is "
        "device.imu_nested_carrier's arm.",
))


def _accelerometer_assertion(arrays):
    """An Accelerometer riding a 0.25 m turntable arm driven at 2.0 rad/s
    under gravity 9.81 must read the centripetal acceleration omega^2 * r
    (~1.0 m/s^2, judged against the rotation rate the supervisor
    independently measured, within 0.3 m/s^2 or 25%) in its horizontal
    channels ON TOP of a 9.81 +/- 0.5 m/s^2 gravity reaction in its vertical
    channel, while a second Accelerometer on a passive-hinge body the
    supervisor proves motionless reads ~[0, 0, 9.81] (horizontal < 0.3,
    vertical 9.81 +/- 0.5) in the same run. Channel magnitudes only — no
    sign convention is asserted (rule 3). A device that is accepted and
    produces no finite sample, or a near-zero magnitude, under standing
    gravity is broken: gravity is the analytic constant zeros cannot fake,
    which is why this probe — alone among the turntable four — keeps
    gravity on."""
    prem = _turntable_premise(arrays, "Accelerometer")
    if prem is not None:
        return prem
    # Rate over the SAME tail window the device reading is taken from, so the
    # velocity motor's ramp-up is not charged to the device (see _gyro).
    rate, total, _ = _turntable_rotation(arrays, tail_frac=0.2)
    expected_c = rate * rate * TURNTABLE_RADIUS
    spin = _finite_vecs(arrays.get("sensor_acc_spin"))
    static = _finite_vecs(arrays.get("sensor_acc_static"))
    ev = {"supervisor_measured_rad_s": rate,
          "supervisor_rotation_rad": total,
          "expected_centripetal_m_s2": expected_c,
          "expected_gravity_m_s2": 9.81}
    if not spin:
        if arrays.get("device_exists_acc_spin") is False:
            return Verdict(INCONCLUSIVE, ev,
                           "the turntable Accelerometer device was never "
                           "found — an instrument failure, not a reading")
        return Verdict(BROKEN, ev,
                       "the Accelerometer was accepted and produced NO finite "
                       "sample in 2 s on a body that is measurably turning "
                       "under 9.81 m/s^2 gravity — "
                       "OmAccelerometer::computeValue writes no value at all "
                       "when it cannot serve a reading, so an all-NaN series "
                       "is the device publishing nothing, not a warm-up "
                       "artefact (rule 5 filters only the pre-enable rows)")
    tail = spin[-max(1, len(spin) // 5):]
    horiz = sum(math.hypot(v[0], v[1]) for v in tail) / len(tail)
    vert = sum(abs(v[2]) for v in tail) / len(tail)
    mag = sum(_norm(v) for v in tail) / len(tail)
    ev["spin_horizontal_m_s2"] = horiz
    ev["spin_vertical_abs_m_s2"] = vert
    ev["spin_magnitude_m_s2"] = mag
    ev["spin_mean_xyz_m_s2"] = [sum(v[i] for v in tail) / len(tail)
                                for i in range(3)]
    if mag < 0.5:
        return Verdict(BROKEN, ev,
                       "the Accelerometer reads |a| = %.4f m/s^2 while its "
                       "body turns at %.2f rad/s under 9.81 m/s^2 gravity — "
                       "a working device cannot read below ~9.8 here, and "
                       "near-zero is the reading a dead one produces"
                       % (mag, rate))
    if abs(vert - 9.81) > 0.5:
        return Verdict(DEGRADED, ev,
                       "the gravity channel reads %.3f m/s^2 against the "
                       "analytic 9.81" % vert)
    if abs(horiz - expected_c) > max(0.3, 0.25 * expected_c):
        return Verdict(DEGRADED, ev,
                       "the horizontal channels read %.3f m/s^2 of "
                       "centripetal acceleration against the analytic %.3f "
                       "(= measured rate^2 x %.2f m)"
                       % (horiz, expected_c, TURNTABLE_RADIUS))
    # NEGATIVE ARM: the resting device. Its own premise first (rule 2) — the
    # reference body must be provably at rest, witnessed by quat:REFERENCE.
    ref_total, ref_n = _quat_rotation_total(arrays.get("quat_REFERENCE"))
    ref_still = bool(ref_n) and ref_total < 0.01
    ev["reference_rotation_rad"] = ref_total
    if not static:
        if arrays.get("device_exists_acc_static") is False:
            return Verdict(INCONCLUSIVE, ev,
                           "the resting Accelerometer device was never found "
                           "— an instrument failure, not a reading")
        return Verdict(DEGRADED, ev,
                       "the RESTING Accelerometer produced no finite sample "
                       "while the identical device on the driven arm reads — "
                       "under standing gravity a resting accelerometer must "
                       "read ~9.81 m/s^2, not nothing")
    stail = static[-max(1, len(static) // 5):]
    s_h = sum(math.hypot(v[0], v[1]) for v in stail) / len(stail)
    s_v = sum(abs(v[2]) for v in stail) / len(stail)
    ev["static_horizontal_m_s2"] = s_h
    ev["static_vertical_abs_m_s2"] = s_v
    if ref_still and (s_h > 0.3 or abs(s_v - 9.81) > 0.5):
        return Verdict(DEGRADED, ev,
                       "the RESTING Accelerometer reads [h=%.3f, |v|=%.3f] "
                       "m/s^2 on a body the supervisor measured as motionless "
                       "(%.5f rad of rotation) — expected ~[0, 0, 9.81]"
                       % (s_h, s_v, ref_total))
    return Verdict(WORKS, ev)


PROBES[-1].assertion = _accelerometer_assertion

_p(Probe(
    id="device.imu_nested_carrier",
    family=FAM_DEVICE,
    claim="Gyro + Accelerometer on a folded nested-Solid carrier — the URDF "
          "importer's exact IMU emission pattern",
    # NEW 2026-09-01, and built to go RED NOW and GREEN LATER. The tree's
    # known IMU defect is that a Gyro/Accelerometer on a FOLDED carrier — a
    # nested Solid with no joint between it and its parent, which is exactly
    # what the URDF importer emits for every IMU cluster
    # (OmUrdfImporter.cpp:1103: boundingObject Box 0.001^3 + physics
    # Physics { density -1 mass 0.001 }) — reads zeros or nothing, because
    # declaring `physics` on a jointless nested Solid does not give it a
    # Newton body and OmGyro/OmAccelerometer::computeValue have no
    # bodyHandle to ask (README rule 11 measured the gyro form of this:
    # [0, 0, 0.0] on the fold vs [0, 0, 2.0000] direct, same run). With the
    # current engine this probe is therefore EXPECTED broken; after the
    # pending carrierBodyHandle fix it must go green — which is the point:
    # a green that can go red, and a red with a fix that must flip it
    # (rule 9). The direct-mount pair in the same world is the in-run
    # CONTROL, so a broken here can never be a rig artefact: the identical
    # devices, on the same driven body, read correctly in the same run or
    # the verdict is inconclusive.
    gravity=0.0,
    world=lambda: "",
    prober_children=_turntable(
        # The IMU_CARRIER fold is offset straight UP so its horizontal
        # radius — the centripetal lever arm — is identical to the direct
        # devices' 0.25 m: a rigid fold co-rotates, so every expected value
        # is the same as the direct arm's by construction.
        carried="""          Gyro { name "gyro_direct" }
          Accelerometer { name "acc_direct" }
          DEF IMU_CARRIER Solid {
            translation 0 0 0.05
            name "imu_carrier"
            children [
              Gyro { name "gyro_nested" }
              Accelerometer { name "acc_nested" }
            ]
            boundingObject Box { size 0.001 0.001 0.001 }
            physics Physics { density -1 mass 0.001 }
          }
"""),
    measure=("sensor:gyro_direct", "sensor:acc_direct",
             "sensor:gyro_nested", "sensor:acc_nested",
             "quat:SUBJECT", "pos:SUBJECT"),
    act=("motor_velocity:m:%s:0.0" % _g(TURNTABLE_RAD_S),),
    duration=2.0,
    assertion=None,
    doc="AGENTS.md ROS-2 sensor lane (Gyro constant [0,0,0] / Accelerometer "
        "silent on a URDF Husky) + README rule 11, which localises it: the "
        "importer's IMU carrier pattern (OmUrdfImporter.cpp:1103) owns no "
        "Newton body, so the devices have nothing to read. Expected to flip "
        "to `works` when the carrierBodyHandle fix lands (noted 2026-09-01).",
    documented_as=WORKS,  # flipped 2026-09-01: AGENTS.md re-measured (carrierBodyHandle fix, commit bde550489)
))


def _imu_nested_carrier_assertion(arrays):
    """A Gyro and an Accelerometer on a FOLDED carrier — a jointless nested
    Solid carrying `boundingObject Box 0.001^3` + `physics Physics { density
    -1 mass 0.001 }`, the URDF importer's exact IMU emission pattern — inside
    a turntable arm driven at 2.0 rad/s (gravity 0) must read what the same
    two devices mounted DIRECTLY on the arm read in the same run: a rigid
    fold co-rotates, so gyro ~2.0 rad/s (within 15% of the
    supervisor-measured rate) and accelerometer ~rate^2 x 0.25 m/s^2 of
    centripetal acceleration (within 0.3 m/s^2 or 30%). The direct pair is
    the in-run CONTROL: when it fails, the verdict is inconclusive, never a
    carrier finding. With the shipped engine the nested pair is expected
    BROKEN — the fold owns no Newton body, so the devices publish zeros or
    nothing — and after the pending carrierBodyHandle fix this row must go
    green (rule 9; authored 2026-09-01)."""
    prem = _turntable_premise(arrays, "nested-carrier IMU pair")
    if prem is not None:
        return prem
    rate, _, _ = _turntable_rotation(arrays, tail_frac=0.2)
    expected_c = rate * rate * TURNTABLE_RADIUS

    def tail_mean_norm(key):
        v = _finite_vecs(arrays.get(key))
        if not v:
            return None, 0
        tail = v[-max(1, len(v) // 5):]
        return sum(_norm(x) for x in tail) / len(tail), len(v)

    g_direct, g_direct_n = tail_mean_norm("sensor_gyro_direct")
    a_direct, a_direct_n = tail_mean_norm("sensor_acc_direct")
    g_nested, g_nested_n = tail_mean_norm("sensor_gyro_nested")
    a_nested, a_nested_n = tail_mean_norm("sensor_acc_nested")
    fmt = lambda x: "no finite sample" if x is None else "%.4f" % x
    ev = {"supervisor_measured_rad_s": rate,
          "expected_centripetal_m_s2": expected_c,
          "direct_gyro_rad_s": g_direct, "nested_gyro_rad_s": g_nested,
          "direct_acc_m_s2": a_direct, "nested_acc_m_s2": a_nested,
          "finite_samples": {"gyro_direct": g_direct_n,
                             "acc_direct": a_direct_n,
                             "gyro_nested": g_nested_n,
                             "acc_nested": a_nested_n}}
    # CONTROLS FIRST (rule 2). Each nested arm is judged only against a
    # direct-mount control that demonstrably works; a dead control is the
    # sibling device probe's finding, not this one's.
    gyro_ctrl_ok = g_direct is not None and abs(g_direct - rate) <= 0.15 * rate
    acc_ctrl_ok = (a_direct is not None
                   and abs(a_direct - expected_c) <= max(0.3,
                                                         0.30 * expected_c))
    ev["controls_ok"] = {"gyro": gyro_ctrl_ok, "accelerometer": acc_ctrl_ok}
    if not gyro_ctrl_ok and not acc_ctrl_ok:
        return Verdict(INCONCLUSIVE, ev,
                       "NEITHER direct-mount control device reads correctly "
                       "(gyro %s rad/s vs %.4f, accelerometer %s m/s^2 vs "
                       "%.4f), so the rig cannot attribute anything to the "
                       "nested carrier — see device.gyro / "
                       "device.accelerometer for the device-level verdicts"
                       % (fmt(g_direct), rate, fmt(a_direct), expected_c))
    arms = {}
    if gyro_ctrl_ok:
        arms["gyro"] = (g_nested is not None
                        and abs(g_nested - rate) <= 0.15 * rate)
    if acc_ctrl_ok:
        arms["accelerometer"] = (a_nested is not None
                                 and abs(a_nested - expected_c)
                                 <= max(0.3, 0.30 * expected_c))
    good = sorted(k for k, ok in arms.items() if ok)
    bad = sorted(k for k, ok in arms.items() if not ok)
    ev["arms_judged"] = sorted(arms)
    if not bad:
        note = None
        if len(arms) < 2:
            note = ("only the %s arm could be judged (its sibling's direct "
                    "control failed)" % good[0])
        return Verdict(WORKS, ev, note)
    if not good:
        return Verdict(BROKEN, ev,
                       "the folded carrier serves NEITHER device: nested gyro "
                       "reads %s rad/s against a working direct control at "
                       "%s, nested accelerometer %s m/s^2 against %s — the "
                       "jointless nested Solid owns no Newton body, so "
                       "computeValue has no bodyHandle to read, and every "
                       "URDF-imported IMU cluster is in exactly this state. "
                       "This row is EXPECTED broken on the shipped engine and "
                       "must go green when the carrierBodyHandle fix lands "
                       "(2026-09-01)."
                       % (fmt(g_nested), fmt(g_direct),
                          fmt(a_nested), fmt(a_direct)))
    return Verdict(DEGRADED, ev,
                   "the folded carrier serves %s but not %s (nested gyro %s "
                   "rad/s vs direct %s; nested accelerometer %s m/s^2 vs "
                   "direct %s)"
                   % (", ".join(good), ", ".join(bad),
                      fmt(g_nested), fmt(g_direct),
                      fmt(a_nested), fmt(a_direct)))


PROBES[-1].assertion = _imu_nested_carrier_assertion

_p(Probe(
    id="device.lidar",
    family=FAM_DEVICE,
    claim="Lidar — planar range image",
    world=lambda: floor() + body("WALL", "Box { size 0.2 8 4 }", "3 0 1.5",
                                 physics=False, name="wall"),
    prober_children="""    Lidar {
      name "lidar"
      horizontalResolution 32
      fieldOfView 1.5
      numberOfLayers 1
      minRange 0.05
      maxRange 10
    }
""",
    measure=("sensor:lidar",),
    duration=1.0,
    needs_rendering=True,
    assertion=None,
))


def _lidar_assertion(arrays):
    """A 32-beam lidar facing a wall 3 m away (near face 2.9 m) must return a
    range image whose minimum lands within 0.3 m of 2.9. An all-infinite or
    all-maxRange image means the scan is not intersecting the scene."""
    a = arrays.get("sensor_lidar")
    if a is None or len(a) == 0:
        return Verdict(INCONCLUSIVE, note="no Lidar image")
    img = [float(x) for x in a[-1] if x == x and abs(float(x)) != float("inf")]
    if not img:
        return Verdict(BROKEN, {"finite_returns": 0},
                       "every lidar beam returned inf/NaN against a wall 2.9 m "
                       "away")
    lo = min(img)
    ev = {"min_range_m": lo, "expected_m": 2.9, "finite_returns": len(img),
          "beams": len(a[-1])}
    if abs(lo - 2.9) <= 0.3:
        return Verdict(WORKS, ev)
    if lo >= 9.9:
        return Verdict(BROKEN, ev,
                       "every beam is pinned at maxRange with a wall at 2.9 m")
    return Verdict(DEGRADED, ev,
                   "closest lidar return %.3f m against a wall at 2.9 m" % lo)


PROBES[-1].assertion = _lidar_assertion

_p(Probe(
    id="device.camera",
    family=FAM_DEVICE,
    claim="Camera — renders a non-degenerate image in a headless run",
    # Rendering is out of scope for this suite, but a camera that returns no
    # image at all is a SIMULATION capability failure (recognition, agent
    # perception and the capture pipeline all sit on it), so the probe asserts
    # only that pixels exist and are not uniform.
    world=lambda: floor() + body("WALL", "Box { size 0.2 4 4 }", "2 0 1.5",
                                 physics=False, name="wall",
                                 color="0.9 0.15 0.1"),
    prober_children="""    Camera {
      name "cam"
      width 32
      height 32
      fieldOfView 1.0
    }
""",
    measure=("camera:cam",),
    duration=1.0,
    needs_rendering=True,
    assertion=None,
    doc="scope: image EXISTENCE only; rendering quality is deliberately out "
        "of OmniBench's scope",
))


def _camera_assertion(arrays):
    """A 32x32 Camera pointed at a red wall must return width*height pixels
    that are not all identical. This is an existence check, not a rendering
    benchmark: OmniBench does not grade image quality."""
    # camera:NAME is a ONE-SHOT stats dict, not a per-step series -- indexing
    # it like an array raised KeyError(-1) and scored a working camera
    # `inconclusive`.
    stats = arrays.get("camera_cam")
    if not isinstance(stats, dict):
        return Verdict(INCONCLUSIVE, note="no camera frame recorded")
    ev = dict(stats)
    if not ev.get("pixels"):
        return Verdict(BROKEN, ev, "the camera returned no image buffer")
    if ev.get("distinct_values", 0) <= 1:
        return Verdict(BROKEN, ev,
                       "the camera returned a uniform image (%s distinct "
                       "values) facing a red wall" % ev.get("distinct_values"))
    return Verdict(WORKS, ev)


PROBES[-1].assertion = _camera_assertion

def _connector_part(defname, translation, conn_name):
    """A free 0.5 kg part carrying a PASSIVE Connector on its top face,
    x-axis up (rotation 0 1 0 -1.5708 maps the connector's +x to world +z).
    Two are authored: HELD, whose connector sits 20 mm under the prober's
    active one, and CONTROL, an identical twin 0.8 m away — outside the
    active side's 0.5 m distanceTolerance, so nothing can ever lock it. Same
    node structure on both, per the lane's in-run-control rule: the twins
    must differ ONLY in whether the active connector can reach them."""
    return """DEF %s Solid {
  translation %s
  name "%s"
  children [
    DEF %s_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.3 0.8 0.5 roughness 1 metalness 0 }
      geometry Box { size 0.1 0.1 0.1 }
    }
    Connector {
      translation 0 0 0.08
      rotation 0 1 0 -1.5708
      name "%s"
      model "omnibench"
      type "passive"
      distanceTolerance 0.5
      axisTolerance 3.14
      rotationTolerance 3.14
      numberOfRotations 0
    }
  ]
  boundingObject USE %s_SHAPE
  physics Physics { density -1 mass 0.5 }
}
""" % (defname, translation, defname.lower(), defname, conn_name, defname)


_p(Probe(
    id="device.connector_weld",
    family=FAM_DEVICE,
    claim="Connector — runtime rigid attachment (weld) between bodies",
    # THE MATING PAIR THE OLD PROBE DELIBERATELY LACKED (its verdict was
    # capped at `degraded`: "weld HOLDING is not measured"). Authored per
    # docs/reference/connector.md + projects/samples/devices/worlds/
    # connector.omniworld: two connectors lock when their x-axes are parallel
    # within axisTolerance but POINTED IN OPPOSITE DIRECTIONS, distance
    # between origins within distanceTolerance, model strings identical, and
    # an "active" one locks to a "passive" one. numberOfRotations 0 skips the
    # rotational criterion.
    #
    # THE RIG IS A GRAVITY-HANG WITH AN IN-RUN CONTROL. The prober robot is a
    # STATIC anchor (physics NULL, no boundingObject) at z=1.5 carrying a
    # BODILESS active Connector pointing straight down (x-axis to world -z,
    # origin at z=1.3). OmConnector.cpp's own slot rule is what makes that an
    # anchor: "a bodiless active side welds the passive side's body to the
    # world" (ensureNewtonWeldSlot). 20 mm beneath it hangs HELD, a free
    # 0.5 kg part with the mating passive Connector; 0.8 m away hangs
    # CONTROL, its identical twin that nothing can reach. lock() fires at
    # t=0.1 s — the part has free-fallen ~5 cm by then, still well inside the
    # 0.5 m distanceTolerance, and the Newton weld engages AT THE CURRENT
    # POSE by design (`snap` is a documented no-op stub on this engine), so
    # the hold is asserted about post-lock stability, not an exact height.
    world=lambda: (floor()
                   + _connector_part("HELD", "0 0 1.2", "held_conn")
                   + _connector_part("CONTROL", "0.8 0 1.2", "ctrl_conn")),
    prober_children="""    Connector {
      translation 0 0 -0.2
      rotation 0 1 0 1.5708
      name "conn"
      model "omnibench"
      type "active"
      autoLock FALSE
      distanceTolerance 0.5
      axisTolerance 3.14
      rotationTolerance 3.14
      numberOfRotations 0
    }
""",
    measure=("device_exists:conn", "pos:HELD", "pos:CONTROL"),
    act=("connector_lock:conn:0.1",),
    log_capture=("could not be attached", "physically INERT"),
    duration=2.5,
    assertion=None,
    doc="AGENTS.md — welds (Connector / VacuumGripper) went native on Newton; "
        "docs/reference/connector.md (mating rules)",
    # The matrix's standing claim as of 2026-09-01 (holding unmeasured, capped
    # at degraded). The hold is now measured against an in-run control; a
    # passing run lands on `works` and the audit flags the flip — round-2
    # protocol.
    documented_as=WORKS,  # flipped 2026-09-01 round 3: weld HOLDS measured with a mating pair + falling control
))


def _connector_assertion(arrays):
    """An 'active' Connector on a STATIC anchor, locked at t=0.1 s to the
    'passive' Connector of a free 0.5 kg part hanging 20 mm beneath it, must
    weld that part to the world: from t=0.3 s on, the part's centre must stay
    above z=1.0 m (it starts at 1.2 and loses only the ~5 cm of pre-lock free
    fall) with at most 50 mm of drift, for the remaining 2.2 s. The identical
    unlocked twin 0.8 m away — outside the active side's reach — must FALL
    and rest on the floor near z=0.60 m; a 'hold' the control reproduces
    would be a scene failure, not a weld, and scores `inconclusive`. A held
    part that ends at the control's rest height fell exactly like its twin:
    the lock was accepted and constrains nothing — `broken`."""
    ok = arrays.get("device_exists_conn")
    ev = {"device_present": bool(ok)}
    if not ok:
        return Verdict(ABSENT, ev, "no Connector device was found on the robot")
    no_attach = _log_hits(arrays, "could not be attached")
    inert = _log_hits(arrays, "physically INERT")
    ev["attach_refusal_lines"] = no_attach
    ev["weld_gate_inert_lines"] = inert
    held = arrays.get("pos_HELD")
    ctrl = arrays.get("pos_CONTROL")
    t = arrays.get("t")
    if not held or not ctrl or not t or len(t) != len(held):
        return Verdict(INCONCLUSIVE, ev, "pose series missing or misaligned")
    ctrl_end = float(ctrl[-1][2])
    ev["control_final_z_m"] = ctrl_end
    if ctrl_end > 1.0:
        return Verdict(INCONCLUSIVE, ev,
                       "the CONTROL twin never fell (final z=%.3f m) — the "
                       "scene itself is not simulating, so the held part's "
                       "stability proves nothing" % ctrl_end)
    if inert:
        return Verdict(DEGRADED, ev,
                       "the engine reports Connector locks physically INERT "
                       "(the Newton weld gate is off in this environment): %s"
                       % inert[0][:160])
    if no_attach:
        return Verdict(INCONCLUSIVE, ev,
                       "the engine refused the attachment (%s) — a rig "
                       "authoring failure, not a weld measurement"
                       % no_attach[0][:160])
    post = [float(p[2]) for p, ti in zip(held, t) if float(ti) >= 0.3
            and p is not None]
    if len(post) < 2:
        return Verdict(INCONCLUSIVE, ev, "no post-lock samples recorded")
    lo, hi = min(post), max(post)
    ev.update({"held_min_z_after_lock_m": lo, "held_max_z_after_lock_m": hi,
               "held_drift_m": hi - lo})
    if lo <= 0.7:
        return Verdict(
            BROKEN, ev,
            "the held part fell to z=%.3f m — the same floor its unlocked "
            "twin rests on (%.3f m): lock() was accepted and the weld "
            "constrains nothing" % (lo, ctrl_end))
    if lo < 1.0 or (hi - lo) > 0.05:
        return Verdict(
            DEGRADED, ev,
            "the weld holds partially: the part sagged to z=%.3f m and "
            "drifted %.1f mm after the lock, against the <= 50 mm a rigid "
            "weld allows" % (lo, (hi - lo) * 1e3))
    return Verdict(WORKS, ev)


PROBES[-1].assertion = _connector_assertion

_p(Probe(
    id="device.emitter_receiver",
    family=FAM_DEVICE,
    claim="Emitter/Receiver — inter-robot messaging",
    # TWO robots, necessarily. The engine refuses same-robot delivery by
    # design ("robot cannot send message to self", OmReceiver::transmitPacket),
    # so the first version of this probe -- both devices on the prober --
    # could never have passed and scored the capability `broken` for obeying a
    # documented rule. Reading the dispatch code is what settled it, and it is
    # the clearest example in this lane of why a `broken` verdict has to be
    # chased to a mechanism before it is published.
    world=lambda: floor() + """DEF EMITTER_BOT Robot {
  translation 1 0 1.5
  name "emitter_bot"
  controller "omnibench_emitter"
  children [
    Emitter { name "emit" range -1 channel 1 }
  ]
  physics NULL
}
""",
    prober_children="""    Receiver { name "recv" channel 1 }
""",
    measure=("receiver:recv",),
    duration=1.0,
    assertion=None,
))


def _radio_assertion(arrays):
    """A packet emitted by a SECOND robot on channel 1 must arrive at this
    robot's Receiver on channel 1 within the run. Two robots because the
    engine refuses same-robot delivery by design. This proves the transport
    only — range attenuation, aperture and infra-red occlusion are separate
    behaviours this probe does not exercise."""
    a = arrays.get("radio_recv")
    if a is None:
        return Verdict(INCONCLUSIVE, note="radio probe produced no result")
    ev = dict(a) if isinstance(a, dict) else {"raw": a}
    if ev.get("received"):
        return Verdict(WORKS, ev)
    if not ev.get("receiver_present"):
        return Verdict(ABSENT, ev, "no Receiver device found on the prober")
    return Verdict(BROKEN, ev,
                   "a packet emitted by a second robot on channel 1 never "
                   "reached a Receiver on channel 1")


PROBES[-1].assertion = _radio_assertion


# ---------------------------------------------------------------------------
# Propeller — the foundation of an aircraft hardware-in-the-loop lane
# ---------------------------------------------------------------------------
#: Commanded rotor speed, rad/s. Well under the motor's maxVelocity, and
#: PROP_MAX_TORQUE is sized so the motor reaches it inside ONE basic timestep
#: -- see that constant, and do NOT reinstate the belief this comment used to
#: carry ("`acceleration` defaults to -1 = instant"), which is true of
#: OmMotor::runControl and FALSE of the kinematic path a Propeller uses.
PROP_OMEGA = 100.0
#: `maxTorque` on the propeller rotors, and it is a MEASUREMENT-CRITICAL value
#: rather than a plausible-looking number.
#:
#: A Propeller's rotor is not a physical joint: OmPropeller::prePhysicsStep
#: drives it through `OmMotor::runKinematicControl`, whose spin-up rate is
#:
#:     acc = mAcceleration->value();
#:     if (acc == -1.0 || acc > mMotorForceOrTorque) acc = mMotorForceOrTorque;
#:
#: -- i.e. the DEFAULT `acceleration -1` does not mean "instant" here, it means
#: "ramp at maxTorque rad/s^2". With the original `maxTorque 100` that is a
#: 1.0 s ramp to 100 rad/s, and thrust goes as omega^2, so the first second of
#: every propeller run carried almost no thrust.
#:
#: MEASURED on device.propeller_thrust (gravity 0 and torqueConstants 0 0, so
#: a(t) = t1*omega(t)^2/m exactly and omega(t) = sqrt(a/t1) is a direct read of
#: the rotor speed), machine 9722d23d12a3, binary dd7ae9921b6629ba: omega rose
#: LINEARLY at 99.97 rad/s^2 from the command instant t=0.1958 s and reached
#: 100 rad/s at t=1.196 s. That ramp is what made device.propeller_inflow
#: publish `degraded` on 2026-08-22 -- its early window (0.152-1.200 s) sat
#: almost entirely inside the spin-up, so decaying THRUST and rising ROTOR
#: SPEED were indistinguishable. 50000 rad/s^2 crosses 100 rad/s in 2 ms,
#: inside one 4 ms step, so omega is a known constant for the whole run.
#: It cannot leak into the thrust: OmPropeller clamps only mCurrentTorque to
#: maxForceOrTorque, and these probes declare torqueConstants 0 0.
PROP_MAX_TORQUE = 50000.0
#: Airframe mass, kg. Exactly 1 on purpose: it makes the analytic acceleration
#: numerically equal to the thrust, so a reader can check the claim in their
#: head instead of trusting a division.
PROP_MASS = 1.0
#: thrustConstants[0] for the static-thrust probe -> T = 0.001 * 100 * 100 = 10 N.
PROP_T1 = 0.001
#: thrustConstants[0] / [1] for the inflow probe. t1 gives 5 N of static thrust
#: against 9.81 N of weight, and t2 = 0.01 puts the terminal descent speed at
#: (m*g - t1*omega^2) / (t2*omega) = 4.81 m/s IF the inflow term is simulated.
PROP_INFLOW_T1 = 0.0005
PROP_INFLOW_T2 = 0.01


def _propeller_rig(motor_name, t1, t2):
    """Prober-body children for the Propeller probes: an airframe shape plus a
    rotor on the +z shaft.

    ⚠ THE AIRFRAME IS THE PROBER ROBOT ITSELF, and that is a measured choice,
    not a shortcut. A Propeller delivers its wrench to `upperSolid()`, so the
    carrier has to be a real Newton body -- and the obvious rig, a `DEF SUBJECT
    Solid` nested in the prober with its own `physics` and `boundingObject`, is
    NOT one. Measured on that exact scene: the engine reported `registered 2
    dynamic + 0 static Newton bodies` for a world holding the Robot, that
    nested Solid and one free top-level Solid -- the nested Solid is merged
    into its parent's body and never becomes a body of its own (README rule 11,
    the same asymmetry that leaves the URDF importer's IMU cluster unable to
    serve a Gyro). Mounting the rotor there and then reporting "no thrust"
    would have been a statement about lane 4's rig.
    So the prober overrides `physics` + `boundingObject` (the pattern the
    TouchSensor probes already use) and becomes a free-flying airframe -- which
    is also the topology the shipped `propeller.omniworld` helicopter uses --
    and the registration line is captured so the premise is CHECKED rather than
    argued.

    `torqueConstants 0 0` and `centerOfThrust 0 0 0` remove the two rotation
    confounds: no reaction spin, and no lever arm between the thrust point and
    the body origin. What is left is a pure linear acceleration with a
    closed-form answer.

    `maxTorque PROP_MAX_TORQUE` removes the THIRD confound, the rotor spin-up
    ramp, and it is the one that actually bit (rule 4 -- remove the confound
    rather than reasoning around it). See PROP_MAX_TORQUE for the mechanism and
    the measurement.
    """
    return """    DEF PROBER_BODY Shape {
      appearance PBRAppearance { baseColor 0.9 0.7 0.2 roughness 1 metalness 0 }
      geometry Box { size 0.3 0.3 0.1 }
    }
    DEF ROTOR Propeller {
      shaftAxis 0 0 1
      centerOfThrust 0 0 0
      thrustConstants %s %s
      torqueConstants 0 0
      device RotationalMotor {
        name "%s"
        maxVelocity 200
        maxTorque %s
      }
    }
""" % (_g(t1), _g(t2), motor_name, _g(PROP_MAX_TORQUE))


#: The engine's own complaint when a Propeller cannot find a body to push.
#: Load-bearing for the mechanism half of the verdict: the probe's airframe
#: DOES declare a Physics node, so this line firing is the engine contradicting
#: the world file, not advice.
PROP_NO_BODY_WARNING = "Adds a Physics node to Solid ancestors"

_p(Probe(
    id="device.propeller_thrust",
    family=FAM_DEVICE,
    claim="Propeller — a motorised rotor produces its analytic thrust",
    # Gravity 0 (rule 4): with weight in the scene, "the aircraft did not climb"
    # would be a statement about thrust-to-weight, and a thrust anywhere below
    # 9.81 N would be indistinguishable from no thrust at all.
    gravity=0.0,
    world=lambda: "",
    prober_translation="0 0 2",
    prober_children=_propeller_rig("rotor", PROP_T1, 0.0),
    prober_bounding="  boundingObject USE PROBER_BODY\n",
    prober_physics="  physics Physics { density -1 mass %s }\n" % _g(PROP_MASS),
    # A Robot's own boundingObject is not a Newton collider by default
    # (worldinfo.md: wheel/foot-only collision, so a chassis envelope cannot pin
    # the body). Nothing here collides, but the flag is what makes the airframe
    # a fully-shaped rigid body rather than a bare point mass.
    world_info="  newtonRobotColliders TRUE\n",
    measure=("pos:OMNIBENCH_PROBER", "quat:OMNIBENCH_PROBER"),
    act=("motor_velocity:rotor:%s:0.2" % _g(PROP_OMEGA),),
    duration=3.0,
    log_capture=("[OmNewtonBackend] registered", PROP_NO_BODY_WARNING),
    assertion=None,
    doc="docs/reference/propeller.md — T = t1*|omega|*omega applied at "
        "centerOfThrust; src/omnisim/nodes/OmPropeller.cpp:217-256 delivers it "
        "through OmSolid::applyExternalForceNewton",
    documented_as=WORKS,
))


def _propeller_thrust_assertion(arrays):
    """A Propeller with thrustConstants 0.001 0, spun at 100 rad/s by its
    RotationalMotor, produces T = 0.001 * |omega| * omega = 10 N along its
    shaft axis. Bolted to a 1 kg airframe in a gravity-free world, that is an
    acceleration of 10.0 m/s^2 along +z, +/- 5%.

    An airframe that does not move at all is a propeller whose wrench never
    reaches the solver. The premise -- that the airframe IS a Newton body -- is
    read from the engine's own registration line rather than assumed, so a rig
    that never had a body is reported as an instrument failure instead of as an
    engine defect."""
    expected = PROP_T1 * PROP_OMEGA * PROP_OMEGA / PROP_MASS
    a, window = _accel_along(arrays, "pos_OMNIBENCH_PROBER", 2)
    bodies = _newton_dynamic_bodies(arrays)
    no_body_warning = _log_hits(arrays, PROP_NO_BODY_WARNING)
    travel = _travel(arrays, "pos_OMNIBENCH_PROBER", 2)
    spin, _n = _quat_rotation_total(arrays.get("quat_OMNIBENCH_PROBER"))
    if a is None:
        return Verdict(INCONCLUSIVE,
                       {"newton_dynamic_bodies": bodies},
                       "no usable trajectory was recorded for the airframe")
    ev = {
        "measured_accel_m_s2": a,
        "analytic_accel_m_s2": expected,
        "accel_ratio": a / expected if expected else None,
        "axial_travel_m": travel,
        "newton_dynamic_bodies": bodies,
        "thrust_N": PROP_T1 * PROP_OMEGA * PROP_OMEGA,
        "airframe_mass_kg": PROP_MASS,
        "commanded_omega_rad_s": PROP_OMEGA,
        "body_rotation_rad": spin,
        "window": window,
        "engine_no_body_warning": no_body_warning,
    }
    # PREMISE FIRST (rule 2). "It never moved" is a finding only if there was
    # something there to move.
    if bodies is not None and bodies < 1:
        return Verdict(INCONCLUSIVE, ev,
                       "the engine registered %d dynamic Newton bodies, so the "
                       "airframe is not a rigid body at all and nothing about "
                       "the Propeller follows" % bodies)
    if abs(a) >= 0.95 * expected and abs(a) <= 1.05 * expected:
        return Verdict(WORKS, ev)
    if abs(a) <= 0.01 * expected:
        note = ("the rotor turned at %.0f rad/s for %.1f s and the airframe "
                "moved %.3g m: the thrust wrench never reached the solver"
                % (PROP_OMEGA, (window or {}).get("t2_s", 0.0), travel or 0.0))
        if no_body_warning:
            note += (". The engine warns %r on a Solid that DOES declare a "
                     "Physics node -- OmPropeller gates thrust on the legacy "
                     "ODE body handle (OmSolidMerger::mBody), which no longer "
                     "exists, so the gate can never open"
                     % no_body_warning[0].split(": ")[-1])
        return Verdict(BROKEN, ev, note)
    return Verdict(DEGRADED, ev,
                   "the airframe accelerated at %.4f m/s^2 against an analytic "
                   "%.4f (ratio %.4f)" % (a, expected, a / expected))


PROBES[-1].assertion = _propeller_thrust_assertion

_p(Probe(
    id="device.propeller_inflow",
    family=FAM_DEVICE,
    claim="Propeller thrust decays with axial airspeed (thrustConstants[1], "
          "the speed-of-advance term)",
    # GRAVITY IS THE INSTRUMENT HERE, NOT A CONFOUND -- and this is the one
    # place in the file that deliberately keeps it. A .wbt cannot author an
    # initial velocity, and the supervisor's velocity write is a separate
    # capability whose failure would be unattributable inside this probe. So
    # the airspeed is produced by letting the airframe FALL along its own shaft
    # axis: V grows from 0 through the run, and the two measurement windows are
    # low-airspeed and high-airspeed slices of ONE run. Gravity itself is
    # measured by phenomenon.gravity_is_honoured, so it is a checked input.
    gravity=9.81,
    world=lambda: "",
    # High enough that 3 s of free fall (44 m at g, 22 m with thrust) cannot
    # reach z=0, so no implicit ground plane can end the run early.
    prober_translation="0 0 100",
    prober_children=_propeller_rig("rotor", PROP_INFLOW_T1, PROP_INFLOW_T2),
    prober_bounding="  boundingObject USE PROBER_BODY\n",
    prober_physics="  physics Physics { density -1 mass %s }\n" % _g(PROP_MASS),
    world_info="  newtonRobotColliders TRUE\n",
    measure=("pos:OMNIBENCH_PROBER",),
    # COMMANDED AT t=0, not partway in, and with PROP_MAX_TORQUE behind it: the
    # rotor is therefore at its full 100 rad/s before the airframe has moved,
    # so the early window opens at a MEASURED airspeed of ~0 and "early = low
    # airspeed" is a fact about the recording rather than an assumption. The
    # old rig commanded at t=0.1 against a 1.0 s spin-up ramp, which is the
    # whole reason this probe once read `degraded`.
    act=("motor_velocity:rotor:%s:0.0" % _g(PROP_OMEGA),),
    duration=3.0,
    log_capture=("[OmNewtonBackend] registered", PROP_NO_BODY_WARNING),
    assertion=None,
    doc="docs/reference/propeller.md — 'THE INFLOW (SPEED-OF-ADVANCE) TERM IS "
        "NOT SIMULATED, so thrustConstants[1] and torqueConstants[1] have NO "
        "EFFECT'; OmPropeller.cpp:229-234 pins V = 0.0 with the ODE point-"
        "velocity read gone",
    documented_as=WORKS,  # flipped 2026-09-01: AGENTS.md re-measured (inflow V now read from body point velocity, commit bde550489)
))


def _propeller_inflow_assertion(arrays):
    """A rotor holding 5 N of static thrust under a 1 kg airframe's 9.81 N of
    weight, with thrustConstants 0.0005 0.01, must reach a TERMINAL descent
    speed of (m*g - t1*omega^2) / (t2*omega) = 4.81 m/s: as the airframe falls,
    axial airspeed builds, the inflow term adds thrust, and the descent
    acceleration decays toward zero with a 1 s time constant.

    The rotor is at its commanded 100 rad/s before the airframe has moved (see
    PROP_MAX_TORQUE), so the two windows are an AIRSPEED sweep and nothing
    else, and the recorded descent speed at each of them is published as
    evidence rather than assumed. Two independent readings decide the verdict:

    * the ratio of the late-window acceleration to the early-window one -- a
      simulated inflow term drives it toward 0, an omega^2-only thrust model
      holds it at 1.0; and
    * the late-window acceleration against the closed-form CONSTANT-thrust
      value -(g - t1*omega^2/m) = -4.810 m/s^2. Landing on that number while
      the airspeed has tripled is the positive signature of a thrust that does
      not know about airspeed, not merely the absence of decay.

    ⚠ Two other outcomes exist and the evidence separates them.
    (a) If BOTH windows read the full 9.81 m/s^2, no thrust is being delivered
        at all, so the inflow term is dead for a reason that has nothing to do
        with thrustConstants[1]. Still a `broken` inflow model, but the finding
        belongs to device.propeller_thrust and the note says so.
    (b) If the EARLY window reads ~g and the late one does not, the rotor was
        still spinning up while the early window was open -- an instrument
        failure, reported `inconclusive`. That is not hypothetical: it is what
        this probe published as `degraded` on 2026-08-22 (early -7.906, late
        -4.810, ratio 0.608) once the engine's external-wrench gate was fixed
        and thrust started arriving. The ramp was measured at 99.97 rad/s^2
        over a full second; nothing about thrustConstants[1] was involved."""
    g = 9.81
    static_thrust = PROP_INFLOW_T1 * PROP_OMEGA * PROP_OMEGA
    powered = g - static_thrust / PROP_MASS
    terminal = (PROP_MASS * g - static_thrust) / (PROP_INFLOW_T2 * PROP_OMEGA)
    # The early window opens at t=0 because the rotor is already at speed
    # there; the late one is the last 30% of the run, by which point the
    # airframe is falling several times faster than the terminal speed a
    # simulated inflow term would have imposed.
    early, w_early = _accel_along(arrays, "pos_OMNIBENCH_PROBER", 2,
                                  lo=0.0, hi=0.15)
    late, w_late = _accel_along(arrays, "pos_OMNIBENCH_PROBER", 2,
                                lo=0.70, hi=1.0)
    bodies = _newton_dynamic_bodies(arrays)
    if early is None or late is None:
        return Verdict(INCONCLUSIVE, {"newton_dynamic_bodies": bodies},
                       "no usable trajectory was recorded for the airframe")
    v_early = _speed_along(arrays, "pos_OMNIBENCH_PROBER", 2, 0.075)
    v_late = _speed_along(arrays, "pos_OMNIBENCH_PROBER", 2, 0.85)
    ratio = abs(late) / abs(early) if abs(early) > 1e-9 else None
    ev = {
        "early_accel_m_s2": early,
        "late_accel_m_s2": late,
        "late_over_early": ratio,
        "early_airspeed_m_s": v_early,
        "late_airspeed_m_s": v_late,
        "accel_if_thrust_constant_m_s2": -powered,
        "late_over_constant_thrust": (abs(late) / powered) if powered else None,
        "accel_if_no_thrust_m_s2": -g,
        "predicted_terminal_speed_m_s": terminal,
        "newton_dynamic_bodies": bodies,
        "static_thrust_N": static_thrust,
        "t2_inflow_constant": PROP_INFLOW_T2,
        "early_window": w_early,
        "late_window": w_late,
    }
    if bodies is not None and bodies < 1:
        return Verdict(INCONCLUSIVE, ev,
                       "the engine registered %d dynamic Newton bodies, so "
                       "there is no airframe to measure an airspeed on"
                       % bodies)
    # PREMISE (rule 2): was the rotor at speed while the early window was open?
    # A spin-up ramp and a decaying thrust are the SAME trajectory, so this has
    # to be excluded before either can be claimed. The discriminator is the
    # late window: a ramp ends, so it reads the powered value; a propeller
    # delivering nothing reads g at BOTH ends.
    if abs(early) >= 0.95 * g:
        if abs(late) >= 0.90 * g:
            return Verdict(
                BROKEN, ev,
                "OVER-DETERMINED, and the attribution matters: the airframe "
                "fell at %.3f m/s^2 early and %.3f late against %.3f for "
                "gravity alone, so NO thrust is being delivered and the inflow "
                "term cannot be isolated. Thrust measurably does not respond "
                "to airspeed, but the mechanism is the one "
                "device.propeller_thrust reports, not thrustConstants[1]."
                % (abs(early), abs(late), g))
        return Verdict(
            INCONCLUSIVE, ev,
            "INSTRUMENT FAILURE, not a device finding: the early window read "
            "%.3f m/s^2 (gravity alone is %.3f) while the late one read %.3f, "
            "so the rotor was still SPINNING UP while the low-airspeed window "
            "was open and a rising omega is indistinguishable from a decaying "
            "thrust. Check maxTorque against PROP_MAX_TORQUE -- "
            "OmMotor::runKinematicControl ramps at maxTorque rad/s^2, and "
            "`acceleration -1` does not mean instant on that path."
            % (abs(early), g, abs(late)))
    if ratio is None:
        return Verdict(INCONCLUSIVE, ev, "the early window measured no motion")
    if ratio <= 0.5:
        return Verdict(WORKS, ev)
    def _sp(v):
        return "unmeasured" if v is None else "%.2f" % abs(v)

    sweep = ("the airframe went from %s to %s m/s of axial airspeed"
             % (_sp(v_early), _sp(v_late)))
    if 0.9 <= ratio <= 1.1:
        held = abs(abs(late) - powered) <= 0.02 * powered
        return Verdict(
            BROKEN, ev,
            "%s and the descent acceleration did not move (%.4f -> %.4f "
            "m/s^2, ratio %.3f)%s: thrust is constant at its static value and "
            "thrustConstants[1] has no effect. OmPropeller.cpp pins the "
            "speed-of-advance V to 0.0, so the inflow term is multiplied by "
            "zero on every tick."
            % (sweep, abs(early), abs(late), ratio,
               ", landing on the closed-form constant-thrust value %.3f m/s^2 "
               "(ratio %.5f)" % (powered, abs(late) / powered) if held
               else " (which is NOT the closed-form constant-thrust value "
                    "%.3f m/s^2, so check device.propeller_thrust too)"
                    % powered))
    return Verdict(DEGRADED, ev,
                   "%s and the descent acceleration fell to %.0f%% of its "
                   "early value -- some airspeed dependence, but not the "
                   "terminal-velocity behaviour the declared t2 predicts"
                   % (sweep, 100.0 * ratio))


PROBES[-1].assertion = _propeller_inflow_assertion


# ===========================================================================
# FAMILY: phenomenon — behaviours that are not one node
# ===========================================================================
_p(Probe(
    id="phenomenon.gravity_is_honoured",
    family=FAM_PHENOMENON,
    claim="WorldInfo.gravity reaches the solver (non-Earth values included)",
    # THE defect this lane would have caught: gravity was never plumbed, so
    # every Newton world ran at -9.81 regardless of what the file said.
    gravity=3.72,   # Mars
    world=lambda: body("SUBJECT", "Sphere { radius 0.1 subdivision 3 }",
                       "0 0 20.0"),
    measure=("pos:SUBJECT",),
    duration=2.0,
    assertion=None,
    doc="docs/benchmarks/ — gravity never plumbed, fixed in e7b9fb11",
    documented_as=WORKS,
))


def _gravity_assertion(arrays):
    """A sphere in free fall in a world declaring gravity 3.72 m/s^2 (Mars)
    must fall 0.5*3.72*t^2 = 7.44 m in 2 s, +/- 5%. Falling 19.6 m instead
    means the world's gravity never reached the solver and Earth gravity was
    substituted — a defect that is invisible to every load-only check."""
    p = arrays.get("pos_SUBJECT")
    t = arrays.get("t")
    if p is None or t is None or len(p) < 2:
        return Verdict(INCONCLUSIVE, note="no trajectory recorded")
    dt = float(t[-1]) - float(t[0])
    drop = float(p[0][2]) - float(p[-1][2])
    g_meas = 2.0 * drop / (dt * dt) if dt > 0 else None
    ev = {"declared_g_m_s2": 3.72, "measured_g_m_s2": g_meas,
          "drop_m": drop, "window_s": dt}
    if g_meas is None:
        return Verdict(INCONCLUSIVE, ev, "zero-length window")
    if abs(g_meas - 3.72) <= 0.05 * 3.72:
        return Verdict(WORKS, ev)
    if abs(g_meas - 9.81) <= 0.05 * 9.81:
        return Verdict(BROKEN, ev,
                       "the world declares g=3.72 m/s^2 and the body fell at "
                       "%.3f m/s^2 — Earth gravity was substituted" % g_meas)
    return Verdict(DEGRADED, ev,
                   "declared g=3.72, measured %.3f m/s^2" % g_meas)


PROBES[-1].assertion = _gravity_assertion

_p(Probe(
    id="phenomenon.coordinate_system_nue",
    family=FAM_PHENOMENON,
    claim="WorldInfo.coordinateSystem NUE — the up-axis reaches the solver",
    # 210 NUE worlds had gravity projected to zero and never fell (c77cbe98).
    coordinate_system="NUE",
    world=lambda: body("SUBJECT", "Sphere { radius 0.1 subdivision 3 }",
                       "0 20 0"),
    prober_translation="0 3 0",
    measure=("pos:SUBJECT",),
    duration=2.0,
    assertion=None,
    doc="AGENTS.md — coordinateSystem reaching the solver, c77cbe98",
    documented_as=WORKS,
))


def _nue_assertion(arrays):
    """In an NUE world (up = +y) a sphere released at y=20 must fall along
    -y at 9.81 m/s^2, dropping 19.62 m in 2 s +/- 5%. Not falling at all is
    the measured pre-c77cbe98 defect, where gravity was projected onto the
    wrong axis and came out zero."""
    p = arrays.get("pos_SUBJECT")
    t = arrays.get("t")
    if p is None or t is None or len(p) < 2:
        return Verdict(INCONCLUSIVE, note="no trajectory recorded")
    dt = float(t[-1]) - float(t[0])
    drop_y = float(p[0][1]) - float(p[-1][1])
    g_meas = 2.0 * drop_y / (dt * dt) if dt > 0 else None
    ev = {"up_axis": "+y (NUE)", "drop_along_y_m": drop_y,
          "measured_g_m_s2": g_meas, "window_s": dt,
          "drift_x_m": float(p[-1][0]) - float(p[0][0]),
          "drift_z_m": float(p[-1][2]) - float(p[0][2])}
    if g_meas is None:
        return Verdict(INCONCLUSIVE, ev, "zero-length window")
    if abs(g_meas - 9.81) <= 0.05 * 9.81:
        return Verdict(WORKS, ev)
    if abs(drop_y) < 0.1:
        return Verdict(BROKEN, ev,
                       "an NUE world's body did not fall at all (%.4f m in "
                       "%.2f s) — gravity is projected onto the wrong axis"
                       % (drop_y, dt))
    return Verdict(DEGRADED, ev,
                   "NUE fall rate %.3f m/s^2 instead of 9.81" % g_meas)


PROBES[-1].assertion = _nue_assertion

_p(Probe(
    id="phenomenon.runtime_node_deletion",
    family=FAM_PHENOMENON,
    claim="A node deleted at runtime stops colliding "
          "(via simulationRebuildPhysics, 2026-09-01)",
    # AGENTS.md documents the DEFAULT behaviour as measured and unfixed: the
    # frozen MuJoCo model keeps a deleted floor's geometry, so it silently
    # holds a body up. As of 2026-09-01 the capability EXISTS through a
    # documented verb — wb_supervisor_simulation_rebuild_physics /
    # supervisor.simulationRebuildPhysics — which purges the model. The probe
    # is the honest TWO-ARM shape: the default arm (t=1.5–3.0 s, after
    # remove() and before the rebuild) documents the frozen model, and the
    # rebuild arm (t=3.0–5.0 s) proves the fix; the verdict is `works` only
    # when the rebuild arm passes. The rebuild call is getattr-guarded in the
    # prober — a libController that predates the verb records the premise and
    # the row lands on `inconclusive` naming the stale libController.
    world=lambda: floor() + body("SUBJECT", "Box { size 0.2 0.2 0.2 }",
                                 "0 0 1.0"),
    measure=("pos:SUBJECT",),
    act=("delete_node:FLOOR:1.5", "rebuild_physics:3.0"),
    duration=5.0,
    assertion=None,
    doc="AGENTS.md — 'a deleted wall still stops a robot and a deleted floor "
        "still holds a body up, silently' (the default arm); "
        "wb_supervisor_simulation_rebuild_physics (2026-09-01, the fix arm)",
    # The matrix's standing claim as of 2026-09-01 (the pre-rebuild-verb
    # measurement). When the rebuild arm passes, this row flips to `works`
    # and the audit flags it — the correct protocol: the parent reconciles
    # the docs AFTER the measurement, never before.
    documented_as=WORKS,  # flipped 2026-09-01 round 3: works via the documented rebuild verb; the default arm still documents the frozen-model phantom
))


def _deletion_assertion(arrays):
    """Two arms in one run. DEFAULT ARM: a 0.2 m box settled at z=0.65 m on a
    floor removed by supervisor remove() at t=1.5 s is expected (per the
    shipped engine's frozen MuJoCo model) to STAY at 0.65 through t=3.0 s —
    recorded as evidence, and if it instead falls, the defect is natively
    fixed and the row is `works` on the default path. REBUILD ARM:
    supervisor.simulationRebuildPhysics() at t=3.0 s must purge the deleted
    geometry, so the box must actually FALL, reaching z < 0.4 m by t=5 s
    (2 s of free fall is ~20 m; there is nothing left below to land on). A
    box still at 0.65 after the rebuild means the verb does not purge the
    model either — `broken`. A missing rebuild binding is a stale
    libController: `inconclusive`, with the default arm's phantom still
    recorded in evidence."""
    p = arrays.get("pos_SUBJECT")
    t = arrays.get("t")
    if not p or not t or len(p) != len(t) or len(p) < 4:
        return Verdict(INCONCLUSIVE, note="no trajectory recorded")

    def z_at_last_before(cut):
        best = None
        for zi, ti in zip(p, t):
            if zi is None:
                continue
            if float(ti) < cut:
                best = float(zi[2])
        return best

    z_settle = z_at_last_before(1.5)     # just before the delete
    z_phantom = z_at_last_before(3.0)    # just before the rebuild
    z_end = float(p[-1][2])
    rb = arrays.get("acted_rebuild_physics")
    ev = {"z_before_delete_m": z_settle, "z_before_rebuild_m": z_phantom,
          "z_at_end_m": z_end, "delete_at_s": 1.5, "rebuild_at_s": 3.0,
          "rebuild_premise": rb}
    if z_settle is None or z_phantom is None:
        return Verdict(INCONCLUSIVE, ev, "trajectory has holes at the arm "
                                         "boundaries")
    if abs(z_settle - 0.65) > 0.02:
        return Verdict(INCONCLUSIVE, ev,
                       "the box never settled at the analytic 0.65 m before "
                       "the delete (z=%.4f) — rig failure, the arms have no "
                       "baseline" % z_settle)
    if z_phantom < 0.4:
        # The default arm alone released the body: the frozen-model defect is
        # natively fixed and the rebuild verb was not even needed.
        return Verdict(WORKS, ev,
                       "deletion took effect WITHOUT the rebuild verb — the "
                       "box was already at z=%.3f m before t=3.0 s, so the "
                       "frozen-model defect is natively fixed" % z_phantom)
    phantom_held = abs(z_phantom - 0.65) <= 0.02
    if not isinstance(rb, dict) or not rb.get("binding_present"):
        return Verdict(INCONCLUSIVE, ev,
                       "supervisor.simulationRebuildPhysics is missing from "
                       "this libController (stale libController — rebuild "
                       "the controller libs / run `omnisim doctor`). The "
                       "default arm did measure the frozen-model phantom "
                       "(z=%.4f m held for 1.5 s after the delete), but the "
                       "rebuild arm could not run." % z_phantom)
    if not rb.get("called") or rb.get("error"):
        return Verdict(INCONCLUSIVE, ev,
                       "the rebuild call itself failed (%s) — instrument, "
                       "not a capability verdict" % (rb.get("error"),))
    if z_end < 0.4:
        if phantom_held:
            return Verdict(WORKS, ev,
                           "via the documented workflow: the default arm "
                           "held the phantom at z=%.4f m for the full 1.5 s "
                           "after the delete (the frozen model, as "
                           "documented), and simulationRebuildPhysics() at "
                           "t=3.0 s released it to z=%.3f m" %
                           (z_phantom, z_end))
        return Verdict(DEGRADED, ev,
                       "the rebuild released the box (z=%.3f m at end) but "
                       "the default arm drifted to z=%.4f m instead of "
                       "holding the documented 0.65 m phantom — the default "
                       "behaviour changed and deserves its own attribution"
                       % (z_end, z_phantom))
    return Verdict(BROKEN, ev,
                   "the box stayed at z=%.4f m for 2 s after "
                   "simulationRebuildPhysics() — the rebuild verb does not "
                   "purge the deleted node's geometry from the solver either"
                   % z_end)


PROBES[-1].assertion = _deletion_assertion

_p(Probe(
    id="phenomenon.restitution_declared",
    family=FAM_PHENOMENON,
    claim="Restitution (bounciness) is declarable per-material in the .wbt",
    # lane1R's finding: there is no restitution field on the Newton path; e
    # can only be identified through the contact spring (ke, kd).
    world=lambda: floor() + body("SUBJECT", "Sphere { radius 0.1 subdivision 3 }",
                                 "0 0 1.55"),
    world_info='  contactProperties [\n    ContactProperties { bounce 0.8 }\n  ]\n',
    measure=("pos:SUBJECT",),
    duration=4.0,
    assertion=None,
    doc="tests/benchmarks/omnibench/lane1r/README.md — 'No restitution field "
        "exists on the Newton path'",
    documented_as=BROKEN,
))


def _restitution_assertion(arrays):
    """A sphere dropped 1.0 m onto a floor in a world declaring
    ContactProperties.bounce 0.8 must rebound to at least 0.5 m above the
    floor (e=0.8 predicts 0.64 m). Declaring bounce and getting a dead drop
    means the field is parsed and unread — the world cannot state its own
    restitution, which is exactly what lane 1 has to work around with
    newtonContactKd."""
    p = arrays.get("pos_SUBJECT")
    if p is None or len(p) < 3:
        return Verdict(INCONCLUSIVE, note="no trajectory recorded")
    zs = [float(v[2]) for v in p]
    floor_top = 0.55 + 0.1          # rest height of the sphere centre
    # first local maximum after the initial descent
    lowest_i = min(range(len(zs)), key=lambda i: zs[i] if i < len(zs) else 1e9)
    peak = max(zs[lowest_i:]) if lowest_i < len(zs) - 1 else zs[-1]
    rebound = peak - floor_top
    ev = {"declared_bounce": 0.8, "rebound_height_m": rebound,
          "predicted_rebound_m": 0.64, "drop_height_m": 1.0}
    if rebound >= 0.5:
        return Verdict(WORKS, ev)
    if rebound < 0.02:
        return Verdict(BROKEN, ev,
                       "the world declares bounce 0.8 and the sphere rebounded "
                       "%.1f mm — the restitution declaration does not reach "
                       "the solver" % (rebound * 1e3))
    return Verdict(DEGRADED, ev,
                   "declared bounce 0.8 predicts a 0.64 m rebound; measured "
                   "%.3f m" % rebound)


PROBES[-1].assertion = _restitution_assertion


#: The 55 deg incline shared by the two friction probes, and the ONE piece of
#: geometry both of their verdicts rest on. Kept as a helper rather than
#: duplicated so the positive and negative arms cannot drift apart: they must
#: differ ONLY in the declared `newtonGroundMu`, or the pair stops bracketing
#: anything.
#:
#: THETA = 55 deg, so tan(theta) = 1.428148 -- deliberately above atan(1.0),
#: because a slope a mu=1.0 default would also hold cannot show that the
#: DECLARED value reached the solver.
#:
#: The subject is a 0.6 x 0.3 x 0.06 slab, NOT a cube. Its footprint along the
#: slope is b = 0.6 against a height h = 0.06, so b/h = 10 and it topples only
#: past atan(10) = 84.3 deg. That is what isolates friction: on this body the
#: slide is the only degree of freedom the slope can excite, so a measured
#: displacement is a Coulomb result and nothing else. (Its 0.2 m cube
#: predecessor had b/h = 1.0 and toppled at 45 deg -- see the note on the
#: probe below.)
#:
#: Placement is exact, not eyeballed: rotating +z about +y by theta gives the
#: surface normal n = (sin theta, 0, cos theta) = (0.8191520, 0, 0.5735764);
#: the ramp centre is (0, 0, 0.5) with half-thickness 0.1, so the contact face
#: sits at 0.1*n and the slab centre one half-height further along n:
#:   (0.1 + 0.03) * n = (0.1064898, 0, 0.0745649)  ->  (0.1064898, 0, 0.5745649)
#: A body placed even a few cm off the face FALLS onto it, and the skid that
#: follows an impact is not a friction measurement -- an early version of this
#: probe floated the subject 0.19 m up and published the resulting 0.919 m
#: skid as a defect.
_INCLINE_RAD = 0.9599310886          # 55 deg
_INCLINE_TAN = 1.428148              # tan(55 deg), the Coulomb bound


def _incline_world():
    return """DEF RAMP Solid {
  translation 0 0 0.5
  rotation 0 1 0 0.9599310886
  name "ramp"
  children [
    DEF RAMP_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.45 0.47 0.5 roughness 1 metalness 0 }
      geometry Box { size 6 3 0.2 }
    }
  ]
  boundingObject USE RAMP_SHAPE
}
""" + body("SUBJECT", "Box { size 0.6 0.3 0.06 }",
           "0.1064898 0 0.5745649",
           extra="  rotation 0 1 0 0.9599310886\n")


_p(Probe(
    id="phenomenon.friction_declared_in_world",
    family=FAM_PHENOMENON,
    claim="A world can state its own friction and have it reach the solver",
    # The translation audit's systemic finding, as a DYNAMIC probe: declare a
    # high friction in the field the engine reads and check the slab sticks on
    # an incline that a mu=1.0 default would also hold... so use an incline
    # ABOVE atan(1.0) where only the declared mu=2.0 can stick.
    #
    # ⚠ THIS PROBE PUBLISHED `broken` FROM 2026-08-13 TO 2026-08-17 AND THE
    # PROBE WAS WRONG, NOT THE ENGINE. It dropped a 0.2 m CUBE on the 55 deg
    # ramp, and a block is in equilibrium on an incline only if BOTH
    #     mu >= tan(theta)      (it does not slide)
    # AND tan(theta) < b/h      (it does not topple),
    # where b is the footprint along the slope and h the height. A cube has
    # b/h = 1.0 and tan(55 deg) = 1.428, so the scene was statically
    # IMPOSSIBLE before friction was ever consulted -- the cube tipped onto an
    # edge and tumbled, and the 24.355 m it travelled was published as a
    # contact-solver defect. The 30 deg control below never caught it because
    # 30 deg is under the 45 deg topple bound of a cube.
    #
    # THE SHARPEST REFUTATION, and the one to quote: re-run in bare MuJoCo
    # (3.11.0, elliptic cone, impratio 10 -- the same contact settings this
    # world declares, on a solver that is not our integration at all), the old
    # cube travels
    #     mu=2.0 -> 23.30 m (tilt 176 deg) | mu=10 -> 23.46 m | mu=100 -> 22.29 m
    # It is still "sliding" 22 m at mu=100. A probe whose verdict does not
    # move when the variable under test is raised fiftyfold was not measuring
    # that variable, and the in-engine 24.355 m is reproduced by toppling
    # alone.
    #
    # The fix is geometry, not tuning: a LOW-CoM SLAB with b/h = 10 (topple
    # angle atan(10) = 84.3 deg, far above the 55 deg slope) so the only thing
    # that can move this body is friction. Same bare-MuJoCo harness, same ramp:
    #     mu=2.0    -> 0.0006 m (tilt 0.002 deg)  HOLDS
    #     mu=1.4281 -> 0.0017 m (tilt 0.002 deg)  HOLDS  <- = tan(55 deg)
    #     mu=1.3    -> 1.4343 m in 2 s            SLIDES <- analytic 1.442 m
    # i.e. the Coulomb bound is reproduced to four decimal places and the
    # slide rate to 0.5%. Declared friction reaching the contact was never in
    # doubt; the scene was.
    #
    # It is also the lane's own rule that a green which cannot go red is not
    # evidence: `phenomenon.friction_slides_below_coulomb_bound` below is the
    # NEGATIVE ARM -- the identical slab at a declared mu BELOW tan(theta),
    # which must slide. The pair brackets the analytic bound from both sides.
    world_info='  newtonGroundMu 2.0\n  newtonCone "elliptic"\n  newtonImpratio 10\n',
    world=_incline_world,
    measure=("pos:SUBJECT",),
    duration=3.0,
    assertion=None,
    doc="tests/benchmarks/omnibench/lane1/translation_audit.py",
    # documented_as is deliberately UNSET. The docs claim that declaring
    # newtonGroundMu makes the value reach the solver, and the translation
    # audit confirms that it does (3/3 geoms at mu 2) -- so flagging this as a
    # doc mismatch would attribute the failure to the wrong claim. What this
    # probe measures is the physical consequence, which no doc asserts.
))


def _friction_assertion(arrays):
    """A low-CoM slab resting on a 55 deg incline in a world declaring
    newtonGroundMu 2.0 must stay put: Coulomb requires only mu >= tan(55 deg)
    = 1.4281, so mu=2.0 carries 40% margin and the slab should slide less than
    5 cm in 3 s.

    THE ASSERTION IS AGAINST THE ANALYTIC BOUND, NOT AGAINST A RECORDED
    GOLDEN. Down-slope acceleration under Coulomb is
        a = g (sin theta - mu cos theta)
    which at mu = 2.0 is -3.218 m/s^2 -- negative, i.e. static friction is not
    even saturated and no slide is admissible. That is why 5 cm over 3 s is a
    tolerance on solver noise rather than a tuned threshold: the physical
    prediction is exactly zero.

    ⚠ THE BODY MUST NOT BE ABLE TO TOPPLE, OR THIS MEASURES THE WRONG THING.
    A block on an incline is in equilibrium only if mu >= tan(theta) AND
    tan(theta) < b/h. The probe's original 0.2 m cube had b/h = 1.0 against
    tan(55 deg) = 1.428, so it tipped and tumbled regardless of friction, and
    the 24.355 m it travelled published as `broken` for four days. The slab is
    b/h = 10 (topple angle 84.3 deg), which removes that degree of freedom.

    ATTRIBUTION, measured rather than assumed. lane1/translation_audit.py on
    this exact world reports "3/3 geoms carry the declared mu 2", so the
    declaration DOES reach the solver. Re-run the audit before repeating any
    conclusion from this probe:

        python tests/benchmarks/omnibench/lane1/translation_audit.py --world \\
          tests/benchmarks/omnibench/lane4/worlds/\\
phenomenon_friction_declared_in_world.wbt
    """
    p = arrays.get("pos_SUBJECT")
    if p is None or len(p) < 2:
        return Verdict(INCONCLUSIVE, note="no trajectory recorded")
    slide = math.dist(tuple(p[-1]), tuple(p[0]))
    # Key ORDER is load-bearing: report.py publishes the first three scalar
    # evidence items as the row's numbers, so the MEASUREMENT has to lead. The
    # previous order opened with three constants read straight off the probe
    # definition, which printed a row that restated its own inputs.
    ev = {"slide_m": slide,
          "declared_mu": 2.0, "required_mu_to_stick": 1.4281,
          "engine_default_mu": 1.0, "incline_deg": 55.0,
          "coulomb_accel_m_s2": -3.2177,
          "subject": "slab 0.6 x 0.3 x 0.06, b/h = 10, topple angle 84.3 deg",
          "negative_arm": "phenomenon.friction_slides_below_coulomb_bound",
          "declaration_reaches_solver":
              "yes -- translation_audit.py reports 3/3 geoms at mu 2"}
    if slide <= 0.05:
        return Verdict(WORKS, ev)
    return Verdict(
        BROKEN, ev,
        "the slab slid %.3f m on a 55 deg slope although every geom carries "
        "the declared mu=2.0 and Coulomb needs only 1.43 to hold it. The "
        "declaration reaches the solver (audited); the CONTACT does not "
        "deliver the friction it was given. (This body cannot topple -- "
        "b/h = 10 -- so a displacement here is a slide.)" % slide)


PROBES[-1].assertion = _friction_assertion

_p(Probe(
    id="phenomenon.friction_slides_below_coulomb_bound",
    family=FAM_PHENOMENON,
    claim="Declared friction BELOW the Coulomb bound correctly fails to hold",
    # THE NEGATIVE ARM, and the reason the positive one is evidence at all.
    # This lane's own rule is that a green which cannot go red proves nothing,
    # and a "declared friction works" probe with no failing arm is exactly
    # that: a body that never moves is equally consistent with the engine
    # honouring mu=2.0, with it clamping every contact to some huge value, and
    # with it welding the subject to the ramp.
    #
    # Identical slab, identical 55 deg ramp, ONE field different: mu = 1.3,
    # which is 9% BELOW tan(55 deg) = 1.4281 and -- deliberately -- 30% ABOVE
    # the engine default of 1.0. So a pass here rules out both failure modes
    # at once: the subject must slide (the declaration is not being silently
    # raised) and it must slide at the rate 1.3 predicts, not the rate 1.0
    # predicts (the declaration is not being silently ignored either).
    #
    # Analytic: a = g(sin 55 - 1.3 cos 55) = 0.7211 m/s^2, so 1.442 m in 2 s.
    # Duration is 2.0 s rather than the lane's usual 3.0 for a physical
    # reason: at 3 s the prediction is 3.245 m and the ramp offers only 3 m of
    # down-slope run, so the slab would leave the ramp and the measurement
    # would become a fall.
    world_info='  newtonGroundMu 1.3\n  newtonCone "elliptic"\n  newtonImpratio 10\n',
    world=_incline_world,
    measure=("pos:SUBJECT",),
    duration=2.0,
    assertion=None,
    doc="tests/benchmarks/omnibench/lane1/translation_audit.py",
))


def _friction_negative_assertion(arrays):
    """The same slab on the same 55 deg incline, at a declared mu of 1.3 --
    BELOW tan(55 deg) = 1.4281 -- must slide, and must slide at roughly the
    rate Coulomb predicts for 1.3.

        a = g (sin theta - mu cos theta)
          = 9.81 (0.8191520 - 1.3 * 0.5735764) = 0.7211 m/s^2
        s(2 s) = 0.5 a t^2 = 1.442 m

    The hard assertion is only the SIGN of the finding -- slide > 0.30 m --
    because a tight tolerance on the distance would be a golden, not a
    prediction, and soft contact settling in the first few ms costs a few cm.
    The ratio to the analytic distance is published as evidence so the row
    stays auditable, and the two silent-failure modes this arm exists to catch
    both blow the 0.30 m gate wide open:

      * stuck (slide ~ 0)  -> the declared 1.3 is not what the contact used;
        something is raising it above the bound. This is the failure that
        would otherwise let the mu=2.0 arm pass for the wrong reason.
      * mu=1.0 rate (a = 2.406 m/s^2, 4.81 m in 2 s -- off the ramp) -> the
        declaration was ignored and the default was used.
    """
    p = arrays.get("pos_SUBJECT")
    if p is None or len(p) < 2:
        return Verdict(INCONCLUSIVE, note="no trajectory recorded")
    slide = math.dist(tuple(p[-1]), tuple(p[0]))
    predicted = 1.442
    # Measurement first, then the analytic value it is being judged against --
    # report.py publishes only the first three scalars, and on this row the
    # pair (measured, predicted) IS the finding.
    ev = {"slide_m": slide,
          "predicted_slide_m_at_2s": predicted,
          "slide_vs_predicted": round(slide / predicted, 3) if predicted else None,
          "declared_mu": 1.3, "required_mu_to_stick": 1.4281,
          "engine_default_mu": 1.0, "incline_deg": 55.0,
          "coulomb_accel_m_s2": 0.7211,
          "subject": "slab 0.6 x 0.3 x 0.06, b/h = 10, topple angle 84.3 deg",
          "positive_arm": "phenomenon.friction_declared_in_world"}
    if slide > 0.30:
        return Verdict(WORKS, ev)
    return Verdict(
        BROKEN, ev,
        "the slab held on a 55 deg slope at a declared mu=1.3, which is BELOW "
        "the tan(55 deg) = 1.4281 Coulomb needs -- it slid only %.3f m where "
        "1.442 m was predicted. Friction is being delivered above what the "
        "world declared, which also means the mu=2.0 arm of this pair is "
        "passing for a reason other than the one it claims." % slide)


PROBES[-1].assertion = _friction_negative_assertion

_p(Probe(
    id="phenomenon.friction_holds_shallow_incline",
    family=FAM_PHENOMENON,
    claim="Declared friction holds a body on a SHALLOW (30 deg) incline",
    # Bounds the 55 deg finding. "Friction is broken" and "friction holds to
    # 30 deg but not 55" are different statements with different consequences,
    # and only the second one tells a user whether their scene is buildable.
    # 30 deg needs mu >= 0.577 -- comfortably inside even the engine default
    # of 1.0, let alone the declared 2.0.
    world_info='  newtonGroundMu 2.0\n  newtonCone "elliptic"\n  newtonImpratio 10\n',
    world=lambda: (
        """DEF RAMP Solid {
  translation 0 0 0.5
  rotation 0 1 0 0.5235987756
  name "ramp"
  children [
    DEF RAMP_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.45 0.47 0.5 roughness 1 metalness 0 }
      geometry Box { size 6 3 0.2 }
    }
  ]
  boundingObject USE RAMP_SHAPE
}
""" + body("SUBJECT", "Box { size 0.2 0.2 0.2 }",
           # surface normal for 30 deg = (sin30, 0, cos30) = (0.5, 0, 0.86603)
           #   ramp centre 0.5 + 0.2 * n  ->  (0.1, 0, 0.673205)
           "0.1 0 0.673205",
           extra="  rotation 0 1 0 0.5235987756\n")),
    measure=("pos:SUBJECT",),
    duration=3.0,
    assertion=None,
))


def _friction_shallow_assertion(arrays):
    """A box on a 30 deg incline (needs mu >= 0.577) with a declared mu of 2.0
    must slide less than 5 cm in 3 s. This probe exists to bound its 55 deg
    sibling: if the shallow slope holds and the steep one does not, the honest
    statement is that declared friction holds up to some angle well below what
    Coulomb allows -- not that friction is broken."""
    p = arrays.get("pos_SUBJECT")
    if p is None or len(p) < 2:
        return Verdict(INCONCLUSIVE, note="no trajectory recorded")
    slide = math.dist(tuple(p[-1]), tuple(p[0]))
    ev = {"declared_mu": 2.0, "incline_deg": 30.0,
          "required_mu_to_stick": 0.5774, "slide_m": slide}
    if slide <= 0.05:
        return Verdict(WORKS, ev)
    return Verdict(BROKEN, ev,
                   "the box slid %.3f m on a 30 deg slope needing only "
                   "mu=0.58, with mu=2.0 declared" % slide)


PROBES[-1].assertion = _friction_shallow_assertion

_p(Probe(
    id="phenomenon.body_sleep_disabled",
    family=FAM_PHENOMENON,
    claim="No body sleep — a long-idle body still reports contacts",
    # ODE auto-disabled idle bodies and a sleeping body generated no contacts,
    # which is why /sim/contacts?wake=1 existed. Newton has no sleep; pin it.
    world=lambda: floor() + body("SUBJECT", "Box { size 0.2 0.2 0.2 }",
                                 "0 0 0.75"),
    world_info="  physicsDisableTime 1\n",
    measure=("contacts:SUBJECT", "pos:SUBJECT"),
    duration=6.0,
    assertion=None,
    doc="AGENTS.md — 'Newton has no body sleep'; ?wake=1 is a no-op",
    documented_as=WORKS,
))


def _sleep_assertion(arrays):
    """A box left undisturbed for 6 s in a world declaring
    physicsDisableTime 1 (the ODE auto-sleep knob) must STILL report contacts
    in the final second. Contacts vanishing after ~1 s would mean auto-sleep
    is live and an idle body has become invisible to contact queries."""
    c = _finite(arrays.get("contacts_SUBJECT"))
    if len(c) < 4:
        return Verdict(INCONCLUSIVE, note="no contact counts recorded")
    tail = c[-max(1, len(c) // 6):]
    head = c[len(c) // 6: len(c) // 3]
    ev = {"mean_contacts_first_third": sum(head) / max(1, len(head)),
          "mean_contacts_last_sixth": sum(tail) / max(1, len(tail)),
          "declared_physicsDisableTime_s": 1.0}
    if ev["mean_contacts_last_sixth"] >= 1.0:
        return Verdict(WORKS, ev)
    if ev["mean_contacts_first_third"] >= 1.0:
        return Verdict(BROKEN, ev,
                       "contacts stopped being reported after the body went "
                       "idle — auto-sleep appears to be active")
    return Verdict(INCONCLUSIVE, ev,
                   "the body never reported contacts at all, so sleep cannot "
                   "be distinguished from a contact-readback failure")


PROBES[-1].assertion = _sleep_assertion

_p(Probe(
    id="phenomenon.implicit_ground_plane",
    family=FAM_PHENOMENON,
    claim="Newton adds an IMPLICIT ground plane at z=0 that no world declares",
    # ⚠ THE EXPECTED VERDICT FLIPPED ON 2026-08-12, and this row is kept
    # (rather than deleted) precisely so the flip is auditable.
    #
    # It used to be `documented_as=WORKS`: Newton added the plane
    # UNCONDITIONALLY, giving every world an undeclared, infinite collision
    # surface at up-axis 0 that appears in no world file and no scene tree.
    # That is what let a world whose floor sits at z=0 pass identically with a
    # working collider and a broken one — how the statics-off defect survived
    # for months, and why every other probe in this lane puts its floor at
    # z=0.55 instead.
    #
    # It is now a DECLARED SUBSTITUTION: the plane is added in finalize() if
    # and only if the world declared a `Plane` collider that had to be dropped
    # (newton's MuJoCo converter cannot build a Plane attached to our
    # weld-pinned statics), and the choice is logged either way. This probe's
    # world declares NO floor at all, so there is nothing to substitute for and
    # the sphere must now FALL. `OMNISIM_NEWTON_GROUND_PLANE=1` restores the
    # old unconditional plane exactly (for a bisect); `=0` refuses it even for
    # the substitution case.
    world=lambda: body("SUBJECT", "Sphere { radius 0.1 subdivision 3 }",
                       "0 0 3.0"),
    measure=("pos:SUBJECT",),
    duration=4.0,
    assertion=None,
    doc="the phantom z=0 plane that masked the statics-off defect — REMOVED as "
        "an unconditional behaviour 2026-08-12, now added only to substitute "
        "for a dropped authored Plane collider",
    documented_as=ABSENT,
))


def _implicit_plane_assertion(arrays):
    """A sphere dropped in a world containing NO floor and NO `Plane` collider
    must FALL — there is nothing for the implicit plane to substitute for.

    ⚠ THIS ASSERTION'S EXPECTED VERDICT IS THE OPPOSITE OF WHAT IT WAS BEFORE
    2026-08-12, and the inversion is the point of keeping the row. It used to
    demand the sphere STOP at z = 0.1 m (its own radius) on an unconditional
    implicit plane, so that the plane's removal could not happen silently.
    The plane has now been removed as an unconditional behaviour on purpose,
    so `absent` is the PASS and `works` is the regression: a sphere resting at
    0.1 m in a world that declares no ground means the unconditional plane is
    back, and with it the masking that hid the statics-off defect and the
    AgentBench C2 fall-through case from `--fail-on-runaway`.

    Neither verdict is a bug report on its own — this row records WHICH engine
    you are talking to. `OMNISIM_NEWTON_GROUND_PLANE=1` reproduces the old
    behaviour exactly, so a `works` here first means "check whether that
    variable is set in this environment"."""
    z = _final(arrays, "pos_SUBJECT", 2)
    if z is None:
        return Verdict(INCONCLUSIVE, note="no pose recorded for SUBJECT")
    ev = {"rest_z_m": z, "expected_rest_z_m": None,
          "world_declares_a_floor": False,
          "world_declares_a_plane_collider": False,
          "rest_z_if_unconditional_plane": 0.1,
          "revert_hatch": "OMNISIM_NEWTON_GROUND_PLANE=1"}
    if z < -1.0:
        return Verdict(
            ABSENT, ev,
            "correct since 2026-08-12: the sphere fell to z=%.3f m in a world "
            "that declares no ground, so no undeclared collision surface was "
            "substituted for one it never asked for" % z)
    if abs(z - 0.1) <= 0.01:
        return Verdict(
            WORKS, ev,
            "the UNCONDITIONAL implicit ground plane is back: the sphere rests "
            "at z=%.4f m (its own radius) in a world that declares no ground "
            "and no Plane collider. Check OMNISIM_NEWTON_GROUND_PLANE before "
            "reporting this as an engine regression." % z)
    return Verdict(DEGRADED, ev,
                   "the sphere neither fell nor rested on an implicit plane: "
                   "z=%.4f m, matching neither hypothesis" % z)


PROBES[-1].assertion = _implicit_plane_assertion

_p(Probe(
    id="phenomenon.physics_plugin_api",
    family=FAM_PHENOMENON,
    claim="Custom physics plugin (webots_physics_init/_collide/_step)",
    kind=KIND_STATIC,
    world_info='  physics "omnibench_plugin"\n',
    world=lambda: floor(),
    absent_markers=("omnibench_plugin", "physics plugin"),
    doc="AGENTS.md — the physics-plugin API went with ODE; WorldInfo.physics "
        "still parses but any value other than \"<none>\" is ignored",
    documented_as=ABSENT,
))


#: The supervisor force probe's numbers. Mass 2 kg and 10 N so the analytic
#: acceleration is a round 5 m/s^2 that a reader can check without dividing.
EXTFORCE_N = 10.0
EXTFORCE_MASS = 2.0
#: Lever arm for the offset arm, in the body's OWN frame -- the engine
#: transforms it by the Solid's matrix (OmSupervisorUtilities.cpp:1204), so
#: `addForceWithOffset` takes a LOCAL point and a world-frame force.
EXTFORCE_OFFSET_M = 0.05
#: The engine's refusal when the supervisor cannot find a body to push.
EXTFORCE_REFUSAL = "add_force"

_p(Probe(
    id="phenomenon.supervisor_external_force",
    family=FAM_PHENOMENON,
    claim="Supervisor addForce / addForceWithOffset reach the Newton solver",
    # THE capability an aerodynamic model is built on: lift, drag and a
    # control-surface moment are all external wrenches on a body the solver
    # already owns. Gravity 0 (rule 4) so the acceleration is the applied
    # force divided by the mass and nothing else.
    gravity=0.0,
    world=lambda: (
        body("SUBJECT", "Box { size 0.2 0.2 0.2 }", "2 0 1.5",
             mass=EXTFORCE_MASS)
        + body("OFFSET_SUBJECT", "Box { size 0.4 0.2 0.2 }", "-2 0 1.5",
               mass=EXTFORCE_MASS, name="offset_subject",
               color="0.3 0.6 0.85")),
    measure=("pos:SUBJECT", "pos:OFFSET_SUBJECT", "quat:OFFSET_SUBJECT"),
    # PERSISTENT, not one-shot. A supervisor wrench is consumed into
    # state.body_f and the accumulator is cleared after the tick, so a single
    # call is a 4 ms impulse and would measure a velocity step rather than an
    # acceleration. The prober re-applies both every step from t=0.2 and
    # records how many times it actually did.
    act=("add_force:SUBJECT:0:0:%s:0.2" % _g(EXTFORCE_N),
         "add_force_offset:OFFSET_SUBJECT:0:0:%s:%s:0:0:0.2"
         % (_g(EXTFORCE_N), _g(EXTFORCE_OFFSET_M))),
    duration=3.0,
    log_capture=("[OmNewtonBackend] registered", EXTFORCE_REFUSAL),
    assertion=None,
    doc="OmSupervisorUtilities.cpp:1167/1213 route add_force and "
        "add_force_with_offset through OmSolid::applyExternalForceNewton "
        "(OmSolid.cpp:4598). ⚠ OmSolid::addForceAtPosition / addTorque are "
        "EMPTY STUBS (OmSolid.cpp:4585-4589), so this probe must exercise the "
        "SUPERVISOR path and no other. Control arm for an A/B: "
        "run_coverage.py --probes phenomenon.supervisor_external_force --env "
        "OMNISIM_NEWTON_NO_EXT_FORCE=1, the documented pre-W3.1 revert hatch "
        "(OmSolid.cpp:4606).",
    documented_as=WORKS,
))


def _supervisor_force_assertion(arrays):
    """A constant 10 N applied to a free 2 kg body in a gravity-free world,
    re-applied every tick through Supervisor addForce, must accelerate it at
    exactly F/m = 5.0 m/s^2 along the force direction, +/- 5%.

    The same 10 N applied through addForceWithOffset, 5 cm off the body's
    origin, must produce the SAME 5.0 m/s^2 at the centre of mass -- net force
    sets the linear response wherever it is applied -- and must additionally
    ROTATE the body, which a force acting ahead of or behind a wing's centre of
    pressure is exactly what a pitching moment is.

    A body that does not move while the supervisor call is made hundreds of
    times is an external-force path that never reaches the solver. The call
    count is recorded, so 'the force was dropped' and 'the force was never
    asked for' are separated by measurement."""
    expected = EXTFORCE_N / EXTFORCE_MASS
    a, window = _accel_along(arrays, "pos_SUBJECT", 2)
    a_off, _w = _accel_along(arrays, "pos_OFFSET_SUBJECT", 2)
    spin, _n = _quat_rotation_total(arrays.get("quat_OFFSET_SUBJECT"))
    acted = arrays.get("acted_add_force_SUBJECT") or {}
    acted_off = arrays.get("acted_add_force_offset_OFFSET_SUBJECT") or {}
    calls = acted.get("calls")
    bodies = _newton_dynamic_bodies(arrays)
    refusal = _log_hits(arrays, EXTFORCE_REFUSAL)
    # Keep only lines that are actually diagnostics: the controller's own
    # command line is echoed into the engine log and contains the act spec.
    refusal = [l for l in refusal if "WARNING" in l.upper() or "ERROR" in l.upper()]
    if a is None:
        return Verdict(INCONCLUSIVE, {"force_applications": calls,
                                      "newton_dynamic_bodies": bodies},
                       "no usable trajectory was recorded for SUBJECT")
    ev = {
        "measured_accel_m_s2": a,
        "analytic_accel_m_s2": expected,
        "accel_ratio": a / expected if expected else None,
        "offset_arm_accel_m_s2": a_off,
        "offset_arm_rotation_rad": spin,
        "force_applications": calls,
        "offset_force_applications": acted_off.get("calls"),
        "force_N": EXTFORCE_N,
        "mass_kg": EXTFORCE_MASS,
        "axial_travel_m": _travel(arrays, "pos_SUBJECT", 2),
        "newton_dynamic_bodies": bodies,
        "window": window,
        "engine_refusal": refusal,
    }
    # PREMISE (rule 2): a force that was never applied says nothing about the
    # path that would have carried it.
    if not calls:
        return Verdict(
            INCONCLUSIVE, ev,
            "the supervisor addForce call was made %r times (node_found=%r), "
            "so this is a RIG failure and says nothing about the external-force "
            "path" % (calls, acted.get("node_found")))
    if bodies is not None and bodies < 2:
        return Verdict(INCONCLUSIVE, ev,
                       "the engine registered %d dynamic Newton bodies for a "
                       "world declaring two free Solids, so at least one "
                       "subject is not a rigid body" % bodies)
    rotated = spin is not None and spin > 0.2
    linear_ok = abs(a - expected) <= 0.05 * expected
    offset_linear_ok = (a_off is not None
                        and abs(a_off - expected) <= 0.05 * expected)
    if linear_ok and offset_linear_ok and rotated:
        return Verdict(WORKS, ev)
    if abs(a) <= 0.01 * expected:
        note = ("%d supervisor addForce calls delivering %.0f N to a %.0f kg "
                "body moved it %.3g m: the external-force path never reaches "
                "the solver"
                % (calls, EXTFORCE_N, EXTFORCE_MASS, ev["axial_travel_m"] or 0.0))
        if refusal:
            note += (". The engine answers %r for a body Newton registered as "
                     "DYNAMIC -- OmSolid::bodyMerger() returns the legacy ODE "
                     "handle OmSolidMerger::mBody, left permanently NULL by the "
                     "ODE deletion, so the supervisor takes its 'kinematic "
                     "Solid' branch and applyExternalForceNewton is never called"
                     % refusal[0][-110:])
        return Verdict(BROKEN, ev, note)
    if linear_ok and not rotated:
        return Verdict(DEGRADED, ev,
                       "the centre-force arm accelerated correctly at %.4f "
                       "m/s^2, but the 5 cm offset arm swept only %.4f rad -- "
                       "the offset is being dropped, so a force can be applied "
                       "but no moment" % (a, spin or 0.0))
    return Verdict(DEGRADED, ev,
                   "the body accelerated at %.4f m/s^2 against an analytic "
                   "%.4f (ratio %.4f)" % (a, expected, a / expected))


PROBES[-1].assertion = _supervisor_force_assertion


def by_family():
    out = {f: [] for f in FAMILIES}
    for p in PROBES:
        out[p.family].append(p)
    return out


def get(probe_id):
    for p in PROBES:
        if p.id == probe_id:
            return p
    raise KeyError(probe_id)


def validate_registry():
    """Structural self-check — every dynamic probe must carry an assertion
    with a docstring (it is published as the physical claim), and every id
    must be unique."""
    errs = []
    seen = set()
    for p in PROBES:
        if p.id in seen:
            errs.append("duplicate probe id %r" % p.id)
        seen.add(p.id)
        if p.family not in FAMILIES:
            errs.append("%s: unknown family %r" % (p.id, p.family))
        if p.kind == KIND_DYNAMIC:
            if p.assertion is None:
                errs.append("%s: dynamic probe has no assertion" % p.id)
            elif not (p.assertion.__doc__ or "").strip():
                errs.append("%s: assertion has no docstring (the docstring IS "
                            "the published physical claim)" % p.id)
            if p.world is None:
                errs.append("%s: dynamic probe has no world" % p.id)
        else:
            if not p.absent_markers:
                errs.append("%s: static probe has no absent_markers" % p.id)
        if p.documented_as is not None and p.documented_as not in VERDICTS:
            errs.append("%s: documented_as=%r is not a verdict"
                        % (p.id, p.documented_as))
    return errs


# ---------------------------------------------------------------------------
# red-capability self-test
# ---------------------------------------------------------------------------
def self_test():
    """Feed SYNTHETIC recordings to the turntable and aircraft assertions and
    check each lands on the verdict that recording's physics demands.

    Rule 9 says a green that cannot be made to go red is not evidence. That is
    not a claim about a probe's assertion so much as about the whole
    instrument, and the three device probes rebuilt on the turntable are the
    lane's own worked example of getting it wrong: `device.gyro` published
    `works` for months on `omega = [0,0,0]`, a reading a DEAD gyro produces
    identically. This function is the standing guard against that returning.
    It runs offline in milliseconds and needs no engine, so there is no excuse
    for trusting one of these greens without it.

    Modelled on lane1/translation_audit.py --self-test, for the same reason:
    prove the instrument can go red BEFORE believing that it is green.

    The aircraft cases are here for the mirror-image reason. Those three
    probes measure `broken` on the shipped engine, and a RED verdict needs its
    instrument proved just as hard as a green one: each carries a synthetic
    run in which the capability WORKS, so "it reported broken" cannot be a
    stuck needle. Both directions are covered, plus the rig failures that must
    come back `inconclusive` rather than as a statement about the engine.

    Returns a list of failure strings (empty == all good).
    """
    dt, n, rate, r = 0.004, 501, TURNTABLE_RAD_S, TURNTABLE_RADIUS

    def yaw_quat(a):
        return [math.cos(a / 2.0), 0.0, 0.0, math.sin(a / 2.0)]

    def wrap(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def rig(rotating=True):
        t = [k * dt for k in range(n)]
        th = [(rate * x if rotating else 0.0) for x in t]
        return th, {
            "t": t,
            "quat_SUBJECT": [yaw_quat(a) for a in th],
            "quat_REFERENCE": [[1.0, 0.0, 0.0, 0.0]] * n,
            "pos_SUBJECT": [[r * math.cos(a), r * math.sin(a), 1.5]
                            for a in th],
            "pos_OMNIBENCH_PROBER": [[0.0, 0.0, 1.5]] * n,
        }

    def gyro(spin, static, rotating=True):
        _, a = rig(rotating)
        a["sensor_gyro_spin"] = [list(spin)] * n
        a["sensor_gyro_static"] = [list(static)] * n
        return a

    def imu(spin_chan, tilt, level, rotating=True):
        th, a = rig(rotating)
        a["sensor_imu_spin"] = [
            [wrap(x) if i == spin_chan else 0.0 for i in range(3)] for x in th]
        a["sensor_imu_tilt"] = [list(tilt)] * n
        a["sensor_imu_level"] = [list(level)] * n
        return a

    def gps(spin, static=None, rotating=True):
        _, a = rig(rotating)
        a["sensor_gps_spin"] = (list(a["pos_SUBJECT"]) if spin is None
                                else [list(spin)] * n)
        a["sensor_gps_static"] = ([[0.0, 0.0, 1.5]] * n if static is None
                                  else static)
        return a

    def acc(spin, static, rotating=True, exists=True):
        """Accelerometer rig: `spin`/`static` are constant [x,y,z] readings
        (None = an all-NaN series, the publishes-nothing failure mode)."""
        _, a = rig(rotating)
        nan3 = [float("nan")] * 3
        a["sensor_acc_spin"] = [list(spin) if spin is not None else nan3] * n
        a["sensor_acc_static"] = ([list(static) if static is not None
                                   else nan3] * n)
        a["device_exists_acc_spin"] = exists
        a["device_exists_acc_static"] = exists
        return a

    def nested(gd, ad, gn, an, rotating=True):
        """Nested-carrier rig: direct gyro/acc and nested gyro/acc constant
        readings (None = all-NaN, the carrier publishing nothing)."""
        _, a = rig(rotating)
        nan3 = [float("nan")] * 3
        a["sensor_gyro_direct"] = [list(gd) if gd is not None else nan3] * n
        a["sensor_acc_direct"] = [list(ad) if ad is not None else nan3] * n
        a["sensor_gyro_nested"] = [list(gn) if gn is not None else nan3] * n
        a["sensor_acc_nested"] = [list(an) if an is not None else nan3] * n
        return a

    def with_series(arrays, key, value):
        """Override one recorded series and hand the recording back, so a case
        can be written as a one-line deviation from a healthy rig."""
        arrays[key] = value
        return arrays

    # ---- aircraft rigs: free-flight / free-fall trajectories --------------
    # All three assertions read acceleration through _accel_along, which is a
    # second difference and therefore EXACT on a constant-acceleration
    # trajectory. The synthetic runs below are quadratics (or the closed-form
    # solution of the inflow ODE), sampled at the same 4 ms as the engine, so a
    # case here is the trajectory the physics demands and nothing else.
    AN, ADT = 751, 0.004
    at = [k * ADT for k in range(AN)]

    def air_log(bodies=1, prop_warn=False, force_refusal=False):
        out = []
        if bodies is not None:
            out.append("INFO: [OmNewtonBackend] registered %d dynamic + 0 "
                       "static Newton bodies (+%d this pass) (statics: )"
                       % (bodies, bodies))
        if prop_warn:
            out.append("WARNING: DEF OMNIBENCH_PROBER Robot > DEF ROTOR "
                       "Propeller: " + PROP_NO_BODY_WARNING
                       + " to enable thrust and torque effect.")
        if force_refusal:
            out.append("WARNING: DEF OMNIBENCH_PROBER Robot: "
                       "wb_supervisor_node_add_force() can't be used with a "
                       "kinematic Solid")
        return out

    def flight(accel, bodies=1, prop_warn=False):
        """Gravity-free climb under a constant thrust of accel*mass."""
        return {"t": at,
                "pos_OMNIBENCH_PROBER": [[0.0, 0.0, 2.0 + 0.5 * accel * x * x]
                                         for x in at],
                "quat_OMNIBENCH_PROBER": [[1.0, 0.0, 0.0, 0.0]] * AN,
                "engine_log": air_log(bodies, prop_warn=prop_warn)}

    def descent(kind, bodies=1):
        """Powered descent. `inflow` is the closed-form solution of
        m*dv/dt = -m*g + t1*w^2 - t2*w*v (thrust rises with airspeed, so the
        fall reaches a terminal speed); `constant` is the same static thrust
        with NO airspeed dependence; `none` is gravity alone; `partial` decays
        the acceleration only 20% across the run; `ramp` is `constant` thrust
        behind the MEASURED rotor spin-up (omega rising linearly at
        maxTorque = 100 rad/s^2 for a full second), which is the recording that
        made this probe publish `degraded` on 2026-08-22."""
        g, m = 9.81, PROP_MASS
        st = PROP_INFLOW_T1 * PROP_OMEGA * PROP_OMEGA
        powered = g - st / m
        vt = (m * g - st) / (PROP_INFLOW_T2 * PROP_OMEGA)
        tau = m / (PROP_INFLOW_T2 * PROP_OMEGA)
        if kind == "inflow":
            z = [100.0 - vt * (x - tau * (1.0 - math.exp(-x / tau)))
                 for x in at]
        elif kind == "constant":
            z = [100.0 - 0.5 * powered * x * x for x in at]
        elif kind == "partial":
            z = [100.0 - 0.5 * powered * x * x
                 + (0.05 * powered / 3.0) * x ** 3 for x in at]
        elif kind == "ramp":
            # a(x) = -g + t1*min(RAMP*x, omega)^2 / m, integrated twice from
            # rest. RAMP is the OLD maxTorque, i.e. the pre-repair rig.
            RAMP = 100.0
            z, v, zz = [], 0.0, 100.0
            for x in at:
                w = min(RAMP * x, PROP_OMEGA)
                z.append(zz)
                a = -g + PROP_INFLOW_T1 * w * w / m
                zz += v * ADT + 0.5 * a * ADT * ADT
                v += a * ADT
        else:
            z = [100.0 - 0.5 * g * x * x for x in at]
        return {"t": at,
                "pos_OMNIBENCH_PROBER": [[0.0, 0.0, v] for v in z],
                "engine_log": air_log(bodies)}

    def pitch_quat(a):
        return [math.cos(a / 2.0), 0.0, math.sin(a / 2.0), 0.0]

    def extforce(accel, off_accel=None, alpha=15.0, calls=700, bodies=2,
                 refusal=False):
        off = accel if off_accel is None else off_accel
        return {
            "t": at,
            "pos_SUBJECT": [[2.0, 0.0, 1.5 + 0.5 * accel * x * x] for x in at],
            "pos_OFFSET_SUBJECT": [[-2.0, 0.0, 1.5 + 0.5 * off * x * x]
                                   for x in at],
            "quat_OFFSET_SUBJECT": [pitch_quat(0.5 * alpha * x * x)
                                    for x in at],
            "acted_add_force_SUBJECT": {"calls": calls, "node_found": True,
                                        "error": None},
            "acted_add_force_offset_OFFSET_SUBJECT": {
                "calls": calls, "node_found": True, "error": None},
            "engine_log": air_log(bodies, force_refusal=refusal),
        }

    # ---- particle rigs: synthetic getParticleStats recordings -------------
    # Every case is the trajectory its physics demands, sampled like the
    # engine would, so a green here proves the assertion reads the stats
    # frames right and a red proves it can refuse them — both directions,
    # per rule 9.
    PPN = 251

    def pframe(count, mn, mx, cen, nf=0, status=0):
        return {"status": status, "count": count, "min": list(mn),
                "max": list(mx), "centroid": list(cen), "non_finite": nf}

    def ramp(x, knee=0.4):
        return min(x / knee, 1.0)

    def cloth_rec(count=441, nf=0, status=0, pin_drop=0.0, flat=False,
                  over=False, sphere=0.65, exists=True, missing=False):
        frames = []
        for k in range(PPN):
            x = k / (PPN - 1.0)
            ext = 0.0 if flat else (1.2 * x if over else 0.9 * x)
            top = 2.0 - pin_drop * x
            frames.append(pframe(
                count, [0.0, 0.0, top - max(ext, 0.001)], [1.0, 1.0, top],
                [0.5, 0.5, top - ext / 2.0],
                nf=(nf if k == PPN - 1 else 0), status=status))
        return {"t": [3.0 * k / (PPN - 1.0) for k in range(PPN)],
                "particles_SHEET": ([None] * PPN if missing else frames),
                "node_exists_SHEET": exists,
                "pos_SUBJECT": [[-0.6, 0.0, sphere]] * PPN,
                "engine_log": []}

    def fem_rec(cz_fn, ez_fn, exy_fn, count=125, nf=0, status=0,
                sphere=0.65):
        frames, ts = [], []
        for k in range(PPN):
            x = k / (PPN - 1.0)
            ts.append(3.0 * x)
            c, ez, exy = cz_fn(x), ez_fn(x), exy_fn(x)
            frames.append(pframe(
                count, [-exy / 2.0, -exy / 2.0, c - ez / 2.0],
                [exy / 2.0, exy / 2.0, c + ez / 2.0], [0.0, 0.0, c],
                nf=nf, status=status))
        return {"t": ts, "particles_SUBJECT_BLOB": frames,
                "node_exists_SUBJECT_BLOB": True,
                "pos_SUBJECT": [[0.6, 0.0, sphere]] * PPN}

    def grains_rec(settle=True, count=64, status=0, exists=True,
                   missing=False, log=()):
        frames, ts = [], []
        for k in range(PPN):
            x = k / (PPN - 1.0)
            ts.append(4.0 * x)
            r = ramp(x) if settle else x        # settle=False: never arrests
            c = 1.75 - 1.70 * r
            e = 0.5 - 0.4 * r
            frames.append(pframe(count, [-0.4, -0.4, c - e / 2.0],
                                 [0.4, 0.4, c + e / 2.0], [0.0, 0.0, c],
                                 status=status))
        return {"t": ts,
                "particles_SUBJECT_GRAINS": ([None] * PPN if missing
                                             else frames),
                "node_exists_SUBJECT_GRAINS": exists,
                "pos_SUBJECT": [[-2.0, 0.0, 0.65]] * PPN,
                "engine_log": list(log)}

    GRAINS_INERT_LOG = ("WARNING: GranularGroup is inert: CUDA is not "
                        "available on this build/box. Particles will not "
                        "simulate; the world remains loadable.",)

    def bed_rec(kind="works", nf=0, status=0, log=()):
        frames, ts = [], []
        for k in range(PPN):
            x = k / (PPN - 1.0)
            ts.append(4.0 * x)
            r = 0.0 if kind == "frozen" else ramp(x)
            c = 0.80 - 0.19 * r
            e = 0.20 - 0.07 * r
            frames.append(pframe(8125, [-0.2, -0.2, c - e / 2.0],
                                 [0.2, 0.2, c + e / 2.0], [0.0, 0.0, c],
                                 nf=nf, status=status))
        return {"t": ts, "particles_BED": frames, "node_exists_BED": True,
                "engine_log": list(log)}

    BED_REFUSAL_LOG = ("ERROR: [python] GranularBed requires CUDA and the "
                       "model finalized on device 'cpu'. SolverImplicitMPM "
                       "is a warp GPU solver ...",)

    def weld_rec(held="hold", ctrl_fall=True, exists=True, log=()):
        ts = [2.5 * k / (PPN - 1.0) for k in range(PPN)]
        held_z, ctrl_z = [], []
        for ti in ts:
            ctrl_z.append(max(0.60, 1.2 - 4.905 * ti * ti) if ctrl_fall
                          else 1.2)
            if held == "hold":
                h = 1.2 - 4.905 * min(ti, 0.1) ** 2
            elif held == "fall":
                h = max(0.60, 1.2 - 4.905 * ti * ti)
            else:                                # "sag": a creeping weld
                h = 1.2 - 0.1 * (ti / 2.5)
            held_z.append(h)
        return {"t": ts, "device_exists_conn": exists,
                "pos_HELD": [[0.0, 0.0, z] for z in held_z],
                "pos_CONTROL": [[0.8, 0.0, z] for z in ctrl_z],
                "engine_log": list(log)}

    WELD_INERT_LOG = ("WARNING: DEF OMNIBENCH_PROBER Robot > Connector "
                      "'conn' locks are physically INERT under the Newton "
                      "backend: ...",)

    def delrec(phantom=True, rebuild_releases=True, binding=True,
               called=True, settled=True, native_fall=False):
        ts = [5.0 * k / 500.0 for k in range(501)]
        zs = []
        for ti in ts:
            if ti < 0.4:
                z = max(0.65, 1.0 - 4.905 * ti * ti)
            elif ti < 1.5:
                z = 0.65 if settled else 0.9
            elif ti < 3.0:
                z = (0.65 - 4.905 * (ti - 1.5) ** 2 if native_fall
                     else 0.65)
            else:
                z = (0.65 - 4.905 * (ti - 3.0) ** 2 if (rebuild_releases
                     or native_fall) else 0.65)
            zs.append(z)
        rec = {"t": ts, "pos_SUBJECT": [[0.0, 0.0, z] for z in zs]}
        if binding is not None:
            rec["acted_rebuild_physics"] = {
                "requested_t": 3.0, "binding_present": binding,
                "called": called, "error": None}
        return rec

    z3 = [0.0, 0.0, 0.0]
    spun = [0.0, 0.0, rate]
    tilted = [IMU_TILT_RAD, 0.0, 0.0]
    cases = [
        # (label, assertion, arrays, expected verdict)
        ("gyro reads the commanded rate", _gyro_assertion,
         gyro(spun, z3), WORKS),
        # THE defect this repair exists for: the exact evidence the old probe
        # published as `works`.
        ("gyro reads [0,0,0] while the body turns", _gyro_assertion,
         gyro(z3, z3), BROKEN),
        ("gyro reads half the measured rate", _gyro_assertion,
         gyro([0.0, 0.0, rate / 2.0], z3), DEGRADED),
        ("gyro returns a CONSTANT (resting arm reads it too)", _gyro_assertion,
         gyro(spun, spun), DEGRADED),
        ("turntable never turned -> rig failure, not a gyro finding",
         _gyro_assertion, gyro(z3, z3, rotating=False), INCONCLUSIVE),

        ("imu yaw sweeps with the body", _imu_assertion,
         imu(2, tilted, z3), WORKS),
        ("imu orientation frozen while the body turns", _imu_assertion,
         with_series(imu(2, tilted, z3), "sensor_imu_spin", [z3] * n), BROKEN),
        ("imu tilt reads 0.0 -- the old vacuous evidence", _imu_assertion,
         imu(2, z3, z3), DEGRADED),
        ("imu negative arm: the LEVEL device reads tilted", _imu_assertion,
         imu(2, tilted, [0.2, 0.0, 0.0]), DEGRADED),
        # Euler-convention robustness: the same physical rotation reported in
        # channel 0 must still pass (rule 3).
        ("imu reports the rotation in channel 0, not yaw", _imu_assertion,
         imu(0, tilted, z3), WORKS),

        ("gps tracks the moving body", _gps_assertion, gps(None), WORKS),
        ("gps FROZEN at its spawn pose", _gps_assertion,
         gps([r, 0.0, 1.5]), BROKEN),
        ("gps biased by 5 cm", _gps_assertion,
         with_series(gps(None), "sensor_gps_spin",
                     [[p[0] + 0.05, p[1], p[2]]
                      for p in rig()[1]["pos_SUBJECT"]]), DEGRADED),
        ("gps negative arm: the PARKED device reports motion", _gps_assertion,
         with_series(gps(None), "sensor_gps_static",
                     list(rig()[1]["pos_SUBJECT"])), DEGRADED),

        # ---- accelerometer (gravity ON: centripetal rate^2*r + g) --------
        ("accelerometer reads centripetal + gravity, resting arm reads g",
         _accelerometer_assertion,
         acc([rate * rate * r, 0.0, 9.81], [0.0, 0.0, 9.81]), WORKS),
        # The known failure mode: accepted, publishes NOTHING (all-NaN).
        ("accelerometer publishes no sample under gravity + rotation",
         _accelerometer_assertion, acc(None, [0.0, 0.0, 9.81]), BROKEN),
        ("accelerometer reads [0,0,0] under 9.81 gravity",
         _accelerometer_assertion,
         acc([0.0, 0.0, 0.0], [0.0, 0.0, 9.81]), BROKEN),
        ("accelerometer misses the centripetal term",
         _accelerometer_assertion,
         acc([0.0, 0.0, 9.81], [0.0, 0.0, 9.81]), DEGRADED),
        ("accelerometer negative arm: the RESTING device reads zeros",
         _accelerometer_assertion,
         acc([rate * rate * r, 0.0, 9.81], [0.0, 0.0, 0.0]), DEGRADED),
        ("accelerometer device never found -> instrument failure",
         _accelerometer_assertion,
         acc(None, None, exists=False), INCONCLUSIVE),
        ("accelerometer turntable never turned -> rig failure",
         _accelerometer_assertion,
         acc([0.0, 0.0, 9.81], [0.0, 0.0, 9.81], rotating=False),
         INCONCLUSIVE),

        # ---- nested-carrier IMU (gravity 0: centripetal only) ------------
        ("nested carrier serves both devices (post-fix green)",
         _imu_nested_carrier_assertion,
         nested([0.0, 0.0, rate], [rate * rate * r, 0.0, 0.0],
                [0.0, 0.0, rate], [rate * rate * r, 0.0, 0.0]), WORKS),
        # The shipped engine: controls read, the fold reads zeros (gyro) /
        # nothing (accelerometer) -- README rule 11's own measurement.
        ("nested carrier reads zeros/NaN while the controls track",
         _imu_nested_carrier_assertion,
         nested([0.0, 0.0, rate], [rate * rate * r, 0.0, 0.0],
                [0.0, 0.0, 0.0], None), BROKEN),
        ("nested carrier serves the gyro but not the accelerometer",
         _imu_nested_carrier_assertion,
         nested([0.0, 0.0, rate], [rate * rate * r, 0.0, 0.0],
                [0.0, 0.0, rate], [0.0, 0.0, 0.0]), DEGRADED),
        ("nested carrier: BOTH controls dead -> not a carrier finding",
         _imu_nested_carrier_assertion,
         nested([0.0, 0.0, 0.0], None, [0.0, 0.0, 0.0], None), INCONCLUSIVE),
        ("nested carrier turntable never turned -> rig failure",
         _imu_nested_carrier_assertion,
         nested([0.0, 0.0, 0.0], None, [0.0, 0.0, 0.0], None,
                rotating=False), INCONCLUSIVE),

        # ---- aircraft: Propeller static thrust ---------------------------
        ("propeller lifts the airframe at T/m", _propeller_thrust_assertion,
         flight(PROP_T1 * PROP_OMEGA * PROP_OMEGA / PROP_MASS), WORKS),
        # THE RED ARM: the rotor spins and the airframe never moves. This is
        # what the shipped engine measures, and a green that could not produce
        # it would not be evidence.
        ("propeller: rotor spinning, airframe never moves",
         _propeller_thrust_assertion, flight(0.0, prop_warn=True), BROKEN),
        ("propeller delivers half its analytic thrust",
         _propeller_thrust_assertion,
         flight(0.5 * PROP_T1 * PROP_OMEGA * PROP_OMEGA / PROP_MASS), DEGRADED),
        ("propeller: no Newton body -> rig failure, not a device finding",
         _propeller_thrust_assertion, flight(0.0, bodies=0, prop_warn=True),
         INCONCLUSIVE),

        # ---- aircraft: Propeller inflow (speed-of-advance) term -----------
        ("inflow: thrust rises with airspeed, descent reaches terminal speed",
         _propeller_inflow_assertion, descent("inflow"), WORKS),
        ("inflow: descent acceleration constant, only the omega^2 term",
         _propeller_inflow_assertion, descent("constant"), BROKEN),
        # The over-determined case: the whole wrench is dropped, so the ratio
        # is 1.0 for a reason that is not about thrustConstants[1]. It is still
        # `broken`, and the note must attribute it elsewhere.
        ("inflow: no thrust at all, the airframe falls at g",
         _propeller_inflow_assertion, descent("none"), BROKEN),
        # THE RECALIBRATION ARM (2026-08-22). Constant thrust behind the
        # measured 1.0 s rotor spin-up: the trajectory this probe scored
        # `degraded -- some airspeed dependence` on, from a rising omega and
        # nothing else. It must now report an INSTRUMENT failure, never a
        # verdict about thrustConstants[1].
        ("inflow: rotor still spinning up -> instrument failure, not a finding",
         _propeller_inflow_assertion, descent("ramp"), INCONCLUSIVE),
        ("inflow: partial airspeed dependence", _propeller_inflow_assertion,
         descent("partial"), DEGRADED),
        ("inflow: no trajectory recorded -> instrument failure",
         _propeller_inflow_assertion,
         {"t": [], "pos_OMNIBENCH_PROBER": [], "engine_log": []}, INCONCLUSIVE),

        # ---- aircraft: supervisor external force --------------------------
        ("supervisor force accelerates at F/m and the offset arm rotates",
         _supervisor_force_assertion,
         extforce(EXTFORCE_N / EXTFORCE_MASS), WORKS),
        # THE RED ARM: the call is made 700 times and nothing moves. This is
        # what the shipped engine measures.
        ("supervisor force applied 700 times, the body never moved",
         _supervisor_force_assertion, extforce(0.0, refusal=True), BROKEN),
        ("supervisor force lands but the offset produces no moment",
         _supervisor_force_assertion,
         extforce(EXTFORCE_N / EXTFORCE_MASS, alpha=0.0), DEGRADED),
        ("supervisor force: the call was never made -> rig failure",
         _supervisor_force_assertion, extforce(0.0, calls=0), INCONCLUSIVE),

        # ---- cloth: the measured drape (round 3, getParticleStats) --------
        ("cloth drapes: 441 particles, extent grows, pin holds",
         _cloth_assertion, cloth_rec(), WORKS),
        # THE RED ARM this rework exists for: particles allocated, fabric
        # never moves -- the old log-scrape verdict could not see this.
        ("cloth registered and NEVER moves", _cloth_assertion,
         cloth_rec(flat=True), BROKEN),
        ("cloth readback status -9 (stale libController) -> inconclusive",
         _cloth_assertion, cloth_rec(status=-9), INCONCLUSIVE),
        ("cloth readback missing entirely -> inconclusive",
         _cloth_assertion, cloth_rec(missing=True), INCONCLUSIVE),
        ("cloth refusal status is NEVER broken", _cloth_assertion,
         cloth_rec(status=-5), INCONCLUSIVE),
        ("cloth carries 440 particles, not the authored 441",
         _cloth_assertion, cloth_rec(count=440), DEGRADED),
        ("cloth pinned edge lets go", _cloth_assertion,
         cloth_rec(pin_drop=0.4), DEGRADED),
        ("cloth over-stretches past the free-edge bound", _cloth_assertion,
         cloth_rec(over=True), DEGRADED),
        ("cloth run has non-finite particles", _cloth_assertion,
         cloth_rec(nf=3), DEGRADED),
        ("cloth perturbs the rigid scene", _cloth_assertion,
         cloth_rec(sphere=0.30), BROKEN),
        ("cloth node not in the scene tree", _cloth_assertion,
         cloth_rec(exists=False), ABSENT),

        # ---- soft body: fall + arrest + deform (round 3) ------------------
        ("soft body falls, arrests and squashes", _soft_body_assertion,
         fem_rec(lambda x: 1.1 - 0.48 * ramp(x),
                 lambda x: 0.2 - 0.06 * ramp(x),
                 lambda x: 0.2 + 0.05 * ramp(x)), WORKS),
        ("soft body rigid-translates without deforming",
         _soft_body_assertion,
         fem_rec(lambda x: 1.1 - 0.48 * ramp(x),
                 lambda x: 0.2, lambda x: 0.2), DEGRADED),
        ("soft body never falls -> the tets do nothing",
         _soft_body_assertion,
         fem_rec(lambda x: 1.1, lambda x: 0.2, lambda x: 0.2), BROKEN),
        ("soft body still moving at run end", _soft_body_assertion,
         fem_rec(lambda x: 1.1 - 0.5 * x,
                 lambda x: 0.2 - 0.06 * ramp(x),
                 lambda x: 0.2 + 0.05 * ramp(x)), DEGRADED),
        ("soft body readback refuses -> inconclusive", _soft_body_assertion,
         fem_rec(lambda x: 1.1, lambda x: 0.2, lambda x: 0.2, status=-3),
         INCONCLUSIVE),
        ("soft body carries 124 particles", _soft_body_assertion,
         fem_rec(lambda x: 1.1 - 0.48 * ramp(x),
                 lambda x: 0.2 - 0.06 * ramp(x),
                 lambda x: 0.2 + 0.05 * ramp(x), count=124), DEGRADED),

        # ---- granular group: CUDA scope + measured settle (round 3) -------
        ("granular group settles onto its own floor", _granular_assertion,
         grains_rec(), WORKS),
        # The shipped no-CUDA box: the engine's own inert line is a SCOPE
        # statement, and the old `degraded -- no readback` excuse is retired.
        ("granular group CUDA-inert line -> absent, never broken",
         _granular_assertion, grains_rec(missing=True,
                                         log=GRAINS_INERT_LOG), ABSENT),
        ("granular group status -5 -> absent (the binding's inert code)",
         _granular_assertion, grains_rec(status=-5), ABSENT),
        ("granular group readback missing, no inert line -> inconclusive",
         _granular_assertion, grains_rec(missing=True), INCONCLUSIVE),
        ("granular group never arrests", _granular_assertion,
         grains_rec(settle=False), DEGRADED),
        ("granular group node not in the scene tree", _granular_assertion,
         grains_rec(exists=False), ABSENT),
        ("granular group allocated 63 of 64", _granular_assertion,
         grains_rec(count=63), DEGRADED),

        # ---- granular bed (MPM): settle + the named CUDA refusal ----------
        ("granular bed drops into the pen and settles",
         _granular_bed_assertion, bed_rec(), WORKS),
        ("granular bed CUDA refusal -> absent with the named reason",
         _granular_bed_assertion, bed_rec(log=BED_REFUSAL_LOG), ABSENT),
        # The reference world's documented failure mode: the MPM solver is
        # silently skipped and the bed registers, renders and does nothing.
        ("granular bed frozen at its authored pose", _granular_bed_assertion,
         bed_rec(kind="frozen"), BROKEN),
        ("granular bed goes non-finite", _granular_bed_assertion,
         bed_rec(nf=1), DEGRADED),
        ("granular bed stale libController -> inconclusive",
         _granular_bed_assertion, bed_rec(status=-9), INCONCLUSIVE),

        # ---- connector weld: gravity-hang vs the in-run control -----------
        ("weld holds the hanging part while the twin falls",
         _connector_assertion, weld_rec(), WORKS),
        # THE RED ARM: lock accepted, part falls exactly like the control.
        ("weld constrains nothing -- held part falls with its twin",
         _connector_assertion, weld_rec(held="fall"), BROKEN),
        ("weld creeps -- the part sags after the lock", _connector_assertion,
         weld_rec(held="sag"), DEGRADED),
        ("control twin never fell -> scene failure, not a weld verdict",
         _connector_assertion, weld_rec(ctrl_fall=False), INCONCLUSIVE),
        ("connector device missing", _connector_assertion,
         weld_rec(exists=False), ABSENT),
        ("weld gate reported inert -> degraded, names the gate",
         _connector_assertion, weld_rec(held="fall", log=WELD_INERT_LOG),
         DEGRADED),

        # ---- runtime deletion: the two-arm rebuild workflow (round 3) -----
        ("phantom holds, simulationRebuildPhysics releases the box",
         _deletion_assertion, delrec(), WORKS),
        ("rebuild verb does not purge the deleted floor either",
         _deletion_assertion, delrec(rebuild_releases=False), BROKEN),
        ("rebuild binding missing -> stale libController, inconclusive",
         _deletion_assertion, delrec(binding=False), INCONCLUSIVE),
        ("rebuild premise never recorded -> inconclusive",
         _deletion_assertion, delrec(binding=None), INCONCLUSIVE),
        ("rebuild call raised -> inconclusive", _deletion_assertion,
         delrec(called=False), INCONCLUSIVE),
        ("deletion works natively, no rebuild needed", _deletion_assertion,
         delrec(native_fall=True), WORKS),
        ("box never settled before the delete -> rig failure",
         _deletion_assertion, delrec(settled=False), INCONCLUSIVE),
    ]
    fails = []
    for label, fn, arrays, want in cases:
        got = fn(arrays)
        if got.verdict != want:
            fails.append("%s: got %s, expected %s (evidence %r)"
                         % (label, got.verdict, want, got.evidence))
    return fails


if __name__ == "__main__":
    import sys as _sys
    if "--self-test" in _sys.argv:
        _fails = self_test()
        print("lane4 device-assertion self-test")
        for _f in _fails:
            print("  FAIL " + _f)
        if _fails:
            _sys.exit(1)
        print("all cases produced the expected verdict -- the turntable and "
              "aircraft assertions are RED-capable, not merely green (and the "
              "ones that measure BROKEN can also go green)")
        _sys.exit(0)
    problems = validate_registry()
    fams = by_family()
    print("OmniBench lane 4a capability registry: %d probes" % len(PROBES))
    for f in FAMILIES:
        print("  %-12s %d" % (f, len(fams[f])))
    print("  %-12s %d dynamic / %d static"
          % ("kind",
             sum(1 for p in PROBES if p.kind == KIND_DYNAMIC),
             sum(1 for p in PROBES if p.kind == KIND_STATIC)))
    documented = [p for p in PROBES if p.documented_as]
    print("  %-12s %d probes carry a documented_as claim to audit"
          % ("doc-audit", len(documented)))
    if problems:
        print("\nREGISTRY PROBLEMS:")
        for e in problems:
            print("  - " + e)
        _sys.exit(1)
    print("\nregistry OK")
