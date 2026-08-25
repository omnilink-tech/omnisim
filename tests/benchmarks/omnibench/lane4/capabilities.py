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
    claim="GranularGroup — bulk particulate media (sand/gravel)",
    # src/omnisim/nodes/OmGranularGroup.cpp exists; whether the node reaches
    # the Newton solver is exactly what is unmeasured. Declared minimally and
    # dropped onto the floor: if the particles are simulated at all their
    # centroid must fall and then STOP on the floor.
    # ⚠ THESE FIELD NAMES ARE THE SCHEMA'S, AND THEY WERE WRONG UNTIL 2026-08-15.
    # This probe declared `particleCount` / `particleRadius`; GranularGroup.wrl
    # has `count` / `radius`. An undeclared field is a "Skipped unknown field"
    # ERROR, which takes a headless run's exit code to 1 -- so this world was
    # failing on its own authoring, not on the capability, and the node-exists
    # assertion below could never have been reached honestly.
    world=lambda: floor() + """DEF SUBJECT_GRAINS GranularGroup {
  translation 0 0 1.2
  count 64
  radius 0.02
}
""" + body("SUBJECT", "Sphere { radius 0.1 subdivision 3 }", "0 0 1.2"),
    measure=("pos:SUBJECT", "node_exists:SUBJECT_GRAINS"),
    duration=3.0,
    assertion=None,   # set below
    doc="src/omnisim/nodes/OmGranularGroup.cpp",
))


def _granular_assertion(arrays):
    """The GranularGroup node must both EXIST in the scene tree after load and
    leave the co-dropped reference sphere's rest height unchanged (0.65 m).
    A node that parses but never reaches the solver is `broken`, not `works`:
    the world author gets a scene that looks right and simulates nothing."""
    exists = arrays.get("node_exists_SUBJECT_GRAINS")
    z = _final(arrays, "pos_SUBJECT", 2)
    ev = {"granular_node_in_scene_tree": bool(exists),
          "reference_sphere_rest_z_m": z}
    if not exists:
        return Verdict(
            ABSENT, ev,
            "GranularGroup did not survive into the scene tree — the node is "
            "not usable from a .wbt even though the C++ class exists")
    if z is None:
        return Verdict(INCONCLUSIVE, ev, "reference sphere not recorded")
    # The node exists. We cannot read particle state from the supervisor API,
    # so this deliberately claims only what it measured.
    return Verdict(
        DEGRADED, ev,
        "GranularGroup parses and appears in the scene tree, but this lane "
        "has NO way to read particle state through the supervisor API, so "
        "'the particles are simulated' is UNMEASURED. Not counted as working.")


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
    measure=("pos:SUBJECT", "node_exists:SHEET"),
    log_capture=("Cloth '",),
    duration=3.0,
    assertion=None,   # set below
    doc="resources/nodes/Cloth.wrl (field contract) + "
        "docs/developer/cloth-simulation.md (the solver requirement is "
        "section 0)",
    documented_as=DEGRADED,
))


#: (dimX + 1) * (dimY + 1) for the patch declared above -- particles, not cells.
#: Asserting the exact count is what makes the engine's registration line
#: evidence rather than a reassuring sentence: a wrong count means the node
#: reached the solver carrying geometry the world did not author.
CLOTH_PARTICLES = 21 * 21


def _cloth_assertion(arrays):
    """A 20x20-cell `Cloth` must register exactly 441 particles with newton's
    SolverVBD (the count its authored dimX/dimY imply), survive into the scene
    tree, and leave a reference sphere dropped beside it resting at the
    analytic rigid rest height of 0.65 m. Registering ZERO particles is the
    documented inert case -- the patch renders at its rest pose and never
    moves -- and is scored `broken`, not `absent`."""
    exists = arrays.get("node_exists_SHEET")
    z = _final(arrays, "pos_SUBJECT", 2)
    lines = arrays.get("engine_log") or []
    registered, inert = None, False
    for ln in lines:
        m = re.search(r"registered (\d+) particles", ln)
        if m:
            registered = int(m.group(1))
        if "registered no particles" in ln:
            inert = True
    ev = {"cloth_node_in_scene_tree": bool(exists),
          "registered_particles": registered,
          "expected_particles": CLOTH_PARTICLES,
          "reference_sphere_rest_z_m": z,
          "engine_reported_inert": inert}
    if not exists:
        return Verdict(
            ABSENT, ev,
            "Cloth did not survive into the scene tree — the node is not "
            "usable from a .wbt even though the C++ class exists")
    if inert or registered == 0:
        return Verdict(
            BROKEN, ev,
            "the Cloth parsed and appears in the scene tree but registered NO "
            "particles, so it renders at its rest pose and never moves")
    if registered is None:
        return Verdict(
            INCONCLUSIVE, ev,
            "the engine logged no Cloth registration line either way, so this "
            "run cannot say whether the node reached a solver")
    if registered != CLOTH_PARTICLES:
        return Verdict(
            DEGRADED, ev,
            "the Cloth registered %d particles, not the %d its authored "
            "dimX/dimY imply — the node reached the solver carrying geometry "
            "the world did not author" % (registered, CLOTH_PARTICLES))
    if z is None:
        return Verdict(INCONCLUSIVE, ev, "reference sphere not recorded")
    if abs(z - 0.65) > 0.02:
        return Verdict(
            BROKEN, ev,
            "the reference sphere rests at z=%.4f m instead of 0.6500 — moving "
            "to the coupled VBD solver perturbed the rigid scene" % z)
    # The node reached the solver with the geometry the world authored, and the
    # coupled solver left the rigid half of the scene analytically correct.
    # STOPPING HERE IS DELIBERATE. Particle positions have no supervisor
    # accessor (`GET /scene/tree?bounds=1` reports `bounds: null` for a Cloth --
    # Cloth.wrl "FRAMING"), so this lane cannot see the sheet fall, drape or
    # self-collide, and "the fabric simulates correctly" is NOT a claim these
    # arrays support. `works` would be an overclaim on an engine self-report.
    #
    # It IS measured, elsewhere and against negative controls:
    # docs/developer/cloth-simulation.md records a gripper holding a flat patch
    # to -0.92 mm and a T-shirt to -1.50 mm of tracking error, each with a
    # jaws-never-close control at -249.67 / -173.06 mm. Read that before
    # quoting this row as the state of cloth.
    return Verdict(
        DEGRADED, ev,
        "Cloth reaches newton's SolverVBD with the authored particle count "
        "(%d) and does not perturb the rigid scene, but particle state has NO "
        "supervisor accessor, so the drape itself is UNMEASURED IN THIS LANE. "
        "Not counted as working here; measured against negative controls in "
        "docs/developer/cloth-simulation.md." % registered)


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
    measure=("pos:SUBJECT", "node_exists:SUBJECT_BLOB"),
    duration=3.0,
    assertion=None,   # set below
    absent_markers=("SoftBody",),
    doc="src/omnisim/nodes/OmSoftBody.cpp",
    documented_as=DEGRADED,
))


def _soft_body_assertion(arrays):
    """A 4x4x4-cell `SoftBody` must register particles with newton's SolverVBD,
    survive into the scene tree, and leave the co-dropped reference sphere's
    rest height unchanged (0.65 m). A node that parses but registers ZERO
    particles is `broken`, not `works`: the world author gets a scene that looks
    right and simulates nothing."""
    exists = arrays.get("node_exists_SUBJECT_BLOB")
    z = _final(arrays, "pos_SUBJECT", 2)
    registered, inert = None, False
    for ln in (arrays.get("engine_log") or []):
        m = re.search(r"registered (\d+) particles", ln)
        if m:
            registered = int(m.group(1))
        if "registered no particles" in ln:
            inert = True
    ev = {"soft_body_node_in_scene_tree": bool(exists),
          "registered_particles": registered,
          "engine_reported_inert": inert,
          "reference_sphere_rest_z_m": z}
    if not exists:
        return Verdict(
            ABSENT, ev,
            "SoftBody did not survive into the scene tree — the node is not "
            "usable from a .wbt even though the C++ class exists")
    if inert or registered == 0:
        return Verdict(
            BROKEN, ev,
            "the SoftBody parsed and appears in the scene tree but registered "
            "NO particles, so it neither moves nor renders")
    if z is None:
        return Verdict(INCONCLUSIVE, ev, "reference sphere not recorded")
    if abs(z - 0.65) > 0.02:
        return Verdict(
            BROKEN, ev,
            "the reference sphere rests at z=%.4f m instead of 0.6500 — declaring a "
            "SoftBody perturbed the rigid scene around it" % z)
    # The node exists and does not disturb the rigid world. Particle state is not
    # readable through the supervisor API, so this claims nothing further.
    #
    # ⚠ A force-transmission probe WAS attempted and withdrawn, which is worth
    # recording so it is not re-attempted blind. Landing a 27 kg soft block on a
    # 2 kg rigid box gives a real signal — measured through the runtime, the box
    # sits 1.0 mm lower (0.648893 loaded vs 0.649892 alone, against 0.108 mm of
    # unloaded penetration). But staged in THIS lane's world the box was driven
    # clean through the floor to z = -0.32, which the assertion's own floor guard
    # caught. Unattributed: the identical masses and geometry are stable when
    # driven directly through World, so it is the staging, not the capability.
    # A decisive probe needs the deformable readback surface, not a cleverer rig.
    return Verdict(
        DEGRADED, ev,
        "SoftBody reaches newton's SolverVBD (%s particles registered), appears in "
        "the scene tree and does not perturb the rigid scene, but this lane has NO "
        "way to read particle state through the supervisor API, so 'the tets are "
        "simulated' is UNMEASURED. Not counted as working. Measured elsewhere: "
        "docs/developer/newton-capability-frontier.md." % registered)


PROBES[-1].assertion = _soft_body_assertion


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
    # STAYS `broken`, and that is this probe's whole point. 2094660ef flipped
    # OMNISIM_NEWTON_BALL_HINGE2 on for BallJoint AND Hinge2Joint and AGENTS.md
    # was rewritten to say both actuate -- but the commit's own evidence is the
    # hinge2 arm of tests/test_newton_ball_hinge2.py, and that file's BALL arm
    # is PASSIVE (PositionSensor only, a gravity pendulum). No test anywhere
    # drives a motorised BallJoint. Measured here with the gate on and the
    # velocity-wheel confound removed, it does not move. AGENTS.md was corrected
    # in the same change that set this back to BROKEN.
    doc="AGENTS.md — motorised BallJoint does NOT actuate (measured 2026-08-17)",
    documented_as=BROKEN,
))


def _ball_assertion(arrays):
    """A motorised BallJoint commanded to 0.8 rad in a GRAVITY-FREE world must
    move the arm by at least 5 cm. Gravity is off so the motor is the only
    thing that can move it. The probe reads the ARM POSE, not the joint angle,
    because the documented failure is that the motors are accepted and
    silently ignored AND their position sensors read 0 — a joint-angle-only
    test cannot tell a working joint with a dead sensor from a dead joint."""
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
            "the BallJoint's motors were accepted but the arm never moved "
            "(%.2e m) with OMNISIM_NEWTON_BALL_HINGE2 ON (its default since "
            "2094660ef). That commit flipped the gate for BOTH joint types, "
            "and it lands for Hinge2Joint only: the same repair that took "
            "joint.hinge2_motor to `works` (declare the motors' min/maxPosition "
            "so they are servos, not velocity wheels) leaves this probe "
            "BIT-IDENTICAL at 2.67e-07 m. The angle READBACK meanwhile travels "
            "%.3f rad, so the sensor is live while the body is not -- the "
            "engine's own warning says the BALL element is emitted with its "
            "per-axis limits unmapped." % (disp, _span(arrays, "joint_JP")))
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

_p(Probe(
    id="device.connector_weld",
    family=FAM_DEVICE,
    claim="Connector — runtime rigid attachment (weld) between bodies",
    world=lambda: floor(),
    prober_children="""    Connector {
      name "conn"
      model "omnibench"
      type "active"
      autoLock FALSE
      distanceTolerance 0.5
      axisTolerance 3.15
      rotationTolerance 3.15
      numberOfRotations 0
      children [
        DEF PROBE_SHAPE Shape {
          appearance PBRAppearance { baseColor 0.3 0.8 0.5 roughness 1 metalness 0 }
          geometry Box { size 0.1 0.1 0.1 }
        }
      ]
      boundingObject USE PROBE_SHAPE
      physics Physics { density -1 mass 0.5 }
    }
""",
    measure=("device_exists:conn", "pos:OMNIBENCH_PROBER"),
    act=("connector_lock:conn:0.5",),
    duration=2.0,
    assertion=None,
    doc="AGENTS.md — welds (Connector / VacuumGripper) went native on Newton",
))


def _connector_assertion(arrays):
    """The Connector device must be present and accept lock(). This probe
    deliberately claims ONLY device presence + a lock call that does not
    error: a two-body weld needs a second Connector authored in a mating
    pose, which this lane does not build, so 'the weld holds' is UNMEASURED
    here and the verdict is capped at `degraded`."""
    ok = arrays.get("device_exists_conn")
    ev = {"device_present": bool(ok),
          "scope": "presence + lock() accepted; weld HOLDING is not measured "
                   "by this probe"}
    if not ok:
        return Verdict(ABSENT, ev, "no Connector device was found on the robot")
    return Verdict(DEGRADED, ev,
                   "Connector is present and lock() was accepted, but whether "
                   "the weld actually constrains two bodies is UNMEASURED")


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
    documented_as=BROKEN,
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
    claim="A node deleted at runtime stops colliding",
    # AGENTS.md documents this as MEASURED AND UNFIXED: a deleted floor still
    # holds a body up. Pinned here so the defect has a benchmark row and a
    # future fix has something to flip.
    world=lambda: floor() + body("SUBJECT", "Box { size 0.2 0.2 0.2 }",
                                 "0 0 1.0"),
    measure=("pos:SUBJECT",),
    act=("delete_node:FLOOR:1.5",),
    duration=4.0,
    assertion=None,
    doc="AGENTS.md — 'a deleted wall still stops a robot and a deleted floor "
        "still holds a body up, silently'",
    documented_as=BROKEN,
))


def _deletion_assertion(arrays):
    """A box resting on a floor at z=0.65 m must resume falling once that
    floor is removed with supervisor remove() at t=1.5 s, and be below
    z=0.4 m by t=4 s (2.5 s of free fall is 30 m). Staying at 0.65 means the
    deleted node's geometry is still in the solver's model."""
    p = arrays.get("pos_SUBJECT")
    if p is None or len(p) < 2:
        return Verdict(INCONCLUSIVE, note="no trajectory recorded")
    z_end = float(p[-1][2])
    z_mid = float(p[len(p) // 2][2])
    ev = {"z_before_delete_m": z_mid, "z_at_end_m": z_end,
          "delete_at_s": 1.5}
    if z_end < 0.4:
        return Verdict(WORKS, ev)
    if abs(z_end - 0.65) <= 0.02:
        return Verdict(BROKEN, ev,
                       "the box stayed at z=%.4f m for 2.5 s after its floor "
                       "was deleted — the removed node still collides"
                       % z_end)
    return Verdict(DEGRADED, ev,
                   "the box only reached z=%.4f m after its floor was deleted"
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
