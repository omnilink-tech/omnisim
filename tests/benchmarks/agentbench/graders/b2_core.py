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

"""B2 ``subject_in_frame`` -- the **sim-neutral core** of the inspect tier.

The prompt is *"Point the camera at the red cylinder and prove to me it is in
frame."* Two deliverables, and the grader measures both itself:

  1. **the camera actually points at the subject** -- graded as solid geometry,
     in degrees and metres, from the final camera pose and the subject's own
     world-space bounds. Nothing here reads a pixel, a screenshot, a render
     statistic or any one simulator's response shape (SPEC 6.2.6);
  2. **the proof is checkable and true** -- the agent's committed claim is
     compared against the geometry the core derived. SPEC 8.1.2: ground truth,
     never narration. An answer that *says* the subject is centred while the
     camera is 40 degrees off it fails, and so does an answer that fabricates a
     plausible off-axis angle over a camera that happens to be aimed correctly.

**Why B2 needs a scrupulously neutral core.** B2 is the admitted seeding of the
decision set with our surface's best case: a one-call subject-relative framing
verb is the most differentiated thing we own, so if the surface pays anywhere it
pays here (validation plan 2.1). A grader that quietly encoded that verb's own
notion of "in frame" -- its screen-space bounding box, its ``fits`` flag, its
margin convention -- would be grading our API against itself. So every threshold
below is an angle or a metre, computed from a camera position, a unit forward
axis, a field of view, and an axis-aligned bounding box: the five quantities
every simulator with a camera can state, in the units they are stated in.

The geometry, stated once
-------------------------

Let ``f`` be the camera's unit forward axis, ``p`` its position, and ``c`` the
subject's bounds centre. ``d = unit(c - p)`` is the direction to the subject.

* **Clearance.** When the camera also reports an up axis, the core resolves the
  full camera basis and computes the yaw/pitch of ``d`` in it, then
  ``clearance = min(half_fov_h - |yaw|, half_fov_v - |pitch|)`` -- the angular
  distance from the subject centre to the nearest frame edge, positive inside.
  When no up axis is available the core falls back to the cone inscribed in the
  frustum: ``clearance = min(half_fov_h, half_fov_v) - angle(f, d)``. The
  fallback is **conservative**: everything it accepts is genuinely in frame on
  any renderer, but a subject parked in a frame *corner* is not credited. Which
  of the two ran is recorded in every verdict, because the two are not
  like-for-like and a cross-simulator reader must be able to see that.
* **Angular size.** ``2 * asin(r / dist)`` over the subject's bounding-sphere
  radius ``r``. This is the "not a speck at a kilometre" floor: at the default
  45 degree horizontal field of view, the 2.0 degree floor is about 57 pixels
  across a 1280-wide render, which is a thing a human can see and point at.
* **Discrimination.** The same off-axis angle is computed for every distractor
  the task names. Aiming at a distractor is the obvious wrong answer, and the
  only way to catch it in physical units is to check that the subject is the
  best-centred candidate.

Two ambiguities are resolved in the agent's favour, on purpose, and both are
recorded rather than buried:

* two bodies on the same ray from the camera are geometrically
  indistinguishable, so the discrimination clause allows the subject to be
  behind a distractor by up to ``TIE_DEG``. Geometry cannot tell those apart and
  the grader must not invent a distinction it cannot measure;
* "off-axis by X degrees" legitimately means the total off-axis angle, the yaw,
  or the pitch. The stated angle is scored against whichever it is closest to.

The camera evidence types
-------------------------

``CameraPose`` / ``ViewEvidence`` / ``check_view`` were declared here first so
B2 could be written and tested before the neutral bundle grew a camera; at
integration they were lifted **verbatim** into ``graders/evidence.py`` (where
``EvidenceBundle`` now carries an optional ``view`` field) and this module
re-imports them, so existing imports keep working and there is exactly one
definition.
"""

from __future__ import annotations

import math
import re

from agentbench.graders import physical as ph
from agentbench.graders.evidence import (  # noqa: F401  (re-exported: the
    EMPTY_VIEW, CameraPose, ViewEvidence,  # camera types were born here and
    check_view)                            # lifted into evidence.py verbatim)
from agentbench.graders.verdict import (ARTIFACT_INVALID, ARTIFACT_RUNS,
                                        CORE_PHYSICAL, CORE_STRUCTURAL,
                                        GRADED_PASS, INVALID, Falsifier,
                                        Verdict)

TASK = "B2_subject_in_frame"

# --- task data (the world the task ships) -----------------------------------

SUBJECT_NAME = "red_cylinder"
DISTRACTOR_NAMES = ("blue_cylinder", "green_cylinder", "yellow_crate",
                    "grey_sphere")

# --- thresholds (mirrored in the task's meta.json constants block) ----------

#: Angular distance the subject centre must keep from the nearest frame edge.
#: Not zero, because a subject exactly on the edge is half out of shot and
#: because every simulator's viewport aspect differs by a degree or two.
MARGIN_DEG = 2.0
#: Angular diameter floor -- "in frame" must not mean "a speck on the horizon".
MIN_ANGULAR_SIZE_DEG = 2.0
#: How far behind a distractor the subject may sit and still count as the aim
#: point. Two bodies on one ray are geometrically indistinguishable.
TIE_DEG = 0.10
#: What counts as having moved the camera at all (either clause suffices).
MIN_AIM_CHANGE_DEG = 1.0
MIN_MOVE_M = 0.05
#: Tolerances on the numbers the answer commits to.
ANGLE_TOL_DEG = 2.0
DISTANCE_TOL_M = 1.0
#: How far from a cue word a number may sit and still be read as that cue's
#: number. Same value and same reasoning as the sibling inspect-tier core.
CUE_WINDOW_CHARS = 40


# --- vector helpers (SO(3) and nothing else) --------------------------------


def _vec(v):
    if v is None:
        return None
    try:
        out = tuple(float(x) for x in v)
    except (TypeError, ValueError):
        return None
    return out if len(out) == 3 else None


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


def _unit(v):
    v = _vec(v)
    if v is None:
        return None
    n = _norm(v)
    return None if n < 1e-12 else tuple(x / n for x in v)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def angle_between_deg(a, b):
    """Angle between two direction vectors, degrees in [0, 180]."""
    ua, ub = _unit(a), _unit(b)
    if ua is None or ub is None:
        return None
    return math.degrees(math.acos(max(-1.0, min(1.0, _dot(ua, ub)))))


def camera_basis(forward, up):
    """``(forward, left, up)`` orthonormal world axes, or ``None``.

    ``left = unit(up x forward)`` and the returned up is re-orthogonalised
    against forward, so a caller may hand in any up hint that is not parallel
    to the view direction. Returns ``None`` when it is (a straight-down view
    with a vertical up hint), which drops the frustum test back to the
    inscribed cone rather than inventing a roll.
    """
    f, u = _unit(forward), _unit(up)
    if f is None or u is None:
        return None
    if 1.0 - abs(_dot(f, u)) < 1e-9:
        return None
    left = _unit(_cross(u, f))
    if left is None:
        return None
    return f, left, _cross(f, left)


def bounding_sphere(body):
    """``(centre_xyz, radius_m)`` from a body's world-space bounds, or None.

    The circumscribed sphere of the axis-aligned box. Deliberately the *outer*
    sphere: it over-states angular size slightly, which is the forgiving
    direction for a floor whose job is to reject a speck.
    """
    if body is None or not getattr(body, "has_aabb", False):
        return None
    lo, hi = _vec(body.aabb_min), _vec(body.aabb_max)
    if lo is None or hi is None:
        return None
    centre = tuple((a + b) / 2.0 for a, b in zip(lo, hi))
    radius = 0.5 * _norm(_sub(hi, lo))
    return centre, radius


def angular_size_deg(radius_m, distance_m):
    """Angular diameter of a sphere of ``radius_m`` seen from ``distance_m``."""
    if radius_m is None or distance_m is None:
        return None
    if distance_m <= radius_m:
        return 180.0
    return math.degrees(2.0 * math.asin(radius_m / distance_m))


def frame_geometry(cam, target_centre):
    """Where ``target_centre`` sits in ``cam``'s frame, in degrees and metres.

    Returns ``{distance_m, offaxis_deg, yaw_deg, pitch_deg, clearance_deg,
    in_front, method, half_fov_h_deg, half_fov_v_deg, fov_note}`` or ``None``
    when the pose is not usable. ``yaw_deg`` / ``pitch_deg`` are ``None`` under
    the inscribed-cone fallback.
    """
    if cam is None or not cam.usable:
        return None
    p, f = _vec(cam.position), _unit(cam.forward)
    c = _vec(target_centre)
    if c is None:
        return None
    delta = _sub(c, p)
    dist = _norm(delta)
    if dist < 1e-9:
        return None
    d = tuple(x / dist for x in delta)
    half_h, half_v, fov_note = cam.half_angles()
    offaxis = math.degrees(math.acos(max(-1.0, min(1.0, _dot(f, d)))))
    basis = camera_basis(f, cam.up) if cam.up is not None else None
    if basis is not None:
        fwd, left, up = basis
        xf, xl, xu = _dot(d, fwd), _dot(d, left), _dot(d, up)
        yaw = math.degrees(math.atan2(xl, xf))
        pitch = math.degrees(math.atan2(xu, math.hypot(xl, xf)))
        clearance = min(math.degrees(half_h) - abs(yaw),
                        math.degrees(half_v) - abs(pitch))
        method = "per-axis frustum test (an up axis was reported)"
    else:
        yaw = pitch = None
        clearance = min(math.degrees(half_h), math.degrees(half_v)) - offaxis
        method = ("cone inscribed in the frustum (no up axis reported, so "
                  "frame corners are not credited)")
    return {"distance_m": dist, "offaxis_deg": offaxis, "yaw_deg": yaw,
            "pitch_deg": pitch, "clearance_deg": clearance,
            "in_front": _dot(f, d) > 0.0, "method": method,
            "half_fov_h_deg": math.degrees(half_h),
            "half_fov_v_deg": math.degrees(half_v), "fov_note": fov_note}


# --- ground truth -----------------------------------------------------------


def measure_ground_truth(bundle, view):
    """Everything the assertions compare against, in degrees and metres.

    ``None`` when the subject cannot be located with bounds -- which is a
    broken measurement, not a failed agent, and the caller turns it into
    ``INVALID``.
    """
    subject = bundle.t0.by_name(SUBJECT_NAME)
    sphere = bounding_sphere(subject)
    if sphere is None:
        return None
    centre, radius = sphere
    geom = frame_geometry(view.final, centre)
    if geom is None:
        return None
    out = dict(geom)
    out["subject"] = SUBJECT_NAME
    out["subject_centre_m"] = centre
    out["subject_radius_m"] = radius
    out["angular_size_deg"] = angular_size_deg(radius, geom["distance_m"])
    others = {}
    for name in DISTRACTOR_NAMES:
        other = bundle.t0.by_name(name)
        s = bounding_sphere(other)
        if s is None:
            continue
        g = frame_geometry(view.final, s[0])
        if g is not None:
            others[name] = g["offaxis_deg"]
    out["distractor_offaxis_deg"] = others
    out["aim_change_deg"] = angle_between_deg(
        view.initial.forward, view.final.forward) if view.initial else None
    if view.initial and _vec(view.initial.position):
        out["camera_move_m"] = _norm(_sub(_vec(view.final.position),
                                          _vec(view.initial.position)))
    else:
        out["camera_move_m"] = None
    return out


# --- reading the agent's committed proof ------------------------------------

# The conclusion the answer commits to. Both polarities are matched and the one
# whose match ENDS LAST wins, so "it was out of frame; it is now in frame"
# reads as the affirmative it plainly is.
_CLAIM_POS = re.compile(
    r"in\s+(?:the\s+)?(?:frame|view|shot)\b|in-frame|on[\s-]screen|"
    r"\bvisible\b|\bframed\b|cent(?:re|er)ed\b|cent(?:re|er)d\b|"
    r"fills?\s+the\s+frame", re.IGNORECASE)
_CLAIM_NEG = re.compile(
    r"out\s+of\s+(?:the\s+)?(?:frame|view|shot)\b|off[\s-]screen|"
    r"(?:not|isn'?t|is\s+not|are\s+not|aren'?t|never|no\s+longer|cannot|"
    r"can'?t|could\s*n[o']t|fail(?:s|ed)?\s+to)\s+(?:\w+\s+){0,3}?"
    r"(?:in\s+(?:the\s+)?(?:frame|view|shot)|visible|on[\s-]screen|framed|"
    r"see\b|cent(?:re|er)ed)", re.IGNORECASE)

# Words that mark an angle as THE off-axis angle the proof commits to.
_OFFSET_CUE = re.compile(
    r"off[\s-]?axis|offset|off[\s-]cent(?:re|er)|from\s+(?:the\s+)?"
    r"(?:frame\s+)?cent(?:re|er)|angular\s+(?:offset|error|deviation)|"
    r"boresight|away\s+from\s+cent(?:re|er)", re.IGNORECASE)
# ...and a length as THE camera-to-subject range.
_RANGE_CUE = re.compile(
    r"distance|range|\baway\b|from\s+the\s+camera|camera\s+(?:is|sits|stands)|"
    r"stand[\s-]?off|\bback\b", re.IGNORECASE)

_ANGLE = re.compile(
    r"([-+]?\d+(?:\.\d+)?)\s*(?:°|deg\b|degs\b|degrees?\b|"
    r"rad\b|radians?\b)", re.IGNORECASE)
_RADIAN = re.compile(r"rad", re.IGNORECASE)


def angle_spans(text):
    """``[(degrees, start, end)]`` for every angle in the answer, in order.

    Radians are converted. Extraction only -- the number still has to match the
    measured geometry.
    """
    out = []
    for m in _ANGLE.finditer(text or ""):
        value = float(m.group(1))
        if _RADIAN.search(m.group(0)):
            value = math.degrees(value)
        out.append((value, m.start(), m.end()))
    return out


def _gap(a, b):
    """Characters between two ``(start, end)`` spans; 0 when they overlap."""
    (a0, a1), (b0, b1) = a, b
    if a1 <= b0:
        return b0 - a1
    if b1 <= a0:
        return a0 - b1
    return 0


def _nearest_to_cue(spans, cue, text):
    """``(value, how)`` for the span closest to a cue, or ``(None, why)``.

    The same commit-to-one-number discipline the sibling inspect core uses: an
    answer that sprays plausible numbers must not be scored on its luckiest
    one, so exactly one is picked and the rest are recorded unscored.
    """
    if not spans:
        return None, "the answer states no such quantity"
    cues = [m.span() for m in cue.finditer(text or "")]
    if cues:
        gap, _i, value = min(
            (min(_gap((s, e), c) for c in cues), i, v)
            for i, (v, s, e) in enumerate(spans))
        if gap <= CUE_WINDOW_CHARS:
            return value, ("the value closest to a cue word (%d characters "
                           "away)" % gap)
    return None, ("the answer states a value but never near a cue word, so it "
                  "commits to nothing")


def claim_polarity(answer):
    """``"in_frame"`` / ``"not_in_frame"`` / ``None`` -- what the answer says.

    ``None`` means the answer never states a conclusion about framing at all,
    which is the narration-only failure the task is written to reject.
    """
    text = answer or ""
    pos = [m.end() for m in _CLAIM_POS.finditer(text)]
    neg = [m.end() for m in _CLAIM_NEG.finditer(text)]
    if not pos and not neg:
        return None
    last_pos = max(pos) if pos else -1
    last_neg = max(neg) if neg else -1
    return "in_frame" if last_pos > last_neg else "not_in_frame"


def committed_proof(answer):
    """What the answer commits to, as numbers: polarity, angle, range."""
    text = answer or ""
    angle, angle_how = _nearest_to_cue(angle_spans(text), _OFFSET_CUE, text)
    rng, rng_how = _nearest_to_cue(ph.distance_spans(text), _RANGE_CUE, text)
    return {"polarity": claim_polarity(text),
            "offaxis_deg": angle, "offaxis_how": angle_how,
            "range_m": rng, "range_how": rng_how,
            "all_angles_deg": [v for v, _s, _e in angle_spans(text)][:12],
            "all_lengths_m": ph.distances_in(text)[:12],
            "all_numbers": ph.numbers_in(text)[:12]}


# --- the verdict ------------------------------------------------------------


def grade(bundle, *, view=None, answer="", self_verified=False):
    v = Verdict(TASK, self_verified=self_verified)
    v.note("every threshold here is an angle or a metre computed from the "
           "camera pose and the subject's own world-space bounds; no pixel, "
           "screenshot or render statistic is read (SPEC 6.2.6)")
    for note in bundle.notes:
        v.note(note)

    if bundle.t0.error:
        v.outcome = INVALID
        v.note("ground truth unavailable: %s" % bundle.t0.error)
        return v

    # Explicit argument first, then the bundle's own camera channel (the
    # ``view`` field evidence.py grew at integration), then the honest
    # absence marker.
    if view is None:
        view = bundle.view if bundle.view is not None else EMPTY_VIEW
    v.measurements["camera_evidence"] = {
        "source": view.source, "error": view.error,
        "artifact_parsed": view.artifact_parsed,
        "problems": check_view(view)}
    v.measurements["answer"] = answer

    # An agent that broke the scene it was editing. This is an AGENT failure,
    # not a broken stack, so it is a red B2.1 rather than INVALID -- and it is
    # checked before the evidence gates below, which would otherwise swallow it
    # as "no camera pose".
    if view.artifact_parsed is False:
        v.progress = ARTIFACT_INVALID
        v.add("B2.1", False, "the camera was aimed somewhere new",
              measured={"scene readable": False},
              threshold={"scene readable": True},
              detail="the edited scene could not be read back",
              basis=CORE_STRUCTURAL,
              falsifiers=[Falsifier(
                  "the scene is readable",
                  "an edit that leaves the scene unreadable",
                  "a read-back was attempted", True, detail=view.source)])
        return v.finish(GRADED_PASS)

    # Missing camera or missing subject bounds is a broken MEASUREMENT. It is
    # never a PASS and never a FAIL: with no pose there is nothing to compare,
    # and scoring the agent on our own missing instrument would be grading the
    # stack (SPEC 3.3 INVALID, validation plan 5.5).
    if view.final is None or not view.final.usable:
        v.outcome = INVALID
        v.note("no usable final camera pose: %s"
               % ("; ".join(check_view(view)) or "absent"))
        return v
    if view.initial is None or not view.initial.usable:
        v.outcome = INVALID
        v.note("no usable initial camera pose, so the do-nothing agent cannot "
               "be told apart from a working one -- the run is unattributable "
               "rather than passed")
        return v

    truth = measure_ground_truth(bundle, view)
    if truth is None:
        v.outcome = INVALID
        v.note("ground truth unavailable: the subject %r was not found with "
               "world-space bounds at a frozen t=0" % SUBJECT_NAME)
        return v

    v.progress = ARTIFACT_RUNS
    v.measurements["truth"] = _rounded(truth)
    proof = committed_proof(answer)
    v.measurements["committed_proof"] = proof

    # --- B2.1 the camera moved at all --------------------------------------
    aim = truth["aim_change_deg"]
    moved = truth["camera_move_m"]
    changed = bool((aim is not None and aim >= MIN_AIM_CHANGE_DEG)
                   or (moved is not None and moved >= MIN_MOVE_M))
    v.add("B2.1", changed, "the camera was aimed somewhere new",
          measured={"aim change (deg)": _r(aim),
                    "camera moved (m)": _r(moved),
                    "scene readable": view.artifact_parsed},
          threshold={"aim change (deg)": ">= %.2f" % MIN_AIM_CHANGE_DEG,
                     "or camera moved (m)": ">= %.2f" % MIN_MOVE_M},
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "the camera changed",
              "a camera left exactly where the task shipped it",
              "both the initial and the final camera pose were recovered",
              aim is not None or moved is not None,
              detail="initial from %s; final from %s"
                     % (view.initial.source or "?", view.final.source or "?"))])

    # --- B2.2 the subject is inside the frustum, with margin ----------------
    clearance = truth["clearance_deg"]
    v.add("B2.2", bool(clearance is not None and clearance >= MARGIN_DEG),
          "the subject sits inside the frame with angular margin",
          measured={"clearance to nearest frame edge (deg)": _r(clearance),
                    "off-axis (deg)": _r(truth["offaxis_deg"]),
                    "yaw (deg)": _r(truth["yaw_deg"]),
                    "pitch (deg)": _r(truth["pitch_deg"]),
                    "half field of view h/v (deg)":
                        [_r(truth["half_fov_h_deg"]),
                         _r(truth["half_fov_v_deg"])],
                    "in front of the camera": truth["in_front"],
                    "range (m)": _r(truth["distance_m"]),
                    "method": truth["method"]},
          threshold={"clearance (deg)": ">= %.2f" % MARGIN_DEG},
          detail=truth["fov_note"], basis=CORE_PHYSICAL,
          falsifiers=[
              Falsifier("the subject is in frame",
                        "a camera whose forward axis is further from the "
                        "subject centre than its own half field of view",
                        "a measured camera pose and a measured subject centre",
                        True, detail=view.final.source),
              Falsifier("the field of view is known",
                        "a camera with a field of view so wide that nothing "
                        "is ever out of frame",
                        "the camera reported at least one field-of-view angle",
                        view.final.fov_h_rad is not None
                        or view.final.fov_v_rad is not None,
                        detail=truth["fov_note"])])

    # --- B2.3 it is not a speck --------------------------------------------
    size = truth["angular_size_deg"]
    v.add("B2.3", bool(size is not None and size >= MIN_ANGULAR_SIZE_DEG),
          "the subject is big enough on screen to be the subject",
          measured={"angular diameter (deg)": _r(size),
                    "range (m)": _r(truth["distance_m"]),
                    "bounding radius (m)": _r(truth["subject_radius_m"])},
          threshold={"angular diameter (deg)":
                     ">= %.2f" % MIN_ANGULAR_SIZE_DEG},
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "the subject is large enough",
              "a camera aimed correctly from so far away that the subject is "
              "a speck",
              "the subject has measured world-space bounds, so a radius and "
              "a range exist", truth["subject_radius_m"] is not None,
              detail=bundle.t0.source)])

    # --- B2.4 it is the SUBJECT, not a distractor --------------------------
    others = truth["distractor_offaxis_deg"]
    best_name = min(others, key=others.get) if others else None
    best = others[best_name] if best_name else None
    v.add("B2.4",
          bool(best is None or truth["offaxis_deg"] <= best + TIE_DEG),
          "the subject is the best-centred body in the scene",
          measured={"subject off-axis (deg)": _r(truth["offaxis_deg"]),
                    "closest distractor": best_name,
                    "its off-axis (deg)": _r(best),
                    "every distractor (deg)":
                        {k: _r(x) for k, x in sorted(others.items())}},
          threshold={"subject off-axis (deg)":
                     "<= the best distractor's + %.2f" % TIE_DEG},
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "the subject beats every distractor",
              "a camera aimed at one of the other bodies instead",
              "at least one distractor body was found with world-space "
              "bounds", bool(others),
              detail="found %d of %d named distractors"
                     % (len(others), len(DISTRACTOR_NAMES)))])

    # --- B2.5 the proof is committed, not narrated -------------------------
    n_numbers = len(proof["all_numbers"])
    v.add("B2.5",
          bool(proof["polarity"] == "in_frame" and n_numbers >= 1),
          "the answer commits to an in-frame claim backed by a number",
          measured={"claim": proof["polarity"],
                    "numbers in the answer": n_numbers,
                    "angles (deg)": proof["all_angles_deg"],
                    "lengths (m)": proof["all_lengths_m"]},
          threshold={"claim": "in_frame", "numbers": ">= 1"},
          basis=CORE_STRUCTURAL,
          falsifiers=[Falsifier(
              "a claim was made",
              "an answer that describes the work but never states, with a "
              "number, that the subject ended up in frame",
              "the agent's final message was captured", answer is not None)])

    # --- B2.6 ...and the proof is TRUE -------------------------------------
    in_frame = bool(clearance is not None and clearance >= 0.0)
    polarity_ok = bool(proof["polarity"] == "in_frame" and in_frame)
    # "off-axis by X" legitimately reads as the total angle, the yaw or the
    # pitch. Scored against whichever it is nearest -- the ambiguity is the
    # question's, not the answer's.
    readings = [truth["offaxis_deg"]]
    for key in ("yaw_deg", "pitch_deg"):
        if truth[key] is not None:
            readings.append(abs(truth[key]))
    angle_err = (min(abs(proof["offaxis_deg"] - x) for x in readings)
                 if proof["offaxis_deg"] is not None else None)
    range_err = (abs(proof["range_m"] - truth["distance_m"])
                 if proof["range_m"] is not None else None)
    angle_ok = angle_err is None or angle_err <= ANGLE_TOL_DEG
    range_ok = range_err is None or range_err <= DISTANCE_TOL_M
    v.add("B2.6", bool(polarity_ok and angle_ok and range_ok),
          "the committed proof agrees with the measured geometry",
          measured={"claim": proof["polarity"],
                    "measured in frame": in_frame,
                    "stated off-axis (deg)": _r(proof["offaxis_deg"]),
                    "chosen by": proof["offaxis_how"],
                    "measured readings (deg)": [_r(x) for x in readings],
                    "off-axis error (deg)": _r(angle_err),
                    "stated range (m)": _r(proof["range_m"]),
                    "chosen by (range)": proof["range_how"],
                    "measured range (m)": _r(truth["distance_m"]),
                    "range error (m)": _r(range_err),
                    "numbers stated but not scored":
                        [x for x in proof["all_numbers"]
                         if x not in (proof["offaxis_deg"],
                                      proof["range_m"])][:12]},
          threshold={"claim": "matches the measured framing",
                     "off-axis error (deg)": "<= %.2f" % ANGLE_TOL_DEG,
                     "range error (m)": "<= %.2f" % DISTANCE_TOL_M},
          basis=CORE_PHYSICAL,
          falsifiers=[
              # The witness is the CLEARANCE alone. A missing conclusion is
              # not a missing witness -- it is one of the ways this clause
              # fails, and treating it as vacuity would report the narration
              # -only answer as "could not have failed" when it just did.
              Falsifier("the claim matches the geometry",
                        "an answer asserting the subject is in frame while "
                        "the measured clearance is negative, or an answer "
                        "that states no conclusion at all",
                        "a measured clearance from the camera pose",
                        clearance is not None),
              Falsifier("the stated off-axis angle is true",
                        "an answer quoting a small, confident off-axis angle "
                        "it never measured",
                        "the answer committed to an off-axis angle",
                        proof["offaxis_deg"] is not None,
                        detail=proof["offaxis_how"]),
              Falsifier("the stated range is true",
                        "an answer quoting a range that is not the camera's",
                        "the answer committed to a camera-to-subject range",
                        proof["range_m"] is not None,
                        detail=proof["range_how"])])

    return v.finish(GRADED_PASS)


# --- presentation -----------------------------------------------------------


def _r(x, nd=4):
    return None if x is None else round(float(x), nd)


def _rounded(d):
    out = {}
    for k, x in d.items():
        if isinstance(x, float):
            out[k] = _r(x)
        elif isinstance(x, dict):
            out[k] = {kk: _r(xx) if isinstance(xx, float) else xx
                      for kk, xx in x.items()}
        elif isinstance(x, tuple):
            out[k] = [_r(xx) if isinstance(xx, float) else xx for xx in x]
        else:
            out[k] = x
    return out
