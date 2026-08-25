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

"""selftest.py -- prove the ladder can go RED.

A green that has never been shown to go red is worth nothing.  The week before
this ladder was written produced, in this repo, a collision assertion that
could not fail, a readiness check that greened two cells it had never gated,
and a golden that signed a physics-free world off as fixed.  Every one of them
was green every single time it ran.

Two independent proofs, both driven by ``run_ladder.py --self-test``:

**A. Assertion mutation (pure -- no simulator, milliseconds).**  For every
rung, build the measurement an ideal engine would produce, confirm every check
is green, then perturb ONE quantity past its tolerance and confirm that exactly
that check goes red and the others stay green.  This proves each assertion is
wired to a quantity and discriminates on it: a check reading a key nothing ever
writes, or comparing a value to itself, cannot survive it.

Two extra pure cases guard failure modes that have actually bitten here:

* a ``None`` measurement (the arm could not measure it) must be RED, never
  skipped.  "We did not look" reading as "nothing was wrong" is exactly the
  log-only-PASS bug.
* a value exactly ON the tolerance boundary must be green and one just outside
  it red, so the boundary is where the derivation says it is.

**B. Live fault injection (a real engine, end to end).**  A real world or a
real controller is broken in a way that reproduces a defect this repo has
actually shipped, run through the whole path -- launch, load, step, sample,
reduce, judge -- and the named check must come back red while its named
companions stay green.  This proves the plumbing can CARRY a failure, not
merely that a comparison operator works.

The most important case is ``rung 4 / slide``: the chassis is dragged at
exactly the right speed with its wheels commanded to zero.  ``distance`` must
stay GREEN and ``rolling_consistency`` must go RED.  That is the shape of the
bug that let 33 worlds ship with wheels that never turned, and it is the whole
reason rung 4 asserts two things instead of one.
"""

from __future__ import annotations

import os
import sys

HERE = os.path.abspath(os.path.dirname(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import analysis                                  # noqa: E402
import rungs                                     # noqa: E402


# --------------------------------------------------------------------------
# A. the ideal measurement per rung -- built from the ANALYTIC values only
# --------------------------------------------------------------------------

def ideal(rung):
    """What a perfect engine would measure.  Derived, never recorded."""
    rung = int(rung)
    if rung == 0:
        return {"steps": float(rungs.RUNG0_STEPS), "exit_code": 0.0,
                "sim_time_end": rungs.RUNG0_STEPS * rungs.DT}
    if rung == 1:
        return {"rest_z": rungs.REST_Z, "z_drift": 0.0, "penetration": 0.0}
    if rung == 2:
        return {"spawn_z": rungs.RUNG2_SPAWN_Z,
                "fall_interval": rungs.fall_interval_s(
                    rungs.RUNG2_SPAWN_Z, rungs.RUNG2_GATE_HI,
                    rungs.RUNG2_GATE_LO),
                "fall_time_abs": rungs.fall_time_s(rungs.RUNG2_DROP_M),
                "rest_z": rungs.REST_Z}
    if rung == 3:
        win = rungs.RUNG3_WIN_A[1] - rungs.RUNG3_WIN_A[0]
        return {"omega_driven": rungs.RUNG3_OMEGA_CMD, "omega_zero": 0.0,
                "angle_driven": rungs.RUNG3_OMEGA_CMD * win}
    if rung == 4:
        win = rungs.RUNG4_WIN[1] - rungs.RUNG4_WIN[0]
        return {"distance": rungs.rolling_speed(rungs.RUNG4_OMEGA_CMD) * win,
                "wheel_omega": rungs.RUNG4_OMEGA_CMD, "roll_ratio": 1.0,
                "lateral": 0.0, "roll_overrun": 0.0, "ride_dev": 0.0}
    if rung == 5:
        return {"range_static": rungs.RUNG5_STANDOFF,
                "range_final": rungs.RUNG5_FINAL_RANGE,
                "range_residual": 0.0,
                "sweep_span": rungs.RUNG5_TRAVEL}
    if rung == 6:
        return {"stop_gap": rungs.RUNG6_STOP_GAP - rungs.RUNG6_STOP_BOUND / 2.0,
                "min_gap": rungs.RUNG6_STOP_GAP - rungs.RUNG6_STOP_BOUND / 2.0,
                "trigger_reading": (rungs.RUNG6_STOP_GAP
                                    - 0.5 * rungs.RUNG6_CRUISE_V * rungs.DT),
                "range_residual": 0.0, "stop_creep": 0.0, "wheel_stop": 0.0}
    if rung == 7:
        return {"distance_worst": 0.0, "wheel_omega_worst": 0.0,
                "min_separation": rungs.RUNG7_LANE_DY, "lateral_worst": 0.0,
                "roll_overrun_worst": 0.0, "ride_worst": 0.0}
    if rung == 8:
        return {"part_rest_z": rungs.RUNG8_PART_Z0, "carry_rel": 0.0,
                "lift_height": rungs.RUNG8_PART_TARGET_Z,
                "place_x": rungs.RUNG8_TRAVERSE_X,
                "hold_clearance": rungs.RUNG8_LIFT_H,
                "part_speed_max": 0.0}
    if rung == 9:
        return {"repeat_delta": 0.0, "repeat_length": 0.0,
                "sensitivity_shortfall": 0.0,
                "fall_interval": rungs.fall_interval_s(
                    rungs.RUNG9_SPAWN_Z, rungs.RUNG9_GATE_HI,
                    rungs.RUNG9_GATE_LO),
                "distinct_processes": float(len(rungs.RUNG9_RUNS))}
    if rung == 11:
        return {"distance_worst": 0.0, "wheel_omega_worst": 0.0,
                "roll_overrun_worst": 0.0, "ride_worst": 0.0,
                "lateral_worst": 0.0, "separation_shortfall": 0.0,
                "robots_seen": float(rungs.RUNG11_ROBOTS_TOTAL)}
    if rung == 18:
        # An engine that reproduced a tossed cube EXACTLY.  Nothing has ever
        # done this and nothing will -- the best measured engine is 13.5% of
        # cube width out -- but the ideal is what the mutation proof perturbs
        # away from, and it must be the physical ideal rather than anyone's
        # score, or the proof would be anchored to a simulator.
        return {"real_pos_err": 0.0, "real_rot_err": 0.0,
                "tunnel_depth": 0.0, "ic_shortfall": 0.0,
                "tosses_missing": 0.0}
    raise ValueError(rung)


def _break_value(chk, good):
    """A value 10% outside ``chk``'s tolerance."""
    if chk.tol == 0.0:
        return (good or 0.0) + 1.0
    if chk.rel:
        denom = abs(chk.expected) if chk.expected else 1.0
        return chk.expected + denom * chk.tol * 1.1
    return chk.expected + chk.tol * 1.1


# 0.999, not 1.0.  "Exactly at the tolerance" is not a representable request:
# expected + tol differenced back against expected lands one ULP OUTSIDE tol
# for most decimal values (0.6 + 0.005 - 0.6 = 0.005000000000000004), so an
# exact-boundary case tests floating-point rounding rather than the tolerance.
# The pair (99.9% -> green, 110% -> red) brackets the boundary to 0.1% of the
# tolerance, which is the real claim: the check flips where it is documented
# to flip, not decades away.  This was found BY the self-test, on its first
# run, which is the argument for having one.
_EDGE_FRACTION = 0.999


def _edge_value(chk):
    """A value just inside the tolerance boundary (must stay green)."""
    if chk.rel:
        denom = abs(chk.expected) if chk.expected else 1.0
        return chk.expected + denom * chk.tol * _EDGE_FRACTION
    return chk.expected + chk.tol * _EDGE_FRACTION


def mutation_cases():
    """-> list of {rung, target, kind, ok, detail}.

    ``ALL_RUNGS``, not ``RUNGS``: the mutation proof is pure arithmetic and
    costs milliseconds, so there is no reason for the on-demand rungs'
    assertions to go unproved just because their LIVE cells cost minutes.
    """
    out = []
    for rung in rungs.ALL_RUNGS:
        base = ideal(rung)
        checks = rungs.check_rung(rung, base)
        bad = [c for c in checks if not c.ok]
        out.append({
            "rung": rung, "target": "(all)", "kind": "baseline-green",
            "ok": not bad,
            "detail": ("%d checks green on the ideal measurement"
                       % len(checks)) if not bad else
                      ("NOT GREEN on the ideal measurement: %s"
                       % ", ".join(c.name for c in bad))})
        for chk in checks:
            mutated = dict(base)
            mutated[chk.key] = _break_value(chk, base.get(chk.key))
            after = {c.name: c for c in rungs.check_rung(rung, mutated)}
            target = after[chk.name]
            others_ok = all(c.ok for n, c in after.items() if n != chk.name)
            out.append({
                "rung": rung, "target": chk.name, "kind": "mutation",
                "ok": (not target.ok) and others_ok,
                "detail": "%s=%s vs %s +/-%s -> %s%s" % (
                    chk.key, _f(target.measured), _f(target.expected),
                    _f(target.tol),
                    "RED" if not target.ok else "STILL GREEN (bad)",
                    "" if others_ok else "; collateral: another check flipped")
            })
            gone = dict(base)
            gone[chk.key] = None
            g = {c.name: c for c in rungs.check_rung(rung, gone)}[chk.name]
            out.append({
                "rung": rung, "target": chk.name, "kind": "none-is-red",
                "ok": not g.ok,
                "detail": ("unmeasured -> RED" if not g.ok
                           else "unmeasured read as GREEN (bad)")})
            if chk.tol > 0:
                edge = dict(base)
                edge[chk.key] = _edge_value(chk)
                e = {c.name: c
                     for c in rungs.check_rung(rung, edge)}[chk.name]
                out.append({
                    "rung": rung, "target": chk.name, "kind": "boundary",
                    "ok": e.ok,
                    "detail": ("99.9%% of tol -> GREEN, 110%% -> RED"
                               if e.ok else
                               "99.9% of tol -> RED (the boundary is not "
                               "where the derivation says)")})
    return out


# --------------------------------------------------------------------------
# B. live fault injection through a real engine
# --------------------------------------------------------------------------
#
# (rung, fault, the check that MUST go red, the checks that MUST stay green)
LIVE_FAULTS = (
    (0, "short_run", "steps_completed", ()),
    (1, "no_floor", "rest_z", ()),
    (2, "half_gravity", "fall_interval", ("spawn_z",)),
    (3, "ignore_zero", "omega_zero", ("omega_driven",)),
    (4, "slide", "rolling_consistency", ("distance",)),
    # Rung 5.  The two faults split the rung along its own seam: one moves the
    # SCENE and leaves the motion intact, the other stops the MOTION and leaves
    # the scene intact, and between them every check is shown to be wired to a
    # different fact.
    (5, "no_sweep", "range_final", ("range_static", "range_tracks")),
    (5, "wall_shifted", "range_static", ("sweep_span",)),
    # Rung 6.  ``no_stop`` breaks the ACTING half and must leave the SENSING
    # half green -- the seam that matters, because a controller that reads a
    # sensor correctly and then does nothing with it is a real and common bug.
    #
    # ``trigger_reading`` is the whole of that claim, and ``sensor_agrees`` is
    # deliberately NOT in the list.  MEASURED: a rover that never stops ends
    # with its sensor BURIED INSIDE the wall (nose against the face at
    # x = 3.0, sensor 20 mm further on), where "distance to the near face" is
    # a distance to a surface behind the ray and the residual reads 0.293 m.
    # That is a property of the fault, not of the sensor -- the honest run
    # measures 2.8e-06 on the same check -- and listing it here would demand a
    # green from a comparison that has stopped being defined.
    (6, "no_stop", "stop_gap", ("trigger_reading",)),
    # ``bounce`` is rung 6's ``slide``: the rover runs into the wall and is
    # then put back at exactly the right resting place.  ``stop_gap`` is green
    # and only the whole-run ``min_gap`` can see it.
    (6, "bounce", "min_gap", ("stop_gap", "wheel_stop", "stop_creep")),
    (7, "stalled_robot", "distance_worst", ("min_separation",
                                            "lateral_worst")),
    (7, "lane_offset", "min_separation", ("distance_worst",
                                          "wheel_omega_worst")),
    # Rung 8.  ``no_grip`` is the CAUSAL CONTROL, and it is the thing rung 4
    # does not have: the same scene with the fingers never closed must leave
    # the payload on the table.  Without it, a grasp asserted only by the
    # payload's pose cannot distinguish friction from a weld.
    (8, "no_grip", "lift_height", ("part_rest_z", "part_speed_max")),
    (8, "no_traverse", "place_x", ("lift_height", "carry_rel",
                                   "hold_clearance")),
    (8, "drop_mid_carry", "part_speed_max", ("part_rest_z",)),
    # Rung 9.  ``frozen`` is the important one and it is rung 9's ``slide``: a
    # world that cannot move is PERFECTLY deterministic, so ``repeat_delta``
    # must stay GREEN while the two checks that exist to stop that -- the
    # sensitivity control and the analytic anchor -- both go red.  Without it,
    # rung 9 would be satisfiable by an engine that simulated nothing.
    (9, "seed_nudge", "repeat_delta", ("sensitivity_shortfall",
                                       "fall_interval")),
    (9, "frozen", ("fall_interval", "sensitivity_shortfall"),
     ("repeat_delta",)),
    (9, "short_b", "repeat_length", ("fall_interval",)),
    # Rung 11.  Both are rung 7's, re-aimed at a 16-robot fleet: one breaks a
    # robot's own motion and must leave the fleet's geometry alone, the other
    # breaks the geometry and must leave every per-robot number alone.  The
    # lane offset is a SPAWN offset and never a per-step write -- CONTRACT.md
    # section 6 records why that distinction cost a whole fault.
    (11, "stalled_robot", "distance_worst", ("separation_shortfall",
                                             "lateral_worst")),
    (11, "lane_offset", "separation_shortfall", ("distance_worst",
                                                 "wheel_omega_worst")),
    # Rung 18.  ``wrong_omega_frame`` must leave ``replay_ic_fidelity`` GREEN
    # and that is the seam: the engine accepted exactly what it was handed, so
    # the IC check correctly does not blame it for the harness's frame error.
    # Only agreement with the recording sees it.
    (18, "ic_drop_velocity", "replay_ic_fidelity", ("tunnel_depth",),
     {"subset": "fault"}),
    (18, "wrong_omega_frame", "real_rot_err", ("replay_ic_fidelity",
                                               "tunnel_depth"),
     {"subset": "fault"}),
    (18, "table_hologram", "tunnel_depth", ("replay_ic_fidelity",),
     {"subset": "fault"}),
)


def _fault_entry(entry):
    """Normalise a LIVE_FAULTS row.

    ``must_red`` may name one check or several: rung 9's ``frozen`` has to red
    BOTH the sensitivity control and the analytic anchor, because a frozen
    world satisfies determinism and only those two can say so.  A row may also
    carry per-fault kwargs for ``arm.run`` -- rung 18's faults run the
    contract's small toss subset, which is used for the fault AND for the
    baseline it is judged against, or the two would not be comparable.
    """
    rung, fault, must_red, must_green = entry[:4]
    kw = entry[4] if len(entry) > 4 else {}
    reds = (must_red,) if isinstance(must_red, str) else tuple(must_red)
    return rung, fault, reds, tuple(must_green), dict(kw)


def live_cases(arm, out_root, only_rungs=None, **run_kw):
    """Run every live fault on ``arm`` and report whether each went red.

    A must-green companion is only evidence that the fault is SURGICAL if that
    check was green without the fault.  On a rung the engine already fails, an
    honest fault would otherwise be reported as "did not go red" when its
    must-red went red exactly as required and only its companions were already
    broken -- which reads as a defect in the LADDER and is not one.

    So the honest cell is run once per rung and cached, and a companion that
    was already red is reported as MASKED: the fault cannot be shown surgical
    on it, and that is a fact about the engine, not about the fault.  The
    failure itself stays loud where it belongs -- in the ladder table, on that
    rung's own row.
    """
    ok, why = arm.available()
    if not ok:
        return [{"rung": None, "target": arm.NAME, "kind": "live",
                 "ok": False,
                 "detail": "%s arm unavailable: %s" % (arm.NAME, why)}]

    baseline = {}

    def baseline_greens(rung, kw):
        """Names of the checks that are green on the HONEST scene."""
        key = (rung, tuple(sorted(kw.items())))
        if key not in baseline:
            d = os.path.join(out_root, "selftest",
                             "%s_rung%d_baseline" % (arm.NAME, rung))
            os.makedirs(d, exist_ok=True)
            s, mt = arm.run(rung, d, fault="none", **dict(run_kw, **kw))
            bm = analysis.reduce_samples(s, exit_code=mt.get("exit_code"))
            baseline[key] = {c.name for c in rungs.check_rung(rung, bm)
                             if c.ok}
        return baseline[key]

    out = []
    for entry in LIVE_FAULTS:
        rung, fault, must_red, must_green, kw = _fault_entry(entry)
        if only_rungs is not None and rung not in only_rungs:
            continue
        d = os.path.join(out_root, "selftest",
                         "%s_rung%d_%s" % (arm.NAME, rung, fault))
        os.makedirs(d, exist_ok=True)
        samples, meta = arm.run(rung, d, fault=fault,
                                **dict(run_kw, **kw))
        m = analysis.reduce_samples(samples, exit_code=meta.get("exit_code"))
        checks = {c.name: c for c in rungs.check_rung(rung, m)}
        reds = [checks.get(n) for n in must_red]
        was_green = baseline_greens(rung, kw) if must_green else set()
        greens = [checks[n] for n in must_green if n in checks]
        testable = [g for g in greens if g.name in was_green]
        masked = [g.name for g in greens if g.name not in was_green]
        good = (all(r is not None and not r.ok for r in reds)
                and all(g.ok for g in testable))
        bits = []
        for name, red in zip(must_red, reds):
            bits.append("%s=%s vs %s +/-%s -> %s" % (
                name, _f(red.measured if red else None),
                _f(red.expected if red else None),
                _f(red.tol if red else None),
                "RED" if (red is not None and not red.ok)
                else "STILL GREEN (bad)"))
        detail = "; ".join(bits)
        for g in testable:
            detail += "; %s stayed %s (%s)" % (
                g.name, "GREEN" if g.ok else "RED (bad)", _f(g.measured))
        if masked:
            detail += ("; MASKED (already red on the honest scene, so this "
                       "fault cannot be shown surgical on them): %s"
                       % ", ".join(masked))
        if meta.get("error"):
            detail += " | arm: %s" % str(meta["error"])[:160]
        out.append({"rung": rung, "target": ",".join(must_red), "kind": "live",
                    "fault": fault, "ok": good, "detail": detail,
                    "masked": masked,
                    "measurement": {k: v for k, v in m.items()
                                    if not isinstance(v, (list, dict))}})
    return out


def _f(v):
    if v is None:
        return "None"
    if isinstance(v, float):
        return "%.6g" % v
    return str(v)


def run_self_test(out_root, arm=None, only_rungs=None, **run_kw):
    cases = mutation_cases()
    if arm is not None:
        cases += live_cases(arm, out_root, only_rungs=only_rungs, **run_kw)
    failed = [c for c in cases if not c["ok"]]
    return cases, failed


def detector_validation(cases):
    """Which rungs' must-red faults have been shown to go red HERE.

    CONTRACT.md amendment C, adopted from OmniBench lane 4b, which reported
    ``cliff_detector_validated: false`` and refused to call its own green a
    pass.  A rung whose battery has never been run on an arm -- or was run and
    did not go red -- is ``UNVALIDATED`` on that arm, and UNVALIDATED IS NOT A
    PASS.  A green that cannot be made red is worth nothing, and this is the
    record of which ones have been.
    """
    live = [c for c in cases if c["kind"] == "live"]
    have = {}
    for c in live:
        have.setdefault(c["rung"], []).append(c)
    out = {}
    for rung in rungs.ALL_RUNGS:
        want = [e for e in LIVE_FAULTS if _fault_entry(e)[0] == rung]
        got = have.get(rung, [])
        if not want:
            out[rung] = {"status": "NO_BATTERY",
                         "why": "no fault is declared for this rung"}
        elif len(got) < len(want):
            out[rung] = {"status": "UNVALIDATED", "ran": len(got),
                         "declared": len(want),
                         "why": "the battery was not run here"}
        elif any(not c["ok"] for c in got):
            out[rung] = {"status": "UNVALIDATED", "ran": len(got),
                         "declared": len(want),
                         "why": "a declared fault did not go red"}
        else:
            out[rung] = {"status": "VALIDATED", "ran": len(got),
                         "declared": len(want)}
    return out
