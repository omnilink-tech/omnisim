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

"""redcheck.py -- prove the MuJoCo arm's green rows can go red.

A passing row is only evidence if the same plumbing would have reported a
failure had one occurred.  This script breaks the PHYSICS (never the
measurement) three ways and requires the named assertion to fail each time,
then requires two identical runs to agree bit for bit.

    python redcheck.py

Exit 0 if every expectation held.  Each fault is a corruption of the scene or
the actuation, so what is being proven is that the measurement path reaches
the quantity it claims to measure -- a self-test that instead poked the
measured value would prove only that arithmetic works.
"""

from __future__ import annotations

import importlib.util
import os
import sys

HERE = os.path.abspath(os.path.dirname(__file__))


def _load_shared():
    """Bootstrap this arm's loader by path -- never ``import shared``/``run``.
    See ``shared.py``'s docstring for why a generic module name is unsafe in a
    process that loads all three arms."""
    key = "ladder0_mujoco_shared"
    if key in sys.modules:
        return sys.modules[key]
    sp = importlib.util.spec_from_file_location(
        key, os.path.join(HERE, "shared.py"))
    mod = importlib.util.module_from_spec(sp)
    sys.modules[key] = mod
    sp.loader.exec_module(mod)
    return mod


shared = _load_shared()
spec = shared.spec
run = shared.sibling("run")

# fault -> (rung, {checks that MUST be red}, {checks that MUST stay green}).
#
# THE CONTRACT'S OWN BATTERY IS NOT DUPLICATED HERE.  ``selftest.LIVE_FAULTS``
# is the authority for the fourteen faults CONTRACT.md section 6 requires, and
# running it against this arm is what proves those; a second copy in this file
# would be a place for the two to disagree, and this arm has already been
# bitten once by exactly that (``arm.SUPPORTED_FAULTS`` was a hand-written set
# that fell behind ``run.FAULTS``, so nine implemented faults reported
# themselves as unimplemented and the self-test scored the proofs MISSING
# rather than failed).
#
# What IS here is this arm's own extras -- faults the contract does not ask
# for, each proving something its battery does not.  Every scene is the shared
# contract verbatim: this arm has no floor override, because a scene number an
# arm can set for itself is a scene number that can disagree with the
# expectation the row is judged against.
EXPECTED_RED = {
    # Older names for two of the contract's, kept because the committed result
    # files use them.
    "gravity_half": (2, {"fall_interval", "fall_time_abs"}, ("spawn_z",)),
    "slide_not_roll": (4, {"wheel_omega", "rolling_consistency"},
                       ("distance",)),
    # A contact SOFTER than the solver's own defaults imply.  The contract's
    # battery has no fault that separates "the body is on the wrong surface"
    # from "the contact is too soft", and rung 1 asserts both.
    "soft_floor": (1, {"rest_z", "contact_penetration"}, ()),
    # A pinch BELOW the Coulomb bound m g / (2 mu).  The pads are in contact
    # for the entire run and the part never leaves the table: the case a
    # contact COUNT scores as a successful grasp, which is why rung 8 is
    # asserted geometrically.
    "weak_grip": (8, {"lift_height", "hold_clearance"},
                  ("part_rest_z", "part_speed_max")),
}


def _checks(rung, fault=None, integrator=None):
    """One CELL -- several runs for rungs 9, 11 and 18, one for the rest.

    It goes through ``run_cell`` rather than ``run_rung`` for the multi-run
    rungs on purpose: rung 9's whole content is the agreement of replicas that
    came from DIFFERENT PROCESSES, and a self-test that ran one replica in this
    interpreter would prove nothing about the thing being asserted.
    """
    import tempfile
    if int(rung) in run.MULTI_RUN:
        with tempfile.TemporaryDirectory(prefix="ladder0_redcheck_") as d:
            samples, meta = run.run_cell(rung, d, fault=fault,
                                         integrator=integrator)
    else:
        samples, meta = run.run_rung(rung, integrator=integrator, fault=fault)
    m, checks = run.judge(samples, meta)
    return m, {c["name"]: c for c in checks}


def main():
    failures = []

    # --- A. every fault must redden its named assertions -----------------
    for fault, (rung, must_be_red, must_stay_green) in sorted(
            EXPECTED_RED.items()):
        _m, checks = _checks(rung, fault=fault)
        red = {n for n, c in checks.items() if not c["ok"]}
        missing = must_be_red - red
        broke = [n for n in must_stay_green if n in red]
        print("fault %-16s rung %d  red={%s}  %s"
              % (fault, rung, ",".join(sorted(red)) or "-",
                 "OK" if not (missing or broke) else
                 "DID NOT REDDEN %s%s" % (sorted(missing),
                                          "; COLLATERAL %s" % broke
                                          if broke else "")))
        if missing:
            failures.append("%s failed to redden %s" % (fault, sorted(missing)))
        if broke:
            failures.append("%s reddened %s, which must stay green"
                            % (fault, broke))

    # --- B. the same faults' controls must be green ----------------------
    # (otherwise "it went red" proves nothing about the fault)
    for rung in (1, 2, 4, 8):
        _m, checks = _checks(rung)
        red = {n for n, c in checks.items() if not c["ok"]}
        print("control          rung %d  red={%s}  %s"
              % (rung, ",".join(sorted(red)) or "-", "OK" if not red else "BAD"))
        if red:
            failures.append("control rung %d was not green: %s"
                            % (rung, sorted(red)))

    # --- C. the scenes must FOLLOW the contract, not match it -------------
    # Equality with today's number is not evidence of derivation: a hard-coded
    # copy passes that test every time, and this arm's committed models drifted
    # in exactly that way once already.  So move each constant and require the
    # emitted MJCF to move with it.  Every entry is (attribute, new value, the
    # rung to emit, a string that must then appear in it).
    scenes = shared.sibling("scenes")
    # (attribute, new value, rung, a string that must then appear, run spec).
    # The run spec is one entry of ``scenes.run_specs`` and exists because a
    # multi-run rung's scene is a FAMILY: rung 11's lane spacing is invisible
    # at N = 1, where the only lane is y = 0.
    MOVED = [
        ("RUNG5_WALL_FACE_X", 7.25, 5, 'pos="7.35 0'),          # near face
        ("RUNG5_SENSOR_DX", 0.37, 5, 'pos="0.37 0 0"'),
        ("RUNG5_CARRIER_EDGE", 0.24, 5, 'size="0.12 0.12 0.12"'),
        ("RUNG6_SENSOR_DX", 0.41, 6, 'pos="0.41 0'),
        ("RUNG6_WALL_FACE_X", 4.5, 6, 'pos="4.6 0'),
        # RUNG7_Y, not RUNG7_LANE_DY.  The contract DERIVES the lane tuple
        # from the spacing at import time, so mutating the spacing afterwards
        # cannot move anything -- this probe reported "SCENE DID NOT MOVE" for
        # RUNG7_LANE_DY, and the honest reading of that is that the scene
        # follows the published tuple, which is the constant it is supposed to
        # follow.  Both live in rungs.py and are computed together, so they
        # cannot drift apart; what CAN drift is an arm that hard-codes lanes,
        # and that is what this catches.
        ("RUNG7_Y", (-4.0, -2.0, 0.0, 2.0, 4.0), 7, 'pos="0 -4.0 0.6"'),
        ("RUNG8_PART_EDGE", 0.09, 8, 'size="0.045 0.045 0.045"'),
        ("RUNG8_TABLE_TOP", 1.31, 8, None),   # part rides on the table top
        ("RUNG8_PAD_OPEN_Y", 0.077, 8, 'pos="0 0.077 0"'),
        ("RUNG8_MU", 4.75, 8, 'friction="4.75'),
        # Rung 9.  The pile and the dropped cube both follow the contract, and
        # both are written as EXACT float64 literals -- see the exactness
        # section below for why that is not cosmetic.
        ("RUNG9_DROP_XY", 0.137, 9, 'pos="0.137 0.137'),
        ("RUNG9_PILE_Z", 0.77, 9, '0.77'),
        ("RUNG9_SPAWN_Z", 1.77, 9, '1.77'),
        # Rung 11.  The lane spacing is read at CALL time by ``rung11_y``, so
        # unlike rung 7's it is the spacing itself that has to move the scene.
        ("RUNG11_LANE_DY", 2.5, 11, 'pos="0 -3.75', {"n": 4}),
        ("WHEEL_R", 0.13, 11, 'size="0.13"', {"n": 4}),
    ]
    for entry in MOVED:
        attr, value, rung, needle = entry[:4]
        run_spec = entry[4] if len(entry) > 4 else None
        before = getattr(spec, attr)
        baseline = scenes.mjcf(rung, run=run_spec)
        try:
            setattr(spec, attr, value)
            after = scenes.mjcf(rung, run=run_spec)
        finally:
            setattr(spec, attr, before)
        moved = after != baseline
        found = needle is None or needle in after
        ok = moved and found
        print("derives-from  %-22s rung %d  %s"
              % (attr, rung, "OK" if ok else
                 ("SCENE DID NOT MOVE" if not moved
                  else "moved, but %r is absent -- wrong quantity?" % needle)))
        if not ok:
            failures.append("rung %d does not derive %s from the contract"
                            % (rung, attr))

    # --- C2. rung 9's fault must SURVIVE THE FORMATTER --------------------
    # This arm's own, and it guards the way a red proof stops proving anything
    # without ever failing: ``RUNG9_FAULT_NUDGE`` is far below what ``%.6f``
    # can represent, so that formatter writes the faulted scene BYTE-IDENTICAL
    # to the honest one and ``seed_nudge`` silently does not happen -- while
    # still reporting that it ran.  Nothing downstream can see that, so it is
    # checked here: the emitted scene must differ, it must differ by the
    # contract's own number, and ``%.6f`` must still collapse that number, so
    # the hazard is re-demonstrated rather than remembered.  (The magnitude is
    # never restated in this arm: it moved 1e-12 -> 1e-7 mid-construction, and
    # a comment quoting it would now be wrong while every check still passed.)
    nudged = [s for s in scenes.run_specs(9, "seed_nudge")
              if s.get("nudge_m")]
    if len(nudged) != 1:
        failures.append("rung 9's seed_nudge should nudge exactly one replica, "
                        "not %d" % len(nudged))
    honest = scenes.mjcf(9, run={"tag": "b", "eps_m": 0.0, "nudge_m": 0.0})
    faulted = scenes.mjcf(9, run=nudged[0]) if nudged else honest
    want = repr(float(spec.RUNG9_DROP_XY + spec.RUNG9_FAULT_NUDGE))
    ok = faulted != honest and want in faulted and float(want) != \
        spec.RUNG9_DROP_XY
    print("float64-exact  rung 9 nudge %-11s %s"
          % (want, "OK" if ok else
             "THE FAULT DID NOT REACH THE SCENE -- a formatter ate the nudge"))
    if not ok:
        failures.append("rung 9's seed_nudge does not survive the scene "
                        "formatter, so its red proof proves nothing")
    # and the negative control: the formatter this file uses everywhere else
    # WOULD have eaten it, which is why the exact one exists.
    if ("%.6f" % (spec.RUNG9_DROP_XY + spec.RUNG9_FAULT_NUDGE)
            != "%.6f" % spec.RUNG9_DROP_XY):
        failures.append("the %.6f control no longer demonstrates the hazard; "
                        "re-derive the exactness argument")

    # --- C3. rung 18's scene must FOLLOW lane1r, and say so when it cannot -
    # Rung 18's ground truth is another lane's recording and the contract says
    # lane1r is right.  Two things have to hold for that to mean anything: the
    # scene must move when lane1r's numbers move, and a lane1r that disagrees
    # with ITSELF must raise rather than let this arm pick a side.
    try:
        D = spec.rung18_dataset()
    except Exception as exc:                            # noqa: BLE001
        print("lane1r         rung 18  NOT PRESENT (%r) -- provenance "
              "unproven" % (exc,))
        failures.append("rung 18's provenance could not be proven: %r" % (exc,))
        D = None
    if D is not None:
        base_xml = scenes.mjcf(18)
        before = D.CUBE_INERTIA
        try:
            D.CUBE_INERTIA = before * 2.0
            scenes._RUNG18_FACTS = None
            scenes.mjcf(18)
            drifted = "did NOT raise"
        except Exception:                               # noqa: BLE001
            drifted = "raised"
        finally:
            D.CUBE_INERTIA = before
            scenes._RUNG18_FACTS = None
        print("lane1r-agrees  rung 18  dataset vs world disagreement %s  %s"
              % (drifted, "OK" if drifted == "raised" else "BAD"))
        if drifted != "raised":
            failures.append("rung 18 tolerates lane1r disagreeing with itself "
                            "about the cube's inertia")
        # and the scene must actually carry lane1r's numbers rather than a copy
        f = scenes.rung18_facts()
        for needle, what in ((repr(f["cube_inertia_kg_m2"]), "inertia"),
                             (str(f["cube_mass_kg"]), "mass"),
                             ("%g" % (f["cube_edge_m"] / 2.0), "half edge")):
            if needle not in base_xml:
                failures.append("rung 18's scene does not carry lane1r's %s "
                                "(%s)" % (what, needle))
        print("lane1r-derives rung 18  edge/mass/inertia/mu/dt from %s  OK"
              % os.path.basename(f["lane1r_dataset"]))

    # --- D. determinism: two identical runs must agree exactly -----------
    for rung in spec.RUNGS:
        a, _ = _checks(rung)
        b, _ = _checks(rung)
        diff = [k for k in a
                if isinstance(a[k], float) and isinstance(b.get(k), float)
                and a[k] != b[k]]
        print("determinism      rung %d  %s"
              % (rung, "bitwise" if not diff else "DIFFERS on %s" % diff))
        if diff:
            failures.append("rung %d not reproducible: %s" % (rung, diff))

    print("")
    if failures:
        for f in failures:
            print("FAIL: %s" % f)
        return 1
    print("redcheck: all expectations held")
    return 0


if __name__ == "__main__":
    sys.exit(main())
