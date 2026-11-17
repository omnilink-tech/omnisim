## SolidReference

```
SolidReference {
  SFString solidName ""   # any string
}
```

### Description

A [SolidReference](#solidreference) can be used inside the `endPoint` field of a [Joint](joint.md) node to refer either to an existing [Solid](solid.md) or to the static environment.
The only constraint when referring to a [Solid](solid.md) is that both [Solid](solid.md) and [Joint](joint.md) must be descendants of a common upper [Solid](solid.md).

> ⛔ **CLOSING A MECHANICAL LOOP DOES NOT WORK ON THIS ENGINE: IT KILLS PHYSICS
> FOR THE WHOLE WORLD.** (It used to do that *while `run-headless` printed
> PASS*; that half was fixed on 2026-08-16 — see item 5 — but the loop itself
> is as fatal as it ever was.)
> This section used to say "Mechanical loops can be closed this way" — inherited
> from upstream, where ODE closed loops. Newton/MuJoCo is a **tree**-articulation
> solver, and OmniSim has no equality-constraint path to substitute, so a second
> [Joint](joint.md) arriving at a [Solid](solid.md) that already has a joint
> parent is not a loop — it is a malformed articulation.
>
> **Measured 2026-08-13** (tree `227b35c36`, `omnisim-bin.exe` sha256[:16]
> `1b82affcd3956d95`, machine `9722d23d12a3`, `newtonSolver "mujoco"`, newton
> 1.2.0 / warp 1.13.0 / mujoco 3.8.1, device `cpu`), on the two-hinge example
> that used to sit in this page:
>
> 1. **Both joints reach the builder.** The engine logs them itself, with the
>    same parent and the same child:
>    `hinge joint 0 (parent=body 2, child=body 3)` and
>    `hinge joint 1 (parent=body 2, child=body 3)`.
> 2. **`SolverMuJoCo` construction then fails outright** —
>    `ValueError: Multiple joints lead to body 3`, raised from newton's
>    `_src/utils/topology.py:47` via `solver_mujoco.py:4468`
>    (`_convert_to_mjc` → `topological_sort`).
> 3. **There is no solver left to substitute**, so the world gets *no Newton
>    world at all*: no `.newton.json` verdict sidecar, and not one
>    `[OmNewtonBackend] step` line.
> 4. **The blast radius is the entire world, not the robot.** A 1 kg box two
>    metres away, sharing nothing with the loop, was released at z = 3.0 and
>    read **exactly 3.0 after 1000 steps** (moved 0.000000 m). Deleting the
>    loop joint and changing nothing else, the same box falls and settles at
>    **0.649892 m** and the sidecar reports `finalised: true, degraded: false`.
> 5. ✅ **`run-headless` now FAILS on this (fixed 2026-08-16).** It used to print
>    `PASS` and exit 0 (`0 errors, 4254 warnings`), because the finalize raise
>    was logged at WARNING and the lane counts only `ERROR:`/`FATAL:` lines.
>    `OmNewtonBackend::finalizeWorld()` now reports it through
>    `reportPyErrorFatal()` at **ERROR**, leading with `THIS WORLD HAS NO
>    PHYSICS` and carrying the Python exception. Re-measured on a two-hinge
>    reproducer: pre-fix `0 errors, 1049 warnings … PASS` exit 0, post-fix
>    `1 errors, 1 warnings … FAIL` exit 1, with the same world minus the loop
>    joint still `PASS` at exit 0. Pinned by
>    [`tests/test_newton_finalize_failure_is_error.py`](../../tests/test_newton_finalize_failure_is_error.py);
>    the harness classifies the line as `NEWTON_WORLD_NOT_BUILT`.
>    ⚠️ **`--fail-on-runaway` still cannot see this class** — a body frozen at
>    its authored pose is indistinguishable from one still legally mid-air —
>    and neither can `--until-finalized`, which proves load + finalize and this
>    *is* the finalize failing.
> 6. Finalize is still retried every tick (a failed finalize builds nothing, so
>    the world never closes for build), so the log still fills with the
>    per-tick `INFO` re-assert lines. The **report** is now latched to once per
>    world, which is what the 4254 figure above was: 237 KB of log in a 10 s run
>    before, 30 KB after. The retry itself is unchanged, so a long run can still
>    time out the validation harness's supervisor RPC.
>
> ⚠️ **CORRECTION (2026-08-16): `Body N has multiple parents in this
> articulation` IS one of the two real wordings, and this paragraph used to deny
> it.** The denial was half right: that string does appear in a source comment
> ([`omnisim_newton_runtime.py:2345`](../../src/omnisim/physics/omnisim_newton_runtime.py))
> describing a *different*, already-fixed eager-add ordering bug — but newton
> also **raises** it, from its own `_src/sim/builder.py:3111`. Measured on
> newton 1.5.0, both wordings out of one runtime: the flat two-hinge reproducer
> above raises `Multiple joints lead to body N` (from `topological_sort`), while
> the nested loop in the shipped `coupled_motors.omniworld` raises `Body 8 has
> multiple parents in this articulation: 7 and 9`. **Match on either.**
>
> ✅ **The one shipped casualty is FIXED (2026-08-16).**
> [`projects/samples/devices/worlds/coupled_motors.omniworld`](../../projects/samples/devices/worlds/coupled_motors.omniworld)
> closed a loop through each gripper's passive pivot and therefore **had no
> physics at all** — silently from the ODE deletion until the severity fix made
> it fail `run-headless` (`1 errors, 1 warnings … FAIL`, exit 1, no
> `.newton.json` sidecar), which was that fix exposing an existing defect, not a
> new break; [`roll-check.md:348`](../developer/roll-check.md) had already
> recorded that its probe "never produced a document, before or after". It also
> refuted ladder0 finding **F12**, which predicted the loop joint would be
> *silently never registered* with no fatal: the joint reaches the builder and
> newton refuses the whole model.
> **The fix was the tree half of the advice below** — each finger is now an open
> chain with the fingertip folded rigidly into it, and both four-bar rockers are
> gone. It reads `0 errors, 0 warnings … PASS`, exit 0, sidecar `finalised:
> true, degraded: false`, and passes `--fail-on-runaway`; the gripper picks the
> block up and sets it down every cycle for 80 s of sim time. The cost is that
> the jaws pivot instead of staying parallel, which is recorded in a header
> comment and in `WorldInfo.info` so nobody re-adds the linkage.
>
> ⚠️ **One world in the tree still closes a loop and it must STAY that way:**
> [`tests/manual_tests/worlds/interaction_with_solid_reference_model.omniworld`](../../tests/manual_tests/worlds/interaction_with_solid_reference_model.omniworld)
> — the loop is its subject under test (issue #7064). It is a manual/GUI world
> in no automated lane. Note it currently fails *earlier and differently*: the
> engine dies at load with `STATUS_STACK_OVERFLOW` (exit `3221225725`) and never
> reaches finalize, so it is **not** an instance of the failure documented on
> this page. See [`tests/manual_tests/README.md`](../../tests/manual_tests/README.md).
>
> And the loop joint is **not** quietly
> dropped before the builder: the guards that look like they would drop it do
> something narrower. `collectSolidChildren` skips only the *endpoint*, and
> still appends the joint ([`OmSolid.cpp:1546,1549`](../../src/omnisim/nodes/OmSolid.cpp));
> `OmBasicJoint::setJoint()` rejects only a reference that resolves to nothing
> ([`OmBasicJoint.cpp:868`](../../src/omnisim/nodes/OmBasicJoint.cpp)), and
> `solidEndPoint()` resolves a `SolidReference` to the referenced `Solid`
> (`:1015`), so a *valid* loop reference passes it; and the Newton enqueue in
> `postFinalize` has no `solidReference()` guard at all. It reaches the builder.
>
> **What still works:** `solidName "<static environment>"` — anchoring a joint
> to the world. That is a tree edge, not a loop. **What to do instead:** model
> the mechanism as a tree and, if the loop is load-bearing, drive the dependent
> joint from a controller.

### Field Summary

- `solidName`: This field specifies either the static environment (if the value is `<static environment>`) or the name of an existing [Solid](solid.md) node to be linked with the [Joint](joint.md)'s closest upper [Solid](solid.md) node.
Referring to the [Joint](joint.md) closest upper [Solid](solid.md) node or to a [Solid](solid.md) node which has no common upper [Solid](solid.md) with the [Joint](joint.md) is prohibited.

### Mechanical Loop Example — ⛔ DOES NOT RUN, kept only to name the shape

⚠️ **This is the example that was measured failing above. Do not copy it.** It is
retained so the failure has a name you can match against your own world: two
[HingeJoint](hingejoint.md)s under one upper [Solid](solid.md), the second
reaching the first's endpoint through a [SolidReference](#solidreference). Under
Newton this produces `ValueError: Multiple joints lead to body N` and leaves the
**whole world** without physics. (`run-headless` now FAILs on it — see item 5
above; it printed PASS until 2026-08-16, and this sentence used to say so.)

```
HingeJoint {
  jointParameters HingeJointParameters {
    anchor 0 0 1
  }
  endPoint Solid {
    name "front axle"
  }
}
HingeJoint {
  jointParameters HingeJointParameters {
    anchor 0 0 -1
  }
  endPoint SolidReference {
    solidName "front axle"
  }
}
```
