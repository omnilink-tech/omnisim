# Real bin grasping, and the cold-first-load trap

> **UPDATE (2026-07-05) — the cold-first-load under-track NO LONGER REPRODUCES on the current
> binary. Verified.** Two independent cold-vs-warm measurements (genuine cold = fresh process +
> `OMNISIM_NO_WARMUP=1`, both confirmed running MuJoCo `cpu/mj_step`):
> 1. **Bare 6-DOF arm, direct joint command** (no closed-loop masking): cold and warm settle
>    **bit-identical to 6 decimals** — same joint coordinates *and* same physical end-effector XYZ
>    (`-0.127032, 0, 1.203212`). Method to re-measure: a bare fixed-base arm world with
>    `WorldInfo.newtonSolver "mujoco"`, command a fixed gravity-loaded joint target, settle, and
>    diff the position-sensor readings + the deepest link's `getPosition()` across a cold run
>    (`OMNISIM_NO_WARMUP=1`) vs a warm run (`OMNISIM_FORCE_WARMUP=1`).
> 2. **Full bin-grasp demo** (the exact demo below): cold and warm are **bit-identical at
>    every phase** — descend lands `tcp_z=0.081` in *both* (the doc's historical cold/warm split of
>    0.081 vs 0.093 is gone), same trajectory to 3 decimals.
>
> Since a warm run *is* the reload (verified: teardown/finalise markers present) and cold matches it
> exactly, **cold is already the good state.** Likely fixed by **`eb86f888`** ("make the Newton solver
> choice survive the GUI's multi-build load") + the finalize-time solver re-assert
> (`WbNewtonBackend::finalizeWorld`): cold now builds **MuJoCo** instead of falling back to XPBD, and
> the articulation tracks. **The controller-side `warmup_reload` helper is therefore defensive-only
> now — and disabled by default** (a no-op unless `OMNISIM_FORCE_WARMUP=1` re-enables it;
> `OMNISIM_NO_WARMUP=1` still forces it off and wins — see the Reference section below); it is no
> longer load-bearing. The journey below is kept as the historical record of how the trap was found and mitigated.

> **THE DEMO IN THIS DOC WAS RETIRED (2026-07-05).** While verifying the cold-load fix (above), the
> one-wall bin grasp was found to *slip* headless (bin never lifts) in **both** cold and warm —
> independent of the warm-up, so NOT the cold-load bug. An exhaustive investigation then proved the
> slip is **not fixable by any parameter**: sweeps of finger stiffness (`FINGER_KE` 10k–80k), contact
> stiffness (`CONTACT_KE` 15k–40k), friction (`GROUND_MU` 3.5–8), `IMPRATIO`/`ITERS`, commanded width
> (`GRIPW`), grasp depth, **and** bin mass (down to 0.2 kg) all fail. Instrumented pad-vs-wall
> geometry confirmed the pads *do* reach the wall; the real failure mode is that **a one-wall pinch of
> a FREE bin has nothing to react against — closing the jaws shoves/flips the whole bin** (measured:
> bin flips to 92° and is shoved 156 mm on close), and the gripper's 85 mm opening cannot span the
> bin's 120 mm walls to do a stable two-wall enclosure. This **vindicates Act 2's original physical
> conclusion** (which Act 3–4 had wrongly overturned — that "success" was the cold-load trap masking a
> grasp that was never robust). Since the demo is a *superseded diagnostic* (production picking uses
> a suction end-effector, 36/36), the world + controller + run script were **removed** rather than
> propped up with a stabilization weld (which would just duplicate the earlier weld-stabilized carry
> demo). The journey below is retained as the historical record and for its lessons; the referenced
> demo files no longer exist.

A developer journey doc. It records how we got a 6-DOF arm to grab a bin **by one wall**
with a real friction grip (no weld) — and, more importantly, the multi-hour wrong
turn that taught us a sim-wide testing trap. If you are debugging a grasp, an
insertion, or any precise-contact task and it "doesn't work," **read the lessons at
the bottom first.**

## The goal

A 6-DOF arm + Robotiq 2F-85 gripper picks up a movable bin/tote and manipulates it. The
earlier "carry demo" held the tote with a **grasp-stabilization weld** — the controller
pins the tote's pose to the gripper each tick. It works, but it
looks fake: the bin tracks the gripper rigidly, with no physical compliance. The user
wanted the real thing — grab a wall, let the bin hang and swing, manipulate it the way
a robot handles a real bin.

## Act 1 — Isolate the real grip

We built an instrumented, isolated world + controller (both since removed — see the
retirement note above). No weld anywhere: if the bin moves with the gripper, it is
because contact friction holds it. Every phase logs the gripper TCP, the bin pose, the
tilt, the finger gap, and the bin-relative-to-TCP vector (constant ⇒ grip holds;
drifting ⇒ slip).

Two real modelling bugs surfaced immediately, and both are kept as deliverables:

1. **The 2F-85 was modelled wrong — it bottomed out 26 mm apart.** The finger pads
   were mounted at y=±0.020 with 14 mm thickness, so at full close the inner faces
   stopped **26 mm apart**. The gripper physically could not pinch anything thinner
   than 26 mm — a realistic ~6 mm tote wall just floated untouched in the gap. The
   real 2F-85 closes to ~0 mm. Fixed in the grasp-tuned gripper URDF by mounting the pads at
   y=±0.007 (half-thickness) so they meet at y=0 when q=0, giving the correct 0–85 mm
   stroke. (Not yet promoted to the production gripper URDF; safe to promote —
   every existing grab is on objects >26 mm.)

2. **A position-servo finger is limp on a thin wall.** Grip force = ke·error capped at
   the effort limit. The per-joint stiffness is `ke = effort·10 = 500`, which only
   saturates a 50 N finger at a ~100 mm error — useless on a 6 mm wall (~3 mm
   interference ⇒ ~1.5 N). The global `OMNISIM_NEWTON_TARGET_KE` would fix the finger
   but also stiffen the **arm** hinges and make them violent. So we added a
   **finger-only** knob, `OMNISIM_NEWTON_FINGER_KE` / `_FINGER_KD`, applied only to
   prismatic/slider joints (`WbNewtonBackend.cpp`). The arm keeps its safe gains.

## Act 2 — The exhaustive sweep that "proved" it was impossible (it was wrong)

We then swept the grip: bin mass 0.4–1.5 kg, μ 1.2–4.0, finger ke 5k–25k, contact ke
2.5k–50k, wall 6 & 12 mm, impratio 10–150, iters 200–400, gentle damped close. **Every
single run failed to lift the bin.** Light bins got shoved out of the jaws on close;
heavy bins rotated 90° and slipped on lift; a firm grip squirted/flipped the bin.

We wrote a confident conclusion: *a rigid box pad on a thin wall makes an unstable
contact that the penalty-contact friction can't sustain; a one-wall friction grasp of
a free bin is irreducibly marginal; the only real fix is to model compliant rubber
pads.* We even staked the recommendation on it.

**This was completely wrong, and the reason is the trap below.**

## Act 3 — The user's one-line clue

> "When I reset the world (Ctrl-Shift-R), it worked perfectly. Can you explain how
> this is possible?"

The same demo, same config, **failed on the first GUI load but worked after a world
reload.** That single observation invalidated the entire sweep — because it meant the
grip physics was fine and something about the *cold first load* was the problem.

The kicker: **`headless_runner.py` launches a fresh process every run, which is always
a cold first load.** Every one of our ~15 sweep runs was stuck in exactly the failing
condition. We had been testing only the broken case and never knew it.

## Act 4 — Reproduce and characterise

We reproduced the reload headless by having the controller call `robot.worldReload()`
once at startup. The warm (post-reload) run grasped and **lifted the bin cleanly**
(z 0.05 → 0.20, the bin-relative-to-TCP vector dead constant). Diffing the cold vs warm
logs pinned the mechanism:

- On a **cold first load** the arm **undershoots its commanded IK pose by ~14 mm**
  (descend lands tcp z=0.081 vs the commanded 0.093), so the jaws close on the wall
  misaligned and the marginal grasp slips.
- After a **reload** the arm tracks to err ~1.7 mm (lands 0.093) and the grip holds.

It is fully deterministic (cold result is byte-identical across `ITERS=300` vs `2000`),
the solver is MuJoCo `cpu/mj_step` in both, the at-rest pose is identical, and
re-commanding the descend pose 8× does **not** fix the cold undershoot. So a **world
rebuild** re-initialises the Newton/MuJoCo articulation in a way the first build does
not — a first-load articulation-tracking bug.

## Act 5 — Engine root-cause attempts (what did NOT work)

The comment near the solver construction (`MuJoCo also caches qpos at solver
construction… a rebuild forces the standing config`) pointed at the solver build, so we
tried:

- **Rebuild the `SolverMuJoCo` a second time in `finalize()`** → no effect (byte-identical
  cold failure).
- **Rebuild the solver after the world has stepped N times** (`OMNISIM_NEWTON_STEP_REBUILD`)
  → no effect.

So it is **not** the solver construction. Only a **full world rebuild** fixes it,
which means the difference lives in the broader C++ world build — most likely the
ODE↔Newton body-disable ordering / node finalize on the first load. **That deep root
cause is still open** (tracked in the backlog). Both experiment branches were reverted;
the finger-ke knob and the gripper close-fix are the kept engine deliverables.

## Act 6 — The warm-up fix (and a GUI race)

Since a full reload reliably fixes it, we made the warm-up automatic: a controller-side
`warmup_reload(robot)` helper reloads the world **once** at controller
startup, keyed per simulator launch (survives the reload-restart, re-warms a new launch,
can't loop). Two gotchas, both fixed:

1. `worldReload()` **raises if called before the controller has stepped** — step ~10
   ticks first.
2. After a successful `worldReload()` the controller must **`os._exit(0)`** (step until
   the teardown delivers −1, then exit), **not** return into the caller. If the cold
   controller keeps running it races the reload teardown, and in the **windowed GUI** the
   app quits cleanly mid-reload (exit 0 — "the window closed by itself"). Headless
   tolerated the sloppy version; the GUI did not.

With both fixed, the demo runs the full sequence in **headless and GUI**: grab the wall,
lift, carry left/right, tilt ~23° (the grip holds through the tilt), set down.

Also observed (related, separate): the GUI's **first** build can fall back to
`SolverXPBD` even with `WorldInfo.newtonSolver "mujoco"` — the solver preference is
plumbed from `WbSolid`'s Newton flush and can arrive *after* the solver is built. The
warm reload gets MuJoCo, so warmup_reload masks it; a proper fix sets the preference
before `finalizeWorld` builds the solver.

## Act 7 — Make the trap impossible to fall into again

The wasted time came from headless silently testing only the cold case. Guard rails:

- **`headless_runner.py` defaults to WARM** — it sets a per-launch `OMNISIM_WARMUP_TOKEN`
  so warmup-aware controllers reload once at startup and results match a stabilised
  session. `--cold` tests the raw first-load on purpose. A cold-start note prints every
  run.
- **`AGENTS.md` §3b** documents the trap with the rule: *if a grasp/insertion "fails"
  headless, reload (or check the warm path) before concluding the physics is wrong.*

## Lessons (the part to actually remember)

1. **A fresh OmniSim process is always a COLD first load, and cold ≠ warm for precise
   contact.** Position tracking is degraded until a world reload. Grasps, pinches,
   insertions — anything that needs the arm to be exactly where it was told — can fail
   cold and work warm.
2. **Headless = always cold.** If a precise-contact task fails headless, **reload before
   concluding the physics or your approach is wrong.** We wrote a long, confident,
   *wrong* analysis because we never reloaded.
3. **One contradicting data point beats a big confident sweep.** "It works after a
   reload" instantly invalidated ~15 runs of conclusions. When a user reports behaviour
   that contradicts your model, chase *that*, hard, before defending the model.
4. **Reproduce the good case, then diff against the bad case.** The cold-vs-warm log
   diff (arm lands at 0.081 vs 0.093) is what turned "the physics can't do this" into "a
   1 cm positioning bug."
5. **A workaround that's reliable headless can be flaky in the GUI.** `worldReload` from
   a controller races the windowed event loop; it needed `os._exit(0)` to be safe.

## Reference

- **The one-wall friction-grip demo was RETIRED (2026-07-05)** — its world, controller, and run
  script were removed (see the retirement note at
  the top). The one-wall grasp of a free bin is fundamentally marginal; production picking uses a
  suction end-effector (36/36). The corrected gripper URDF remains.
- **Warm-up:** the controller-side `warmup_reload(robot)` helper is now a **no-op by default** (the
  cold-load bug is fixed). `OMNISIM_FORCE_WARMUP=1` re-enables the old reload; `OMNISIM_NO_WARMUP=1`
  forces off.
- **Headless:** cold and warm are equivalent now; `--cold` on `headless_runner.py` just force-disables
  the (already-off) warm-up.
- **Commits:** `ae904467` (real grip + cold-start fix + headless guard), `95700fd8`
  (GUI warm-up race fix).
- **Open:** the first-load articulation-tracking root cause (only a full world rebuild
  fixes it); the GUI XPBD-first-load solver-preference timing.
