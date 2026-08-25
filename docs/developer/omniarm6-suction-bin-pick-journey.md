# Emptying a dense bin: the suction-cup pivot (and the warm-reload bounce)

How the OmniArm 6 bin-picking demo went from "best ~6–21/36 and unreliable" to a
**deterministic 36/36**, by changing the *end-effector* instead of fighting the
physics — and the one-line velocity bug that hid behind a cold solver.

Demo: `projects/samples/demos/worlds/flagship/omniarm6_bin_picking.omniworld`
Controller: `projects/samples/demos/controllers/omniarm6_bin_picking/omniarm6_bin_picking.py`
Robot: `projects/robots/omnisim/omniarm6/omniarm6_suction.urdf`
Commits: `dab181a9` (suction v1), `727eb01c` (real cup + bounce fix).

## The goal

Empty a DENSE box — 36 cubes (18 red / 18 blue, 4×3×3, three layers, dumped with
random jitter + rotation) — and colour-sort every cube into its matching tray.
The bar the user set was not speed; it was **reliability**: deterministic and
repeatable, to the point that 5 instances run at once all land the same correct
result.

## Act 1 — The finger gripper can't, and shaking makes it worse

A Robotiq 2F-85 (85 mm jaws) on 50 mm cubes that have tessellated into a solid
block has nowhere to put a second jaw. Best results hovered at ~18/36, and every
attempt to *create* clearance backfired:

- **Shaking / agitating** the bin to tumble cubes free flings them out of reach
  (X-slosh sent cubes to x≈0.9, Y-slosh to y≈1.0). Gentle shakes don't singulate;
  vigorous ones scatter. ~6–17/36.
- **Pushing** to declutter just re-packs the pile. Four push variants (single-step
  dormant 18, forced 17, learned-PPO 6, heuristic sequential 8) all did **worse**
  than no push. The bottleneck was end-effector geometry, not the policy.

## Act 2 — Change the object? (pipes, L-shapes, bars — all dead ends)

The user's insight: maybe the *cubes* are the problem — "they can be stacked in a
way where it's impossible to grab." So we tried shapes that are "always grabbable":

- **Straight pipes** → roll, slip from the grip during carry. 0 net placed.
- **L-shapes** (compound multi-geometry body) → the friction grasp/weld won't hold
  a multi-Group body at all; z never changed. 0 lifts.
- **Flat bars** (thin asymmetric box) → the jaw never contacts the thin part.

Every shape traded one failure for another. The lesson forming: don't fight the
*pack* — change how you *grip*.

## Act 3 — Suction

A vacuum cup doesn't need clearance around a part. It grips the **top face** from
directly above and lifts. Packing, rolling, and jaw geometry stop mattering: the
pile empties top-down, cube by cube, with no shaking.

v1 (`dab181a9`) modelled the cup as a kinematic **weld**: descend onto the cube's
top, weld it to the tool, lift, carry, release. It hit a deterministic 32/36 — a
real jump, and reliable. But it was built on the 2F-85: the fingers were still the
visible tool, with a tiny decorative cup pinned on top.

## Act 4 — "Make it actually real"

The user watched it in the GUI and was blunt: the cup looked fake, and the held
cube "holds apart… like a force field or magic power." Two concrete problems:

1. **The visible end-effector was still a finger gripper.** A cup stuck on jaws
   reads as jaws.
2. **The cube floated ~45 mm below the tool with an air gap** (the weld recorded
   the descent gap, ~0.045 m). It looked levitated, not gripped.

The fix was to build an *actual vacuum tool*:

- **`omniarm6_suction.urdf`** = the arm-only `omniarm6.urdf` + a flared rubber suction-cup
  tool link on the flange (metal adapter → dark neck → black bellows → wide lip).
  **No fingers.** It is **visual-only** (no `<collision>`) so it carries no collider
  and can't knock the pile — the "grip" is the weld, so the cup needs no contact.
  The cup lip sits at the same tool point (`OZ=0.25`, flange + 0.085 m) the 2F-85
  throat used, so every existing grasp pose stayed valid.
- The world + `gen_bin_picking_world.py` point at the new URDF; the bridge runs with
  `gripper_id=None` (pure IK — every `act_grasp/open/width` is a None-safe no-op).
- **Flush hold:** the grabbed cube now rides exactly one half-cube (`CUP_HOLD_DZ =
  0.025 m`) below the lip, so its TOP FACE is against the cup — no gap. It's fixed,
  not measured from the descent, so a slight arm undershoot can't reopen a gap; the
  vacuum simply snaps the cube up onto the cup.

First GUI run: clean and convincing.

## Act 5 — The bounce that only appeared after a reload

Then the user reloaded the world (Ctrl+Shift+R) and the cube started **bouncing /
jittering** under the cup. The exact same controller, the same world — perfect on
the first load, bouncy after a reload. Why?

The held cube is a **dynamic** (physics) body that we teleport every tick to follow
the cup. We were setting its *position* but never its *velocity*. So it kept
accumulating velocity — gravity pulling down, plus the arm's own motion — and the
solver fought our per-tick teleport. The visual is a vibration.

Why only after a reload? **A fresh process is a COLD load**: the Newton/MuJoCo
solver is sluggish and quietly damps the accumulated velocity. **Ctrl+Shift+R
warms the solver** (stiffer, more responsive), so the same velocity now shows up as
a bounce. This is the *opposite polarity* of the usual cold-first-load trap (where
cold UNDER-tracks and a reload fixes it; see
`real-grasp-and-the-cold-first-load-trap.md`). Here cold **hid** a bug.

**The fix is one line, applied every tick while a cube is held:**

```python
_node.setVelocity([0.0] * 6)   # pin: no residual velocity -> no bounce
```

That turns the carry into a true kinematic hold — no velocity to accumulate, no
fight with the solver — clean in cold and warm states alike.

## The payoff

Killing the bounce wasn't only cosmetic: the bounce had been knocking cubes off the
cup mid-carry. With it gone, the warm result jumped **31/36 → 36/36**. A stale
metric was also masking it — `emptied` was measured *before* the recovery pass, so
a genuine 36/36 run printed `FAIL`; recompute it from the final scene.

Final, verified with `scripts/dev/run_bin_picking_parallel.ps1`:

```
[1..5] emptied=36/36 sorted=36/36 box_empty=True remaining=[] | sim=780.0s   (identical)
5 / 5 matched the expected sorted=36/36
```

The whole bin, every cube in the right tray, **byte-identical sim time across all
five parallel instances** — deterministic and repeatable, exactly the bar.

## Lessons (the part to actually remember)

1. **When the grip can't, change the end-effector, not the object.** Pipes,
   L-shapes and bars each swapped one failure for another. Suction defeated the
   whole class of pack/roll/clearance problems at once.
2. **A kinematically-carried dynamic body must have its velocity zeroed every
   tick**, not just its position. Position-only tracking accumulates velocity that
   fights the teleport. `setVelocity([0]*6)`. This is latent in *any* weld/carry
   (e.g. the bin-tilt `_apply_hold`).
3. **A cold solver can hide a real bug.** "Works on first load, breaks after
   reload" is a signal, not noise — the warm (reloaded) state is the honest one.
   Test the warm path; the headless runner reloads by default for exactly this.
4. **Make the demo, not just the mechanism, look real.** "It works" and "it reads
   as the real thing" are different acceptance bars; the user holds the second one.
5. **Recompute success metrics from the final scene.** A counter measured mid-run
   will lie once a later stage changes the world.

## Reference

- Real suction tool: `projects/robots/omnisim/omniarm6/omniarm6_suction.urdf` (visual-only
  cup, no fingers; cup lip at flange + 0.085 m).
- Controller knobs: `CUP_HOLD_DZ` (flush offset), `OZ` (tool point = lip),
  `CARRY_Z`. Bridge runs `gripper_id=None`.
- Launch (GUI): `projects/samples/demos/worlds/flagship/run_omniarm6_bin_picking.ps1`.
  Determinism: `scripts/dev/run_bin_picking_parallel.ps1` (expects `sorted=36/36`).
- Agent-launched GUI note: `OMNISIM_NO_WARMUP=1` persists a detached launch; the
  warm-up reload closes a detached GUI at ~30 s (fine when a human launches it).
