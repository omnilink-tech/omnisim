# P6 damage-event parity captures

Quantitative parity captures for the P6 damage-suite row of
[engine-migration-plan.md §13.3](../engine-migration-plan.md). The
companion harness lives in `scripts/dev/damage_events_capture.py`
(stream-to-JSONL) + `scripts/dev/damage_events_diff.py` (parity
report).

## Captures

| File | Backend | World | Duration | Events |
|---|---|---|---|---|
| [p6_ode_20260528.jsonl](p6_ode_20260528.jsonl) | ODE | `projects/robot_combat/worlds/tests/husky_head_on_ode.wbt` | 90 s wall | 149 |
| *(not kept — see below)* | Newton | `projects/robot_combat/worlds/tests/husky_head_on.wbt` (auto → Newton) | 90 s wall | 0 |

The 2026-05-28 Newton capture emitted **zero** events (the chassis-freeze bug
below), so its JSONL was a zero-byte file carrying no data the table does not
already state. It is not kept in the tree; the finding it produced is recorded
in full below, and the bug it exposed was fixed and re-captured on 2026-05-29.

## How to re-run

```bash
# ODE
python scripts/dev/damage_events_capture.py \
    projects/robot_combat/worlds/tests/husky_head_on_ode.wbt \
    --duration 90 \
    --output _scratch/p6_ode_$(date +%Y%m%d).jsonl

# Newton (host must be Phase-D-enabled so auto resolves to Newton)
python scripts/dev/damage_events_capture.py \
    projects/robot_combat/worlds/tests/husky_head_on.wbt \
    --duration 90 \
    --output _scratch/p6_newton_$(date +%Y%m%d).jsonl

# Diff
python scripts/dev/damage_events_diff.py \
    --baseline _scratch/p6_ode_$(date +%Y%m%d).jsonl \
    --candidate _scratch/p6_newton_$(date +%Y%m%d).jsonl
```

## 2026-05-28 finding

ODE produces a clean 149-event head-on (chassis = 48, top_plate = 84,
front_bumper = 15, rear_bumper = 2, sum_J ≈ 1194.8). Newton produces
ZERO events on the same world configuration.

The damage tracker is not the regression — the simulator's
`[WbNewtonBackend] step N` log lines show both husky chassis bodies
held at their starting positions (`b2=(-17.986,0,0.122)`,
`b7=(17.991,0,0.122)`) for the full 7.68 s captured, even though
the motor `target_vel reached … rad/s` lines fire. The wheels are
registered + the velocity targets are accepted but the bodies don't
translate. Without the head-on actually happening, there's nothing
for the damage tracker to register.

This contradicts the Phase D landing row's "Newton 16 events" claim;
something has regressed or the original capture was on a different
configuration. Resolving this Newton-motor-without-translation bug
is a prerequisite for closing P6 with a parity ratio. Tracking
under the broader humanoid-balance-gap pattern (see
[humanoid-balance-gap.md](../humanoid-balance-gap.md) for the related
Atlas + G1 case) until isolated.

## 2026-05-29 update — RESOLVED to practical parity

The "Newton = 0 events" was the chassis-freeze bug (Phase-D `"auto"`
registered the static arena/sun-marker as dynamic Newton roots, whose
per-step pose `changed` fired `resetJointsToDefaults()` on the shared
articulation every tick → every robot frozen at spawn). Fixed in
`d56cbf5` (skip no-Physics Solids). With the huskies moving, two new
issues surfaced and were fixed:

- **Head-on XPBD NaN** (`3ba5e079`): at 50/30 rad/s the head-on crash
  NaN'd the solver at step 1. `OMNISIM_NEWTON_SUBSTEPS` (default 1;
  set 4 for the capture) sub-steps the solver → finite.
- **57k jitter inflation** (`dadd50ae`): the tracker's synthetic
  `impulse_J = mass*|Δv|` proxy clears the buffer threshold every step
  on Newton because the body_qd write-back jitters. `OMNISIM_DAMAGE_VEL_SMOOTH`
  (EMA low-pass on per-body velocity; default-off so ODE is unchanged)
  removes the jitter.

**Debounced capture (50/30 head-on, 30 s):**

| File | Backend | Settings | Events |
|---|---|---|---|
| [p6_ode_raw_20260529.jsonl](p6_ode_raw_20260529.jsonl) | ODE | raw | 39 |
| [p6_newton_substep4_velsmooth5_20260529.jsonl](p6_newton_substep4_velsmooth5_20260529.jsonl) | Newton | `OMNISIM_NEWTON_SUBSTEPS=4 OMNISIM_DAMAGE_VEL_SMOOTH=5` | 58 |

Newton **57,161 → 58** vs ODE 39 — within the `damage_events_diff` 10×
tolerance (practical parity). The part *distribution* still differs
(Newton wheel-weighted from the more-violent XPBD tumble at this
unrealistic 50/30 speed; ODE bumper/plate-weighted), and exact
backend-symmetric parity is bounded by the synthetic-impulse hack — the
clean fix is an engine-side contact-impulse/depth API (ODE already
computes `dContactGeom.depth` but discards it before the controller
stream). Re-run:

```bash
OMNISIM_NEWTON_SUBSTEPS=4 OMNISIM_DAMAGE_VEL_SMOOTH=5 \
  python scripts/dev/damage_events_capture.py \
  projects/robot_combat/worlds/tests/husky_head_on.wbt \
  --duration 30 --output _scratch/p6_newton.jsonl
```

## 2026-05-29 — depth-LEVEL gating RULED OUT; `vel-smooth` stays the mechanism

With the real `ContactPoint.depth` now landed (the contact-impulse/depth API,
commit `94e07156`), the obvious next step was to retire the `vel-smooth`
velocity-proxy hack by thresholding on penetration depth instead. **Tested and
ruled out:** a depth GATE that skips any contact shallower than a threshold does
NOT fix the Newton inflation. Measured on `husky_head_on.wbt` (Newton,
`SUBSTEPS=4`, ~60 s, summary side-log):

| `OMNISIM_DAMAGE_DEPTH_GATE` | `event_counter` | `depth_gated` |
|---|---:|---:|
| (off) | 9306 | 0 |
| 0.005 m | 8340 | 965 (plateaus early) |

The gate drops only the ~965 *shallow* contacts during the **approach** phase,
then stops: during the sustained crash **every contact penetrates well past
5 mm**, so the jitter events pass the gate (per-part buffered counts are
identical to no-gate). **The inflation is velocity (`body_qd`) jitter during
DEEP sustained contact, not shallow contacts** — so a depth-*level* gate
fundamentally can't separate jitter-during-deep-contact from a real impact
(both are deep). The depth-gate experiment was reverted (no ineffective knob
shipped).

**Conclusion:** `OMNISIM_DAMAGE_VEL_SMOOTH` (low-pass the body velocity before
differencing) remains the correct mechanism and already meets the P6 10×
parity (58 vs 39). The `ContactPoint.depth` API is still the right
infrastructure for a *future* refinement, but it must be a depth-**derivative**
(Δpenetration per step → real impacts spike, sustained contact ≈ 0) or a
depth-as-magnitude re-calibration — NOT a depth-level gate. Both are larger,
lower-priority work; P6 parity is met as-is.
