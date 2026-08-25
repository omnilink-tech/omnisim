# Warehouse line measurement

`measure_line.py` turns "we optimised the warehouse demo" into a claim you can
check. It polls the three OmniLink bridges in
`projects/samples/demos/worlds/flagship/warehouse_omnilink.omniworld` over loopback
HTTP and reports throughput, cycle time, arm idle-wait, per-stage durations,
and — the metrics this work is actually aimed at — **per-tug path length and
total rotation**.

Run it before a change and after a change, and diff the JSON. Without a
baseline there is nothing to diff, and no optimisation can be verified.

---

## Launch the world

```bash
python scripts/dev/headless_runner.py \
    projects/samples/demos/worlds/flagship/warehouse_omnilink.omniworld \
    --gui --realtime --duration 1800
```

`--realtime` matters: without it the runner passes `--mode=fast`, and boxes/min
stops being a wall-clock rate. Drop `--gui` for a windowless run. Set
`--duration` to cover your whole measuring session.

> **Do not launch this world with `launch.bat` if you care about the chat
> layer — FIXED 2026-07-28, but know the shape.** `launch.bat` used to
> *prepend* `msys64\mingw64\bin\newton-runtime` to `PATH`. The engine spawns
> controllers with the bare command `python`, so the bridges ran on the bundled
> physics interpreter, which has no `omnisim_bridges` — the deferred-intent tool
> layer and the shared status/resume intents then fell back to their
> "package absent" stubs behind a bare `except Exception`, silently.
> `launch.bat` and `omnisim/dev/runner.py` (`webots_env()`, which backs
> `python -m omnisim run-headless` / `run-world`) now **append** that directory,
> so a system Python wins; the bundled one remains a last resort for a box with
> no Python. Newton is unaffected — its interpreter comes from `python312.dll`
> in the binary's own directory plus the `python312._pth` beside it, never from
> `PATH`. `measure_line.py` itself only reads `/state` and was unaffected either
> way; `bench_omnilink.py` refuses a relay-less run outright — see
> [`BENCH_OMNILINK.md`](BENCH_OMNILINK.md) §2.1. Still trapped:
> `python -m omnisim run-agent`. Invoking `scripts/dev/headless_runner.py`
> directly, as above, was and remains safe.

The autonomous loop needs no LLM, no key and no network. Give it ~15 s after
launch for the three bridge controllers to boot and arm their idle loops, then
start measuring. The bridges bind loopback only:

| Robot | Bridge | Port |
|---|---|---|
| `OMNIARM6` pick arm | `omnilink_arm_bridge` | `8765` |
| `TUG_A` dispatch tug | `omnilink_mobile_bridge` | `8766` |
| `TUG_B` return tug | `omnilink_mobile_bridge` | `8767` |

## Run the harness

```bash
# Ten minutes at the default 2 Hz, JSON to a file, human summary to stdout.
python tests/benchmarks/warehouse/measure_line.py \
    --duration 600 --label baseline \
    --out tests/benchmarks/warehouse/results/baseline.json

# After the change.
python tests/benchmarks/warehouse/measure_line.py \
    --duration 600 --label "kit-to-demand" \
    --out tests/benchmarks/warehouse/results/after.json

# As a gate: fail unless the run was long enough to see 5 complete cycles.
python tests/benchmarks/warehouse/measure_line.py --duration 900 --min-cycles 5 --out ...
```

The pure math (angle unwrapping, path integration, run segmentation, counter
intervals) is unit-tested with synthetic data and needs no simulator:

```bash
python tests/benchmarks/warehouse/measure_line.py --selftest
python tests/benchmarks/warehouse/test_measure_line.py     # same thing
python -m pytest tests/benchmarks/warehouse/test_measure_line.py
```

Ctrl-C stops early and still prints and writes a report for what it collected.

### Exit codes

| Code | Meaning |
|---|---|
| `0` | measured cleanly |
| `2` | a bridge did not answer preflight, or the loaded world has no `line` block (it is not the warehouse) |
| `3` | a bridge stopped answering mid-run — partial JSON is still written |
| `4` | bad arguments |
| `5` | `--min-cycles` not met: the run was too short to conclude anything |

### Useful flags

| Flag | Default | Why you would change it |
|---|---|---|
| `--duration` | `600` | Cycle stats need cycles. At ~100 s per box, 600 s buys ~5. |
| `--hz` | `2.0` | Poll rate per bridge. **Do not go below ~1.5 Hz** — see *Rotation* below. |
| `--min-cycles` | `0` | Turn the run into a pass/fail gate. |
| `--no-raw` | (raw on) | Drop the embedded sample series. ~1 MB per 10-minute run at 2 Hz. |
| `--label` | `""` | Free text stored in the JSON, e.g. the commit you are testing. |
| `--pos-deadband-m` | `0.001` | Per-sample translation below this is treated as float noise. |
| `--yaw-deadband-deg` | `0.02` | Per-sample rotation below this is treated as float noise. |
| `--stationary-speed-m-s` | `0.02` | Below this **and** the yaw threshold, a tug counts as stationary. |
| `--stationary-omega-deg` | `1.0` | deg/s below which a tug counts as not turning. |
| `--min-cycles`, `--token`, `--host`, `--*-port`, `--timeout` | | parallel runs, tagged runs, non-default ports |

---

## What the harness reads

Everything comes from **`GET /state`** on each bridge. Nothing else is called
except one `GET /capabilities` per tug at preflight (to read the published
`max_angular_rad_s` yaw-rate ceiling).

**`GET :8765/state`** — OMNIARM6, and the authoritative record of the line:

```jsonc
{ "id": "omniarm6", "model": "OmniArm 6", "sim_time": 812.48, "mode": "...",
  "idle_loop": { "mode": "pick", "picks": 41,
                 "leg": "idle|pick|place|respawn", "paused": false },
  "line": { "active": true,
            "fill_box": "BOX_2",
            "fill_state": "belt|at_fill|filled|loading|loaded",
            "placed": 2, "target": 3, "remaining_in_box": 1,
            "queued": 2, "loaded": null,
            "in_transit": [ { "box": "BOX_1", "trolley": "TROLLEY_C",
                              "delivered": false } ],
            "boxes_filled_total": 13, "shipped_total": 12,
            "boxes_on_line": 3, "loads_out": 1 } }
```

**`GET :8766/state`, `GET :8767/state`** — the tugs:

```jsonc
{ "id": "tug_a", "model": "OmniTug 500",
  "x": 6.42, "y": -3.10, "yaw": 1.5708,      // radians, wrapped to (-pi, pi]
  "v_linear": 0.6, "v_angular": 0.0,          // the COMMAND, not a measurement
  "sim_time": 812.44, "carrying": "TROLLEY_C" | null,
  "towed": { "def": "...", "x": .., "y": .., "yaw": .., "artic_deg": .. },
  "idle_loop": { "leg": "idle|docking|holding|to_park|to_collect|to_dock_e|
                         back_aisle|returning|lane_fetch|lane_conveyor|
                         stage_to_station|shuttle_in|shuttle_out",
                 "mode": "dispatch|trolley_return",
                 "cycles": 7, "jobs_total": 9, "delivered_total": 7,
                 "paused": false, "holding": false, "holds_total": 2,
                 "hold_secs_max": 4.5, "park_row_count": 4, "cart_xy": {…} } }
```

`GET` is used deliberately and exclusively. **Any POST outside a bridge's read
allowlist calls `note_external_command` and pauses that robot's idle loop for
~60 s** — a measurement tool that pauses what it measures is worthless. `GET`
never arms the pause. The harness also writes exactly one file, the one you
name with `--out`; it never opens a `.wbt`, a controller, or a log.

---

## What each metric means

### Throughput

| Metric | Definition |
|---|---|
| `boxes_shipped` | Total increments of `line.shipped_total` inside the window. A box "ships" ~12 s after its cart reaches a park spot: the box recycles to the belt entry and its parts respawn on the feeder. This is the line's completed-unit counter. |
| `boxes_per_minute` | `boxes_shipped / observed_wall_minutes`. **The headline number.** |
| `boxes_filled` / `fills_per_minute` | Increments of `line.boxes_filled_total` — the arm reaching its part target. Upstream of shipping, so it moves first when the *pick cell* speeds up. |
| `picks` / `picks_per_minute` | Increments of `idle_loop.picks` — individual parts placed into a box. |

### Cycle time

Seconds between **consecutive** increments of a counter.

`cycle_time_s.shipped` is the per-box cycle; `.filled` is the fill cycle;
`.pick` is per part. Each reports `n / mean / median / p95 / min / max`.

The lead-in (window start → first increment) and the tail (last increment →
window end) are **excluded**: they are partial cycles and including them drags
every mean toward zero in proportion to how short the run was. So **N intervals
requires N+1 increments** — a 5-minute run may report `n=2`. `p95` on `n<20` is
barely distinguishable from `max`; the summary prints `n` beside it for that
reason.

### Arm idle-wait

Two different numbers, both wanted:

| Metric | Definition |
|---|---|
| `fill_blocked_per_cycle_s` | Duration distribution of the `filled` stage: the box has hit its part target and is **waiting for an empty cart**. This is the 40–65 s blemish `WAREHOUSE_OMNILINK.md` describes, and the number an optimisation should move. |
| `fill_blocked_total_s`, `fill_blocked_frac_of_run` | The same, aggregated — what share of the whole run the line spent blocked. |
| `arm_not_working_s` | Wall seconds with `idle_loop.leg` in `{idle, respawn}` and `paused == false`: the arm's own loop reporting that it has nothing to do. |
| `arm_paused_by_operator_s` | Wall seconds with `idle_loop.paused == true` — someone was talking to the robot. **Should be 0 in a clean measurement run**; if it is not, the run is contaminated. |

### Per-stage durations

A box's journey, reconstructed from `line.fill_state` for the front box and
`line.in_transit` for boxes already on a cart:

| Stage | Meaning |
|---|---|
| `belt` | riding the inbound belt toward the fill stop, or queued behind |
| `at_fill` | stopped at `FILL_STOP`; the arm is filling it |
| `filled` | full, waiting for an empty cart at the fill station |
| `loading` | gliding down the outfeed spur onto the cart deck |
| `loaded` | set in the same lock that pops the box off the queue, so it is essentially never *observable* as `fill_state`; expect `n=0` |
| `towing` | on a cart, `tug_a` hauling it to a park spot (`in_transit`, `delivered=false`) |
| `at_dispatch` | parked; the ~12 s ship timer is running (`in_transit`, `delivered=true`) |

Each reports `n / mean / median / p95 / total_observed_s / censored_runs`.

### Per-tug motion — the headline metrics for this work

| Metric | Definition |
|---|---|
| `path_length_m` | Sum of straight-line distance between consecutive pose samples. Under-reads curvature by O(chord error): at 2 Hz and 0.6 m/s the chord is 0.3 m, so a tight turn loses a percent or two. **Consistent across runs**, which is what a before/after diff needs. |
| `net_displacement_m` | Straight line from the first sample to the last. |
| `path_efficiency` | `net / path`. Low is normal for a tug doing loops; it is only meaningful compared against another run of the same duration. |
| `rotation_deg` | **Sum of \|per-step rotation\|** — every degree the tug turned, either direction. The "unnecessary rotation" number. |
| `net_rotation_deg` | **Signed** sum. |
| `rotation_efficiency` | `\|net\| / total`. A tug that turns 90° left then 90° right scores total 180°, net 0°, efficiency 0 — all of that rotation was wasted. |
| `rotation_deg_per_m` | Degrees turned per metre travelled. The cleanest single "is the route twitchy?" scalar, and it normalises out run length. |
| `stationary_s`, `stationary_frac` | Time with pose-derived speed below `--stationary-speed-m-s` **and** yaw rate below `--stationary-omega-deg`. A tug spinning on the spot is *not* stationary. |
| `by_load` | Path / rotation / time split by `carrying` — loaded vs deadheading. |
| `by_leg` | The same, split by `idle_loop.leg`, so you can see *which leg* of the choreography spends the rotation. |

Speeds are derived from **poses**, never from the reported `v_linear` /
`v_angular`. The OMNITUG500 is kinematic and its reported velocity is the *command*
— exactly the number a route change could make look good without the robot
moving any less.

### Rotation: how the angle unwrapping is handled

This is the single easiest thing to get wrong, so it is stated explicitly.

Both bridges publish `yaw` already **wrapped into (-π, π]** (`_read_pose`
returns `wrap_pi(yaw - yaw_offset)`). A tug turning steadily through north
emits `… 3.10, 3.14, -3.14, -3.10 …`. Summing naive differences scores that
0.08 rad step as **−6.20 rad — 355° of rotation that never happened**, once per
crossing. Worse, a tug *parked* facing north with float jitter across the
boundary accumulates 360° per dither while standing perfectly still.

The fix is to **wrap the difference, not the angles**:

```python
step = (cur - prev + pi) % (2*pi) - pi     # -> (-pi, pi]
total_rotation = sum(abs(step) for each consecutive pair)
net_rotation   = sum(step for each consecutive pair)
```

Three consequences, all handled and all reported:

1. **The assumption.** This is correct only while the true rotation between two
   samples is **less than π**. Above that, direction is unrecoverable from
   heading alone (a +190° turn and a −170° turn produce an identical sample
   pair) and the metric silently *under*-reports. The OMNITUG500's ceiling is
   `v_max·r/ht = 10.0 × 0.10 / 0.30 = 3.33 rad/s`, so the poll rate must exceed
   `3.33/π = 1.06 Hz`. The 2 Hz default clears it by 1.9×. Each tug's
   `unwrap_audit` block reports `bridge_max_yaw_rate_rad_s`, `nyquist_min_hz`
   and the `nyquist_margin` for the rate **actually achieved**, and the run
   warns if that margin drops below 1.5.
2. **Near-limit steps are counted, not hidden.** `aliasing_suspect_steps`
   counts individual steps past 90° and `max_yaw_step_deg` reports the worst
   one. Non-zero is not automatically wrong — it means you were within 2× of
   the wrap limit and should re-run at a higher `--hz` before trusting the
   figure.
3. **Deadbanding is one-sided.** The `--yaw-deadband-deg` floor is applied only
   to the *absolute* sum, so a parked tug does not accumulate float noise;
   `rotation_deg_raw` reports the undeadbanded value so the noise floor stays
   visible instead of being silently chosen for you. **Net** rotation is never
   deadbanded — dropping steps from a signed sum would bias it.

The behaviour is pinned by unit tests in `test_measure_line.py`: a full
revolution scores 360° (not 0, not 720), a parked tug dithering across ±π
scores <0.001°, +90°/−90° scores total 180° / net 0°, and the 200°-in-one-step
aliasing case is asserted to under-report **and** to raise the suspect counter.

### Timing conventions

- **Durations are wall-clock seconds**, measured by the poller.
- **`realtime_factor`** is reported per bridge from its `sim_time`
  (`sim seconds / wall second`). Every wall-clock number here is only
  comparable across runs **at the same realtime factor** — a "20 % faster line"
  that is really a 20 % faster simulator is exactly what this catches. The
  demo's stated baseline is ~0.95–1.0× on an RTX 3060 laptop.
- **`stalled_wall_s`** is wall time during which `sim_time` did not advance
  (paused sim, or a stalled controller).
- **A transition is placed at the midpoint** of the two samples that bracket
  it. That is the zero-bias choice: residual error per boundary is uniform on
  ±½ a poll period instead of always late by up to a full one. At 2 Hz that is
  ±0.25 s per boundary.
- **Runs touching either end of the window are `censored`** — we joined or left
  them part-way through. Censored runs are excluded from every duration
  *distribution* and included only in `total_observed_s`. The count is reported
  next to each stage.

---

## What this does NOT measure

Be blunt about this before quoting any number from it.

- **Not the physics.** It reads what the bridges *publish*. If a cart is
  clipping through a wall, if a box is resting on nothing, if the trolley
  pinning is drifting — none of that appears here. `run-headless
  --fail-on-runaway` and your eyes are the tools for that.
- **Not tug-tug interference, congestion or near-misses.** `holds_total` /
  `hold_secs_max` / `holding_frac` are surfaced because the tugs already track
  them, but there is no clearance computation and no collision check. The
  OMNITUG500 is kinematic and collides with nothing anyway.
- **Not path *optimality*.** It measures the path the tugs took. It has no
  model of the shortest legal route through the site, so it cannot tell you how
  much of the measured path length was avoidable — only whether a change made
  it longer or shorter.
- **Not sim-time durations.** Every stage and cycle figure is wall-clock.
  `realtime_factor` is reported so you can tell whether two runs are
  comparable, but the durations themselves are not rescaled by it.
- **Not the LLM/chat layer.** Latency, token cost, tool-selection accuracy and
  everything else about the conversational surface are out of scope. The demo's
  autonomous loop runs with no LLM, and that is what this measures.
- **Not a controlled experiment on its own.** Same machine, same `--duration`,
  same build, sim playing the whole time, and **nobody chatting to a robot
  mid-run** (a single prompt pauses that robot's loop for ~60 s and wrecks the
  window). Check `arm_paused_by_operator_s` and each tug's
  `paused_by_operator_frac` are zero before you believe a diff.
- **Not free of observer effect — but close.** Three `GET /state` calls per
  poll, each a loopback read of already-cached state. `poll_latency_ms` is
  reported per bridge so you can see what it cost. Do not raise `--hz` past
  what the unwrap bound needs.
- **Not statistically strong at short durations.** At ~100 s per box, a
  10-minute run yields ~5 cycles. That is enough for a mean and a median; it is
  not enough for a p95, and it is not enough to call a <20 % change. Use
  `--min-cycles` to stop yourself.
- **Not machine-attributed.** It records no GPU/CPU identity. Per this repo's
  rule, run `python projects/policies/common/env_fingerprint.py` alongside it
  and keep the two results together, or a number cannot be resolved to a box
  later.

---

## Files

| File | What it is |
|---|---|
| `measure_line.py` | The harness. Pure helpers at the top, HTTP + analysis below. |
| `test_measure_line.py` | 37 synthetic unit tests for the pure math. No simulator, no network. |
| `results/` | Suggested home for `--out` JSON (created on demand; not tracked). |

---

# Clearance measurement (`clearance_monitor.py`)

`measure_line.py` answers *how much did the line ship*. This answers a
different question: **how close did it ever come to hitting something** — and
it answers it in centimetres, with a timestamp and a named pair, instead of
the adjective "it avoids things".

```bash
# Harness running (best obstacle fidelity):
python tests/benchmarks/warehouse/clearance_monitor.py --duration 1200 \
    --out tests/benchmarks/warehouse/results/clearance.json

# No harness — static parse of the world file instead (weaker; see below):
python tests/benchmarks/warehouse/clearance_monitor.py --duration 1200 \
    --source world \
    --world projects/samples/demos/worlds/flagship/warehouse_omnilink.omniworld

# Validate the tool itself: geometry unit tests + a full pipeline run against
# a fake bridge. No simulator, no GPU, no network beyond loopback.
python tests/benchmarks/warehouse/clearance_monitor.py --selftest
```

## THIS IS A GEOMETRIC MEASUREMENT, NOT A PHYSICS RESULT

The OMNITUG500 tugs are **kinematic**: no collider, no mass, no Newton body.
Nothing in the physics engine can stop one. Collision-free operation is
produced entirely by the navigation layer in `omnilink_mobile_bridge.py`
(swept oriented-footprint SAT tests against obstacles harvested by walking the
live scene at startup, plus peer yielding and a drawbar-angle clamp).

So a reported clearance of 8 cm means *the navigator drove a footprint that
came within 8 cm of a shape*. It does **not** mean the physics kept them
apart — had the navigator driven straight through, the physics would not have
objected and this tool would simply have printed a negative number. Every
figure here is evidence about the **navigation layer**, and must never be
quoted as "the physics prevented a collision". The tool prints this in its own
output for the same reason.

That is also exactly why the measurement is worth making: when nothing can
physically stop the vehicle, an independent geometric audit is the only
evidence there is. The monitor never consults the navigator's own belief about
where it was safe — it reads published pose and applies its own geometry.

## What it measures

Every sample (default 10 Hz, `GET` only — a POST would arm the ~60 s idle-loop
pause and perturb the run) it computes the signed 2D separation for:

| Category | Pair |
|---|---|
| `tug_vs_static` | each tug against every static structure rectangle |
| `tug_vs_tug` | tug_a against tug_b |
| `tug_vs_cart` | a tug against a cart it is **not** towing |
| `cart_vs_static` | a towed cart against static structure |
| `tug_vs_own_tow` | *diagnostic*: a tug against the cart it **is** towing |

**Clearance** is `> 0` for the true minimum distance between the two
boundaries, `0` for touching, and `< 0` for `-penetration depth`. **Any
negative value is an intersection** and is reported at the top of the summary
with full detail; the process then exits `6`, so a demo rehearsal can be
gated on it.

`tug_vs_own_tow` is excluded from the headline and from the exit-code gate.
A tug and the cart on its drawbar are close on purpose and can legally touch
on a tight turn; folding that in would replace the real signal with a constant
and fail the gate on a designed condition. It is still measured, still in the
category table, and an actual overlap is still surfaced — as
`own_tow_overlaps`, under its own name, because it is a jackknife question
rather than a collision one.

**Oriented boxes (SAT), not AABBs and not circles.** The SAT routine is
*ported* from `MobileBridge._obb_clearance`, not reimplemented — two
implementations of the same test would eventually disagree with no way to tell
which was right. The oriented part is not fussiness: the bridge's own
`_obb_vs_aabb` comment records that the axis-aligned shortcut "cost a whole
measurement run", reporting the tug inside a conveyor kerb while its true
footprint was 0.45 m clear. `--selftest` pins a hand-computed case where the
two methods disagree by construction (see below).

**Measured footprints, not round numbers.** OMNITUG500 `0.7162 x 1.2595 x
0.2963 m` (union of its 8 STL meshes; mesh centred on origin in X/Y, bottom
flush at `z=0`, local `+Y` forward). Trolley collider `0.70 x 0.70 x 0.325`.
Both are in one labelled constants block with provenance, and both are
overridable (`--tug-footprint`, `--cart-footprint`). The published
`/state.yaw` is the **heading** (the `+Y`-forward mesh offset is already
removed by `_read_pose`), so the long axis lies along it — a 90° error here
would corrupt every number, and it is asserted in `--selftest`.

## The two obstacle sources are not equally trustworthy

The report always names the source it used, and says what that source cannot
see.

- **`harness`** (preferred) — `GET /scene/tree?bounds=1` walks a **live scene
  graph**, so it sees mesh bounds, PROTO expansions and URDF geometry. Caveat:
  the harness runs its own simulator process, so unless it is driving the demo
  the poses are the world's *authored* ones. That is identical to live for
  immobile structure, and wrong for anything that moves — which is why mobile
  DEFs are excluded and carts are tracked from bridge telemetry instead.
- **`world`** (fallback) — a static `.wbt` parse. **Weaker.** It sees only
  `Box`/`Cylinder`/`Capsule`/`Sphere`/`Plane` primitives written literally in
  the file: no PROTO expansion, no mesh reads, no `URDFRobot` interiors. It
  therefore **under-counts obstacles and over-states clearance**, and the
  summary says so on every run.

Measured on `warehouse_omnilink.omniworld`, the static parse resolves 148
rectangles across 47 bodies **and leaves 15 nodes unchecked — including all 11
`Wall` PROTOs of the building shell.** Nodes whose geometry cannot be
resolved are **listed by name** (DEF-less ones tagged with their authored
position, e.g. `<Wall @ -15.00,+1.00>`), never silently dropped: a clearance
number that quietly excluded the walls is worse than no number.

**Visual-only props are included as obstacles on purpose.** Most of this
world's bodies are non-collidable, but the demo's claim is that it never
*looks* like it clips anything, so a prop with no `boundingObject` is still
something the tug must not drive through on camera.

Obstacles set aside by the height rules are reported too, by name and rule:
`driven_over` (top at or below `--drive-over-z`, default 0.06 m — the
OMNITUG500's ground clearance, cited from the navigator's `DRIVE_OVER_Z`; floor
decals and in-floor drag-chain conveyor decks land here) and `overhead`
(bottom at or above the body's own height).

## Output and exit codes

Human summary to stdout, machine JSON to `--out`: headline minimum with pair,
sim time, wall time and world position; a per-category `min` / `p1` / `median`
table; and the excursion runs below `--warn-m` (default 0.15 m) as a time
series.

| Code | Meaning |
|---|---|
| 0 | measured cleanly, no intersection |
| 2 | a bridge did not answer preflight, **or does not publish `yaw`** |
| 3 | a bridge stopped answering mid-run (partial JSON still written) |
| 4 | bad arguments |
| 5 | no obstacle source could be resolved |
| 6 | **an intersection occurred** — the gate |
| 7 | `--selftest` failed |

An oriented footprint needs a heading, so a bridge that publishes no `yaw` is
a **hard stop** rather than a silent downgrade to an axis-aligned box — that
substitution is the documented cause of a phantom incursion in the navigator's
own history. If yaw goes missing mid-run those samples are excluded and
counted in `not_measured`.

## What it does NOT measure

- **Carts that are not currently under tow.** A bridge publishes only the cart
  it is towing, so parked, staged and conveyor-borne trolleys have no live
  pose: they are neither obstacles nor measured bodies.
- **The OMNIARM6 pick cell.** The arm is a `URDFRobot` whose links move, so a
  startup snapshot would be a stale obstacle; it is excluded by the mobile-DEF
  rule and no world-space link poses are published to replace it. Clearance
  between a tug and the arm or its pedestal is therefore not measured. (The
  navigator handles static-base machines with a declared keep-out box; this
  tool has no equivalent yet — that is the obvious next increment.)
- **Vertical clearance.** Every number is a 2D separation in XY, gated by a
  z-overlap test. It is not a 3D distance.
- **Anything between polls.** At 10 Hz a tug at 1 m/s moves 100 mm between
  samples, so a transient closer approach can fall between two readings.
- **Pairs outside the broad phase.** Only pairs within `--broad-m` (default
  3.0 m) of contact are recorded, so the percentiles describe near-field
  geometry, not the whole scene.
- **Rotated obstacles exactly.** Obstacle rectangles are world-space
  *axis-aligned* envelopes (that is what both sources provide), so a rotated
  obstacle is over-sized and clearance against it is under-stated — the safe
  direction for a safety metric, but not exact.

## Validation (`--selftest`, 12 checks, no simulator)

SAT math against hand-computed cases: a **touching** pair returns exactly zero
and is *not* flagged as an intersection; an **overlapping** pair returns
`-0.2` penetration depth; a disjoint corner pair returns `√2`; symmetry and
self-intersection hold.

**The AABB-vs-OBB divergence case** — the one that justifies the whole
approach. A OMNITUG500-sized footprint at a **27° heading** beside a 0.6 m corner
post at `(0.55, 0.55)`:

| Method | Answer |
|---|---|
| Oriented (SAT, used here) | **+0.10999836 m clear** |
| Axis-aligned envelope (rejected) | **overlap** (negative) |

The two disagree by more than 0.1 m and *in the direction that matters*: an
AABB monitor reports a collision that never happened. The expected value is
derived independently in the test (the post's near corner projected onto the
heading unit vector, minus the footprint half-length) and asserted to 1e-9.

Then the whole pipeline runs against a **fake bridge** emitting a scripted
trajectory past a known obstacle, with the answers hand-computed in advance:

| Check | Expected | Result |
|---|---|---|
| tug vs wall (drives to x=1.0, wall at x=2) | `2.0 - (1.0 + 0.62975)` = **0.37025 m** | matches |
| tug vs tug (tug_b parked at y=4) | `4.0 - 2 x 0.3581` = **3.2838 m** | matches |
| towed cart vs wall | **1.1500 m** | matches |
| drawbar penetration depth | `-(0.62975 + 0.35 - 0.5)` = **-0.47975 m** | matches |
| driving into the wall | exit **6**, intersection reported | matches |
| own-tow overlap | exit **0**, `own_tow_overlaps > 0` | matches |

Also covered: the static `.wbt` parser (a 45°-spun 2x2 box must have a
`2√2` envelope), the z filters (a decal, a roof beam and a degenerate box are
each dropped by the right rule and named), and the harness source against a
synthetic `/scene/tree` (Shape-level bounds preferred over the coarser union,
mobile DEFs excluded, un-bounded nodes reported unchecked, and
`bounds_included: false` raising rather than yielding a silently empty
obstacle set).

| File | What it is |
|---|---|
| `clearance_monitor.py` | The clearance harness. Geometry + sources at the top, collection and report below; `--selftest` at the bottom. |
