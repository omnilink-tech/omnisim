# Mavic corridor benchmark

A navigation regression for the Mavic bridge: fly a fixed eight-waypoint
serpentine through a three-block warehouse corridor and measure how close the
airframe comes to the shelving. It exists because the corner-cutting introduced
with the v8.2.0 velocity-damping change was invisible to every other test in the
tree: every flight still reported eight arrivals, while the hull was clearing the
shelves by millimetres.

    worlds/corridor_scan.omniworld          the corridor scene
    worlds/corridor_scan_blocked.omniworld  the same, plus a 0.6 x 0.6 x 2.0 m
                                            blocker at (4.0, 2.0) in lane two
    controllers/mavic_omnilink_bridge/      a relay, see below
    corridor_mission.py                     flies the mission and measures it
    lane_scoring.py                         scores lane offset and recovery from
                                            kept samples, with a CLI to re-score
                                            a past campaign
    test_lane_scoring.py                    its three rules, no engine
    baseline_metrics.py                     the kinematic baseline, and the
                                            shared geometry both sides score
                                            against

The `worlds/` and `controllers/` layout is required, not decorative. A controller
is resolved against the project directory that owns the world, so a world sitting
flat in this directory cannot see `projects/samples/demos/controllers/`: the
engine falls back to `generic` without failing, the bridge never binds 6090, and
the mission script waits on a port nothing is listening to. The controller here
is a relay that execs the shipped `mavic_omnilink_bridge` rather than a copy of
it, so this benchmark tests the real bridge.

## Running

Both worlds resolve the Mavic URDF relatively and use the stock
`mavic_omnilink_bridge` on port 6090, so nothing needs installing.

    set OMNISIM_URDF_USE_SENSORS=1
    omnisim-bin worlds/corridor_scan.omniworld --batch --mode=fast --no-rendering
    python corridor_mission.py --condition none --repeat 1 --out run.json

`--condition` is `none`, `blocked` (use the blocked world) or `goal_shift`.
`--keep-trajectories` stores every sample, including yaw, mode and fault, so
every metric can be recomputed without re-flying. The kinematic baseline runs
without an engine:

    python baseline_metrics.py --runs 60 --seed 1 --solid-shelves --gate next

Use one engine process per flight, and do not run another engine anywhere on the
machine while a flight is in the air. The engine runs `--mode=fast` while the
bridge is driven over HTTP, so wall-clock latency decides how many simulation
steps pass between a waypoint arriving and the next command landing: the flight
is reproducible on a quiet machine and not reproducible on a busy one. Measured
on three consecutive `none` flights, same build, same worlds: with a second
engine running the flight collided, at 32.09 m of path in 36.37 s; alone it
cleared by 0.0973 m of hull at 33.38 m in 40.44 s, inside the band below. A
flight that comes in short and fast is this, not the controller.

The mission script connects on the bridge's first answer, waits for the airframe
to come to rest, and refuses to fly if it has settled more than 0.15 m from the
pose the world authors.

## The two clearance numbers

`min_clearance_m` is measured from the body origin. It is the like-for-like
figure against the kinematic baseline, which is a point mass, and it is the one
to compare across the two halves of the study.

`min_hull_clearance_m` scores the collider the Mavic actually has, one
0.30 x 0.08 x 0.05 m box at the body origin (the propeller links carry none, per
issue #10), as an oriented rectangle against the obstacle footprint. It is the
only one a collision can be read off: the body origin never enters an obstacle
even while the airframe is pressed against it, so a point metric reports a clean
flight through a sustained contact.

Contact is counted over the airborne part of the whole flight rather than
between the first and last waypoint arrival. An aircraft that wedges after its
last arrival has the entire wedged period outside that window, and two flights
that spent 213 s and 40 s pressed into an obstacle scored zero contact samples
before this was changed.

## Regression thresholds

On v8.4.0 with `newtonGroundMu` declared in both worlds, the `none` condition
completes 8/8 and clears the shelving by **0.084-0.109 m of hull**
(0.236-0.259 m from the body origin), over three flights on one machine and six
on another. A flight occasionally takes a wider line -- one of those nine
cleared by 0.306 m -- so a single high reading is not a pass signal on its own.
The tight spot is the same place every time: the first turn past shelf 1, at
about (-0.07, 4.23), around t = 15 s.

This band replaces a "0.17-0.21 m of hull" figure published with the benchmark
on 2026-09-08. That figure does not reproduce and is withdrawn. It was measured
before the release, and as written it would have failed every flight on the
build it was meant to gate, which is backwards for a regression threshold. Two
engine changes that landed between it and v8.4.0 were tested as explanations and
both were refuted: reverting the URDF inertia work (`OMNISIM_URDF_USE_INERTIA=0
OMNISIM_NEWTON_INERTIA_COM=0`) moves hull clearance to 0.036 m, i.e. the fix
improves clearance rather than tightening it, and reverting the wheel rotor
armature (`OMNISIM_NEWTON_WHEEL_ARMATURE_RATIO=1`, which reaches this aircraft
because the propeller motors are configured as velocity wheels) leaves it at
0.097 m. The bridge is unchanged since the benchmark landed. So quote the band
above, and attribute any new number to the machine that produced it.

The two failure signatures this benchmark exists to catch:

  - hull clearance collapsing toward zero while completion stays at 8/8, which
    is corner-cutting, not a navigation failure, and
  - a flight that stops making progress and is not reported as such.

## Lane offset, and the signature that needs it

A contact the aircraft survives is not free. It delays the return to the lane,
and an obstacle corner sitting inside that recovery distance is what collects.
Nothing in the summary metrics distinguishes this from ordinary corner-cutting:
completion stays 8/8, and minimum clearance moves only at the one corner.

`lane_scoring.py` measures it from samples already in the report, so a question
about a past campaign is answered by re-scoring rather than re-flying:

    python lane_scoring.py run.json --leg 3 --stations 0.6 0.9

Measured on v8.4.0, leg 4 ((4.0, -0.5) to (9.0, -0.5)), offset toward the shelf
side at 0.6 m and 0.9 m along it, against minimum hull clearance to shelf 2
whose near corner is at (5.0, 0.0):

    six none flights          0.164-0.190   0.067-0.094   0.260-0.306
    blocked, light graze      0.166         0.091         0.264
    blocked, hard graze       0.318-0.433   0.275-0.390   0.000-0.103

The light-graze flight is the control inside the condition: same world, same
start, same bridge, and indistinguishable from the unblocked flights at every
station. The only variable is whether contact happened.

Two things this is not. It is not a threshold: the `blocked` condition does not
deliver the same outcome mix on two machines, and where one campaign completed
6/6 another wedged three flights against the blocker, all three labelled
correctly. So the population the table describes is the sub-population that gets
past the obstacle. And it is not a claim that the flight stays off-lane, which is
the opposite of what the samples say: after the graze the later legs track the
lane at least as well as the control, and the deepest graze tracks it best, which
is what losing energy to a contact looks like.

A station is a distance along the leg, not a coordinate, and a station inside the
turn is reported as `in-turn` rather than as a number. The reason is in
`lane_scoring.py`; the short version is that a wide corner crosses a near station
three times and the three readings on one flight differed by 2.5 m.

Each reading carries two numbers, and the table above quotes `offset_m`, the
value at the first sample past the station, which is the sample `t`, `x` and `y`
describe. `offset_interp_m` is the same quantity interpolated onto the station
itself. They differ because a sample sits a fraction of a sample period past the
station, and on an aircraft recovering toward its lane that is always the low
side: over twelve v8.4.0 flights re-scored here the sample reading is 0.002 to
0.024 m low, never high. The bias is one-directional and grows with the sample
period -- at a quarter of the rate one of those readings moved 0.065 m and one
changed sign -- so quote `offset_interp_m` when comparing campaigns that were not
sampled at the same `SAMPLE_PERIOD_S`, and quote either one consistently within a
campaign.
