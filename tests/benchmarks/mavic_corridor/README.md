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

Use one engine process per flight. The mission script connects on the bridge's
first answer, waits for the airframe to come to rest, and refuses to fly if it
has settled more than 0.15 m from the pose the world authors.

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

On a build with the integral position hold, three flights of the `none`
condition from the authored start clear the shelving by 0.17-0.21 m of hull and
complete 3/3. The two failure signatures this benchmark exists to catch:

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
