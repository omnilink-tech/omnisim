# X30 plant inspection — a legged robot driven by OmniLink

A Deep Robotics X30 quadruped on an inspection round of a process-plant yard,
driven entirely through OmniLink: the operator talks, the robot walks.

- **World:** [`projects/samples/demos/worlds/showcase/x30_plant_inspection.omniworld`](../../../projects/samples/demos/worlds/showcase/x30_plant_inspection.omniworld)
- **Robot bridge:** [`omnilink_quadruped_bridge`](../../../projects/samples/demos/controllers/omnilink_quadruped_bridge/) with `--locomotion trot` (port 8796)
- **Draft film (2 min 56 s, not in git):** `social/youtube_videos/omnilink/x30_plant_inspection_draft.mp4` (silent, for muted-autoplay feeds) and `x30_plant_inspection_draft_music.mp4` (quiet ambient bed, for YouTube) on the dev machine; its production notes are [`evidence/take2/PRODUCTION_NOTES.md`](evidence/take2/PRODUCTION_NOTES.md)

The site: two storage tanks in a concrete bund, a pipe rack with valve
manifold V-7, feed pump P-101 with its pressure gauge, motor-control cabinets
E-1..E-4, a control-room container, a forklift, pallets and barrels, a fenced
perimeter and a painted green walkway joining seven named places.

## What it is — and is not

| | |
|---|---|
| Legs | **Real contact physics** (Newton). A scripted trot: foot trajectories through closed-form leg IK to 12 position servos. Nothing pins or drags the body; it moves because its feet push. |
| Learning | **None in the control.** No RL policy, no neural network, no balance controller. The only ML is the language model OmniLink calls for sentences its parser does not answer; it chooses tools, it never touches a joint. |
| Where it is | The robot's pose comes from the Supervisor (standing in for leg odometry + IMU). |
| Obstacles | **Nothing senses them.** `go_to` follows the walkway graph declared in the world; `walk` goes straight. |
| Ground | Flat only (the trot is unmeasured on slopes; the crawl preset is the one proven on rough ground). |
| Gauges | Not read. There is no camera on the robot. |

Say this wherever the footage is shown.

## Run it

```bash
python -m omnisim key                     # an OmniKey is required for /prompt
python -m omnisim run-world projects/samples/demos/worlds/showcase/x30_plant_inspection.omniworld
```

Right-click the robot → *Show Robot Window* and type, or use HTTP:

```bash
curl -s -X POST http://127.0.0.1:8796/prompt -H "Content-Type: application/json" \
     -d '{"text": "Head over to the feed pump and check on it.", "timeout_s": 300}'
```

Orders that work (all verified live, see [`evidence/`](evidence/)):

| Say | What happens |
|---|---|
| `Walk forward 2 metres.` · `turn left 90 degrees` | Parser, no model. Measured walk / turn. |
| `Head over to the feed pump and check on it.` | Model → `go_to(pump P-101)` along the walkway |
| `Go to the tank.` | It asks: T-1 or T-2? `T-2, please.` is understood in context |
| `Stop.` (mid-walk) | Skips the queue, fades the gait out, reports a measured standstill |
| `Actually, change of plan: go to the control room instead.` (mid-walk) | Halts first, then goes |
| `Inspect tank T-1, then return to the dock.` | Two `go_to` calls in order; home is the dock |
| `Where are you?` | Answered from the measured pose, never moves |
| `walk forward 80 metres` · `go to the boiler house` | Declined |

Places: `charging dock`, `control room`, `pump P-101`, `tank T-1`, `tank T-2`,
`electrical cabinet E-3`, `valve manifold V-7`.

⚠️ Phrase sequences with **"then"**. The safety gate reads "…when you are
done" as an order that waits on a future event and refuses the whole
sentence (`deferred`).

## How the robot is controlled

```
operator sentence
  └─ POST /prompt ── OmniLink parser (exact orders) ── or ── OmniLink model relay (the rest)
                            └───────── safety gate vets every tool call ─────────┘
  └─ bridge verbs: walk{distance} · turn{angle_rad} · go_to{place} · walk_to{x,y} · stop · stand · sit
  └─ CrawlDriver (_crawl_motion.py): walkway routing (shortest path, via points),
     heading hold, stop leads from the measured speed, correction passes, pivot hold
  └─ Deep Robotics gait model (deep_robotics_crawl.Gait): trot foot paths → leg IK → 12 servo targets
  └─ Newton physics
```

Every motion verb blocks until the robot has stopped and reports what was
**measured** (`achieved`, `error`, `settled`), never the number it was asked.

## Measured (dev laptop, 2026-09-26)

| Result | Evidence |
|---|---|
| Trot (1.1 m/s stride at 2.2 Hz) delivers ~0.45 m/s; the crawl ~0.22 m/s | `evidence/probe/results/sw_*.jsonl` (sweep; `sw_t_*` = trot) |
| 20 mixed orders over 264 s of sim, never tipped (body up-z ≥ 0.996) | `evidence/probe/results/trot_endure.jsonl` |
| Pivot hold: in-place drift 25 cm → 7.5 cm per turn, turns within 0.5° | `evidence/probe/results/trot_pivot.jsonl` |
| Final driver: 9 orders settled, 0 corrections, walk-to within 6–8 cm | `evidence/probe/results/trot_v3.jsonl` |
| To P-101 along 11.6 m of walkway (in the film take): 33.8 s of sim, 6.8 cm from target | `evidence/take2/events.jsonl` (reply 2, `last_motion`) |
| Ambiguity, stop mid-walk (0.76 s), change of plan, declines | `evidence/live_checks/scen1.jsonl`, `scen2.jsonl` |
| The whole film take: 8 prompts, replies verbatim, measured arrival errors 7 cm | `evidence/take2/events.jsonl`, `frame_poses.jsonl` |

The probe files were written while the driver was being tuned: the earlier
ones (`r2`, `sw_*`, `trot_c1`, `trot_endure`) predate the pivot hold and the
final stop leads; `trot_pivot` and `trot_v3` are the committed behaviour.
Re-run any of them with the rig in [`evidence/probe/`](evidence/probe/):
set `controllerArgs` in `worlds/lab.omniworld` (`--gait`, `--vx`, `--freq`,
`--wz`, `--seq "walk:2,turn:90,goto:1:1:0"`, `--out <file>`) and
`python -m omnisim run-headless <that world> --duration 60`.
[`evidence/live_checks/scenarios.py`](evidence/live_checks/scenarios.py)
replays the OmniLink checks against a running world.

## Files here

| File | What |
|---|---|
| [`generate_world.py`](generate_world.py) | Builds the world and its sign textures from one site table (places, walkway, scene share coordinates). Reproduces the committed world byte for byte. |
| [`film/take.py`](film/take.py) | Records a take: the prompt list, the camera plan, every reply logged |
| [`film/edit.py`](film/edit.py) | Cuts the take into the minimal OmniLink style ([OMNILINK_DEMOS.md](../../../scripts/cinema/OMNILINK_DEMOS.md)); every edit logged |
| [`film/notes.py`](film/notes.py) | Writes the production notes from the take |
| [`film/music.py`](film/music.py) | Optional: lays a quiet, original synthesized ambient bed under a finished cut (the approved style is silent; the music version is an owner-approved exception for YouTube) |
| [`evidence/`](evidence/) | Probe rig + results, live OmniLink checks, the film take's logs |

## Open

- A parsed `walk` replies with the order ("Walk (distance=2.0).") instead of the measured distance: `route._measured_motion` in `omnisim-bridges` does not list `walk` yet.
- A camera on the X30 and a gauge-reading tool, so "check the pump" reads the gauge.
