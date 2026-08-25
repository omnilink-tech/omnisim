# Harness READ-path latencies, post-`--light` — 2026-07-31

**What this is.** Phase R item 7 of
[`agent-edge-validation-plan.md`](agent-edge-validation-plan.md): the validation harness's
read paths, re-measured on the same 298-node 10-Husky scene class as the pre-`--light`
baseline in [`agent-native-api.md`](agent-native-api.md) Appendix A.2, in **both** session
modes (`light: false`, the default, and `light: true`), three trials per endpoint, all
trials reported. Every number below was measured live this session; nothing is estimated.

## Environment

| | |
|---|---|
| machine id | `9722d23d12a3` (same machine as the A.2 baseline) — AMD64 16-core, RTX 3060 Laptop GPU, Windows 11 |
| commit | `5dc31a84` (A.2 baseline was build `806b055c`) |
| engine binary | `msys64/mingw64/bin/omnisim-bin.exe` sha256 `1d7186d8f24c3d34`, mtime 1785348841 |
| libController | `lib/controller/Controller.dll` sha256 `417dd4d28bfd6a91` — `omnisim doctor`: engine + libController compatible, no blocker |
| backend | **Newton, solver `XPBD(iters=10)`, `degraded: false`** — provenance: the engine's own `.newton.json` verdict sidecar (`/capabilities` → `physics.source: "sidecar"`), re-verified fresh after every reload. A.2's "Newton (default)" column is the same default solver (A.1 labels it "Newton (XPBD, default)"), so the comparison is like-for-like. |
| world | `projects/_scratch/g1fix/worlds/bench10_husky.wbt` — the gitignored scratch copy of the audit's 298-node 10-Husky scene (10 × URDF Husky on `drive_forward`, `RectangleArena` 160 × 160, `basicTimeStep 16`; no `newtonSolver` pin) |
| harness | `scripts/harness/omnisim_harness.py --auto-port` → took `6789`/`6790` (the default pair was free); system Python 3.12.9 with Pillow 11.0.0 |
| protocol | `curl` `%{time_total}`, 3 trials per endpoint per mode, 125 s client cap (no call hit it); the engine runs `--mode=fast` with a non-synchronized supervisor, so the sim **free-runs between trials** — see the session-age caveat below |

## Loads

| Load | Mode | Time |
|---|---|---|
| cold (engine start) | default (non-light) | **55.6 s** |
| hot reload | light | **23.4 s**, then 19.9 s on a second light reload |
| hot reload | default (non-light) | **95.5 s** |

(A.2 baseline: 28.5 / 36.1 s cold Newton. Cold load has gotten slower on this build/scene;
the non-light *hot* reload at 95.5 s was taken late in an aged non-light session — see caveat.)

## The trial table

All values in seconds, `trial1 / trial2 / trial3` → **median**. "default-aged" = the first
non-light battery, measured in the order listed over ~25 min of free-running session;
"default-fresh" = a control re-measure immediately after a fresh non-light reload;
"light" = immediately after a fresh `{"light": true}` reload.

| Endpoint | default (aged session) | default (fresh reload) | light | A.2 baseline (Newton, non-light) |
|---|---|---|---|---|
| `GET /scene/tree` | 17.0 / 38.6 / 50.9 → **38.6** | 26.2 / 43.6 / 41.3 → **41.3** | 12.1 / 11.0 / 10.9 → **11.0** | 23.0 |
| `GET /scene/tree?bounds=1` | 90.1 / 61.6 / 65.8 → **65.8** | — | 44.6 / 18.9 / 19.5 → **19.5** | — |
| `GET /robots` | 49.9 / 56.0 / 56.7 → **56.0** | 43.7 / 43.0 / 43.3 → **43.3** | 16.2 / 15.8 / 15.0 → **15.8** | 22.1 / 22.9 |
| `GET /robot/HUSKY_0/joints` | 33.6 / 39.7 / 39.3 → **39.3** | — | 1.06 / 0.89 / 0.80 → **0.89** | — |
| `GET /sim/contacts` (no wake) | 47.7 / 60.8 / 59.6 → **59.6** | — | 13.5 / 11.4 / 11.9 → **11.9** | — |
| `GET /sim/events` (empty cursors) | 34.9 / 43.5 / 41.1 → **41.1** | — | 0.038 / 0.023 / 0.011 → **0.023** | — |
| `POST /sim/step {steps:1}` | 62.6 / 68.2 / 70.5 → **68.2** | 47.1 / 53.4 / 45.3 → **47.1** | 0.044 / 0.049 / 0.044 → **0.044** | 26.6 / 27.1 |
| `GET /capabilities` | 73.5 / 81.6 / 73.1 → **73.5** | 51.6 / 52.5 / 55.1 → **52.5** | 0.047 / 0.051 / 0.051 → **0.051** | — |
| `GET /sim/state` | — | 14.0 / 25.9 / 26.7 → **25.9** | 0.006 / 0.006 / 0.008 → **0.006** | (0.0016 on the 17-node A.1 scene) |

## Verdict per endpoint

| Endpoint | Verdict |
|---|---|
| `POST /sim/step {1}` | **FIXED in light mode** (27 s → 0.044 s, ~600×, confirming `06a0e23d`). **Regressed/worse in non-light** (median 47–68 s vs the baseline's 26.6–27.1 s). |
| `GET /sim/events` | **Fixed in light mode** (0.02 s). Non-light: 35–44 s — the harness-side merge is fast, but the request still waits behind the saturated supervisor. |
| `GET /capabilities` | **Fixed in light mode** (0.05 s). Non-light: 52–82 s — it consults the supervisor (step-cost / physics blocks) and queues like everything else. |
| `GET /sim/state` | **Fixed in light mode** (0.006 s). Non-light: 14–27 s — *newly measured as slow*: on the 17-node A.1 scene it was 1.6 ms; on this scene in a non-light session even the "never touches the sim" endpoint pays the queue. |
| `GET /robot/<def>/joints` | **Effectively fixed in light mode** (0.9 s). Non-light: 34–40 s. |
| `GET /scene/tree` | **Still slow.** Light halves it vs baseline (23.0 s → 11.0 s) but 11 s for 298 nodes (~37 ms/node) remains an order of magnitude above what an agent loop can afford. Non-light it is now *worse* than baseline (26–51 s). |
| `GET /scene/tree?bounds=1` | **Still slow** (light median 19.5 s; documented as walking all geometry, and it does). |
| `GET /robots` | **Still slow** (light 15.8 s vs baseline 22–23 s; non-light 43–57 s). |
| `GET /sim/contacts` | **Still slow** (light 11.9 s — light mode does *not* change its answer, per the README, and does not change its per-call scene walk either; non-light 48–61 s). |

**The one-line story:** `--light` fully fixes every endpoint whose cost was the *supervisor's
per-step trackers* (`/sim/step`, `/sim/events`, `/capabilities`, `/sim/state`, and
near-fixes `/robot/<def>/joints`), but the endpoints that themselves **walk the scene
per call** (`/scene/tree`, `?bounds=1`, `/robots`, `/sim/contacts`) still cost 11–20 s
on a 298-node scene even in light mode. The 23 s `/scene/tree` of the A.2 baseline is
**halved, not fixed**.

## Two findings the baseline did not contain

1. **Non-light sessions degrade with session age.** The first non-light battery's
   latencies grew monotonically as the session free-ran (e.g. `/scene/tree`
   17.0 → 38.6 → 50.9 s; the fresh-reload control reproduced the shape: 26.2 s on the
   first trial, ~41–44 s after). Every non-light median above is therefore an
   *aged-session* number, and the A.2 baseline's 23.0 s is consistent with our
   *early-session* first trials (17.0 / 26.2 s). Confound, stated plainly: the sim
   free-runs at `--mode=fast` between and during trials, so scene state (10 Huskies
   driving, contact history, event backlog) differs at every trial; cause not isolated
   here — what is established is that a non-light session on this scene gets slower
   the longer it lives, and a light session does not (light trials were flat or
   improving across all endpoints).
2. **In a non-light session, *every* HTTP request queues behind the supervisor** —
   including `/sim/state` (14–27 s here vs 1.6 ms on the small A.1 scene) and
   `/capabilities` (52–82 s). An agent that never reads contacts still pays for the
   trackers on every call it makes. Practical consequence, same as the AGENTS.md
   guidance but now measured on the read paths: **on any multi-robot scene, load with
   `{"light": true}` unless you specifically need `/sim/grips` or `contact.*` /
   `grip.*` / `joint.limit_hit` events.**

## Implication for Phase R / the AgentBench campaign

A `shell+tools` condition run against a **non-light** session on a scene of this size
would lose tens of seconds per tool call to the harness itself — exactly the
"fixable defect misread as 'the surface is useless'" that item 7 exists to prevent.
The campaign's harness condition must (a) load with `light: true` by default, and
(b) treat `/scene/tree`, `/scene/tree?bounds=1`, `/robots`, and `/sim/contacts` as
*known-slow* primitives (~11–20 s on 298 nodes) pending a per-call-walk fix; agents
should prefer `/robot/<def>/joints` (0.9 s) and the event stream (0.02 s) for state
polling.

## Reproducing

```bash
# Same recipe as agent-native-api.md A.5; --auto-port picks the pair and prints it to stderr.
OMNISIM_HOME=o:/omnisim \
OMNISIM_LOG_PATH=<scratch>/h_lat.log \
PATH="/o/omnisim/msys64/mingw64/bin:$PATH" \
  /c/Users/<you>/AppData/Local/Programs/Python/Python312/python.exe \
  scripts/harness/omnisim_harness.py --auto-port

# world: projects/_scratch/g1fix/worlds/bench10_husky.wbt  (gitignored scratch; if absent,
# recreate per its header comment: the 8-husky physics demo geometry extended to 10 robots)
# default mode:  POST /world/load {"path": ..., "wait_s": 120}
# light mode:    POST /world/load {"path": ..., "wait_s": 120, "light": true}
# backend check: read <OMNISIM_LOG_PATH>.newton.json after EACH load (this session:
#                {"backend":"newton","degraded":false,"finalised":true,"solver":"XPBD(iters=10)"})
```

---

## Addendum (2026-08-01): after the per-call-walk fix (`518a335e`)

Everything above is the BEFORE state. The per-call-walk fix shipped in `518a335e`
("stop paying the free-run tax while walking them"); this addendum is the re-measure,
same protocol (3 trials → median, `curl %{time_total}`, `--auto-port`, backend
provenance re-read from the `.newton.json` sidecar after every load), same machine.

### Environment (after-session)

| | |
|---|---|
| machine id | `9722d23d12a3` — same box, binary (`1d7186d8f24c3d34`) and libController (`417dd4d28bfd6a91`) as the table above |
| code | the tree committed as `518a335e` (measurements ran on the identical working tree; the supervisor is re-injected from disk on every `/world/load`, so every load below ran the fixed code) |
| harness | `scripts/harness/omnisim_harness.py --auto-port` → `6789`/`6790`; system Python 3.12.9 |
| backend | per-load sidecar: stress scene **Newton `XPBD(iters=10)`, `degraded: false`**; task worlds per their own pins (B1/B2/C2 `defaultPhysicsBackend "ode"` → sidecar honestly absent/stale, `/capabilities` → `physics.source: "sidecar_stale"`; B3 **Newton `MuJoCo (cpu/mj_step)`**, fresh sidecar). ⚠ **2026-08-08:** `bdc02139` deleted ODE. The B1/B2/C2 AgentBench fixtures **still load and still score — with no physics at all, silently**: a `defaultPhysicsBackend "ode"` pin still *wins* and resolves to an inert stub, with no FATAL, no ERROR and no warning. Every ODE row in this session is a historical measurement on a configuration that cannot be reconstructed, and any *new* run of those fixtures measures a world where nothing moves. |
| contention note | the stress batteries ran alone; the B1m/B3m task-world batteries ran while a concurrent AgentBench integration run had its own engine on this box — small scenes, so the numbers are still sub-0.1 s, but they carry that confound |

### Where the time actually went (profile, measured live)

- **Every supervisor read is one engine round-trip** — `getPosition`, `getOrientation`,
  `getMFNode`, `getSFNode`, every SF getter, `getVelocity`, and the first
  `getContactPoints` per node per step. Free (client-side cached in libController, no
  round-trip): `getTypeName`, `getBaseTypeName`, `getDef`, `getId`, `getField` after its
  first fetch, MF `getCount` (the engine *pushes* `C_SUPERVISOR_FIELD_COUNT_CHANGED` /
  `C_SUPERVISOR_NODE_REMOVE_NODE`, so those stay fresh without polling).
- The plain `/scene/tree` walk was ~900 round-trips (298 nodes × ~3); probed on the live
  scene, one round-trip costs **~6 ms against the free-running engine** (the request
  waits for the engine to service it between physics steps) → ~11 s per call. That is
  the whole story: **round-trip count × free-run service latency**, not Python time and
  not payload size.
- **Paused, the same round-trip costs ~0.15 ms (40×)**, with ~1.2 ms to enter the pause
  and a verified restore (probe: 6.27 → 0.15 → 5.82 ms/read across
  freerun → paused → restored).
- **145 of the 298 nodes are non-posed** (100 Shape, 40 HingeJoint, WorldInfo, sky…):
  each old call paid 290 round-trips for values that are NaN *by construction* — and
  pushed ~145 `wb_supervisor_node_get_position() can exclusively be used with Pose`
  warnings into the engine log per call.

### What changed (all supervisor-side; response schemas untouched)

1. **Paused read bursts** (`observe.paused_reads`): the read-heavy RPCs —
   `scene_tree`, `scene_bounds`, `scene_node`, `get_viewpoint` in dispatch, and the
   self-pausing readers `robots_list` / `robot_joints` / `robot_devices` /
   `sim_contacts` — pause the engine for the duration of their walk and restore the
   prior mode (exception-safe, re-entrant, no-op for unit-test stubs). The `wake=1`
   contact read advances its settle steps **before** pausing (stepping a paused engine
   deadlocks; an AST tripwire in `tests/harness/test_read_paths.py` pins this). Side
   effect worth having: each response is now a **consistent single-instant snapshot**
   instead of a walk smeared over hundreds of free-running steps. The sim still
   free-runs *between* calls.
2. **Posed-node classification** (`observe.node_is_posed`, tables shared with
   `geometry.py`): non-Pose node types answer `position`/`orientation` as nulls
   directly — zero round-trips, zero warning flood; unknown types classify themselves
   from their first measured read (a wrong table entry cannot null out a real pose, and
   measurements never override the tables). Pose **values** are never cached.

### After table — stress scene (298-node 10-Husky, Newton XPBD), medians of 3

| Endpoint | light BEFORE | light AFTER | Δ | non-light BEFORE (fresh / aged) | non-light AFTER (fresh) |
|---|---|---|---|---|---|
| `GET /scene/tree` | 11.0 | **0.111** | ~99× | 41.3 / 38.6 | **28.9** |
| `GET /scene/tree?bounds=1` | 19.5 | **0.194** (0.46 cold) | ~100× | — / 65.8 | **28.4** |
| `GET /robots` | 15.8 | **0.142** | ~111× | 43.3 / 56.0 | **27.8** |
| `GET /sim/contacts` | 11.9 | **0.130** | ~92× | — / 59.6 | **28.0** |
| `GET /robot/HUSKY_0/joints` | 0.89 | **0.031** | ~29× | — / 39.3 | **28.2** |
| `GET /sim/state` | 0.006 | 0.009 | unchanged | 25.9 / — | 28.6 |
| `GET /sim/events` | 0.023 | 0.013 | unchanged | — / 41.1 | not re-measured |
| `POST /sim/step {1}` | 0.044 | 0.029 | not regressed | 47.1 / 68.2 | 48.8 (1 trial) |

Reading the non-light column honestly: the reads no longer add *their own* cost, but
**every** non-light RPC on this scene still pays the queue — the supervisor's main loop
services one request per iteration, and one iteration = the per-step trackers'
~2000 free-run round-trips (~14 s; a request landing mid-iteration waits up to two,
hence the ~28 s plateau, and one lucky sample measured 1.4 s). That floor is the
trackers' free-run cost, deliberately untouched: pausing *per-step* trackers would
throttle the engine for the rendered non-light damage demos. The campaign guidance
stands — **load `{"light": true}`**; what the fix adds is that in light mode the read
endpoints are no longer "known-slow primitives" at all.

### After table — AgentBench Phase W task worlds (light, medians of 3)

| World (backend, nodes) | `/scene/tree` | `?bounds=1` (warm) | `/robots` | `/sim/contacts` |
|---|---|---|---|---|
| B1 `six_huskies` materialized (ODE, 182) ⚠ historical | 0.065 | 0.104 | 0.084 | 0.076 |
| B2 `frame_the_cylinder` (ODE, ~40) ⚠ historical | 0.008 | 0.011 | 0.007 | 0.009 |
| B3 `two_huskies` materialized (Newton MuJoCo cpu, 68) | 0.034 | 0.044 | 0.038 | 0.032 |
| C2 `fall_through` (ODE, ~20) ⚠ historical | 0.006 | 0.008 | 0.006 | 0.007 |

⚠ **The three rows marked historical are UNREPRODUCIBLE since `bdc02139`** — their worlds pin
`defaultPhysicsBackend "ode"`, a backend that no longer exists. The numbers are preserved
verbatim; they are 2026-08-01 measurements of a deleted configuration, not a baseline a re-run
can be checked against. Only the B3 (Newton) row can be re-measured, and the whole-table
conclusion below ("every task-world read endpoint is now well under 0.1 s") therefore now rests
on **one** re-runnable row.

⚠ **Re-running those three fixtures does not fail — it lies.** The `"ode"` pin still wins and
resolves to an inert stub: the world loads, the harness answers, the latency numbers come out
plausible, and **nothing in the scene is being simulated** — no FATAL, no ERROR, no warning. For
a *latency* table that is merely an unstated confound; for anything measuring behaviour it is a
fabricated result. **Operational rule for any deploy, CI or benchmark run whose result you intend
to trust: set `OMNISIM_REQUIRE_NEWTON=1`, and prove Newton drove the world by asserting the
`.newton.json` sidecar — never by exit code, never by a tail-scraped log line.** An *absent*
Newton runtime is silent (only an installed-but-broken one FATALs), so without that flag the
no-physics case is invisible.

Two honesty notes on this table. (1) B1/B3 as committed are **templates** (`url
"@HUSKY_URDF@"`) that only the AgentBench runner materializes; loaded verbatim, B3
fails to parse and B1 loads *without its Huskies* — an earlier battery measured
exactly that and its 5–12 ms rows describe robot-less scenes, so the rows above are
from copies materialized the way the runner does it
(`agentbench.common.paths.HUSKY_URDF`). (2) C1 is skipped by design — its initial
world deliberately does not load. For the campaign: every task-world read endpoint is
now **well under 0.1 s**, i.e. harness read latency can no longer be the reason a
`shell+tools` agent loses on wall clock.

### Session aging: the degradation no longer reproduces

The finding above ("non-light sessions degrade with session age", 17 → 51 s over
~25 min) was re-tested on a fresh non-light session of the same world, sampled every
~2.5 min for **27.5 min** of free-run: `/scene/tree` stayed **flat at 28–31 s**
(12 samples; one 56 s outlier was this author's own concurrent probing, measured and
excluded; one 1.4 s `/sim/state` sample was a lucky queue-arrival phase). Candidates
eliminated by measurement on the aged session: scene growth (node count 298 at minute
0 and minute 25 — no debris), per-round-trip latency drift (7–8 ms aged vs ~6 ms
fresh), contact-set growth (the supervisor contact query returns zero pairs on this
scene throughout), and event accumulation (supervisor event-bus total after 30 min:
**0**). What the old sessions had that the new ones do not: each read call itself
injected ~900 free-run round-trips *and* ~145 engine-log warning lines into the loop —
the leading (unproven, since the old code is gone from the walk) candidate for the
growth term. Recorded as: **fixed in effect, mechanism not isolated**.

### Pre-existing gap surfaced (not caused, A/B-verified): Newton supervisor contacts

`GET /sim/contacts` returns **zero contacts under the Newton backend** in every
configuration tried: the stress scene (XPBD, 10 driving Huskies, 151 solids answering,
`wake=1` included) and materialized B3 (MuJoCo cpu, two resting Huskies). This
predates the fix — measured identical with the pause disabled on the same live scene —
and is a *backend* gap, not a walk bug: on ODE (materialized B1, six resting Huskies)
the same paused walk returns **113 contact points, 69 paired**. The `tracking` block
and wake semantics are unchanged either way. Worth its own follow-up; until then,
"empty contacts on a Newton world" means *the engine's supervisor contact query has
nothing to say*, not "nothing is touching".

> **⚠ 2026-08-08 — this A/B has lost its control side.** The **113 contact points, 69 paired**
> figure is preserved as a dated historical measurement and is **unrepeatable**: `bdc02139`
> deleted ODE, and B1 (which pinned it) now runs on the inert no-physics stub — it will answer
> `/sim/contacts` with an empty set for the opposite reason, silently. What made "Newton returns 0 contacts"
> a demonstrable *gap* was exactly the ODE comparison — a Newton-only run returning zero is
> indistinguishable from "nothing is touching" without it. Two updates on the Newton side:
> **native contact readback is default-ON since 2026-08-07**, so `/sim/contacts` and
> `/sim/grips` are no longer structurally blind and a resting body does report its contacts;
> and `wake=1` remains a no-op on Newton (no sleep) that still costs two steps. Re-measure the
> Newton side on its own terms before quoting this section as a live gap.

### Loads (this session, for completeness)

Cold engine start + light load: 9.5 s; hot light reloads: 4–9 s; hot non-light reload:
44.6 s (vs 95.5 s aged above — but measured on a fresh session, so not a like-for-like
improvement claim). Cold-load variance remains disk-cache-dominated; see the README's
cold-load note.
