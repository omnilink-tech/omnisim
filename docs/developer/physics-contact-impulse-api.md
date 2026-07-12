# Contact-impulse / depth API — design + landing record

> **Update 2026-06-08 — read first.** This doc covers the **ODE** per-contact
> `depth` wire API (landed 2026-05-29, below). Since then Newton grew a **native**
> contact source — `WbNewtonBackend::getContacts()`
> ([WbNewtonBackend.hpp:175](../../src/omnisim/physics/WbNewtonBackend.hpp)) — which
> feeds the same damage / contact-points supervisor lists when
> `OMNISIM_NEWTON_NATIVE_CONTACTS` is set (see
> [newton-ode-replacement-plan.md](newton-ode-replacement-plan.md) §W4). That work
> also **measured §2's assumption wrong for *pure-Newton* bodies**: a Newton
> Solid's disabled ODE proxy does **not** collide (`getContactPoints` returned 0
> for it vs 8 native), so native Newton contacts are *necessary*, not just a
> cleaner replacement — and native is now the sole source for a Newton-backed
> Solid (to avoid double-counting). Caveat: under the default **XPBD** solver the
> native `forceMag` is 0 (positional solve) and `depth` is reported as 0 (XPBD
> exposes no clean penetration witness); impulse magnitude only populates under
> the **MuJoCo** solver. So treat §2's "ODE still runs collision for Newton
> bodies" as **superseded for pure-Newton Solids**.

**Status (2026-05-29): ✅ DEPTH API LANDED + VERIFIED** (the atomic append, §3).
`ContactPoint.depth` (C `WbContactPoint.depth`, Python `cp.depth` / `cp.getDepth()`)
now carries the real ODE `dContactGeom.depth` per contact. Landed as one atomic
pass with **all** consumers recompiled (no concurrent session was running):
engine + `Controller.dll` (C lib) + `CppController.dll` (C++ lib) +
`contact_points_supervisor` + `bounding_object_regeneration`; Python rides
`node.py`. **Verified:** the `contact_self_check` gate (box on RectangleArena)
reads 4 sane contacts with `depth` tracking the physics — 0.01575 m at impact
decaying to ~0.0002 m at rest — zero-break (positions still correct), and the
ODE determinism smoke (accelerometer / contact_points / template_deterministic)
stays green. The original "NOT landed / coordinate carefully" caution below is
retained as the historical design record; the coordination requirement was met
(no session mid-run + every consumer rebuilt + both paths verified).

**Follow-up — `damage_tracker` migration LANDED opt-in (2026-06-09, L4):** the
Python consumer now scores per-part contacts on `cp.depth` behind
`OMNISIM_DAMAGE_USE_DEPTH` (default OFF → byte-identical to the proxy). Design,
the head-on-capture findings (wheels flood on resting penetration → depth scopes
to non-load-bearing parts), the scale calibration, and what's still open are in
**§7 below**. `OMNISIM_DAMAGE_VEL_SMOOTH` is **not yet retired** — it remains the
XPBD wheel-jitter workaround until L1 exposes a penetration/impulse witness under
the default solver (see §7).

**Migration is ENTANGLED, not a drop-in (code audit 2026-05-29 — read before
attempting).** The 57k-spurious-event problem lives in the **synthetic
chassis-dv path** (`damage_tracker.py` ~1197-1381): it reads `mass*|Δv|` from the
URDFRobot body velocity, NOT per-contact data, *because URDFRobot consolidates
contact reporting to the leaf (wheel) bodies* — so `getContactPoints` on the
robot does not surface the chassis impact as a contact. `cp.depth` therefore
**cannot directly replace** the chassis magnitude; depth is only a clean
magnitude for parts whose contacts ARE individually reported (non-consolidated
slabs/bumpers). A correct migration must: (a) keep a velocity/synthetic path for
the consolidated chassis (depth won't help there — the jitter is in `body_qd`,
fixable by the existing vel-smooth or a Newton-side velocity filter), and
(b) use `cp.depth` as the magnitude for per-part contacts where it's reported.
The current contact-vs-synthetic split (the `synth_high_z_count_this_step` gate
at ~1207-1268 + the contact-path branch below ~1387) is where depth slots in.
Recommend doing it OPT-IN first (`OMNISIM_DAMAGE_USE_DEPTH`, default-off →
existing scoring byte-unchanged) and validating each path against a head-on
capture before flipping any default. Threshold units change (penetration metres
vs the current Joule proxy), so the per-part `*_threshold_J` tables need
re-derivation — that's the calibration cost, and why this is a focused pass, not
a quick edit.

---

**Original design caution (historical, 2026-05-29):** the change is a wire-protocol
flip that must move the engine, the controller C library, and the Python binding
**atomically** — see §4. (This was satisfied at landing.)

## 1 — Why

The damage tracker
([damage_tracker.py](../../projects/default/controllers/harness_supervisor/damage_tracker.py))
has no real per-contact impulse to work with: the controller contact API
exposes only `point[3]` + colliding-Solid `node_id`. It therefore
*synthesises* impulse as `mass * |Δv|` from per-body `getVelocity()`
deltas. On ODE that proxy is fine (rolling/resting Δv ≈ 0); on Newton the
`body_qd` write-back jitters, so the proxy clears the buffer threshold
every step and a 90 s head-on logs 57,161 spurious "impact"s vs ODE's 149.

The `OMNISIM_DAMAGE_VEL_SMOOTH` knob (`dadd50ae`) works around this by
low-pass-filtering the velocity (57,161 → 58, within 10× of ODE), but it
is a Newton-side compensation that must stay default-off because its lag
hurts ODE's genuine events. **The clean, backend-symmetric fix is to
forward the REAL per-contact penetration depth** (and optionally the
contact force) that the engine already computes, so the tracker thresholds
on a true contact magnitude instead of a velocity proxy.

## 2 — Key facts (code audit 2026-05-29)

- `dContactGeom.depth` is computed by ODE and **already survives intact
  into `WbOdeContact`** (which stores the whole `dContactGeom` by value).
  It is simply dropped at `WbSolid::extractContactPoints`, which keeps only
  `cg.pos`.
- For Newton-backed Solids depth IS available: the Solid keeps an ODE geom
  with `enableForContactPoint=true`, and `odeNearCallback` produces real
  ODE contacts (with depth) against it — the same code path as ODE-native
  contacts. (Newton drives the *pose*; ODE still runs *collision*.)
- The wire protocol is **unversioned, position-based binary** (no
  handshake / version field). The 32-byte-per-point layout is enforced in
  THREE independent places that must move together:
  - C++ serializer `WbSupervisorUtilities::pushContactPointsToStream`
    (writes `3 doubles + 1 int` per point).
  - C reader `src/controller/c/.../supervisor.c` (reads into a malloc'd
    `WbContactPoint[]`).
  - Python binding `lib/controller/python/controller/node.py`
    (hardcodes `format_size = 32`, `struct.unpack_from('3di', …)`).
- `struct WbContactPoint { double point[3]; int node_id; int padding; }`
  is 32 bytes (24+4+4). **Do NOT reuse the `padding` slot** — the server
  never writes it, so a client reading it as depth would get uninitialised
  garbage. Append `depth` as a brand-new trailing field.
- Consumers of `getContactPoints` in-tree: 1 C controller
  (`projects/samples/demos/controllers/contact_points_supervisor/contact_points_supervisor.c`)
  and several Python controllers (`damage_tracker.py`, `harness_supervisor.py`,
  `event_bus.py`, `observe.py`, two `battlebot_damage_director.py`). The C
  one needs recompiling; the Python ones ride on the `node.py` change.

## 3 — The 6-layer append-only edit

| # | Layer | File | Change |
|---|---|---|---|
| 1 | extraction | `WbSolid.cpp::extractContactPoints` (~3067-3104) | alongside `mListOfContactPoints.append(v)`, append `cg.depth` to a new parallel `QVector<double> mContactPointDepths` (+ global variant); declare in `WbSolid.hpp` near `mListOfContactPoints`; reset in `resetContactPoints()`/`resetContactPointsAndSupportPolygon()`. Add `computedContactPointDepths(includeDescendants)` accessor. |
| 2 | serialize | `WbSupervisorUtilities.cpp::pushContactPointsToStream` (~2027-2044) | after `stream << (int)uniqueId()` in the per-point loop, append `stream << (double)depths.at(i);` |
| 3 | C struct | `include/controller/c/omnisim/contact_point.h` (+ `webots/contact_point.h` mirror) | append `double depth;` AFTER `node_id` → `{double point[3]; int node_id; int _pad; double depth;}` = 40 bytes |
| 4 | C reader | `src/controller/c/.../supervisor.c` (~1262-1276) | bump the per-point read size to 40 and read the trailing double into `depth`; adjust the malloc stride |
| 5 | Python | `lib/controller/python/controller/node.py` (`ContactPoint`, `getContactPoints` ~24-33/169-177) | **alignment-sensitive** — `3 doubles + int + double` is NOT `'3did'` (the trailing double forces 8-byte alignment → a 4-byte gap after `node_id`). Use `'3dixxxxd'` OR (safer) a `ctypes.Structure` with `.from_buffer_copy` so the compiler's padding rules — not a hand-counted format — define the layout. Set `format_size = 40`. Expose `cp.depth`. |
| 6 | C controller | `contact_points_supervisor.c` | recompile against the new header (no logic change unless it prints depth). |

Optional Layer 1b (real force/impulse, not just depth): attach
`dJointSetFeedback` to each contact joint in `odeNearCallback`
(~882-905, mirroring `WbMotor::setupJointFeedback`), store the feedback on
`WbOdeContact`, and serialize `|fb->f1|` as a second new field. Heavier;
depth alone is enough for the damage threshold.

Then in `damage_tracker.py`: use `cp.depth` (real penetration) as the
impact magnitude / threshold instead of the synthetic `mass*|Δv|`, and
retire `OMNISIM_DAMAGE_VEL_SMOOTH` (or keep it as a fallback for the
pre-depth wire).

## 4 — Hazard: the flip must be atomic

The wire format is unversioned, so the engine, `node.py`, the C struct,
and the C controller must all change **in the same commit + rebuild**. A
half-landed state is actively dangerous:

- If `node.py` is updated to read 40 bytes but the running
  `omnisim-bin.exe` still writes 32, **every Python contact consumer
  mis-parses** — including `harness_supervisor.py`, which a concurrent
  session may be running.
- This is exactly why it is **not landed now**: as of 2026-05-29 the
  canonical binary was locked by 3 concurrent `omnisim-bin.exe` processes
  (a G1 training/deploy session). Flipping `node.py` without an atomic
  canonical rebuild would have broken their contact reads.

**Landing procedure:** confirm no `omnisim-bin.exe` is running →
edit all 6 layers + `damage_tracker.py` → rebuild the canonical binary
(NEWTON=ON) and the controller lib + `contact_points_supervisor.c` → run
the `contact_points` smoke world (C consumer) AND the Newton head-on
damage capture (Python consumer) → confirm the C smoke still passes and
the tracker now reports sane depth-thresholded events **without** the
vel-smooth knob → single commit.

## 5 — Backward compatibility

Internal only: OmniSim builds the controller lib in-tree, so engine +
`node.py` + C lib ship together — append-only is safe across the bundled
controllers once recompiled. The only externally-visible break is a
**precompiled third-party controller** linked against the old 32-byte
struct; documented as the one accepted break of the unversioned protocol
(the protocol has never carried a version field). If third-party
stability becomes a requirement, the alternative is a NEW additive
supervisor message (`get_contact_point_depths`) that leaves the existing
`getContactPoints` wire untouched — more code, zero break, and it
degrades gracefully (a new tracker calling it on an old engine gets a
"no such field" → falls back to the proxy).

## 6 — Implementation-attempt findings (2026-05-29) — read before landing

A full implementation of §3 was written, built, and then **reverted** because
the blast radius proved broader than safely landable in one autonomous pass.
The code is sound; what's missing is the *coordinated multi-consumer rebuild +
a test that actually exercises a C wire consumer*. Captured here so the next
(coordinated) attempt is fast and safe:

**Confirmations (the §3 edits are correct as written):**
- The C reader (`supervisor.c` ~1270) is **stream-based** (`request_read_double`
  / `request_read_int32` per field), NOT a struct memcpy — so appending
  `points[i].depth = request_read_double(r);` after `node_id` is alignment-safe.
  The serializer (`WbSupervisorUtilities.cpp` ~2037) is likewise field-by-field;
  append `stream << (double)depth;` after the `node_id` write.
- `node.py::getContactPoints` reads the **C struct array** via ctypes
  (`wb_supervisor_node_get_contact_points` → `WbContactPoint[]`), not the raw
  wire — so `format_size` must equal `sizeof(WbContactPoint)`. With the trailing
  `double depth`, the struct is **40 bytes** (24 + 4 + 4-pad + 8) and depth sits
  at **offset 32**. Verified parse: `struct.unpack_from('3di', buf, 40*i)` for
  point+node_id and `struct.unpack_from('d', buf, 40*i + 32)[0]` for depth
  (round-tripped against a synthetic 40-byte buffer — correct).
- Engine `WbSolid` side: store `cg.depth` in parallel `QVector<double>
  mListOfContactPointDepths` / `mGlobalListOfContactPointDepths` (reset in both
  `resetContactPoints` and `resetContactPointsAndSupportPolygon`), expose via
  `computedContactPointDepths(includeDescendants)`. This compiled clean.

**The blast radius (why it needs coordination, expanded from §2/§4):**
- **The `contact_points` smoke does NOT verify a C wire consumer.** The test
  world's controller `tests/physics/controllers/contact_points/contact_points.c`
  does **not** call `wb_supervisor_node_get_contact_points` (grep-confirmed) — so
  a green `contact_points` smoke proves nothing about the wire. The *actual*
  in-tree C/C++ consumers that MUST be recompiled against the new 40-byte struct
  are: `projects/samples/demos/controllers/contact_points_supervisor/…c`,
  `tests/protos/controllers/bounding_object_regeneration/…c`, and the C++ lib
  `src/controller/cpp/Node.cpp`. None are in the smoke set; the samples one is
  in no automated test at all.
- Controllers load `Controller.dll` **dynamically**, but each C/C++ consumer's
  struct *access* uses its own compiled `sizeof(WbContactPoint)` — so a stale
  (32-byte) consumer reading the rebuilt DLL's 40-byte array mis-indexes
  `points[1+]`. EVERY C/C++ consumer must be recompiled, not just the lib.
- Full rebuild chain for an atomic landing: engine (`build_with_cd.sh`,
  NEWTON=ON) + `Controller.dll` (`make release` in `src/controller/c`) + the C++
  lib (`src/controller/cpp`) + each C consumer (`make` in its dir, after
  `make clean` — the header dep is not tracked) + `node.py` (no build).
- **Add a real verification consumer first.** Before landing, add (or adapt) a
  test world whose controller actually calls `getContactPoints` and asserts a
  non-zero `depth` on a resting/penetrating contact, on BOTH the C path
  (a C supervisor) and the Python path (`node.py`). Without it there is no
  regression gate for the wire change.

**Recommendation:** prefer the §5 **additive `get_contact_point_depths` message**
over the atomic append. It leaves the 32-byte `getContactPoints` wire untouched
(zero risk to every existing consumer — the dangerous part), needs no recompile
of unrelated contact consumers, and degrades gracefully on an old engine. The
"more code" cost is small next to the multi-consumer-rebuild + missing-test risk
the append carries.

**Complete consumer inventory (repo-wide grep, 2026-05-29) — the append's exact
recompile checklist.** If the atomic append is chosen anyway, EVERY item below
must be rebuilt in the same pass (else it mis-reads the new wire); the Python
ones ride `node.py` and need no rebuild:

| Consumer | Type | Action for the append |
|---|---|---|
| `src/controller/c/supervisor.c` | C lib reader | rebuild `Controller.dll` (`make release` in `src/controller/c`) |
| `include/controller/c/omnisim/supervisor.h` | C decl | header only — no compile |
| `src/controller/cpp/Node.cpp` | C++ lib | **just delegates** to the C fn (`return wb_supervisor_node_get_contact_points(...)`) — no C++ *controller* consumes it directly; rebuild the C++ lib (`src/controller/cpp`) |
| `projects/samples/demos/controllers/contact_points_supervisor/contact_points_supervisor.c` | C controller | `make clean && make` (header dep untracked); **in no automated test** |
| `tests/protos/controllers/bounding_object_regeneration/bounding_object_regeneration.c` | C controller | `make clean && make` |
| `src/omnisim/core/WbLanguage.cpp` | engine (API registration only, not a wire consumer) | rebuilt with the engine anyway |
| `damage_tracker.py`, `event_bus.py`, `harness_supervisor.py`, `observe.py`, 2× `battlebot_damage_director.py` | Python | ride `node.py` — **no rebuild** |

**Verification gate — ADDED 2026-05-29** (was missing). The harness now lives at
`projects/samples/demos/worlds/misc/contact_self_check.wbt` +
`controllers/contact_self_check/` — a box on a `RectangleArena` floor whose
supervisor reads its own `getContactPoints(True)` and logs count + per-point
`depth` to `CONTACT_CHECK_LOG`. Verified today on the current binary: it reports
**4 sane corner contacts** (`pt0≈[-0.1,-0.1,-0.0002]`, at the floor surface),
`RESULT … PASS`, and `depth0=n/a` (the `ContactPoint` has no `.depth` attr until
the API lands — the correct pre-landing baseline). When the depth API is landed,
extend this controller's `RESULT` line to assert `depth > 0`. (Lesson: a raw
`Solid`+`Box` boundingObject floor does NOT generate contacts for a dropped box —
the dynamic body tunnels; the `RectangleArena` PROTO floor is the reliable
collider, which is why husky/G1 worlds use it.) Note
`tests/physics/controllers/contact_points/contact_points.c` is NOT a consumer
(it validates contact *physics* via a CONE's translation, not the API), so the
existing `contact_points` smoke gates nothing for the wire.

**Coordination requirement (why this is not a blind-autonomous change):** §4's
atomicity hazard means the engine + libs + all C consumers must flip together AND
no `omnisim-bin.exe` may be running during the rebuild (a live session on the old
binary mis-reads the moment the new `node.py`/DLL lands). An autonomous pass
cannot guarantee no session starts mid-rebuild — so land this with the user aware
("binary is free, go"), or via the additive message which sidesteps the hazard.

---

## 7 — Python consumer migration — LANDED opt-in (2026-06-09, lane L4)

The §3 wire (`cp.depth`, 40-byte `WbContactPoint`) was already live in the
canonical binary + `node.py`; this pass is the **Python-only** consumer side the
§3 hazard does NOT apply to (no engine/lib rebuild, no wire change, no binary
lock). Landed as two commits on `damage_tracker.py` + a standalone unit test.

**What landed.**
- `OMNISIM_DAMAGE_USE_DEPTH` (default **OFF**). When OFF, per-part scoring is
  **byte-identical** to the historical `mass*|Δv|` proxy — proven by
  `test_contact_impulse.py` (a 6-case sweep asserting OFF == `mass*dv` and that
  depth is ignored entirely). So every shipping demo and the ODE battlebot games
  are provably unchanged.
- When ON, a per-part contact scores on `mass * depth_scale * cp.depth`
  (`_contact_impulse_J()` pure helper). `OMNISIM_DAMAGE_DEPTH_SCALE` (1/s,
  default **100**) maps penetration metres into the *existing* per-part
  `*_threshold_J` range, so **no threshold table is re-derived** — calibration is
  one tunable constant. `damage_state` surfaces `use_depth` / `depth_scale` /
  `depth_contacts_{used,fallback}`.

**Head-on capture finding — depth scopes to NON-load-bearing parts.** First cut
applied depth to every per-part contact. The `husky_head_on_ode.wbt` capture
(`scripts/dev/damage_events_capture.py`, depth ON) exposed the flaw: husky wheels
rest **~0.03 m into the floor under the robot's weight** — a large, *steady*
penetration, not the sub-mm a light dropped box shows — so `mass*100*depth ≈ 8 J`
per wheel **every step**: 689 events in 12 s, all resting-wheel spam, the exact
resting-contact failure depth was meant to kill. Fix: depth scores only parts
**not in `profile.wheel_parts`**; wheels keep the velocity proxy (rolling/resting
`Δv≈0` already suppresses them). Re-validated on the same 20 s head-on:

| run | wheels | bumper | top_plate | chassis | total |
|-----|-------:|-------:|----------:|--------:|------:|
| depth OFF (proxy)              | 53 | 17 |  3 |  0 |  73 |
| depth ON, all parts (1st cut)  | 689* | 0 | 0 | 0 | 689* (12 s, flood) |
| depth ON, non-load-bearing     | ~77 (proxy) | 26 | 63 | 41 | 207 |

Per-contact depth magnitudes on the collision parts are a sane **~0.5–1.3 J**
(below the 3 J decal threshold — they accumulate HP without spawning decal spam);
nothing is one-shot by the depth path. The big head-on break (≈1010 J →
bumper/top_plate `broken`) is the **unchanged synthetic-chassis path**
(`0.5·m·Δv²`), identical to the OFF run.

**Still open (handoffs + follow-ups).**
- **`OMNISIM_DAMAGE_VEL_SMOOTH` not retired.** Under the default **XPBD** Newton
  solver `cp.depth` is 0, so depth mode transparently falls back to the proxy
  everywhere → the headline Newton-wheel-jitter (57k events) is *not* fixed by
  depth today. Retiring vel-smooth needs L1 to expose a real penetration/impulse
  witness under XPBD (the W4 contact-impulse-under-XPBD task) — a **hand-off to
  L1**, recorded here, not an L4 edit to `WbNewtonBackend`.
- **Battlebot directors — migrated opt-in (2026-06-09), calibration pending.**
  Both `battlebot_damage_director.py` (`robot_combat/.../battlebots` + `.../orc`)
  now carry the same `OMNISIM_DAMAGE_USE_DEPTH` opt-in: chassis/weapon strikes
  score on `cp.depth` (harvested per-part in the bot-on-bot cross-reference
  loop), wheels keep the proxy, `OMNISIM_DAMAGE_DEPTH_SCALE` shared. Default OFF
  → byte-identical. The director's bot-on-bot contact gate already excludes
  resting ground contacts, so the wheel-flood doesn't recur here. **Validation
  boundary:** the flag-ON path is AST-clean + default-OFF byte-identical by
  construction, but was **not** capturable headlessly — `omnisim-bin` is a
  windows-subsystem binary (no piped controller stderr), the directors have no
  stderr-tee like `harness_supervisor`, and the duel world's JSON output points
  at a stale `d:/` path. So flag-ON `depth_scale` calibration against the
  directors' low thresholds (chassis 2.0 / weapon 0.5 J) is the open follow-up;
  visual confirmation is a GUI run away.
- **Default flip + threshold re-derivation.** Making depth the default would want
  the synthetic-chassis path off velocity too, and a per-part `*_threshold_J`
  re-derivation in penetration units rather than the single-scale shim. Deferred
  behind the opt-in until the XPBD witness lands.
- **Newton+MuJoCo capture.** Depth mode was validated under ODE (depth populated)
  and confirmed inert under XPBD (depth 0 → proxy). A MuJoCo-solver head-on world
  would exercise the depth path on Newton; not run this pass.
