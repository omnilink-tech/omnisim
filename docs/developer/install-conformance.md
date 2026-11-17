# OmniSim install conformance self-test

**What it is.** A self-test that runs a small set of leading demos on a fresh
machine, compares what they do to a calibrated tolerance band, and reports
**PASS / PASS-WITH-DRIFT / FAIL** — so a first-time user (or CI) learns
immediately whether *this clone, on this machine, with this resolved backend*
behaves like a known-good reference, and if not, what to change.

**Status (this commit).** Phase 0 + Phase 1 ship a **non-gating reporter**:
`python -m omnisim verify-install`. It does not block any launch yet. The
first-run gate is Phase 3 (designed below, not built). Code lives in
[`omnisim/conformance/`](../../omnisim/conformance/).

---

## 1. What this guarantees (and what it cannot)

Bit-for-bit identical behaviour across machines is **not achievable** and the
self-test does not claim it. Newton/GPU contact-resolution ordering, SIMD/FMA
width, and GPU driver rasterization all introduce legitimate, non-defect
variance. (The fourth source in the original list — reduced-coordinate (Newton) vs
maximal-coordinate (ODE) **model design** — is gone: `bdc02139` deleted ODE, so only one
model design ships and cross-model variance is no longer a category.) See [physics-and-determinism.md](physics-and-determinism.md)
("replicability is not guaranteed" under multithreading) and the cold-load
trap in [real-grasp-and-the-cold-first-load-trap.md](real-grasp-and-the-cold-first-load-trap.md).

So the contract is **behavioral equivalence within tolerance bands + config
conformance**, with two tiers of assertion:

**Bit-exact canaries (HARD — these can FAIL the run):**

1. **omniworld determinism** — same `(recipe, seed, params)` → byte-identical
   `.wbt`. Compared *generate-twice-same-process* (version-independent), so an
   omniworld library bump that legitimately changes bytes does not false-fail.
2. **World parse + asset-locality** — the generated world validates and is
   local-asset-only.
3. **Binary / load liveness** — the world loads, the sim steps, the engine log
   is produced with zero `ERROR:`/`FATAL:` lines.
4. **Backend-resolution proof** — the `[OmNewtonBackend] world finalised
   (solver=...)` line is present (on a Newton-intended host). This is the
   *only* proof Newton drove the world; `imports OK` is **not** (AGENTS.md §0).

**Everything physical is a SOFT band (a miss is DRIFT, never broken):** Husky
displacement, render exposure, warning counts, grasp counts. Measured,
compared to a per-backend band, reported PASS / DRIFT / INFO.

This is the whole design tension resolved in one rule: **HARD = the canaries +
liveness; SOFT = all physics.** It is why the gate (Phase 3) can be mandatory
without manufacturing false "your install is broken" failures.

---

## 2. Command surface + first-run enforcement

### 2.1 The command (Phase 1 — shipped)

```
python -m omnisim verify-install                 # core lane, human report
python -m omnisim verify-install --fast          # fast subset only (canaries + load liveness)
python -m omnisim verify-install --json          # machine-readable (schema omnisim.install_check/v1)
python -m omnisim verify-install --report        # also write a scrubbed {json,md} bundle
python -m omnisim verify-install --strict        # PASS-WITH-DRIFT -> non-zero exit (CI)
python -m omnisim verify-install --fingerprint-only   # just the config fingerprint
python -m omnisim verify-install --deep          # Phase 4 placeholder (never gates)

python -m omnisim doctor --fingerprint           # fingerprint alongside the doctor ground-truth
```

It is its own verb, **not** folded into `doctor`: AGENTS.md §0 makes every
agent run `doctor` first-turn and relies on it being instant and read-only.
A ~60 s sim run inside `doctor` would poison that contract. `verify-install`
reuses `doctor`'s `resolve_webots_binary()`, the `env_fingerprint` collector,
and `dev.run_headless()` — it re-implements no launch logic.

stdout carries **only** the report (human text or JSON); all run-time chatter
(the headless subprocesses, progress lines) is routed to stderr, so
`verify-install --json | jq` is a clean contract.

Exit codes: `0` = PASS or PASS-WITH-DRIFT (drift is non-fatal by design),
`1` = FAIL, `3` = DRIFT under `--strict`.

### 2.2 The gate (Phase 3 — in progress)

[`gate.py`](../../omnisim/conformance/gate.py) is built and wired into **both**
the Python `run-world` path and `launch.bat` (the no-arg demo launcher). The
`make release` pre-warm hook is intentionally skipped (it would tax every build
on a shared dev machine and the stamp is per-home anyway). Mandatory must mean
**mandatory on the interactive human demo path only**, and must be structurally
invisible to automation. The gate exposes one lazy predicate inserted at exactly
the human entry points:

| Surface | Where | Behaviour |
|---|---|---|
| `python -m omnisim run-world` | `cli.py` `_run_world()` | gate before launch |
| `launch.bat` (no world → demo launcher) | the no-arg branch | gate before launch |
| `make release` post-build | Makefile `release` tail (`verify-install --post-build \|\| true`) | advisory, pre-warms the stamp; never fails the build |

It is **NOT** wired into `run-headless`, `harness`, `test-*`, `capture`,
`cinema`, `damage`, `run-agent`, or `doctor` — agents drive those, never
`run-world`, so the gate is off-by-construction for them.

The gate clears silently (exit 0, no sim spin-up) if **any** hold:

| # | Condition | Why |
|---|---|---|
| 1 | `OMNISIM_SKIP_CONFORMANCE=1` or `CI=true` | blanket automation/CI escape (mirrors `OMNISIM_SKIP_PUSH_CHECK`) |
| 2 | argv has `--batch` / `--mode=fast` / `--no-rendering` / `--minimize` | structural auto-skip for all headless/parallel work (AGENTS.md §3e) |
| 3 | valid stamp `~/.omnisim/conformance.json` whose `fingerprint_id` matches | a prior pass on this binary+backend isn't re-paid |
| 4 | `~/.omnisim/conformance.skip` (set by `--never-ask`) | the developer owns their environment |

Mandatory-but-bypassable, not a hard block: a hard gate would wedge the
load-bearing multi-instance / CI / Docker (GPU-absent) story, and a first-run
check that can't be turned off becomes an outage the moment the check flakes.

---

## 3. The demo set

### 3.1 Core lane (mandatory tier — fast, GPU-optional)

Each runs warm (`headless_runner.py` sets `OMNISIM_WARMUP_TOKEN` by default).
The **fast subset** (`--fast`, checks 1–2) is what the gate will block on; the
rest complete the verdict. Manifest: [`manifest.json`](../../omnisim/conformance/manifest.json).

| # | Check (world) | Subsystem(s) | HARD signal | SOFT band |
|---|---|---|---|---|
| 1 | `empty_startup` — `resources/projects/worlds/empty.omniworld` | parser, core init, headless launch + DLL/PATH | loads, 0 errors | — |
| 2 | `omniworld_determinism` — generate `flat_ground` seed 7 ×2 | omniworld PRNG→.wbt determinism, VRML parse, asset-locality | byte-identical sha256, validate ok | — |
| 3 | `warehouse_husky` | URDF importer, physics, supervisor, motor/sensor, WREN render | loads, 0 errors | warning count |
| 4 | `newton_husky_smoke_test` | Newton **resolution** (⚠ no ODE fallback exists since `bdc02139`) | loads, 0 errors, `world finalised` — **unconditional assert; the SKIP-on-ODE arm is deleted** | — |
| 5 | `husky_maze` | URDF, supervisor pose, lidar, wall-follow nav, IPC | loads, 0 errors | warning count |
| 6 | `warehouse_render` | validation harness, WREN render exposure, free-running robot motion | — *(all SOFT; a harness hiccup → demo SKIP, never FAIL)* | displacement, mean_brightness, black/saturated_pct, world_errors |

These five touch every gate-relevant subsystem. **GPU-optional:** the PASS
verdict requires 1, 2, 3, 5.

> **⚠ 2026-08-08 — the GPU-optional argument no longer routes through ODE, and that is a
> DIFFERENT claim.** `bdc02139` deleted ODE: it is **not a supported config**. ⚠ But it does not
> announce itself — an explicit `"ode"` pin still *wins* and resolves to an inert stub, so such a
> world **still loads and still runs, with no physics at all, silently** (no FATAL, no ERROR, no
> warning). Likewise only an *installed-but-broken* Newton runtime FATALs; an **absent** one lands
> on the same silent stub. **So this self-test must run under `OMNISIM_REQUIRE_NEWTON=1`, and
> check 4 must assert the `.newton.json` sidecar — a PASS from liveness alone cannot distinguish
> "Newton drove it" from "nothing was simulated".** That is the exact failure class the retirement
> rule forbids (*a wrong result is worse than a lost one*; a reachable stub must warn — it does
> not). **Open defect, recorded here deliberately.**
> So the three sentences this paragraph used to carry are void — "all of which pass on ODE",
> "on an ODE host it SKIPs and the demo still passes (ODE is a supported config)", and "only
> `OMNISIM_REQUIRE_NEWTON=1` turns an ODE resolution into a FAIL". Newton is unconditional and
> check 4's `newton_finalised` is an unconditional HARD assert with no SKIP arm.
> **GPU-optional now means CPU `SolverMuJoCo` (`mj_step`)** — a genuine CPU path that needs no
> GPU, so a GPU-less box can still PASS. But it is a *different* guarantee from the old one:
> the fallback is a solver choice inside one backend, not a second backend, and the CPU-path
> bands have not been recalibrated for it.

### 3.2 Deep lane (`--deep`, Phase 4 — never gates)

For "I'm about to trust this box for physics/RL/render work." Reports numbers,
fails the install for nothing. Reuses the existing oracles verbatim:
a demo-world liveness probe (liveness + INFO count, never a hard count),
`scripts/dev/wgpu_probe_golden.py --check` (5/255 band, auto-skip if no Vulkan
device), `physics_oracle`/`render_oracle`, the G1 spec Tier-1 CPU conformance.

**Deliberately not in any gate:** RL/ONNX walk durability (research-OPEN per
[rl-current-state.md](rl-current-state.md)), precise friction grasp counts,
wgpu pixel parity — all config/hardware-sensitive or open.

---

## 4. Acceptance manifest format

One checked-in JSON document (`omnisim.install_check.manifest/v1`) with a
calibration header and one entry per demo. Schema:

```jsonc
{
  "schema": "omnisim.install_check.manifest/v1",
  "owner": "physics-platform",
  "calibrated_on": null,            // env_fingerprint of the reference host (null = uncalibrated)
  "last_calibrated_commit": null,
  "demos": [{
    "id": "...", "lane": "core|deep", "fast": true|false,
    "world": "repo-rel .wbt" | {"recipe","seed","params"},
    "run_mode": "headless|generate|harness",
    "warm_or_cold": "warm",
    "requires": {"resolved_backend": "newton|any"},   // "ode" dropped -- backend deleted (bdc02139)
    "run_args": {"duration_s": 14},
    "metrics": [{"name","source","severity":"HARD|SOFT|INFO","type"}],
    "tolerance_bands": {"<metric>": {"newton": {...}, "any": {...}}},   // "ode" band dropped
    "pass_predicate": "human-readable note"
  }]
}
```

`source` names the literal emitter the runner reads:

- `headless.load_ok | .error_count | .warning_count | .finalised | .solver` — from the per-check headless run + engine-log scrape.
- `omniworld.sha_match | .validate_ok` — generate-twice + `omniworld validate`.
- `harness:GET /robots#<DEF>.displacement`, `harness:GET /world/render_stats#<field>` — Phase 1.5 (reported PENDING today).

**Band rules** (in [`compare.py`](../../omnisim/conformance/compare.py)):
`equals` / `max` / `min` (HARD, zero-width); `band_min`+`drift_min` /
`band_max`+`drift_max` (SOFT one-sided); `expected`+`abs_tol`+`drift_abs_tol`
(SOFT two-sided); `skip` (backend-exclusive). Bands were selected by the run's
**resolved backend** (`newton`/`ode`/`any`), because Newton and ODE were
different physical models and never bit-equal. ⚠ **Since `bdc02139` there is one backend, so
this is a single-band model**: `newton` and `any` are the only keys, backend-keyed band
selection is vestigial, and the cross-backend inequality that justified it no longer applies.
The live variance to band against is now *solver* and *host* (CPU `mj_step` vs GPU
`mujoco_warp`, GPU model, thread count) — see
[determinism-scope.md](../benchmarks/determinism-scope.md).

### 4.2 Calibration (Phase 1 ships uncalibrated)

The current manifest carries deliberately generous placeholder bands and
`calibrated_on: null`, so SOFT misses report as DRIFT only. Calibration
(Phase 1.5) records reference values from N=3 runs on a declared known-good
host into `calibrated_on` + per-metric bands, frozen with the calibration
fingerprint and `last_calibrated_commit`. Widening a band is a reviewed step
in the PR that moves the behaviour; bands stay generous-by-construction so
normal engine drift doesn't trip them. **A run is judged only against bands
whose backend matches its resolved fingerprint** — cross-config comparison is
forbidden.

---

## 5. Config fingerprint + result classification

### 5.1 Fingerprint ([`fingerprint.py`](../../omnisim/conformance/fingerprint.py))

Wraps `projects/policies/common/env_fingerprint.collect()` (the load-bearing
producer that reads the authoritative `world finalised (solver=...)` line) and
adds the install-relevant fields it doesn't carry: `resolved_backend`,
`newton_runtime_present` (is the runtime bundled next to the binary?),
`pillow_present`, `warp_version`, `mujoco_version`, `cpu_core_count`,
`render_backend`, `omnisim_binary`, `binary_mtime`.

`fingerprint_id` is a 12-hex digest over **only the install-behaviour-affecting
subset** (commit, resolved backend, solver, Newton-runtime presence, GPU model
with driver stripped, OS, render backend, knobs, binary mtime). A rebuild or an
ODE↔Newton swap invalidates it (exactly when install behaviour can change); a
cosmetic env tweak does not.

### 5.2 Classification ([`compare.py`](../../omnisim/conformance/compare.py))

```
per metric:  HARD  meets band -> PASS         else -> FAIL
             SOFT  in band -> PASS; in drift_band -> DRIFT; beyond -> FAIL
             INFO  always recorded, never gates
             harness-sourced -> PENDING (Phase 1.5)

per demo:    FAIL  if any HARD FAIL (or HARD metric errored)
             DRIFT else if any SOFT DRIFT
             PASS  else

per install: FAIL            if any demo FAIL
             PASS-WITH-DRIFT else if any demo DRIFT or any advisory is "drift"
             PASS            else
```

One soft miss never fails the install; a missing Newton runtime is FAIL *only*
when `OMNISIM_REQUIRE_NEWTON` demanded it; GPU absence is DRIFT/INFO, not
failure.

---

## 6. Report + remediation

### 6.1 Scrub-by-default ([`scrub.py`](../../omnisim/conformance/scrub.py))

Everything that leaves the machine is scrubbed first: home dirs → `~`,
`OMNISIM_HOME` → `<OMNISIM_HOME>`, `C:\Users\<name>` / `/home/<name>` →
`<user>`. The fingerprint embeds absolute paths that on most machines carry the
OS username; the report (and any `--report` bundle under `install-reports/`,
gitignored) is safe to paste into a public issue. No raw engine log is bundled —
only parsed, allow-listed fields. (Phase 2 hardens this with the deny-list
regex pass and an explicit `--share` upload step.)

### 6.2 Remediation advisor ([`advisor.py`](../../omnisim/conformance/advisor.py))

A keyed lookup over the fingerprint that maps a cause to the exact fix.
Implemented today: `BINARY_MISSING`, `NEWTON_FELL_BACK_TO_ODE`,
`NEWTON_PRESENT_BUT_NOT_DRIVING`, `MISSING_PILLOW`, `ODD_SUBSTEPS`. The full
table (stale binary, degraded XPBD solver, wrong renderer, require-newton
violation, cold-load grasp miss) lands with the gate.

---

## 7. Top risks + how the design defuses them

| Risk | Defused by |
|---|---|
| Non-deterministic physics count in a pass/fail gate | grasp/count checks are DEEP-only; the gate asserts liveness, never a count |
| "Mandatory" breaks CI / Docker / parallel farms | gate only on interactive `run-world`/launcher; auto-skip on `--batch`/CI; fingerprint-keyed stamp |
| Asserting non-bit-exact results | HARD = 4 canaries + liveness only; all physics is SOFT |
| GPU-absent host fails a Newton assert | ⚠ **the old defusal is void** (`bdc02139`): there is no "ODE host", and ODE is **not a PASS**. Nor is it a clean FAIL — an `"ode"` pin still loads onto an inert stub and runs with no physics, silently, and an *absent* Newton runtime does the same. The surviving defusal is that Newton's **CPU `SolverMuJoCo` (`mj_step`)** path needs no GPU, so a GPU-less box resolves Newton and passes; the CPU bands are uncalibrated. **Run with `OMNISIM_REQUIRE_NEWTON=1` and assert the `.newton.json` sidecar, or the no-physics case passes as healthy.** |
| Cold-launch / port-collision false-fail | warm by default; **each check runs on its own free port** + one retry; transient noted, not failed |
| Bands rot, no owner | one checked-in manifest with calibration fingerprint + named owner; band-widening is a reviewed PR step |
| Report leaks usernames/paths | scrub-by-default, allow-listed fields, no raw log, gitignored output |
| Wall-clock truncation on slow hosts | checks assert liveness + (future) sim-step counts, not wall-time distance |

---

## 8. Implementation status

**Legend:** ✅ shipped this commit · 🔶 partial · ⬜ designed, not built.

- ✅ **Phase 0 — fingerprint**: `fingerprint.py` (+ `doctor --fingerprint`), `fingerprint_id`, scrub.
- ✅ **Phase 1 — reporter**: `manifest.json` (5 core checks), `checks.py` (headless + omniworld), `compare.py` (bands + roll-up), `advisor.py`, `report.py` (human + JSON + bundle), `cli.py`, wired into `omnisim/cli.py` as `verify-install`. Non-gating.
- 🔶 **Phase 1.5 (in progress)**: the `harness` run mode is implemented ([`harness_client.py`](../../omnisim/conformance/harness_client.py) + `run_harness_check`) — the `warehouse_render` demo loads via the validation harness, free-runs on wall-clock, and measures render exposure + robot displacement, all SOFT so a harness hiccup makes the demo SKIP (never FAIL). SOFT bands are **preliminarily** calibrated from a single dev-box reference run. Remaining: N=3 multi-host calibration; `/sim/step` is fragile (returns 503) so displacement is wall-clock-coarse by design.
- ⬜ **Phase 2**: deny-list scrub pass + explicit `--share`; full advisor table.
- ✅ **Phase 3**: `gate.py` ships — fast-lane gate + fingerprint-keyed stamp (`~/.omnisim/conformance.json`) + escape hatches (`OMNISIM_SKIP_CONFORMANCE`/`CI`, batch flags, valid stamp, `--never-ask` skip file) + **fail-open** (any internal error lets the launch proceed). Wired into **both** `run-world` and `launch.bat` (no-arg launcher); `verify-install --gate` exits **42** on a deliberate FAIL so the launcher blocks only on that, never on a python/import problem. `--post-build`/`--never-ask` flags ship. The `make release` pre-warm hook is intentionally skipped (build tax on shared machines). Remaining: the full FAIL terminal UX polish.
- ⬜ **Phase 4**: the `--deep` lane (reuses `physics_oracle` / `render_oracle` / `wgpu_probe_golden` / G1 spec Tier-1).

**Reused, not rebuilt:** `doctor`, `env_fingerprint.collect()`, `dev.run_headless`
(`headless_runner.py`), `omniworld` generate/validate. Net-new is the thin
orchestration layer in `omnisim/conformance/`.
