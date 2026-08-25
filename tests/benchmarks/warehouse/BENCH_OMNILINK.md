# What does OmniLink add, and is it more than generic LLM tool calling?

`bench_omnilink.py` answers that question with numbers, on the
`warehouse_omnilink` demo. The local-Ollama condition is essential: offline
regex versus either LLM measures the value of language-model tool calling;
local Ollama versus OmniLink is the comparison that can isolate
**OmniLink-specific** value.

---

## 0. The framing, before any number

**The 5-stage production line is fully autonomous deterministic Python inside
the three bridge controllers.** It needs no LLM, no `OMNI_KEY` and no network.
Two consequences, and every metric in this harness is shaped by them:

1. **OmniLink cannot make the line faster.** There is no LLM in the control
   loop to speed up. Any claim of the form "OmniLink improved throughput" is
   false by construction.
2. **OmniLink can only add operator capability, and it costs throughput.**
   Any operator command — chat *or* a direct bridge tool call — instantly
   pauses that robot's idle loop. So a *lower* boxes/min in the interactive
   conditions is the **expected price of interacting with a running line**,
   not a defect.

The harness therefore measures exactly two axes and refuses to blend them:

| axis | what it is | direction that is "good" |
|---|---|---|
| **capability** | can the operator's instruction actually be carried out | higher |
| **throughput cost** | what the interaction cost the line | lower, but **non-zero is expected** |

> ### ⚠️ Before you run it, or read a result file, read §9
>
> This harness has produced a **silently invalid** result: a mixture of live
> bridges and *orphaned* ones — controller processes whose engine was killed,
> whose main thread is blocked forever in `robot.step()`, and whose HTTP thread
> is still answering 200 on the same port a fresh bridge just bound. Every
> number in that run was about a robot that did not exist.
>
> Three hard gates now make that a **refusal** (exit `7`/`8`/`9`) rather than a
> footnote, and every result file carries an `integrity` block. §9 is the full
> account: how it happens, what is defended, and what still is not.

---

## 1. The one rule that makes this a measurement

> **Every verdict comes from measured robot state. Never from the reply text.**

For each prompt the harness snapshots pose / joints / `idle_loop.paused`
**before**, issues the prompt, polls state for a bounded window, and decides
pass/fail from the **delta** against an explicit per-prompt predicate. The
reply text is recorded as evidence and is never the verdict.

A model that replies *"Driving forward 1.00 m now (~4.2 s)."* while the tug
does not move **scores 0**. That is not hypothetical — it is the failure mode
documented in [`docs/developer/tool-design-for-agents.md`](../../../docs/developer/tool-design-for-agents.md),
where 26 % of agent turns contained a fabrication and every one was about the
agent's own past actions. The unit tests named `test_ADVERSARIAL_*` exist
solely to hold this line; if one ever passes while the predicate returns
`pass`, the harness has stopped measuring the robot.

### The two secondary signals (deliberately not part of the score)

* **`answer_accuracy`** — a pure question has no robot-state consequence, so
  state-scoring alone cannot tell a correct answer from *"I don't recognise
  that."* The harness reads the **ground truth out of the bridges' own
  `/state`** (never from the model) and checks whether the reply contains it.
  Reported under `scores.answer_accuracy_SECONDARY`, never merged into the
  state score.
* **`latency_s`** — wall time of the `/prompt` round trip.

---

## 2. Launching the world, and running all four conditions

### 2.1 Launch the world — **not** with `launch.bat`

```bash
# Windowed, 1x, PATH-safe. Give it a duration that covers your whole session.
python scripts/dev/headless_runner.py \
    projects/samples/demos/worlds/flagship/warehouse_omnilink.omniworld \
    --gui --realtime --duration 1800

# Or truly headless (no window) at 1x:
python scripts/dev/headless_runner.py \
    projects/samples/demos/worlds/flagship/warehouse_omnilink.omniworld \
    --realtime --duration 1800
```

Give it ~15 s after launch for the three bridge controllers to boot and arm
their idle loops. `--realtime` matters: without it `headless_runner.py` passes
`--mode=fast` and boxes/min stops being a wall-clock rate.

> #### ⚠️ Why not `launch.bat` — verified, and it silently kills `--mode omnilink`
>
> `launch.bat:20-23` builds the child `PATH` like this:
>
> ```bat
> set "PATH=%OMNISIM_HOME%\msys64\mingw64\bin;%PATH%"
> if exist "%OMNISIM_HOME%\msys64\mingw64\bin\newton-runtime\python.exe" (
>     set "PATH=%OMNISIM_HOME%\msys64\mingw64\bin\newton-runtime;%PATH%"
> )
> ```
>
> That `python.exe` **exists** in this checkout (CPython 3.12.9, vendored with
> the Newton runtime), so `newton-runtime` ends up **first** on `PATH`. The
> engine spawns every Python controller with the bare command `python`
> (`src/omnisim/control/OmController.cpp:693`, preference
> `General/pythonCommand`, default `"python"`), resolved from `PATH` — so the
> bridges run on the Newton interpreter. Verified on this clone:
>
> ```
> msys64/mingw64/bin/newton-runtime/python.exe -c "import omnilink"
>   -> ModuleNotFoundError: No module named 'omnilink'
> ```
>
> `omnisim_bridges` still imports (the `_omnilink_relay` shim puts
> `packages/omnisim-bridges/src` on `sys.path` itself), so the bridges come up
> and serve HTTP normally — but `OmniLinkRelay.__init__` calls
> `check_omnilink_installation()` (`relay.py:422`), which raises because
> `omnilink` is missing. `setup_omnilink_relay` catches it and returns `None`,
> so **the offline regex router answers every `/prompt`** and `GET /usage`
> reports `enabled: false`. A run labelled `--mode omnilink` would be measuring
> the regex router.
>
> **Correction to a claim you may have been told:** that fallback is *not*
> silent in current source. The `except` tail of `setup_omnilink_relay` prints
> `"!! OmniLink relay setup FAILED … there is NO LLM in the loop"` **plus a
> traceback**, in both bridges (`omnilink_mobile_bridge.py:6364`,
> `omnilink_arm_bridge.py:5177`, as of `4219c38d` — that file moves, grep the
> string rather than trusting the number). What makes it easy to miss is where
> the text goes: `omnisim-bin.exe` is a GUI-subsystem binary on Windows, so
> controller stdout does not reach the console you launched from. The engine
> gate below is what turns it into a refusal instead of a message you never saw.
>
> **`python -m omnisim run-headless` had the SAME trap — FIXED 2026-07-28.**
> `omnisim/dev/runner.py` (`webots_env()`) used to *prepend* `newton-runtime` to
> `PATH`, and `headless_runner.py` only prepends `msys64\mingw64\bin` in front of
> it — a directory with no `python.exe` — so `newton-runtime` still won. Both
> `webots_env()` and `launch.bat` now **append** that directory instead, so a
> system Python wins and the bundled interpreter stays as a last resort. Newton
> was never at risk: the engine's embedded interpreter is resolved from
> `python312.dll` in the binary's OWN directory (searched before `PATH`) plus the
> `python312._pth` beside it, not from `PATH` — verified over 30+ launches, all
> of which wrote a non-degraded Newton verdict sidecar.
>
> Invoking `scripts/dev/headless_runner.py` **directly**, as above, was and
> remains safe: it inherits your shell's `PATH` unmodified.
> `python -m omnisim run-agent` (via `scripts/dev/omnisim_run_agent.py`) is
> **still trapped** — it prepends, and `tests/omnilink_integration/test_run_agent.py`
> pins that behaviour. Whatever wrapper you use, check first:
>
> ```bash
> python -c "import omnilink, omnisim_bridges; print('ok')"   # the interpreter
> where python                                                # PATH order
> ```

### 2.2 Run the four conditions

The chat mode is chosen when the **controllers start**; this harness cannot
switch it. `--mode` is a **label you assert**, which the harness then verifies
against every bridge's `GET /usage` — and **refuses to run** when the two can
be shown to disagree (§2.3).

```bash
# ---- A: control. No interaction at all -- the throughput reference. -------
# Launch the world however you like; nothing is sent to it.
python tests/benchmarks/warehouse/bench_omnilink.py --mode none \
    --duration 900 --out tests/benchmarks/warehouse/results/none.json

# ---- B: OFFLINE regex router ---------------------------------------------
#   unset OMNI_KEY and set OMNISIM_OLLAMA=0 BEFORE launching the world,
#   so no relay is constructed at controller start.
#     Windows:  set OMNI_KEY=  &&  set OMNISIM_OLLAMA=0
#     bash:     unset OMNI_KEY; export OMNISIM_OLLAMA=0
python tests/benchmarks/warehouse/bench_omnilink.py --mode offline \
    --duration 900 --out tests/benchmarks/warehouse/results/offline.json

# ---- C: LOCAL Ollama -- generic LLM tool-calling control -----------------
#   unset OMNI_KEY and make sure Ollama is answering on 127.0.0.1:11434
#   BEFORE launching the world. Do not set OMNISIM_OLLAMA=0.
#     Windows:  set OMNI_KEY=  &&  set OMNISIM_OLLAMA=1
#     bash:     unset OMNI_KEY; export OMNISIM_OLLAMA=1
python tests/benchmarks/warehouse/bench_omnilink.py --mode local \
    --duration 900 --out tests/benchmarks/warehouse/results/local.json

# ---- D: OMNILINK platform ------------------------------------------------
#   set OMNI_KEY=olink_...   BEFORE launching the world, in the SAME shell
#   that launches it. Also set OMNISIM_OLLAMA=0 unless you want the HYBRID
#   relay, which serves ordinary turns from a LOCAL model (see 2.3).
#   (`python -m omnisim key` prints the exact line; `python -m omnisim byok`
#    connects a model-provider key, which is a separate second step.)
python tests/benchmarks/warehouse/bench_omnilink.py --mode omnilink \
    --duration 900 --out tests/benchmarks/warehouse/results/omnilink.json

# ---- side by side --------------------------------------------------------
python tests/benchmarks/warehouse/bench_omnilink.py --compare \
    tests/benchmarks/warehouse/results/none.json \
    tests/benchmarks/warehouse/results/offline.json \
    tests/benchmarks/warehouse/results/local.json \
    tests/benchmarks/warehouse/results/omnilink.json
```

Each chat mode is selected when the controllers start, so run every condition
against a **fresh launch of the same world revision and build**, with the
corresponding environment set before launch. Keep a
`python projects/policies/common/env_fingerprint.py` beside the four results —
per this repo's rule, an unattributed number cannot be resolved to a machine
later.

Interpret the comparisons separately:

* `offline` → `local` asks whether generic LLM tool calling beats regex.
* `local` → `omnilink` asks whether OmniLink itself adds anything on this
  suite. If those columns tie, this suite has **not** proven platform-specific
  value; memory, telemetry, voice and site-level Foreman workflows require
  their own probes.

```bash
python tests/benchmarks/warehouse/bench_omnilink.py --print-suite   # audit the experiment
python tests/benchmarks/warehouse/bench_omnilink.py --selftest      # 114 unit tests, no sim
```

### 2.3 Mode verification: what is now PROVEN, and what still cannot be

`GET /usage` answers `{"enabled": false}` **exactly when the bridge has no
relay** (`omnilink_mobile_bridge.py` `_route_get`: `if relay is None`) — the
offline regex router is what answers `/prompt`.

**`enabled: true` is necessary but NOT sufficient for `--mode omnilink`.** A
local `OllamaRelay` reports it too. The discriminator is the *shape* each relay
writes into `/usage.latest`:

| relay | `latest.engine` | `latest.text` | classified as |
|---|---|---|---|
| `OllamaRelay` (`relay.py:1981-1989`) | `"ollama:<model>"` | `"local <model>: N in / M out tok, …"` | `local_ollama` |
| `OllamaRelay` cloud fallback (`relay.py:1867-1868`) | `"<cloud engine>"` | `"cloud fallback via <engine>"` | `cloud_via_ollama_fallback` |
| `OmniLinkRelay` (`relay.py:1082-1085`) | *absent* | the platform rollup: `"window=…s tokens=… → … credits/hour"` | `cloud_omnilink` |
| no relay | — | — | `none` |

Two asymmetries that are easy to get backwards, and that the classifier
encodes explicitly:

* **`latest` is `null` until a chat turn COMPLETES.** A null before the first
  prompt proves nothing, which is why the decisive check runs *after the first
  probe*, not at preflight.
* **`OMNILINK_USAGE=0` suppresses the cloud relay's meter entirely.**
  `USAGE_ENABLED_DEFAULT` (`relay.py:277`) goes False → `OmniLinkRelay._meter`
  stays `None` → `_last_usage` is `None` **forever**. `OllamaRelay` writes
  `_last_usage` directly and is unaffected. So `latest: null` is **not**
  evidence of a cloud relay either. It is evidence of nothing.

The gate therefore splits provable from unprovable:

| situation | outcome |
|---|---|
| `--mode omnilink`, a bridge reports `enabled: false` | **refused, exit 9** |
| `--mode omnilink`, a bridge published an Ollama signature | **refused, exit 9** |
| `--mode local`, every bridge published an Ollama signature | `engine_verified: verified` |
| `--mode local`, a bridge published an offline or cloud signature | **refused, exit 9** |
| `--mode local`, `latest` null / unrecognised | `engine_verified: unverified` + warning — the run continues, and **must not be quoted as a verified local-LLM control** |
| `--mode offline`, any bridge published an LLM signature | **refused, exit 9** |
| `--mode omnilink`, every bridge published the platform rollup | `engine_verified: verified` |
| `--mode omnilink`, `latest` null / unrecognised | `engine_verified: unverified` + warning — the run continues, and **must not be quoted as a verified OmniLink result** |
| `cloud fallback via …` seen | warning: the platform was reached, but by a **hybrid** relay whose ordinary turns are local |
| `--mode none` | `not_applicable` — no prompts are issued, so the chat layer never runs |

`--allow-mode-mismatch` downgrades the refusals to warnings (recorded in
`warnings`). The liveness and identity gates have **no** bypass.

What still **cannot** be proven from these endpoints: *which cloud model*
answered. `engine=` is not published for the platform path, so
`OMNILINK_ENGINE=g1-engine` vs `g4-engine` is still an asserted label.

---

## 3. The exact suite

Ten prompts, fixed order, identical text in every mode, in one auditable
module-level list (`SUITE` in `bench_omnilink.py`). `--print-suite` dumps it
with a sha256 fingerprint that `--compare` uses to prove two runs ran the same
experiment.

| # | key | tier | target | prompt | predicate |
|---|---|---|---|---|---|
| 1 | `t1_stop_literal` | 1-literal | tug | `stop` | `at_rest` |
| 2 | `t2_drive_1m` | 2-parametric | tug | `drive forward 1 meter` | `net_translation` |
| 3 | `t3_where_are_you` | 3-query | tug | `where are you?` | `not_parked` |
| 4 | `t4_compose_back_then_turn` | 4-compositional | tug | `back up half a metre, then turn left 45 degrees` | `translate_then_rotate` |
| 5 | `t5_count_parked_carts` | 5-world-state | tug | `how many carts are parked in the row right now?` | `not_parked` + answer check |
| 6 | `t6_resume_literal` | 6-resume | tug | `carry on, back to work` | `resumed` |
| 7 | `t7_resume_oblique` | 6-resume | tug | `I'm finished with you for now -- the line needs you more than I do, so go finish the job you were on.` | `resumed` |
| 8 | `a1_arm_stop_literal` | 1-literal | arm | `stop` | `joints_at_rest` |
| 9 | `a2_arm_shipped_count` | 5-world-state | arm | `how many boxes have you shipped so far on this shift?` | `not_parked` + answer check |
| 10 | `a3_arm_resume_oblique` | 6-resume | arm | `that's everything I needed -- the belt is backing up, so get yourself kitting again.` | `resumed` |

`--tug-robot` picks which tug the `tug` probes target (default `tug_b`, the
return tug).

---

## 4. Every scoring predicate, precisely

All five are pure functions of `(before, after, series)` dicts and are unit
tested against synthetic snapshots. Tolerances are CLI/params-tunable and are
written into the JSON, and every raw measurement is recorded — so any verdict
can be **re-derived at a different tolerance** from the result file without
re-running.

### `at_rest` — "stop"

* **Passes iff** over the window after a `grace_s = 2.5 s` braking period:
  `max per-step speed ≤ 0.03 m/s` **and** `net drift ≤ 0.08 m`.
* Speeds are derived **from poses, not from `v_linear`**. `v_linear` is the
  *commanded* value — precisely the number that can read 0 while the wheels
  still turn.
* Records `trivially_satisfied: true` when the robot was already stationary
  before the command (measured by two pre-probe pose reads `--pre-probe-gap-s`
  apart). Still a pass, but flagged so a reader can discount it.
* `inconclusive` with < 2 post-grace samples.

### `net_translation` — "drive forward 1 meter"

* Displacement is resolved into the robot's **initial body frame**, so:
  * `forward_distance`: `|forward_m − 1.0| ≤ 0.15 m` — a metre *backwards* or
    a metre *sideways* fails.
  * `heading_held`: `|Δyaw| ≤ 12°` — a metre driven round an arc is not what
    was asked.
* Both sub-goals must pass. `lateral_m` is recorded as informational.

### `translate_then_rotate` — the compositional probe

* Two independently scored sub-goals, **binary verdict** (a robot that did
  half of what it was told did not do what it was told), with the partial
  count recorded as `subgoals_met: "1/2"`:
  * `translation`: `| |displacement| − 0.5 m | ≤ 0.18 m`
  * `rotation`: `|wrap(Δyaw − 45°)| ≤ 12°` — wrapped, so a crossing of ±180°
    is not scored as a 340° error.
* **Order is evidence, not a gate.** The displacement bearing lies along the
  *old* heading if the robot reversed first and along the *new* one if it
  turned first; `inferred_order` records which, with both residuals. It does
  not gate the verdict, because the operator's end state is what matters.

### `not_parked` — the query probes

Deliberately **not** "the robot did not move": the idle loop is *supposed* to
keep working through a question, so scoring stillness would mark the correct
behaviour as a failure.

* **Passes iff** the robot was running before (`idle_loop.paused == false`)
  and is still running after. The bridges implement this by rolling the pause
  back when a chat turn used only read-only tools.
* `inconclusive` when it was already paused before the question (nothing was
  preserved) or when the robot publishes no `idle_loop` — never a free pass.
* Movement during the window is allowed and recorded.

### `joints_at_rest` — the arm's "stop"

* **Passes iff** the joint vector `q` spread over the post-grace window is
  `≤ 0.03 rad`.

### `resumed` — **the discriminator**

* **Precondition:** the robot must be paused when the prompt is sent (the
  harness guarantees this — §6). Otherwise `inconclusive`.
* Finds the **first** sample where `idle_loop.paused` goes `false`, then
  attributes the cause (§5).
* **Passes iff** an un-pause was observed **and** it was caused by a tool
  call. Waiting out the timer is a **fail**: the robot came back, but nothing
  the agent did caused it.

---

## 5. The discriminator metric: time-to-resume, and *what caused it*

This is the number the whole harness exists to produce, so the attribution is
spelled out rather than assumed. All times are measured from the moment the
prompt was **sent**.

```
timer_due   = min(resume_s,  resume_s − t_arm_before_send)
tool_bound  = reply_latency + tool_grace_s     (default 5 s)
timer_bound = timer_due − timer_tol_s          (default 8 s)
```

| condition | `cause` | verdict |
|---|---|---|
| un-paused at or before `tool_bound` | `tool` | **pass** — a tool call happens *before* the reply returns, so an un-pause by then was caused by something the agent called |
| un-paused at or after `timer_bound` | `auto_timer` | **fail** — the ~60 s quiet window expired; the clock released it |
| in between | `unclear` | **fail** — not scored as a success |
| never un-paused in the window | `none` | **fail** |
| `tool_bound ≥ timer_bound` | `unseparable` | **inconclusive** — the model was so slow its reply lands near the timer; crediting the agent here would be a fabrication |

With the defaults (`resume_s = 60`, `tool_grace = 5`, `timer_tol = 8`) the
bands are `≤ ~7 s → agent`, `≥ ~49 s → timer`, and a wide unambiguous gap
between. The CLI **refuses to start** if
`resume_s ≤ tool_grace + 2·timer_tol`, because the two causes would not be
separable at all.

`--resume-s` **must match the controllers' `--idle-resume-s`** or the
attribution is wrong. The bridges do not publish it on any endpoint, so:

```bash
--calibrate-resume    # MEASURE it: park a robot with a direct /stop_robot,
                      # poll idle_loop.paused until it clears, use the elapsed
                      # time. Costs one pause-length of throughput.
```

The JSON records `run.resume_s_source` as `measured` or `asserted`.

The verify window for a resume probe is `resume_s + 25 s` — deliberately
**past** the timer, because a window that ends before the timer fires cannot
tell "the agent did it" from "the clock did it".

---

## 6. The setup/reset protocol (why the results are comparable)

Before each probe the harness puts the target robot into the probe's required
precondition using **direct endpoints, never chat**, so the precondition is
established identically in every mode and the thing under test stays the thing
under test:

* probes needing a running robot → `POST /resume_autonomy`, then wait
  `--post-reset-quiet-s` (default 2.5 s);
* resume probes → `POST /stop_robot`, then wait `--post-setup-quiet-s`
  (default 3 s). The timestamp of this POST is what `t_arm_before_send` refers
  to.

> **The 1.5 s trap, and why the CLI enforces it.** `act_resume_autonomy` sets
> a ~1.5 s exemption window during which the *next* command's idle-loop pause
> is deliberately swallowed (so a resume cannot re-arm the pause it just
> lifted). A prompt fired inside that window would silently fail to arm the
> pause and **every downstream measurement would be wrong**. The harness
> refuses to run with `--post-reset-quiet-s < 1.6`.

After the last probe the harness resumes every robot before measuring the
recovery window, so recovery throughput is not contaminated by a leftover
pause. `--no-reset-between` disables the inter-probe resets (preconditions
then drift between modes — don't, unless you know why).

### Busy handling (409)

The bridges reject a second motion with **HTTP 409** while one is in flight
(`busy_rejecting_actions`: `drive_to`, `drive_forward`, `turn`;
`busy_overriding_actions`: `stop_robot`, `set_velocity`). A 409 is a
**protocol refusal, not a failure of the mode under test**, so the harness
retries `--retry-busy` times with `--busy-backoff-s`, and only if it is still
busy records `inconclusive` — never `fail`.

Two shapes are detected, because there are two doors: a direct tool route
answers HTTP 409, while `POST /prompt` answers **200** and buries the refusal
inside the `actions` list. Missing the second would score a refusal as a
model failure.

---

## 7. Throughput cost

A background sampler polls all three bridges' **`GET /state`** (never POST) at
`--hz` for the whole run, and the analysis is done by
**`measure_line.py`'s own `analyse_line` / `analyse_tug`**, so every throughput
number is defined identically to the baseline harness — see
[`README.md`](README.md) → *What each metric means*.

Four windows are reported:

| window | what it is |
|---|---|
| `baseline` | `--baseline-s` (default 180 s) of quiet before any prompt — the within-run reference |
| `intervention` | the suite. In `--mode none` this is a quiet hold of the same shape, so the columns are comparable |
| `recovery` | `--settle-s` (default 120 s, should exceed `resume_s`) after the last prompt |
| `whole_run` | everything |

Per window: `boxes_per_minute`, `fills_per_minute`, `picks_per_minute`,
`arm_not_working_s`, **`arm_paused_by_operator_s`**, `fill_blocked_total_s`,
`complete_ship_cycles`, `realtime_factor`, and per-tug
`paused_by_operator_frac` / path / rotation.

`throughput_cost.throughput_ratio` = intervention ÷ baseline boxes/min.
**A ratio below 1.0 is the expected cost of talking to a running line.**

---

## 8. Output

* **Human summary** to stdout: the **integrity gates first**, then scores,
  per-prompt verdict + measured delta + the reply as evidence, the
  discriminator table, the throughput table.
* **Machine JSON** to `--out`: everything above plus per-probe `attempts`,
  raw `measured` fields, `subgoals`, `text_check`, `phases`, `mode_probe`,
  **`integrity`** (§9), `calibration`, `suite_sha256`, `warnings`.
* **`--compare a.json b.json c.json`**: one side-by-side table, with a
  suite-fingerprint mismatch warning, an **`engine verified` / `liveness gate`
  / `identity gate`** row per column, and the caveats printed inline.

**Secrets.** `OMNI_KEY` is read from the environment only. It is never sent,
logged or written. Every recorded string passes through a redactor that masks
both the literal values of known key env vars *and* generic token shapes
(`olink_…`, `sk-…`, `Bearer …`, `AIza…`). The JSON records only
`omni_key_present_in_harness_env: true|false` — no value, no fingerprint.

**Exit codes**

| code | meaning |
|---|---|
| `0` | ran to completion (**probe failures are results, not errors — still 0**) |
| `2` | preflight failed: a bridge did not answer, or this is not the warehouse world |
| `3` | aborted mid-run (partial JSON still written) |
| `4` | bad arguments |
| `6` | `--compare` could not read a file |
| **`7`** | **LIVENESS GATE** — a bridge answered 200 with a frozen tick loop. You are talking to an orphaned controller, or the sim is paused. |
| **`8`** | **IDENTITY GATE** — the process behind a bridge port is ambiguous (duplicate listener) or changed during the run. |
| **`9`** | **ENGINE GATE** — the engine that answered contradicts `--mode`. |

`7`, `8` and `9` say *the measurement is invalid*, not *the robot failed*. They
outrank everything else: an integrity failure is reported even when every probe
passed, and `8` outranks `7` outranks `9` outranks `3`.

---

## 9. Known failure modes of the MEASUREMENT ITSELF

Everything in §11 is a limit on what the numbers *mean*. This section is worse:
it is how the harness can produce a fully-formed, confidently-wrong result
about a robot that **does not exist**. It has happened.

All three defences below refuse the run rather than warn. The liveness and
identity gates have **no bypass at all**; only the engine gate has one
(`--allow-mode-mismatch`), because a mislabelled run is still a real
measurement of *something*, whereas a dead robot is not.

### 9.1 The orphaned-bridge hijack — the one that actually bit

**What happens.** `scripts/dev/headless_runner.py:817` stops the simulator with
`proc.terminate()` — `TerminateProcess` on Windows. The engine is killed
outright, so it never tells its controllers to quit. Each bridge controller
survives as an orphan in a very specific state:

* its **main thread** blocks forever inside `robot.step()` on a dead IPC pipe,
  so nothing ticks, nothing moves, and its `Supervisor` is gone;
* its **daemon HTTP thread keeps serving**, answering `/state`, `/usage`,
  `/prompt` and `/stop_robot` with clean 200s from the last state it held.

**Why a fresh run can end up talking to it.** `ThreadingHTTPServer` sets
`allow_reuse_address = 1` (`HTTPServer` sets it; `TCPServer` does not). On
Windows SO_REUSEADDR lets a **new** process bind an **already-bound**
`127.0.0.1` port. Reproduced on this box with two plain
`ThreadingHTTPServer`s on one loopback port:

```
  TCP    127.0.0.1:8799   0.0.0.0:0   LISTENING   33336   <- second binder
  TCP    127.0.0.1:8799   0.0.0.0:0   LISTENING   31664   <- first binder
  10 GETs -> {'31664': 10}            # the EARLIER binder answered all ten
```

So a freshly launched bridge can be **shadowed in full by last week's corpse**.
Which process wins is not specified by the API and is not something this
harness controls; do not assume the observed ordering generalises.

**What it looks like in a result file, if nothing checks.** Measured:

* two bridges reporting `enabled: false` — stale *offline-mode* corpses from a
  previous run — while the third was genuinely live;
* a `drive forward 1 meter` probe scoring `moved -0.346 m`, because the command
  reached a process whose `Supervisor` was dead, so nothing executed. The
  harness's own anti-fabrication rule worked perfectly and produced a
  meaningless number, because the *state it read* was also from the corpse.
* This box demonstrably leaks these processes: one `omnisim-bin.exe` had been
  alive since 2026-07-24.

It is not a rare shape. While the gates were being *tested*, a leftover stub
bridge from an earlier test run held the test ports and answered two
supposedly-fresh runs end to end — the swap under test never happened, both
runs looked clean, and the only tell was a sim clock that had been counting for
five minutes. If it can happen to a test whose entire subject is this bug, it
can happen to a benchmark run.

**The defences.**

| gate | what it proves | failure |
|---|---|---|
| liveness | `/state.last_tick_at` advances across a `--liveness-gap-s` (default 1.2 s) window. It is stamped from the controller's own tick loop (`MobileBridge.tick`, `ArmBridge.tick`), so a corpse's value never moves. | exit `7`, before any measurement |
| duplicate listener | `netstat -ano` shows exactly one PID `LISTENING` per bridge port | exit `8`, at preflight |
| identity | the same PID, the same `/state.id`/`model`, a non-decreasing sim clock and non-decreasing session counters, start → end | exit `8`, at close |

A frozen tick loop has **two** possible causes and the harness names both,
because they are indistinguishable from outside the process: an orphaned
controller, or a **paused simulation**. Either way nothing is being stepped.

**If a gate trips:**

```bat
taskkill /IM omnisim-bin.exe /F
netstat -ano -p TCP | findstr ":876"      :: find leftover bridge pythons
taskkill /PID <pid> /F                    :: one per stale listener
```
Confirm the ports show **no `LISTENING` row at all**, then relaunch per §2.1.

### 9.2 The limits of the identity check — stated, not hidden

There is **no process identity on any bridge endpoint**: no pid, no boot id, no
process start time. `/protocol` carries only a static `{name, robot_id}`. The
gate is therefore built from the strongest signals that *do* exist, and they
are not equally strong:

* **OS listener PID** — a *true* fingerprint, and the only direct view of the
  duplicate-bind state. Windows-only and best-effort: no `netstat`, a
  non-English state column, or `--no-port-identity` all degrade it. An empty
  parse is treated as **inconclusive**, never as "nothing is bound".
* **`/state.id` + `model`** — catches a different *robot* behind the port.
* **sim clock + monotone session counters** (`idle_loop.cycles`,
  `jobs_total`, `delivered_total`, `holds_total`, `line.shipped_total`) —
  these reset to 0 when a controller starts, so a decrease is proof of a
  restart. But they are **necessary conditions, not a fingerprint**: two live
  processes whose clocks and counters happen to be consistent are
  indistinguishable this way.

**Measured hole, and the fix.** Comparing only the start and end snapshots is
a weak mid-run detector: a process that takes the port over halfway through has
the rest of the run to climb its counters back above the preflight values. In
an end-to-end test with `--no-port-identity`, a deliberate mid-run process swap
was scored **`stable`** by the endpoint comparison alone. The gate therefore
also scans the **full** background `sim_time` series (the sampler already
records it at `--hz` for the whole run) for a backward jump. On the same test
that caught it immediately:

```
sim_time went BACKWARDS 1 time(s) (13.351 -> 0.101 at run t=10.5s)  -> exit 8
```

Worth noting from that run: **`polls_failed` was 0**. On Windows a connection
to a closed port takes ~2 s to refuse, so the replacement server was up before
any poll timed out — the swap left *no gap at all* in the sampled series. A
"no failed polls" run is not evidence of continuity.

So `stable` means **nothing contradicted continuity**, which is weaker than
proof, and the JSON says exactly that in `integrity.identity.LIMITS`. Prefer a
run with the pid probe enabled.

### 9.3 Silent engine substitution

Covered in full in §2.3. In short: `enabled: true` does not mean "the cloud
answered", `latest: null` does not mean "no relay", and the launch path can
strip `omnilink` from the controllers' interpreter without the operator ever
seeing the message that says so. The engine gate refuses what it can prove and
marks the rest `unverified`.

### 9.4 What is still NOT defended

* **A bridge that is alive but attached to a *different world*.** Nothing in
  `/state` names the `.wbt`. `measure_line.preflight` requires the arm's `line`
  block, which rules out a non-warehouse world, but not a second warehouse.
* **Two live simulators, one shadowed.** If both are genuinely ticking, the
  liveness gate passes both and only the duplicate-listener check separates
  them — so on a platform without the `netstat` probe this is undetected.
* **A swap during the closing gap.** The identity re-read happens after the
  recovery window; a swap in the last second is caught, but a swap *between*
  the closing read and the report is not (nor could it matter).
* **Cross-machine bridges.** `--host` is honoured but the OS listener probe is
  local-only; against a remote host it reports `supported: false`.

---

## 10. Verified against the source — one correction worth knowing

**The offline regex router *does* have a resume intent.** It is easy to
believe otherwise, and the design of probe 7 depends on the truth:

* `omnilink_mobile_bridge.py:2207` and `omnilink_arm_bridge.py:2670` check
  `shared_is_resume(s)` **first** in `IntentRouter.dispatch`, before every
  motion rule.
* It is `RESUME_RE` in
  [`packages/omnisim-bridges/src/omnisim_bridges/intent_router.py`](../../../packages/omnisim-bridges/src/omnisim_bridges/intent_router.py):
  `resume | carry on | keep going/working/at it | continue | as you were |
  proceed | back to work/it | get back to work | go back to work/what you were
  | restart your work/loop/autonomy | unpause`.
* It is **conditional**: the import is wrapped in `try/except`, and a clone
  without the `omnisim_bridges` package importable falls back to
  `shared_is_resume = None`, which disables the intent entirely.

So `t6_resume_literal` ("carry on, back to work") is expected to **pass**
offline — it matches `RESUME_RE` — and it is kept precisely because it pins
the contract and detects the no-package configuration. The real discriminator
is **`t7_resume_oblique`**, phrased to carry the same intent while matching
none of those alternatives (no *resume*, *carry on*, *keep going*, *continue*,
*as you were*, *proceed*, *back to*, *restart*, *unpause*, and no bare *back*
for the reverse-drive rule to eat). A regex router cannot get the robot
working again from it; an LLM holding `resume_autonomy` can.

Each suite entry carries an `offline_expectation` field. **It is a prediction
recorded for falsification and is never an input to any verdict.** If a run
contradicts it, the prediction was wrong — update it, do not adjust the score.

---

## 11. What this does **NOT** measure — threats to validity

Be blunt about all of this before quoting a number.

* **A lower boxes/min under interaction is EXPECTED, not a defect.** The line
  is autonomous deterministic Python; no chat mode can speed it up, and every
  operator command parks a robot for the quiet window. Reading the throughput
  column as "OmniLink is worse" is a misreading of the experiment. The only
  interesting throughput question is *how much* it cost and whether the line
  recovered.
* **n = 1 per prompt per run.** Ten prompts, one attempt each. That is an
  anecdote, not a distribution. Repeat each condition several times before
  believing any difference, and treat a one-probe swing as noise.
* **LLMs are nondeterministic.** The same prompt to the same model can call a
  different tool on the next run. `--mode omnilink` is not reproducible in the
  way `--mode offline` is, and a single omnilink run cannot be compared
  against a single offline run with any confidence.
* **The four conditions are not simultaneous.** They run in separate launches
  whose state trajectories (cart positions, park-row occupancy, queue depth)
  diverge. The baseline window inside each run exists to absorb some of that,
  but it does not make the conditions matched.
* **Intervention windows differ in length between modes** — an LLM turn is far
  slower than a regex match. Compare **rates**, and check the `window s` row.
  A longer window also means more time for the line to do something unrelated.
* **One machine, one build, one world.** Nothing here records GPU/CPU
  identity; run `python projects/policies/common/env_fingerprint.py` alongside
  and keep the results together, or the numbers cannot be attributed later.
* **The harness perturbs what it measures — on purpose.** It POSTs. Setup
  `/stop_robot` and `/resume_autonomy` calls park and un-park robots; the arm
  `stop` probe interrupts a pick mid-cycle. This is unavoidable (interaction
  *is* the subject) but it means these runs are **not** valid inputs to a
  line-throughput comparison. Use `measure_line.py` for that.
* **`--mode` is asserted, not switched.** The harness now refuses a run whose
  engine can be *shown* to contradict the label (§2.3) — but "could not tell"
  is a real outcome: check `integrity.engine_verified` before quoting anything,
  and treat `unverified` as unverified. *Which cloud model* answered is still
  entirely an assertion.
* **A green integrity block is not a guarantee.** `stable` means nothing
  contradicted continuity; the gates cannot see a second live simulator, a
  different world, or (without the Windows `netstat` probe) two live processes
  on one port. §9.2 and §9.4 enumerate the holes.
* **Not the physics.** It reads what the bridges publish. A tug clipping
  through a wall, a cart resting on nothing — none of that appears here.
* **Answer accuracy is a numeric containment check**, not comprehension. A
  reply that states the right number for the wrong reason scores as correct,
  and a correct answer phrased without the number scores as wrong. It is
  secondary evidence and is never part of the state score.
* **Tool-selection quality, token cost and dialogue quality are out of scope.**
  Latency is recorded; cost is not. The suite also does not exercise memory,
  telemetry, voice or the Warehouse-Foreman, so a local/OmniLink tie is a
  limitation of this experiment, not proof that those platform features have
  no value.
* **Predicate tolerances are judgement calls.** 0.15 m and 12° are defaults
  chosen against the mobile bridge's measured accuracy (mean |error| 0.44°,
  max 0.97°, n=8 after the turn-law fix in `52f3f6ca`). Every raw measurement
  is in the JSON, so re-derive at your own tolerance rather than arguing about
  the default.
* **The state score mixes tiers.** 7/10 says nothing about *which* 7. Read
  `by_tier` and the per-prompt table; a mode that passes every literal probe
  and fails every compositional one is a very different thing from the
  reverse.

---

## 12. Files

| File | What it is |
|---|---|
| `bench_omnilink.py` | The harness. `SUITE` and the predicates are at the top; the three integrity gates sit between the predicates and the report builder. |
| `test_bench_omnilink.py` | 117 synthetic unit tests for the pure predicates **and the integrity gates**, including the `test_ADVERSARIAL_*` set (frozen bridge, mid-run swap, duplicate listener, Ollama-shaped usage under `--mode omnilink`). No simulator, no network. |
| `BENCH_OMNILINK.md` | This file. |
| `measure_line.py` | The throughput baseline harness this one reuses (GET-only). |
| `README.md` | Metric definitions for everything under `throughput`. |
| `results/` | Suggested home for `--out` JSON (created on demand; not tracked). |
