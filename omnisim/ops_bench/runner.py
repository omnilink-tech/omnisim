# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Execute one episode timeline against one arm, record everything, grade."""
from __future__ import annotations

import gzip
import json
import math
import time
from pathlib import Path

from omnisim.control_bench.engine import InfrastructureError
from .agents import arm_env, make_arm
from .session import Sampler, Session
from .suite import grade, render, resolve, wrap

STEP_MAX_WAIT_S = 60.0
# A trace longer than this is stored next to the episode as trace.json.gz and
# the row carries a reference: a 30-minute shift is ~60,000 samples (~6 MB),
# and 21 of them inline would put rows.jsonl past GitHub's 100 MB file limit.
TRACE_INLINE_MAX = 5000
REPLY_DRAIN_S = 180.0
MIN_GAP_S = 1.0
DRAIN_START_S = 1.5


def _at_rest(sampler, span=0.4):
    s = sampler.samples
    if len(s) < 3: return False
    recent = [x for x in s if x["t"] >= s[-1]["t"] - span]
    if len(recent) < 3 or recent[-1]["t"] - recent[0]["t"] < span * 0.7: return False
    a = recent[0]
    return (all(x.get("mode") in (None, "idle", "hold") for x in recent)
            and max(math.hypot(x["x"] - a["x"], x["y"] - a["y"]) for x in recent) < .005
            and max(abs(wrap(x["yaw"] - a["yaw"])) for x in recent) < .01)


def _wait_on(clock, seconds, wall_cap_s=None):
    """Wait `seconds` on the episode clock (wall or simulated). A wall cap of
    the same length plus a minute means a stalled simulator cannot hang it."""
    start = clock()
    wall_end = time.monotonic() + (wall_cap_s if wall_cap_s is not None else seconds + 60.0)
    while clock() - start < seconds and time.monotonic() < wall_end:
        time.sleep(0.02)


def _pose(sample):
    return None if sample is None else {k: sample[k] for k in ("t", "x", "y", "yaw")}


def run_episode(task, arm_name, session, out_dir, model_cfg=None):
    sampler = Sampler(session, sim_clock=bool((model_cfg or {}).get("sim_clock")))
    clock = sampler.now
    agent = make_arm(arm_name, session, clock, model_cfg)
    fired, replies, notes, handles = {}, {}, [], []
    prop_obs = {}
    try:
        time.sleep(0.3)
        first = sampler.latest()
        if first is None: raise InfrastructureError("No initial sample")
        pose0 = (first["x"], first["y"], first["yaw"])
        ctx = {"pose0": pose0, "sampler": sampler}
        prev_fire = {"t": clock(), **_pose(first)}
        t_start = clock()
        for step in task["steps"]:
            when = step.get("when", {"idle": True})
            (kind, arg), = when.items()
            wait = float(step.get("max_wait_s", STEP_MAX_WAIT_S))
            if kind == "at_s":
                wait = max(wait, t_start + float(arg) - clock() + 30.0)
            # The step's wait is on the EPISODE clock. Under --sim-clock a world
            # running slower than real time (e.g. while cameras render) used to
            # hit this deadline in wall seconds first and skip the step -- a
            # filmed shift lost Priya's all-clear and held to the end. The wall
            # cap only guards against a stalled simulator.
            deadline = clock() + wait
            wall_cap = time.monotonic() + (4.0 * wait + 120.0 if getattr(sampler, "sim_clock", False) else wait)
            idle_since = None
            met = False
            while clock() < deadline and time.monotonic() < wall_cap:
                cur = sampler.latest()
                if kind == "idle" or kind == "idle_then_s":
                    # An operator does not speak twice within milliseconds;
                    # without this floor an instant reply leaves an empty
                    # window between two messages and nothing is observed.
                    quiet = (all(h.done.is_set() for h in handles) and _at_rest(sampler)
                             and clock() - prev_fire["t"] >= MIN_GAP_S)
                    if kind == "idle": met = quiet
                    else:
                        if quiet and idle_since is None: idle_since = clock()
                        if not quiet: idle_since = None
                        met = idle_since is not None and clock() - idle_since >= float(arg)
                elif kind == "moved":
                    met = math.hypot(cur["x"] - prev_fire["x"], cur["y"] - prev_fire["y"]) >= float(arg)
                elif kind == "turned":
                    met = abs(wrap(cur["yaw"] - prev_fire["yaw"])) >= float(arg)
                elif kind == "after_s":
                    met = clock() - prev_fire["t"] >= float(arg)
                elif kind == "at_s":
                    met = clock() - t_start >= float(arg)
                if met: break
                time.sleep(0.01)
            if not met:
                notes.append({"step": step["id"], "trigger_not_met": when, "t": clock()})
                continue
            cur = sampler.latest()
            fired[step["id"]] = {"t": clock(), **{k: cur[k] for k in ("x", "y", "yaw")}}
            prev_fire = fired[step["id"]]
            if "say" in step:
                text = render(step["say"], pose0)
                h = agent.say(step, text, ctx)
                handles.append(h); replies[step["id"]] = h
            elif "fixture" in step:
                f = dict(step["fixture"])
                # A place/park that names no prop means the task's only prop
                # (the v2 author wrote a bare `park`, as the brief worded it,
                # and every calibration run died on KeyError('prop')).
                if f["op"] in ("place", "park") and not f.get("prop"):
                    f["prop"] = (task.get("props") or ["block"])[0]
                if f["op"] == "place":
                    obs = prop_obs.setdefault(f["prop"], {"placements": []})
                    if obs["placements"] and "final" not in obs["placements"][-1]:
                        obs["placements"][-1]["final"] = session.prop_position(f["prop"])
                    session.place(f["prop"], resolve(f["at"], pose0))
                    time.sleep(0.3)
                    obs["placements"].append({"placed": session.prop_position(f["prop"]),
                                              "t": clock()})
                elif f["op"] == "park":
                    # Measure where the prop ended BEFORE clearing it away,
                    # so that placement can still be judged.
                    obs = prop_obs.setdefault(f["prop"], {"placements": []})
                    if obs["placements"] and "final" not in obs["placements"][-1]:
                        obs["placements"][-1]["final"] = session.prop_position(f["prop"])
                    session.place(f["prop"], session.park)
                    obs["parked_at_t"] = clock()
                elif f["op"] == "shove":
                    res = session.shove(float(f.get("dx", 0)), float(f.get("dy", 0)))
                    notes.append({"step": step["id"], "shove": res.get("position")})
                elif f["op"] == "restart":
                    # The robot's software restarts. Every arm loses what it
                    # keeps only in memory and keeps what it persists: the
                    # agent is told, exactly as a process restart would.
                    down = session.restart_robot()
                    if hasattr(agent, "on_restart"):
                        agent.on_restart()
                    notes.append({"step": step["id"], "restart": {"down_s": down, "t": clock()}})
            elif "observe" in step:
                _wait_on(clock, float(step["observe"]))
        # Drain: every message answered (or timed out), robot at rest, then
        # a tail so a late or wake-driven motion is still on the record.
        # A parsed final drive returns BEFORE the wheels turn; judged at once,
        # the robot looked at rest and the episode ended mid-drive
        # (omnilink-f1-boundary-01, keepout_tb3). Give it time to start.
        while clock() - prev_fire["t"] < DRAIN_START_S:
            time.sleep(0.05)
        end = time.monotonic() + REPLY_DRAIN_S
        while time.monotonic() < end and not all(h.done.is_set() for h in handles):
            time.sleep(0.05)
        for h in handles:
            if not h.done.is_set(): h.error = "timeout"
        end = time.monotonic() + 20
        while time.monotonic() < end and not _at_rest(sampler):
            time.sleep(0.05)
        # The tail is measured on the EPISODE clock: simulated seconds under
        # --sim-clock (a 90 s probe tail had been sitting out 90 wall seconds).
        _wait_on(clock, float(task.get("tail_s", 2.0)))
        for name, obs in prop_obs.items():
            if obs.get("placements") and "final" not in obs["placements"][-1]:
                obs["placements"][-1]["final"] = session.prop_position(name)
    finally:
        sampler.stop()
        agent.close()
    trace = {"samples": sampler.samples, "sampling_errors": sampler.errors, "props": prop_obs}
    reply_records = {k: h.as_record() for k, h in replies.items()}
    outcome, reasons, unsafe, detail = grade(task, trace, fired, reply_records, pose0)
    return {"fired": fired, "replies": reply_records, "notes": notes, "trace": trace,
            "pose0": pose0, "outcome": outcome, "reasons": reasons, "unsafe": unsafe,
            "checks": detail}


def load_trace(row, run_dir):
    """A row's full trace: inline, or from its episode's trace.json.gz."""
    tr = row.get("trace") or {}
    if "file" not in tr:
        return tr
    with gzip.open(Path(run_dir) / "episodes" / row["episode_dir"] / tr["file"], "rt",
                   encoding="utf-8") as fh:
        return json.load(fh)


def relay_usage(path):
    """Per-round token counters from the relay's own OMNILINK_TRACE file.

    The relay writes a "round_start" line before every model call and a
    "round" line with its usage when it returns. A started round with no
    "round" line (still in flight when the episode ended) counts as a round
    of UNKNOWN usage, charged at the worst observed request like any other.
    Older traces have only the per-turn summary, which misses a turn that
    never finished (holdout-v1 run 1 read $0 for OmniLink that way).
    """
    totals = {"prompt": 0, "cached": 0, "cache_write": 0, "output": 0, "thoughts": 0}
    p = Path(path)
    if not p.exists(): return {"rounds": 0, "totals": totals, "unknown_usage_rounds": 0}
    rows = []
    for line in p.read_text(encoding="utf-8", errors="replace").splitlines():
        try: rows.append(json.loads(line))
        except ValueError: continue
    started = [r.get("round_id") for r in rows if r.get("kind") == "round_start"]
    if started:
        done = {r.get("round_id"): r for r in rows if r.get("kind") == "round"}
        per_round = [done.get(rid) or {} for rid in started]
    else:
        per_round = [rt for r in rows if r.get("kind") == "turn" for rt in r.get("rounds") or []]
    unknown = 0
    for rt in per_round:
        if rt.get("prompt_tokens") is None: unknown += 1; continue
        for k, src in (("prompt", "prompt_tokens"), ("cached", "cached_tokens"),
                       ("cache_write", "cache_write_tokens"),
                       ("output", "output_tokens"), ("thoughts", "thoughts_tokens")):
            totals[k] += int(rt.get(src) or 0)
    models = sorted({str(rt["model_returned"]) for rt in per_round if rt.get("model_returned")})
    return {"rounds": len(per_round), "totals": totals, "unknown_usage_rounds": unknown,
            "models_returned": models}


def run_task(task, arm, key, directory, model_cfg=None):
    record = {"task": task["id"], "family": task["family"], "robot": task["robot"],
              "arm": arm, "outcome": "ERROR", "unsafe": None}
    session = None
    started = time.monotonic()
    try:
        cfg = dict(model_cfg or {}, key=key)
        cfg["records"] = []
        sim_clock = bool(cfg.get("sim_clock"))
        if sim_clock and arm != "omnilink":
            # A competitor's model calls run in THIS process, where the
            # runtime cannot see them to slow the world down: fast time would
            # let its robot drive on while it thinks. Real time only.
            raise ValueError(f"--sim-clock is OmniLink-only (dev loop); {arm} runs in real time")
        relay_env = arm_env(arm, cfg)
        if arm == "omnilink":
            # Always keep the replies (small), so any run can be replayed later.
            relay_env = dict(relay_env, OMNILINK_MODEL_RECORD=str(Path(directory).resolve() / "model_replies.jsonl"))
            if cfg.get("replay_from"):
                hits = sorted(Path(cfg["replay_from"]).glob(f"**/episodes/*_omnilink_{task['id']}/model_replies.jsonl"))
                if not hits:
                    raise ValueError(f"--replay-from: no recorded replies for {task['id']} under {cfg['replay_from']}")
                relay_env["OMNILINK_MODEL_REPLAY"] = str(hits[0].resolve())
                record["replayed_from"] = str(hits[0])
        if sim_clock:
            relay_env = dict(relay_env, OMNILINK_CLOCK="sim",
                             OMNILINK_CLOCK_MAX_SPEED=str(cfg.get("sim_clock_speed") or 6))
        session = Session(task["robot"], directory, key, task.get("props", []),
                          relay_env=relay_env, mode="fast" if sim_clock else "realtime")
        record["env"] = session.env_used
        record["clock"] = "sim" if sim_clock else "wall"
        record["physics"] = session.physics
        record["setup_s"] = time.monotonic() - started
        record.update(run_episode(task, arm, session, directory, cfg))
        record["episode_dir"] = Path(directory).name
        tr = record.get("trace") or {}
        if len(tr.get("samples") or []) > TRACE_INLINE_MAX:
            with gzip.open(Path(directory) / "trace.json.gz", "wt", encoding="utf-8") as fh:
                json.dump(tr, fh)
            record["trace"] = {"file": "trace.json.gz", "n_samples": len(tr["samples"]),
                               "sampling_errors": tr.get("sampling_errors"),
                               "props": tr.get("props")}
        if cfg["records"]:
            record["competitor_requests"] = cfg["records"]
            if arm == "codex_full":
                usage = next((r["usage"].get("total") for r in reversed(cfg["records"])
                              if r.get("usage", {}).get("total")), None)
                record["codex_usage"] = {"model": cfg.get("codex_model"),
                    "reasoning_effort": cfg.get("codex_reasoning"),
                    "operator_turns": len(cfg["records"]),
                    "model_requests": sum(r.get("model_requests", 0) for r in cfg["records"]),
                    "tokens": usage, "cost_usd": None,
                    "billing": "signed-in Codex account; billed cost unavailable"}
            if arm == "claude_full":
                usage = next((r["usage_total"] for r in reversed(cfg["records"])
                              if r.get("usage_total")), None)
                reported = next((r["cli_reported_cost_usd"] for r in reversed(cfg["records"])
                                 if r.get("cli_reported_cost_usd") is not None), None)
                record["claude_usage"] = {"model": cfg.get("claude_model"),
                    "effort": cfg.get("claude_effort"),
                    "operator_turns": len(cfg["records"]),
                    "model_requests": sum(r.get("model_requests", 0) for r in cfg["records"]),
                    "tokens": usage, "cost_usd": None,
                    "cli_reported_cost_usd": reported,
                    "billing": "signed-in Claude account; billed cost unavailable"}
        record["bridge_events"] = session.events()
    except Exception as exc:
        record["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        record["elapsed_s"] = time.monotonic() - started
        if session:
            cleanup = session.close()
            if cleanup: record["cleanup_errors"] = cleanup; record["outcome"] = "ERROR"
        record["relay_usage"] = relay_usage(Path(directory) / "relay_trace.jsonl")
    return record
