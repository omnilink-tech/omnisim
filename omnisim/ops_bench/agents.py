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
"""Arms. Every arm receives operator text as it is spoken, concurrently.

`say()` never blocks: an operator does not wait for the robot to finish before
saying "stop". Whether a message preempts, queues or is ignored is the
behaviour under test, not something this runner decides.
"""
from __future__ import annotations

import threading
import time

from omnisim.control_bench.engine import InfrastructureError
from .suite import render, resolve

from .competitors import COMPETITOR_ARMS

ARMS = ("omnilink", "oracle", "oracle_bad", *COMPETITOR_ARMS, "codex_full", "claude_full")

import re as _re
PROVIDER_FAILURE = _re.compile(
    r"rate-limited|rate limited|overloaded|prepayment|BYOK_REQUIRED|model-provider key|"
    r"PRIMARY_ENGINE_FAILED|HTTP 429|HTTP 50[023]|transient", _re.IGNORECASE)

# The complete OmniLink control experience in OmniSim, as shipped: the
# bridge's /prompt (parser first, relay model on what it declines, the gate on
# whatever produced the frames), with deferred intents, the action journal,
# the bridge event stream and event wakes ON. Memory, profile sync and the
# edge connector stay OFF: they write to a shared hosted profile and would
# leak one episode into the next. That is disclosed, not hidden.
OMNILINK_ENV = {"OMNILINK_PRESENCE": "1", "OMNILINK_EVENT_WAKE": "1",
                "OMNILINK_MEMORY": "0", "OMNILINK_PROFILE_SYNC": "0",
                "OMNILINK_EDGE": "0", "OMNILINK_USAGE": "0"}


class Reply:
    def __init__(self, step_id, text, t_sent):
        self.step_id, self.prompt, self.t_sent = step_id, text, t_sent
        self.t_done = None; self.text = ""; self.raw = None; self.error = None
        self.done = threading.Event()

    def as_record(self):
        return {"step": self.step_id, "prompt": self.prompt, "t_sent": self.t_sent,
                "t_done": self.t_done, "text": self.text, "error": self.error, "raw": self.raw}


class OmniLinkAgent:
    name = "omnilink"
    env = OMNILINK_ENV

    def __init__(self, session, clock, **_):
        self.session, self.clock = session, clock

    def say(self, step, text, ctx):
        r = Reply(step["id"], text, self.clock())
        def run():
            try:
                out = self.session.post("/prompt", {"text": text, "timeout_s": 150}, timeout=170)
                r.raw = out
                r.text = str(out.get("response") or out.get("error") or "")
                # An error with no answer is unanswered, even when the bridge
                # prefixed the reply with its halt note ("I stopped what I was
                # doing first.") -- that sentence is the bridge's, not an answer
                # to the operator (omnilink-f3-interrupt-01 graded it as one).
                answer = str(out.get("response") or "")
                if out.get("halted_first"):
                    answer = answer.replace("I stopped what I was doing first.", "", 1)
                if out.get("error") and not answer.strip():
                    r.error = str(out.get("error"))
                # The PROVIDER failed, not the agent: a rate limit, an
                # overload, depleted credit, a missing provider key. That is
                # infrastructure (an ERROR episode, re-run per the
                # preregistration), never a FAIL charged to the arm.
                if out.get("error") and PROVIDER_FAILURE.search(str(out.get("error"))):
                    r.error = "transport:" + str(out.get("error"))[:200]
            except InfrastructureError as exc:
                r.error = "timeout" if "timed out" in str(exc).lower() else f"transport:{exc}"
            except Exception as exc:  # recorded, never swallowed silently
                r.error = f"{type(exc).__name__}:{exc}"
            finally:
                r.t_done = self.clock(); r.done.set()
        threading.Thread(target=run, daemon=True).start()
        return r

    def close(self):
        pass


class OracleAgent:
    """Calibration ONLY: scripted typed calls prove fixtures and graders.

    `oracle` plays the behaviour a correct agent would show; `oracle_bad`
    plays a named WRONG behaviour so every check is seen to go red at least
    once. Neither ever produces agent evidence. Typed calls go to /tool with
    no utterance: they calibrate the fixture, not the language gate.
    """
    env = {}

    def __init__(self, session, clock, bad=False, **_):
        self.session, self.clock, self.bad = session, clock, bad
        self.name = "oracle_bad" if bad else "oracle"
        # A later message may cancel what an EARLIER script has not done
        # yet, exactly as a correct agent drops the rest of a plan. A cancel
        # is a generation bump: it stops scripts that started before it and
        # never the ones that start after -- it used to be a flag that was
        # never cleared, so one interruption in a long shift silently skipped
        # every later message's script (holdout-calibration-01, F9).
        self._gen_lock = threading.Lock()
        self.cancel_gen = 0

    def say(self, step, text, ctx):
        r = Reply(step["id"], text, self.clock())
        script = step.get("calibration_bad" if self.bad and "calibration_bad" in step else "calibration", [])
        with self._gen_lock:
            my_gen = self.cancel_gen
        def run():
            outs = []
            try:
                for a in script:
                    tool = a["tool"]
                    if tool == "_cancel":
                        with self._gen_lock:
                            self.cancel_gen += 1
                        continue
                    if self.cancel_gen > my_gen and not a.get("after_cancel"):
                        break
                    if tool == "_wait":
                        time.sleep(float(a["args"]["s"])); continue
                    if tool == "_reply":
                        r.text = self._reply_text(a["args"], ctx); continue
                    args = {k: resolve(v, ctx["pose0"]) if isinstance(v, str) else v
                            for k, v in a["args"].items()}
                    if tool == "stop_robot":
                        # The bridge serialises /tool and /prompt behind one
                        # action lock; /stop_robot is the only route exempt
                        # from it (the escape hatch). A correct agent that
                        # must halt a running motion uses it, so the oracle
                        # does too. That /tool stop_robot cannot preempt is
                        # a PRODUCT finding, recorded in FINDINGS.md.
                        out = self.session.post("/stop_robot", {}, timeout=30)
                    else:
                        out = self.session.tool(tool, args)
                    outs.append({"tool": tool, "args": args, "out": out})
            except Exception as exc:
                r.error = f"{type(exc).__name__}:{exc}"
            finally:
                r.raw = {"calibration": outs}
                r.t_done = self.clock(); r.done.set()
        threading.Thread(target=run, daemon=True).start()
        return r

    def _reply_text(self, args, ctx):
        if args.get("path_length_so_far"):
            from .suite import path_length
            return (args.get("prefix", "") +
                    f"I have driven {path_length(ctx['sampler'].samples):.2f} metres in total.")
        if args.get("count_visits_so_far"):
            # A correct robot reports the TRUE count, measured the way the
            # grader measures it -- a scripted literal was wrong whenever a
            # correct route passed through the station once more than planned
            # (shift-v2-calibration-03, 1 run in 4).
            from .suite import visits
            cv = args["count_visits_so_far"]
            n = visits(ctx["sampler"].samples, resolve(cv["at"], ctx["pose0"]), float(cv.get("radius", .5)))
            return args.get("prefix", "") + f"{n} times." + args.get("suffix", "")
        return render(args.get("text", ""), ctx["pose0"])

    def close(self):
        pass


def make_arm(name, session, clock, model_cfg=None):
    if name == "codex_full":
        from .codex_agent import CodexAgent
        return CodexAgent(session, clock, model_cfg or {})
    if name == "claude_full":
        from .claude_agent import ClaudeAgent
        return ClaudeAgent(session, clock, model_cfg or {})
    if name == "omnilink": return OmniLinkAgent(session, clock)
    if name in ("oracle", "oracle_bad"): return OracleAgent(session, clock, bad=name == "oracle_bad")
    if name in COMPETITOR_ARMS:
        from .competitors import CompetitorAgent, ModelClient
        cfg = model_cfg or {}
        if not cfg.get("key") or not cfg.get("engine"):
            raise ValueError("competitor arms need --engine (and the OmniKey)")
        client = ModelClient(cfg["key"], cfg["engine"], cfg.get("model", ""),
                             f"OmniSim-{session.robot}", int(cfg.get("max_requests", 60)),
                             log=cfg.setdefault("records", []))
        return CompetitorAgent(session, clock, name, client)
    raise ValueError(f"Unknown arm {name}")


def arm_env(name, model_cfg=None):
    """The relay's environment. OmniLink's relay gets the SAME engine and model
    the competitors are given, so the model is held constant across arms."""
    if name != "omnilink":
        return {}
    env = dict(OMNILINK_ENV)
    cfg = model_cfg or {}
    if cfg.get("engine"):
        env["OMNILINK_ENGINE"] = cfg["engine"]
        env["OMNILINK_MODEL"] = cfg.get("model", "")
    return env
