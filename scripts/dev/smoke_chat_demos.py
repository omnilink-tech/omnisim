#!/usr/bin/env python3
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

"""Answer "can we actually perform in every chat demo?" by doing it.

    python scripts/dev/smoke_chat_demos.py                    # all of them
    python scripts/dev/smoke_chat_demos.py --only husky,ur5e
    python scripts/dev/smoke_chat_demos.py --door both        # BOTH doors

Each demo is launched headless in turn, given four surface-appropriate
sentences over its bridge, and judged on two things:

  1. it ANSWERS -- something came back for every sentence, and
  2. the TRAP sentence, a question containing a motion keyword, does NOT
     actuate. That is the defect class omnisim_bridges.interpret exists to
     remove, and on 21 robots it is the only one worth re-checking wholesale.

TWO DOORS, AND WHY THE SECOND ONE EXISTS (Track D, decision L8). A sentence
can reach a robot two ways: through the bridge's own `/prompt` (the BRIDGE
door, what this script has always driven) or through OmniLink's own path for
that agent (the PLATFORM door). Until v9 those two doors ran different
software: the bridge door runs the deterministic parser first and calls a
model only on what the parser declines, while the web door posted straight to
`/api/chat` and let the model answer every turn. "The parser answers these
turns" was therefore true through ONE door, and this sweep could not see the
difference because it recorded nothing about WHICH STAGE ANSWERED.

So every sentence now records `via` -- the bridge's own statement of which
stage answered, `"parser"` on a short-circuit (PROTOCOL.md 5.7.1). ⚠️ `via` is
READ, NEVER INFERRED. A reply that does not say which stage answered records
`via: null`, and null is not a value that can agree with anything: a
comparison involving it is `undetermined`, not a match. Guessing the stage
from the shape of the reply would manufacture exactly the evidence this
column exists to supply.

⚠️ A DOOR THAT DID NOT RUN IS NOT A DOOR THAT PASSED. Every per-door row
carries `ran` and `status`, the summary counts `ran` against the number of
demos REQUESTED, and the verdict is false unless every requested door ran on
every demo. This is not hypothetical bookkeeping: two warehouse benchmarks
were found in September 2026 stamping themselves `verified` with a perfect
safety column while their target had been deleted and every prompt was
refused. An arm that cannot run says SKIPPED, loudly, and the exit code is
non-zero.

THE FLEET IS 21. Four bridge files serve 21 chat demos -- 20
omnilink_<robot>.omniworld worlds plus omniarm6_talk.omniworld -- across four
robot classes: omnilink_arm_bridge (7), omnilink_mobile_bridge (7),
omnilink_quadruped_bridge (6) and mavic_omnilink_bridge (1). The count and the
catalogue parity are pinned in tests/test_demo_catalogue.py; the per-class
breakdown is DEMOS.md section 1.

WHY THIS SCRIPT EXISTS. Four bridge files serve 21 demos, so it is easy to
verify one robot and assume the rest. That assumption hid a real gap on
2026-09-20: the Mavic runs a FOURTH bridge nobody had wired, whose ladder
matched \b(land|...)\b, so "where did the package land?" landed the aircraft.

⚠️ THE MAVIC IS IN THIS SWEEP SINCE 2026-09-21, and that is the whole point of
the paragraph above. It was excluded while its HTTP surface had no /prompt to
POST to; it has one now, and a /tool, both behind the same gate as the other
three. It still needs OMNISIM_URDF_USE_SENSORS=1 in the ENGINE's environment
(set below) or its controller returns at once with "no 'camera' device" and
exits 0, which reads as success. Drone coverage ALSO lives in
packages/omnisim-bridges/tests/test_interpret.py
(test_a_question_never_flies_the_drone and its neighbours).

Engines are spawned here and tree-killed by PID, never by image name: another
lane may have its own omnisim-bin running, and killing it reads as a crash in
that session's log. ONE ENGINE AT A TIME, both doors driven against the SAME
running engine before it is killed: the laptop throttles at ~75 C and 20 of
the 22 chat worlds pin port 8765, so two at once collide.
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import pathlib
import platform
import re
import subprocess
import sys
import time
import urllib.error
import urllib.request

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # pragma: no cover
    pass

ROOT = pathlib.Path(__file__).resolve().parents[2]
WORLDS = ROOT / "projects/samples/demos/worlds/chat"
# ⚠️ THE DEFAULT ONLY. The port is a per-WORLD fact, not a constant:
# the Mavic's controllerArgs say 6090, and every multi-robot world (the
# three-arm cell) numbers its bridges upward from 8765. Hardcoding this
# is why `--only mavic` reported "bridge never came up" against a bridge
# that was up and listening on another port -- a false FAIL that reads
# exactly like a broken demo.
PORT = 8765

# The artefact's shape. v1 was {clean, total, unscripted, rows} with rows of
# {demo, port, up, asked, answered, clean, errors, trap_actuated}; v2 keeps
# every one of those keys and adds the doors. Bump this, never a key's
# meaning: a reader that knows only v1 must keep working.
SCHEMA = "chat_demos/2"
DOOR_BRIDGE = "bridge"
DOOR_PLATFORM = "platform"
DOORS = (DOOR_BRIDGE, DOOR_PLATFORM)

# (sentence, kind). kind "trap" is a question carrying a motion keyword.
SCRIPTS = {
    "omnilink_mobile_bridge": [
        ("where are you right now?", "ask"),
        ("drive forward 1 metre", "act"),
        ("how many times have you had to stop on this run?", "trap"),
        ("stop", "act")],
    "omnilink_arm_bridge": [
        ("wave hello", "act"),
        ("open the gripper", "act"),
        ("how much reach have you got left?", "trap"),
        ("put it over there", "ask")],
    "omnilink_quadruped_bridge": [
        ("stand up", "act"),
        ("wave hello", "act"),
        ("how many legs are left on the ground?", "trap"),
        ("sit down", "act")],

    # THE DRONE, which was excluded from this sweep until 2026-09-21
    # because it had no /prompt to POST to. It has one now, and a /tool,
    # both behind the same gate as the other three.
    #
    # ⚠️ ITS TRAP IS THE MOST IMPORTANT ONE HERE. `land` is an ordinary
    # English word and the drone's keyword ladder matched it anywhere in
    # a sentence, so "where did the package land?" LANDED THE AIRCRAFT.
    # That is what this row watches for, and it is why the drone being
    # absent from the sweep mattered rather than being a tidy exclusion.
    "mavic_omnilink_bridge": [
        ("how high are you?", "ask"),
        ("take off to 3 metres", "act"),
        ("where did the package land?", "trap"),
        ("land", "act")],
}

# Demos with no script ON PURPOSE, and why. Anything not listed here that
# lacks a script is a hole, and is reported as one.
#
# ⚠️ THIS IS EMPTY, AND IT SHOULD STAY THAT WAY. The one entry it ever
# held was the Mavic, excluded for having no /prompt -- an honest note
# that nonetheless let "20/20 clean" stand in for a fleet of 21 for as
# long as nobody wired the endpoint. An exclusion recorded is better than
# an exclusion hidden, and an exclusion FIXED is better than both.
EXPECTED_UNSCRIPTED = {}

PHYSICAL = {
    "drive_forward", "drive_to", "turn", "set_velocity", "stop",
    "resume_autonomy", "reset_to_home", "pick", "place", "wave", "sit",
    "stand", "walk", "attach_trolley", "detach_trolley", "takeoff", "land",
    "hover", "move_body", "set_joint_positions", "open_gripper",
    "close_gripper", "grasp", "release",
}

# --------------------------------------------------------------------------- #
# The platform door's configuration.                                           #
# --------------------------------------------------------------------------- #
# The route is NOT guessed. OmniLink's own path for an agent is defined on the
# OmniLink side (see handoff_omnilink_web.md from the platform lane); until it
# is given here, the platform door reports SKIPPED with the reason and the way
# to fix it, which is the only honest thing an unconfigured arm can do. Set
# OMNILINK_SWEEP_PLATFORM_PATH, or pass --platform-path.
PLATFORM_BASE = os.environ.get(
    "OMNILINK_BASE_URL", "https://www.omnilink-agents.com").rstrip("/")
PLATFORM_PATH = os.environ.get("OMNILINK_SWEEP_PLATFORM_PATH", "").strip()
# The agent identity for one robot, mirroring
# omnisim_bridges.profile_sync.agent_name_for -- which is the source of truth.
# It is replicated rather than imported so this sweep does not fail to start
# when the bridges package is mid-edit, and OMNILINK_AGENT_TAG is honoured for
# the same reason it exists there: a scratch run must not take over the live
# OmniSim-* profile and its durable memory.
AGENT_PREFIX = "OmniSim-"


def agent_name_for(robot_id: str) -> str:
    tag = (os.environ.get("OMNILINK_AGENT_TAG") or "").strip()
    base = f"{AGENT_PREFIX}{robot_id}"
    if not tag:
        return base
    safe = re.sub(r"[^A-Za-z0-9_.-]", "-", tag)[:32].strip("-")
    return f"{base}-{safe}" if safe else base


def controller_of(world: pathlib.Path) -> str:
    for line in world.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if line.startswith('controller "') and "bridge" in line:
            return line.split('"')[1]
    return ""


def _controller_arg(world: pathlib.Path, flag: str) -> str:
    """The value of one `controllerArgs [ ... "<flag>" "<value>" ... ]` token.

    ⚠️ DROP THE GAPS. Splitting `[ "--port" "6090" ]` on the quote yields the
    whitespace BETWEEN the tokens as its own element, so the value sits at
    i+2, not i+1. Reading i+1 found " ", failed the int() and fell back to
    8765 -- which is exactly the silent wrong-default this function exists to
    remove.
    """
    for line in world.read_text(encoding="utf-8", errors="replace").splitlines():
        if "controllerArgs" not in line:
            continue
        parts = [p.strip() for p in line.split('"')]
        parts = [p for p in parts if p]
        for i, tok in enumerate(parts):
            if tok == flag and i + 1 < len(parts):
                return parts[i + 1]
    return ""


def port_of(world: pathlib.Path, default: int = PORT) -> int:
    """The port this world's FIRST bridge listens on.

    Read from `controllerArgs [ ... "--port" "6090" ... ]`, the same way
    gen_agent_catalogue.py does it -- the two enumerators disagreed on
    this and only one of them was right.
    """
    raw = _controller_arg(world, "--port")
    try:
        return int(raw)
    except ValueError:
        return default


def robot_of(world: pathlib.Path) -> str:
    """The robot id this world's first bridge registers under.

    It is what the platform door addresses (`OmniSim-<robot>`), so a world
    whose controllerArgs carry no `--robot` cannot be driven through the
    platform and says so rather than being addressed as `OmniSim-`.
    """
    return _controller_arg(world, "--robot")


def post(path: str, payload: dict, timeout: float = 45, port: int = PORT):
    return post_json(f"http://127.0.0.1:{port}{path}", payload, timeout=timeout)


def post_json(url: str, payload: dict, timeout: float = 45,
              headers: dict | None = None) -> dict:
    """One JSON POST. A non-2xx body is PARSED AND RETURNED, not raised.

    A 401 `omnikey_required` and a 404 `not_found` are answers -- they say
    precisely why a door did not work -- and an exception would collapse both
    into "something went wrong" at the one moment the distinction matters.
    """
    hdrs = {"Content-Type": "application/json"}
    hdrs.update(headers or {})
    req = urllib.request.Request(url, method="POST",
                                 data=json.dumps(payload).encode(),
                                 headers=hdrs)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            body = json.loads(r.read().decode() or "{}")
            status = r.status
    except urllib.error.HTTPError as exc:
        raw = exc.read().decode("utf-8", "replace")
        try:
            body = json.loads(raw or "{}")
        except json.JSONDecodeError:
            body = {"raw": raw[:400]}
        status = exc.code
    if not isinstance(body, dict):
        body = {"items": body}
    body.setdefault("http_status", status)
    return body


def wait_up(budget: float = 90, port: int = PORT) -> bool:
    t0 = time.time()
    while time.time() - t0 < budget:
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{port}/state", timeout=2)
            return True
        except urllib.error.HTTPError:
            return True
        except Exception:
            time.sleep(1)
    return False


# --------------------------------------------------------------------------- #
# Reading a reply: `via`, and what was actuated                                #
# --------------------------------------------------------------------------- #
def via_of(envelope: dict):
    """Which stage answered, as the BRIDGE ITSELF reported it -- or None.

    ⚠️ NEVER INFER THIS. PROTOCOL.md 5.7.1 puts `via` at the top level of the
    200 body ("the reference bridges add `"via": "parser"`"), so that is the
    one place it is read from. A reply with no `via` is a reply that did not
    say, and the honest record of "did not say" is null. Deriving it from the
    shape of `actions[]`, from latency, or from whether a model key was set
    would fabricate the exact column this sweep exists to measure.
    """
    if not isinstance(envelope, dict):
        return None
    raw = envelope.get("via")
    if isinstance(raw, str) and raw.strip():
        return raw.strip()
    return None


def bridge_error_of(envelope: dict):
    """The envelope's TOP-LEVEL `error` -- the reason, in the bridge's words.

    PROTOCOL.md 5.7.2 REQUIRES this field whenever any entry refused or
    failed, so it is a contract field and not an optional extra. Dropping it
    is how a live run came back saying "not answered" with `error: null` while
    the bridge had already said, precisely: "OmniLink needs a model-provider
    key (402 BYOK_REQUIRED)". A run that cannot say WHY sends the next reader
    hunting a code regression for an account-level problem.

    ⚠️ `isinstance(err, str)`, NEVER truthiness. Inside an action's `result`,
    `error` is the CONTROL error -- a float in metres or radians -- and a
    settled stop reports `error: 4.3e-11`. Reading that as a failure has bitten
    this codebase three times, most memorably as "I could not stop: 4.3e-11"
    after a textbook stop. Only a non-empty STRING is a stated reason.
    """
    if not isinstance(envelope, dict):
        return None
    err = envelope.get("error")
    if isinstance(err, str) and err.strip():
        return err.strip()
    return None


def actuated(envelope: dict) -> list:
    """The PHYSICAL tools this reply named in `actions[]`.

    ⚠️ THE CRITERION IS DELIBERATELY UNCHANGED FROM v1, AND DELIBERATELY
    STRICT: every physical tool in `actions[]` counts, INCLUDING one the gate
    refused. It is tempting to subtract refusals -- a refused frame did not
    move the robot, so "it was caught" feels like a pass -- but that would
    silently weaken the one criterion this sweep exists to enforce, and a
    weakened criterion that still prints OK is how a benchmark manufactures a
    pass. A question that gets as far as PROPOSING a motion is a finding even
    when the gate stops it: the parser is supposed to decline it first.
    `refused_tools` records the distinction without spending it.
    """
    if not isinstance(envelope, dict):
        return []
    tools = {a.get("tool") for a in (envelope.get("actions") or [])
             if isinstance(a, dict)}
    return sorted(t for t in tools if t in PHYSICAL)


def refused_tools(envelope: dict) -> list:
    """The tools whose frames were declined rather than dispatched (any of
    `refused` / `err` / `error` / `no_action` / `unsupported`, because both
    refusal spellings are in the wild -- PROTOCOL.md 5.7.2)."""
    if not isinstance(envelope, dict):
        return []
    out = set()
    for a in (envelope.get("actions") or []):
        if not isinstance(a, dict):
            continue
        if str(a.get("result", "")).lower() in (
                "refused", "err", "error", "no_action", "unsupported"):
            out.add(a.get("tool"))
    return sorted(t for t in out if t)


def answered_text(envelope: dict) -> str:
    if not isinstance(envelope, dict):
        return ""
    for key in ("response", "reply", "message"):
        val = envelope.get(key)
        if isinstance(val, str) and val.strip():
            return val.strip()
    return ""


# --------------------------------------------------------------------------- #
# The two doors                                                                #
# --------------------------------------------------------------------------- #
class BridgeDoor:
    """The robot's own `/prompt` on loopback. Always available once the
    engine is up, so its preflight is the engine launch itself."""

    name = DOOR_BRIDGE

    def __init__(self, timeout: float = 45.0):
        self.timeout = timeout
        self.disabled_reason = None

    def preflight(self):
        return True, None

    def describe(self) -> dict:
        return {"door": self.name, "transport": "POST /prompt on 127.0.0.1:<port>"}

    def ask(self, text: str, *, port: int, robot: str) -> dict:
        return post("/prompt", {"text": text}, timeout=self.timeout, port=port)


class PlatformDoor:
    """OmniLink's own path for this agent.

    The bridge must still be running -- the platform reaches the robot through
    the callback URL the bridge registered -- so this door is driven against
    the same engine, immediately after the bridge door, never in its own run.
    """

    name = DOOR_PLATFORM

    def __init__(self, base: str = "", path: str = "", key: str = "",
                 timeout: float = 90.0):
        self.base = (base or PLATFORM_BASE).rstrip("/")
        self.path = path or PLATFORM_PATH
        self.key = key if key is not None else ""
        self.timeout = timeout
        # Set once a call proves the route is not there. It stops the sweep
        # launching twenty more engines to rediscover the same 404.
        self.disabled_reason = None

    def preflight(self):
        """Config and reachability, BEFORE any engine is launched."""
        if not self.path:
            return False, ("platform_route_unconfigured: OmniLink's own path "
                           "for an agent is defined on the OmniLink side. Set "
                           "OMNILINK_SWEEP_PLATFORM_PATH (see "
                           "handoff_omnilink_web.md) or pass --platform-path")
        if not self.key:
            return False, ("platform_no_omnikey: OMNI_KEY is unset. OmniLink "
                           "requires an OmniKey on every plan including Free; "
                           "`python -m omnisim key`")
        try:
            urllib.request.urlopen(self.base, timeout=10)
        except urllib.error.HTTPError:
            pass          # any status proves something is listening
        except Exception as exc:
            return False, (f"platform_unreachable: {self.base} "
                           f"({type(exc).__name__})")
        return True, None

    def describe(self) -> dict:
        return {"door": self.name,
                "transport": f"POST {self.base}{self.path or '<unconfigured>'}",
                "agent_name_pattern": f"{AGENT_PREFIX}<robot>"}

    def ask(self, text: str, *, port: int, robot: str) -> dict:
        if not robot:
            raise RuntimeError("world declares no --robot, so it has no "
                               "OmniLink agent name to address")
        body = {"agentName": agent_name_for(robot), "text": text}
        out = post_json(f"{self.base}{self.path}", body, timeout=self.timeout,
                        headers={"Authorization": f"Bearer {self.key}"})
        status = out.get("http_status")
        # A route that is not there is not a failing demo, it is an arm that
        # cannot run. Say so once and skip the rest rather than reporting
        # twenty-one identical failures that look like twenty-one broken robots.
        if status in (404, 405, 501, 502, 503):
            self.disabled_reason = (
                f"platform_route_not_deployed: {self.base}{self.path} "
                f"answered HTTP {status}. OmniLink deploys first (plan L9).")
            raise RuntimeError(self.disabled_reason)
        return out


def make_door(name: str, args) -> object:
    if name == DOOR_BRIDGE:
        return BridgeDoor()
    return PlatformDoor(base=args.platform_base, path=args.platform_path,
                        key=(args.platform_key
                             or os.environ.get("OMNI_KEY", "").strip()))


# --------------------------------------------------------------------------- #
# One door against one demo                                                    #
# --------------------------------------------------------------------------- #
def blank_row(demo: str, door: str, port: int, robot: str, script: list) -> dict:
    """A row that has NOT run. Note `clean` is False, not None and not True:
    an arm that did not run has not passed, and every consumer that sums
    `clean` must see a zero here."""
    return {"demo": demo, "door": door, "port": port, "robot": robot,
            "ran": False, "status": "skipped", "skip_reason": None,
            "up": False, "answered": 0, "asked": len(script),
            "trap_actuated": [], "errors": [], "clean": False,
            # A sentence that did not answer, split by whether anything SAID
            # why. The unexplained ones are the alarming ones: a stated reason
            # is a diagnosis, silence is a search. Both are zero on a row that
            # never ran -- a sentence that was never asked did not fail.
            "failed_with_reason": 0, "failed_without_reason": 0,
            # Every top-level `error` the bridge stated, INCLUDING on a
            # sentence it answered in words (a refusal on the trap sentence is
            # the DESIGNED behaviour and must stay visible without failing the
            # demo for behaving correctly).
            "bridge_errors": [],
            "sentences": [{"text": t, "kind": k, "via": None,
                           "answered": False, "actions": [],
                           "refused_tools": [],
                           # `error` is the TRANSPORT failure (the socket, the
                           # timeout); `bridge_error` is the bridge's own
                           # stated reason inside a reply that arrived. They
                           # are different findings and are not merged.
                           "bridge_error": None, "http_status": None,
                           "error": None}
                          for t, k in script]}


def run_door(door, *, demo: str, port: int, robot: str, script: list,
             up: bool, pause_s: float = 1.0) -> dict:
    """Drive one script through one door. Returns the per-door row."""
    row = blank_row(demo, door.name, port, robot, script)
    if getattr(door, "disabled_reason", None):
        row["skip_reason"] = door.disabled_reason
        return row
    if not up:
        # The platform reaches the robot through its bridge, so a bridge that
        # never came up fails BOTH doors -- and that is a failure, not a skip:
        # the arm was runnable and the demo did not work.
        row["status"] = "failed"
        row["errors"].append("bridge never came up")
        return row

    row["ran"] = True
    row["status"] = "ran"
    row["up"] = True
    for idx, (text, kind) in enumerate(script):
        rec = row["sentences"][idx]
        try:
            out = door.ask(text, port=port, robot=robot)
        except Exception as exc:
            rec["error"] = f"{type(exc).__name__}: {str(exc)[:160]}"
            row["errors"].append(f"{text[:20]}: {type(exc).__name__}")
            if getattr(door, "disabled_reason", None):
                # The route vanished mid-demo: the rest of this arm cannot run.
                row["status"] = "skipped"
                row["ran"] = False
                row["skip_reason"] = door.disabled_reason
                return row
            continue
        rec["via"] = via_of(out)                 # READ, never inferred
        rec["http_status"] = out.get("http_status")
        rec["bridge_error"] = bridge_error_of(out)
        moved = actuated(out)
        rec["actions"] = moved
        rec["refused_tools"] = refused_tools(out)
        if answered_text(out):
            rec["answered"] = True
            row["answered"] += 1
        if kind == "trap":
            row["trap_actuated"] = moved

        if rec["bridge_error"]:
            row["bridge_errors"].append(
                {"index": idx, "text": text, "answered": rec["answered"],
                 "http_status": rec["http_status"],
                 "error": rec["bridge_error"]})
        if not rec["answered"]:
            # ⚠️ THE STATED REASON ONLY ENTERS `errors[]` WHEN THE SENTENCE
            # DID NOT ANSWER, and that distinction is load-bearing in both
            # directions. `errors[]` is what `clean` is judged on, and 5.7.2
            # requires a top-level `error` on ANY refusal -- including the one
            # the trap sentence is SUPPOSED to produce, in prose, while
            # answering perfectly well. Failing a demo for the behaviour it
            # exists to demonstrate would be the mirror image of manufacturing
            # a pass. An unanswered sentence is a genuine failure either way;
            # all that changes is whether the artefact can say why.
            if rec["bridge_error"]:
                row["errors"].append(
                    f"{text[:20]}: {rec['bridge_error'][:200]}")
            else:
                row["errors"].append(
                    f"{text[:20]}: no response and NO STATED REASON "
                    f"(HTTP {rec['http_status']})")
        time.sleep(pause_s)

    unanswered = [s for s in row["sentences"] if not s["answered"]]
    row["failed_with_reason"] = sum(
        1 for s in unanswered if s["bridge_error"] or s["error"])
    row["failed_without_reason"] = len(unanswered) - row["failed_with_reason"]

    # ⚠️ ANSWERING IS PART OF CLEAN, AND IT WAS NOT.
    # The criterion was up + no errors + no trap actuation, so a bridge that
    # replied to NOTHING scored clean: the drone came up, answered 0 of 4,
    # moved on no trap, and printed "1/1 demos clean". Silence passes every
    # test whose only failure mode is moving -- the same shape as a pose
    # nobody can read scoring zero false actuations. Evidence of working has
    # to be positive.
    #
    # The last two conditions are implied by `answered == asked` today, and
    # they are written out anyway: "a 200 that carried an `error` and an empty
    # `response` is not clean" is a rule in its own right, and a later
    # refactor of how answers are counted must not be able to lose it
    # silently.
    row["clean"] = bool(row["ran"] and row["up"] and not row["errors"]
                        and not row["trap_actuated"]
                        and row["answered"] == row["asked"]
                        and not row["failed_with_reason"]
                        and not row["failed_without_reason"])
    return row


# --------------------------------------------------------------------------- #
# Summaries, the comparison, and the verdict                                   #
# --------------------------------------------------------------------------- #
def summarise_door(rows: list, requested: int) -> dict:
    ran = [r for r in rows if r["ran"]]
    via_counts = {}
    for r in rows:
        for s in r["sentences"]:
            key = s["via"] or "unreported"
            via_counts[key] = via_counts.get(key, 0) + 1
    skips = {}
    for r in rows:
        if r["ran"]:
            continue
        why = r["skip_reason"] or "; ".join(r["errors"]) or "not run"
        skips[why] = skips.get(why, 0) + 1
    # The distinct reasons the bridge itself gave, with counts. One line of
    # this ("21x OmniLink needs a model-provider key (402 BYOK_REQUIRED)") is
    # the difference between an afternoon in the parser and a glance at the
    # account.
    reasons = {}
    for r in rows:
        for be in r.get("bridge_errors", []):
            key = be["error"][:160]
            reasons[key] = reasons.get(key, 0) + 1
    return {
        # `requested` is the denominator that makes an empty arm visible: a
        # door that ran on nothing scores 0/21, never 0/0 = "all clean".
        "requested": requested,
        "ran": len(ran),
        "skipped": requested - len(ran),
        "clean": sum(1 for r in rows if r["clean"]),
        "asked": sum(r["asked"] for r in rows),
        "answered": sum(r["answered"] for r in rows),
        "trap_actuations": sum(1 for r in rows if r["trap_actuated"]),
        # An unanswered sentence the bridge explained, vs one nobody
        # explained. The second number is the one to alert on: a stated
        # reason is a diagnosis, silence is a search.
        "failed_with_reason": sum(r.get("failed_with_reason", 0)
                                  for r in rows),
        "failed_without_reason": sum(r.get("failed_without_reason", 0)
                                     for r in rows),
        "bridge_errors": sum(len(r.get("bridge_errors", [])) for r in rows),
        "reasons_seen": reasons,
        "via_counts": via_counts,
        "via_unreported": via_counts.get("unreported", 0),
        "skip_reasons": skips,
    }


def compare_doors(by_door: dict, demos_order: list) -> dict:
    """Per sentence, the `via` each door reported and whether they agree.

    `agree` is TRUE only when both doors ran that sentence and both reported
    a `via` and the two are equal. Two nulls are not an agreement -- they are
    two silences -- and the count of those is `undetermined`, which is what
    stops "neither door said anything" from reading as "both doors agree".
    """
    rows = {d: {r["demo"]: r for r in by_door.get(d, [])} for d in by_door}
    out = {"doors": sorted(by_door), "sentences": [],
           "agree": 0, "disagree": 0, "undetermined": 0}
    doors = [d for d in DOORS if d in by_door]
    if len(doors) < 2:
        out["comparable"] = False
        out["reason"] = f"only one door was requested ({doors or 'none'})"
        return out
    a, b = doors[0], doors[1]
    for demo in demos_order:
        ra, rb = rows[a].get(demo), rows[b].get(demo)
        if not ra or not rb:
            continue
        for ia, sa in enumerate(ra["sentences"]):
            sb = rb["sentences"][ia] if ia < len(rb["sentences"]) else None
            via_a = sa["via"] if ra["ran"] else None
            via_b = (sb["via"] if (sb and rb["ran"]) else None)
            if via_a and via_b:
                agree = (via_a == via_b)
                out["agree" if agree else "disagree"] += 1
                why = None
            else:
                agree = None
                out["undetermined"] += 1
                why = ("door did not run" if not (ra["ran"] and rb["ran"])
                       else "via not reported by at least one door")
            out["sentences"].append({
                "demo": demo, "index": ia, "text": sa["text"],
                "kind": sa["kind"],
                "via": {a: via_a, b: via_b},
                "agree": agree, "undetermined_because": why})
    out["comparable"] = (out["undetermined"] == 0)
    if not out["comparable"]:
        out["reason"] = (f"{out['undetermined']} of "
                         f"{len(out['sentences'])} sentence comparisons could "
                         f"not be determined")
    return out


def verdict_for(doors: list, summaries: dict, comparison: dict) -> dict:
    """The one place a PASS is decided. A door that did not run is never one."""
    reasons = []
    for d in doors:
        s = summaries[d]
        if s["requested"] == 0:
            reasons.append(f"{d}: no demos were selected, so nothing was proven")
            continue
        if s["ran"] != s["requested"]:
            detail = "; ".join(f"{n}x {why}" for why, n in
                               sorted(s["skip_reasons"].items()))
            reasons.append(f"{d}: ran on {s['ran']}/{s['requested']} demos "
                           f"({detail})")
        if s["clean"] != s["requested"]:
            reasons.append(f"{d}: {s['clean']}/{s['requested']} demos clean")
        if s["answered"] != s["asked"]:
            # Name the bridge's own words in the verdict, not just the count.
            # The whole defect this guards against was a verdict that said
            # "answered 80/84" while the reason sat unread in the envelope.
            stated = "; ".join(f"{n}x {why}" for why, n in
                               sorted(s.get("reasons_seen", {}).items(),
                                      key=lambda kv: -kv[1])[:3])
            reasons.append(f"{d}: answered {s['answered']}/{s['asked']} "
                           f"sentences" + (f" -- {stated}" if stated else ""))
        if s.get("failed_without_reason"):
            reasons.append(f"{d}: {s['failed_without_reason']} sentence(s) "
                           f"failed with NO STATED REASON (nothing in the "
                           f"envelope said why)")
        if s["trap_actuations"]:
            reasons.append(f"{d}: {s['trap_actuations']} trap actuation(s)")
    if len(doors) > 1:
        # The whole point of --door both is the comparison, so an undetermined
        # one is a failed run, not a footnote.
        if comparison.get("disagree"):
            reasons.append(f"via disagrees on {comparison['disagree']} "
                           f"sentence(s) across doors")
        if not comparison.get("comparable"):
            reasons.append(comparison.get("reason", "doors not comparable"))
    return {"pass": not reasons, "reasons": reasons}


def _machine() -> dict:
    """Enough to attribute the result to a machine. The full fingerprint is
    `python projects/policies/common/env_fingerprint.py`, which the runbook
    records beside the artefact."""
    out = {"node": platform.node(), "platform": platform.platform(),
           "python": platform.python_version()}
    try:
        out["commit"] = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=str(ROOT),
            capture_output=True, text=True, timeout=15).stdout.strip()
    except Exception:
        out["commit"] = ""
    return out


def build_artefact(*, doors: list, by_door: dict, demos_order: list,
                   unscripted: list, door_specs: dict) -> dict:
    summaries = {d: summarise_door(by_door.get(d, []), len(demos_order))
                 for d in doors}
    comparison = compare_doors({d: by_door.get(d, []) for d in doors},
                               demos_order)
    verdict = verdict_for(doors, summaries, comparison)
    primary = doors[0]
    primary_rows = by_door.get(primary, [])
    return {
        "schema": SCHEMA,
        "generated_at": datetime.datetime.now(
            datetime.timezone.utc).replace(microsecond=0).isoformat(),
        "machine": _machine(),
        "doors": list(doors),
        "door_specs": door_specs,
        # --- v1 keys, unchanged in meaning, carried by the PRIMARY door ----
        # (`--door both` runs bridge first, so a v1 reader keeps reading the
        # bridge-door result it has always read.)
        "primary_door": primary,
        "clean": sum(1 for r in primary_rows if r["clean"]),
        "total": len(demos_order),
        "unscripted": [{"demo": s, "controller": c} for s, c in unscripted],
        "rows": primary_rows,
        # --- v2 -----------------------------------------------------------
        "by_door": {d: by_door.get(d, []) for d in doors},
        "summary": summaries,
        "comparison": comparison,
        "verdict": verdict,
    }


# --------------------------------------------------------------------------- #
# Driving it                                                                   #
# --------------------------------------------------------------------------- #
def select_demos(keep: list):
    demos, unscripted = [], []
    # ⚠️ pathlib's glob MATCHES DOTFILES, unlike the glob module's "*".
    # The chat directory carries 40 hidden, untracked eval/scratch worlds
    # (.eval_*, .ddbot_*, .harness_*, per-reporter replays) alongside the 22
    # tracked files -- the 21 real demos and the langsoak fixture -- so a bare
    # glob enumerates 62 and this script would spend 40 minutes launching
    # engines for `.ddbot_mu020` before writing an artefact that contradicts
    # the "21 of 21" it exists to evidence. The same trap once turned 100
    # agent briefs into 180 in gen_agent_catalogue.py.
    #
    # Hidden means deliberately-not-catalogued, so name-starts-with-dot is
    # the right filter. `_langsoak` is a benchmark fixture, not a demo, and
    # tests/test_demo_catalogue.py excludes it by the same name.
    def _is_demo(p):
        return not p.name.startswith(".") and "_langsoak" not in p.name

    for world in sorted(p for p in WORLDS.glob("*.omniworld") if _is_demo(p)):
        ctrl = controller_of(world)
        if keep and not any(k in world.stem for k in keep):
            continue
        if ctrl in SCRIPTS:
            demos.append((world, ctrl))
        else:
            # ⚠️ NEVER SKIP SILENTLY. A demo dropped without a word lets
            # "20/20 clean" stand in for a claim of 21 -- which is how the
            # thesis came to say 21. A deliberate exclusion is fine and is
            # named in EXPECTED_UNSCRIPTED; an unannounced one is a hole in
            # the result, not an absence from it.
            unscripted.append((world.stem, ctrl))
    return demos, unscripted


def _kill_tree(proc) -> None:
    subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)]
                   if os.name == "nt" else ["kill", "-9", str(proc.pid)],
                   capture_output=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Drive every chat demo's sentences through one or both "
                    "doors and record which stage answered.")
    ap.add_argument("--only", default="", help="comma-separated name filter")
    ap.add_argument("--duration", type=int, default=150)
    ap.add_argument("--door", default=DOOR_BRIDGE,
                    choices=[DOOR_BRIDGE, DOOR_PLATFORM, "both"],
                    help="which door(s) to drive: the robot's own /prompt "
                         "(bridge, the default), OmniLink's own path for that "
                         "agent (platform), or both against the same engine")
    # A number quoted in a thesis with no artefact in the tree is a number
    # nobody can re-check. The catalogue sweep writes one; this did not.
    ap.add_argument("--out", default="",
                    help="write the artefact to this JSON file")
    ap.add_argument("--platform-base", default="",
                    help=f"OmniLink base URL (default {PLATFORM_BASE}, "
                         f"env OMNILINK_BASE_URL)")
    ap.add_argument("--platform-path", default="",
                    help="the platform route that drives one agent with one "
                         "sentence (env OMNILINK_SWEEP_PLATFORM_PATH). Without "
                         "it the platform door is SKIPPED, never passed.")
    ap.add_argument("--platform-key", default="",
                    help="OmniKey for the platform door (default $OMNI_KEY)")
    ap.add_argument("--no-launch", action="store_true",
                    help="do not spawn engines: every selected demo's bridge "
                         "is already listening (a world you have open, or a "
                         "fake bridge under test)")
    ap.add_argument("--port-override", type=int, default=0,
                    help="talk to this port instead of the one the world "
                         "declares (used with --no-launch)")
    ap.add_argument("--pause", type=float, default=1.0,
                    help="seconds between sentences")
    args = ap.parse_args(argv)
    keep = [s for s in args.only.split(",") if s]
    doors = list(DOORS) if args.door == "both" else [args.door]

    demos, unscripted = select_demos(keep)
    if not demos:
        # Report the exclusions BEFORE bailing out, or `--only mavic` prints
        # a bare "no matching demos" and looks like a typo rather than a
        # documented design decision.
        for stem, ctrl in unscripted:
            why = EXPECTED_UNSCRIPTED.get(ctrl)
            print(f"  -- {stem} not scripted, by design: {why}" if why else
                  f"  !! {stem} ({ctrl}) has NO SCRIPT. That is a HOLE.")
        print("no matching demos with a script")
        return 1

    # Preflight every door BEFORE launching a single engine: discovering that
    # the platform is not deployed after twenty-one engine launches wastes an
    # hour and heats the laptop for nothing.
    impls, specs = {}, {}
    for name in doors:
        impl = make_door(name, args)
        ok, why = impl.preflight()
        if not ok:
            impl.disabled_reason = why
            print(f"  !! door {name!r} cannot run: {why}", flush=True)
        impls[name] = impl
        specs[name] = dict(impl.describe(), preflight_ok=ok, reason=why)
    if all(getattr(i, "disabled_reason", None) for i in impls.values()):
        print("\n=== every requested door is unavailable: NOTHING WAS TESTED ===")
        # Still write the artefact, so the skip is on the record rather than
        # being an absence someone later reads as "we never ran it".
        by_door = {n: [blank_row(w.stem, n, port_of(w), robot_of(w),
                                 SCRIPTS[c]) for w, c in demos]
                   for n in doors}
        for name in doors:
            for r in by_door[name]:
                r["skip_reason"] = impls[name].disabled_reason
        art = build_artefact(doors=doors, by_door=by_door,
                             demos_order=[w.stem for w, _c in demos],
                             unscripted=unscripted, door_specs=specs)
        if args.out:
            pathlib.Path(args.out).write_text(
                json.dumps(art, indent=2), encoding="utf-8")
            print(f"  wrote {args.out}")
        return 1

    by_door = {n: [] for n in doors}
    demos_order = [w.stem for w, _c in demos]
    for world, ctrl in demos:
        script = SCRIPTS[ctrl]
        port = args.port_override or port_of(world)
        robot = robot_of(world)
        proc = None
        if not args.no_launch:
            proc = subprocess.Popen(
                [sys.executable, "-m", "omnisim", "run-headless", str(world),
                 "--duration", str(args.duration)],
                cwd=str(ROOT), stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env={**os.environ, "OMNISIM_URDF_USE_SENSORS": "1"})
        try:
            up = wait_up(port=port)
            # BOTH doors against the SAME engine, in order: one engine at a
            # time is the thermal rule, and it also means the two doors see
            # the same robot in the same world rather than two launches that
            # could differ.
            for name in doors:
                row = run_door(impls[name], demo=world.stem, port=port,
                               robot=robot, script=script, up=up,
                               pause_s=args.pause)
                by_door[name].append(row)
                mark = ("OK " if row["clean"] else
                        "SKIP" if not row["ran"] else "!! ")
                vias = ",".join(s["via"] or "-" for s in row["sentences"])
                # ⚠️ THE REASON GOES ON THE CONSOLE LINE, not only into the
                # JSON. Whoever is watching a two-hour sweep scroll past is
                # the person who needs to know it is an account key and not a
                # code regression, and they are not reading the artefact yet.
                detail = row["skip_reason"] or "; ".join(row["errors"])
                if not detail and row["bridge_errors"]:
                    detail = (f"answered, but stated "
                              f"{len(row['bridge_errors'])} refusal/error(s)")
                print(f"{mark} {row['demo']:<26} door={name:<8} "
                      f"answered={row['answered']}/{row['asked']} "
                      f"via=[{vias}] trap_moved={row['trap_actuated']} "
                      f"{detail[:160]}", flush=True)
        finally:
            if proc is not None:
                _kill_tree(proc)
                time.sleep(2)

    if unscripted:
        print()
        for stem, ctrl in unscripted:
            why = EXPECTED_UNSCRIPTED.get(ctrl)
            if why:
                print(f"  -- {stem} not scripted, by design: {why}")
            else:
                print(f"  !! {stem} ({ctrl}) has NO SCRIPT and was not "
                      f"tested. That is a HOLE in this result.")

    art = build_artefact(doors=doors, by_door=by_door,
                         demos_order=demos_order, unscripted=unscripted,
                         door_specs=specs)
    print()
    for name in doors:
        s = art["summary"][name]
        print(f"=== door {name}: {s['clean']}/{s['requested']} demos clean, "
              f"ran on {s['ran']}/{s['requested']}, "
              f"answered {s['answered']}/{s['asked']}, "
              f"traps actuated {s['trap_actuations']}, "
              f"via unreported {s['via_unreported']} ===")
        for why, n in sorted(s["skip_reasons"].items()):
            print(f"    SKIPPED x{n}: {why}")
        if s["failed_with_reason"] or s["failed_without_reason"]:
            print(f"    unanswered: {s['failed_with_reason']} with a stated "
                  f"reason, {s['failed_without_reason']} WITH NONE")
        for why, n in sorted(s["reasons_seen"].items(), key=lambda kv: -kv[1]):
            print(f"    x{n} the bridge said: {why}")
    if len(doors) > 1:
        c = art["comparison"]
        print(f"=== via across doors: {c['agree']} agree, "
              f"{c['disagree']} disagree, {c['undetermined']} undetermined ===")
    v = art["verdict"]
    print(f"=== VERDICT: {'PASS' if v['pass'] else 'FAIL'} ===")
    for why in v["reasons"]:
        print(f"    {why}")
    if args.out:
        pathlib.Path(args.out).write_text(json.dumps(art, indent=2),
                                          encoding="utf-8")
        print(f"  wrote {args.out}")
    return 0 if v["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
