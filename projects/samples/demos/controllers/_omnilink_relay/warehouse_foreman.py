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

"""warehouse_foreman — the site-level agent for the OmniLink warehouse demo.

A plain script (NOT a Webots controller). It pushes an OmniLink agent
profile named "Warehouse-Foreman" that supervises the three robot agents
the flagship warehouse world registers when its bridges boot with
OMNI_KEY set:

    OmniSim-omniarm6  the OmniArm 6 pick-station arm  (omnilink_arm_bridge, :8765)
    OmniSim-tug_a     dispatch tug (OmniTug 500)      (omnilink_mobile_bridge, :8766)
    OmniSim-tug_b     return tug (OmniTug 500)        (omnilink_mobile_bridge, :8767)

World: projects/samples/demos/worlds/flagship/warehouse_omnilink.omniworld

Why this script runs the crew loop itself
-----------------------------------------
The platform ships a built-in ``delegate_to_agent`` tool, and the obvious
design is to let the Foreman use it: no local tools, no callback surface,
delegation entirely platform-side. **That design cannot ground itself, and
this script used to be built on it.** Three properties of the platform
make it a dead end for robot supervision — all three verified against a
live warehouse world:

1. *A delegated agent's tools never run.* ``executeDelegation``
   (``api/_agent-delegation.ts``) starts a child chat and keeps only its
   ``text``. Every robot profile answers a state question by emitting a
   ``get_robot_state`` tool call with empty text, and its
   ``toolCallbackUrl`` is ``127.0.0.1`` — which the server documents it
   cannot reach (``api/_unattended-tool-loop.ts``). So the child's tool
   call is returned and dropped, and the delegation result is the empty
   string.
2. *Only the first delegation of a turn executes.* ``findDelegationCall``
   is a ``.find()``, so a Foreman that correctly asks all three robots in
   one turn gets exactly one of them run and the rest silently discarded.
3. *There is no synthesis turn.* The delegation result is string-appended
   to the parent's text; the parent model never sees it and never gets to
   reason over it.

Net effect on the old design: the operator got an empty reply. So the
Foreman keeps a real tool surface and runs the crew loop **here**, on the
operator's machine, which is the one place that can actually reach the
robots. ``ask_robot`` reads a robot's authoritative state over its
bridge's read-only ``/state`` route; ``command_robot`` hands an
instruction to that robot's *own* agent via the bridge's ``/prompt``
route, so the machine agent still interprets and executes it with its own
tools. Every fact in a Foreman answer therefore traces to a live robot,
and every command lands on a real actuator.

Usage
-----
    set OMNI_KEY=olink_...

    # Push / refresh the Warehouse-Foreman profile (idempotent):
    python warehouse_foreman.py            # or: warehouse_foreman.py push

    # One chat turn with the full delegation trace printed:
    python warehouse_foreman.py ask "why is the line slow?"

    # Just show what the crew reports, no LLM turn (free, instant):
    python warehouse_foreman.py crew

Environment
-----------
    OMNI_KEY               required
    OMNILINK_ENGINE        engine id (default "g1-engine")
    OMNILINK_FOREMAN_PORTS "omniarm6=8765,tug_a=8766,tug_b=8767" — override
                           when the world runs on non-default bridge ports
    OMNILINK_AGENT_TAG     set for ANY run that is not the shipped demo. It
                           suffixes this agent's name AND the crew names it
                           supervises, so a test gets its own profiles and
                           memory instead of taking over the real ones. Must
                           match the tag the bridges were launched with.
"""

from __future__ import annotations

import json
import os
import re
import sys
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# The installable package is canonical for both simulator controllers and this
# standalone site-level agent.
_REPO_ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(_REPO_ROOT / "packages" / "omnisim-bridges" / "src"))

from omnisim_bridges.relay import (  # noqa: E402
    BASE_URL,
    REQUEST_TIMEOUT,
    check_omnilink_installation,
)
from omnisim_bridges.profile_sync import ensure_profile  # noqa: E402

def _tagged(name: str) -> str:
    """Apply OMNILINK_AGENT_TAG the same way the bridges do.

    The bridges honour the tag (profile_sync.agent_name_for) so a test run
    gets its own profile and its own memory. The Foreman hard-coded both its
    own name and the crew's, so a tagged run could not isolate it at all: it
    would push over the shipped Warehouse-Foreman profile and then supervise
    the SHIPPED robots rather than the tagged ones under test. Flagged by an
    audit that could verify every robot and not the thing that supervises
    them.

    Kept in one helper rather than importing agent_name_for, because the
    Foreman's own name has no "OmniSim-" prefix -- the tag has to attach to
    whatever string it is given.
    """
    tag = (os.environ.get("OMNILINK_AGENT_TAG") or "").strip()
    if not tag:
        return name
    safe = re.sub(r"[^A-Za-z0-9_.-]", "-", tag)[:32].strip("-")
    return f"{name}-{safe}" if safe else name


AGENT_NAME = _tagged("Warehouse-Foreman")

# The bridges push their own profiles with whatever OMNILINK_ENGINE the world
# was launched with; the Foreman follows the same env so the whole site runs
# one engine. NOTE: this used to be hardcoded "g5-engine", which is not a
# valid engine id — every turn silently fell back to g1-engine.
ENGINE = os.environ.get("OMNILINK_ENGINE", "").strip() or "g1-engine"

# robot key -> (agent profile name, default bridge port, one-line role)
CREW: Dict[str, Tuple[str, int, str]] = {
    "omniarm6": (_tagged("OmniSim-omniarm6"), 8765,
              "LINE MASTER — the OmniArm 6 pick-station arm. Fills boxes and owns "
              "the authoritative line record."),
    "tug_a": (_tagged("OmniSim-tug_a"), 8766,
              "DISPATCH — tows loaded trolleys from the fill station out to "
              "the park row."),
    "tug_b": (_tagged("OmniSim-tug_b"), 8767,
              "RETURN — brings emptied trolleys back via the floor conveyor "
              "and re-spots them at the fill station."),
}

SUPERVISED_AGENTS = tuple(name for name, _, _ in CREW.values())

#: How many chat rounds the local crew loop will run before giving up.
#: One round to call tools, one to synthesize, one spare.
MAX_TOOL_ROUNDS = 3

#: Per-request timeout when talking to a robot bridge. A /prompt round-trip
#: runs the machine agent's own LLM turn, so it needs real headroom.
BRIDGE_READ_TIMEOUT_S = 10.0
BRIDGE_PROMPT_TIMEOUT_S = 60.0


def _ports() -> Dict[str, int]:
    """Resolve bridge ports, honouring OMNILINK_FOREMAN_PORTS."""
    ports = {key: port for key, (_, port, _) in CREW.items()}
    raw = os.environ.get("OMNILINK_FOREMAN_PORTS", "").strip()
    for chunk in (c for c in raw.split(",") if c.strip()):
        key, _, value = chunk.partition("=")
        key = key.strip()
        if key in ports and value.strip().isdigit():
            ports[key] = int(value.strip())
    return ports


MAIN_TASK = """You are the Warehouse Foreman, the site-level supervisor for an
OmniSim warehouse worked by three robots. You are the only agent the operator
talks to, and you answer for the whole line.

YOUR CREW (address them by these exact robot keys):

- "omniarm6" — LINE MASTER. The OmniArm 6 6-DOF arm at the fill station, fitted with
  a suction pad. It picks parts off the feeder tray and drops them into the
  box stopped in front of it. It also holds the authoritative record of what
  the line is doing: its "line" block reports the box being filled
  (fill_box), that box's stage (fill_state), how many parts are in it
  (placed) out of the target, how many boxes are queued behind it (queued),
  and what is in transit.
- "tug_a" — DISPATCH. An OmniTug 500 tug. It waits for a loaded trolley at
  the fill station, tows it to the park row, drops it, and returns empty.
- "tug_b" — RETURN. The second OmniTug 500 tug. It collects emptied trolleys and
  sends them back up the floor conveyor, then re-spots an empty trolley on
  the fill station the moment it goes vacant.

THE LOOP YOU SUPERVISE:

  CONVEYOR -> FILL (omniarm6) -> DISPATCH (tug_a) -> RETURN (tug_b) -> repeat

Boxes ride the inbound belt and queue; the frontmost stops at the fill line.
omniarm6 fills it, the filled box transfers onto the empty trolley at the fill
station, tug_a tows that trolley to the park row, and tug_b returns an empty
trolley so filling can resume. The stages are coupled: the arm CANNOT start a
new box while the fill station still holds a loaded trolley, so an idle arm is
usually a downstream symptom, not an arm fault.

HOW YOU WORK — you have no motors of your own, only your crew:

- ask_robot(robot, question): read a robot's live state. FREE and instant.
- command_robot(robot, command): hand an instruction to that robot's own
  agent, which interprets and executes it.

RULES — these are not optional:

1. NEVER state a fact about a robot you have not read this turn. If you did
   not call ask_robot for it, you do not know it. No guessing, no plausible
   numbers, no remembered values. In particular, only say the arm is holding
   a part if omniarm6's own gripper.holding field came back true — that field
   is the ONLY evidence for it, and it is usually false because the arm
   spends most of its cycle between picks.
2. For ANY question about the line, the site, throughput, or "what's going
   on" — call ask_robot for ALL THREE robots in your first turn, in
   parallel. A line diagnosis needs all three stages.
3. A HEALTHY LINE IS THE COMMON CASE. Every robot moving, cycles climbing,
   nobody paused, boxes queued — that is the line working, and the honest
   answer is "it is running normally", with the numbers. Do NOT manufacture
   a bottleneck to sound useful. Only call something blocked or starved when
   a field you actually read says so.
4. When something IS wrong, diagnose as a pipeline: the symptom shows up one
   stage AWAY from the cause, so name the STAGE that is starved (waiting on
   upstream) or BLOCKED (waiting on downstream), not just the robot that
   looks idle. The patterns below are DIAGNOSTIC HYPOTHESES TO TEST against
   what you read — they are not observations, and none of them is true until
   a field you read confirms it:
   - arm idle with no box at the fill station -> conveyor dry (check queued),
     or the previous load has not been towed away yet -> look at tug_a.
   - arm idle AND gripper.holding true -> no empty trolley spotted, which is
     tug_b's leg. Requires gripper.holding to actually be true.
   - box filled but not departing -> tug_a has not collected it.
   - a tug whose leg reads "holding" -> it is waiting for the peer to clear
     the shared lane. Normal contention; it costs cycle time. Report it as
     queueing, not as a fault.
   - paused=true on any robot -> someone chatted to it directly; it resumes
     on its own after about a minute of quiet. Not a fault.
5. A COMMAND from the operator ("stop tug_a", "home the arm", "carry on")
   goes to exactly one robot via command_robot. Then report what that robot
   actually reported back.
6. Attribute every fact to the robot that reported it, with the number.
   "tug_a is idle at the park row with nothing in tow" beats "the tugs look
   fine". If a robot did not answer, say so plainly and answer around it.
7. HOW MUCH WORK A ROBOT HAS DONE comes from exactly two fields, and they
   are the only ones that mean it: `delivered_total` (carts THIS tug parked
   this session) and `jobs_total` (all its completed jobs). Anything named
   `*_NOT_MY_WORK` is a fact about the SITE — how many carts are sitting in
   the park row right now, including ones that were there before the shift —
   and it drops when a cart is collected out. Never convert a site count into
   a robot's achievement. If `delivered_total` is 0 the tug has parked
   nothing, however full the row looks. For the arm, `boxes_filled_total` and
   `shipped_total` are the equivalents.
8. STANDING ORDERS ARE PART OF THE PICTURE, and an operator asking "what's
   going on" wants them. Every robot's state carries `pending_intents`
   (deferred things it is waiting to do, with the trigger and the operator's
   own words), `constraints` (standing restrictions it is enforcing),
   `autonomy_hold` (whether it is stopped and waiting to be told to carry on)
   and `notifications` (messages it is holding FOR THE OPERATOR). Read them.
   A robot that is HELD is not idle, it is waiting on a human, and that is the
   single most important thing to lead with — say who it is waiting on and
   what it is holding. A pending intent that has already fired is finished;
   do not report it as still outstanding.
9. If a robot reports `restored_from_previous_run`, its commitments came back
   from before a reload. Say so if it is relevant — an operator should not
   discover by accident that a promise survived a restart.

Lead with the answer in one sentence, then the evidence, then anything that
needs a human decision. Talk like a shift foreman, not a status page."""


TOOL_DEFS: List[Dict[str, Any]] = [
    {
        "name": "ask_robot",
        "description": (
            "Ask one robot in the crew for its live state. Returns that "
            "robot's authoritative status record. Call this for every robot "
            "that could hold part of the answer before you reply. Read-only "
            "and instant — it never disturbs the robot's work."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "robot": {
                    "type": "string",
                    "enum": list(CREW.keys()),
                    "description": "Which robot to ask: omniarm6, tug_a or tug_b.",
                },
                "question": {
                    "type": "string",
                    "description": (
                        "What you want to know. Recorded in the trace so the "
                        "operator can see why you asked."
                    ),
                },
            },
            "required": ["robot"],
        },
    },
    {
        "name": "command_robot",
        "description": (
            "Give one robot an instruction in plain language. It is handed to "
            "that robot's own agent, which interprets it and drives its own "
            "motors. Use for any request that should change what a robot is "
            "doing (stop, resume, home, drive somewhere). Not for questions."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "robot": {
                    "type": "string",
                    "enum": list(CREW.keys()),
                    "description": "Which robot to command.",
                },
                "command": {
                    "type": "string",
                    "description": "The instruction, e.g. 'stop' or 'carry on'.",
                },
            },
            "required": ["robot", "command"],
        },
    },
]


# ── bridge I/O ────────────────────────────────────────────────────────

def _bridge_post(port: int, path: str, body: Dict[str, Any],
                 timeout: float) -> Dict[str, Any]:
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}",
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _resolve(robot: Any) -> Tuple[Optional[str], Optional[str]]:
    """Map a model-supplied robot name onto a crew key. Returns (key, error)."""
    key = str(robot or "").strip().lower()
    # Tolerate the model addressing a robot by its full profile name.
    if key.startswith("omnisim-"):
        key = key[len("omnisim-"):]
    if key in CREW:
        return key, None
    return None, (f"unknown robot {robot!r}; the crew is "
                  f"{', '.join(CREW)}")


def ask_robot(robot: Any, question: Any = "") -> Dict[str, Any]:
    """Read one robot's authoritative live state.

    Deliberately uses the bridge's read-only ``/state`` route rather than
    ``/tool {"tool": "get_robot_state"}``. Both return the same record, but
    the bridges' idle-loop pause exemption is keyed on the HTTP *path*, so a
    read issued through ``/tool`` is scored as an operator command and
    silently pauses the production line. See the bug note in the report /
    ``omnilink_arm_bridge.py`` route table.
    """
    key, err = _resolve(robot)
    if err:
        return {"error": err}
    _, _, role = CREW[key]
    port = _ports()[key]
    try:
        state = _bridge_post(port, "/state", {}, BRIDGE_READ_TIMEOUT_S)
    except urllib.error.URLError as e:
        return {"robot": key, "reachable": False,
                "error": f"{key} did not answer on :{port} ({e.reason}). "
                         "Its bridge may not be running."}
    except Exception as e:
        return {"robot": key, "reachable": False,
                "error": f"{key} did not answer on :{port} ({type(e).__name__}: {e})"}
    return {"robot": key, "role": role, "reachable": True,
            "asked": str(question or "state"),
            "state": _disambiguate(state)}


def _disambiguate(state: Any) -> Any:
    """Rename the two fields that make a Foreman claim work it did not see.

    `idle_loop.delivered` and `idle_loop.parked` are the park row's CURRENT
    OCCUPANCY -- they count carts that were already sitting there at world
    init, and they go DOWN when a cart is collected out. Read as a tally of
    a robot's labour they are simply wrong, and the names invite exactly that
    reading. The per-robot agents were fixed by correcting their tool
    description; the Foreman reads raw /state and reproduced the same
    fabrication one level up ("tug_a has parked 4 trolleys this shift" against
    a measured delivered_total of 2).

    So the Foreman never sees the ambiguous names. The values are preserved
    under names that cannot be misread, and the honest counters are left
    exactly as they are.
    """
    if not isinstance(state, dict):
        return state
    loop = state.get("idle_loop")
    if not isinstance(loop, dict):
        return state
    out = dict(state)
    loop = dict(loop)
    if "delivered" in loop:
        loop["park_row_count_NOT_MY_WORK"] = loop.pop("delivered")
    if "parked" in loop:
        loop["park_row_occupants_NOT_MY_WORK"] = loop.pop("parked")
    out["idle_loop"] = loop
    return out


def command_robot(robot: Any, command: Any = "") -> Dict[str, Any]:
    """Hand an instruction to a robot's own agent for it to execute."""
    key, err = _resolve(robot)
    if err:
        return {"error": err}
    text = str(command or "").strip()
    if not text:
        return {"error": "command must be a non-empty instruction"}
    port = _ports()[key]
    try:
        result = _bridge_post(port, "/prompt", {"text": text},
                              BRIDGE_PROMPT_TIMEOUT_S)
    except Exception as e:
        return {"robot": key, "delivered": False,
                "error": f"{key} did not accept the command on :{port} "
                         f"({type(e).__name__}: {e})"}
    # Read back the resulting state so the Foreman reports what actually
    # happened rather than what it asked for.
    after: Any
    try:
        after = _bridge_post(port, "/state", {}, BRIDGE_READ_TIMEOUT_S)
    except Exception as e:
        after = {"error": f"state read-back failed ({type(e).__name__}: {e})"}
    return {"robot": key, "delivered": True, "command": text,
            "robot_reply": result, "state_after": after}


HANDLERS = {"ask_robot": ask_robot, "command_robot": command_robot}


def _summarize(name: str, args: Dict[str, Any], result: Dict[str, Any]) -> str:
    """One-line human summary of a crew call, for the delegation trace."""
    who = args.get("robot", "?")
    if result.get("error"):
        return f"{who}: ERROR {result['error']}"
    if name == "ask_robot":
        st = result.get("state") or {}
        bits = [f"mode={st.get('mode')}"]
        line = st.get("line")
        if isinstance(line, dict):
            bits.append(
                f"fill_box={line.get('fill_box')} stage={line.get('fill_state')} "
                f"placed={line.get('placed')}/{line.get('target')} "
                f"queued={line.get('queued')}"
            )
        idle = st.get("idle_loop")
        if isinstance(idle, dict):
            bits.append(f"leg={idle.get('leg', idle.get('mode'))} "
                        f"cycles={idle.get('cycles', idle.get('picks'))} "
                        f"paused={idle.get('paused')}")
        # Surfaced because these are the two fields the Foreman is most prone
        # to assert without evidence ("holding a part", "moving at N m/s").
        # Printing them makes every claim auditable straight from the trace.
        grip = st.get("gripper")
        if isinstance(grip, dict):
            bits.append(f"holding={grip.get('holding')}")
        if st.get("v_linear") is not None:
            bits.append(f"v={st.get('v_linear'):.2f}")
        if st.get("carrying") is not None:
            bits.append(f"carrying={st.get('carrying')}")
        return f"{who}: " + " ".join(str(b) for b in bits)
    reply = result.get("robot_reply")
    text = ""
    if isinstance(reply, dict):
        text = str(reply.get("text") or reply.get("status") or "")[:120]
    return f"{who}: commanded {args.get('command')!r} -> {text or 'accepted'}"


# ── platform plumbing ────────────────────────────────────────────────

def _client():
    """OmniLinkClient wired exactly like the in-sim bridges (same base
    URL, timeout, and version floor)."""
    check_omnilink_installation()
    from omnilink.client import OmniLinkClient  # noqa: PLC0415

    omni_key = os.environ.get("OMNI_KEY", "").strip()
    if not omni_key:
        sys.exit("OMNI_KEY is not set. Export your OmniLink key first "
                 "(set OMNI_KEY=olink_...).")
    return OmniLinkClient(omni_key=omni_key, base_url=BASE_URL,
                          timeout=REQUEST_TIMEOUT)


def push_profile(client) -> str:
    """Idempotent Warehouse-Foreman profile push. Returns the profile id."""
    pid = ensure_profile(
        client,
        AGENT_NAME,
        main_task=MAIN_TASK,
        tool_defs=TOOL_DEFS,
        engine=ENGINE,
    )
    if pid is None:
        sys.exit("[warehouse_foreman] profile push failed (see message above)")
    print(f"[warehouse_foreman] profile '{AGENT_NAME}' ready (id={pid}, "
          f"engine={ENGINE}); supervises: {', '.join(SUPERVISED_AGENTS)}")
    return pid


def crew_report() -> None:
    """Print what every robot reports right now. No LLM turn, no cost."""
    ports = _ports()
    print(f"[warehouse_foreman] crew status ({AGENT_NAME}):")
    for key in CREW:
        result = ask_robot(key)
        print(f"  :{ports[key]:<5} {_summarize('ask_robot', {'robot': key}, result)}")


def ask(client, question: str) -> Dict[str, Any]:
    """One operator question, answered by consulting the crew.

    Runs the tool loop locally because that is the only place the robots are
    reachable (see the module docstring). Every crew call is printed as it
    happens, so the delegation is visible rather than implied.
    """
    messages: List[Dict[str, Any]] = [{"role": "user", "content": question}]
    tool_names = ", ".join(t["name"] for t in TOOL_DEFS)
    t_start = time.time()
    text = ""
    trace: List[Dict[str, Any]] = []
    rounds = 0

    for round_no in range(MAX_TOOL_ROUNDS):
        rounds = round_no + 1
        data = client.chat(
            messages=messages,
            agent_name=AGENT_NAME,
            engine=ENGINE,
            system_instruction={
                "mainTask": MAIN_TASK,
                "availableTools": tool_names,
                "availableToolDetails": TOOL_DEFS,
                "allowToolUse": True,
            },
            usePromptPipeline=True,
            skipMemory=True,
            requestId=str(uuid.uuid4()),
        )
        if not data.get("ok", True) and data.get("error"):
            sys.exit(f"[warehouse_foreman] chat failed: {data['error']}")

        text = data.get("text") or ""
        tool_calls = data.get("toolCalls") or []
        if not tool_calls:
            break

        assistant_msg: Dict[str, Any] = {"role": "assistant", "content": text}
        assistant_msg["tool_calls"] = tool_calls
        messages.append(assistant_msg)

        print(f"-- crew round {round_no + 1}: consulting "
              f"{len(tool_calls)} robot(s) --")
        for call in tool_calls:
            name = call.get("name", "")
            args = call.get("arguments") or {}
            handler = HANDLERS.get(name)
            t0 = time.time()
            if handler is None:
                result: Dict[str, Any] = {
                    "error": f"unknown tool {name!r}; available: {tool_names}"
                }
            else:
                try:
                    result = handler(**args)
                except TypeError as e:
                    result = {"error": f"bad arguments for {name}: {e}"}
                except Exception as e:
                    result = {"error": f"{name} failed: {type(e).__name__}: {e}"}
            dt = (time.time() - t0) * 1000.0
            print(f"   [{name}] {_summarize(name, args, result)}  ({dt:.0f}ms)")
            trace.append({
                "round": round_no + 1,
                "name": name,
                "arguments": dict(args),
                "result": result,
                "elapsed_ms": round(dt, 1),
            })
            messages.append({
                "role": "tool",
                "tool_call_id": call.get("id") or name,
                "name": name,
                "content": json.dumps(result, default=str),
            })
    else:
        print(f"[warehouse_foreman] hit the {MAX_TOOL_ROUNDS}-round crew limit")

    print(f"\n-- foreman ({time.time() - t_start:.1f}s, engine={ENGINE}) --")
    print(text or "(no text response)")
    return {
        "text": text,
        "trace": trace,
        "rounds": rounds,
        "elapsed_s": round(time.time() - t_start, 3),
        "engine": ENGINE,
        "agent_name": AGENT_NAME,
    }


def main(argv) -> None:
    cmd = argv[1] if len(argv) > 1 else "push"
    if cmd == "push":
        push_profile(_client())
    elif cmd == "crew":
        crew_report()
    elif cmd == "ask":
        if len(argv) < 3 or not argv[2].strip():
            sys.exit('usage: python warehouse_foreman.py ask "your question"')
        client = _client()
        push_profile(client)  # keep the profile fresh before chatting
        ask(client, argv[2])
    else:
        sys.exit('usage: python warehouse_foreman.py [push | crew | ask "question"]')


if __name__ == "__main__":
    main(sys.argv)
