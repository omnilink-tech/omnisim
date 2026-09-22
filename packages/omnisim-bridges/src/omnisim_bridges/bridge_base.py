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

"""BridgeBase -- abstract Webots-less bridge for real robots.

This is the sim-to-real seam: the OmniLink agent code (relay, parser,
tool definitions) is byte-identical between OmniSim and a real robot
integration. The only thing that changes is what `act_*` does:
in OmniSim it calls `motor.setPosition(...)`; on a real robot it calls
into your fleet's SDK.

The stub here provides:

  - `BridgeBase` -- abstract class with the methods every robot
    must implement (`act_stop`, `act_set_velocity`, `act_drive_forward`,
    `act_turn`, `act_set_joint_positions`, `act_set_tcp_target`,
    `act_open_gripper`, `act_close_gripper`, `act_reset_to_home`,
    `get_state`). Subclasses pick the subset that applies to their
    robot kind (arm / mobile / both).

  - `serve_http(bridge, port)` -- exactly the HTTP surface the OmniSim
    arm + mobile bridges expose (`/list_robots`, `/get_robot_state`,
    `/prompt`, `/tool`, `/usage`, ...), so an OmniLink-Foreman or
    OmniLink-Picker driving the simulated robot today drives the real
    robot tomorrow by changing one URL.

To build your own real-robot bridge, copy `arm_bridge_stub.py` (next
to this file), replace `MockArmDriver` with your robot SDK's client,
run it. That's the whole sim-to-real story.

This file has zero Webots imports and zero OmniSim-internal imports.
It depends only on the Python stdlib + optionally the omnilink package
for the OmniLink relay path (which the bridge can run in or skip).
"""

from __future__ import annotations

import abc
import inspect
import json
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, List, Optional
from urllib.parse import parse_qs, urlparse

from .events import EventRing, emit_gate_refusal
from .http_security import (
    RequestError,
    RequestIdGuard,
    allowed_origins,
    check_authorization,
    check_protocol_version,
    checked_origin,
    configured_token,
    error_envelope,
    finite_number,
    number_list,
    read_json,
    require_field,
    validate_bind,
    validate_request_id,
    WIRE_SERVICE,
    WIRE_VERSION,
)


# ── D1: the two clocks, and waits counted in sim steps ──────────────

class SimClock:
    """The bridge's SIM clock and step counter, cached once per tick.

    ⚠️ THE CACHE IS THE POINT, NOT AN OPTIMISATION. The controller API is
    not thread-safe: a `robot.getTime()` issued from an HTTP or relay thread
    stalls the controller<->sim step exchange, and a couple of threaded
    reads per second dragged the warehouse demo to ~0.2x realtime. So the
    SIM THREAD writes these two numbers once per `robot.step()` and every
    other thread reads them. Nothing else may ask the engine what time it
    is.

    Two clocks, different rulers, and mixing them is how waits used to be
    wrong: `sim_time` is the physics clock (what §5.3 means by `sim_time`
    and `last_tick_at`), `wall_time` is `time.time()` and belongs only in
    `wall_time` and in liveness checks. A wall-clock timeout silently
    truncated motions whenever the sim ran below realtime, which on a
    loaded world is most of the time.
    """

    def __init__(self, dt_s: float = 0.032) -> None:
        dt = float(dt_s)
        self.dt = dt if dt > 1e-6 else 0.032
        self.sim_time = 0.0
        self.step = 0
        self.wall_time = time.time()
        self.started = False

    def tick(self, sim_time: Optional[float] = None) -> None:
        """Call ONCE per `robot.step()`, from the sim thread only."""
        if sim_time is None:
            self.sim_time += self.dt
        else:
            try:
                self.sim_time = float(sim_time)
            except (TypeError, ValueError):
                self.sim_time += self.dt
        self.step += 1
        self.wall_time = time.time()
        self.started = True

    def steps_for(self, seconds: float) -> int:
        """How many basic steps `seconds` of SIM time is worth."""
        try:
            n = int(round(float(seconds) / self.dt))
        except (TypeError, ValueError, ZeroDivisionError):
            return 1
        return max(1, n)

    def budget(self, timeout_s: float) -> "StepBudget":
        return StepBudget(self, timeout_s)

    def snapshot(self) -> Dict[str, Any]:
        return {"sim_time": self.sim_time, "step": self.step,
                "wall_time": self.wall_time, "dt": self.dt}


class StepBudget:
    """A wait measured in SIM STEPS. The verdict is never a wall clock.

    Taken at the start of a wait; `expired()` is true once the sim has
    advanced the budgeted number of steps. The polling `time.sleep` in the
    waiter stays -- it yields the HTTP thread and is not a measurement --
    but `timed_out` and `settled` come from here.

    `stalled()` is a SEPARATE question and deliberately wall-clock: "has the
    sim thread stopped?". A stall is not a timeout, it is a fault
    (`fault.telemetry_stale`), and conflating them is what made a paused
    world report every motion as having timed out.
    """

    def __init__(self, clock: SimClock, timeout_s: float) -> None:
        self.clock = clock
        self.timeout_s = float(timeout_s)
        self.steps = clock.steps_for(timeout_s)
        self.start_step = clock.step
        self.start_sim = clock.sim_time
        self.wall_start = time.time()

    @property
    def elapsed_steps(self) -> int:
        return max(0, self.clock.step - self.start_step)

    @property
    def elapsed_sim_s(self) -> float:
        return max(0.0, self.clock.sim_time - self.start_sim)

    def remaining_steps(self) -> int:
        return max(0, self.steps - self.elapsed_steps)

    def expired(self) -> bool:
        return self.elapsed_steps >= self.steps

    def stalled(self, last_tick_wall: float, grace_s: float = 2.0) -> bool:
        """The world is not stepping -- not the same thing as expired.

        ⚠️ MEASURED FROM WHEN THE WAIT BEGAN, not from the last tick alone. A
        lockstep hold (D6) freezes `last_tick_at` on purpose between
        commands, so a wait that starts inside a hold would otherwise be
        called stalled on its first poll, before the loop has even seen the
        work. MEASURED 2026-09-22: `/drive_forward {"wait": true}` issued 6 s
        into a hold answered `stalled: true, steps: 0` in 0.0 s while the
        robot then drove 0.9855 m. A world that really stops still reports
        a stall, `grace_s` after the wait began.
        """
        try:
            ref = max(float(last_tick_wall), self.wall_start)
            return (time.time() - ref) > float(grace_s)
        except (TypeError, ValueError):
            return False

    def result(self) -> Dict[str, Any]:
        """The three fields every measured result gains (plan D1 step 2)."""
        return {
            "sim_time_start": round(self.start_sim, 6),
            "sim_time_end": round(self.clock.sim_time, 6),
            "steps": self.elapsed_steps,
        }


# ── D2 (bridge half): the `safety_gate` block of PROTOCOL §5.2.1 ─────

# The rule names this gate can emit, from the §5.2.1 enumeration. `gate.py`
# is the source of truth; `test_gate_block_rules_exist_in_gate` pins every
# name below to a `Rejection(..., "<name>", ...)` site in it, so the two
# cannot drift silently. `unknown_tool` is deliberately absent: the gate
# emits it internally and every call site drops it, so a client will never
# see one.
GATE_RULES = (
    "interrogative", "prohibition", "self_negating", "reported_speech",
    "retracted", "deferred", "contradiction", "invented_magnitude",
    "sign_conflict", "implausible", "unknown_arg", "missing_arg",
    "bad_type", "not_finite", "unresolved_referent", "needs_authorization",
)


def safety_gate_block(surface: Optional[str], *,
                      gated_paths: Optional[List[str]] = None,
                      ungated_paths: Optional[List[str]] = None,
                      present: bool = True) -> Dict[str, Any]:
    """Build `/capabilities.safety_gate` exactly as PROTOCOL §5.2.1 spells it.

    ⚠️ `ungated_paths` IS THE LOAD-BEARING HALF, and it is the caller's job
    to get it right for its own route table. Publishing `present: true` with
    no `ungated_paths` invites the reading "everything here is vetted",
    which is false of every bridge in this tree: the direct REST verbs
    (`/drive_forward`, `/turn`, `/set_velocity`, `/action`) bypass the gate
    entirely and always have. A client that cannot discover that cannot
    decide whether it is itself responsible for bounding a command.

    The rails are `gate.py`'s constants, read at call time so a change there
    cannot leave a stale number published here. They are RAILS, not bounds:
    "nobody meant this", never "the robot can do this".
    """
    try:
        from . import gate as _g
        rails = {
            "max_distance_m": _g.MAX_DISTANCE_M,
            "max_angle_rad": _g.MAX_ANGLE_RAD,
            "max_speed_mps": _g.MAX_SPEED_MPS,
            "max_yaw_rate_rps": _g.MAX_YAW_RATE_RPS,
            "max_altitude_m": _g.MAX_ALTITUDE_M,
            "max_body_shift_m": _g.MAX_BODY_SHIFT_M,
        }
    except Exception:                                    # pragma: no cover
        # The gate could not be imported -- which is exactly when
        # `vet_toolcall` refuses every physical tool. Say so rather than
        # publishing rails that are not enforcing anything.
        return {"present": False, "implementation": "omnisim_bridges.gate",
                "surface": surface, "fails_closed": True,
                "gated_paths": list(gated_paths or []),
                "ungated_paths": list(ungated_paths or []),
                "note": "the safety gate module could not be imported; "
                        "physical tool calls are refused outright"}
    air = (surface == "drone")
    return {
        "present": bool(present),
        "implementation": "omnisim_bridges.gate",
        "surface": surface,
        # `vet_toolcall` refuses every PHYSICAL tool when the gate will not
        # import or raises. That is what fails_closed means, and it is only
        # claimed because it is what the code does.
        "fails_closed": True,
        "gated_paths": list(gated_paths or []),
        "ungated_paths": list(ungated_paths or []),
        "rules": list(GATE_RULES),
        "rails": rails,
        # Which rail this bridge's `move_body{vertical}` takes.
        # `move_body{vertical}` is a 120 m climb on a drone and a 1.0 m body
        # shift on a quadruped; the surface above is what picks between them,
        # and publishing the choice is how a client checks we picked the one
        # it expected.
        #
        # ⚠️ `null` WHEN THIS BRIDGE DECLARES NO SURFACE, and that is not the
        # same as "the strictest rail". With no caller surface the gate falls
        # back to the surface RECORDED on the spec when exactly one class
        # registered the tool, and only discards it for a genuinely shared
        # one -- so a surface-less bridge cannot say here which rail it will
        # get without knowing what else registered. Publishing a guess would
        # be publishing a safety property nobody checked.
        "vertical_rail": (("max_altitude_m" if air else "max_body_shift_m")
                          if surface else None),
    }


# The reference handler's OWN route table, split by whether the gate is on
# it. Every bridge publishes its own; these are this file's.
BASE_GATED_PATHS = ["/prompt", "/tool"]
BASE_UNGATED_PATHS = [
    "/set_velocity", "/drive_forward", "/turn", "/stop_robot",
    "/reset_to_home", "/set_joint_positions", "/set_tcp_target",
    "/open_gripper", "/close_gripper", "/set_gripper_width", "/grasp",
    "/release",
]


def _capabilities_body(bridge: Any, surface: Optional[str] = None) -> Dict[str, Any]:
    """`/capabilities` with the §5.2.1 `safety_gate` block attached.

    A COPY: the bridge's own `capabilities` dict is never mutated, because
    it is a long-lived object other code reads and a response builder has no
    business editing it.
    """
    body = dict(getattr(bridge, "capabilities", {}) or {})
    body["safety_gate"] = safety_gate_block(
        surface or getattr(bridge, "surface", None),
        gated_paths=list(getattr(bridge, "GATED_PATHS", BASE_GATED_PATHS)),
        ungated_paths=list(getattr(bridge, "UNGATED_PATHS", BASE_UNGATED_PATHS)),
    )
    return body


def _plain_set(obj: Any, name: str, value: Any) -> None:
    """Set an attribute unless the CLASS computes it.

    `BridgeBase.sim_time` is a property over the clock; a demo bridge stores
    a plain float that its tick refreshes. Both are the contract -- a reader
    does `getattr(bridge, "sim_time", 0.0)` and cannot tell -- so the
    installer must not try to overwrite the computed one and raise.
    """
    if isinstance(getattr(type(obj), name, None), property):
        return
    setattr(obj, name, value)


def attach_telemetry(bridge: Any, *, dt_s: float = 0.032, robot_id: str = "",
                     surface: Optional[str] = None, world: str = "",
                     joints: bool = True, contacts: bool = True,
                     capacity: Optional[int] = None) -> Any:
    """Install the D1 clock, the D4 ring + detectors and the D6 hold.

    ONE call, so five bridges cannot drift into five slightly different
    wirings the way six copies of the gate wrapper did. Every attribute it
    sets is one the relay, the wake dispatcher and the presence heartbeat
    read with `getattr(bridge, name, default)`.

    Returns the bridge, so a constructor can end with it.
    """
    from .events import (
        ContactDetector, EventRing, FaultDetector, JointLimitDetector,
        harness_feed,
    )
    from .hold import HoldLease

    bridge.sim_clock = SimClock(dt_s)
    bridge.clock = bridge.sim_clock          # the name the demo bridges use
    _plain_set(bridge, "sim_time", 0.0)
    _plain_set(bridge, "sim_step", 0)
    if not getattr(bridge, "world", ""):
        _plain_set(bridge, "world", world)
    if surface is not None and not getattr(bridge, "surface", None):
        _plain_set(bridge, "surface", surface)
    ring = EventRing(capacity, robot=robot_id or getattr(bridge, "robot_id", ""))
    bridge.events = ring
    bridge.fault_detector = FaultDetector(ring)
    bridge.contact_detector = ContactDetector(ring) if contacts else None
    bridge.joint_detector = JointLimitDetector(ring) if joints else None
    bridge.hold = HoldLease(ring, robot_id or getattr(bridge, "robot_id", ""))
    # D4 step 3: when a World Harness is attached (OMNISIM_HARNESS_URL), its
    # events are mirrored onto this ring with `source: "harness"`, so one
    # client loop sees what the robot measured AND what the supervisor saw.
    # Off its own daemon thread, never the tick -- see HarnessEventFeed.
    bridge.harness_feed = harness_feed(ring, robot=robot_id,
                                       clock=bridge.sim_clock)
    return bridge


def telemetry_tick(bridge: Any, sim_time: Optional[float] = None) -> None:
    """SIM THREAD ONLY, once per `robot.step()`.

    Advances the clock and MIRRORS it onto plain attributes, because every
    other thread reads `bridge.sim_time` / `bridge.sim_step` and a plain
    attribute read cannot accidentally become a Robot API call the way a
    property on a subclass might. `last_tick_at` stays the WALL clock here
    -- it is the liveness ruler the staleness detector differences, and the
    §5.3 field of the same name is computed in `get_state`.
    """
    clock = getattr(bridge, "sim_clock", None) or getattr(bridge, "clock", None)
    if clock is None:
        return
    clock.tick(sim_time)
    _plain_set(bridge, "sim_time", clock.sim_time)
    _plain_set(bridge, "sim_step", clock.step)
    detector = getattr(bridge, "fault_detector", None)
    if detector is not None:
        detector.clear_stale()
        detector.on_tick(getattr(bridge, "fault", None),
                         sim_time=clock.sim_time, step=clock.step,
                         robot=str(getattr(bridge, "robot_id", "") or ""))


def attach_relay(relay: Any, bridge: Any, *,
                 event_sink: Optional[Callable[[str, Dict[str, Any]], None]] = None,
                 window_raiser: Optional[Callable[[], None]] = None) -> Any:
    """Connect a relay to the bridge it reports on (plan D4 step 5, D5).

    ⚠️ WITHOUT THIS CALL THE WHOLE EVENT LOOP IS DEAD CODE. The relay owns
    the wake dispatcher and the presence heartbeat, and both read the
    bridge through one handle: no handle, no ring, so presence reports the
    old three keys and ZERO wakes fire, forever -- silently, because
    "nothing woke me" and "nothing happened" look identical from outside.

    It lives here rather than in each bridge for the reason the
    `get_action_history` tool nearly went missing on the bridges added
    later: a per-bridge opt-in is a list somebody has to remember to
    extend.

    Every argument is optional and every failure is swallowed: a relay from
    an older package has none of these methods, and a bridge must still come
    up when the seam it is reaching for is not there.
    """
    if relay is None:
        return relay
    for name, arg in (("attach_bridge", bridge),
                      ("set_event_sink", event_sink),
                      ("set_window_raiser", window_raiser)):
        if arg is None:
            continue
        fn = getattr(relay, name, None)
        if fn is None:
            continue
        try:
            fn(arg)
        except Exception as exc:                        # pragma: no cover
            print(f"[bridge_base] relay.{name} failed: "
                  f"{type(exc).__name__}: {exc}", flush=True)
    return relay


def close_relay(relay: Any) -> None:
    """Flush and stop a relay when its world shuts down cleanly (plan D7).

    The relay syncs the action journal to the platform on a 30 s beat, and
    `close()` flushes whatever the last beat has not sent yet. Until
    2026-09-22 NO shipped bridge called it, so a clean world close or reload
    lost up to a beat of the record -- the newest actions, which are the ones
    an operator is most likely to ask about next.

    Call it where a bridge's main loop exits because `step()` returned -1.
    It is not an atexit hook on purpose: the engine usually kills
    controllers rather than letting them exit, and a hook doing seconds of
    HTTP at interpreter shutdown would also fire in every test process.
    So a hard kill still loses up to one beat, and that is stated, not
    hidden. Every failure is swallowed: shutdown must never raise.
    """
    if relay is None:
        return
    fn = getattr(relay, "close", None)
    if fn is None:
        return
    try:
        fn()
    except Exception as exc:                            # pragma: no cover
        print(f"[bridge_base] relay.close failed: "
              f"{type(exc).__name__}: {exc}", flush=True)


def profile_extras(bridge: Any, *, http_port: int,
                   surface: Optional[str] = None,
                   actions: Optional[List[str]] = None) -> Dict[str, Any]:
    """The additive `ensure_profile` keywords for the prompt door (plan D3).

    ⚠️ `prompt_callback_url` AND `surface` ARE THE SWITCH. Until a bridge
    passes them the platform has no way to know this robot serves a
    sentence door at all, so it keeps using `/api/chat` + `/tool` and the
    whole parser-first handoff is dead code on the live tree.

    `safety_gate` is built from `safety_gate_block`, the same function
    `/capabilities` uses, so the profile and the capability block cannot
    disagree about which paths are vetted -- two hand-written copies of
    that list is exactly how the two gates drifted.

    ⚠️ AND READ `profile_sync.agent_name_for` BEFORE WIRING A SCRATCH RUN.
    The agent name keys the live profile AND its durable memory, so an
    untagged scratch run on scratch ports does not coexist with the demo,
    it REPOINTS it -- and now at two dead URLs rather than one, the tool
    door and the sentence door. `OMNILINK_AGENT_TAG` is the fix and the
    only one.
    """
    caps = dict(getattr(bridge, "capabilities", {}) or {})
    surface = surface or getattr(bridge, "surface", None)
    gate_block = caps.get("safety_gate")
    if not gate_block:
        gate_block = safety_gate_block(
            surface,
            gated_paths=list(getattr(bridge, "GATED_PATHS", BASE_GATED_PATHS)),
            ungated_paths=list(getattr(bridge, "UNGATED_PATHS",
                                       BASE_UNGATED_PATHS)))
    if actions is None:
        declared = caps.get("actions")
        actions = list(declared) if isinstance(declared, list) else None
    if actions is None:
        actions = []
    # `prompt` is what the platform looks for before it takes this door.
    if "prompt" not in actions:
        actions = list(actions) + ["prompt"]
    return {
        "prompt_callback_url": f"http://127.0.0.1:{int(http_port)}/prompt",
        "surface": surface,
        "actions": actions,
        "safety_gate": gate_block,
    }


def events_summary(bridge: Any) -> Dict[str, Any]:
    """The `/state.events` block, on a bridge that may have no ring."""
    ring = getattr(bridge, "events", None)
    if ring is None:
        return {"total": 0, "last": None, "next_since": 0, "dropped": 0}
    return ring.summary()


def events_query(path: str) -> Dict[str, Any]:
    """`?since=&limit=&types=` off a request path. Tolerant by design: a
    malformed cursor reads as 0 rather than 400, because an agent that has
    lost its place needs the stream more than it needs a lecture."""
    try:
        qs = parse_qs(urlparse(path or "").query)
    except Exception:                                   # pragma: no cover
        return {"since": 0, "limit": 100, "types": None}

    def _int(name: str, default: int) -> int:
        raw = (qs.get(name) or [""])[0]
        try:
            return int(raw)
        except (TypeError, ValueError):
            return default

    raw_types = (qs.get("types") or [""])[0]
    types = [t.strip() for t in raw_types.split(",") if t.strip()] or None
    return {"since": _int("since", 0), "limit": _int("limit", 100),
            "types": types}


def serve_events(bridge: Any, path: str) -> Dict[str, Any]:
    """THE ONE `GET /events` implementation -- shared by every bridge.

    Same envelope as the harness's `/sim/events` (`events[]`, `next_since`,
    `dropped`) so one client loop drains both. `robot` and `sim_time` are
    the bridge's cached values; nothing here touches the Robot API, so it is
    safe on an HTTP thread.

    It also POLLS THE STALENESS DETECTOR on the way through. That is not
    decoration: `fault.telemetry_stale` means the sim thread has stopped, so
    a detector that only ran in `tick()` could never fire for it. The reader
    is the only thread still running when that happens.
    """
    q = events_query(path)
    ring = getattr(bridge, "events", None)
    if ring is None:
        return {"events": [], "next_since": int(q["since"]), "dropped": 0,
                "total": 0,
                "error": "this bridge serves no event ring"}
    detector = getattr(bridge, "fault_detector", None)
    if detector is not None:
        try:
            detector.poll_stale(
                wall_now=time.time(),
                last_tick_wall=float(getattr(bridge, "last_tick_at", 0.0) or 0.0),
                sim_time=float(getattr(bridge, "sim_time", 0.0) or 0.0),
                step=int(getattr(bridge, "sim_step", 0) or 0),
                robot=str(getattr(bridge, "robot_id", "") or ""))
        except Exception:                               # pragma: no cover
            pass
    out = ring.since(q["since"], limit=q["limit"], types=q["types"])
    out["robot"] = str(getattr(bridge, "robot_id", "") or "")
    out["sim_time"] = float(getattr(bridge, "sim_time", 0.0) or 0.0)
    out["step"] = int(getattr(bridge, "sim_step", 0) or 0)
    return out


class BridgeBase(abc.ABC):
    """Subclass and implement the act_* methods that apply to your
    robot. Methods you don't implement default to a clean "unsupported"
    response that the OmniLink relay surfaces as a tool error -- no
    crashes, no NotImplementedError leaking up the wire.
    """

    # Filled in by the subclass.
    robot_id: str = "robot_0"
    model: str = "GenericRobot"
    capabilities: Dict[str, Any] = {}

    # ⚠️ SET THIS. It is the robot CLASS this bridge serves -- "mobile",
    # "arm", "quadruped" or "drone" (the names in interpret.py) -- and the
    # safety gate uses it to pick which magnitude rail a shared tool is
    # judged against: `move_body{vertical: 40}` is a routine climb on a
    # drone and an absurd body shift on a quadruped.
    #
    # ⚠️ WHAT LEAVING IT None ACTUALLY DOES. This comment used to say the
    # gate "falls back to whatever the tool's spec recorded, which is the
    # strictest rail" -- two clauses that contradict each other, and the
    # second is FALSE. It is the THIRD copy of that sentence found in this
    # package; the first two were load-bearing (`vet_toolcall`, whose
    # reading was copied into `safety_gate_block` and made it publish a
    # body-shift rail for a bridge that cannot know which rail applies).
    # The rule in `gate.check` is:
    #
    #   - the CALLER's surface always wins;
    #   - with no caller surface, the gate falls back to the surface
    #     recorded on the spec, but ONLY when exactly one robot class ever
    #     registered that tool;
    #   - when two or more registered it -- a genuinely SHARED tool -- the
    #     recorded surface is discarded and the tool takes the ground /
    #     body-shift rail (1.0 m);
    #   - `takeoff` / `land` / `hover` are air by name whatever is claimed.
    #
    # So "None means the strictest rail" is true of a SHARED tool and false
    # of every other, and it was a live false-refusal bug in the other
    # direction: on the shipped tree only the drone registers `move_body`,
    # so with no surface a 119 m climb is correctly ALLOWED on the air rail.
    # Leaving this None is not a safe default, it is an UNKNOWN one -- a
    # bridge that knows its own class must always declare it, because the
    # fallback is a guess and this attribute is a fact.
    surface: Optional[str] = None

    # ── Telemetry every consumer may read defensively ─────────────
    #
    # These six names are a CONTRACT with the relay, the event dispatcher
    # and the presence heartbeat, which all read them with
    # `getattr(bridge, name, default)` and must never crash on a bridge that
    # predates them. They are lazy rather than set in `__init__`, because
    # the documented way to write a bridge is to subclass and define your
    # own `__init__` without calling super() -- the quickstart in this
    # package's docstring does exactly that -- and a contract that only
    # holds for cooperative subclasses is not a contract.
    fault: Optional[str] = None       # None when healthy (PROTOCOL §5.3)
    world: str = ""                   # world / site name, "" when unknown

    @property
    def sim_clock(self) -> SimClock:
        clock = self.__dict__.get("_sim_clock")
        if clock is None:
            clock = SimClock()
            self.__dict__["_sim_clock"] = clock
        return clock

    @sim_clock.setter
    def sim_clock(self, clock: SimClock) -> None:
        self.__dict__["_sim_clock"] = clock

    @property
    def sim_time(self) -> float:
        """SIM seconds, cached by the sim thread. Never the wall clock."""
        return self.sim_clock.sim_time

    @property
    def sim_step(self) -> int:
        """Basic steps this bridge has ticked."""
        return self.sim_clock.step

    @property
    def events(self) -> EventRing:
        ring = self.__dict__.get("_event_ring")
        if ring is None:
            ring = EventRing(robot=self.robot_id)
            self.__dict__["_event_ring"] = ring
        return ring

    @events.setter
    def events(self, ring: EventRing) -> None:
        self.__dict__["_event_ring"] = ring

    @property
    def held(self) -> bool:
        """True while a lockstep hold (D6) is freezing the world."""
        hold = getattr(self, "hold", None)
        return bool(getattr(hold, "held", False))

    def clock_tick(self, sim_time: Optional[float] = None) -> None:
        """SIM THREAD ONLY. Advance the cached clock one basic step."""
        self.sim_clock.tick(sim_time)

    # ── Stop is mandatory. Every other action is opt-in. ──────────

    @abc.abstractmethod
    def act_stop(self) -> Dict[str, Any]:
        """Emergency halt. Idempotent. ALWAYS available -- this is the
        one method every bridge must implement. Return at minimum
        {"halted_at": <unix-seconds>}."""
        raise NotImplementedError

    # ── State read ────────────────────────────────────────────────

    def get_state(self) -> Dict[str, Any]:
        """Return a snapshot of the robot's state. Subclasses should
        override; the base returns a minimal record so /get_robot_state
        always answers something.

        ⚠️ `last_tick_at` IS SIM SECONDS, as PROTOCOL §5.3 has always shown
        it. It used to send `time.time()` -- 1.7 billion where the spec's
        example reads 12.448 -- so a client differencing it against
        `sim_time` got the age of the Unix epoch. The wall clock has its own
        field now and nothing else reports it.
        """
        return {
            "id": self.robot_id,
            "model": self.model,
            "sim_time": self.sim_time,
            "last_tick_at": self.sim_time,
            "wall_time": time.time(),
            "step": self.sim_step,
            "held": self.held,
            "fault": self.fault,
            "events": self.events.summary(),
        }

    # ── Optional motion actions. Default to "not supported". ──────

    def act_set_velocity(self, linear: float, angular: float) -> Dict[str, Any]:
        return {"error": "set_velocity not supported by this robot kind"}

    def act_drive_forward(self, distance: float, speed: Optional[float] = None) -> Dict[str, Any]:
        return {"error": "drive_forward not supported by this robot kind"}

    def act_turn(self, angle_rad: float) -> Dict[str, Any]:
        return {"error": "turn not supported by this robot kind"}

    def act_set_joint_positions(self, q: List[float], duration_s: float = 1.2) -> Dict[str, Any]:
        return {"error": "set_joint_positions not supported by this robot kind"}

    def act_set_tcp_target(self, xyz: List[float]) -> Dict[str, Any]:
        return {"error": "set_tcp_target not supported by this robot kind"}

    def act_reset_to_home(self) -> Dict[str, Any]:
        return {"error": "reset_to_home not supported by this robot kind"}

    def act_open_gripper(self) -> Dict[str, Any]:
        return {"error": "open_gripper not supported (no gripper on this robot)"}

    def act_close_gripper(self) -> Dict[str, Any]:
        return {"error": "close_gripper not supported (no gripper on this robot)"}

    def act_set_gripper_width(self, width: float) -> Dict[str, Any]:
        return {"error": "set_gripper_width not supported (no gripper on this robot)"}

    def act_grasp(self, force: Optional[float] = None,
                  width: Optional[float] = None) -> Dict[str, Any]:
        return {"error": "grasp not supported (no gripper on this robot)"}

    def act_release(self) -> Dict[str, Any]:
        return {"error": "release not supported (no gripper on this robot)"}

    # ── Natural-language prompt fallback ──────────────────────────

    def act_prompt(self, text: str) -> Dict[str, Any]:
        """Subclasses wire an authenticated OmniLink relay here."""
        from .access import connection_error
        return connection_error()


# ── HTTP server (Axis-normalised, mirrors omnilink_*_bridge) ────────

def _make_handler(
    bridge: BridgeBase,
    *,
    token: str = "",
    trusted_origins: Optional[set[str]] = None,
    surface: Optional[str] = None,
) -> Any:
    """Returns a BaseHTTPRequestHandler subclass bound to `bridge`. The
    routes match the OmniSim bridges' surface exactly so any existing
    OmniLink agent (Foreman, Picker, Roomba, Axis) drives this bridge
    by pointing its callback URL at us."""

    action_lock = threading.RLock()
    request_ids = RequestIdGuard()
    trusted = trusted_origins or allowed_origins()

    class _H(BaseHTTPRequestHandler):
        # HTTP/1.1 keep-alive (back-ported from the mobile bridge, which
        # shipped it first): safe here only because every response path sets
        # an accurate Content-Length. If you add a response path, it MUST set
        # Content-Length or the client will hang waiting for a body. The
        # timeout reaps idle persistent connections so they do not park
        # ThreadingHTTPServer threads forever.
        protocol_version = "HTTP/1.1"
        timeout = 30

        def log_message(self, *args, **kwargs):
            return

        def _json(self, code: int, obj: Any, origin: Optional[str] = None) -> None:
            data = json.dumps(obj, default=str).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("X-OmniSim-Wire", WIRE_VERSION)
            self.send_header("X-OmniSim-Service", WIRE_SERVICE)
            if origin:
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _guard(self, *, authorize: bool = False) -> Optional[str]:
            check_protocol_version(self.headers)
            origin = checked_origin(self.headers, trusted)
            if authorize:
                check_authorization(self.headers, token)
            return origin

        def _failure(self, exc: RequestError, origin: Optional[str] = None) -> None:
            self._json(exc.status, error_envelope(exc.code, exc.message, exc.details), origin)

        def _invoke(self, fn: Callable[[], Any], *, stop: bool = False) -> Any:
            request_ids.claim(
                getattr(self, "_request_path", self.path),
                getattr(self, "_request_id", None),
            )
            if stop:
                return fn()
            with action_lock:
                return fn()

        def _read_json(self, *, allow_empty: bool = True) -> Dict[str, Any]:
            return read_json(self, allow_empty=allow_empty)

        def _cors_preflight(self) -> Optional[str]:
            origin = self._guard(authorize=False)
            requested_headers = (self.headers.get("Access-Control-Request-Headers") or "").lower()
            allowed = {"content-type", "authorization", "x-omnisim-token"}
            requested = {h.strip() for h in requested_headers.split(",") if h.strip()}
            if not requested.issubset(allowed):
                raise RequestError(403, "header_not_allowed", "CORS request includes a disallowed header.")
            return origin

        def do_OPTIONS(self) -> None:
            try:
                origin = self._cors_preflight()
                self.send_response(204)
                if origin:
                    self.send_header("Access-Control-Allow-Origin", origin)
                    self.send_header("Vary", "Origin")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header(
                    "Access-Control-Allow-Headers", "Content-Type, Authorization, X-OmniSim-Token"
                )
                self.end_headers()
            except RequestError as exc:
                self._failure(exc)

        def do_GET(self) -> None:
            origin: Optional[str] = None
            try:
                origin = self._guard()
                path = self.path.split("?", 1)[0].rstrip("/") or "/"
                if path in ("/state", "/get_robot_state"):
                    return self._json(200, bridge.get_state(), origin)
                if path in ("/capabilities", "/list_robots"):
                    return self._json(200, [{
                        "id": bridge.robot_id, "model": bridge.model,
                        "capabilities": _capabilities_body(bridge, surface),
                    }], origin)
                if path == "/events":
                    return self._json(200, serve_events(bridge, self.path), origin)
                if path == "/healthz":
                    return self._json(200, {"ok": True}, origin)
                if path == "/protocol":
                    return self._json(200, {
                        "ok": True,
                        "omnisim_wire": WIRE_VERSION,
                        "service": WIRE_SERVICE,
                        "service_versions": {WIRE_SERVICE: WIRE_VERSION},
                        "instance": {"name": bridge.__class__.__name__, "robot_id": bridge.robot_id},
                        "extensions": [],
                    }, origin)
                if path == "/usage":
                    return self._json(200, {"enabled": False}, origin)
                return self._json(404, error_envelope("not_found", "Endpoint not found."), origin)
            except RequestError as exc:
                self._failure(exc, origin)

        def do_POST(self) -> None:
            origin: Optional[str] = None
            try:
                origin = self._guard(authorize=True)
                body = self._read_json()
                path = self.path.split("?", 1)[0].rstrip("/") or "/"
                self._request_path = path
                self._request_id = validate_request_id(body.get("id"))
                if path in ("/state", "/get_robot_state"):
                    return self._json(200, bridge.get_state(), origin)
                if path in ("/list_robots", "/capabilities"):
                    return self._json(200, [{
                        "id": bridge.robot_id, "model": bridge.model,
                        "capabilities": _capabilities_body(bridge, surface),
                    }], origin)
                if path == "/events":
                    return self._json(200, serve_events(bridge, self.path), origin)
                if path == "/stop_robot":
                    return self._json(200, self._invoke(bridge.act_stop, stop=True), origin)
                if path == "/reset_to_home":
                    result = self._invoke(bridge.act_reset_to_home)
                    return self._action_response(result, origin)
                if path == "/set_velocity":
                    linear = finite_number(require_field(body, "linear"), "linear")
                    angular = finite_number(require_field(body, "angular"), "angular")
                    return self._action_response(
                        self._invoke(lambda: bridge.act_set_velocity(linear, angular)), origin
                    )
                if path == "/drive_forward":
                    distance = finite_number(require_field(body, "distance"), "distance")
                    speed = body.get("speed")
                    if speed is not None:
                        speed = finite_number(speed, "speed")
                    return self._action_response(
                        self._invoke(lambda: bridge.act_drive_forward(distance, speed)), origin
                    )
                if path == "/turn":
                    angle_value = body.get("angle_rad", body.get("angle"))
                    if angle_value is None:
                        raise RequestError(400, "missing_field", "Required field 'angle_rad' is missing.")
                    angle = finite_number(angle_value, "angle_rad")
                    return self._action_response(self._invoke(lambda: bridge.act_turn(angle)), origin)
                if path == "/set_joint_positions":
                    q = number_list(require_field(body, "q"), "q")
                    duration = body.get("duration_s", 1.2)
                    duration_s = finite_number(duration, "duration_s")
                    return self._action_response(
                        self._invoke(lambda: bridge.act_set_joint_positions(q, duration_s)), origin
                    )
                if path == "/set_tcp_target":
                    xyz = number_list(require_field(body, "xyz"), "xyz", length=3)
                    return self._action_response(self._invoke(lambda: bridge.act_set_tcp_target(xyz)), origin)
                if path == "/open_gripper":
                    return self._action_response(self._invoke(bridge.act_open_gripper), origin)
                if path == "/close_gripper":
                    return self._action_response(self._invoke(bridge.act_close_gripper), origin)
                if path == "/set_gripper_width":
                    width = finite_number(require_field(body, "width"), "width")
                    return self._action_response(
                        self._invoke(lambda: bridge.act_set_gripper_width(width)), origin
                    )
                if path == "/grasp":
                    force = body.get("force")
                    width = body.get("width")
                    if force is not None:
                        force = finite_number(force, "force")
                    if width is not None:
                        width = finite_number(width, "width")
                    return self._action_response(
                        self._invoke(lambda: bridge.act_grasp(force=force, width=width)), origin
                    )
                if path == "/release":
                    return self._action_response(self._invoke(bridge.act_release), origin)
                if path == "/prompt":
                    text_value = require_field(body, "text")
                    if not isinstance(text_value, str) or not text_value.strip():
                        raise RequestError(400, "invalid_type", "Field 'text' must be a non-empty string.")
                    # PROTOCOL.md §5.7.2 / plan D3: `via` names WHICH STAGE
                    # answered, and it is REQUIRED on a 200. The sweep's
                    # two-door comparison is built on it and refuses to
                    # guess -- a missing `via` is recorded as null, the
                    # comparison reads `undetermined` and the run fails by
                    # design. So it is stamped here rather than hoped for.
                    from .route import stamp_via
                    return self._action_response(
                        stamp_via(self._invoke(
                            lambda: bridge.act_prompt(text_value.strip()))),
                        origin
                    )
                if path == "/tool":
                    tool_value = require_field(body, "tool")
                    if not isinstance(tool_value, str) or not tool_value.strip():
                        raise RequestError(400, "invalid_type", "Field 'tool' must be a non-empty string.")
                    tool_name = tool_value.strip()
                    # ONE implementation, shared with the four demo bridges.
                    # `surface` is this bridge's robot class, so the rails it
                    # is judged against are its own and not the strictest set.
                    code, payload = serve_tool(
                        tool_name, body,
                        lambda args: self._invoke(
                            lambda: _dispatch_tool(bridge, tool_name, args),
                            stop=tool_name == "stop_robot"),
                        surface=surface or getattr(bridge, "surface", None),
                        bridge=bridge)
                    return self._json(code, payload, origin)
                return self._json(
                    404, error_envelope("not_found", "Endpoint not found.", {"path": path}), origin
                )
            except RequestError as exc:
                self._failure(exc, origin)
            except (TypeError, ValueError) as exc:
                self._failure(RequestError(400, "invalid_arguments", str(exc)), origin)
            except Exception as exc:
                self._json(
                    500,
                    error_envelope("internal_error", "Robot action failed.", {"type": type(exc).__name__}),
                    origin,
                )

        def _action_response(self, result: Any, origin: Optional[str]) -> None:
            # Only a STRING is a failure. `error` in a measured result is the
            # CONTROL error -- a float, achieved minus commanded -- so a
            # settled 0.5 m drive carries error = -0.00086 and the truthiness
            # test would have answered 501 not_supported to a motion that
            # worked. Measured 2026-09-22: the shipped bridges override these
            # routes, so this particular copy is not on a live path today and
            # `POST /drive_forward {wait: true}` correctly returns 200. It is
            # corrected anyway, because the next bridge to inherit the base
            # handler would get the bug for free, and its sibling in
            # `serve_tool` WAS live (see the note there).
            failure = result.get("error") if isinstance(result, dict) else None
            if isinstance(failure, str) and failure:
                self._json(501, error_envelope("not_supported", failure), origin)
                return
            self._json(200, result, origin)

    return _H


_TOOL_ALIASES: Dict[str, str] = {
    "stop_robot":          "act_stop",
    "get_robot_state":     "get_state",
    "reset_to_home":       "act_reset_to_home",
    "set_velocity":        "act_set_velocity",
    "drive_forward":       "act_drive_forward",
    "turn":                "act_turn",
    "set_joint_positions": "act_set_joint_positions",
    "set_tcp_target":      "act_set_tcp_target",
    "open_gripper":        "act_open_gripper",
    "close_gripper":       "act_close_gripper",
    "set_gripper_width":   "act_set_gripper_width",
    "grasp":               "act_grasp",
    "release":             "act_release",
}


def _dispatch_tool(bridge: BridgeBase, tool_name: str, args: Dict[str, Any]) -> Dict[str, Any]:
    """Map an OmniLink tool name onto a bridge act_* method.

    Existence and argument checking only -- the safety vetting lives in
    `serve_tool`, which calls this as its `dispatch`.
    """
    method_name = _TOOL_ALIASES.get(tool_name)
    if method_name is None:
        return {"error": f"unknown tool: {tool_name}", "known": sorted(_TOOL_ALIASES.keys())}
    method = getattr(bridge, method_name, None)
    if method is None:
        return {"error": f"bridge does not implement {method_name}"}
    try:
        inspect.signature(method).bind(**args)
    except TypeError as exc:
        raise RequestError(400, "invalid_arguments", str(exc), {"tool": tool_name}) from exc
    return method(**args)


# ── The ONE fail-closed gate wrapper ─────────────────────────────────
#
# Only reached when `gate` will not import at all, so it cannot ask the
# registry whether a tool is physical and has to guess from the name.
# It mirrors gate._PHYSICAL_NAME_HINTS plus the drone verbs the Mavic's
# own (now deleted) copy carried, because the union is the fail-CLOSED
# direction: refusing a read-only tool on a total gate failure is
# survivable, waving a rotor command through is not.
_PHYSICAL_NAME_HINTS = (
    "drive", "turn", "set_velocity", "stop", "reset", "resume_autonomy",
    "move", "walk", "takeoff", "land", "hover", "pick", "place", "grip",
    "grasp", "release", "sit", "stand", "joint", "tcp", "trolley",
    "goto", "yaw", "fly", "climb",
)

# Fields that belong to the TRANSPORT, not to the tool. `id` is wire-
# protocol request-idempotency metadata (PROTOCOL.md 3.3); left in the body
# it reaches the gate as an argument and a perfectly good
# `takeoff{altitude: 3}` comes back `unknown_arg: id is not a parameter of
# takeoff` -- a transport detail wearing a safety verdict's clothes. That
# is a measured failure, not a hypothetical: the Mavic refused a good
# takeoff for exactly this reason until 2026-09-21.
#
# ⚠️ `surface` IS STRIPPED AND ITS VALUE IS THROWN AWAY. Both halves matter.
#
# Stripped, because the OmniLink platform now sends it on every `/tool`
# call: without this line it lands in the ARGUMENT set and the gate answers
# `400 unknown_arg: surface is not a parameter of drive_forward` -- which
# would refuse EVERY tool call against EVERY shipped bridge. (The platform
# probes per endpoint and drops the field on that error, so the failure
# would have shown up as a silent capability regression rather than an
# outage. Do not rely on that probe; it exists so an old bridge and a new
# platform still work.)
#
# And the value is DROPPED rather than honoured, which is the half that
# matters. The rail a shared verb is judged against is picked by the
# surface, so a caller who could declare `"drone"` would buy the 120 m
# altitude rail for a quadruped's `move_body{vertical}` -- turning a safety
# property into a client-chosen setting. THE BRIDGE'S OWN SURFACE IS
# AUTHORITATIVE and is passed by the handler that owns the route. Nobody
# should later "fix" this by threading the caller's value through.
_TRANSPORT_FIELDS = ("tool", "id", "utterance", "request_id", "surface")


def tool_args(body: Dict[str, Any]) -> Dict[str, Any]:
    """The tool's ARGUMENTS: the request body minus the transport fields."""
    return {k: v for k, v in (body or {}).items() if k not in _TRANSPORT_FIELDS}


def vet_toolcall(tool_name: str, args: Dict[str, Any],
                 utterance: str = "",
                 surface: Optional[str] = None) -> Optional[str]:
    """Vet a bare tool call. None means allow; a string is the reason.

    ⚠️ THE LOGIC LIVES IN gate.reject_toolcall(), NOT HERE. What remains
    is the fail-closed wrapper, which is the one thing the gate cannot do
    for itself: if importing it fails, a physical tool must still be
    refused.

    ⚠️ AND THERE IS EXACTLY ONE OF IT. This file is the reference every
    external bridge copies, and the vetting it used to contain was copied
    with it -- into the mobile, arm, quadruped and Mavic controllers and
    into `relay`, six near-identical wrappers each re-declaring its own
    "is this physical" list. That is how b977772b0 came to add a gate here
    and miss all four bridges, and how the Mavic's copy came to omit
    `grasp`, `joint` and `tcp`. All six were deleted on 2026-09-22 and
    every caller now lands here.

    `surface` is the robot class the CALLER serves ("mobile", "arm",
    "quadruped", "drone"). It picks the rail where two classes share a
    tool: a drone's `move_body{vertical}` is a climb railed at 120 m, a
    quadruped's is a body shift railed at 1.0 m.

    ⚠️ WHAT PASSING None ACTUALLY DOES, because this docstring used to get
    it wrong in a way that reads as a safety guarantee. It said "falls back
    to whatever the spec recorded, which is the strictest rail" -- two
    clauses that contradict each other, and the second is false. The rule
    in `gate.check` is:

      - the CALLER's surface always wins;
      - with no caller surface, the gate falls back to the surface recorded
        on the spec, but ONLY when exactly one class ever registered that
        tool;
      - when two or more classes registered it -- a genuinely SHARED tool --
        the recorded surface is discarded and the tool takes the ground /
        body-shift rail (1.0 m), because guessing air would raise a 1 m
        limit to 120 m while guessing ground is wrong only in the direction
        that refuses a legitimate climb;
      - `takeoff` / `land` / `hover` are air by name whatever the caller says.

    So "None means the strictest rail" is true of a SHARED tool and false of
    every other. Measured on the shipped tree, where only the Mavic
    registers `move_body`: `check(..., [move_body{vertical: 119}])` with no
    surface returns `[]` -- the air rail, allowed. Reading the old sentence
    literally and building a client on it (say, a caller that skipped
    passing `surface` believing it was choosing the safe option) would have
    been wrong about which rail it was getting.

    A bridge that knows its own class should still always send it: the
    fallback is a guess and this argument is a fact.
    """
    try:
        from . import gate as _g
    except Exception as exc:                       # pragma: no cover
        low = (tool_name or "").lower()
        return (f"safety gate unavailable ({type(exc).__name__})"
                if any(k in low for k in _PHYSICAL_NAME_HINTS) else None)
    try:
        return _g.reject_toolcall(tool_name, args, utterance, surface=surface)
    except Exception as exc:                       # pragma: no cover
        try:
            physical = _g.is_physical(tool_name)
        except Exception:
            low = (tool_name or "").lower()
            physical = any(k in low for k in _PHYSICAL_NAME_HINTS)
        return f"safety gate errored ({type(exc).__name__})" if physical else None


#: A rule name from §5.8.3's enumeration: lowercase, snake_case, no spaces.
_RULE_NAME = re.compile(r"^[a-z][a-z0-9_]*$")


def refusal_rule(reason: Any) -> str:
    """The machine-readable rule behind a gate refusal, from its reason string.

    THE PARSER LIVES HERE BECAUSE BOTH PRODUCERS OF THAT STRING DO.
    `gate.reject_toolcall` formats its verdict as `"<rule>: <detail>"`, and
    `vet_toolcall`'s fail-closed wrapper -- five lines up this file --
    answers `"safety gate unavailable (ImportError)"`, which is a real
    refusal with NO rule name in it. A consumer re-parsing the prose has to
    know about both forms, and the one place that knows about both is the
    file that contains one and calls the other.

    ⚠️ THE FAIL-CLOSED CASE GETS `gate_unavailable`, NEVER A GUESS. The
    enumeration is open and a client must tolerate an unknown name, but it
    must never be told `interrogative` when the truth is that nothing was
    checked. That name is spelled identically in `relay._refusal_rule`; the
    two producers of `gate.refused` must agree, because a consumer branching
    on the rule cannot tell which of them wrote the event.

    Returns `"gate_unavailable"` for anything it cannot parse -- never `""`.
    An empty rule looks populated to a schema check and is unusable for the
    automated reaction the field exists to enable, which is the same failure
    shape as a rule readable only out of prose.
    """
    head, sep, _detail = str(reason or "").partition(": ")
    if sep and _RULE_NAME.match(head):
        return head
    return "gate_unavailable"


def _emit_refusal(bridge: Any, tool_name: str, reason: str, *,
                  origin: str = "tool", utterance: str = "",
                  rule: str = "") -> None:
    """File `gate.refused` on the bridge's ring, if it has one.

    Defensive on every read: this is called from an HTTP thread on a bridge
    object that may predate the ring entirely (an external bridge built
    against the v8 reference handler), and a refusal must never turn into a
    500 because the telemetry it wanted to record was absent.
    """
    if bridge is None:
        return
    try:
        emit_gate_refusal(
            getattr(bridge, "events", None), tool_name, reason,
            sim_time=float(getattr(bridge, "sim_time", 0.0) or 0.0),
            step=int(getattr(bridge, "sim_step", 0) or 0),
            robot=str(getattr(bridge, "robot_id", "") or ""),
            # ⚠️ PARSED HERE, NOT LEFT TO EVERY CONSUMER. `rule` is the field
            # a program branches on -- `reason` is prose and may change
            # between releases -- so an empty one makes the event unusable
            # for exactly the automated reaction it exists to enable, while
            # still looking populated to a schema check. Measured on a live
            # Husky: a correctly refused 300 m drive filed
            # `rule: ""` beside `reason: "implausible: distance=300 ..."`.
            rule=rule or refusal_rule(reason), origin=origin,
            utterance=utterance)
    except Exception:                                   # pragma: no cover
        pass


def serve_tool(tool_name: str, body: Dict[str, Any], dispatch: Any,
               *, surface: Optional[str] = None,
               registered: bool = True,
               bridge: Any = None,
               origin: str = "tool") -> tuple:
    """THE ONE `/tool` HTTP implementation. Returns (status_code, envelope).

    Five handlers used to spell this out: this file's, and one in each of
    the four demo bridges. They agreed on the happy path and disagreed
    everywhere else -- three of them never stripped `id`, one of them
    never stripped `utterance`, and each vetted through its own copy of
    the fail-closed wrapper. `/tool` is the omnilink-agents.com web UI
    posting tool calls a model produced on its side, and it is the
    reference implementation every external bridge copies, so a
    disagreement here propagates.

    The order is fixed and is the whole contract:

      1. strip the transport fields, so the gate judges ARGUMENTS only;
      2. refuse an unregistered tool (503) -- the bridge's own registry
         decides existence, never the gate, which deliberately drops
         `unknown_tool`;
      3. vet with the CALLER's surface, fail-closed (400);
      4. dispatch, and report what it returned.

    There is no utterance on this path unless the caller sent one, so the
    INTENT rules (interrogative, prohibition, reported speech, retraction)
    cannot fire and are not expected to. What does fire is the half that
    needs no sentence: magnitude rails, schema validation and unresolved
    deixis. That is what stops `set_velocity {v: 40}` and
    `drive_forward {distance: 300}` arriving over HTTP -- measured
    2026-09-21, when a gated `bridge_base` still let a 300 m drive through
    a bridge's own copy of this handler and the call hung for 90 s while
    the robot drove it.
    """
    args = tool_args(body)
    utterance = str((body or {}).get("utterance") or "")
    if not registered:
        return 503, {"status": "err", "tool": tool_name,
                     "error": "tool_not_registered"}
    reason = vet_toolcall(tool_name, args, utterance, surface=surface)
    if reason is not None:
        # D4: a refusal is an EVENT, not just a status code. It is the one
        # thing that happens on this path which the operator never sees --
        # the caller gets a 400 and the robot's own agent learns nothing --
        # so it goes on the ring with the surface it arrived on. `origin`
        # is what stops a refusal the agent caused in its own turn from
        # buying it a second model turn (events.may_wake).
        _emit_refusal(bridge, tool_name, reason, origin=origin,
                      utterance=utterance)
        # `rule` rides in the 400 too, for the same reason it rides on the
        # event: the caller of `/tool` has exactly the same re-parse problem
        # as a reader of `/events`, and `reason` is prose it must not branch
        # on. Additive under 1.1 -- `message` is unchanged.
        return 400, error_envelope("refused_by_gate", reason,
                                   {"tool": tool_name,
                                    "rule": refusal_rule(reason)})
    try:
        result = dispatch(args)
    except RequestError:
        raise
    except Exception as exc:
        return 500, {"status": "err", "tool": tool_name,
                     "error": "tool_execution_failed",
                     "detail": f"{type(exc).__name__}: {exc}"}
    # ⚠️ `error` in a motion result is the CONTROL error -- a FLOAT, the signed
    # difference between what was commanded and what the robot achieved. A
    # perfect 2 m drive comes back with error = -0.0022885799407958984, and
    # `if result.get("error")` reads that as a failure because a non-zero float
    # is truthy. Measured 2026-09-22 on the Husky: `POST /tool drive_forward
    # {distance: 1.0}` returned `accepted: true, achieved: 0.9977, settled:
    # true` under `"status": "err"`. Only a control error of EXACTLY 0.0 would
    # have passed, so every successful motion reported failure -- and since the
    # six per-bridge copies of this handler were collapsed into this one
    # function earlier today, the bug was about to become universal instead of
    # merely widespread. An agent driving through `/tool` would have concluded
    # its every command failed.
    #
    # A REAL failure is a STRING: "unknown tool ...", "tool_not_registered".
    # So the failure test is the type, not the truthiness. This is the same
    # trap recorded in AGENTS.md ("`error` is the CONTROL error, a float"),
    # which once printed "I could not stop: 4.3e-11" after a perfect stop.
    failure = result.get("error") if isinstance(result, dict) else None
    if isinstance(failure, str) and failure:
        code = 404 if failure.startswith("unknown tool") else 200
        return code, {"status": "err", "tool": tool_name, "result": result}
    return 200, {"status": "ok", "tool": tool_name, "result": result}


def serve_http(
    bridge: BridgeBase,
    port: int = 8765,
    *,
    host: str = "127.0.0.1",
    token: Optional[str] = None,
    origins: Optional[List[str]] = None,
    surface: Optional[str] = None,
) -> ThreadingHTTPServer:
    """Spin up the bridge HTTP server in a background thread. Returns
    the server instance so the caller can `server.shutdown()` cleanly."""
    resolved_token = configured_token(token)
    validate_bind(host, resolved_token)
    server = ThreadingHTTPServer(
        (host, port),
        _make_handler(bridge, token=resolved_token,
                      trusted_origins=allowed_origins(origins),
                      surface=surface or getattr(bridge, "surface", None)),
    )
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[bridge_base] HTTP listening on http://{host}:{server.server_address[1]}")
    return server
