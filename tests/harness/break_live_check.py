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

"""Live proof that break-on-event freezes a real engine (v9 C2).

Not a unit test: this drives a RUNNING harness. Start one first --

    python -m omnisim harness --port 6789
    python tests/harness/break_live_check.py --url http://127.0.0.1:6789

-- and it runs four cases, printing a transcript and a JSON verdict:

  CASE 0 (pause)       POST /sim/pause / /sim/resume on their own (v9 C1):
                       both clocks stop across a real sleep, /sim/step
                       single-steps and re-holds, resume is idempotent, a
                       second pause extends the lease, and the lease expires
                       on its own with nobody calling resume.
  CASE 1 (contact)     a box is dropped, a break armed on `contact.began`
                       filtered to it, and the freeze is proven by: the clock
                       stopping across a real sleep, the box sitting at contact
                       height, and a SECOND box still in mid-air (which is what
                       separates "frozen at the event" from "settled"), then
                       /sim/step 10 advancing exactly 10 basic steps and
                       re-holding, then resume returning the scene to free-run.
  CASE 2 (joint limit) the same cycle, on a gravity-driven hinge running into
                       its own stop, broken on `joint.limit_hit`.
  CASE 3 (honesty)     the same break armed in a LIGHT session, which must be
                       REFUSED with `event_type_silenced_in_light_mode` rather
                       than silently accepted -- a break that can never fire is
                       indistinguishable from a bug that never happened.

`--json <path>` writes the verdict (including the MEASURED hold latency) so a
report can cite a file rather than a memory. The measured numbers belong to
the machine that ran it: pair them with
`python projects/policies/common/env_fingerprint.py`.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
WORLDS = Path(__file__).resolve().parent / "worlds"

DROP_WORLD = WORLDS / "break_drop.omniworld"
JOINT_WORLD = WORLDS / "break_joint.omniworld"


class CheckError(AssertionError):
    pass


class Client:
    def __init__(self, base: str, verbose: bool = True):
        self.base = base.rstrip("/")
        self.verbose = verbose
        self.transcript: list[dict] = []

    def _record(self, method: str, path: str, body, status: int, out) -> None:
        self.transcript.append({"method": method, "path": path, "body": body,
                                "status": status, "response": out})
        if self.verbose:
            short = json.dumps(out)
            if len(short) > 600:
                short = short[:600] + " ...(truncated)"
            print(f"  {method} {path} -> {status} {short}", flush=True)

    def request(self, method: str, path: str, body=None, timeout: float = 120.0):
        data = None
        headers = {}
        if body is not None:
            data = json.dumps(body).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(self.base + path, data=data,
                                     headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                out = json.loads(resp.read().decode("utf-8") or "{}")
                status = resp.status
        except urllib.error.HTTPError as exc:
            raw = exc.read().decode("utf-8") or "{}"
            try:
                out = json.loads(raw)
            except ValueError:
                out = {"raw": raw}
            status = exc.code
        self._record(method, path, body, status, out)
        return status, out

    def get(self, path, **kw):
        return self.request("GET", path, **kw)

    def post(self, path, body=None, **kw):
        return self.request("POST", path, body if body is not None else {}, **kw)

    def delete(self, path, **kw):
        return self.request("DELETE", path, **kw)


def expect(cond, message: str) -> None:
    if not cond:
        raise CheckError(message)


def node_z(client: Client, def_name: str) -> float:
    status, out = client.get(f"/scene/node/{def_name}")
    expect(status == 200, f"/scene/node/{def_name} -> {status}: {out}")
    return float(out["position"][2])


def sim_state(client: Client) -> dict:
    status, out = client.get("/sim/state")
    expect(status == 200, f"/sim/state -> {status}: {out}")
    return out


def wait_for_break(client: Client, break_id: str, timeout_s: float = 30.0) -> dict:
    """Poll /sim/state until the break records a hit. Returns the hit."""
    deadline = time.time() + timeout_s
    last = None
    while time.time() < deadline:
        last = sim_state(client)
        hit = last.get("break_hit")
        if hit and hit.get("break_id") == break_id:
            return hit
        time.sleep(0.25)
    raise CheckError(f"break {break_id} never fired within {timeout_s}s; "
                     f"last /sim/state = {json.dumps(last)}")


def prove_frozen(client: Client, sleep_s: float = 1.5) -> dict:
    """Two clock reads separated by a REAL sleep. BOTH clocks must be equal.

    Both, because they are different rulers: `sim_time_ms` is the injected
    supervisor's own per-iteration counter and `engine_time_ms` is the
    engine's. Checking only the supervisor's would pass over an engine that
    was still free-running -- which is exactly what happened before the loop
    learned to skip its bookkeeping while held.
    """
    first = sim_state(client)
    time.sleep(sleep_s)
    second = sim_state(client)
    expect(first["sim_time_ms"] == second["sim_time_ms"],
           f"the supervisor clock advanced while held: "
           f"{first['sim_time_ms']} -> {second['sim_time_ms']} over {sleep_s}s")
    expect(first["engine_time_ms"] == second["engine_time_ms"],
           f"the ENGINE clock advanced while held: "
           f"{first['engine_time_ms']} -> {second['engine_time_ms']} over {sleep_s}s")
    return {"sim_time_ms": first["sim_time_ms"],
            "engine_time_ms": first["engine_time_ms"],
            "held_for_wall_s": sleep_s}


GRAVITY = 9.81
WITNESS_DROP_Z = 25.0


def witness_implied_engine_ms(z: float) -> float:
    """Turn the free-falling witness's height back into a sim time.

    `z = z0 - 1/2 g t^2`, so `t = sqrt(2 (z0 - z) / g)`. It is an INDEPENDENT
    ruler for the engine clock the harness reports: the scene itself, not the
    supervisor's bookkeeping.
    """
    drop = max(0.0, WITNESS_DROP_Z - z)
    return (2.0 * drop / GRAVITY) ** 0.5 * 1000.0


def single_step_and_rehold(client: Client, steps: int, basic_ms: float) -> dict:
    before = sim_state(client)
    status, out = client.post("/sim/step", {"steps": steps})
    expect(status == 200, f"/sim/step -> {status}: {out}")
    expect(out.get("paused") is True,
           f"/sim/step did not re-take the pause: {out}")
    expect(isinstance(out.get("lease_remaining_ms"), int)
           and out["lease_remaining_ms"] > 0,
           f"/sim/step reported no lease_remaining_ms: {out}")
    after = sim_state(client)
    advanced = after["sim_time_ms"] - before["sim_time_ms"]
    expect(abs(advanced - steps * basic_ms) < 1e-6,
           f"/sim/step {steps} advanced {advanced} ms, expected {steps * basic_ms}")
    engine_advanced = after["engine_time_ms"] - before["engine_time_ms"]
    # ⚠ THE ENGINE CLOCK IS NOT EXACT HERE, and pretending otherwise would be
    # the lie. Lifting the pause, stepping and re-QUEUING pause is a race the
    # supervisor binding gives no way to close: the re-pause is serviced at an
    # engine step boundary and a free-running engine gets some more steps in
    # first. MEASURED 2026-09-22, 20 x `/sim/step 1`: engine advanced 8-24 ms
    # for a requested 8 (mean 14.4); 20 x `/sim/step 10`: 80-112 ms for a
    # requested 80 (mean 95.6), and up to 120 ms on a loaded machine. So the
    # check is one-sided -- never LESS than requested -- and the overshoot is
    # reported, which is what `/sim/step`'s own `engine_advanced_ms` is for.
    expect(engine_advanced >= steps * basic_ms - 1e-6,
           f"/sim/step {steps} moved the ENGINE clock by only {engine_advanced} ms, "
           f"less than the requested {steps * basic_ms}")
    # And it must STILL be held afterwards.
    prove_frozen(client, sleep_s=0.8)
    return {"steps": steps, "supervisor_advanced_ms": advanced,
            "engine_advanced_ms": engine_advanced,
            "engine_advanced_ms_reported_by_step": out.get("engine_advanced_ms"),
            "lease_remaining_ms": out["lease_remaining_ms"]}


# ---------------------------------------------------------------------------
# CASE 0 -- the pause verbs themselves (v9 C1)
# ---------------------------------------------------------------------------


def case_pause(client: Client) -> dict:
    """POST /sim/pause and /sim/resume, against a live engine.

    Five claims, each one a thing the pause has to be true about:
      1. it stops the clock -- BOTH clocks -- across a real sleep;
      2. /sim/step {steps: N} while held advances exactly N basic steps on the
         supervisor clock, reports `paused` and `lease_remaining_ms`, and
         leaves the engine held;
      3. resume is idempotent, and resuming with nothing held is not an error;
      4. a second pause while held EXTENDS the lease instead of failing;
      5. the lease EXPIRES on its own and the loop resumes free-running, with
         nobody calling resume -- the safety property that a crashed client
         cannot freeze the engine.
    """
    print("\n=== CASE 0: /sim/pause + /sim/resume (v9 C1) ===", flush=True)
    status, out = client.post("/world/load",
                              {"path": str(DROP_WORLD), "light": False},
                              timeout=180)
    expect(status == 200 and out.get("ok"), f"/world/load -> {status}: {out}")
    basic_ms = float(sim_state(client)["basic_time_step_ms"])

    # (3a) resume with nothing held is not an error.
    status, idle_resume = client.post("/sim/resume")
    expect(status == 200, f"/sim/resume with no lease -> {status}: {idle_resume}")
    expect(idle_resume["paused"] is False and idle_resume["was_paused"] is False,
           idle_resume)

    # (1) the clock stops.
    status, paused = client.post("/sim/pause", {"lease_ms": 120000})
    expect(status == 200, f"/sim/pause -> {status}: {paused}")
    expect(paused["paused"] is True and paused["lease_ms"] == 120000, paused)
    frozen = prove_frozen(client, sleep_s=2.0)

    # (2) single-stepping under the hold.
    stepped = single_step_and_rehold(client, 1, basic_ms)
    stepped10 = single_step_and_rehold(client, 10, basic_ms)

    # (4) a second pause extends rather than failing.
    before_extend = sim_state(client)["lease_remaining_ms"]
    status, extended = client.post("/sim/pause", {"lease_ms": 200000})
    expect(status == 200, f"second /sim/pause -> {status}: {extended}")
    expect(extended["paused"] is True, extended)
    expect(extended["lease_remaining_ms"] > before_extend,
           f"a second pause did not extend the lease: {before_extend} -> {extended}")
    expect(extended["paused_at_sim_ms"] == paused["paused_at_sim_ms"],
           f"a re-take moved paused_at_sim_ms: {paused} -> {extended}")

    # (3b) resume is idempotent.
    status, r1 = client.post("/sim/resume")
    expect(status == 200 and r1["was_paused"] is True, r1)
    status, r2 = client.post("/sim/resume")
    expect(status == 200 and r2["was_paused"] is False,
           f"a second resume was not a no-op: {r2}")
    a = sim_state(client)
    time.sleep(1.0)
    b = sim_state(client)
    expect(b["engine_time_ms"] > a["engine_time_ms"],
           f"the engine did not free-run after resume: {a} -> {b}")

    # (5) the lease expires on its own. NOBODY calls resume here.
    status, short = client.post("/sim/pause", {"lease_ms": 1000})
    expect(status == 200 and short["lease_ms"] == 1000, short)
    held = sim_state(client)
    expect(held["paused"] is True, held)
    time.sleep(4.0)
    freed = sim_state(client)
    expect(freed["paused"] is False,
           f"the lease did not self-expire: {freed}")
    expect(freed["engine_time_ms"] > held["engine_time_ms"],
           f"the engine did not resume after the lease expired: "
           f"{held['engine_time_ms']} -> {freed['engine_time_ms']}")
    # And the supervisor is still answering -- the expiry path used to leave
    # it blocked in supervisor.step() against a paused engine.
    time.sleep(1.0)
    still = sim_state(client)
    expect(still["supervisor_connected"] is True,
           f"the supervisor did not survive the lease expiry: {still}")

    return {
        "basic_time_step_ms": basic_ms,
        "resume_with_no_lease": idle_resume,
        "pause": paused,
        "frozen": frozen,
        "step_1": stepped,
        "step_10": stepped10,
        "extend": {"before_ms": before_extend, "after": extended},
        "resume_idempotent": [r1, r2],
        "free_running_after_resume": {"engine_time_ms": [a["engine_time_ms"],
                                                         b["engine_time_ms"]]},
        "lease_expiry": {
            "lease_ms": 1000,
            "held": {"paused": held["paused"],
                     "engine_time_ms": held["engine_time_ms"]},
            "after_4s": {"paused": freed["paused"],
                         "engine_time_ms": freed["engine_time_ms"]},
            "supervisor_still_connected": still["supervisor_connected"],
        },
    }


# ---------------------------------------------------------------------------
# CASE 1 -- contact
# ---------------------------------------------------------------------------


def case_contact(client: Client) -> dict:
    """CASE 1: freeze a falling box at the instant it touches the floor.

    Run in TWO phases, because they answer two different questions and only
    one of them is deterministic.

    PHASE A, free-running: arm, let the simulation run, see whether the break
    catches it. This is the workflow an agent actually wants, and it works --
    but its resolution is one supervisor tick, and on a three-body world the
    engine free-runs in --mode=fast so far ahead of the injected controller
    that a tick has MEASURED anywhere from 8 ms to over 2 s of engine time
    depending on machine load. A whole drop can happen inside one tick, and
    then the contact SET never changes between polls and no contact.began is
    emitted at all. So phase A is recorded as evidence, not asserted.

    PHASE B, held and single-stepped: take the pause first, then advance one
    basic step at a time. `/sim/step` polls the trackers once per basic step,
    so detection is per-step and the freeze lands on the exact step of the
    contact. This is where the assertions live, and it is also the workflow a
    debugger uses: hold, step, look.
    """
    print("\n=== CASE 1: break on contact.began (non-light session) ===", flush=True)
    status, out = client.post("/world/load",
                              {"path": str(DROP_WORLD), "light": False},
                              timeout=180)
    expect(status == 200 and out.get("ok"), f"/world/load -> {status}: {out}")
    expect(out.get("supervisor_connected"), f"no supervisor: {out}")
    basic_ms = float(sim_state(client)["basic_time_step_ms"])
    break_body = {
        "types": ["contact.began"],
        "filter": {"def": "BREAK_BOX", "counterpart": "BREAK_FLOOR"},
        "lease_ms": 300000,
        "once": True,
    }

    # -- PHASE A: free-running -------------------------------------------
    status, armed = client.post("/sim/break", break_body)
    expect(status == 200, f"/sim/break -> {status}: {armed}")
    expect(armed["armed_types"] == ["contact.began"], armed)
    expect(armed["diagnostics"] == [], armed)
    free_id = armed["break_id"]

    status, listed = client.get("/sim/breaks")
    expect(status == 200, listed)
    expect([b["break_id"] for b in listed["breaks"]] == [free_id], listed)
    expect(listed["breaks"][0]["armed"] is True, listed)

    status, reset = client.post("/sim/reset", {"restore": "__init__"}, timeout=180)
    expect(status == 200, f"/sim/reset -> {status}: {reset}")
    poses = reset["verification"]["poses_after"]
    expect(abs(poses["BREAK_BOX"][2] - 5.0) < 1e-3,
           f"reset did not re-lift the box: {poses}")

    free_run: dict = {"fired": False}
    try:
        hit_a = wait_for_break(client, free_id, timeout_s=20.0)
        free_run = {
            "fired": True,
            "matched": hit_a["matched"],
            "engine_time_ms_at_hold": hit_a.get("engine_time_ms_at_hold"),
            "hold_latency_ms": hit_a["hold_latency_ms"],
            "hold_latency_engine_ms_max": hit_a.get("hold_latency_engine_ms_max"),
            "box_z": node_z(client, "BREAK_BOX"),
            "witness_z": node_z(client, "BREAK_WITNESS"),
        }
    except CheckError as exc:
        free_run = {"fired": False, "why": str(exc)[:300]}
    client.post("/sim/resume")
    status, removed = client.delete(f"/sim/break/{free_id}")
    expect(status == 200 and removed["removed"] is True, removed)
    status, gone = client.delete(f"/sim/break/{free_id}")
    expect(gone.get("code") == "BREAK_NOT_FOUND",
           f"a second delete of the same break was not a 404: {status} {gone}")

    # -- PHASE B: held, single-stepped -----------------------------------
    status, paused = client.post("/sim/pause", {"lease_ms": 300000})
    expect(status == 200 and paused["paused"] is True, paused)
    status, reset = client.post("/sim/reset", {"restore": "__init__"}, timeout=180)
    expect(status == 200, f"/sim/reset (held) -> {status}: {reset}")
    poses = reset["verification"]["poses_after"]
    expect(abs(poses["BREAK_BOX"][2] - 5.0) < 1e-3,
           f"reset did not re-lift the box: {poses}")

    status, armed = client.post("/sim/break", break_body)
    expect(status == 200, f"/sim/break -> {status}: {armed}")
    break_id = armed["break_id"]
    # The ENGINE clock is absolute (it does NOT rewind with /sim/reset -- it
    # read 118 s and 152 s on successive runs of this same fixture), so the
    # witness cross-check has to be a DELTA from the moment of the reset. The
    # engine is held here, so this read is exact.
    engine_t0 = sim_state(client)["engine_time_ms"]

    # ONE call: "advance up to 400 basic steps, and stop on the breakpoint".
    # The step batch aborts on the step the break fires, so this is `continue`
    # in a debugger, and `steps_executed` says where it stopped.
    status, ran = client.post("/sim/step", {"steps": 400}, timeout=180)
    expect(status == 200, f"/sim/step -> {status}: {ran}")
    expect(ran.get("stopped_on_break") == break_id,
           f"/sim/step 400 did not stop on the break: {ran}")
    expect(ran["steps_executed"] < ran["steps_requested"],
           f"the batch ran to the end instead of stopping: {ran}")
    steps_taken = ran["steps_executed"]
    state = sim_state(client)
    hit = state.get("break_hit")
    expect(hit is not None and hit.get("break_id") == break_id,
           f"/sim/state carries no break_hit for {break_id}: {state}")

    state = sim_state(client)
    expect(state.get("paused") is True, f"/sim/state does not report paused: {state}")
    frozen = prove_frozen(client)

    box_z = node_z(client, "BREAK_BOX")
    witness_z = node_z(client, "BREAK_WITNESS")
    # Contact height for a 0.2 box on a floor whose top is z=0: NOT the 5 m it
    # was dropped from, and NOT through the floor.
    expect(abs(box_z - 0.1) < 0.06,
           f"BREAK_BOX is at z={box_z}, not at contact height 0.1")
    # THE discriminator. A settled scene has the witness resting at ~0.3; it
    # is still ~20 m up at the moment the box lands, so this one number
    # separates "frozen at the event" from "frozen after everything settled".
    expect(witness_z > 5.0,
           f"BREAK_WITNESS is at z={witness_z}: the scene had already settled, "
           f"so the freeze was not at the moment of the event")
    # Independent cross-check on the engine clock: the witness is in free
    # fall, so its height IS a clock.
    implied = witness_implied_engine_ms(witness_z)
    engine_at_hold = hit.get("engine_time_ms_at_hold")

    stepped = single_step_and_rehold(client, 10, basic_ms)
    witness_after_step = node_z(client, "BREAK_WITNESS")
    expect(witness_after_step < witness_z,
           f"10 steps did not move the falling witness: {witness_z} -> {witness_after_step}")

    status, resumed = client.post("/sim/resume")
    expect(status == 200 and resumed["paused"] is False, resumed)
    a = sim_state(client)
    time.sleep(1.5)
    b = sim_state(client)
    expect(b["sim_time_ms"] > a["sim_time_ms"],
           f"the supervisor clock did not restart after resume: {a} -> {b}")
    expect(b["engine_time_ms"] > a["engine_time_ms"],
           f"the engine clock did not restart after resume: {a} -> {b}")

    client.delete(f"/sim/break/{break_id}")
    return {
        "break_id": break_id,
        "free_running_phase": free_run,
        "steps_to_contact": steps_taken,
        "continue_response": ran,
        "hit": hit,
        "hold_latency_ms": hit["hold_latency_ms"],
        "hold_latency_steps": hit["hold_latency_steps"],
        "hold_latency_engine_ms_max": hit.get("hold_latency_engine_ms_max"),
        "engine_time_ms_at_hold": engine_at_hold,
        "basic_time_step_ms": basic_ms,
        "frozen": frozen,
        "box_z_at_break": box_z,
        "witness_z_at_break": witness_z,
        "witness_implied_engine_ms": implied,
        "engine_time_ms_at_reset": engine_t0,
        "engine_ms_since_reset_at_hold": (
            None if engine_at_hold is None else engine_at_hold - engine_t0),
        "witness_vs_reported_engine_ms": (
            None if engine_at_hold is None
            else implied - (engine_at_hold - engine_t0)),
        "witness_z_after_10_steps": witness_after_step,
        "step": stepped,
        "clock_after_resume": {"sim_time_ms": [a["sim_time_ms"], b["sim_time_ms"]],
                               "engine_time_ms": [a["engine_time_ms"], b["engine_time_ms"]]},
    }


# ---------------------------------------------------------------------------
# CASE 2 -- joint limit
# ---------------------------------------------------------------------------


def case_joint(client: Client) -> dict:
    print("\n=== CASE 2: break on joint.limit_hit (non-light session) ===", flush=True)
    status, out = client.post("/world/load",
                              {"path": str(JOINT_WORLD), "light": False},
                              timeout=180)
    expect(status == 200 and out.get("ok"), f"/world/load -> {status}: {out}")
    basic_ms = float(sim_state(client)["basic_time_step_ms"])

    status, joints = client.get("/robot/BREAK_ARM/joints")
    expect(status == 200, f"/robot/BREAK_ARM/joints -> {status}: {joints}")

    status, armed = client.post("/sim/break", {
        "types": ["joint.limit_hit"],
        "filter": {"joint": "arm_motor"},
        "lease_ms": 120000,
        "once": True,
    })
    expect(status == 200, f"/sim/break -> {status}: {armed}")
    break_id = armed["break_id"]

    status, reset = client.post("/sim/reset", {"restore": "__init__"}, timeout=180)
    expect(status == 200, f"/sim/reset -> {status}: {reset}")
    # Drive the hinge past its own stop. Gravity alone does NOT do it: the
    # RotationalMotor damps the joint even with no controller attached (the
    # engine logs the hinge as `motorized: kd=15`), so the link hangs where it
    # was authored. MEASURED 2026-09-22 -- waiting 25 s for gravity produced
    # no joint.limit_hit at all.
    fired_by = "joints/set"
    status, driven = client.post("/robot/BREAK_ARM/joints/set",
                                 {"joints": {"arm_motor": 1.2}, "settle_steps": 60},
                                 timeout=180)
    expect(status == 200, f"/robot/BREAK_ARM/joints/set -> {status}: {driven}")
    hit = wait_for_break(client, break_id, timeout_s=30.0)

    state = sim_state(client)
    expect(state.get("paused") is True, f"/sim/state does not report paused: {state}")
    expect(hit["matched"]["joint"] == "arm_motor", hit)
    frozen = prove_frozen(client)

    witness_z = node_z(client, "BREAK_WITNESS")
    status, joints_at_break = client.get("/robot/BREAK_ARM/joints")
    expect(status == 200, joints_at_break)
    hinge = [j for j in joints_at_break.get("joints", [])
             if j.get("name") == "arm_motor"]
    expect(hinge, f"no arm_motor in {joints_at_break}")

    stepped = single_step_and_rehold(client, 10, basic_ms)

    status, resumed = client.post("/sim/resume")
    expect(status == 200 and resumed["paused"] is False, resumed)
    a = sim_state(client)
    time.sleep(1.5)
    b = sim_state(client)
    expect(b["engine_time_ms"] > a["engine_time_ms"],
           f"the engine clock did not restart after resume: {a} -> {b}")

    client.delete(f"/sim/break/{break_id}")
    return {
        "break_id": break_id,
        "hit": hit,
        "hold_latency_ms": hit["hold_latency_ms"],
        "hold_latency_steps": hit["hold_latency_steps"],
        "hold_latency_engine_ms_max": hit.get("hold_latency_engine_ms_max"),
        "engine_time_ms_at_hold": hit.get("engine_time_ms_at_hold"),
        "basic_time_step_ms": basic_ms,
        "frozen": frozen,
        "witness_z_at_break": witness_z,
        "witness_implied_engine_ms": witness_implied_engine_ms(witness_z),
        "fired_by": fired_by,
        "joint_at_break": hinge[0],
        "step": stepped,
        "clock_after_resume": {"sim_time_ms": [a["sim_time_ms"], b["sim_time_ms"]],
                               "engine_time_ms": [a["engine_time_ms"], b["engine_time_ms"]]},
    }


# ---------------------------------------------------------------------------
# CASE 3 -- the light-session refusal
# ---------------------------------------------------------------------------


def case_light_refusal(client: Client) -> dict:
    print("\n=== CASE 3: the same break in a LIGHT session must be REFUSED ===",
          flush=True)
    status, out = client.post("/world/load",
                              {"path": str(DROP_WORLD), "light": True},
                              timeout=180)
    expect(status == 200 and out.get("ok"), f"/world/load -> {status}: {out}")

    status, refused = client.post("/sim/break", {
        "types": ["contact.began"],
        "filter": {"def": "BREAK_BOX"},
    })
    expect(status == 400, f"a light session accepted a contact break: {status} {refused}")
    expect(refused.get("code") == "BREAK_EVENT_TYPE_UNAVAILABLE", refused)
    codes = [d.get("code") for d in refused.get("diagnostics") or []]
    expect("event_type_silenced_in_light_mode" in codes, refused)
    diag = next(d for d in refused["diagnostics"]
                if d["code"] == "event_type_silenced_in_light_mode")
    expect('"light": false' in diag.get("workaround", ""),
           f"the refusal does not name the workaround: {diag}")

    status, listed = client.get("/sim/breaks")
    expect(status == 200 and listed["breaks"] == [],
           f"a refused break was armed anyway: {listed}")
    expect("contact.began" in listed.get("silenced_types", []), listed)

    # A break the light session CAN serve is still accepted -- the refusal is
    # scoped to what is actually silenced, not to light mode as a whole.
    status, ok = client.post("/sim/break", {"types": ["damage.impact"]})
    expect(status == 200, f"a light session refused a damage break too: {ok}")
    client.delete(f"/sim/break/{ok['break_id']}")
    return {"refusal": refused, "silenced_types": listed.get("silenced_types"),
            "damage_break_accepted": ok["break_id"]}


# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="http://127.0.0.1:6789")
    ap.add_argument("--json", dest="json_out", default="")
    ap.add_argument("--cases", default="0,1,2,3")
    args = ap.parse_args(argv)

    client = Client(args.url)
    wanted = {c.strip() for c in args.cases.split(",") if c.strip()}
    results: dict = {"url": args.url, "cases": {}}
    failures: list[str] = []
    for name, fn in (("0", case_pause), ("1", case_contact), ("2", case_joint),
                     ("3", case_light_refusal)):
        if name not in wanted:
            continue
        try:
            results["cases"][name] = {"ok": True, "detail": fn(client)}
            print(f"--- CASE {name}: PASS", flush=True)
        except CheckError as exc:
            results["cases"][name] = {"ok": False, "error": str(exc)}
            failures.append(f"CASE {name}: {exc}")
            print(f"--- CASE {name}: FAIL: {exc}", flush=True)

    results["ok"] = not failures
    results["failures"] = failures
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(results, indent=2), encoding="utf-8")
        print(f"\nwrote {args.json_out}", flush=True)
    if failures:
        print("\nFAILURES:\n  " + "\n  ".join(failures), flush=True)
        return 1
    print("\nALL CASES PASS", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
