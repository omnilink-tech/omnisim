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

"""OmniSim agent-facing validation harness.

A long-running HTTP service that wraps a headless OmniSim subprocess so coding
agents can load worlds, fetch structured load diagnostics, and probe the
running scene without launching the desktop GUI or paying full process-start
cost per check.

Endpoints:
    GET  /capabilities        -> what this harness can do, whether the physics
                                 backend actually came up (read from the engine's
                                 .newton.json verdict sidecar; Newton is the only
                                 backend since src/ode was deleted, so the
                                 question is "did it finalise", not "which one"
                                 -- and `source` names the provenance rather than
                                 guessing), which event types are live, what is
                                 explicitly NOT supported and why (harness gaps
                                 AND engine gaps), and a measured per-step cost so
                                 a step budget can be sized instead of discovered
                                 by timing out. (The ~790x /sim/step figure quoted
                                 below belongs to light mode, not to a choice of
                                 backend.)
                                 ?probe_step=1 advances ONE step to measure the
                                 cost on this world if none has been measured.
    POST /world/load          {"path": "...", "wait_s": 30.0, "with_supervisor": true,
                               "light"?: bool, "tracking"?: {...}}
                              ⭐ LIGHT IS THE DEFAULT SINCE 2026-09-02. A request
                              that names neither `light` nor `tracking` runs
                              light; the response's `tracking` block then
                              carries `default_applied: true` and one sentence
                              naming how to get the trackers back. An explicit
                              `light` (either value) or any `tracking` object
                              always wins. OMNISIM_HARNESS_LIGHT=0 restores
                              full tracking as the process-wide default
                              (unset or =1 -> light); the startup banner names
                              which default is armed.
                              light=true injects the supervisor with --light:
                              it drops the per-step contact / joint-limit /
                              grip trackers, which walk the whole scene graph
                              every basic step. Measured on a 298-node
                              10-Husky world under Newton (machine
                              9722d23d12a3): /sim/step 27.0 s -> 0.034 s per
                              step, a 10-step advance 120.0 s -> 0.19 s
                              (~790x / ~630x). ⚠ THOSE ARE PRE-3b952b61d
                              FIGURES (public issue #4 quoted them back at
                              us). Re-measured 2026-08-29 on the same world
                              (husky_fleet_arena, 309 nodes, CPU mj_step,
                              same machine): full /sim/step 1 = 573-606 ms vs
                              light 6-35 ms (~17x); 10 steps 2855-3187 ms vs
                              48-67 ms (~47x); the load itself 12.1 s vs
                              4.1 s. Smaller, still an order of magnitude,
                              so the advice stands. Every supervised
                              /world/load response now carries a `tracking`
                              block naming the mode and this cost.
                              What light mode actually costs
                              you: /sim/grips returns an empty list with
                              tracking.enabled=false, and the contact / grip /
                              joint-limit EVENT types stop being produced.
                              /sim/contacts is NOT affected -- it walks the
                              scene per call (observe.collect_contacts) and
                              never reads the tracker, so it answers exactly as
                              it does in full mode. The default WAS false
                              (backward compatible) until 2026-09-02; the
                              measured reason for the flip is the fleet-arena
                              row above (2026-08-29: 12.1 s vs 4.1 s to load, 17-47x per step; re-measured 2026-09-02 on the current engine at 5.2 s vs 4.65 s and 2.3x per
                              step) plus the cloth world (13.4 s vs 3.1 s to
                              reload) -- an agent that forgot the flag got a
                              harness slower than the run-headless it
                              replaces. OMNISIM_HARNESS_LIGHT is parsed as a
                              real boolean: unset or =1 -> light, =0 -> full.
    POST /world/sync          {"path"?: "...", "settle_steps"?: 1,
                               "reset_physics"?: true, "wait_s"?: 30.0}
                              Default edit-iteration path. Applies proven
                              root-DEF pose-only changes live in one batch;
                              automatically performs /world/load semantics for
                              every other edit.
    GET  /world/diagnostics
    POST /world/screenshot    {"path"?: str, "quality"?: int}  -> image/png or {path}
    GET  /world/render_stats  -> {mean_brightness, mean_rgb, max_rgb, saturated_pct, ...}
    GET  /scene/tree?bounds=1        (bounds=1 attaches world-space geometric
                                      bounds per node: center, radius, bbox)
    GET  /scene/node/{def}?bounds=1&probe=1
                                     (probe=1 additionally runs the SLOW exact
                                      bounding-sphere oracle, see /scene/frame)
    GET  /scene/viewpoint            -> the live camera: position, orientation,
                                        fieldOfView, near/far, follow, plus derived
                                        forward/up/right unit vectors and the
                                        horizontal+vertical FOV for the real aspect
    POST /scene/look_at       {"position": [x,y,z], "target": [x,y,z], "push"?: bool}
    POST /scene/frame         {"def"|"defs"|"target"+"radius", "mode"?, "margin"?,
                               "aspect"?, "push"?}
                                     -> aim AND distance in one call; returns the
                                        chosen pose plus a numeric verification that
                                        the subject is inside the frame
    POST /scene/orbit         {"azimuth_deg"?, "elevation_deg"?, "dolly"?, "pan"?,
                               "center"?|"def"?, "push"?}
                                     -> incremental nudge relative to the CURRENT view
    GET  /scene/visible?defs=A,B&all=1&limit=200
                                     -> what is in frame right now: frustum test,
                                        screen-space bbox/centroid in pixels, distance,
                                        angular offset + a human-readable hint
    POST /scene/spawn         {"vrml"|"type"+"fields"|"urdf"|"clone", "def"?,
                               "name"?, "translation"?, "rotation"?, "parent"?,
                               "index"?, "settle_steps"?}
                                     -> import a node into the LIVE scene
                                        (Field.importMFNodeFromString) instead of
                                        hand-writing .wbt text and paying a load;
                                        returns the new node's DEF/id/type/pose.
                                        "clone": "<DEF>" copies an existing node
                                        via the engine's own Node.exportString --
                                        the ONLY way to spawn a URDF robot,
                                        because URDFRobot is expanded by the file
                                        tokenizer and a string import never sees
                                        it (see /capabilities not_supported).
    POST /scene/delete        {"def"|"defs", "settle_steps"?}
                                     -> Node.remove, per-DEF result + a
                                        verification that the DEF no longer
                                        resolves
    POST /scene/set_pose      {"def", "translation"?, "rotation"?,
                               "reset_physics"?: true, "settle_steps"?: 1}
                                     -> move an existing node; reports the
                                        read-back pose and its delta from the
                                        request
    POST /sim/step            {"steps"?: int}
    POST /sim/reset           {"restore"?: "__init__"|<name>|null, "verify"?: true}
                                     -> rewinds the clock AND restores the
                                        engine's own parse-time state, i.e. the
                                        world as the .wbt authored it
                                        (simulationReset alone does not)
    POST /sim/snapshot        {"name"?: str}   -> named engine-side state
    POST /sim/restore         {"name"?: str, "settle_steps"?: 1}
                                     -> restore a named snapshot; the response
                                        carries how far the restore actually
                                        landed, not an assertion that it did
    GET  /sim/snapshots       -> the names taken in this world
    GET  /sim/state            -> harness session metadata PLUS the simulation
                                  clock: {sim_time_ms, basic_time_step_ms,
                                  sim_time_source}. The clock is best-effort and
                                  never blocks a load poll -- when it is null,
                                  sim_time_source says why.
    GET  /sim/contacts?wake=1&settle_steps=2
                               -> {contacts: [{a_def, b_def, point, paired}],
                                   tracking: {scope, solids_walked, completeness,
                                              empty_set_reasons[], inert_pinned_solids[],
                                              bodies_at_rest[]}}
                                  An EMPTY list is not proof of no contact, and this
                                  response NEVER claims to be complete. The reasons are
                                  enumerated in tracking.empty_set_reasons; the one that
                                  is MEASURED per Solid is a `physicsBackend "ode"` pin
                                  (no ODE exists, so the field yields no physics at all
                                  -- no gravity, no contact), reported in
                                  tracking.inert_pinned_solids. ⚠ There is NO body-sleep
                                  mechanism in this engine: `wake=1` is a documented
                                  no-op kept for compatibility. It used to write
                                  WorldInfo.physicsDisableTime (a field with no reader in
                                  the engine) and advance 2 steps, i.e. mutate the world
                                  during a read while measuring nothing.
    GET  /sim/grips            -> {grips: [{gripper_def, held_def, since_t_ms}]}
    GET  /sim/events?since=<sup_seq>&log_since=<log_seq>&limit=<int>&types=<csv>
                                 -> unified event stream (sim + controller log + world log)
                                    {events:[{seq, source:"sup"|"log", t_sim_ms?, t_wall?,
                                              type, ...}], next_since, next_log_since,
                                     dropped_sup, dropped_log}
                                    Event types: contact.began/ended, joint.limit_hit,
                                    grip.acquired/released, damage.impact,
                                    damage.state_transition, controller.log,
                                    world.warning, world.error
    GET  /robots               -> {robots: [{def, name, model, controller, type,
                                              position, orientation, num_joints}]}
    GET  /robot/{def}/joints   -> {robot, joints: [{name, type, position, velocity,
                                                     lower, upper, hit_limit}]}
    GET  /robot/{def}/devices  -> {robot, devices: [{name, type}]}
    GET  /robot/{def}/sensor/{name}  501 — supervisor cannot read live sensor data
                                from devices it does not own; use /joints for joint
                                positions or a per-robot helper controller for cameras
                                and lidars.
    GET  /robot/damage         -> {robot, attached, parts, damage:{part:{state,hp,...}}, ...}
    GET  /robot/damage/events?since=<int>&limit=<int>
                                 -> {events: [...], last_step_id, events_total}
                                    (back-compat filtered view onto /sim/events damage.*)
    POST /robot/damage/reset    heal all parts back to pristine without resetting the sim
    POST /robot/damage/inject  {"part": "wheel_fl", "state"?: "broken", "hp_delta"?: -50}
                                 test/debug hook to set a part's state directly
    GET  /healthz

When `with_supervisor` is true (default), the harness writes a sibling
`.harness_<name>.wbt` next to the user's world that appends a generic
Supervisor robot, then launches OmniSim on the sibling. The supervisor
controller (projects/default/controllers/harness_supervisor) opens a TCP
socket on 127.0.0.1:6790 that the harness uses for screenshots, scene
queries, step, and reset. The original world file is never touched and the
sibling is removed on next load or shutdown.

See scripts/harness/README.md and AGENTS.md section 5.
"""

from __future__ import annotations

import argparse
import collections
import functools
import inspect
import hashlib
import json
import math
import os
import re
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import textwrap
import threading
import time
import traceback
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

try:
    from PIL import Image
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False

sys.path.insert(0, str(Path(__file__).resolve().parent))
from diagnostic_codes import classify_text  # noqa: E402
import spatial  # noqa: E402  (camera framing math; loads omniworld.viewpoint)

REPO_ROOT = Path(__file__).resolve().parents[2]

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
from omnisim.paths import linux_runtime_env  # noqa: E402

DEFAULT_HOST = "127.0.0.1"

# Engine run mode (public issue #13). The harness pinned a literal --mode=fast
# (~13x real time on machine 9722d23d12a3) with no way to ask for real time --
# and sensor-driven ROS 2 stacks (Nav2 / slam_toolbox) need wall-clock-paced
# /clock and /scan: at 13x, the clock cadence the HTTP surface sustains lands
# ~8 s of sim time apart and slam_toolbox's scan<->TF alignment breaks (the
# first Nav2 bring-up patched this file by hand; measured in that campaign:
# fast ~13x, realtime ~0.6x). Resolution order: --engine-mode flag beats
# OMNISIM_HARNESS_ENGINE_MODE, which beats the "fast" default. An unknown
# value warns and falls back to fast rather than refusing to serve.
ENGINE_MODES = ("fast", "realtime")


def resolve_engine_mode(flag_value: str | None, env_value: str | None) -> tuple[str, str | None]:
    """(mode, warning-or-None) from the CLI flag and the env var."""
    for source, value in (("--engine-mode", flag_value),
                          ("OMNISIM_HARNESS_ENGINE_MODE", env_value)):
        if value is None or value == "":
            continue
        v = value.strip().lower()
        if v in ENGINE_MODES:
            return v, None
        return "fast", (f"{source}={value!r} is not one of {ENGINE_MODES}; "
                        f"running --mode=fast")
    return "fast", None


ENGINE_MODE = "fast"  # set once in main(); read where the engine command line is built
DEFAULT_PORT = 6789
# Supervised loads return as soon as the supervisor binds, so a generous
# default only costs time when something is genuinely slow or broken. A
# cold simulator start (CUDA + warp init) takes ~20s on a mid-range GPU
# laptop — the old 3s default missed it every time and bricked the
# session. Loads without a supervisor have no positive signal and always
# sleep the full window, so they keep a short default.
DEFAULT_LOAD_WAIT_S = 30.0
DEFAULT_LOAD_WAIT_BARE_S = 3.0
# `wait_s` bounds how long a single POST /world/load *blocks the caller* —
# it is NOT the supervisor bind window. The bind wait continues in a
# background thread after the request returns (see `_bind_waiter`): it keeps
# waiting as long as the engine process is alive and visibly making progress
# (engine log / stdout still growing), and only gives up at
# LOAD_BIND_HARD_CEILING_S or after LOAD_PROGRESS_STALL_S without progress.
# A slow cold load (warehouse_husky takes 46–79 s on WSL2) therefore binds
# even when the caller's wait_s was short — the caller sees
# "load_in_progress" and can poll /sim/state or re-POST the same path.
MAX_LOAD_WAIT_S = 300.0
# Absolute ceiling on the background supervisor-bind window, measured from
# engine launch. A truly wedged engine is terminated (cleanly) at this point
# so the harness stays recoverable.
LOAD_BIND_HARD_CEILING_S = 300.0
# After the caller's wait_s window has expired, the background waiter keeps
# going only while the engine shows signs of life. If neither the engine log
# nor its stdout/stderr grow for this long, the load is declared stalled and
# the engine is terminated + reported.
LOAD_PROGRESS_STALL_S = 30.0

SUPERVISOR_HOST = "127.0.0.1"
# Default supervisor IPC port. Each `HarnessState` carries its own
# `supervisor_port` attribute so that a second harness on a different
# `--port` can use a different supervisor port (otherwise two harnesses
# would race for the same TCP listener inside the OmniSim subprocess).
DEFAULT_SUPERVISOR_PORT = 6790
SUPERVISOR_PORT = DEFAULT_SUPERVISOR_PORT  # back-compat alias for callers/tests
SUPERVISOR_CONNECT_TIMEOUT_S = 30.0
# Per-RPC timeout on an ESTABLISHED supervisor connection. Generous on
# purpose: on slow platforms (WSL2 software GL + CPU physics) a live
# supervisor's main loop can take tens of seconds per iteration while the
# scene settles, and an RPC is only serviced once per iteration. A dead
# socket still fails fast (recv returns EOF immediately); this timeout only
# gates the live-but-busy case, where waiting is the correct behavior.
SUPERVISOR_RPC_TIMEOUT_S = 120.0
SUPERVISOR_POLL_INTERVAL_S = 0.1
# Ping timeout used while *probing* for a supervisor bind. Must be short: a
# TCP connect can succeed against the supervisor's listen backlog while the
# engine is still loading (the controller binds before its first sim step),
# and a ping on such a socket blocks until the step loop services it. The
# old behavior used the full 30 s RPC timeout here, which parked HTTP
# threads for 30 s apiece against a loading engine.
SUPERVISOR_BIND_PING_TIMEOUT_S = 3.0
# The background bind waiter escalates its ping patience up to this bound.
# On slow platforms (WSL2: software GL + CPU physics, sim at ~0.1x) the
# supervisor's main loop takes tens of seconds per iteration during early
# settling, so a ping is only answered after the current iteration
# completes — a fixed short timeout can NEVER bind there (measured via
# py-spy: accept() reached ~60 s in, first iterations 30–120 s each).
SUPERVISOR_BIND_PING_TIMEOUT_MAX_S = 90.0
# Delay before the second ping of the double-ping stability check (see
# `_try_connect_supervisor(stability_check=True)`).
SUPERVISOR_STABILITY_RECHECK_S = 0.4
HOT_RELOAD_WAIT_S = 15.0
# How long a hot reload waits for the OLD supervisor's listener to drop
# after the world_load RPC before polling for the new one. The old listener
# keeps accepting until the engine actually swaps worlds; reconnecting too
# early adopts the DYING controller (its ping still answers) and the first
# real RPC then fails with "supervisor closed the connection". Kept short:
# on a very fast swap the NEW listener can replace the old one between
# polls (port reads continuously "up"), and we don't want tiny-world hot
# reloads paying the full window — the double-ping stability check covers
# that residual race.
HOT_RELOAD_LISTENER_DOWN_WAIT_S = 3.0
# Poll granularity for the two hot-reload waits (old listener dropping, new
# supervisor appearing). Deliberately finer than SUPERVISOR_POLL_INTERVAL_S:
# those two phases are ~313 ms and ~503 ms of a ~912 ms reload and consist
# ENTIRELY of waiting, so the interval is quantisation error, not work. The
# broad 100 ms default stays where it is -- this only tightens the reload path.
HOT_RELOAD_POLL_INTERVAL_S = 0.02

# Supervisor commands that are safe to transparently retry once on a fresh
# connection after an RPC failure (e.g. the socket belonged to a supervisor
# torn down by a world swap). Mutating commands — step, reset, world_load,
# damage_* writes — are deliberately excluded: the harness cannot know
# whether the supervisor processed them before the socket died.
IDEMPOTENT_SUPERVISOR_COMMANDS = frozenset({
    "ping", "sim_state", "scene_tree", "scene_node", "robots_list",
    "robot_joints", "robot_devices", "sim_contacts", "sim_grips",
    "damage_state", "damage_events", "damage_geometry_stats",
    "events_drain", "screenshot", "set_viewpoint",
    "get_viewpoint", "scene_bounds",
    # Read-only additions. The mutation verbs (scene_spawn / scene_delete /
    # scene_set_pose / scene_set_poses / sim_snapshot / sim_restore) are deliberately absent:
    # the harness cannot know whether the supervisor applied them before the
    # socket died, and a silently retried spawn would duplicate a robot.
    "capabilities", "sim_snapshots",
    # solve_ik is a PURE PREVIEW (World.solve_ik owns its buffers and never
    # writes solver state; nothing in the scene moves), so a transparent
    # replay after a dead socket is safe — it just re-answers.
    "solve_ik",
    # particle_stats is a PURE READ off the engine's per-step particle caches
    # (the same _particle_q_cache the render readback shares; the GranularGroup
    # arm reads the host buffer the wgpu draw reads). Nothing in the scene
    # moves, so a transparent replay after a dead socket just re-answers.
    "particle_stats",
})


def is_retryable_supervisor_call(cmd: str, args: dict | None = None) -> bool:
    """May this RPC be transparently REPLAYED on a fresh connection?

    Membership of IDEMPOTENT_SUPERVISOR_COMMANDS is now sufficient. It was not,
    for exactly one reason: `sim_contacts` used to change category with its
    ARGUMENTS -- `wake` made the supervisor rewrite
    WorldInfo.physicsDisableTime and ADVANCE the world by `settle_steps`, so a
    reconnect replay double-advanced the sim during what the caller had
    documented as a read.

    `wake` is a no-op as of 2026-08-08 (there is no body sleep to clear, and the
    field it wrote has no reader in the engine), so `sim_contacts` is a pure read
    with or without it and the argument-sensitive carve-out is gone. The
    `args` parameter is kept: the signature is the extension point for the next
    verb that genuinely does change category with its arguments, and the
    carve-out is documented above rather than deleted so it is not re-derived
    from scratch.
    """
    if cmd not in IDEMPOTENT_SUPERVISOR_COMMANDS:
        return False
    return True


# The `name` (and `controller`) of the Robot the harness injects into every
# world it loads. It is DERIVED into both stanzas below and into the roster
# filters on /robots and /scene/tree, so the filter can never drift from the
# thing actually injected.
#
# ⚠ WHY THE FILTERS EXIST. This node is NOT in the user's .wbt -- the harness
# put it there -- but every scene enumeration counted it, so an agent asserting
# "exactly 10 robots" on a 10-robot world read 11 (measured in 2 of 3 cells,
# 2026-08-12; it showed up as `'#939' | 'harness_supervisor'`, the `#939` being
# observe.list_robots' node-id fallback for a node with no DEF).
HARNESS_SUPERVISOR_NAME = "harness_supervisor"

SUPERVISOR_INJECT_STANZA = f"""
Robot {{
  name "{HARNESS_SUPERVISOR_NAME}"
  controller "{HARNESS_SUPERVISOR_NAME}"
  supervisor TRUE
  synchronization FALSE
}}
"""

# Same stanza, but passing --light so the injected supervisor skips the
# contact / joint-limit / grip trackers and sub-samples damage FX (the
# controller has honoured --light since P6; until now the harness passed no
# controllerArgs at all, so the flag was unreachable over HTTP).
#
# Why this exists: the trackers walk the whole scene graph every basic step,
# so /sim/step cost grows with node count -- measured at seconds per 16 ms
# step on a multi-robot world, which no agent verification loop can afford.
# Light mode trades /sim/grips (empty, tracking.enabled=false) and the
# contact.* / grip.* / joint.limit_hit events for a usable step; /sim/contacts
# is walked per call and unaffected. It is the DEFAULT since 2026-09-02 (see
# LIGHT_DEFAULT_* below). See docs/developer/agent-native-api.md G1.
SUPERVISOR_INJECT_STANZA_LIGHT = f"""
Robot {{
  name "{HARNESS_SUPERVISOR_NAME}"
  controller "{HARNESS_SUPERVISOR_NAME}"
  controllerArgs [ "--light" ]
  supervisor TRUE
  synchronization FALSE
}}
"""


# ---------------------------------------------------------------------------
# What /sim/reset actually does to actuation (the disclosure it owes callers)
# ---------------------------------------------------------------------------
#
# ⚠ THE HIGHEST-SEVERITY THING THIS HARNESS CANNOT FIX FROM PYTHON.
# MEASURED 2026-08-12 (2 of 3 agent cells): fresh harness, single load,
# supervisor connected. `/sim/reset` reported "authored poses restored"; 1250
# subsequent `/sim/step`s advanced 20.0 s of sim time at normal per-step cost;
# all 10 robots read 0.00 m net displacement AND 0.00 m path. A second cell
# read its wheel joints frozen at 980.14 rad. The same world drives
# 57.9-89.3 m/robot under `run-headless`, so the WORLD is fine.
#
# ROOT CAUSE, read from the engine source (not inferred from the symptom):
#   src/omnisim/engine/OmSimulationWorld.cpp  OmSimulationWorld::reset()
#       -> root()->reset("__init__")            walks the WHOLE scene
#   src/omnisim/nodes/OmRobot.cpp:257           OmRobot::reset()
#   src/omnisim/nodes/OmMotor.cpp:796-809       OmMotor::reset()
#         mUserControl   = false;               drops force mode
#         mRawInput      = 0.0;
#         mTargetPosition = j ? position() : 0.0;   <-- the killer
# A wheel running in VELOCITY mode was set up with `setPosition(inf)`, so its
# target is infinity and the PD servo drives forever. The reset re-pins that
# target to the joint's CURRENT position -- which is precisely the 980.14 rad
# the second cell read back -- turning every velocity-mode motor into a
# position hold at wherever it happened to be.
#
# And nothing re-issues the command: the supervisor path passes
# `restartControllers = false` (OmSupervisorUtilities.cpp:610 ->
# OmSimulationWorld.cpp:355), so a controller that called
# setPosition/setVelocity once at startup and then only loops `robot.step()`
# never sends another one. The clock keeps advancing at full speed and nothing
# moves, for the rest of the session.
#
# The fix belongs in the engine (or in a supervisor verb that re-arms motors);
# src/ is not this lane's to change. What the harness MUST do is stop letting
# an agent read a bare success and conclude the physics is broken.
RESET_ACTUATION_WARNING = (
    "THE RESET RE-PINNED EVERY MOTOR IN THE SCENE and restarted no controller, "
    "so a robot driven by a start-up setPosition(inf)/setVelocity() will now "
    "hold position and never move again, while the clock keeps advancing "
    "normally. See `actuation` for the mechanism and the two workarounds."
)

RESET_ACTUATION_DISCLOSURE = {
    "motors_retargeted": True,
    "controllers_restarted": False,
    "mechanism": (
        "the engine's reset walks the whole scene (OmSimulationWorld::reset -> "
        "root()->reset(\"__init__\")) and OmMotor::reset "
        "(src/omnisim/nodes/OmMotor.cpp:796-809) clears mUserControl and re-pins "
        "mTargetPosition to the joint's CURRENT position. A wheel running in "
        "velocity mode was set up with setPosition(inf), so its target goes from "
        "infinity to a finite angle and the motor becomes a POSITION HOLD where it "
        "stopped. The supervisor path passes restartControllers=false "
        "(OmSupervisorUtilities.cpp:610), so no controller restarts to re-issue "
        "the command."),
    "measured": (
        "2026-08-12: 10 robots, 1250 steps after the reset, 20.0 s of sim time at "
        "normal per-step cost, 0.00 m net and 0.00 m path each; a second cell read "
        "its wheel joints frozen at 980.14 rad -- exactly the re-pinned target. The "
        "same world drives 57.9-89.3 m/robot under run-headless."),
    "workarounds": [
        "have the controller re-issue its motor commands after a reset "
        "(setPosition(inf) + setVelocity(v) for a velocity-mode wheel)",
        "POST /world/load the same world instead of resetting -- a load starts "
        "fresh controllers, which re-issue their start-up commands",
    ],
    "scope": (
        "applies to any motor whose command was issued once and not repeated; a "
        "controller that writes its motor targets every tick is unaffected."),
}


# ---------------------------------------------------------------------------
# The tracking-mode DEFAULT for POST /world/load (light since 2026-09-02)
# ---------------------------------------------------------------------------
#
# The injected supervisor's per-step contact / joint-limit / grip trackers walk
# the whole scene graph every basic step. With them on, the harness is SLOWER
# than the run-headless it exists to replace -- measured on machine
# 9722d23d12a3: fleet arena (309 nodes, CPU mj_step, 2026-08-29) loads in
# 12.1 s full vs 4.1 s light, /sim/step 1 costs 573-606 ms vs 6-35 ms (~17x),
# 10 steps 2855-3187 ms vs 48-67 ms (~47x); the 10-node cloth world reloads in
# 13.4 s full vs 3.1 s light (2026-08-14). An agent that forgot `light: true`
# got the slow path, which is the wrong default for the primary audience. So
# since 2026-09-02 a /world/load that names NEITHER `light` NOR `tracking`
# runs light, and says so (`tracking.default_applied: true`). The contract is
# dual: an explicit `light` (either value) or any `tracking` object always
# wins, and OMNISIM_HARNESS_LIGHT=0 restores full tracking as the
# process-wide default (unset or =1 -> light). Nothing else changed: what
# light mode drops (/sim/grips, the contact.* / grip.* / joint.limit_hit
# events) and what it keeps (/sim/contacts) is unchanged, and the first
# tracker-fed read in a light session emits one world.warning naming the
# mode it ran in and how to get the tracker back.
LIGHT_DEFAULT_ENV = "OMNISIM_HARNESS_LIGHT"
LIGHT_DEFAULT_SINCE = "2026-09-02"
LIGHT_DEFAULT_WHY = (
    "with the per-step trackers on, the harness is slower than re-running "
    "run-headless: fleet arena (309 nodes, CPU mj_step, 2026-08-29) 12.1 s "
    "to load vs 4.1 s light, /sim/step 1 573-606 ms vs 6-35 ms (~17x), "
    "10 steps 2855-3187 ms vs 48-67 ms (~47x); cloth world reload 13.4 s vs "
    "3.1 s light")
LIGHT_DEFAULT_REVERT = (
    "pass {\"light\": false} on the request for all three trackers, or "
    "{\"tracking\": {\"contacts\": bool, \"joint_limits\": bool, \"grips\": bool}} "
    "for exactly the ones you need; OMNISIM_HARNESS_LIGHT=0 makes full "
    "tracking the process-wide default again (unset or =1 -> light)")
# world.warning code for the first tracker-fed read in a session whose
# trackers are not running (GET /sim/grips answering with tracking.enabled=false).
LIGHT_MODE_READ_CODE = "TRACKER_NOT_RUNNING"


def resolve_light_default() -> tuple[bool, str]:
    """(default, source) for the tracking mode a /world/load runs in when the
    request names neither `light` nor `tracking`.

    Unset -> light (the built-in default since 2026-09-02); =1/true -> light;
    =0/false -> full. Value-parsed via env_flag, so `=0` means OFF.
    """
    raw = os.environ.get(LIGHT_DEFAULT_ENV)
    if raw is None:
        return True, "built-in"
    return env_flag(LIGHT_DEFAULT_ENV, default=True), f"{LIGHT_DEFAULT_ENV}={raw.strip()}"


def tracking_default_block(default_light: bool, source: str) -> dict:
    """The one description of the tracking default, served on the load
    response (when it was applied) and on GET /capabilities -> limits, so
    the two cannot drift."""
    return {
        "light": bool(default_light),
        "mode": "light" if default_light else "full",
        "source": source,
        "since": LIGHT_DEFAULT_SINCE,
        "why": LIGHT_DEFAULT_WHY,
        "revert": LIGHT_DEFAULT_REVERT,
        "explicit_wins": ("an explicit `light` (either value) or any `tracking` object "
                          "on the request always overrides this default"),
    }


# ---------------------------------------------------------------------------
# Runtime scene mutation vs the frozen solver (POST /scene/spawn, /scene/delete)
# ---------------------------------------------------------------------------
#
# ⚠ MEASURED 2026-08-17 (internal parity plan, item W1.7, §5 q1). The Newton/MuJoCo
# model is FROZEN at finalizeWorld(): OmNewtonBackend.cpp:2384 sets
# openForBuild=false, every addBody/addShape* verb guards on it, and there is
# no remove path either. The harness loads and finalizes a world before it
# serves any scene verb, so EVERY mid-session spawn/delete hits the frozen
# model: a spawned dynamic body never falls (a 0.2 m box released at z=1.5
# read z=1.5 unchanged after 2200 steps / ~87 s sim time, while its authored
# twin settled at 0.599892), a spawned static body never collides, and a
# deleted node's geometry stays in the solver as a phantom (a deleted floor
# still holds bodies up, a deleted wall still blocks robots and rays).
# Engine-side the failure is SILENT -- 0 errors, 0 warnings, and the spawn
# response reads `verification.node_resolved: true` -- so until the engine fix
# lands (weeks, not hours), the harness's job is to make the lie loud: every
# successful spawn/delete response carries `physics_warning`, and the FIRST
# use of each verb per world-load emits one world.warning into the event
# stream (per-load, not per-request, so a loop of spawns cannot flood the
# ring; see HarnessState.runtime_mutation_warning).
RUNTIME_MUTATION_CODE = "RUNTIME_MUTATION_NOT_IN_SOLVER"

SPAWN_PHYSICS_WARNING = {
    "code": RUNTIME_MUTATION_CODE,
    "message": (
        "the Newton/MuJoCo model is frozen at world finalize, so this spawned "
        "node renders and appears in /scene/tree but has NO physics: a dynamic "
        "body will not fall, a static body will not collide, and rays pass "
        "through it regardless (measured 2026-08-17; engine fix tracked as "
        "internal parity plan, item W1.7). Reload the world (POST /world/load or "
        "POST /world/sync) to give it physics."),
}

DELETE_PHYSICS_WARNING = {
    "code": RUNTIME_MUTATION_CODE,
    "message": (
        "the Newton/MuJoCo model is frozen at world finalize, so the deleted "
        "node(s) are gone from the scene graph and the render but their "
        "colliders remain in the solver as PHANTOMS: a deleted wall still "
        "blocks robots and rays, and a deleted floor still holds bodies up, "
        "silently (measured 2026-08-17; engine fix tracked as "
        "internal parity plan, item W1.7). Reload the world (POST /world/load) to "
        "purge them."),
}


def is_harness_injected(entry: dict) -> bool:
    """True when a robot/scene-tree entry is the supervisor the HARNESS injected.

    Matched on `name` OR `controller`, both of which the stanza above sets:
    `name` is what the engine reports for the node, `controller` survives even
    if a world happens to rename the node. Never matched on the DEF, which the
    stanza does not set (hence the `#<id>` fallback an agent sees).
    """
    if not isinstance(entry, dict):
        return False
    return (entry.get("name") == HARNESS_SUPERVISOR_NAME
            or entry.get("controller") == HARNESS_SUPERVISOR_NAME)


# ---------------------------------------------------------------------------
# Capability discovery (GET /capabilities)
# ---------------------------------------------------------------------------

# Wire-protocol version this harness speaks (PROTOCOL.md).
OMNISIM_WIRE_VERSION = "1.1"

# The route table, published verbatim by /capabilities. It is DECLARED here
# rather than derived, because a route's meaning is not in its dispatch line —
# but it is also CROSS-CHECKED against the handler source at request time
# (`verify_routes`), so a route added to do_GET/do_POST and not declared here
# shows up in the response as `undeclared` instead of being invisible. That
# check is the whole reason this table is trustworthy: the last hand-maintained
# copy of it (PROTOCOL.md §7) was missing eight endpoints.
ROUTES: tuple[dict, ...] = (
    {"method": "GET", "path": "/capabilities", "summary": "This document: backend, limits, event types, what is not supported.",
     "params": ["probe_step"]},
    {"method": "GET", "path": "/healthz", "summary": "Liveness; never touches the simulator."},
    {"method": "GET", "path": "/debug/read_bench",
     "summary": "Diagnostic: measured cost of one supervisor read on this session, free-running vs paused.",
     "params": ["n"]},
    {"method": "GET", "path": "/world/diagnostics", "summary": "Structured diagnostics from the current load."},
    {"method": "GET", "path": "/world/render_stats", "summary": "Exposure statistics of a fresh render (0-255 scale)."},
    {"method": "GET", "path": "/scene/tree", "summary": "Flat node list; ?bounds=1 attaches world-space bounds.",
     "params": ["bounds"]},
    {"method": "GET", "path": "/scene/node/<def>", "summary": "Field dump + contacts for one node.",
     "params": ["bounds", "probe"]},
    {"method": "GET", "path": "/scene/node/<def>/particles",
     "summary": "Particle stats for one Cloth/SoftBody/GranularBed/GranularGroup node: count, "
                "world-frame min/max/centroid over the FINITE particles, and non_finite (a "
                "diverging cloth reads as a rising non_finite, never a NaN centroid). PURE READ "
                "off the engine's per-step particle cache; ?sample=N adds every N-th particle's "
                "xyz.",
     "params": ["sample"]},
    {"method": "GET", "path": "/scene/viewpoint", "summary": "Read the live camera back, with resolved per-axis FOV."},
    {"method": "GET", "path": "/scene/visible", "summary": "What is on screen now: frustum test, pixel bbox, angular offset.",
     "params": ["defs", "all", "limit"]},
    {"method": "GET", "path": "/sim/state", "summary": "Harness session metadata (not scene state)."},
    {"method": "GET", "path": "/sim/contacts",
     "summary": "Global contact set + `tracking` scope. Walked per call, so it answers in light "
                "mode too. An EMPTY list is never proof of no contact and the response never "
                "claims completeness: tracking.empty_set_reasons enumerates the real causes "
                "(a Solid pinned physicsBackend \"ode\" has no physics at all and is listed, "
                "measured, in tracking.inert_pinned_solids; a Newton runtime that never came "
                "up; a Solid the backend never registered). There is NO body-sleep mechanism "
                "in this engine, and ?wake=1 is a documented no-op kept for compatibility.",
     "params": ["wake", "settle_steps"]},
    {"method": "GET", "path": "/sim/grips",
     "summary": "Inferred grips + `tracking` scope. In light mode the tracker does not exist, "
                "so the list is empty with tracking.enabled=false (NOT measured, as distinct "
                "from nothing gripped)."},
    {"method": "GET", "path": "/sim/events", "summary": "Cursor-paged unified event stream with drop counters.",
     "params": ["since", "log_since", "limit", "types"]},
    {"method": "GET", "path": "/sim/snapshots", "summary": "Named state snapshots taken in this world."},
    {"method": "GET", "path": "/robots",
     "summary": "Every Robot in the scene with pose and joint count. The harness's own injected "
                "supervisor is EXCLUDED (it is not in your .wbt) and named in `harness_injected`; "
                "?include_harness=1 lists it, flagged.",
     "params": ["include_harness"]},
    {"method": "GET", "path": "/robot/<def>/joints", "summary": "Per-joint position/velocity/limits + hit_limit."},
    {"method": "GET", "path": "/robot/<def>/devices", "summary": "Device inventory of a robot's subtree."},
    {"method": "GET", "path": "/robot/<def>/sensor/<name>", "summary": "501 by design; see not_supported."},
    {"method": "GET", "path": "/robot/damage", "summary": "Damage state of the tracked robot."},
    {"method": "GET", "path": "/robot/damage/events", "summary": "Filtered view of damage.* events.",
     "params": ["since", "limit"]},
    {"method": "POST", "path": "/world/load",
     "summary": "Load or hot-reload a world (.omniworld or legacy .wbt); structured diagnostics. "
                "Runs LIGHT by default since 2026-09-02 when neither `light` nor `tracking` is "
                "named (tracking.default_applied says so; OMNISIM_HARNESS_LIGHT=0 restores full).",
     "body": ["path", "wait_s", "with_supervisor", "light", "tracking"]},
    {"method": "POST", "path": "/world/sync",
     "summary": "Default agent edit loop: live-apply proven root pose-only changes; safely reload everything else.",
     "body": ["path", "settle_steps", "reset_physics", "wait_s", "light"]},
    {"method": "POST", "path": "/world/screenshot",
     "summary": "Render a PNG (body or server-side path). NEVER answers 200 without a picture: "
                "empty or non-PNG bytes are a 502 SCREENSHOT_EMPTY / SCREENSHOT_NOT_PNG.",
     "body": ["path", "quality"]},
    {"method": "POST", "path": "/scene/look_at", "summary": "Aim the camera from a position at a target.",
     "body": ["position", "target", "push"]},
    {"method": "POST", "path": "/scene/frame", "summary": "Aim AND distance from real bounds, with numeric verification.",
     "body": ["def", "defs", "target", "radius", "mode", "margin", "aspect", "push"]},
    {"method": "POST", "path": "/scene/orbit", "summary": "Relative camera nudge around the current view.",
     "body": ["azimuth_deg", "elevation_deg", "dolly", "pan", "center", "def", "push"]},
    {"method": "POST", "path": "/scene/spawn",
     "summary": "Import a node into the live scene from VRML, a type+fields spec, or a clone of an "
                "existing DEF. ⛔ BY DEFAULT A SCENE-GRAPH VERB, NOT A PHYSICS VERB: the Newton/MuJoCo "
                "model is frozen at world finalize, so without opting in the spawned node has NO "
                "physics (dynamic never falls, static never collides; measured 2026-08-17) and the "
                "response carries `physics_warning` RUNTIME_MUTATION_NOT_IN_SOLVER saying so. Pass "
                "{\"physics\": \"rebuild\"} (or POST /sim/rebuild_physics afterwards) and the node "
                "IS simulated: the Newton world is rebuilt at the scene's current poses, 97-267 ms "
                "measured, refused on cloth/soft/granular worlds, engaged welds dropped. See "
                "not_supported scene.runtime_mutation_physics.",
     "body": ["vrml", "type", "fields", "urdf", "clone", "def", "name",
              "translation", "rotation", "parent", "index", "settle_steps",
              "reset_physics", "physics"]},
    {"method": "POST", "path": "/scene/delete",
     "summary": "Remove nodes by DEF. ⛔ BY DEFAULT the frozen solver model KEEPS the deleted "
                "colliders as phantoms (a deleted wall still blocks rays and robots, a deleted "
                "floor still holds bodies up; measured 2026-08-17) and the response carries "
                "`physics_warning` RUNTIME_MUTATION_NOT_IN_SOLVER. Pass {\"physics\": \"rebuild\"} "
                "(or POST /sim/rebuild_physics afterwards) and the deleted geometry genuinely stops "
                "colliding -- same 97-267 ms rebuild and the same caveats as /scene/spawn; see "
                "not_supported scene.runtime_mutation_physics.",
     "body": ["def", "defs", "settle_steps", "physics"]},
    {"method": "POST", "path": "/scene/set_pose", "summary": "Move an existing node by DEF.",
     "body": ["def", "translation", "rotation", "reset_physics", "settle_steps"]},
    {"method": "POST", "path": "/sim/rebuild_physics",
     "summary": "W1.7: rebuild the Newton world at the scene's CURRENT poses -- runtime-spawned "
                "nodes gain physics, deleted ones lose it. Refused (409 REBUILD_REFUSED) on "
                "cloth/soft/granular worlds. The rebuild itself measures 97-267 ms (machine "
                "9722d23d12a3, CPU mj_step); the HTTP round trip adds the settle steps. Engaged "
                "Connector/VacuumGripper welds "
                "are DROPPED (re-lock from the controller). Also available as "
                "{\"physics\": \"rebuild\"} on /scene/spawn and /scene/delete.",
     "body": ["settle_steps"]},
    {"method": "POST", "path": "/sim/step", "summary": "Advance N basic timesteps.", "body": ["steps"]},
    {"method": "POST", "path": "/sim/reset",
     "summary": "Rewind the clock and restore the authored state. ⚠ ALSO RE-PINS EVERY MOTOR and "
                "restarts no controller, so a robot commanded once at start-up stops moving for "
                "good -- the response's `actuation` block carries the mechanism and the workarounds.",
     "body": ["restore", "verify", "settle_steps"]},
    {"method": "POST", "path": "/sim/snapshot", "summary": "Save a named engine-side state snapshot.",
     "body": ["name"]},
    {"method": "POST", "path": "/sim/restore", "summary": "Restore a named snapshot; reports how far it landed.",
     "body": ["name", "settle_steps"]},
    {"method": "POST", "path": "/robot/<def>/joints/set",
     "summary": "Supervisor-driven joint position targets, settle-and-verify. NOT a teleport: "
                "under Newton the write re-pins each motor's PD setpoint, so the joint CONVERGES "
                "over the settled steps and every joint reports measured commanded/achieved/error, "
                "never the argument echoed back. Targets beyond the joint's hard stops are clamped "
                "and flagged `clamped`; a motor with no position limits is built as a velocity "
                "wheel (ke=0) whose position targets the physics silently ignores — reported per "
                "joint as position_controllable:false with the mechanism, never bare success. "
                "Joint names are the same ones GET /robot/<def>/joints reports. Works in light "
                "and heavy supervisor mode.",
     "body": ["joints", "names", "positions", "settle_steps"]},
    {"method": "POST", "path": "/robot/<def>/ik",
     "summary": "Batched inverse-kinematics PREVIEW against the live Newton model (W2.1). PURE "
                "READ: nothing moves. Give an 'effector' (DEF of the end-effector Solid) and "
                "'targets' ([[x,y,z], ...], world frame; optional 'rotations' [[qx,qy,qz,qw], ...] "
                "and 'tool_offset' [x,y,z] in the effector's frame); the engine solves every "
                "Hinge/Slider joint of that robot registered with the physics backend and returns "
                "per-target angles keyed by the same joint names GET /robot/<def>/joints reports, "
                "clamped to the authored limits, plus residual_m per target in METRES measured by "
                "forward kinematics on exactly the returned angles — reject a target on its "
                "residual instead of driving to it. Apply via POST /robot/<def>/joints/set. "
                "⚠ The FIRST solve per world compiles a warp kernel (~8 s measured cold, ~150 ms "
                "warm; solve_ms in the response is the measured cost). Verified on the default CPU "
                "'mujoco' solver only; unverified on mujoco_warp.",
     "body": ["effector", "targets", "rotations", "tool_offset", "iterations"]},
    {"method": "POST", "path": "/robot/damage/reset", "summary": "Heal all parts without resetting the sim."},
    {"method": "POST", "path": "/robot/damage/inject", "summary": "Set a part's damage state directly.",
     "body": ["part", "state", "hp_delta"]},
)

# Event types produced by the HARNESS process (the supervisor's seven come
# from event_bus.SUPERVISOR_EVENT_TYPES and are fetched over the RPC).
# Declared next to LogRingBuffer, which emits them, and cross-checked the same
# way as the supervisor's — see `verify_log_event_types`.
LOG_EVENT_TYPES = ("controller.log", "world.warning", "world.error")


# ---------------------------------------------------------------------------
# ENGINE gaps published by /capabilities.not_supported
# ---------------------------------------------------------------------------
#
# ⚠ WHY THIS EXISTS. `not_supported` was a 10-entry literal inside the
# capabilities builder and every entry described a HARNESS ENDPOINT gap
# (`sim.pause`, `world.save`, `worlds.list`...). Nothing in it described the
# ENGINE -- so the one class of gap an agent cannot discover by any other means
# was the one class the discovery endpoint did not serve. An unimplemented HTTP
# verb announces itself: you call it and get a 404/501. An unimplemented
# ACTUATOR does not: you command it, the call succeeds, the sensor reads 0, and
# the honest reading ("this feature does not exist") is indistinguishable from
# the wrong one ("my gains are off") without reading C++.
#
# Every `reason` below is taken from the comment at the emit/UNIMPLEMENTED site
# named in `source`; nothing here is inferred. `diagnostic` names the code that
# fires on the load when the engine says something at all -- and where it is
# null, that is the point: the failure is SILENT and this entry is the only
# warning an agent will get.
#
# Almost all of these are consequences of ODE's deletion (src/ode, bdc02139) with
# no Newton-native replacement yet. They are gaps, not regressions to hide.
ENGINE_NOT_SUPPORTED: list[dict] = [
    # ⚠ THE TWO JOINT ENTRIES BELOW DESCRIBE FEATURES THAT NOW WORK. They used
    # to read "UNIMPLEMENTED ... gated OFF by default ... does not move at all",
    # which was true against the vendored newton 1.2.0 and false from
    # 2026-08-17 on (the gate is DEFAULT ON, OmBasicJoint::newtonBallHinge2Enabled).
    # They are kept, keys unchanged, because (a) the BallJoint has a REAL
    # residual gap (per-axis stops) that belongs in this table, (b) the revert
    # hatch disables both joint types at once and an agent must be able to find
    # that, and (c) "the entry vanished" is indistinguishable from "nobody
    # looked". Each carries a `status` field so a reader scanning feature names
    # is not told a working actuator is unsupported.
    {"feature": "joint.motorised_balljoint_actuation",
     "scope": "engine",
     "status": "WORKS since 2026-08-17 (actuation measured 2026-09-01); residual gap: per-axis stops",
     "reason": "MOTORISED BallJoint ACTUATION WORKS. The Newton d6 path (registerNewtonMultiDof / "
               "pushNewtonMultiDofTargets, reached from OmBasicJoint's per-tick push) is DEFAULT ON "
               "since 2026-08-17 behind OMNISIM_NEWTON_BALL_HINGE2 (value-parsed). It was default OFF "
               "while the vendored runtime was newton 1.2.0, whose d6 -> MuJoCo actuator mapping did "
               "not drive multi-DoF position targets; newton 1.5.0 (b56be84a0) does. Measured "
               "2026-09-01 with a corrected probe (OmniBench lane 4 `joint.ball_motor`): commanded "
               "0.8 rad about an axis WITH a lever arm, the arm displaced 0.1884 m against an analytic "
               "0.1947 -- PASS. The earlier `broken` verdict commanded the one axis that passes "
               "through the arm's own origin, which no actuator can displace, so it never measured "
               "actuation at all. WHAT IS STILL NOT SUPPORTED: the BALL element is emitted "
               "`limited: False`, so a BallJointParameters' per-axis minStop/maxStop are NOT "
               "enforced by the solver and the joint swings past its authored stops; the engine "
               "warns per motorised ball at registration.",
     "symptom": "none for actuation. For the residual gap: a driven or passive ball joint travels "
                "past its authored per-axis stops with no error; its PositionSensor readback is live "
                "and reports the overshoot honestly.",
     "source": "src/omnisim/nodes/OmBasicJoint.cpp (newtonBallHinge2Enabled), "
               "src/omnisim/nodes/OmBallJoint.cpp (prePhysicsStep, applyToOdeMinAndMaxStop)",
     "diagnostic": "JOINT_REGISTRATION_FAILED when the d6 could not be registered; the unmapped "
                   "stops are warned per joint at registration (the joint-feature diagnostic named "
                   "under joint.fields below); an overshoot itself is SILENT",
     "workaround": "for hard stops, model the DOF as a chain of HingeJoints (hinge limits ARE "
                   "enforced) or clamp targets in the controller; verify motion from "
                   "GET /robot/<def>/joints across two reads rather than trusting the command. "
                   "OMNISIM_NEWTON_BALL_HINGE2=0 is the exact-revert hatch back to a PASSIVE "
                   "constraint (no actuation, no angle readback) for an A/B -- it disables "
                   "Hinge2Joint too, so a joint that will not move is that hatch before it is the "
                   "solver."},
    {"feature": "joint.motorised_hinge2joint_actuation",
     "scope": "engine",
     "status": "WORKS since 2026-08-17 (measured 2026-08-17); no residual gap",
     "reason": "MOTORISED Hinge2Joint ACTUATION WORKS -- same d6 path and the same DEFAULT ON gate as "
               "the BallJoint entry above. Measured 2026-08-17 (machine 9722d23d12a3, binary "
               "13906cc6f12451eb, CPU mj_step, gravity 0 so the motor is the only thing that can move "
               "the arm): OmniBench lane 4 `joint.hinge2_motor` went broken -> works, tracking its "
               "commanded 0.8 rad exactly and carrying the arm 0.1951 m; tests/test_newton_ball_hinge2.py's "
               "hinge2 arm went from xfail to XPASS (both axes inside 0.05 rad of their commands, axis 1 "
               "does not drift when only axis 2 is re-commanded). Hinge2 axes ARE limited -- the d6 "
               "carries per-axis limits -- so unlike the ball there is no stops gap. Declaring the "
               "motors' minPosition/maxPosition (or the joint's minStop/maxStop) is what took the "
               "probe to `works`: a motor with no limits is built as a velocity wheel (ke = 0), see "
               "joint.fields below.",
     "symptom": "none. Passive Hinge2 constraints (free-spinning casters) were never affected.",
     "source": "src/omnisim/nodes/OmBasicJoint.cpp (newtonBallHinge2Enabled), "
               "src/omnisim/nodes/OmHinge2Joint.cpp (prePhysicsStep)",
     "diagnostic": "JOINT_REGISTRATION_FAILED when the d6 could not be registered; otherwise none",
     "workaround": "none needed. Kept so the revert hatch (OMNISIM_NEWTON_BALL_HINGE2=0 disables BOTH "
                   "joint types) and the history stay discoverable: if a Hinge2 does not move, check "
                   "that variable and the motor's limits before suspecting the solver."},
    {"feature": "sensor.touchsensor_force / force-3d (authored WITHOUT a Physics node)",
     "scope": "engine",
     "status": "WORKS when authored correctly (measured 2026-08-15); two silent zeros remain",
     "reason": "the force source is the Newton mount-wrench readback, which can only answer for a "
               "sensor that was UN-FOLDED into a Newton body of its own; the ODE mount-joint feedback "
               "it used to fall back to has been removed, so when the native readback cannot answer "
               "there is NO force source at all. Since ee069b326 (2026-08-13) a BUMPER un-folds on its "
               "boundingObject alone, but \"force\" / \"force-3d\" still require a Physics node -- "
               "deliberately, because their value is the MOUNT WRENCH, which is undefined for a "
               "sensor that is not an inertial body in its own right. Re-measured 2026-08-15 (machine "
               "9722d23d12a3, CPU mj_step): bumper 1.0 on 10/10 resting samples and 0.0 in free "
               "flight; force 981.00004 N against an analytic 981.",
     "symptom": "a force sensor authored without Physics reads 0.0 for ever -- that 0 is \"not "
                "measured\", never \"no force\". Two more SILENT ZEROS survive in a WORKING force sensor "
                "and are authoring traps, not engine defects: (1) the value is the mount wrench "
                "projected onto the sensor's own +X axis, so a sensor authored without a rotation "
                "(+X horizontal) reads ~8e-16 N under a vertical load; (2) TouchSensor.wrl defaults "
                "lookupTable to [0 0 0, 5000 50000 0], a 10x gain, so a 19.62 N load reads 196.2.",
     "source": "src/omnisim/nodes/OmTouchSensor.cpp (computeValue), "
               "src/omnisim/nodes/OmSolid.cpp (isUnfoldedTouchSensor)",
     "diagnostic": "SENSOR_NO_SOURCE for the never-un-folded case; the aim and lookupTable traps are "
                   "SILENT",
     "workaround": "give a force sensor a Physics node; aim +X into the load (rotation 0 1 0 1.5708 "
                   "points it down under ENU); declare `lookupTable [ ]` for raw newtons; and check "
                   "the load for SENSOR_NO_SOURCE before believing any reading. Proving a grasp "
                   "geometrically (the part is airborne and tracks the gripper) remains the stronger "
                   "claim; GET /sim/contacts is the contact read that never depends on this device."},
    {"feature": "physics.backend_ode (physicsBackend \"ode\", defaultPhysicsBackend \"ode\")",
     "scope": "engine",
     "reason": "src/ode was DELETED (bdc02139). The field still parses, and what it now selects is "
               "OmOdeBackend's inert dispatcher stub -- every verb returns -1 -- so the Solid gets "
               "NO gravity and NO contact while the world loads clean. An explicit "
               "physicsBackend \"ode\" is no longer an opt-out of Newton; it is an opt-out of "
               "physics. Since 2026-08-08 the engine WARNS once per such Solid (\"asks for "
               "physicsBackend 'ode' ... will have NO gravity and NO contact\"); an older log "
               "will not have that line.",
     "symptom": "the body never falls, never collides, and reports zero contact points for ever, "
                "with boundingObject and physics both present. The world loads with no error.",
     "source": "src/omnisim/nodes/OmSolid.cpp (flushPendingNewtonRegistrations)",
     "diagnostic": "SOLID_ODE_PIN_INERT (one per pinned Solid that declares collision or mass; "
                   "coalesced by this harness) -- and NEWTON_ZERO_DYNAMIC_BODIES when it was the "
                   "world's only dynamic body",
     "workaround": "delete the field (the \"auto\" default resolves to Newton; write \"newton\" to be "
                   "explicit; never write \"ode\" into a new world). GET /scene/node/<def> -> "
                   "fields.physics_backend.inert tells you per node, and GET /sim/contacts -> "
                   "tracking.inert_pinned_solids lists them all."},
    {"feature": "sound.contact",
     "reason": "CONTACT SOUND IS SILENT ON THE NEWTON BACKEND, and has been since Newton became the "
               "default. Its producer was the ODE near-callback's contact list, keyed on ODE geom ids; "
               "that whole subsystem was deleted with the ODE layer on 2026-09-02, and nothing feeds "
               "contact sounds from the body-indexed Newton contact snapshot yet. Contact-sound MASS "
               "WEIGHTING is separately unimplemented.",
     "scope": "engine",
     "symptom": "collisions are mute. Motor sounds are unaffected.",
     "source": "src/omnisim/sound/OmSoundEngine.cpp (updateAfterPhysicsStep; the contact producer is gone)",
     "diagnostic": None,
     "workaround": "none in-engine. Detect the collision from /sim/events contact.began and add "
                   "audio in post."},
    # The two GUI entries that used to sit here (gui.wgpu_main_view_overlays and
    # gui.contact_points_overlay) are GONE, deliberately. Both were written the
    # week the 2026-08-19 main-view flip left every WREN-drawn surface dark, and
    # both recommended `renderBackend "wren"` / OMNISIM_FORCE_WREN=1 as the way
    # back -- a renderer that was DELETED on 2026-08-23 (976b9449d; both spellings
    # are now warned no-ops). The surfaces themselves were wired back into the
    # wgpu frame: selection outline + every Optional Rendering item with a
    # collector, contact-point crosses included (W4a, OmView3D::renderMainFrameViaWgpu
    # -> OmWgpuView::collectContactCrosses reading computedContactPoints()), the
    # manipulator gizmo (P8, OmGizmoLines) and device HUD insets + supervisor labels
    # (P7, OmHudOverlay) -- hatches OMNISIM_WGPU_OVERLAYS / _GIZMO / _HUD = 0,
    # all default ON. Agents never needed the pixels anyway: read state through
    # /scene/tree, /scene/node/<def>, /robots and /sim/contacts.
    {"feature": "motor.force_feedback (getForceFeedback / getTorqueFeedback)",
     "scope": "engine",
     "reason": "the feedback read went through the ODE joint (dJointGetFeedback via "
               "OmBasicJoint::jointID()), and mJoint is now assigned NULL and nothing else -- so "
               "jointID() is permanently NULL and computeFeedback() returns 0.0 before doing any "
               "work. Propeller thrust/torque is the one exception: it is computed from the motor's "
               "own model, not from the solver.",
     "symptom": "0.0 from every motorised joint's force/torque feedback, with no warning. "
                "(Relatedly: turning a motor OFF is unimplemented -- a motor switched off keeps "
                "whatever target the last per-tick Newton push wrote.)",
     "source": "src/omnisim/nodes/OmRotationalMotor.cpp (computeFeedback, turnOffMotor), "
               "src/omnisim/nodes/OmMotor.cpp (the computeFeedback callers), "
               "src/omnisim/nodes/OmBasicJoint.hpp (jointID)",
     "diagnostic": None,
     "workaround": "none. Do not use force feedback as a contact or load signal on this build."},
    {"feature": "sensor.occlusion_rays vs a node DELETED at runtime",
     "scope": "engine",
     "reason": "occlusion itself WORKS again (fixed 2026-08-08: the ray carrier is now plain "
               "start/direction/length members answered by the Newton raycast service, the same "
               "pattern Receiver and LightSensor already used -- a wall between sensor and target "
               "now blocks it, verified by a control/differential pair in "
               "tests/test_newton_radar_occlusion_parity.py). What does NOT work BY DEFAULT is a node "
               "removed by the supervisor at runtime: the Newton world is frozen at finalize and has "
               "no INCREMENTAL remove path, so the deleted geometry stays in the MuJoCo model and "
               "keeps blocking rays until the world is rebuilt or reloaded.",
     "symptom": "after wb_supervisor_node_remove_node(), a target that the removed node used to "
                "hide STILL reports as occluded. Measured on tests/api/worlds/camera_recognition.omniworld: "
                "the 8-vs-7 occlusion count now passes, and the later assertion -- 1 object visible "
                "once the occluders are removed -- reads 0.",
     "source": "src/omnisim/nodes/utils/OmObjectDetection.cpp (working); the gap is the frozen "
               "model -- src/omnisim/physics/OmNewtonBackend.cpp (finalizeWorld) -- and the fix is the "
               "whole-world rebuild verb wb_supervisor_simulation_rebuild_physics "
               "(include/controller/c/omnisim/supervisor.h)",
     "diagnostic": "OCCLUSION_RAYS_UNANSWERED",
     "workaround": "POST /sim/rebuild_physics after removing collidable nodes (W1.7, 2026-09-01; "
                   "97-267 ms, refused on particle worlds), or reload the world. This is "
                   "NOT limited to rays: CONTACTS against a deleted static are affected too, "
                   "MEASURED 2026-08-08 -- a body resting on an elevated floor stayed at z=0.5999 "
                   "for 61,440 steps after the floor was deleted, and the engine's own step log "
                   "still listed the deleted floor's body at its authored pose. A deleted wall "
                   "still stops a robot. Tested on one static Solid under the default CPU solver; "
                   "dynamic bodies, robots and mujoco_warp are unmeasured."},
    {"feature": "scene.runtime_mutation_physics (/scene/spawn and /scene/delete reach the solver)",
     "scope": "engine",
     "reason": "the Newton/MuJoCo model is FROZEN at finalizeWorld(): openForBuild goes false, "
               "every addBody/addShape* verb guards on it, ensureWorldOpen() refuses to reopen "
               "mid-run, and there is no incremental removeBody path either. So BY DEFAULT a node "
               "spawned mid-session is NEVER registered with the solver, and a node deleted "
               "mid-session leaves its colliders in it -- the exact mirror pair (see the "
               "occlusion-rays entry above for the delete half's measurements).",
     "symptom": "a spawned dynamic body never falls (measured 2026-08-17: released at z=1.5, "
                "unchanged after 2200 steps / ~87 s sim time, while its authored twin settled at "
                "0.599892) and a spawned static body never collides; a deleted wall still blocks "
                "robots and rays and a deleted floor still holds bodies up. Engine-side: 0 errors, "
                "0 warnings, and the spawn response reads verification.node_resolved: true.",
     "source": "src/omnisim/physics/OmNewtonBackend.cpp (finalizeWorld sets openForBuild = false; "
               "ensureWorldOpen refuses to reopen); the opt-in fix shipped 2026-09-01 as W1.7 "
               "(88487d988): wb_supervisor_simulation_rebuild_physics, "
               "include/controller/c/omnisim/supervisor.h",
     "diagnostic": "SILENT engine-side; this harness attaches `physics_warning` "
                   "RUNTIME_MUTATION_NOT_IN_SOLVER to every successful /scene/spawn and "
                   "/scene/delete response that did not opt into the rebuild, and emits one "
                   "world.warning per verb per world-load",
     "workaround": "FIXED 2026-09-01 (W1.7): POST /sim/rebuild_physics -- or pass "
                   "{\"physics\": \"rebuild\"} on /scene/spawn / /scene/delete -- rebuilds the "
                   "Newton world at the scene's CURRENT poses in 97-267 ms (measured), so spawned "
                   "nodes gain physics and deleted ones lose it. Verified: a spawned box landed "
                   "bit-identical to its authored twin (0.599892258644104) and an 8-robot world "
                   "drove through a mid-run rebuild at unchanged speed. Caveats: 409 "
                   "REBUILD_REFUSED on Cloth/SoftBody/GranularBed worlds (they re-register from "
                   "authored state -- reload those); engaged Connector/VacuumGripper welds are "
                   "DROPPED with a loud warning (re-lock from the controller); bitwise "
                   "step-for-step continuation across a rebuild is not claimed. The DEFAULT "
                   "spawn/delete behaviour is unchanged (this entry stays so the default's "
                   "physics_warning remains explained); /scene/set_pose is unaffected (it moves "
                   "a body the solver already knows)."},
    {"feature": "solid.setInertiaMatrixFromBoundingObject",
     "scope": "engine",
     "reason": "computing an inertia matrix from the bounding object is unavailable -- it ran ODE's "
               "mass integrator, and ODE has been removed. The inertiaMatrix fields are NOT "
               "modified.",
     "symptom": "the call warns and leaves the fields as they were, so a body keeps whatever "
                "inertia it had (possibly the identity matrix).",
     "source": "src/omnisim/nodes/OmSolid.cpp (setInertiaMatrixFromBoundingObject)",
     "diagnostic": "INERTIA_FROM_BOUNDING_OBJECT_UNAVAILABLE",
     "workaround": "author an explicit `inertiaMatrix` (and `centerOfMass`) in the Physics node."},
    {"feature": "track.propulsion (a Track's belt drives nothing)",
     "scope": "engine",
     "reason": "OmTrack computes a belt surface velocity and exposes it as contactSurfaceVelocity(), "
               "which has ZERO readers in the tree: it was consumed by ODE's contact-surface-velocity "
               "mechanism, and nothing replaced it when ODE was deleted (bdc02139). The belt "
               "ANIMATION (belt elements, wheel spin, texture scroll, the PositionSensor) was "
               "separately dead on both renderers until 2026-08-23 -- prePhysicsStep gated the whole "
               "update on a dBodyID that had been NULL since the deletion -- and works again; the "
               "PROPULSION does not.",
     "symptom": "a tracked robot's belt animates and its PositionSensor advances while the chassis "
                "does not move. No error, no warning.",
     "source": "src/omnisim/nodes/OmTrack.cpp (prePhysicsStep), "
               "src/omnisim/nodes/OmTrack.hpp (contactSurfaceVelocity, unread)",
     "diagnostic": None,
     "workaround": "none in-engine. Propel the chassis through wheels (HingeJoint + RotationalMotor "
                   "DO drive on Newton) and keep the Track for its visuals, or move the body "
                   "kinematically from a supervisor. Making a tracked robot drive under Newton is "
                   "open work."},
    {"feature": "joint.fields (stopErp/stopCfm, springs+damping, suspension, BallJoint hard "
                "stops, Connector snap alignment, Propeller inflow, Track force control, motor "
                "turn-off)",
     "scope": "engine",
     "reason": "each of these had ODE as its ONLY implementation and was deleted with it (commit "
               "5b380175, which removed 46 dead odeBackend() call sites / ~113 verb calls that were "
               "writing into an inert stub while looking like working code). Specifically: "
               "HingeJointParameters.stopErp/stopCfm named ODE's error-reduction / "
               "constraint-force-mixing pair, and Newton's joint limits are a spring tuned by "
               "OMNISIM_NEWTON_LIMIT_KE/KD instead; spring/damping/staticFriction and "
               "suspension*/suspensionAxis were ODE motor state; BallJoint per-axis stops are not "
               "mapped (the BALL element is `limited: False`; Hinge2 axes ARE limited since the d6 "
               "path went default-on 2026-08-17); Connector snap rotated the two parent ODE bodies "
               "(both handles are permanently NULL, so the whole snap chain computes a quaternion "
               "that lands nowhere -- the Newton weld attaches WITHOUT snapping); Propeller's axial "
               "inflow term needed a body point-velocity read, so V is pinned to 0 and only the "
               "omega^2 term survives; Track force control needed the belt body's mass from "
               "dBodyGetMass; turning a motor off zeroed the ODE motor velocity. Related, and "
               "NOT a gap any more: a motor with no minPosition/maxPosition (and no joint stops) is "
               "built as a VELOCITY WHEEL with ke = 0, and since 2026-09-01 the first finite "
               "setPosition() from the CONTROLLER promotes it to a position servo (wire-level "
               "latch; OMNISIM_NEWTON_PROMOTE_SERVO=0 reverts) -- supervisor-side "
               "/robot/<def>/joints/set writes do NOT promote, which is why that verb reports "
               "position_controllable: false for such a joint.",
     "symptom": "the field is accepted and does nothing. A spring that does not push reads as a "
                "physics bug rather than as a missing feature -- which is why the engine warns per "
                "joint for the subset it can detect.",
     "source": "src/omnisim/nodes/OmHingeJoint.cpp (applyToOdeStopErp, "
               "applyToOdeSpringAndDampingConstants, prePhysicsStep), "
               "src/omnisim/nodes/OmHinge2Joint.cpp (applyToOdeSpringAndDampingConstants), "
               "src/omnisim/nodes/OmBallJoint.cpp (applyToOdeMinAndMaxStop), "
               "src/omnisim/nodes/OmConnector.cpp (snapXAxes), "
               "src/omnisim/nodes/OmPropeller.cpp (prePhysicsStep), "
               "src/omnisim/nodes/OmTrack.cpp (prePhysicsStep), "
               "src/omnisim/nodes/OmRotationalMotor.cpp (turnOffMotor), "
               "src/omnisim/nodes/OmLinearMotor.cpp (turnOffMotor)",
     "diagnostic": "JOINT_FEATURE_UNIMPLEMENTED (Hinge2Joint / BallJoint only -- the others are "
                   "SILENT)",
     "workaround": "for limits, tune OMNISIM_NEWTON_LIMIT_KE/KD; for suspension and springs, model "
                   "them as an actuated joint driven by your controller; for Track force control, "
                   "use position or velocity control (both unaffected)."},
]

# Supervisor CommandError text -> (HTTP status, machine-branchable code) for the
# mutation / state verbs. Substring match on the message the supervisor raises;
# anything unmatched keeps the historical 503.
SUPERVISOR_ERROR_CODE_MAP: tuple[tuple[str, int, str], ...] = (
    ("to spawn into", 404, "PARENT_DEF_NOT_FOUND"),
    ("to clone", 404, "CLONE_DEF_NOT_FOUND"),
    ("is already taken by an existing", 409, "DEF_TAKEN"),
    ("no node with DEF", 404, "DEF_NOT_FOUND"),
    ("no snapshot named", 404, "SNAPSHOT_NOT_FOUND"),
    ("are reserved", 400, "SNAPSHOT_NAME_RESERVED"),
    ("has no 'translation' field", 422, "FIELD_NOT_ON_NODE"),
    ("has no 'rotation' field", 422, "FIELD_NOT_ON_NODE"),
    ("unknown joint name", 422, "JOINT_NOT_FOUND"),
    ("match multiple joints", 409, "JOINT_NAME_AMBIGUOUS"),
    # solve_ik (POST /robot/<def>/ik). "no node with DEF" above already covers
    # the missing robot/effector (404 DEF_NOT_FOUND). The 503s are engine
    # state a retry may fix (a world mid-finalize); the 422s are the world's
    # own shape, which a retry never fixes.
    ("no IK-solvable joints", 422, "IK_NO_JOINTS"),
    ("has no Newton physics body", 422, "IK_NO_BODY"),
    ("IK solver failed", 500, "IK_SOLVER_FAILED"),
    ("IK unavailable", 503, "IK_UNAVAILABLE"),
    ("requires", 400, "ARGUMENT_MISSING"),
)

# Message fragments that mean the SUPERVISOR (or the link to it) is the
# problem, not the caller's request. Everything the supervisor's dispatch
# raises arrives here as bare prose -- a `CommandError` (the caller asked for
# something impossible) and a transport failure are indistinguishable by type,
# so they are told apart by the text each layer is known to produce.
# SupervisorClient composes the transport ones; harness_supervisor.py's
# dispatch composes the rest.
SUPERVISOR_TRANSPORT_MARKERS: tuple[str, ...] = (
    "supervisor RPC failed",
    "supervisor is not connected",
    "supervisor not connected",
    "could not connect to supervisor",
    "supervisor closed the connection",
    "world load in progress",
    "bad response length",
    "is unavailable",
)

# Codes the classifier can produce that no rule table owns; declared so
# `known_request_error_codes()` can publish them on /capabilities.
SUPERVISOR_CLASSIFIER_CODES: tuple[str, ...] = (
    "ARGUMENT_INVALID", "SUPERVISOR_UNAVAILABLE", "SUPERVISOR_INTERNAL_ERROR",
    "SUPERVISOR_LOST",
)

# /world/screenshot's two refusals, in (empty, not-a-PNG) order. Declared here
# because the endpoint picks between them at runtime, and a code that only
# exists inside a conditional is invisible to the source scanner that publishes
# the enum on /capabilities.
SCREENSHOT_ERROR_CODES: tuple[str, str] = ("SCREENSHOT_EMPTY", "SCREENSHOT_NOT_PNG")


def classify_supervisor_error(message: str) -> tuple[int, str]:
    """Map a supervisor RPC failure onto (HTTP status, machine-branchable code).

    ⚠ WHY THIS EXISTS. Every supervisor rejection used to come back as a 503,
    including the caller's own bad argument. Measured 2026-08-12:
    `POST /sim/reset {"restore": 7}` answered
    `503 {"error": "'restore' must be a snapshot name or null"}` -- and
    `urllib`/`requests` RAISE on a 5xx, so the agent saw a server failure and
    retried a request that could never succeed. A client error is a 4xx; a
    5xx is a promise that retrying might help.

    The three categories, in precedence order:
      1. `internal: ...`  -> 500. The supervisor's dispatch prefixes an
         unhandled exception with this; it is our bug, not the caller's.
      2. transport prose  -> 503. The link is down or the world is loading;
         retrying is exactly right.
      3. everything else  -> a `CommandError` from the supervisor's dispatch,
         i.e. the caller asked for something it cannot have. Refined by
         SUPERVISOR_ERROR_CODE_MAP (404 for a missing DEF, 409 for a taken
         one, ...) and 400 otherwise.
    """
    msg = message or ""
    if msg.startswith("internal:") or ": internal:" in msg:
        return 500, "SUPERVISOR_INTERNAL_ERROR"
    for marker in SUPERVISOR_TRANSPORT_MARKERS:
        if marker in msg:
            return 503, "SUPERVISOR_UNAVAILABLE"
    for needle, status, code in SUPERVISOR_ERROR_CODE_MAP:
        if needle in msg:
            return status, code
    return 400, "ARGUMENT_INVALID"

_ROUTE_LITERAL_RE = re.compile(r'"(/[a-z_/<>]*)"')
_HARNESS_CODE_RE = re.compile(r'"code":\s*"([A-Z][A-Z0-9_]+)"')
_LOG_TYPE_RE = re.compile(r'"type":\s*"([a-z_]+\.[a-z_]+)"|type_ = "([a-z_]+\.[a-z_]+)"')


@functools.lru_cache(maxsize=4)
def verify_routes(handler_source: str) -> dict:
    """Cross-check the declared ROUTES table against the handler source.

    Route matching in `do_GET`/`do_POST` uses four different syntactic forms
    (`path ==`, `path in (...)`, `base ==`, `startswith`) plus per-segment
    matching, so the scan looks for path-shaped string literals rather than
    trying to parse the dispatch logic. A declared route counts as found when
    its literal prefix (up to the first `<`) appears in the source.
    """
    literals = {m for m in _ROUTE_LITERAL_RE.findall(handler_source) if m.startswith("/")}
    declared_prefixes = {r["path"].split("<", 1)[0] for r in ROUTES}
    not_found = [r["path"] for r in ROUTES
                 if r["path"] not in literals and r["path"].split("<", 1)[0] not in literals]
    undeclared = sorted(lit for lit in literals
                        if lit not in declared_prefixes
                        and not any(lit.startswith(p) or p.startswith(lit)
                                    for p in declared_prefixes))
    return {
        "declared": len(ROUTES),
        "scanned_literals": len(literals),
        "declared_not_found_in_source": sorted(not_found),
        "undeclared_literals": undeclared,
        "verified": not not_found and not undeclared,
        "source": "declared table cross-checked against the request handler's source",
    }


@functools.lru_cache(maxsize=4)
def verify_log_event_types(source: str) -> dict:
    """Cross-check LOG_EVENT_TYPES against what LogRingBuffer actually emits."""
    found = sorted({a or b for a, b in _LOG_TYPE_RE.findall(source) if (a or b)})
    declared = list(LOG_EVENT_TYPES)
    undeclared = [t for t in found if t not in declared]
    missing = [t for t in declared if t not in found]
    return {
        "types": declared,
        "emitters_found": found,
        "undeclared": undeclared,
        "declared_not_emitted": missing,
        "verified": not undeclared and not missing,
        "source": "scanned LogRingBuffer's emit sites in omnisim_harness.py",
    }


# Load diagnostics the HARNESS synthesizes itself (the rest come from the
# classifier's rule table over the engine log).
HARNESS_DIAGNOSTIC_CODES = (
    "LAUNCHER_DLL_NOT_FOUND", "SIMULATOR_EXITED_NONZERO",
    "SUPERVISOR_BIND_STALLED", "SUPERVISOR_BIND_CEILING",
    "WORLD_DIR_NOT_WRITABLE",
)


_OWN_SOURCE_CACHE: str | None = None


def _own_source() -> str:
    # Memoised: the file cannot change meaning in-process, and /capabilities
    # used to re-read this ~300 KB source (twice) on every call.
    global _OWN_SOURCE_CACHE
    if _OWN_SOURCE_CACHE is None:
        try:
            _OWN_SOURCE_CACHE = Path(__file__).read_text(
                encoding="utf-8", errors="replace")
        except OSError:
            _OWN_SOURCE_CACHE = ""
    return _OWN_SOURCE_CACHE


def known_diagnostic_codes() -> list[str]:
    """Every *load diagnostic* code a client can see: the classifier's rule
    table (anchored in real OmLog call sites) plus the ones the harness
    synthesizes when the engine never got far enough to log.
    """
    import diagnostic_codes as dc
    codes = {rule[2] for rule in getattr(dc, "_RULES", ())}
    codes.update(getattr(dc, "CUDA_CODES", ()))
    # Codes the classifier SYNTHESIZES from a matched rule rather than owning a
    # rule of their own (NEWTON_ZERO_DYNAMIC_BODIES is refined out of the
    # registration census). Without this they were reachable but undiscoverable.
    codes.update(getattr(dc, "SYNTHESIZED_CODES", ()))
    codes.add("UNKNOWN")
    codes.update(HARNESS_DIAGNOSTIC_CODES)
    return sorted(codes)


@functools.lru_cache(maxsize=1)
def known_request_error_codes() -> list[str]:
    """Machine-branchable `code` values on 4xx request-error bodies.

    Scanned from the harness's own source, so a new one cannot be added
    without appearing here. Most harness errors are still free text (§16 of
    PROTOCOL.md) — this is the subset that carries a code.
    """
    scanned = set(_HARNESS_CODE_RE.findall(_own_source()))
    scanned.update(code for _, _, code in SUPERVISOR_ERROR_CODE_MAP)
    scanned.update(SUPERVISOR_CLASSIFIER_CODES)
    scanned.update(SCREENSHOT_ERROR_CODES)
    return sorted(scanned - set(HARNESS_DIAGNOSTIC_CODES))


# The bracketed engine log tag is named after the emitting C++ class, and those
# classes are being renamed Wb* -> Om*.  Match BOTH prefixes, permanently, so
# this stays correct on either side of the rename and old logs keep parsing.
_NEWTON_FINALISED_SOLVER_RE = re.compile(
    r"\[(?:Wb|Om)NewtonBackend\] world finalised \(solver=")


def read_newton_verdict(log_path: Path) -> dict:
    """The engine's own backend verdict for the run that owns `log_path`.

    `OmNewtonBackend::finalizeWorld` writes `<log>.newton.json` when Newton
    finalises a world, and `OmLog` deletes a stale copy when it truncates the
    log at startup — so the file's presence is the authoritative "Newton drove
    this world" signal, and its absence means ODE *or* a load that never
    reached finalize. Both are reported distinctly; guessing "ODE" from a
    missing file is exactly the mistake AGENTS.md warns about.
    """
    sidecar = Path(str(log_path) + ".newton.json")
    out: dict = {"sidecar_path": str(sidecar)}
    if sidecar.exists():
        try:
            data = json.loads(sidecar.read_text(encoding="utf-8", errors="replace"))
        except (OSError, ValueError) as exc:
            out.update({"backend": "unknown", "source": "sidecar_unreadable",
                        "detail": str(exc)})
            return out
        out.update({
            "backend": data.get("backend", "newton"),
            "solver": data.get("solver"),
            "degraded": data.get("degraded"),
            "finalised": data.get("finalised"),
            "source": "sidecar",
        })
        try:
            out["sidecar_age_s"] = round(time.time() - sidecar.stat().st_mtime, 1)
        except OSError:
            pass
        return out
    # No sidecar: fall back to the engine log's own finalise line. Scan the
    # WHOLE file, not the tail — a tail-only read used to miss the load-time
    # line on a large log and falsely report ODE (fixed in ad9fff48).
    try:
        text = log_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        text = ""
    # Accept BOTH the current "[OmNewtonBackend]" tag and the post-rename
    # "[OmNewtonBackend]" one: the tag is named after the emitting C++ class and
    # those classes are being renamed Wb* -> Om*.  Dual-accepting means
    # /capabilities.physics provenance keeps resolving across the rename in
    # either direction, and log files captured before it keep parsing forever.
    m = _NEWTON_FINALISED_SOLVER_RE.search(text)
    if m:
        # The solver name itself contains parentheses ("XPBD(iters=10)"), so
        # take the rest of the LINE and drop the single trailing ')'.
        line = text[m.end():].splitlines()[0].strip()
        out.update({"backend": "newton",
                    "solver": line[:-1] if line.endswith(")") else line,
                    "source": "engine_log"})
        return out
    out.update({
        # Unverified, NOT "ode": the only honest reading of a missing sidecar is
        # that we cannot tell, and the likeliest cause is a run too short to
        # reach world finalize.
        "backend": "unverified",
        "source": "sidecar_absent",
        "detail": (
            "no .newton.json sidecar and no Newton finalise line in the engine "
            "log. Newton is the only backend, so this does NOT mean some other "
            "engine drove the world -- it means this run never reached world "
            "finalize (budget --duration >= 15 s, >= 45 s on slow or virtualised "
            "disks), or the runtime failed to come up. A too-short run proves "
            "nothing about the backend."
        ),
    })
    return out


_BODY_CENSUS_CACHE: dict = {}

_BODY_CENSUS_RE = re.compile(
    r"\[(?:Wb|Om)NewtonBackend\] registered (\d+) dynamic \+ (\d+) static Newton bodies")


def read_body_census(log_path: Path) -> dict:
    """The engine's own registration census for the CURRENT world.

    `[OmNewtonBackend] registered N dynamic + M static Newton bodies` is the one
    line that answers "is this world actually being simulated" with a number,
    and it was unreachable over HTTP: it is emitted at INFO, and INFO was dropped
    twice (no classifier pattern, and `emit_world_diagnostic` bailed on it).

    Chosen exposure: BOTH. It is promoted to a branchable diagnostic
    (`NEWTON_ZERO_DYNAMIC_BODIES` when N == 0, `NEWTON_BODIES_REGISTERED`
    otherwise) so it shows up on `/world/load` and `/sim/events`, AND the raw
    counts are served here on `/capabilities.physics` so an agent can read the
    number without correlating a cursor. A diagnostic alone would be missable by
    anyone who did not poll at load time; a field alone would not reach the
    stream where load failures are already being read.

    The LAST census in the log wins: hot reloads append, so the last one belongs
    to the world that is loaded now.
    """
    out: dict = {"dynamic_bodies_registered": None,
                 "static_bodies_registered": None,
                 "source": "engine_log_census"}
    # Cached on (path, mtime_ns, size): the log only grows between reloads,
    # and /capabilities used to re-read + re-regex the whole file per call.
    try:
        st = log_path.stat()
        cache_key = (str(log_path), st.st_mtime_ns, st.st_size)
    except OSError:
        out["source"] = "log_unreadable"
        return out
    cached = _BODY_CENSUS_CACHE.get("entry")
    if cached is not None and cached[0] == cache_key:
        return dict(cached[1])
    try:
        text = log_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        out["source"] = "log_unreadable"
        return out
    matches = _BODY_CENSUS_RE.findall(text)
    if not matches:
        out["source"] = "census_absent"
        out["detail"] = (
            "the engine logged no registration census. It is written on the tick that "
            "builds the world, so this means the world never got that far (or nothing "
            "registered at all) -- NOT that the count is zero.")
        _BODY_CENSUS_CACHE["entry"] = (cache_key, dict(out))
        return out
    dynamic, static = matches[-1]
    out["dynamic_bodies_registered"] = int(dynamic)
    out["static_bodies_registered"] = int(static)
    if out["dynamic_bodies_registered"] == 0:
        out["warning"] = (
            "ZERO dynamic bodies were registered with the physics backend. Either this "
            "world authors no dynamic body at all (legal -- a static scene), or every "
            "one it authored fell out of the simulation and nothing will move, fall or "
            "collide. Read the load diagnostics: NO_PHYSICS_BACKEND, "
            "NEWTON_RUNTIME_ABSENT/BROKEN, SOLID_ODE_PIN_INERT, NEWTON_ENFORCE_REFUSED.")
    _BODY_CENSUS_CACHE["entry"] = (cache_key, dict(out))
    return out


def sim_version() -> str | None:
    try:
        import omnisim as _omnisim_pkg
        return getattr(_omnisim_pkg, "__version__", None)
    except Exception:  # noqa: BLE001
        return None


def git_commit(repo_root: Path) -> str | None:
    head = repo_root / ".git" / "HEAD"
    try:
        raw = head.read_text(encoding="utf-8").strip()
    except OSError:
        return None
    if raw.startswith("ref: "):
        ref = repo_root / ".git" / raw[5:]
        try:
            return ref.read_text(encoding="utf-8").strip()[:8]
        except OSError:
            packed = repo_root / ".git" / "packed-refs"
            try:
                for line in packed.read_text(encoding="utf-8").splitlines():
                    if line.endswith(" " + raw[5:]):
                        return line.split(" ", 1)[0][:8]
            except OSError:
                return None
            return None
    return raw[:8]


# ---------------------------------------------------------------------------
# VRML composition for POST /scene/spawn
# ---------------------------------------------------------------------------


def vrml_value(value) -> str:
    """Serialize a JSON value as a VRML field value.

    Strings are quoted verbatim — a `url` must therefore use forward slashes,
    which is what the engine wants on every platform anyway. Nested nodes are
    deliberately unsupported: pass the whole node as `vrml` instead of trying
    to express VRML in JSON.
    """
    if isinstance(value, bool):
        return "TRUE" if value else "FALSE"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return repr(round(value, 9))
    if isinstance(value, str):
        return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'
    if isinstance(value, (list, tuple)):
        if not value:
            return "[ ]"  # an empty MF field; NOT the empty string
        if all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value):
            return " ".join(vrml_value(v) for v in value)
        return "[ " + " ".join(vrml_value(v) for v in value) + " ]"
    if value is None:
        return "NULL"
    raise ValueError(
        f"cannot express {type(value).__name__} as a VRML field value; "
        "pass the whole node as 'vrml' instead")


def _skip_quoted_vrml(text: str, i: int) -> int:
    """Index just past the string literal starting at ``text[i] == '"'``."""
    i += 1
    n = len(text)
    while i < n and text[i] != '"':
        if text[i] == "\\":
            i += 1
        i += 1
    return i + 1


def top_level_field_names(vrml: str) -> set[str]:
    """The identifiers that appear at the TOP level of a VRML node string.

    Depth-aware, because a substring test is not, and that is the whole bug it
    exists to fix. `compose_spawn_vrml` used to ask ``"translation" not in
    body`` before splicing the caller's pose in -- so a node whose
    ``boundingObject Pose { translation 0 0 0.5 ... }`` merely MENTIONS the word
    made it believe the caller's translation was already there. Measured: a
    spawn carrying that boundingObject plus ``"translation":[3,0,1]`` got the
    rotation and lost the translation entirely (the supervisor's field-write
    fallback is clone-only, so nothing recovered it), and the only trace was
    ``verification.pose_delta_m`` reading 3.162.

    Same scan as the supervisor's ``replace_top_level_field``
    (projects/default/controllers/harness_supervisor/harness_supervisor.py):
    quoted strings are skipped whole, and ``{`` / ``[`` raise the depth so a
    child node's fields are never confused for the node's own. The two live in
    different processes and cannot share a module, so
    tests/harness/test_mutation_verbs.py cross-checks them against each other
    on the same inputs.

    Deliberately loose about what it collects: field VALUES that are bare
    identifiers (``TRUE``, ``NULL``, a nested node's type name) land in the set
    too. Callers ask "is this field name present at depth 1", and no field
    value is ever the bare token ``translation`` or ``rotation``.
    """
    try:
        open_brace = vrml.index("{")
    except ValueError:
        return set()
    names: set[str] = set()
    depth = 1
    i = open_brace + 1
    n = len(vrml)
    while i < n:
        ch = vrml[i]
        if ch == '"':
            i = _skip_quoted_vrml(vrml, i)
            continue
        if ch in "{[":
            depth += 1
            i += 1
            continue
        if ch in "}]":
            depth -= 1
            if depth == 0:
                break
            i += 1
            continue
        if depth == 1 and (ch.isalpha() or ch == "_"):
            k = i
            while k < n and (vrml[k].isalnum() or vrml[k] == "_"):
                k += 1
            names.add(vrml[i:k])
            i = k
            continue
        i += 1
    return names


def compose_spawn_vrml(spec: dict, repo_root: Path) -> tuple[str, str | None]:
    """Turn a /scene/spawn body into (vrml_node_string, def_name).

    Three input shapes, in priority order:
      {"vrml": "DEF H URDFRobot { ... }"}   full control, passed through
      {"urdf": "<path>", ...}               sugar for URDFRobot { url ... }
      {"type": "Solid", "fields": {...}}    composed from JSON

    `def`, `translation` and `rotation` are spliced in for the composed forms,
    and appended as fields for a raw `vrml` string that does not already carry
    them — splicing the pose into the node text (rather than setting the field
    after import) means the body is created AT the pose, so it never appears at
    the origin for one step and never needs a settle step to be correct.
    """
    def_name = spec.get("def")
    if def_name is not None and (not isinstance(def_name, str) or not def_name.strip()):
        raise ValueError("'def' must be a non-empty string")
    translation = spec.get("translation")
    rotation = spec.get("rotation")
    if translation is not None and (not isinstance(translation, list) or len(translation) != 3):
        raise ValueError("'translation' must be a list of 3 numbers")
    if rotation is not None and (not isinstance(rotation, list) or len(rotation) != 4):
        raise ValueError("'rotation' must be [ax, ay, az, angle]")

    raw = spec.get("vrml")
    if isinstance(raw, str) and raw.strip():
        body = raw.strip()
        # Depth-aware, NOT a substring test: `boundingObject Pose { translation
        # ... }` mentions the word without the NODE carrying the field, and a
        # substring test therefore dropped the caller's pose silently.
        own_fields = top_level_field_names(body)
        extra: list[str] = []
        if translation is not None and "translation" not in own_fields:
            extra.append(f"  translation {vrml_value(translation)}")
        if rotation is not None and "rotation" not in own_fields:
            extra.append(f"  rotation {vrml_value(rotation)}")
        if extra:
            open_brace = body.find("{")
            if open_brace < 0:
                raise ValueError("'vrml' does not look like a node: no '{' found")
            body = (body[:open_brace + 1] + "\n" + "\n".join(extra) + "\n"
                    + body[open_brace + 1:])
        if def_name and not body.startswith("DEF "):
            body = f"DEF {def_name} {body}"
        return body, (def_name or None)

    fields = dict(spec.get("fields") or {})
    node_type = spec.get("type")
    urdf = spec.get("urdf")
    if urdf:
        if not isinstance(urdf, str):
            raise ValueError("'urdf' must be a path string")
        path = Path(urdf)
        if not path.is_absolute():
            path = repo_root / urdf
        # The engine resolves a URDFRobot url off disk; forward slashes work on
        # every platform and a backslash inside a VRML string does not.
        fields.setdefault("url", path.as_posix())
        node_type = node_type or "URDFRobot"
    if not node_type or not isinstance(node_type, str):
        raise ValueError("provide one of: 'vrml', 'urdf', or 'type' (+ optional 'fields')")
    if translation is not None:
        fields["translation"] = translation
    if rotation is not None:
        fields["rotation"] = rotation
    lines = [f"  {name} {vrml_value(value)}" for name, value in fields.items()]
    head = f"DEF {def_name} {node_type}" if def_name else node_type
    return head + " {\n" + "\n".join(lines) + "\n}", (def_name or None)


def infer_omnisim_home() -> Path:
    return Path(os.environ.get("OMNISIM_HOME", str(REPO_ROOT)))


def resolve_omnisim_binary(omnisim_home: Path) -> Path:
    if sys.platform == "win32":
        candidates = [
            omnisim_home / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe",
            omnisim_home / "msys64" / "mingw64" / "bin" / "webots.exe",
        ]
    elif sys.platform == "darwin":
        candidates = [omnisim_home / "Contents" / "MacOS" / "omnisim", omnisim_home / "Contents" / "MacOS" / "webots"]
    else:
        candidates = [omnisim_home / "bin" / "omnisim-bin", omnisim_home / "webots"]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    # Distinguish the two causes. A wrong OMNISIM_HOME and an unbuilt tree
    # look identical here, and telling someone to "set OMNISIM_HOME" when it
    # is already right sends them chasing the wrong thing.
    looks_like_a_checkout = (omnisim_home / "AGENTS.md").exists()
    if looks_like_a_checkout:
        raise RuntimeError(
            f"No OmniSim binary under {omnisim_home} -- this looks like a\n"
            f"checkout that has not been built yet (OMNISIM_HOME itself is fine).\n"
            f"  Build it:  build_omni.bat        (Windows)\n"
            f"             python -m omnisim build all   (other platforms)\n"
             "  then:      make -C src/omnisim bundle-newton-runtime\n"
             "  check:     python -m omnisim doctor"
        )
    raise RuntimeError(
        f"Cannot find the OmniSim binary under {omnisim_home}. "
        "Set OMNISIM_HOME to a built tree or install location."
    )


# How many individual diagnostics of one code survive verbatim before the rest
# are coalesced into a single summary entry. 8 is enough to name the offending
# nodes in a small world while capping a large one.
DIAGNOSTIC_COALESCE_LIMIT = 8


def coalesce_diagnostics(diags: list[dict],
                         limit: int = DIAGNOSTIC_COALESCE_LIMIT) -> list[dict]:
    """Cap a per-node diagnostic flood at `limit` entries per code.

    ⚠ WHY. The engine warns ONCE PER SOLID for a `physicsBackend "ode"` pin
    (OmSolid.cpp gates it on `!s->mWarnedOdePinInert`, a per-node flag -- so the
    cap is per node, not per world, and it is deliberate: the engine cannot know
    which node the reader cares about). Harness-side that is a different problem:
    a 300-collider world would put 300 near-identical dicts into ONE
    `/world/load` response body, and push ~600 events (each diagnostic becomes a
    world.warning) into a 4096-slot ring buffer that also has to carry every
    controller.log line. The signal is "these N nodes are inert", which one entry
    carries better than 300.
    So: the FIRST `limit` of each code pass through verbatim (an agent still gets
    real, individually-addressable nodes to fix), and the remainder collapse into
    one entry carrying the total and the node list. Nothing is silently dropped
    -- the summary names what it absorbed.

    `UNKNOWN` is never coalesced: it is a bucket of unrelated messages that only
    share a lack of rule, so merging them would destroy the only information they
    have. Order is preserved: each summary lands where its code's overflow began.
    """
    # IDEMPOTENT: an entry this function already produced carries
    # `coalesced: True` and must be passed through untouched. Applying the
    # function twice otherwise treats its own summary as a 9th individual
    # diagnostic and collapses it into a fresh, EMPTY summary -- measured live on
    # the hot-reload path, where a 12-node flood came back as
    # `occurrences: 9, nodes: []`. Double application must be a no-op, not a
    # quietly wrong count.
    def _is_summary(d: dict) -> bool:
        return bool(d.get("coalesced"))

    counts: dict[tuple, int] = {}
    for diag in diags:
        code = diag.get("code")
        if code == "UNKNOWN" or code is None or _is_summary(diag):
            continue
        counts[(code, diag.get("severity"))] = counts.get((code, diag.get("severity")), 0) + 1
    if not any(n > limit for n in counts.values()):
        return diags

    seen: dict[tuple, int] = {}
    summaries: dict[tuple, dict] = {}
    out: list[dict] = []
    for diag in diags:
        code = diag.get("code")
        key = (code, diag.get("severity"))
        if code == "UNKNOWN" or code is None or _is_summary(diag) \
                or counts.get(key, 0) <= limit:
            out.append(diag)
            continue
        seen[key] = seen.get(key, 0) + 1
        if seen[key] <= limit:
            out.append(diag)
            continue
        summary = summaries.get(key)
        if summary is None:
            summary = {
                "code": code,
                "severity": diag.get("severity"),
                "coalesced": True,
                "occurrences": counts[key],
                "shown_individually": limit,
                "suppressed_duplicates": 0,
                "nodes": [],
                "message": (
                    f"{counts[key]} occurrences of {code} on this load; the first {limit} "
                    "are listed individually above and the rest are summarised here. The "
                    "engine emits this warning once per NODE, by design -- this cap is the "
                    "harness's, so one response cannot be flooded by a large world."),
            }
            if diag.get("hint"):
                summary["hint"] = diag["hint"]
            summaries[key] = summary
            out.append(summary)
        summary["suppressed_duplicates"] += 1
        node = diag.get("node_def") or diag.get("source_path")
        if node and len(summary["nodes"]) < 64:
            summary["nodes"].append(node)
    return out


def parse_log_lines(text: str) -> list[dict]:
    """Convert raw omnisim_log.txt content into a structured diagnostic list.

    Coalesced per code (see `coalesce_diagnostics`) so a per-node engine warning
    cannot flood one `/world/load` body or evict real events from the log ring.
    """
    return coalesce_diagnostics(classify_text(text))


def compute_look_at_orientation(position: list[float], target: list[float],
                                up: tuple[float, float, float] = (0.0, 0.0, 1.0)) -> list[float]:
    """Return the axis-angle orientation that aims the OmniSim Viewpoint at
    `target` from `position`, keeping the horizon level.

    The result is the four-tuple [ax, ay, az, angle] that the Viewpoint's
    `orientation` SFRotation field expects. Pure math, no IPC.

    The camera's local frame is +X forward, +Y left, +Z up. Building the full
    basis from an explicit `up` is what keeps the horizon level: the shortest-arc
    rotation from +X to the view direction also aims correctly but leaves roll
    uncontrolled (up to 92 deg for a view from behind, 75 deg for a plain
    isometric view), which is what this used to do.

    Kept numerically identical to ``omniworld.viewpoint.look_at`` — that module
    is the convention's reference implementation and bakes the same orientations
    into ``.wbt`` files. This copy exists only so the harness stays stdlib-only.
    """
    px, py, pz = position
    tx, ty, tz = target
    fx, fy, fz = tx - px, ty - py, tz - pz
    n = math.sqrt(fx * fx + fy * fy + fz * fz)
    if n < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    fx, fy, fz = fx / n, fy / n, fz / n

    ux, uy, uz = up
    un = math.sqrt(ux * ux + uy * uy + uz * uz)
    if un < 1e-12:
        ux, uy, uz = 0.0, 0.0, 1.0
        un = 1.0
    ux, uy, uz = ux / un, uy / un, uz / un
    # Degenerate when up is (anti)parallel to the view direction — e.g. a straight
    # top-down view with the default +Z up. Fall back to +Y, then +X.
    if 1.0 - abs(fx * ux + fy * uy + fz * uz) < 1e-6:
        ux, uy, uz = 0.0, 1.0, 0.0
        if 1.0 - abs(fx * ux + fy * uy + fz * uz) < 1e-6:
            ux, uy, uz = 1.0, 0.0, 0.0

    # Re-orthogonalise up against forward, then y = up x forward (right-handed).
    d = fx * ux + fy * uy + fz * uz
    ux, uy, uz = ux - d * fx, uy - d * fy, uz - d * fz
    un = math.sqrt(ux * ux + uy * uy + uz * uz)
    if un < 1e-12:
        return [0.0, 0.0, 1.0, 0.0]
    ux, uy, uz = ux / un, uy / un, uz / un
    yx = uy * fz - uz * fy
    yy = uz * fx - ux * fz
    yz = ux * fy - uy * fx

    # Columns are the camera basis: [forward | left | up].
    r = [[fx, yx, ux],
         [fy, yy, uy],
         [fz, yz, uz]]
    tr = r[0][0] + r[1][1] + r[2][2]
    angle = math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))
    if angle < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]

    s = 2.0 * math.sin(angle)
    if abs(s) < 1e-9:
        # ~180 deg: extract the axis from the diagonal (R = 2 a a^T - I).
        diag = (r[0][0], r[1][1], r[2][2])
        i = diag.index(max(diag))
        a = [0.0, 0.0, 0.0]
        a[i] = math.sqrt(max(0.0, (r[i][i] + 1.0) / 2.0))
        for j in range(3):
            if j != i:
                a[j] = (r[i][j] + r[j][i]) / (4.0 * a[i]) if a[i] > 1e-9 else 0.0
        an = math.sqrt(sum(c * c for c in a)) or 1.0
        return [a[0] / an, a[1] / an, a[2] / an, angle]

    return [(r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s, angle]


def png_size(data: bytes) -> tuple[int, int] | None:
    """(width, height) from a PNG's IHDR chunk. Stdlib only — the harness uses
    this to learn the REAL 3D-view aspect ratio from a screenshot instead of
    assuming 16:9, which is what every framing computation depends on.
    """
    if len(data) < 24 or data[:8] != b"\x89PNG\r\n\x1a\n" or data[12:16] != b"IHDR":
        return None
    width, height = struct.unpack(">II", data[16:24])
    if width <= 0 or height <= 0:
        return None
    return int(width), int(height)


# The strings that mean "no" wherever this harness reads a boolean, from a
# query string or from the environment. Shared so the two cannot drift.
FALSEY_STRINGS = ("", "0", "false", "no")


def parse_bool_param(qs: dict, name: str, default: bool = False) -> bool:
    raw = qs.get(name)
    if not raw:
        return default
    return str(raw[0]).strip().lower() not in FALSEY_STRINGS


def env_flag(name: str, default: bool = False) -> bool:
    """Read a boolean environment variable the way the person setting it means it.

    `bool(os.environ.get(name))` is TRUE for the string "0" -- so
    OMNISIM_HARNESS_LIGHT=0, the obvious way to turn light mode OFF, turned it
    ON, silently dropping /sim/grips and every contact / joint-limit / grip
    event. UNSET falls through to `default`; anything in FALSEY_STRINGS is
    False; everything else is True.
    """
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in FALSEY_STRINGS


def bounds_union(entries: list[dict]) -> dict | None:
    """Union a list of bounds dicts (from the supervisor) into one AABB."""
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    found = False
    exact = True
    for entry in entries:
        bmin = entry.get("bbox_min")
        bmax = entry.get("bbox_max")
        if not bmin or not bmax:
            continue
        found = True
        exact = exact and bool(entry.get("exact", True))
        for i in range(3):
            lo[i] = min(lo[i], float(bmin[i]))
            hi[i] = max(hi[i], float(bmax[i]))
    if not found:
        return None
    size = [hi[i] - lo[i] for i in range(3)]
    center = [(lo[i] + hi[i]) / 2.0 for i in range(3)]
    radius = 0.5 * math.sqrt(sum(s * s for s in size))
    return {
        "center": [round(c, 6) for c in center],
        "radius": round(radius, 6),
        "bbox_min": [round(v, 6) for v in lo],
        "bbox_max": [round(v, 6) for v in hi],
        "size": [round(s, 6) for s in size],
        "exact": exact,
    }


def screen_bbox(bbox_min, bbox_max, eye, orientation, fov, aspect,
                width=None, height=None) -> dict:
    """Screen-space extent of a world AABB: project its 8 corners.

    Corners behind the camera cannot be projected (they invert), so they are
    counted and reported rather than folded into the box — a caller can tell
    "fully on screen" from "clipped by the near plane".
    """
    corners = []
    for x in (bbox_min[0], bbox_max[0]):
        for y in (bbox_min[1], bbox_max[1]):
            for z in (bbox_min[2], bbox_max[2]):
                corners.append((x, y, z))
    xs: list[float] = []
    ys: list[float] = []
    behind = 0
    for corner in corners:
        proj = spatial.project(corner, eye, orientation, fov, aspect, width, height)
        if proj["behind_camera"]:
            behind += 1
            continue
        xs.append(proj["ndc_x"])
        ys.append(proj["ndc_y"])
    if not xs:
        return {"corners_behind_camera": behind, "ndc": None, "pixels": None}
    ndc = [round(min(xs), 6), round(min(ys), 6), round(max(xs), 6), round(max(ys), 6)]
    out = {"corners_behind_camera": behind, "ndc": ndc, "pixels": None}
    if width and height:
        out["pixels"] = [round(ndc[0] * width, 1), round(ndc[1] * height, 1),
                         round(ndc[2] * width, 1), round(ndc[3] * height, 1)]
    return out


def compute_render_stats(image_bytes: bytes) -> dict:
    """Quick brightness statistics over a PNG screenshot. Used by
    /world/render_stats so callers can check exposure without paying a
    full image round-trip + manual inspection.
    """
    if not _HAS_PIL:
        raise RuntimeError(
            "Pillow is not installed; render stats require it. "
            "Install with: pip install Pillow"
        )
    import io
    from PIL import ImageChops, ImageStat
    image = Image.open(io.BytesIO(image_bytes)).convert("RGB")
    width, height = image.size
    n = width * height
    if n == 0:
        return {"width": width, "height": height, "pixels": 0}
    # All-C-accelerated stats: no full-image Python materialization and no
    # per-pixel loop (the old list(getdata()) + for-loop cost ~64 MB and
    # seconds on a 4K frame). Equivalent semantics, verified by
    # tests/harness/test_helpers.py::test_render_stats_*.
    sum_r, sum_g, sum_b = ImageStat.Stat(image).sum
    (_, max_r), (_, max_g), (_, max_b) = image.getextrema()
    # Per-pixel max(r, g, b) as an L image, then a 256-bin histogram — the
    # saturated/black thresholds are on that per-pixel channel maximum.
    chan_r, chan_g, chan_b = image.split()
    chan_max = ImageChops.lighter(ImageChops.lighter(chan_r, chan_g), chan_b)
    hist = chan_max.histogram()
    saturated = sum(hist[250:256])
    black = sum(hist[0:6])
    mean = ((sum_r + sum_g + sum_b) / 3.0) / n
    sat_pct = 100.0 * saturated / n
    blk_pct = 100.0 * black / n
    warnings: list[str] = []
    if sat_pct > 30:
        warnings.append(
            f"blown out: {sat_pct:.1f}% of pixels are saturated; "
            "reduce DirectionalLight/PointLight intensities"
        )
    if blk_pct > 60:
        warnings.append(
            f"underexposed: {blk_pct:.1f}% of pixels are near-black; "
            "increase light intensities or check camera framing"
        )
    return {
        "width": width,
        "height": height,
        "pixels": n,
        "mean_brightness": round(mean, 2),
        "mean_rgb": [round(sum_r / n, 2), round(sum_g / n, 2), round(sum_b / n, 2)],
        "max_rgb": [max_r, max_g, max_b],
        "saturated_pct": round(sat_pct, 2),
        "black_pct": round(blk_pct, 2),
        "warnings": warnings,
    }


def sanitize_nonfinite(obj):
    """Recursively replace non-finite floats (NaN, +/-Infinity) with None.

    Python's json module happily emits bare ``NaN`` / ``Infinity`` tokens,
    which are NOT valid JSON (RFC 8259) — non-Python clients choke on a
    /scene/tree response containing them (the supervisor forwards whatever
    the engine reports, and uninitialised transforms can carry NaNs).
    Every HTTP response body passes through this before ``json.dumps(...,
    allow_nan=False)``, so the invariant is enforced at the boundary rather
    than at each producer.

    Dict keys are handled too: json.dumps stringifies float keys with
    ``repr``, which would still emit ``NaN`` as a key (and raises under
    allow_nan=False), so non-finite float keys become "null".
    """
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            if isinstance(k, float) and not math.isfinite(k):
                k = "null"
            out[k] = sanitize_nonfinite(v)
        return out
    if isinstance(obj, (list, tuple)):
        return [sanitize_nonfinite(v) for v in obj]
    return obj


# ---------------------------------------------------------------------------
# Conservative authored-world sync planning
# ---------------------------------------------------------------------------

# This is intentionally a lexer, not a second VRML parser.  It only needs to
# prove one narrow statement: after comments and numeric spelling are ignored,
# are the old and new files identical except for direct translation/rotation
# fields on root-level DEF nodes?  Anything it cannot prove falls back to the
# real engine parser through /world/load.
_WBT_TOKEN_RE = re.compile(
    r'#[^\r\n]*|"(?:\\.|[^"\\])*"|'
    r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?|'
    r'[A-Za-z_][A-Za-z0-9_.+:-]*|[^\s]',
    re.MULTILINE,
)
_WBT_NUMBER_RE = re.compile(
    r'^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$'
)
_POSE_FIELD_WIDTH = {"translation": 3, "rotation": 4}


def _wbt_tokens(source: str) -> list[str]:
    """Return significant WBT/VRML tokens, excluding line comments."""
    return [m.group(0) for m in _WBT_TOKEN_RE.finditer(source)
            if not m.group(0).startswith("#")]


def _root_pose_fields(source: str) -> dict:
    """Describe numeric pose fields on root-level, DEF-named scene nodes.

    The return shape is private to :func:`plan_world_sync`.  ``error`` is set
    whenever the source is ambiguous for live application; callers must then
    reload rather than guess.
    """
    tokens = _wbt_tokens(source)
    brace = 0
    bracket = 0
    depth: list[tuple[int, int]] = []
    malformed_depth = False
    for token in tokens:
        depth.append((brace, bracket))
        if token == "{":
            brace += 1
        elif token == "}":
            brace -= 1
            malformed_depth = malformed_depth or brace < 0
        elif token == "[":
            bracket += 1
        elif token == "]":
            bracket -= 1
            malformed_depth = malformed_depth or bracket < 0
    if malformed_depth or brace != 0 or bracket != 0:
        return {"tokens": tokens, "poses": {}, "mask": {},
                "error": "unbalanced braces or brackets"}

    poses: dict[str, dict] = {}
    mask: dict[int, str] = {}
    i = 0
    while i + 3 < len(tokens):
        if (depth[i] != (0, 0) or tokens[i] != "DEF"
                or not re.match(r"^[A-Za-z_]", tokens[i + 1])
                or tokens[i + 3] != "{"):
            i += 1
            continue
        def_name = tokens[i + 1]
        if def_name in poses:
            return {"tokens": tokens, "poses": poses, "mask": mask,
                    "error": f"duplicate root DEF {def_name!r}"}
        open_depth = depth[i + 3][0]
        end = i + 4
        while end < len(tokens):
            if tokens[end] == "}" and depth[end][0] == open_depth + 1:
                break
            end += 1
        if end >= len(tokens):
            return {"tokens": tokens, "poses": poses, "mask": mask,
                    "error": f"unterminated root DEF {def_name!r}"}

        node_pose: dict[str, list[float]] = {}
        j = i + 4
        while j < end:
            field = tokens[j]
            width = _POSE_FIELD_WIDTH.get(field)
            if depth[j] == (open_depth + 1, 0) and width is not None:
                value_indexes = list(range(j + 1, j + 1 + width))
                if (value_indexes[-1] >= end
                        or any(not _WBT_NUMBER_RE.match(tokens[k])
                               for k in value_indexes)):
                    # IS bindings, expressions, malformed values, etc. stay
                    # unmasked so a changed field forces a real reload.
                    j += 1
                    continue
                if field in node_pose:
                    return {"tokens": tokens, "poses": poses, "mask": mask,
                            "error": f"duplicate {field} on root DEF {def_name!r}"}
                values = [float(tokens[k]) for k in value_indexes]
                if any(not math.isfinite(v) for v in values):
                    return {"tokens": tokens, "poses": poses, "mask": mask,
                            "error": f"non-finite {field} on root DEF {def_name!r}"}
                node_pose[field] = values
                for offset, token_index in enumerate(value_indexes):
                    mask[token_index] = f"<POSE:{def_name}:{field}:{offset}>"
                j = value_indexes[-1] + 1
                continue
            j += 1
        poses[def_name] = node_pose
        i = end + 1
    return {"tokens": tokens, "poses": poses, "mask": mask, "error": None}


def plan_world_sync(previous_source: str, current_source: str) -> dict:
    """Classify an authored-world edit for the agent iteration fast path.

    ``live_pose`` is returned only after token-for-token proof that every
    semantic change is a numeric pose value on an existing root DEF.  All
    other edits return ``full_reload``.  This one-sided contract is what makes
    the endpoint safe as a default: false negatives cost time; false positives
    would run a scene different from the file.
    """
    old = _root_pose_fields(previous_source)
    new = _root_pose_fields(current_source)
    if old["error"] or new["error"]:
        return {"mode": "full_reload",
                "reason": old["error"] or new["error"], "changes": []}

    old_masked = [old["mask"].get(i, token)
                  for i, token in enumerate(old["tokens"])]
    new_masked = [new["mask"].get(i, token)
                  for i, token in enumerate(new["tokens"])]
    if old_masked != new_masked:
        return {"mode": "full_reload",
                "reason": "edit changes more than root-node pose values",
                "changes": []}

    changes: list[dict] = []
    for def_name in old["poses"]:
        old_pose = old["poses"][def_name]
        new_pose = new["poses"].get(def_name)
        if new_pose is None:
            return {"mode": "full_reload",
                    "reason": f"root DEF {def_name!r} changed or disappeared",
                    "changes": []}
        change: dict = {"def": def_name, "before": {}}
        for field in _POSE_FIELD_WIDTH:
            if old_pose.get(field) != new_pose.get(field):
                # Adding/removing a pose field changes the token skeleton and
                # was already rejected above, so both values exist here.
                change[field] = new_pose[field]
                change["before"][field] = old_pose[field]
        if len(change) > 2:
            changes.append(change)
    return {
        "mode": "live_pose" if changes else "no_change",
        "reason": ("only root-node pose values changed" if changes
                   else "authored world is semantically unchanged"),
        "changes": changes,
    }


def sibling_path_for(world: Path) -> Path:
    """Return the temp sibling path used to host the supervisor injection.

    Lives in the same directory as the original so all relative asset paths
    resolve. Leading dot keeps it conventionally hidden.
    """
    return world.with_name(f".harness_{world.name}")


def supervisor_controller_args(light: bool = False,
                               tracking: dict | None = None) -> list[str]:
    """controllerArgs for the injected supervisor.

    --light drops all three per-step trackers; the per-tracker flags
    (public issue #4) drop exactly one each. GripTracker consumes
    ContactTracker's pairs, so contacts=false implies grips off too
    (the supervisor enforces that; the flag list stays explicit).
    """
    if light:
        return ["--light"]
    t = tracking or {}
    args: list[str] = []
    if t.get("contacts") is False:
        args.append("--no-contacts")
    if t.get("joint_limits") is False:
        args.append("--no-joint-limits")
    if t.get("grips") is False:
        args.append("--no-grips")
    return args


def write_sibling_world(original: Path, light: bool = False,
                        source_text: str | None = None,
                        tracking: dict | None = None) -> Path:
    """Write a sibling copy with the generic supervisor Robot appended.

    ``source_text`` pins the exact source captured by the load/sync request so
    a second editor save during engine startup cannot make the runtime and the
    later diff baseline disagree. Returns the sibling path.

    light=True injects `controllerArgs [ "--light" ]`, which drops the
    per-step scene-graph trackers. /sim/contacts and /sim/grips then return
    empty; /sim/step gets dramatically cheaper on large scenes.
    """
    sibling = sibling_path_for(original)
    content = (source_text if source_text is not None
               else original.read_text(encoding="utf-8", errors="replace"))
    if not content.endswith("\n"):
        content += "\n"
    args = supervisor_controller_args(light, tracking)
    if not args:
        content += SUPERVISOR_INJECT_STANZA
    elif args == ["--light"]:
        content += SUPERVISOR_INJECT_STANZA_LIGHT
    else:
        quoted = " ".join(f'"{a}"' for a in args)
        content += (
            "\nRobot {\n"
            f'  name "{HARNESS_SUPERVISOR_NAME}"\n'
            f'  controller "{HARNESS_SUPERVISOR_NAME}"\n'
            f"  controllerArgs [ {quoted} ]\n"
            "  supervisor TRUE\n"
            "  synchronization FALSE\n"
            "}\n")
    sibling.write_text(content, encoding="utf-8")
    return sibling


# ---------------------------------------------------------------------------
# Supervisor IPC
# ---------------------------------------------------------------------------


class SupervisorRPCError(Exception):
    pass


class SupervisorClient:
    """Length-prefixed JSON RPC over a single persistent TCP connection.

    All sends and receives are serialized through a single lock; the harness's
    HTTP server is multithreaded so concurrent requests must not interleave on
    the wire. Each RPC blocks until the supervisor responds (or
    SUPERVISOR_RPC_TIMEOUT_S elapses), which is the right shape for a
    request/response control plane.
    """

    def __init__(self):
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._next_id = 1

    def is_connected(self) -> bool:
        return self._sock is not None

    def connect(self, host: str, port: int, deadline_s: float) -> None:
        deadline = time.time() + deadline_s
        last_exc: Exception | None = None
        while time.time() < deadline:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2.0)
                sock.connect((host, port))
                sock.settimeout(SUPERVISOR_RPC_TIMEOUT_S)
                with self._lock:
                    self._sock = sock
                return
            except OSError as exc:
                last_exc = exc
                time.sleep(0.1)
        raise SupervisorRPCError(
            f"could not connect to supervisor at {host}:{port} within {deadline_s:.1f}s "
            f"(last error: {last_exc})"
        )

    def set_rpc_timeout(self, timeout_s: float) -> None:
        """Adjust the per-RPC socket timeout on the live connection.

        Used to run the bind-probe pings with a short timeout (a connect can
        land in the supervisor's listen backlog while the engine is still
        loading; a ping there blocks until the step loop runs) and then
        restore the full RPC timeout once the supervisor is known-live.
        """
        with self._lock:
            if self._sock is not None:
                self._sock.settimeout(timeout_s)

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None

    def call(self, cmd: str, args: dict | None = None) -> dict:
        with self._lock:
            if self._sock is None:
                raise SupervisorRPCError("supervisor is not connected")
            req_id = self._next_id
            self._next_id += 1
            request = {"id": req_id, "cmd": cmd, "args": args or {}}
            payload = json.dumps(request).encode("utf-8")
            try:
                self._sock.sendall(struct.pack(">I", len(payload)) + payload)
                header = self._recv_exact(4)
                (length,) = struct.unpack(">I", header)
                if length == 0 or length > 16 * 1024 * 1024:
                    raise SupervisorRPCError(f"bad response length: {length}")
                body = self._recv_exact(length)
            except (OSError, SupervisorRPCError) as exc:
                self._drop_locked()
                raise SupervisorRPCError(f"supervisor RPC failed: {exc}") from exc
            response = json.loads(body.decode("utf-8"))
            if not response.get("ok"):
                raise SupervisorRPCError(response.get("error", "supervisor rejected request"))
            return response.get("result") or {}

    def _recv_exact(self, n: int) -> bytes:
        assert self._sock is not None
        chunks: list[bytes] = []
        remaining = n
        while remaining > 0:
            chunk = self._sock.recv(remaining)
            if not chunk:
                raise SupervisorRPCError("supervisor closed the connection")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _drop_locked(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None


# ---------------------------------------------------------------------------
# Log capture (controller stdout/stderr + omnisim_log.txt deltas)
# ---------------------------------------------------------------------------


DEFAULT_LOG_BUFFER_SIZE = 4096


class LogRingBuffer:
    """Thread-safe ring buffer of log events.

    Holds two kinds of events: `controller.log` (one per line read off
    the OmniSim subprocess's stdout/stderr) and `world.warning` /
    `world.error` (parsed from omnisim_log.txt deltas). Both share a
    monotonic seq counter so an agent driving `/sim/events` can pull
    them with one cursor.

    Each event:
        {"seq": int, "type": str, "stream"?: str, "line"?: str,
         "code"?: str, "message"?: str, "t_wall": float}
    """

    def __init__(self, maxlen: int = DEFAULT_LOG_BUFFER_SIZE):
        self._lines: collections.deque = collections.deque(maxlen=maxlen)
        self._counter = 0
        self._dropped = 0
        self._lock = threading.Lock()

    def emit_controller_log(self, stream: str, line: str) -> None:
        with self._lock:
            if len(self._lines) == self._lines.maxlen:
                self._dropped += 1
            self._counter += 1
            self._lines.append({
                "seq": self._counter,
                "type": "controller.log",
                "stream": stream,
                "line": line,
                "t_wall": time.time(),
            })

    def emit_world_diagnostic(self, diag: dict) -> None:
        # Map severity to a type. fatal/error -> world.error;
        # warning -> world.warning; `info` is still NOT streamed.
        #
        # ⚠ THE ENGINE'S PHYSICS VERDICT NOW REACHES THIS STREAM ANYWAY, and it
        # did not before. `[OmNewtonBackend] registered N dynamic + M static
        # Newton bodies` is an INFO line, and INFO was dropped TWICE (the
        # classifier had no INFO header pattern, and this method bailed on it), so
        # the one number that says whether a world is being simulated could not
        # reach an agent by any route. The fix is at the classifier: the census is
        # now matched, and `N == 0` is REFINED into `NEWTON_ZERO_DYNAMIC_BODIES`
        # at WARNING severity -- so the actionable case arrives as an ordinary
        # `world.warning` and needs no new event type (PROTOCOL.md §10.1's ten
        # types stand). The non-zero case stays out of the stream on purpose:
        # it is not actionable, and it is served on /world/diagnostics and on
        # /capabilities.physics.bodies where a number belongs.
        sev = diag.get("severity")
        if sev in ("fatal", "error"):
            type_ = "world.error"
        elif sev == "warning":
            type_ = "world.warning"
        else:
            return
        with self._lock:
            if len(self._lines) == self._lines.maxlen:
                self._dropped += 1
            self._counter += 1
            evt = {
                "seq": self._counter,
                "type": type_,
                "code": diag.get("code"),
                "message": diag.get("message"),
                "raw": diag.get("raw"),
                "t_wall": time.time(),
            }
            # Carry the branchable extras the classifier extracted (hint,
            # node_def, dynamic_bodies_registered, ...). Without this the
            # event stream flattened every diagnostic back to code+message,
            # i.e. back to prose an agent has to parse.
            for key in ("hint", "node_def", "source_path", "detail", "reading",
                        "dynamic_bodies_registered", "static_bodies_registered",
                        "coalesced", "occurrences", "suppressed_duplicates"):
                if key in diag:
                    evt[key] = diag[key]
            self._lines.append(evt)

    def since(self, since_seq: int, limit: int = 256,
              types: list[str] | None = None) -> list[dict]:
        type_set = set(types) if types is not None else None
        with self._lock:
            out: list[dict] = []
            for evt in self._lines:
                if evt["seq"] <= since_seq:
                    continue
                if type_set is not None and evt["type"] not in type_set:
                    continue
                out.append(evt)
                if len(out) >= limit:
                    break
            return out

    @property
    def total(self) -> int:
        with self._lock:
            return self._counter

    @property
    def dropped(self) -> int:
        with self._lock:
            return self._dropped


def _pump_pipe(pipe, stream_name: str, log_buffer: LogRingBuffer,
               forward_to) -> None:
    """Daemon thread body: read lines off `pipe`, push each as a
    `controller.log` event into `log_buffer`, and forward the line to
    `forward_to` (typically sys.stdout/sys.stderr) so an operator
    running the harness in a terminal still sees the live output.

    Exits cleanly on EOF, which happens when the OmniSim subprocess is
    terminated. A new thread is started for each cold launch.
    """
    try:
        for raw_line in iter(pipe.readline, b""):
            try:
                line = raw_line.decode("utf-8", errors="replace").rstrip("\r\n")
            except Exception:  # noqa: BLE001
                continue
            log_buffer.emit_controller_log(stream_name, line)
            try:
                forward_to.write(line + "\n")
                forward_to.flush()
            except Exception:  # noqa: BLE001
                pass
    finally:
        try:
            pipe.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# Harness state
# ---------------------------------------------------------------------------


class HarnessState:
    """Owns the OmniSim subprocess, sibling file lifecycle, and supervisor RPC."""

    # Class-level default so test stubs built via __new__ resolve it too;
    # __init__ shadows it per instance.
    tracking_supervisor: dict | None = None
    # Class-level defaults so a HarnessState built without __init__ (the test
    # suite's bare_state) still answers the tracking-default questions.
    default_light: bool = True
    default_light_source: str = "built-in"
    # True when the CURRENT load's tracking mode came from the default rather
    # than from an explicit `light` / `tracking` on the request; it decides
    # the wording of the tracker warning and is reported on /capabilities.
    light_default_applied: bool = False
    _light_read_warned: set | None = None

    def __init__(
        self,
        omnisim_home: Path,
        supervisor_host: str = SUPERVISOR_HOST,
        supervisor_port: int = DEFAULT_SUPERVISOR_PORT,
    ):
        self.omnisim_home = omnisim_home
        self.binary = resolve_omnisim_binary(omnisim_home)
        # Default for the injected supervisor's --light flag: LIGHT since
        # 2026-09-02 (OMNISIM_HARNESS_LIGHT=0 restores full). A load may
        # override per request; the hot-reload path reuses this value.
        self.default_light, self.default_light_source = resolve_light_default()
        self.light_supervisor = self.default_light
        self.light_default_applied = False
        self._light_read_warned = set()
        # Per-tracker toggles for the injected supervisor (public issue #4);
        # set per /world/load request, reused by hot reload and /world/sync.
        self.tracking_supervisor: dict | None = None
        # Honour OMNISIM_LOG_PATH: the engine writes its log there when set
        # (OmLog reads it as an override), so the harness must read the same
        # file or diagnostics/progress tracking silently watch the wrong
        # path in multi-instance runs.
        _log_override = os.environ.get("OMNISIM_LOG_PATH")
        self.log_path = Path(_log_override) if _log_override else omnisim_home / "omnisim_log.txt"
        self.supervisor_host = supervisor_host
        self.supervisor_port = supervisor_port
        self.lock = threading.Lock()
        self.proc: subprocess.Popen | None = None
        self.current_world: str | None = None
        self.current_sibling: Path | None = None
        # Exact authored source used for the current runtime. /world/sync
        # compares the edited file against this snapshot; it is never inferred
        # from the injected sibling, which contains harness-owned text.
        self.current_source_text: str | None = None
        self.last_load_started_at: float | None = None
        self.last_load_completed_at: float | None = None
        self.last_load_ok: bool | None = None
        self.last_load_ms: int | None = None
        self.last_diagnostics: list[dict] = []
        self.last_exit_code: int | None = None
        self.supervisor: SupervisorClient | None = None
        self.supervisor_connected_at: float | None = None
        self.started_at = time.time()
        # Phase 2: controller-log capture and world-log delta tail.
        # The buffer survives across cold launches; reader threads are
        # restarted per cold launch (the old ones drain to EOF when the
        # subprocess pipe closes). Hot reloads keep the same subprocess
        # so the existing threads stay attached.
        self.log_buffer = LogRingBuffer()
        self._stdout_thread: threading.Thread | None = None
        self._stderr_thread: threading.Thread | None = None
        self._world_log_offset = 0
        self._world_log_lock = threading.Lock()
        # Supervisor-bind lifecycle. Each cold launch gets a generation
        # number and (when supervised) a background bind-waiter thread that
        # owns the wait for the supervisor to come up. Killing the engine or
        # committing to a hot reload bumps the generation, which tells any
        # in-flight waiter it is stale (it then discards its connection
        # instead of adopting it) — one supervisor per load, stale ones
        # reaped.
        self._load_generation = 0
        self._bind_state: dict | None = None
        # Serializes /world/load requests; concurrent loads on the threaded
        # HTTP server used to race _kill_running against a live launch.
        self._load_lock = threading.Lock()
        # Last observed 3D-view size, learned from any screenshot we see. The
        # engine's `fieldOfView` is the angle on the LARGER viewport dimension
        # (VRML semantics, OmViewpoint::updateFieldOfViewY), so EVERY framing
        # and projection number depends on the real aspect ratio. Guessing
        # 16:9 is what makes a framed subject overflow the frame vertically.
        self.viewport: tuple[int, int] | None = None
        # Rolling wall-clock cost of /sim/step on the CURRENT world, published
        # by /capabilities so an agent can size a step budget against the RPC
        # timeout instead of discovering it by killing its session. Telemetry
        # only — nothing server-side branches on it, deliberately (see
        # docs/developer/agent-native-api.md §5). Cleared on every load,
        # because the cost is a property of the world + backend + light mode.
        self.step_samples: collections.deque = collections.deque(maxlen=32)
        # Which runtime-mutation verbs ("spawn" / "delete") have already put
        # their one loud world.warning into the log ring for the CURRENT
        # world-load. Cleared on every load (cold and hot), because a reload
        # rebuilds the solver model — the next mutation lies afresh and the
        # warning must fire afresh. See runtime_mutation_warning().
        self._runtime_mutation_warned: set[str] = set()

    # -- step-cost telemetry ------------------------------------------------

    def note_step(self, steps: int, wall_s: float) -> None:
        if steps > 0 and wall_s >= 0.0:
            with self.lock:
                self.step_samples.append(wall_s / float(steps))

    def step_cost(self) -> dict | None:
        """Median seconds per basic step on this world, or None if unmeasured."""
        with self.lock:
            samples = sorted(self.step_samples)
        if not samples:
            return None
        mid = len(samples) // 2
        median = samples[mid] if len(samples) % 2 else (samples[mid - 1] + samples[mid]) / 2.0
        return {
            "median_s_per_step": round(median, 6),
            "min_s_per_step": round(samples[0], 6),
            "max_s_per_step": round(samples[-1], 6),
            "samples": len(samples),
            "source": "rolling median of the last /sim/step calls on this world",
        }

    # -- runtime scene mutation disclosure ---------------------------------

    def runtime_mutation_warning(self, verb: str) -> dict:
        """The `physics_warning` block for a successful /scene/spawn or
        /scene/delete response (internal parity plan, item W1.7 honest interim).

        Every successful call gets the block; the FIRST call per verb per
        world-load additionally emits one `world.warning` into the log ring
        (so the defect reaches /sim/events and the operator terminal, not
        just the caller that already holds the field) — per-load, not
        per-request, so a spawn loop cannot flood the ring.
        """
        warning = (SPAWN_PHYSICS_WARNING if verb == "spawn"
                   else DELETE_PHYSICS_WARNING)
        with self.lock:
            first = verb not in self._runtime_mutation_warned
            self._runtime_mutation_warned.add(verb)
        if first:
            self.log_buffer.emit_world_diagnostic({
                "severity": "warning",
                "code": warning["code"],
                "message": f"POST /scene/{verb}: {warning['message']}",
            })
            print(f"[harness] WARNING {warning['code']} (POST /scene/{verb}): "
                  f"{warning['message']}", file=sys.stderr, flush=True)
        return dict(warning)

    # -- tracking mode disclosure (light is the default since 2026-09-02) ---

    def tracking_block(self, light: bool, default_applied: bool = False) -> dict:
        """The `tracking` block of a supervised load response: which mode ran,
        what it costs, and -- when the mode came from the default rather than
        the request -- `default_applied: true` plus one sentence naming how to
        get the trackers back."""
        light = bool(light)
        per_tracker = supervisor_controller_args(False, self.tracking_supervisor)
        block: dict = {
            "light": light,
            "mode": ("light" if light else ("partial" if per_tracker else "full")),
            "disabled_flags": (["--light"] if light else per_tracker),
            "default_applied": bool(default_applied),
            "hint": (
                "light mode: /sim/grips is empty and contact.*/grip.*/joint.limit_hit "
                "events are not produced; /sim/contacts still answers."
                if light else
                ("partial tracking: " + " ".join(per_tracker) + " -- the disabled "
                 "trackers' event types go quiet (GET /capabilities -> "
                 "event_types_detail.suppressed names them); /sim/contacts always "
                 "answers regardless.")
                if per_tracker else
                "FULL tracking (requested explicitly, or via OMNISIM_HARNESS_LIGHT=0; light "
                "is the default since 2026-09-02): the supervisor walks the scene every basic "
                "step for contact / grip / joint-limit events. Measured on the 10-Husky world "
                "(309 nodes, CPU mj_step, 2026-08-29): /sim/step 1 ~0.6 s vs ~0.01-0.03 s "
                "light (17x), 10 steps ~3 s vs ~0.06 s (47x). Pass {\"light\": true} unless "
                "you need /sim/grips or those events."),
        }
        if default_applied:
            block["default"] = tracking_default_block(self.default_light, self.default_light_source)
            block["default_note"] = (
                (f"{'LIGHT' if light else 'FULL'} tracking was applied BY DEFAULT "
                 f"({self.default_light_source}; light is the default since "
                 f"{LIGHT_DEFAULT_SINCE}) because the request named neither `light` nor "
                 f"`tracking`: ")
                + (LIGHT_DEFAULT_REVERT if light else
                   "pass {\"light\": true} (or unset OMNISIM_HARNESS_LIGHT) for the "
                   "2.3x cheaper light steps (fleet arena, 2026-09-02; 17-47x on the 2026-08-29 engine)"))
        return block

    def light_read_warning(self, surface: str, detail: str | None = None) -> None:
        """Emit ONE world.warning per world-load the first time a tracker-fed
        read (GET /sim/grips) answers from a session whose tracker is not
        running, naming whether the mode was the DEFAULT or requested and how
        to get the tracker back. Per-load, not per-request, so a polling
        loop cannot flood the ring."""
        with self.lock:
            if self._light_read_warned is None:
                self._light_read_warned = set()
            first = surface not in self._light_read_warned
            self._light_read_warned.add(surface)
        if not first:
            return
        if self.light_default_applied:
            how = (f"this session runs {'LIGHT' if self.light_supervisor else 'partial'} "
                   f"tracking BY DEFAULT ({self.default_light_source}; light is the default "
                   f"since {LIGHT_DEFAULT_SINCE}) -- the load request named neither `light` "
                   f"nor `tracking`")
        elif self.light_supervisor:
            how = "the load request asked for light mode ({\"light\": true})"
        else:
            how = ("the load request disabled this tracker via its `tracking` object "
                   + json.dumps(self.tracking_supervisor or {}))
        message = (
            f"{surface}: the tracker behind this read is NOT RUNNING, so its empty "
            f"answer means NOT MEASURED, not 'nothing there' -- {how}. To get it back: "
            f"{LIGHT_DEFAULT_REVERT}. GET /sim/contacts is unaffected (walked per call)."
            + (f" {detail}" if detail else ""))
        self.log_buffer.emit_world_diagnostic({
            "severity": "warning",
            "code": LIGHT_MODE_READ_CODE,
            "message": message,
        })
        print(f"[harness] WARNING {LIGHT_MODE_READ_CODE} ({surface}): {message}",
              file=sys.stderr, flush=True)

    def _kill_running(self) -> None:
        self._load_generation += 1  # invalidate any in-flight bind waiter
        if self.supervisor is not None:
            self.supervisor.close()
            self.supervisor = None
            self.supervisor_connected_at = None
        if self.proc is None:
            return
        if self.proc.poll() is None:
            try:
                # On Windows, terminating omnisim-bin leaves its python
                # controller children orphaned, and those children keep the
                # supervisor TCP port bound -- so the NEXT /world/load fails to
                # bind and reports SUPERVISOR_BIND_STALLED, a diagnostic that
                # names the symptom and not this cause. Worse, an orphaned
                # engine keeps FREE-RUNNING its world: measured 2026-08-15,
                # four of them pinned a laptop GPU at 96% and 87 C until they
                # were killed by hand. Kill the whole process tree.
                # (Ported from the capture service, which hit this first on
                # back-to-back renders: scripts/capture/omnisim_capture.py,
                # _kill_running.)
                if sys.platform == "win32":
                    try:
                        subprocess.run(
                            ["taskkill", "/F", "/T", "/PID", str(self.proc.pid)],
                            timeout=10, capture_output=True, check=False,
                        )
                    except Exception:
                        self.proc.terminate()
                else:
                    self.proc.terminate()
                try:
                    self.proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait(timeout=2)
            except Exception as exc:
                print(f"[harness] error terminating prior subprocess: {exc}", file=sys.stderr)
        self.proc = None

    def _cleanup_sibling(self) -> None:
        if self.current_sibling is not None:
            try:
                self.current_sibling.unlink()
            except OSError:
                pass
            self.current_sibling = None

    def _log_size(self) -> int:
        """Current engine-log size in bytes, or 0 when it is not there yet."""
        try:
            return self.log_path.stat().st_size
        except OSError:
            return 0

    def _log_tail_since(self, offset: int) -> str:
        """Engine-log text written after `offset` bytes.

        Used by the hot-reload path so a reload reports ITS OWN diagnostics
        instead of the whole accumulated log. A shrink (the engine truncates the
        log at cold start) resets to 0 rather than skipping past valid content --
        the same rule `_drain_world_log_into_buffer` already uses. Deliberately
        does NOT touch `_world_log_offset`: that cursor belongs to /sim/events and
        moving it here would silently eat events.
        """
        try:
            size = self.log_path.stat().st_size
        except OSError:
            return ""
        if size < offset:
            offset = 0
        if size == offset:
            return ""
        try:
            with self.log_path.open("rb") as f:
                f.seek(offset)
                return f.read().decode("utf-8", errors="replace")
        except OSError:
            return ""

    def _read_log(self) -> str:
        if not self.log_path.exists():
            return ""
        try:
            return self.log_path.read_text(errors="replace")
        except OSError as exc:
            return f"WARNING: harness could not read log: {exc}\n"

    def _drain_world_log_into_buffer(self) -> None:
        """Read any new bytes from omnisim_log.txt, parse them through
        the diagnostic classifier, and push warnings/errors into
        `log_buffer`. Called lazily on `/sim/events` so we don't pay
        the parse cost on every tick.

        File truncation (cold launch) is handled by `_world_log_offset`
        being reset there. If the file shrinks unexpectedly we reset to
        0 to avoid skipping past valid content.
        """
        if not self.log_path.exists():
            return
        with self._world_log_lock:
            try:
                size = self.log_path.stat().st_size
            except OSError:
                return
            if size < self._world_log_offset:
                self._world_log_offset = 0
            if size == self._world_log_offset:
                return
            try:
                with self.log_path.open("rb") as f:
                    f.seek(self._world_log_offset)
                    new_bytes = f.read()
                self._world_log_offset = size
            except OSError:
                return
        try:
            text = new_bytes.decode("utf-8", errors="replace")
        except Exception:  # noqa: BLE001
            return
        for diag in parse_log_lines(text):
            self.log_buffer.emit_world_diagnostic(diag)

    def _try_connect_supervisor(
        self,
        deadline: float,
        ping_timeout_s: float = SUPERVISOR_BIND_PING_TIMEOUT_S,
        stability_check: bool = False,
        poll_interval_s: float = SUPERVISOR_POLL_INTERVAL_S,
    ) -> SupervisorClient | None:
        """Poll the supervisor TCP port until it accepts a connection or the
        deadline expires. Returns a connected client, or None if the
        subprocess died or the deadline was missed.

        Pings run with a short timeout — a connect can land in the listen
        backlog of a supervisor whose step loop isn't servicing requests
        yet, and the full 30 s RPC timeout here used to park callers.

        `stability_check=True` adds a second ping after a short delay. A
        supervisor being torn down by an in-flight world swap often answers
        one final ping before its process dies; the second ping weeds out
        that dying controller (previously observed as "supervisor closed
        the connection" on the first real RPC after a hot reload).
        """
        ping_to = ping_timeout_s
        while time.time() < deadline:
            if self.proc is None or self.proc.poll() is not None:
                return None
            client = SupervisorClient()
            try:
                client.connect(self.supervisor_host, self.supervisor_port, deadline_s=0.5)
            except SupervisorRPCError:
                # No listener yet — keep the short cadence.
                client.close()
                time.sleep(poll_interval_s)
                continue
            try:
                # Note: `deadline` gates NEW connect attempts; an in-flight
                # ping is allowed its full patience even if it overshoots
                # the deadline — aborting it early guarantees failure on a
                # busy supervisor that was about to answer.
                client.set_rpc_timeout(ping_to)
                client.call("ping")
                if stability_check:
                    time.sleep(SUPERVISOR_STABILITY_RECHECK_S)
                    client.call("ping")
                client.set_rpc_timeout(SUPERVISOR_RPC_TIMEOUT_S)
                return client
            except SupervisorRPCError:
                # A supervisor is LISTENING but did not answer in time — on
                # slow platforms its main loop takes tens of seconds per
                # iteration, so escalate the ping patience instead of
                # hammering with a timeout that can never succeed.
                client.close()
                ping_to = min(ping_to * 3.0, SUPERVISOR_BIND_PING_TIMEOUT_MAX_S)
                time.sleep(SUPERVISOR_POLL_INTERVAL_S)
        return None

    @staticmethod
    def _engine_cpu_jiffies(proc: subprocess.Popen | None) -> int:
        """Cumulative CPU jiffies of the engine process (Linux only; 0
        elsewhere). The engine's step-log cadence doubles (steps 1, 30, 60,
        120, 240, ...) so log growth alone has quiet gaps of minutes on a
        slow platform — but a loading/stepping engine always accumulates
        CPU time, which makes this the reliable liveness signal.
        """
        if proc is None:
            return 0
        try:
            with open(f"/proc/{proc.pid}/stat", "rb") as f:
                raw = f.read()
            # Fields after the last ')' (comm may contain spaces/parens):
            # index 11 and 12 of the remainder are utime and stime.
            rest = raw[raw.rindex(b")") + 2:].split()
            return int(rest[11]) + int(rest[12])
        except (OSError, ValueError, IndexError):
            return 0

    def _engine_progress_marker(self, proc: subprocess.Popen | None = None) -> tuple[int, int, int]:
        """A cheap monotone fingerprint of engine liveness: (engine log
        size, total stdout/stderr lines captured, engine CPU jiffies). Any
        growth means the engine is still doing work, so the bind window
        should not expire. A wedged engine that spins CPU forever is caught
        by the hard ceiling instead.
        """
        size = 0
        try:
            size = self.log_path.stat().st_size
        except OSError:
            pass
        return (size, self.log_buffer.total, self._engine_cpu_jiffies(proc))

    def _bind_waiter(self, bind: dict, proc: subprocess.Popen) -> None:
        """Background owner of the supervisor-bind wait for one cold launch.

        Runs until one of:
          connected      — supervisor bound; adopt it into harness state.
          process_exited — the engine died; diagnostics tell the story.
          stalled        — caller patience expired AND no engine-log/stdout
                           progress for LOAD_PROGRESS_STALL_S; the engine is
                           terminated cleanly so the session is recoverable.
          ceiling        — LOAD_BIND_HARD_CEILING_S from launch; ditto.
          superseded     — a newer load/kill bumped the generation; discard.

        The foreground /world/load request waits on bind["event"] for up to
        the caller's wait_s and reports whatever state the waiter has
        reached; if the waiter is still going, the caller gets a distinct
        "load_in_progress" response and the wait continues here.
        """
        gen = bind["gen"]
        launched_at = bind["started_at"]
        patience_s = bind["patience_s"]
        hard_deadline = launched_at + LOAD_BIND_HARD_CEILING_S
        last_marker = self._engine_progress_marker(proc)
        last_progress_t = time.time()
        status = "superseded"
        detail = ""
        exit_code: int | None = None
        client: SupervisorClient | None = None
        # Ping patience escalates: short at first (fast platforms bind in
        # one round), growing toward the max when the supervisor accepts
        # connections but is too IPC-bound to answer quickly (slow
        # platforms: one main-loop iteration can take tens of seconds).
        ping_timeout_s = SUPERVISOR_BIND_PING_TIMEOUT_S
        while True:
            if gen != self._load_generation:
                status = "superseded"
                break
            ret = proc.poll()
            if ret is not None:
                status = "process_exited"
                exit_code = ret
                detail = f"simulator exited with code {ret} before the supervisor bound"
                break
            now = time.time()
            if now >= hard_deadline:
                status = "ceiling"
                detail = (
                    f"supervisor did not bind within the "
                    f"{LOAD_BIND_HARD_CEILING_S:.0f}s hard ceiling "
                    f"(engine was alive but never came up); engine terminated"
                )
                break
            marker = self._engine_progress_marker(proc)
            if marker != last_marker:
                last_marker = marker
                last_progress_t = now
            if now >= launched_at + patience_s and now - last_progress_t >= LOAD_PROGRESS_STALL_S:
                status = "stalled"
                detail = (
                    f"engine made no visible progress (log/stdout/cpu) for "
                    f"{LOAD_PROGRESS_STALL_S:.0f}s after the {patience_s:.1f}s "
                    f"wait window; engine terminated"
                )
                break
            client = self._try_connect_supervisor(
                now + 1.0,
                ping_timeout_s=min(ping_timeout_s, max(1.0, hard_deadline - now)),
            )
            if client is not None:
                status = "connected"
                break
            ping_timeout_s = min(ping_timeout_s * 3.0, SUPERVISOR_BIND_PING_TIMEOUT_MAX_S)

        elapsed = time.time() - launched_at
        if status in ("stalled", "ceiling"):
            with self.lock:
                if gen == self._load_generation and self.proc is proc:
                    print(f"[harness] bind waiter: {detail} (after {elapsed:.1f}s)",
                          file=sys.stderr)
                    self._kill_running()
                    self._cleanup_sibling()
                    self.last_diagnostics = list(self.last_diagnostics) + [{
                        "code": "SUPERVISOR_BIND_STALLED" if status == "stalled"
                                else "SUPERVISOR_BIND_CEILING",
                        "severity": "error",
                        "message": detail,
                        "raw": f"elapsed_s={elapsed:.1f}",
                    }]
                    self.last_load_ok = False
                    self.last_load_completed_at = time.time()
                    self.last_load_ms = int(elapsed * 1000)
                else:
                    status = "superseded"
        with self.lock:
            if status == "connected":
                if gen == self._load_generation and self.proc is proc:
                    self.supervisor = client
                    self.supervisor_connected_at = time.time()
                    # Record success for /sim/state pollers even if the
                    # foreground request already returned load_in_progress.
                    self.last_load_ok = True
                    self.last_load_completed_at = time.time()
                    self.last_load_ms = int(elapsed * 1000)
                    self.last_exit_code = None
                else:
                    if client is not None:
                        client.close()
                    status = "superseded"
            bind["status"] = status
            bind["detail"] = detail
            bind["exit_code"] = exit_code
            bind["elapsed_s"] = round(elapsed, 1)
        bind["event"].set()

    def _read_diagnostics(self, exit_code: int | None) -> list[dict]:
        diagnostics = parse_log_lines(self._read_log())
        # 0xC0000135 (STATUS_DLL_NOT_FOUND) is a Windows-only exit code, and
        # the msys64-DLL advice is meaningless on other platforms.
        if sys.platform == "win32" and exit_code == 3221225781 and not diagnostics:
            diagnostics.append({
                "code": "LAUNCHER_DLL_NOT_FOUND",
                "severity": "fatal",
                "message": (
                    "Simulator binary exited with STATUS_DLL_NOT_FOUND (0xC0000135). "
                    "The bundled runtime at $OMNISIM_HOME/msys64/mingw64/bin is missing required DLLs. "
                    "Add a complete msys2 mingw64 bin directory to PATH before starting the harness."
                ),
                "raw": f"exit_code={exit_code}",
            })
        elif exit_code is not None and exit_code != 0 and not any(d["severity"] in ("fatal", "error") for d in diagnostics):
            diagnostics.append({
                "code": "SIMULATOR_EXITED_NONZERO",
                "severity": "error",
                "message": f"Simulator subprocess exited with code {exit_code} but no error was logged.",
                "raw": f"exit_code={exit_code}",
            })
        return diagnostics

    def _try_hot_reload(self, world: Path, wait_s: float = HOT_RELOAD_WAIT_S,
                        source_text: str | None = None) -> dict | None:
        """Attempt to reload the simulator into the given world without
        restarting the OmniSim process. Requires an existing live supervisor.
        Returns a result dict on success, None if the path is not viable
        (caller should fall back to cold launch).
        """
        with self.lock:
            current_sup = self.supervisor
            current_proc = self.proc
            old_sibling = self.current_sibling
        if current_proc is None or current_proc.poll() is not None:
            return None
        if current_sup is None or not current_sup.is_connected():
            # The supervisor may have bound after the previous load's wait
            # window expired — adopt it now rather than falling back to a
            # cold relaunch of the whole simulator. Double-ping so we don't
            # adopt a controller that is mid-teardown.
            current_sup = self._try_connect_supervisor(
                time.time() + 2.0, stability_check=True
            )
            if current_sup is None:
                return None
            with self.lock:
                self.supervisor = current_sup
                self.supervisor_connected_at = time.time()

        started_at = time.time()
        # Where the engine log ends BEFORE the swap, so the diagnostics for this
        # load can be cut out of the delta. See the return block below for why
        # this is not optional.
        log_mark_before = self._log_size()
        try:
            new_sibling = write_sibling_world(
                world, self.light_supervisor, source_text=source_text,
                tracking=self.tracking_supervisor)
        except OSError:
            return None

        with self.lock:
            # Committing to the swap: invalidate any background bind waiter
            # still running for the previous load so it can't adopt (and
            # clobber) the supervisor we are about to negotiate.
            self._load_generation += 1
            self._bind_state = None

        try:
            current_sup.call("world_load", {"path": str(new_sibling)})
        except SupervisorRPCError:
            # The old supervisor may have already started shutting down — this
            # is benign as long as the simulator is still alive. Continue to
            # the reconnect step.
            pass
        # The world swap fires on the next sim step, after which the old
        # supervisor controller terminates and a new one starts. Drop the
        # stale connection and wait for the new one to come up.
        current_sup.close()
        with self.lock:
            self.supervisor = None
            self.supervisor_connected_at = None

        # The OLD supervisor keeps listening until the engine actually swaps
        # worlds, so an immediate reconnect can adopt the dying controller —
        # its ping still answers, and the connection then dies on the first
        # real RPC. Wait for the old listener to visibly drop first. If we
        # never see it drop (the swap can complete between polls), fall
        # through — the double-ping stability check below catches a
        # dying-controller adoption anyway.
        down_deadline = time.time() + HOT_RELOAD_LISTENER_DOWN_WAIT_S
        while time.time() < down_deadline:
            if current_proc.poll() is not None:
                return None  # engine died during the swap; cold-launch fallback
            if not _tcp_port_in_use(self.supervisor_host, self.supervisor_port,
                                    timeout=0.2):
                break
            # 20 ms, not 100. This loop is pure WAITING for an event whose real
            # timing we do not control, so the poll interval is a straight
            # quantisation error on every hot reload -- MEASURED, this phase
            # takes ~313 ms of a ~912 ms reload, so a 100 ms grid can be adding
            # up to a third of its own duration. The probe is a local TCP
            # connect against a port that is either up or down; at this rate it
            # is far cheaper than the swap it is watching.
            time.sleep(HOT_RELOAD_POLL_INTERVAL_S)

        # Scale the reconnect window with the caller's patience: a warm
        # world swap still re-parses the whole scene, which on slow
        # platforms takes longer than the old fixed 15 s.
        #
        # ⚠ THE ADOPTION MUST BE PROVEN BY A REAL RPC, NOT BY PINGS.
        # `_try_connect_supervisor(stability_check=True)` double-pings, and
        # that is still not enough: the OUTGOING supervisor keeps listening
        # until the engine actually swaps, and it answers pings the whole time
        # it is dying. The listener-down poll above misses the swap whenever it
        # completes between polls. When a corpse got adopted the harness
        # reported `load_state: "complete"` / `supervisor: "connected"` on a
        # session that had no supervisor at all -- measured 2026-08-12 as
        # "the supervisor never rebinds on the second load", with
        # `supervisor_connected: false` alongside `load_state: "complete"` and
        # wheel joints frozen at 980.14 rad for the rest of the run.
        #
        # So: keep re-binding until a candidate answers a real `sim_state`, or
        # the deadline expires. Rejected candidates are closed, not leaked.
        deadline = time.time() + max(HOT_RELOAD_WAIT_S, wait_s)
        new_client: SupervisorClient | None = None
        rejected = 0
        while time.time() < deadline:
            # stability_check=False ON PURPOSE, and only here. That flag adds a
            # fixed SUPERVISOR_STABILITY_RECHECK_S (0.4 s) sleep before a second
            # ping, to weed out an outgoing supervisor that answers one final
            # ping before dying. On THIS path that is redundant: the very next
            # line proves adoption with a real `sim_state` RPC, which is
            # strictly stronger than a second ping -- a dying listener fails it
            # and gets rejected by the loop either way. So the sleep bought
            # nothing and cost 0.4 s of every hot reload, which is ~a third of
            # the ~1.25 s floor (MEASURED: empty.wbt reloads in the same ~1.24 s
            # as a real world, i.e. the floor is overhead, not parse).
            # The other call sites keep stability_check=True -- they have no
            # follow-up RPC to lean on.
            candidate = self._try_connect_supervisor(
                deadline, stability_check=False,
                poll_interval_s=HOT_RELOAD_POLL_INTERVAL_S)
            if candidate is None:
                break
            try:
                candidate.call("sim_state")
            except SupervisorRPCError:
                # A listener that cannot answer is the outgoing supervisor.
                # Drop it and go round again — this is the rebind.
                candidate.close()
                rejected += 1
                time.sleep(SUPERVISOR_POLL_INTERVAL_S)
                continue
            new_client = candidate
            break
        load_ms = int((time.time() - started_at) * 1000)

        if new_client is None:
            # Hot reload didn't come back. The subprocess might still be alive
            # but in an inconsistent state — the safe move is to fall back to
            # a cold launch from the caller.
            if rejected:
                print(f"[harness] hot reload: rejected {rejected} supervisor "
                      f"connection(s) that answered ping but not a real RPC; "
                      f"falling back to a cold launch", file=sys.stderr, flush=True)
            return None

        with self.lock:
            self.supervisor = new_client
            self.supervisor_connected_at = time.time()
            # The new world is the new sibling; the old sibling (if different
            # name) is now stale and can be removed.
            if old_sibling is not None and old_sibling != new_sibling:
                try:
                    old_sibling.unlink()
                except OSError:
                    pass
            self.current_sibling = new_sibling
            self.current_world = str(world)
            self.step_samples.clear()  # new world -> new step cost
            self._runtime_mutation_warned.clear()  # reload rebuilt the solver
            self._light_read_warned = set()  # new load -> the tracker warning re-fires
            self.last_load_started_at = started_at
            self.last_load_completed_at = time.time()
        # ⚠ A HOT RELOAD USED TO REPORT `diagnostics: []` UNCONDITIONALLY, and
        # that is the same defect class as everything else fixed in this pass.
        # Hot reload is the loop AGENTS.md §5 tells agents to iterate in -- edit
        # the .wbt, re-POST /world/load, read the result -- so an agent doing
        # exactly what it was told got `{"ok": true, "diagnostics": []}` on a
        # world where the engine had just emitted SOLID_ODE_PIN_INERT and
        # NEWTON_ZERO_DYNAMIC_BODIES. An empty list read as "clean world"; it
        # actually meant "not collected". Only a COLD load ever reported anything.
        #
        # Fixed by classifying the log DELTA written since the swap began, which
        # is also strictly better than the cold path's whole-file read: the log is
        # truncated only at engine startup, so a whole-file parse after a reload
        # re-reports every previous world's lines as if they were this load's.
        # parse_log_lines already coalesces; do NOT wrap it again.
        diagnostics = parse_log_lines(self._log_tail_since(log_mark_before))
        fatal_or_error = any(d["severity"] in ("fatal", "error") for d in diagnostics)

        with self.lock:
            self.last_load_ok = not fatal_or_error
            self.last_load_ms = load_ms
            self.last_diagnostics = diagnostics
            self.last_exit_code = None

        return {
            "ok": not fatal_or_error,
            "world": str(world),
            "load_ms": load_ms,
            "exit_code": None,
            "diagnostics": diagnostics,
            "diagnostics_scope": ("the log delta written by THIS reload only -- not the "
                                  "whole log, which still holds every previous world's "
                                  "lines"),
            "load_state": "complete" if not fatal_or_error else "failed",
            "supervisor": "connected",
            # Proven by the real `sim_state` RPC above, not by a ping.
            "supervisor_connected": True,
            "supervisor_rebind_rejections": rejected,
            "hot_reloaded": True,
        }

    def load_world(self, world_path: str, wait_s: float, with_supervisor: bool,
                   light: bool = False, default_applied: bool = False) -> dict:
        """Launch OmniSim on the given world. When with_supervisor is true (the
        default), inject a supervisor via a sibling file and connect to it.
        Reuses the running subprocess via a hot-reload RPC when possible.

        `default_applied` says the caller named neither `light` nor
        `tracking`, so `light` is the process default (light since
        2026-09-02); the response's `tracking` block reports it.
        """
        self.light_default_applied = bool(default_applied)
        world = Path(world_path)
        if not world.is_absolute():
            world = REPO_ROOT / world
        if not world.exists():
            return {
                "ok": False,
                "error": f"world not found: {world}",
                "diagnostics": [],
                "load_ms": 0,
            }

        wait_s = max(0.1, min(MAX_LOAD_WAIT_S, float(wait_s)))

        # A supervised load of this SAME world may still be binding in the
        # background (the previous request's wait_s expired while the engine
        # was loading). Join it instead of killing the loading engine — the
        # old cold-fallback used to terminate the engine mid-GL-init, which
        # is how sessions ended up wedged (XIO / frozen log).
        if with_supervisor:
            joined = self._join_in_flight_load(world, wait_s)
            if joined is not None:
                return joined

        # Serialize concurrent /world/load requests (threaded HTTP server).
        if not self._load_lock.acquire(blocking=False):
            return {
                "ok": False,
                "error": "another /world/load is already in flight; retry when it returns",
                "load_state": "busy",
                "diagnostics": [],
                "load_ms": 0,
            }
        try:
            try:
                source_at_request = world.read_text(
                    encoding="utf-8", errors="replace")
            except OSError:
                source_at_request = None
            result = self._load_world_locked(
                world, wait_s, with_supervisor, light,
                source_text=source_at_request)
            # Public issue #4: say which tracking mode this load runs in and what
            # it costs, IN the response -- an agent that never read the docstring
            # otherwise learns about the full-mode tax only by timing out.
            if isinstance(result, dict) and with_supervisor and "error" not in result:
                result["engine_mode"] = ENGINE_MODE
                result["tracking"] = self.tracking_block(light, default_applied)
            if result.get("ok") and result.get("load_state") != "failed":
                with self.lock:
                    if self.current_world == str(world):
                        self.current_source_text = source_at_request
            return result
        finally:
            self._load_lock.release()

    def sync_world(self, world_path: str | None, wait_s: float,
                   settle_steps: int = 1, reset_physics: bool = True,
                   light: bool | None = None) -> dict:
        """Make the running scene match an edited authored world safely.

        Pose-only edits on root DEF nodes use one validated supervisor batch.
        Every other edit goes through the normal engine reload.  This is the
        agent-facing default iteration primitive: it optimizes only when the
        lexer proves equivalence outside the supported fields.
        """
        with self.lock:
            active_path = self.current_world
            previous_source = self.current_source_text
            active_light = self.light_supervisor
            client = self.supervisor
            proc_alive = self.proc is not None and self.proc.poll() is None
            load_ok = self.last_load_ok
            generation = self._load_generation
        selected = world_path or active_path
        if not isinstance(selected, str) or not selected:
            return {"ok": False, "error": "path is required when no world is loaded",
                    "mode": "rejected", "diagnostics": []}
        world = Path(selected)
        if not world.is_absolute():
            world = REPO_ROOT / world
        world = world.resolve()
        if not world.exists():
            return {"ok": False, "error": f"world not found: {world}",
                    "mode": "rejected", "diagnostics": []}
        try:
            current_source = world.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            return {"ok": False, "error": f"could not read world: {exc}",
                    "mode": "rejected", "diagnostics": []}
        try:
            settle_steps = int(settle_steps)
        except (TypeError, ValueError):
            return {"ok": False, "error": "settle_steps must be an integer",
                    "mode": "rejected", "diagnostics": []}
        if settle_steps < 0:
            return {"ok": False, "error": "settle_steps must be >= 0",
                    "mode": "rejected", "diagnostics": []}
        wait_s = max(0.1, min(MAX_LOAD_WAIT_S, float(wait_s)))
        chosen_light = active_light if light is None else bool(light)
        # No `light` on the request keeps the running mode (an edit loop must
        # not flip trackers under the caller); that mode is "the default" only
        # when nothing is loaded yet or the running mode itself came from it.
        default_applied = light is None and (active_path is None or self.light_default_applied)

        same_world = active_path is not None and Path(active_path).resolve() == world
        live_ready = (same_world and previous_source is not None and load_ok is True and proc_alive
                      and client is not None and client.is_connected()
                      and chosen_light == active_light)
        if live_ready:
            plan = plan_world_sync(previous_source, current_source)
        else:
            reasons = []
            if not same_world:
                reasons.append("different world")
            if previous_source is None:
                reasons.append("no loaded-source snapshot")
            if load_ok is not True:
                reasons.append("current load is not verified successful")
            if not proc_alive or client is None or not client.is_connected():
                reasons.append("live supervisor unavailable")
            if chosen_light != active_light:
                reasons.append("light mode changed")
            plan = {"mode": "full_reload", "changes": [],
                    "reason": ", ".join(reasons) or "live sync unavailable"}

        if not self._load_lock.acquire(blocking=False):
            return {"ok": False,
                    "error": "another world load or sync is already in flight; retry",
                    "mode": "busy", "diagnostics": []}
        started = time.perf_counter()
        try:
            with self.lock:
                state_changed = (self._load_generation != generation
                                 or self.current_world != active_path)
            if state_changed:
                plan = {"mode": "full_reload", "changes": [],
                        "reason": "running world changed while sync was being planned"}
            try:
                latest_source = world.read_text(encoding="utf-8", errors="replace")
            except OSError:
                latest_source = current_source
            if latest_source != current_source:
                current_source = latest_source
                plan = {"mode": "full_reload", "changes": [],
                        "reason": "authored file changed while sync was being planned"}
            if plan["mode"] == "no_change":
                with self.lock:
                    self.current_source_text = current_source
                return {"ok": True, "world": str(world), "mode": "no_change",
                        "fallback": False, "reason": plan["reason"],
                        "changes": [], "wall_ms": int((time.perf_counter() - started) * 1000),
                        "diagnostics": []}

            if plan["mode"] == "live_pose":
                try:
                    result = self.supervisor_call("scene_set_poses", {
                        "changes": [{key: value for key, value in change.items()
                                     if key in ("def", "translation", "rotation")}
                                    for change in plan["changes"]],
                        "reset_physics": bool(reset_physics),
                        "settle_steps": settle_steps,
                    })
                except SupervisorRPCError as exc:
                    # A transport failure leaves application state unknowable.
                    # Re-parse the authored file so runtime and disk converge.
                    plan = {"mode": "full_reload", "changes": [],
                            "reason": f"live pose batch failed ({exc}); reloading safely"}
                else:
                    wall_s = time.perf_counter() - started
                    self.note_step(settle_steps, wall_s)
                    result_changes = result.get("changes", [])
                    authored_before = {change["def"]: change.get("before", {})
                                       for change in plan["changes"]}
                    for change in result_changes:
                        change["authored_before"] = authored_before.get(
                            change.get("def"), {})
                    try:
                        after_source = world.read_text(
                            encoding="utf-8", errors="replace")
                    except OSError:
                        after_source = current_source
                    if after_source != current_source:
                        current_source = after_source
                        plan = {"mode": "full_reload", "changes": [],
                                "reason": ("authored file changed during live pose batch; "
                                           "reloading latest source")}
                    else:
                        with self.lock:
                            self.current_source_text = current_source
                        return {
                            "ok": True, "world": str(world), "mode": "live_pose",
                            "fallback": False, "reason": plan["reason"],
                            "changes": result_changes,
                            "verification": result.get("verification", {}),
                            "sim_time_ms": result.get("sim_time_ms"),
                            "settle_steps": settle_steps,
                            "wall_ms": int(wall_s * 1000), "diagnostics": [],
                        }

            # _try_hot_reload writes the injected sibling from this process-wide
            # setting; update it before entering the ordinary load path.
            self.light_supervisor = chosen_light
            self.light_default_applied = default_applied
            result = self._load_world_locked(
                world, wait_s, True, chosen_light, source_text=current_source)
            result = dict(result)
            result["mode"] = "full_reload"
            result["fallback"] = True
            result["reason"] = plan["reason"]
            result["wall_ms"] = int((time.perf_counter() - started) * 1000)
            if "error" not in result:
                # The MCP load_world tool reaches a first load THROUGH this
                # path, so the tracking disclosure must ride here too.
                result["engine_mode"] = ENGINE_MODE
                result["tracking"] = self.tracking_block(chosen_light, default_applied)
            if result.get("ok") and result.get("load_state") != "failed":
                with self.lock:
                    self.current_source_text = current_source
            return result
        finally:
            self._load_lock.release()

    def _join_in_flight_load(self, world: Path, wait_s: float) -> dict | None:
        """If a background bind wait for this same world is still running,
        block on it (up to wait_s) and report its outcome instead of
        starting a competing load."""
        with self.lock:
            bind = self._bind_state
            proc_alive = self.proc is not None and self.proc.poll() is None
        if bind is None or bind.get("status") != "waiting" or not proc_alive:
            return None
        if bind.get("world") != str(world):
            return None
        bind["event"].wait(timeout=wait_s)
        result = self._supervised_load_result(world, bind, wait_s)
        result["joined_in_flight"] = True
        return result

    def _load_world_locked(self, world: Path, wait_s: float, with_supervisor: bool,
                           light: bool = False,
                           source_text: str | None = None) -> dict:
        # Hot reload: only possible when a supervisor is already connected
        # AND the new request also wants supervisor. Otherwise fall through
        # to the cold-launch path.
        if with_supervisor:
            hot = self._try_hot_reload(world, wait_s, source_text=source_text)
            if hot is not None:
                return hot

        with self.lock:
            self._kill_running()
            self._cleanup_sibling()
            self.current_world = str(world)
            self.step_samples.clear()  # new world -> new step cost
            self._runtime_mutation_warned.clear()  # reload rebuilt the solver
            self._light_read_warned = set()  # new load -> the tracker warning re-fires
            self.last_load_started_at = time.time()
            self.last_load_completed_at = None
            self.last_load_ok = None
            self.last_diagnostics = []
            self.last_exit_code = None
            try:
                if self.log_path.exists():
                    self.log_path.unlink()
            except OSError:
                pass

            target_world = world
            if with_supervisor:
                try:
                    self.current_sibling = write_sibling_world(
                        world, light, source_text=source_text,
                        tracking=self.tracking_supervisor)
                    target_world = self.current_sibling
                except OSError as exc:
                    # Every /world/load with a supervisor -- the default, and
                    # what the MCP `load_world` tool sends -- writes
                    # .harness_<world> NEXT TO the world so its relative asset
                    # paths still resolve. The Windows installer defaults to
                    # {autopf}\OmniSim, i.e. C:\Program Files, which a normal
                    # (UAC-filtered) shell cannot write even for the admin who
                    # installed it. So the FIRST thing an agent does with a
                    # packaged install failed with a bare errno and no remedy.
                    return {
                        "ok": False,
                        "error": f"could not write supervisor sibling: {exc}",
                        "diagnostics": [{
                            "code": "WORLD_DIR_NOT_WRITABLE",
                            "severity": "fatal",
                            "message": (
                                f"cannot write the supervisor sibling next to {world}: "
                                f"{exc}. The harness needs write access to the world's "
                                "own directory. Copy the world and its project tree "
                                "somewhere writable, or reinstall OmniSim outside "
                                "C:\\Program Files -- a default Windows install is "
                                "not writable by the shell you are running in."),
                        }],
                        "load_ms": 0,
                    }

            cmd = [
                str(self.binary),
                str(target_world),
                "--batch",
                "--mode=" + ENGINE_MODE,
                "--minimize",
                "--stdout",
                "--stderr",
            ]
            # exportImage requires rendering to be enabled; only suppress
            # rendering when the supervisor is not in play.
            if not with_supervisor:
                cmd.insert(4, "--no-rendering")
            env = os.environ.copy()
            env["OMNISIM_HOME"] = str(self.omnisim_home)
            # Pin the engine's log to the file the harness watches — the
            # bind waiter's progress detection and the diagnostics reader
            # both tail self.log_path, so the two must never diverge.
            env["OMNISIM_LOG_PATH"] = str(self.log_path)
            env["OMNISIM_HARNESS_SUPERVISOR_HOST"] = self.supervisor_host
            env["OMNISIM_HARNESS_SUPERVISOR_PORT"] = str(self.supervisor_port)
            if sys.platform == "win32":
                env["PATH"] = str(self.binary.parent) + ";" + env.get("PATH", "")
            # Linux: spawning omnisim-bin directly bypasses the `webots`
            # launcher shell — supply the runtime env it would otherwise miss
            # (bundled-Qt LD_LIBRARY_PATH, QT_QPA_PLATFORM, WEBOTS_TMPDIR,
            # LIBGL_ALWAYS_SOFTWARE). No-op on other platforms.
            env = linux_runtime_env(self.omnisim_home, env)

            try:
                self.proc = subprocess.Popen(
                    cmd,
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    bufsize=1,  # line-buffered on the parent side
                )
            except OSError as exc:
                self.last_load_completed_at = time.time()
                self.last_load_ok = False
                self.last_load_ms = 0
                self._cleanup_sibling()
                return {
                    "ok": False,
                    "error": f"failed to launch simulator: {exc}",
                    "diagnostics": [],
                    "load_ms": 0,
                }
            # Spin up the stdout/stderr pumps for this cold launch.
            # Daemon threads — they exit on pipe EOF when the
            # subprocess terminates, and the harness shuts down with
            # them on process exit even if EOF hasn't arrived.
            assert self.proc.stdout is not None and self.proc.stderr is not None
            self._stdout_thread = threading.Thread(
                target=_pump_pipe,
                args=(self.proc.stdout, "stdout", self.log_buffer, sys.stdout),
                daemon=True,
                name="harness-stdout-pump",
            )
            self._stderr_thread = threading.Thread(
                target=_pump_pipe,
                args=(self.proc.stderr, "stderr", self.log_buffer, sys.stderr),
                daemon=True,
                name="harness-stderr-pump",
            )
            self._stdout_thread.start()
            self._stderr_thread.start()
            # Reset the world-log tail offset for the new process —
            # the file just got truncated above.
            with self._world_log_lock:
                self._world_log_offset = 0
            launched_at = self.last_load_started_at
            proc = self.proc
            bind: dict | None = None
            if with_supervisor:
                bind = {
                    "gen": self._load_generation,
                    "status": "waiting",
                    "world": str(world),
                    "started_at": launched_at,
                    "patience_s": wait_s,
                    "detail": "",
                    "exit_code": None,
                    "elapsed_s": None,
                    "event": threading.Event(),
                }
                self._bind_state = bind

        if with_supervisor:
            assert bind is not None
            threading.Thread(
                target=self._bind_waiter,
                args=(bind, proc),
                daemon=True,
                name="harness-supervisor-bind",
            ).start()
            bind["event"].wait(timeout=max(0.0, launched_at + wait_s - time.time()))
            return self._supervised_load_result(world, bind, wait_s)

        # Bare (no-supervisor) load: no positive signal exists; sleep until
        # deadline or subprocess exit, mirroring M0/M1 behavior.
        deadline = launched_at + wait_s
        exit_code: int | None = None
        while time.time() < deadline:
            ret = proc.poll() if proc else None
            if ret is not None:
                exit_code = ret
                break
            time.sleep(0.05)

        diagnostics = self._read_diagnostics(exit_code)
        fatal_or_error = any(d["severity"] in ("fatal", "error") for d in diagnostics)
        ok = (exit_code is None or exit_code == 0) and not fatal_or_error
        load_ms = int((time.time() - launched_at) * 1000)

        with self.lock:
            self.last_load_completed_at = time.time()
            self.last_load_ok = ok
            self.last_load_ms = load_ms
            self.last_diagnostics = diagnostics
            self.last_exit_code = exit_code

        return {
            "ok": ok,
            "world": str(world),
            "load_ms": load_ms,
            "exit_code": exit_code,
            "diagnostics": diagnostics,
            # Requested without one, so this is not a failure -- but the field
            # is present on EVERY load result so a caller can branch on one key
            # instead of parsing the prose `supervisor` string.
            "supervisor_connected": False,
            "supervisor": "not requested (with_supervisor=false)",
        }

    def _supervised_load_result(self, world: Path, bind: dict, wait_s: float) -> dict:
        """Build the /world/load response for a supervised load from the
        bind waiter's current state. Distinguishes:

          connected       -> load complete (ok per diagnostics)
          waiting         -> engine alive, still loading: report
                             load_in_progress; the background waiter keeps
                             going and the session stays recoverable
          process_exited  -> engine died: structured diagnostics
          stalled/ceiling -> the waiter terminated the engine and recorded
                             a SUPERVISOR_BIND_* diagnostic; clean failure
        """
        with self.lock:
            status = bind["status"]
            detail = bind.get("detail") or ""
            bind_exit = bind.get("exit_code")
            proc = self.proc
        launched_at = bind["started_at"]
        load_ms = int((time.time() - launched_at) * 1000)

        if status == "waiting":
            return {
                "ok": True,
                "world": str(world),
                "load_ms": load_ms,
                "exit_code": None,
                "diagnostics": [],
                "load_state": "in_progress",
                "supervisor": (
                    f"load_in_progress: engine alive and still loading after "
                    f"{wait_s:.1f}s; the harness keeps waiting in the background "
                    f"(progress-aware, hard ceiling {LOAD_BIND_HARD_CEILING_S:.0f}s). "
                    f"Poll GET /sim/state until supervisor_connected, or re-POST "
                    f"/world/load with the same path to keep blocking."
                ),
                "supervisor_connected": False,
                "hot_reloaded": False,
            }

        if status == "connected":
            exit_code = proc.poll() if proc is not None else None
            diagnostics = self._read_diagnostics(exit_code)
            fatal_or_error = any(d["severity"] in ("fatal", "error") for d in diagnostics)
            ok = (exit_code is None or exit_code == 0) and not fatal_or_error
            supervisor_status = "connected"
            # ⚠ A SUPERVISED LOAD IS NOT "complete" WITHOUT A SUPERVISOR.
            # The bind waiter's `connected` is a record of a moment that has
            # already passed: the client it adopted can be dropped by any
            # concurrent RPC, or can turn out to have been the outgoing
            # supervisor of the previous world. Re-check before claiming it.
            state_label = "complete"
            live = False
            if not ok:
                # Probe ONLY a load that otherwise looks good: an RPC against a
                # wedged engine can sit on the socket timeout, and a load we
                # have already failed has nothing to learn from it.
                state_label = "failed"
                supervisor_status = "not started (load failed)"
                with self.lock:
                    if self.supervisor is not None:
                        self.supervisor.close()
                        self.supervisor = None
                        self.supervisor_connected_at = None
            else:
                live, live_detail = self.supervisor_live_check()
                if not live:
                    ok = False
                    state_label = "bind_failed"
                    supervisor_status = (
                        "unavailable: the engine loaded but the supervisor did not "
                        f"survive the load ({live_detail or 'no supervisor connection'}). "
                        "Re-POST /world/load; the harness will cold-launch.")
            with self.lock:
                self.last_load_completed_at = time.time()
                self.last_load_ok = ok
                self.last_load_ms = load_ms
                self.last_diagnostics = diagnostics
                self.last_exit_code = exit_code
            return {
                "ok": ok,
                "world": str(world),
                "load_ms": load_ms,
                "exit_code": exit_code,
                "diagnostics": diagnostics,
                "load_state": state_label,
                "supervisor": supervisor_status,
                "supervisor_connected": live,
                "hot_reloaded": False,
            }

        if status == "process_exited":
            diagnostics = self._read_diagnostics(bind_exit)
            fatal_or_error = any(d["severity"] in ("fatal", "error") for d in diagnostics)
            ok = (bind_exit == 0) and not fatal_or_error
            with self.lock:
                self.last_load_completed_at = time.time()
                self.last_load_ok = ok
                self.last_load_ms = load_ms
                self.last_diagnostics = diagnostics
                self.last_exit_code = bind_exit
            return {
                "ok": ok,
                "world": str(world),
                "load_ms": load_ms,
                "exit_code": bind_exit,
                "diagnostics": diagnostics,
                "load_state": "failed" if not ok else "bind_failed",
                "supervisor": "not started (load failed)" if not ok
                              else f"unavailable: {detail}",
                "supervisor_connected": False,
                "hot_reloaded": False,
            }

        # stalled / ceiling / superseded: the waiter already terminated the
        # engine and recorded diagnostics (stalled/ceiling), or a newer load
        # took over (superseded).
        with self.lock:
            diagnostics = list(self.last_diagnostics)
        return {
            "ok": False,
            "world": str(world),
            "load_ms": load_ms,
            "exit_code": None,
            "diagnostics": diagnostics,
            "load_state": "bind_failed" if status in ("stalled", "ceiling")
                          else "superseded",
            "supervisor": f"unavailable: {detail}" if detail
                          else f"unavailable: {status}",
            "supervisor_connected": False,
            "hot_reloaded": False,
        }

    def diagnostics(self) -> dict:
        live = parse_log_lines(self._read_log())
        with self.lock:
            return {
                "world": self.current_world,
                "load_ok": self.last_load_ok,
                "load_ms": self.last_load_ms,
                "diagnostics": live,
            }

    def sim_state(self) -> dict:
        # The simulation CLOCK, asked for outside the lock and strictly
        # best-effort. /sim/state is the endpoint an agent polls while a load is
        # in flight, so it must never block on the simulator -- but it also used
        # to carry no sim time at all, which is why a session could read
        # "sim_time_ms: null" and conclude the clock was broken. Now it either
        # carries the engine's own clock or says, in `sim_time_source`, exactly
        # why it could not.
        sim_time_ms: float | None = None
        basic_time_step_ms: float | None = None
        sim_time_source = "supervisor not connected"
        supervisor = self.supervisor
        bind = self._bind_state
        if bind is not None and bind.get("status") == "waiting":
            sim_time_source = "world load in flight; the simulator was not asked"
        elif supervisor is not None and supervisor.is_connected():
            try:
                clock = supervisor.call("sim_state")
                sim_time_ms = clock.get("sim_time_ms")
                basic_time_step_ms = clock.get("basic_time_step_ms")
                sim_time_source = "supervisor sim_state RPC"
            except SupervisorRPCError as exc:
                sim_time_source = f"supervisor sim_state RPC failed: {exc}"

        with self.lock:
            running = self.proc is not None and self.proc.poll() is None
            exit_code = None if running else (self.proc.poll() if self.proc else None)
            supervisor_connected = self.supervisor is not None and self.supervisor.is_connected()
            bind = self._bind_state
            supervisor_bind: dict | None = None
            if bind is not None:
                elapsed = bind.get("elapsed_s")
                if elapsed is None:
                    elapsed = round(time.time() - bind["started_at"], 1)
                supervisor_bind = {
                    "status": bind["status"],
                    "world": bind["world"],
                    "elapsed_s": elapsed,
                    "detail": bind.get("detail") or "",
                }
            load_state = "idle"
            if bind is not None and bind["status"] == "waiting":
                load_state = "in_progress"
            elif self.last_load_ok is True:
                load_state = "complete"
                # ⚠ `load_state: "complete"` NEXT TO `supervisor_connected:
                # false` WAS THE MEASURED CONTRADICTION (2026-08-12). A load is
                # only complete while the thing that makes the session usable
                # is still there. `current_sibling` is the marker of a
                # SUPERVISED load — a bare `with_supervisor=false` load has no
                # supervisor by design and stays "complete".
                if self.current_sibling is not None and not supervisor_connected:
                    load_state = "supervisor_lost"
            elif self.last_load_ok is False:
                load_state = "failed"
            return {
                "world": self.current_world,
                "running": running,
                "exit_code": exit_code,
                "load_ok": self.last_load_ok,
                "load_ms": self.last_load_ms,
                "load_state": load_state,
                "load_started_at": self.last_load_started_at,
                "load_completed_at": self.last_load_completed_at,
                "supervisor_connected": supervisor_connected,
                "supervisor_connected_at": self.supervisor_connected_at,
                "supervisor_bind": supervisor_bind,
                "sim_time_ms": sim_time_ms,
                "basic_time_step_ms": basic_time_step_ms,
                "sim_time_source": sim_time_source,
                "binary": str(self.binary),
                "webots_home": str(self.omnisim_home),
            }

    def capabilities(self, handler_source: str = "", probe_step: bool = False) -> dict:
        """Assemble GET /capabilities.

        Design rule (docs/developer/agent-native-api.md P1): it must publish
        the weaknesses too, because those are what an agent has to plan
        around. Hence `not_supported` with reasons and workarounds, an honest
        `physics.source` (the engine's own verdict sidecar, not a guess), and a
        measured step cost rather than a nominal one.
        """
        state = self.sim_state()
        sup: dict | None = None
        sup_error: str | None = None
        try:
            sup = self.supervisor_call("capabilities")
        except SupervisorRPCError as exc:
            sup_error = str(exc)

        physics = read_newton_verdict(self.log_path)
        # "Which backend finalised" is not the same question as "did this world
        # get any bodies". A world can finalise on Newton and still register ZERO
        # dynamic bodies (every Solid pinned "ode", or a capability-gated
        # articulation), so the sidecar reads healthy while nothing moves. Serve
        # the engine's own census next to the verdict.
        physics["bodies"] = read_body_census(self.log_path)
        load_started_at = state.get("load_started_at")
        if not load_started_at:
            # ⚠ NO WORLD HAS BEEN LOADED IN THIS HARNESS SESSION, so NOTHING in
            # the engine log or its sidecar can describe it. Both files are
            # left behind by whatever ran last -- a `run-headless`, a previous
            # harness, another lane -- and the harness had been serving them
            # verbatim: MEASURED 2026-08-12, `/capabilities` on a harness with
            # no world loaded returned a full attestation (backend newton,
            # solver named, a 50-body census) with `source: "sidecar"` at
            # `sidecar_age_s: 85.6`. That directly contradicts the documented
            # contract -- the sidecar's presence means "Newton drove THIS run"
            # -- and voids any provenance claim built on it.
            reason = (
                "no world has been loaded in this harness session, so the engine "
                "log and its .newton.json sidecar were left by an EARLIER run "
                "(a run-headless, a previous harness, another lane). Nothing "
                "here describes this session. POST /world/load first, then read "
                "this field again.")
            if physics.get("source") in ("sidecar", "engine_log"):
                physics["stale"] = True
                physics["source"] = "sidecar_stale"
                physics["backend"] = "unverified"
                physics["detail"] = reason
            else:
                physics.setdefault("source", "sidecar_absent")
                physics["backend"] = "unverified"
                physics["detail"] = reason
            # The census comes from the same foreign log, and it is the number
            # an agent quotes as "the world IS being simulated".
            physics["bodies"] = {
                "dynamic_bodies_registered": None,
                "static_bodies_registered": None,
                "source": "no_world_loaded",
                "detail": reason,
            }
        elif "sidecar_age_s" in physics:
            # A sidecar older than the current load belongs to the PREVIOUS
            # world (the engine rewrites it per finalize, but a load that never
            # finalised leaves the old one in place). Say so instead of
            # attributing a stale verdict to this world.
            sidecar_wall = time.time() - float(physics["sidecar_age_s"])
            if sidecar_wall < float(load_started_at) - 1.0:
                physics["stale"] = True
                physics["source"] = "sidecar_stale"
                # `backend` is the field an agent branches on, so it must not
                # keep asserting "newton" off a verdict we have just said
                # belongs to a different world.
                physics["backend"] = "unverified"
                physics["detail"] = (
                    "the sidecar predates the current load, so it describes the "
                    "PREVIOUS world; treat the backend as unverified for this one")

        light = bool(sup.get("light")) if sup else bool(self.light_supervisor)
        basic_step_ms = (sup or {}).get("basic_time_step_ms")
        if basic_step_ms:
            physics["basic_time_step_ms"] = basic_step_ms

        probe: dict | None = None
        if probe_step and sup is not None:
            t0 = time.time()
            try:
                self.supervisor_call("step", {"steps": 1})
                wall = time.time() - t0
                self.note_step(1, wall)
                probe = {"steps": 1, "wall_s": round(wall, 4),
                         "note": "advanced the simulation by one basic step to measure it"}
            except SupervisorRPCError as exc:
                probe = {"error": str(exc)}

        cost = self.step_cost()
        limits: dict = {
            "supervisor_rpc_timeout_s": SUPERVISOR_RPC_TIMEOUT_S,
            "max_steps_per_request": None,
            "max_steps_per_request_note": (
                "unbounded by the harness; the real limit is time — a step "
                "request that exceeds supervisor_rpc_timeout_s drops the "
                "supervisor socket, and `step` is not auto-retried"),
            "max_load_wait_s": MAX_LOAD_WAIT_S,
            "load_bind_hard_ceiling_s": LOAD_BIND_HARD_CEILING_S,
            "events_limit_default": 256,
            "events_limit_max": 1024,
            "log_buffer_events": DEFAULT_LOG_BUFFER_SIZE,
            "step_cost": cost,
            # The tracking mode a /world/load runs in when the request names
            # neither `light` nor `tracking` -- the same block the load
            # response carries when it applied it, built by one function.
            "tracking_default": tracking_default_block(
                self.default_light, self.default_light_source),
        }
        if cost:
            budget = int(0.6 * SUPERVISOR_RPC_TIMEOUT_S / max(cost["median_s_per_step"], 1e-6))
            limits["recommended_max_steps_per_request"] = max(1, budget)
            limits["recommended_max_steps_per_request_formula"] = (
                "floor(0.6 * supervisor_rpc_timeout_s / step_cost.median_s_per_step)")
        else:
            limits["recommended_max_steps_per_request"] = None
            limits["step_cost_hint"] = (
                "no /sim/step measured on this world yet — call "
                "GET /capabilities?probe_step=1 to measure one, or POST "
                "/sim/step {\"steps\": 1} and read the cost back here")

        event_types: list[str] = list(LOG_EVENT_TYPES)
        events_detail: dict = {"harness": verify_log_event_types(_own_source())}
        if sup and isinstance(sup.get("event_types"), dict):
            sup_events = sup["event_types"]
            event_types = list(sup_events.get("types") or []) + list(LOG_EVENT_TYPES)
            events_detail["supervisor"] = sup_events
            events_detail["active"] = list(sup_events.get("active") or []) + list(LOG_EVENT_TYPES)
            events_detail["suppressed"] = list(sup_events.get("suppressed") or [])
        else:
            events_detail["supervisor"] = None
            events_detail["note"] = (
                "supervisor-side event types are served from the running "
                "supervisor (which scans its own emit() call sites); load a "
                "world with with_supervisor=true to see all ten")

        features = [
            "world.load", "world.sync", "world.hot_reload", "world.diagnostics",
            "world.screenshot", "world.render_stats",
            "scene.tree", "scene.bounds", "scene.bounds_probe", "scene.node",
            "scene.viewpoint", "scene.look_at", "scene.frame", "scene.orbit",
            "scene.visible",
            "scene.spawn", "scene.delete", "scene.set_pose",
            "sim.step", "sim.reset", "sim.snapshot", "sim.restore",
            "sim.contacts", "sim.grips", "sim.state",
            "events.cursor", "robot.joints", "robot.devices", "robot.damage",
            "capabilities",
        ]
        not_supported = [
            {"feature": "robot.sensor_read", "code": "effector_unavailable",
             "http": 501,
             "reason": "OmniSim restricts device APIs to the controller that owns the device; "
                       "the supervisor cannot honestly read a sibling robot's lidar/camera/IMU.",
             "workaround": "GET /robot/<def>/joints for kinematic state, or a Robot Bridge (PROTOCOL.md §5)."},
            {"feature": "sim.pause",
             "reason": "not implemented: Supervisor.simulationSetMode() is in the binding but not wired to HTTP.",
             "workaround": "None for holding the world still between requests: the injected supervisor is "
                           "non-synchronized, so the --mode=fast engine FREE-RUNS between your calls "
                           "(measured; see docs/developer/harness-latency-2026-07-31.md). The read "
                           "endpoints (/scene/tree, /robots, /sim/contacts, bounds) do pause the engine "
                           "internally for the duration of their walk, so each response is a consistent "
                           "single-instant snapshot -- but time keeps running between calls."},
            {"feature": "entity.velocity",
             "reason": "not implemented: Node.getVelocity()/setVelocity() are in the binding but not exposed.",
             "workaround": "Differentiate positions across two /scene/tree reads, or /robot/<def>/joints velocity."},
            {"feature": "scene.spawn_many",
             "reason": "not implemented: /scene/spawn takes one node per call (no batch form yet).",
             "workaround": "Loop /scene/spawn; each call is sub-second in light mode."},
            {"feature": "scene.spawn(urdf) / scene.spawn(type=URDFRobot)",
             "code": "SPAWN_REJECTED",
             "reason": "an engine constraint, not a harness gap: `URDFRobot { url ... }` is a "
                       "tokenizer-level SOURCE expansion applied only in OmTokenizer::tokenizeFile "
                       "(src/omnisim/vrml/OmTokenizer.cpp:412). A supervisor import goes through "
                       "tokenizeString, which never expands it, so OmParser::protoNodeList() treats "
                       "URDFRobot as a PROTO and OmNodeOperations::importNode refuses it as not "
                       "declared IMPORTABLE. The same refusal applies to any PROTO the loaded world "
                       "does not declare in its IMPORTABLE EXTERNPROTO list.",
             "workaround": "POST /scene/spawn {\"clone\": \"<DEF of an existing robot>\"} — the engine "
                           "exports the ALREADY-EXPANDED node (Node.exportString) and that imports "
                           "fine; or put the URDFRobot block in the .wbt and hot-reload."},
            {"feature": "sim.reset(preserves actuation)",
             "reason": RESET_ACTUATION_DISCLOSURE["mechanism"],
             "measured": RESET_ACTUATION_DISCLOSURE["measured"],
             "workaround": "; or ".join(RESET_ACTUATION_DISCLOSURE["workarounds"])},
            {"feature": "world.validate",
             "reason": "not implemented (agent-native-api.md P5): omniworld.validation.validate() is in-tree "
                       "and ~4 ms, but no endpoint calls it.",
             "workaround": "python scripts/dev/omniworld.py validate <world.omniworld>"},
            {"feature": "world.generate",
             "reason": "not implemented (agent-native-api.md P5): omniworld is CLI-only.",
             "workaround": "python scripts/dev/omniworld.py generate <recipe> --seed N --out <path>"},
            {"feature": "world.save",
             "reason": "not implemented: Supervisor.worldSave() is in the binding but not exposed, so a scene "
                       "composed with /scene/spawn cannot yet be persisted to a .wbt.",
             "workaround": "Compose the same scene as text and POST /world/load it."},
            {"feature": "worlds.list",
             "reason": "not implemented: no GetAvailableWorlds equivalent.",
             "workaround": "ls projects/samples/demos/worlds/ out of band."},
            {"feature": "events.streaming",
             "reason": "deliberately out of scope (PROTOCOL.md §15); the cursor-paged /sim/events with "
                       "dropped_sup / dropped_log counters covers the need.",
             "workaround": "Poll GET /sim/events with both cursors."},
        ] + ENGINE_NOT_SUPPORTED
        if light:
            # SCOPED to what --light actually breaks. This used to claim
            # sim.contacts as well and recommend a ~790x-cost reload to get it
            # back -- false, and false in the expensive direction: /sim/contacts
            # is served by observe.collect_contacts, which walks the scene per
            # call and never touches ContactTracker. Only the tracker-fed
            # surfaces are gone.
            not_supported.append({
                "feature": "sim.grips + events.contact/grip/joint",
                "reason": "the supervisor is running with --light, which does not construct the "
                          "contact, joint-limit and grip trackers (they walk the whole scene graph "
                          "every basic step). GET /sim/contacts is UNAFFECTED: it walks the scene "
                          "per call (observe.collect_contacts) and never reads a tracker, so it "
                          "answers in light mode exactly as it does in full mode.",
                "workaround": "GET /sim/grips still answers, with tracking.enabled=false so an empty "
                              "list is not mistaken for 'nothing is gripped'; derive a grip from "
                              "GET /sim/contacts (gripper subtree vs held solid), or reload with "
                              "{\"light\": false} (all trackers) or a `tracking` object (just the "
                              "ones you need) to get them back, at about 2.3x the single /sim/step cost measured 2026-09-02 (17-47x on the 2026-08-29 engine) "
                              "measured on the 309-node fleet arena. Light is the DEFAULT since "
                              "2026-09-02 (limits.tracking_default; OMNISIM_HARNESS_LIGHT=0 "
                              "restores full as the process default)."})

        return {
            "ok": True,
            "omnisim_wire": OMNISIM_WIRE_VERSION,
            "service": "world_harness",
            "sim_version": sim_version(),
            "build": {
                "commit": git_commit(REPO_ROOT),
                "binary": str(self.binary),
                "omnisim_home": str(self.omnisim_home),
                "harness_python": sys.version.split()[0],
                "pillow": _HAS_PIL,
            },
            "machine": {
                "host": socket.gethostname(),
                "platform": sys.platform,
                "note": "the canonical machine id (GPU + CPU + engine/libController hashes) comes from "
                        "python projects/policies/common/env_fingerprint.py",
            },
            "physics": physics,
            "supervisor": {
                "connected": bool(state.get("supervisor_connected")),
                "host": self.supervisor_host,
                "port": self.supervisor_port,
                "light": light,
                "light_default_applied": bool(self.light_default_applied),
                "tracking": (sup or {}).get("tracking"),
                "error": sup_error,
                "commands": (sup or {}).get("commands"),
                "commands_source": (sup or {}).get("commands_source"),
                "snapshots": (sup or {}).get("snapshots"),
            },
            "world": {
                "path": state.get("world"),
                "load_ok": state.get("load_ok"),
                "load_ms": state.get("load_ms"),
                "load_state": state.get("load_state"),
            },
            "features": features,
            "not_supported": not_supported,
            "limits": limits,
            "endpoints": [dict(r) for r in ROUTES],
            "endpoints_verification": verify_routes(handler_source),
            "event_types": event_types,
            "event_types_detail": events_detail,
            "diagnostic_codes": known_diagnostic_codes(),
            "request_error_codes": known_request_error_codes(),
            "step_probe": probe,
        }

    def supervisor_live_check(self, probe: bool = True) -> tuple[bool, str]:
        """Is there a WORKING supervisor behind this session right now?

        ⚠ WHY A REAL RPC AND NOT A PING. The old supervisor keeps LISTENING
        until the engine actually swaps worlds, and it answers `ping` while it
        is being torn down -- so neither an open socket nor a ping can tell a
        live controller from a corpse. Measured 2026-08-12: a second
        `/world/load` returned `ok` with `load_state: "complete"` while
        `/sim/state` read `supervisor_connected: false`, and the session was
        dead for the rest of the run (wheel joints frozen at 980.14 rad across
        a "successful" reload). One real RPC settles it.

        Returns (live, detail). A failed probe also CLEARS the dead client, so
        the very next `/sim/state` is honest without waiting for a caller to
        trip over it.
        """
        with self.lock:
            client = self.supervisor
        if client is None or not client.is_connected():
            return False, "no supervisor connection"
        if not probe:
            return True, ""
        try:
            client.call("sim_state")
        except SupervisorRPCError as exc:
            with self.lock:
                if self.supervisor is client:
                    self.supervisor = None
                    self.supervisor_connected_at = None
            return False, str(exc)
        return True, ""

    def supervisor_call(self, cmd: str, args: dict | None = None) -> dict:
        with self.lock:
            client = self.supervisor
            proc_alive = self.proc is not None and self.proc.poll() is None
            bind = self._bind_state
        if (client is None or not client.is_connected()) and proc_alive:
            if bind is not None and bind.get("status") == "waiting":
                # A load is still binding in the background. Don't race it
                # with our own connect (and don't block this HTTP thread on
                # a loading engine) — report the in-progress state.
                raise SupervisorRPCError(
                    "supervisor not connected yet: world load in progress "
                    "(poll GET /sim/state until supervisor_connected)"
                )
            # The supervisor may have bound after the load window expired
            # (cold CUDA/warp starts take ~20s) or a previous RPC hiccup
            # dropped the socket. One short reconnect attempt per call
            # un-bricks the session instead of 503ing forever. Double-ping
            # so we don't adopt a controller that is mid-teardown.
            client = self._try_connect_supervisor(time.time() + 2.0, stability_check=True)
            if client is not None:
                with self.lock:
                    self.supervisor = client
                    self.supervisor_connected_at = time.time()
        if client is None or not client.is_connected():
            raise SupervisorRPCError("supervisor not connected (load a world with with_supervisor=true)")
        try:
            return client.call(cmd, args)
        except SupervisorRPCError:
            # The connection may have belonged to a supervisor torn down by
            # a world swap (its socket dies on the first RPC after the
            # swap). For idempotent commands, reconnect once — with the
            # stability check — and retry transparently. Mutating commands
            # propagate the error: we can't know if they were applied.
            with self.lock:
                if self.supervisor is client:
                    self.supervisor = None
                    self.supervisor_connected_at = None
                proc_alive = self.proc is not None and self.proc.poll() is None
            if not is_retryable_supervisor_call(cmd, args) or not proc_alive:
                raise
            retry_client = self._try_connect_supervisor(
                time.time() + 5.0, stability_check=True
            )
            if retry_client is None:
                raise
            with self.lock:
                self.supervisor = retry_client
                self.supervisor_connected_at = time.time()
            return retry_client.call(cmd, args)

    # -- camera / viewport -------------------------------------------------

    def note_png_bytes(self, data: bytes) -> None:
        size = png_size(data)
        if size is not None:
            with self.lock:
                self.viewport = size

    def note_png_path(self, path: Path) -> None:
        try:
            with path.open("rb") as fh:
                head = fh.read(24)
        except OSError:
            return
        self.note_png_bytes(head)

    def note_render(self, digest: str, sim_time_ms) -> dict:
        """Record a rendered frame and report whether the buffer is moving.

        ⚠ WHY THIS EXISTS. ``/world/screenshot`` can return a perfectly valid
        PNG that is not a picture of the scene. Measured 2026-08-03: four
        screenshots taken while a cube demonstrably fell -- its pose changed at
        every step, read from the same supervisor in the same instants -- came
        back BYTE-IDENTICAL, same sha256, same 6,342,147 bytes, and the same
        digest persisted across harness restarts and separate engine processes.
        Dropping ``--minimize`` did not change it; nor did ``--mode=realtime``.
        So the frame is not a stale render of this scene, it is a buffer that
        was never rendered into, handed back with HTTP 200.

        The cost is not only wasted time. The T2 agent on 2026-08-03 took its
        proof shots this way, noticed the pixels disagreed with the pose data
        ("the live block position said 'in the bin' while the pixels said 'at
        home'"), and had to throw the frames away and redo the whole visual
        pass through the capture service -- about sixteen minutes of a
        sixty-four minute session. Had it not noticed, it would have shipped
        images that showed the opposite of what it claimed.

        So the harness now watches its own output: identical bytes while the
        simulation clock advanced means the renderer is not updating, and the
        response says so instead of implying a picture.
        """
        with self.lock:
            prev_digest = getattr(self, "_last_png_digest", None)
            prev_time = getattr(self, "_last_png_sim_ms", None)
            self._last_png_digest = digest
            self._last_png_sim_ms = sim_time_ms
            streak = getattr(self, "_png_identical_streak", 0)
            if prev_digest is not None and prev_digest == digest:
                streak += 1
            else:
                streak = 0
            self._png_identical_streak = streak

        advanced = (prev_time is not None and sim_time_ms is not None
                    and sim_time_ms > prev_time)
        stale = bool(streak and advanced)
        out = {"digest": digest, "identical_to_previous": bool(streak),
               "sim_time_ms": sim_time_ms,
               "identical_streak": streak}
        if stale:
            out["warning"] = (
                "STALE RENDER: this PNG is byte-identical to the previous one "
                "although the simulation clock advanced from %s ms to %s ms. "
                "The render buffer is not updating, so this image is NOT a "
                "picture of the current scene and must not be used as visual "
                "evidence. Use the capture service (port 6791), which renders "
                "through a Camera sensor and updates headless."
                % (prev_time, sim_time_ms))
        return out

    def ensure_viewport(self) -> tuple[tuple[int, int] | None, str]:
        """Return ``((width, height), source)`` for the live 3D view.

        Cached from any screenshot the harness has already taken; if none has
        been taken yet, one cheap off-screen render establishes it. Falls back
        to ``None`` (callers then assume ``spatial.DEFAULT_ASPECT``) if the
        supervisor is unreachable — reported honestly via ``source``.
        """
        with self.lock:
            cached = self.viewport
        if cached is not None:
            return cached, "screenshot"
        fd, name = tempfile.mkstemp(prefix="omnisim_harness_vp_", suffix=".png")
        os.close(fd)
        tmp_path = Path(name)
        try:
            self.supervisor_call("screenshot", {"path": str(tmp_path), "quality": 1})
            self.note_png_path(tmp_path)
        except SupervisorRPCError:
            return None, "unavailable"
        finally:
            try:
                tmp_path.unlink()
            except OSError:
                pass
        with self.lock:
            return self.viewport, ("screenshot" if self.viewport else "unavailable")

    def camera_context(self) -> dict:
        """Everything needed to project or re-aim: pose, FOV, aspect, axes.

        Raises SupervisorRPCError if the supervisor is not reachable.
        """
        raw = self.supervisor_call("get_viewpoint")
        position = [float(v) for v in (raw.get("position") or [0.0, 0.0, 0.0])]
        orientation = [float(v) for v in
                       (raw.get("orientation") or [0.0, 0.0, 1.0, 0.0])]
        fov = float(raw.get("fieldOfView") or spatial.DEFAULT_FOV)
        viewport, source = self.ensure_viewport()
        if viewport:
            width, height = viewport
            aspect = width / float(height)
        else:
            width = height = None
            aspect = spatial.DEFAULT_ASPECT
        forward, left, up = spatial.camera_axes(orientation)
        axes = spatial.fov_axes(fov, aspect)
        return {
            "position": position,
            "orientation": orientation,
            "field_of_view": fov,
            "near": raw.get("near"),
            "far": raw.get("far"),
            "follow": raw.get("follow"),
            "follow_type": raw.get("followType"),
            "follow_smoothness": raw.get("followSmoothness"),
            "projection_mode": raw.get("projectionMode"),
            "exposure": raw.get("exposure"),
            "forward": [round(v, 6) for v in forward],
            "left": [round(v, 6) for v in left],
            "right": [round(-v, 6) for v in left],
            "up": [round(v, 6) for v in up],
            "aspect": round(aspect, 6),
            "viewport": {"width": width, "height": height, "source": source},
            "fov_h_deg": round(axes["fov_h_deg"], 3),
            "fov_v_deg": round(axes["fov_v_deg"], 3),
            "half_fov_h_deg": round(axes["half_fov_h_deg"], 3),
            "half_fov_v_deg": round(axes["half_fov_v_deg"], 3),
            "fov_semantics": (
                "Viewpoint.fieldOfView is the angle on the LARGER viewport "
                "dimension (VRML). fov_h_deg / fov_v_deg are the resolved "
                "per-axis angles for the aspect above."
            ),
            "raw": raw,
        }

    def shutdown(self) -> None:
        with self.lock:
            self._kill_running()
            self._cleanup_sibling()


# ---------------------------------------------------------------------------
# HTTP handlers
# ---------------------------------------------------------------------------


def make_handler(state: HarnessState):
    _source_cache: dict[str, str] = {}

    def _handler_source() -> str:
        """The request handler's own source, for /capabilities' route check.

        Read once and cached; `verify_routes` scans it for path literals so a
        route that exists in dispatch but not in ROUTES cannot hide.
        """
        if "src" not in _source_cache:
            try:
                _source_cache["src"] = inspect.getsource(Handler)
            except (OSError, TypeError):
                _source_cache["src"] = ""
        return _source_cache["src"]

    class Handler(BaseHTTPRequestHandler):
        # HTTP/1.1 with keep-alive: pooled clients (the ROS bridge client,
        # the MCP wrapper, any http.client user) reuse one TCP connection
        # instead of paying connect + TIME_WAIT per request. Every response
        # path sets Content-Length (_json/_png), which keep-alive requires.
        protocol_version = "HTTP/1.1"
        # Without a timeout an idle keep-alive client parks a server thread
        # in rfile.readline() forever; socket.timeout is turned into
        # close_connection by handle_one_request.
        timeout = 30

        def log_message(self, fmt, *args):
            pass

        def do_GET(self):  # noqa: N802
            self._fenced(self._route_GET)

        def do_POST(self):  # noqa: N802
            self._fenced(self._route_POST)

        def _fenced(self, route) -> None:
            """Run a router with a last-resort exception fence.

            Under HTTP/1.0 an unhandled handler exception produced an empty
            reply (RemoteDisconnected, zero diagnostics); under keep-alive it
            would additionally desync a pooled connection. Turn it into a
            coded 500 and close this connection so the stream can never be
            left mid-body.
            """
            te = (self.headers.get("Transfer-Encoding") or "").lower()
            if "chunked" in te:
                # _read_json honours Content-Length only; silently treating a
                # chunked body as empty would also leave the chunks on the
                # wire and poison the next request on this connection.
                self.close_connection = True
                self._json(411, {"ok": False, "code": "LENGTH_REQUIRED",
                                 "error": "chunked Transfer-Encoding is not "
                                          "supported; send Content-Length"})
                return
            try:
                route()
            except Exception as exc:  # noqa: BLE001 - last-resort fence
                self.close_connection = True
                try:
                    self._json(500, {"ok": False, "code": "HARNESS_INTERNAL",
                                     "error": f"{type(exc).__name__}: {exc}",
                                     "traceback": traceback.format_exc(limit=8)})
                except Exception:  # noqa: BLE001 - headers already sent
                    pass

        def _json(self, code: int, obj: dict) -> None:
            # Strict JSON at the boundary: the supervisor can forward NaN /
            # Infinity floats (Python's json parses AND emits them, but they
            # are invalid JSON and break non-Python clients). Sanitize to
            # null first; allow_nan=False guarantees nothing slips through.
            body = json.dumps(sanitize_nonfinite(obj), allow_nan=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _png(self, body: bytes, extra_headers: dict | None = None) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(body)))
            for k, v in (extra_headers or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self) -> dict:
            n = int(self.headers.get("Content-Length") or "0")
            if n == 0:
                return {}
            raw = self.rfile.read(n).decode("utf-8") or "{}"
            return json.loads(raw)

        def _drain_body(self) -> None:
            """Read and discard any client-sent request body. Used on
            POST handlers that don't otherwise parse a body. Windows
            clients see ConnectionReset if the body is still in flight
            when the server closes the socket after responding.
            """
            n = int(self.headers.get("Content-Length") or "0")
            if n > 0:
                try:
                    self.rfile.read(n)
                except OSError:
                    pass

        def _supervisor_call(self, cmd: str, args: dict | None = None) -> dict | None:
            # Delegates to the coded classifier: a wrong DEF is the caller's
            # mistake (4xx + branchable code), not a simulator outage, and
            # every error body carries a machine-branchable `code`.
            return self._supervisor_call_coded(cmd, args)

        def _supervisor_call_coded(self, cmd: str, args: dict | None = None) -> dict | None:
            """Like _supervisor_call, but classifies the failure instead of
            calling everything a 503.

            A wrong DEF -- or a `restore` that is not a string -- is the
            caller's mistake, not the simulator being unavailable, and
            503-with-prose is the failure mode PROTOCOL.md §16 complains
            about. Used by every endpoint that forwards the caller's own
            arguments to the supervisor; see `classify_supervisor_error`.
            """
            try:
                return state.supervisor_call(cmd, args)
            except SupervisorRPCError as exc:
                msg = str(exc)
                status, code = classify_supervisor_error(msg)
                self._json(status, {"ok": False, "error": msg, "code": code})
                return None

        # -- spatial helpers ------------------------------------------------

        def _resolve_subject(self, body: dict) -> dict | None:
            """Turn a /scene/frame or /scene/orbit body into a subject.

            Accepts, in priority order:
              {"def": "HUSKY"}          one node, subtree bounds
              {"defs": ["A", "B"]}      union of several nodes' bounds
              {"target": [x,y,z], "radius": r}   explicit sphere

            Returns {"center", "radius", "bbox_min"?, "bbox_max"?, "source",
            "rotation"?, "exact"?} or None (a 4xx has already been written).
            """
            defs: list[str] = []
            if isinstance(body.get("def"), str) and body["def"]:
                defs = [body["def"]]
            elif isinstance(body.get("defs"), list):
                defs = [d for d in body["defs"] if isinstance(d, str) and d]

            if defs:
                result = self._supervisor_call("scene_bounds", {"defs": defs})
                if result is None:
                    return None
                table = result.get("bounds") or {}
                missing = [d for d in defs if d not in table]
                if missing:
                    # Only on the error path: re-query unfiltered so the agent
                    # gets a list of DEFs it CAN frame instead of an empty one.
                    try:
                        table = (state.supervisor_call("scene_bounds").get("bounds")
                                 or table)
                    except SupervisorRPCError:
                        pass
                    self._json(404, {
                        "error": f"no measurable geometry for DEF(s): {missing}",
                        "code": "SUBJECT_BOUNDS_NOT_FOUND",
                        "hint": (
                            "The DEF may not exist, or the node has no geometry "
                            "the harness can measure. GET /scene/tree?bounds=1 "
                            "lists every node that does."
                        ),
                        "available": sorted(table)[:50],
                    })
                    return None
                entries = [table[d] for d in defs]
                union = bounds_union(entries)
                if union is None:
                    self._json(404, {
                        "error": f"DEF(s) {defs} have no usable bounding box",
                        "code": "SUBJECT_BOUNDS_EMPTY",
                    })
                    return None
                union["source"] = f"bounds({', '.join(defs)})"
                # Subject-relative modes need the subject's own frame; only
                # meaningful for a single node.
                if len(defs) == 1:
                    rot = entries[0].get("orientation")
                    if rot and len(rot) == 9:
                        union["rotation"] = [float(v) for v in rot]
                return union

            target = body.get("target") or body.get("center")
            if isinstance(target, list) and len(target) == 3:
                try:
                    center = [float(v) for v in target]
                    radius = float(body.get("radius", 1.0))
                except (TypeError, ValueError):
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "target/radius must be numbers"})
                    return None
                if radius <= 0.0:
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "radius must be > 0"})
                    return None
                return {"center": center, "radius": radius,
                        "bbox_min": [center[i] - radius for i in range(3)],
                        "bbox_max": [center[i] + radius for i in range(3)],
                        "exact": True, "source": "explicit target+radius"}

            self._json(400, {
                "error": "provide one of: 'def', 'defs', or 'target' + 'radius'",
                "code": "SUBJECT_UNSPECIFIED",
            })
            return None

        def _handle_frame(self, body: dict) -> None:
            subject = self._resolve_subject(body)
            if subject is None:
                return
            try:
                camera = state.camera_context()
            except SupervisorRPCError as exc:
                self._json(503, {"ok": False, "code": "SUPERVISOR_UNAVAILABLE", "error": str(exc)})
                return
            try:
                aspect = float(body.get("aspect") or camera["aspect"])
                fov = float(body.get("fov") or camera["field_of_view"])
                margin = float(body.get("margin") or spatial.DEFAULT_MARGIN)
                radius = float(body.get("radius_override") or subject["radius"])
            except (TypeError, ValueError):
                self._json(400, {"ok": False, "code": "BAD_REQUEST",
                                 "error": "aspect/fov/margin/radius_override "
                                          "must be numbers"})
                return
            rotation = subject.get("rotation") if body.get(
                "subject_relative", True) else None
            try:
                eye, orientation, meta = spatial.frame_pose(
                    subject["center"], radius, mode=body.get("mode", "hero"),
                    fov=fov, aspect=aspect, margin=margin,
                    subject_rotation=rotation,
                )
            except ValueError as exc:
                self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": str(exc), "code": "BAD_VIEW_MODE"})
                return

            push = bool(body.get("push", True))
            pushed = False
            if push:
                if self._supervisor_call(
                    "set_viewpoint", {"position": eye, "orientation": orientation}
                ) is None:
                    return
                pushed = True

            viewport = camera["viewport"]
            width, height = viewport.get("width"), viewport.get("height")
            verification = spatial.framing_verification(
                subject["center"], radius, eye, orientation, fov, aspect,
                width, height)
            if subject.get("bbox_min") and subject.get("bbox_max"):
                verification["subject_screen_bbox"] = screen_bbox(
                    subject["bbox_min"], subject["bbox_max"], eye, orientation,
                    fov, aspect, width, height)
            self._json(200, {
                "position": [round(v, 6) for v in eye],
                "orientation": [round(v, 9) for v in orientation],
                "target": subject["center"],
                "pushed": pushed,
                # So a caller can put the camera back without a second read.
                "previous_position": camera["position"],
                "previous_orientation": camera["orientation"],
                "subject": subject,
                "framing": meta,
                "relative_to": "subject" if rotation else "world",
                "camera": {"field_of_view": fov, "aspect": aspect,
                           "viewport": viewport},
                "verification": verification,
            })

        def _handle_orbit(self, body: dict) -> None:
            try:
                camera = state.camera_context()
            except SupervisorRPCError as exc:
                self._json(503, {"ok": False, "code": "SUPERVISOR_UNAVAILABLE", "error": str(exc)})
                return
            eye = camera["position"]
            orientation = camera["orientation"]

            # The orbit centre, in priority order: explicit `center`, the
            # bounds centre of a named `def`, or a point `distance` metres
            # straight ahead (the implicit look-at point of the current view).
            center = None
            center_source = ""
            raw_center = body.get("center")
            if isinstance(raw_center, list) and len(raw_center) == 3:
                try:
                    center = [float(v) for v in raw_center]
                except (TypeError, ValueError):
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "center must be 3 numbers"})
                    return
                center_source = "explicit center"
            elif isinstance(body.get("def"), str) and body["def"]:
                subject = self._resolve_subject({"def": body["def"]})
                if subject is None:
                    return
                center = subject["center"]
                center_source = subject["source"]
            else:
                try:
                    distance = float(body.get("distance", 10.0))
                except (TypeError, ValueError):
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "distance must be a number"})
                    return
                forward = camera["forward"]
                center = [eye[i] + forward[i] * distance for i in range(3)]
                center_source = f"{distance:g} m along the current view axis"

            try:
                new_eye, new_orientation, meta = spatial.orbit_pose(
                    eye, orientation, center,
                    azimuth_deg=float(body.get("azimuth_deg", 0.0)),
                    elevation_deg=float(body.get("elevation_deg", 0.0)),
                    dolly=float(body.get("dolly", 1.0)),
                    pan=body.get("pan"),
                )
            except (TypeError, ValueError) as exc:
                self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": f"bad orbit parameter: {exc}"})
                return

            push = bool(body.get("push", True))
            pushed = False
            if push:
                if self._supervisor_call(
                    "set_viewpoint",
                    {"position": new_eye, "orientation": new_orientation},
                ) is None:
                    return
                pushed = True
            meta["center_source"] = center_source
            self._json(200, {
                "position": [round(v, 6) for v in new_eye],
                "orientation": [round(v, 9) for v in new_orientation],
                "previous_position": [round(v, 6) for v in eye],
                "previous_orientation": orientation,
                "pushed": pushed,
                "orbit": meta,
                "camera": {"field_of_view": camera["field_of_view"],
                           "aspect": camera["aspect"],
                           "viewport": camera["viewport"]},
            })

        def _handle_visible(self, qs: dict) -> None:
            try:
                camera = state.camera_context()
            except SupervisorRPCError as exc:
                self._json(503, {"ok": False, "code": "SUPERVISOR_UNAVAILABLE", "error": str(exc)})
                return
            defs_csv = qs.get("defs", [None])[0]
            wanted = [d for d in defs_csv.split(",") if d] if defs_csv else None
            include_all = parse_bool_param(qs, "all", False)
            try:
                limit = int(qs.get("limit", ["200"])[0])
            except ValueError:
                self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "'limit' must be an integer"})
                return
            limit = max(1, min(limit, 2000))

            args: dict = {}
            if wanted:
                args["defs"] = wanted
            result = self._supervisor_call("scene_bounds", args)
            if result is None:
                return
            table = result.get("bounds") or {}

            eye = camera["position"]
            orientation = camera["orientation"]
            fov = camera["field_of_view"]
            aspect = camera["aspect"]
            width = camera["viewport"].get("width")
            height = camera["viewport"].get("height")

            rows: list[dict] = []
            for name, entry in table.items():
                if wanted is None and not include_all and name.startswith("#"):
                    # DEF-less nodes are noise unless explicitly asked for.
                    continue
                center = entry.get("center")
                if not center:
                    continue
                proj = spatial.project(center, eye, orientation, fov, aspect,
                                       width, height)
                box = screen_bbox(entry["bbox_min"], entry["bbox_max"], eye,
                                  orientation, fov, aspect, width, height)
                on_screen = bool(proj["in_frame"])
                if not on_screen and box.get("ndc"):
                    nx0, ny0, nx1, ny1 = box["ndc"]
                    # Partially visible: its screen box overlaps the viewport
                    # even though the centroid does not land inside it.
                    on_screen = nx1 >= 0.0 and nx0 <= 1.0 and ny1 >= 0.0 and ny0 <= 1.0
                rows.append({
                    "def": name,
                    "type": entry.get("type"),
                    "center": center,
                    "radius": entry.get("radius"),
                    "distance": proj["distance"],
                    "in_frame": bool(proj["in_frame"]),
                    "on_screen": on_screen,
                    "behind_camera": proj["behind_camera"],
                    "centroid_ndc": (
                        None if proj["ndc_x"] is None
                        else [proj["ndc_x"], proj["ndc_y"]]
                    ),
                    "centroid_pixel": proj["pixel"],
                    "screen_bbox_ndc": box.get("ndc"),
                    "screen_bbox_pixels": box.get("pixels"),
                    "yaw_deg": proj["yaw_deg"],
                    "pitch_deg": proj["pitch_deg"],
                    "angle_off_axis_deg": proj["angle_off_axis_deg"],
                    "hint": spatial.offset_hint(
                        proj, partial=on_screen and not proj["in_frame"]),
                    "bounds_exact": entry.get("exact", True),
                })
            rows.sort(key=lambda r: (not r["on_screen"], r["distance"]))
            visible = [r for r in rows if r["on_screen"]]
            self._json(200, {
                "camera": {
                    "position": eye,
                    "orientation": orientation,
                    "field_of_view": fov,
                    "aspect": aspect,
                    "viewport": camera["viewport"],
                    "half_fov_h_deg": camera["half_fov_h_deg"],
                    "half_fov_v_deg": camera["half_fov_v_deg"],
                    "forward": camera["forward"],
                },
                "nodes": rows[:limit],
                "counts": {"considered": len(rows), "on_screen": len(visible),
                           "returned": min(len(rows), limit)},
                "pixel_basis": camera["viewport"]["source"],
            })

        def _route_GET(self):  # noqa: N802
            path = self.path
            if path == "/healthz":
                self._json(200, {"ok": True, "uptime_s": time.time() - state.started_at})
                return
            if urlparse(path).path == "/capabilities":
                qs = parse_qs(urlparse(path).query)
                self._json(200, state.capabilities(
                    handler_source=_handler_source(),
                    probe_step=parse_bool_param(qs, "probe_step", False),
                ))
                return
            if path == "/sim/snapshots":
                result = self._supervisor_call("sim_snapshots")
                if result is not None:
                    self._json(200, result)
                return
            if path == "/world/diagnostics":
                self._json(200, state.diagnostics())
                return
            if path == "/sim/state":
                self._json(200, state.sim_state())
                return
            parsed_early = urlparse(path)
            if parsed_early.path == "/scene/tree":
                # `bounds=1` attaches per-node world-space geometric bounds.
                # Opt-in: it walks every geometry node (and reads mesh files
                # off disk, cached), so the default stays a cheap pose dump.
                qs = parse_qs(parsed_early.query)
                want_bounds = parse_bool_param(qs, "bounds", False)
                result = self._supervisor_call(
                    "scene_tree", {"bounds": True} if want_bounds else {}
                )
                if result is not None:
                    # Same problem as /robots, opposite remedy: the tree is a
                    # dump of what is IN the scene, so the injected supervisor
                    # must stay in it -- but it must be LABELLED, because it is
                    # not in the .wbt the agent is authoring and it is the
                    # harness that put it there.
                    nodes = list(result.get("nodes") or [])
                    injected = [n.get("name") for n in nodes if is_harness_injected(n)]
                    if injected:
                        nodes = [dict(n, harness_injected=True)
                                 if is_harness_injected(n) else n for n in nodes]
                        result = dict(result)
                        result["nodes"] = nodes
                    else:
                        result = dict(result)
                    result["harness_injected"] = injected
                    self._json(200, result)
                return

            if parsed_early.path == "/scene/viewpoint":
                try:
                    self._json(200, state.camera_context())
                except SupervisorRPCError as exc:
                    self._json(503, {"ok": False, "code": "SUPERVISOR_UNAVAILABLE", "error": str(exc)})
                return

            if parsed_early.path == "/scene/visible":
                self._handle_visible(parse_qs(parsed_early.query))
                return
            if path == "/world/render_stats":
                if not _HAS_PIL:
                    self._json(503, {
                        "ok": False,
                        "code": "PILLOW_MISSING",
                        "error": "Pillow is not installed; render stats require it. "
                                 "pip install Pillow (an install gap on the "
                                 "harness host, not a simulator outage)"
                    })
                    return
                fd, name = tempfile.mkstemp(prefix="omnisim_harness_stats_", suffix=".png")
                os.close(fd)
                tmp_path = Path(name)
                try:
                    result = self._supervisor_call(
                        "screenshot", {"path": str(tmp_path), "quality": 100}
                    )
                    if result is None:
                        return
                    image_bytes = tmp_path.read_bytes()
                finally:
                    try:
                        tmp_path.unlink()
                    except OSError:
                        pass
                state.note_png_bytes(image_bytes)
                try:
                    stats = compute_render_stats(image_bytes)
                except RuntimeError as exc:
                    self._json(503, {"ok": False, "code": "SUPERVISOR_UNAVAILABLE", "error": str(exc)})
                    return
                self._json(200, stats)
                return
            if parsed_early.path.startswith("/scene/node/"):
                # /scene/node/<def>/particles — matched by path SEGMENT (the
                # /robot/<def>/joints convention) BEFORE the generic
                # /scene/node/<def> handler below, which would otherwise
                # swallow "<def>/particles" as a DEF.
                _pparts = parsed_early.path[1:].split("/")
                if len(_pparts) == 4 and _pparts[3] == "particles":
                    def_name = _pparts[2]
                    if not def_name:
                        self._json(400, {"ok": False, "code": "BAD_REQUEST",
                                         "error": "missing DEF in /scene/node/<def>/particles"})
                        return
                    qs = parse_qs(parsed_early.query)
                    sample = 0
                    if "sample" in qs:
                        try:
                            sample = int(qs["sample"][0])
                        except ValueError:
                            self._json(400, {"ok": False, "code": "BAD_REQUEST",
                                             "error": "'sample' must be an integer stride "
                                                      "(0 = stats only)"})
                            return
                    t0 = time.perf_counter()
                    # Coded: an unknown DEF is a 404 and a non-particle node a
                    # 4xx, not a simulator outage.
                    result = self._supervisor_call_coded(
                        "particle_stats", {"def": def_name, "sample_stride": sample})
                    if result is None:
                        return
                    result["rpc_ms"] = round((time.perf_counter() - t0) * 1000.0, 1)
                    self._json(200, result)
                    return
            if parsed_early.path.startswith("/scene/node/"):
                def_name = parsed_early.path[len("/scene/node/"):]
                if not def_name:
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "missing DEF in /scene/node/<def>"})
                    return
                qs = parse_qs(parsed_early.query)
                args: dict = {"def": def_name}
                if parse_bool_param(qs, "bounds", False):
                    args["bounds"] = True
                # Coded: an unknown DEF is a 404, not a simulator outage.
                result = self._supervisor_call_coded("scene_node", args)
                if result is None:
                    return
                if parse_bool_param(qs, "probe", False):
                    # Exactness oracle: invert the engine's own
                    # moveViewpoint bounding-sphere fit. SLOW (seconds) and
                    # it steps the sim, hence opt-in.
                    viewport, _src = state.ensure_viewport()
                    aspect = (viewport[0] / float(viewport[1])) if viewport else 1.0
                    try:
                        result["bounds_probe"] = state.supervisor_call(
                            "bounds_probe", {"def": def_name, "aspect": aspect}
                        )
                    except SupervisorRPCError as exc:
                        result["bounds_probe"] = {"error": str(exc)}
                self._json(200, result)
                return

            # Strip query string for path-only routing on the damage
            # endpoints; query params are read separately below.
            parsed = urlparse(path)
            base = parsed.path

            if base == "/robot/damage":
                result = self._supervisor_call("damage_state")
                if result is not None:
                    self._json(200, result)
                return

            if base == "/robot/damage/events":
                qs = parse_qs(parsed.query)
                args: dict = {}
                if "since" in qs:
                    try:
                        args["since"] = int(qs["since"][0])
                    except ValueError:
                        self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "'since' must be an integer"})
                        return
                if "limit" in qs:
                    try:
                        args["limit"] = int(qs["limit"][0])
                    except ValueError:
                        self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "'limit' must be an integer"})
                        return
                result = self._supervisor_call("damage_events", args)
                if result is not None:
                    self._json(200, result)
                return

            if base == "/robots":
                result = self._supervisor_call("robots_list")
                if result is not None:
                    # ⚠ THE HARNESS'S OWN SUPERVISOR IS NOT ONE OF THE USER'S
                    # ROBOTS. It is a Robot node the harness injects into a
                    # sibling copy of the world, so it appears in the engine's
                    # roster -- and an agent asserting "exactly 10 robots" on a
                    # 10-robot world read 11 (measured in 2 of 3 cells,
                    # 2026-08-12). Excluded by default; `?include_harness=1`
                    # puts it back, marked. Never excluded SILENTLY: the count
                    # that changed is named in `harness_injected`.
                    qs = parse_qs(parsed.query)
                    include = parse_bool_param(qs, "include_harness", False)
                    robots = list(result.get("robots") or [])
                    injected = [r.get("name") for r in robots if is_harness_injected(r)]
                    if include:
                        robots = [dict(r, harness_injected=True) if is_harness_injected(r)
                                  else r for r in robots]
                    else:
                        robots = [r for r in robots if not is_harness_injected(r)]
                    result = dict(result)
                    result["robots"] = robots
                    result["harness_injected"] = injected
                    result["harness_injected_note"] = (
                        "the harness injects one supervisor Robot ("
                        f"name {HARNESS_SUPERVISOR_NAME!r}) into a sibling copy of every "
                        "world it loads; it is NOT in your .wbt. "
                        + ("It is listed above and flagged `harness_injected`."
                           if include else
                           "It is excluded from `robots` above; pass "
                           "?include_harness=1 to see it."))
                    self._json(200, result)
                return

            if base == "/sim/contacts":
                # ?wake=1 is a documented NO-OP kept so old callers do not 400
                # (there is no body sleep in this engine and the field it used to
                # write has no reader). The response always carries `tracking`,
                # which enumerates the REAL reasons an empty set can be empty and
                # never claims the list is complete.
                qs = parse_qs(parsed.query)
                args = {}
                if qs.get("wake", ["0"])[0] not in ("0", "", "false", "no"):
                    args["wake"] = True
                    if "settle_steps" in qs:
                        try:
                            args["settle_steps"] = int(qs["settle_steps"][0])
                        except ValueError:
                            self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "'settle_steps' must be an integer"})
                            return
                # wake=1 advances the sim, so it is NOT the idempotent read the
                # transparent-retry list assumes; _supervisor_call is still the
                # right transport, the mutation is simply declared in the reply.
                result = self._supervisor_call("sim_contacts", args)
                if result is not None:
                    self._json(200, result)
                return

            if base == "/sim/grips":
                result = self._supervisor_call("sim_grips")
                if result is not None:
                    # A grips answer from a session with no GripTracker is
                    # NOT MEASURED; say so once per load on the event stream,
                    # naming whether light was the default or requested.
                    tracking = result.get("tracking") if isinstance(result, dict) else None
                    if isinstance(tracking, dict) and tracking.get("enabled") is False:
                        warn = getattr(state, "light_read_warning", None)
                        if warn is not None:
                            warn("GET /sim/grips")
                    self._json(200, result)
                return

            if base == "/sim/events":
                # Composer: pulls supervisor-side events via the
                # events_drain RPC and merges in harness-side log
                # events. Two cursors so callers can keep them in sync
                # independently.
                qs = parse_qs(parsed.query)
                try:
                    sup_since = int(qs.get("since", ["0"])[0])
                    log_since = int(qs.get("log_since", ["0"])[0])
                    limit = int(qs.get("limit", ["256"])[0])
                except ValueError:
                    self._json(400, {
                        "ok": False,
                        "code": "BAD_REQUEST",
                        "error": "since/log_since/limit must be integers",
                    })
                    return
                if limit < 1:
                    limit = 1
                if limit > 1024:
                    limit = 1024
                types_csv = qs.get("types", [None])[0]
                types_list = [t for t in types_csv.split(",")
                              if t] if types_csv else None
                # Always tail the world log first so any error/warning
                # written since the last call shows up in this batch.
                state._drain_world_log_into_buffer()
                # Supervisor side (best-effort: if not connected, we
                # still return harness-side log events).
                sup_args: dict = {"since": sup_since, "limit": limit}
                if types_list is not None:
                    sup_args["types"] = types_list
                sup_result: dict = {
                    "events": [], "next_seq": sup_since,
                    "total": 0, "dropped": 0, "buffered": 0,
                }
                try:
                    sup_result = state.supervisor_call(
                        "events_drain", sup_args
                    )
                except SupervisorRPCError:
                    pass
                log_events = state.log_buffer.since(
                    log_since, limit, types=types_list
                )
                events: list[dict] = []
                for evt in sup_result.get("events", []):
                    e = dict(evt)
                    e["source"] = "sup"
                    events.append(e)
                for evt in log_events:
                    e = dict(evt)
                    e["source"] = "log"
                    events.append(e)
                next_log_since = (
                    log_events[-1]["seq"] if log_events else log_since
                )
                payload = {
                    "events": events,
                    "next_since": sup_result.get("next_seq", sup_since),
                    "next_log_since": next_log_since,
                    "dropped_sup": sup_result.get("dropped", 0),
                    "dropped_log": state.log_buffer.dropped,
                }
                if types_list:
                    # The filter is an exact-match allowlist; an unknown or
                    # suppressed type yields an empty stream, not an error.
                    # Name the misses so a silent empty result explains
                    # itself. Supervisor types come from the RPC result when
                    # connected (event_bus.SUPERVISOR_EVENT_TYPES); offline,
                    # only log-side types can be checked.
                    known = set(sup_result.get("types") or ()) | set(
                        LOG_EVENT_TYPES)
                    if known - set(LOG_EVENT_TYPES):
                        unmatched = [t for t in types_list if t not in known]
                        if unmatched:
                            payload["unmatched_types"] = unmatched
                self._json(200, payload)
                return

            # /robot/<def>/joints, /robot/<def>/devices,
            # /robot/<def>/sensor/<name> — split AFTER /robot/damage
            # routes have had their chance to match.
            if base.startswith("/robot/"):
                parts = base[1:].split("/")
                if len(parts) >= 3:
                    def_name = parts[1]
                    suffix = parts[2]
                    if suffix == "joints":
                        # Coded: a DEF that is not in the scene is the
                        # caller's mistake (404), not a simulator outage.
                        result = self._supervisor_call_coded(
                            "robot_joints", {"def": def_name}
                        )
                        if result is not None:
                            self._json(200, result)
                        return
                    if suffix == "devices":
                        result = self._supervisor_call_coded(
                            "robot_devices", {"def": def_name}
                        )
                        if result is not None:
                            self._json(200, result)
                        return
                    if suffix == "sensor" and len(parts) >= 4:
                        # The supervisor cannot read live sensor values
                        # from devices owned by other robots' controllers.
                        # Return 501 with the workaround so the agent
                        # can branch deterministically rather than
                        # eyeball-debugging an empty payload. /joints
                        # already covers the common case (joint
                        # positions == PositionSensor values).
                        sensor_name = "/".join(parts[3:])
                        self._json(501, {
                            "ok": False,
                            "code": "SENSOR_NOT_SUPERVISOR_READABLE",
                            "error": (
                                "live sensor reads not supported from the "
                                "supervisor (OmniSim restricts device APIs "
                                "to the controller that owns the device). "
                                "Use /robot/<def>/joints for joint positions, "
                                "or run a per-robot helper controller that "
                                "exports its sensor data over its own "
                                "endpoint."
                            ),
                            "robot": def_name,
                            "sensor": sensor_name,
                        })
                        return

            if path.startswith("/debug/read_bench"):
                # Diagnostic (declared in ROUTES since 2026-09-01; PROTOCOL.md
                # §7.35): measures the per-read cost of
                # one supervisor getter, free-running vs paused.
                qs = parse_qs(urlparse(path).query)
                n = int(qs.get("n", ["50"])[0])
                result = self._supervisor_call("diag_read_bench", {"n": n})
                if result is not None:
                    self._json(200, result)
                return

            self._json(404, {"ok": False, "code": "UNKNOWN_ROUTE",
                             "error": f"not found: {path}"})

        def _route_POST(self):  # noqa: N802
            path = self.path
            if path == "/world/sync":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                world_path = body.get("path")
                if world_path is not None and (not isinstance(world_path, str)
                                               or not world_path):
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "path must be a non-empty string"})
                    return
                try:
                    wait_s = float(body.get("wait_s", DEFAULT_LOAD_WAIT_S))
                except (TypeError, ValueError):
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "wait_s must be a number"})
                    return
                light = body.get("light")
                result = state.sync_world(
                    world_path, wait_s,
                    settle_steps=body.get("settle_steps", 1),
                    reset_physics=bool(body.get("reset_physics", True)),
                    light=None if light is None else bool(light),
                )
                status = 200 if result.get("ok") else (
                    409 if result.get("mode") == "busy" else 422)
                self._json(status, result)
                return

            if path == "/world/load":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                world_path = body.get("path")
                if not isinstance(world_path, str) or not world_path:
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "path is required"})
                    return
                with_supervisor = bool(body.get("with_supervisor", True))
                default_wait = DEFAULT_LOAD_WAIT_S if with_supervisor else DEFAULT_LOAD_WAIT_BARE_S
                wait_s = body.get("wait_s", default_wait)
                try:
                    wait_s = float(wait_s)
                except (TypeError, ValueError):
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "wait_s must be a number"})
                    return
                tracking = body.get("tracking")
                if tracking is not None:
                    if (not isinstance(tracking, dict)
                            or not set(tracking) <= {"contacts", "joint_limits", "grips"}
                            or not all(isinstance(v, bool) for v in tracking.values())):
                        self._json(400, {
                            "ok": False, "code": "BAD_REQUEST",
                            "error": ("'tracking' must be a dict with boolean "
                                      "values for any of: contacts, "
                                      "joint_limits, grips")})
                        return
                # Tracking mode (dual contract, light default since 2026-09-02):
                # an explicit `light` wins; a `tracking` object with no `light`
                # is partial mode (light=false + per-tracker flags); neither
                # named -> the process default (state.default_light, i.e.
                # OMNISIM_HARNESS_LIGHT: unset/1 -> light, 0 -> full), and the
                # response says so via tracking.default_applied.
                light_raw = body.get("light")
                if light_raw is not None:
                    light, default_applied = bool(light_raw), False
                elif tracking is not None:
                    light, default_applied = False, False
                else:
                    light, default_applied = bool(state.default_light), True
                state.light_supervisor = light
                state.tracking_supervisor = tracking
                result = state.load_world(world_path, wait_s, with_supervisor, light,
                                          default_applied=default_applied)
                self._json(200 if result.get("ok") else 422, result)
                return

            if path == "/sim/rebuild_physics":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                log_mark = state.log_buffer.total
                rb = self._supervisor_call_coded(
                    "rebuild_physics",
                    {"settle_steps": body.get("settle_steps", 8)})
                if rb is None:
                    return
                # Surface any refusal WARNING the engine logged during the
                # settle window, so the caller does not have to poll events.
                state._drain_world_log_into_buffer()
                refusals = [e.get("message") for e in state.log_buffer.since(
                            log_mark, 1024, types=["world.warning"])
                            if "physics rebuild REFUSED" in (e.get("message") or "")]
                out = {"ok": True, **rb}
                if refusals:
                    out["ok"] = False
                    out["code"] = "REBUILD_REFUSED"
                    out["error"] = refusals[-1]
                self._json(200 if out["ok"] else 409, out)
                return

            if path == "/sim/step":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                try:
                    steps = int(body.get("steps", 1))
                except (TypeError, ValueError):
                    self._json(400, {"ok": False, "code": "BAD_REQUEST",
                                     "error": "steps must be an integer"})
                    return
                if steps < 1:
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "steps must be >= 1"})
                    return
                t0 = time.time()
                result = self._supervisor_call("step", {"steps": steps})
                if result is not None:
                    wall = time.time() - t0
                    # Feed /capabilities' measured step cost so the NEXT budget
                    # decision is informed by this call.
                    state.note_step(steps, wall)
                    result = dict(result)
                    result["wall_ms"] = int(wall * 1000)
                    result["steps_executed"] = steps
                    self._json(200, result)
                return

            if path == "/sim/reset":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                args: dict = {}
                if "restore" in body:
                    args["restore"] = body["restore"]
                if "verify" in body:
                    args["verify"] = bool(body["verify"])
                if "settle_steps" in body:
                    try:
                        args["settle_steps"] = int(body["settle_steps"])
                    except (TypeError, ValueError):
                        self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "'settle_steps' must be an integer"})
                        return
                # Coded: `{"restore": 7}` is the caller's mistake, and it used
                # to come back as a 503 that urllib/requests RAISE on, so the
                # agent saw a server outage and retried (measured 2026-08-12).
                result = self._supervisor_call_coded("reset", args)
                if result is None:
                    return
                # A reset is only meaningful if the supervisor survived it.
                live, live_detail = state.supervisor_live_check()
                if not live:
                    self._json(503, {
                        "ok": False,
                        "code": "SUPERVISOR_LOST",
                        "error": (
                            "the reset RPC returned but the supervisor is no longer "
                            f"answering ({live_detail or 'no supervisor connection'}), "
                            "so nothing about the scene state can be attested. "
                            "Re-POST /world/load."),
                        "supervisor_connected": False,
                    })
                    return
                result = dict(result)
                result["supervisor_connected"] = True
                # ⚠ THE RESET SILENCES THE SCENE. See RESET_ACTUATION_DISCLOSURE
                # for the mechanism and the measurement; a bare success here is
                # what let an agent conclude the physics was broken.
                result["actuation"] = dict(RESET_ACTUATION_DISCLOSURE)
                prior = result.get("warning")
                result["warning"] = (f"{prior} | {RESET_ACTUATION_WARNING}"
                                     if prior else RESET_ACTUATION_WARNING)
                self._json(200, result)
                return

            if path in ("/sim/snapshot", "/sim/restore"):
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                name = body.get("name", "default")
                if not isinstance(name, str) or not name:
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "'name' must be a non-empty string"})
                    return
                args = {"name": name}
                if "settle_steps" in body:
                    try:
                        args["settle_steps"] = int(body["settle_steps"])
                    except (TypeError, ValueError):
                        self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "'settle_steps' must be an integer"})
                        return
                cmd = "sim_snapshot" if path == "/sim/snapshot" else "sim_restore"
                result = self._supervisor_call_coded(cmd, args)
                if result is not None:
                    self._json(200, result)
                return

            if path == "/scene/spawn":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                vrml = ""
                if isinstance(body.get("clone"), str) and body["clone"]:
                    # Clone path: the VRML comes from the engine
                    # (Node.exportString), not from this process.
                    args = {"clone": body["clone"]}
                    if isinstance(body.get("def"), str) and body["def"]:
                        args["def"] = body["def"]
                    for key in ("translation", "rotation", "name", "parent",
                                "index", "settle_steps", "reset_physics"):
                        if key in body:
                            args[key] = body[key]
                else:
                    try:
                        vrml, def_name = compose_spawn_vrml(body, REPO_ROOT)
                    except ValueError as exc:
                        self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": str(exc), "code": "SPAWN_SPEC_INVALID"})
                        return
                    args = {"vrml": vrml}
                    if def_name:
                        args["def"] = def_name
                    for key in ("parent", "index", "settle_steps", "name"):
                        if key in body:
                            args[key] = body[key]
                log_mark = state.log_buffer.total
                try:
                    result = state.supervisor_call("scene_spawn", args)
                except SupervisorRPCError as exc:
                    # A bad DEF (parent / clone source) is a plain 404 with a
                    # code, not a rejected-VRML report.
                    for needle, status, code in SUPERVISOR_ERROR_CODE_MAP:
                        if needle in str(exc):
                            self._json(status, {"ok": False, "error": str(exc),
                                                "code": code})
                            return
                    # A rejected node string is the common failure, and the
                    # actionable detail is the engine's parse error — which
                    # lands in the engine log, not in the RPC reply. Fetch it
                    # so the agent gets the reason AND the exact text that was
                    # rejected in one response instead of a bare 503.
                    state._drain_world_log_into_buffer()
                    engine = [
                        {"code": e.get("code"), "message": e.get("message")}
                        for e in state.log_buffer.since(
                            log_mark, 20, types=["world.error", "world.warning"])
                    ]
                    self._json(422, {
                        "ok": False,
                        "error": str(exc),
                        "code": "SPAWN_REJECTED",
                        "vrml": vrml,
                        "engine_diagnostics": engine,
                    })
                    return
                result = dict(result)
                if vrml:
                    result["vrml"] = vrml
                # Honest interim for W1.7: the node is in the scene graph but
                # NOT in the frozen solver model, on every input form
                # (vrml / type+fields / clone alike — the gate is the solver's,
                # not the composer's). Unconditional on purpose: the harness
                # finalizes the world before serving, so there is no
                # mid-session spawn that reaches the solver.
                if body.get("physics") == "rebuild":
                    rb = self._supervisor_call_coded(
                        "rebuild_physics",
                        {"settle_steps": body.get("rebuild_settle_steps", 8)})
                    if rb is None:
                        return
                    result["physics"] = {"mode": "rebuild", **rb}
                else:
                    result["physics_warning"] = state.runtime_mutation_warning("spawn")
                self._json(200, result)
                return

            if path == "/scene/delete":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                args = {}
                if isinstance(body.get("defs"), list):
                    args["defs"] = body["defs"]
                elif isinstance(body.get("def"), str) and body["def"]:
                    args["def"] = body["def"]
                else:
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "provide 'def' (string) or 'defs' (list of strings)"})
                    return
                if "settle_steps" in body:
                    args["settle_steps"] = body["settle_steps"]
                result = self._supervisor_call_coded("scene_delete", args)
                if result is not None:
                    result = dict(result)
                    if body.get("physics") == "rebuild":
                        rb = self._supervisor_call_coded(
                            "rebuild_physics",
                            {"settle_steps": body.get("rebuild_settle_steps", 8)})
                        if rb is None:
                            return
                        result["physics"] = {"mode": "rebuild", **rb}
                    else:
                        # Honest interim for W1.7: the node left the scene
                        # graph but its colliders remain in the frozen solver
                        # model until a rebuild.
                        result["physics_warning"] = state.runtime_mutation_warning("delete")
                    self._json(200, result)
                return

            if path == "/scene/set_pose":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                if not isinstance(body.get("def"), str) or not body["def"]:
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "'def' is required"})
                    return
                args = {"def": body["def"]}
                for key in ("translation", "rotation", "reset_physics", "settle_steps"):
                    if key in body:
                        args[key] = body[key]
                if "translation" not in args and "rotation" not in args:
                    self._json(400, {
                        "error": "provide 'translation' and/or 'rotation'",
                        "code": "POSE_UNSPECIFIED",
                    })
                    return
                result = self._supervisor_call_coded("scene_set_pose", args)
                if result is not None:
                    self._json(200, result)
                return

            # /robot/<def>/joints/set — the harness's first robot-COMMANDING
            # endpoint (internal parity plan, item W2.1). Matched by segments (like the
            # GET /robot/... family) so the DEF can be anything; the fixed
            # /robot/damage/* literals below cannot collide (3 segments vs 4).
            joint_set_parts = path.split("?", 1)[0].strip("/").split("/")
            if (len(joint_set_parts) == 4 and joint_set_parts[0] == "robot"
                    and joint_set_parts[2] == "joints"
                    and joint_set_parts[3] == "set"):
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                joints = body.get("joints")
                if joints is None and isinstance(body.get("names"), list) \
                        and isinstance(body.get("positions"), list):
                    names = body["names"]
                    positions = body["positions"]
                    if len(names) != len(positions):
                        self._json(400, {
                            "error": ("'names' and 'positions' must be the "
                                      "same length"),
                            "code": "JOINTS_UNSPECIFIED",
                        })
                        return
                    if len(set(names)) != len(names):
                        self._json(400, {
                            "error": "'names' contains duplicates",
                            "code": "JOINTS_UNSPECIFIED",
                        })
                        return
                    joints = dict(zip(names, positions))
                if not isinstance(joints, dict) or not joints:
                    self._json(400, {
                        "error": ("provide 'joints' ({name: target_rad_or_m}) "
                                  "or parallel 'names' + 'positions' lists; "
                                  "joint names are the ones GET "
                                  "/robot/<def>/joints reports"),
                        "code": "JOINTS_UNSPECIFIED",
                    })
                    return
                args = {"def": joint_set_parts[1], "joints": joints}
                if "settle_steps" in body:
                    args["settle_steps"] = body["settle_steps"]
                # Coded: an unknown DEF is a 404, an unknown joint name a 422
                # JOINT_NOT_FOUND — the caller's mistake, not an outage. A
                # WRITE verb: deliberately NOT in
                # IDEMPOTENT_SUPERVISOR_COMMANDS (a replayed write after a
                # socket death could double-apply against a moved scene).
                result = self._supervisor_call_coded("set_joint_positions", args)
                if result is not None:
                    self._json(200, result)
                return

            # /robot/<def>/ik — batched IK PREVIEW (internal parity plan, item W2.1).
            # A pure read: World.solve_ik never moves anything; the caller
            # applies the returned angles (or not) via /robot/<def>/joints/set.
            # Segment-matched like joints/set; the /robot/damage/* literals
            # cannot collide (their third segment is reset/inject, not ik).
            ik_parts = path.split("?", 1)[0].strip("/").split("/")
            if (len(ik_parts) == 3 and ik_parts[0] == "robot"
                    and ik_parts[2] == "ik"):
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                if not isinstance(body.get("effector"), str) or not body["effector"]:
                    self._json(400, {
                        "error": ("'effector' is required: the DEF of the "
                                  "end-effector Solid whose position the "
                                  "targets constrain"),
                        "code": "EFFECTOR_UNSPECIFIED",
                    })
                    return
                if not isinstance(body.get("targets"), list) or not body["targets"]:
                    self._json(400, {
                        "error": ("'targets' is required: a non-empty list of "
                                  "[x, y, z] world-frame positions"),
                        "code": "TARGETS_UNSPECIFIED",
                    })
                    return
                args = {"def": ik_parts[1], "effector": body["effector"],
                        "targets": body["targets"]}
                for key in ("rotations", "tool_offset", "iterations"):
                    if key in body:
                        args[key] = body[key]
                # In IDEMPOTENT_SUPERVISOR_COMMANDS: a replayed preview after
                # a dead socket just re-answers — nothing to double-apply.
                result = self._supervisor_call_coded("solve_ik", args)
                if result is not None:
                    self._json(200, result)
                return

            if path == "/robot/damage/reset":
                # Drain any client-sent body so the socket can be closed
                # cleanly. Windows clients see a ConnectionReset otherwise
                # when their request body is still in flight as the
                # response arrives.
                self._drain_body()
                result = self._supervisor_call("damage_reset")
                if result is not None:
                    self._json(200, result)
                return

            if path == "/robot/damage/inject":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                part = body.get("part")
                if not isinstance(part, str) or not part:
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "'part' is required"})
                    return
                args: dict = {"part": part}
                if "hp_delta" in body:
                    try:
                        args["hp_delta"] = float(body["hp_delta"])
                    except (TypeError, ValueError):
                        self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "'hp_delta' must be a number"})
                        return
                if "state" in body:
                    args["state"] = body["state"]
                # Coded: an unknown part name is the caller's mistake.
                result = self._supervisor_call_coded("damage_inject", args)
                if result is not None:
                    self._json(200, result)
                return

            if path == "/scene/look_at":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                position = body.get("position")
                target = body.get("target")
                if not isinstance(position, list) or len(position) != 3:
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "position must be a list of 3 numbers"})
                    return
                if not isinstance(target, list) or len(target) != 3:
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": "target must be a list of 3 numbers"})
                    return
                try:
                    pos_f = [float(x) for x in position]
                    tgt_f = [float(x) for x in target]
                except (TypeError, ValueError) as exc:
                    self._json(400, {"ok": False, "code": "BAD_REQUEST", "error": f"bad number: {exc}"})
                    return
                orientation = compute_look_at_orientation(pos_f, tgt_f)
                push = bool(body.get("push", True))
                pushed: dict | None = None
                if push:
                    pushed = self._supervisor_call(
                        "set_viewpoint", {"position": pos_f, "orientation": orientation}
                    )
                    if pushed is None:
                        return
                self._json(200, {
                    "position": pos_f,
                    "target": tgt_f,
                    "orientation": orientation,
                    "pushed": push,
                })
                return

            if path in ("/scene/frame", "/scene/orbit"):
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                if path == "/scene/frame":
                    self._handle_frame(body)
                else:
                    self._handle_orbit(body)
                return

            if path == "/world/screenshot":
                try:
                    body = self._read_json()
                except Exception as exc:  # noqa: BLE001
                    self._json(400, {"ok": False, "code": "BAD_JSON",
                                     "error": f"bad json: {exc}"})
                    return
                try:
                    quality = int(body.get("quality", 90))
                except (TypeError, ValueError):
                    self._json(400, {"ok": False, "code": "BAD_REQUEST",
                                     "error": "quality must be an integer"})
                    return
                user_path = body.get("path")
                tmp_path: Path
                if isinstance(user_path, str) and user_path:
                    tmp_path = Path(user_path)
                else:
                    fd, name = tempfile.mkstemp(prefix="omnisim_harness_", suffix=".png")
                    os.close(fd)
                    tmp_path = Path(name)
                result = self._supervisor_call(
                    "screenshot", {"path": str(tmp_path), "quality": quality}
                )
                if result is None:
                    return
                # Read the bytes either way: the staleness check needs the
                # whole frame, and a caller writing to its own path deserves
                # the warning just as much as one taking the body.
                read_error: str | None = None
                try:
                    image_bytes = tmp_path.read_bytes()
                except OSError as exc:
                    image_bytes = b""
                    read_error = str(exc)

                # ⚠ NEVER ANSWER 200 WITHOUT A PICTURE.
                # MEASURED 2026-08-12 in 2 of 3 agent cells: this endpoint
                # returned HTTP 200, `Content-Type: image/png` and a ZERO-BYTE
                # body. It is not the rendering-disabled case -- `--no-rendering`
                # is only passed when with_supervisor is false, and the same
                # scene rendered 603 KB through the capture service moments
                # later. The supervisor reported success; nothing checked that a
                # file had actually landed.
                #
                # The cost was out of all proportion to the size: agents worked
                # around it by reaching for the capture service, which renders
                # 1920x1080 through WREN and took the owner's laptop GPU to
                # 86 C, and the `.capture_*` sibling world it leaves behind got
                # a correct 10-robot run graded FAIL on the wrong file. Produce
                # the image or fail loudly -- an empty body is not a third
                # option. (This is the SECOND silent failure of this endpoint:
                # `note_render` below documents the first, a byte-identical
                # frame served while the scene demonstrably moved.)
                if not image_bytes or png_size(image_bytes) is None:
                    if not (isinstance(user_path, str) and user_path):
                        try:
                            tmp_path.unlink()
                        except OSError:
                            pass
                    empty = not image_bytes
                    # Both codes are declared in SCREENSHOT_ERROR_CODES so
                    # /capabilities publishes them: the source scanner only sees
                    # `"code": "<LITERAL>"`, and a code an agent cannot discover
                    # is a code it cannot branch on.
                    code = SCREENSHOT_ERROR_CODES[0] if empty else SCREENSHOT_ERROR_CODES[1]
                    self._json(502, {
                        "ok": False,
                        "code": code,
                        "error": (
                            ("the supervisor reported a successful screenshot but no image "
                             "bytes reached the harness")
                            if empty else
                            ("the supervisor wrote %d bytes that are not a PNG (no IHDR "
                             "header)" % len(image_bytes))),
                        "bytes": len(image_bytes),
                        "render_path": str(tmp_path),
                        "read_error": read_error,
                        "supervisor_reply": result,
                        "hint": (
                            "the engine renders through WREN and `exportImage` needs "
                            "rendering ENABLED: check the world loaded with "
                            "with_supervisor=true (a bare load passes --no-rendering), that "
                            "the engine process is still alive (GET /sim/state), and that "
                            "the harness can read the render path above. Do NOT fall back "
                            "to the capture service to work around this -- it renders "
                            "1920x1080 through WREN and leaves a .capture_* sibling world "
                            "that later graders mistake for the real one."),
                    })
                    return

                render = {}
                digest = hashlib.sha256(image_bytes).hexdigest()[:16]
                sim_ms = None
                try:
                    sim_ms = (state.sim_state() or {}).get("sim_time_ms")
                except Exception:  # noqa: BLE001
                    sim_ms = None
                render = state.note_render(digest, sim_ms)
                if render.get("warning"):
                    print(f"[harness] {render['warning']}", file=sys.stderr, flush=True)

                if isinstance(user_path, str) and user_path:
                    # Learn the real 3D-view size from the header; every
                    # framing/projection number depends on the aspect ratio.
                    state.note_png_path(tmp_path)
                    # `bytes` + `pixels` so a caller that never opens the file
                    # can still tell a picture from an empty placeholder.
                    body = {"path": str(tmp_path), "render": render,
                            "bytes": len(image_bytes),
                            "pixels": list(png_size(image_bytes) or ())}
                    if render.get("warning"):
                        body["warning"] = render["warning"]
                    self._json(200, body)
                    return
                try:
                    pass
                finally:
                    try:
                        tmp_path.unlink()
                    except OSError:
                        pass
                state.note_png_bytes(image_bytes)
                # The body is a PNG, so the warning cannot ride in JSON --
                # it goes in a header, which curl -i and every HTTP client
                # can see, and it is logged above regardless.
                self._png(image_bytes,
                          extra_headers=({"X-OmniSim-Render-Warning":
                                          render["warning"].replace("\n", " ")}
                                         if render.get("warning") else None))
                return

            # Keep-alive: an unread request body would poison the next
            # request on this connection, so drain before answering.
            self._drain_body()
            self._json(404, {"ok": False, "code": "UNKNOWN_ROUTE",
                             "error": f"not found: {path}"})

    return Handler


def _tcp_port_in_use(host: str, port: int, timeout: float = 0.3) -> bool:
    """Return True if `host:port` accepts a TCP connection within `timeout`."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        return sock.connect_ex((host, port)) == 0
    finally:
        sock.close()


def _find_free_port_pair(
    host: str, start_port: int, max_pairs: int = 50
) -> tuple[int, int] | None:
    """Scan ``(p, p+1)`` pairs starting at ``start_port`` and return the
    first pair where both ports are free, or ``None`` if no free pair is
    found within ``max_pairs`` attempts.

    Pairs step by 2 so the HTTP and supervisor ports stay adjacent (the
    convention the rest of the system assumes when only one is named).
    """
    port = start_port
    for _ in range(max_pairs):
        if not _tcp_port_in_use(host, port) and not _tcp_port_in_use(host, port + 1):
            return port, port + 1
        port += 2
    return None


def probe_existing_harness(
    host: str, port: int, timeout: float = 0.6
) -> dict | None:
    """Identify what (if anything) is listening on `host:port`.

    Returns:
        None — nothing is listening; safe to bind.
        {"kind": "harness", "state": {...}} — an OmniSim harness is up; the
            payload is its `/sim/state` body so the caller can show the
            agent which world/binary the existing session is using.
        {"kind": "non_harness_http"} — port answers HTTP but not as us.
        {"kind": "non_http"} — port is bound but not HTTP-speaking.

    The shape lets `serve()` branch on the failure mode without crashing
    on `OSError: [Errno 10048]` (Windows) / EADDRINUSE — the agent gets a
    structured message it can act on instead of an opaque traceback.
    """
    if not _tcp_port_in_use(host, port, timeout=timeout):
        return None
    url = f"http://{host}:{port}/sim/state"
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "omnisim-harness-preflight"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            body = resp.read().decode("utf-8", errors="replace")
        try:
            state = json.loads(body)
        except json.JSONDecodeError:
            return {"kind": "non_harness_http"}
        # The harness's /sim/state always carries these keys; use them as
        # a fingerprint so we don't false-positive on an unrelated HTTP
        # service that happens to return JSON.
        if isinstance(state, dict) and "webots_home" in state and "binary" in state:
            return {"kind": "harness", "state": state}
        return {"kind": "non_harness_http"}
    except (urllib.error.URLError, OSError, ConnectionError, TimeoutError):
        return {"kind": "non_http"}


def _format_existing_harness_guidance(
    host: str,
    port: int,
    requested_supervisor_port: int,
    state: dict,
) -> str:
    """Build the structured 'a harness is already running' message.

    The format is shaped for an LLM agent reading a tool's stderr: short
    lines, fixed-width fields, two clearly numbered options, and copy-
    pasteable commands. The agent can either reuse the existing
    harness (cheap; hot reload swaps the world in ~600 ms) or start a
    parallel one on a different port pair. We pick `port + 100` for the
    suggestion to stay well clear of `:6790`/`:6791` (supervisor +
    capture) so the suggested numbers are always free in a fresh repo.
    """
    uptime = state.get("uptime_s")
    try:
        uptime_str = f"{float(uptime):.0f}s" if uptime is not None else "?"
    except (TypeError, ValueError):
        uptime_str = "?"
    parallel_http = port + 100
    parallel_sup = parallel_http + 1
    sup_status = "connected" if state.get("supervisor_connected") else "(not connected)"
    lines = [
        f"[harness] another OmniSim harness is already running on http://{host}:{port}",
        "[harness]",
        f"[harness]   uptime         {uptime_str}",
        f"[harness]   omnisim binary  {state.get('binary') or '?'}",
        f"[harness]   omnisim home    {state.get('webots_home') or '?'}",
        f"[harness]   current world  {state.get('world') or '(none loaded)'}",
        f"[harness]   load_ok        {state.get('load_ok')}",
        f"[harness]   supervisor     {sup_status}",
        "[harness]",
        "[harness] you have two options:",
        "[harness]",
        "[harness] (1) reuse the running harness  (recommended; cheapest path,",
        "[harness]     hot reload swaps the world in ~600 ms without restarting OmniSim):",
        "[harness]",
        f"[harness]       curl -X POST http://{host}:{port}/world/load \\",
        "[harness]            -H 'Content-Type: application/json' \\",
        "[harness]            -d '{\"path\": \"<your-world.wbt>\"}'",
        "[harness]",
        f"[harness]     all read-only endpoints (/sim/state, /scene/tree, /world/screenshot,",
        f"[harness]     /sim/events, /robots, /robot/damage) are already live on :{port};",
        "[harness]     if you only need to inspect state, just call them directly.",
        "[harness]",
        "[harness] (2) start a parallel harness on a separate port pair  (use this if the",
        "[harness]     other session is mid-task and you don't want to disturb its world):",
        "[harness]",
        f"[harness]       python -m omnisim harness --port {parallel_http} \\",
        f"[harness]                                  --supervisor-port {parallel_sup}",
        "[harness]",
        "[harness]     (--supervisor-port defaults to --port + 1; you can omit it if",
        "[harness]      that's free.)",
        "[harness]",
        f"[harness] refusing to bind on :{port}; pick one of the options above.",
    ]
    return "\n".join(lines)


def _print_to_stderr(text: str) -> None:
    sys.stderr.write(text + "\n")
    sys.stderr.flush()


def serve(
    host: str, port: int, supervisor_port: int, *, auto_port: bool = False
) -> int:
    # Preflight: refuse to start (with actionable guidance) if anything is
    # already on either port, so an agent in a parallel session sees a
    # clean explanation instead of `OSError: [WinError 10048]`.
    existing = probe_existing_harness(host, port)
    sup_busy = _tcp_port_in_use(host, supervisor_port)
    if auto_port and (existing is not None or sup_busy):
        # Skip the configured pair and scan upward for a free one. Start
        # two above the requested port so we don't immediately retry the
        # busy pair on the first iteration of the scan.
        pair = _find_free_port_pair(host, max(port, supervisor_port) + 2)
        if pair is None:
            _print_to_stderr(
                f"[harness] --auto-port: could not find a free port pair near "
                f":{port}; pass `--port <free>` manually."
            )
            return 2
        chosen_port, chosen_sup = pair
        _print_to_stderr(
            f"[harness] --auto-port: :{port}/:{supervisor_port} taken; "
            f"using :{chosen_port}/:{chosen_sup} instead."
        )
        port, supervisor_port = chosen_port, chosen_sup
        existing = None
        sup_busy = False
    elif auto_port:
        # The defaults were free, so no scan happened -- but the caller asked
        # for --auto-port precisely to DISCOVER the listening pair, so the
        # announcement must be unconditional (AGENTS.md promises it).
        _print_to_stderr(
            f"[harness] --auto-port: using :{port}/:{supervisor_port}."
        )
    if existing is not None:
        kind = existing.get("kind")
        if kind == "harness":
            _print_to_stderr(_format_existing_harness_guidance(
                host, port, supervisor_port, existing.get("state") or {}
            ))
        elif kind == "non_harness_http":
            _print_to_stderr(
                f"[harness] port {host}:{port} is already bound by a non-OmniSim HTTP "
                f"service; pass `--port <free>` and `--supervisor-port <free+1>`, "
                f"or re-run with `--auto-port` to pick a free pair automatically."
            )
        else:
            _print_to_stderr(
                f"[harness] port {host}:{port} is already bound (not HTTP); "
                f"pass `--port <free>` and `--supervisor-port <free+1>`, "
                f"or re-run with `--auto-port` to pick a free pair automatically."
            )
        return 2
    if sup_busy:
        _print_to_stderr(textwrap.dedent(f"""\
            [harness] supervisor port {host}:{supervisor_port} is already in use.
            [harness] this is the TCP port the injected supervisor controller binds inside the
            [harness] OmniSim subprocess; if it is taken, world loads will fail.
            [harness] either stop the conflicting service, pick a free pair, e.g.:
            [harness]
            [harness]     python -m omnisim harness --port {port + 100} --supervisor-port {port + 101}
            [harness]
            [harness] or re-run with `--auto-port` to pick a free pair automatically.""").rstrip())
        return 2

    state = HarnessState(
        infer_omnisim_home(),
        supervisor_host=SUPERVISOR_HOST,
        supervisor_port=supervisor_port,
    )
    try:
        server = ThreadingHTTPServer((host, port), make_handler(state))
    except OSError as exc:
        # TOCTOU: another process bound the port between probe and bind.
        # Re-run the probe so the agent still gets the structured guidance
        # rather than a raw OSError trace.
        late = probe_existing_harness(host, port)
        if late is not None and late.get("kind") == "harness":
            _print_to_stderr(_format_existing_harness_guidance(
                host, port, supervisor_port, late.get("state") or {}
            ))
        else:
            _print_to_stderr(
                f"[harness] could not bind {host}:{port}: {exc}. "
                f"pass `--port <free>` and `--supervisor-port <free+1>`."
            )
        return 2

    def handle_signal(signum, _frame):
        print(f"[harness] signal {signum} received; shutting down")
        state.shutdown()
        threading.Thread(target=server.shutdown, daemon=True).start()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, handle_signal)
        except (ValueError, OSError):
            pass

    print(f"[harness] listening on http://{host}:{port}")
    print(f"[harness] omnisim binary: {state.binary}")
    print(f"[harness] omnisim home:   {state.omnisim_home}")
    print(f"[harness] log file:      {state.log_path}")
    print(f"[harness] supervisor:    {state.supervisor_host}:{state.supervisor_port}")
    if not _HAS_PIL:
        print(
            "[harness] note: Pillow not installed; /world/render_stats will return 503. "
            "pip install Pillow",
            file=sys.stderr,
        )
    try:
        server.serve_forever()
    finally:
        state.shutdown()
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description="OmniSim agent-facing validation harness")
    p.add_argument("--host", default=DEFAULT_HOST, help=f"bind host (default: {DEFAULT_HOST}, loopback only)")
    p.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"bind port (default: {DEFAULT_PORT})")
    p.add_argument(
        "--supervisor-port",
        type=int,
        default=None,
        help=(
            f"TCP port the injected supervisor controller binds inside the "
            f"OmniSim subprocess (default: --port + 1, i.e. {DEFAULT_PORT + 1} "
            f"when --port is the default). Pass an explicit value to run a "
            f"second harness alongside an existing one."
        ),
    )
    p.add_argument(
        "--engine-mode",
        choices=list(ENGINE_MODES),
        default=None,
        help=(
            "Engine run mode (default: fast, ~13x real time -- also settable via "
            "OMNISIM_HARNESS_ENGINE_MODE). Pass 'realtime' for wall-clock-paced "
            "simulation: sensor-driven ROS 2 stacks (Nav2, slam_toolbox) need it, "
            "because at 13x the /clock cadence this HTTP surface sustains lands "
            "seconds of sim time apart and scan<->TF alignment breaks (issue #13)."
        ),
    )
    p.add_argument(
        "--auto-port",
        action="store_true",
        help=(
            "If the requested port pair is already in use, scan upward for "
            "a free (port, port+1) pair and bind there instead of failing. "
            "The chosen ports are printed to stderr so callers/agents can "
            "discover the actual listening address."
        ),
    )
    args = p.parse_args()
    global ENGINE_MODE
    ENGINE_MODE, mode_warning = resolve_engine_mode(
        args.engine_mode, os.environ.get("OMNISIM_HARNESS_ENGINE_MODE"))
    if mode_warning:
        print(f"[harness] WARNING: {mode_warning}", file=sys.stderr)
    if ENGINE_MODE != "fast":
        print(f"[harness] engine mode: {ENGINE_MODE}", file=sys.stderr)
    # Loud on purpose: the tracking default flipped to light on 2026-09-02,
    # and an operator expecting /sim/grips or contact.* events from a bare
    # /world/load must learn it from the banner, not from an empty answer.
    default_light, default_source = resolve_light_default()
    if default_light:
        print(f"[harness] tracking default: LIGHT ({default_source}; since "
              f"{LIGHT_DEFAULT_SINCE}) -- a /world/load naming neither `light` nor "
              f"`tracking` drops the contact/joint-limit/grip trackers (/sim/grips empty, "
              f"contact.*/grip.*/joint.limit_hit quiet; /sim/contacts unaffected). "
              f"{LIGHT_DEFAULT_ENV}=0 restores full tracking as the default.",
              file=sys.stderr)
    else:
        print(f"[harness] tracking default: FULL ({default_source}) -- every /world/load "
              f"without `light`/`tracking` walks the scene every basic step: /sim/step "
              f"costs about 2.3x more per single step on the fleet arena (2026-09-02; ~17-47x before the 2026-09-02 engine fixes). Unset "
              f"{LIGHT_DEFAULT_ENV} (or =1) for the light default.",
              file=sys.stderr)
    supervisor_port = args.supervisor_port if args.supervisor_port is not None else args.port + 1
    return serve(args.host, args.port, supervisor_port, auto_port=args.auto_port)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except RuntimeError as exc:
        # An unbuilt clone is an ordinary, expected state -- report it as
        # advice, not as a stack trace an agent has to parse.
        print(f"\n{exc}\n", file=sys.stderr)
        sys.exit(1)
