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

"""Diagnostic-code mapper for the OmniSim agent-facing validation harness.

Free-text log lines from `omnisim_log.txt` (and the C++ side of the simulator)
are mapped to structured diagnostic dicts that an agent can branch on.

Two input shapes are supported:

1. **Tagged lines** emitted by the C++ side when `OMNISIM_STRUCTURED_LOG=1`:

       [OMNISIM-DIAG] code=PROTO_NAME_MISMATCH severity=error file=foo.wbt line=42 message=...

   These short-circuit the regex matcher and are the most reliable signal.

2. **Legacy free-text lines** prefixed by `ERROR:` / `WARNING:` / `FATAL:`.
   These are pattern-matched against the rules below; rules are anchored in
   the actual strings emitted by the simulator (grep `OmLog::error` /
   `tr(...)` calls in `src/omnisim/`).

3. **`INFO:` lines, but only the ones a rule names.** INFO used to be dropped
   before any rule could see it, which threw away the engine's single best
   physics verdict: `[OmNewtonBackend] registered N dynamic + M static Newton
   bodies` is emitted at INFO, and `N == 0` on a world that meant to simulate
   something is the one condition that catches EVERY no-physics cause at once
   (an absent/broken Newton runtime, a `physicsBackend "ode"` pin, a
   capability-gated articulation). So the INFO header is now parsed, and an
   INFO line that matches no rule still returns None — INFO is opt-in per
   rule, never a stream of per-step chatter.

Anything ERROR/WARNING/FATAL that matches no rule is returned with
`code = "UNKNOWN"` and the full raw line, so nothing is silently dropped.

EVERY code carries a static `hint`: what to DO, keyed by code in `HINTS`.
The engine's own message says what happened; the hint says what the agent's
next call should be, and it is not derivable from the message text. The
harness-synthesized codes (`HARNESS_SYNTHESIZED_CODES`, produced by
omnisim_harness.py when the engine never got far enough to log) are keyed here
too, so `hint_for(code)` answers for the whole vocabulary an agent can see.
`tests/harness/test_diagnostic_hints.py` pins that no code is hint-less.

The mapping is intentionally a flat ordered list — readability and the
ability to add a rule by appending one tuple matter more than performance.
The first matching rule wins.
"""

from __future__ import annotations

import re
from typing import Iterable

# Severity strings used in diagnostic dicts.
FATAL = "fatal"
ERROR = "error"
WARNING = "warning"
# `info` exists ONLY for the handful of INFO lines a rule explicitly names (see
# the module docstring). An unmatched INFO line is still dropped, so adding this
# severity cannot turn the engine's per-step chatter into diagnostics.
INFO = "info"

# Tagged-line format emitted from C++ when OMNISIM_STRUCTURED_LOG=1.
# Example:
#   [OMNISIM-DIAG] code=WORLD_PARSE_INVALID_TOKENS severity=error message=...
_TAG_PREFIX = "[OMNISIM-DIAG]"
_TAG_KV = re.compile(r'(\w+)=(?:"((?:[^"\\]|\\.)*)"|(\S+))')

# Header-stripping for legacy lines from OmLog.cpp.
_HEADER_PATTERNS: tuple[tuple[str, str], ...] = (
    ("FATAL: ", FATAL),
    # The engine's Qt message handler (gui/main.cpp) writes qFatal() output as
    # "Qt Fatal: <msg>" and then abort()s -- always fatal, never benign, and the
    # only place the Qt platform-plugin failure (no display / partial libxcb set)
    # is recorded, since omnisim-bin's stderr is discarded on Windows. Public
    # issue #6: before this header the classifier saw NOTHING in that log, so
    # /world/load reported the generic SIMULATOR_EXITED_NONZERO.
    ("Qt Fatal: ", FATAL),
    ("ERROR: ", ERROR),
    ("WARNING: ", WARNING),
    # INFO is parsed but NOT passed through unmatched -- see `classify_line`.
    ("INFO: ", INFO),
)

# A rule: (severity, compiled regex, code, dict of group name -> field name).
# `severity` of None means "match any severity".
# Groups named in the regex that map to a field name are extracted into the
# diagnostic dict alongside the standard fields.
_Rule = tuple[str | None, re.Pattern[str], str, dict[str, str]]


def _r(severity: str | None, pattern: str, code: str, **groups: str) -> _Rule:
    return (severity, re.compile(pattern), code, groups)


# Codes emitted exclusively via the tagged-line path (`OmLog::diagnostic`),
# so they need no regex rule — the tagged-line parser short-circuits and
# preserves the code field verbatim. Listed here so an agent reading the
# harness vocabulary sees the full set of CUDA_* codes the C++ side may emit.
# Source: src/omnisim/compute/cuda/OmCudaError.hpp.
CUDA_CODES: tuple[str, ...] = (
    "CUDA_NOT_AVAILABLE",
    "CUDA_DEVICE_INIT_FAILED",
    "CUDA_DRIVER_TOO_OLD",
    "CUDA_COMPUTE_CAPABILITY_TOO_OLD",
    "CUDA_OUT_OF_MEMORY",
    "CUDA_KERNEL_LAUNCH_FAILED",
    "CUDA_KERNEL_EXECUTION_ERROR",
    "CUDA_MEMCPY_FAILED",
    "CUDA_GL_INTEROP_NOT_IMPLEMENTED",
)


# Order matters: more specific rules come before generic ones.
# Patterns are anchored in real OmLog::error / OmLog::warning call sites in
# src/omnisim/. Updating the simulator's user-facing strings should be paired
# with updates here; the test in tests/harness/test_diagnostics.py will catch
# rules that no longer match.
_RULES: tuple[_Rule, ...] = (
    # ---- Qt platform init (Qt itself, via the handler in gui/main.cpp) ----
    # The engine aborts BEFORE opening the world: no display (Linux), a partial
    # libxcb set, or a QT_QPA_PLATFORM naming a plugin this build does not ship.
    # Verified 2026-08-29: QT_QPA_PLATFORM=xcb on Windows -> exit 3, header-only
    # log carrying exactly this line. Fix on Linux: xvfb-run -a + the libxcb set
    # from `linux_bootstrap.sh deps`.
    _r(FATAL, r"^This application failed to start because no Qt platform plugin could be initialized",
       "QT_PLATFORM_PLUGIN_FAILED"),

    # ---- World file gating (OmApplication.cpp) ----
    # Dual-read: the engine names '.omniworld' and mentions the legacy '.wbt'.
    # The older single-extension wording is still matched so this classifier keeps
    # working against a pre-migration binary.
    _r(ERROR,
       r"^Could not open file: '(?P<file>[^']+)'\. The world file extension must be "
       r"'\.(?:omniworld' \(or the legacy '\.wbt'\)|wbt')\.$",
       "WORLD_WRONG_EXTENSION", file="source_path"),
    _r(ERROR, r"^Could not open file: '(?P<file>[^']+)'\.$",
       "WORLD_FILE_NOT_FOUND", file="source_path"),
    _r(ERROR, r"^'(?P<file>[^']+)': Failed to load due to invalid token\(s\)\.$",
       "WORLD_PARSE_INVALID_TOKENS", file="source_path"),
    _r(ERROR, r"^'(?P<file>[^']+)': Failed to load due to syntax error\(s\)\.$",
       "WORLD_PARSE_SYNTAX_ERROR", file="source_path"),

    # ---- Tokenizer / parser (OmTokenizer.cpp) ----
    _r(ERROR, r"^File is empty: '(?P<file>[^']+)'\.$",
       "WORLD_FILE_EMPTY", file="source_path"),
    _r(ERROR, r"^'(?P<file>[^']+)': error: Missing header\.$",
       "HEADER_MISSING", file="source_path"),
    _r(ERROR, r"^'(?P<file>[^']+)': Invalid header\.$",
       "HEADER_INVALID", file="source_path"),
    _r(WARNING, r"^'(?P<file>[^']+)': Missing header\.$",
       "HEADER_MISSING", file="source_path"),
    # Generic parser error with location: 'file':line:col: error: message.
    _r(ERROR, r"^'(?P<file>[^']+)':(?P<line>\d+):(?P<column>\d+): error: (?P<detail>.+?)\.$",
       "PARSE_ERROR", file="source_path", line="line", column="column", detail="detail"),

    # ---- PROTO model (OmProtoModel.cpp) ----
    _r(ERROR, r"^Recursive definition of PROTO node '(?P<proto>[^']+)' is not supported$",
       "PROTO_RECURSIVE", proto="node_def"),
    _r(ERROR, r"^PROTO node '(?P<proto>[^']+)' cannot have a base node name$",
       "PROTO_BASE_NAME_INVALID", proto="node_def"),
    _r(ERROR, r"^'(?P<proto>[^']+)' PROTO identifier does not match filename$",
       "PROTO_NAME_MISMATCH", proto="node_def"),
    _r(ERROR, r"^Errors when parsing the PROTO parameters$",
       "PROTO_PARAM_ERROR"),

    # ---- EXTERNPROTO and asset retrieval ----
    _r(ERROR, r"^Error downloading EXTERNPROTO '(?P<proto>[^']+)': (?P<detail>.+)$",
       "EXTERNPROTO_DOWNLOAD_FAILED", proto="node_def", detail="detail"),
    _r(ERROR, r"^Cannot download '(?P<url>[^']+)', error code: (?P<status>\d+): (?P<detail>.+)$",
       "ASSET_DOWNLOAD_FAILED", url="source_path", status="detail", detail="detail"),
    _r(WARNING, r"^Texture file could not be read: '(?P<file>[^']+)'$",
       "TEXTURE_READ_FAILED", file="source_path"),
    _r(WARNING, r"^Mesh file could not be read: '(?P<file>[^']+)'$",
       "MESH_READ_FAILED", file="source_path"),
    _r(WARNING, r"^URDF link '(?P<link>[^']+)': .* mesh '(?P<mesh>[^']+)' could not be resolved.*$",
       "URDF_MESH_UNRESOLVED", link="node_def", mesh="source_path"),

    # ---- Controller lifecycle (OmController.cpp) ----
    _r(WARNING, r"^'(?P<controller>[^']+)' controller crashed\.$",
       "CONTROLLER_CRASHED", controller="node_def"),
    _r(WARNING, r"^'(?P<controller>[^']+)' controller exited with status: (?P<status>\d+)\.$",
       "CONTROLLER_EXITED_NONZERO", controller="node_def", status="detail"),

    # =====================================================================
    # PHYSICS / BACKEND (2026-08-08).  Until now this table had ~20 rules and
    # NOT ONE about physics, so every message from the ODE-deletion campaign --
    # the ones that explain why a world loads clean and then does nothing --
    # classified as `code: "UNKNOWN"`, i.e. as prose. AGENTS.md §5 tells agents
    # to "branch on diagnostics[].code rather than regex-matching messages", so
    # UNKNOWN is the same as unreachable.
    #
    # Every pattern below is copied from a real emit site in src/omnisim/ (the
    # file is named on each rule). Messages routed through OmNode::parsingWarn /
    # ::warn are prefixed with the node's scene path ("DEF BALL Solid: ..."), so
    # those rules capture that prefix as `node_def` instead of anchoring at ^.
    # =====================================================================

    # A Solid that pins physicsBackend "ode" gets NO gravity and NO contact:
    # src/omnisim/nodes/OmSolid.cpp (flushPendingNewtonRegistrations, ~3172).
    # One line per pinned Solid that declares collision or mass -- see
    # coalesce_diagnostics() in omnisim_harness.py for the flood cap.
    _r(WARNING, r'^(?P<node>.*?): This Solid asks for physicsBackend "ode", which no longer '
                r"selects a physics engine",
       "SOLID_ODE_PIN_INERT", node="node_def"),

    # No backend at all: src/omnisim/physics/OmPhysicsBackend.cpp (~366).
    # ERROR severity in the engine, so it also takes a headless exit code to 1.
    _r(ERROR, r"^\[physics\] NO PHYSICS BACKEND IS AVAILABLE",
       "NO_PHYSICS_BACKEND"),

    # Retired ODE selectors, warned and IGNORED: OmPhysicsBackend.cpp (~287)
    # and OmNewtonBackend.cpp (~5482). The run is Newton either way.
    _r(WARNING, r"^\[physics\] (?P<variable>OMNISIM_[A-Z_]+) is set but RETIRED and IGNORED",
       "RETIRED_ODE_SELECTOR", variable="detail"),
    _r(WARNING, r"^\[(?:Wb|Om)NewtonBackend\] (?P<variable>OMNISIM_ALLOW_ODE_FALLBACK) is set but "
                r"RETIRED and IGNORED",
       "RETIRED_ODE_SELECTOR", variable="detail"),

    # Newton runtime ABSENT (not installed) -- OmNewtonBackend.cpp ~5298/5309
    # (import failures) and ~5449 (the OMNISIM_REQUIRE_NEWTON fatal).
    # NB: the engine's import-failure text still says "Falling back to ODE",
    # which is stale -- there is no ODE to fall back to. The hint corrects it.
    _r(WARNING, r"^\[(?:Wb|Om)NewtonBackend\] `import (?P<module>warp|newton)` failed",
       "NEWTON_RUNTIME_ABSENT", module="detail"),
    _r(FATAL, r"^\[(?:Wb|Om)NewtonBackend\] OMNISIM_REQUIRE_NEWTON is set but the Newton runtime is"
              r" not installed",
       "NEWTON_RUNTIME_ABSENT"),

    # Newton runtime INSTALLED but would not come up -- OmNewtonBackend.cpp
    # ~5484 (the refusal, FATAL), ~5323/~5360 (attribute/API drift), ~5109
    # (reportPyError), ~5161 (the OMNISIM_NEWTON_SIMULATE_BROKEN injector),
    # ~5227/~5260 (embedded-interpreter init).
    _r(FATAL, r"^\[(?:Wb|Om)NewtonBackend\] this world asked for the Newton backend, and the Newton "
              r"runtime is INSTALLED but did not come up: (?P<detail>.+)$",
       "NEWTON_RUNTIME_BROKEN", detail="detail"),
    _r(WARNING, r"^\[(?:Wb|Om)NewtonBackend\] newton\.ModelBuilder attribute missing",
       "NEWTON_RUNTIME_BROKEN"),
    _r(WARNING, r"^\[(?:Wb|Om)NewtonBackend\] helper module missing `World` class",
       "NEWTON_RUNTIME_BROKEN"),
    # finalize() raised -> NO Newton world was built AT ALL, so the whole scene
    # is inert: every body frozen at its authored pose, no contacts, no
    # actuation. ERROR severity in the engine since 2026-08-16
    # (OmNewtonBackend.cpp, reportPyErrorFatal), so it also takes a headless
    # exit code to 1 -- before that it was a WARNING and `run-headless` scored
    # a physics-less world as PASS. Distinct code from NEWTON_RUNTIME_BROKEN
    # because the runtime here is FINE: it is the WORLD the solver refused.
    _r(ERROR, r"^\[(?:Wb|Om)NewtonBackend\] \w+ FAILED -- THIS WORLD HAS NO PHYSICS\."
              r"(?:.*?The Python exception was: (?P<detail>.+))?$",
       "NEWTON_WORLD_NOT_BUILT", detail="detail"),
    _r(WARNING, r"^\[(?:Wb|Om)NewtonBackend\] (?P<detail>.+? (?:raised:|failed \(no Python error\)).*)$",
       "NEWTON_RUNTIME_BROKEN", detail="detail"),
    _r(WARNING, r"^\[(?:Wb|Om)NewtonBackend\] OMNISIM_NEWTON_SIMULATE_BROKEN is set",
       "NEWTON_RUNTIME_BROKEN"),
    _r(WARNING, r"^\[(?:Wb|Om)NewtonBackend\] embedded Python init failed",
       "NEWTON_RUNTIME_BROKEN"),

    # A whole articulation refused rather than silently dropped out:
    # OmSolid.cpp ~3203 (OmLog::fatal, [newton-enforce]).
    _r(FATAL, r"^\[newton-enforce\] Solid '(?P<node>[^']+)' would silently drop out of the simulation",
       "NEWTON_ENFORCE_REFUSED", node="node_def"),
    # ...and the one case that is routed instead of refused: OmSolid.cpp ~3195.
    _r(WARNING, r"^\[capability-gate\] Solid '(?P<node>[^']+)' is a KINEMATIC articulation",
       "KINEMATIC_ARTICULATION", node="node_def"),

    # Joints that registered no Newton joint: OmBasicJoint.cpp ~428 / ~494.
    # Its motors do not move it and its position sensors read 0.
    _r(WARNING, r"^(?P<joint_type>BallJoint|Hinge2Joint) '(?P<endpoint>[^']*)' could not be registered "
                r"with the Newton physics backend",
       "JOINT_REGISTRATION_FAILED", joint_type="detail", endpoint="node_def"),
    # Joint FIELDS the backend does not implement: OmHinge2Joint.cpp ~360.
    _r(WARNING, r"^(?P<joint_type>\w+) '(?P<endpoint>[^']*)': the Newton physics backend does not "
                r"implement (?P<features>.+?), so ",
       "JOINT_FEATURE_UNIMPLEMENTED", endpoint="node_def", features="detail"),

    # TouchSensor with no source at all: OmTouchSensor.cpp ~281 / ~353.
    # A 0 from these is "not measured", never "no force".
    _r(WARNING, r'^(?P<node>.*?): This "(?P<sensor_type>[^"]+)" TouchSensor has NO (?:force|touch) source '
                r"and will read 0",
       "SENSOR_NO_SOURCE", node="node_def", sensor_type="detail"),

    # Static colliders that collide with nothing: OmSolid.cpp ~3932.
    _r(WARNING, r"^(?P<count>\d+) static collider\(s\) in this world are NOT registered with the Newton "
                r"backend",
       "NEWTON_STATICS_NOT_REGISTERED", count="detail"),
    # No static collision surface at all -- bodies rest on the implicit z=0
    # plane, which is not a node: OmSolid.cpp ~3966.
    _r(WARNING, r"^This world declares NO static collision surface",
       "NO_STATIC_COLLISION_SURFACE"),
    # Contact queries structurally blind: OmSolid.cpp ~4402.
    _r(WARNING, r"^Contact queries on Newton-backed Solids return an EMPTY set",
       "CONTACT_QUERIES_BLIND"),
    # A world tuning friction through a field this backend does not read:
    # OmSolid.cpp ~4035.
    _r(WARNING, r"^WorldInfo\.contactProperties declares coulombFriction (?P<declared>[-\d.e+]+)",
       "CONTACT_PROPERTIES_IGNORED", declared="detail"),
    # Occlusion was ASKED FOR but no ray could be cast this tick (gate off, or the
    # Newton raycast service declined). Replaces the old
    # "occlusion rays are not simulated" rule, which matched a warning that no
    # longer exists: the rays themselves were repaired on 2026-08-08, so the only
    # remaining message is this one, and the device keeps its PREVIOUS verdicts
    # rather than promoting anything to "visible".
    _r(WARNING, r"^(?:Radar|Camera Recognition) 'occlusion' is (?:TRUE|enabled) but the Newton raycast service",
       "OCCLUSION_RAYS_UNANSWERED"),
    # setInertiaMatrixFromBoundingObject: OmSolid.cpp ~1444.
    _r(WARNING, r"^'(?P<node>[^']+)': computing an inertia matrix from the bounding object is unavailable",
       "INERTIA_FROM_BOUNDING_OBJECT_UNAVAILABLE", node="node_def"),

    # THE FIELD-NAME TRAP (AGENTS.md): `physicsBackend` is per-Solid; the
    # WorldInfo name is `defaultPhysicsBackend`. The engine reports the misname
    # as a parse ERROR, which takes a headless exit code to 1 -- so a typo reads
    # as a crash and the diagnosis goes to the physics. Both rules must precede
    # the generic PARSE_ERROR rule... which they do not, textually, so they are
    # matched first by being listed here only because PARSE_ERROR's `detail`
    # group would otherwise swallow them: see `_classify_line`'s specificity
    # pass, which retries the specific table before the generic one.
    _r(ERROR, r"^'(?P<file>[^']+)':(?P<line>\d+):(?P<column>\d+): error: Skipped unknown "
              r"'physicsBackend' field in WorldInfo node\.$",
       "WORLDINFO_PHYSICSBACKEND_MISNAMED", file="source_path", line="line", column="column"),
    _r(ERROR, r"^'(?P<file>[^']+)':(?P<line>\d+):(?P<column>\d+): error: Skipped unknown "
              r"'(?P<field>[^']+)' field in (?P<node>\w+) node\.$",
       "UNKNOWN_FIELD_IN_NODE", file="source_path", line="line", column="column",
       field="detail", node="node_def"),

    # ---- The registration census (INFO). THE physics verdict. --------------
    # src/omnisim/nodes/OmSolid.cpp ~3945. `dynamic == 0` is refined into
    # NEWTON_ZERO_DYNAMIC_BODIES by `_refine` below.
    _r(INFO, r"^\[(?:Wb|Om)NewtonBackend\] registered (?P<dynamic>\d+) dynamic \+ (?P<static>\d+) static "
             r"Newton bodies",
       "NEWTON_BODIES_REGISTERED", dynamic="dynamic_bodies_registered",
       static="static_bodies_registered"),
)


# Codes whose specific rule would otherwise be shadowed by an earlier, more
# generic rule in the flat table. `_classify_line` tries these FIRST.
# (PARSE_ERROR's `error: (?P<detail>.+?)\.` matches any located parse error, so
# the two Skipped-unknown-field rules can never be reached by table order.)
_SPECIFIC_FIRST: tuple[str, ...] = (
    "WORLDINFO_PHYSICSBACKEND_MISNAMED",
    "UNKNOWN_FIELD_IN_NODE",
)

# What to DO about a code. The engine's message says what happened; this says
# what the agent's next call is. Attached to every diagnostic that carries the
# code, including tagged-line ones. Every code in `all_codes()` has one.
HINTS: dict[str, str] = {
    # ---- Qt / process ------------------------------------------------------
    "QT_PLATFORM_PLUGIN_FAILED": (
        "the engine aborted BEFORE opening the world, so nothing below this line is about "
        "your world. Linux: run under `xvfb-run -a` and install the libxcb set "
        "(`bash scripts/install/linux_bootstrap.sh deps`); on a GPU-less box set "
        "OMNISIM_NO_WINDOW=1 for headless smokes. Any platform: unset a QT_QPA_PLATFORM that "
        "names a plugin this build does not ship. Then `python -m omnisim doctor`."),
    # ---- World file gating / tokenizer / parser ----------------------------
    "WORLD_WRONG_EXTENSION": (
        "the engine only opens `.omniworld` (or the legacy `.wbt`). Rename the file -- new "
        "worlds are always written as .omniworld (dual-read, single-write) -- and reload. "
        "This is never a physics or parse problem."),
    "WORLD_FILE_NOT_FOUND": (
        "the path did not resolve. Harness paths resolve against the repo root of the clone "
        "whose omnisim_harness.py is running, NOT your working directory -- pass an absolute "
        "path and `ls` it first. `python -m omnisim run-headless <world> --until-finalized` "
        "reproduces the load outside the harness."),
    "WORLD_PARSE_INVALID_TOKENS": (
        "the tokenizer rejected the file before parsing: an unterminated string, a stray "
        "character, a BOM or a non-UTF-8 byte. Read the PARSE_ERROR diagnostics on the same "
        "load for line:column; if there are none, check the header line "
        "(`#OMNISIM R2025a utf8`, or the legacy `#VRML_SIM R2025a utf8`) and the encoding."),
    "WORLD_PARSE_SYNTAX_ERROR": (
        "the structure is wrong: unbalanced braces/brackets, a field outside its node, or a "
        "node the engine does not declare. The PARSE_ERROR / UNKNOWN_FIELD_IN_NODE diagnostics "
        "on this load carry line:column -- fix the FIRST one, later ones are usually cascades. "
        "`python scripts/dev/omniworld.py validate <world>` runs the offline checks in ~4 ms."),
    "WORLD_FILE_EMPTY": (
        "zero bytes: a generator that wrote nothing, or a save that raced a reload. "
        "Regenerate the world (omniworld generate / your gen_*.py), check `wc -c`, reload."),
    "HEADER_MISSING": (
        "the first line must be the header: `#OMNISIM R2025a utf8` (the legacy "
        "`#VRML_SIM R2025a utf8` still parses). Add it and reload."),
    "HEADER_INVALID": (
        "a header line exists but is not one the tokenizer accepts. Use exactly "
        "`#OMNISIM R2025a utf8` (or the legacy `#VRML_SIM R2025a utf8`); a Webots version tag "
        "this engine does not know reads as invalid."),
    "PARSE_ERROR": (
        "open source_path at line:column and read `detail` -- it names the token the parser "
        "expected. Fix the first PARSE_ERROR on the load; the rest are usually cascades. "
        "`Skipped unknown '<f>' field` is UNKNOWN_FIELD_IN_NODE (a misspelt or non-existent "
        "field); `Missing declaration for '<N>'` is an undeclared NODE -- it needs an "
        "EXTERNPROTO line, or it is a node this engine no longer has (Fluid, Radio, "
        "Microphone are gone; check docs/reference/)."),
    # ---- PROTO ----------------------------------------------------------------
    "PROTO_RECURSIVE": (
        "the PROTO instantiates itself, directly or through another PROTO. Break the cycle: "
        "a PROTO body may not reference its own name -- use a base node or a second PROTO "
        "for the inner instance."),
    "PROTO_BASE_NAME_INVALID": (
        "the PROTO is named like a built-in base node (Solid, Robot, Shape ...). Rename the "
        "PROTO and its file to a name that is not a base node, and update every EXTERNPROTO "
        "line that imports it."),
    "PROTO_NAME_MISMATCH": (
        "`PROTO <Name>` inside the file must equal the filename stem: `PROTO Foo` lives in "
        "Foo.proto. Rename one to match (and the EXTERNPROTO lines that reference it). The "
        "engine refuses the mismatch, so that node is MISSING from the world -- do not read "
        "a pose for it."),
    "PROTO_PARAM_ERROR": (
        "the PROTO's parameter block (`field SFFloat x 1.0` ...) did not parse. Every "
        "parameter needs a type, a name and a default OF THAT TYPE; a default of the wrong "
        "type or a missing bracket on an MF field is the usual cause."),
    # ---- EXTERNPROTO / assets -------------------------------------------------
    "EXTERNPROTO_DOWNLOAD_FAILED": (
        "the EXTERNPROTO URL could not be fetched (`detail`: a 404 is a wrong URL or a "
        "renamed PROTO; a connection error is an offline box). Smoke and benchmark worlds "
        "must be LOCAL-ASSET-ONLY: replace the http(s):// URL with a repo-relative path or "
        "an `omnisim://` URL for a shipped PROTO. `omniworld validate` enforces this."),
    "ASSET_DOWNLOAD_FAILED": (
        "a texture/mesh/sound URL could not be fetched (`detail` carries the HTTP status). "
        "Vendor the asset next to the world and reference it relatively; worlds in smoke or "
        "benchmark lanes must never reach the network."),
    "TEXTURE_READ_FAILED": (
        "the texture path resolved but could not be read or decoded: wrong path CASE "
        "(matters on Linux), a non-image file, an unsupported format, or a path relative to "
        "the wrong directory (texture URLs resolve against the world file, or the PROTO "
        "file when set inside one). The load continued with an untextured surface -- fix "
        "the path, then POST /world/sync."),
    "MESH_READ_FAILED": (
        "the mesh file could not be read: wrong path or an unsupported format. The Shape "
        "renders empty, and if the mesh was a boundingObject it now COLLIDES WITH NOTHING -- "
        "after fixing the path, confirm GET /scene/node/<def> -> boundingObject.present. "
        "URDF meshes are URDF_MESH_UNRESOLVED."),
    "URDF_MESH_UNRESOLVED": (
        "a `<mesh filename=...>` in the URDF did not resolve: `package://` needs the package "
        "directory reachable from the URDF's location, and plain paths are relative to the "
        "URDF file. Run `python scripts/dev/urdf_import.py <urdf> --report --strict` for a "
        "structured preflight; docs/developer/urdf-import-debugging.md has the rules."),
    # ---- Controllers -----------------------------------------------------------
    "CONTROLLER_CRASHED": (
        "the controller PROCESS died (a native crash or an exception that escaped its "
        "interpreter), so that robot executes zero further steps while the world keeps "
        "running and a bare log check still PASSes. The traceback is in the controller's "
        "own stdout/stderr -- GET /sim/events?types=controller.log -- not in the engine "
        "log. Reproduce with `python -m omnisim run-headless <world> --duration 10` and "
        "read omnisim_log.txt."),
    "CONTROLLER_EXITED_NONZERO": (
        "the controller exited with the status in `detail`, usually an ImportError or a "
        "traceback at start-up -- the robot never moved, although the world loaded. "
        "GET /sim/events?types=controller.log carries the traceback. A missing module must "
        "be installed into the `python` the ENGINE spawns (the one on PATH; `python -m "
        "omnisim doctor` names it), not into a venv. run-headless already treats this as "
        "a controller-start failure and FAILs the run."),
    # ---- Physics / backend ------------------------------------------------------
    "SOLID_ODE_PIN_INERT": (
        "this node is INERT, not slow: it will never fall, never collide, and "
        "GET /sim/contacts can never report a contact for it. Delete the "
        "`physicsBackend \"ode\"` field from the Solid (there is no ODE to select "
        "any more) and reload. If the node is meant to be visual-only, delete its "
        "boundingObject/physics instead so nothing expects it to collide."),
    "NO_PHYSICS_BACKEND": (
        "NOTHING in this world is simulated -- it will load and stand still. This "
        "is a broken install, not a world bug: run `python -m omnisim doctor`, then "
        "`make -C src/omnisim bundle-newton-runtime` on Windows, or pip install "
        "torch warp-lang newton mujoco mujoco-warp into the SYSTEM python3 on Linux."),
    "RETIRED_ODE_SELECTOR": (
        "unset the variable. It does NOT give you an ODE run -- ODE was deleted and "
        "the engine ignores it -- so any result you attribute to ODE is actually a "
        "Newton result."),
    "NEWTON_RUNTIME_ABSENT": (
        "the engine's own text may say 'falling back to ODE'; that text is stale, "
        "there is no ODE to fall back to and the world runs with no physics. Install "
        "the runtime, then confirm with GET /capabilities -> physics.source == "
        "\"sidecar\"."),
    "NEWTON_RUNTIME_BROKEN": (
        "installed but would not come up. A world that asked for Newton is REFUSED "
        "(fatal) rather than quietly simulated by nothing. Fix the runtime; do not "
        "reach for OMNISIM_ALLOW_ODE_FALLBACK, which is retired and ignored."),
    "NEWTON_WORLD_NOT_BUILT": (
        "the RUNTIME is fine -- the SOLVER REFUSED THIS WORLD, and nothing in it is "
        "simulated: no gravity, no contacts, no actuation, every body frozen at its "
        "authored pose. Read the `detail` (the Python exception): \"Multiple joints "
        "lead to body N\" is a loop-closing SolidReference, which MuJoCo -- a tree-"
        "articulation solver -- cannot represent; model the mechanism as a tree and "
        "drive the dependent joint from a controller. Fix the world and reload; do NOT "
        "read any pose, height or contact from this load."),
    "NEWTON_ZERO_DYNAMIC_BODIES": (
        "read the other diagnostics on this load before anything else: this single "
        "number is downstream of every no-physics cause (NO_PHYSICS_BACKEND, "
        "NEWTON_RUNTIME_ABSENT/BROKEN, SOLID_ODE_PIN_INERT, NEWTON_ENFORCE_REFUSED). "
        "If the world genuinely authors no dynamic body, it is correct and expected."),
    "JOINT_REGISTRATION_FAILED": (
        "this Ball/Hinge2 joint registered NO Newton d6, so its motors will not move it and "
        "its position sensors read 0 -- do not read that 0 as 'the motor is holding'. "
        "Motorised BallJoint / Hinge2Joint actuation itself WORKS (default ON since "
        "2026-08-17): check that OMNISIM_NEWTON_BALL_HINGE2 is not set to 0, then the "
        "joint's authoring (an endPoint Solid with a Physics node). Residual, by design: "
        "a BallJoint's per-axis stops are not enforced (see /capabilities not_supported)."),
    "JOINT_FEATURE_UNIMPLEMENTED": (
        "the CONSTRAINT holds (the bodies stay connected) but the named fields do "
        "nothing -- do not tune them expecting an effect. For limits set "
        "OMNISIM_NEWTON_LIMIT_KE/KD; for springs / suspension drive the joint from your "
        "controller (GET /capabilities -> not_supported joint.fields lists each field)."),
    "SENSOR_NO_SOURCE": (
        "this sensor's 0 means NOT MEASURED. Do not use it as evidence of no contact "
        "or no force; prove the contact geometrically or with GET /sim/contacts."),
    "NEWTON_STATICS_NOT_REGISTERED": (
        "the named colliders are intangible: a raised floor/table/wall lets bodies "
        "pass through and settle on the implicit z=0 plane. Set "
        "WorldInfo.newtonStatics TRUE."),
    "NO_STATIC_COLLISION_SURFACE": (
        "the surface holding this scene up is not in the file and cannot be found by "
        "DEF or seen in GET /scene/tree. Add a floor Solid with a boundingObject."),
    "CONTACT_QUERIES_BLIND": (
        "an empty GET /sim/contacts on this world means 'cannot see', not 'nothing "
        "is touching'. Unset OMNISIM_NEWTON_NATIVE_CONTACTS (or set it to 1)."),
    "CONTACT_PROPERTIES_IGNORED": (
        "the friction you declared is not in effect (the effective value is 1.0). Set "
        "WorldInfo.newtonGroundMu to the value you want."),
    "OCCLUSION_RAYS_UNANSWERED": (
        "occlusion was requested but no ray could be cast this tick, so the device is "
        "reporting its PREVIOUS verdicts -- an unfiltered target list, not a measured "
        "one. Nothing was promoted to \"visible\". Fix the raycast service (leave "
        "OMNISIM_NEWTON_RAYCAST unset; check the Newton runtime; on newtonSolver "
        "\"mujoco_warp\" rays are DECLINED by design -- use the CPU solver) rather than "
        "trusting the list. NOTE: occlusion itself works as of 2026-08-08; a target still "
        "hidden by a node you DELETED at runtime is a different, SILENT gap -- the deleted "
        "geometry stays in the frozen MuJoCo model until POST /sim/rebuild_physics (W1.7, "
        "2026-09-01) or a reload."),
    "INERTIA_FROM_BOUNDING_OBJECT_UNAVAILABLE": (
        "the inertiaMatrix fields were NOT modified, so the body keeps whatever inertia it "
        "had (possibly the identity matrix). Author an explicit `inertiaMatrix` (and "
        "`centerOfMass`) in the Physics node and reload."),
    "WORLDINFO_PHYSICSBACKEND_MISNAMED": (
        "the WORLD-level field is `defaultPhysicsBackend`; `physicsBackend` is "
        "per-Solid. This is a one-word typo, NOT a physics failure -- but the engine "
        "reports it at ERROR severity, which takes a headless run's exit code to 1, "
        "so it reads like a crash."),
    "UNKNOWN_FIELD_IN_NODE": (
        "the field name does not exist on that node, so its value is silently "
        "ignored. Check the spelling against docs/reference/ for that node."),
    "NEWTON_ENFORCE_REFUSED": (
        "refused on purpose: this articulation would have run with no physics at all. "
        "Make the named feature Newton-compatible."),
    "KINEMATIC_ARTICULATION": (
        "the chain animates through the scene tree; no solver simulates it, so it "
        "will not react to contact. Give the joint's endPoint a Physics node to "
        "simulate it dynamically."),
    "NEWTON_BODIES_REGISTERED": (
        "informational: the registration census (N dynamic + M static Newton bodies). "
        "Compare N with the dynamic bodies you authored -- fewer means one was dropped; the "
        "other diagnostics on this load say which (SOLID_ODE_PIN_INERT, "
        "KINEMATIC_ARTICULATION, NEWTON_STATICS_NOT_REGISTERED). N == 0 is reported "
        "separately as NEWTON_ZERO_DYNAMIC_BODIES."),
    # ---- CUDA (tagged-line only; src/omnisim/compute/cuda/OmCudaError.hpp) ------
    "CUDA_NOT_AVAILABLE": (
        "no CUDA device or driver is visible to the engine, so the CUDA granular path "
        "(GranularBed / GranularGroup) cannot run; ordinary Newton physics on the CPU "
        "solver is unaffected. Check `nvidia-smi`; on a CPU-only box do not author "
        "Granular nodes and do not select newtonSolver \"mujoco_warp\"."),
    "CUDA_DEVICE_INIT_FAILED": (
        "a CUDA device exists but could not be initialised: another process holding it "
        "exclusively, a driver in a bad state, or a container started without `--gpus all`. "
        "Check `nvidia-smi`, close the other GPU user, retry; reset the driver (or the pod) "
        "if it persists."),
    "CUDA_DRIVER_TOO_OLD": (
        "the NVIDIA driver predates the CUDA runtime the engine was built against. Update "
        "the driver to the version the message names; nothing in the world file fixes this."),
    "CUDA_COMPUTE_CAPABILITY_TOO_OLD": (
        "this GPU's compute capability is below what the engine's kernels were compiled "
        "for. Run the CUDA-dependent nodes on a newer GPU (or on the CPU path where one "
        "exists); not fixable from the world."),
    "CUDA_OUT_OF_MEMORY": (
        "the GPU ran out of memory: too many granular particles, too large a warp env "
        "batch, or another process (a training run, a second engine) holding VRAM. Reduce "
        "the particle count / batch, close the other GPU user (`nvidia-smi` names it), "
        "retry."),
    "CUDA_KERNEL_LAUNCH_FAILED": (
        "a kernel failed to launch -- usually the consequence of an earlier CUDA_* error "
        "logged just above, or a driver/runtime mismatch. Read the preceding CUDA "
        "diagnostic first; if this stands alone, treat it as an engine bug and report it "
        "with the world and `nvidia-smi` output."),
    "CUDA_KERNEL_EXECUTION_ERROR": (
        "a kernel ran and faulted (out-of-range access, NaN cascade). The CUDA context is "
        "poisoned after this: reload the world and do not trust any particle state from "
        "this load. Report it with the world that triggers it."),
    "CUDA_MEMCPY_FAILED": (
        "a host<->device copy failed, almost always after an earlier CUDA error or an "
        "out-of-memory. Read the preceding CUDA_* diagnostic; reload the world once its "
        "cause is fixed."),
    "CUDA_GL_INTEROP_NOT_IMPLEMENTED": (
        "this build does not share buffers between CUDA and the renderer, so particle "
        "rendering goes through a CPU copy -- slower, not wrong. Nothing to fix in the "
        "world; keep particle counts modest for interactive use."),
    # ---- The fall-through -----------------------------------------------------
    "UNKNOWN": (
        "this ERROR/WARNING/FATAL line matched no rule, so there is no code to branch on -- "
        "read `raw` and decide from the text. If it recurs, add a rule to "
        "scripts/harness/diagnostic_codes.py anchored in the engine's emit site "
        "(`git grep` the message in src/omnisim/) so the next agent gets a code."),
    # ---- Synthesized by the HARNESS (omnisim_harness.py) -------------------------
    "LAUNCHER_DLL_NOT_FOUND": (
        "the engine subprocess could not load its DLLs (Windows exit 0xC0000135): the "
        "harness was started with a PATH that lacks msys64/mingw64/bin. Start it as "
        "`python -m omnisim harness` (the module form prepends the bundled Qt DLLs), not "
        "the raw script, and restart the harness after fixing PATH."),
    "SIMULATOR_EXITED_NONZERO": (
        "the engine exited before the world came up and logged nothing this classifier "
        "recognises. Read the whole engine log (omnisim_log.txt / OMNISIM_LOG_PATH); run "
        "`python -m omnisim doctor` (missing Newton runtime, stale libController); "
        "reproduce with `python -m omnisim run-headless <world> --until-finalized`. Retry "
        "ONCE before diagnosing the world: a second engine starting against a running one "
        "used to die in the Windows console-attach race (fixed 2026-08-29)."),
    "SUPERVISOR_BIND_STALLED": (
        "the injected supervisor never connected inside the wait. Causes, in order: a "
        "controller that cannot start at all (stale libController -> `python -m omnisim "
        "doctor`); an asset-heavy cold load still in progress (46-79 s measured on "
        "virtualised disks -- raise wait_s); a supervisor port already owned by another "
        "harness (start yours with --auto-port, or --supervisor-port)."),
    "SUPERVISOR_BIND_CEILING": (
        "the supervisor bind wait hit the harness's hard ceiling, so a longer wait_s will "
        "not help. Read the engine log for the world's own errors, run `python -m omnisim "
        "doctor`, and make sure no second harness owns port 6790 (`--auto-port`)."),
    "WORLD_DIR_NOT_WRITABLE": (
        "the harness injects its supervisor by writing a sibling copy of the world next to "
        "the file, and that directory refused the write (an install under Program Files is "
        "the usual cause). Copy the world into a writable directory -- or make the "
        "directory writable -- and load from there."),
}

# Backwards-compatible private alias (older callers and tests read `_HINTS`).
_HINTS = HINTS


def _refine(diag: dict) -> dict:
    """Second pass over a matched diagnostic, for verdicts that depend on a
    captured NUMBER rather than on the message text.

    Only one today, and it is the highest-value signal in the whole log: the
    registration census is an INFO line, and `0 dynamic` on it is the single
    condition that catches every no-physics cause at once. Splitting it out into
    its own WARNING code is what makes it branchable -- an agent should not have
    to parse an int out of a message to learn that nothing in its world moves.
    """
    if diag.get("code") != "NEWTON_BODIES_REGISTERED":
        return diag
    try:
        dynamic = int(diag.get("dynamic_bodies_registered"))
    except (TypeError, ValueError):
        return diag
    diag["dynamic_bodies_registered"] = dynamic
    try:
        diag["static_bodies_registered"] = int(diag.get("static_bodies_registered"))
    except (TypeError, ValueError):
        pass
    if dynamic == 0:
        diag["code"] = "NEWTON_ZERO_DYNAMIC_BODIES"
        # WARNING, not ERROR: a world of nothing but static scenery is a legal
        # world, and this cannot tell that apart from a world whose dynamic
        # bodies all fell out. It says both readings and leaves the verdict to
        # the codes alongside it.
        diag["severity"] = WARNING
        diag["reading"] = (
            "the physics backend registered ZERO dynamic bodies for this world. "
            "Either the world authors no dynamic body at all (legal -- a static "
            "scene), or every dynamic body it authored fell out of the simulation "
            "and NOTHING in the scene will move, fall or collide. The other "
            "diagnostics on this load say which.")
    return diag


# Codes produced by `_refine` rather than by a rule of their own, so callers
# enumerating the vocabulary (harness `known_diagnostic_codes()`) see them.
SYNTHESIZED_CODES: tuple[str, ...] = (
    "NEWTON_ZERO_DYNAMIC_BODIES",
)

# Codes the HARNESS synthesizes itself (omnisim_harness.py
# HARNESS_DIAGNOSTIC_CODES -- the two tuples are asserted equal by
# tests/harness/test_diagnostic_hints.py) when the engine never got far enough
# to log anything this classifier could match. Keyed in `HINTS` so the hint
# vocabulary covers them; the harness is the one that attaches them.
HARNESS_SYNTHESIZED_CODES: tuple[str, ...] = (
    "LAUNCHER_DLL_NOT_FOUND",
    "SIMULATOR_EXITED_NONZERO",
    "SUPERVISOR_BIND_STALLED",
    "SUPERVISOR_BIND_CEILING",
    "WORLD_DIR_NOT_WRITABLE",
)


def all_codes() -> tuple[str, ...]:
    """Every code this module can put a `code` field on, plus the harness's."""
    codes = {rule[2] for rule in _RULES}
    codes.update(CUDA_CODES)
    codes.update(SYNTHESIZED_CODES)
    codes.update(HARNESS_SYNTHESIZED_CODES)
    codes.add("UNKNOWN")
    return tuple(sorted(codes))


def hint_for(code: str) -> str | None:
    """The next-action hint for a code, or None for a code this module does not know."""
    return HINTS.get(code)


def _ordered_rules() -> tuple[_Rule, ...]:
    """The rule table with `_SPECIFIC_FIRST` codes hoisted to the front.

    Hoisting rather than reordering the literal table keeps each rule next to
    the family it documents, and makes the shadowing relationship explicit
    instead of load-bearing on line order.
    """
    head = tuple(r for r in _RULES if r[2] in _SPECIFIC_FIRST)
    tail = tuple(r for r in _RULES if r[2] not in _SPECIFIC_FIRST)
    return head + tail


_RULES_ORDERED: tuple[_Rule, ...] = _ordered_rules()


def _classify_line(severity: str, message: str) -> dict:
    for sev_filter, regex, code, groups in _RULES_ORDERED:
        if sev_filter is not None and sev_filter != severity:
            continue
        match = regex.match(message)
        if match is None:
            continue
        diag = {
            "code": code,
            "severity": severity,
            "message": message,
        }
        for group_name, field in groups.items():
            try:
                value = match.group(group_name)
            except IndexError:
                continue
            if value is None:
                continue
            if field == "line" or field == "column":
                try:
                    value = int(value)
                except ValueError:
                    pass
            diag[field] = value
        diag = _refine(diag)
        hint = HINTS.get(diag["code"])
        if hint:
            diag["hint"] = hint
        return diag
    return {"code": "UNKNOWN", "severity": severity, "message": message,
            "hint": HINTS["UNKNOWN"]}


def _parse_tagged_line(payload: str) -> dict | None:
    fields: dict[str, str] = {}
    for match in _TAG_KV.finditer(payload):
        key = match.group(1)
        quoted = match.group(2)
        bare = match.group(3)
        value = quoted if quoted is not None else bare
        if value is not None:
            fields[key] = value.replace(r"\"", '"').replace(r"\\", "\\")
    if "code" not in fields:
        return None
    severity = fields.pop("severity", ERROR).lower()
    if severity not in (FATAL, ERROR, WARNING):
        severity = ERROR
    diag: dict = {
        "code": fields.pop("code"),
        "severity": severity,
        "message": fields.pop("message", ""),
    }
    if "file" in fields:
        diag["source_path"] = fields.pop("file")
    if "line" in fields:
        try:
            diag["line"] = int(fields.pop("line"))
        except ValueError:
            diag["line"] = fields.pop("line")
    if "node" in fields:
        diag["node_def"] = fields.pop("node")
    diag.update(fields)
    hint = HINTS.get(diag["code"])
    if hint:
        diag["hint"] = hint
    return diag


def classify_line(line: str) -> dict | None:
    """Classify one log line into a diagnostic dict, or None to ignore it."""
    stripped = line.strip()
    if not stripped:
        return None

    if stripped.startswith(_TAG_PREFIX):
        payload = stripped[len(_TAG_PREFIX):].strip()
        diag = _parse_tagged_line(payload)
        if diag is not None:
            diag["raw"] = stripped
            return diag

    for header, severity in _HEADER_PATTERNS:
        if stripped.startswith(header):
            message = stripped[len(header):].strip()
            diag = _classify_line(severity, message)
            if severity == INFO and diag["code"] == "UNKNOWN":
                # INFO is opt-in per rule. The engine emits INFO at tick rate
                # (per-step body poses, controller start lines), so passing
                # unmatched INFO through as `UNKNOWN` would flood /sim/events and
                # evict real events from the ring buffer -- measured once already
                # at 96.7% of the stream from a single INFO line.
                return None
            diag["raw"] = stripped
            return diag

    return None


def classify_lines(lines: Iterable[str]) -> list[dict]:
    """Classify a stream of log lines, dropping ones that aren't diagnostics."""
    out: list[dict] = []
    for line in lines:
        diag = classify_line(line)
        if diag is not None:
            out.append(diag)
    return out


def classify_text(text: str) -> list[dict]:
    """Classify the contents of an `omnisim_log.txt`-style log file."""
    return classify_lines(text.splitlines())
