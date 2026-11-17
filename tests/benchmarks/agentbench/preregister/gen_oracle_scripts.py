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

"""Generate the Lane B scripted per-task oracles (plan 2.1 / 2.2 / 5.5).

One oracle script per (task, simulator); each is replayed under BOTH tool
conditions of its simulator by ``run_oracles.py``, because the minimal
competent path for every one of these five tasks is file-level (see the
per-script ``minimality`` blocks) and the ``shell`` tool set is a subset of
every ``shell+tools`` set. The scripts are the runner's own
``agentbench/script/v1`` format and are replayed by the REAL scripted backend
through the REAL tool sets -- nothing here bypasses the loop, the ledger or
the graders.

Deterministic on purpose: the emitted JSON depends only on the committed task
worlds and the string constants below, so a test can regenerate the scripts
and assert byte-equality with the committed copies (freshness guard --
the same pattern the coverage table uses).

Call-granularity convention (recorded here because the oracle counts define
the plan-2.2 granularity/distinctness verdicts, and a convention that shifted
per cell could manufacture either verdict):

* one logical action = one tool call: each file read, each file write, each
  simulator invocation is its own call; result-reading from a call's own
  output is free (the runner returns stdout with the call);
* a competent agent reads a file before editing it (the read is counted);
* repair tasks (C1, C2) include exactly one verification run -- "fix it"
  without a load check is not competent; C2's includes the physical proof its
  prompt demands;
* pure-measurement tasks (B1, B3) and the camera task (B2) take their answer
  from the task's own initial scene, whose authored geometry is decisive.
  The worlds deliberately do NOT answer their own tasks in comments (the
  B1 disclosure was stripped 2026-08-01 before the freeze), so B1's oracle
  must MEASURE: it reads the scene for the poses and the robot model's own
  geometry source for the footprint, and computes the pairwise clearances
  itself. No compound shell one-liners are used to hide actions inside a
  single call.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
AGENTBENCH = HERE.parent
sys.path.insert(0, str(AGENTBENCH.parent))

TASKS = AGENTBENCH / "tasks"
SCRIPTS = AGENTBENCH / "runner" / "scripts"
SCHEMA = "agentbench/script/v1"

# --- deterministic world edits ----------------------------------------------

ORIENT_INITIAL = "orientation 0.20664424 0.13429253 -0.96915617 2.01746471"
# Aim (0,0,5) -> the red cylinder at (8,8,0.6), FLU look-at (same math that
# baked the shipped initial pose; verified live on both arms).
ORIENT_AT_RED = "orientation -0.16845545 0.40668743 0.89790205 0.86444318"

C1_BRACE_OLD = "  ]\n\nDEF PALLET_B"
C1_BRACE_NEW = "  ]\n}\n\nDEF PALLET_B"

C2_OMNISIM_FLOOR_OLD = "  ]\n}\n\nDEF CRATE_BOT"
C2_OMNISIM_FLOOR_NEW = ("  ]\n  boundingObject Box {\n    size 20 20 0.1\n"
                        "  }\n}\n\nDEF CRATE_BOT")


def _read(task, sub, name):
    return (TASKS / task / sub / name).read_text(encoding="utf-8")


def _replace_once(text, old, new, what):
    n = text.count(old)
    if n != 1:
        raise AssertionError(
            "edit %r: expected exactly 1 occurrence of the anchor, found %d "
            "-- the task world drifted; re-derive the oracle edit" % (what, n))
    return text.replace(old, new)


def b2_fixed(sim):
    name = "frame_the_cylinder.wbt"
    sub = "initial" if sim == "omnisim" else "initial_webots"
    text = _read("B2_subject_in_frame", sub, name)
    return _replace_once(text, ORIENT_INITIAL, ORIENT_AT_RED,
                         "B2 %s viewpoint" % sim)


def c1_fixed(sim):
    sub = "initial" if sim == "omnisim" else "initial_webots"
    text = _read("C1_parse_error_fix", sub, "parse_error.wbt")
    text = _replace_once(text, "Soild", "Solid", "C1 %s node type" % sim)
    return _replace_once(text, C1_BRACE_OLD, C1_BRACE_NEW,
                         "C1 %s missing brace" % sim)


def c2_fixed(sim):
    if sim == "omnisim":
        text = _read("C2_fall_through_floor", "initial", "fall_through.wbt")
        return _replace_once(text, C2_OMNISIM_FLOOR_OLD, C2_OMNISIM_FLOOR_NEW,
                             "C2 omnisim floor boundingObject")
    text = _read("C2_fall_through_floor", "initial_webots",
                 "fall_through.wbt")
    if not text.endswith("  ]\n}\n"):
        raise AssertionError("C2 webots floor block is no longer the last "
                             "node; re-derive the oracle edit")
    return (text[: -len("  ]\n}\n")]
            + "  ]\n  boundingObject Box {\n    size 20 20 0.1\n  }\n}\n")


# --- the C2 webots probe (the agent's own verification instrument) ----------

Z_PROBE_PY = '''\
"""Supervisor probe: sample the crate's z for 5 s, print the track, quit."""
from controller import Supervisor

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
crate = sup.getFromDef("CRATE_BOT")
zs = [crate.getPosition()[2]]
while sup.getTime() < 5.0:
    if sup.step(dt) == -1:
        break
    zs.append(crate.getPosition()[2])
print("Z_TRACK start=%.3f end=%.3f min=%.3f samples=%d"
      % (zs[0], zs[-1], min(zs), len(zs)), flush=True)
sup.simulationQuit(0)
sup.step(dt)
'''

Z_PROBE_STANZA = '''
Robot {
  name "z_probe"
  controller "z_probe"
  supervisor TRUE
}
'''


def c2_webots_probe_world():
    return c2_fixed("webots") + Z_PROBE_STANZA


# --- upstream invocations over WSL (the shell agent's own idiom) ------------
#
# Quoting, measured rather than assumed: ``wsl.exe`` wraps each argv token in
# double quotes and hands the joined line to the distro's login shell, which
# expands ``$`` once before the inner ``bash -c`` runs -- so every ``$`` the
# inner script needs is escaped ``\$``, and the command uses no double quotes
# at all (msys re-quoting mangles them). Paths carry no spaces by
# construction (the run scratch lives under the repo). ``grep -e A -e B``
# replaces a quoted alternation for the same reason.

_WSL_TEMPLATE = (
    "wsl -d Ubuntu-22.04 -- bash -c '"
    "W={world}; D=\\${{W%%:*}}; W=/mnt/\\${{D,,}}\\${{W#?:}}; "
    "WEBOTS_HOME=/opt/upstream-webots/R2025a "
    "timeout {t} xvfb-run -a /opt/upstream-webots/R2025a/webots "
    "--batch --mode=fast --no-rendering --minimize --stdout --stderr "
    "--port=1507 \\$W 2>&1 | {filter}; echo WEBOTS_RC=\\$?'")


def wsl_webots_cmd(world, *, timeout_s, filter_cmd):
    return _WSL_TEMPLATE.format(world=world, t=int(timeout_s),
                                filter=filter_cmd)


# --- turns ------------------------------------------------------------------


def _turn(text, calls=None, tokens_in=4000, tokens_out=300):
    t = {"text": text,
         "usage": {"tokens_in": tokens_in, "tokens_out": tokens_out}}
    if calls:
        t["tool_calls"] = calls
    return t


def _call(name, **arguments):
    return {"name": name, "arguments": arguments}


def b1_script(sim):
    if sim == "omnisim":
        world = "six_huskies.wbt"
        geom_call = _call("read_file", path="{{HUSKY_URDF}}")
        geom_note = ("The world instantiates six copies of one URDF robot; "
                     "the poses alone do not decide overlap, so I need the "
                     "model's own collision geometry. Reading the Husky "
                     "URDF the world's `url` fields point at.")
        answer = (
            "Yes - husky_one and husky_two are overlapping. Measured from "
            "the two files: the URDF's base collision box is "
            "0.9874 x 0.5709 m in plan, and the six authored translations "
            "give pairwise centre separations of 0.32 m for "
            "(husky_one, husky_two) - dx 0.3, dy 0.1 - against 3.70 m for "
            "the next-closest pair. Half-extents 0.494 + 0.494 = 0.99 m on "
            "x and 0.285 + 0.285 = 0.57 m on y both exceed the "
            "separations, so their bounding boxes interpenetrate deeply on "
            "every axis; every one of the other 14 pairs is metres clear.")
    else:
        world = "six_pioneers.wbt"
        # The Pioneer3at PROTO resolves from upstream's pre-seeded,
        # content-addressed asset cache (offline rule, BRINGUP.md sec. 1),
        # so the geometry read locates it by content, then extracts the
        # boundingObject box sizes. WSL quoting per _WSL_TEMPLATE's note:
        # no double quotes, no spaces or backslash classes in patterns
        # (`size.0` -- the regex dot stands in for the space, which would
        # otherwise be token-split), and \\$ for inner expansion.
        geom_call = _call(
            "run_shell",
            command=("wsl -d Ubuntu-22.04 -- bash -c "
                     "'P=\\$(grep -rlE PROTO.Pioneer3at "
                     "/root/.cache/Cyberbotics/Webots/assets); "
                     "grep -nE -e boundingObject -e size.0 \\$P "
                     "| head -40'"),
            timeout_s=120)
        geom_note = ("The world instantiates six Pioneer3at PROTOs from "
                     "upstream's pre-seeded asset cache; the poses alone do "
                     "not decide overlap, so I need the model's own "
                     "collision geometry. Locating the cached PROTO by "
                     "content and extracting its boundingObject box sizes.")
        answer = (
            "Yes - robot_one and robot_two are overlapping. Measured from "
            "the scene and the PROTO's own collision geometry: the "
            "Pioneer 3-AT body's boundingObject boxes are "
            "0.197 x 0.2 x 0.402 and 0.264 x 0.2 x 0.332 (a body about "
            "0.5 m long and 0.4 m wide), and the six authored translations "
            "give a centre separation of 0.168 m for (robot_one, "
            "robot_two) - dx 0.16, dy 0.05 - against 3.84 m for the "
            "next-closest pair. The body half-extents exceed the "
            "separation on every axis, so their bounding boxes "
            "interpenetrate deeply; every one of the other 14 pairs is "
            "metres clear.")
    return {
        "schema": SCHEMA,
        "name": "oracle_b1_%s" % sim,
        "description": (
            "B1 overlap_audit oracle (%s): MEASURE, then answer. Read the "
            "scene for the six authored poses, read the robot model's own "
            "geometry source for the footprint, compute the pairwise "
            "clearances, and answer with the one overlapping pair. The "
            "grader re-measures the loaded scene itself and compares the "
            "answer to measured pairwise AABB clearances." % sim),
        "minimality": [
            "The world no longer discloses the answer (its header comments "
            "were stripped 2026-08-01 so the task prices measurement, not "
            "file-reading). The poses live in the world file; the footprint "
            "lives in the robot model's own geometry source (the Husky URDF "
            "on the OmniSim arm; the cached Pioneer3at PROTO's "
            "boundingObject boxes on the Webots arm). Overlap = separation "
            "vs summed half-extents needs BOTH, so two reads + the computed "
            "answer is the minimal competent path on every surface; one "
            "read cannot honestly decide overlap for a model whose size the "
            "world does not state.",
            "Bridge/tools alternatives enumerated (all strictly costlier): "
            "OmniSim shell+tools: start harness (run_shell) + load_world + "
            "get_scene_tree?bounds=1 = 3+ calls. Webots bridge: start "
            "simulator (run_shell) + 6x wb_supervisor_node_get_from_def + "
            "6x wb_supervisor_node_get_position (+ quit) = 14 calls, and "
            "positions alone do not give AABBs. Neither surface can answer "
            "in fewer than two calls, so no condition can beat this path.",
            "No verification call: the answer IS the deliverable and the "
            "grader measures the scene independently; a self-check run "
            "would be padding.",
        ],
        "turns": [
            _turn("The scene is a static file in my working directory. "
                  "Reading it for the robot poses.",
                  [_call("read_file", path=world)]),
            _turn(geom_note, [geom_call]),
            _turn(answer, tokens_out=260),
        ],
    }


def b3_script(sim):
    world = ("two_huskies.wbt" if sim == "omnisim" else "two_pioneers.wbt")
    robots = "Huskies" if sim == "omnisim" else "robots"
    return {
        "schema": SCHEMA,
        "name": "oracle_b3_%s" % sim,
        "description": (
            "B3 measure_and_report oracle (%s): read the scene description, "
            "compute the centre-to-centre distance from the authored "
            "translations, answer which robot is taller. The grader "
            "re-measures the loaded scene itself." % sim),
        "minimality": [
            "Static scene; the authored translations decide both questions "
            "(planar offset (6.4, 2.2) plus the 0.4 m plinth -> 6.78 m, "
            "within the grader's 0.30 m tolerance of the measured value on "
            "both arms; 'taller' is decided by the plinth). One read + the "
            "committed answer is minimal on every surface.",
            "Alternatives (all costlier): OmniSim shell+tools harness path "
            "= 3+ calls (start, load, read bounds); Webots bridge = 5+ "
            "calls (start, 2x get_from_def, 2x get_position, quit) and "
            "still yields no AABB top for 'taller'.",
        ],
        "turns": [
            _turn("Static scene, authored translations in the file. "
                  "Reading it.",
                  [_call("read_file", path=world)]),
            _turn("The two %s are 6.78 m apart centre to centre (planar "
                  "offset (6.4, 2.2) plus 0.4 m of height). The taller one "
                  "is husky_plinth, which stands on the 0.4 m plinth."
                  % robots, tokens_out=160),
        ],
    }


def b2_script(sim):
    fixed = b2_fixed(sim)
    return {
        "schema": SCHEMA,
        "name": "oracle_b2_%s" % sim,
        "description": (
            "B2 subject_in_frame oracle (%s): read the world, re-aim the "
            "authored Viewpoint at the red cylinder with a computed FLU "
            "look-at orientation, and commit to the checkable proof numbers "
            "(off-axis angle, range). The grader measures the final pose "
            "from the LIVE loaded scene and checks the numbers." % sim),
        "minimality": [
            "The subject's position is authored in the file, so the look-at "
            "orientation is computable without a live read; the deliverable "
            "is the final camera pose in the artifact plus the committed "
            "numbers. read + write + answer = 2 calls, available on every "
            "surface.",
            "Alternatives (all costlier): OmniSim shell+tools 'frame' verb "
            "needs the harness up and the pose persisted back to the file: "
            "start harness + load_world + frame + write_file (+ optional "
            "kill) = 4-5 calls. Webots bridge: start simulator + "
            "get_from_def(Viewpoint has no DEF; field walk from root) + 2x "
            "set_sf + world_save + quit >= 6 calls. The one-call framing "
            "verb cannot beat a 2-call file path it still has to persist.",
            "Proof is the committed numeric claim (grader-measured); a "
            "screenshot/visible self-check is optional competence excluded "
            "symmetrically on every surface.",
        ],
        "turns": [
            _turn("The subject and the camera are both authored in the "
                  "world file; I can compute the exact look-at orientation "
                  "and bake it. Reading the file first.",
                  [_call("read_file", path="frame_the_cylinder.wbt")]),
            _turn("Re-aiming the Viewpoint from (0,0,5) at the red cylinder "
                  "at (8,8,0.6): FLU look-at gives axis-angle "
                  "-0.16845545 0.40668743 0.89790205 0.86444318. Writing "
                  "the edited world.",
                  [_call("write_file", path="frame_the_cylinder.wbt",
                         content=fixed)],
                  tokens_out=2200),
            _turn("Done - the camera now points at the red cylinder and it "
                  "is in frame, dead centre: off-axis 0.0 deg from the "
                  "camera axis, at a range of 12.14 m, angular diameter "
                  "about 6.6 deg, well inside the 45 deg field of view "
                  "with margin on every side.", tokens_out=200),
        ],
    }


def c1_script(sim):
    fixed = c1_fixed(sim)
    if sim == "omnisim":
        verify = _call(
            "run_shell",
            command=("python -m omnisim run-headless "
                     "\"{{SCRATCH}}/parse_error.wbt\" --duration 5"),
            cwd="{{REPO}}", timeout_s=170)
        verify_note = ("Verifying the repaired world actually loads and "
                       "steps, headless.")
    else:
        verify = _call(
            "run_shell",
            command=wsl_webots_cmd(
                "{{SCRATCH}}/parse_error.wbt", timeout_s=25,
                filter_cmd="grep -e ERROR -e error -e Starting | head -30"),
            timeout_s=170)
        verify_note = ("Verifying the repaired world loads under the "
                       "installed upstream Webots (headless under xvfb; "
                       "killed by timeout after loading, since upstream has "
                       "no auto-exit - a clean load shows no ERROR lines).")
    return {
        "schema": SCHEMA,
        "name": "oracle_c1_%s" % sim,
        "description": (
            "C1 parse_error_fix oracle (%s): read the broken world, repair "
            "BOTH defects in place (the unbalanced brace on pallet_a and "
            "the undefined 'Soild' node type on pallet_b) without deleting "
            "any node, then verify the repaired world loads." % sim),
        "minimality": [
            "read + write + one verification load = 3 calls. The read and "
            "write are irreducible (the deliverable is the repaired file); "
            "the verification load is the competence floor for 'fix it' - "
            "a fix never loaded is a guess.",
            "Bridge alternative (webots): the world does not load, so no "
            "supervisor - and therefore no bridge - can run against it; "
            "the file repair must happen at file level on every surface. "
            "OmniSim shell+tools alternative: load_world via harness "
            "instead of run-headless = same call count, needs the harness "
            "started first (+1). This path is minimal on every surface.",
        ],
        "turns": [
            _turn("Reading the broken world to find what the parser "
                  "chokes on.",
                  [_call("read_file", path="parse_error.wbt")]),
            _turn("Two defects: DEF PALLET_A Solid is missing its closing "
                  "brace, and DEF PALLET_B's node type is misspelled "
                  "'Soild'. Repairing both in place - no nodes deleted.",
                  [_call("write_file", path="parse_error.wbt",
                         content=fixed)],
                  tokens_out=2000),
            _turn(verify_note, [verify]),
            _turn("Fixed. The world had two defects: pallet_a's Solid "
                  "block was never closed (missing brace), and pallet_b "
                  "was declared with the undefined node type 'Soild' "
                  "instead of Solid. Both repaired in place; the "
                  "verification run above loads the full scene - floor, "
                  "probe_bot, pallet_a, pallet_b - with no parse errors.",
                  tokens_out=220),
        ],
    }


def c2_script(sim):
    fixed = c2_fixed(sim)
    if sim == "omnisim":
        turns = [
            _turn("Reading the world to see why the crate falls through "
                  "the floor.",
                  [_call("read_file", path="fall_through.wbt")]),
            _turn("DEF FLOOR is a Solid with visible geometry but NO "
                  "boundingObject - scenery, not a collision surface. "
                  "Adding a boundingObject Box matching the visible slab.",
                  [_call("write_file", path="fall_through.wbt",
                         content=fixed)],
                  tokens_out=1400),
            _turn("Proving it stays up: a 10 s headless re-run with the "
                  "runaway watchdog, which fails if any body leaves the "
                  "world through a missing collision surface.",
                  [_call("run_shell",
                         command=("python -m omnisim run-headless "
                                  "\"{{SCRATCH}}/fall_through.wbt\" "
                                  "--duration 10 --fail-on-runaway"),
                         cwd="{{REPO}}", timeout_s=170)]),
            _turn("Fixed and proven. DEF FLOOR had no boundingObject, so "
                  "it had no collision surface and the crate fell straight "
                  "through. I added a boundingObject Box 20 x 20 x 0.1 "
                  "matching the visible slab. Proof: the 10 s re-run above "
                  "PASSes under --fail-on-runaway - the watchdog samples "
                  "every dynamic body and confirms none left the world; "
                  "the crate drops from z = 1.0 m, lands on the floor and "
                  "stays there.", tokens_out=240),
        ]
        minimality = [
            "read + write + one proof run = 3 calls. The proof run is "
            "demanded by the prompt ('prove it stays up'); "
            "--fail-on-runaway is the installed simulator's own "
            "physical-verdict lane, so the proof costs one call.",
            "shell+tools alternative: harness start + load + step + "
            "contacts read = 4+ calls and still needs the file edit. Not "
            "cheaper.",
        ]
    else:
        probe_world = c2_webots_probe_world()
        turns = [
            _turn("Reading the world to see why the crate falls through "
                  "the floor.",
                  [_call("read_file", path="fall_through.wbt")]),
            _turn("DEF FLOOR is a Solid with visible geometry but NO "
                  "boundingObject - scenery, not a collision surface. "
                  "Upstream Webots has no built-in physical-verdict "
                  "runner, so I will prove the fix with my own Supervisor "
                  "probe: it samples the crate's z for 5 s and prints the "
                  "track. Writing the probe controller first.",
                  [_call("write_file",
                         path="probe/controllers/z_probe/z_probe.py",
                         content=Z_PROBE_PY)],
                  tokens_out=900),
            _turn("A probe copy of the FIXED world with the probe "
                  "Supervisor appended (upstream has no auto-exit; the "
                  "probe also owns termination).",
                  [_call("write_file", path="probe/worlds/c2_check.wbt",
                         content=probe_world)],
                  tokens_out=1600),
            _turn("Now the fix itself, in place.",
                  [_call("write_file", path="fall_through.wbt",
                         content=fixed)],
                  tokens_out=1300),
            _turn("Running the probe world headless under the installed "
                  "upstream Webots.",
                  [_call("run_shell",
                         command=wsl_webots_cmd(
                             "{{SCRATCH}}/probe/worlds/c2_check.wbt",
                             timeout_s=60,
                             filter_cmd="grep -e Z_TRACK -e ERROR "
                                        "| head -10"),
                         timeout_s=170)]),
            _turn("Fixed and proven. DEF FLOOR had no boundingObject, so "
                  "it had no collision surface and the crate fell straight "
                  "through. I added a boundingObject Box 20 x 20 x 0.1 "
                  "matching the visible slab. Proof: the Z_TRACK line "
                  "above is the crate's measured height over a 5 s re-run "
                  "- it drops from z = 1.0 m, settles at about z = 0.25 m "
                  "(floor top 0.05 m plus half the crate's 0.4 m height) "
                  "and never goes lower, so it rests ON the floor instead "
                  "of passing through it.", tokens_out=260),
        ]
        minimality = [
            "read + fix + proof = 5 calls: upstream ships no headless "
            "physical-verdict runner, so the proof instrument (a "
            "Supervisor z-probe: its controller, and a probe world that "
            "can terminate itself) must be authored by the agent - two "
            "writes the OmniSim arm does not need. That asymmetry is the "
            "measured cost difference, not padding.",
            "Bridge alternative: start simulator on the broken world + "
            "get_from_def(FLOOR) + get_field(boundingObject) + "
            "import_sf_node_from_string + world_save + get_from_def"
            "(CRATE_BOT) + step + get_position + quit = 9 bridge calls "
            "after the run_shell start, and world_save at t>0 bakes a "
            "fallen pose unless the import-save ordering is exactly "
            "right. Strictly costlier and more fragile than the file "
            "path; the bridge is a superset of shell, so the shell path "
            "is its minimal path too.",
            "The probe world is written BEFORE the fixed world so the "
            "grader's most-recently-modified artifact-discovery rule "
            "picks the task world, not the probe copy.",
        ]
    return {
        "schema": SCHEMA,
        "name": "oracle_c2_%s" % sim,
        "description": (
            "C2 fall_through_floor oracle (%s): diagnose the missing "
            "collision surface, repair it in place, and prove the crate "
            "now rests on the floor with a measured re-run." % sim),
        "minimality": minimality,
        "turns": turns,
    }


GENERATORS = {
    "oracle_b1_omnisim.json": lambda: b1_script("omnisim"),
    "oracle_b1_webots.json": lambda: b1_script("webots"),
    "oracle_b2_omnisim.json": lambda: b2_script("omnisim"),
    "oracle_b2_webots.json": lambda: b2_script("webots"),
    "oracle_b3_omnisim.json": lambda: b3_script("omnisim"),
    "oracle_b3_webots.json": lambda: b3_script("webots"),
    "oracle_c1_omnisim.json": lambda: c1_script("omnisim"),
    "oracle_c1_webots.json": lambda: c1_script("webots"),
    "oracle_c2_omnisim.json": lambda: c2_script("omnisim"),
    "oracle_c2_webots.json": lambda: c2_script("webots"),
}


def render(name):
    return json.dumps(GENERATORS[name](), indent=2) + "\n"


def main(check=False):
    stale = []
    for name in sorted(GENERATORS):
        path = SCRIPTS / name
        text = render(name)
        if check:
            got = path.read_text(encoding="utf-8") if path.is_file() else None
            if got != text:
                stale.append(name)
            continue
        path.write_text(text, encoding="utf-8")
        print("wrote %s (%d bytes)" % (path, len(text)))
    if check and stale:
        print("STALE: %s" % ", ".join(stale))
        return 1
    if check:
        print("all %d oracle scripts are fresh" % len(GENERATORS))
    return 0


if __name__ == "__main__":
    sys.exit(main(check="--check" in sys.argv[1:]))
