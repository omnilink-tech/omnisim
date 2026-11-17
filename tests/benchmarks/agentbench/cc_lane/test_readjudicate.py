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

"""Tests for the leaked-harness fixes (2026-08-01 phasew forensics):
readjudicate.py, the port-hygiene reaper, and the publish staging dir."""

import json
import socket
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
BENCHMARKS = HERE.parent.parent
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench.cc_lane import readjudicate as rj                # noqa: E402
from agentbench.cc_lane import run_campaign_cc as campaign_mod   # noqa: E402
from agentbench.cc_lane import stage_workspaces as staging       # noqa: E402

TASK = "B2_subject_in_frame"
SIM = "omnisim"


# --- foreign-world witness -------------------------------------------------------


def _w(tmp_path, lines):
    p = tmp_path / "transcript.jsonl"
    p.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return p


def test_witness_matches_harness_state_with_repo_world(tmp_path):
    # exactly the shape measured in the poisoned r1/r2 transcripts: a
    # /sim/state tool result naming a world under the REAL repo
    line = json.dumps({"type": "user", "message": {"content": [{
        "type": "tool_result",
        "content": '{"world": "O:\\\\omnisim\\\\projects\\\\samples\\\\demos'
                   '\\\\worlds\\\\flagship\\\\warehouse_omnilink.omniworld", '
                   '"running": true}'}]}})
    hits = rj.foreign_world_witness(_w(tmp_path, [line]))
    assert len(hits) == 1
    assert "warehouse_omnilink" in hits[0] or "projects" in hits[0]


def test_witness_matches_posix_repo_path(tmp_path):
    hits = rj.foreign_world_witness(_w(tmp_path, [
        '{"text": "world: /o/omnisim/projects/x/y.wbt loaded"}']))
    assert len(hits) == 1


def test_witness_ignores_workspace_worlds_and_plain_repo_prose(tmp_path):
    lines = [
        # a workspace-instance world: hermetic, must NOT match
        json.dumps({"c": '{"world": "C:\\\\Users\\\\x\\\\AppData\\\\Local'
                         '\\\\Temp\\\\agentbench_cc\\\\instances\\\\i1'
                         '\\\\frame_the_cylinder.wbt"}'}),
        # repo mentioned without a world binding: must NOT match
        json.dumps({"c": "see O:/omnisim/projects for demos"}),
    ]
    assert rj.foreign_world_witness(_w(tmp_path, lines)) == []


def test_witness_empty_for_missing_transcript(tmp_path):
    assert rj.foreign_world_witness(tmp_path / "nope.jsonl") == []


# --- rewrite_rows end-to-end -----------------------------------------------------


def _mk_campaign(tmp_path, cells):
    """cells: {rep: (outcome, transcript_lines, artifact_bytes)}. Builds the
    campaign dir + a tasks dir whose staged initial world is b"INITIAL"."""
    camp = tmp_path / "campaign"
    tasks = tmp_path / "tasks"
    (tasks / TASK / "initial").mkdir(parents=True)
    (tasks / TASK / "initial" / "frame_the_cylinder.wbt").write_bytes(
        b"INITIAL")
    group = camp / "groups" / ("%s_%s" % (TASK, SIM))
    group.mkdir(parents=True)
    rows = []
    for rep, (outcome, tlines, art) in sorted(cells.items()):
        cdir = camp / "cells" / ("%s_%s_r%d" % (TASK, SIM, rep))
        (cdir / "artifact").mkdir(parents=True)
        (cdir / "artifact" / "frame_the_cylinder.wbt").write_bytes(art)
        (cdir / "transcript.jsonl").write_text("\n".join(tlines) + "\n",
                                               encoding="utf-8")
        rows.append(json.dumps({"task": TASK, "sim": SIM, "repeat": rep,
                                "outcome": outcome, "notes": []}))
    (group / "rows.jsonl").write_text("\n".join(rows) + "\n",
                                      encoding="utf-8")
    return camp, tasks


FOREIGN = ('{"world": "O:\\\\omnisim\\\\projects\\\\samples\\\\demos'
           '\\\\worlds\\\\flagship\\\\husky_maze_visual.omniworld"}')
CLEAN = '{"world": "C:\\\\Temp\\\\instances\\\\i\\\\frame_the_cylinder.wbt"}'


def test_contaminated_fail_with_untouched_artifact_becomes_invalid(tmp_path):
    camp, tasks = _mk_campaign(tmp_path, {
        0: ("FAIL", [FOREIGN], b"INITIAL")})
    rulings = rj.rewrite_rows(camp, TASK, SIM, apply=True, tasks_dir=tasks,
                              context_note="ctx", out=lambda *a: None)
    assert rulings[0]["invalidate"] is True
    row = json.loads((camp / "groups" / ("%s_%s" % (TASK, SIM))
                      / "rows.jsonl").read_text().strip())
    assert row["outcome"] == "INVALID"
    assert row["grading_recovery"]["original_outcome"] == "FAIL"
    assert row["grading_recovery"]["context"] == "ctx"
    assert any("grading_recovery" in n for n in row["notes"])
    rec = json.loads((camp / "cells" / ("%s_%s_r0" % (TASK, SIM))
                      / "grading_recovery.json").read_text())
    assert rec["revised_outcome"] == "INVALID"


def test_pass_rows_and_edited_artifacts_are_never_rewritten(tmp_path):
    camp, tasks = _mk_campaign(tmp_path, {
        0: ("PASS", [FOREIGN], b"INITIAL"),      # PASS: never rewritten
        1: ("FAIL", [FOREIGN], b"EDITED"),       # earned FAIL: stands
        2: ("FAIL", [CLEAN], b"INITIAL"),        # no witness: stands
    })
    rj.rewrite_rows(camp, TASK, SIM, apply=True, tasks_dir=tasks,
                    out=lambda *a: None)
    lines = (camp / "groups" / ("%s_%s" % (TASK, SIM))
             / "rows.jsonl").read_text().strip().splitlines()
    outcomes = {json.loads(l)["repeat"]: json.loads(l)["outcome"]
                for l in lines}
    assert outcomes == {0: "PASS", 1: "FAIL", 2: "FAIL"}
    for l in lines:
        assert "grading_recovery" not in json.loads(l)


def test_dry_run_writes_nothing(tmp_path):
    camp, tasks = _mk_campaign(tmp_path, {0: ("FAIL", [FOREIGN], b"INITIAL")})
    rows_p = camp / "groups" / ("%s_%s" % (TASK, SIM)) / "rows.jsonl"
    before = rows_p.read_text()
    rulings = rj.rewrite_rows(camp, TASK, SIM, apply=False, tasks_dir=tasks,
                              out=lambda *a: None)
    assert rulings[0]["invalidate"] is True
    assert rows_p.read_text() == before
    assert not (camp / "cells" / ("%s_%s_r0" % (TASK, SIM))
                / "grading_recovery.json").exists()


def test_invalid_rows_gain_provenance_but_stay_invalid(tmp_path):
    # r0's shape: already INVALID (stack broke) AND contaminated -- the
    # outcome does not change but the evidence is recorded.
    camp, tasks = _mk_campaign(tmp_path, {0: ("INVALID", [FOREIGN],
                                              b"INITIAL")})
    rj.rewrite_rows(camp, TASK, SIM, apply=True, tasks_dir=tasks,
                    out=lambda *a: None)
    row = json.loads((camp / "groups" / ("%s_%s" % (TASK, SIM))
                      / "rows.jsonl").read_text().strip())
    assert row["outcome"] == "INVALID"
    assert row["grading_recovery"]["original_outcome"] == "INVALID"


# --- answer-task mode (B1/B3) ----------------------------------------------------

SELF_LAUNCH = json.dumps({"type": "assistant", "message": {"content": [{
    "type": "tool_use", "name": "Bash", "input": {
        "command": "cd /o/omnisim && python scripts/harness/"
                   "omnisim_harness.py --port 6789 &"}}]}})


def test_answer_task_found_foreign_world_invalidates(tmp_path):
    camp, tasks = _mk_campaign(tmp_path, {0: ("FAIL", [FOREIGN], b"INITIAL")})
    rulings = rj.rewrite_rows(camp, TASK, SIM, apply=True, tasks_dir=tasks,
                              answer_task=True, out=lambda *a: None)
    assert rulings[0]["invalidate"] is True
    assert rulings[0]["self_launched_harness"] is False
    row = json.loads((camp / "groups" / ("%s_%s" % (TASK, SIM))
                      / "rows.jsonl").read_text().strip())
    assert row["outcome"] == "INVALID"


def test_answer_task_self_launched_escape_stands(tmp_path):
    # B1 r4's shape: the agent LAUNCHED the harness itself -- a workspace
    # escape is the agent's own failure, never re-adjudicated
    camp, tasks = _mk_campaign(tmp_path, {
        0: ("FAIL", [SELF_LAUNCH, FOREIGN], b"INITIAL")})
    rulings = rj.rewrite_rows(camp, TASK, SIM, apply=True, tasks_dir=tasks,
                              answer_task=True, out=lambda *a: None)
    assert rulings[0]["invalidate"] is False
    assert rulings[0]["self_launched_harness"] is True
    row = json.loads((camp / "groups" / ("%s_%s" % (TASK, SIM))
                      / "rows.jsonl").read_text().strip())
    assert row["outcome"] == "FAIL"
    assert "grading_recovery" not in row


def test_answer_task_no_witness_stands(tmp_path):
    camp, tasks = _mk_campaign(tmp_path, {0: ("FAIL", [CLEAN], b"INITIAL")})
    rulings = rj.rewrite_rows(camp, TASK, SIM, apply=True, tasks_dir=tasks,
                              answer_task=True, out=lambda *a: None)
    assert rulings[0]["invalidate"] is False


def test_self_launch_detector_ignores_tool_results_mentioning_harness(
        tmp_path):
    # a tool RESULT that merely mentions the script (ls output, docs) is not
    # a launch
    line = json.dumps({"type": "user", "message": {"content": [{
        "type": "tool_result", "tool_use_id": "toolu_x",
        "content": "scripts/harness/omnisim_harness.py  README.md"}]}})
    assert rj.self_launched_harness(_w(tmp_path, [line])) is False
    assert rj.self_launched_harness(_w(tmp_path, [SELF_LAUNCH])) is True


# --- port hygiene ---------------------------------------------------------------


def test_port_listeners_sees_a_live_listener_and_reaper_protects_self():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)
        port = srv.getsockname()[1]
        hits = staging.port_listeners(ports=(port,))
        assert any(p == port for p, _pid in hits)
        # the listener is THIS process: the reaper must refuse to kill it
        # and record the collision instead
        records = staging.reap_port_listeners(ports=(port,))
        assert records, "collision must be recorded, not silently skipped"
        assert all(r["action"] == "skipped_protected" for r in records
                   if r["where"] == "port:%d" % port)
    finally:
        srv.close()


def test_port_listeners_empty_when_nothing_listens():
    # grab a free port, close it, and expect no listener on it
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    assert staging.port_listeners(ports=(port,)) == []
    assert staging.reap_port_listeners(ports=(port,)) == []


def test_default_ports_are_the_harness_and_capture_pairs():
    assert staging.DEFAULT_HARNESS_PORTS == (6789, 6790, 6791, 6792)


# --- artifact discovery through junctions ---------------------------------------

import os                                                        # noqa: E402
import time                                                      # noqa: E402

import pytest                                                    # noqa: E402

from agentbench.cc_lane import run_cc_cell as cell               # noqa: E402

needs_windows = pytest.mark.skipif(os.name != "nt",
                                   reason="junctions are Windows-only")


@needs_windows
def test_discover_artifact_finds_worlds_written_through_a_junction(tmp_path):
    # the A1 r0-r2 defect: the template junctions projects/ into a shared
    # target; the agent writes its world through the junction; the
    # link-refusing walk is blind to it and the cell blocked
    target = tmp_path / "shared_repo" / "projects" / "worlds"
    target.mkdir(parents=True)
    ws = tmp_path / "ws"
    ws.mkdir()
    staging.make_junction(ws / "projects", tmp_path / "shared_repo"
                          / "projects")
    start = time.time() - 1
    written = target / "authored.wbt"
    written.write_text("WorldInfo {}\n", encoding="utf-8")
    art, rule = cell.discover_artifact(ws, [], start)
    assert art is not None
    assert art.name == "authored.wbt"
    assert "junction" in rule
    # ...and a pre-existing (old-mtime) world through the junction is NOT
    # collected: the mtime gate holds on the fallback pass too
    old = target / "preexisting.wbt"
    old.write_text("WorldInfo {}\n", encoding="utf-8")
    os.utime(old, (start - 100, start - 100))
    art2, _ = cell.discover_artifact(ws, [], start)
    assert art2.name == "authored.wbt"


@needs_windows
def test_discover_artifact_still_prefers_workspace_proper_over_junction(
        tmp_path):
    target = tmp_path / "shared" / "projects"
    target.mkdir(parents=True)
    ws = tmp_path / "ws"
    ws.mkdir()
    staging.make_junction(ws / "projects", target)
    start = time.time() - 1
    (target / "through_junction.wbt").write_text("A", encoding="utf-8")
    (ws / "in_workspace.wbt").write_text("B", encoding="utf-8")
    art, rule = cell.discover_artifact(ws, [], start)
    assert art.name == "in_workspace.wbt"
    assert "junction" not in rule


# --- publish staging dir ---------------------------------------------------------


def test_publish_staging_dir_is_outside_groups(tmp_path):
    d = campaign_mod.publish_staging_dir(tmp_path / "camp", "phasew_B2_om")
    assert "groups" not in d.parts
    assert d.name == "phasew_B2_om"
    assert d.parent.name == "publish_staging"
    assert (tmp_path / "camp") in d.parents
