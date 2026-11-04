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

"""A URDF <rest> start angle survives a full-turn range; effort="0" is logged once and consistently.

Engine-free source pins for three defects found on the Ti5 T170 lane (2026-10-19,
fixed 2026-10-23). The behaviour itself -- sensors reading the rest angle on the
first step, sensor and physical angle agreeing, the log lines -- is checked by a
real engine run in tests/test_urdf_rest_wide_range.py; these pins keep the unit
lane from going green while the rule is quietly put back.

1. initialJointPosition() in src/omnisim/vrml/OmUrdfImporter.cpp returned early for
   a revolute joint whose limits cover a full turn (lower <= -(pi-0.01) and
   upper >= pi-0.01) BEFORE it read <rest>, and never read it for a continuous
   joint, so those joints started at 0 with nothing logged. The rest must be
   read first, and the `position` line must not be gated on the full-range test.
2. Every importer warning was printed twice: OmProtoManager::undeclaredProtoNodes
   tokenizes the world as a pre-scan, and the tokenizer expanded each URDFRobot
   there as well as in the load proper.
3. For an effort="0" joint the importer says "passive", and OmBasicJoint followed
   with "a SYNTHETIC position gain ... OmniSim drives it" -- false since the
   zero effort limit reaches the solver (2026-10-10). That warning now fires
   only with OMNISIM_NEWTON_AVAILABLE_TORQUE=0, the one case where it is true.

    python -m pytest tests/dev/test_urdf_rest_full_turn.py -v
"""

from __future__ import annotations

import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SRC = REPO / "src" / "omnisim"


def _read(rel: str) -> str:
    return (SRC / rel).read_text(encoding="utf-8")


def _function(src: str, signature: str) -> str:
    """The body of a top-level C++ function: from its signature to the first column-0 brace."""
    start = src.index(signature)
    end = src.index("\n}\n", start)
    return src[start:end]


IMPORTER = _read("vrml/OmUrdfImporter.cpp")
RULE = _function(IMPORTER, "bool initialJointPosition(const UrdfJoint &joint, double &q0) {")
PARAMS = _function(IMPORTER, "void appendJointPhysicsParameters(QString &out, const UrdfJoint &joint,")


def test_rest_is_read_before_the_full_turn_return():
    rest = RULE.index("if (joint.hasRest)")
    full = RULE.index("joint.lower <= -kPiLimit && joint.upper >= kPiLimit")
    assert rest < full, "a <rest> must win before the full-revolution early return drops it"


def test_rest_is_read_before_the_limits_are_required():
    rest = RULE.index("if (joint.hasRest)")
    needs_limits = RULE.index("!joint.hasLower || !joint.hasUpper")
    assert rest < needs_limits, "a continuous joint, or one with incomplete limits, must keep its <rest>"


def test_continuous_joints_reach_the_rest_branch():
    gate = re.search(r'if \(joint\.type != "revolute" && joint\.type != "continuous"\)\s*return false;', RULE)
    assert gate and gate.start() < RULE.index("if (joint.hasRest)")


def test_position_line_is_not_gated_on_the_full_range():
    calls = [ln for ln in PARAMS.splitlines() if "initialJointPosition(joint, q0)" in ln]
    assert calls, "appendJointPhysicsParameters must still write the `position` line"
    assert not any("fullRange" in ln for ln in calls), calls
    assert re.search(r'if \(joint\.type == "revolute" \|\| joint\.type == "continuous"\) \{\s*'
                     r'double q0 = 0\.0;\s*if \(initialJointPosition\(joint, q0\)\)', PARAMS)


def test_rest_outside_its_limits_is_named():
    assert "lies outside its own limits" in PARAMS and ".arg(joint.name)" in PARAMS


def test_world_prescan_does_not_expand_urdf_robots():
    prescan = _function(_read("vrml/OmProtoManager.cpp"),
                        "QMap<QString, QString> OmProtoManager::undeclaredProtoNodes(")
    off = prescan.index("tokenizer.setExpandUrdfRobots(false);")
    assert off < prescan.index("tokenizer.tokenize(filename);")
    tokenizer = _read("vrml/OmTokenizer.cpp")
    assert re.search(r'if \(mExpandUrdfRobots && contents\.contains\("URDFRobot"\)\)', tokenizer)
    assert "mExpandUrdfRobots(true)" in tokenizer, "the load proper must keep expanding"


def test_synthetic_gain_warning_only_when_effort_zero_is_not_applied():
    joint = _read("nodes/OmBasicJoint.cpp")
    passive = joint.index("if (effortLimit <= 0.0 && motor != nullptr && newtonAvailableTorqueEnabled()) {")
    synthetic = joint.index("SYNTHETIC position gain")
    fallback = joint.index("} else if (effortLimit <= 0.0 && motor != nullptr) {")
    assert passive < fallback < synthetic, "the SYNTHETIC warning must sit in the hatch-off branch"
    assert "so the joint is passive" in joint[passive:fallback]
    assert "OmLog::info(" in joint[passive:fallback], "the passive case states a fact, it is not a warning"
