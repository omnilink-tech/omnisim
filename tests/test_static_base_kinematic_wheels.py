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

"""A staticBase wheeled robot must not be driven by Webots' kinematic wheel model.

Engine-free half of the pin; the live arms (base displacement measured on a
headless run) are tests/test_static_base_kinematic_wheels_engine.py, whose
docstring has the measurements.

The defect (found 2026-10-20): `URDFRobot { staticBase TRUE }` strips only the
root link's Physics, so the Robot node has none while its wheel Solids keep
theirs. OmKinematicDifferentialWheels (inherited from upstream) attached to any
Robot without Physics that has two equal, equidistant cylinder wheels on
parallel axes, and moved the whole robot by omega * r each step: a staticBase
NORI A3 slid 0.61 m in 2 s with its wheels at 4 rad/s, z frozen. The guard now
refuses the kinematic model when either wheel carries Physics (such a Robot is a
static base under docs/reference/physics.md); value-parsed hatch
OMNISIM_KINEMATIC_WHEELS_WITH_PHYSICS=1 restores the slide. The Newton weld
census also names the physics-less owner of the welded links, so the line can
no longer be read as a FLOATING robot being partly welded (the misreading that
started this).

    python -m pytest tests/test_static_base_kinematic_wheels.py -v
"""

from __future__ import annotations

import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
DW_CPP = REPO / "src" / "omnisim" / "nodes" / "utils" / "OmKinematicDifferentialWheels.cpp"
SOLID_CPP = REPO / "src" / "omnisim" / "nodes" / "OmSolid.cpp"
HATCH = "OMNISIM_KINEMATIC_WHEELS_WITH_PHYSICS"


# --------------------------------------------------------------------------- source arms


def _create_body() -> str:
    text = DW_CPP.read_text(encoding="utf-8")
    start = text.index("OmKinematicDifferentialWheels::createKinematicDifferentialWheelsIfNeeded(OmRobot *robot)")
    return text[start:]


def test_hatch_is_value_parsed_and_off_by_default():
    text = DW_CPP.read_text(encoding="utf-8")
    m = re.search(r"static bool kinematicWheelsWithPhysicsEnabled\(\) \{(.*?)\n\}", text, re.S)
    assert m, "the hatch reader is gone"
    body = m.group(1)
    assert f'getenv("{HATCH}")' in body
    # unset -> OFF; "0" / "false" / "off" / "no" -> OFF (value-parsed, not presence-gated)
    assert "!v.empty() &&" in body
    for off in ('"0"', '"false"', '"off"', '"no"'):
        assert off in body


def test_wheels_with_physics_refuse_the_kinematic_mode_before_it_is_built():
    body = _create_body()
    guard = body.find("solidEndPoint()->physics() != nullptr")
    built = body.find("return new OmKinematicDifferentialWheels")
    assert guard != -1, "the physics-bearing-wheels guard is gone"
    assert built != -1
    assert guard < built, "the guard must run before the kinematic model is constructed"
    assert "!kinematicWheelsWithPhysicsEnabled()" in body[:built]
    # both wheels are checked, and the refusal says so in the log
    assert body.count("solidEndPoint()->physics() != nullptr") >= 2
    assert "is a static base" in body and HATCH in body


def test_weld_census_names_the_physics_less_owner():
    text = SOLID_CPP.read_text(encoding="utf-8")
    i = text.index("Solid(s) with Physics under physics-less ancestors attached to the ")
    line = text[i:i + 900]
    assert "a static base" in line and "not a free one" in line
    assert "weldedToWorldOwners" in line
