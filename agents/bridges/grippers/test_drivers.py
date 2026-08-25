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

# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Register-level + surface tests for the real-gripper drivers.

Runs with no hardware (DryTransport). Works under pytest or as a script:

    python agents/bridges/grippers/test_drivers.py
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from grippers import make_driver, GRIPPER_SPECS                    # noqa: E402
from grippers.robotiq import (                                     # noqa: E402
    Robotiq2FDriver, pack_output, unpack_input, OUTPUT_BASE)


def test_robotiq_pack_unpack_roundtrip():
    regs = pack_output(r_act=1, r_gto=1, position=200, speed=255, force=100)
    # position lands in byte3 = low byte of reg1
    assert regs[1] & 0xFF == 200
    # rACT/rGTO land in byte0 = high byte of reg0
    assert (regs[0] >> 8) & 0x09 == 0x09
    dec = unpack_input(regs)
    assert dec["gPR"] == 200


def test_robotiq_width_to_position():
    g = Robotiq2FDriver(max_width=0.085)
    g.connect()
    g.open()
    assert g.state()["rPR"] == 0          # open -> 0
    g.close()
    assert g.state()["rPR"] == 255        # closed -> 255
    g.set_width(0.0425)                   # half
    assert 120 <= g.state()["rPR"] <= 136
    # commands actually hit the transport
    writes = [op for op in g.transport.log if op[0] == "write_registers"
              and op[1][0] == OUTPUT_BASE]
    assert len(writes) >= 4


def test_every_spec_builds_and_runs():
    for gid in GRIPPER_SPECS:
        d = make_driver(gid)
        d.connect()
        for verb in (d.open, d.close, d.release):
            st = verb()
            assert {"kind", "model", "holding", "connected"} <= set(st)
        if d.max_width:
            assert d.set_width(d.max_width * 0.5)["width"] is not None
        assert d.grasp(force=0.5)["holding"] is True
        assert d.release()["holding"] is False


def test_unknown_id_falls_back_to_mock():
    d = make_driver("does_not_exist")
    assert d.driver_id == "mock"
    assert "unknown" in d.model


def test_vacuum_has_no_width_control():
    d = make_driver("vacuum")
    assert d.set_width(0.01).get("error") == "width_control_unavailable"
    assert d.capabilities()["has_width_control"] is False


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"{len(fns)} tests passed")


if __name__ == "__main__":
    _run_all()
