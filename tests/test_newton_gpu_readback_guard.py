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

"""`raycast_batch` and `get_contacts` must not read mj_data off the CPU solver.

WHY THIS EXISTS (internal parity plan, item W1.1)
-------------------------------------------
newton's `SolverMuJoCo.step()` writes `self.mj_data` ONLY on its
`use_mujoco_cpu` branch (solver_mujoco.py:3831-3838). The `mujoco_warp` branch
steps `mjw_data` on the device and never touches mj_data again -- but mj_data
still EXISTS there, because `put_data()` seeds the GPU copy from it at build
time (:7092, :7461).

So a reader of mj_data that is not CPU-scoped does not fail on the GPU path.
It answers, confidently and with no diagnostic of any kind, against the scene
AS AUTHORED AT t=0. `weld_engage` (-2), `weld_release` (-2), `touch_force`
([]), `_capture_constraint_readbacks` and `_refresh_mj_cartesian` all carry the
guard. `raycast_batch` and `get_contacts` did not, which put the entire ray
sensor family (DistanceSensor / Receiver / LightSensor / Radar / Camera
recognition occlusion) and the entire contact family (getContactPoints,
/sim/contacts, /sim/grips, the damage tracker) on frozen data under
`newtonSolver "mujoco_warp"`.

THE RIG. A stub solver, so the assertion is on the GUARD and not on a GPU
being present: mj_data records whether it was read at all. The three arms are
the three states that matter and each must produce a different answer --
    CPU  (use_mujoco_cpu=True)  -> mj_data IS read      (nothing changed)
    GPU  (use_mujoco_cpu=False) -> mj_data is NOT read  (the fix)
    GPU + OMNISIM_NEWTON_GPU_STALE_READBACK=1 -> read again (exact revert)
-- so a guard that fired always, or never, fails here either way. Deleting
either `_gpu_readback_declined(...)` call site turns the GPU arms red.

    python -m pytest tests/test_newton_gpu_readback_guard.py -v
"""
from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
PHYSICS = REPO / "src" / "omnisim" / "physics"

HATCH = "OMNISIM_NEWTON_GPU_STALE_READBACK"


def _runtime():
    """Import the engine's Newton runtime module with warp/newton stubbed.

    The module imports `warp` and `newton` at top level for the embedded
    interpreter's benefit; neither is needed to exercise a guard, and pinning
    the test to a machine with the GPU stack installed would make it skip
    exactly where it is most likely to regress.
    """
    for name in ("warp", "newton"):
        sys.modules.setdefault(name, types.ModuleType(name))
    if str(PHYSICS) not in sys.path:
        sys.path.insert(0, str(PHYSICS))
    import omnisim_newton_runtime  # noqa: PLC0415  (deliberately late)
    return omnisim_newton_runtime


class _Reached(Exception):
    """Raised from the first thing raycast_batch does past the guard."""


class _StubData:
    """mj_data. Reading `ncon` is the tell that the frozen array was consulted."""

    def __init__(self):
        self.read = False

    @property
    def ncon(self):
        self.read = True
        return 0


class _StubSolver:
    def __init__(self, cpu):
        self.use_mujoco_cpu = cpu
        self.mj_model = object()
        self.mj_data = _StubData()


def _raise_reached():
    raise _Reached("raycast_batch got past the guard")


def _world(cpu):
    """A World with no physics in it -- only the two readbacks are exercised."""
    mod = _runtime()
    w = mod.World.__new__(mod.World)
    sv = _StubSolver(cpu)
    w._mjc_solver = lambda: sv
    w.model = None                 # narrow-phase fallback then yields []
    w.state_a = None
    w._raycast_maps = lambda: ([], [])
    return w, sv


# --------------------------------------------------------------------- rays

def test_raycast_declines_on_the_gpu_solver(monkeypatch):
    monkeypatch.delenv(HATCH, raising=False)
    w, sv = _world(cpu=False)
    out = w.raycast_batch([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 5.0])
    assert out == [], ("raycast_batch answered on mujoco_warp; the C++ side "
                       "unpacks a full-length list as a real verdict")
    assert w._gpu_readback_declined("raycast") is True


def test_raycast_still_serves_the_cpu_solver(monkeypatch):
    """The guard must be scoped, not blanket: CPU mj_step is the live path."""
    monkeypatch.delenv(HATCH, raising=False)
    w, sv = _world(cpu=True)
    assert w._gpu_readback_declined("raycast") is False
    w._raycast_maps = _raise_reached
    with pytest.raises(_Reached):
        w.raycast_batch([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 5.0])


# ----------------------------------------------------------------- contacts

def test_contacts_do_not_read_frozen_mj_data_on_the_gpu_solver(monkeypatch):
    monkeypatch.delenv(HATCH, raising=False)
    w, sv = _world(cpu=False)
    assert w.get_contacts() == []
    assert not sv.mj_data.read, (
        "get_contacts read mj_data.ncon under mujoco_warp. That array is "
        "frozen at the build pose: ncon is 0 for the whole run, so the "
        "`_cn <= 0` early return published 'nothing is touching' for every "
        "contact consumer AND made the live newton narrow-phase below it "
        "unreachable")


def test_contacts_still_read_mj_data_on_the_cpu_solver(monkeypatch):
    monkeypatch.delenv(HATCH, raising=False)
    w, sv = _world(cpu=True)
    w.get_contacts()
    assert sv.mj_data.read, ("get_contacts stopped reading mj_data on CPU "
                             "mj_step, where it is the authoritative source")


# -------------------------------------------------------------------- hatch

@pytest.mark.parametrize("value", ["1", "true", "on"])
def test_hatch_restores_the_pre_fix_behaviour_exactly(monkeypatch, value):
    """An exact-revert switch that has quietly stopped reverting is worse than
    none, and it is what makes the arms above evidence rather than assertion:
    one env var apart, the same rig reproduces the defect."""
    monkeypatch.setenv(HATCH, value)
    w, sv = _world(cpu=False)
    w.get_contacts()
    assert sv.mj_data.read, "%s=%s no longer restores the mj_data read" % (HATCH, value)

    w2, _ = _world(cpu=False)
    w2._raycast_maps = _raise_reached
    with pytest.raises(_Reached, match="past the guard"):
        w2.raycast_batch([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 5.0])


@pytest.mark.parametrize("value", ["0", "false", "off", "no", ""])
def test_hatch_is_value_parsed_not_presence_gated(monkeypatch, value):
    """`OMNISIM_REQUIRE_NEWTON` is presence-gated and `=0` ARMS it; that trap is
    documented in AGENTS.md and must not be reproduced by a new variable."""
    monkeypatch.setenv(HATCH, value)
    w, sv = _world(cpu=False)
    w.get_contacts()
    assert not sv.mj_data.read, ("%s=%s must mean OFF -- a presence-gated read "
                                 "makes '=0' arm the hatch" % (HATCH, value))
