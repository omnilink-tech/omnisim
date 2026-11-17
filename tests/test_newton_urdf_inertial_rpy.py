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

"""URDF <inertial><origin rpy> rotates the inertia tensor into the link frame (engine half).

The defect this pins (2026-10-15): the native URDF importer (OmUrdfImporter.cpp)
used only the inertial origin's xyz (Physics.centerOfMass) and dropped its rpy, so
a tensor written in a rotated inertial frame reached the solver unrotated. Mass and
centre of mass were right; the rotational inertia about every axis was not.
spot_description's hips and lower legs arrived with their principal moments on the
wrong axes. parseLink now emits I_link = R I R^T (URDF fixed-axis rpy).

Probe: two passive (effort 0) pendulums hinged about Y, arm COM 0.1 m out, 1 kg,
released horizontal. `rot` declares a rod along its inertial X (ixx .001, iyy .05,
izz .05) in a frame yawed 90 deg, so the rod lies along the hinge axis and the
link-frame I_y(com) is 0.001; `ref` declares the same body pre-rotated with rpy 0.
They must swing together. With the rpy dropped (OMNISIM_URDF_INERTIAL_RPY=0) `rot`
gets I_y(com) 0.05 -- 0.06 about the pivot instead of 0.011 -- and lags far behind.

The engine-free half (the offline converter and the convention) is
tests/dev/test_urdf_import_inertial_rpy.py.

    python -m pytest tests/test_newton_urdf_inertial_rpy.py -v

OMNISIM_BINARY selects a scratch build. Two engine launches, a few seconds each.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
PIVOT_Z = 0.5
ARM = 0.1

_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)

ARMS = {
    "rot": ("0 0 1.5707963267948966", 'ixx="0.001" iyy="0.05" izz="0.05" ixy="0" ixz="0" iyz="0"'),
    "ref": ("0 0 0", 'ixx="0.05" iyy="0.001" izz="0.05" ixy="0" ixz="0" iyz="0"'),
}


def _urdf(tag: str, rpy: str, inertia: str) -> str:
    return f"""<?xml version="1.0"?>
<robot name="pend_{tag}">
  <link name="base_{tag}">
    <inertial><mass value="1.0"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <link name="arm_{tag}">
    <inertial><origin xyz="{ARM} 0 0" rpy="{rpy}"/><mass value="1.0"/><inertia {inertia}/></inertial>
  </link>
  <joint name="swing_{tag}" type="continuous">
    <parent link="base_{tag}"/><child link="arm_{tag}"/><axis xyz="0 1 0"/>
    <limit effort="0" velocity="100"/>
  </joint>
  <link name="bob_{tag}">
    <inertial><mass value="0.001"/><inertia ixx="1e-7" iyy="1e-7" izz="1e-7" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <joint name="bob_mount_{tag}" type="fixed">
    <parent link="arm_{tag}"/><child link="bob_{tag}"/><origin xyz="{ARM} 0 0"/>
  </joint>
</robot>
"""


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(_binary() is None, reason="no simulator binary in this clone; build first")


def _bob_heights(tmp_path: Path, extra_env: dict | None = None) -> dict:
    robots = []
    for i, (tag, (rpy, inertia)) in enumerate(ARMS.items()):
        u = tmp_path / f"pend_{tag}.urdf"
        u.write_text(_urdf(tag, rpy, inertia), encoding="utf-8")
        robots.append(f'URDFRobot {{ url "{u.as_posix()}" translation 0 {i * 1.0} {PIVOT_Z} name "pend_{tag}" '
                      f'staticBase TRUE controller "<none>" }}')
    world = tmp_path / "pendulums.omniworld"
    world.write_text("#VRML_SIM R2025a utf8\nWorldInfo { basicTimeStep 2 }\n"
                     "Viewpoint { orientation 0 0 1 0 position -3 0 1 }\n" + "\n".join(robots) + "\n",
                     encoding="utf-8")
    traj, log = tmp_path / "traj.tsv", tmp_path / "log.txt"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_ALL="1", OMNISIM_PROBE_TRAJ_MS="300")
    env.pop("OMNISIM_URDF_INERTIAL_RPY", None)
    env.update(extra_env or {})
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        "--stdout", "--stderr", str(world)], env=env, timeout=120, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    z: dict = {}
    if traj.is_file():
        for line in traj.read_text(encoding="utf-8").splitlines():
            c = line.split("\t")
            if len(c) >= 5 and c[1].startswith("bob_"):
                z.setdefault(c[1], {})[round(float(c[0]))] = float(c[4])
    if not z:
        text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up (%r); the run produced no data" % sig)
        pytest.fail("the pendulum probe produced no trajectory:\n%s" % text[-1500:])
    return z


def _drop_at(series: dict, t_ms: int) -> float:
    return PIVOT_Z - series[t_ms]


def test_rotated_inertial_frame_swings_like_its_prerotated_twin(tmp_path):
    z = _bob_heights(tmp_path)
    assert {"bob_rot", "bob_ref"} <= set(z), f"bobs missing from the trajectory probe: {sorted(z)}"
    rot, ref = z["bob_rot"], z["bob_ref"]
    common = sorted(set(rot) & set(ref))
    assert common and common[-1] >= 200, f"trajectory too short: last common sample at {common[-1:]} ms"
    assert _drop_at(ref, 150) > 0.03, (
        f"the reference pendulum should be swinging by 150 ms (I_pivot 0.011); bob dropped only "
        f"{_drop_at(ref, 150):.4f} m")
    worst = max(abs(rot[t] - ref[t]) for t in common)
    assert worst < 0.002, (
        f"a tensor in a yawed inertial frame must swing exactly like the same tensor written pre-rotated; "
        f"bob z differs by up to {worst:.4f} m (rot drop at 150 ms {_drop_at(rot, 150):.4f}, ref "
        f"{_drop_at(ref, 150):.4f}). The <inertial> origin rpy is being dropped")


def test_hatch_restores_the_unrotated_tensor(tmp_path):
    z = _bob_heights(tmp_path, {"OMNISIM_URDF_INERTIAL_RPY": "0"})
    rot, ref = z["bob_rot"], z["bob_ref"]
    assert _drop_at(rot, 150) < 0.5 * _drop_at(ref, 150), (
        f"OMNISIM_URDF_INERTIAL_RPY=0 must restore the unrotated tensor (I_pivot 0.06 instead of 0.011), "
        f"which swings far slower; drops at 150 ms rot {_drop_at(rot, 150):.4f} vs ref "
        f"{_drop_at(ref, 150):.4f}")
