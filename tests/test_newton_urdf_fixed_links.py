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

"""URDF links on FIXED joints must reach Newton where, and as, the URDF says.

WHAT THIS PINS (found 2026-10-03 on the shipped TurtleBot3 Burger and the RT
Raspberry Pi Mouse; rigs under social/launch/company_outreach_2026-10-03/)
-----------------------------------------------------------------------------
The compiled MuJoCo model of an as-published Burger carried five defects, all
in how fixed-joint links are flattened into one Newton body:

1. RE-ROOT OFFSET DROPPED (OmUrdfImporter emitRobot). The importer promotes
   base_link past the empty base_footprint frame and dropped the +0.010 m
   base_joint offset, so the wheel bodies sat at z 0.023 in the Robot frame
   instead of 0.033. Hatch: OMNISIM_URDF_REROOT_KEEP_OFFSET=0.
2. COMPOSITE INERTIA OFF (OmSolid flush). The chassis got the SUMMED mass
   (0.9447 kg) but ipos (0,0,0) and base_link's own tensor -- the 0.114 kg
   lidar 172 mm up and the caster were never placed. Hatch:
   OMNISIM_NEWTON_COMPOSITE_INERTIA=0 (value-parsed; it was presence-gated
   opt-in, so `=0` used to ARM it).
3. FIXED-CHILD COLLIDERS DROPPED (OmSolid flush). Only the merge leader's own
   boundingObject registered, so the caster box never collided and the Burger
   rocked about its axle forever. Hatch: OMNISIM_NEWTON_FIXED_CHILD_COLLIDERS=0.
4. A second <collision> on base_link "not registered". Until 2026-10-17 a
   Robot wrapper's OWN boundingObject was skipped under the default
   newtonRobotColliders FALSE ("a chassis envelope starves the wheels").
   Since then it registers unless it reaches more than 2 mm below the robot's
   own wheels (World._resolve_robot_root_shapes): this Burger's chassis box
   (underside 8.5 mm above the tyre plane) and its caster bump (0.5 mm above)
   both collide by default. Hatch: OMNISIM_NEWTON_ROBOT_ROOT_COLLIDER=0.
5. URDF <dynamics damping friction> NOT APPLIED (OmBasicJoint flush ->
   set_joint_passive_dynamics). Hatch: OMNISIM_NEWTON_JOINT_DYNAMICS=0.

The probe is a primitives-only Burger (no meshes) using the published TB3
numbers, read back from the OMNISIM_NEWTON_DUMP_MJMODEL introspection dump.
Three launches: default (rcF), newtonRobotColliders+newtonCompoundColliders
TRUE (rcT), and every hatch at 0 (the pre-2026-10-03 model, byte for byte on
the asserted fields).

NEEDS AN ENGINE BUILT FROM THE 2026-10-03 SOURCES and the runtime bundle
re-staged with them; on an older binary the default-run assertions fail with
the old numbers, which is the point. Honours OMNISIM_BINARY.

    python -m pytest tests/test_newton_urdf_fixed_links.py -v
"""
from __future__ import annotations

import os
import re
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
WORLDS = REPO / "tests" / "physics" / "worlds"

BASE_JOINT_Z = 0.010
WHEEL_Z_IN_BASE_LINK = 0.023

# (mass, position in base_footprint, 3x3 tensor about own COM, frame-aligned)
BASE_LINK = (0.82573504, (0.0, 0.0, BASE_JOINT_Z),
             np.array([[2.2124416e-03, -1.2294101e-05, 3.4938785e-05],
                       [-1.2294101e-05, 2.1193702e-03, -5.0120904e-06],
                       [3.4938785e-05, -5.0120904e-06, 2.0064271e-03]]))
CASTER = (0.005, (-0.081, 0.0, BASE_JOINT_Z - 0.004), np.eye(3) * 0.001)
LIDAR = (0.114, (-0.032, 0.0, BASE_JOINT_Z + 0.172), np.eye(3) * 0.001)

URDF = """<?xml version="1.0"?>
<robot name="tb3probe">
  <link name="base_footprint"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/><child link="base_link"/>
    <origin xyz="0 0 0.010" rpy="0 0 0"/>
  </joint>
  <link name="base_link">
    <collision><origin xyz="-0.032 0 0.070"/><geometry><box size="0.140 0.140 0.143"/></geometry></collision>
    <collision><origin xyz="-0.081 0 -0.005"/><geometry><box size="0.030 0.020 0.009"/></geometry></collision>
    <inertial><origin xyz="0 0 0"/><mass value="8.2573504e-01"/>
      <inertia ixx="2.2124416e-03" ixy="-1.2294101e-05" ixz="3.4938785e-05" iyy="2.1193702e-03" iyz="-5.0120904e-06" izz="2.0064271e-03"/></inertial>
  </link>
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/><child link="wheel_left_link"/>
    <origin xyz="0.0 0.08 0.023" rpy="-1.57 0 0"/><axis xyz="0 0 1"/>
    <dynamics damping="1.0" friction="0.5"/>
  </joint>
  <link name="wheel_left_link">
    <collision><geometry><cylinder length="0.018" radius="0.033"/></geometry></collision>
    <inertial><mass value="2.8498940e-02"/>
      <inertia ixx="1.1175580e-05" ixy="0" ixz="0" iyy="1.1192413e-05" iyz="0" izz="2.0712558e-05"/></inertial>
  </link>
  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/><child link="wheel_right_link"/>
    <origin xyz="0.0 -0.080 0.023" rpy="-1.57 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <link name="wheel_right_link">
    <collision><geometry><cylinder length="0.018" radius="0.033"/></geometry></collision>
    <inertial><mass value="2.8498940e-02"/>
      <inertia ixx="1.1175580e-05" ixy="0" ixz="0" iyy="1.1192413e-05" iyz="0" izz="2.0712558e-05"/></inertial>
  </link>
  <joint name="caster_back_joint" type="fixed">
    <parent link="base_link"/><child link="caster_back_link"/>
    <origin xyz="-0.081 0 -0.004" rpy="-1.57 0 0"/>
  </joint>
  <link name="caster_back_link">
    <collision><origin xyz="0 0.001 0"/><geometry><box size="0.030 0.009 0.020"/></geometry></collision>
    <inertial><mass value="0.005"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
  </link>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/><child link="imu_link"/><origin xyz="-0.032 0 0.068"/>
  </joint>
  <link name="imu_link"/>
  <joint name="scan_joint" type="fixed">
    <parent link="base_link"/><child link="base_scan"/><origin xyz="-0.032 0 0.172"/>
  </joint>
  <link name="base_scan">
    <collision><origin xyz="0.015 0 -0.0065"/><geometry><cylinder length="0.0315" radius="0.055"/></geometry></collision>
    <inertial><mass value="0.114"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
  </link>
</robot>
"""

HATCHES_OFF = {
    "OMNISIM_URDF_REROOT_KEEP_OFFSET": "0",
    "OMNISIM_NEWTON_COMPOSITE_INERTIA": "0",
    "OMNISIM_NEWTON_FIXED_CHILD_COLLIDERS": "0",
    "OMNISIM_NEWTON_JOINT_DYNAMICS": "0",
    # The mobile-base lane's placeholder fix (2026-10-03) stops the 1 mm wrapper
    # sphere colliding once the robot has real colliders; the old model needs it back.
    "OMNISIM_NEWTON_WRAPPER_PLACEHOLDER_COLLIDES": "1",
    # The robot-root-collider rule (2026-10-17) registers base_link's own
    # boundingObject by default; the old model left it out.
    "OMNISIM_NEWTON_ROBOT_ROOT_COLLIDER": "0",
}


def _world_text(robot_colliders: bool) -> str:
    flag = "TRUE" if robot_colliders else "FALSE"
    return ("#OMNISIM R2025a utf8\n\n"
            "WorldInfo {\n  basicTimeStep 4\n  newtonSolver \"mujoco\"\n"
            f"  newtonRobotColliders {flag}\n  newtonCompoundColliders {flag}\n}}\n"
            "Viewpoint { position 1 -1 1 }\n"
            "Solid {\n  translation 0 0 -0.05\n  name \"floor\"\n"
            "  children [ Shape { geometry DEF FLOOR Box { size 4 4 0.1 } } ]\n"
            "  boundingObject USE FLOOR\n}\n"
            "URDFRobot {\n  url \"tb3probe.urdf\"\n  translation 0 0 0\n  name \"tb3probe\"\n"
            "  controller \"<none>\"\n}\n")


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override and Path(override).exists():
        return Path(override)
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None, reason="no omnisim-bin in this clone")

_VEC = r"\[([^\]]*)\]"
_BODY = re.compile(r"^body (\d+) \S+\s+mass=(\S+) ipos=" + _VEC + r" pos=" + _VEC + r" inertia=" + _VEC, re.M)
_GEOM = re.compile(r"^geom \d+ \S+\s+body=(\d+) type=(\d+) condim=\d+ contype=(\d+) conaff=(\d+) .* size="
                   + _VEC + r" pos=" + _VEC, re.M)
_DOF = re.compile(r"^dof (\d+) damping=(\S+) armature=\S+ frictionloss=(\S+)", re.M)


def _vec(s):
    return np.array([float(v) for v in s.replace("np.float64(", "").replace(")", "").split(",")])


def _dump(tmp_path, tag, robot_colliders, extra_env=None):
    WORLDS.mkdir(parents=True, exist_ok=True)
    stem = f".urdf_fixed_links_{tag}"
    world = WORLDS / f"{stem}.omniworld"
    urdf = WORLDS / "tb3probe.urdf"
    urdf.write_text(URDF, encoding="utf-8")
    world.write_text(_world_text(robot_colliders), encoding="utf-8")
    text = ""
    try:
        for attempt in (1, 2):  # the Newton FFI bring-up flakes on a few % of launches
            dump = tmp_path / f"mjmodel_{tag}_{attempt}.txt"
            env = {k: v for k, v in os.environ.items()
                   if not (k.startswith("OMNISIM_NEWTON_") or k.startswith("OMNISIM_URDF_"))}
            env.update(OMNISIM_HOME=os.environ.get("OMNISIM_HOME") or str(REPO), OMNISIM_LOG_PATH=str(tmp_path / f"engine_{tag}_{attempt}.log"),
                       OMNISIM_NEWTON_DUMP_MJMODEL=str(dump))
            env.update(extra_env or {})
            proc = subprocess.Popen(
                [str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                 "--stdout", "--stderr"],
                cwd=str(REPO), env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                deadline = time.time() + 120
                while time.time() < deadline and proc.poll() is None:
                    if dump.exists() and "\ndof " in dump.read_text(errors="replace"):
                        time.sleep(0.5)  # let the one-shot dump finish writing
                        text = dump.read_text(errors="replace")
                        break
                    time.sleep(0.5)
            finally:
                if proc.poll() is None:
                    if os.name == "nt":
                        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], capture_output=True)
                    else:
                        proc.kill()
                    proc.wait()
            if text:
                break
    finally:
        for p in (world, urdf):
            try:
                p.unlink()
            except OSError:
                pass
    assert text, f"{tag}: the engine wrote no MuJoCo dump (Newton did not come up?)"
    return text


def _parse(text):
    bodies = {int(m.group(1)): dict(mass=float(m.group(2)), ipos=_vec(m.group(3)), pos=_vec(m.group(4)),
                                     inertia=_vec(m.group(5)))
              for m in _BODY.finditer(text)}
    geoms = [dict(body=int(m.group(1)), type=int(m.group(2)), collides=int(m.group(3)) != 0 or int(m.group(4)) != 0,
                  size=_vec(m.group(5)), pos=_vec(m.group(6)))
             for m in _GEOM.finditer(text)]
    dofs = {int(m.group(1)): (float(m.group(2)), float(m.group(3))) for m in _DOF.finditer(text)}
    # the chassis is the robot body carrying the free joint: the heaviest one
    chassis = max((b for b in bodies if b != 0), key=lambda b: bodies[b]["mass"])
    wheels = sorted(b for b in bodies if b not in (0, chassis) and abs(bodies[b]["mass"] - 0.0285) < 1e-3)
    return bodies, geoms, dofs, chassis, wheels


def _composite():
    parts = (BASE_LINK, CASTER, LIDAR)
    m = sum(p[0] for p in parts)
    c = sum(p[0] * np.array(p[1]) for p in parts) / m
    inertia = np.zeros((3, 3))
    for pm, pc, pI in parts:
        d = np.array(pc) - c
        inertia += pI + pm * (d.dot(d) * np.eye(3) - np.outer(d, d))
    return m, c, np.sort(np.linalg.eigvalsh(inertia))


@pytest.fixture(scope="module")
def dumps(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("urdf_fixed_links")
    return {
        "rcF": _parse(_dump(tmp, "rcF", False, {"OMNISIM_NEWTON_JOINT_DYNAMICS": "1"})),
        "rcT": _parse(_dump(tmp, "rcT", True, {"OMNISIM_NEWTON_JOINT_DYNAMICS": "1"})),
        "off": _parse(_dump(tmp, "off", False, HATCHES_OFF)),
    }


def test_reroot_offset_is_kept(dumps):
    bodies, _g, _d, chassis, wheels = dumps["rcF"]
    assert len(wheels) == 2, f"expected two wheel bodies, got {wheels}"
    for w in wheels:
        z = bodies[w]["pos"][2]
        assert abs(z - (BASE_JOINT_Z + WHEEL_Z_IN_BASE_LINK)) < 1e-6, (
            f"wheel body z {z:.4f} in the Robot frame; 0.033 is base_joint (0.010) + the wheel joint (0.023). "
            f"0.023 is the dropped re-root offset (OMNISIM_URDF_REROOT_KEEP_OFFSET)")


def test_composite_mass_com_inertia(dumps):
    bodies, _g, _d, chassis, _w = dumps["rcF"]
    m, c, eig = _composite()
    b = bodies[chassis]
    assert abs(b["mass"] - m) < 1e-5, f"chassis mass {b['mass']} vs composite {m}"
    # The MuJoCo text dump prints body ipos to 3 decimals, so allow half a unit in
    # the last printed place (5e-4) plus a little slack.
    assert np.allclose(b["ipos"], c, atol=6e-4), (
        f"chassis COM {b['ipos']} vs composite {c} (base_footprint frame) -- (0,0,0) is the leader-only rollup")
    assert np.allclose(np.sort(b["inertia"]), eig, rtol=2e-3), (
        f"chassis principal inertia {np.sort(b['inertia'])} vs composite {eig}")


def test_fixed_child_colliders_register_with_robot_colliders_false(dumps):
    _b, geoms, _d, chassis, _w = dumps["rcF"]
    on_chassis = [g for g in geoms if g["body"] == chassis]
    sizes = [g["size"] for g in on_chassis]
    assert not any(abs(s[0] - 0.001) < 1e-6 for s in sizes), (
        f"the chassis still carries the 1 mm wrapper placeholder: {sizes}")
    # caster box (type 6) with the -1.57 rotation folded in, underside ~1.5 mm above the floor
    casters = [g for g in on_chassis if g["type"] == 6 and abs(sorted(g["size"])[0] - 0.0045) < 1e-4]
    assert casters, f"caster box missing from the chassis body: {on_chassis}"
    assert np.allclose(casters[0]["pos"][[0, 2]], [-0.081, BASE_JOINT_Z - 0.004], atol=2e-3), casters[0]
    # capsule (3) before 403f36a84; a real cylinder (5) since a fixed-child Cylinder stays one
    lidars = [g for g in on_chassis if g["type"] in (3, 5) and abs(g["size"][0] - 0.055) < 1e-4]
    assert lidars, f"lidar capsule missing from the chassis body: {on_chassis}"
    # since 2026-10-17 the wrapper's OWN chassis box registers AND collides by default: its
    # underside is 8.5 mm above the tyre plane, so it cannot hold the wheels off the floor
    own = [g for g in on_chassis if g["type"] == 6 and abs(g["size"][0] - 0.07) < 1e-4]
    assert own and all(g["collides"] for g in own), (
        f"base_link's own chassis box must register and collide by default (robot root collider rule): {on_chassis}")


def test_every_collision_registers_with_robot_and_compound_colliders(dumps):
    _b, geoms, _d, chassis, _w = dumps["rcT"]
    boxes = sorted(tuple(np.round(sorted(g["size"]), 4)) for g in geoms if g["body"] == chassis and g["type"] == 6)
    assert (0.07, 0.07, 0.0715) in boxes, f"base_link's chassis box: {boxes}"
    assert boxes.count((0.0045, 0.01, 0.015)) == 2, (
        f"base_link's SECOND collision and the caster link's box must both register: {boxes}")


def test_urdf_wheel_dynamics_reach_mujoco(dumps):
    _b, _g, dofs, _c, _w = dumps["rcF"]
    # free joint = dofs 0..5, wheels follow in registration order
    wheel = {k: v for k, v in dofs.items() if k >= 6}
    assert (1.0, 0.5) in wheel.values(), f"left wheel <dynamics damping=1 friction=0.5> not applied: {wheel}"
    assert (0.0, 0.0) in wheel.values(), f"right wheel declares no dynamics and must keep none: {wheel}"


def test_hatches_restore_the_old_model(dumps):
    bodies, geoms, dofs, chassis, wheels = dumps["off"]
    for w in wheels:
        assert abs(bodies[w]["pos"][2] - WHEEL_Z_IN_BASE_LINK) < 1e-6, bodies[w]
    assert np.allclose(bodies[chassis]["ipos"], 0.0, atol=1e-9), bodies[chassis]
    on_chassis = [g for g in geoms if g["body"] == chassis]
    assert len(on_chassis) == 1 and abs(on_chassis[0]["size"][0] - 0.001) < 1e-6, on_chassis
    assert all(v == (0.0, 0.0) for k, v in dofs.items() if k >= 6), dofs
