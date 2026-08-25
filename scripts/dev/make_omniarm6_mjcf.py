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

"""Emit a TORQUE-LIMITED, fixed-base OMNIARM6 MJCF for the Shadowing Ghost Generator.

The toss-to-place ghost (Shadowing Component 1) needs a MuJoCo model of OMNIARM6 whose
generated swing is *feasible by construction* w.r.t. the real motor torques. Unlike the G1
walk MJCF (which deliberately left actuator force unbounded to byte-match the deploy PD), the
throw ghost MUST respect torque limits -- the whole feasibility story on this arm is that the
weak distal joints (J5/J6 = 34 N.m) cannot whip the payload, so a feasible throw has to be
powered by the strong proximal joints (J1/J2 = 194 N.m, J3/J4 = 102 N.m). We therefore set
each actuator's ``forcerange`` to the URDF effort limit, so the MPPI planner physically
cannot exceed motor torque.

Differences from make_g1_mjcf.py (deliberate, documented):
  * FIXED BASE -- no freejoint; link0 is welded to the world (a bolted-down arm).
  * position actuators are named ``{joint}_pos`` (the Ghost Generator's control space) and
    carry forcerange=+/-effort and ctrlrange=joint range (torque + joint-limit honest).
  * a small ``payload`` body is welded at the gripper TCP carrying the thrown object's mass,
    so the swing's torque feasibility includes the part being thrown. It is also the body the
    generator reads release position/velocity from (task_body="payload").

Run from a native shell where ``mujoco`` (CPU) imports; mjwarp/CUDA are NOT needed.

  python scripts/dev/make_omniarm6_mjcf.py            # generate
  python scripts/dev/make_omniarm6_mjcf.py --verify   # generate, then CPU-validate
"""
from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
URDF = REPO / "projects/robots/omnisim/omniarm6/omniarm6.urdf"
MJCF_OUT = REPO / "projects/robots/omnisim/omniarm6/mjcf/omniarm6_throw.mjcf.xml"

ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")

# Position-actuator gain. kp is high so the controller saturates torque (bang-bang) during the
# fling. We deliberately use NO separate velocity actuator (KV=0): velocity damping + the motor
# speed cap both come from per-joint DAMPING set to a motor curve b = effort/vel_limit, so the
# net torque reaches zero exactly at the rated joint speed -- the joint physically cannot exceed
# its velocity limit (the binding feasibility constraint for a light thrown payload). This makes
# the generated swing feasible w.r.t. BOTH torque (forcerange) and velocity (damping) by
# construction, with no fragile planner penalty to tune.
KP = 300.0
KV = 0.0
SUB_DT = 0.002

# Default thrown-part mass (kg) welded at the TCP -- a representative graspable line part.
PAYLOAD_MASS = 0.2
PAYLOAD_TCP_OFFSET_Z = 0.04   # object centre below/above the flange, in link6 frame (m)
LINK6_TO_FLANGE_Z = 0.1655    # link6flange fixed-joint offset from the URDF


def _urdf_limits(urdf_path: Path):
    """Return {joint: (lo, hi, effort, velocity)} from the URDF <limit> tags."""
    root = ET.parse(str(urdf_path)).getroot()
    out = {}
    for j in root.findall("joint"):
        name = j.get("name")
        lim = j.find("limit")
        if lim is None:
            continue
        out[name] = (float(lim.get("lower", "0")), float(lim.get("upper", "0")),
                     float(lim.get("effort", "0")), float(lim.get("velocity", "0")))
    return out


def _stripped_urdf_xml(urdf_path: Path) -> str:
    """URDF with all <visual>/<collision> removed -> MjSpec parses it with no mesh/meshdir
    dependency. This is a dynamics model; render + collision geometry are not needed for the
    arm-only swing ghost (the thrown object's flight is exact projectile physics, decoupled)."""
    root = ET.parse(str(urdf_path)).getroot()
    for link in root.findall("link"):
        for tag in ("visual", "collision"):
            for el in list(link.findall(tag)):
                link.remove(el)
    return ET.tostring(root, encoding="unicode")


def build_spec(payload_mass: float = PAYLOAD_MASS):
    import mujoco

    limits = _urdf_limits(URDF)
    spec = mujoco.MjSpec.from_string(_stripped_urdf_xml(URDF))
    spec.compiler.fusestatic = 0          # keep flange/gripper_tcp welded bodies addressable
    spec.modelname = "omniarm6_throw"
    spec.option.timestep = SUB_DT

    # --- ground plane (harmless; the arm-only swing never touches it) ---
    if not any(g.type == mujoco.mjtGeom.mjGEOM_PLANE for g in spec.worldbody.geoms):
        ground = spec.worldbody.add_geom()
        ground.name = "ground"
        ground.type = mujoco.mjtGeom.mjGEOM_PLANE
        ground.size[:] = [50.0, 50.0, 0.1]
        ground.friction[:] = [1.0, 0.005, 0.0001]
        ground.condim = 3

    # --- NO freejoint: link0 stays welded to the world (bolted-down arm) ---

    # --- payload body welded at the TCP (mass = thrown part) ---
    bodies = {b.name: b for b in _all_bodies(spec)}
    link6 = bodies.get("link6")
    if link6 is None:
        raise RuntimeError("link6 not found in OMNIARM6 MJCF")
    pay = link6.add_body()
    pay.name = "payload"
    pay.pos[:] = [0.0, 0.0, LINK6_TO_FLANGE_Z + PAYLOAD_TCP_OFFSET_Z]
    pay.mass = float(payload_mass)
    # small isotropic inertia for a ~5 cm part; orientation-free
    i = (2.0 / 5.0) * float(payload_mass) * (0.03 ** 2)
    pay.inertia[:] = [i, i, i]
    pay.ipos[:] = [0.0, 0.0, 0.0]
    pay.iquat[:] = [1.0, 0.0, 0.0, 0.0]
    pay.explicitinertial = True

    # --- torque-limited position actuators, one per arm joint, named {joint}_pos ---
    spec_joints = {j.name: j for j in spec.joints}
    for jn in ARM_JOINTS:
        j = spec_joints.get(jn)
        if j is None:
            raise RuntimeError(f"joint {jn!r} missing from OMNIARM6 MJCF")
        lo, hi, effort, vel = limits[jn]
        j.limited = True
        j.range[:] = [lo, hi]
        # motor speed cap: net torque -> 0 at the rated joint speed (b * vel == effort).
        j.damping[0] = effort / vel
        a = spec.add_actuator()
        a.name = f"{jn}_pos"
        a.target = jn
        a.trntype = mujoco.mjtTrn.mjTRN_JOINT
        a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        a.gainprm[0] = KP
        a.biasprm[1] = -KP
        a.biasprm[2] = -KV
        a.ctrllimited = True
        a.ctrlrange[:] = [lo, hi]
        a.forcelimited = True
        a.forcerange[:] = [-effort, effort]    # <-- the feasibility-by-construction clamp
    return spec


def _all_bodies(spec):
    """Flat list of every body in the spec (worldbody children, transitively)."""
    out = []

    def walk(b):
        for c in b.bodies:
            out.append(c)
            walk(c)
    walk(spec.worldbody)
    return out


def generate() -> int:
    import mujoco
    spec = build_spec()
    model = spec.compile()
    print(f"[make_omniarm6_mjcf] compile OK: nq={model.nq} nv={model.nv} nu={model.nu} "
          f"njnt={model.njnt} nbody={model.nbody}")
    MJCF_OUT.parent.mkdir(parents=True, exist_ok=True)
    MJCF_OUT.write_text(spec.to_xml(), encoding="utf-8")
    print(f"[make_omniarm6_mjcf] wrote {MJCF_OUT.relative_to(REPO)} ({MJCF_OUT.stat().st_size} B)")
    m2 = mujoco.MjModel.from_xml_path(str(MJCF_OUT))
    print(f"[make_omniarm6_mjcf] reload OK: nq={m2.nq} nv={m2.nv} nu={m2.nu}")
    return 0


def verify() -> int:
    import mujoco
    import numpy as np
    if not MJCF_OUT.exists():
        print(f"[verify] {MJCF_OUT} missing -- run the generator first.", file=sys.stderr)
        return 1
    m = mujoco.MjModel.from_xml_path(str(MJCF_OUT))
    limits = _urdf_limits(URDF)
    fails = []

    def check(c, msg):
        print(f"[verify] [{'PASS' if c else 'FAIL'}] {msg}")
        if not c:
            fails.append(msg)

    # fixed base: nq == nv == 6 (six hinges, no free joint)
    check(m.nq == 6 and m.nv == 6, f"fixed base, 6 dof -> nq={m.nq} nv={m.nv}")
    check(m.nu == 6, f"6 position actuators -> nu={m.nu}")
    # actuator force ranges == URDF effort
    fok = True
    for jn in ARM_JOINTS:
        aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{jn}_pos")
        eff = limits[jn][2]
        fr = m.actuator_forcerange[aid]
        if abs(fr[0] + eff) > 1e-4 or abs(fr[1] - eff) > 1e-4:
            fok = False
            print(f"[verify]   {jn}_pos forcerange {tuple(fr)} != +/-{eff}")
    check(fok, "actuator forcerange == URDF effort limits")
    # payload present with the requested mass
    pid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "payload")
    check(pid >= 0, "payload body present")
    if pid >= 0:
        check(abs(float(m.body_mass[pid]) - PAYLOAD_MASS) < 1e-6,
              f"payload mass == {PAYLOAD_MASS} -> {float(m.body_mass[pid])}")
    # steps without NaN
    d = mujoco.MjData(m)
    for _ in range(50):
        mujoco.mj_step(m, d)
    check(bool(np.isfinite(d.qpos).all()), "50 steps finite (stable PD)")

    print()
    if fails:
        print(f"[verify] RESULT: FAIL ({len(fails)})")
        return 1
    print("[verify] RESULT: PASS")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--check-only", action="store_true")
    args = ap.parse_args()
    if args.check_only:
        return verify()
    rc = generate()
    if rc == 0 and args.verify:
        return verify()
    return rc


if __name__ == "__main__":
    sys.exit(main())
